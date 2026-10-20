#!/usr/bin/env python3
"""ROS 2-node: verbindt de micro:Maqueen (micro:bit) met ROS 2.

Abonneert op:   /cmd_vel            (geometry_msgs/Twist)
Publiceert:     /odom               (nav_msgs/Odometry)  + TF odom -> base_link
                /maqueen/range      (sensor_msgs/Range)  ultrasone afstand
                /maqueen/line       (std_msgs/Int8MultiArray) [links, rechts]
                /maqueen/heading    (std_msgs/Float32)   kompas in graden
                /maqueen/button     (std_msgs/String)    "A", "B" of "AB"
Abonneert op:   /maqueen/reset      (std_msgs/Empty)     noodstop opheffen
                TF base_link -> ultrasonic (vast)

Positie (odometrie): de Maqueen heeft geen wielencoders. De afgelegde weg
wordt geschat uit de motorcommando's (snelheid x tijd); de richting komt
van het kompas van de micro:bit. Dat is ruw, maar genoeg om een kaart te maken.

Parameters:
  port        /dev/ttyACM0 (kabel) of socket://host:7777 (Bluetooth-relay)
  baud        115200
  max_lin     m/s bij PWM 255 (meten! standaard 0.2)
  wheel_base  afstand tussen de wielen in m (0.08)
  min_pwm     kleinste PWM waarbij de motoren echt draaien (50)
  sensor_x    afstand ultrasone sensor voor het midden in m (0.04)
  use_compass richting uit kompas halen (true)
  cmd_timeout s zonder /cmd_vel waarna de robot stopt (0.6)
"""
import math
import threading
import time

import rclpy
import serial
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import Empty, Float32, Int8MultiArray, String
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from logboek import Logboek


def quat_z(yaw):
    return 0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)


class MaqueenBridge(Node):
    def __init__(self):
        super().__init__("maqueen_bridge")
        self.L = Logboek(self.get_logger(), "bridge")
        p = self.declare_parameter
        port = p("port", "/dev/ttyACM0").value
        baud = p("baud", 115200).value
        self.max_lin = p("max_lin", 0.2).value
        self.wheel_base = p("wheel_base", 0.08).value
        self.min_pwm = p("min_pwm", 50).value
        self.sensor_x = p("sensor_x", 0.04).value
        self.use_compass = p("use_compass", True).value
        # 0 = automatisch bepalen, 1 of -1 = vast. De micro:bit staat rechtop op de
        # Maqueen, daardoor kan de draairichting van het kompas omgekeerd zijn.
        self.kompas_teken = p("compass_sign", 0).value
        self.cmd_timeout = p("cmd_timeout", 0.6).value

        self.L.info(f"Verbinden met {port} ...")
        self.ser = serial.serial_for_url(port, baudrate=baud, timeout=0.1)
        self.L.info("Verbonden")

        self.lock = threading.Lock()
        self.links = 0
        self.rechts = 0
        self.laatste_cmd = 0.0
        self.kompas_ruw = None      # kompas in radialen tegen de klok in, t.o.v. start
        self.kompas_nul = None
        self.vorige_ruw = None
        self.teken_som = 0.0
        self.gedraaid_zonder_kompas = 0.0
        self.kompas_offset = 0.0

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.vorige_tijd = time.monotonic()

        self.create_subscription(Twist, "cmd_vel", self.on_cmd_vel, 10)
        self.pub_range = self.create_publisher(Range, "maqueen/range", 10)
        self.pub_line = self.create_publisher(Int8MultiArray, "maqueen/line", 10)
        self.pub_heading = self.create_publisher(Float32, "maqueen/heading", 10)
        self.pub_odom = self.create_publisher(Odometry, "odom", 10)
        self.pub_knop = self.create_publisher(String, "maqueen/button", 10)
        self.create_subscription(Empty, "maqueen/reset", self.on_reset, 10)
        self.tf = TransformBroadcaster(self)

        self.static_tf = StaticTransformBroadcaster(self)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "base_link"
        t.child_frame_id = "ultrasonic"
        t.transform.translation.x = self.sensor_x
        t.transform.rotation.w = 1.0
        self.static_tf.sendTransform(t)

        self.create_timer(0.1, self.tick)
        self.create_timer(5.0, self.hartslag)
        self.aantal_metingen = 0
        self.laatste_meting = time.monotonic()
        self.laatste_status = None
        threading.Thread(target=self.read_loop, daemon=True).start()

    # ---------- commando's ----------
    def on_cmd_vel(self, msg: Twist):
        v_l = msg.linear.x - msg.angular.z * self.wheel_base / 2
        v_r = msg.linear.x + msg.angular.z * self.wheel_base / 2
        schaal = 255 / self.max_lin
        with self.lock:
            self.links = self.pwm(v_l * schaal)
            self.rechts = self.pwm(v_r * schaal)
            self.laatste_cmd = time.monotonic()

    def on_reset(self, _msg):
        """Noodstop op de micro:bit opheffen (bij starten vanaf de pc)."""
        try:
            self.ser.write(b"R\n")
        except serial.SerialException as e:
            self.L.error(f"Schrijffout: {e}")

    def pwm(self, waarde):
        waarde = int(max(-255, min(255, waarde)))
        if waarde == 0:
            return 0
        if abs(waarde) < self.min_pwm:
            return int(math.copysign(self.min_pwm, waarde))
        return waarde

    def tick(self):
        nu = time.monotonic()
        with self.lock:
            if nu - self.laatste_cmd > self.cmd_timeout:
                self.links = self.rechts = 0
            links, rechts = self.links, self.rechts
            ruw = self.kompas_ruw
        try:
            self.ser.write(f"M {links} {rechts}\n".encode())
        except serial.SerialException as e:
            self.L.error(f"Schrijffout: {e}")

        # Odometrie bijwerken
        dt = nu - self.vorige_tijd
        self.vorige_tijd = nu
        v_l = links / 255 * self.max_lin
        v_r = rechts / 255 * self.max_lin
        v = (v_l + v_r) / 2
        w = (v_r - v_l) / self.wheel_base
        self.bepaal_kompas_teken(ruw, w, dt)
        if self.use_compass and self.kompas_teken != 0 and ruw is not None:
            yaw = self.kompas_teken * ruw + self.kompas_offset
            self.yaw = math.atan2(math.sin(yaw), math.cos(yaw))
        else:
            self.yaw = math.atan2(math.sin(self.yaw + w * dt), math.cos(self.yaw + w * dt))
        self.x += v * math.cos(self.yaw) * dt
        self.y += v * math.sin(self.yaw) * dt
        self.publiceer_odom(v, w)

    def hartslag(self):
        nu = time.monotonic()
        stil = nu - self.laatste_meting
        per_s = self.aantal_metingen / 5.0
        self.aantal_metingen = 0
        if stil > 2.0:
            self.L.warn(f"GEEN gegevens van de robot sinds {stil:.0f} s. Draait ble_relay.py en staat "
                        "de robot aan (vinkje op het scherm)?")
            return
        d = self.laatste_status or []
        kompas = {0: "nog niet gekoppeld (eerst een rondje draaien)", 1: "normaal",
                  -1: "omgekeerd"}.get(self.kompas_teken, "?")
        self.L.info(
            f"Robot: {per_s:.0f} metingen/s | afstand {d[3] if len(d) > 3 else '?'} cm | "
            f"lijn {d[1] if len(d) > 1 else '?'}/{d[2] if len(d) > 2 else '?'} | "
            f"kompas {d[4] if len(d) > 4 else '-'} gr ({kompas}) | motoren L{self.links} R{self.rechts} | "
            f"pos ({self.x:.2f},{self.y:.2f}) {math.degrees(self.yaw):.0f}gr")
        if per_s < 5:
            self.L.warn("Weinig metingen per seconde: Bluetooth zwak of robot te ver weg")

    def bepaal_kompas_teken(self, ruw, w, dt):
        """Vergelijk de draairichting van de motoren met die van het kompas."""
        if not self.use_compass or ruw is None or self.kompas_teken != 0:
            self.vorige_ruw = ruw
            return
        if self.vorige_ruw is not None and abs(w) > 0.3:
            d = math.atan2(math.sin(ruw - self.vorige_ruw), math.cos(ruw - self.vorige_ruw))
            if abs(d) < 0.6:
                self.teken_som += d if w > 0 else -d
            self.gedraaid_zonder_kompas += abs(w) * dt
        self.vorige_ruw = ruw
        if abs(self.teken_som) > 1.5:
            self.kompas_teken = 1 if self.teken_som > 0 else -1
            # zonder sprong overschakelen van geschatte richting naar kompas
            self.kompas_offset = self.yaw - self.kompas_teken * ruw
            self.L.info(
                "Kompas gekoppeld (" + ("normaal" if self.kompas_teken > 0 else "omgekeerd") + ")")
        elif self.gedraaid_zonder_kompas > 4 * math.pi and self.teken_som == 0.0:
            self.L.warn("Kompas verandert niet bij draaien; ik gebruik geschatte draaiing")
            self.gedraaid_zonder_kompas = -1e9

    def publiceer_odom(self, v, w):
        stamp = self.get_clock().now().to_msg()
        qx, qy, qz, qw = quat_z(self.yaw)

        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf.sendTransform(t)

        o = Odometry()
        o.header.stamp = stamp
        o.header.frame_id = "odom"
        o.child_frame_id = "base_link"
        o.pose.pose.position.x = self.x
        o.pose.pose.position.y = self.y
        o.pose.pose.orientation.z = qz
        o.pose.pose.orientation.w = qw
        o.twist.twist.linear.x = v
        o.twist.twist.angular.z = w
        self.pub_odom.publish(o)

    # ---------- sensoren ----------
    def read_loop(self):
        while rclpy.ok():
            try:
                regel = self.ser.readline().decode(errors="ignore").strip()
            except serial.SerialException as e:
                self.L.error(f"Leesfout: {e}")
                time.sleep(1)
                continue
            delen = regel.split()
            if len(delen) == 2 and delen[0] == "K":
                self.L.info(f"Knop {delen[1]} ingedrukt op de robot")
                self.pub_knop.publish(String(data=delen[1]))
                continue
            if len(delen) not in (4, 5) or delen[0] != "S":
                if regel:
                    self.L.debug(f"Onbekende regel van de robot: {regel!r}")
                continue
            self.aantal_metingen += 1
            self.laatste_meting = time.monotonic()
            self.laatste_status = delen
            try:
                lijn_l, lijn_r = int(delen[1]), int(delen[2])
                afstand = float(delen[3])
                kompas = float(delen[4]) if len(delen) == 5 else None
            except ValueError:
                continue

            line = Int8MultiArray()
            line.data = [lijn_l, lijn_r]
            self.pub_line.publish(line)

            if kompas is not None:
                self.pub_heading.publish(Float32(data=kompas))
                # Kompas: graden met de klok mee vanaf noord.
                # ROS: radialen tegen de klok in. Start = richting 0.
                with self.lock:
                    if self.kompas_nul is None:
                        self.kompas_nul = kompas
                    verschil = math.radians(kompas - self.kompas_nul)
                    self.kompas_ruw = math.atan2(-math.sin(verschil), math.cos(verschil))

            r = Range()
            r.header.stamp = self.get_clock().now().to_msg()
            r.header.frame_id = "ultrasonic"
            r.radiation_type = Range.ULTRASOUND
            r.field_of_view = math.radians(15)
            r.min_range = 0.02
            r.max_range = 4.0
            r.range = afstand / 100.0 if afstand > 0 else float("inf")
            self.pub_range.publish(r)

    def destroy_node(self):
        try:
            self.ser.write(b"M 0 0\n")
            self.ser.close()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = MaqueenBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()