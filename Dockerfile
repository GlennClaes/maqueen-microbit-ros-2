#!/usr/bin/env python3
"""SLAM, verkennen, objecten en naar huis rijden voor de micro:Maqueen.

Sensoren: 1 ultrasone afstandssensor, kompas, geschatte afgelegde weg.

Wat deze node doet
  * KAART:      elke afstandsmeting tekent een kegel in een rasterkaart (/map)
  * POSITIE:    na elk rondje meten wordt het rondje over de bestaande kaart
                gelegd (scan matching). Ligt het beter met een kleine
                verschuiving, dan corrigeert de node de positie (TF map->odom).
  * VERKENNEN:  zoekt op de kaart de dichtstbijzijnde grens tussen bekend-vrij
                en onbekend gebied (frontier), plant een pad eromheen
                (Dijkstra, met veiligheidsmarge rond muren) en rijdt erheen.
                Pas daar meet hij opnieuw rondom.
  * NAAR HUIS:  is alles verkend (of de tijd op), dan plant hij een pad
                terug naar het startpunt en stopt daar.
  * OBJECTEN:   losse obstakels met vrije ruimte rondom worden herkend als
                object (plaats + grootte). Muren niet.

Bedienen (ook via de knoppen op de micro:bit: A=stop, B=verkennen, A+B=naar huis):
  ros2 param set /maqueen_slam mode verkennen
  ros2 param set /maqueen_slam mode stop
  ros2 param set /maqueen_slam mode naar_huis
  ros2 param set /maqueen_slam mode reset_kaart

Topics: /map /scan /maqueen/plan /maqueen/traject /maqueen/markers
        /maqueen/objects (JSON) /maqueen/status
"""
import heapq
import json
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from scipy import ndimage
from sensor_msgs.msg import LaserScan, Range
from std_msgs.msg import Empty, Int8MultiArray, String
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

L_VRIJ = -0.3
L_VRIJ_RAND = -0.12
L_BEZET = 0.9
L_GRENS = 5.0
SCAN_BAKJES = 72
BUREN = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
         (-1, -1, 1.414), (-1, 1, 1.414), (1, -1, 1.414), (1, 1, 1.414)]
MODI = ("stop", "verkennen", "naar_huis", "reset_kaart")


def hoekverschil(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


def yaw_uit_quat(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def matrix(x, y, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, x], [s, c, y], [0.0, 0.0, 1.0]])


def schijf(r):
    yy, xx = np.mgrid[-r:r + 1, -r:r + 1]
    return (xx * xx + yy * yy) <= r * r


class MaqueenSlam(Node):
    AANPASBAAR = {
        "turn_speed": "draaisnelheid",
        "drive_speed": "rijsnelheid",
        "safe_dist": "veilig",
        "rescan_dist": "herscan_afstand",
        "cliff_stop": "cliff_stop",
        "black_value": "zwart",
        "free_on_miss": "vrij_bij_mis",
        "scan_match": "scan_match",
        "max_explore_time": "max_tijd",
    }

    def __init__(self):
        super().__init__("maqueen_slam")
        p = self.declare_parameter
        self.res = p("resolution", 0.05).value
        grootte = p("size_m", 8.0).value
        self.max_range = p("max_range", 1.5).value
        self.vrij_bij_mis = p("free_on_miss", 0.5).value
        self.bundel = math.radians(p("beam_deg", 15.0).value)
        self.sensor_x = p("sensor_x", 0.04).value
        self.draaisnelheid = p("turn_speed", 2.0).value
        self.rijsnelheid = p("drive_speed", 0.15).value
        self.veilig = p("safe_dist", 0.18).value
        self.robot_straal = p("robot_radius", 0.07).value
        self.marge = p("inflate", 0.05).value
        self.herscan_afstand = p("rescan_dist", 1.0).value
        self.scan_match = p("scan_match", True).value
        self.cliff_stop = p("cliff_stop", False).value
        self.zwart = p("black_value", 0).value
        self.max_tijd = p("max_explore_time", 600.0).value
        p("mode", "stop")
        self.add_on_set_parameters_callback(self.param_gewijzigd)

        self.n = int(round(grootte / self.res))
        self.oorsprong = -grootte / 2
        self.grid = np.zeros((self.n, self.n), dtype=np.float32)

        self.T = np.eye(3)            # map <- odom (correctie door scan matching)
        self.odom = None              # (x, y, yaw) in odom
        self.laatste_afstand = (float("inf"), 0.0)
        self.lijn = None

        self.modus = "STIL"           # STIL, VERKENNEN, NAAR_HUIS, KLAAR
        self.sub = "-"                # SCAN, PLAN, VOLGEN, ACHTERUIT
        self.sub_start = time.monotonic()
        self.verken_start = 0.0
        self.scan_buffer = []
        self.scan_vorige_yaw = None
        self.scan_gedraaid = 0.0
        self.laatste_scan_pos = (0.0, 0.0)
        self.pad = []
        self.doel = None
        self.mislukte_doelen = []
        self.laatste_plan = 0.0
        self.obstakels = 0
        self.traject = []
        self.objecten = []
        self.info = "klaar om te starten"

        kaart_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                               reliability=ReliabilityPolicy.RELIABLE)
        self.pub_map = self.create_publisher(OccupancyGrid, "map", kaart_qos)
        self.pub_scan = self.create_publisher(LaserScan, "scan", 10)
        self.pub_cmd = self.create_publisher(Twist, "cmd_vel", 10)
        self.pub_plan = self.create_publisher(Path, "maqueen/plan", 10)
        self.pub_traject = self.create_publisher(Path, "maqueen/traject", 10)
        self.pub_markers = self.create_publisher(MarkerArray, "maqueen/markers", 10)
        self.pub_objecten = self.create_publisher(String, "maqueen/objects", 10)
        self.pub_status = self.create_publisher(String, "maqueen/status", 10)
        self.pub_reset = self.create_publisher(Empty, "maqueen/reset", 10)
        self.create_subscription(Odometry, "odom", self.on_odom, 20)
        self.create_subscription(Range, "maqueen/range", self.on_range, 20)
        self.create_subscription(Int8MultiArray, "maqueen/line", self.on_line, 20)
        self.create_subscription(String, "maqueen/button", self.on_knop, 10)

        self.tf = TransformBroadcaster(self)
        self.create_timer(0.1, self.publiceer_tf)
        self.create_timer(0.1, self.regel)
        self.create_timer(1.0, self.publiceer_kaart)
        self.create_timer(0.5, self.publiceer_markers)
        self.get_logger().info("Klaar. Start met: ros2 param set /maqueen_slam mode verkennen")

    # ================= bediening =================
    def param_gewijzigd(self, params):
        for prm in params:
            if prm.name == "mode":
                if prm.value not in MODI:
                    return SetParametersResult(successful=False,
                                               reason=f"kies uit {', '.join(MODI)}")
                self.zet_modus(prm.value)
            elif prm.name in self.AANPASBAAR:
                setattr(self, self.AANPASBAAR[prm.name], prm.value)
        return SetParametersResult(successful=True)

    def on_knop(self, msg: String):
        modus = {"A": "stop", "B": "verkennen", "AB": "naar_huis"}.get(msg.data.strip())
        if modus:
            self.get_logger().info(f"Knop {msg.data} op de robot -> {modus}")
            self.set_parameters([Parameter("mode", value=modus)])

    def zet_modus(self, modus):
        if modus == "stop":
            self.modus, self.info = "STIL", "gestopt"
            self.wissel("-")
            self.pad = []
            self.pub_cmd.publish(Twist())
        elif modus == "verkennen":
            self.pub_reset.publish(Empty())
            self.modus, self.info = "VERKENNEN", "verkennen"
            self.verken_start = time.monotonic()
            self.mislukte_doelen = []
            self.start_scan()
        elif modus == "naar_huis":
            self.pub_reset.publish(Empty())
            self.modus, self.info = "NAAR_HUIS", "op weg naar huis"
            self.wissel("PLAN")
        elif modus == "reset_kaart":
            self.grid[:] = 0
            self.traject = []
            self.objecten = []
            self.modus, self.info = "STIL", "kaart gewist"
            self.wissel("-")
            self.pub_cmd.publish(Twist())
        self.get_logger().info(f"Modus: {self.modus}")

    def wissel(self, sub):
        self.sub = sub
        self.sub_start = time.monotonic()

    # ================= sensoren =================
    def on_odom(self, msg: Odometry):
        p = msg.pose.pose
        self.odom = (p.position.x, p.position.y, yaw_uit_quat(p.orientation))
        x, y, _ = self.pose_kaart()
        if not self.traject or math.hypot(x - self.traject[-1][0], y - self.traject[-1][1]) > 0.04:
            self.traject.append((x, y))
            if len(self.traject) > 3000:
                self.traject = self.traject[-3000:]

    def on_line(self, msg: Int8MultiArray):
        self.lijn = list(msg.data)

    def on_range(self, msg: Range):
        r = msg.range
        self.laatste_afstand = (r, time.monotonic())
        if self.odom is None:
            return
        if self.sub == "SCAN":
            self.scan_buffer.append((*self.odom, r))
            return
        self.integreer_meting(self.pose_kaart(), r)

    def pose_kaart(self, odom=None):
        x, y, yaw = odom if odom is not None else self.odom
        p = self.T @ np.array([x, y, 1.0])
        return float(p[0]), float(p[1]), hoekverschil(yaw + math.atan2(self.T[1, 0], self.T[0, 0]), 0)

    # ================= kaart =================
    def cel(self, x, y):
        ix = int(math.floor((x - self.oorsprong) / self.res))
        iy = int(math.floor((y - self.oorsprong) / self.res))
        if 0 <= ix < self.n and 0 <= iy < self.n:
            return iy, ix
        return None

    def wereld(self, iy, ix):
        return (self.oorsprong + (ix + 0.5) * self.res,
                self.oorsprong + (iy + 0.5) * self.res)

    def integreer_meting(self, pose, r):
        x, y, yaw = pose
        if math.isfinite(r) and 0.04 <= r <= self.max_range:
            einde, raak = r, True
        elif math.isfinite(r) and r > self.max_range:
            einde, raak = self.max_range, False
        else:
            einde, raak = self.vrij_bij_mis, False
        sx = x + self.sensor_x * math.cos(yaw)
        sy = y + self.sensor_x * math.sin(yaw)

        kern, rand, bezet = set(), set(), set()
        halve = self.bundel / 2
        stapjes = np.arange(0.0, max(einde - 1.5 * self.res, 0.0), self.res / 2)
        for h in np.linspace(-halve, halve, 5):
            in_kern = abs(h) <= halve / 2 + 1e-9
            ca, sa = math.cos(yaw + h), math.sin(yaw + h)
            doel = kern if in_kern else rand
            for d in stapjes:
                c = self.cel(sx + d * ca, sy + d * sa)
                if c is not None:
                    doel.add(c)
            if raak and in_kern:
                c = self.cel(sx + einde * ca, sy + einde * sa)
                if c is not None:
                    bezet.add(c)
        kern -= bezet
        rand -= bezet | kern
        for cellen, w in ((kern, L_VRIJ), (rand, L_VRIJ_RAND), (bezet, L_BEZET)):
            if cellen:
                iy, ix = zip(*cellen)
                self.grid[iy, ix] += w
        np.clip(self.grid, -L_GRENS, L_GRENS, out=self.grid)

    # ================= scan + scan matching =================
    def start_scan(self):
        self.scan_buffer = []
        self.scan_vorige_yaw = None
        self.scan_gedraaid = 0.0
        self.pad = []
        self.wissel("SCAN")

    def eindpunten(self, metingen):
        punten = []
        for x, y, yaw, r in metingen:
            if math.isfinite(r) and 0.04 <= r <= self.max_range:
                d = self.sensor_x + r
                punten.append((x + d * math.cos(yaw), y + d * math.sin(yaw)))
        return np.array(punten)

    def match(self, metingen):
        """Zoek de kleine verschuiving/draaiing waarbij de scan het best op de kaart past."""
        bezet = self.grid > 1.0
        if bezet.sum() < 25:
            return
        P_odom = self.eindpunten(metingen)
        if len(P_odom) < 12:
            return
        P = (self.T @ np.c_[P_odom, np.ones(len(P_odom))].T).T[:, :2]
        cx, cy, _ = self.pose_kaart()
        c = np.array([cx, cy])

        afstand = ndimage.distance_transform_edt(~bezet) * self.res
        veld = np.exp(-(afstand ** 2) / (2 * 0.05 ** 2))
        stap = np.arange(-0.15, 0.1501, 0.025)
        DX, DY = [a.ravel() for a in np.meshgrid(stap, stap)]
        straf = 0.1 * len(P) * np.hypot(DX, DY) / 0.15  # kleine voorkeur voor geen correctie

        beste = None
        score0 = None
        for graden in np.arange(-8, 8.01, 2):
            th = math.radians(graden)
            R = np.array([[math.cos(th), -math.sin(th)], [math.sin(th), math.cos(th)]])
            Q = (P - c) @ R.T + c
            X = Q[None, :, 0] + DX[:, None]
            Y = Q[None, :, 1] + DY[:, None]
            ix = np.floor((X - self.oorsprong) / self.res).astype(int)
            iy = np.floor((Y - self.oorsprong) / self.res).astype(int)
            binnen = (ix >= 0) & (ix < self.n) & (iy >= 0) & (iy < self.n)
            waarden = np.zeros(X.shape)
            waarden[binnen] = veld[iy[binnen], ix[binnen]]
            scores = waarden.sum(axis=1)
            if graden == 0:
                score0 = scores[np.argmin(np.hypot(DX, DY))]
            scores = scores - straf - 0.02 * len(P) * abs(graden) / 8
            i = int(np.argmax(scores))
            if beste is None or scores[i] > beste[0]:
                beste = (scores[i], DX[i], DY[i], th)

        s, dx, dy, th = beste
        if s > score0 + 0.05 * len(P) and (dx or dy or th):
            D = matrix(cx + dx, cy + dy, 0.0) @ matrix(0, 0, th) @ matrix(-cx, -cy, 0.0)
            self.T = D @ self.T
            self.get_logger().info(
                f"Positie gecorrigeerd: {dx*100:+.0f} cm, {dy*100:+.0f} cm, {math.degrees(th):+.0f} graden")

    def scan_afronden(self):
        metingen = self.scan_buffer
        self.scan_buffer = []
        if self.scan_match:
            self.match(metingen)
        for x, y, yaw, r in metingen:
            self.integreer_meting(self.pose_kaart((x, y, yaw)), r)
        x, y, yaw_nu = self.pose_kaart()
        self.laatste_scan_pos = (x, y)
        self.publiceer_scan(metingen, yaw_nu)

    # ================= padplanning =================
    def plan_kaarten(self):
        bezet = self.grid > 0.4
        r = int(math.ceil((self.robot_straal + self.marge) / self.res))
        opgeblazen = ndimage.binary_dilation(bezet, structure=schijf(r))
        rijdbaar = (self.grid < -0.2) & ~opgeblazen
        onbekend = np.abs(self.grid) < 0.05
        return rijdbaar, onbekend, bezet

    def dijkstra(self, start, rijdbaar, bezet):
        """Kortste afstanden vanaf start over rijdbare cellen (binnen het bekende gebied)."""
        sy, sx = start
        ontsnap = int(math.ceil(0.15 / self.res))
        toegestaan = rijdbaar.copy()
        y0, y1 = max(sy - ontsnap, 0), min(sy + ontsnap + 1, self.n)
        x0, x1 = max(sx - ontsnap, 0), min(sx + ontsnap + 1, self.n)
        toegestaan[y0:y1, x0:x1] |= ~bezet[y0:y1, x0:x1]

        ys, xs = np.nonzero(toegestaan | (np.abs(self.grid) >= 0.05))
        ys = np.append(ys, sy)
        xs = np.append(xs, sx)
        by0, by1 = max(ys.min() - 2, 0), min(ys.max() + 3, self.n)
        bx0, bx1 = max(xs.min() - 2, 0), min(xs.max() + 3, self.n)
        sub = toegestaan[by0:by1, bx0:bx1]
        H, W = sub.shape
        dist = np.full((H, W), np.inf)
        ouder = np.full((H, W), -1, dtype=np.int64)
        s = (sy - by0, sx - bx0)
        dist[s] = 0.0
        heap = [(0.0, s[0], s[1])]
        while heap:
            d, y, x = heapq.heappop(heap)
            if d > dist[y, x]:
                continue
            for dy, dx, w in BUREN:
                ny, nx = y + dy, x + dx
                if 0 <= ny < H and 0 <= nx < W and sub[ny, nx]:
                    nd = d + w
                    if nd < dist[ny, nx]:
                        dist[ny, nx] = nd
                        ouder[ny, nx] = y * W + x
                        heapq.heappush(heap, (nd, ny, nx))
        return dist, ouder, (by0, bx0, W)

    def pad_terug(self, ouder, kader, doel):
        by0, bx0, W = kader
        y, x = doel
        cellen = []
        while True:
            cellen.append((y + by0, x + bx0))
            o = ouder[y, x]
            if o < 0:
                break
            y, x = divmod(int(o), W)
        cellen.reverse()
        # elke 2e cel, maar het eindpunt altijd mee
        idx = list(range(2, len(cellen), 2))
        if len(cellen) > 1 and (not idx or idx[-1] != len(cellen) - 1):
            idx.append(len(cellen) - 1)
        return [self.wereld(*cellen[i]) for i in idx]

    def plan_frontier(self, x, y):
        start = self.cel(x, y)
        if start is None:
            return None
        rijdbaar, onbekend, bezet = self.plan_kaarten()
        dist, ouder, kader = self.dijkstra(start, rijdbaar, bezet)
        by0, bx0, W = kader
        H = dist.shape[0]
        kruis = ndimage.generate_binary_structure(2, 1)
        grens = rijdbaar & ndimage.binary_dilation(onbekend, structure=kruis)
        dichtheid = ndimage.uniform_filter(grens.astype(float), size=5) * 25
        grens &= dichtheid >= 3
        sub = grens[by0:by0 + H, bx0:bx0 + W] & np.isfinite(dist)
        kandidaten = np.argwhere(sub)
        beste = None
        for cy, cx in kandidaten:
            d = dist[cy, cx] * self.res
            if d < 0.25:
                continue
            wx, wy = self.wereld(cy + by0, cx + bx0)
            if any(math.hypot(wx - fx, wy - fy) < 0.3 for fx, fy in self.mislukte_doelen):
                continue
            if beste is None or d < beste[0]:
                beste = (d, cy, cx, wx, wy)
        if beste is None:
            return None
        _, cy, cx, wx, wy = beste
        return self.pad_terug(ouder, kader, (cy, cx)), (wx, wy)

    def plan_naar(self, x, y, doel_xy):
        start = self.cel(x, y)
        doel = self.cel(*doel_xy)
        if start is None or doel is None:
            return None
        rijdbaar, _, bezet = self.plan_kaarten()
        dist, ouder, kader = self.dijkstra(start, rijdbaar, bezet)
        by0, bx0, W = kader
        bereikbaar = np.argwhere(np.isfinite(dist))
        if len(bereikbaar) == 0:
            return None
        gy, gx = doel[0] - by0, doel[1] - bx0
        afst = np.hypot(bereikbaar[:, 0] - gy, bereikbaar[:, 1] - gx)
        cy, cx = bereikbaar[int(np.argmin(afst))]
        return self.pad_terug(ouder, kader, (cy, cx)), self.wereld(cy + by0, cx + bx0)

    # ================= besturing (10x per seconde) =================
    def regel(self):
        if self.modus in ("STIL", "KLAAR") or self.odom is None:
            self.publiceer_status()
            return
        x, y, yaw = self.pose_kaart()
        nu = time.monotonic()
        duur = nu - self.sub_start
        cmd = Twist()

        if (self.cliff_stop and self.sub == "VOLGEN" and self.lijn is not None
                and all(v == self.zwart for v in self.lijn)):
            self.get_logger().info("Rand gedetecteerd, achteruit")
            if self.doel:
                self.mislukte_doelen.append(self.doel)
            self.wissel("ACHTERUIT")

        if self.modus == "VERKENNEN" and nu - self.verken_start > self.max_tijd:
            self.get_logger().info("Tijd om, terug naar huis")
            self.modus, self.info = "NAAR_HUIS", "tijd om, naar huis"
            self.wissel("PLAN")

        if self.sub == "SCAN":
            if self.scan_vorige_yaw is not None:
                self.scan_gedraaid += hoekverschil(self.odom[2], self.scan_vorige_yaw)
            self.scan_vorige_yaw = self.odom[2]
            if abs(self.scan_gedraaid) >= 2 * math.pi + 0.15:
                self.scan_afronden()
                self.wissel("PLAN")
            elif duur > 40:
                self.get_logger().error("Geen rondje gedraaid in 40 s (kompas? MIN_PWM te laag?). Gestopt.")
                self.zet_modus("stop")
                return
            else:
                self.info = "rondje meten"
                cmd.angular.z = self.draaisnelheid

        elif self.sub == "PLAN":
            self.laatste_plan = nu
            if self.modus == "VERKENNEN":
                res = self.plan_frontier(x, y)
                if res is None:
                    self.get_logger().info("Alles verkend! Terug naar huis.")
                    self.modus, self.info = "NAAR_HUIS", "alles verkend, naar huis"
                    res = self.plan_naar(x, y, (0.0, 0.0))
            else:
                res = self.plan_naar(x, y, (0.0, 0.0))
            if self.modus == "NAAR_HUIS":
                if math.hypot(x, y) < 0.12:
                    self.klaar()
                    return
                # Dichterbij kan niet (kaart rond het startpunt onzeker): hier stoppen
                if res is not None and math.hypot(res[1][0] - x, res[1][1] - y) < 0.1:
                    self.klaar("zo dicht mogelijk bij huis")
                    return
            if res is None:
                self.get_logger().warn("Geen pad gevonden; opnieuw meten")
                self.start_scan()
            else:
                self.pad, self.doel = res
                self.info = ("rijdt naar onbekend gebied" if self.modus == "VERKENNEN"
                             else "rijdt naar huis")
                self.wissel("VOLGEN")

        elif self.sub == "VOLGEN":
            while self.pad and math.hypot(self.pad[0][0] - x, self.pad[0][1] - y) < 0.08:
                self.pad.pop(0)
            if not self.pad:
                self.obstakels = 0
                if self.modus == "VERKENNEN":
                    self.start_scan()
                elif math.hypot(x, y) < 0.15 or math.hypot(
                        self.doel[0] - x, self.doel[1] - y) < 0.1:
                    self.klaar()
                else:
                    self.wissel("PLAN")
                return self.pub_cmd.publish(cmd)
            if self.modus == "VERKENNEN" and math.hypot(
                    x - self.laatste_scan_pos[0], y - self.laatste_scan_pos[1]) > self.herscan_afstand:
                self.start_scan()
                return self.pub_cmd.publish(cmd)
            if nu - self.laatste_plan > 3.0 or duur > 40:
                if duur > 40 and self.doel:
                    self.mislukte_doelen.append(self.doel)
                self.wissel("PLAN")
                return self.pub_cmd.publish(cmd)

            doel = self.pad[-1]
            for px, py in self.pad:
                if math.hypot(px - x, py - y) >= 0.2:
                    doel = (px, py)
                    break
            fout = hoekverschil(math.atan2(doel[1] - y, doel[0] - x), yaw)
            if abs(fout) > 0.4:
                cmd.angular.z = math.copysign(self.draaisnelheid, fout)
            else:
                r, t_r = self.laatste_afstand
                if math.isfinite(r) and 0.04 <= r < self.veilig and nu - t_r < 0.4:
                    self.obstakels += 1
                    self.get_logger().info("Obstakel, nieuw pad zoeken")
                    if self.obstakels >= 3:
                        if self.doel:
                            self.mislukte_doelen.append(self.doel)
                        self.obstakels = 0
                        self.wissel("ACHTERUIT")
                    else:
                        self.wissel("PLAN")
                    return self.pub_cmd.publish(cmd)
                cmd.linear.x = self.rijsnelheid
                cmd.angular.z = max(-self.draaisnelheid, min(self.draaisnelheid, 2.0 * fout))

        elif self.sub == "ACHTERUIT":
            if duur > 0.8:
                self.start_scan()
            else:
                cmd.linear.x = -self.rijsnelheid

        self.pub_cmd.publish(cmd)
        self.publiceer_status()

    def klaar(self, tekst="thuis! verkennen klaar"):
        self.modus, self.info = "KLAAR", tekst
        self.wissel("-")
        self.pad = []
        self.pub_cmd.publish(Twist())
        self.get_logger().info(f"Thuis aangekomen. {len(self.objecten)} object(en) gevonden.")

    # ================= objecten =================
    def zoek_objecten(self):
        zeker = self.grid > 1.5
        labels, aantal = ndimage.label(zeker, structure=np.ones((3, 3)))
        if aantal == 0:
            self.objecten = []
            return
        vrij = self.grid < -0.2
        objecten = []
        for i, sl in enumerate(ndimage.find_objects(labels), start=1):
            if sl is None:
                continue
            masker = labels[sl] == i
            cellen = np.argwhere(masker)
            if len(cellen) < 2:
                continue
            h = (sl[0].stop - sl[0].start) * self.res
            w = (sl[1].stop - sl[1].start) * self.res
            grootte = math.hypot(h, w)
            if grootte > 0.45:
                continue  # te groot: muur of kast
            # rondom moet vooral vrije ruimte zijn (muren hebben onbekend gebied erachter)
            ry0, ry1 = max(sl[0].start - 3, 0), min(sl[0].stop + 3, self.n)
            rx0, rx1 = max(sl[1].start - 3, 0), min(sl[1].stop + 3, self.n)
            blok = labels[ry0:ry1, rx0:rx1] == i
            ring = ndimage.binary_dilation(blok, iterations=3) & ~ndimage.binary_dilation(blok, iterations=1)
            if ring.sum() == 0:
                continue
            ring_vrij = vrij[ry0:ry1, rx0:rx1][ring].mean()
            ring_bezet = (self.grid[ry0:ry1, rx0:rx1][ring] > 0.4).mean()
            if ring_vrij < 0.65 or ring_bezet > 0.08:
                continue  # hangt vast aan iets anders: stuk muur of hoek
            cy = cellen[:, 0].mean() + sl[0].start
            cx = cellen[:, 1].mean() + sl[1].start
            wx = self.oorsprong + (cx + 0.5) * self.res
            wy = self.oorsprong + (cy + 0.5) * self.res
            objecten.append({"x": round(float(wx), 2), "y": round(float(wy), 2),
                             "grootte_cm": int(round(max(h, w) * 100))})
        objecten.sort(key=lambda o: (o["x"], o["y"]))
        if len(objecten) != len(self.objecten):
            self.get_logger().info(f"{len(objecten)} object(en) op de kaart")
        self.objecten = objecten
        self.pub_objecten.publish(String(data=json.dumps(objecten)))

    # ================= publiceren =================
    def publiceer_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"
        t.child_frame_id = "odom"
        t.transform.translation.x = float(self.T[0, 2])
        t.transform.translation.y = float(self.T[1, 2])
        th = math.atan2(self.T[1, 0], self.T[0, 0])
        t.transform.rotation.z = math.sin(th / 2)
        t.transform.rotation.w = math.cos(th / 2)
        self.tf.sendTransform(t)

    def publiceer_status(self):
        self.pub_status.publish(String(data=f"{self.modus} / {self.sub}: {self.info}"))

    def publiceer_kaart(self):
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.info.resolution = float(self.res)
        msg.info.width = self.n
        msg.info.height = self.n
        msg.info.origin.position.x = float(self.oorsprong)
        msg.info.origin.position.y = float(self.oorsprong)
        msg.info.origin.orientation.w = 1.0
        g = self.grid.ravel()
        data = np.full(g.shape, -1, dtype=np.int8)
        bekend = np.abs(g) > 0.05
        kans = 1.0 - 1.0 / (1.0 + np.exp(g[bekend]))
        data[bekend] = np.round(kans * 100).astype(np.int8)
        msg.data = data.tolist()
        self.pub_map.publish(msg)
        self.zoek_objecten()

    def publiceer_scan(self, metingen, yaw_nu):
        bakjes = [float("inf")] * SCAN_BAKJES
        stap = 2 * math.pi / SCAN_BAKJES
        for x, y, yaw, r in metingen:
            if not (math.isfinite(r) and 0.04 <= r <= self.max_range):
                continue
            yaw_m = self.pose_kaart((x, y, yaw))[2]
            i = int((hoekverschil(yaw_m, yaw_nu) + math.pi) / stap) % SCAN_BAKJES
            bakjes[i] = min(bakjes[i], r)
        s = LaserScan()
        s.header.stamp = self.get_clock().now().to_msg()
        s.header.frame_id = "ultrasonic"
        s.angle_min = -math.pi
        s.angle_max = math.pi - stap
        s.angle_increment = stap
        s.range_min = 0.04
        s.range_max = float(self.max_range)
        s.ranges = bakjes
        self.pub_scan.publish(s)

    def maak_pad(self, punten):
        p = Path()
        p.header.stamp = self.get_clock().now().to_msg()
        p.header.frame_id = "map"
        for x, y in punten:
            ps = PoseStamped()
            ps.header = p.header
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.orientation.w = 1.0
            p.poses.append(ps)
        return p

    def marker(self, id_, soort, x, y, z, schaal, kleur, tekst=""):
        m = Marker()
        m.header.frame_id = "map"
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "maqueen"
        m.id = id_
        m.type = soort
        m.action = Marker.ADD
        m.pose.position.x = float(x)
        m.pose.position.y = float(y)
        m.pose.position.z = float(z)
        m.pose.orientation.w = 1.0
        m.scale.x, m.scale.y, m.scale.z = (float(v) for v in schaal)
        m.color.r, m.color.g, m.color.b, m.color.a = (float(v) for v in kleur)
        m.text = tekst
        return m

    def publiceer_markers(self):
        self.pub_traject.publish(self.maak_pad(self.traject))
        self.pub_plan.publish(self.maak_pad(self.pad))
        ma = MarkerArray()
        wis = Marker()
        wis.action = Marker.DELETEALL
        ma.markers.append(wis)

        ma.markers.append(self.marker(1, Marker.CYLINDER, 0, 0, 0.005, (0.16, 0.16, 0.01), (0.1, 0.8, 0.2, 0.9)))
        ma.markers.append(self.marker(2, Marker.TEXT_VIEW_FACING, 0, 0, 0.15, (0, 0, 0.08), (0.1, 0.9, 0.2, 1), "THUIS"))

        if self.odom is not None:
            x, y, yaw = self.pose_kaart()
            ma.markers.append(self.marker(3, Marker.CYLINDER, x, y, 0.03, (0.1, 0.1, 0.05), (0.1, 0.4, 1.0, 1.0)))
            pijl = self.marker(4, Marker.ARROW, x, y, 0.06, (0.14, 0.025, 0.025), (0.1, 0.4, 1.0, 1.0))
            pijl.pose.orientation.z = math.sin(yaw / 2)
            pijl.pose.orientation.w = math.cos(yaw / 2)
            ma.markers.append(pijl)
            ma.markers.append(self.marker(5, Marker.TEXT_VIEW_FACING, x, y, 0.22, (0, 0, 0.06),
                                          (1, 1, 1, 1), f"{self.modus}: {self.info}"))
        if self.doel and self.modus in ("VERKENNEN", "NAAR_HUIS"):
            ma.markers.append(self.marker(6, Marker.SPHERE, self.doel[0], self.doel[1], 0.03,
                                          (0.08, 0.08, 0.08), (1.0, 0.85, 0.0, 1.0)))
        for i, o in enumerate(self.objecten):
            d = max(o["grootte_cm"] / 100, 0.08)
            ma.markers.append(self.marker(100 + i, Marker.CYLINDER, o["x"], o["y"], 0.05,
                                          (d, d, 0.1), (0.7, 0.2, 0.9, 0.8)))
            ma.markers.append(self.marker(200 + i, Marker.TEXT_VIEW_FACING, o["x"], o["y"], 0.18,
                                          (0, 0, 0.06), (0.9, 0.6, 1.0, 1),
                                          f"object {i + 1} ({o['grootte_cm']} cm)"))
        self.pub_markers.publish(ma)


def main():
    rclpy.init()
    node = MaqueenSlam()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub_cmd.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
