# MicroPython voor de micro:bit op een micro:Maqueen V4.x
# Flash via https://python.microbit.org
#
# Protocol over USB-serieel (115200 baud):
#   PC -> micro:bit :  "M <links> <rechts>\n"   snelheden -255..255
#   micro:bit -> PC :  "S <lijnL> <lijnR> <afstand_cm> <kompas_graden>\n"  ~10x per seconde
# Alleen om met de USB-kabel te testen; de draadloze versie staat in ../microbit
# Veiligheid: komt er 0,5 s geen commando binnen, dan stoppen de motoren.

from microbit import i2c, pin1, pin2, pin13, pin14, uart, display, Image, compass
import utime
from machine import time_pulse_us

MAQUEEN_ADDR = 0x10


def motor(reg, speed):
    speed = max(-255, min(255, int(speed)))
    richting = 0 if speed >= 0 else 1
    i2c.write(MAQUEEN_ADDR, bytes([reg, richting, abs(speed)]))


def motors(links, rechts):
    motor(0x00, links)   # M1 = links
    motor(0x02, rechts)  # M2 = rechts


def afstand_cm():
    pin1.write_digital(0)
    utime.sleep_us(2)
    pin1.write_digital(1)
    utime.sleep_us(10)
    pin1.write_digital(0)
    t = time_pulse_us(pin2, 1, 30000)
    if t < 0:
        return -1
    return round(t / 58.0, 1)


motors(0, 0)
compass.heading()  # start kalibratie indien nodig
uart.init(baudrate=115200)
display.show(Image.HAPPY)

buffer = b""
laatste_cmd = utime.ticks_ms()
laatste_status = utime.ticks_ms()

while True:
    # Commando's lezen
    if uart.any():
        data = uart.read()
        if data:
            buffer += data
            while b"\n" in buffer:
                regel, buffer = buffer.split(b"\n", 1)
                delen = regel.strip().split()
                if len(delen) == 3 and delen[0] == b"M":
                    try:
                        motors(int(delen[1]), int(delen[2]))
                        laatste_cmd = utime.ticks_ms()
                    except ValueError:
                        pass

    nu = utime.ticks_ms()

    # Watchdog: stoppen als de PC zwijgt
    if utime.ticks_diff(nu, laatste_cmd) > 500:
        motors(0, 0)
        laatste_cmd = nu

    # Sensoren doorsturen
    if utime.ticks_diff(nu, laatste_status) >= 100:
        laatste_status = nu
        uart.write("S {} {} {} {}\n".format(
            pin13.read_digital(), pin14.read_digital(), afstand_cm(),
            compass.heading()))
