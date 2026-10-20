#!/usr/bin/env python3
"""SLAM, verkennen, objecten en naar huis rijden voor de micro:Maqueen.

Sensoren: 1 ultrasone afstandssensor, kompas, geschatte afgelegde weg.

Wat deze node doet
  * KAART:      elke afstandsmeting tekent een kegel in een rasterkaart (/map)
  * POSITIE:    na elk rondje meten wordt het rondje over de bestaande kaart
                gelegd (scan matching). Ligt het beter met een kleine
                verschuiving, dan corrigeert de node de positie (TF map->odom).
  * POSEGRAAF:  elk rondje is een knoop. Komt hij terug bij een eerder rondje,
                dan worden beide rondjes op elkaar gelegd (lussluiting). De
                hele graaf wordt dan geoptimaliseerd (Gauss-Newton) en de kaart
                opnieuw opgebouwd met de verbeterde posities. Bij thuiskomst
                doet hij een laatste rondje om de grote lus te sluiten.
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
from geometry_msgs.msg import Point, PoseStamped, TransformStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from scipy import ndimage
from sensor_msgs.msg import LaserScan, Range
from std_msgs.msg import Empty, Int8MultiArray, String
from tf2_ros import TransformBroadcaster

from logboek import Logboek
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


def t2v(M):
    return np.array([M[0, 2], M[1, 2], math.atan2(M[1, 0], M[0, 0])])


def rot(th):
    c, s = math.cos(th), math.sin(th)
    return np.array([[c, -s], [s, c]])


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
        self.L = Logboek(self.get_logger(), "slam")
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
        self.marge = p("inflate", 0.08).value
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
        self.kamer = None
        self.laatste_scan = []
        self.lege_plannen = 0
        self.huis_via_kaart = False
        self.eindscan = False
        self.tweede_poging = False
        self.keyframes = []           # rondjes: O (odom), X (kaart), metingen, punten
        self.randen = []              # (i, j, z, omega, lus)
        self.losse = []               # metingen zonder rondje: (x, y, yaw, r) in kaart
        self.traject_rel = []         # (rondje, x, y) relatief, voor herbouwen
        self.verboden = []            # onthouden obstakels (x, y): daar rijdt hij nooit meer heen
        self.verboden_rel = []
        self.huis = (0.0, 0.0, 0.0)   # thuispunt (x, y, richting) = start van de ronde
        self.huis_rel = (-1, 0.0, 0.0, 0.0)
        self.huis_gezet = False
        self.huis_index = 0           # waar in het traject de ronde begon
        self.na_achteruit = "PLAN"
        self.dichtbij = 0
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
        self.pub_kamer = self.create_publisher(String, "maqueen/kamer", 10)
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
        self.create_timer(2.0, self.hartslag)
        self.L.info("Klaar. Start met: ros2 param set /maqueen_slam mode verkennen")

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
            self.L.info(f"Knop {msg.data} op de robot -> {modus}")
            self.set_parameters([Parameter("mode", value=modus)])

    def zet_modus(self, modus):
        if modus == "stop":
            self.modus, self.info = "STIL", "gestopt"
            self.wissel("-")
            self.pad = []
            self.pub_cmd.publish(Twist())
            self.bewaar_traject()
        elif modus == "verkennen":
            if (not self.huis_gezet or self.modus == "KLAAR") and self.odom is not None:
                self.zet_huis()
            self.pub_reset.publish(Empty())
            self.modus, self.info = "VERKENNEN", "verkennen"
            self.verken_start = time.monotonic()
            self.mislukte_doelen = []
            self.lege_plannen = 0
            self.eindscan = False
            self.tweede_poging = False
            self.start_scan()
        elif modus == "naar_huis":
            hx, hy, _ = self.huis
            if self.odom is not None and math.hypot(self.pose_kaart()[0] - hx, self.pose_kaart()[1] - hy) < 0.12:
                self.modus, self.info = "KLAAR", "staat al thuis"
                self.L.info("Staat al thuis (binnen 12 cm van het startpunt)")
                return
            self.pub_reset.publish(Empty())
            self.huis_via_kaart = False
            self.eindscan = False
            self.tweede_poging = False
            self.modus, self.info = "NAAR_HUIS", "op weg naar huis"
            self.wissel("PLAN")
        elif modus == "reset_kaart":
            self.grid[:] = 0
            self.traject = []
            self.traject_rel = []
            self.keyframes = []
            self.randen = []
            self.losse = []
            self.verboden = []
            self.verboden_rel = []
            self.huis_gezet = False
            self.huis_index = 0
            self.objecten = []
            self.modus, self.info = "STIL", "kaart gewist"
            self.wissel("-")
            self.pub_cmd.publish(Twist())
        self.L.info(f"Modus gevraagd: {modus} -> nu {self.modus} ({self.info})")

    UITLEG = {
        "SCAN": "rondje meten", "PLAN": "route plannen", "VOLGEN": "route volgen",
        "ACHTERUIT": "achteruit van obstakel", "UITLIJNEN": "in startrichting draaien", "-": "stil",
    }

    def wissel(self, sub):
        if sub != self.sub:
            tekst = f"[{self.modus}] {self.sub} -> {sub} ({self.UITLEG.get(sub, sub)})"
            if {sub, self.sub} == {"PLAN", "VOLGEN"}:
                self.L.debug(tekst)   # route bijwerken gebeurt elke 3 s: niet in de gewone log
            else:
                self.L.info(tekst)
        self.sub = sub
        self.sub_start = time.monotonic()

    # ================= sensoren =================
    def on_odom(self, msg: Odometry):
        p = msg.pose.pose
        self.odom = (p.position.x, p.position.y, yaw_uit_quat(p.orientation))
        x, y, _ = self.pose_kaart()
        if not self.traject or math.hypot(x - self.traject[-1][0], y - self.traject[-1][1]) > 0.04:
            self.traject.append((x, y))
            if self.keyframes:
                k = len(self.keyframes) - 1
                rel = np.linalg.inv(self.keyframes[k]["O"]) @ matrix(*self.odom)
                self.traject_rel.append((k, rel[0, 2], rel[1, 2]))
            else:
                self.traject_rel.append((-1, x, y))


    def on_line(self, msg: Int8MultiArray):
        self.lijn = list(msg.data)

    def on_range(self, msg: Range):
        r = msg.range
        self.laatste_afstand = (r, time.monotonic())
        # pas een obstakel bij 2 korte metingen na elkaar (losse foute echo's negeren)
        if math.isfinite(r) and 0.04 <= r < self.veilig:
            self.dichtbij += 1
        else:
            self.dichtbij = 0
        if self.odom is None:
            return
        if self.sub == "SCAN":
            self.scan_buffer.append((*self.odom, r))
            return
        self.integreer_meting(self.pose_kaart(), r)
        if self.keyframes:
            kf = self.keyframes[-1]
            rel = t2v(np.linalg.inv(kf["O"]) @ matrix(*self.odom))
            kf["metingen"].append((rel[0], rel[1], rel[2], r))
        else:
            self.losse.append((*self.pose_kaart(), r))

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
            self.L.info(
                f"Positie gecorrigeerd: {dx*100:+.0f} cm, {dy*100:+.0f} cm, {math.degrees(th):+.0f} graden")

    def scan_afronden(self):
        metingen = self.scan_buffer
        self.scan_buffer = []
        if self.scan_match:
            self.match(metingen)

        # nieuw rondje (keyframe) in de posegraaf
        O = matrix(*self.odom)
        X = self.T @ O
        O_inv = np.linalg.inv(O)
        rel = [(*t2v(O_inv @ matrix(mx, my, myaw)), r) for mx, my, myaw, r in metingen]
        punten = [(rx + (self.sensor_x + r) * math.cos(ryaw), ry + (self.sensor_x + r) * math.sin(ryaw))
                  for rx, ry, ryaw, r in rel if math.isfinite(r) and 0.04 <= r <= self.max_range]
        k = len(self.keyframes)
        self.keyframes.append({"O": O, "X": X, "metingen": rel,
                               "punten": np.array(punten).reshape(-1, 2), "veld": None})
        if k > 0:
            z = t2v(np.linalg.inv(self.keyframes[k - 1]["X"]) @ X)
            self.randen.append((k - 1, k, z, np.array([400.0, 400.0, 100.0]), False))
        for x, y, yaw, r in metingen:
            self.integreer_meting(self.pose_kaart((x, y, yaw)), r)
        if self.zoek_lus(k):
            voor = self.keyframes[k]["X"].copy()
            self.optimaliseer()
            na = self.keyframes[k]["X"]
            self.T = na @ np.linalg.inv(O)
            self.herbouw_kaart()
            d = t2v(np.linalg.inv(voor) @ na)
            self.L.info(
                f"Kaart bijgewerkt na lussluiting: robot {math.hypot(d[0], d[1])*100:.0f} cm "
                f"en {math.degrees(d[2]):+.0f} graden verschoven")
        x, y, yaw_nu = self.pose_kaart()
        self.laatste_scan_pos = (x, y)
        self.laatste_scan = [(self.pose_kaart((mx, my, myaw))[2], r) for mx, my, myaw, r in metingen]
        self.publiceer_scan(metingen, yaw_nu)

    # ================= posegraaf =================
    def veld_van(self, kf):
        """Waarschijnlijkheidsveld rond de punten van een rondje (in eigen assen)."""
        if kf["veld"] is None:
            P = kf["punten"]
            res = 0.025
            lo = P.min(axis=0) - 0.5
            hi = P.max(axis=0) + 0.5
            w, h = (np.ceil((hi - lo) / res).astype(int) + 1)
            bezet = np.zeros((h, w), dtype=bool)
            ij = np.floor((P - lo) / res).astype(int)
            bezet[ij[:, 1], ij[:, 0]] = True
            d = ndimage.distance_transform_edt(~bezet) * res
            kf["veld"] = (np.exp(-(d ** 2) / (2 * 0.05 ** 2)), lo, res)
        return kf["veld"]

    def scan_op_scan(self, i, k, z0):
        """Leg rondje k op rondje i, rond de schatting z0 (k in assen van i)."""
        Pk = self.keyframes[k]["punten"]
        veld, lo, res = self.veld_van(self.keyframes[i])
        H, W = veld.shape
        stap = np.arange(-0.3, 0.3001, 0.03)
        DX, DY = [a.ravel() for a in np.meshgrid(stap, stap)]
        beste = (-1.0, None)
        for graden in np.arange(-15, 15.01, 3):
            th = z0[2] + math.radians(graden)
            Q = Pk @ rot(th).T
            X = Q[None, :, 0] + z0[0] + DX[:, None]
            Y = Q[None, :, 1] + z0[1] + DY[:, None]
            ix = np.floor((X - lo[0]) / res).astype(int)
            iy = np.floor((Y - lo[1]) / res).astype(int)
            binnen = (ix >= 0) & (ix < W) & (iy >= 0) & (iy < H)
            v = np.zeros(X.shape)
            v[binnen] = veld[iy[binnen], ix[binnen]]
            gem = v.mean(axis=1)
            j = int(np.argmax(gem))
            if gem[j] > beste[0]:
                beste = (gem[j], np.array([z0[0] + DX[j], z0[1] + DY[j], th]))
        return beste

    def zoek_lus(self, k):
        kf = self.keyframes[k]
        if len(kf["punten"]) < 15:
            return False
        pk = kf["X"][:2, 2]
        kandidaten = sorted(
            (np.hypot(*(self.keyframes[i]["X"][:2, 2] - pk)), i) for i in range(max(k - 2, 0)))
        gevonden = False
        for afstand, i in kandidaten[:3]:
            if afstand > 0.8 or len(self.keyframes[i]["punten"]) < 15:
                break
            z0 = t2v(np.linalg.inv(self.keyframes[i]["X"]) @ kf["X"])
            score, z = self.scan_op_scan(i, k, z0)
            if score >= 0.45:
                self.randen.append((i, k, z, np.array([1100.0, 1100.0, 400.0]), True))
                fout = math.hypot(z[0] - z0[0], z[1] - z0[1])
                self.L.info(
                    f"Lus gesloten: rondje {k} past op rondje {i} (score {score:.2f}, "
                    f"afwijking {fout*100:.0f} cm)")
                gevonden = True
        return gevonden

    def optimaliseer(self, iteraties=15):
        """Gauss-Newton op de posegraaf; rondje 0 ligt vast."""
        n = len(self.keyframes)
        x = np.array([t2v(kf["X"]) for kf in self.keyframes])
        for _ in range(iteraties):
            H = np.zeros((3 * n, 3 * n))
            b = np.zeros(3 * n)
            for i, j, z, om, _lus in self.randen:
                xi, xj = x[i], x[j]
                Ri, Rz = rot(xi[2]), rot(z[2])
                dt = xj[:2] - xi[:2]
                e = np.zeros(3)
                e[:2] = Rz.T @ (Ri.T @ dt - z[:2])
                e[2] = hoekverschil(xj[2] - xi[2] - z[2], 0)
                c, s_ = math.cos(xi[2]), math.sin(xi[2])
                dRiT = np.array([[-s_, c], [-c, -s_]])
                A = np.zeros((3, 3))
                B = np.zeros((3, 3))
                A[:2, :2] = -Rz.T @ Ri.T
                A[:2, 2] = Rz.T @ dRiT @ dt
                A[2, 2] = -1.0
                B[:2, :2] = Rz.T @ Ri.T
                B[2, 2] = 1.0
                Om = np.diag(om)
                si, sj = slice(3 * i, 3 * i + 3), slice(3 * j, 3 * j + 3)
                H[si, si] += A.T @ Om @ A
                H[si, sj] += A.T @ Om @ B
                H[sj, si] += B.T @ Om @ A
                H[sj, sj] += B.T @ Om @ B
                b[si] += A.T @ Om @ e
                b[sj] += B.T @ Om @ e
            H[0:3, 0:3] += np.eye(3) * 1e6
            dx = np.linalg.solve(H, -b).reshape(n, 3)
            x += dx
            x[:, 2] = np.arctan2(np.sin(x[:, 2]), np.cos(x[:, 2]))
            if np.abs(dx).max() < 1e-4:
                break
        for kf, v in zip(self.keyframes, x):
            kf["X"] = matrix(*v)

    def herbouw_kaart(self):
        """Kaart opnieuw tekenen met de verbeterde posities van alle rondjes."""
        self.grid[:] = 0
        for x, y, yaw, r in self.losse:
            self.integreer_meting((x, y, yaw), r)
        for kf in self.keyframes:
            X = kf["X"]
            for rx, ry, ryaw, r in kf["metingen"]:
                self.integreer_meting(tuple(t2v(X @ matrix(rx, ry, ryaw))), r)
        nieuw = []
        for k, rx, ry in self.traject_rel:
            if k < 0:
                nieuw.append((rx, ry))
            else:
                p = self.keyframes[k]["X"] @ np.array([rx, ry, 1.0])
                nieuw.append((float(p[0]), float(p[1])))
        self.traject = nieuw
        self.huis = self.uit_rondje(self.huis_rel)
        self.verboden = [self.uit_rondje((k, rx, ry, 0.0))[:2] for k, rx, ry, _ in self.verboden_rel]

    # ================= padplanning =================
    def plan_kaarten(self):
        # Alleen cellen die meerdere keren als bezet gemeten zijn tellen als muur:
        # losse foute echo's mogen de route niet blokkeren.
        bezet = self.grid > 1.2
        if self.verboden:
            onthouden = np.zeros_like(bezet)
            for vx, vy in self.verboden:
                c = self.cel(vx, vy)
                if c is not None:
                    onthouden[c] = True
            bezet |= ndimage.binary_dilation(onthouden, structure=schijf(2))
        r = int(math.ceil((self.robot_straal + self.marge) / self.res))
        opgeblazen = ndimage.binary_dilation(bezet, structure=schijf(r))
        rijdbaar = (self.grid < -0.1) & ~opgeblazen
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

    def plan_vrije_richting(self, x, y):
        """Reserve: richting uit het laatste rondje met veel vrije ruimte + onbekend gebied."""
        metingen = []
        for yaw_m, r in self.laatste_scan:
            r = r if math.isfinite(r) else self.vrij_bij_mis
            if r >= 0.04:
                metingen.append((yaw_m, min(r, self.max_range)))
        if len(metingen) < 10:
            return None
        venster = math.radians(12)
        beste = None
        for a, _ in metingen:
            vrij = min(r for b, r in metingen if abs(hoekverschil(a, b)) <= venster)
            afstand = min(0.6, vrij - self.veilig - 0.05)
            if afstand < 0.15:
                continue
            onbekend = 0
            totaal = 0
            for d in np.arange(0.1, vrij + 0.5, self.res):
                c = self.cel(x + d * math.cos(a), y + d * math.sin(a))
                if c is not None:
                    totaal += 1
                    onbekend += abs(self.grid[c]) < 0.05
            score = afstand + 1.5 * (onbekend / totaal if totaal else 0)
            if beste is None or score > beste[0]:
                beste = (score, a, afstand)
        if beste is None:
            return None
        _, a, afstand = beste
        doel = (x + afstand * math.cos(a), y + afstand * math.sin(a))
        return [doel], doel

    def naar_rondje(self, x, y, yaw=0.0):
        """Kaartpunt opslaan relatief t.o.v. het laatste rondje (schuift mee bij lussluiting)."""
        if self.keyframes:
            k = len(self.keyframes) - 1
            r = t2v(np.linalg.inv(self.keyframes[k]["X"]) @ matrix(x, y, yaw))
            return (k, r[0], r[1], r[2])
        return (-1, x, y, yaw)

    def uit_rondje(self, rel):
        k, rx, ry, ryaw = rel
        if k < 0:
            return rx, ry, ryaw
        v = t2v(self.keyframes[k]["X"] @ matrix(rx, ry, ryaw))
        return float(v[0]), float(v[1]), float(v[2])

    def zet_huis(self):
        self.huis = self.pose_kaart()
        self.huis_rel = self.naar_rondje(*self.huis)
        self.huis_gezet = True
        self.huis_index = len(self.traject)
        self.L.info(f"Thuispunt (start van de ronde): {self.huis[0]:.2f}, {self.huis[1]:.2f}")

    def markeer_verboden(self, x, y, reden):
        if any(math.hypot(x - vx, y - vy) < 0.05 for vx, vy in self.verboden):
            return
        self.verboden.append((x, y))
        self.verboden_rel.append(self.naar_rondje(x, y))
        self.L.info(f"{reden} onthouden op ({x:.2f}, {y:.2f}); daar rijd ik niet meer heen")

    def plan_huis(self, x, y):
        if self.huis_via_kaart:
            return self.plan_naar(x, y, self.huis[:2])
        return self.pad_via_traject(x, y) or self.plan_naar(x, y, self.huis[:2])

    def pad_via_traject(self, x, y):
        """Het gereden pad terug volgen, met lussen eruit geknipt."""
        tr = self.traject[self.huis_index:] + [(x, y)]
        if len(tr) < 2:
            return None
        terug = []
        i = len(tr) - 1
        while i > 0:
            px, py = tr[i]
            j = next(k for k in range(i + 1) if math.hypot(tr[k][0] - px, tr[k][1] - py) <= 0.2)
            if j >= i:
                j = i - 1
            terug.append(tr[j])
            i = j
        pad = []
        for p in terug:
            if not pad or math.hypot(p[0] - pad[-1][0], p[1] - pad[-1][1]) >= 0.1:
                pad.append(p)
        pad.append(self.huis[:2])
        return pad, self.huis[:2]

    def bewaar_traject(self):
        try:
            with open("/kaarten/traject.csv", "w") as f:
                f.write("x,y\n")
                for px, py in self.traject:
                    f.write(f"{px:.3f},{py:.3f}\n")
        except OSError:
            pass

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
        hx, hy, hyaw = self.huis
        nu = time.monotonic()
        duur = nu - self.sub_start
        cmd = Twist()

        if (self.cliff_stop and self.sub == "VOLGEN" and self.lijn is not None
                and all(v == self.zwart for v in self.lijn)):
            self.markeer_verboden(x + 0.08 * math.cos(yaw), y + 0.08 * math.sin(yaw), "Rand")
            if self.doel:
                self.mislukte_doelen.append(self.doel)
            self.na_achteruit = "SCAN"
            self.wissel("ACHTERUIT")

        if self.modus == "VERKENNEN" and nu - self.verken_start > self.max_tijd:
            self.L.info("Tijd om, terug naar huis")
            self.modus, self.info = "NAAR_HUIS", "tijd om, naar huis"
            self.huis_via_kaart = False
            self.wissel("PLAN")

        if self.sub == "SCAN":
            if self.scan_vorige_yaw is not None:
                self.scan_gedraaid += hoekverschil(self.odom[2], self.scan_vorige_yaw)
            self.scan_vorige_yaw = self.odom[2]
            if abs(self.scan_gedraaid) >= 2 * math.pi + 0.15:
                self.scan_afronden()
                if self.eindscan:
                    self.eindscan = False
                    x2, y2, _ = self.pose_kaart()
                    if math.hypot(x2 - self.huis[0], y2 - self.huis[1]) > 0.15 and not self.tweede_poging:
                        # na de correctie blijkt hij nog niet thuis: laatste stukje rijden
                        self.tweede_poging = True
                        self.info = "positie verbeterd, laatste stukje naar huis"
                        self.wissel("PLAN")
                        return
                    self.wissel("UITLIJNEN")
                    return
                self.wissel("PLAN")
            elif duur > 40:
                self.L.error("Geen rondje gedraaid in 40 s (kompas? MIN_PWM te laag?). Gestopt.")
                self.zet_modus("stop")
                return
            else:
                self.info = "rondje meten"
                cmd.angular.z = self.draaisnelheid

        elif self.sub == "PLAN":
            self.laatste_plan = nu
            if self.modus == "VERKENNEN":
                res = self.plan_frontier(x, y)
                if res is not None:
                    self.lege_plannen = 0
                else:
                    # Geen grens op de kaart gevonden: rij de richting met de meeste
                    # vrije ruimte uit het laatste rondje in. Pas na 3x niets is hij klaar.
                    self.lege_plannen += 1
                    res = self.plan_vrije_richting(x, y) if self.lege_plannen < 3 else None
                    if res is not None:
                        self.L.info("Geen grens op de kaart, ik rij de vrijste kant op")
                if res is None:
                    self.L.info("Alles verkend! Terug naar huis.")
                    self.modus, self.info = "NAAR_HUIS", "alles verkend, naar huis"
                    self.huis_via_kaart = False
                    res = self.plan_huis(x, y)
            else:
                res = self.plan_huis(x, y)
            if self.modus == "NAAR_HUIS":
                if math.hypot(x - hx, y - hy) < 0.12:
                    self.thuis()
                    return
                # Dichterbij kan niet (kaart rond het startpunt onzeker): hier stoppen
                if res is not None and math.hypot(res[1][0] - x, res[1][1] - y) < 0.1:
                    self.klaar("zo dicht mogelijk bij huis")
                    return
            if res is None:
                self.L.warn("Geen pad gevonden; opnieuw meten")
                self.start_scan()
            else:
                self.pad, self.doel = res
                self.info = ("rijdt naar onbekend gebied" if self.modus == "VERKENNEN"
                             else "rijdt naar huis")
                lengte = sum(math.hypot(b[0]-a[0], b[1]-a[1])
                             for a, b in zip([(x, y)] + self.pad[:-1], self.pad))
                bron = ("kaart" if self.modus == "VERKENNEN" or self.huis_via_kaart else "eigen spoor")
                self.L.debug(f"Route ({bron}): doel ({self.doel[0]:.2f},{self.doel[1]:.2f}), "
                             f"{lengte:.2f} m, {len(self.pad)} punten")
                self.wissel("VOLGEN")

        elif self.sub == "VOLGEN":
            while self.pad and math.hypot(self.pad[0][0] - x, self.pad[0][1] - y) < 0.08:
                self.pad.pop(0)
            if not self.pad:
                self.obstakels = 0
                if self.modus == "VERKENNEN":
                    self.start_scan()
                elif math.hypot(x - hx, y - hy) < 0.15 or math.hypot(
                        self.doel[0] - x, self.doel[1] - y) < 0.1:
                    self.thuis()
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
                vers = math.isfinite(r) and 0.04 <= r and nu - t_r < 0.4
                if vers and r < self.veilig and self.dichtbij >= 2:
                    sx = x + (self.sensor_x + r) * math.cos(yaw)
                    sy = y + (self.sensor_x + r) * math.sin(yaw)
                    self.markeer_verboden(sx, sy, "Obstakel")
                    self.obstakels += 1
                    self.na_achteruit = "PLAN"
                    if self.obstakels >= 3:
                        if self.doel:
                            self.mislukte_doelen.append(self.doel)
                        if self.modus == "NAAR_HUIS" and not self.huis_via_kaart:
                            self.L.info("Oude pad versperd, ik plan een route over de kaart")
                            self.huis_via_kaart = True
                        self.obstakels = 0
                        self.na_achteruit = "SCAN"
                    self.wissel("ACHTERUIT")
                    return self.pub_cmd.publish(cmd)
                # afremmen als er iets dichterbij komt
                factor = 1.0
                if vers and r < self.veilig + 0.3:
                    factor = max(0.35, (r - self.veilig) / 0.3)
                cmd.linear.x = self.rijsnelheid * factor
                cmd.angular.z = max(-self.draaisnelheid, min(self.draaisnelheid, 2.0 * fout))

        elif self.sub == "ACHTERUIT":
            if duur > 0.6:
                if self.na_achteruit == "SCAN":
                    self.start_scan()
                else:
                    self.wissel("PLAN")
            else:
                cmd.linear.x = -self.rijsnelheid

        elif self.sub == "UITLIJNEN":
            # zoals een Roomba: thuis in dezelfde richting als bij de start
            fout = hoekverschil(hyaw, yaw)
            if abs(fout) < 0.15 or duur > 10:
                self.klaar()
                return
            cmd.angular.z = math.copysign(self.draaisnelheid, fout)
            self.info = "terug in startrichting draaien"

        self.pub_cmd.publish(cmd)
        self.publiceer_status()

    def hartslag(self):
        """Om de 2 s een samenvatting in de log."""
        if self.odom is None:
            self.L.warn("Nog geen positie (odom) ontvangen: draait de bridge en is de robot verbonden?")
            return
        if self.modus in ("STIL", "KLAAR") and getattr(self, "_stil_gelogd", False):
            return
        self._stil_gelogd = self.modus in ("STIL", "KLAAR")
        x, y, yaw = self.pose_kaart()
        hx, hy, _ = self.huis
        r, t_r = self.laatste_afstand
        oud = time.monotonic() - t_r
        afst = f"{r*100:.0f} cm" if math.isfinite(r) else "geen echo"
        if oud > 1.0:
            afst += f" (OUD: {oud:.0f} s geen meting!)"
        doel = f"({self.doel[0]:.2f},{self.doel[1]:.2f}) nog {len(self.pad)} punten" if self.doel else "-"
        lussen = sum(1 for r_ in self.randen if r_[4])
        self.L.info(
            f"[{self.modus}/{self.sub}] pos ({x:.2f},{y:.2f}) {math.degrees(yaw):.0f}gr | "
            f"thuis op {math.hypot(x-hx, y-hy):.2f} m | afstand {afst} | doel {doel} | "
            f"rondjes {len(self.keyframes)}, lussen {lussen}, obstakels {len(self.verboden)}, "
            f"objecten {len(self.objecten)}")

    def thuis(self):
        """Bij thuiskomst nog een rondje meten: sluit de grote lus met het eerste rondje."""
        if self.keyframes and not self.eindscan and not self.tweede_poging and len(self.keyframes) > 1:
            self.eindscan = True
            self.info = "laatste rondje om de kaart te verbeteren"
            self.start_scan()
            self.pub_cmd.publish(Twist())
        else:
            self.wissel("UITLIJNEN")

    def klaar(self, tekst="thuis! verkennen klaar"):
        self.modus, self.info = "KLAAR", tekst
        self.wissel("-")
        self.pad = []
        self.pub_cmd.publish(Twist())
        self.bewaar_traject()
        self.L.info(f"Thuis aangekomen. {len(self.objecten)} object(en) gevonden. "
                               "Pad bewaard in kaarten/traject.csv")

    # ================= objecten =================
    def schat_kamer(self):
        """Oppervlakte en afmetingen van de vrije ruimte (lengte x breedte langs de hoofdassen)."""
        vrij = np.argwhere(self.grid < -0.5)
        if len(vrij) < 20:
            self.kamer = None
            return
        pts = (vrij[:, ::-1] + 0.5) * self.res + self.oorsprong
        midden = pts.mean(axis=0)
        _, vecs = np.linalg.eigh(np.cov((pts - midden).T))
        proj = (pts - midden) @ vecs
        maten = np.percentile(proj, 98, axis=0) - np.percentile(proj, 2, axis=0) + 2 * self.veilig
        lengte, breedte = float(max(maten)), float(min(maten))
        self.kamer = {"oppervlakte_m2": round(len(vrij) * self.res ** 2, 1),
                      "lengte_m": round(lengte, 1), "breedte_m": round(breedte, 1),
                      "objecten": len(self.objecten)}
        self.pub_kamer.publish(String(data=json.dumps(self.kamer)))

    def zoek_objecten(self):
        self.schat_kamer()
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
            self.L.debug(f"{len(objecten)} object(en) op de kaart")
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

        hx, hy, hyaw = self.huis
        ma.markers.append(self.marker(1, Marker.CYLINDER, hx, hy, 0.005, (0.16, 0.16, 0.01), (0.1, 0.8, 0.2, 0.9)))
        ma.markers.append(self.marker(2, Marker.TEXT_VIEW_FACING, hx, hy, 0.15, (0, 0, 0.08), (0.1, 0.9, 0.2, 1), "THUIS"))
        hp = self.marker(12, Marker.ARROW, hx, hy, 0.01, (0.12, 0.02, 0.02), (0.1, 0.9, 0.2, 1))
        hp.pose.orientation.z = math.sin(hyaw / 2)
        hp.pose.orientation.w = math.cos(hyaw / 2)
        ma.markers.append(hp)
        if self.verboden:
            vb = self.marker(11, Marker.CUBE_LIST, 0, 0, 0.03, (0.06, 0.06, 0.06), (1.0, 0.1, 0.1, 0.9))
            vb.points = [Point(x=float(vx), y=float(vy), z=0.0) for vx, vy in self.verboden]
            ma.markers.append(vb)
        if self.kamer:
            k = self.kamer
            ma.markers.append(self.marker(7, Marker.TEXT_VIEW_FACING, hx, hy - 0.25, 0.15, (0, 0, 0.07),
                                          (1, 1, 0.6, 1),
                                          f"kamer ~{k['lengte_m']} x {k['breedte_m']} m, "
                                          f"vrij {k['oppervlakte_m2']} m2, {k['objecten']} object(en)"))

        if self.odom is not None:
            x, y, yaw = self.pose_kaart()
            ma.markers.append(self.marker(3, Marker.CYLINDER, x, y, 0.03, (0.1, 0.1, 0.05), (0.1, 0.4, 1.0, 1.0)))
            pijl = self.marker(4, Marker.ARROW, x, y, 0.06, (0.14, 0.025, 0.025), (0.1, 0.4, 1.0, 1.0))
            pijl.pose.orientation.z = math.sin(yaw / 2)
            pijl.pose.orientation.w = math.cos(yaw / 2)
            ma.markers.append(pijl)
            ma.markers.append(self.marker(5, Marker.TEXT_VIEW_FACING, x, y, 0.22, (0, 0, 0.06),
                                          (1, 1, 1, 1), f"{self.modus}: {self.info}"))
        if len(self.traject) > 1:
            lijn = self.marker(8, Marker.LINE_STRIP, 0, 0, 0.01, (0.015, 0, 0), (0.2, 0.5, 1.0, 0.9))
            lijn.points = [Point(x=float(px), y=float(py), z=0.0) for px, py in self.traject]
            ma.markers.append(lijn)
        if self.keyframes:
            knopen = self.marker(9, Marker.SPHERE_LIST, 0, 0, 0.02, (0.05, 0.05, 0.05), (0.0, 0.9, 0.9, 0.9))
            knopen.points = [Point(x=float(kf["X"][0, 2]), y=float(kf["X"][1, 2]), z=0.0)
                             for kf in self.keyframes]
            ma.markers.append(knopen)
            lussen = self.marker(10, Marker.LINE_LIST, 0, 0, 0.02, (0.012, 0, 0), (1.0, 0.2, 0.2, 0.9))
            for i, j, _z, _om, lus in self.randen:
                if lus:
                    for kk in (i, j):
                        X = self.keyframes[kk]["X"]
                        lussen.points.append(Point(x=float(X[0, 2]), y=float(X[1, 2]), z=0.0))
            if lussen.points:
                ma.markers.append(lussen)
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