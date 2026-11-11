"""Render a video of the RV-100 mechanical drive: straight runs, turning, moving at an angle, a ramp, a door
threshold and tip-over stability, with live wheel speeds, motor torque and suspension travel.

    python drive_video.py                  # writes Simulation/Drive/RV-100_drive.mp4
    python drive_video.py --stills         # also saves one PNG per segment
    python drive_video.py --only 6,7       # render just some segments (1-based), for checking

Needs numpy, trimesh, pyvista and imageio-ffmpeg (the %LOCALAPPDATA%\\rvcv venv has them).
Geometry is the per-component STLs from export_fov_stl.py, so every part sits where the CAD puts it. This is
a kinematic and quasi-static illustration, not a dynamics simulation: the body pose on uneven ground comes
from the wheel and caster contacts, the front caster spring takes up what the rigid supports cannot, and
motor torque and current use the same model and numbers as Motion/hand_calcs_drivetrain.txt.
"""
import argparse
import textwrap
import glob
import math
import os
import sys

import numpy as np
import pyvista as pv
import trimesh

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from export_urdf import ROOT, STL_DIR, SW_TO_ROS, parse_name, read_globals  # noqa: E402
from bay_video import COLOUR, hex_rgb, smooth, translate, cam_at  # noqa: E402

FPS = 30
G = read_globals()
mm = 1e-3
g0 = 9.81

# ---- robot numbers (CAD, globals, validation report)
MASS = 8.67                                   # kg, MassProps/mass_com_tipover.txt
COM = np.array([-2.0, 0.1, 145.9]) * mm       # ROS, floor under the axle
IZZ = 0.2145                                  # kg m^2 about the COM
R_W = G["WHEEL_D"] / 2 * mm                   # 0.072
TRACK = G["TRACK"] * mm                       # 0.365
A_Z = G["AXLE_Z"] * mm                        # axle height
R_C = G["CASTER_D"] / 2 * mm                  # caster wheel radius
TRAIL = G["CASTER_TRAIL"] * mm
SWIVEL_F, SWIVEL_R = G["CASTER_X"] * mm, -G["CASTER_X"] * mm
XF, XR = SWIVEL_F - TRAIL, SWIVEL_R - TRAIL   # caster contacts when trailing (driving forward)
TRAVEL = (G["SPRING_L"] - G["SPRING_SEAT"]) * mm
K_SPRINGS = 2 * G["SPRING_K"] / mm            # N/m, both springs
N_WHEEL = 41.9                                # N per drive wheel, static split
CRR, ETA = 0.02, 0.9
TAU_CONT, TAU_INST, TAU_STALL, I0, ISTALL = 0.98, 2.45, 2.65, 0.2, 5.5
TIP_ANGLE = 33.2                              # deg, rear-left/right edge, drawer closed

SKINS = {"RV-313", "RV-314", "RV-315", "RV-305"}
GREEN, AMBER, RED, BLUE = "#16a34a", "#d97706", "#dc2626", "#2563eb"


# ---------------------------------------------------------------------------------------------------- maths
def rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1.0]])


def ry(b):
    c, s = math.cos(b), math.sin(b)
    return np.array([[c, 0, s, 0], [0, 1, 0, 0], [-s, 0, c, 0], [0, 0, 0, 1.0]])


def rot_axis(u, a):
    x, y, z = u / np.linalg.norm(u)
    c, s, C = math.cos(a), math.sin(a), 1 - math.cos(a)
    R = np.eye(4)
    R[:3, :3] = [[c + x * x * C, x * y * C - z * s, x * z * C + y * s],
                 [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
                 [z * x * C - y * s, z * y * C + x * s, c + z * z * C]]
    return R


def about(M, centre):
    return translate(centre) @ M @ translate(-np.asarray(centre, float))


def body_tf(x, y, yaw, za=A_Z, phi=0.0):
    """base_footprint pose: axle centre at (x, y, za), heading yaw, nose-up pitch phi about the axle."""
    return translate((x, y, za)) @ rz(yaw) @ ry(-phi) @ translate((0, 0, -A_Z))


def hull2d(p):
    p = np.unique(np.round(p, 5), axis=0)
    p = p[np.lexsort((p[:, 1], p[:, 0]))]

    def half(pts):
        h = []
        for q in pts:
            while len(h) >= 2 and (h[-1][0] - h[-2][0]) * (q[1] - h[-2][1]) - (h[-1][1] - h[-2][1]) * (q[0] - h[-2][0]) <= 0:
                h.pop()
            h.append(q)
        return h
    lo, hi = half(p), half(p[::-1])
    return np.array(lo[:-1] + hi[:-1])


# --------------------------------------------------------------------------------------------- motion
def trap(L, vmax, amax, dmax=None):
    """Trapezoidal profile over distance L. Returns per-frame (s, v, a)."""
    dmax = dmax or amax
    sign = 1 if L >= 0 else -1
    L = abs(L)
    v_peak = min(vmax, math.sqrt(2 * L * amax * dmax / (amax + dmax)))
    t1, t3 = v_peak / amax, v_peak / dmax
    t2 = (L - v_peak**2 / (2 * amax) - v_peak**2 / (2 * dmax)) / v_peak
    T = t1 + t2 + t3
    out = []
    for k in range(int(math.ceil(T * FPS)) + 1):
        t = min(T, k / FPS)
        if t < t1:
            s, v, a = 0.5 * amax * t * t, amax * t, amax
        elif t < t1 + t2:
            s, v, a = v_peak**2 / (2 * amax) + v_peak * (t - t1), v_peak, 0.0
        else:
            u = t - t1 - t2
            s, v, a = v_peak**2 / (2 * amax) + v_peak * t2 + v_peak * u - 0.5 * dmax * u * u, v_peak - dmax * u, -dmax
        if k / FPS >= T:
            s, v, a = L, 0.0, 0.0
        out.append((sign * s, sign * v, sign * a))
    return out


class Mover:
    """Integrates primitives (straight, spin, arc) into per-frame states."""

    def __init__(self, x=0.0, y=0.0, yaw=0.0):
        self.x, self.y, self.yaw = x, y, yaw
        self.frames = []

    def _push(self, v, w, a, al):
        self.frames.append(dict(x=self.x, y=self.y, yaw=self.yaw, v=v, w=w, a=a, al=al))

    def hold(self, sec):
        for _ in range(int(sec * FPS)):
            self._push(0.0, 0.0, 0.0, 0.0)

    def straight(self, L, vmax, amax, dmax=None):
        x0, y0, prev = self.x, self.y, 0.0
        for s, v, a in trap(L, vmax, amax, dmax):
            self.x, self.y = x0 + s * math.cos(self.yaw), y0 + s * math.sin(self.yaw)
            self._push(v, 0.0, a, 0.0)

    def spin(self, ang, wmax, almax):
        a0 = self.yaw
        for s, w, al in trap(ang, wmax, almax):
            self.yaw = a0 + s
            self._push(0.0, w, 0.0, al)

    def arc(self, R, ang, vmax, amax):
        """Arc of radius R (left turn for ang > 0)."""
        x0, y0, a0 = self.x, self.y, self.yaw
        sg = 1 if ang > 0 else -1
        cx, cy = x0 - sg * R * math.sin(a0), y0 + sg * R * math.cos(a0)
        k = sg / R
        for s, v, a in trap(R * abs(ang), vmax, amax):
            th = a0 + k * s
            self.x, self.y, self.yaw = cx + sg * R * math.sin(th), cy - sg * R * math.cos(th), th
            self._push(v, k * v, a, k * a)
        return cx, cy


# --------------------------------------------------------------------------------------------- terrain
def ground_flat(x):
    return np.zeros_like(np.asarray(x, float))


RAMP_X0, RAMP_DEG, RAMP_L = 0.40, 5.0, 0.70
RAMP_H = RAMP_L * math.tan(math.radians(RAMP_DEG))


def ground_ramp(x):
    x = np.asarray(x, float)
    return np.clip(x - RAMP_X0, 0, RAMP_L) * math.tan(math.radians(RAMP_DEG))


STEP_X0, STEP_W, STEP_H = 0.30, 0.06, 0.015


def ground_step(x):
    x = np.asarray(x, float)
    return np.where((x >= STEP_X0) & (x <= STEP_X0 + STEP_W), STEP_H, 0.0)


def centre_height(gfun, x, r):
    """Height of a wheel centre of radius r rolling over gfun at x; also the contact x."""
    xs = np.linspace(x - r * 0.999, x + r * 0.999, 401)
    z = gfun(xs) + np.sqrt(np.maximum(0.0, r * r - (xs - x) ** 2))
    i = int(np.argmax(z))
    return z[i], xs[i]


def world_pt(Xa, za, phi, bx, bz):
    return Xa + bx * math.cos(phi) - (bz - A_Z) * math.sin(phi), za + bx * math.sin(phi) + (bz - A_Z) * math.cos(phi)


SUPPORTS = {"W": (0.0, A_Z, R_W), "R": (XR, R_C, R_C), "F": (XF, R_C, R_C), "Fb": (XF, R_C + TRAVEL, R_C)}


def gap(gfun, Xa, za, phi, key):
    bx, bz, r = SUPPORTS[key]
    wx, wz = world_pt(Xa, za, phi, bx, bz)
    return wz - centre_height(gfun, wx, r)[0]


def pose_on(gfun, Xa, A, B):
    """Axle height and pitch with supports A and B touching the ground."""
    bxA, bzA, rA = SUPPORTS[A]

    def za_for(phi):
        lo, hi = 0.0, 0.3
        for _ in range(40):                      # solve A on the ground for this pitch
            mid = (lo + hi) / 2
            lo, hi = (mid, hi) if gap(gfun, Xa, mid, phi, A) < 0 else (lo, mid)
        return (lo + hi) / 2

    lo, hi = -0.35, 0.35
    s_lo = np.sign(gap(gfun, Xa, za_for(lo), lo, B))
    for _ in range(40):
        mid = (lo + hi) / 2
        s = np.sign(gap(gfun, Xa, za_for(mid), mid, B))
        lo, hi = (mid, hi) if s == s_lo else (lo, mid)
    phi = (lo + hi) / 2
    return za_for(phi), phi


def terrain_pose(gfun, Xa):
    """Quasi-static pose: wheels and rigid rear caster carry the body; the front spring takes up to TRAVEL."""
    za, phi = pose_on(gfun, Xa, "W", "R")
    comp = -gap(gfun, Xa, za, phi, "F")
    lifted = 0.0
    if comp > TRAVEL:                            # spring bottomed: front caster becomes a rigid support
        za, phi = pose_on(gfun, Xa, "W", "Fb")
        if gap(gfun, Xa, za, phi, "R") < -1e-5:  # rear would dig in: robot rocks onto both casters
            za, phi = pose_on(gfun, Xa, "R", "Fb")
            lifted = gap(gfun, Xa, za, phi, "W")
        comp = TRAVEL
    _, cx = centre_height(gfun, Xa, R_W)
    return za, phi, max(0.0, comp), max(0.0, lifted), (cx - Xa) if lifted < 1e-4 else 0.0


# ------------------------------------------------------------------------------------------- drivetrain
def wheel_load(v, a, al, dx, side, lifted):
    """Torque (N m) and current (A) for one motor. side -1 left, +1 right. Same model as the hand calcs."""
    if lifted > 1e-4:
        return 0.0, I0
    F = MASS / 2 * a + side * IZZ * al / TRACK + math.copysign(CRR * N_WHEEL, v) * (abs(v) > 1e-4)
    tau = (F * R_W + N_WHEEL * dx) / ETA
    return tau, I0 + (ISTALL - I0) * min(1.0, abs(tau) / TAU_STALL)


def rpm(v):
    return v / R_W * 60 / (2 * math.pi)


# ------------------------------------------------------------------------------------------------- scene
def prism_xz(pts_xz, y0, y1):
    pts = np.array([[x, y0, z] for x, z in pts_xz])
    poly = pv.PolyData(pts, faces=np.r_[len(pts), np.arange(len(pts))])
    return poly.extrude((0, y1 - y0, 0), capping=True)


def poly_xy(pts, z=0.0, closed=True):
    P = np.c_[pts, np.full(len(pts), z)]
    return pv.lines_from_points(np.vstack([P, P[:1]]) if closed else P)


def fill_xy(pts, z=0.0):
    P = np.c_[pts, np.full(len(pts), z)]
    return pv.PolyData(P, faces=np.r_[len(P), np.arange(len(P))])


def circle_xz(r, n=48, y=0.0):
    t = np.linspace(0, 2 * math.pi, n + 1)
    return pv.lines_from_points(np.c_[r * np.cos(t), np.full(n + 1, y), r * np.sin(t)])


def disc_xz(r, n=48, y=0.0):
    t = np.linspace(0, 2 * math.pi, n, endpoint=False)
    P = np.c_[r * np.cos(t), np.full(n, y), r * np.sin(t)]
    return pv.PolyData(P, faces=np.r_[n, np.arange(n)])


def ring_xy(r, z=0.002, n=120):
    t = np.linspace(0, 2 * math.pi, n + 1)
    return pv.lines_from_points(np.c_[r * np.cos(t), r * np.sin(t), np.full(n + 1, z)])


def arrow(start, d, scale):
    return pv.Arrow(start=start, direction=d, scale=scale, tip_length=0.28, tip_radius=0.09, shaft_radius=0.035)


# --------------------------------------------------------------------------------------------- segments
def build_segments(R_SWEEP):
    S = []
    # 1 intro
    m = Mover()
    m.hold(7.0)
    S.append(dict(key="intro", title="RoamView RV-100 drive", text="Differential drive: two 144 mm wheels on a "
                  "365 mm track, a sprung caster in front and a rigid one behind. 8.67 kg, centre of mass 146 mm up.",
                  frames=m.frames, plan="plan", ghost=True,
                  cam=lambda f, t: cam_at((0, 0, 0.16), 1.45, 26, -150 + 6 * t)))
    # 2 straight
    m = Mover(-0.35, 0.0, 0.0)
    m.hold(0.8)
    m.straight(0.575, 0.5, 0.5, 1.0)
    m.hold(1.4)
    S.append(dict(key="straight", title="Straight line", text="Both wheels at the same speed: 0.5 m/s is 66 rpm. "
                  "Braking at 1 m/s² stops it in 125 mm; the front springs take the 11 N nose dive.",
                  frames=m.frames, plan="side_flat",
                  cam=lambda f, t: cam_at((f["x"] + 0.05, f["y"], 0.24), 1.95, 12, -112)))
    # 3 spin
    m = Mover(0.0, 0.0, 0.0)
    m.hold(0.8)
    m.spin(2 * math.pi, 1.2, 1.5)
    m.hold(0.8)
    S.append(dict(key="spin", title="Turning on the spot", text="The wheels run in opposite directions and the robot "
                  f"pivots on the axle centre. It sweeps a {2 * R_SWEEP / mm:.0f} mm circle, so it can turn round in "
                  "a narrow corridor.", frames=m.frames, plan="plan", ring=R_SWEEP, icr=True,
                  cam=lambda f, t: cam_at((0, 0, 0.12), 1.85, 46, -120 + 3 * t)))
    # 4 arc
    m = Mover(-0.25, -0.45, 0.0)
    m.hold(0.8)
    centre = m.arc(0.5, math.pi / 2, 0.3, 0.4)
    m.hold(1.0)
    S.append(dict(key="arc", title="Turning on an arc", text="The outer wheel runs faster than the inner one. On a "
                  "0.5 m radius at 0.3 m/s the wheels turn at 25 and 54 rpm, about the instant centre of rotation (ICR).",
                  frames=m.frames, plan="plan", icr=True, plan_focal=(0.02, -0.15), plan_scale=0.62,
                  cam=lambda f, t: cam_at((0.0, -0.15, 0.05), 2.2, 50, -90 + 2 * t)))
    # 5 at an angle
    m = Mover(-0.55, -0.35, 0.0)
    m.hold(1.6)
    m.spin(math.radians(35), 1.0, 1.5)
    m.hold(0.3)
    m.straight(0.75, 0.35, 0.4)
    m.hold(0.3)
    m.arc(0.35, -math.radians(35), 0.25, 0.4)
    m.hold(1.0)
    S.append(dict(key="angle", title="Moving at an angle", text="A differential drive cannot slide sideways. To move "
                  "at an angle it turns to the new heading, drives straight along it, then curves back onto its course.",
                  frames=m.frames, plan="plan", icr=True, plan_focal=(-0.05, 0.0), plan_scale=0.70,
                  cam=lambda f, t: cam_at((-0.05, 0.0, 0.05), 2.1, 52, -100 + 2 * t)))
    # 6 ramp
    m = Mover(-0.10, 0.0, 0.0)
    m.hold(0.6)
    m.straight(1.45, 0.3, 0.5)
    m.hold(0.8)
    S.append(dict(key="ramp", title=f"Climbing a {RAMP_DEG:.0f}° ramp", text="Each motor needs about 0.36 N·m at "
                  "0.3 m/s, 37 % of its continuous rating. The front caster spring follows the change in slope.",
                  frames=m.frames, plan="side", ground=ground_ramp,
                  cam=lambda f, t: cam_at((f["x"] + 0.05, 0, f["za"] + 0.18), 2.0, 10, -105)))
    # 7 threshold
    m = Mover(-0.05, 0.0, 0.0)
    m.hold(0.6)
    m.straight(0.68, 0.12, 0.3)
    m.hold(0.8)
    S.append(dict(key="step", title=f"Crossing a {STEP_H / mm:.0f} mm door threshold", text="The front caster rides up "
                  f"on its springs ({TRAVEL / mm:.0f} mm of travel), the drive wheels climb the edge, then the rigid "
                  "rear caster follows.", frames=m.frames, plan="side", ground=ground_step, ghost=True,
                  cam=lambda f, t: cam_at((f["x"] + 0.03, 0, 0.12), 1.45, 6, -96)))
    # 8 tip
    m = Mover()
    m.hold(7.0)
    S.append(dict(key="tip", title="Tip-over stability", text="With the rear caster turned inward (worst case) it "
                  "stays up to 33.2°; 28.0° and 24.3° with the battery drawer out 150 and 250 mm.",
                  frames=m.frames, plan="tip", ghost=True,
                  cam=lambda f, t: cam_at((0, 0, 0.16), 2.0, 9, 58 + 2 * t)))
    # 9 summary
    m = Mover()
    m.hold(6.0)
    S.append(dict(key="summary", title="Drive characteristics", text="Top speed about 1.1 m/s under load; Nav2 runs it "
                  "at 0.3 to 0.5 m/s, well inside the motor ratings.", frames=m.frames, plan="plan",
                  cam=lambda f, t: cam_at((0, 0, 0.20), 1.6, 22, -70 + 5 * t)))

    # terrain poses
    for s in S:
        gfun = s.get("ground")
        for f in s["frames"]:
            if gfun is not None:
                f["za"], f["phi"], f["comp"], f["lift"], f["dx"] = terrain_pose(gfun, f["x"])
            else:
                f.update(za=A_Z, phi=0.0, comp=0.0, lift=0.0, dx=0.0)
        if s["key"] == "straight":               # braking dive on the front springs, lightly filtered
            c = 0.0
            for f in s["frames"]:
                dec = max(0.0, -f["a"])
                target = MASS * dec * COM[2] / XF / K_SPRINGS
                c += (target - c) * (1 - math.exp(-1 / FPS / 0.06))
                f["comp"], f["phi"] = c, -math.atan2(c, XF)
        if s["key"] == "tip":
            n = len(s["frames"])
            for k, f in enumerate(s["frames"]):
                t = k / FPS
                f["tilt"] = TIP_ANGLE * (smooth((t - 0.6) / 3.6) - smooth((t - 5.6) / 1.2))
    return S


# ----------------------------------------------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--out", default=os.path.join(ROOT, "Simulation", "Drive", "RV-100_drive.mp4"))
    ap.add_argument("--stills", action="store_true")
    ap.add_argument("--only", default="")
    ap.add_argument("--size", default="1920x1080")
    a = ap.parse_args()
    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    W, H = (int(v) for v in a.size.split("x"))

    # ---- parts
    parts, allv = [], []
    for path in sorted(glob.glob(os.path.join(STL_DIR, "RV-100 - *.STL"))):
        pn, module = parse_name(path)
        if pn not in COLOUR:
            continue
        tm = trimesh.load(path, force="mesh")
        tm.apply_transform(SW_TO_ROS)
        key = pn
        if pn == "RV-504":
            key = "caster_f" if module.startswith("RV-120") else "caster_r"
        parts.append(dict(pn=pn, key=key, mesh=pv.wrap(tm), centre=tm.bounds.mean(axis=0), rgb=hex_rgb(COLOUR[pn]),
                          top=tm.bounds[1][2], bot=tm.bounds[0][2]))
        allv.append(tm.vertices)
    V = np.vstack(allv)
    R_SWEEP = float(np.hypot(V[:, 0], V[:, 1]).max())
    HULL_XY = hull2d(V[:, :2])
    HULL_XZ = hull2d(V[V[:, 2] < 0.30][:, [0, 2]])
    spring_top = max(p["top"] for p in parts if p["pn"] == "RV-554")
    spring_len = spring_top - min(p["bot"] for p in parts if p["pn"] == "RV-554")
    print(f"{len(parts)} parts, swept radius {R_SWEEP / mm:.0f} mm, body {np.ptp(V[:, 0]) / mm:.0f} x "
          f"{np.ptp(V[:, 1]) / mm:.0f} x {np.ptp(V[:, 2]) / mm:.0f} mm")

    S = build_segments(R_SWEEP)
    only = {int(v) for v in a.only.split(",") if v}

    pl = pv.Plotter(shape=(2, 2), groups=[(np.s_[:], 0)], col_weights=[2.35, 1], row_weights=[1.15, 0.85],
                    off_screen=True, window_size=(W, H), border=False)

    # ================================================================ main view
    pl.subplot(0, 0)
    pl.set_background("#eef1f5", top="#ffffff", all_renderers=False)
    pl.enable_depth_peeling(number_of_peels=8)
    floor = pl.add_mesh(pv.Plane(center=(0.25, 0, -0.0006), direction=(0, 0, 1), i_size=6, j_size=5),
                        color="#d9dde3", ambient=0.25)
    grid = pl.add_mesh(pv.merge([pv.Line((v, -2.5, 0), (v, 2.5, 0)) for v in np.arange(-2.75, 3.26, 0.25)] +
                                [pv.Line((-2.75, v, 0), (3.25, v, 0)) for v in np.arange(-2.5, 2.51, 0.25)]),
                       color="#c3c8d0", line_width=1)
    ramp_mesh = prism_xz([(RAMP_X0, 0), (RAMP_X0 + RAMP_L, RAMP_H), (2.6, RAMP_H), (2.6, 0)], -0.55, 0.55)
    ramp = pl.add_mesh(ramp_mesh, color="#b9a98f", ambient=0.3, diffuse=0.7)
    step = pl.add_mesh(pv.Box((STEP_X0, STEP_X0 + STEP_W, -0.55, 0.55, 0, STEP_H)), color="#8b6b4a", ambient=0.3)
    table = pl.add_mesh(pv.Box((-0.42, 0.42, -0.42, 0.42, -0.014, -0.0005)), color="#cbd2da", ambient=0.3)
    edge_p1, edge_p2 = np.array([0.0, TRACK / 2, 0.0]), np.array([XR + 2 * TRAIL, 0.0, 0.0])   # rear caster inward
    u_edge = edge_p2 - edge_p1
    tip_line = pl.add_mesh(pv.Line(edge_p1 - 1.6 * u_edge, edge_p2 + 1.6 * u_edge), color=RED, line_width=5)
    for act in (ramp, step, table, tip_line):
        act.visibility = False

    for p in parts:
        p["actor"] = pl.add_mesh(p["mesh"], color=p["rgb"], ambient=0.22, diffuse=0.75, specular=0.2,
                                 smooth_shading=False)
    com_ball = pl.add_mesh(pv.Sphere(radius=0.013, center=COM), color="#facc15", ambient=0.6)
    plumb = pl.add_mesh(pv.Line((0, 0, 0), (0, 0, 1)), color="#a16207", line_width=4)
    plumb_dot = pl.add_mesh(pv.Sphere(radius=0.008), color="#a16207")
    ring3 = pl.add_mesh(ring_xy(R_SWEEP), color=BLUE, line_width=4)
    icr_pole = pl.add_mesh(pv.Cylinder(center=(0, 0, 0.06), direction=(0, 0, 1), radius=0.009, height=0.12),
                           color=RED)
    icr_line = pl.add_mesh(pv.Line((0, 0, 0), (1, 0, 0)), color=RED, line_width=3)
    trail3 = pl.add_mesh(pv.PolyData(np.zeros((1, 3))), color=BLUE, line_width=4)
    heading_line = pl.add_mesh(pv.Line((-0.55, -0.35, 0.002), (-0.55 + 1.3 * math.cos(math.radians(35)),
                                                               -0.35 + 1.3 * math.sin(math.radians(35)), 0.002)),
                               color=AMBER, line_width=3)
    side_arrow = pl.add_mesh(arrow((-0.55, -0.35, 0.03), (0, 1, 0), 0.32), color=RED, opacity=0.85)
    for act in (com_ball, plumb, plumb_dot, ring3, icr_pole, icr_line, heading_line, side_arrow):
        act.visibility = False

    def labels(items, name):
        acts = []
        for i, (pt, txt) in enumerate(items):
            act = pl.add_point_labels([pt], [txt], font_size=15, point_size=8, point_color="#111827",
                                      text_color="#111827", shape="rounded_rect", shape_color="#ffffff",
                                      shape_opacity=0.85, always_visible=True, show_points=False,
                                      name=f"{name}{i}")
            act.visibility = False
            acts.append(act)
        return acts
    seg_labels = {
        "intro": labels([((0.0, TRACK / 2 + 0.03, 0.15), "144 mm wheel"), ((0.0, -TRACK / 2, 0.0), "track 365 mm"),
                         (COM, "centre of mass, 146 mm up"), ((XF, 0, 0.03), "sprung front caster"),
                         ((XR, 0, 0.03), "rigid rear caster")], "li"),
        "spin": labels([((R_SWEEP * 0.71, -R_SWEEP * 0.71, 0.0), f"swept radius {R_SWEEP / mm:.0f} mm")], "ls"),
        "arc": labels([((-0.25, 0.05, 0.12), "ICR, R 0.5 m")], "la"),
        "angle": labels([((-0.55, -0.03, 0.03), "no sideways motion"),
                         ((-0.55 + 1.2 * math.cos(math.radians(35)), -0.35 + 1.2 * math.sin(math.radians(35)), 0.01),
                          "35° course")], "lg"),
        "tip": labels([(COM, "centre of mass")], "lt"),
    }
    head = pl.add_text("", position=(32, H - 72), font_size=21, color="#111827", font="arial")
    body = pl.add_text("", position=(32, H - 108), font_size=11, color="#374151", font="arial")
    warn = pl.add_text("", position=(32, H - 146), font_size=12, color=RED, font="arial")
    count = pl.add_text("", position=(int(W * 0.70) - 110, 18), font_size=11, color="#6b7280", font="arial")
    pl.add_text("RV-100 CAD, Revision A. Kinematic and quasi-static illustration; torque and current from the "
                "drivetrain hand calcs.", position=(32, 18), font_size=8, color="#6b7280", font="arial")
    for t_act, op in ((head, 0.88), (body, 0.88), (warn, 0.88)):
        tp = t_act.GetTextProperty()
        tp.SetBackgroundColor(1, 1, 1)
        tp.SetBackgroundOpacity(op)
    cam = pl.camera

    # ================================================================ schematic view (plan / side)
    pl.subplot(0, 1)
    pl.set_background("#f8fafc", all_renderers=False)
    pl.enable_parallel_projection()
    sch_title = pl.add_text("", position=(12, int(H * 1.15 / 2) - 30), font_size=10, color="#111827", font="arial")
    plan = dict(
        grid=pl.add_mesh(pv.merge([pv.Line((v, -2.5, -0.01), (v, 2.5, -0.01)) for v in np.arange(-2.5, 2.51, 0.25)] +
                                  [pv.Line((-2.5, v, -0.01), (2.5, v, -0.01)) for v in np.arange(-2.5, 2.51, 0.25)]),
                         color="#dde2e8", line_width=1),
        body=pl.add_mesh(fill_xy(HULL_XY), color="#e5e7eb"),
        outline=pl.add_mesh(poly_xy(HULL_XY, 0.001), color="#374151", line_width=2),
        wheels=pl.add_mesh(pv.Box((-R_W, R_W, TRACK / 2 - 0.026, TRACK / 2 + 0.026, 0.002, 0.003)) +
                           pv.Box((-R_W, R_W, -TRACK / 2 - 0.026, -TRACK / 2 + 0.026, 0.002, 0.003)), color="#111827"),
        axle=pl.add_mesh(pv.Line((0, -TRACK / 2, 0.004), (0, TRACK / 2, 0.004)), color="#6b7280", line_width=2),
        nose=pl.add_mesh(arrow((0.05, 0, 0.004), (1, 0, 0), 0.14), color="#374151"),
        casters=pl.add_mesh(pv.Disc(center=(XF, 0, 0.004), inner=0, outer=0.012, c_res=24) +
                            pv.Disc(center=(XR, 0, 0.004), inner=0, outer=0.012, c_res=24), color="#6b7280"),
    )
    plan_moving = ("body", "outline", "wheels", "axle", "nose", "casters")
    plan_l = pl.add_mesh(arrow((0, 0, 0), (1, 0, 0), 0.1), color=GREEN)
    plan_r = pl.add_mesh(arrow((0, 0, 0), (1, 0, 0), 0.1), color=GREEN)
    plan_icr = pl.add_mesh(pv.Disc(inner=0, outer=0.018, c_res=24), color=RED)
    plan_icr_lines = pl.add_mesh(pv.Line((0, 0, 0), (1, 0, 0)), color=RED, line_width=2)
    plan_trail = pl.add_mesh(pv.PolyData(np.zeros((1, 3))), color=BLUE, line_width=3)
    plan_ring = pl.add_mesh(ring_xy(R_SWEEP, 0.005), color=BLUE, line_width=2)
    plan_head = pl.add_mesh(pv.Line((-0.55, -0.35, 0.005), (-0.55 + 1.3 * math.cos(math.radians(35)),
                                                            -0.35 + 1.3 * math.sin(math.radians(35)), 0.005)),
                            color=AMBER, line_width=2)
    supp = np.array([[0, TRACK / 2], [XF, 0], [0, -TRACK / 2], [XR + 2 * TRAIL, 0]])
    tip_poly = pl.add_mesh(poly_xy(supp, 0.006), color="#111827", line_width=3)
    tip_edge = pl.add_mesh(pv.Line((0, TRACK / 2, 0.007), (XR + 2 * TRAIL, 0, 0.007)), color=RED, line_width=5)
    tip_com = pl.add_mesh(pv.Disc(inner=0, outer=0.011, c_res=24), color="#ca8a04")
    # side schematic (x right, z up, seen from -y)
    side = dict(
        body=pl.add_mesh(pv.PolyData(np.c_[HULL_XZ[:, 0], np.full(len(HULL_XZ), 0.02), HULL_XZ[:, 1]],
                                     faces=np.r_[len(HULL_XZ), np.arange(len(HULL_XZ))]), color="#e5e7eb"),
        outline=pl.add_mesh(pv.lines_from_points(np.c_[np.r_[HULL_XZ[:, 0], HULL_XZ[0, 0]],
                                                       np.full(len(HULL_XZ) + 1, -0.01),
                                                       np.r_[HULL_XZ[:, 1], HULL_XZ[0, 1]]]),
                            color="#6b7280", line_width=1),
    )
    side_wheel = pl.add_mesh(disc_xz(R_W, y=0.01), color="#1f2937")
    side_hub = pl.add_mesh(pv.Line((0, -0.02, 0), (R_W * 0.9, -0.02, 0)), color="#e5e7eb", line_width=3)
    side_cf = pl.add_mesh(disc_xz(R_C, y=0.0), color=GREEN)
    side_cr = pl.add_mesh(disc_xz(R_C, y=0.0), color="#6b7280")
    side_ground = pl.add_mesh(pv.PolyData(np.zeros((1, 3))), color="#8b6b4a", line_width=4)
    side_spring = pl.add_mesh(pv.Line((0, 0, 0), (0, 0, 1)), color=GREEN, line_width=7)
    side_items = list(side.values()) + [side_wheel, side_hub, side_cf, side_cr, side_ground, side_spring]
    cam_s = pl.camera

    # ================================================================ status
    pl.subplot(1, 1)
    pl.set_background("#111827", all_renderers=False)
    status = pl.add_text("", position=(20, 20), font_size=11, color="#e5e7eb", font="courier")

    # ---------------------------------------------------------------------------------------------- render
    pl.subplot(0, 0)
    pl.open_movie(a.out, framerate=FPS, quality=8, macro_block_size=8)
    wl = wr = 0.0
    prev_cam, total = None, 0
    for si, seg in enumerate(S):
        if only and si + 1 not in only:
            continue
        key, frames = seg["key"], seg["frames"]
        head.SetInput(seg["title"])
        wrapped = textwrap.wrap(seg["text"], 100)
        body.SetInput(chr(10).join(wrapped))
        body.SetPosition(32, H - 108 - 19 * (len(wrapped) - 1))
        warn.SetPosition(32, H - 146 - 19 * (len(wrapped) - 1))
        count.SetInput(f"{si + 1} / {len(S)}")
        for k2, acts in seg_labels.items():
            for act in acts:
                act.visibility = k2 == key
        gfun = seg.get("ground")
        floor.visibility = grid.visibility = key != "tip"
        ramp.visibility, step.visibility = key == "ramp", key == "step"
        table.visibility = tip_line.visibility = key == "tip"
        ring3.visibility = plan_ring.visibility = bool(seg.get("ring"))
        heading_line.visibility = plan_head.visibility = key == "angle"
        com_ball.visibility = key in ("intro", "tip")
        plumb.visibility = plumb_dot.visibility = key == "tip"
        is_side = seg["plan"] in ("side", "side_flat")
        for act in side_items:
            act.visibility = is_side
        for nm, act in plan.items():
            act.visibility = not is_side and not (key == "tip" and nm in ("grid",))
        for act in (plan_l, plan_r, plan_icr, plan_icr_lines, plan_trail):
            act.visibility = seg["plan"] == "plan"
        tip_poly.visibility = tip_edge.visibility = tip_com.visibility = key == "tip"
        if is_side:
            gx = np.linspace(-1.0, 2.6, 1500)
            gz = gfun(gx) if gfun else np.zeros_like(gx)
            side_ground.mapper.dataset.copy_from(pv.lines_from_points(np.c_[gx, np.full(len(gx), -0.03), gz]))
        sch_title.SetInput({"plan": "Plan view: wheel speeds, ICR (red)", "side_flat": "Side view: pitch, front spring",
                            "side": "Side view: contacts, front spring", "tip": "Support polygon, CoM (plan)"}[seg["plan"]])
        trail_pts = []
        trail3.mapper.dataset.copy_from(pv.PolyData(np.zeros((1, 3))))
        plan_trail.mapper.dataset.copy_from(pv.PolyData(np.zeros((1, 3))))
        psi_f = psi_r = 0.0
        cf = cr = None
        start_cam = prev_cam
        for k, f in enumerate(frames):
            t = k / FPS
            x, y, yaw, za, phi, comp = f["x"], f["y"], f["yaw"], f["za"], f["phi"], f["comp"]
            T = body_tf(x, y, yaw, za, phi)
            if key == "tip":
                ang = math.radians(f["tilt"])
                Rt = about(rot_axis(u_edge, ang), edge_p1)
                T = Rt @ T
                table.user_matrix = Rt
                tip_line.user_matrix = Rt
            v, w = f["v"], f["w"]
            vl, vr = v - w * TRACK / 2, v + w * TRACK / 2
            wl += vl / R_W / FPS
            wr += vr / R_W / FPS
            # casters trail behind their swivels (tractrix); the tip test turns the rear one inward
            sf, sr = (T @ [SWIVEL_F, 0, 0, 1])[:2], (T @ [SWIVEL_R, 0, 0, 1])[:2]
            if cf is None:
                cf, cr = (T @ [XF, 0, 0, 1])[:2], (T @ [XR, 0, 0, 1])[:2]
            for s_, c_ in ((sf, "f"), (sr, "r")):
                cc = cf if c_ == "f" else cr
                d = cc - s_
                n = np.linalg.norm(d)
                cc = s_ + TRAIL * d / n if n > 1e-9 else cc
                if c_ == "f":
                    cf = cc
                else:
                    cr = cc
            Tinv = np.linalg.inv(T)
            lf, lr = (Tinv @ [*cf, 0, 1])[:2] - [SWIVEL_F, 0], (Tinv @ [*cr, 0, 1])[:2] - [SWIVEL_R, 0]
            psi_f, psi_r = math.atan2(lf[1], lf[0]) - math.pi, math.atan2(lr[1], lr[0]) - math.pi
            if key == "tip":
                psi_f, psi_r = 0.0, math.pi
            ghost = seg.get("ghost") and key != "step"
            for p in parts:
                M = T
                pn = p["pn"]
                if pn in ("RV-503", "RV-502"):
                    M = T @ about(ry(wl if p["centre"][1] > 0 else wr), (0, 0, A_Z))
                elif p["key"] == "caster_f":
                    M = T @ translate((0, 0, comp)) @ about(rz(psi_f), (SWIVEL_F, 0, 0))
                elif p["key"] == "caster_r":
                    M = T @ about(rz(psi_r), (SWIVEL_R, 0, 0))
                elif pn == "RV-309":
                    M = T @ translate((0, 0, comp))
                elif pn == "RV-554":
                    M = T @ about(np.diag([1, 1, (spring_len - comp) / spring_len, 1.0]), (0, 0, spring_top))
                p["actor"].user_matrix = M
                op = 1.0
                if ghost and (pn in SKINS or pn in ("RV-410", "RV-431", "RV-430", "RV-301", "RV-302", "RV-552")):
                    op = 0.28
                elif key == "step" and (pn in SKINS or (pn == "RV-503" and p["centre"][1] < 0)):
                    op = 0.3
                p["actor"].prop.opacity = op
            # extras in the main view
            if key == "tip":
                cw = (T @ [*COM, 1])[:3]
                n_pl = (Rt @ [0, 0, 1, 0])[:3]
                p0 = (Rt @ [*edge_p1, 1])[:3]
                tt = np.dot(n_pl, cw - p0) / n_pl[2]
                hit = cw - [0, 0, tt]
                com_ball.user_matrix = T
                plumb.mapper.dataset.copy_from(pv.Line(cw, hit))
                plumb_dot.user_matrix = translate(hit)
                hit_b = (np.linalg.inv(Rt) @ [*hit, 1])[:3]
                tip_com.user_matrix = translate((hit_b[0], hit_b[1], 0.008))
                frac = f["tilt"] / TIP_ANGLE
                col = GREEN if frac < 0.7 else (AMBER if frac < 0.985 else RED)
                plumb.prop.color = plumb_dot.prop.color = tip_com.prop.color = col
            elif key == "intro":
                com_ball.user_matrix = T
            mid = np.array([x, y, 0.003])
            if seg["plan"] == "plan" and abs(v) + abs(w) > 1e-4:
                trail_pts.append(mid.copy())
                if len(trail_pts) > 1:
                    trail3.mapper.dataset.copy_from(pv.lines_from_points(np.array(trail_pts)))
                    plan_trail.mapper.dataset.copy_from(pv.lines_from_points(np.array(trail_pts) + [0, 0, 0.004]))
            trail3.visibility = seg["plan"] == "plan" and len(trail_pts) > 1
            icr = None
            if seg.get("icr") and abs(w) > 0.02:
                R = v / w
                if abs(R) < 2.0:
                    icr = np.array([x - R * math.sin(yaw), y + R * math.cos(yaw), 0.003])
            icr_pole.visibility = icr_line.visibility = icr is not None
            plan_icr.visibility = plan_icr_lines.visibility = icr is not None
            if icr is not None:
                icr_pole.user_matrix = translate((icr[0], icr[1], 0))
                wpL = np.array([x - TRACK / 2 * math.sin(yaw), y + TRACK / 2 * math.cos(yaw), 0.004])
                wpR = np.array([x + TRACK / 2 * math.sin(yaw), y - TRACK / 2 * math.cos(yaw), 0.004])
                far = wpL if np.linalg.norm(wpL - icr) > np.linalg.norm(wpR - icr) else wpR
                icr_line.mapper.dataset.copy_from(pv.Line(icr, far))
                plan_icr.user_matrix = translate((icr[0], icr[1], 0.006))
                plan_icr_lines.mapper.dataset.copy_from(pv.Line(icr + [0, 0, 0.003], far + [0, 0, 0.003]))
            side_arrow.visibility = key == "angle" and t < 3.2
            # schematic
            Tp = T if key != "tip" else np.eye(4)
            if not is_side:
                for nm in plan_moving:
                    plan[nm].user_matrix = Tp
                for act, vi, sy in ((plan_l, vl, 1), (plan_r, vr, -1)):
                    act.visibility = seg["plan"] == "plan" and abs(vi) > 0.004
                    if act.visibility:
                        o = (Tp @ [0, sy * TRACK / 2, 0.01, 1])[:3]
                        d = np.array([math.cos(yaw), math.sin(yaw), 0]) * np.sign(vi)
                        act.mapper.dataset.copy_from(arrow(o, d, 0.08 + abs(vi) * 0.6))
                        act.prop.color = GREEN if vi > 0 else AMBER
                cam_s.up = (0, 1, 0)
                if key == "tip":
                    fp, sc = (0, 0, 0), 0.30
                else:
                    fp = (*seg.get("plan_focal", (x, y)), 0)
                    sc = seg.get("plan_scale", 0.42 if seg.get("ring") is None else 0.36)
                cam_s.focal_point, cam_s.position = fp, (fp[0], fp[1], 5)
                cam_s.parallel_scale = sc
            else:
                Ts = translate((x, 0, za)) @ ry(-phi) @ translate((0, 0, -A_Z))
                for act in side.values():
                    act.user_matrix = Ts
                side_wheel.user_matrix = Ts @ translate((0, 0, A_Z))
                side_hub.user_matrix = Ts @ translate((0, 0, A_Z)) @ ry(wl)
                side_cf.user_matrix = Ts @ translate((XF, 0, R_C + comp))
                side_cr.user_matrix = Ts @ translate((XR, 0, R_C))
                top_f = (Ts @ [SWIVEL_F, -0.03, 0.087, 1])[:3]
                bot_f = (Ts @ [SWIVEL_F, -0.03, 0.071 + comp, 1])[:3]
                side_spring.mapper.dataset.copy_from(pv.Line(bot_f, top_f))
                fr = comp / TRAVEL
                cc = GREEN if fr < 0.6 else (AMBER if fr < 0.99 else RED)
                side_cf.prop.color = side_spring.prop.color = cc
                side_wheel.prop.color = RED if f["lift"] > 2e-4 else "#1f2937"
                cam_s.up = (0, 0, 1)
                cam_s.focal_point = (x - 0.015, 0, za + 0.05)
                cam_s.position = (x - 0.015, -5, za + 0.05)
                cam_s.parallel_scale = 0.21
            cam_s.clipping_range = (0.1, 20)
            # status
            tl, il = wheel_load(vl, f["a"], f["al"], f["dx"], -1, f["lift"])
            tr, ir = wheel_load(vr, f["a"], f["al"], f["dx"], +1, f["lift"])
            R = "straight" if abs(w) < 1e-3 else ("0 (on the spot)" if abs(v) < 1e-3 else f"{abs(v / w):.2f} m")
            lines = [f"speed    {v:5.2f} m/s   turn {math.degrees(w):+6.1f} deg/s",
                     f"radius   {R}",
                     f"wheels   L {rpm(vl):+6.1f} rpm  R {rpm(vr):+6.1f} rpm",
                     f"torque   L {tl:+5.2f}  R {tr:+5.2f} N.m",
                     "         limit 0.98 cont, 2.45 peak",
                     f"current  L {il:4.2f}   R {ir:4.2f} A",
                     f"pitch    {math.degrees(phi):+5.1f} deg",
                     f"spring   {comp / mm:4.1f} of {TRAVEL / mm:.0f} mm (front caster)"]
            if key == "tip":
                ang_now = f["tilt"]
                lines = [f"tilt        {ang_now:5.1f} deg",
                         f"tips at     {TIP_ANGLE:.1f} deg (rear edges)",
                         f"margin      {max(0.0, TIP_ANGLE - ang_now):5.1f} deg",
                         "drawer out 150 mm   28.0 deg",
                         "drawer out 250 mm   24.3 deg",
                         "forward tip  0.77 g braking",
                         "required    20 deg every way"]
            elif key in ("intro", "summary"):
                lines = ["mass        8.67 kg, CoM 146 mm up",
                         "wheels      144 x 52 mm, track 365",
                         "motors      Pololu 70:1, 150 rpm",
                         "top speed   about 1.1 m/s loaded",
                         f"turning     {2 * R_SWEEP / mm:.0f} mm swept circle",
                         "front caster sprung, 10 mm travel",
                         "climbs      5 deg ramp, 15 mm edge",
                         "tip angle   33.2 deg minimum"]
            status.SetInput("\n".join(lines))
            if key == "step" and f["lift"] > 2e-4:
                warn.SetInput(f"Front spring bottomed: the robot rocks onto its casters and the drive wheels lift "
                              f"{f['lift'] / mm:.1f} mm")
            elif key == "step" and f["comp"] >= TRAVEL - 1e-5:
                warn.SetInput("Front spring at full travel")
            else:
                warn.SetInput("")
            # camera
            pos, foc = seg["cam"](f, t)
            pos, foc = np.asarray(pos, float), np.asarray(foc, float)
            if start_cam is not None:
                kk = smooth(t / 1.3)
                pos = start_cam[0] * (1 - kk) + pos * kk
                foc = start_cam[1] * (1 - kk) + foc * kk
            cam.position, cam.focal_point, cam.up = pos, foc, (0, 0, 1)
            cam.clipping_range = (0.01, 20)
            pl.write_frame()
            total += 1
            if a.stills and k == int(len(frames) * (0.55 if key != "tip" else 0.62)):
                pl.screenshot(os.path.splitext(a.out)[0] + f"_seg{si + 1:02d}.png")
        prev_cam = (np.array(cam.position), np.array(cam.focal_point))
        if key == "step":
            print(f"threshold: max spring {max(f['comp'] for f in frames) / mm:.1f} mm, max wheel lift "
                  f"{max(f['lift'] for f in frames) / mm:.2f} mm, max pitch "
                  f"{max(abs(math.degrees(f['phi'])) for f in frames):.1f} deg")
        if key == "ramp":
            print(f"ramp: max pitch {max(math.degrees(f['phi']) for f in frames):.2f} deg, max spring "
                  f"{max(f['comp'] for f in frames) / mm:.1f} mm")
    pl.close()
    print(f"{total} frames, {total / FPS:.1f} s\nwrote {a.out}")


if __name__ == "__main__":
    main()
