"""
RoamView Rev B interconnect model - single source of truth for the Rev B layout.

Frame (same as the SOLIDWORKS macro): +X forward, +Y up, +Z robot right, mm.
Origin = floor point under the drive-axle centre (ROS base_footprint).

Rev B decisions (2026-09-27): 125 mm drive wheels, low-profile 3-inch casters at X +-118,
sprung drive modules (trailing arm, one spring per side, 10 mm travel), front bumper with
two switches, carry handle on the rear posts. Layout follows old-designs/DESIGN_RULES.md.
"""
import math
from shapely.geometry import box as sbox, Point, LineString
from shapely.ops import unary_union

# ---- global geometry -------------------------------------------------------
PLATE_L, PLATE_W, CORNER_R = 320, 260, 20
POST_X, POST_Z, POST_H = 140, 110, 120
BASE_T, DECK_T = 6, 3
WHEEL_D, WHEEL_W, TRACK = 125, 30, 224
AXLE_Y = WHEEL_D / 2                        # 62.5
MOTOR_R, MOTOR_FACE = 18.5, 86
TRAVEL, DROOP = 10, 5
BASE_Y = AXLE_Y + MOTOR_R + TRAVEL + 2      # 93, lower deck underside
BASE_TOP = BASE_Y + BASE_T                  # 99
DECK_Y = BASE_TOP + POST_H                  # 219, top deck underside
DECK_TOP = DECK_Y + DECK_T                  # 222
RISER_H = 40
LIDAR_Y = DECK_TOP + RISER_H                # 262, LiDAR base
SCAN_Y = LIDAR_Y + 24                       # 286, scan plane
SCAN_BAND = (SCAN_Y - 15, SCAN_Y + 15)      # 271..301 keep-out
WELL_HALF, WELL_DEPTH, WELL_R = 75, 42, 6
WELL_Z = PLATE_W / 2 - WELL_DEPTH           # 88
CASTER_X, CASTER_WD, CASTER_H, CASTER_SPACER, CASTER_TRAIL = 118, 76.2, 89, 4, 20
CASTER_SWING = math.hypot(CASTER_TRAIL + CASTER_WD / 2, 15) + 2   # ~62 mm
PIVOT = (-66.0, 72.0)                       # trailing-arm pivot (x, y)
SPRING_X, SPRING_Z = 32.0, 72.0
M25, M3, M4, M5 = 2.7, 3.4, 4.5, 5.5
Y0, T0, TU = BASE_TOP, DECK_TOP, DECK_Y     # lower deck top, top deck top, top deck underside
# Raspberry Pi 5 (turned: SD card end forward at X -36, USB/Ethernet end at X -121): the 58 x 49 mounting
# pattern sits 3.5 mm in from the SD end and from both long edges, 23.5 mm from the USB/Ethernet end.
PI_HOLES = [(x, z) for x in (-97.5, -39.5) for z in (-92.5, -43.5)]
# ESP32 carrier = Adafruit half-size Perma-Proto (81 x 51 mm): two holes on the long centre line, 73.7 mm apart.
ESP_HOLES = [(-121.35, 61.5), (-47.65, 61.5)]
# Adafruit VL53L4CD (25.40 x 17.78 mm) has four 2.5 mm holes on a 20.32 x 12.70 mm pattern; two diagonal ones are used.
CLIFF_HOLES = [(-10.16, -6.35), (10.16, 6.35)]

# ---- primitive helpers -------------------------------------------------------
def B(x0, x1, y0, y1, z0, z1, color=None, name=None):
    return dict(t="box", min=[min(x0, x1), min(y0, y1), min(z0, z1)],
                max=[max(x0, x1), max(y0, y1), max(z0, z1)], color=color, name=name)


def LATHE(axis, origin, profile, color=None, name=None):
    return dict(t="lathe", axis=axis, origin=list(origin), profile=[list(p) for p in profile], color=color, name=name)


def CYL(axis, origin, r, a0, a1, color=None, name=None):
    return LATHE(axis, origin, [(0, a0), (r, a0), (r, a1), (0, a1)], color, name)


def P2020(origin, h, color=None):
    return dict(t="p2020", origin=list(origin), h=h, color=color)


def ZONE(x0, x1, y0, y1, z0, z1, label):
    return dict(min=[min(x0, x1), min(y0, y1), min(z0, z1)], max=[max(x0, x1), max(y0, y1), max(z0, z1)], label=label)


def side_zone(face, d, depth=25.0, pad=4.0):
    """Finger zone in front of a connector face.
    face = (x0,x1,y0,y1,z0,z1) box of the connector; d = '+x','-x','+z','-z','+y','-y'."""
    x0, x1, y0, y1, z0, z1 = face
    x0, x1, y0, y1, z0, z1 = x0 - pad, x1 + pad, y0 - pad, y1 + pad, z0 - pad, z1 + pad
    ax, sg = d[1], d[0]
    if ax == "x":
        return (x1 - pad, x1 - pad + depth, y0, y1, z0, z1) if sg == "+" else (x0 + pad - depth, x0 + pad, y0, y1, z0, z1)
    if ax == "z":
        return (x0, x1, y0, y1, z1 - pad, z1 - pad + depth) if sg == "+" else (x0, x1, y0, y1, z0 + pad - depth, z0 + pad)
    return (x0, x1, y1 - pad, y1 - pad + depth, z0, z1) if sg == "+" else (x0, x1, y0 + pad - depth, y0 + pad, z0, z1)


C = dict(
    wheel=[0.12, 0.12, 0.12], motor=[0.55, 0.55, 0.60], motor_can=[0.36, 0.37, 0.39], enc=[0.08, 0.08, 0.09],
    alu=[0.74, 0.76, 0.79], arm=[0.69, 0.19, 0.12], steel=[0.62, 0.64, 0.67], spring=[0.80, 0.80, 0.82],
    caster=[0.18, 0.19, 0.21], battery=[0.10, 0.35, 0.80], tray=[0.20, 0.22, 0.25], strap=[0.10, 0.10, 0.10],
    post=[0.75, 0.75, 0.78], riser=[0.95, 0.60, 0.10], lidar=[0.15, 0.15, 0.18], oak=[0.08, 0.08, 0.08],
    estop=[0.85, 0.10, 0.10], estop_body=[0.95, 0.80, 0.10], pi=[0.10, 0.55, 0.25], hat=[0.05, 0.42, 0.20],
    pcb=[0.07, 0.30, 0.20], pcb_dark=[0.10, 0.12, 0.14], pcb_blue=[0.08, 0.20, 0.45], pcb_purple=[0.30, 0.16, 0.45],
    pcb_red=[0.60, 0.10, 0.10], black=[0.06, 0.06, 0.07], brass=[0.80, 0.62, 0.28], white=[0.92, 0.92, 0.88],
    silver=[0.72, 0.73, 0.75], cap=[0.12, 0.20, 0.40], lens_red=[0.95, 0.15, 0.12], lens_green=[0.15, 0.80, 0.30],
    speaker=[0.18, 0.19, 0.21], dock=[0.26, 0.28, 0.31], tag_w=[0.96, 0.96, 0.96], standoff=[0.93, 0.93, 0.90],
    standoff_m=[0.78, 0.66, 0.36], bumper=[0.16, 0.17, 0.19], rubber=[0.10, 0.10, 0.10], yellow=[0.93, 0.72, 0.12],
    orange=[0.95, 0.45, 0.10], wago=[0.85, 0.87, 0.90], wago_lever=[0.95, 0.55, 0.10], fuse15=[0.20, 0.45, 0.85],
    fuse3=[0.85, 0.65, 0.20], fuse5=[0.80, 0.70, 0.50],
)


def standoffs(pts, y0, y1, r=2.5, color=None):
    return [CYL("y", (x, y0, z), r, 0, y1 - y0, color or C["standoff"], "Standoff") for x, z in pts]


def part(pid, name, sys, prims, desc, *, group="new", file="", mass=0.0, zones=(), join=(), service=False,
         moving=False, hidden=False, level=""):
    return dict(id=pid, name=name, sys=sys, prims=prims, desc=desc, group=group, file=file, mass=mass,
                zones=list(zones), join=list(join), service=service, moving=moving, hidden=hidden, level=level)


# ---- plates ------------------------------------------------------------------
def _rrect(hx, hz, r):
    return sbox(-hx + r, -hz + r, hx - r, hz - r).buffer(r, quad_segs=24)


def slot(x1, z1, x2, z2, w):
    if (x1, z1) == (x2, z2):
        return Point(x1, z1).buffer(w / 2, quad_segs=24)
    return LineString([(x1, z1), (x2, z2)]).buffer(w / 2, quad_segs=24)


def hole(x, z, d):
    return Point(x, z).buffer(d / 2, quad_segs=24)


def wells():
    hz = PLATE_W / 2
    w = []
    for s in (1, -1):
        w += [sbox(-WELL_HALF, min(s * (WELL_Z + WELL_R), s * 140), WELL_HALF, max(s * (WELL_Z + WELL_R), s * 140)),
              sbox(-WELL_HALF + WELL_R, min(s * WELL_Z, s * 140), WELL_HALF - WELL_R, max(s * WELL_Z, s * 140)),
              Point(WELL_HALF - WELL_R, s * (WELL_Z + WELL_R)).buffer(WELL_R, quad_segs=24),
              Point(-WELL_HALF + WELL_R, s * (WELL_Z + WELL_R)).buffer(WELL_R, quad_segs=24)]
    return unary_union(w)


# pass-throughs (all lined with edge grommet)
PT_LOWER = {
    "rear": slot(-74, 32, -74, 62, 16),          # motor + encoder leads, rear cliff sensors (from under the deck)
    "front": slot(90, -60, 110, -60, 16),        # front-left cliff sensor, left bumper switch (from under the deck)
    "front_r": slot(130, 51, 130, 61, 14),       # front-right cliff sensor, right bumper switch
}
PT_TOP = {
    "left": slot(-60, -112, -40, -112, 12),      # Pi power, REC light
    "right": slot(-60, 112, -40, 112, 12),       # ESP32 power, e-stop echo, status light, K2
    "rear": slot(-122, 110, -98, 110, 12),       # deck signal cable to the ESP32 carrier
    "front": slot(64, -24, 86, -24, 12),         # OAK power up to the Y-adapter
    "centre": slot(92, 50, 92, 62, 12),          # RC receiver to K2
}


def baseplate_holes():
    g = {}
    g["posts"] = [hole(sx * POST_X, sz * POST_Z, M5) for sx in (1, -1) for sz in (1, -1)]
    g["caster_bolts"] = [hole(sx * CASTER_X + dx, dz, M5) for sx in (1, -1) for dx in (-24, 24) for dz in (-24, 24)]
    g["spring_holes"] = [hole(SPRING_X, s * SPRING_Z, 18) for s in (1, -1)]
    g["spring_cup_bolts"] = [hole(SPRING_X + dx, s * SPRING_Z, M25) for s in (1, -1) for dx in (-14.5, 14.5)]
    g["hanger_bolts"] = [hole(PIVOT[0] + dx, s * 82, M4) for s in (1, -1) for dx in (-6, 6)]
    g["battery_tray"] = [hole(xv, sv * 34, M3) for xv in (-28, 48) for sv in (1, -1)]
    g["battery_straps"] = [slot(xc - 7, sv * 43, xc + 7, sv * 43, 4) for xc in (-12.5, 32.5) for sv in (1, -1)]
    g["pdb"] = [hole(-132.5, -81.35, M3), hole(-132.5, -7.65, M3)]
    g["driver"] = [hole(x, z, M3) for x in (-153, -113) for z in (39, 79)]
    g["wago"] = [hole(-63.5, z, M3) for z in (-41, 17)]
    g["contacts"] = [hole(150, sz * zv, M4) for sz in (1, -1) for zv in (50, 64)]
    g["bumper"] = [hole(152, sz * 110, M4) for sz in (1, -1)]   # plus the corner post's own M5 bolt
    g["cliff"] = [hole(sx * 131 + dx, sz * 87 + dz, M25) for sx in (1, -1) for sz in (1, -1) for dx, dz in CLIFF_HOLES]
    g["f2"] = [hole(94, 65.5, M3)]
    g["pass_through"] = list(PT_LOWER.values())
    return g


def topdeck_holes():
    g = {}
    g["posts"] = [hole(sx * POST_X, sz * POST_Z, M5) for sx in (1, -1) for sz in (1, -1)]
    g["lidar_bolts"] = [hole(xv, zv, M3) for xv in (20, -20) for zv in (20, -20)]
    g["lidar_cable"] = [hole(0, 0, 20)]
    g["pi5"] = [hole(x, z, M25) for x, z in PI_HOLES]           # 58 x 49, 3.5 mm in from the SD end
    g["esp32"] = [hole(x, z, M3) for x, z in ESP_HOLES]         # half-size Perma-Proto: 2 holes, 73.7 mm
    g["estop"] = [hole(-105, 0, 22.5)]
    g["camera"] = [hole(xv, zv, M4) for xv in (120, 140) for zv in (30, -30)]
    g["indicators"] = [hole(130, 80, 16), hole(130, -80, 16)]
    g["handle"] = [hole(-151, sz * 116, M5) for sz in (1, -1)]
    g["power_blocks"] = [hole(xv, sz * zv, M3) for sz in (1, -1) for xv in (-26, 26) for zv in (84, 118)]
    g["k2"] = [hole(xv, zv, M25) for xv in (66, 101) for zv in (34, 51)]
    g["j2"] = [hole(-47, 0, M3)]
    g["mic"] = [hole(70 + 30 * math.cos(a), -92 + 30 * math.sin(a), M3) for a in (0.785, 2.356, 3.927, 5.498)]
    g["speaker"] = [hole(78 + 17 * math.cos(a), 100 + 17 * math.sin(a), M3) for a in (0, 2.094, 4.189)]
    g["pass_through"] = list(PT_TOP.values())
    return g


def baseplate_outline():
    return _rrect(PLATE_L / 2, PLATE_W / 2, CORNER_R).difference(wells())


def plates():
    bh = baseplate_holes()
    th = topdeck_holes()
    base = baseplate_outline().difference(unary_union([h for v in bh.values() for h in v]))
    top = _rrect(PLATE_L / 2, PLATE_W / 2, CORNER_R).difference(unary_union([h for v in th.values() for h in v]))
    return [
        dict(id="baseplate", name="Lower deck (baseplate)", y0=BASE_Y, y1=BASE_TOP, poly=base, holes=bh,
             color=[0.78, 0.80, 0.85], partno="RV-CH-001B",
             desc="6061-T6 plate, 320 x 260 x 6 mm. Rev B: wells lengthened to 150 mm for the 125 mm wheels, spring holes, "
                  "3-inch caster pattern, new pass-throughs and mounting holes"),
        dict(id="topdeck", name="Top deck", y0=DECK_Y, y1=DECK_TOP, poly=top, holes=th,
             color=[0.78, 0.80, 0.85], partno="RV-CH-002B",
             desc="6061-T6 plate, 320 x 260 x 3 mm. Rev B: new pass-throughs, hanging power blocks, handle and e-stop holes"),
    ]


# ---- drivetrain ---------------------------------------------------------------
WHEEL_PROFILE = [(0, -15), (15, -15), (15, -10), (47, -10), (47, -15), (62.5, -15), (62.5, 15), (47, 15), (47, 10),
                 (15, 10), (15, 15), (0, 15)]


def rot_about_pivot(p, deg):
    """Rotate point p=(x,y,z) about the pivot axis (parallel to Z) by deg (positive lifts the wheel)."""
    a = math.radians(deg)
    dx, dy = p[0] - PIVOT[0], p[1] - PIVOT[1]
    return (PIVOT[0] + dx * math.cos(a) - dy * math.sin(a), PIVOT[1] + dx * math.sin(a) + dy * math.cos(a), p[2])


BUMP_DEG = math.degrees(math.asin(TRAVEL / (0 - PIVOT[0])))      # wheel up 10 mm
DROOP_DEG = -math.degrees(math.asin(DROOP / (0 - PIVOT[0])))     # wheel down 5 mm


def drive_module(s):
    """Gearmotor + module plate + spring tab for side s (-1 left, +1 right), rest pose."""
    zf = s * MOTOR_FACE
    # gearmotor axis along Z; axial coordinate a measured along +Z from the motor face
    inw = -s
    prof = [(0, 18 * s), (3, 18 * s), (3, 0), (18.5, 0), (18.5, 38 * inw), (17.5, 38 * inw), (17.5, 70 * inw),
            (16, 70 * inw), (16, 80 * inw), (0, 80 * inw)]
    zp0, zp1 = sorted((zf, zf + 5 * s))
    prims = [
        LATHE("z", (0, AXLE_Y, zf), prof, C["motor"], "Gearmotor 37D with 64 CPR encoder"),
        B(-24, 0, 42, 82, zp0, zp1, C["arm"], "Module plate"),
        B(0, 24, 42, 78, zp0, zp1, C["arm"], "Module plate"),
        B(PIVOT[0] - 8, -24, PIVOT[1] - 8, PIVOT[1] + 8, zp0, zp1, C["arm"], "Trailing arm"),
        CYL("z", (PIVOT[0], PIVOT[1], 0), 9, zp0, zp1, C["arm"], "Pivot boss"),
        B(24, 40, 63, 67, *sorted((s * 60, zf + 5 * s)), C["arm"], "Spring tab"),
    ]
    return prims


def spring_parts(s):
    z = s * SPRING_Z
    zig = []
    top, bot, n = 110.0, 67.0, 9
    for i in range(n + 1):
        a = bot + (top - bot) * i / n
        zig.append((7.2 if i % 2 else 5.4, a - bot))
    prof = [(0, 0)] + zig + [(0, top - bot)]
    cup = [(9.5, 0), (17.5, 0), (17.5, 2), (13, 2), (13, 14), (0, 14), (0, 11), (9.5, 11)]
    return [
        LATHE("y", (SPRING_X, bot, z), prof, C["spring"], "Spring"),
        CYL("y", (SPRING_X, 63, z), 2.5, 0, 55, C["steel"], "Guide bolt (droop stop)"),
        LATHE("y", (SPRING_X, BASE_TOP, z), cup, C["alu"], "Spring cup"),
        CYL("y", (SPRING_X, BASE_TOP + 14, z), 4.5, 0, 4, C["steel"], "Stop nut"),
    ]


def caster(sx):
    xc = sx * CASTER_X
    xw = xc - sx * CASTER_TRAIL      # wheel trails toward the robot centre (parked pose)
    r = CASTER_WD / 2
    return [
        B(xc - 30, xc + 30, BASE_Y - CASTER_SPACER, BASE_Y, -30, 30, C["tray"], "Spacer 4 mm"),
        B(xc - 30, xc + 30, BASE_Y - CASTER_SPACER - 3, BASE_Y - CASTER_SPACER, -30, 30, C["steel"], "Top plate"),
        CYL("y", (xc, 80, 0), 22, 0, 6, C["steel"], "Swivel race"),
        B(min(xc - 16, xw - 9), max(xc + 16, xw + 9), 76.5, 80, -21, 21, C["steel"], "Fork yoke"),
        B(xw - 9, xw + 9, 30, 78, -20.5, -17.5, C["steel"], "Fork leg"),
        B(xw - 9, xw + 9, 30, 78, 17.5, 20.5, C["steel"], "Fork leg"),
        LATHE("z", (xw, r, 0), [(0, -15), (r, -15), (r, 15), (0, 15)], C["caster"], "Wheel 3 in"),
        CYL("z", (xw, r, 0), 9, -17.5, 17.5, C["steel"], "Axle"),
    ]


ESTOP_PROFILE = [(0, -45), (15, -45), (15, -DECK_T), (11, -DECK_T), (11, 0), (15, 0), (15, 18), (20, 18),
                 (20, 30), (0, 30)]

PARTS = []


def add(p):
    PARTS.append(p)
    return p


# frame
for n, x, z in (("fl", POST_X, -POST_Z), ("fr", POST_X, POST_Z), ("rl", -POST_X, -POST_Z), ("rr", -POST_X, POST_Z)):
    add(part(f"post_{n}", f"Deck post {n.upper()}", "frame", [P2020((x, BASE_TOP, z), POST_H, C["post"])],
             "2020 aluminium extrusion, 120 mm", group="existing", file="Components/RoamView_Post2020.SLDPRT",
             level="between"))

for s, side in ((-1, "l"), (1, "r")):
    nm = "left" if s < 0 else "right"
    add(part(f"wheel_{side}", f"Drive wheel, {nm}", "drive",
             [LATHE("z", (0, AXLE_Y, s * TRACK / 2), WHEEL_PROFILE, C["wheel"], "Wheel 125 x 30")],
             "125 x 30 mm drive wheel on a 6 mm D-shaft hub (placeholder: pick a wheel with a hub for the 37D shaft). "
             "Rev B: was 100 x 26 mm", group="existing", file="Components/RoamView_DriveWheel.SLDPRT",
             join=[f"motor_{side}"], moving=True, level="under"))
    add(part(f"motor_{side}", f"Drive module, {nm}", "drive", drive_module(s),
             "37 mm gearmotor with 64 CPR encoder (placeholder, recheck torque: 125 mm wheels need about 25 % more) "
             "bolted to a 5 mm aluminium module plate. The plate is the trailing arm: it pivots 66 mm behind the axle "
             "and its front tab carries the spring. 10 mm bump travel, 5 mm droop", group="new",
             file="Components/RoamView_DriveModule.SLDPRT", join=[f"wheel_{side}", f"hanger_{side}", f"spring_{side}"],
             moving=True, level="under", mass=0.36))
    zh0, zh1 = sorted((s * 78, s * 86))
    add(part(f"hanger_{side}", f"Pivot hanger, {nm}", "drive",
             [B(PIVOT[0] - 10, PIVOT[0] + 10, PIVOT[1] - 8, BASE_Y, zh0, zh1, C["alu"], "Hanger"),
              CYL("z", (PIVOT[0], PIVOT[1], 0), 4, *sorted((s * 76, s * 93)), C["steel"], "Pivot pin 8 mm")],
             "Aluminium hanger bolted under the lower deck (2 x M4); 8 mm shoulder-bolt pivot with bronze bushings",
             file="Components/RoamView_PivotHanger.SLDPRT", join=[f"motor_{side}"], level="under", mass=0.03))
    add(part(f"spring_{side}", f"Suspension spring, {nm}", "drive", spring_parts(s),
             "Compression spring (about 43 mm installed, 16 mm OD) on an M5 guide bolt. It pushes the module tab down "
             "through an 18 mm hole in the lower deck and seats in a cup on the deck top; the nut on the bolt is the droop "
             "stop. Rate and preload to set after weighing the robot", file="Components/RoamView_SpringCup.SLDPRT",
             join=[f"motor_{side}"], level="under", mass=0.02))

for sx, n in ((1, "f"), (-1, "r")):
    add(part(f"caster_{n}", "Caster, front" if sx > 0 else "Caster, rear", "drive", caster(sx),
             "Low-profile 3-inch swivel plate caster, about 89 mm tall (3-1/2 in class), on a 4 mm spacer: mount face "
             "93 mm. Placeholder until a caster is chosen. Rev B: was a 2-inch caster at X +-130",
             group="existing", file="Components/RoamView_Caster3in.SLDPRT", level="under", mass=0.20))

# battery (moved +10 mm forward, in a tray; slides out forward)
bx0, bx1 = -35, 55
add(part("battery_tray", "Battery tray", "power",
         [B(bx0 - 3, bx1 + 3, Y0, Y0 + 2, -40.5, 40.5, C["tray"], "Tray"),
          B(bx0 - 3, bx0, Y0 + 2, Y0 + 12, -40.5, 40.5, C["tray"], "Rear stop"),
          B(bx1, bx1 + 3, Y0 + 2, Y0 + 8, -40.5, 40.5, C["tray"], "Front lip")],
         "3 mm printed or aluminium tray on four M3 screws: rear stop and a 6 mm front lip hold the pack against 3 g",
         join=["battery"], level="lower", mass=0.05))
add(part("battery", "Battery, 4S2P 18650", "power",
         [B(bx0, bx1, Y0 + 2, Y0 + 44, -37.5, 37.5, C["battery"], "Pack"),
          B(-21.5, -3.5, Y0 + 44, Y0 + 45.5, -38.5, 38.5, C["strap"], "Strap"),
          B(23.5, 41.5, Y0 + 44, Y0 + 45.5, -38.5, 38.5, C["strap"], "Strap"),
          B(-19.5, -5.5, Y0 + 2, Y0 + 44, -39.5, -37.5, C["strap"], "Strap"),
          B(-19.5, -5.5, Y0 + 2, Y0 + 44, 37.5, 39.5, C["strap"], "Strap"),
          B(25.5, 39.5, Y0 + 2, Y0 + 44, -39.5, -37.5, C["strap"], "Strap"),
          B(25.5, 39.5, Y0 + 2, Y0 + 44, 37.5, 39.5, C["strap"], "Strap")],
         "14.8 V nominal, about 7 Ah, pre-built pack with protection board and thermistor. 90 x 75 x 42 mm, in a tray "
         "with a rear stop and a 6 mm front lip, held by two straps. To remove: unplug the XT60, undo the straps, lift "
         "over the lip and slide it out the front between the decks", group="existing",
         file="Components/RoamView_Battery4S2P.SLDPRT", join=["battery_tray", "xt60"], service=True, level="lower",
         mass=0.70))

# ---- lower deck: power ----------------------------------------------------------
PDB_X = (-158, -107)
PDB_Z = (-85, -4)
pdb_prims = standoffs([(-132.5, -81.35), (-132.5, -7.65)], Y0, Y0 + 6) + [
    B(*PDB_X, Y0 + 6, Y0 + 7.6, *PDB_Z, C["pcb"], "Perma-Proto half-size board"),
    B(-156, -130, Y0 + 7.6, Y0 + 33.1, -83, -61, C["black"], "K1 relay CB1aF-P-12V"),
    B(-127, -119, Y0 + 7.6, Y0 + 13, -83, -65, C["black"], "R1, D3, D4 (K1 coil)"),
    B(-141, -118.1, Y0 + 7.6, Y0 + 10.3, -57, -34.2, C["pcb_purple"], "INA260 #1 (motor)"),
    B(-139, -129, Y0 + 10.3, Y0 + 20.3, -57, -49, C["black"], "INA260 terminal block"),
    B(-156, -148, Y0 + 7.6, Y0 + 21.6, -29, -9, C["black"], "F1 holder (30 A PCB blade)"),
    B(-154, -150, Y0 + 21.6, Y0 + 28.6, -25, -13, C["fuse15"], "F1 15 A fuse"),
    B(-144, -136, Y0 + 7.6, Y0 + 21.6, -29, -9, C["black"], "F3 holder"),
    B(-142, -138, Y0 + 21.6, Y0 + 28.6, -25, -13, C["fuse3"], "F3 3 A fuse"),
    B(-132, -124, Y0 + 7.6, Y0 + 21.6, -29, -9, C["black"], "F4 holder"),
    B(-130, -126, Y0 + 21.6, Y0 + 28.6, -25, -13, C["fuse3"], "F4 3 A fuse"),
    B(-117, -107, Y0 + 7.6, Y0 + 17.6, -83, -7, C["black"], "Terminal blocks TB1-TB4 (wire entry +X)"),
]
add(part("pdb", "Power board", "power", pdb_prims,
         "Perma-Proto half-size board (81 x 51 mm) on nylon standoffs: main fuse F1 (15 A) and branch fuses F3/F4 (3 A) "
         "in PCB blade holders, K1 motor relay with its 15 ohm coil resistor and flyback parts, and INA260 #1 on the motor "
         "rail. Every path over 3 A is wire soldered straight to the relay "
         "and fuse-holder pins, not board traces. Wires enter the terminal blocks on the aisle side",
         zones=[ZONE(-107, -82, Y0 + 7, Y0 + 22, -83, -7, "TB1-TB4 wire entry"),
                ZONE(-156, -124, Y0 + 28.6, Y0 + 53.6, -29, -9, "Fuse pull")],
         service=True, level="lower", mass=0.07))

add(part("wago", "Bus and ground splices", "power",
         [B(-74, -53, Y0, Y0 + 3, -45, 21, C["tray"], "Carrier"),
          B(-74, -53, Y0 + 3, Y0 + 11.5, -45, -14, C["wago"], "WAGO 221-415 GND star"),
          B(-74, -53, Y0 + 3, Y0 + 11.5, -10, 21, C["wago"], "WAGO 221-415 pack node +"),
          B(-60, -53, Y0 + 11.5, Y0 + 14, -45, 21, C["wago_lever"], "Levers")],
         "Two WAGO 221-415 lever splices in a screw-down carrier. Pack node +: pack, dock charge input and the feed to F1. "
         "Star ground: pack return, power-board ground, dock return and the frame-bond lug (the only frame connection)",
         zones=[ZONE(-99, -74, Y0 + 3, Y0 + 12, -45, 21, "Wire entry")],
         service=True, level="lower", mass=0.04))

add(part("xt60", "Pack connector XT60", "power",
         [B(-32, -24, Y0 + 45.5, Y0 + 54.5, -17, 17, C["yellow"], "XT60 (mated)")],
         "Pack disconnect, riding on the pack's rear end on its own lead. Unplug it first; the pack half leaves with "
         "the pack", join=["battery"], zones=[ZONE(-34, -22, Y0 + 54.5, Y0 + 79.5, -19, 19, "Grip")], service=True,
         level="lower", mass=0.01))

add(part("driver", "Motor driver", "drive",
         standoffs([(-153, 39), (-113, 39), (-153, 79), (-113, 79)], Y0, Y0 + 6) + [
             B(-158, -108, Y0 + 6, Y0 + 7.6, 34, 84, C["pcb_dark"], "DFR0601 board 50 x 50"),
             B(-146, -120, Y0 + 7.6, Y0 + 18.5, 42, 76, C["black"], "MOSFETs and heatsink"),
             B(-117, -108, Y0 + 7.6, Y0 + 18.5, 36, 82, C["black"], "Power and motor terminals (wire entry +X)"),
             B(-156, -149, Y0 + 7.6, Y0 + 16, 42, 62, C["white"], "Control header")],
         "DFRobot DFR0601 dual-channel driver, 12 A per channel, 6.5-37 V, 50 x 50 x 12.5 mm. Control: PWM, INA, INB per "
         "channel from the ESP32",
         zones=[ZONE(-108, -83, Y0 + 7, Y0 + 20, 36, 82, "Terminals"), ZONE(-156, -149, Y0 + 16, Y0 + 41, 42, 62, "Control plug")],
         service=True, level="lower", mass=0.06))

add(part("chg", "Charge input guard", "power",
         [B(66, 122, Y0, Y0 + 13, 55, 76, C["black"], "F2 inline holder, 5 A"),
          B(86, 98.7, Y0 + 13, Y0 + 15, 58, 65.6, C["pcb_blue"], "D1 ideal diode (Pololu 5382)")],
         "D1 (Pololu 5382 ideal diode, 10 A) and F2 (5 A ATO in an inline holder) on the + contact lead: the pack can "
         "never back-feed the exposed contacts", zones=[ZONE(66, 122, Y0 + 15, Y0 + 40, 55, 76, "Holder lid")],
         level="lower", mass=0.03))

for sz, pol in ((1, "+"), (-1, "-")):
    z0, z1 = sorted((sz * 45, sz * 69))
    add(part(f"contact_{'p' if sz > 0 else 'n'}", f"Charge contact {pol}", "power",
             [B(138, 163, Y0, Y0 + 3, z0, z1, C["brass"], f"Contact {pol} (top leg)"),
              B(160, 163, 62, Y0, z0, z1, C["brass"], f"Contact {pol} (face)")],
             f"Brass L-contact on two M4 holes with an insulating washer. Face 62-99 mm above the floor, 3 mm behind the "
             f"bumper face; the dock's spring pins reach it through the bumper window", join=["bumper"], level="lower",
             mass=0.02))

# bumper (ahead of the contacts, notch for the front caster swing)
bump = []
for sz in (1, -1):
    zw0, zw1 = sorted((sz * 48, sz * 66))
    if sz > 0:
        bump += [B(166, 171, 56, 99, 35, 48, C["bumper"], "Bumper face"), B(166, 171, 56, 99, 66, 125, C["bumper"], "Bumper face"),
                 B(166, 171, 56, 66, 48, 66, C["bumper"], "Bumper face"), B(166, 171, 90, 99, 48, 66, C["bumper"], "Bumper face")]
    else:
        bump += [B(166, 171, 56, 99, -48, -35, C["bumper"], "Bumper face"), B(166, 171, 56, 99, -125, -66, C["bumper"], "Bumper face"),
                 B(166, 171, 56, 66, -66, -48, C["bumper"], "Bumper face"), B(166, 171, 90, 99, -66, -48, C["bumper"], "Bumper face")]
    zb0, zb1 = sorted((sz * 106, sz * 118))
    zs0, zs1 = sorted((sz * 88, sz * 108))
    bump += [B(136, 166, 87, 93, zb0, zb1, C["steel"], "Flexure bracket"),
             B(159.5, 166, 76, 86.2, zs0, zs1, C["black"], "Switch SS-5GL2")]
bump += [B(166, 171, 82, 99, -35, 35, C["bumper"], "Bumper face")]
add(part("bumper", "Front bumper", "safety", bump,
         "5 mm PETG bumper face 250 mm wide, 56-100 mm above the floor, on two flexure brackets under the lower deck, each "
         "clamped by its corner post's M5 bolt plus one M4 screw. "
         "Two Omron SS-5GL2 switches close when it moves 2 mm. Windows let the dock's pins reach the charge contacts; "
         "the centre notch clears the front caster's swing", join=["contact_p", "contact_n"], level="under", mass=0.08))

for n, x, z in (("fl", 131, -87), ("fr", 131, 87), ("rl", -131, -87), ("rr", -131, 87)):
    xin = -1 if x > 0 else 1
    add(part(f"cliff_{n}", f"Cliff sensor {n.upper()}", "sensing",
             standoffs([(x + dx, z + dz) for dx, dz in CLIFF_HOLES], BASE_Y - 3, BASE_Y, 1.5) + [
                 B(x - 12.7, x + 12.7, BASE_Y - 4.6, BASE_Y - 3, z - 8.89, z + 8.89, C["pcb"], "VL53L4CD board"),
                 B(x - 2.5, x + 2.5, BASE_Y - 6, BASE_Y - 4.6, z - 1.5, z + 1.5, C["black"], "ToF sensor")],
             "Adafruit VL53L4CD time-of-flight sensor (25.5 x 17.7 mm) hanging under the lower deck, looking straight down "
             "at the floor 87 mm below. I2C at 3.3 V" + (" with no XSHUT line: its board pull-up keeps it on and the "
             "firmware gives it a new address first, then releases the other three one at a time" if n == "rr"
             else " with its own XSHUT line"),
             zones=[ZONE(x + xin * 12.75, x + xin * 37.75, BASE_Y - 8, BASE_Y - 1, z - 6, z + 6, "Cable")],
             level="under", mass=0.002))

# ---- lower-deck side of the deck interface (between the decks) ----------------
add(part("plug_pwr", "Deck plug, power (6-pin)", "power",
         [B(-141, -131, 165, 191, -99, -89, C["black"], "Micro-Fit 3.0, 6 pos, mated")],
         "Inline Molex Micro-Fit 3.0 (2 x 3), tied to the rear-left post: compute feed, sensor feed and e-stop loop. "
         "Unplug it to lift the top deck", join=["post_rl"],
         zones=[ZONE(-143, -129, 140, 165, -101, -87, "Unplug (down)"), ZONE(-143, -129, 191, 216, -101, -87, "Unplug (up)")],
         service=True, level="between", mass=0.01))
add(part("plug_sig", "Deck plug, signals (24-pin)", "compute",
         [B(-141, -131, 155, 194, 89, 99, C["black"], "Micro-Fit 3.0, 24 pos, mated")],
         "Inline Molex Micro-Fit 3.0 (2 x 12), tied to the rear-right post: encoders, driver control, I2C, 3.3 V and 5 V, "
         "cliff-sensor XSHUT lines, bumper switches. Unplug it to lift the top deck. Rev B: 24 pins (was 20)",
         join=["post_rr"],
         zones=[ZONE(-143, -129, 130, 155, 87, 101, "Unplug (down)"), ZONE(-143, -129, 194, 219, 87, 101, "Unplug (up)")],
         service=True, level="between", mass=0.02))

# ---- top deck -------------------------------------------------------------------
add(part("riser", "LiDAR riser", "sensing", [B(-25, 25, T0, T0 + RISER_H, -25, 25, C["riser"])],
         "3D-printed riser 50 x 50 x 40 mm, PETG 40 % infill, through-bolted, 12 mm cable bore on its axis. "
         "Rev B: 40 mm tall (was 60) so the scan plane stays near 286 mm", group="existing",
         file="Components/RoamView_LidarRiser.SLDPRT", join=["lidar"], level="top", mass=0.04))
add(part("lidar", "D500 LiDAR", "sensing",
         [B(-27, 27, LIDAR_Y, LIDAR_Y + 20, -23, 23, C["lidar"], "Base"),
          LATHE("y", (0, LIDAR_Y, 0), [(0, 20), (19, 20), (19, 35), (0, 35)], C["lidar"], "Head")],
         "D500 DTOF LiDAR: 360 deg, 0.03-12 m, 10 Hz, UART 230400. Scan plane about 286 mm above the floor",
         group="existing", file="Components/RoamView_LidarD500.SLDPRT", join=["riser"], level="top", mass=0.05))
add(part("oak", "OAK-D Pro W", "sensing",
         [B(115, 155, T0, T0 + 3, -40, 40, C["oak"], "Bracket base"), B(124, 127, T0, T0 + 40, -30, 30, C["oak"], "Bracket upright"),
          B(127, 149, T0 + 10, T0 + 40, -48.5, 48.5, C["oak"], "Camera")],
         "Stereo depth + 12 MP colour, IMU, IR projector. USB-C on its back face", group="existing",
         file="Components/RoamView_OakDProW.SLDPRT",
         zones=[ZONE(102, 127, T0 + 18, T0 + 32, -47, -33, "USB-C")], level="top", mass=0.12))
add(part("estop", "E-stop", "safety",
         [LATHE("y", (-105, T0, 0), ESTOP_PROFILE, C["estop"], "Mushroom and body"),
          B(-121, -89, TU - 44, TU - 26, -14, 14, C["estop_body"], "Contact blocks NC1 + NC2")],
         "22 mm latching mushroom e-stop with two NC blocks: NC1 opens K1's coil, NC2 reports to the ESP32. "
         "Rev B: moved to X -105 so the handle and the Pi's ports stay clear of it",
         group="existing", file="Components/RoamView_EStop.SLDPRT",
         zones=[ZONE(-127, -83, T0 + 30, T0 + 55, -22, 22, "Palm")], level="top", mass=0.08))

PI_X, PI_Z = (-121.0, -36.0), (-96.0, -40.0)
pxc, pzc = (PI_X[0] + PI_X[1]) / 2, (PI_Z[0] + PI_Z[1]) / 2
PI_Y = T0 + 6
pi_holes = PI_HOLES
add(part("pi5", "Raspberry Pi 5", "compute",
         standoffs(pi_holes, T0, PI_Y, 2.5) + [
             B(*PI_X, PI_Y, PI_Y + 1.6, *PI_Z, C["pi"], "Board"),
             B(pxc - 15, pxc + 25, PI_Y + 1.6, PI_Y + 11.6, pzc - 20, pzc + 20, C["silver"], "Active cooler"),
             B(-121, -103.5, PI_Y + 1.6, PI_Y + 17.6, -94.5, -41.5, C["silver"], "USB and Ethernet"),
             B(-50.5, -44, PI_Y + 1.6, PI_Y + 4.8, -96, -89, C["silver"], "USB-C power"),
             B(-65, -58.6, PI_Y + 1.6, PI_Y + 4.6, -96, -89, C["silver"], "micro-HDMI 0"),
             B(-78.4, -72, PI_Y + 1.6, PI_Y + 4.6, -96, -89, C["silver"], "micro-HDMI 1")],
         "Raspberry Pi 5 on 6 mm nylon standoffs, turned so the USB/Ethernet ports face the rear edge, the USB-C power "
         "and micro-HDMI ports face the left edge, and the SD card faces forward",
         group="existing", file="Components/RoamView_Pi5.SLDPRT", join=["pi_hat"],
         zones=[ZONE(-146, -121, PI_Y + 1, PI_Y + 18, -95, -41, "USB, Ethernet"),
                ZONE(-83, -40, PI_Y, PI_Y + 7, -121, -96, "USB-C, micro-HDMI"),
                ZONE(-36, -11, PI_Y - 4, PI_Y + 2, -80, -56, "SD card")],
         service=True, level="top", mass=0.05))
HAT_Y = PI_Y + 1.6 + 20
add(part("pi_hat", "Pi GPIO HAT", "compute",
         standoffs(pi_holes, PI_Y + 1.6, HAT_Y, 2.5) + [
             B(-101, -36, HAT_Y, HAT_Y + 2, -96.25, -39.75, C["hat"], "Perma-Proto HAT (35 mm fan cut-out)"),
             B(-93.9, -43.1, PI_Y + 1.6, HAT_Y, -46, -41, C["black"], "Stacking header"),
             B(-44, -36, HAT_Y + 2, HAT_Y + 9, -90, -50, C["white"], "JST-XH headers (vertical)")],
         "Adafruit Perma-Proto HAT on an extra-tall stacking header, with a 35 mm hole over the cooler fan. Locking "
         "JST-XH headers for UART0 (ESP32 link), UART2 (LiDAR) and the recording light",
         join=["pi5"], zones=[ZONE(-46, -34, HAT_Y + 9, HAT_Y + 34, -92, -48, "Plugs")], service=True, level="top",
         mass=0.02))

ESP_X, ESP_Z = (-125.0, -44.0), (36.0, 87.0)
ESP_Y = T0 + 5
add(part("esp32", "ESP32-S3 controller", "compute",
         standoffs(ESP_HOLES, T0, ESP_Y, 2.5) + [
             B(*ESP_X, ESP_Y, ESP_Y + 1.6, *ESP_Z, C["pcb"], "Perma-Proto half-size carrier"),
             B(-126, -63, ESP_Y + 1.6, ESP_Y + 10.1, 48.3, 51.3, C["black"], "Socket row"),
             B(-126, -63, ESP_Y + 1.6, ESP_Y + 10.1, 71.7, 74.7, C["black"], "Socket row"),
             B(-126, -63, ESP_Y + 10.1, ESP_Y + 11.7, 48.8, 74.2, C["pcb_dark"], "DevKitC-1"),
             B(-81, -63, ESP_Y + 11.7, ESP_Y + 14.9, 48.8, 74.2, C["silver"], "WROOM-2 module"),
             B(-127, -119, ESP_Y + 11.7, ESP_Y + 15, 51, 72, C["silver"], "USB-C x 2"),
             B(-60, -52, ESP_Y + 1.6, ESP_Y + 6.6, 48, 75, C["black"], "74LVC244"),
             B(-49, -45, ESP_Y + 1.6, ESP_Y + 7.6, 38, 85, C["white"], "JST-XH headers (vertical)"),
             B(-120, -80, ESP_Y + 1.6, ESP_Y + 10.6, 77, 86, C["black"], "2 x 12 box header (deck signals)")],
         "ESP32-S3-DevKitC-1-N32R16V in sockets on a Perma-Proto carrier at the top deck's rear-right. Its two USB-C "
         "ports face the rear edge so a laptop plugs in with the robot assembled. The carrier holds the 74LVC244 that "
         "shifts the 5 V encoder signals and K2's 5 V status output, the I2C pull-ups and locking headers",
         zones=[ZONE(-152, -127, ESP_Y + 9, ESP_Y + 18, 49, 74, "USB-C"),
                ZONE(-51, -43, ESP_Y + 7.6, ESP_Y + 32.6, 36, 87, "Plugs"),
                ZONE(-122, -78, ESP_Y + 10.6, ESP_Y + 35.6, 75, 88, "Deck signal plug")],
         service=True, level="top", mass=0.05))

add(part("rx1", "RC receiver (wireless e-stop)", "safety",
         [B(35, 82, T0, T0 + 15, 40, 66, C["black"], "Receiver 47 x 26 x 15"),
          CYL("y", (40, T0 + 15, 44), 0.8, 0, 22, C["black"], "Antenna"), CYL("y", (40, T0 + 15, 62), 0.8, 0, 22, C["black"], "Antenna")],
         "2.4 GHz receiver with failsafe (47 x 26 x 15 mm, placeholder). Antennas vertical, tips below the LiDAR band",
         zones=[ZONE(82, 107, T0, T0 + 15, 40, 66, "Servo leads")], level="top", mass=0.02))

add(part("yadapter", "OAK power Y-adapter", "sensing",
         [B(55, 95, T0, T0 + 10, -8, 8, C["black"], "Y-adapter")],
         "Luxonis USB-C Y-adapter: camera port faces right, host port faces the rear, power port faces left",
         zones=[ZONE(65, 85, T0 + 1, T0 + 10, 8, 33, "Camera"), ZONE(30, 55, T0 + 1, T0 + 10, -7, 7, "Host"),
                ZONE(65, 85, T0 + 1, T0 + 10, -33, -8, "Power")], level="top", mass=0.015))

add(part("mic", "Microphone array", "compute",
         standoffs([(70 + 30 * math.cos(a), -92 + 30 * math.sin(a)) for a in (0.785, 2.356, 3.927, 5.498)], T0, T0 + 4) + [
             CYL("y", (70, T0 + 4, -92), 35, 0, 1.6, C["pcb_dark"], "Mic array board 70 mm"),
             CYL("y", (70, T0 + 5.6, -92), 12, 0, 4, C["black"], "DSP and LEDs")],
         "Proposed replacement for the Anker S330, which is 120 mm across and does not fit: a 70 mm USB 4-mic array "
         "with echo cancellation (e.g. Seeed ReSpeaker USB Mic Array v2.0). Team decision",
         zones=[ZONE(10, 35, T0 + 4, T0 + 12, -100, -84, "USB")], level="top", mass=0.03))
add(part("speaker", "USB speaker", "compute",
         [CYL("y", (78, T0, 100), 22, 0, 30, C["speaker"], "Speaker 44 mm")],
         "Small USB-powered speaker for two-way audio (placeholder). Team decision together with the mic array",
         level="top", mass=0.08))

for pid, nm, z, lens in (("led_rec", "Recording light", -80, "lens_red"), ("led_status", "Status light", 80, "lens_green")):
    add(part(pid, nm, "safety",
             [CYL("y", (130, T0, z), 10, 0, 4, C["black"], "Bezel"), CYL("y", (130, T0 + 4, z), 7, 0, 3, C[lens], "Lens"),
              CYL("y", (130, TU - 22, z), 7.5, 0, 22, C["black"], "Body")],
             "16 mm panel indicator" + (", lit whenever video or audio leaves the robot" if pid == "led_rec" else
                                        " driven by the ESP32: ready, moving, fault, e-stop"),
             level="top", mass=0.01))

add(part("handle", "Carry handle", "frame",
         [B(-156, -136, T0, T0 + 3, 102, 121, C["alu"], "Foot"), B(-156, -136, T0, T0 + 3, -121, -102, C["alu"], "Foot"),
          CYL("y", (-148, T0 + 3, 113), 6, 0, 35, C["alu"], "Leg"), CYL("y", (-148, T0 + 3, -113), 6, 0, 35, C["alu"], "Leg"),
          CYL("z", (-148, T0 + 38, 0), 8, -119, 119, C["rubber"], "Grip")],
         "16 mm grip across the rear, 38 mm above the top deck, bolted through the deck into the rear posts so the load "
         "goes into the frame. Top at 268 mm, below the LiDAR band. Lift here, never by the LiDAR",
         join=["post_rl", "post_rr"], level="top", mass=0.10))

# ---- top deck underside (hanging) -------------------------------------------------
def power_block(pid, name, sz, buck, ina):
    z0, z1 = sorted((sz * 82, sz * 122))
    zi0, zi1 = sorted((sz * 86, sz * 109))
    zb0, zb1 = sorted((sz * 86, sz * 111.4))
    prims = standoffs([(xv, sz * zv) for xv in (-26, 26) for zv in (84, 118)], TU - 8, TU, 2.0) + [
        B(-30, 30, TU - 9.6, TU - 8, z0, z1, C["pcb"], "Carrier board"),
        B(-28, -5.1, TU - 12.3, TU - 9.6, zi0, zi1, C["pcb_purple"], ina),
        B(-25, -15, TU - 22.3, TU - 12.3, zi0 + 2, zi0 + 10, C["black"], "INA260 terminal block"),
        B(1.6, 27, TU - 19.1, TU - 9.6, zb0, zb1, C["pcb_blue"], buck),
        B(-28, 28, TU - 20, TU - 9.6, *sorted((sz * 113, sz * 121)), C["white"], "JST-XH headers (face down)"),
    ]
    return prims


add(part("cpwr", "Compute power block", "power",
         power_block("cpwr", "Compute power block", -1, "Buck A D36V50F5 5.1 V", "INA260 #2 (compute)") + [
             B(-24, -16.4, TU - 12, TU - 9.6, -97, -84.3, C["pcb_blue"], "D2 ideal diode"),
             CYL("y", (16, TU - 36.6, -95), 8, 0, 27, C["cap"], "C1 4700 uF 25 V")],
         "Hangs under the top deck: INA260 #2, ideal diode D2 and the 4700 uF hold-up capacitor on Buck A's 14.8 V input "
         "(about 30 ms of Pi power through a motor-inrush sag), then Buck A (Pololu D36V50F5) to 5.1 V for the Pi",
         zones=[ZONE(-28, 28, TU - 45, TU - 20, -121, -113, "Plugs")], service=True, level="under_top", mass=0.04))
add(part("spwr", "Sensor power block", "power",
         power_block("spwr", "Sensor power block", 1, "Buck B D36V50F5 5 V", "INA260 #3 (sensor)"),
         "Hangs under the top deck: INA260 #3 and Buck B (Pololu D36V50F5) to 5 V for the OAK-D, LiDAR, ESP32 side, "
         "encoders and RC relay",
         zones=[ZONE(-28, 28, TU - 45, TU - 20, 113, 121, "Plugs")], service=True, level="under_top", mass=0.03))
add(part("k2", "RC e-stop relay K2", "safety",
         standoffs([(xv, zv) for xv in (66, 101) for zv in (34, 51)], TU - 5, TU, 1.5) + [
             B(62, 105, TU - 6.6, TU - 5, 30, 55, C["pcb_blue"], "Pololu RC switch with relay"),
             B(67, 89.5, TU - 25.4, TU - 6.6, 34, 50.5, C["black"], "Relay G5LE")],
         "Pololu RC switch with relay (#2804) hanging under the top deck. Its contact sits in series with the e-stop NC1 "
         "in K1's coil loop, so losing the radio link or flipping the transmitter switch stops the motors. Its OUT pin "
         "(5 V) reports the relay state to the ESP32 through the 74LVC244",
         zones=[ZONE(105, 130, TU - 12, TU - 3, 30, 55, "Leads")], level="under_top", mass=0.01))
add(part("j2", "LiDAR splice", "sensing",
         [B(-54, -40, TU - 10, TU - 2, -6, 6, C["white"], "Splice block")],
         "Splits the LiDAR's 4-wire lead: TX to the Pi, 5 V and ground to Buck B", level="under_top", mass=0.003))

# ---- dock ------------------------------------------------------------------------
add(part("dock", "Charging dock", "dock",
         [B(176, 260, 0, 300, -140, 140, C["dock"], "Dock body"),
          B(170, 176, 70, 84, -62, -52, C["brass"], "Spring contact -"),
          B(170, 176, 70, 84, 52, 62, C["brass"], "Spring contact +"),
          B(175, 176, 170, 290, -60, 60, C["tag_w"], "AprilTag tag36h11, 100 mm")],
         "16.8 V CC/CV 3 A charger, spring pins that reach through the bumper windows, AprilTag target. Shown docked",
         hidden=True, level="dock"))

PART = {p["id"]: p for p in PARTS}


# ---- ports: where cables attach ---------------------------------------------------
# pos = where the conductor leaves the connector; dir = direction it leaves in.
def port(pid, part, pos, d, label):
    return pid, dict(part=part, pos=list(pos), dir=list(d), label=label)


PX, NX_, PZ, NZ_, PY, NY_ = (1, 0, 0), (-1, 0, 0), (0, 0, 1), (0, 0, -1), (0, 1, 0), (0, -1, 0)
ESP_J = ESP_Y + 9.0
PORTS = dict([
    # pack, splices, power board
    port("xt60.+", "xt60", (-30, Y0 + 50, -19), NZ_, "XT60 robot half +"),
    port("xt60.-", "xt60", (-26, Y0 + 50, -19), NZ_, "XT60 robot half -"),
    port("wpk.1", "wago", (-76, Y0 + 7, -6), NX_, "Pack node + / 1"),
    port("wpk.2", "wago", (-76, Y0 + 7, 1), NX_, "Pack node + / 2"),
    port("wpk.3", "wago", (-76, Y0 + 7, 8), NX_, "Pack node + / 3"),
    port("wg.1", "wago", (-76, Y0 + 7, -41), NX_, "Star ground / 1"),
    port("wg.2", "wago", (-76, Y0 + 7, -34), NX_, "Star ground / 2"),
    port("wg.3", "wago", (-76, Y0 + 7, -27), NX_, "Star ground / 3"),
    port("pdb.in+", "pdb", (-105, Y0 + 12, -80), PX, "TB1-1 IN+ (to F1)"),
    port("pdb.in-", "pdb", (-105, Y0 + 12, -75), PX, "TB1-2 GND"),
    port("pdb.mot+", "pdb", (-105, Y0 + 12, -68), PX, "TB2-1 MOT+ (after K1, INA260 #1)"),
    port("pdb.mot-", "pdb", (-105, Y0 + 12, -63), PX, "TB2-2 MOT-"),
    port("pdb.cmp+", "pdb", (-105, Y0 + 12, -56), PX, "TB3-1 CMP+ (F3)"),
    port("pdb.cmp-", "pdb", (-105, Y0 + 12, -51), PX, "TB3-2 CMP-"),
    port("pdb.sns+", "pdb", (-105, Y0 + 12, -44), PX, "TB3-3 SNS+ (F4)"),
    port("pdb.sns-", "pdb", (-105, Y0 + 12, -39), PX, "TB3-4 SNS-"),
    port("pdb.coil+", "pdb", (-105, Y0 + 12, -32), PX, "TB4-1 K1 coil feed"),
    port("pdb.coil-", "pdb", (-105, Y0 + 12, -27), PX, "TB4-2 K1 coil return"),
    port("pdb.i2c", "pdb", (-130, Y0 + 16, -8), PY, "INA260 #1 I2C (JST-XH 4)"),
    # motor driver and drive modules
    port("drv.vin+", "driver", (-106, Y0 + 14, 39), PX, "VM"),
    port("drv.vin-", "driver", (-106, Y0 + 14, 44), PX, "GND"),
    port("drv.m1a", "driver", (-106, Y0 + 14, 54), PX, "M1+"),
    port("drv.m1b", "driver", (-106, Y0 + 14, 59), PX, "M1-"),
    port("drv.m2a", "driver", (-106, Y0 + 14, 68), PX, "M2+"),
    port("drv.m2b", "driver", (-106, Y0 + 14, 73), PX, "M2-"),
    port("drv.ctrl", "driver", (-152.5, Y0 + 18, 52), PY, "PWM1 INA1 INB1 PWM2 INA2 INB2 GND"),
    port("mot_l.pwr+", "motor_l", (-17, AXLE_Y + 4, -13), NX_, "Motor red"),
    port("mot_l.pwr-", "motor_l", (-17, AXLE_Y + 4, -9), NX_, "Motor black"),
    port("mot_l.enc", "motor_l", (-17, AXLE_Y - 4, -11), NX_, "Encoder: GND, Vcc, A, B"),
    port("mot_r.pwr+", "motor_r", (-17, AXLE_Y + 4, 13), NX_, "Motor red"),
    port("mot_r.pwr-", "motor_r", (-17, AXLE_Y + 4, 9), NX_, "Motor black"),
    port("mot_r.enc", "motor_r", (-17, AXLE_Y - 4, 11), NX_, "Encoder: GND, Vcc, A, B"),
    # sensors and switches under the lower deck
    port("cliff_fl.conn", "cliff_fl", (116, BASE_Y - 5, -87), NX_, "3V3, GND, SDA, SCL, XSHUT"),
    port("cliff_fr.conn", "cliff_fr", (116, BASE_Y - 5, 87), NX_, "3V3, GND, SDA, SCL, XSHUT"),
    port("cliff_rl.conn", "cliff_rl", (-116, BASE_Y - 5, -87), PX, "3V3, GND, SDA, SCL, XSHUT"),
    port("cliff_rr.conn", "cliff_rr", (-116, BASE_Y - 5, 87), PX, "3V3, GND, SDA, SCL (no XSHUT)"),
    port("bump.l", "bumper", (157, 81, -98), NX_, "Left switch C, NO"),
    port("bump.r", "bumper", (157, 81, 98), NX_, "Right switch C, NO"),
    # charge path
    port("cc+", "contact_p", (145, Y0 + 5, 57), PY, "Contact + lug"),
    port("cc-", "contact_n", (145, Y0 + 5, -57), PY, "Contact - lug"),
    port("chg.in", "chg", (124, Y0 + 6, 65.5), PX, "F2 holder lead (from contact +)"),
    port("chg.out", "chg", (64, Y0 + 6, 65.5), NX_, "D1 OUT"),
    # deck plugs
    port("plp.cmp+", "plug_pwr", (-138, 163, -97), NY_, "Pin 1 CMP+"),
    port("plp.cmp-", "plug_pwr", (-134, 163, -97), NY_, "Pin 2 CMP-"),
    port("plp.sns+", "plug_pwr", (-138, 163, -94), NY_, "Pin 3 SNS+"),
    port("plp.sns-", "plug_pwr", (-134, 163, -94), NY_, "Pin 4 SNS-"),
    port("plp.es+", "plug_pwr", (-138, 163, -91), NY_, "Pin 5 loop out"),
    port("plp.es-", "plug_pwr", (-134, 163, -91), NY_, "Pin 6 loop return"),
    port("pup.cmp+", "plug_pwr", (-138, 193, -97), PY, "Pin 1 CMP+"),
    port("pup.cmp-", "plug_pwr", (-134, 193, -97), PY, "Pin 2 CMP-"),
    port("pup.sns+", "plug_pwr", (-138, 193, -94), PY, "Pin 3 SNS+"),
    port("pup.sns-", "plug_pwr", (-134, 193, -94), PY, "Pin 4 SNS-"),
    port("pup.es+", "plug_pwr", (-138, 193, -91), PY, "Pin 5 loop out"),
    port("pup.es-", "plug_pwr", (-134, 193, -91), PY, "Pin 6 loop return"),
    port("pls.drv", "plug_sig", (-138, 153, 90), NY_, "Pins 1-7 driver"),
    port("pls.enc_l", "plug_sig", (-134, 153, 90), NY_, "Pins 8-9 left encoder"),
    port("pls.enc_r", "plug_sig", (-138, 153, 93), NY_, "Pins 10-11 right encoder"),
    port("pls.pdb", "plug_sig", (-134, 153, 93), NY_, "I2C, 3V3"),
    port("pls.cfl", "plug_sig", (-138, 153, 95), NY_, "XSHUT1"),
    port("pls.cfr", "plug_sig", (-134, 153, 95), NY_, "XSHUT2"),
    port("pls.crl", "plug_sig", (-138, 153, 97), NY_, "XSHUT3"),
    port("pls.crr", "plug_sig", (-134, 153, 97), NY_, "I2C and 3V3 branch (no XSHUT)"),
    port("pls.bump", "plug_sig", (-136, 153, 98), NY_, "BUMP_L, BUMP_R"),
    port("pus", "plug_sig", (-136, 196, 94), PY, "24-pin upper half"),
    # top deck underside
    port("cpwr.in+", "cpwr", (-31, TU - 11, -90), NX_, "INA260 #2 VIN+"),
    port("cpwr.in-", "cpwr", (-31, TU - 11, -94), NX_, "GND"),
    port("cpwr.out+", "cpwr", (-31, TU - 11, -108), NX_, "Buck A OUT 5.1 V"),
    port("cpwr.out-", "cpwr", (-31, TU - 11, -112), NX_, "GND"),
    port("cpwr.i2c", "cpwr", (31, TU - 11, -100), PX, "INA260 #2 I2C"),
    port("spwr.in+", "spwr", (-31, TU - 11, 90), NX_, "INA260 #3 VIN+"),
    port("spwr.in-", "spwr", (-31, TU - 11, 94), NX_, "GND"),
    port("spwr.esp+", "spwr", (-31, TU - 11, 108), NX_, "5 V to ESP32 side"),
    port("spwr.esp-", "spwr", (-31, TU - 11, 112), NX_, "GND"),
    port("spwr.oak+", "spwr", (31, TU - 11, 88), PX, "5 V to OAK-D"),
    port("spwr.oak-", "spwr", (31, TU - 11, 92), PX, "GND"),
    port("spwr.k2+", "spwr", (31, TU - 11, 100), PX, "5 V to K2"),
    port("spwr.k2-", "spwr", (31, TU - 11, 104), PX, "GND"),
    port("spwr.i2c", "spwr", (31, TU - 11, 114), PX, "INA260 #3 I2C in"),
    port("spwr.lid+", "spwr", (-12, TU - 11, 81), NZ_, "5 V to LiDAR"),
    port("spwr.lid-", "spwr", (-8, TU - 11, 81), NZ_, "GND"),
    port("spwr.i2c2", "spwr", (10, TU - 11, 81), NZ_, "I2C on to INA260 #2"),
    port("k2.loop+", "k2", (106, TU - 7, 33), PX, "Relay COM"),
    port("k2.loop-", "k2", (106, TU - 7, 37), PX, "Relay NC"),
    port("k2.servo", "k2", (106, TU - 7, 43), PX, "RC input (servo lead)"),
    port("k2.stat", "k2", (106, TU - 7, 48), PX, "GOOD, OUT, GND"),
    port("k2.pwr+", "k2", (106, TU - 7, 51), PX, "5 V"),
    port("k2.pwr-", "k2", (106, TU - 7, 54), PX, "GND"),
    port("j2.lidar", "j2", (-39, TU - 6, 0), PX, "From LiDAR"),
    port("j2.pwr", "j2", (-55, TU - 6, 3), NX_, "5 V, GND"),
    port("j2.uart", "j2", (-55, TU - 6, -3), NX_, "TX, GND"),
    port("es.nc1+", "estop", (-123, 184, -8), NX_, "NC1 in"),
    port("es.nc1-", "estop", (-123, 184, -4), NX_, "Loop return (through K2)"),
    port("es.k2+", "estop", (-87, 184, -8), PX, "NC1 out to K2"),
    port("es.k2-", "estop", (-87, 184, -4), PX, "Loop return from K2"),
    port("es.nc2", "estop", (-123, 184, 6), NX_, "NC2 (echo)"),
    port("led_rec.t", "led_rec", (130, TU - 24, -80), NY_, "LED terminals"),
    port("led_status.t", "led_status", (130, TU - 24, 80), NY_, "LED terminals"),
    # top deck
    port("lidar.conn", "lidar", (0, LIDAR_Y, 0), NY_, "ZH1.5 4-pin"),
    port("pi.usbc", "pi5", (-47.25, PI_Y + 3.2, -99), NZ_, "USB-C power in"),
    port("pi.usb3", "pi5", (-123.5, PI_Y + 9, -60), NX_, "USB 3.0 (blue)"),
    port("pi.usb2", "pi5", (-123.5, PI_Y + 9, -78), NX_, "USB 2.0 (black, lower)"),
    port("pi.usb2b", "pi5", (-123.5, PI_Y + 16, -78), NX_, "USB 2.0 (black, upper)"),
    port("hat.uart0", "pi_hat", (-40, HAT_Y + 10, -84), PY, "UART0: pins 8, 10, 6"),
    port("hat.uart2", "pi_hat", (-40, HAT_Y + 10, -70), PY, "UART2 RX: pins 29, 30"),
    port("hat.led", "pi_hat", (-40, HAT_Y + 10, -56), PY, "Pin 16 via transistor, 5 V"),
    port("esp.sig", "esp32", (-100, ESP_Y + 11.6, 81.5), PY, "2 x 12 box header"),
    port("esp.uart", "esp32", (-47, ESP_J, 42), PY, "UART1 GPIO17/18, GND"),
    port("esp.pwr+", "esp32", (-47, ESP_J, 48), PY, "5 V in"),
    port("esp.pwr-", "esp32", (-47, ESP_J, 51), PY, "GND"),
    port("esp.estop", "esp32", (-47, ESP_J, 58), PY, "GPIO1, GND"),
    port("esp.led", "esp32", (-47, ESP_J, 64), PY, "GPIO2 via transistor"),
    port("esp.k2", "esp32", (-47, ESP_J, 70), PY, "K2 OUT to GPIO40 via the 74LVC244"),
    port("esp.i2c", "esp32", (-47, ESP_J, 78), PY, "I2C to the top-deck INA260s"),
    port("rx1.servo", "rx1", (84, T0 + 7, 53), PX, "CH5 servo lead"),
    port("ya.cam", "yadapter", (75, T0 + 5, 10), PZ, "Camera USB-C"),
    port("ya.host", "yadapter", (53, T0 + 5, 0), NX_, "Host USB-C"),
    port("ya.pwr", "yadapter", (75, T0 + 5, -10), NZ_, "Power USB-C"),
    port("oak.usbc", "oak", (125, T0 + 25, -40), NX_, "USB-C"),
    port("mic.usb", "mic", (33, T0 + 8, -92), NX_, "micro-USB"),
    port("spk.usb", "speaker", (55, T0 + 15, 100), NX_, "USB (captive cable)"),
])


def PWR(cid, name, cat, a, b, gauge, od, spec, conn, notes="", single=False, via=None):
    return dict(id=cid, name=name, layer="power", cat=cat, a=a, b=b, gauge=gauge, od=od, spec=spec,
                conn=conn, notes=notes, pair=not single, via=via or [])


def DAT(cid, name, cat, a, b, conductors, od, spec, conn, pins, notes="", via=None):
    return dict(id=cid, name=name, layer="data", cat=cat, a=a, b=b, gauge=conductors, od=od, spec=spec,
                conn=conn, pins=pins, notes=notes, pair=False, via=via or [])


CABLES = [
    # ----------------------------------------------------------------- power --
    PWR("P01", "Pack to splices", "supply", ("xt60.+", "xt60.-"), ("wpk.1", "wg.1"), "2 x 14 AWG silicone", 3.2,
        "14.8 V nominal (12.0-16.8 V), up to 15 A", "XT60 robot half to the WAGO pack node and star ground",
        "The XT60 is the service disconnect: unplug it before touching the power board."),
    PWR("P02", "Motor rail", "motor", ("pdb.mot+", "pdb.mot-"), ("drv.vin+", "drv.vin-"), "2 x 14 AWG silicone", 3.2,
        "14.8 V switched by K1, 3 A typical, 10 A peak (both motors)", "TB2 on the power board to the driver's VM/GND screws",
        "Dead whenever the e-stop is pressed or the RC link drops: K1's coil loop opens in hardware."),
    PWR("P03", "Left motor", "motor", ("drv.m1a", "drv.m1b"), ("mot_l.pwr+", "mot_l.pwr-"), "2 x 18 AWG, twisted", 2.3,
        "0-14.8 V PWM at 20 kHz", "Driver M1 screws to the motor leads (Micro-Fit 2-pin at the module)",
        "Leave a 40 mm loop at the module: it moves 10 mm up and 5 mm down."),
    PWR("P04", "Right motor", "motor", ("drv.m2a", "drv.m2b"), ("mot_r.pwr+", "mot_r.pwr-"), "2 x 18 AWG, twisted", 2.3,
        "0-14.8 V PWM at 20 kHz", "Driver M2 screws to the motor leads (Micro-Fit 2-pin at the module)",
        "Leave a 40 mm loop at the module."),
    PWR("P05", "Compute feed, lower", "compute", ("pdb.cmp+", "pdb.cmp-"), ("plp.cmp+", "plp.cmp-"), "2 x 18 AWG", 2.3,
        "14.8 V, 1 A typical, fused 3 A (F3)", "TB3 to deck power plug pins 1-2", ""),
    PWR("P06", "Compute feed, upper", "compute", ("pup.cmp+", "pup.cmp-"), ("cpwr.in+", "cpwr.in-"), "2 x 18 AWG", 2.3,
        "14.8 V into INA260 #2, D2, C1 and Buck A", "Deck power plug pins 1-2 to the compute block", ""),
    PWR("P07", "Pi 5 power", "compute", ("cpwr.out+", "cpwr.out-"), "pi.usbc", "2 x 18 AWG USB-C pigtail", 2.3,
        "5.1 V, 5 A max", "Buck A output to a USB-C plug",
        "The Pi cannot negotiate 5 A over a plain pigtail, so it limits USB to 600 mA; add usb_max_current_enable=1 if needed."),
    PWR("P08", "Sensor feed, lower", "sensor", ("pdb.sns+", "pdb.sns-"), ("plp.sns+", "plp.sns-"), "2 x 18 AWG", 2.3,
        "14.8 V, 0.8 A typical, fused 3 A (F4)", "TB3 to deck power plug pins 3-4", ""),
    PWR("P09", "Sensor feed, upper", "sensor", ("pup.sns+", "pup.sns-"), ("spwr.in+", "spwr.in-"), "2 x 18 AWG", 2.3,
        "14.8 V into INA260 #3 and Buck B", "Deck power plug pins 3-4 to the sensor block", ""),
    PWR("P10", "OAK-D power", "sensor", ("spwr.oak+", "spwr.oak-"), "ya.pwr", "2 x 20 AWG USB-C lead", 2.0,
        "5 V, 0.9 A typical, 3 A worst case", "Buck B output to the Y-adapter's power port", "Keeps up to 15 W off the Pi's USB."),
    PWR("P11", "LiDAR power", "sensor", ("spwr.lid+", "spwr.lid-"), "j2.pwr", "2 x 24 AWG", 1.5,
        "5 V, 0.29 A", "Buck B output to the LiDAR splice", ""),
    PWR("P12", "Controller power", "sensor", ("spwr.esp+", "spwr.esp-"), ("esp.pwr+", "esp.pwr-"), "2 x 22 AWG", 1.7,
        "5 V, 0.6 A: ESP32, encoders, cliff sensors (3.3 V from the DevKit), level shifter", "JST-XH 2-pin at the carrier", ""),
    PWR("P13", "RC relay power", "sensor", ("spwr.k2+", "spwr.k2-"), ("k2.pwr+", "k2.pwr-"), "2 x 24 AWG", 1.5,
        "5 V, 80 mA (relay + receiver)", "JST-XH 2-pin", "Also powers the receiver through the servo lead."),
    PWR("P14", "Power board feed", "supply", ("wpk.2", "wg.2"), ("pdb.in+", "pdb.in-"), "2 x 14 AWG silicone", 3.2,
        "14.8 V into F1 (15 A) and the power-board ground", "WAGO to TB1", ""),
    PWR("P15", "Charge contact +", "supply", "cc+", "chg.in", "1 x 16 AWG", 2.6,
        "16.8 V CC/CV from the dock, 3 A", "M4 ring lug on the contact to the F2 holder lead", "", single=True),
    PWR("P16", "Charge contact -", "supply", "cc-", "wg.3", "1 x 16 AWG", 2.6,
        "Charge return, 3 A", "M4 ring lug to the star ground", "", single=True),
    PWR("P17", "Charge to pack", "supply", "chg.out", "wpk.3", "1 x 16 AWG", 2.6,
        "16.8 V, 3 A into the pack node ahead of F1", "D1 OUT to the WAGO pack node",
        "The robot keeps running on dock power while it charges.", single=True),
    PWR("P18", "E-stop loop, lower", "supply", ("pdb.coil+", "pdb.coil-"), ("plp.es+", "plp.es-"), "2 x 20 AWG", 2.0,
        "14.8 V coil circuit, 0.12 A", "TB4 to deck power plug pins 5-6", ""),
    PWR("P19", "E-stop loop, upper", "supply", ("pup.es+", "pup.es-"), ("es.nc1+", "es.nc1-"), "2 x 20 AWG", 2.0,
        "14.8 V coil circuit through NC1", "Deck power plug pins 5-6 to the e-stop block", ""),
    PWR("P20", "E-stop loop to K2", "supply", ("es.k2+", "es.k2-"), ("k2.loop+", "k2.loop-"), "2 x 20 AWG", 2.0,
        "Coil circuit through K2's contact", "E-stop block to K2's screw terminals",
        "Press the e-stop, switch the transmitter off or lose the radio link: the loop opens and K1 drops the motor rail."),
    # ------------------------------------------------------------------ data --
    DAT("D01", "OAK-D to Y-adapter", "usb", "oak.usbc", "ya.cam", "USB-C to USB-C, 0.15 m", 4.2,
        "USB 3.2 Gen 1, 5 Gbit/s: video, depth, IMU", "USB-C both ends", ""),
    DAT("D02", "Y-adapter to Pi", "usb", "ya.host", "pi.usb3", "USB-C to USB-A 3.0, 0.3 m", 4.2,
        "USB 3.2 Gen 1 to the Pi's blue port", "USB-C to USB-A", "Blue USB 3.0 port, rear edge"),
    DAT("D03", "LiDAR lead", "uart", "lidar.conn", "j2.lidar", "4 x 26 AWG (ZH1.5 4-pin)", 2.8,
        "TX 3.3 V UART 230400 8N1, PWM (unused), GND, 5 V", "ZH1.5T-4P at the LiDAR, splice at J2",
        "Pin 1 TX, pin 2 PWM (open), pin 3 GND, pin 4 5 V", "Down the riser bore and the 20 mm deck hole."),
    DAT("D04", "LiDAR scan data", "uart", "j2.uart", "hat.uart2", "2 x 26 AWG", 1.8,
        "UART 230400, LiDAR to Pi only (/scan at 10 Hz)", "Splice to the HAT's JST-XH",
        "LiDAR TX to Pi pin 29 (GPIO5, UART2 RX); GND to pin 30. dtoverlay=uart2-pi5", ""),
    DAT("D05", "micro-ROS link", "uart", "hat.uart0", "esp.uart", "3 x 26 AWG", 2.0,
        "UART 921600, full duplex: /cmd_vel down at 50 Hz, odometry and IMU up at 100 Hz", "JST-XH 3-pin both ends",
        "Pi pin 8 (GPIO14 TX) to ESP32 GPIO18; Pi pin 10 (GPIO15 RX) to GPIO17; pin 6 GND. dtoverlay=uart0-pi5",
        "The 300 ms command watchdog lives on the ESP32 end."),
    DAT("D06", "Deck signal cable", "io", "pus", "esp.sig", "24 x 26 AWG, 2 x 12 IDC/Micro-Fit", 6.5,
        "Everything the ESP32 needs from the lower deck", "Micro-Fit 24 at the deck plug, 2 x 12 IDC at the carrier",
        "Encoders 4, driver 6, I2C 2, XSHUT 3, bumper 2, 5 V, 3V3, GND x 2, 3 spare",
        "Unplug at the deck plug to lift the top deck."),
    DAT("D07", "Motor commands", "drive", "pls.drv", "drv.ctrl", "7 x 26 AWG", 2.6,
        "PWM 20 kHz plus INA/INB per channel, 3.3 V logic", "Deck plug branch to the driver's control header",
        "GPIO4 PWM1, GPIO5 INA1, GPIO6 INB1, GPIO7 PWM2, GPIO15 INA2, GPIO16 INB2, GND", ""),
    DAT("D08", "Left encoder", "drive", "mot_l.enc", "pls.enc_l", "4 x 26 AWG", 2.2,
        "Quadrature A/B, 5 V swing", "Micro-Fit 4-pin at the module, deck plug branch",
        "GND, 5 V, A to GPIO10 and B to GPIO11 through the 74LVC244",
        "Leave a 40 mm loop at the module."),
    DAT("D09", "Right encoder", "drive", "mot_r.enc", "pls.enc_r", "4 x 26 AWG", 2.2,
        "Quadrature A/B, 5 V swing", "Micro-Fit 4-pin at the module, deck plug branch",
        "GND, 5 V, A to GPIO12 and B to GPIO13 through the 74LVC244", ""),
    DAT("D10", "Motor current sense", "io", "pls.pdb", "pdb.i2c", "4 x 26 AWG", 2.2,
        "I2C 400 kHz to INA260 #1 (0x40), 3.3 V", "Deck plug branch to JST-XH 4", "3V3, GND, SDA GPIO8, SCL GPIO9", ""),
    DAT("D11", "Cliff sensor FL", "io", "pls.cfl", "cliff_fl.conn", "5 x 26 AWG", 2.4,
        "I2C at 3.3 V plus XSHUT", "Deck plug branch to the STEMMA QT (JST-SH 4) plus the XSHUT header pin", "XSHUT1 GPIO21", "Through the front pass-through."),
    DAT("D12", "Cliff sensor FR", "io", "pls.cfr", "cliff_fr.conn", "5 x 26 AWG", 2.4,
        "I2C at 3.3 V plus XSHUT", "Deck plug branch to the STEMMA QT (JST-SH 4) plus the XSHUT header pin", "XSHUT2 GPIO14", ""),
    DAT("D13", "Cliff sensor RL", "io", "pls.crl", "cliff_rl.conn", "5 x 26 AWG", 2.4,
        "I2C at 3.3 V plus XSHUT", "Deck plug branch to the STEMMA QT (JST-SH 4) plus the XSHUT header pin", "XSHUT3 GPIO39", ""),
    DAT("D14", "Cliff sensor RR", "io", "pls.crr", "cliff_rr.conn", "4 x 26 AWG", 2.2,
        "I2C at 3.3 V", "Deck plug branch to the STEMMA QT (JST-SH 4)", "3V3, GND, SDA, SCL. No XSHUT: the firmware readdresses this sensor first",
        "Rev B: its XSHUT line (GPIO40) went to K2's status signal."),
    DAT("D15", "Bumper switch L", "io", "pls.bump", "bump.l", "2 x 24 AWG", 1.8,
        "Contact closure, pull-up on the ESP32", "Deck plug branch to the switch tabs", "BUMP_L GPIO41, GND", ""),
    DAT("D16", "Bumper switch R", "io", "pls.bump", "bump.r", "2 x 24 AWG", 1.8,
        "Contact closure, pull-up on the ESP32", "Deck plug branch to the switch tabs", "BUMP_R GPIO42, GND", ""),
    DAT("D17", "E-stop echo", "io", "es.nc2", "esp.estop", "2 x 26 AWG", 1.8,
        "Contact state for logging", "NC2 block to JST-XH 2", "GPIO1 (pull-up), GND. High when pressed",
        "Reporting only. Stopping never depends on this wire."),
    DAT("D18", "Status light", "io", "esp.led", "led_status.t", "2 x 24 AWG", 1.8,
        "5 V LED, switched low-side", "JST-XH to the indicator's tabs", "GPIO2 drives the transistor", ""),
    DAT("D19", "Recording light", "io", "hat.led", "led_rec.t", "2 x 24 AWG", 1.8,
        "5 V LED, switched low-side", "JST-XH to the indicator's tabs", "Pi pin 16 (GPIO23) drives the transistor; 5 V from pin 4",
        "Driven by the Pi because the Pi knows when a stream is live."),
    DAT("D20", "Microphone array", "usb", "mic.usb", "pi.usb2", "USB 2.0 micro-B to A, 0.3 m", 3.8,
        "USB audio class, 16 kHz, 4 mics + echo reference", "micro-USB to USB-A", "Black USB 2.0 port", ""),
    DAT("D21", "Speaker", "usb", "spk.usb", "pi.usb2b", "USB 2.0 captive cable", 3.8,
        "USB audio playback", "USB-A", "Black USB 2.0 port", ""),
    DAT("D22", "RC receiver to K2", "io", "rx1.servo", "k2.servo", "3 x 26 AWG servo lead", 2.0,
        "RC PWM, 5 V and ground to the receiver", "Servo plug both ends", "Receiver CH5 (switch channel)", ""),
    DAT("D23", "K2 status", "io", "k2.stat", "esp.k2", "3 x 26 AWG", 2.0,
        "OUT (relay on). K2's outputs swing to 5 V through 220 ohm", "JST-XH 3",
        "OUT to GPIO40 through a spare 74LVC244 channel; GOOD is a spare conductor, not connected at the carrier; GND",
        "Never straight to an ESP32 pin: K2's outputs are 5 V, and GPIO47/48 run at 1.8 V on the N32R16V."),
    DAT("D24", "Top-deck I2C", "io", "esp.i2c", "spwr.i2c", "4 x 26 AWG", 2.2,
        "I2C to INA260 #3 (0x44)", "JST-XH 4 both ends", "3V3, GND, SDA, SCL", ""),
    DAT("D25", "I2C to compute block", "io", "spwr.i2c2", "cpwr.i2c", "4 x 26 AWG", 2.2,
        "I2C on to INA260 #2 (0x41)", "JST-XH 4 both ends", "3V3, GND, SDA, SCL", ""),
]
CABLE = {c["id"]: c for c in CABLES}

MANUAL = {
    "D03": [(0, LIDAR_Y, 0), (0, T0 + 20, 0), (0, T0, 0), (0, TU - 4, 0), (-8, TU - 6, 0), (-39, TU - 6, 0)],
}

CAT_LABEL = dict(supply="Pack, bus and charge", motor="Motor rail, 14.8 V switched", compute="Compute rail",
                 sensor="Sensor rail", usb="USB", uart="Serial (UART)", drive="Drive signals", io="Sensor and safety I/O")


def keepouts(XX, YY, ZZ):
    """Router keep-out volumes (DESIGN_RULES.md section 2), in mm."""
    ax, az = abs(XX), abs(ZZ)
    k = (ax > 158) | (az > 127)                                                         # footprint
    k |= (ax > POST_X) & (az > POST_Z) & (((ax - POST_X) ** 2 + (az - POST_Z) ** 2) > (CORNER_R - 3) ** 2)  # rounded corners
    k |= YY < 55                                                                        # ground clearance
    k |= (abs(XX) < WELL_HALF + 3) & (abs(ZZ) > WELL_Z - 3) & (YY < WHEEL_D + TRAVEL + 6)  # wheel wells
    k |= YY > SCAN_BAND[0] - 3                                                          # LiDAR band
    k |= (XX > 146) & (YY > T0 + 8)                                                     # OAK-D view
    k |= (XX > -96) & (XX < -60) & (ZZ > -86) & (ZZ < -50) & (YY > HAT_Y)              # Pi fan opening in the HAT
    k |= (XX > 55) & (XX < 175) & (abs(ZZ) < 42) & (YY > Y0) & (YY < Y0 + 53)        # battery slide-out path
    k |= (XX > -40) & (XX < 60) & (abs(ZZ) < 42) & (YY > Y0 + 45) & (YY < TU - 18)  # above the pack
    for sx in (1, -1):                                                                  # caster swing
        k |= (((XX - sx * CASTER_X) ** 2 + ZZ ** 2) < 64 ** 2) & (YY < BASE_Y)
    return k
