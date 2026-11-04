"""3D-printed parts (3DPrinted). PETG unless noted; bumper TPU 95A.

Heat-set insert holes (brass, e.g. CNC Kitchen / Ruthex): M2.5 d3.6 x 4.5, M3 d4.0 x 5.7, M4 d5.6 x 8.1.
Each part's origin is the layout interface frame it mounts to (see IF_SPECS in build_000_master_layout.py).

    python build_300_printed.py [301 302 ...]
"""
import math
import os
import sys
import traceback

sys.path.insert(0, os.path.dirname(__file__))
from cots import Cots  # noqa: E402
from rv_sw import app, cast, read_globals  # noqa: E402

G = read_globals()
Q = lambda n: f'"{n}"'          # noqa: E731
PR = "3DPrinted"
PETG, TPU = (0.9, 0.91, 0.93), (0.83, 0.34, 0.1)
M25, M3, M4 = (3.6, 4.5), (4.0, 5.7), (5.6, 8.1)


def p301_head(sw):
    """Local origin = tower IF_Head (top of the posts, between them). Front-open shell; bezel closes the front,
    touchscreen sits on the 35 deg rear slope."""
    c = Cots(sw, "RV-301", "Head", PR, "Head shell (front-open, 3mm walls), carries bezel, OAK and 7in display",
             source="PRINT", color=PETG, Material="PETG", Print="split L/R if bed < 210mm; 3 perimeters",
             DimSource="layout SK_Side_Tower head profile; posts bolt up with 2x M5 into the 2020 bores")
    ox, oz = G["POST_X"], G["HEAD_Z0"]
    x0, x1, xt, zt = G["HEAD_X0"] - ox, G["HEAD_X1"] - ox, G["HEAD_TOP_X"] - ox, G["HEAD_TOP"] - oz
    zr = zt - (xt - x0) * math.tan(math.radians(G["SCREEN_ANG"]))
    W = G["HEAD_W"]
    prof = [(x0, 0), (x1, 0), (x1, zt), (xt, zt), (x0, zr)]
    c.feature("Body", "y", -W / 2, [("poly", prof)], W, +1)
    c.p.shell("Shell", (x1, 0, zt / 2), 3.0)
    c.feature("FloorHoles", "z", 0, [("circle", (0, sy * G["POST_DY"] / 2), 5.3) for sy in (1, -1)]
              + [("circle", (-25, 0), 20.0)], 3, +1, cut=True)
    c.frame("IF_Mount")
    c.frame("IF_Bezel", (x1, 0, 0))
    c.frame("IF_YAdapter", (-40, 0, 3))
    c.frame_line("IF_Display", "y", 0, (x0, zr), (xt, zt))           # x up the slope, z out of the screen
    return c.finish()


def p302_bezel(sw):
    """Local origin = head IF_Bezel (front face, floor level). Snap-fit cover (tabs not modelled)."""
    c = Cots(sw, "RV-302", "HeadBezel", PR, "Head front bezel with OAK wedge mount and USB-C pass-through",
             source="PRINT", color=(0.2, 0.22, 0.25), Material="PETG", DimSource="head front opening")
    H = G["HEAD_TOP"] - G["HEAD_Z0"]
    zl = G["LENS_Z"] - G["HEAD_Z0"]
    c.feature("Plate", "x", 0, [("rect", (-G["HEAD_W"] / 2, 0), (G["HEAD_W"] / 2, H))], G["BEZEL_T"], +1)
    c.feature("WedgeScrews", "x", 0, [("circle", (sy * 15, zl), 3.4) for sy in (1, -1)], G["BEZEL_T"], +1, cut=True)
    c.feature("USB_Slot", "x", 0, [("rect", (-7, zl - 34), (7, zl - 22))], G["BEZEL_T"], +1, cut=True)
    c.frame("IF_Mount")
    c.frame("IF_Wedge", (G["BEZEL_T"], 0, zl))
    return c.finish()


def p303_wedge(sw):
    """Local origin = back face at lens height. 5 deg wedge: the OAK's front tilts OAK_TILT down."""
    c = Cots(sw, "RV-303", "OAK_Wedge", PR, "OAK-D Pro W 5 deg tilt wedge (VESA M4 to OAK, M3 inserts to bezel)",
             source="PRINT", color=(0.2, 0.22, 0.25), Material="PETG", DimSource="OAK 97x29.5, OAK_TILT")
    t0, tilt = 6.0, math.radians(G["OAK_TILT"])
    top = t0 + 29.5 * math.tan(tilt)
    c.feature("Body", "y", -48.5, [("poly", [(0, -19.5), (t0, -19.5), (top, 10), (0, 10)])], 97, +1)
    c.feature("Inserts_M3", "x", 0, [("circle", (sy * 15, 0), M3[0]) for sy in (1, -1)], M3[1], +1, cut=True)
    c.feature("VESA_M4", "x", 0, [("circle", (sy * 37.5, -4.75), 4.5) for sy in (1, -1)], 12, +1, cut=True)
    xf = t0 + 19.5 * math.tan(tilt)                                  # front face at lens height
    c.frame_line("IF_OAK", "y", 0, (xf, 0), (xf + 10 * math.cos(tilt), -10 * math.sin(tilt)), origin="start")
    c.frame("IF_Mount")
    return c.finish()


def p304_pedestal(sw):
    """Local origin = IF_LidarPedestal (deck top at LIDAR_X). Top face at LIDAR_MOUNT_Z puts the scan at SCAN_Z."""
    c = Cots(sw, "RV-304", "LidarPedestal", PR, "LiDAR pedestal, M4 to deck grid, M2.5 inserts for STL-19P",
             source="PRINT", color=PETG, Material="PETG", link_globals=True,
             DimSource="height = LIDAR_MOUNT_Z - DECK_Z; LiDAR hole positions APPROX (see RV-512)")
    H = G["LIDAR_MOUNT_Z"] - G["DECK_Z"]
    He = f'{Q("LIDAR_MOUNT_Z")} - {Q("DECK_Z")}'
    c.feature("Base", "z", 0, [("rectc", (0, 0), 50, 50)], 4, +1)
    c.feature("DeckHoles", "z", 0, [("circle", (sx * 20, sy * 20), 4.3) for sx in (1, -1) for sy in (1, -1)], 4, +1,
              cut=True)
    c.feature("Column", "z", 4, [("rectc", (0, 0), 30, 30)], H - 8, +1, depth_expr=f"{He} - 8")
    c.feature("Top", "z", H - 4, [("rectc", (0, 0), 58, 58)], 4, +1, plane_expr=f"{He} - 4")
    c.feature("Inserts_M25", "z", H, [("circle", (4.93, 23.4), M25[0]), ("circle", (4.93, -23.4), M25[0]),
                                      ("circle", (-23.39, 0), M25[0])], 4, -1, cut=True, plane_expr=He)
    c.frame("IF_Mount")
    c.frame("IF_Lidar", (0, 0, H), exprs={"z": He})
    return c.finish()


def cliff_windows(z_ref):
    """Bumper / skin cut-outs around the two cliff cradles of one end, as (y, z) rects relative to z_ref.
    Window: cradle + 1.5 mm, down to where the sensor cone leaves the bumper face, and widened inboard for the
    STEMMA cable that drops to the skin hole below the MidCross."""
    g = cliff_geom()
    zm = G["MIDX_Z"] - z_ref
    w2 = CRADLE_W / 2
    win = [("rect", (sy * G["CLIFF_Y"] - (w2 + 11 if sy > 0 else w2 + 1.5), zm + g["exit_z"] - 4),
            (sy * G["CLIFF_Y"] + (w2 + 1.5 if sy > 0 else w2 + 11), zm + g["zt"] + 2)) for sy in (1, -1)]
    return win, zm


def p305_bumper(sw):
    """Local origin = IF_Bumper (front of body, BUMPER_Z0). TPU; two microswitches in back pockets, two windows
    for the front cliff-sensor cradles (RV-307), which are fixed to the frame and sit 2 mm behind the face."""
    c = Cots(sw, "RV-305", "Bumper", PR, "Front soft bumper (TPU): 2 microswitch pockets, 2 cliff-sensor windows",
             source="PRINT", color=TPU, Material="TPU 95A", link_globals=True,
             DimSource="globals BODY_W, BUMPER_T, BUMPER_Z0/Z1, CLIFF_*")
    hw = G["BODY_W"] / 2 - 8
    c.feature("Body", "x", 0, [("rect", (-hw, 0), (hw, G["BUMPER_Z1"] - G["BUMPER_Z0"]),
                                {"w": f'{Q("BODY_W")} - 16', "h": f'{Q("BUMPER_Z1")} - {Q("BUMPER_Z0")}',
                                 "pu": f'{Q("BODY_W")} / 2 - 8'})],
              G["BUMPER_T"], +1, depth_expr=Q("BUMPER_T"))
    c.feature("SwitchPockets", "x", 0, [("rectc", (sy * 55, 45), 20, 12) for sy in (1, -1)], 8, +1, cut=True)
    c.feature("GuidePins", "x", 0, [("circle", (sy * 150, 20), 5.0) for sy in (1, -1)], 10, +1, cut=True)
    win, _ = cliff_windows(G["BUMPER_Z0"])
    c.feature("CliffWindows", "x", 0, win, G["BUMPER_T"], +1, cut=True)
    c.frame("IF_Mount")
    return c.finish()


def p306_padholder(sw):
    """Local origin = IF_ChargePads (pad contact face, CONTACT_X / CONTACT_Z). Hangs under the floor pan."""
    c = Cots(sw, "RV-306", "ChargePadHolder", PR, "Charging pad holder with dock-detect magnet pocket",
             source="PRINT", color=PETG, Material="PETG",
             DimSource="pads at CONTACT_Y; magnet d6x3 for the dock reed/Hall switch")
    cy = G["CONTACT_Y"]
    c.feature("Plate", "z", 0, [("rectc", (0, 0), 40, 160)], 5, +1)
    c.feature("PadPockets", "z", 0, [("rectc", (0, sy * cy), 20, 26) for sy in (1, -1)], 1, +1, cut=True)
    c.feature("PadInserts_M3", "z", 0, [("circle", (0, sy * cy), M3[0]) for sy in (1, -1)], 5, +1, cut=True)
    c.feature("Magnet", "z", 0, [("circle", (0, 0), 6.0)], 3, +1, cut=True)
    c.feature("FrameBolts", "z", 0, [("circle", (-14, sy * 60), 4.3) for sy in (1, -1)], 5, +1, cut=True)
    c.frame("IF_Mount")
    c.frame("IF_PadL", (0, cy, 0))
    c.frame("IF_PadR", (0, -cy, 0))
    return c.finish()


TOF_STACK, TOF_HALF = 2.6, 17.7 / 2      # VL53L4CD: board 1.6 + die 1.0; board 17.7 wide up the tilted face
CRADLE_W, CRADLE_BOLT_Y, CRADLE_SEAT = 46.0, 18.0, 6.0


def cliff_geom():
    """Cliff cradle geometry in the IF_Cliff frame: skin outer face, MidCross slot height, x outward.
    The board lies flush on a face tilted CLIFF_ANG from vertical so the die sits at CLIFF_X / CLIFF_Z."""
    a = math.radians(G["CLIFF_ANG"])
    n, t = (math.sin(a), -math.cos(a)), (math.cos(a), math.sin(a))     # face normal (out-down), up the face
    x_skin = G["RAIL_X"] + G["PROFILE"] / 2 + G["SKIN_T"]
    die = (G["CLIFF_X"] - x_skin, G["CLIFF_Z"] - G["MIDX_Z"])
    b = (die[0] - TOF_STACK * n[0], die[1] - TOF_STACK * n[1])          # board back centre = IF_Sensor

    def pt(u, v=0.0):                                                   # u up the face, v into the cradle
        return b[0] + u * t[0] - v * n[0], b[1] + u * t[1] - v * n[1]

    def z_at(x):                                                        # face height at x
        return b[1] + (x - b[0]) * math.tan(a)

    xf, zt = G["BUMPER_T"] - 2, 8.0          # front face 2 mm behind the bumper face; top clears the M4 head
    lo, hi = pt(-TOF_HALF), pt(TOF_HALF, -1.6)
    assert lo[0] >= 0.2, f"board touches the skin ({lo[0]:.2f})"
    assert hi[0] <= xf, f"board stands proud of the cradle front ({hi[0]:.2f} > {xf})"
    assert z_at(CRADLE_SEAT) <= -4.5, "M4 counterbore seat breaks into the sensor face"
    # lowest point where the 18 deg cone leaves the bumper's front plane
    exit_z = die[1] - (G["BUMPER_T"] - die[0]) / math.tan(a - math.radians(9))
    return dict(pt=pt, xf=xf, zt=zt, z0=z_at(0), zf=z_at(xf), exit_z=exit_z)


def p307_cliffmount(sw):
    """Local origin = IF_Cliff_xx on the end skin's outer face at MidCross height, x outward. Rigid cradle:
    2x M4 through the skin into the MidCross T-slot; the VL53L4CD lies flush on a CLIFF_ANG face, its STEMMA
    plugs in side pockets that open to the cradle flanks (cable runs down to a skin hole below the MidCross)."""
    g = cliff_geom()
    c = Cots(sw, "RV-307", "CliffSensorMount", PR,
             f"VL53L4CD cliff sensor cradle ({G['CLIFF_ANG']:g} deg), clamps the end skin to the MidCross",
             source="PRINT", color=PETG, Material="PETG", Print="on its side, no supports",
             DimSource="CLIFF_X/Z/ANG; board screws APPROX (2x M2.5 self-tapping, drill with the board as template)")
    W = CRADLE_W
    c.feature("Body", "y", -W / 2, [("poly", [(0, g["z0"]), (g["xf"], g["zf"]), (g["xf"], g["zt"]), (0, g["zt"])])],
              W, +1)
    bolts = [(sy * CRADLE_BOLT_Y, 0) for sy in (1, -1)]
    c.feature("SkinBolts", "x", 0, [("circle", p, 4.5) for p in bolts], g["xf"], +1, cut=True)
    c.feature("BoltHeads", "x", CRADLE_SEAT, [("circle", p, 8.0) for p in bolts], g["xf"] - CRADLE_SEAT, +1, cut=True)
    plug = [g["pt"](u, v) for u, v in ((-2.65, 0), (2.65, 0), (2.65, 3.5), (-2.65, 3.5))]
    for sy, nm in ((1, "PlugL"), (-1, "PlugR")):                       # board plugs at +-9 across the cradle
        c.feature(nm, "y", sy * 5.5, [("poly", plug)], W / 2 - 5.5, sy, cut=True)
    c.frame("IF_Mount")
    c.frame_line("IF_Sensor", "y", 0, g["pt"](-TOF_HALF), g["pt"](TOF_HALF), x_axis="y")
    return c.finish()


def caster_block(c, rigid):
    """Shared body: top plate under the mid cross member + back wall bolted to it (toward local +x)."""
    c.feature("Top", "z", 14 if not rigid else 0, [("rect", (-25, -30), (38, 30))], 9.5 if not rigid else 23.5, +1)
    if not rigid:
        c.feature("BackWall", "z", 3.5, [("rect", (32, -30), (38, 30))], 10.5, +1)
    c.feature("RailBolts", "x", 38, [("circle", (sy * 15, 13.5), 4.3) for sy in (1, -1)], 8, -1, cut=True)


def p308_housing(sw):
    c = Cots(sw, "RV-308", "FrontCasterHousing", PR, "Front sprung caster housing (2 guide pins + 2 springs)",
             source="PRINT", color=PETG, Material="PETG",
             DimSource="10mm travel (carriage top 4 -> 14); bolts to front MidCross member")
    caster_block(c, rigid=False)
    c.feature("GuidePins", "z", 14, [("circle", (sx * 18, 0), 4.2) for sx in (1, -1)], 9.5, +1, cut=True)
    c.feature("SpringSeats", "z", 14, [("circle", (0, sy * 18), 10.0) for sy in (1, -1)], 5, +1, cut=True)
    c.frame("IF_Mount")
    return c.finish()


def p309_carriage(sw):
    c = Cots(sw, "RV-309", "FrontCasterCarriage", PR, "Front caster sprung carriage plate", source="PRINT",
             color=PETG, Material="PETG", DimSource="caster 30x30 bolt pattern, guide pins at +-18")
    c.feature("Plate", "z", 0, [("rectc", (0, 0), 44, 44)], 4, +1)
    c.feature("Holes", "z", 0, [("circle", (sx * 15, sy * 15), 4.5) for sx in (1, -1) for sy in (1, -1)]
              + [("circle", (sx * 18, 0), 4.4) for sx in (1, -1)], 4, +1, cut=True)
    c.frame("IF_Mount")
    return c.finish()


def p310_riser(sw):
    c = Cots(sw, "RV-310", "RearCasterRiser", PR, "Rear rigid caster riser (M4 inserts for caster plate)",
             source="PRINT", color=PETG, Material="PETG", DimSource="caster 30x30 pattern; bolts to rear MidCross")
    caster_block(c, rigid=True)
    c.feature("Inserts_M4", "z", 0, [("circle", (sx * 15, sy * 15), M4[0]) for sx in (1, -1) for sy in (1, -1)],
              M4[1], +1, cut=True)
    c.frame("IF_Mount")
    return c.finish()


def p311_computetray(sw):
    """Local origin = IF_Compute (pan top). Pi 5 at +y, ESP32 expansion board at -y."""
    c = Cots(sw, "RV-311", "ComputeTray", PR, "Compute tray: Pi 5 + ESP32 expansion board standoffs",
             source="PRINT", color=PETG, Material="PETG", link_globals=True,
             DimSource="COMPUTE_L x COMPUTE_W; Pi 58x49 holes; ESP32 board 4x M4 APPROX")
    c.feature("Plate", "z", 0, [("rectc", (0, 0), G["COMPUTE_L"], G["COMPUTE_W"],
                                 {"w": Q("COMPUTE_L"), "h": Q("COMPUTE_W")})], 3, +1)
    pi = [(-39, 38 + sy * 24.5) for sy in (1, -1)] + [(19, 38 + sy * 24.5) for sy in (1, -1)]
    esp = [(sx * 34.5, -45 + sy * 23.5) for sx in (1, -1) for sy in (1, -1)]
    c.feature("Standoffs", "z", 3, [("circle", p, 6.0) for p in pi] + [("circle", p, 10.0) for p in esp], 6, +1)
    c.feature("Inserts", "z", 9, [("circle", p, M25[0]) for p in pi] + [("circle", p, M4[0]) for p in esp], 6, -1,
              cut=True)
    c.feature("PanHoles", "z", 0, [("circle", (sx * 38, sy * 72), 4.3) for sx in (1, -1) for sy in (1, -1)], 3, +1,
              cut=True)
    c.frame("IF_Mount")
    c.frame("IF_Pi", (0, 38, 9))
    c.frame("IF_ESP32", (0, -45, 9))
    return c.finish()


def p312_drivermount(sw):
    c = Cots(sw, "RV-312", "DriverMount", PR, "DFR0601 motor driver mount (M3 inserts)", source="PRINT",
             color=PETG, Material="PETG", DimSource="DFR0601 45x45 hole square")
    c.feature("Plate", "z", 0, [("rectc", (0, 0), 56, 56)], 3, +1)
    pts = [(sx * 22.5, sy * 22.5) for sx in (1, -1) for sy in (1, -1)]
    c.feature("Standoffs", "z", 3, [("circle", p, 6.0) for p in pts], 5, +1)
    c.feature("Inserts_M3", "z", 8, [("circle", p, M3[0]) for p in pts], M3[1], -1, cut=True)
    c.feature("PanHoles", "z", 0, [("circle", (sx * 24, 0), 4.3) for sx in (1, -1)], 3, +1, cut=True)
    c.frame("IF_Mount")
    c.frame("IF_Driver", (0, 0, 8))
    return c.finish()


SKIN_H = f'{Q("TOPRING_Z")} + {Q("PROFILE")} / 2 - {Q("PAN_TOP")}'


def p313_skinside(sw):
    """Local origin = IF_SkinL/R (rail outer face, pan top, x = 0); +y outward. Wheel notch around the axle."""
    c = Cots(sw, "RV-313", "SkinSide", PR, "Side skin panel with wheel notch (used L and R)", source="PRINT",
             color=(0.23, 0.28, 0.34), Material="PETG", link_globals=True, Print="416 long: print in 2 halves",
             DimSource="frame length, TOPRING_Z, WHEEL_D/AXLE_Z notch")
    L2 = G["RAIL_X"] + G["PROFILE"] / 2
    h = G["TOPRING_Z"] + G["PROFILE"] / 2 - G["PAN_TOP"]
    c.feature("Panel", "y", 0, [("rect", (-L2, 0), (L2, h), {"w": f'2 * ({Q("RAIL_X")} + {Q("PROFILE")} / 2)',
                                                             "h": SKIN_H, "pu": f'{Q("RAIL_X")} + {Q("PROFILE")} / 2'})],
              G["SKIN_T"], +1, depth_expr=Q("SKIN_T"))
    nw = G["WHEEL_D"] / 2 + 5
    nh = G["AXLE_Z"] + G["WHEEL_D"] / 2 + 6 - G["PAN_TOP"]
    c.feature("WheelNotch", "y", 0, [("rect", (-nw, 0), (nw, nh), {"w": f'{Q("WHEEL_D")} + 10',
                                                                 "h": f'{Q("AXLE_Z")} + {Q("WHEEL_D")} / 2 + 6 - {Q("PAN_TOP")}',
                                                                 "pu": f'{Q("WHEEL_D")} / 2 + 5'})],
              G["SKIN_T"], +1, cut=True)
    c.frame("IF_Mount")
    return c.finish()


def skin_end(sw, pn, name, desc, opening):
    c = Cots(sw, pn, name, PR, desc, source="PRINT", color=(0.23, 0.28, 0.34), Material="PETG", link_globals=True,
             DimSource="frame width, TOPRING_Z")
    W2 = G["RAIL_Y"] + G["PROFILE"] / 2
    h = G["TOPRING_Z"] + G["PROFILE"] / 2 - G["PAN_TOP"]
    c.feature("Panel", "x", 0, [("rect", (-W2, 0), (W2, h), {"w": f'2 * ({Q("RAIL_Y")} + {Q("PROFILE")} / 2)',
                                                             "h": SKIN_H, "pu": f'{Q("RAIL_Y")} + {Q("PROFILE")} / 2'})],
              G["SKIN_T"], +1, depth_expr=Q("SKIN_T"))
    if opening:
        # opening clears the battery by 4 mm (sweep check found the pack top hitting a 46 mm opening)
        z0 = G["DRAWER_Z"] - 2 - G["PAN_TOP"]
        z1 = G["DRAWER_Z"] + G["SM_T"] + G["BAT_H"] + 4 - G["PAN_TOP"]
        c.feature("DrawerOpening", "x", 0, [("rect", (-66, z0), (66, z1))], G["SKIN_T"], +1, cut=True)
    else:
        c.feature("LightSlot", "x", 0, [("rect", (-130, h - 16), (130, h - 10))], G["SKIN_T"], +1, cut=True)
    # cliff cradles (RV-307): 2x M4 through the skin into the MidCross slot; cable hole inboard, between the
    # bottom ring and the MidCross
    zm = G["MIDX_Z"] - G["PAN_TOP"]
    c.feature("CliffBolts", "x", 0, [("circle", (sy * (G["CLIFF_Y"] + dy), zm), 4.5)
                                     for sy in (1, -1) for dy in (CRADLE_BOLT_Y, -CRADLE_BOLT_Y)],
              G["SKIN_T"], +1, cut=True)
    zc = G["BOTRING_Z"] + G["PROFILE"] / 2 + 4.5 - G["PAN_TOP"]
    c.feature("CliffCables", "x", 0, [("rect", (sy * (G["CLIFF_Y"] - CRADLE_W / 2 - 1) - 4, zc),
                                       (sy * (G["CLIFF_Y"] - CRADLE_W / 2 - 1) + 4, zc + 8)) for sy in (1, -1)],
              G["SKIN_T"], +1, cut=True)
    c.frame("IF_Mount")
    return c.finish()


def p314_skinfront(sw):
    return skin_end(sw, "RV-314", "SkinFront", "Front skin panel with status light slot", False)


def p315_skinrear(sw):
    return skin_end(sw, "RV-315", "SkinRear", "Rear skin panel with battery drawer opening", True)


def p316_drawerface(sw):
    """Local origin = tray IF_Face (rear end of the tray floor). Face plate + pull handle, outside the tray."""
    c = Cots(sw, "RV-316", "DrawerFace", PR, "Battery drawer face with pull handle", source="PRINT",
             color=(0.17, 0.2, 0.24), Material="PETG", DimSource="covers rear opening (battery + 4 mm)")
    top = G["SM_T"] + G["BAT_H"] + 4
    c.feature("Plate", "x", 0, [("rect", (-65, -2), (65, top))], 4, -1)
    c.feature("Handle", "x", -4, [("rect", (-35, top - 22), (35, top - 14))], 10, -1)
    c.frame("IF_Mount")
    return c.finish()


PARTS = {"301": p301_head, "302": p302_bezel, "303": p303_wedge, "304": p304_pedestal, "305": p305_bumper,
         "306": p306_padholder, "307": p307_cliffmount, "308": p308_housing, "309": p309_carriage,
         "310": p310_riser, "311": p311_computetray, "312": p312_drivermount, "313": p313_skinside,
         "314": p314_skinfront, "315": p315_skinrear, "316": p316_drawerface}

if __name__ == "__main__":
    sw = app()
    ok = 0
    keys = sys.argv[1:] or list(PARTS)
    for k in keys:
        try:
            r = PARTS[k](sw)
            ok += not r["not_fully_defined"]
            print(f"OK   {r['part']:40s} sketches={r['sketches']:2d} not_fully_defined={r['not_fully_defined']}")
        except Exception as e:
            traceback.print_exc()
            print(f"FAIL RV-{k}: {e}")
            for d in (sw.GetDocuments() or []):
                doc = cast(d, "IModelDoc2")
                if doc.GetPathName() == "":
                    sw.QuitDoc(doc.GetTitle())
    print(f"\n{ok}/{len(keys)} parts clean")
