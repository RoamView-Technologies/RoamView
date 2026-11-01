"""Sheet-metal parts (04_SheetMetal_Weldments/SheetMetal), 5052-H32 aluminium.

RV-430 deck plate (2 mm, 20 mm M4 interconnect grid)   RV-431 floor pan (1.5 mm)
RV-432 battery drawer tray (U, 1.5 mm)                 RV-433 drawer guide (L, 1.5 mm, used L and R)
RV-434 motor bracket (L, 3 mm, used L and R)           RV-435 charge pad (brass contact, 1 mm)

    python build_430_sheetmetal.py [430 431 ...]
"""
import os
import sys
import traceback

sys.path.insert(0, os.path.dirname(__file__))
from cots import Cots  # noqa: E402
from rv_sw import app, cast, read_globals  # noqa: E402

G = read_globals()
Q = lambda n: f'"{n}"'          # noqa: E731
SM = r"04_SheetMetal_Weldments\SheetMetal"
ALU = (0.82, 0.83, 0.85)


def p430_deck(sw):
    """Local origin = IF_Deck (deck underside, centre). Grid holes: M4 clearance on GRID pitch."""
    c = Cots(sw, "RV-430", "DeckPlate", SM, "Deck plate 2mm 5052 with 20mm M4 interconnect grid", source="SHEET",
             color=(0.36, 0.42, 0.48), link_globals=True, Material="5052-H32 2.0mm",
             DimSource="globals BODY_L x BODY_W, DECK_T, GRID")
    T = G["DECK_T"]
    c.sheet("Deck", "z", 0, [("rectc", (0, 0), G["BODY_L"], G["BODY_W"], {"w": Q("BODY_L"), "h": Q("BODY_W")})],
            T, T, want=("z", 0, T))
    nx = int(G["BODY_L"] // G["GRID"]) - 0
    ny = int(G["BODY_W"] // G["GRID"]) - 0
    x0, y0 = -(nx // 2) * G["GRID"], -(ny // 2) * G["GRID"]
    nx, ny = (nx // 2) * 2 + 1, (ny // 2) * 2 + 1
    seed = c.feature("GridSeed", "z", 0, [("circle", (x0, y0), 4.3)], T, +1, cut=True)
    ax = c.dir_axes()
    row = c.p.pattern1d("GridRow", [seed], ax["x"], nx, G["GRID"], "x", +1)
    c.p.pattern1d("GridPattern", [seed, row], ax["y"], ny, G["GRID"], "y", +1)
    c.feature("CablePorts", "z", 0, [("circle", (G["POST_X"] - 20, 0), 25.0), ("circle", (G["LIDAR_X"] - 25, 0), 16.0),
                                     ("circle", (G["ESTOP_X"], G["ESTOP_Y"]), 16.0)], T, +1, cut=True)
    c.frame("IF_Mount")
    c.props.update(GridHoles=f"{nx} x {ny} @ {G['GRID']:g}")
    return c.finish()


def p431_pan(sw):
    """Local origin = IF_FloorPan (pan underside centre). Flat 1.5 mm pan under the bottom ring."""
    c = Cots(sw, "RV-431", "FloorPan", SM, "Floor pan 1.5mm 5052 (caster sweep holes)",
             source="SHEET", color=ALU, link_globals=True, Material="5052-H32 1.5mm",
             DimSource="globals RAIL_X/RAIL_Y/PROFILE/PAN_T/CASTER_X")
    T = G["PAN_T"]
    L, W = 2 * (G["RAIL_X"] + G["PROFILE"] / 2), 2 * (G["RAIL_Y"] + G["PROFILE"] / 2)
    c.sheet("Pan", "z", 0, [("rectc", (0, 0), L, W, {"w": f'2 * ({Q("RAIL_X")} + {Q("PROFILE")} / 2)',
                                                     "h": f'2 * ({Q("RAIL_Y")} + {Q("PROFILE")} / 2)'})],
            T, T, want=("z", 0, T))
    holes = [("circle", (sx * G["CASTER_X"], 0), 84.0, {"pu": Q("CASTER_X")}) for sx in (1, -1)]
    c.feature("Holes", "z", 0, holes, T, +1, cut=True)          # cliff sensors moved to the end skins (RV-307)
    c.frame("IF_Mount")
    return c.finish()


def p432_tray(sw):
    """Local origin = IF_Drawer: tray underside on the centreline at x = 0. U channel, slides out the rear."""
    c = Cots(sw, "RV-432", "BatteryDrawerTray", SM, "Battery drawer tray, 1.5mm 5052 U-channel", source="SHEET",
             color=ALU, Material="5052-H32 1.5mm", DimSource="fits BAT_W 90 + 17mm/side foam, walls 43.5")
    T, x0, x1 = G["SM_T"], -(G["RAIL_X"] + G["PROFILE"] / 2), 125.0
    c.sheet("Tray", "x", x0, [("path", [(-62, 43.5), (-62, T), (62, T), (62, 43.5)])], T, T, depth=x1 - x0, sign=+1,
            want=("y", -62 - T, 62 + T))
    c.feature("FaceHoles", "z", 0, [("circle", (x0 + 8, sy * 40), 3.4) for sy in (1, -1)], T, +1, cut=True)
    c.frame("IF_Mount")
    c.frame("IF_Battery", (G["BAT_X"], 0, T))
    c.frame("IF_Face", (x0, 0, 0))
    return c.finish()


def p433_guide(sw):
    """Local origin = IF_GuideL (drawer running surface on the centreline). L angle, used left and right."""
    c = Cots(sw, "RV-433", "DrawerGuide", SM, "Drawer guide, 1.5mm 5052 L-angle (used L and R)", source="SHEET",
             color=ALU, Material="5052-H32 1.5mm", DimSource="rests on MidCross members at +-RAIL_X")
    T, x0 = G["SM_T"], -(G["RAIL_X"] + G["PROFILE"] / 2)
    c.sheet("Guide", "x", x0, [("path", [(48, 0), (65, 0), (65, 20)])], T, T, depth=-2 * x0, sign=+1,
            want=("y", 48, 65 + T))
    c.feature("Holes", "z", 0, [("circle", (sx * G["RAIL_X"], 56), 4.3) for sx in (1, -1)], T, -1, cut=True)
    c.frame("IF_Mount")
    return c.finish()


def p434_bracket(sw):
    """Local origin = IF_DriveL: gearbox face centre (motor axis), x fwd, y outboard, z up. Used L and R."""
    c = Cots(sw, "RV-434", "MotorBracket", SM, "Motor bracket, 3mm 5052 L (face plate inside wheel recess)",
             source="SHEET", color=(0.55, 0.57, 0.6), Material="5052-H32 3.0mm",
             DimSource="leg rests on bottom side rail; plate fits inside Wasteland d76 recess; inside bend R1.5 "
                       "(clears gearbox by 1.1mm - confirm R with fabricator)")
    T = 3.0
    zl = (G["BOTRING_Z"] + G["PROFILE"] / 2) + T - G["MOTOR_AXIS_Z"]          # leg inner surface (local z)
    yl = (G["RAIL_Y"] - G["PROFILE"] / 2 - 10) - G["MOTOR_FACE_Y"]            # leg end, 10 mm past the rail
    c.sheet("Bracket", "x", -21, [("path", [(0, 21), (0, zl), (yl, zl)])], T, T / 2, depth=42, sign=+1,
            want=("y", yl, T))
    c.feature("MotorHoles", "y", 0, c.bolt_circle((0, 0), 31.0, 6, 3.4) + [("circle", (0, -G["MOTOR_UP"]), 14.0)],
              T, +1, cut=True)
    yr = G["RAIL_Y"] - G["MOTOR_FACE_Y"]
    c.feature("RailHoles", "z", 0, [("circle", (sx * 12, yr), 4.3) for sx in (1, -1)], 30, -1, cut=True)
    c.frame("IF_Mount")
    c.frame("IF_Motor", x="x", y="y", flip_x=True)
    c.frame("IF_Hub", (0, G["BUSHING_L"] + G["HUB_GAP"], -G["MOTOR_UP"]))
    return c.finish()


def p435_pad(sw):
    c = Cots(sw, "RV-435", "ChargePad", SM, "Charging contact pad, 1mm brass", source="SHEET",
             color=(0.85, 0.65, 0.25), Material="C260 brass 1.0mm", DimSource="20 x 26 to suit dock contacts")
    c.sheet("Pad", "z", 0, [("rectc", (0, 0), 20, 26)], 1.0, 1.0, want=("z", 0, 1.0))
    c.feature("Screw", "z", 0, [("circle", (0, 0), 3.4)], 1.0, +1, cut=True)
    c.frame("IF_Mount")
    return c.finish()


PARTS = {"430": p430_deck, "431": p431_pan, "432": p432_tray, "433": p433_guide, "434": p434_bracket,
         "435": p435_pad}

if __name__ == "__main__":
    sw = app()
    for k in sys.argv[1:] or list(PARTS):
        try:
            r = PARTS[k](sw)
            print(f"OK   {r['part']:40s} sketches={r['sketches']:2d} not_fully_defined={r['not_fully_defined']}")
        except Exception as e:
            traceback.print_exc()
            print(f"FAIL RV-{k}: {e}")
            for d in (sw.GetDocuments() or []):
                doc = cast(d, "IModelDoc2")
                if doc.GetPathName() == "":
                    sw.QuitDoc(doc.GetTitle())
