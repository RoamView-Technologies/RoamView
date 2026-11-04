"""Weldments (SheetMetal_Weldments/Weldments).

RV-410 chassis frame: 2020 aluminium members as separate weldment bodies (bottom ring, top ring, corner
uprights, mid cross members that carry the drawer guides, top cross members under the tower/strut/LiDAR).
RV-420 sensor tower: two 2020 posts on foot plates, cross brace, two rear struts that end below the
LiDAR scan plane. Positions and lengths are driven by RV_globals.txt.

Profile is simplified to a 20x20 bar with the 4.2 mm centre bore (T-slots not modelled); the real 2020
mass (about 0.46 kg/m) is applied as a mass override by set_masses.py.

    python build_400_weldments.py [410] [420]
"""
import math
import os
import sys
import traceback

sys.path.insert(0, os.path.dirname(__file__))
from cots import Cots  # noqa: E402
from rv_sw import app, read_globals  # noqa: E402

G = read_globals()
Q = lambda n: f'"{n}"'          # noqa: E731
P, H = G["PROFILE"], G["PROFILE"] / 2
ALU = (0.75, 0.77, 0.8)
WELD = r"SheetMetal_Weldments\Weldments"


def sq(u, v, pu, pv):
    """20x20 member section centred at (u, v) plus its bore; pu/pv are the |centre| expressions."""
    su, sv = (1 if u >= 0 else -1), (1 if v >= 0 else -1)
    # corner nearer the origin on each axis -> |centre| - P/2 (or P/2 - |centre| if the centre is within P/2)
    cu, cv = u - su * H, v - sv * H
    e = {"w": Q("PROFILE"), "h": Q("PROFILE")}
    if abs(u) >= H:
        e["pu"] = f'abs({pu}) - {Q("PROFILE")} / 2'
    if abs(v) >= H:
        e["pv"] = f'abs({pv}) - {Q("PROFILE")} / 2'
    rect = ("rect", (cu, cv), (cu + su * P, cv + sv * P), e)
    ce = {}
    if abs(u) > 1e-9:
        ce["pu"] = f"abs({pu})"
    if abs(v) > 1e-9:
        ce["pv"] = f"abs({pv})"
    return [rect, ("circle", (u, v), 4.2, ce)]


def p410_frame(sw):
    c = Cots(sw, "RV-410", "ChassisFrame_2020", WELD, "Chassis frame, 2020 aluminium extrusion weldment",
             source="WELD", color=ALU, link_globals=True, Material="6063-T5 2020 T-slot",
             DimSource="globals RAIL_X/RAIL_Y/BOTRING_Z/TOPRING_Z/MIDX_Z/PROFILE")
    c.weldment()
    RX, RY = G["RAIL_X"], G["RAIL_Y"]
    xs = -(RX + H)                                         # side rails start at the rear outer face
    xs_e = f'{Q("RAIL_X")} + {Q("PROFILE")} / 2'
    L_side = f'2 * {Q("RAIL_X")} + {Q("PROFILE")}'
    for nm, zc, ze in (("Bot", G["BOTRING_Z"], Q("BOTRING_Z")), ("Top", G["TOPRING_Z"], Q("TOPRING_Z"))):
        c.feature(f"{nm}SideRails", "x", xs, sq(RY, zc, Q("RAIL_Y"), ze) + sq(-RY, zc, Q("RAIL_Y"), ze),
                  2 * RX + P, +1, depth_expr=L_side, merge=False, plane_expr=xs_e)
    ys = -(RY - H)
    ys_e = f'{Q("RAIL_Y")} - {Q("PROFILE")} / 2'
    L_end = f'2 * ({Q("RAIL_Y")} - {Q("PROFILE")} / 2)'
    for nm, zc, ze in (("BotEndRails", G["BOTRING_Z"], Q("BOTRING_Z")), ("TopEndRails", G["TOPRING_Z"], Q("TOPRING_Z")),
                       ("MidCross", G["MIDX_Z"], Q("MIDX_Z"))):
        c.feature(nm, "y", ys, sq(RX, zc, Q("RAIL_X"), ze) + sq(-RX, zc, Q("RAIL_X"), ze),
                  2 * (RY - H), +1, depth_expr=L_end, merge=False, plane_expr=ys_e)
    top_x = [(G["POST_X"], Q("POST_X")), (G["STRUT_X0"], Q("STRUT_X0")), (G["LIDAR_X"], Q("LIDAR_X"))]
    shapes = []
    for x, xe in top_x:
        shapes += sq(x, G["TOPRING_Z"], xe, Q("TOPRING_Z"))
    c.feature("TopCross", "y", ys, shapes, 2 * (RY - H), +1, depth_expr=L_end, merge=False, plane_expr=ys_e)
    z0 = G["BOTRING_Z"] + H
    shapes = []
    for sx in (1, -1):
        for sy in (1, -1):
            shapes += sq(sx * RX, sy * RY, Q("RAIL_X"), Q("RAIL_Y"))
    c.feature("Uprights", "z", z0, shapes, G["TOPRING_Z"] - G["BOTRING_Z"] - P, +1,
              depth_expr=f'{Q("TOPRING_Z")} - {Q("BOTRING_Z")} - {Q("PROFILE")}', merge=False,
              plane_expr=f'{Q("BOTRING_Z")} + {Q("PROFILE")} / 2')
    c.frame("IF_Mount")
    return c.finish()


def p420_tower(sw):
    """Local origin = IF_Tower: deck top, midway between the posts."""
    c = Cots(sw, "RV-420", "SensorTower_2020", WELD, "Sensor tower: twin 2020 posts, brace, rear struts",
             source="WELD", color=ALU, link_globals=True, Material="6063-T5 2020 T-slot / 6061 plate",
             DimSource="globals POST_*, BRACE_Z, STRUT_X0/STRUT_Z, DECK_Z")
    c.weldment()
    DY = G["POST_DY"] / 2
    ht = G["POST_TOP"] - G["DECK_Z"]
    # foot plates 56 x 36 x 5 under each post
    feet = []
    for sy in (1, -1):
        feet.append(("rect", (-28, sy * (DY - 18)), (28, sy * (DY + 18)),
                     {"pu": "28", "pv": f'{Q("POST_DY")} / 2 - 18'}))
    c.feature("FootPlates", "z", 0, feet, 5, +1, merge=False)
    posts = []
    for sy in (1, -1):
        posts += sq(0, sy * DY, "0", f'{Q("POST_DY")} / 2')
    c.feature("Posts", "z", 5, posts, ht - 5, +1, depth_expr=f'{Q("POST_TOP")} - {Q("DECK_Z")} - 5', merge=False)
    bz = G["BRACE_Z"] - G["DECK_Z"]
    c.feature("Brace", "y", -(DY - H), sq(0, bz, "0", f'{Q("BRACE_Z")} - {Q("DECK_Z")}'), 2 * (DY - H), +1,
              depth_expr=f'{Q("POST_DY")} - {Q("PROFILE")}', merge=False,
              plane_expr=f'{Q("POST_DY")} / 2 - {Q("PROFILE")} / 2')
    # rear struts: 20x20 bar from the deck to the rear face of the post, ending below the scan plane
    x0, xe, ze = G["STRUT_X0"] - G["POST_X"], -H, G["STRUT_Z"] - G["DECK_Z"]
    th = math.atan2(ze, xe - x0)
    hw, vw = H / math.sin(th), H / math.cos(th)
    prof = [(x0 - hw, 0), (x0 + hw, 0), (xe, ze - vw), (xe, ze + vw)]
    c.feature("StrutL", "y", DY - H, [("poly", prof)], P, +1, merge=False,
              plane_expr=f'{Q("POST_DY")} / 2 - {Q("PROFILE")} / 2')
    c.feature("StrutR", "y", -(DY + H), [("poly", prof)], P, +1, merge=False,
              plane_expr=f'{Q("POST_DY")} / 2 + {Q("PROFILE")} / 2')
    c.frame("IF_Mount")
    c.frame("IF_Head", (0, 0, ht), exprs={"z": f'{Q("POST_TOP")} - {Q("DECK_Z")}'})
    return c.finish()


PARTS = {"410": p410_frame, "420": p420_tower}

if __name__ == "__main__":
    sw = app()
    for k in sys.argv[1:] or list(PARTS):
        try:
            r = PARTS[k](sw)
            print(f"OK   {r['part']:40s} sketches={r['sketches']:2d} not_fully_defined={r['not_fully_defined']}")
        except Exception as e:
            traceback.print_exc()
            print(f"FAIL RV-{k}: {e}")
            from rv_sw import cast
            for d in (sw.GetDocuments() or []):
                doc = cast(d, "IModelDoc2")
                if doc.GetPathName() == "":
                    sw.QuitDoc(doc.GetTitle())
