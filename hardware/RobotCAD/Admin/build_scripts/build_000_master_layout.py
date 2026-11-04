"""Build RV-000_MasterLayout.SLDPRT: the top-down skeleton every RoamView part and assembly references.

Run:  python build_000_master_layout.py
Every sketch dimension and plane offset is driven by Admin/RV_globals.txt. The script fails if any
sketch ends up not fully defined.
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
from rv_sw import K, ROOT, Part, Sketch, app, cast, read_globals  # noqa: E402

OUT = os.path.join(ROOT, "Layout", "RV-000_MasterLayout.SLDPRT")
G = read_globals()
Q = lambda n: f'"{n}"'          # noqa: E731  global name -> equation token


def build_datums(p):
    pl = p.std_planes()
    d = {}
    for name, base, val, expr in [
        ("PL_Floor", "top", G["FLOOR_Z"], Q("FLOOR_Z")),
        ("PL_Axle", "top", G["AXLE_Z"], Q("AXLE_Z")),
        ("PL_CasterMount", "top", G["CASTER_H"], Q("CASTER_H")),
        ("PL_Deck", "top", G["DECK_Z"], Q("DECK_Z")),
        ("PL_Scan", "top", G["SCAN_Z"], Q("SCAN_Z")),
        ("PL_Lens", "top", G["LENS_Z"], Q("LENS_Z")),
        ("PL_WheelL", "right", G["TRACK"] / 2, f'{Q("TRACK")} / 2'),
        ("PL_WheelR", "right", -G["TRACK"] / 2, f'{Q("TRACK")} / 2'),
        ("PL_CasterF", "front", G["CASTER_X"], Q("CASTER_X")),
        ("PL_CasterR", "front", -G["CASTER_X"], Q("CASTER_X")),
        ("PL_Lidar", "front", G["LIDAR_X"], Q("LIDAR_X")),
        ("PL_Posts", "front", G["POST_X"], f'abs({Q("POST_X")})'),
        ("PL_MotorFaceL", "right", G["MOTOR_FACE_Y"], Q("MOTOR_FACE_Y")),
        ("PL_MotorFaceR", "right", -G["MOTOR_FACE_Y"], Q("MOTOR_FACE_Y")),
        ("PL_MotorAxis", "top", G["MOTOR_AXIS_Z"], Q("MOTOR_AXIS_Z")),
    ]:
        d[name] = p.offset_plane(name, base, val, expr)
    d["AX_Fwd"] = p.axis_from_planes("AX_Fwd", pl["top"], pl["right"])
    d["AX_Left"] = p.axis_from_planes("AX_Left", pl["top"], pl["front"])
    d["AX_Up"] = p.axis_from_planes("AX_Up", pl["front"], pl["right"])
    d["AX_Wheel"] = p.axis_from_planes("AX_Wheel", d["PL_Axle"], pl["front"])
    d["AX_Motor"] = p.axis_from_planes("AX_Motor", d["PL_MotorAxis"], pl["front"])
    d["AX_CasterF"] = p.axis_from_planes("AX_CasterF_Swivel", d["PL_CasterF"], pl["right"])
    d["AX_CasterR"] = p.axis_from_planes("AX_CasterR_Swivel", d["PL_CasterR"], pl["right"])
    d["AX_Lidar"] = p.axis_from_planes("AX_Lidar_Spin", d["PL_Lidar"], pl["right"])
    return pl, d


def ground(s, axis):
    """Construction ground line through the origin, centred on it, along a ROS axis."""
    a = dict(x=((-600, 0, 0), (600, 0, 0)), y=((0, -600, 0), (0, 600, 0)))[axis]
    g = s.line(*a, construction=True)
    s.straight(axis, g)
    s.rel("sgATMIDDLE", "origin", g)
    s.dlen(g, (0, 0, -40), value=1200)
    return g


def rect_dims(s, segs, a, b, ax1, ax2, len1, len2):
    """Size a rectangle with corners a, b: edge along ax1 = len1, edge along ax2 = len2."""
    i1, i2 = "xyz".index(ax1), "xyz".index(ax2)
    m1 = list(a)
    m1[i1] = (a[i1] + b[i1]) / 2
    at1 = list(m1)
    at1[i2] -= 20
    s.dlen(s.edge(segs, tuple(m1)), tuple(at1), expr=len1)
    m2 = list(a)
    m2[i2] = (a[i2] + b[i2]) / 2
    at2 = list(m2)
    at2[i1] -= 20
    s.dlen(s.edge(segs, tuple(m2)), tuple(at2), expr=len2)


def place(s, pt, ros, axes_exprs):
    """Locate a point from the origin along ROS axes: axes_exprs = {'x': expr, 'z': expr}."""
    for ax, expr in axes_exprs.items():
        at = list(ros)
        at["xyz".index(ax)] = at["xyz".index(ax)] / 2
        s.d_along(ax, "origin", pt, tuple(at), expr=expr)


# ------------------------------------------------------------------ side view (Right Plane, y = 0)
def sk_side_chassis(p, pl):
    L, F, D = G["BODY_L"], G["FLOOR_Z"], G["DECK_Z"]
    with Sketch(p, pl["right"], "SK_Side_Chassis") as s:
        g = ground(s, "x")
        a, b = (-L / 2, 0, F), (L / 2, 0, D)
        body = s.rect(a, b)
        rect_dims(s, body, a, b, "x", "z", Q("BODY_L"), f'{Q("DECK_Z")} - {Q("FLOOR_Z")}')
        place(s, s.corner(body, a), a, {"x": f'{Q("BODY_L")} / 2', "z": Q("FLOOR_Z")})
        ba, bb = (L / 2, 0, G["BUMPER_Z0"]), (L / 2 + G["BUMPER_T"], 0, G["BUMPER_Z1"])
        bump = s.rect(ba, bb)
        s.rel("sgCOLINEAR", s.edge(bump, (L / 2, 0, (ba[2] + bb[2]) / 2)), s.edge(body, (L / 2, 0, (F + D) / 2)))
        rect_dims(s, bump, ba, bb, "x", "z", Q("BUMPER_T"), f'{Q("BUMPER_Z1")} - {Q("BUMPER_Z0")}')
        s.d_along("z", "origin", s.corner(bump, ba), (ba[0] + 30, 0, ba[2] / 2), expr=Q("BUMPER_Z0"))
        cp = (G["CONTACT_X"], 0, G["CONTACT_Z"])
        place(s, s.point(cp), cp, {"x": Q("CONTACT_X"), "z": Q("CONTACT_Z")})
        ep = (G["ESTOP_X"], 0, G["ESTOP_TOP"])
        place(s, s.point(ep), ep, {"x": f'abs({Q("ESTOP_X")})', "z": Q("ESTOP_TOP")})


def sk_side_running(p, pl):
    with Sketch(p, pl["right"], "SK_Side_Running") as s:
        c = (0, 0, G["AXLE_Z"])
        wheel = s.circle(c, G["WHEEL_D"] / 2)
        s.align("z", "origin", s.center(wheel))
        s.d_along("z", "origin", s.center(wheel), (20, 0, G["AXLE_Z"] / 2), expr=Q("AXLE_Z"))
        s.ddia(wheel, (60, 0, 110), expr=Q("WHEEL_D"))
        for sx in (1, -1):
            cc = (sx * G["CASTER_X"], 0, G["CASTER_D"] / 2)
            cw = s.circle(cc, G["CASTER_D"] / 2)
            place(s, s.center(cw), cc, {"x": Q("CASTER_X"), "z": f'{Q("CASTER_D")} / 2'})
            s.ddia(cw, (cc[0] + 30, 0, 45), expr=Q("CASTER_D"))
            mp = (cc[0], 0, G["CASTER_H"])
            m = s.point(mp)
            s.align("z", s.center(cw), m)
            s.d_along("z", "origin", m, (cc[0] + 40, 0, mp[2] / 2), expr=Q("CASTER_H"))


def sk_side_tower(p, pl):
    D, PS, PX = G["DECK_Z"], G["POST_SIZE"], G["POST_X"]
    with Sketch(p, pl["right"], "SK_Side_Tower") as s:
        a, b = (PX - PS / 2, 0, D), (PX + PS / 2, 0, G["POST_TOP"])
        post = s.rect(a, b)
        rect_dims(s, post, a, b, "x", "z", Q("POST_SIZE"), f'{Q("POST_TOP")} - {Q("DECK_Z")}')
        fc = (PX + PS / 2, 0, D)
        place(s, s.corner(post, fc), fc, {"x": f'abs({Q("POST_X")} + {Q("POST_SIZE")} / 2)', "z": Q("DECK_Z")})
        # rear diagonal strut: deck -> rear face of the post at brace height
        st0, st1 = (G["STRUT_X0"], 0, D), (PX - PS / 2, 0, G["STRUT_Z"])
        strut = s.line(st0, st1)
        p0, p1 = s.ends(strut)
        place(s, p0, st0, {"x": f'abs({Q("STRUT_X0")})', "z": Q("DECK_Z")})
        s.rel("sgCOINCIDENT", p1, s.edge(post, (PX - PS / 2, 0, (D + G["POST_TOP"]) / 2)))
        s.d_along("z", "origin", p1, (PX - 60, 0, G["STRUT_Z"] / 2), expr=Q("STRUT_Z"))
        # head profile: screen slope driven by SCREEN_ANG
        x0, x1, z0, zt, xt = G["HEAD_X0"], G["HEAD_X1"], G["HEAD_Z0"], G["HEAD_TOP"], G["HEAD_TOP_X"]
        zr = zt - (xt - x0) * math.tan(math.radians(G["SCREEN_ANG"]))
        A, B, C, Dp, E = (x0, 0, z0), (x1, 0, z0), (x1, 0, zt), (xt, 0, zt), (x0, 0, zr)
        hb, hf, ht, hs, hr = s.polyline([A, B, C, Dp, E], closed=True)
        s.straight("x", hb)
        s.straight("z", hf)
        s.straight("x", ht)
        s.straight("z", hr)
        pa = s.ends(hb)[0]
        place(s, pa, A, {"x": f'abs({Q("HEAD_X0")})', "z": Q("HEAD_Z0")})
        s.d_along("x", "origin", s.ends(hb)[1], (x1 / 2, 0, z0 - 20), expr=Q("HEAD_X1"))
        s.d_along("z", "origin", s.ends(hf)[1], (x1 + 40, 0, zt / 2), expr=Q("HEAD_TOP"))
        s.d_along("x", "origin", s.ends(ht)[1], (xt / 2, 0, zt + 20), expr=f'abs({Q("HEAD_TOP_X")})')
        s.dang(ht, hs, (xt - 25, 0, zt - 8), G["SCREEN_ANG"], Q("SCREEN_ANG"))


def sk_side_sensors(p, pl):
    s_ = {}
    with Sketch(p, pl["right"], "SK_Side_Sensors") as s:
        g = ground(s, "x")
        lens = (G["LENS_X"], 0, G["LENS_Z"])
        lp = s.point(lens)
        place(s, lp, lens, {"x": Q("LENS_X"), "z": Q("LENS_Z")})
        s_["lens"] = lp
        s.check("lens")

        def ray(angle_deg, length, construction=True):
            a = math.radians(angle_deg)
            end = (lens[0] + length * math.cos(a), 0, lens[2] + length * math.sin(a))
            seg = s.line(lens, end, construction=construction)
            s.join(lp, s.ends(seg)[0])
            return seg

        href = ray(0, 300)
        s.straight("x", href)
        s.dlen(href, (lens[0] + 150, 0, lens[2] + 15), value=300)
        tilt = G["OAK_TILT"]
        axis = ray(-tilt, 700, construction=False)          # optical axis: a real line so a frame can use it
        s.dlen(axis, (lens[0] + 350, 0, lens[2] - 50), value=700)
        s.dang(href, axis, (lens[0] + 260, 0, lens[2] - 8), tilt, Q("OAK_TILT"))
        s.check("axis")
        s_["axis"] = axis
        hd, hr_ = G["OAK_VFOV_DEPTH"] / 2, G["OAK_VFOV_RGB"] / 2
        up = ray(-tilt + hd, 900)
        s.dlen(up, (lens[0] + 300, 0, lens[2] + 330), value=900)
        s.dang(axis, up, (lens[0] + 200, 0, lens[2] + 40), hd, f'{Q("OAK_VFOV_DEPTH")} / 2')
        s.check("up")
        lo_len = lens[2] / math.sin(math.radians(tilt + hd))
        lo = ray(-tilt - hd, lo_len)
        s.rel("sgCOINCIDENT", s.ends(lo)[1], g)
        s.dang(axis, lo, (lens[0] + 200, 0, lens[2] - 110), hd, f'{Q("OAK_VFOV_DEPTH")} / 2')
        s.check("lo")
        for sgn in (1, -1):
            r = ray(-tilt + sgn * hr_, 600)
            s.dlen(r, (lens[0] + 250, 0, lens[2] + sgn * 180), value=600)
            s.dang(axis, r, (lens[0] + 150, 0, lens[2] + sgn * 30), hr_, f'{Q("OAK_VFOV_RGB")} / 2')
            s.check("rgb")
        # LiDAR spin axis and scan plane
        la, lb = (G["LIDAR_X"], 0, G["DECK_Z"]), (G["LIDAR_X"], 0, G["SCAN_Z"])
        lax = s.line(la, lb, construction=True)
        s.straight("z", lax)
        place(s, s.ends(lax)[0], la, {"x": Q("LIDAR_X"), "z": Q("DECK_Z")})
        s.d_along("z", "origin", s.ends(lax)[1], (G["LIDAR_X"] + 40, 0, G["SCAN_Z"] / 2), expr=Q("SCAN_Z"))
        scan = s.line((G["LIDAR_X"] - 600, 0, G["SCAN_Z"]), (G["LIDAR_X"] + 600, 0, G["SCAN_Z"]), construction=True)
        s.straight("x", scan)
        s.rel("sgATMIDDLE", s.ends(lax)[1], scan)
        s.dlen(scan, (G["LIDAR_X"] + 300, 0, G["SCAN_Z"] + 15), value=1200)
        s.check("lidar")
        # cliff sensor beams, front and rear: sensor die at CLIFF_X / CLIFF_Z, CLIFF_ANG outward from vertical
        for sx in (1, -1):
            c = (sx * G["CLIFF_X"], 0, G["CLIFF_Z"])
            cp = s.point(c)
            place(s, cp, c, {"x": Q("CLIFF_X"), "z": Q("CLIFF_Z")})
            v = s.line(c, (c[0], 0, 0), construction=True)
            s.join(cp, s.ends(v)[0])
            s.straight("z", v)
            s.rel("sgCOINCIDENT", s.ends(v)[1], g)
            dx = G["CLIFF_Z"] * math.tan(math.radians(G["CLIFF_ANG"]))
            bm = s.line(c, (c[0] + sx * dx, 0, 0), construction=True)
            s.join(cp, s.ends(bm)[0])
            s.rel("sgCOINCIDENT", s.ends(bm)[1], g)
            s.dang(v, bm, (c[0] + sx * 4, 0, 12), G["CLIFF_ANG"], Q("CLIFF_ANG"))
            s.check("cliff")
        # dock AprilTag (reference only; the dock part carries the real tag)
        tx = G["LENS_X"] + G["DOCK_TAG_DIST"]
        t0, t1 = (tx, 0, G["DOCK_TAG_Z"] - G["DOCK_TAG_SIZE"] / 2), (tx, 0, G["DOCK_TAG_Z"] + G["DOCK_TAG_SIZE"] / 2)
        tag = s.line(t0, t1)
        s.straight("z", tag)
        s.dlen(tag, (tx + 20, 0, G["DOCK_TAG_Z"]), expr=Q("DOCK_TAG_SIZE"))
        s.d_along("x", lp, s.ends(tag)[0], (lens[0] + 120, 0, t0[2] - 15), expr=Q("DOCK_TAG_DIST"))
        s.d_along("z", "origin", s.ends(tag)[0], (tx + 50, 0, t0[2] / 2),
                  expr=f'{Q("DOCK_TAG_Z")} - {Q("DOCK_TAG_SIZE")} / 2')
        s.check("tag")
    s_["sketch"] = s.feat
    return s_


# ------------------------------------------------------------------ top view (Top Plane, z = 0)
def sk_top_footprint(p, pl):
    L, W = G["BODY_L"], G["BODY_W"]
    with Sketch(p, pl["top"], "SK_Top_Footprint") as s:
        a, b = (-L / 2, -W / 2, 0), (L / 2, W / 2, 0)
        body = s.rect(a, b)
        rect_dims(s, body, a, b, "x", "y", Q("BODY_L"), Q("BODY_W"))
        place(s, s.corner(body, b), b, {"x": f'{Q("BODY_L")} / 2', "y": f'{Q("BODY_W")} / 2'})
        for sy in (1, -1):
            yi, yo = sy * (G["TRACK"] / 2 - G["WHEEL_W"] / 2), sy * (G["TRACK"] / 2 + G["WHEEL_W"] / 2)
            a, b = (-G["WHEEL_D"] / 2, yi, 0), (G["WHEEL_D"] / 2, yo, 0)
            w = s.rect(a, b)
            rect_dims(s, w, a, b, "x", "y", Q("WHEEL_D"), Q("WHEEL_W"))
            ci = (G["WHEEL_D"] / 2, yi, 0)
            place(s, s.corner(w, ci), ci, {"x": f'{Q("WHEEL_D")} / 2', "y": f'{Q("TRACK")} / 2 - {Q("WHEEL_W")} / 2'})
            PX, PS = G["POST_X"], G["POST_SIZE"]
            yi = sy * (G["POST_DY"] / 2 - PS / 2)
            a, b = (PX - PS / 2, yi, 0), (PX + PS / 2, sy * (G["POST_DY"] / 2 + PS / 2), 0)
            pst = s.rect(a, b)
            rect_dims(s, pst, a, b, "x", "y", Q("POST_SIZE"), Q("POST_SIZE"))
            ci = (PX + PS / 2, yi, 0)
            place(s, s.corner(pst, ci), ci, {"x": f'abs({Q("POST_X")} + {Q("POST_SIZE")} / 2)',
                                              "y": f'{Q("POST_DY")} / 2 - {Q("POST_SIZE")} / 2'})
        for sx in (1, -1):
            c = (sx * G["CASTER_X"], 0, 0)
            cp = s.point(c)
            s.align("x", "origin", cp)
            s.d_along("x", "origin", cp, (c[0] / 2, 20, 0), expr=Q("CASTER_X"))
        lc = (G["LIDAR_X"], 0, 0)
        lid = s.circle(lc, 20)
        s.align("x", "origin", s.center(lid))
        s.d_along("x", "origin", s.center(lid), (lc[0] / 2, -20, 0), expr=Q("LIDAR_X"))
        s.ddia(lid, (lc[0] + 25, 25, 0), value=40)
        ep = (G["ESTOP_X"], G["ESTOP_Y"], 0)
        place(s, s.point(ep), ep, {"x": f'abs({Q("ESTOP_X")})', "y": f'abs({Q("ESTOP_Y")})'})
        for sx in (1, -1):
            for sy in (1, -1):
                c = (sx * G["CLIFF_X"], sy * G["CLIFF_Y"], 0)
                place(s, s.point(c), c, {"x": Q("CLIFF_X"), "y": Q("CLIFF_Y")})
        for sy in (1, -1):
            c = (G["CONTACT_X"], sy * G["CONTACT_Y"], 0)
            place(s, s.point(c), c, {"x": Q("CONTACT_X"), "y": Q("CONTACT_Y")})


def sk_top_zones(p, pl):
    with Sketch(p, pl["top"], "SK_Top_Zones") as s:
        zones = [
            ("BAT", (G["BAT_X"] - G["BAT_L"] / 2, -G["BAT_W"] / 2), (G["BAT_X"] + G["BAT_L"] / 2, G["BAT_W"] / 2),
             Q("BAT_L"), Q("BAT_W"), (G["BAT_X"] + G["BAT_L"] / 2, G["BAT_W"] / 2),
             {"x": f'{Q("BAT_X")} + {Q("BAT_L")} / 2', "y": f'{Q("BAT_W")} / 2'}),
            ("PCB", (G["PCB_X"] - G["PCB_L"] / 2, -G["PCB_W"] / 2), (G["PCB_X"] + G["PCB_L"] / 2, G["PCB_W"] / 2),
             Q("PCB_L"), Q("PCB_W"), (G["PCB_X"] - G["PCB_L"] / 2, G["PCB_W"] / 2),
             {"x": f'{Q("PCB_X")} - {Q("PCB_L")} / 2', "y": f'{Q("PCB_W")} / 2'}),
            ("DRV", (G["DRV_X"] - G["DRV_S"] / 2, G["DRV_Y"] - G["DRV_S"] / 2),
             (G["DRV_X"] + G["DRV_S"] / 2, G["DRV_Y"] + G["DRV_S"] / 2),
             Q("DRV_S"), Q("DRV_S"), (G["DRV_X"] - G["DRV_S"] / 2, G["DRV_Y"] - G["DRV_S"] / 2),
             {"x": f'{Q("DRV_X")} - {Q("DRV_S")} / 2', "y": f'{Q("DRV_Y")} - {Q("DRV_S")} / 2'}),
            ("CMP", (G["COMPUTE_X"] - G["COMPUTE_L"] / 2, -G["COMPUTE_W"] / 2),
             (G["COMPUTE_X"] + G["COMPUTE_L"] / 2, G["COMPUTE_W"] / 2),
             Q("COMPUTE_L"), Q("COMPUTE_W"), (G["COMPUTE_X"] + G["COMPUTE_L"] / 2, G["COMPUTE_W"] / 2),
             {"x": f'abs({Q("COMPUTE_X")} + {Q("COMPUTE_L")} / 2)', "y": f'{Q("COMPUTE_W")} / 2'}),
        ]
        for _, a2, b2, lx, ly, c2, ex in zones:
            a, b, c = (a2[0], a2[1], 0), (b2[0], b2[1], 0), (c2[0], c2[1], 0)
            r = s.rect(a, b)
            rect_dims(s, r, a, b, "x", "y", lx, ly)
            place(s, s.corner(r, c), c, ex)


# ------------------------------------------------------------------ front section (Front Plane, x = 0)
def sk_front_section(p, pl):
    W, F, D = G["BODY_W"], G["FLOOR_Z"], G["DECK_Z"]
    with Sketch(p, pl["front"], "SK_Front_Section") as s:
        g = ground(s, "y")
        a, b = (0, -W / 2, F), (0, W / 2, D)
        body = s.rect(a, b)
        rect_dims(s, body, a, b, "y", "z", Q("BODY_W"), f'{Q("DECK_Z")} - {Q("FLOOR_Z")}')
        c = (0, W / 2, F)
        place(s, s.corner(body, c), c, {"y": f'{Q("BODY_W")} / 2', "z": Q("FLOOR_Z")})
        for sy in (1, -1):
            yi, yo = sy * (G["TRACK"] / 2 - G["WHEEL_W"] / 2), sy * (G["TRACK"] / 2 + G["WHEEL_W"] / 2)
            a, b = (0, yi, 0), (0, yo, G["WHEEL_D"])
            w = s.rect(a, b)
            s.rel("sgCOLINEAR", s.edge(w, (0, (yi + yo) / 2, 0)), g)
            rect_dims(s, w, a, b, "y", "z", Q("WHEEL_W"), Q("WHEEL_D"))
            s.d_along("y", "origin", s.corner(w, a), (0, yi / 2, -25), expr=f'{Q("TRACK")} / 2 - {Q("WHEEL_W")} / 2')
            PS = G["POST_SIZE"]
            yi = sy * (G["POST_DY"] / 2 - PS / 2)
            a, b = (0, yi, D), (0, sy * (G["POST_DY"] / 2 + PS / 2), G["POST_TOP"])
            pst = s.rect(a, b)
            rect_dims(s, pst, a, b, "y", "z", Q("POST_SIZE"), f'{Q("POST_TOP")} - {Q("DECK_Z")}')
            place(s, s.corner(pst, a), a, {"y": f'{Q("POST_DY")} / 2 - {Q("POST_SIZE")} / 2', "z": Q("DECK_Z")})
        a, b = (0, -G["HEAD_W"] / 2, G["HEAD_Z0"]), (0, G["HEAD_W"] / 2, G["HEAD_TOP"])
        hd = s.rect(a, b)
        rect_dims(s, hd, a, b, "y", "z", Q("HEAD_W"), f'{Q("HEAD_TOP")} - {Q("HEAD_Z0")}')
        c = (0, G["HEAD_W"] / 2, G["HEAD_Z0"])
        place(s, s.corner(hd, c), c, {"y": f'{Q("HEAD_W")} / 2', "z": Q("HEAD_Z0")})


# ------------------------------------------------------------------ ROS frames
def frames(p, d, sens):
    pts = {
        "PT_BaseLink": p.point_from("PT_BaseLink", d["AX_Wheel"], p.std_planes()["right"]),
        "PT_WheelL": p.point_from("PT_WheelL", d["AX_Wheel"], d["PL_WheelL"]),
        "PT_WheelR": p.point_from("PT_WheelR", d["AX_Wheel"], d["PL_WheelR"]),
        "PT_CasterF": p.point_from("PT_CasterF", d["AX_CasterF"], d["PL_CasterMount"]),
        "PT_CasterR": p.point_from("PT_CasterR", d["AX_CasterR"], d["PL_CasterMount"]),
        "PT_Laser": p.point_from("PT_Laser", d["AX_Lidar"], d["PL_Scan"]),
    }
    X, Y = (1, 0, 0), (0, 1, 0)
    out = {}
    for name, origin in [("base_footprint", "origin"), ("base_link", pts["PT_BaseLink"]),
                         ("wheel_left_joint", pts["PT_WheelL"]), ("wheel_right_joint", pts["PT_WheelR"]),
                         ("caster_front_swivel", pts["PT_CasterF"]), ("caster_rear_swivel", pts["PT_CasterR"]),
                         ("laser", pts["PT_Laser"])]:
        out[name] = p.coordinate_system(name, origin, d["AX_Fwd"], d["AX_Left"], X, Y)[1]
    t = math.radians(G["OAK_TILT"])
    out["oak_link"] = p.coordinate_system("oak_link", sens["lens"], sens["axis"], d["AX_Left"],
                                          (math.cos(t), 0, -math.sin(t)), Y)[1]
    return out


# ------------------------------------------------------------------ module interface frames
F, B, L, R, U = (1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1)
Z0 = (0, None)                      # on the Top Plane


def IF_SPECS():
    """(name, x, y, (z, z_expr), x_dir, y_dir). x/y entries are (value, expr|None); every non-zero
    coordinate is driven by a global so the frames follow the layout."""
    g = G
    SKIN_OUT = g["RAIL_X"] + g["PROFILE"] / 2 + g["SKIN_T"]
    SKIN_OUT_E = f'{Q("RAIL_X")} + {Q("PROFILE")} / 2 + {Q("SKIN_T")}'
    return [
        ("IF_DriveL", (0, None), (g["MOTOR_FACE_Y"], Q("MOTOR_FACE_Y")), (g["MOTOR_AXIS_Z"], Q("MOTOR_AXIS_Z")), F, L),
        ("IF_DriveR", (0, None), (-g["MOTOR_FACE_Y"], Q("MOTOR_FACE_Y")), (g["MOTOR_AXIS_Z"], Q("MOTOR_AXIS_Z")), B, R),
        ("IF_CasterF", (g["CASTER_X"], Q("CASTER_X")), (0, None), (g["CASTER_H"], Q("CASTER_H")), F, L),
        ("IF_CasterR", (-g["CASTER_X"], Q("CASTER_X")), (0, None), (g["CASTER_H"], Q("CASTER_H")), B, R),
        ("IF_Drawer", (0, None), (0, None), (g["DRAWER_Z"], Q("DRAWER_Z")), F, L),
        ("IF_GuideL", (0, None), (0, None), (g["DRAWER_Z"], Q("DRAWER_Z")), F, L),
        ("IF_GuideR", (0, None), (0, None), (g["DRAWER_Z"], Q("DRAWER_Z")), B, R),
        ("IF_Tower", (g["POST_X"], f'abs({Q("POST_X")})'), (0, None), (g["DECK_Z"], Q("DECK_Z")), F, L),
        ("IF_LidarPedestal", (g["LIDAR_X"], Q("LIDAR_X")), (0, None), (g["DECK_Z"], Q("DECK_Z")), F, L),
        ("IF_EStop", (g["ESTOP_X"], f'abs({Q("ESTOP_X")})'), (g["ESTOP_Y"], f'abs({Q("ESTOP_Y")})'),
         (g["DECK_Z"], Q("DECK_Z")), F, L),
        ("IF_Deck", (0, None), (0, None), (g["DECK_Z"] - g["DECK_T"], f'{Q("DECK_Z")} - {Q("DECK_T")}'), F, L),
        ("IF_FloorPan", (0, None), (0, None), (g["FLOOR_Z"], Q("FLOOR_Z")), F, L),
        ("IF_Compute", (g["COMPUTE_X"], f'abs({Q("COMPUTE_X")})'), (0, None), (g["PAN_TOP"], Q("PAN_TOP")), F, L),
        ("IF_Power", (g["PCB_X"], Q("PCB_X")), (0, None), (g["PAN_TOP"], Q("PAN_TOP")), F, L),
        ("IF_MotorDriver", (g["DRV_X"], Q("DRV_X")), (g["DRV_Y"], Q("DRV_Y")), (g["PAN_TOP"], Q("PAN_TOP")), F, L),
        # cliff cradles clamp the end skin to the MidCross member's outer T-slot (frame on the skin's outer face)
        ("IF_Cliff_FL", (SKIN_OUT, SKIN_OUT_E), (g["CLIFF_Y"], Q("CLIFF_Y")), (g["MIDX_Z"], Q("MIDX_Z")), F, L),
        ("IF_Cliff_FR", (SKIN_OUT, SKIN_OUT_E), (-g["CLIFF_Y"], Q("CLIFF_Y")), (g["MIDX_Z"], Q("MIDX_Z")), F, L),
        ("IF_Cliff_RL", (-SKIN_OUT, SKIN_OUT_E), (g["CLIFF_Y"], Q("CLIFF_Y")), (g["MIDX_Z"], Q("MIDX_Z")), B, R),
        ("IF_Cliff_RR", (-SKIN_OUT, SKIN_OUT_E), (-g["CLIFF_Y"], Q("CLIFF_Y")), (g["MIDX_Z"], Q("MIDX_Z")), B, R),
        ("IF_Bumper", (g["BODY_L"] / 2, f'{Q("BODY_L")} / 2'), (0, None), (g["BUMPER_Z0"], Q("BUMPER_Z0")), F, L),
        ("IF_ChargePads", (g["CONTACT_X"], Q("CONTACT_X")), (0, None), (g["CONTACT_Z"], Q("CONTACT_Z")), F, L),
        ("IF_SkinL", (0, None), (g["SKIN_Y"], Q("SKIN_Y")), (g["PAN_TOP"], Q("PAN_TOP")), F, L),
        ("IF_SkinR", (0, None), (-g["SKIN_Y"], Q("SKIN_Y")), (g["PAN_TOP"], Q("PAN_TOP")), B, R),
        ("IF_SkinFront", (g["RAIL_X"] + g["PROFILE"] / 2, f'{Q("RAIL_X")} + {Q("PROFILE")} / 2'), (0, None),
         (g["PAN_TOP"], Q("PAN_TOP")), F, L),
        ("IF_SkinRear", (-(g["RAIL_X"] + g["PROFILE"] / 2), f'{Q("RAIL_X")} + {Q("PROFILE")} / 2'), (0, None),
         (g["PAN_TOP"], Q("PAN_TOP")), B, R),
    ]


def interface_frames(p, d):
    from cots import Cots
    planes = {}
    for name, (x, ex), (y, ey), (z, ez), xd, yd in IF_SPECS():
        key = ez or "0"
        if key not in planes:
            planes[key] = p.std_planes()["top"] if not ez else p.offset_plane(
                "PL_IF_" + "".join(c for c in ez if c.isalnum() or c == "_")[:40], "top", z, ez)
        with Sketch(p, planes[key], f"SK_{name}", anchor=(0, 0, z)) as s:
            pt = s.point((x, y, z))
            Cots.place(s, pt, (x, y, z), "z", {k: v for k, v in (("x", ex), ("y", ey)) if v})
        ax = {(1, 0, 0): d["AX_Fwd"], (-1, 0, 0): d["AX_Fwd"], (0, 1, 0): d["AX_Left"], (0, -1, 0): d["AX_Left"],
              (0, 0, 1): d["AX_Up"], (0, 0, -1): d["AX_Up"]}
        p.coordinate_system(name, pt, ax[xd], ax[yd], xd, yd)


def main():
    sw = app()
    p = Part.new(sw)
    try:
        build(sw, p)
    except Exception:
        if not os.environ.get("RV_KEEP"):
            sw.QuitDoc(p.doc.GetTitle())    # never leave a half-built untitled part behind
        raise


def build(sw, p):
    n = p.link_globals()
    print(f"linked {n} globals")
    pl, d = build_datums(p)
    print("datums ok")
    for fn in (sk_side_chassis, sk_side_running, sk_side_tower):
        fn(p, pl)
        print(" ", p.report[-1])
    sens = sk_side_sensors(p, pl)
    print(" ", p.report[-1])
    for fn in (sk_top_footprint, sk_top_zones, sk_front_section):
        fn(p, pl)
        print(" ", p.report[-1])
    fr = frames(p, d, sens)
    interface_frames(p, d)
    print(f"interface frames: {len(IF_SPECS())}")
    p.set_props(PartNo="RV-000", Description="RoamView master layout (skeleton)", Source="LAYOUT", Revision="A")
    p.rebuild()
    p.save(OUT)
    bad = [k for k, v in p.audit.items() if v != K.swFullyConstrained]
    print("frames:")
    for k, m in fr.items():
        print(f"  {k:22s} origin(SW m)={[round(v, 4) for v in m[9:12]]} x={[round(v, 3) for v in m[0:3]]}")
    print("FULLY DEFINED:", "ALL" if not bad else f"NO -> {bad}")
    print("saved", OUT)


if __name__ == "__main__":
    main()
