"""Declarative builder for simplified COTS / envelope parts.

Each part is modelled in its own local frame with the same axis rules as the robot
(x forward, y left, z up; SolidWorks X = y, Y = z, Z = x). The part origin is its mounting
interface, and every part carries an `IF_Mount` coordinate system there plus any other named
frames (shaft, lens, laser, ...). Sketches are fully defined: every shape is sized and located
from the origin, and the build fails otherwise.

Shapes are given in the sketch plane's 2D coordinates:
    plane axis 'z' at h -> (x, y)      plane axis 'x' at h -> (y, z)      plane axis 'y' at h -> (x, z)
"""
import math
import os

import pythoncom
import win32com.client as w32

from rv_sw import K, MM, ROOT, Part, Sketch, cast


def set_color(doc, rgb, transparency=0.0):
    """Part appearance. The array must go over COM as doubles, or SolidWorks scrambles it."""
    vals = [rgb[0], rgb[1], rgb[2], 0.6, 0.8, 0.3, 0.3, transparency, 0.0]
    doc.MaterialPropertyValues = w32.VARIANT(pythoncom.VT_ARRAY | pythoncom.VT_R8, vals)

PLANE_BASE = dict(x="front", y="right", z="top")
INPLANE = dict(z=("x", "y"), x=("y", "z"), y=("x", "z"))


def to3(axis, at, uv):
    a1, a2 = INPLANE[axis]
    p = {axis: at, a1: uv[0], a2: uv[1]}
    return (p["x"], p["y"], p["z"])


class Cots:
    def __init__(self, sw, part_no, name, folder, desc, source="COTS", color=(0.3, 0.3, 0.32),
                 link_globals=False, transparency=0.0, **props):
        self.sw, self.part_no, self.name = sw, part_no, name
        self.path = os.path.join(ROOT, folder, f"{part_no}_{name}.SLDPRT")
        self.p = Part.new(sw)
        if link_globals:
            self.p.link_globals()
        self.planes = {}
        self.axes = None
        self.props = dict(PartNo=part_no, Description=desc, Source=source, Revision="A", **props)
        self.color, self.transparency = color, transparency
        self.n = 0

    # ---------- datums ----------
    def plane(self, axis, at, expr=None):
        if abs(at) < 1e-9:
            return self.p.std_planes()[PLANE_BASE[axis]]
        key = (axis, round(at, 4))
        if key not in self.planes:
            nm = f"PL_{axis.upper()}{'m' if at < 0 else 'p'}{abs(at):g}".replace(".", "_")
            self.planes[key] = self.p.offset_plane(nm, PLANE_BASE[axis], at, expr or str(abs(at)))
        return self.planes[key]

    def dir_axes(self):
        if self.axes is None:
            pl = self.p.std_planes()
            self.axes = dict(
                x=self.p.axis_from_planes("AX_X", pl["top"], pl["right"]),
                y=self.p.axis_from_planes("AX_Y", pl["top"], pl["front"]),
                z=self.p.axis_from_planes("AX_Z", pl["front"], pl["right"]))
        return self.axes

    # ---------- sketch helpers ----------
    @staticmethod
    def place(s, pt, ros, axis, exprs=None):
        """Fully locate a sketch point from the origin; zero coordinates become alignments.
        exprs: optional {local axis: equation} to drive a position from the globals."""
        exprs = exprs or {}
        a1, a2 = INPLANE[axis]
        v1, v2 = ros["xyz".index(a1)], ros["xyz".index(a2)]
        z1, z2 = abs(v1) < 1e-9, abs(v2) < 1e-9
        if z1 and z2:
            s.rel("sgCOINCIDENT", "origin", pt)
            return
        for a, v, zero, other in ((a1, v1, z1, a2), (a2, v2, z2, a1)):
            if zero:
                s.align(other, "origin", pt)
            else:
                at = list(ros)
                at["xyz".index(a)] = v / 2
                at["xyz".index(other)] += 6
                if a in exprs:
                    s.d_along(a, "origin", pt, tuple(at), expr=exprs[a])
                else:
                    s.d_along(a, "origin", pt, tuple(at), value=abs(v))

    def _shape(self, s, axis, at, shp):
        kind = shp[0]
        if kind == "circle":
            c, d = shp[1], shp[2]
            e = shp[3] if len(shp) > 3 else {}
            a1, a2 = INPLANE[axis]
            c3 = to3(axis, at, c)
            seg = s.circle(c3, d / 2)
            self.place(s, s.center(seg), c3, axis, {k2: e[k1] for k1, k2 in (("pu", a1), ("pv", a2)) if k1 in e})
            s.ddia(seg, to3(axis, at, (c[0] + d / 2 + 3, c[1] + d / 2 + 3)), expr=e.get("d"),
                   value=None if "d" in e else d)
        elif kind == "path":
            pts3 = [to3(axis, at, uv) for uv in shp[1]]
            segs = s.polyline(pts3, closed=False)
            for k, sg in enumerate(segs):
                self.place(s, s.ends(sg)[0], pts3[k], axis)
            self.place(s, s.ends(segs[-1])[1], pts3[-1], axis)
        elif kind == "poly":
            pts3 = [to3(axis, at, uv) for uv in shp[1]]
            segs = s.polyline(pts3, closed=True)
            for sg, p3 in zip(segs, pts3):
                self.place(s, s.ends(sg)[0], p3, axis)
        elif kind in ("rect", "rectc"):
            e = shp[-1] if isinstance(shp[-1], dict) else {}
            a1, a2 = INPLANE[axis]
            if kind == "rectc":
                c, w, h = shp[1], shp[2], shp[3]
                u0, v0, u1, v1 = c[0] - w / 2, c[1] - h / 2, c[0] + w / 2, c[1] + h / 2
                pos = {}
                if "w" in e:
                    assert abs(c[0]) < 1e-9, "expression-driven rectc must be centred"
                    pos[a1] = f'({e["w"]}) / 2'
                if "h" in e:
                    assert abs(c[1]) < 1e-9, "expression-driven rectc must be centred"
                    pos[a2] = f'({e["h"]}) / 2'
            else:
                (u0, v0), (u1, v1) = shp[1], shp[2]
                pos = {k2: e[k1] for k1, k2 in (("pu", a1), ("pv", a2)) if k1 in e}
            a3, b3 = to3(axis, at, (u0, v0)), to3(axis, at, (u1, v1))
            segs = s.rect(a3, b3)
            e1 = s.edge(segs, to3(axis, at, ((u0 + u1) / 2, v0)))
            e2 = s.edge(segs, to3(axis, at, (u0, (v0 + v1) / 2)))
            s.dlen(e1, to3(axis, at, ((u0 + u1) / 2, v0 - 4)), expr=e.get("w"),
                   value=None if "w" in e else abs(u1 - u0))
            s.dlen(e2, to3(axis, at, (u0 - 4, (v0 + v1) / 2)), expr=e.get("h"),
                   value=None if "h" in e else abs(v1 - v0))
            self.place(s, s.corner(segs, a3), a3, axis, pos)
        elif kind == "point":
            _, c = shp
            c3 = to3(axis, at, c)
            pt = s.point(c3)
            self.place(s, pt, c3, axis)
            return pt
        else:
            raise ValueError(kind)

    def feature(self, name, axis, at, shapes, depth, sign, cut=False, depth_expr=None, merge=True, plane_expr=None):
        """Sketch `shapes` on the plane `axis` = at, then extrude (or cut) `depth` toward `sign`."""
        self.n += 1
        s = self.sketch(name, axis, at, shapes, plane_expr)
        return self.p.extrude(s.feat, f"{'CUT' if cut else 'EX'}_{name}", depth, depth_expr or str(depth), axis, sign,
                              at, cut=cut, merge=merge)

    def sketch(self, name, axis, at, shapes, plane_expr=None):
        with Sketch(self.p, self.plane(axis, at, plane_expr), f"SK_{name}", anchor=to3(axis, at, (0, 0))) as s:
            for shp in shapes:
                self._shape(s, axis, at, shp)
        return s

    def sheet(self, name, axis, at, shapes, thick, radius, depth=0, sign=1, want=None, depth_expr=None):
        """Sheet-metal base flange: closed shapes -> flat plate; an open 'path' -> bent profile extruded `depth`."""
        s = self.sketch(name, axis, at, shapes)
        return self.p.base_flange(s.feat, f"SM_{name}", thick, radius, depth, axis, sign, at, depth_expr, want)

    def weldment(self):
        self.p.fm.InsertWeldmentFeature()

    @staticmethod
    def bolt_circle(center, bc_dia, n, hole_dia, start_deg=0):
        r = bc_dia / 2
        return [("circle", (center[0] + r * math.cos(math.radians(start_deg + i * 360 / n)),
                            center[1] + r * math.sin(math.radians(start_deg + i * 360 / n))), hole_dia)
                for i in range(n)]

    # ---------- frames ----------
    UNIT = dict(x=(1, 0, 0), y=(0, 1, 0), z=(0, 0, 1))

    def frame(self, name, origin=(0, 0, 0), x="x", y="y", flip_x=False, flip_y=False, exprs=None):
        """ROS-style frame at a local point. x/y name the local axes the frame's x/y follow."""
        ax = self.dir_axes()
        if all(abs(c) < 1e-9 for c in origin):
            org = "origin"
        else:
            exprs = exprs or {}
            with Sketch(self.p, self.plane("z", origin[2], exprs.get("z")), f"SK_F_{name}",
                        anchor=(0, 0, origin[2])) as s:
                org = s.point(origin)
                self.place(s, org, origin, "z", {k: v for k, v in exprs.items() if k in ("x", "y")})
        wx = tuple(-c if flip_x else c for c in self.UNIT[x])
        wy = tuple(-c if flip_y else c for c in self.UNIT[y])
        self.p.coordinate_system(name, org, ax[x], ax[y], wx, wy)

    def frame_line(self, name, plane_axis, at, p0, p1, y="y", flip_y=False, origin="mid", x_axis=None):
        """Frame whose x axis runs along a sketch line p0 -> p1 (plane 2D coords) - for tilted faces.
        Origin at the line's midpoint (or 'start'). With x_axis (a local axis name) the frame's x follows
        that axis instead and its y runs along the line."""
        ax = self.dir_axes()
        a3, b3 = to3(plane_axis, at, p0), to3(plane_axis, at, p1)
        with Sketch(self.p, self.plane(plane_axis, at), f"SK_F_{name}", anchor=to3(plane_axis, at, (0, 0))) as s:
            ln = s.line(a3, b3, construction=True)
            st, en = s.ends(ln)
            self.place(s, st, a3, plane_axis)
            self.place(s, en, b3, plane_axis)
            if origin == "mid":
                m = tuple((u + v) / 2 for u, v in zip(a3, b3))
                pt = s.point(m)
                s.rel("sgATMIDDLE", pt, ln)
            else:
                pt = st
        d = [v - u for u, v in zip(a3, b3)]
        if x_axis:
            self.p.coordinate_system(name, pt, ax[x_axis], ln, self.UNIT[x_axis], tuple(d))
            return
        wy = tuple(-c if flip_y else c for c in self.UNIT[y])
        self.p.coordinate_system(name, pt, ln, ax[y], tuple(d), wy)

    # ---------- finish ----------
    def finish(self):
        doc = self.p.doc
        r, g, b = self.color
        set_color(doc, (r, g, b), self.transparency)
        self.p.set_props(**{k: v for k, v in self.props.items() if v is not None})
        self.p.save(self.path)
        bad = [k for k, v in self.p.audit.items() if v != K.swFullyConstrained]
        title = self.p.doc.GetTitle()
        self.sw.CloseDoc(title)
        return dict(part=f"{self.part_no}_{self.name}", sketches=len(self.p.audit), not_fully_defined=bad)
