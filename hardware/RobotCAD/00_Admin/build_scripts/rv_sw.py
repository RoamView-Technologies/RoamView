"""Thin SolidWorks API layer for the RoamView build scripts (pywin32, early binding).

Frame convention for every RoamView part and assembly:
    SW +X = robot LEFT   (ROS +y)
    SW +Y = UP           (ROS +z)
    SW +Z = FORWARD      (ROS +x)
so the SolidWorks Front view looks at the robot's front, and the ROS frame is a pure axis
permutation (no mirror). Globals in RV_globals.txt are written in ROS terms (x fwd, y left, z up).
All API lengths are metres; the documents are MMGS.
"""
import math
import os
import re
import pythoncom
import win32com.client as w32
from win32com.client import gencache

SW = gencache.EnsureModule("{83A33D31-27C5-11CE-BFD4-00400513BB57}", 0, 34, 0)
K = gencache.EnsureModule("{4687F359-55D0-4CD3-B6CF-2EB42C11F989}", 0, 34, 0).constants
NOTHING = w32.VARIANT(pythoncom.VT_DISPATCH, None)
MM = 1e-3

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
GLOBALS = os.path.join(ROOT, "00_Admin", "RV_globals.txt")


def ros_to_sw(x, y, z):
    """ROS (x fwd, y left, z up) in mm -> SolidWorks model metres."""
    return (y * MM, z * MM, x * MM)


def dot(a, b):
    return sum(i * j for i, j in zip(a, b))


def unit(v):
    n = math.sqrt(dot(v, v))
    return [i / n for i in v]


def cast(obj, iface):
    if obj is None:
        return None
    return getattr(SW, iface)(getattr(obj, "_oleobj_", obj))


def app():
    sw = cast(w32.Dispatch("SldWorks.Application"), "ISldWorks")
    sw.Visible = True
    sw.SetUserPreferenceToggle(K.swInputDimValOnCreate, False)
    return sw


def read_globals(path=GLOBALS):
    """Evaluate RV_globals.txt in Python too, so scripts can place geometry near its final spot.
    Order-independent, like SolidWorks' automatic solve order."""
    pending = {}
    for line in open(path, encoding="utf-8"):
        if "=" in line:
            name, expr = line.split("=", 1)
            pending[name.strip().strip('"')] = expr.strip()
    vals = {}
    while pending:
        progress = False
        for name, expr in list(pending.items()):
            refs = re.findall(r'"([^"]+)"', expr)
            if all(r in vals for r in refs):
                e = expr
                for r in sorted(refs, key=len, reverse=True):
                    e = e.replace(f'"{r}"', repr(vals[r]))
                vals[name] = float(eval(e, {"__builtins__": {}, "abs": abs}))
                del pending[name]
                progress = True
        if not progress:
            raise ValueError(f"unresolvable globals: {pending}")
    return vals


class Part:
    def __init__(self, sw, doc):
        self.sw = sw
        self.doc = cast(doc, "IModelDoc2")
        self.ext = self.doc.Extension
        self.sm = self.doc.SketchManager
        self.fm = self.doc.FeatureManager
        self.selmgr = cast(self.doc.SelectionManager, "ISelectionMgr")
        self.eq = cast(self.doc.GetEquationMgr(), "IEquationMgr")
        self.report = []           # (sketch name, status) for the fully-defined audit

    # ---------- document ----------
    @classmethod
    def new(cls, sw):
        tmpl = sw.GetUserPreferenceStringValue(K.swDefaultTemplatePart)
        doc = sw.NewDocument(tmpl, 0, 0, 0)
        p = cls(sw, doc)
        p.ext.SetUserPreferenceInteger(K.swUnitSystem, 0, K.swUnitSystem_MMGS)
        p.ext.SetUserPreferenceInteger(K.swUnitsLinearDecimalPlaces, 0, 2)
        p.ext.SetUserPreferenceInteger(K.swUnitsAngularDecimalPlaces, 0, 2)
        p.sm.AddToDB = True                       # no snapping / auto-relations while drawing
        p.sm.DisplayWhenAdded = False
        return p

    def link_globals(self, path=GLOBALS):
        self.eq.AutomaticSolveOrder = True
        self.eq.FilePath = path
        self.eq.LinkToFile = True
        self.eq.UpdateValuesFromExternalEquationFile()
        return self.eq.GetCount()

    def errors(self):
        return [(f.Name, f.GetTypeName2(), f.GetErrorCode2()) for f in self.features() if f.GetErrorCode2()[0] != 0]

    def save(self, path):
        """Save, then prove the file is clean: reopen it, rebuild, and require zero feature errors.
        (SolidWorks leaves an 'unknown' flag on the Equations folder in the session that created the
        equations; it clears on reopen. Any other error fails the save.)"""
        os.makedirs(os.path.dirname(path), exist_ok=True)
        for _ in range(3):
            self.eq.EvaluateAll()
            self.doc.ForceRebuild3(False)
            bad = self.errors()
            if not bad:
                break
        if [b for b in bad if b[1] != "EqnFolder"]:
            raise RuntimeError(f"feature errors before save: {bad}")
        err = self.doc.SaveAs3(path, 0, K.swSaveAsOptions_Silent)
        if err not in (0, K.swFileSaveWithRebuildError):
            raise RuntimeError(f"SaveAs3 returned {err} for {path}")
        self.sw.CloseDoc(self.doc.GetTitle())
        res = self.sw.OpenDoc6(path, K.swDocPART if path.upper().endswith("PRT") else K.swDocASSEMBLY,
                               K.swOpenDocOptions_Silent, "", 0, 0)
        report = self.report
        self.__init__(self.sw, self.sw.ActiveDoc)
        self.report = report
        self.doc.ForceRebuild3(False)
        # re-audit every sketch in the reopened file, not just what the build recorded
        self.audit = {f.Name: cast(f.GetSpecificFeature2(), "ISketch").GetConstrainedStatus()
                      for f in self.features() if f.GetTypeName2() == "ProfileFeature"}
        bad = self.errors()
        if bad:
            raise RuntimeError(f"{path} reopened with errors: {bad}")
        ok = self.doc.Save3(K.swSaveAsOptions_Silent, 0, 0)
        if not ok[0]:
            raise RuntimeError(f"final save failed {ok}")

    def rebuild(self):
        self.doc.ForceRebuild3(False)

    # ---------- features / datums ----------
    def features(self):
        f = cast(self.doc.FirstFeature(), "IFeature")
        while f is not None:
            yield f
            f = cast(f.GetNextFeature(), "IFeature")

    def std_planes(self):
        planes = [f for f in self.features() if f.GetTypeName2() == "RefPlane"][:3]
        return dict(front=planes[0], top=planes[1], right=planes[2])

    def feature(self, name):
        return next((f for f in self.features() if f.Name == name), None)

    def clear(self):
        self.doc.ClearSelection2(True)

    def select_feature(self, feat, append=False, mark=0):
        data = cast(self.selmgr.CreateSelectData(), "ISelectData")
        data.Mark = mark
        return feat.Select2(append, mark)

    # ROS axis carried by each standard plane's normal, and its SolidWorks model index
    NORMAL = dict(front=("x", 2), top=("z", 1), right=("y", 0))

    def offset_plane(self, name, base, offset_mm, expr):
        """Plane parallel to a standard plane at a SIGNED ROS offset, driven by a global.
        The flip is chosen by reading the plane back, never assumed."""
        planes = self.std_planes()
        axis_idx = self.NORMAL[base][1]
        for flip in (False, True):
            self.clear()
            self.select_feature(planes[base])
            opt = K.swRefPlaneReferenceConstraint_Distance
            if flip:
                opt |= K.swRefPlaneReferenceConstraint_OptionFlip
            rp = cast(self.fm.InsertRefPlane(opt, abs(offset_mm) * MM, 0, 0, 0, 0), "IRefPlane")
            if rp is None:
                raise RuntimeError(f"plane {name} failed")
            f = self.last_feature()
            got = cast(rp.Transform, "IMathTransform").ArrayData[9 + axis_idx] / MM
            if abs(got - offset_mm) < 1e-3:
                f.Name = name
                if '"' in str(expr):                 # only global-driven offsets need an equation
                    self.link(f"D1@{name}", expr)
                return f
            self.clear()
            f.Select2(False, 0)
            self.ext.DeleteSelection2(0)
        raise RuntimeError(f"plane {name}: could not place at {offset_mm}")

    def axis_from_planes(self, name, p1, p2):
        before = {f.Name for f in self.features()}
        self.clear()
        self.select_feature(p1, False)
        self.select_feature(p2, True)
        if not self.doc.InsertAxis2(True):
            raise RuntimeError(f"axis {name} failed")
        new = [f for f in self.features() if f.Name not in before and f.GetTypeName2() == "RefAxis"]
        f = new[0] if new else self.last_feature()
        f.Name = name
        return f

    def point_from(self, name, a, b):
        """Reference point at the intersection of two selectable features (axis/plane)."""
        self.clear()
        self.select_feature(a, False)
        self.select_feature(b, True)
        res = self.fm.InsertReferencePoint(K.swRefPointIntersection, 0, 0, 1)
        if not res:
            raise RuntimeError(f"point {name} failed")
        f = cast(res[0], "IFeature")
        f.Name = name
        return f

    def last_feature(self):
        """Newest feature, skipping the Flat-Pattern that sheet-metal parts always keep at the end."""
        for i in range(6):
            f = cast(self.doc.FeatureByPositionReverse(i), "IFeature")
            if f is not None and f.GetTypeName2() not in ("FlatPattern", "TemplateFlatPattern"):
                return f
        return cast(self.doc.FeatureByPositionReverse(0), "IFeature")

    def _mark_select(self, ent, append, mark):
        if isinstance(ent, str) and ent == "origin":
            return self.ext.SelectByID2("Point1@Origin", "EXTSKETCHPOINT", 0, 0, 0, append, mark, None, 0)
        if hasattr(ent, "GetTypeName2"):
            return ent.Select2(append, mark)
        data = cast(self.selmgr.CreateSelectData(), "ISelectData")
        data.Mark = mark
        return ent.Select4(append, data)

    def coordinate_system(self, name, origin, x_ent, y_ent, want_x, want_y):
        """ROS-style frame. want_x / want_y are the expected axis directions as ROS unit vectors;
        the flip flags are found by reading the frame back."""
        wx, wy = ros_to_sw(*want_x), ros_to_sw(*want_y)
        for fx, fy in ((False, False), (True, False), (False, True), (True, True)):
            self.clear()
            self._mark_select(origin, False, 1)
            self._mark_select(x_ent, True, 2)
            self._mark_select(y_ent, True, 4)
            f = cast(self.fm.InsertCoordinateSystem(fx, fy, False), "IFeature")
            if f is None:
                raise RuntimeError(f"coordinate system {name} failed")
            m = self.cs_transform(f.Name)       # check under the default name; rename only the keeper
            ax, ay = m[0:3], m[3:6]
            if dot(ax, unit(wx)) > 0.999 and dot(ay, unit(wy)) > 0.999:
                f.Name = name
                if f.Name != name:
                    raise RuntimeError(f"could not rename coordinate system to {name}")
                return f, m
            self.clear()
            f.Select2(False, 0)
            self.ext.DeleteSelection2(0)
        raise RuntimeError(f"coordinate system {name}: no flip combination matched")

    def cs_transform(self, name):
        t = self.ext.GetCoordinateSystemTransformByName(name)
        return list(cast(t, "IMathTransform").ArrayData)

    def feature_box(self, feat):
        """ROS-frame (mm) bounding box of a feature's faces: (min xyz, max xyz)."""
        lo, hi = [1e9] * 3, [-1e9] * 3
        for fc in feat.GetFaces() or []:
            b = cast(fc, "IFace2").GetBox()
            for corner in ((b[0], b[1], b[2]), (b[3], b[4], b[5])):
                ros = (corner[2] / MM, corner[0] / MM, corner[1] / MM)     # SW (X,Y,Z) -> ROS (x,y,z)
                lo = [min(a, c) for a, c in zip(lo, ros)]
                hi = [max(a, c) for a, c in zip(hi, ros)]
        return lo, hi

    def extrude(self, sketch_feat, name, depth_mm, expr, axis, sign, start_mm, cut=False, merge=True):
        """Blind extrude of a sketch along ROS `axis` toward `sign` (+1/-1) from the sketch plane at
        `start_mm`, driven by `expr`. Direction is checked by reading the feature's faces back."""
        i = "xyz".index(axis)
        for rev in (False, True):
            self.clear()
            sketch_feat.Select2(False, 0)
            if cut:
                f = self.fm.FeatureCut4(True, False, rev, K.swEndCondBlind, 0, depth_mm * MM, 0, False, False,
                                        False, False, 0, 0, False, False, False, False, False, True, True,
                                        True, True, False, 0, 0, False, False)
            else:
                f = self.fm.FeatureExtrusion3(True, False, rev, K.swEndCondBlind, 0, depth_mm * MM, 0, False,
                                              False, False, False, 0, 0, False, False, False, False, merge, True,
                                              True, 0, 0, False)
            f = cast(f, "IFeature")
            if f is None:                       # e.g. a cut aimed away from the material: try the other way
                continue
            lo, hi = self.feature_box(f)
            mid = (lo[i] + hi[i]) / 2
            # an extrude swallowed by existing material has no faces of its own: that is a wrong direction too
            if lo[0] < 1e8 and (mid - start_mm) * sign > 0:
                f.Name = name
                if '"' in str(expr):                 # only global-driven offsets need an equation
                    self.link(f"D1@{name}", expr)
                return f
            self.clear()
            f.Select2(False, 0)
            self.ext.DeleteSelection2(0)
        raise RuntimeError(f"extrude {name}: wrong direction both ways")

    def base_flange(self, sketch_feat, name, thick, radius, depth_mm, axis, sign, start_mm, expr=None,
                    want=None):
        """Sheet-metal base flange from the sketch. For an open profile it is extruded `depth_mm` along
        ROS `axis` toward `sign`. `want` = (axis, lo, hi): the sheet must span lo..hi along that axis, which
        fixes the thickening side (tried both ways, read back)."""
        for thick_dir in (False, True):
            for rev in (False, True):
                self.clear()
                sketch_feat.Select2(False, 0)
                f = self.fm.InsertSheetMetalBaseFlange2(thick * MM, thick_dir, radius * MM, (depth_mm or 0.01) * MM,
                                                        0, rev, K.swEndCondBlind, K.swEndCondBlind, 0, None, True, 0,
                                                        0.0005, 0.0005, 0.5, True, False, True, True)
                f = cast(f, "IFeature")
                if f is None:
                    continue
                lo, hi = self.feature_box(f)
                ok = lo[0] < 1e8
                if ok and depth_mm:
                    i = "xyz".index(axis)
                    ok = (((lo[i] + hi[i]) / 2) - start_mm) * sign > 0
                if ok and want:
                    j = "xyz".index(want[0])
                    ok = abs(lo[j] - want[1]) < 0.05 and abs(hi[j] - want[2]) < 0.05
                if ok:
                    f.Name = name
                    if expr and depth_mm:
                        self.link(f"D1@{name}", expr)
                    return f
                self.doc.EditUndo2(1)            # undo removes flange + sheet-metal folders, keeps the sketch
        raise RuntimeError(f"base flange {name}: no thickness/direction combination matched {want}")

    def shell(self, name, face_point_ros, thick):
        """Shell the solid, removing the face that contains face_point (ROS mm)."""
        self.clear()
        x, y, z = ros_to_sw(*face_point_ros)
        if not self.ext.SelectByID2("", "FACE", x, y, z, False, 0, None, 0):
            raise RuntimeError(f"shell {name}: no face at {face_point_ros}")
        self.doc.InsertFeatureShell(thick * MM, False)
        f = self.last_feature()
        if f.GetTypeName2() != "Shell":
            raise RuntimeError(f"shell {name} failed")
        f.Name = name
        return f

    def pattern1d(self, name, seeds, axis_feat, n, spacing, want_axis, want_sign):
        """Linear pattern of `seeds` along an axis feature; direction verified by reading the result back."""
        i = "xyz".index(want_axis)
        for flip in (False, True):
            self.clear()
            axis_feat.Select2(False, 1)
            for sd in seeds:
                sd.Select2(True, 4)
            f = cast(self.fm.FeatureLinearPattern5(n, spacing * MM, 1, 0, flip, False, "NULL", "NULL", False, False,
                                                   False, False, True, True, False, False, False, False, 0, 0,
                                                   False, False), "IFeature")
            if f is None:
                continue
            lo, hi = self.feature_box(f)
            s0 = [self.feature_box(sd) for sd in seeds]
            seed_mid = sum((a[0][i] + a[1][i]) / 2 for a in s0) / len(s0)
            if lo[0] < 1e8 and ((lo[i] + hi[i]) / 2 - seed_mid) * want_sign > 0:
                f.Name = name
                return f
            self.doc.EditUndo2(1)
        raise RuntimeError(f"pattern {name} failed")

    def set_props(self, **props):
        cpm = self.ext.CustomPropertyManager("")
        for k, v in props.items():
            cpm.Add3(k, K.swCustomInfoText, str(v), K.swCustomPropertyReplaceValue)

    # ---------- equations ----------
    def link(self, dim_full_name, expr):
        """Drive a dimension from a global: link('D1@SK_X', '"BODY_L"')."""
        idx = self.eq.Add2(-1, f'"{dim_full_name}" = {expr}', True)
        if idx < 0:
            raise RuntimeError(f"equation failed: {dim_full_name} = {expr} (status {self.eq.Status})")
        return idx


class Sketch:
    """Context manager: open a sketch on a plane feature and draw with ROS-frame points (mm).
    R(x, y, z) converts a ROS point to this sketch's 2D coordinates, so the same call works on any
    plane. On exit the sketch is renamed, its dimensions are linked to the globals, it is rebuilt
    and its constrained status is recorded for the fully-defined audit."""

    def __init__(self, part, plane_feat, name, anchor=(0, 0, 0)):
        """anchor: any ROS point on the sketch plane (used for direction tests)."""
        self.p, self.plane, self.name, self.anchor = part, plane_feat, name, anchor
        self.dims = []

    def __enter__(self):
        p = self.p
        p.clear()
        p.select_feature(self.plane)
        p.sm.InsertSketch(True)
        self.sk = cast(p.sm.ActiveSketch, "ISketch")
        self.xa = list(cast(self.sk.ModelToSketchTransform, "IMathTransform").ArrayData)
        return self

    def __exit__(self, et, ev, tb):
        p = self.p
        p.sm.InsertSketch(True)          # close the sketch either way
        if et is not None:
            return False
        feat = p.last_feature()
        feat.Name = self.name
        for dim, expr in self.dims:      # drive dimensions from the globals now the sketch has its final name
            if expr is not None:
                p.link(dim.FullName.rsplit("@", 1)[0], expr)
        p.rebuild()
        self.feat = feat
        status = cast(feat.GetSpecificFeature2(), "ISketch").GetConstrainedStatus()
        p.report.append((self.name, status))
        return False

    def check(self, label):
        """Debug aid: print the live constrained status after a block (only when RV_DEBUG is set)."""
        if os.environ.get("RV_DEBUG"):
            print(f"    [{self.name}] {label}: {self.sk.GetConstrainedStatus()}")

    # ----- coordinates -----
    def R(self, x, y, z):
        """ROS point (mm) -> this sketch's 2D coords (mm)."""
        v = ros_to_sw(x, y, z)
        r = self.xa                      # SolidWorks row-vector convention: p' = p.R + T
        q = [v[0] * r[c] + v[1] * r[3 + c] + v[2] * r[6 + c] + r[9 + c] for c in range(3)]
        if abs(q[2]) > 1e-6:
            raise ValueError(f"point {(x, y, z)} is {q[2] * 1000:.2f} mm off the plane of {self.name}")
        return q[0] / MM, q[1] / MM

    # ----- geometry from ROS points -----
    def line(self, a, b, construction=False):
        (x1, y1), (x2, y2) = self.R(*a), self.R(*b)
        seg = cast(self.p.sm.CreateLine(x1 * MM, y1 * MM, 0, x2 * MM, y2 * MM, 0), "ISketchSegment")
        if construction:
            seg.ConstructionGeometry = True
        return seg

    def circle(self, c, r, construction=False):
        x, y = self.R(*c)
        seg = cast(self.p.sm.CreateCircleByRadius(x * MM, y * MM, 0, r * MM), "ISketchSegment")
        if construction:
            seg.ConstructionGeometry = True
        return seg

    def point(self, c):
        x, y = self.R(*c)
        return cast(self.p.sm.CreatePoint(x * MM, y * MM, 0), "ISketchPoint")

    def rect(self, a, b):
        (x1, y1), (x2, y2) = self.R(*a), self.R(*b)
        segs = self.p.sm.CreateCornerRectangle(x1 * MM, y1 * MM, 0, x2 * MM, y2 * MM, 0)
        return [cast(s, "ISketchSegment") for s in segs if s is not None]

    def polyline(self, pts, closed=False):
        segs = [self.line(pts[i], pts[i + 1]) for i in range(len(pts) - 1)]
        if closed:
            segs.append(self.line(pts[-1], pts[0]))
        n = len(segs)
        for i in range(n if closed else n - 1):
            self.join(self.ends(segs[i])[1], self.ends(segs[(i + 1) % n])[0])
        return segs

    # ----- topology helpers -----
    @staticmethod
    def ends(seg):
        ln = cast(seg, "ISketchLine")
        return cast(ln.GetStartPoint2(), "ISketchPoint"), cast(ln.GetEndPoint2(), "ISketchPoint")

    @staticmethod
    def center(seg):
        return cast(cast(seg, "ISketchArc").GetCenterPoint2(), "ISketchPoint")

    @staticmethod
    def pid(pt):
        return tuple(pt.GetID())

    def join(self, a, b):
        """Merge two endpoints that sit on the same spot (AddToDB leaves them separate)."""
        if self.pid(a) != self.pid(b):
            self.rel("sgCOINCIDENT", a, b)

    def corner(self, segs, ros_pt):
        """The rectangle/polyline vertex nearest to a ROS point."""
        tx, ty = self.R(*ros_pt)
        best = None
        for sg in segs:
            for pt in self.ends(sg):
                d = (pt.X / MM - tx) ** 2 + (pt.Y / MM - ty) ** 2
                if best is None or d < best[0]:
                    best = (d, pt)
        return best[1]

    def edge(self, segs, ros_pt):
        """The segment whose midpoint is nearest to a ROS point."""
        tx, ty = self.R(*ros_pt)
        best = None
        for sg in segs:
            a, b = self.ends(sg)
            mx, my = (a.X + b.X) / 2 / MM, (a.Y + b.Y) / 2 / MM
            d = (mx - tx) ** 2 + (my - ty) ** 2
            if best is None or d < best[0]:
                best = (d, sg)
        return best[1]

    def axis_is_horizontal(self, axis):
        """Does a ROS axis ('x'|'y'|'z') run along this sketch's horizontal?"""
        v = dict(x=(100, 0, 0), y=(0, 100, 0), z=(0, 0, 100))[axis]
        o = self.R(*self.anchor)
        q = self.R(*[a + b for a, b in zip(self.anchor, v)])
        return abs(q[0] - o[0]) > abs(q[1] - o[1])

    def d_along(self, axis, a, b, at, expr=None, value=None):
        """Distance between two entities measured along a ROS axis."""
        if self.axis_is_horizontal(axis):
            return self.dh(a, b, at, expr, value)
        return self.dv(a, b, at, expr, value)

    def align(self, axis, a, b):
        """Make two points line up along a ROS axis (same coordinate on the other in-plane axis)."""
        kind = "sgHORIZONTALPOINTS2D" if self.axis_is_horizontal(axis) else "sgVERTICALPOINTS2D"
        self.rel(kind, a, b)

    def straight(self, axis, seg):
        """Make a line run along a ROS axis."""
        self.rel("sgHORIZONTAL2D" if self.axis_is_horizontal(axis) else "sgVERTICAL2D", seg)

    # ----- selection -----
    def sel(self, *ents):
        self.p.clear()
        for i, e in enumerate(ents):
            if isinstance(e, str) and e == "origin":
                ok = self.p.ext.SelectByID2("Point1@Origin", "EXTSKETCHPOINT", 0, 0, 0, i > 0, 0, None, 0)
            else:
                ok = e.Select4(i > 0, None)
            if not ok:
                raise RuntimeError(f"selection {i} failed in {self.name}")

    # ----- relations -----
    def rel(self, kind, *ents):
        self.sel(*ents)
        self.p.doc.SketchAddConstraints(kind)
        self.p.clear()

    # ----- dimensions (placement 'at' is a ROS point) -----
    def _dim(self, maker, at, value, expr):
        x, y = self.R(*at)
        dd = maker(x * MM, y * MM, 0)
        if dd is None:
            raise RuntimeError(f"dimension failed in {self.name} near {at}")
        dim = cast(cast(dd, "IDisplayDimension").GetDimension2(0), "IDimension")
        angular = dim.GetType() == K.swAngularDimension
        if value is not None and not angular:
            dim.SetSystemValue3(value * MM, K.swSetValue_InAllConfigurations, None)
        self.p.clear()
        self.dims.append((dim, expr))
        return dim

    def dh(self, a, b, at, expr=None, value=None):
        self.sel(a, b)
        return self._dim(self.p.doc.AddHorizontalDimension2, at, value, expr)

    def dv(self, a, b, at, expr=None, value=None):
        self.sel(a, b)
        return self._dim(self.p.doc.AddVerticalDimension2, at, value, expr)

    def dlen(self, seg, at, expr=None, value=None):
        self.sel(seg)
        return self._dim(self.p.doc.AddDimension2, at, value, expr)

    def ddia(self, seg, at, expr=None, value=None):
        self.sel(seg)
        return self._dim(self.p.doc.AddDimension2, at, value, expr)

    def dang(self, seg_a, seg_b, at, target_deg, expr):
        """Angle between two lines. If SolidWorks picked the supplementary angle, the
        equation is written as 180 - expr so the design intent still holds."""
        self.sel(seg_a, seg_b)
        dim = self._dim(self.p.doc.AddDimension2, at, None, None)
        got = math.degrees(dim.GetSystemValue3(K.swThisConfiguration, None)[0])
        e = expr if abs(got - target_deg) <= abs(180 - got - target_deg) else f"180 - ({expr})"
        self.dims[-1] = (dim, e)
        return dim
