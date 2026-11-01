"""
RoamView_Build_RevB.py  -  ENGR 490 Capstone, RoamView chassis Rev B (layout 2)

Builds the Rev B SOLIDWORKS model from RoamView_RevB_spec.json, which cad_spec_b.py exports from the
interconnect model (old-designs/Interconnect_RevB/model_b.py + routes_b.npy):

    RoamView_Baseplate_RevB.SLDPRT   lower deck RV-CH-001B (+ STEP, PNG)
    RoamView_TopDeck_RevB.SLDPRT     top deck   RV-CH-002B (+ STEP, PNG)
    Components/*_RevB.SLDPRT         one envelope per bought or printed part, at datasheet size
    RoamView_Harness_RevB.SLDPRT     all 45 cables as swept conductors, one body each
    RoamView_Chassis_RevB.SLDASM     the assembly (+ STEP, PNG views)
    RoamView_build_log_RevB.txt      every feature, check and interference result
    RoamView_build_report_RevB.json  the same checks in machine-readable form

Rev A files are never touched: everything is written to this folder, with a _RevB suffix on every file
name so Rev A and Rev B can be open in SOLIDWORKS at the same time.

Run it with SOLIDWORKS open or closed:
    python RoamView_Build_RevB.py
Needs Python 3 with pywin32 (Anaconda has it). To change the design, edit model_b.py, rerun its checks
(rules_b.py, build_b.py), rerun cad_spec_b.py, then rerun this script.

Frame (all parts and the assembly): +X forward, +Y up, +Z robot right, millimetres in the spec, metres in
the SOLIDWORKS API. Assembly origin = floor point under the drive-axle centre (ROS base_footprint).
Top Plane sketches: sketch x = +X, sketch y = -Z.  Front Plane sketches: sketch x = +X, sketch y = +Y.
"""
import json
import math
import os
import sys
import time
import traceback
import winreg

import pythoncom
import win32com.client as wc
from win32com.client import VARIANT, gencache

HERE = os.path.dirname(os.path.abspath(__file__))
SPEC = os.path.join(HERE, "RoamView_RevB_spec.json")
COMP = os.path.join(HERE, "Components")
LOG = os.path.join(HERE, "RoamView_build_log_RevB.txt")
REPORT = os.path.join(HERE, "RoamView_build_report_RevB.json")
BOX_TOL = 0.25          # mm, allowed difference between a built part's bounding box and the spec


# ---------------------------------------------------------------------------------------------- setup
def sw_folder():
    for root in (winreg.HKEY_LOCAL_MACHINE,):
        try:
            k = winreg.OpenKey(root, r"SOFTWARE\SolidWorks")
            names = [winreg.EnumKey(k, i) for i in range(winreg.QueryInfoKey(k)[0])]
            for n in sorted((n for n in names if n.upper().startswith("SOLIDWORKS 20")), reverse=True):
                try:
                    s = winreg.OpenKey(k, n + r"\Setup")
                    p = winreg.QueryValueEx(s, "SolidWorks Folder")[0]
                    if os.path.exists(os.path.join(p, "sldworks.tlb")):
                        return p
                except OSError:
                    pass
        except OSError:
            pass
    return r"C:\Program Files\SOLIDWORKS Corp\SOLIDWORKS"


SWDIR = sw_folder()
_la = pythoncom.LoadTypeLib(os.path.join(SWDIR, "sldworks.tlb")).GetLibAttr()
SW = gencache.EnsureModule(str(_la[0]), _la[1], _la[3], _la[4])
C = {}
_t = pythoncom.LoadTypeLib(os.path.join(SWDIR, "swconst.tlb"))
for _i in range(_t.GetTypeInfoCount()):
    _ti = _t.GetTypeInfo(_i)
    _ta = _ti.GetTypeAttr()
    if _ta.typekind == pythoncom.TKIND_ENUM:
        for _j in range(_ta.cVars):
            _vd = _ti.GetVarDesc(_j)
            C[_ti.GetNames(_vd.memid)[0]] = _vd.value


def cast(o, iface):
    if o is None:
        return None
    return getattr(SW, iface)(o._oleobj_ if hasattr(o, "_oleobj_") else o)


def dbl(v):
    return VARIANT(pythoncom.VT_ARRAY | pythoncom.VT_R8, [float(x) for x in v])


def m(v):
    return v / 1000.0


_logf = None
REP = dict(started=time.strftime("%Y-%m-%d %H:%M:%S"), plates=[], parts=[], harness=[], assembly={}, errors=[])


def log(s):
    line = time.strftime("%H:%M:%S") + "  " + s
    print(line, flush=True)
    if _logf:
        _logf.write(line + "\n")
        _logf.flush()


# ---------------------------------------------------------------------------------------------- one document
class Doc:
    def __init__(self, sw, kind, path):
        self.sw, self.path = sw, path
        tmpl = sw.GetUserPreferenceStringValue(C["swDefaultTemplatePart"] if kind == "part" else C["swDefaultTemplateAssembly"])
        self.model = cast(sw.NewDocument(tmpl, 0, 0, 0), "IModelDoc2")
        if self.model is None:
            raise RuntimeError("SOLIDWORKS could not create a new %s from %s" % (kind, tmpl))
        self.ext = self.model.Extension
        self.ext.SetUserPreferenceInteger(C["swUnitSystem"], 0, C["swUnitSystem_MMGS"])
        if kind == "part":
            self.part = cast(self.model, "IPartDoc")
            self.skm = self.model.SketchManager
            self.fm = self.model.FeatureManager
            self.planes = {}
        else:
            self.asy = cast(self.model, "IAssemblyDoc")
        self.nfeat = 0

    # --- selection and sketches
    def sel_plane(self, name):
        self.model.ClearSelection2(True)
        if not self.ext.SelectByID2(name, "PLANE", 0, 0, 0, False, 0, None, 0):
            raise RuntimeError("could not select " + name)

    def last(self):
        return cast(self.model.FeatureByPositionReverse(0), "IFeature")

    def plane(self, base, dist):
        """Reference plane parallel to base ('Top Plane' / 'Front Plane') at a signed distance in mm."""
        key = (base, round(dist, 4))
        if abs(dist) < 1e-9:
            return None
        if key not in self.planes:
            self.sel_plane(base)
            flag = C["swRefPlaneReferenceConstraint_Distance"]
            if dist < 0:
                flag += C["swRefPlaneReferenceConstraint_OptionFlip"]
            self.fm.InsertRefPlane(flag, m(abs(dist)), 0, 0.0, 0, 0.0)
            f = self.last()
            f.Name = "PL_%s_%s" % ("Y" if base.startswith("Top") else "Z", ("%.2f" % dist).replace("-", "m").replace(".", "p"))
            self.planes[key] = f
        return self.planes[key]

    def start(self, base="Top Plane", dist=0.0):
        pf = self.plane(base, dist)
        self.model.ClearSelection2(True)
        if pf is None:
            self.sel_plane(base)
        else:
            pf.Select2(False, 0)
        self.skm.InsertSketch(True)
        self.skm.AddToDB = True
        self.skm.DisplayWhenAdded = False

    def end(self, name):
        self.skm.AddToDB = False
        self.skm.DisplayWhenAdded = True
        self.skm.InsertSketch(True)
        f = self.last()
        f.Name = name
        return f

    def line(self, x1, y1, x2, y2):
        self.skm.CreateLine(m(x1), m(y1), 0.0, m(x2), m(y2), 0.0)

    def arc(self, xc, yc, x1, y1, x2, y2, d):
        self.skm.CreateArc(m(xc), m(yc), 0.0, m(x1), m(y1), 0.0, m(x2), m(y2), 0.0, d)

    def circle(self, x, y, dia):
        self.skm.CreateCircleByRadius(m(x), m(y), 0.0, m(dia / 2))

    def rect(self, x1, y1, x2, y2):
        self.line(x1, y1, x2, y1); self.line(x2, y1, x2, y2); self.line(x2, y2, x1, y2); self.line(x1, y2, x1, y1)

    def slot(self, x1, y1, x2, y2, w):
        L = math.hypot(x2 - x1, y2 - y1)
        if L < 1e-6:
            return self.circle(x1, y1, w)
        nx, ny, r = -(y2 - y1) / L, (x2 - x1) / L, w / 2
        self.line(x1 + r * nx, y1 + r * ny, x2 + r * nx, y2 + r * ny)
        self.arc(x2, y2, x2 + r * nx, y2 + r * ny, x2 - r * nx, y2 - r * ny, -1)
        self.line(x2 - r * nx, y2 - r * ny, x1 - r * nx, y1 - r * ny)
        self.arc(x1, y1, x1 - r * nx, y1 - r * ny, x1 + r * nx, y1 + r * ny, -1)

    # --- features
    def volume(self):
        try:
            r = self.ext.GetMassProperties2(1, 0, False)
            arr = r[0] if isinstance(r, tuple) else r
            return arr[3] * 1e9 if arr else -1.0
        except Exception:
            return -1.0

    def extrude(self, sk, name, depth, y0=0.0, color=None, flip_offset=None):
        """Boss from the sketch plane (or from an offset start plane) along the plane normal, depth mm."""
        self.model.ClearSelection2(True)
        sk.Select2(False, 0)
        t0 = C["swStartOffset"] if abs(y0) > 1e-9 else 0
        f = self.fm.FeatureExtrusion2(True, False, False, 0, 0, m(depth), 0.0, False, False, False, False, 0.0, 0.0,
                                      False, False, False, False, True, True, True, t0, m(abs(y0)), y0 < 0)
        if f is None:
            raise RuntimeError("extrusion %s failed" % name)
        f = cast(f, "IFeature")
        f.Name = name
        if color:
            self.color(f, color)
        self.nfeat += 1
        return f

    def cut_through(self, sk, name):
        v0 = self.volume()
        self.model.ClearSelection2(True)
        sk.Select2(False, 0)
        f = self.fm.FeatureCut4(False, False, False, 1, 1, 0.01, 0.01, False, False, False, False, 0.0, 0.0, False, False,
                                False, False, False, True, True, True, True, False, 0, 0.0, False, False)
        if f is None:
            raise RuntimeError("cut %s failed" % name)
        f = cast(f, "IFeature")
        f.Name = name
        v1 = self.volume()
        self.nfeat += 1
        return f, v0, v1

    def revolve(self, sk, name, color=None):
        self.model.ClearSelection2(True)
        sk.Select2(False, 0)
        f = self.fm.FeatureRevolve2(True, True, False, False, False, False, 0, 0, 2 * math.pi, 0.0, False, False, 0.0, 0.0,
                                    0, 0.0, 0.0, True, True, True)
        if f is None:
            raise RuntimeError("revolve %s failed" % name)
        f = cast(f, "IFeature")
        f.Name = name
        if color:
            self.color(f, color)
        self.nfeat += 1
        return f

    def color(self, f, rgb):
        f.SetMaterialPropertyValues2(dbl([rgb[0], rgb[1], rgb[2], 1.0, 1.0, 0.45, 0.35, 0.0, 0.0]), C["swAllConfiguration"], None)

    # --- part-level
    def props(self, d):
        cpm = self.ext.CustomPropertyManager("")
        for k, v in d.items():
            cpm.Add3(k, C["swCustomInfoText"], str(v), C["swCustomPropertyReplaceValue"])

    def box(self):
        b = self.part.GetPartBox(True)
        return [round(v * 1000, 3) for v in b] if b else None

    def save(self, suffix, copy=False):
        opts = C["swSaveAsOptions_Silent"] + (C["swSaveAsOptions_Copy"] if copy else 0)
        target = self.path + suffix
        ok, err, warn = self.ext.SaveAs(target, 0, opts, None, 0, 0)
        log("    save %s: %s" % (os.path.basename(target), "ok" if ok and err == 0 else "ERROR %s" % err) + (" (warning %s)" % warn if warn else ""))
        return bool(ok and err == 0)

    def views(self, names):
        try:
            self.model.ActiveView.EnableGraphicsUpdate = True
        except Exception:
            pass
        for suffix, view, vid in names:
            self.model.ShowNamedView2(view, vid)
            self.model.ViewZoomtofit2()
            self.save(suffix, copy=True)
        self.model.ShowNamedView2("*Isometric", 7)
        self.model.ViewZoomtofit2()

    def close(self):
        try:
            self.sw.CloseDoc(self.model.GetTitle())
        except Exception:
            pass


# ---------------------------------------------------------------------------------------------- geometry helpers
POST2020 = [10, -10, 10, -3.1, 4, -3.1, 4, 3.1, 10, 3.1, 10, 10, 3.1, 10, 3.1, 4, -3.1, 4, -3.1, 10,
            -10, 10, -10, 3.1, -4, 3.1, -4, -3.1, -10, -3.1, -10, -10, -3.1, -10, -3.1, -4, 3.1, -4, 3.1, -10]


def outline(d, o):
    """Deck outline on the Top Plane: rounded rectangle, with the wheel wells cut in for the lower deck."""
    hx, hy, R = o["L"] / 2, o["W"] / 2, o["R"]
    w = o.get("wells")
    if not w:
        d.line(hx, -hy + R, hx, hy - R); d.arc(hx - R, hy - R, hx, hy - R, hx - R, hy, 1)
        d.line(hx - R, hy, -hx + R, hy); d.arc(-hx + R, hy - R, -hx + R, hy, -hx, hy - R, 1)
        d.line(-hx, hy - R, -hx, -hy + R); d.arc(-hx + R, -hy + R, -hx, -hy + R, -hx + R, -hy, 1)
        d.line(-hx + R, -hy, hx - R, -hy); d.arc(hx - R, -hy + R, hx - R, -hy, hx, -hy + R, 1)
        return
    wx, wy, rw = w["half"], w["z"], w["r"]
    d.line(hx, -hy + R, hx, hy - R)
    d.arc(hx - R, hy - R, hx, hy - R, hx - R, hy, 1)
    d.line(hx - R, hy, wx, hy)
    d.line(wx, hy, wx, wy + rw)
    d.arc(wx - rw, wy + rw, wx, wy + rw, wx - rw, wy, -1)
    d.line(wx - rw, wy, -wx + rw, wy)
    d.arc(-wx + rw, wy + rw, -wx + rw, wy, -wx, wy + rw, -1)
    d.line(-wx, wy + rw, -wx, hy)
    d.line(-wx, hy, -hx + R, hy)
    d.arc(-hx + R, hy - R, -hx + R, hy, -hx, hy - R, 1)
    d.line(-hx, hy - R, -hx, -hy + R)
    d.arc(-hx + R, -hy + R, -hx, -hy + R, -hx + R, -hy, 1)
    d.line(-hx + R, -hy, -wx, -hy)
    d.line(-wx, -hy, -wx, -wy - rw)
    d.arc(-wx + rw, -wy - rw, -wx, -wy - rw, -wx + rw, -wy, -1)
    d.line(-wx + rw, -wy, wx - rw, -wy)
    d.arc(wx - rw, -wy - rw, wx - rw, -wy, wx, -wy - rw, -1)
    d.line(wx, -wy - rw, wx, -hy)
    d.line(wx, -hy, hx - R, -hy)
    d.arc(hx - R, -hy + R, hx - R, -hy, hx, -hy + R, 1)


def is_cyl(prof):
    """(r_in, r_out, a0, a1) when the lathe profile is a plain cylinder or tube, else None."""
    if len(prof) != 4:
        return None
    rs, as_ = sorted({p[0] for p in prof}), sorted({p[1] for p in prof})
    if len(rs) != 2 or len(as_) != 2:
        return None
    corners = {(r, a) for r in rs for a in as_}
    if {tuple(p) for p in prof} != corners:
        return None
    return rs[0], rs[1], as_[0], as_[1]


def add_prim(d, i, pr):
    nm = "P%02d_%s" % (i + 1, "".join(ch for ch in (pr.get("name") or pr["t"]) if ch.isalnum())[:24])
    col = pr.get("color")
    if pr["t"] == "box":
        (x0, y0, z0), (x1, y1, z1) = pr["min"], pr["max"]
        if min(x1 - x0, y1 - y0, z1 - z0) < 1e-6:
            log("    skipped flat box " + nm)
            return
        d.start("Top Plane")
        d.rect(x0, -z1, x1, -z0)
        sk = d.end("SK_" + nm)
        d.extrude(sk, nm, y1 - y0, y0, col)
    elif pr["t"] == "p2020":
        ox, oy, oz = pr["origin"]
        d.start("Top Plane")
        n = len(POST2020) // 2
        for k in range(n):
            j = (k + 1) % n
            d.line(ox + POST2020[2 * k], -oz + POST2020[2 * k + 1], ox + POST2020[2 * j], -oz + POST2020[2 * j + 1])
        d.circle(ox, -oz, 4.2)
        sk = d.end("SK_" + nm)
        d.extrude(sk, nm, pr["h"], oy, col)
    else:
        ox, oy, oz = pr["origin"]
        prof = pr["profile"]
        cyl = is_cyl(prof)
        if pr["axis"] == "y":
            if cyl:                                              # circle(s) on the Top Plane, offset start
                r0, r1, a0, a1 = cyl
                d.start("Top Plane")
                d.circle(ox, -oz, 2 * r1)
                if r0 > 1e-6:
                    d.circle(ox, -oz, 2 * r0)
                sk = d.end("SK_" + nm)
                d.extrude(sk, nm, a1 - a0, oy + a0, col)
            else:                                                # profile on a plane parallel to Front at z = oz
                d.start("Front Plane", oz)
                n = len(prof)
                for k in range(n):
                    p, q = prof[k], prof[(k + 1) % n]
                    if (p[0], p[1]) != (q[0], q[1]):
                        d.line(ox + p[0], oy + p[1], ox + q[0], oy + q[1])
                amin, amax = min(p[1] for p in prof), max(p[1] for p in prof)
                d.skm.CreateCenterLine(m(ox), m(oy + amin - 5), 0.0, m(ox), m(oy + amax + 5), 0.0)
                sk = d.end("SK_" + nm)
                d.revolve(sk, nm, col)
        else:
            if cyl:                                              # circle(s) on the Front Plane, offset start along +Z
                r0, r1, a0, a1 = cyl
                d.start("Front Plane")
                d.circle(ox, oy, 2 * r1)
                if r0 > 1e-6:
                    d.circle(ox, oy, 2 * r0)
                sk = d.end("SK_" + nm)
                d.extrude(sk, nm, a1 - a0, oz + a0, col)
            else:                                                # profile on a plane parallel to Top at y = oy
                d.start("Top Plane", oy)
                n = len(prof)
                for k in range(n):
                    p, q = prof[k], prof[(k + 1) % n]
                    if (p[0], p[1]) != (q[0], q[1]):
                        d.line(ox + p[0], -(oz + p[1]), ox + q[0], -(oz + q[1]))
                amin, amax = min(p[1] for p in prof), max(p[1] for p in prof)
                d.skm.CreateCenterLine(m(ox), m(-(oz + amin - 5)), 0.0, m(ox), m(-(oz + amax + 5)), 0.0)
                sk = d.end("SK_" + nm)
                d.revolve(sk, nm, col)


def box_ok(got, want):
    return got is not None and max(abs(a - b) for a, b in zip(got, want)) <= BOX_TOL


# ---------------------------------------------------------------------------------------------- builders
def build_plate(sw, pl):
    t0 = time.time()
    log("=== %s (%s) ===" % (pl["file"], pl["props"]["PartNo"]))
    d = Doc(sw, "part", os.path.join(HERE, pl["file"]))
    rec = dict(file=pl["file"], cuts=[], ok=True)
    try:
        d.start("Top Plane")
        outline(d, pl["outline"])
        sk = d.end("SK_Outline")
        f = d.extrude(sk, "Plate", pl["t"], 0.0, pl["color"])
        log("    plate: volume %.0f mm3" % d.volume())
        for g in pl["groups"]:
            d.start("Top Plane")
            for it in g["items"]:
                if it["k"] == "c":
                    d.circle(it["x"], -it["z"], it["d"])
                else:
                    d.slot(it["x1"], -it["z1"], it["x2"], -it["z2"], it["w"])
            sk = d.end("SK_" + g["name"])
            _, v0, v1 = d.cut_through(sk, "Cut_" + g["name"])
            ok = v1 < v0 - 1e-3
            rec["cuts"].append(dict(name=g["name"], n=len(g["items"]), removed=round(v0 - v1, 1), ok=ok))
            log("    Cut_%s (%d): volume %.0f -> %.0f mm3%s" % (g["name"], len(g["items"]), v0, v1, "" if ok else "  *** removed nothing"))
            rec["ok"] &= ok
        cfg = d.model.ConfigurationManager.ActiveConfiguration.Name
        d.part.SetMaterialPropertyName2(cfg, "SOLIDWORKS Materials", pl["material"])
        d.props(pl["props"])
        d.model.ForceRebuild3(False)
        vol = d.volume()
        b = d.box()
        want = pl["expect"]
        rec.update(volume=round(vol, 1), volume_expected=want["volume"], box=b,
                   volume_ok=abs(vol - want["volume"]) <= max(1.0, 1e-4 * want["volume"]), box_ok=box_ok(b, want["box"]))
        try:
            mp = d.ext.GetMassProperties2(1, 0, False)
            mp = mp[0] if isinstance(mp, tuple) else mp
            rec["mass_kg"] = round(mp[5], 3)
        except Exception:
            pass
        log("    volume %.1f mm3 (model %.1f) %s; box %s %s; mass %s kg" % (
            vol, want["volume"], "OK" if rec["volume_ok"] else "MISMATCH", b, "OK" if rec["box_ok"] else "MISMATCH", rec.get("mass_kg")))
        rec["ok"] &= rec["volume_ok"] and rec["box_ok"]
        d.save(".SLDPRT")
        d.save(".STEP", copy=True)
        d.views([("_iso.png", "*Isometric", 7), ("_top.png", "*Top", 5)])
    except Exception as e:
        rec["ok"] = False
        rec["error"] = repr(e)
        log("*** %s failed: %r" % (pl["file"], e))
        REP["errors"].append("%s: %r" % (pl["file"], e))
        traceback.print_exc()
    finally:
        d.close()
    rec["seconds"] = round(time.time() - t0, 1)
    REP["plates"].append(rec)


def build_part(sw, p):
    t0 = time.time()
    log("--- %s  [%s]" % (p["file"], ", ".join(p["ids"])))
    d = Doc(sw, "part", os.path.join(COMP, p["file"]))
    rec = dict(file=p["file"], ids=p["ids"], ok=True)
    try:
        for i, pr in enumerate(p["prims"]):
            add_prim(d, i, pr)
        d.props(p["props"])
        d.model.ForceRebuild3(False)
        b = d.box()
        rec.update(box=b, box_expected=p["expect"]["box"], box_ok=box_ok(b, p["expect"]["box"]), features=d.nfeat)
        if not rec["box_ok"]:
            log("    box %s vs spec %s  *** MISMATCH" % (b, p["expect"]["box"]))
        rec["ok"] = rec["box_ok"]
        d.save(".SLDPRT")
    except Exception as e:
        rec["ok"] = False
        rec["error"] = repr(e)
        log("*** %s failed: %r" % (p["file"], e))
        REP["errors"].append("%s: %r" % (p["file"], e))
        traceback.print_exc()
    finally:
        d.close()
    rec["seconds"] = round(time.time() - t0, 1)
    REP["parts"].append(rec)


def build_harness(sw, h):
    t0 = time.time()
    log("=== %s (%d conductors) ===" % (h["file"], len(h["conductors"])))
    d = Doc(sw, "part", os.path.join(HERE, h["file"]))
    good = 0
    try:
        for c in h["conductors"]:
            nm = c["cable"] + ("" if c["role"] == "sig" else c["role"])
            rec = dict(name=nm, od=c["od"], points=len(c["pts"]), ok=False)
            try:
                d.model.ClearSelection2(True)
                d.skm.Insert3DSketch(True)
                d.skm.AddToDB = True
                flat = []
                for p in c["pts"]:
                    flat += [m(p[0]), m(p[1]), m(p[2])]
                if len(c["pts"]) == 2:
                    a, b = c["pts"]
                    d.skm.CreateLine(m(a[0]), m(a[1]), m(a[2]), m(b[0]), m(b[1]), m(b[2]))
                else:
                    d.skm.CreateSpline2(dbl(flat), False)
                d.skm.AddToDB = False
                d.skm.Insert3DSketch(True)
                sk = d.last()
                sk.Name = "PATH_" + nm
                f = None
                for scale in (1.0, 0.8):                  # one retry thinner if the tube would self-intersect
                    d.model.ClearSelection2(True)
                    sk.Select2(False, 4)
                    f = d.fm.InsertProtrusionSwept4(False, False, 0, False, False, 0, 0, False, 0.0, 0.0, 0, 0, False, True,
                                                    True, 0.0, True, True, m(c["od"] * scale), 0)
                    if f is not None:
                        if scale != 1.0:
                            rec["drawn_od"] = round(c["od"] * scale, 2)
                            log("    %s drawn at %.1f mm instead of %.1f mm (tight bend)" % (nm, c["od"] * scale, c["od"]))
                        break
                if f is None:
                    raise RuntimeError("sweep returned nothing")
                f = cast(f, "IFeature")
                f.Name = nm
                d.color(f, c["color"])
                rec["ok"] = True
                good += 1
            except Exception as e:
                rec["error"] = repr(e)
                log("    *** %s: %r" % (nm, e))
            REP["harness"].append(rec)
        d.props(dict(Description="RoamView Rev B harness: %d conductors in 45 cables" % len(h["conductors"]),
                     Revision="B", Project="ENGR 490 RoamView",
                     Notes="Routed centre lines from build_b.py (3 mm clearance, lined pass-throughs). Cut lengths: see the harness schedule."))
        log("    swept %d of %d conductors" % (good, len(h["conductors"])))
        d.save(".SLDPRT")
    except Exception as e:
        log("*** harness failed: %r" % (e,))
        REP["errors"].append("harness: %r" % (e,))
        traceback.print_exc()
    finally:
        d.close()
    log("    harness in %.1f s" % (time.time() - t0))
    return good


def place(d, mu, path, org):
    comp = cast(d.asy.AddComponent5(path, 0, "", False, "", m(org[0]), m(org[1]), m(org[2])), "IComponent2")
    if comp is None:
        raise RuntimeError("AddComponent5 failed for " + path)
    comp.Transform2 = mu.CreateTransform(dbl([1, 0, 0, 0, 1, 0, 0, 0, 1, m(org[0]), m(org[1]), m(org[2]), 1, 0, 0, 0]))
    return comp


def interference(d):
    out = []
    try:
        idm = cast(d.asy.InterferenceDetectionManager, "IInterferenceDetectionMgr")
        idm.TreatCoincidenceAsInterference = False
        idm.TreatSubAssembliesAsComponents = True
        idm.IncludeMultibodyPartInterferences = False
        n = idm.GetInterferenceCount()
        arr = idm.GetInterferences() if n else []
        for it in arr or []:
            it = cast(it, "IInterference")
            names = [cast(c, "IComponent2").Name2 for c in it.Components]
            out.append(dict(components=names, volume_mm3=round(it.Volume * 1e9, 1)))
        idm.Done()
    except Exception as e:
        out.append(dict(error=repr(e)))
    return out


def open_hidden(sw, paths):
    """AddComponent5 only accepts documents already loaded in the session, so load every part first,
    without opening a window for each."""
    titles = []
    sw.DocumentVisible(False, C["swDocPART"])
    try:
        for p in paths:
            r = sw.OpenDoc6(p, C["swDocPART"], C["swOpenDocOptions_Silent"], "", 0, 0)
            md = r[0] if isinstance(r, tuple) else r
            if md is None:
                raise RuntimeError("could not load " + p + " (errors %s)" % (r[1:] if isinstance(r, tuple) else "?"))
            titles.append(cast(md, "IModelDoc2").GetTitle())
    finally:
        sw.DocumentVisible(True, C["swDocPART"])
    return titles


def build_assembly(sw, spec, harness_ok):
    t0 = time.time()
    a = spec["assembly"]
    log("=== %s (assembly) ===" % a["file"])
    paths = [os.path.join(HERE, pl["file"] + ".SLDPRT") for pl in spec["plates"]]
    paths += sorted({os.path.join(COMP, oc["file"] + ".SLDPRT") for oc in spec["occurrences"]})
    if harness_ok:
        paths.append(os.path.join(HERE, spec["harness"]["file"] + ".SLDPRT"))
    loaded = open_hidden(sw, paths)
    log("    loaded %d part files" % len(loaded))
    d = Doc(sw, "assembly", os.path.join(HERE, a["file"]))
    mu = cast(sw.GetMathUtility(), "IMathUtility")
    rec = {}
    try:
        comps = []
        for pl in spec["plates"]:
            comps.append(place(d, mu, os.path.join(HERE, pl["file"] + ".SLDPRT"), pl["origin"]))
        for oc in spec["occurrences"]:
            comps.append(place(d, mu, os.path.join(COMP, oc["file"] + ".SLDPRT"), oc["origin"]))
        d.model.ClearSelection2(True)
        for c in comps:
            c.Select4(True, None, False)
        d.asy.FixComponent()
        d.model.ClearSelection2(True)
        d.model.EditRebuild3()
        log("    placed and fixed %d components" % len(comps))
        itf = interference(d)
        rec["interference"] = itf
        log("    interference check (parts only): %d" % len([i for i in itf if "components" in i]))
        for i in itf:
            log("       %s" % (" <-> ".join(i["components"]) + "  %.1f mm3" % i["volume_mm3"] if "components" in i else i))
        if harness_ok:
            hc = place(d, mu, os.path.join(HERE, spec["harness"]["file"] + ".SLDPRT"), [0.0, 0.0, 0.0])
            d.model.ClearSelection2(True)
            hc.Select4(False, None, False)
            d.asy.FixComponent()
            d.model.ClearSelection2(True)
            d.model.EditRebuild3()
        rec["components"] = d.asy.GetComponentCount(True)
        log("    components in the assembly: %d" % rec["components"])
        rec["saved"] = d.save(".SLDASM")
        rec["step"] = d.save(".STEP", copy=True)
        d.views([("_iso.png", "*Isometric", 7), ("_trimetric.png", "*Trimetric", 8), ("_side.png", "*Front", 1),
                 ("_top.png", "*Top", 5), ("_front.png", "*Right", 4)])
    except Exception as e:
        rec["error"] = repr(e)
        log("*** assembly failed: %r" % (e,))
        REP["errors"].append("assembly: %r" % (e,))
        traceback.print_exc()
    finally:
        d.close()
        for t in loaded:
            try:
                sw.CloseDoc(t)
            except Exception:
                pass
    rec["seconds"] = round(time.time() - t0, 1)
    REP["assembly"] = rec


def close_ours(sw):
    """Close any document open from this folder (a previous run), never anything else."""
    docs = sw.GetDocuments() or []
    for dd in docs:
        md = cast(dd, "IModelDoc2")
        p = (md.GetPathName() or "").lower()
        if p.startswith(HERE.lower()):
            log("closing open copy: " + md.GetTitle())
            sw.CloseDoc(md.GetTitle())


def main(only=None):
    global _logf
    os.makedirs(COMP, exist_ok=True)
    _logf = open(LOG, "w", encoding="utf-8")
    spec = json.load(open(SPEC, encoding="utf-8"))
    t0 = time.time()
    sw = SW.ISldWorks(wc.Dispatch("SldWorks.Application")._oleobj_)
    sw.Visible = True
    log("RoamView Rev B build - SOLIDWORKS %s - spec %s (%s)" % (sw.RevisionNumber(), os.path.basename(SPEC), spec["meta"]["built"]))
    close_ours(sw)
    sw.CommandInProgress = True
    try:
        if only in (None, "plates"):
            for pl in spec["plates"]:
                build_plate(sw, pl)
        if only in (None, "parts"):
            for p in spec["parts"]:
                build_part(sw, p)
        hok = True
        if only in (None, "harness"):
            hok = build_harness(sw, spec["harness"]) > 0
        if only in (None, "assembly"):
            build_assembly(sw, spec, hok)
    finally:
        sw.CommandInProgress = False
    REP["seconds"] = round(time.time() - t0, 1)
    bad = [p["file"] for p in REP["plates"] + REP["parts"] if not p.get("ok")]
    hbad = [h["name"] for h in REP["harness"] if not h.get("ok")]
    REP["summary"] = dict(plates=len(REP["plates"]), parts=len(REP["parts"]), failed=bad,
                          conductors=len(REP["harness"]), conductors_failed=hbad)
    json.dump(REP, open(REPORT, "w"), indent=1)
    log("BUILD %s in %.0f s: %d plates, %d part files, %d conductors; failed parts %s; failed conductors %s" % (
        "COMPLETE" if not bad and not hbad and not REP["errors"] else "FINISHED WITH ISSUES", REP["seconds"],
        len(REP["plates"]), len(REP["parts"]), len(REP["harness"]), bad or "none", hbad or "none"))
    _logf.close()


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else None)
