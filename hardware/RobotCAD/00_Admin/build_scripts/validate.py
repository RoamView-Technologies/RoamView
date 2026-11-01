"""Validation helpers (core SolidWorks API only - no Simulation add-in needed).

    python validate.py interference            # RV-100 at rest
    python validate.py sweeps                  # drawer stroke, caster swivel, spring travel
    python validate.py massprops               # mass, COM, inertia; drawer closed/open
Results are appended to 08_Simulation/<area>/*.txt and printed.
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
from rv_sw import K, MM, ROOT, app, cast  # noqa: E402

TOP = os.path.join(ROOT, "02_Assemblies", "RV-100_RoamView.SLDASM")


def open_doc(sw, path):
    typ = K.swDocASSEMBLY if path.upper().endswith("ASM") else K.swDocPART
    res = sw.OpenDoc6(path, typ, K.swOpenDocOptions_Silent, "", 0, 0)
    doc = cast(res[0] if isinstance(res, tuple) else res, "IModelDoc2")
    sw.ActivateDoc3(doc.GetTitle(), False, 0, None)
    return doc


def interferences(doc, label=""):
    """All real (volume) interferences between components; coincident faces are not counted."""
    asm = cast(doc, "IAssemblyDoc")
    mgr = cast(asm.InterferenceDetectionManager, "IInterferenceDetectionMgr")
    mgr.TreatCoincidenceAsInterference = False
    mgr.TreatSubAssembliesAsComponents = False
    mgr.IncludeMultibodyPartInterferences = True
    mgr.IgnoreHiddenBodies = True
    mgr.ShowIgnoredInterferences = False
    mgr.MakeInterferingPartsTransparent = False
    mgr.CreateFastenersFolder = False
    out = []
    for it in mgr.GetInterferences() or []:
        it = cast(it, "IInterference")
        names = sorted(cast(c, "IComponent2").Name2 for c in (it.Components or []))
        out.append((names, it.Volume / MM ** 3))
    mgr.Done()
    return out


def mass_props(doc):
    m = cast(doc.Extension.CreateMassProperty(), "IMassProperty")
    m.UseSystemUnits = True
    c = m.CenterOfMass                      # SolidWorks X=left, Y=up, Z=fwd (metres)
    return m.Mass, (c[2] / MM, c[0] / MM, c[1] / MM)


def report(area, name, lines):
    d = os.path.join(ROOT, "08_Simulation", area)
    os.makedirs(d, exist_ok=True)
    with open(os.path.join(d, name), "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")
    print("\n".join(lines))


def cmd_interference(sw):
    doc = open_doc(sw, TOP)
    doc.ForceRebuild3(False)
    res = interferences(doc)
    lines = [f"RV-100 interference detection at rest (coincident faces ignored): {len(res)} interference(s)"]
    for names, vol in sorted(res, key=lambda r: -r[1]):
        lines.append(f"  {vol:10.1f} mm3   {' <-> '.join(names)}")
    report("Interference", "rest_interference.txt", lines)
    sw.CloseDoc(doc.GetTitle())


# ------------------------------------------------------------------ sweeps (hypothetical positions)
def _xf(sw, arr):
    import pythoncom
    import win32com.client as w32
    mu = cast(sw.GetMathUtility(), "IMathUtility")
    return cast(mu.CreateTransform(w32.VARIANT(pythoncom.VT_ARRAY | pythoncom.VT_R8, list(arr))), "IMathTransform")


def moved(T0, dx=0.0, dz_up=0.0, rot_deg=0.0, pivot_ros=(0, 0)):
    """New component transform: rotate about a vertical axis through pivot (ROS x, y), then translate.
    SolidWorks row-vector convention: p_world = p.R + T, with model X = ROS y, Y = up, Z = ROS x."""
    R = [T0[0:3], T0[3:6], T0[6:9]]
    T = list(T0[9:12])
    t = math.radians(rot_deg)
    Ry = [[math.cos(t), 0, -math.sin(t)], [0, 1, 0], [math.sin(t), 0, math.cos(t)]]
    c = [pivot_ros[1] * MM, 0.0, pivot_ros[0] * MM]
    R2 = [[sum(R[i][k] * Ry[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
    d = [T[i] - c[i] for i in range(3)]
    T2 = [sum(d[k] * Ry[k][j] for k in range(3)) + c[j] for j in range(3)]
    T2[2] += dx * MM
    T2[1] += dz_up * MM
    return [*R2[0], *R2[1], *R2[2], *T2, T0[12], 0, 0, 0]


def interferences_at(sw, doc, comps_xforms):
    import pythoncom
    import win32com.client as w32
    asm = cast(doc, "IAssemblyDoc")
    mgr = cast(asm.InterferenceDetectionManager, "IInterferenceDetectionMgr")
    mgr.TreatCoincidenceAsInterference = False
    mgr.TreatSubAssembliesAsComponents = False
    mgr.IncludeMultibodyPartInterferences = True
    mgr.UseTransform = True
    comps = [c for c, _ in comps_xforms]
    xfs = [_xf(sw, a) for _, a in comps_xforms]
    mgr.SetComponentsAndTransforms(w32.VARIANT(pythoncom.VT_ARRAY | pythoncom.VT_DISPATCH, [c._oleobj_ for c in comps]),
                                   w32.VARIANT(pythoncom.VT_ARRAY | pythoncom.VT_DISPATCH, [x._oleobj_ for x in xfs]))
    moved_names = {c.Name2 for c in comps}
    out = []
    for it in mgr.GetInterferences() or []:
        it = cast(it, "IInterference")
        names = sorted(cast(c, "IComponent2").Name2 for c in (it.Components or []))
        if any(n in moved_names for n in names):
            out.append((names, it.Volume / MM ** 3))
    mgr.Done()
    return out


def cmd_sweeps(sw):
    """Sweep test assembly: RV-100 fixed with its moving parts hidden; free copies of the moving parts are
    placed explicitly at each step and checked with ordinary interference detection."""
    from asm import Asm
    from rv_sw import read_globals
    G = read_globals()
    a = Asm.new(sw)
    top = a.insert(r"02_Assemblies\RV-100_RoamView.SLDASM")
    a.fix(top)
    a.doc.ForceRebuild3(False)
    nested = {cast(c, "IComponent2").Name2: cast(c, "IComponent2")
              for c in cast(a.doc, "IAssemblyDoc").GetComponents(False) or []}
    base = top.Name2 + "/"

    def orig(suffix):
        hit = [c for n, c in nested.items() if n.startswith(base) and n[len(base):].startswith(suffix)]
        if not hit:
            raise RuntimeError(f"no component {suffix}")
        return hit[0]

    targets = {
        "castF": (orig("RV-120_CasterFront_Sprung-1/RV-504"), r"05_COTS\Wheels_Casters\RV-504_Caster_2in_Swivel.SLDPRT"),
        "carriage": (orig("RV-120_CasterFront_Sprung-1/RV-309"), r"03_Custom_3DPrinted\RV-309_FrontCasterCarriage.SLDPRT"),
        "castR": (orig("RV-125_CasterRear_Rigid-1/RV-504"), r"05_COTS\Wheels_Casters\RV-504_Caster_2in_Swivel.SLDPRT"),
        "drawer": (orig("RV-130_BatteryDrawer-1"), r"02_Assemblies\RV-130_BatteryDrawer.SLDASM"),
    }
    free, T0 = {}, {}
    for k, (oc, path) in targets.items():
        T0[k] = list(cast(oc.Transform2, "IMathTransform").ArrayData)
        oc.Visible = K.swComponentHidden
        free[k] = a.insert(path)
        free[k].Transform2 = _xf(sw, T0[k])
    a.doc.ForceRebuild3(False)
    lines = ["RV-100 collision sweeps (moving parts placed explicitly; originals hidden; touching faces ignored)"]

    def place(pairs):
        for k, arr in pairs:
            free[k].Transform2 = _xf(sw, arr)
        a.doc.EditRebuild3()

    def check(label, keys):
        mine = {free[k].Name2 for k in keys}
        res = [(n, v) for n, v in interferences(a.doc) if any(any(x.startswith(m) for m in mine) for x in n) and v > 0.5]
        lines.append(f"  {label:44s} {'CLEAR' if not res else str(len(res)) + ' hit(s)'}" + ("" if not res else "  -> " +
                     "; ".join(f"{' <-> '.join(n)} {v:.0f} mm3" for n, v in sorted(res, key=lambda r: -r[1])[:3])))

    for d in (0, 100, 200, 333):
        place([("drawer", moved(T0["drawer"], dx=-d))])
        check(f"drawer pulled {d} mm", ["drawer"])
    place([("drawer", T0["drawer"])])
    for lift in (0, 10):
        for ang in range(0, 360, 45):
            place([("castF", moved(T0["castF"], dz_up=lift, rot_deg=ang, pivot_ros=(G["CASTER_X"], 0))),
                   ("carriage", moved(T0["carriage"], dz_up=lift))])
            check(f"front caster {ang:3d} deg, spring +{lift} mm", ["castF", "carriage"])
    place([("castF", T0["castF"]), ("carriage", T0["carriage"])])
    for ang in range(0, 360, 45):
        place([("castR", moved(T0["castR"], rot_deg=ang, pivot_ros=(-G["CASTER_X"], 0)))])
        check(f"rear caster {ang:3d} deg", ["castR"])
    report("Interference", "collision_sweeps.txt", lines)
    sw.QuitDoc(a.title)
    a.close_sources()


def cmd_massprops(sw):
    from rv_sw import read_globals
    G = read_globals()
    doc = open_doc(sw, TOP)
    doc.ForceRebuild3(False)
    M, (cx, cy, cz) = mass_props(doc)
    drawer = os.path.join(ROOT, "02_Assemblies", "RV-130_BatteryDrawer.SLDASM")
    ddoc = open_doc(sw, drawer)
    md, _ = mass_props(ddoc)
    sw.CloseDoc(ddoc.GetTitle())
    sw.ActivateDoc3(doc.GetTitle(), False, 0, None)
    lines = [f"RV-100 mass properties (component mass overrides applied)",
             f"  total mass          {M:.2f} kg",
             f"  centre of mass      x {cx:7.1f}  y {cy:7.1f}  z {cz:7.1f} mm  (ROS, origin = floor under axle)",
             f"  drawer + battery    {md:.2f} kg"]
    # support polygon: drive wheel contacts and caster contacts (trail turned inward = worst case)
    trail = 10.0
    F, B = (G["CASTER_X"] - trail, 0.0), (-(G["CASTER_X"] - trail), 0.0)
    Lw, Rw = (0.0, G["TRACK"] / 2), (0.0, -G["TRACK"] / 2)
    edges = {"front-left (F-L)": (F, Lw), "rear-left (L-B)": (Lw, B), "rear-right (B-R)": (B, Rw),
             "front-right (R-F)": (Rw, F)}

    def dist(p, a, b):
        (x0, y0), (x1, y1), (x2, y2) = p, a, b
        return abs((x2 - x1) * (y1 - y0) - (x1 - x0) * (y2 - y1)) / math.hypot(x2 - x1, y2 - y1)

    for case, dx in (("drawer closed", 0.0), ("drawer out 150 mm", -150.0), ("drawer out 250 mm", -250.0)):
        px = cx + md * dx / M
        lines.append(f"  [{case}] COM x = {px:.1f} mm")
        mins = []
        for nm, (a, b) in edges.items():
            d = dist((px, cy), a, b)
            ang = math.degrees(math.atan2(d, cz))
            mins.append(ang)
            lines.append(f"      tip about {nm:20s} margin {d:6.1f} mm  -> static tip angle {ang:5.1f} deg")
        lines.append(f"      minimum tip angle {min(mins):.1f} deg  ({'PASS' if min(mins) >= 20 else 'FAIL'} >= 20 deg)")
    W = M * 9.81
    lines.append(f"  static load split (front caster preload ignored): rear caster "
                 f"{max(0.0, -cx) / G['CASTER_X'] * W:.1f} N, each drive wheel "
                 f"{(W - max(0.0, -cx) / G['CASTER_X'] * W) / 2:.1f} N"
                 + ("" if cx <= 0 else f"  (COM is {cx:.1f} mm AHEAD of the axle: the sprung front caster must carry "
                                       f"{cx / G['CASTER_X'] * W:.1f} N)"))
    report("MassProps", "mass_com_tipover.txt", lines)
    sw.CloseDoc(doc.GetTitle())


if __name__ == "__main__":
    sw = app()
    {"interference": cmd_interference, "sweeps": cmd_sweeps, "massprops": cmd_massprops}[sys.argv[1]](sw)
