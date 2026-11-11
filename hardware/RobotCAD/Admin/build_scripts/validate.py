"""Validation helpers (core SolidWorks API only - no Simulation add-in needed).

    python validate.py interference            # RV-100 at rest
    python validate.py sweeps                  # drawer stroke, caster swivel, spring travel
    python validate.py massprops               # mass, COM, inertia; drawer closed/open
Results are appended to Simulation/<area>/*.txt and printed.
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
from rv_sw import K, MM, ROOT, app, cast  # noqa: E402

TOP = os.path.join(ROOT, "Assemblies", "RV-100_RoamView.SLDASM")


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


def inertia_ros(doc):
    """Inertia tensor about the COM in kg.m2, ROS axes (x fwd = SW Z, y left = SW X, z up = SW Y)."""
    m = cast(doc.Extension.CreateMassProperty(), "IMassProperty")
    m.UseSystemUnits = True
    v = list(m.GetMomentOfInertia(0))       # 0 = about the centre of mass, SW axes, row-major 3x3
    sw_ = [[v[0], v[1], v[2]], [v[3], v[4], v[5]], [v[6], v[7], v[8]]]
    idx = (2, 0, 1)                          # ROS axis i comes from SW axis idx[i]
    return [[sw_[idx[i]][idx[j]] for j in range(3)] for i in range(3)]


def report(area, name, lines):
    d = os.path.join(ROOT, "Simulation", area)
    os.makedirs(d, exist_ok=True)
    with open(os.path.join(d, name), "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")
    print("\n".join(lines))


def cmd_interference(sw):
    """Two passes: RV-100 with the sensor tower hidden, then the tower (RV-140) on its own. One full pass with the
    vendor STEP bodies (Y-adapter, casters) crashes SolidWorks; the tower only touches the rest where its posts sit
    on the deck."""
    sw.CloseAllDocuments(True)
    doc = open_doc(sw, TOP)
    doc.ForceRebuild3(False)
    for c in cast(doc, "IAssemblyDoc").GetComponents(True) or []:
        c = cast(c, "IComponent2")
        if c.Name2.startswith("RV-140_SensorTower"):
            c.Visible = K.swComponentHidden
    res = [("RV-100 (tower hidden)", n, v) for n, v in interferences(doc)]
    sw.CloseAllDocuments(True)
    tdoc = open_doc(sw, os.path.join(ROOT, "Assemblies", "RV-140_SensorTower.SLDASM"))
    res += [("RV-140", n, v) for n, v in interferences(tdoc)]
    sw.CloseAllDocuments(True)
    lines = [f"Interference detection at rest (coincident faces ignored), RV-100 without the tower + RV-140: "
             f"{len(res)} interference(s)"]
    for where, names, vol in sorted(res, key=lambda r: -r[2]):
        lines.append(f"  {vol:10.1f} mm3   {where}: {' <-> '.join(names)}")
    report("Interference", "rest_interference.txt", lines)


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
    top = a.insert(r"Assemblies\RV-100_RoamView.SLDASM")
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
        "castF": (orig("RV-120_CasterFront_Sprung-1/RV-504"), r"COTS\Wheels_Casters\RV-504_Caster_goBILDA_3621.SLDPRT"),
        "carriage": (orig("RV-120_CasterFront_Sprung-1/RV-309"), r"3DPrinted\RV-309_FrontCasterCarriage.SLDPRT"),
        "castR": (orig("RV-125_CasterRear_Rigid-1/RV-504"), r"COTS\Wheels_Casters\RV-504_Caster_goBILDA_3621.SLDPRT"),
        "drawer": (orig("RV-130_BatteryDrawer-1"), r"Assemblies\RV-130_BatteryDrawer.SLDASM"),
    }
    # The tower sits above the deck, out of reach of every moving part checked here. Hiding it keeps the vendor
    # STEP bodies (Y-adapter) out of 28 full interference passes, which otherwise crash SolidWorks.
    orig("RV-140_SensorTower").Visible = K.swComponentHidden
    free, T0 = {}, {}
    for k, (oc, path) in targets.items():
        T0[k] = list(cast(oc.Transform2, "IMathTransform").ArrayData)
        oc.Visible = K.swComponentHidden
        free[k] = a.insert(path)
        free[k].Transform2 = _xf(sw, T0[k])
    a.doc.ForceRebuild3(False)
    lines = ["RV-100 collision sweeps (moving parts placed explicitly; originals and the above-deck tower hidden; "
             "touching faces ignored)"]

    def place(pairs):
        for k, arr in pairs:
            free[k].Transform2 = _xf(sw, arr)
        a.doc.EditRebuild3()

    def check(label, keys, ignore=()):
        """ignore: name fragments whose pairs are expected to overlap (a rigid spring model being compressed)."""
        mine = {free[k].Name2 for k in keys}
        res = [(n, v) for n, v in interferences(a.doc) if any(any(x.startswith(m) for m in mine) for x in n) and v > 0.5
               and not any(f in x for f in ignore for x in n)]
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
            # the springs are modelled rigid at their rest length, so with the carriage lifted they overlap it by
            # design; their compression (solid-height margin) is checked in hand_calcs.py instead
            check(f"front caster {ang:3d} deg, spring +{lift} mm", ["castF", "carriage"],
                  ignore=("RV-554",) if lift else ())
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
    drawer = os.path.join(ROOT, "Assemblies", "RV-130_BatteryDrawer.SLDASM")
    ddoc = open_doc(sw, drawer)
    md, _ = mass_props(ddoc)
    sw.CloseDoc(ddoc.GetTitle())
    sw.ActivateDoc3(doc.GetTitle(), False, 0, None)
    J = inertia_ros(doc)
    with open(os.path.join(ROOT, "Simulation", "MassProps", "mass_inertia.csv"), "w", encoding="utf-8") as f:
        rows = [("mass", f"{M:.4f}", "kg"), ("com_x", f"{cx:.2f}", "mm"), ("com_y", f"{cy:.2f}", "mm"),
                ("com_z", f"{cz:.2f}", "mm"), ("drawer_mass", f"{md:.4f}", "kg")]
        rows += [(f"I{a}{b}", f"{J[i][j]:.6f}", "kg.m2") for i, a in enumerate("xyz")
                 for j, b in enumerate("xyz") if j >= i]
        f.write("quantity,value,unit" + chr(10) + "".join(f"{q},{v},{u}" + chr(10) for q, v, u in rows))
    lines = [f"RV-100 mass properties (component mass overrides applied)",
             f"  total mass          {M:.2f} kg",
             f"  centre of mass      x {cx:7.1f}  y {cy:7.1f}  z {cz:7.1f} mm  (ROS, origin = floor under axle)",
             f"  drawer + battery    {md:.2f} kg",
             f"  inertia about COM   Ixx {J[0][0]:.4f}  Iyy {J[1][1]:.4f}  Izz {J[2][2]:.4f}  "
             f"Ixy {J[0][1]:.4f}  Ixz {J[0][2]:.4f}  Iyz {J[1][2]:.4f} kg.m2 (ROS)"]
    # support polygon: drive wheel contacts and caster contacts (trail turned inward = worst case)
    trail = G["CASTER_TRAIL"]
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
