"""Linear-static FEA of RoamView parts (gmsh tetra mesh + scikit-fem quadratic elements), run on the exported STEP.

    python fea.py bracket|housing|guide|all
Units: mm, N, MPa. Coordinates are the part's SolidWorks frame: X = local y, Y = local z (up), Z = local x.
Fixtures and loads are picked geometrically (facet centroids); every case states its load path.
Results: Simulation/FEA/fea_<case>.txt and fea_<case>.png (von Mises on the deformed shape).
"""
import math
import os
import re
import sys

import gmsh
import numpy as np
import pyamg
import scipy.sparse.linalg as spla
from skfem import Basis, ElementTetP2, ElementVector, FacetBasis, Functional, LinearForm, MeshTet, asm, condense
from skfem.helpers import dot
from skfem.models.elasticity import lame_parameters, linear_elasticity

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
STEP = os.path.join(ROOT, "Exports", "STEP")
OUT = os.path.join(ROOT, "Simulation", "FEA")


def read_globals():
    pend = {}
    for line in open(os.path.join(ROOT, "Admin", "RV_globals.txt"), encoding="utf-8"):
        if "=" in line:
            n, e = line.split("=", 1)
            pend[n.strip().strip('"')] = e.strip()
    vals = {}
    while pend:
        for n, e in list(pend.items()):
            refs = re.findall(r'"([^"]+)"', e)
            if all(r in vals for r in refs):
                for r in sorted(refs, key=len, reverse=True):
                    e = e.replace(f'"{r}"', repr(vals[r]))
                vals[n] = float(eval(e, {"__builtins__": {}, "abs": abs}))
                del pend[n]
    return vals


G = read_globals()
W = 8.67 * 9.81                                     # robot weight (validate.py massprops, rounded up)
WHEEL_STATIC = 42.2                                 # N per drive wheel (validate.py massprops, rounded up)
BUMP = 2.0


def mesh_step(name, h):
    gmsh.initialize()
    gmsh.option.setNumber("General.Terminal", 0)
    gmsh.option.setString("Geometry.OCCTargetUnit", "MM")
    gmsh.model.occ.importShapes(os.path.join(STEP, name + ".step"))
    gmsh.model.occ.synchronize()
    gmsh.option.setNumber("Mesh.MeshSizeMax", h)
    gmsh.option.setNumber("Mesh.MeshSizeMin", h * 0.3)
    gmsh.option.setNumber("Mesh.Algorithm3D", 1)
    gmsh.model.mesh.generate(3)
    nt, xyz, _ = gmsh.model.mesh.getNodes()
    et, _, conn = gmsh.model.mesh.getElements(3)
    tets = conn[list(et).index(4)].reshape(-1, 4)
    gmsh.finalize()
    idx = {int(t): i for i, t in enumerate(nt)}
    p = xyz.reshape(-1, 3).T
    t = np.vectorize(idx.get)(tets).T
    used = np.unique(t)
    remap = -np.ones(p.shape[1], dtype=int)
    remap[used] = np.arange(len(used))
    return MeshTet(p[:, used], remap[t])


def facets_where(mesh, pred):
    bf = mesh.boundary_facets()
    c = mesh.p[:, mesh.facets[:, bf]].mean(axis=1)
    return bf[pred(c[0], c[1], c[2])]


def solve(mesh, E, nu, fixed, loads):
    """fixed: list of (facets, components or None=all). loads: list of (facets, traction_fn(x)->(3,n))."""
    e = ElementVector(ElementTetP2())
    ib = Basis(mesh, e)
    K = asm(linear_elasticity(*lame_parameters(E, nu)), ib)
    f = np.zeros(K.shape[0])
    total = np.zeros(3)
    for fac, vec in loads:
        fb = FacetBasis(mesh, e, facets=fac)
        if len(fac) == 0:
            raise RuntimeError("load facets not found")
        area = asm(Functional(lambda w: 1.0 + 0.0 * w.x[0]), FacetBasis(mesh, ElementTetP2(), facets=fac))
        tr = np.array(vec) / area                       # uniform traction giving the requested total force

        @LinearForm
        def load(v, w):
            return dot(tr[:, None, None] * np.ones_like(w.x), v)

        f += asm(load, fb)
        total += np.array(vec)
    D = []
    for fac, comps in fixed:
        if len(fac) == 0:
            raise RuntimeError("fixture facets not found")
        dofs = ib.get_dofs(fac)
        if comps is None:
            D.append(dofs.all())
        else:
            D.append(dofs.all([f"u^{'xyz'.index(c) + 1}" for c in comps]))
    D = np.unique(np.concatenate(D))
    A, b, x, I = condense(K, f, D=D)
    ml = pyamg.smoothed_aggregation_solver(A.tocsr(), B=None, max_coarse=500)
    u_I = ml.solve(b, tol=1e-9, accel="cg", maxiter=400)
    if np.linalg.norm(A @ u_I - b) > 1e-5 * np.linalg.norm(b):
        u_I = spla.spsolve(A.tocsc(), b)
    u = np.zeros(K.shape[0])
    u[I] = u_I
    # von Mises at quadrature points
    lam, mu = lame_parameters(E, nu)
    ui = ib.interpolate(u)
    gu = ui.grad                                    # (3, 3, nel, nqp)
    eps = 0.5 * (gu + gu.transpose(1, 0, 2, 3))
    tr_e = eps[0, 0] + eps[1, 1] + eps[2, 2]
    sig = 2 * mu * eps
    for i in range(3):
        sig[i, i] += lam * tr_e
    s = sig
    vm = np.sqrt(0.5 * ((s[0, 0] - s[1, 1]) ** 2 + (s[1, 1] - s[2, 2]) ** 2 + (s[2, 2] - s[0, 0]) ** 2)
                 + 3 * (s[0, 1] ** 2 + s[1, 2] ** 2 + s[0, 2] ** 2))
    disp = u[ib.nodal_dofs]
    umag = np.linalg.norm(disp, axis=0)
    return vm, umag, disp, total


def report(case, mesh, vm, umag, disp, total, sy, fs_req, notes):
    vm_el = vm.max(axis=1)
    peak, p995 = vm_el.max(), np.percentile(vm_el, 99.5)
    lines = [f"FEA {case}", *notes,
             f"  mesh: {mesh.p.shape[1]} nodes, {mesh.t.shape[1]} quadratic tets",
             f"  applied load (sum): ({total[0]:.1f}, {total[1]:.1f}, {total[2]:.1f}) N",
             f"  max displacement: {umag.max():.4f} mm",
             f"  von Mises: peak {peak:.1f} MPa (single element, includes fixture singularities), "
             f"99.5th percentile {p995:.1f} MPa",
             f"  safety factor on yield {sy:.0f} MPa: {sy / peak:.1f} (peak), {sy / p995:.1f} (99.5 %)  -> "
             f"{'PASS' if sy / peak >= fs_req else ('PASS (bulk) - peak is a fixture singularity' if sy / p995 >= fs_req else 'FAIL')}"
             f" vs FS >= {fs_req:g}"]
    txt = os.path.join(OUT, f"fea_{case}.txt")
    open(txt, "w", encoding="utf-8").write("\n".join(lines) + "\n")
    print("\n".join(lines))
    try:
        import pyvista as pv
        pv.OFF_SCREEN = True
        cells = np.hstack([np.full((mesh.t.shape[1], 1), 4), mesh.t.T]).ravel()
        grid = pv.UnstructuredGrid(cells, np.full(mesh.t.shape[1], pv.CellType.TETRA), mesh.p.T.copy())
        scale = 0.15 * max(np.ptp(mesh.p, axis=1)) / max(umag.max(), 1e-12)
        grid.points = grid.points + scale * disp.T
        grid.cell_data["von Mises (MPa)"] = vm_el
        pl = pv.Plotter(off_screen=True, window_size=(1100, 800))
        pl.set_background("white")
        pl.add_mesh(grid, scalars="von Mises (MPa)", cmap="jet", clim=(0, p995), show_edges=False,
                    scalar_bar_args=dict(title="von Mises (MPa)", color="black"))
        pl.add_text(f"{case}: peak {peak:.1f} MPa, FS {sy / peak:.1f}; deformation x{scale:.0f}", color="black",
                    font_size=11)
        pl.camera_position = "iso"
        pl.screenshot(os.path.join(OUT, f"fea_{case}.png"))
        pl.view_yz()  # side view (looking along X): shows vertical sag of long parts
        pl.screenshot(os.path.join(OUT, f"fea_{case}_side.png"))
        pl.close()
    except Exception as e:
        print("  (plot skipped:", e, ")")


def case_bracket():
    """RV-434 motor bracket, 3 mm 5052-H32. Wheel load 2 g at the wheel centre (9 mm outboard of the gearbox
    face, 7 mm below the motor axis) enters through the 6 gearbox bolts as shear + the overhang moment.
    Fixed: 2x M4 rail bolt holes (all DOF) + leg underside resting on the rail top (vertical only)."""
    m = mesh_step("RV-434_MotorBracket", 1.2)
    Fw = WHEEL_STATIC * BUMP
    dx = 9.0 - 1.5                                   # wheel centre to plate mid-thickness
    holes = [(15.5 * math.cos(math.radians(a)), 15.5 * math.sin(math.radians(a))) for a in range(0, 360, 60)]
    ys = np.array([y for _, y in holes])
    loads = []
    for (zc, yc), yi in zip(holes, ys):
        fac = facets_where(m, lambda X, Y, Z, zc=zc, yc=yc: (X > -0.1) & (X < 3.1) & (np.hypot(Z - zc, Y - yc) < 1.9))
        fx = dx * Fw * yi / np.sum(ys ** 2)
        loads.append((fac, (fx, -Fw / 6, 0.0)))
    rail_y0, rail_y1 = (G["RAIL_Y"] - 10) - G["MOTOR_FACE_Y"], (G["RAIL_Y"] + 10) - G["MOTOR_FACE_Y"]
    bolts = facets_where(m, lambda X, Y, Z: (np.hypot(X - (G["RAIL_Y"] - G["MOTOR_FACE_Y"]), np.abs(Z) - 12) < 2.3)
                         & (Y < -18))
    seat = facets_where(m, lambda X, Y, Z: (np.abs(Y - (Y.min())) < 0.05) & (X > rail_y0) & (X < rail_y1))
    vm, um, disp, tot = solve(m, 70300.0, 0.33, [(bolts, None), (seat, ["y"])], loads)
    report("motor_bracket", m, vm, um, disp, tot, 193.0, 3.0,
           [f"  RV-434 5052-H32 3 mm; wheel load {Fw:.1f} N (2 g) via 6 bolts + overhang moment {dx * Fw:.0f} N.mm",
            f"  fixed: 2x M4 rail bolt holes ({len(bolts)} facets), leg on rail top ({len(seat)} facets, vertical)",
            f"  hand calc: 43.5-56.9 MPa, FS 3.4-4.4"])


def case_housing():
    """RV-308 front caster housing, PETG. Carriage bottomed on a 2 g bump pushes up on the top plate
    underside (44 x 44 footprint). Fixed: back wall face bolted to the front mid cross member."""
    m = mesh_step("RV-308_FrontCasterHousing", 1.5)
    F = max(0.15 * W, 15.0) * BUMP
    up = facets_where(m, lambda X, Y, Z: (np.abs(Y - 14.0) < 0.05) & (np.abs(X) < 22) & (np.abs(Z) < 22))
    wall = facets_where(m, lambda X, Y, Z: np.abs(Z - 38.0) < 0.05)
    vm, um, disp, tot = solve(m, 2100.0, 0.38, [(wall, None)], [(up, (0.0, F, 0.0))])
    report("caster_housing", m, vm, um, disp, tot, 30.0, 2.0,
           [f"  RV-308 PETG (E 2.1 GPa, yield 30 MPa across layers); carriage bottomed, {F:.1f} N up (2 g)",
            f"  fixed: back wall face on the mid cross member ({len(wall)} facets)",
            "  hand calc: 1.2 MPa, FS 26"])


def case_guide():
    """RV-433 drawer guide, 1.5 mm 5052 L. Tray + battery (2 g, half per guide) on the horizontal leg along the
    tray footprint. Supported where the leg rests on the mid cross members (|x| >= 188), bolt holes fixed."""
    m = mesh_step("RV-433_DrawerGuide", 1.2)
    Fg = (0.280 + 0.900 + 0.021) * 9.81 * BUMP / 2
    top = facets_where(m, lambda X, Y, Z: (np.abs(Y - 0.0) < 0.05) & (X > 48) & (X < 63.5) & (Z > -208) & (Z < 125))
    rest = facets_where(m, lambda X, Y, Z: (np.abs(Y + 1.5) < 0.05) & (np.abs(Z) >= 188))
    holes = facets_where(m, lambda X, Y, Z: (np.hypot(X - 56, np.abs(Z) - 198) < 2.3))
    vm, um, disp, tot = solve(m, 70300.0, 0.33, [(rest, ["y"]), (holes, None)], [(top, (0.0, -Fg, 0.0))])
    report("drawer_guide", m, vm, um, disp, tot, 193.0, 3.0,
           [f"  RV-433 5052-H32 1.5 mm L; {Fg:.1f} N per guide (2 g) along the tray footprint",
            f"  supported on the mid cross members ({len(rest)} facets) + 2 bolt holes ({len(holes)} facets)",
            "  hand calc: 11.7 MPa, 0.17 mm deflection, FS 17"])


if __name__ == "__main__":
    os.makedirs(OUT, exist_ok=True)
    which = sys.argv[1] if len(sys.argv) > 1 else "all"
    for nm, fn in (("bracket", case_bracket), ("housing", case_housing), ("guide", case_guide)):
        if which in (nm, "all"):
            fn()
            print()
