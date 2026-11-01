"""Mass and centre-of-gravity estimate for the Rev B layout (masses in kg, first estimates)."""
import numpy as np
import model_b as M
import rules_b as R

EXTRA = {  # parts whose mass is not set in the model, or better estimates
    "post_fl": 0.06, "post_fr": 0.06, "post_rl": 0.06, "post_rr": 0.06, "wheel_l": 0.12, "wheel_r": 0.12,
    "caster_f": 0.30, "caster_r": 0.30, "riser": 0.04, "lidar": 0.05, "oak": 0.12, "estop": 0.08, "pi5": 0.07,
}
def centre(p):
    lo = np.array([1e9] * 3); hi = -lo
    for pr in p["prims"]:
        g, y0, y1 = R.prim_shape(pr)
        x0, z0, x1, z1 = g.bounds
        lo = np.minimum(lo, [x0, y0, z0]); hi = np.maximum(hi, [x1, y1, z1])
    return (lo + hi) / 2

items = []
for pl in M.plates():
    m = pl["poly"].area * (pl["y1"] - pl["y0"]) * 2.70e-6
    items.append((pl["id"], m, np.array([pl["poly"].centroid.x, (pl["y0"] + pl["y1"]) / 2, pl["poly"].centroid.y])))
for p in M.PARTS:
    if p.get("hidden"):
        continue
    m = EXTRA.get(p["id"], p.get("mass") or 0.0)
    items.append((p["id"], m, centre(p)))
items.append(("cables and fasteners", 0.18, np.array([-40.0, 150.0, 0.0])))
mt = sum(m for _, m, _ in items)
cg = sum(m * c for _, m, c in items) / mt
print(f"mass {mt:.2f} kg; CoG x {cg[0]:.1f} (+ forward), height {cg[1]:.1f}, z {cg[2]:.1f} mm")
H = cg[1]
xf_fwd, xr_fwd = M.CASTER_X - M.CASTER_TRAIL, -M.CASTER_X - M.CASTER_TRAIL      # casters trail while driving forward
xf_rev, xr_rev = M.CASTER_X + M.CASTER_TRAIL, -M.CASTER_X + M.CASTER_TRAIL      # and swing round in reverse
print(f"braking while driving forward: tips forward over the front caster above {(xf_fwd - cg[0]) / H:.2f} g")
print(f"accelerating forward: tips back over the rear caster above {(cg[0] - xr_fwd) / H:.2f} g")
print(f"braking while reversing: tips back over the rear caster above {(cg[0] - xr_rev) / H:.2f} g")
print(f"side tip (wheel track 224, over one drive wheel): {(M.TRACK / 2) / H:.2f} g")

# Cliff stop (DESIGN_RULES.md section 6): the front caster trails 20 mm behind its swivel axis, so it rolls over a
# drop about 33 mm after the front cliff sensors see it. The robot then finishes braking on its drive wheels and the
# rear caster, and the rear caster unloads once a * h > g * (distance of the CoG behind the axle). The springs settling
# onto the bump stop move the CoG about 3 mm further forward relative to the wheels.
A_CLIFF, SAG_SHIFT = 0.5, 3.0


def vol_centre(p):
    tv, acc = 0.0, np.zeros(3)
    for pr in p["prims"]:
        g, y0, y1 = R.prim_shape(pr)
        v = g.area * (y1 - y0)
        acc += v * np.array([g.centroid.x, (y0 + y1) / 2, g.centroid.y]); tv += v
    return acc / tv


alt = [(n, m, vol_centre(M.PART[n]) if n in M.PART else c) for n, m, c in items]
cg2 = sum(m * c for _, m, c in alt) / mt
need = A_CLIFF / 9.81 * H + SAG_SHIFT
for label, x in (("bounding-box centres", cg[0]), ("volume-weighted centres", cg2[0])):
    ok = -x >= need
    print(f"cliff stop ({label}): CoG {-x:.1f} mm behind the axle; a {A_CLIFF} m/s^2 stop needs {need:.1f} mm -> "
          f"{'OK' if ok else 'FAILS'} (rear caster unloads above {9.81 * max(0.0, -x - SAG_SHIFT) / H:.2f} m/s^2)")
for n, m, c in sorted(items, key=lambda t: -t[1])[:8]:
    print(f"   {n:22s} {m:5.2f} kg at x {c[0]:6.1f}, y {c[1]:6.1f}")
