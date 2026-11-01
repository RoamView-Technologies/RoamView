"""Route every cable, check the result, and export model.json + a text report."""
import json
import math
import sys
import time
import numpy as np
from scipy import ndimage

import model_b as M
import route_b as R

ORDER = ["P01", "P14", "P02", "P03", "P04", "P17", "P15", "P16", "P05", "P08", "P18", "P06", "P09", "P19", "P20",
         "P07", "P10", "P11", "P12", "P13",
         "D06", "D07", "D08", "D09", "D10", "D11", "D12", "D13", "D14", "D15", "D16",
         "D01", "D02", "D05", "D04", "D17", "D18", "D19", "D20", "D21", "D22", "D23", "D24", "D25", "D03"]
assert sorted(ORDER) == sorted(M.CABLE), set(M.CABLE) ^ set(ORDER)
LEAD = 6.0          # straight lead-out from every connector (mm)
TERM_ALLOW = 20.0   # per-end allowance for stripping / crimping in the cut length


def port_of(end):
    """Return (centre position, direction, [(role, pos), ...]) for a cable end."""
    if isinstance(end, tuple):
        a, b = M.PORTS[end[0]], M.PORTS[end[1]]
        pa, pb = np.array(a["pos"], float), np.array(b["pos"], float)
        d = np.array(a["dir"], float)
        return (pa + pb) / 2, d, [("+", pa), ("-", pb)]
    p = M.PORTS[end]
    return np.array(p["pos"], float), np.array(p["dir"], float), [("sig", np.array(p["pos"], float))]


def main():
    t0 = time.time()
    real, keep = R.build_grids()
    blocked = real | keep
    dist = ndimage.distance_transform_edt(~real) * R.STEP          # mm to nearest real surface
    air = (dist > 9.0).astype(np.float64)                            # free air: rise vertically, never drift
    R.AIR = air
    base = np.ones(real.shape)
    YY = R.ys[None, :, None]
    above = np.clip((YY - (M.T0 + 5.0)) / 24.0, 0, None)             # lie flat on the top deck
    base = base * (1.0 + np.broadcast_to(above, base.shape))
    occupied = np.zeros_like(real)
    print(f"grid {R.NX}x{R.NY}x{R.NZ} built in {time.time() - t0:.1f}s; blocked {blocked.mean() * 100:.1f}%")

    results = {}
    for cid in ORDER:
        c = M.CABLE[cid]
        pa, da, ta = port_of(c["a"])
        pb, db, tb = port_of(c["b"])
        if cid in M.MANUAL:
            center = R.fillet([np.array(p, float) for p in M.MANUAL[cid]], 6.0)
        else:
            cost = base.copy()
            near = ndimage.binary_dilation(occupied, iterations=2) & ~occupied
            cost[near] *= 0.82
            cost[occupied] *= 12.0
            la, lb = pa + da * LEAD, pb + db * LEAD
            waypoints = [la] + [np.array(v, float) for v in c.get("via", [])] + [lb]
            cells = []
            for w0, w1 in zip(waypoints, waypoints[1:]):
                s = R.nearest_free(w0, blocked)
                t = R.nearest_free(w1, blocked)
                if s is None or t is None:
                    raise SystemExit(f"{cid}: no free cell near {'start' if s is None else 'end'}")
                path = R._astar2(cost, air, blocked, s, t, R.NB, R.NBL, R.HF, R.DG, R.ORTHO, R.STEP, 0.8)
                if len(path) == 0:
                    raise SystemExit(f"{cid}: no route found")
                cells += [R.to_mm(q) for q in path]
            pts = [pa, la] + cells + [lb, pb]
            core = R.shorten(pts[1:-1], blocked, cost)
            rbend = max(6.0, 2.5 * c["od"] * (2 if c["pair"] else 1))
            center = R.fillet([pa] + core + [pb], rbend)
            # mark occupancy along the laid route
            rad = (c["od"] * (2 if c["pair"] else 1)) / 2
            for q in center:
                cc = R.to_cell(q)
                r = max(1, int(math.ceil(rad / R.STEP)))
                sl = tuple(slice(max(0, cc[k] - r), cc[k] + r + 1) for k in range(3))
                occupied[sl] = True
        results[cid] = dict(center=np.array(center))
        conds = make_conds(c, results[cid]["center"])
        L = max(R.length(q["pts"]) for q in conds)
        results[cid].update(conds=conds, length=L)
        print(f"  {cid:4s} {c['name']:24s} {L:6.0f} mm")
    relax(results)
    print(f"routed {len(results)} cables in {time.time() - t0:.1f}s")
    return results, real


def make_conds(c, center):
    pa, da, ta = port_of(c["a"])
    pb, db, tb = port_of(c["b"])
    conds = []
    if c["pair"]:
        n0 = ta[1][1] - ta[0][1] if len(ta) == 2 else np.array([0, 0, 1.0])
        N = R.rmf_offsets(center, n0 if np.linalg.norm(n0) > 1e-6 else np.cross(center[1] - center[0], [0, 1, 0]))
        off = c["od"] / 2 + 0.15
        for sign, role in ((-1, "+"), (1, "-")):
            p = center + sign * off * N
            p = R.blend_end(p, dict(ta)[role] if len(ta) == 2 else ta[0][1], da, at_start=True)
            p = R.blend_end(p, dict(tb)[role] if len(tb) == 2 else tb[0][1], db, at_start=False)
            conds.append(dict(role=role, pts=p))
    else:
        conds.append(dict(role="sig", pts=np.array(center)))
    return conds


def relax(results, iters=60, margin=0.35):
    """Push apart cable centre-lines that overlap (mostly where several cables share a pass-through),
    keeping 10 mm at each end fixed and never moving a point into a part or keep-out."""
    from scipy.spatial import cKDTree
    real15, keep = R.build_grids(c=1.5)
    blocked = real15 | keep
    eff = {cid: (M.CABLE[cid]["od"] + 0.15 if M.CABLE[cid]["pair"] else M.CABLE[cid]["od"] / 2) for cid in results}
    moved_total = 0
    for it in range(iters):
        samples = []
        for cid, r in results.items():
            C = r["center"]
            s = np.concatenate([[0], np.cumsum(np.linalg.norm(np.diff(C, axis=0), axis=1))])
            for i, q in enumerate(C):
                if 10 < s[i] < s[-1] - 10:
                    samples.append((cid, i, q))
        P = np.array([q for _, _, q in samples])
        tree = cKDTree(P)
        disp = {cid: np.zeros_like(r["center"]) for cid, r in results.items()}
        worst = 0.0
        for i, j in tree.query_pairs(12.0):
            a, b = samples[i], samples[j]
            if a[0] == b[0]:
                continue
            need = eff[a[0]] + eff[b[0]] + margin
            d = np.linalg.norm(a[2] - b[2])
            if d >= need or d < 1e-6:
                continue
            pen = need - d
            worst = max(worst, pen)
            u = (a[2] - b[2]) / d
            wa = 0.0 if a[0] in M.MANUAL else (1.0 if b[0] in M.MANUAL else 0.5)
            wb = 1.0 - wa if (a[0] in M.MANUAL or b[0] in M.MANUAL) else 0.5
            disp[a[0]][a[1]] += u * pen * wa
            disp[b[0]][b[1]] -= u * pen * wb
        if worst <= 0.05:
            break
        k = np.exp(-0.5 * (np.arange(-6, 7) / 2.5) ** 2); k /= k.sum()
        for cid, r in results.items():
            if cid in M.MANUAL or not np.any(disp[cid]):
                continue
            C = r["center"]
            D = np.stack([np.convolve(disp[cid][:, a], k, mode="same") for a in range(3)], 1) * 1.6
            s = np.concatenate([[0], np.cumsum(np.linalg.norm(np.diff(C, axis=0), axis=1))])
            w = np.clip(np.minimum(s, s[-1] - s) / 10.0 - 1.0, 0, 1)[:, None]      # 0 within 10 mm of the ends
            Cn = C + D * w
            for i in range(len(C)):
                cc = R.to_cell(Cn[i])
                if (cc < 0).any() or cc[0] >= R.NX or cc[1] >= R.NY or cc[2] >= R.NZ or blocked[tuple(cc)]:
                    Cn[i] = C[i]
            moved_total += 1
            r["center"] = Cn
    for cid, r in results.items():
        c = M.CABLE[cid]
        r["conds"] = make_conds(c, r["center"])
        r["length"] = max(R.length(q["pts"]) for q in r["conds"])
    print(f"relaxed overlaps in {it + 1} passes (worst remaining {worst:.2f} mm)")


# ------------------------------------------------------------------ checks
def check(results):
    """Strict clearance checks with zero routing margin: every conductor sample must
    be outside every part (by its own radius), and may cross a plate only inside a
    cable pass-through (manual LiDAR lead: its deck hole)."""
    strict = np.zeros((R.NX, R.NY, R.NZ), bool)
    for p in M.PARTS:
        if p.get("hidden"):
            continue
        R.rasterize_prims(p["prims"], strict, 0.6)
    import shapely
    from shapely.ops import unary_union
    plates = M.plates()
    issues = []
    for cid, res in results.items():
        c = M.CABLE[cid]
        r = c["od"] / 2
        for cond in res["conds"]:
            P = cond["pts"]
            s = np.concatenate([[0], np.cumsum(np.linalg.norm(np.diff(P, axis=0), axis=1))])
            Ltot = s[-1]
            for q, si in zip(P, s):
                if si < 9 or Ltot - si < 9:
                    continue      # inside the connector / lug
                cc = R.to_cell(q)
                in_bore = cid == "D03" and abs(q[0]) <= 6 and abs(q[2]) <= 6 and M.TU - 2 <= q[1] <= M.LIDAR_Y + 1
                if not in_bore and (cc >= 0).all() and cc[0] < R.NX and cc[1] < R.NY and cc[2] < R.NZ and strict[tuple(cc)]:
                    owner = which_part(q)
                    issues.append((cid, "touches " + owner, q))
                    break
                for pl in plates:
                    if pl["y0"] - r < q[1] < pl["y1"] + r:
                        pt = shapely.Point(q[0], q[2])
                        ok_holes = pl["holes"]["pass_through"] + (pl["holes"].get("lidar_cable", []) if cid == "D03" else [])
                        inside = any(h.buffer(-min(r, 1.0)).contains(pt) for h in ok_holes)
                        on_plate = shapely.Polygon(pl["poly"].exterior).contains(pt)
                        if on_plate and not inside:
                            issues.append((cid, f"crosses {pl['name']} outside a pass-through", q))
                            break
    # cable-to-cable overlaps (outside the first/last 15 mm)
    samples = []
    for cid, res in results.items():
        r = M.CABLE[cid]["od"] / 2
        for cond in res["conds"]:
            P = cond["pts"]
            s = np.concatenate([[0], np.cumsum(np.linalg.norm(np.diff(P, axis=0), axis=1))])
            for q, si in zip(P, s):
                if 15 < si < s[-1] - 15:
                    samples.append((cid, cond["role"], r, q))
    from scipy.spatial import cKDTree
    pts = np.array([q for _, _, _, q in samples])
    tree = cKDTree(pts)
    worst = {}
    for i, j in tree.query_pairs(4.5):
        a, b = samples[i], samples[j]
        if a[0] == b[0]:
            continue
        pen = a[2] + b[2] - np.linalg.norm(a[3] - b[3])
        key = tuple(sorted((a[0], b[0])))
        if pen > worst.get(key, (0,))[0]:
            worst[key] = (pen, a[3])
    overlaps = sorted(((k, v) for k, v in worst.items() if v[0] > 0.6), key=lambda kv: -kv[1][0])
    return issues, overlaps


def which_part(q):
    best = "?"
    for p in M.PARTS:
        if p.get("hidden"):
            continue
        g = np.zeros((1,), bool)
        for pr in p["prims"]:
            if pr["t"] == "box":
                lo, hi = np.array(pr["min"]) - 0.7, np.array(pr["max"]) + 0.7
                if (q >= lo).all() and (q <= hi).all():
                    return p["name"] + " / " + (pr.get("name") or "")
            elif pr["t"] == "p2020":
                o = np.array(pr["origin"])
                if abs(q[0] - o[0]) <= 10.7 and abs(q[2] - o[2]) <= 10.7 and o[1] - 0.7 <= q[1] <= o[1] + pr["h"] + 0.7:
                    return p["name"]
            else:
                o = np.array(pr["origin"]); prof = np.array(pr["profile"])
                if pr["axis"] == "y":
                    a, rad = q[1] - o[1], math.hypot(q[0] - o[0], q[2] - o[2])
                else:
                    a, rad = q[2] - o[2], math.hypot(q[0] - o[0], q[1] - o[1])
                if prof[:, 1].min() - 0.7 <= a <= prof[:, 1].max() + 0.7 and rad <= prof[:, 0].max() + 0.7:
                    best = p["name"] + " / " + (pr.get("name") or "")
    return best


if __name__ == "__main__":
    res, real = main()
    issues, overlaps = check(res)
    print("\nCLEARANCE ISSUES:", len(issues))
    for cid, what, q in issues:
        print(f"  {cid} {what} at ({q[0]:.0f}, {q[1]:.0f}, {q[2]:.0f})")
    print("CABLE OVERLAPS > 0.6 mm:", len(overlaps))
    for (a, b), (pen, q) in overlaps[:20]:
        print(f"  {a} x {b}: {pen:.1f} mm at ({q[0]:.0f}, {q[1]:.0f}, {q[2]:.0f})")
    np.save("routes_b.npy", {k: dict(center=v["center"], conds=[dict(role=c["role"], pts=c["pts"]) for c in v["conds"]], length=v["length"]) for k, v in res.items()}, allow_pickle=True)
