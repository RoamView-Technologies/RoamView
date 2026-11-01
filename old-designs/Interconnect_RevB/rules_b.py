"""Design-rule checks for the Rev B layout (old-designs/DESIGN_RULES.md, sections 2, 3 and 6).

Every prim is treated as an extrusion along Y (boxes, Y-axis lathes, 2020 posts) or, for Z-axis lathes,
as its bounding box. Distances are exact for extrusions: sqrt(d_xz^2 + d_y^2)."""
import math
import sys
import shapely
from shapely.geometry import box as sbox, Point

import model_b as M

SPACING, SERVICE_SPACING, MOVING_CLEAR = 10.0, 15.0, 2.0
HOLE_WEB = 1.5           # mm of plate left between two holes, and at least this (or half the hole width) to an edge


def prim_shape(pr):
    """Return (xz shapely geometry, y0, y1) for a primitive."""
    if pr["t"] == "box":
        a, b = pr["min"], pr["max"]
        return sbox(a[0], a[2], b[0], b[2]), a[1], b[1]
    if pr["t"] == "p2020":
        o = pr["origin"]
        return sbox(o[0] - 10, o[2] - 10, o[0] + 10, o[2] + 10), o[1], o[1] + pr["h"]
    o = pr["origin"]
    prof = pr["profile"]
    rmax = max(p[0] for p in prof)
    amin, amax = min(p[1] for p in prof), max(p[1] for p in prof)
    if pr["axis"] == "y":
        return Point(o[0], o[2]).buffer(rmax, quad_segs=16), o[1] + amin, o[1] + amax
    return sbox(o[0] - rmax, o[2] + amin, o[0] + rmax, o[2] + amax), o[1] - rmax, o[1] + rmax


def shapes(prims):
    return [prim_shape(p) for p in prims]


def gap(sa, sb):
    """Minimum distance between two lists of (xz, y0, y1) shapes."""
    best = 1e9
    for ga, a0, a1 in sa:
        for gb, b0, b1 in sb:
            dy = max(0.0, b0 - a1, a0 - b1)
            if dy >= best:
                continue
            dxz = ga.distance(gb)
            d = math.hypot(dxz, dy)
            if d < best:
                best = d
    return best


def zone_shape(z):
    a, b = z["min"], z["max"]
    return [(sbox(a[0], a[2], b[0], b[2]), a[1], b[1])]


def rotated_prims(prims, deg):
    out = []
    for pr in prims:
        if pr["t"] == "box":
            a, b = pr["min"], pr["max"]
            pts = [M.rot_about_pivot((x, y, 0), deg) for x in (a[0], b[0]) for y in (a[1], b[1])]
            xs, ys = [p[0] for p in pts], [p[1] for p in pts]
            q = dict(pr); q["min"] = [min(xs), min(ys), a[2]]; q["max"] = [max(xs), max(ys), b[2]]
            out.append(q)
        elif pr["t"] == "lathe" and pr["axis"] == "z":
            o = M.rot_about_pivot(pr["origin"], deg)
            q = dict(pr); q["origin"] = [o[0], o[1], pr["origin"][2]]
            out.append(q)
        else:
            out.append(pr)
    return out


def caster_radius(y):
    """Horizontal radius swept by a 3-inch caster (wheel + fork) about its swivel axis at height y."""
    r = M.CASTER_WD / 2
    rr = 0.0
    if 0 <= y <= 2 * r:
        rr = math.hypot(M.CASTER_TRAIL + math.sqrt(max(0.0, r * r - (y - r) ** 2)), 15.0)
    if 30 <= y <= 80:
        rr = max(rr, math.hypot(M.CASTER_TRAIL + 9, 20.5))
    if 76.5 <= y <= 86:
        rr = max(rr, math.hypot(max(16, M.CASTER_TRAIL + 9), 21))
    return rr


def check(verbose=True):
    parts = [p for p in M.PARTS if not p.get("hidden")]
    S = {p["id"]: shapes(p["prims"]) for p in parts}
    joined = set()
    for p in parts:
        for j in p["join"]:
            joined.add(tuple(sorted((p["id"], j))))
    drive = {"wheel_l", "wheel_r", "motor_l", "motor_r", "hanger_l", "hanger_r", "spring_l", "spring_r"}
    issues = []

    # 1. spacing between parts; where a deck plate lies between two prims, only the parts of them
    #    that the plate does not cover can see each other
    # fastener holes are filled by their fasteners: only wells, pass-throughs and cable holes are openings
    from shapely.ops import unary_union as _uu
    _open = {"baseplate": [M.wells()] + list(M.PT_LOWER.values()),
             "topdeck": list(M.PT_TOP.values()) + [M.hole(0, 0, 20)]}
    _outline = M._rrect(M.PLATE_L / 2, M.PLATE_W / 2, M.CORNER_R)
    plates = [(pl["y0"], pl["y1"], _outline.difference(_uu(_open[pl["id"]]))) for pl in M.plates()]

    def gap_sep(sa, sb):
        best = 1e9
        for ga, a0, a1 in sa:
            for gb, b0, b1 in sb:
                dy = max(0.0, b0 - a1, a0 - b1)
                if dy >= best:
                    continue
                g1, g2 = ga, gb
                for p0, p1, poly in plates:
                    if (a1 <= p0 + 0.01 and b0 >= p1 - 0.01) or (b1 <= p0 + 0.01 and a0 >= p1 - 0.01):
                        g1, g2 = g1.difference(poly), g2.difference(poly)
                if g1.is_empty or g2.is_empty:
                    continue
                d = math.hypot(g1.distance(g2), dy)
                best = min(best, d)
        return best

    for i, a in enumerate(parts):
        for b in parts[i + 1:]:
            key = tuple(sorted((a["id"], b["id"])))
            if key in joined:
                continue
            if a["id"] in drive and b["id"] in drive and a["id"][-1] == b["id"][-1]:
                continue
            need = SERVICE_SPACING if (a["service"] or b["service"]) else SPACING
            d = gap_sep(S[a["id"]], S[b["id"]])
            if d < need - 1e-6:
                issues.append(("spacing", f"{a['name']} / {b['name']}: {d:.1f} mm (needs {need:.0f})", (a["id"], b["id"])))

    # 2. finger zones stay free
    for a in parts:
        for z in a["zones"]:
            zs = zone_shape(z)
            for b in parts:
                if b["id"] == a["id"] or tuple(sorted((a["id"], b["id"]))) in joined:
                    continue
                if gap(zs, S[b["id"]]) < 1e-6:
                    issues.append(("zone", f"{a['name']} '{z['label']}' zone is blocked by {b['name']}", (a["id"], b["id"])))

    # 3. footprint
    outline = M._rrect(M.PLATE_L / 2, M.PLATE_W / 2, M.CORNER_R).buffer(0.01)
    exempt = {"bumper", "contact_p", "contact_n", "caster_f", "caster_r"}
    for a in parts:
        if a["id"] in exempt:
            continue
        for g, y0, y1 in S[a["id"]]:
            if not outline.contains(g):
                issues.append(("footprint", f"{a['name']} sticks out of the 320 x 260 footprint", (a["id"],)))
                break

    # 4. LiDAR scan band and 5. OAK-D view
    lo, hi = M.SCAN_BAND
    for a in parts:
        if a["id"] in ("lidar",):
            continue
        if any(y1 > lo and y0 < hi for _, y0, y1 in S[a["id"]]):
            issues.append(("lidar", f"{a['name']} reaches into the LiDAR band {lo:.0f}-{hi:.0f} mm", (a["id"],)))
        if a["id"] != "oak" and any(g.bounds[2] > 149 and y1 > M.T0 + 10 for g, y0, y1 in S[a["id"]]):
            issues.append(("oak", f"{a['name']} is in the OAK-D's view", (a["id"],)))

    # 6. battery removal path and the space above the pack
    lane = [(sbox(58.5, -40.5, 175, 40.5), M.Y0 + 8.5, M.Y0 + 50)]
    above = [(sbox(-38, -40.5, 58, 40.5), M.Y0 + 46.5, M.TU)]
    for a in parts:
        if a["id"] in ("battery", "battery_tray"):
            continue
        if gap(lane, S[a["id"]]) < 1e-6:
            issues.append(("battery", f"{a['name']} blocks the battery's slide-out path", (a["id"],)))
        if a["id"] != "xt60" and gap(above, S[a["id"]]) < 1e-6:
            issues.append(("battery", f"{a['name']} sits above the battery", (a["id"],)))

    # 7. wheel wells
    wells = [(sbox(-M.WELL_HALF, s * M.WELL_Z, M.WELL_HALF, s * 140) if s > 0 else sbox(-M.WELL_HALF, -140, M.WELL_HALF, -M.WELL_Z),
              0, M.WHEEL_D + M.TRAVEL + 5) for s in (1, -1)]
    for a in parts:
        if a["id"] in drive:
            continue
        if gap(wells, S[a["id"]]) < 1e-6:
            issues.append(("well", f"{a['name']} is inside a wheel well", (a["id"],)))

    # 8. suspension sweep: bump and droop poses
    deck = M.baseplate_outline()
    for side in ("l", "r"):
        for deg, label in ((M.BUMP_DEG, "bump"), (M.DROOP_DEG, "droop")):
            mod = shapes(rotated_prims(PART("motor_" + side)["prims"] + PART("wheel_" + side)["prims"], deg))
            for a in parts:
                if a["id"] in drive:
                    continue
                d = gap(mod, S[a["id"]])
                if d < MOVING_CLEAR:
                    issues.append(("sweep", f"Drive module {side.upper()} at {label} comes within {d:.1f} mm of {a['name']}", (a["id"],)))
            for g, y0, y1 in mod:
                if y1 > M.BASE_Y - 1 and g.intersects(deck):
                    issues.append(("sweep", f"Drive module {side.upper()} at {label} hits the lower deck underside (top {y1:.1f})", ("motor_" + side,)))
                    break

    # 9. caster swing
    for cid, sx in (("caster_f", 1), ("caster_r", -1)):
        axis = Point(sx * M.CASTER_X, 0)
        for a in parts:
            if a["id"] == cid:
                continue
            for g, y0, y1 in S[a["id"]]:
                hit = False
                for y in range(0, 87, 2):
                    if y0 <= y <= y1 and g.distance(axis) < caster_radius(y) + MOVING_CLEAR:
                        hit = True
                        break
                if hit:
                    issues.append(("caster", f"{a['name']} is inside the {cid.replace('_', ' ')} swing", (a["id"],)))
                    break

    # 10. plate holes: material between neighbouring holes, and between a hole and the plate edge
    for pl in M.plates():
        outline = M.baseplate_outline() if pl["id"] == "baseplate" else M._rrect(M.PLATE_L / 2, M.PLATE_W / 2, M.CORNER_R)
        edge = outline.exterior
        hs = [(k, i, g) for k, v in pl["holes"].items() for i, g in enumerate(v)]
        for a in range(len(hs)):
            ka, ia, ga = hs[a]
            wa = min(ga.bounds[2] - ga.bounds[0], ga.bounds[3] - ga.bounds[1])
            need_edge = max(HOLE_WEB, min(0.5 * wa, 5.0))
            de = ga.exterior.distance(edge)
            if de < need_edge - 1e-6:
                issues.append(("hole", f"{pl['id']}: {ka}[{ia}] leaves {de:.1f} mm to the plate edge (needs {need_edge:.1f})", ()))
            for b in range(a + 1, len(hs)):
                kb, ib, gb = hs[b]
                d = ga.distance(gb)
                if d < HOLE_WEB - 1e-6:
                    issues.append(("hole", f"{pl['id']}: {ka}[{ia}] and {kb}[{ib}] leave {d:.1f} mm between them (needs {HOLE_WEB:.1f})", ()))

    if verbose:
        print(f"{len(parts)} parts checked; {len(issues)} issues")
        for kind, msg, _ in issues:
            print(f"  [{kind}] {msg}")
    return issues


def PART(pid):
    return M.PART[pid]


if __name__ == "__main__":
    iss = check()
    sys.exit(1 if iss else 0)
