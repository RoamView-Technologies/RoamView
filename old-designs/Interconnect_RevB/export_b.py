"""Export model + routes to model.json for the web page, plus a harness summary."""
import json
import math
import numpy as np
import shapely
from shapely.ops import unary_union

import model_b as M

res = np.load("routes_b.npy", allow_pickle=True).item()
TERM = 20.0


def ring(coords, tol=0.05):
    ls = shapely.LinearRing(coords).simplify(tol)
    return [[round(x, 2), round(z, 2)] for x, z in list(ls.coords)[:-1]]


def sub(P, ds=2.5):
    P = np.asarray(P)
    s = np.concatenate([[0], np.cumsum(np.linalg.norm(np.diff(P, axis=0), axis=1))])
    n = max(2, int(math.ceil(s[-1] / ds)) + 1)
    t = np.linspace(0, s[-1], n)
    Q = np.stack([np.interp(t, s, P[:, k]) for k in range(3)], 1)
    return [[round(float(v), 1) for v in q] for q in Q]


def cut(L):
    return int(math.ceil((L + 2 * TERM) / 10.0) * 10)


plates = []
for pl in M.plates():
    poly = pl["poly"]
    plates.append(dict(id=pl["id"], name=pl["name"], y0=pl["y0"], y1=pl["y1"], color=pl["color"],
                       partno=pl["partno"], desc=pl["desc"], outer=ring(poly.exterior.coords),
                       holes=[ring(r.coords) for r in poly.interiors]))


def end_info(end):
    pid = end[0] if isinstance(end, tuple) else end
    p = M.PORTS[pid]
    labels = [M.PORTS[e]["label"] for e in (end if isinstance(end, tuple) else (end,))]
    uniq = []
    for l in labels:
        if l not in uniq:
            uniq.append(l)
    return dict(part=p["part"], partName=M.PART[p["part"]]["name"], terminal=" / ".join(uniq))


cables = []
for c in M.CABLES:
    r = res[c["id"]]
    conds = [dict(role=q["role"], pts=sub(q["pts"])) for q in r["conds"]]
    L = r["length"]
    cables.append(dict(id=c["id"], name=c["name"], layer=c["layer"], cat=c["cat"], gauge=c["gauge"], od=c["od"],
                       spec=c["spec"], conn=c["conn"], pins=c.get("pins", ""), notes=c.get("notes", ""),
                       pair=c["pair"], a=end_info(c["a"]), b=end_info(c["b"]), length=round(L), cut=cut(L),
                       conductors=conds))

parts = []
for p in M.PARTS:
    parts.append(dict(id=p["id"], name=p["name"], group=p["group"], sys=p["sys"], desc=p["desc"],
                      hidden=bool(p.get("hidden")), file=p.get("file", ""), prims=p["prims"]))

# ---- pass-through usage --------------------------------------------------------
NAMES = {("baseplate", "rear"): "Lower deck rear pass-through", ("baseplate", "front"): "Lower deck front-left pass-through",
         ("baseplate", "front_r"): "Lower deck front-right pass-through", ("topdeck", "left"): "Top deck left pass-through",
         ("topdeck", "right"): "Top deck right pass-through", ("topdeck", "rear"): "Top deck rear pass-through",
         ("topdeck", "front"): "Top deck front pass-through", ("topdeck", "centre"): "Top deck centre pass-through"}
slots = []
for pl in M.plates():
    table = M.PT_LOWER if pl["id"] == "baseplate" else M.PT_TOP
    for key, h in table.items():
        cx, cz = h.centroid.x, h.centroid.y
        users = []
        for c in M.CABLES:
            for q in res[c["id"]]["conds"]:
                P = q["pts"]
                hit = any(pl["y0"] - 0.5 <= p[1] <= pl["y1"] + 0.5 and h.contains(shapely.Point(p[0], p[2])) for p in P)
                if hit:
                    users.append((c["id"], c["od"]))
        fill = sum(math.pi * (od / 2) ** 2 for _, od in users)
        lined = h.buffer(-2.0)                       # edge grommet takes about 2 mm per side
        slots.append(dict(name=NAMES[(pl["id"], key)], plate=pl["id"], x=round(cx, 1), z=round(cz, 1), area=round(lined.area),
                          conductors=len(users), cables=sorted(set(u[0] for u in users)),
                          fill_pct=round(100 * fill / lined.area, 1)))

# ---- wire totals by gauge -------------------------------------------------------
totals = {}
for c in cables:
    g = c["gauge"]
    totals[g] = totals.get(g, 0) + c["cut"]

out = dict(
    meta=dict(title="RoamView Interconnect", rev="B, layout 2", built="2026-09-28",
              frame="+X forward, +Y up, +Z robot right, mm. Origin: floor under the drive-axle centre (ROS base_footprint).",
              source="model_b.py (Rev B layout, checked by rules_b.py); routes from build_b.py"),
    plates=plates, parts=parts, cables=cables, slots=slots,
    totals=[dict(gauge=g, cut_mm=v) for g, v in sorted(totals.items())],
    checks=dict(plate_volume_mm3={pl["id"]: round(pl["poly"].area * (pl["y1"] - pl["y0"])) for pl in M.plates()}),
)
json.dump(out, open("model_b.json", "w"), separators=(",", ":"))
print("model_b.json", round(len(json.dumps(out, separators=(',', ':'))) / 1024), "KB")
for s in slots:
    print(f"{s['name']:32s} {s['conductors']:2d} conductors, fill {s['fill_pct']:4.1f} %  {', '.join(s['cables'])}")
print()
for t in out["totals"]:
    print(f"  {t['gauge']:34s} {t['cut_mm'] / 1000:.2f} m")
print("total cut length", sum(c['cut'] for c in cables) / 1000, "m over", len(cables), "cables")
