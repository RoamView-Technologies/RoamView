"""Export the Rev B interconnect model (model_b.py + routes_b.npy) to a CAD build spec for SOLIDWORKS.

The spec is plain JSON in millimetres, in the robot frame (+X forward, +Y up, +Z robot right; origin on the
floor under the drive-axle centre). RoamView_Build_RevB.py reads it on a Windows PC and builds every part,
the two decks, the harness and the assembly through the SOLIDWORKS API.

    python cad_spec_b.py            -> RoamView_RevB_spec.json
"""
import json
import math
import re
from datetime import date

import numpy as np
import shapely
from shapely.geometry import LineString

import model_b as M

ROUTES = np.load("routes_b.npy", allow_pickle=True).item()
R3 = lambda v: round(float(v), 3)

# ---------------------------------------------------------------------------- plates
def recover(poly):
    """Circle or obround slot back from a buffered shapely polygon (hole() / slot() in model_b)."""
    c = np.array(poly.centroid.coords[0])
    pts = np.array(poly.exterior.coords)[:-1]
    rr = np.linalg.norm(pts - c, axis=1)
    if rr.max() - rr.min() < 0.02:
        return dict(k="c", x=R3(c[0]), z=R3(c[1]), d=R3(2 * rr.max()))
    mrr = np.array(poly.minimum_rotated_rectangle.exterior.coords)[:4]
    e1, e2 = mrr[1] - mrr[0], mrr[2] - mrr[1]
    ax = e1 if np.linalg.norm(e1) > np.linalg.norm(e2) else e2
    ax = ax / np.linalg.norm(ax)
    s = (pts - c) @ ax
    half_total = (s.max() - s.min()) / 2
    seg = LineString([c - ax * 10 * half_total, c + ax * 10 * half_total])
    w = 2 * max(seg.distance(shapely.Point(p)) for p in pts)
    half = half_total - w / 2
    p1, p2 = c - ax * half, c + ax * half
    return dict(k="s", x1=R3(p1[0]), z1=R3(p1[1]), x2=R3(p2[0]), z2=R3(p2[1]), w=R3(w))


GROUP_NAMES = dict(
    posts="DeckPosts_M5", caster_bolts="CasterMounts_M5", spring_holes="SpringHoles_18", spring_cup_bolts="SpringCups_M2p5",
    hanger_bolts="PivotHangers_M4", battery_tray="BatteryTray_M3", battery_straps="BatteryStraps", pdb="PowerBoard_M3",
    driver="MotorDriver_M3", wago="Splices_M3", contacts="ChargeContacts_M4", bumper="Bumper_M4", cliff="CliffSensors_M2p5",
    f2="FuseF2_M3", pass_through="PassThroughs", lidar_bolts="LidarRiser_M3", lidar_cable="LidarCable_20",
    pi5="Pi5_M2p5", esp32="ESP32Carrier_M3", estop="EStop_22p5", camera="Camera_M4", indicators="Lights_16",
    handle="Handle_M5", power_blocks="PowerBlocks_M3", k2="K2Relay_M2p5", j2="LidarSplice_M3", mic="MicArray_M3",
    speaker="Speaker_M3",
)

plates = []
for pl in M.plates():
    groups = []
    for key, geoms in pl["holes"].items():
        groups.append(dict(name=GROUP_NAMES.get(key, key), items=[recover(g) for g in geoms]))
    outline = dict(L=M.PLATE_L, W=M.PLATE_W, R=M.CORNER_R)
    if pl["id"] == "baseplate":
        outline["wells"] = dict(half=M.WELL_HALF, z=M.WELL_Z, r=M.WELL_R)
    t = pl["y1"] - pl["y0"]
    fname = "RoamView_Baseplate_RevB" if pl["id"] == "baseplate" else "RoamView_TopDeck_RevB"
    poly = pl["poly"]
    plates.append(dict(
        id=pl["id"], file=fname, origin=[0.0, pl["y0"], 0.0], t=t, outline=outline, groups=groups,
        material="6061-T6 (SS)", color=pl["color"],
        expect=dict(volume=round(poly.area * t, 1), box=[-M.PLATE_L / 2, 0.0, -M.PLATE_W / 2, M.PLATE_L / 2, t, M.PLATE_W / 2]),
        props=dict(Description=pl["name"] + " (Rev B)", PartNo=pl["partno"], Revision="B", Project="ENGR 490 RoamView",
                   Stock="6061-T6 aluminium plate, %g mm" % t, Notes=pl["desc"][:250])))

# ---------------------------------------------------------------------------- parts
NAMES = dict(
    post="RoamView_Post2020_RevB", wheel="RoamView_DriveWheel125_RevB", motor="RoamView_DriveModule_RevB",
    hanger="RoamView_PivotHanger_RevB", spring="RoamView_SpringAssy_RevB", caster="RoamView_Caster3in_RevB",
    battery_tray="RoamView_BatteryTray_RevB", battery="RoamView_Battery4S2P_RevB", pdb="RoamView_PowerBoard_RevB",
    wago="RoamView_WagoSplices_RevB", xt60="RoamView_XT60_RevB", driver="RoamView_MotorDriver_RevB",
    chg="RoamView_ChargeGuard_RevB", contact="RoamView_ChargeContact_RevB", bumper="RoamView_Bumper_RevB",
    cliff="RoamView_CliffSensor_RevB", plug_pwr="RoamView_DeckPlug6_RevB", plug_sig="RoamView_DeckPlug24_RevB",
    riser="RoamView_LidarRiser40_RevB", lidar="RoamView_LidarD500_RevB", oak="RoamView_OakDProW_RevB",
    estop="RoamView_EStop_RevB", pi5="RoamView_Pi5_RevB", pi_hat="RoamView_PiHat_RevB", esp32="RoamView_ESP32Carrier_RevB",
    rx1="RoamView_RCReceiver_RevB", yadapter="RoamView_OakYAdapter_RevB", mic="RoamView_MicArray_RevB",
    speaker="RoamView_Speaker_RevB", led_rec="RoamView_LightRec_RevB", led_status="RoamView_LightStatus_RevB",
    handle="RoamView_Handle_RevB", cpwr="RoamView_ComputePower_RevB", spwr="RoamView_SensorPower_RevB",
    k2="RoamView_K2Relay_RevB", j2="RoamView_LidarSplice_RevB",
)


def group_of(pid):
    for g in ("post", "wheel", "motor", "hanger", "spring", "caster", "contact", "cliff"):
        if pid.startswith(g + "_"):
            return g, pid[len(g) + 1:].upper()
    return pid, ""


def prim_box(pr):
    if pr["t"] == "box":
        return pr["min"], pr["max"]
    o = pr["origin"]
    if pr["t"] == "p2020":
        return [o[0] - 10, o[1], o[2] - 10], [o[0] + 10, o[1] + pr["h"], o[2] + 10]
    rmax = max(p[0] for p in pr["profile"])
    a0, a1 = min(p[1] for p in pr["profile"]), max(p[1] for p in pr["profile"])
    if pr["axis"] == "y":
        return [o[0] - rmax, o[1] + a0, o[2] - rmax], [o[0] + rmax, o[1] + a1, o[2] + rmax]
    return [o[0] - rmax, o[1] - rmax, o[2] + a0], [o[0] + rmax, o[1] + rmax, o[2] + a1]


def to_local(pr, org):
    q = dict(t=pr["t"], color=[round(c, 3) for c in (pr.get("color") or [0.6, 0.6, 0.6])], name=pr.get("name") or "")
    sub = lambda p: [R3(p[i] - org[i]) for i in range(3)]
    if pr["t"] == "box":
        q["min"], q["max"] = sub(pr["min"]), sub(pr["max"])
    elif pr["t"] == "p2020":
        q["origin"], q["h"] = sub(pr["origin"]), pr["h"]
    else:
        q["axis"], q["origin"], q["profile"] = pr["axis"], sub(pr["origin"]), [[R3(a), R3(b)] for a, b in pr["profile"]]
    return q


def clean(s, n=250):
    s = re.sub(r"\s+", " ", s or "").strip()
    return s if len(s) <= n else s[: n - 3] + "..."


groups = {}
for p in M.PARTS:
    if p.get("hidden"):
        continue
    lo = np.min([prim_box(pr)[0] for pr in p["prims"]], axis=0)
    hi = np.max([prim_box(pr)[1] for pr in p["prims"]], axis=0)
    org = [round(float(lo[0] + hi[0]) / 2, 2), round(float(lo[1]), 2), round(float(lo[2] + hi[2]) / 2, 2)]
    prims = [to_local(pr, org) for pr in p["prims"]]
    sig = json.dumps(prims, sort_keys=True)
    g, suffix = group_of(p["id"])
    groups.setdefault(g, []).append(dict(part=p, org=org, prims=prims, sig=sig, suffix=suffix,
                                         box=[R3(v) for v in list(lo - org) + list(hi - org)]))

parts, occurrences = [], []
for g, items in groups.items():
    sigs = {}
    for it in items:
        sigs.setdefault(it["sig"], []).append(it)
    base = NAMES.get(g, "RoamView_%s_RevB" % "".join(w.capitalize() for w in g.split("_")))
    for k, (sig, same) in enumerate(sigs.items()):
        first = same[0]
        fname = base if len(sigs) == 1 else base.replace("_RevB", "_%s_RevB" % first["suffix"])
        p = first["part"]
        desc = p["desc"]
        placeholder = "placeholder" in desc.lower()
        parts.append(dict(
            file=fname, ids=[s["part"]["id"] for s in same], prims=first["prims"], expect=dict(box=first["box"]),
            props=dict(Description=clean(p["name"] if len(same) == 1 else re.sub(r"\s+(left|right|front|rear|FL|FR|RL|RR|\+|-)$", "", p["name"])),
                       Notes=clean(desc), Revision="B", Project="ENGR 490 RoamView", ModelID=", ".join(s["part"]["id"] for s in same),
                       Placeholder="yes" if placeholder else "no", MassEstimate_kg="%.3f" % (p.get("mass") or 0.0))))
        for s in same:
            occurrences.append(dict(file=fname, id=s["part"]["id"], origin=s["org"]))

# ---------------------------------------------------------------------------- harness
HEX = dict(supply="#e87ba4", motor="#eda100", compute="#2a78d6", sensor="#008300",
           usb="#2a78d6", uart="#008300", drive="#eda100", io="#e87ba4", ret="#2F3A44")
rgb = lambda h: [round(int(h[i:i + 2], 16) / 255, 3) for i in (1, 3, 5)]


def resample(P, ds):
    P = np.asarray(P, float)
    s = np.concatenate([[0], np.cumsum(np.linalg.norm(np.diff(P, axis=0), axis=1))])
    n = max(2, int(np.ceil(s[-1] / ds)) + 1)
    t = np.linspace(0, s[-1], n)
    return np.stack([np.interp(t, s, P[:, k]) for k in range(3)], 1)


def radii(P):
    a, b, c = P[:-2], P[1:-1], P[2:]
    ab, bc, ca = (np.linalg.norm(b - a, axis=1), np.linalg.norm(c - b, axis=1), np.linalg.norm(a - c, axis=1))
    area = np.linalg.norm(np.cross(b - a, c - a), axis=1) / 2
    with np.errstate(divide="ignore", invalid="ignore"):
        return np.where(area < 1e-12, np.inf, ab * bc * ca / (4 * area))


def smooth_for_sweep(P, od):
    """The router's centre lines can turn tighter than a swept circle allows (short grid steps, the inner
    conductor of a pair). Relax only the tight spots, ends fixed, until every bend is >= 0.8 x OD, then
    resample evenly so the SOLIDWORKS spline through the points stays smooth."""
    target = max(0.8 * od, 1.5)
    Q = resample(P, 1.0)
    for _ in range(400):
        r = radii(Q)
        bad = np.where(r < target)[0] + 1
        if len(bad) == 0:
            break
        idx = sorted({j for i in bad for j in range(i - 2, i + 3) if 0 < j < len(Q) - 1})
        idx = np.array(idx)
        Q[idx] = Q[idx] + 0.5 * ((Q[idx - 1] + Q[idx + 1]) / 2 - Q[idx])
    Q = resample(Q, max(2.0, 0.8 * od))
    ref = resample(P, 0.25)
    dev = float(np.max(np.min(np.linalg.norm(Q[:, None, :] - ref[None, :, :], axis=2), axis=1)))
    return Q, dev


conductors = []
for c in M.CABLES:
    r = ROUTES[c["id"]]
    for q in r["conds"]:
        P, dev = smooth_for_sweep(q["pts"], c["od"])
        role = q["role"]
        col = HEX["ret"] if role == "-" else HEX.get(c["cat"], "#7B8894")
        conductors.append(dict(cable=c["id"], name=c["name"], role=role, od=c["od"], color=rgb(col),
                               pts=[[R3(v) for v in p] for p in P], smoothing_mm=round(dev, 2), length=round(float(np.sum(np.linalg.norm(np.diff(np.asarray(q["pts"]), axis=0), axis=1))), 1)))

spec = dict(
    meta=dict(title="RoamView chassis, Rev B layout 2", built=str(date.today()), units="mm",
              frame="+X forward, +Y up, +Z robot right; origin on the floor under the drive-axle centre (ROS base_footprint)",
              source="model_b.py + routes_b.npy (old-designs/Interconnect_RevB), exported by cad_spec_b.py"),
    assembly=dict(file="RoamView_Chassis_RevB"),
    plates=plates, parts=parts, occurrences=occurrences,
    harness=dict(file="RoamView_Harness_RevB", conductors=conductors),
)
json.dump(spec, open("RoamView_RevB_spec.json", "w"), indent=0, separators=(",", ":"),
          default=lambda o: o.item() if hasattr(o, "item") else str(o))
print("plates", len(plates), "| part files", len(parts), "| occurrences", len(occurrences), "| conductors", len(conductors))
print("spec size", round(len(open("RoamView_RevB_spec.json").read()) / 1024), "KB")
for p in parts:
    print("  %-38s %-26s prims %d" % (p["file"], ",".join(p["ids"]), len(p["prims"])))
