"""3D sensor field-of-view / self-occlusion check (ray tracing against the as-built RV-100 meshes).

Run with the scratch venv (trimesh, matplotlib):  python fov_check.py
Inputs : Simulation/FOV/parts/*.STL (one per component, assembly frame), RV_globals.txt
Outputs: Simulation/FOV/fov_report.txt + PNG plots
SolidWorks frame X = ROS y, Y = ROS z, Z = ROS x (mm); everything below is in ROS (x fwd, y left, z up).
"""
import glob
import math
import os
import re

import matplotlib
import numpy as np
import trimesh

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
OUT = os.path.join(ROOT, "Simulation", "FOV")


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


def load_parts():
    meshes, labels = [], []
    for f in sorted(glob.glob(os.path.join(OUT, "parts", "*.STL"))):
        m = trimesh.load(f, force="mesh")
        v = np.asarray(m.vertices)
        m.vertices = np.column_stack([v[:, 2], v[:, 0], v[:, 1]])     # SW -> ROS
        name = os.path.basename(f)[len("RV-100 - "):-4]
        meshes.append(m)
        labels.append(name)
    face_owner = np.concatenate([np.full(len(m.faces), i) for i, m in enumerate(meshes)])
    return trimesh.util.concatenate(meshes), face_owner, labels


MESH, OWNER, LABELS = load_parts()
RAY = trimesh.ray.ray_triangle.RayMeshIntersector(MESH)


def short(label):
    return label.split(" ")[-1].rsplit("-", 1)[0]


def first_hits(origins, dirs, exclude=()):
    """Nearest hit per ray, ignoring faces of components whose label contains any `exclude` text."""
    loc, ri, tri = RAY.intersects_location(origins, dirs, multiple_hits=True)
    n = len(origins)
    dist = np.full(n, np.inf)
    owner = np.full(n, -1)
    if len(ri):
        own = OWNER[tri]
        keep = np.array([not any(x in LABELS[o] for x in exclude) for o in own])
        loc, ri, own = loc[keep], ri[keep], own[keep]
        d = np.linalg.norm(loc - origins[ri], axis=1)
        for r, dd, o in zip(ri, d, own):
            if dd < dist[r]:
                dist[r], owner[r] = dd, o
    return dist, owner


def rot_y(v, ang):          # pitch about +y (positive = nose down for the x axis)
    c, s = math.cos(ang), math.sin(ang)
    return np.array([c * v[0] + s * v[2], v[1], -s * v[0] + c * v[2]])


lines = ["RoamView - 3D sensor field-of-view / self-occlusion check",
         f"mesh: {len(LABELS)} components, {len(MESH.faces)} triangles; robot bounds (ROS mm) "
         f"x {MESH.bounds[0][0]:.0f}..{MESH.bounds[1][0]:.0f}  y {MESH.bounds[0][1]:.0f}..{MESH.bounds[1][1]:.0f}  "
         f"z {MESH.bounds[0][2]:.0f}..{MESH.bounds[1][2]:.0f}", ""]

# ------------------------------------------------------------------ LiDAR
lx, lz = G["LIDAR_X"], G["SCAN_Z"]
az = np.radians(np.arange(0, 360, 0.72))
res = {}
for elev in (0.0, 1.0, -1.0):
    e = math.radians(elev)
    d = np.column_stack([np.cos(az) * math.cos(e), np.sin(az) * math.cos(e), np.full_like(az, math.sin(e))])
    o = np.array([lx, 0, lz]) + d * 22.0                               # start just outside the dome
    dist, own = first_hits(o, d, exclude=("LiDAR",))
    res[elev] = (dist, own)
dist, own = res[0.0]
blocked = dist < 1000
lines.append(f"LiDAR STL-19P @ ({lx:.0f}, 0, {lz:.0f}), scan plane, 0.72 deg steps, self-occlusion within 1 m")
lines.append(f"  blocked: {blocked.sum() * 0.72:.1f} deg of 360 ({blocked.mean() * 100:.1f} %)")
sectors, start = [], None
for i in range(len(az) + 1):
    b = blocked[i % len(az)] if i < len(az) else False
    if b and start is None:
        start = i
    if not b and start is not None:
        idx = range(start, i)
        names = sorted({short(LABELS[own[k]]) for k in idx})
        sectors.append((math.degrees(az[start]), math.degrees(az[i - 1]), min(dist[k] for k in idx), names))
        start = None
for a0, a1, dmin, names in sectors:
    lines.append(f"   {a0:6.1f} .. {a1:6.1f} deg  ({a1 - a0 + 0.72:4.1f} deg)  nearest {dmin:5.0f} mm  {', '.join(names)}")
for elev in (1.0, -1.0):
    lines.append(f"  beam tilted {elev:+.0f} deg: blocked {np.sum(res[elev][0] < 1000) * 0.72:.1f} deg")
lines.append("  (0 deg = forward, angles counter-clockwise / to the left)")
fig = plt.figure(figsize=(6, 6))
ax = fig.add_subplot(projection="polar")
ax.set_theta_zero_location("N")
ax.bar(az, np.where(blocked, 1, 0), width=math.radians(0.72), color="#BC3126", alpha=0.8)
ax.set_yticks([])
ax.set_title("LiDAR scan plane: blocked bearings (red)\nup = forward", fontsize=11)
fig.savefig(os.path.join(OUT, "lidar_occlusion.png"), dpi=110, bbox_inches="tight")
plt.close(fig)
lines.append("")

# ------------------------------------------------------------------ OAK-D Pro W
tilt = math.radians(G["OAK_TILT"])
lens = np.array([G["LENS_X"] - 0.1, 0.0, G["LENS_Z"] - 2.0])           # as-built (tilt pivots the lens ~2 mm down)
cams = {"mono L (depth)": (np.array([0, 37.5, 0]), 127.0, 79.5), "mono R (depth)": (np.array([0, -37.5, 0]), 127.0, 79.5),
        "RGB": (np.array([0, 0, 0]), 95.0, 72.0)}
fig, axs = plt.subplots(1, 3, figsize=(15, 4.2))
floor_min = {}
for (name, (off, hf, vf)), ax in zip(cams.items(), axs):
    hs = np.radians(np.arange(-hf / 2, hf / 2 + 0.01, 1.0))
    vs = np.radians(np.arange(-vf / 2, vf / 2 + 0.01, 1.0))
    H, V = np.meshgrid(hs, vs)
    d = np.stack([np.ones_like(H), np.tan(H), np.tan(V)], -1).reshape(-1, 3)      # rectilinear (pinhole) camera
    d = d / np.linalg.norm(d, axis=1)[:, None]
    d = np.array([rot_y(v, tilt) for v in d])
    o = lens + off + d * 30.0
    dist, own = first_hits(o, d, exclude=("OAK-D",))
    hit = dist < 5000
    frac = hit.mean() * 100
    names = sorted({short(LABELS[k]) for k in own[hit]})
    # nearest visible floor point (ray reaches z = 0 before any robot part)
    t_floor = np.where(d[:, 2] < -1e-6, -(o[:, 2]) / d[:, 2], np.inf)
    vis_floor = t_floor < dist
    fp = o[vis_floor] + d[vis_floor] * t_floor[vis_floor][:, None]
    centre = vis_floor & (np.abs(H.reshape(-1)) <= np.abs(H).min() + 1e-9)
    fpc = o[centre] + d[centre] * t_floor[centre][:, None]
    nearest = (fpc[:, 0] - lens[0]).min() if len(fpc) else float("nan")      # on the centreline, straight ahead
    floor_min[name] = nearest
    lines.append(f"OAK {name:15s} FOV {hf:g} x {vf:g} deg: {frac:4.1f} % of rays hit the robot"
                 + (f" ({', '.join(names)})" if names else "") + f"; floor visible from {nearest:.0f} mm ahead of the lens (centreline)")
    img = hit.reshape(V.shape)
    ax.imshow(img[::-1], extent=[-hf / 2, hf / 2, -vf / 2, vf / 2], cmap="Reds", vmin=0, vmax=1.3)
    ax.set_title(f"{name}: {frac:.1f} % self-occluded", fontsize=10)
    ax.set_xlabel("horizontal deg (+ = left)")
    ax.set_ylabel("vertical deg (+ = up)")
fig.suptitle("OAK-D Pro W fields of view - red = ray hits the robot itself", fontsize=12)
fig.tight_layout()
fig.savefig(os.path.join(OUT, "oak_occlusion.png"), dpi=100)
plt.close(fig)
lines.append(f"  layout hand estimate: 454 mm; as-built depth camera: "
             f"{min(floor_min['mono L (depth)'], floor_min['mono R (depth)']):.0f} mm "
             f"(= {min(floor_min['mono L (depth)'], floor_min['mono R (depth)']) + lens[0] - G['BODY_L'] / 2 - G['BUMPER_T']:.0f} mm ahead of the bumper)")
lines.append("")

# ------------------------------------------------------------------ cliff sensors
a = math.radians(G["CLIFF_ANG"])
fig, ax = plt.subplots(figsize=(7, 5))
ax.add_patch(plt.Rectangle((-G["BODY_L"] / 2, -G["BODY_W"] / 2), G["BODY_L"], G["BODY_W"], fill=False, ec="#555"))
spot_x = {}
for sx, sy, nm in ((1, 1, "FL"), (1, -1, "FR"), (-1, 1, "RL"), (-1, -1, "RR")):
    o0 = np.array([sx * G["CLIFF_X"], sy * G["CLIFF_Y"], G["CLIFF_Z"]])     # sensor die (RV-307 cradle)
    axis = np.array([sx * math.sin(a), 0, -math.cos(a)])
    cone = []
    for th in np.radians(np.arange(0, 9.01, 1.5)):
        for ph in np.radians(np.arange(0, 360, 20 if th > 0 else 360)):
            u = np.array([math.sin(th) * math.cos(ph), math.sin(th) * math.sin(ph), math.cos(th)])
            # rotate (0,0,1)-based cone onto axis
            z = axis / np.linalg.norm(axis)
            xh = np.cross([0, 1, 0], z)
            xh /= np.linalg.norm(xh)
            yh = np.cross(z, xh)
            cone.append(u[0] * xh + u[1] * yh + u[2] * z)
    d = np.array(cone)
    o = o0 + d * 2.0
    dist, own = first_hits(o, d, exclude=("ToF",))
    t_floor = -o[:, 2] / d[:, 2]
    blocked = dist < t_floor
    spot = o + d * t_floor[:, None]
    names = sorted({short(LABELS[k]) for k in own[blocked]})
    spot_x[nm] = spot[:, 0].mean() * sx
    lead_caster = spot_x[nm] - (G["CASTER_X"] + 10 + G["CASTER_D"] / 2)
    lead_wheel = spot_x[nm] - G["WHEEL_D"] / 2
    lines.append(f"Cliff {nm}: sensor at ({o0[0]:.0f}, {o0[1]:.0f}, {o0[2]:.1f}) mm, {G['CLIFF_ANG']:g} deg from vertical, "
                 f"18 deg cone: {blocked.mean() * 100:.0f} % of rays blocked" + (f" by {', '.join(names)}" if names else "")
                 + f"; range {np.mean(t_floor):.0f} mm; floor spot x {spot[:, 0].min():.0f}..{spot[:, 0].max():.0f}, "
                 f"y {spot[:, 1].min():.0f}..{spot[:, 1].max():.0f}; lead over caster tyre (swivelled outward) "
                 f"{lead_caster:.0f} mm, over drive-wheel contact edge {lead_wheel:.0f} mm")
    ax.scatter(spot[:, 0], spot[:, 1], s=6, color="#BC3126" if blocked.any() else "#1B6E3A")
for sy in (1, -1):
    ax.add_patch(plt.Rectangle((-G["WHEEL_D"] / 2, sy * G["TRACK"] / 2 - G["WHEEL_W"] / 2), G["WHEEL_D"], G["WHEEL_W"],
                               color="#222", alpha=0.6))
for sx in (1, -1):
    ax.add_patch(plt.Circle((sx * G["CASTER_X"], 0), 35, fill=False, ec="#888", ls="--"))
ax.set_aspect("equal")
ax.set_title("Cliff-sensor floor spots (green = clear) vs wheels and caster sweep (dashed)")
ax.set_xlabel("x fwd (mm)")
ax.set_ylabel("y left (mm)")
fig.savefig(os.path.join(OUT, "cliff_spots.png"), dpi=110, bbox_inches="tight")
plt.close(fig)
tyre = G["CASTER_X"] - 10 + G["CASTER_D"] / 2            # leading tyre edge, caster trailing (straight driving)
fwd, rev = min(spot_x["FL"], spot_x["FR"]) - tyre, min(spot_x["RL"], spot_x["RR"]) - tyre
lines.append(f"  caster trailing (straight driving): floor spot is {fwd:.0f} mm ahead of the front caster tyre, "
             f"{rev:.0f} mm behind the rear caster tyre")
# cliff = hard stop in the ESP32 firmware (not the Nav2 smoother): 1 m/s^2, 50 ms sensor + loop latency
for v, lead, d in ((0.3, fwd, "forward"), (0.5, fwd, "forward"), (0.3, rev, "reverse")):
    need = v * v / (2 * 1.0) * 1000 + v * 0.05 * 1000
    lines.append(f"  {d} {v} m/s: stop at 1 m/s^2 with 50 ms latency needs {need:.0f} mm -> "
                 f"{'OK' if lead >= need else 'NOT COVERED'} (margin {lead - need:+.0f} mm)")
lines.append("")

# ------------------------------------------------------------------ docking tag line of sight
tx = G["LENS_X"] + G["DOCK_TAG_DIST"]
s2, tz = G["DOCK_TAG_SIZE"] / 2, G["DOCK_TAG_Z"]
pts = np.array([[tx, y, z] for y in (-s2, 0, s2) for z in (tz - s2, tz, tz + s2)])
o = np.repeat(lens[None, :], len(pts), 0)
d = pts - o
L = np.linalg.norm(d, axis=1)
d = d / L[:, None]
dist, own = first_hits(o + d * 30, d, exclude=("OAK-D",))
clear = dist + 30 > L - 1
ang_v = np.degrees(np.arctan2(pts[:, 2] - lens[2], pts[:, 0] - lens[0])) + G["OAK_TILT"]
ang_h = np.degrees(np.arctan2(pts[:, 1], pts[:, 0] - lens[0]))
lines.append(f"Docking tag ({G['DOCK_TAG_SIZE']:.0f} mm at x {tx:.0f}, z {tz:.0f}) seen from the lens when docked: "
             f"{'all 9 sample points CLEAR' if clear.all() else str((~clear).sum()) + ' points blocked'}; "
             f"tag spans {ang_v.min():.1f}..{ang_v.max():.1f} deg vertically and {ang_h.min():.1f}..{ang_h.max():.1f} deg "
             f"horizontally from the optical axis (mono FOV +-39.75 x +-63.5); range {L.min():.0f}..{L.max():.0f} mm "
             f"(mono fixed focus from 196 mm)")

open(os.path.join(OUT, "fov_report.txt"), "w", encoding="utf-8").write("\n".join(lines) + "\n")
print("\n".join(lines))
