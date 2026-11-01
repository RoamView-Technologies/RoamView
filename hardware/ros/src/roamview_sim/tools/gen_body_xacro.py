#!/usr/bin/env python3
"""Generate urdf/roamview_body.xacro (the visual shapes) from the Rev B interconnect model.

    python3 tools/gen_body_xacro.py <path to old-designs/Interconnect_RevB>

Every box and lathe in model_b.py becomes a URDF box or cylinder in the frame of the link that carries it,
so RViz and Gazebo show the same envelopes as the SOLIDWORKS Rev B model. Collision shapes and masses are
written by hand in roamview.urdf.xacro, not here.
Model frame: +X forward, +Y up, +Z right, mm, origin = floor under the axle.  ROS: x = X, y = -Z, z = Y, metres.
"""
import math
import sys
from pathlib import Path

model_dir = Path(sys.argv[1] if len(sys.argv) > 1 else ".").resolve()
sys.path.insert(0, str(model_dir))
import model_b as M  # noqa: E402

AXLE_H = M.AXLE_Y / 1000
PIVOT = (M.PIVOT[0] / 1000, M.PIVOT[1] / 1000)
FRAMES = {  # link -> (origin in base_footprint, yaw)
    "base": ((0.0, 0.0, AXLE_H), 0.0),
    "arm_left": ((PIVOT[0], 0.0885, PIVOT[1]), 0.0),
    "arm_right": ((PIVOT[0], -0.0885, PIVOT[1]), 0.0),
    "wheel_left": ((0.0, M.TRACK / 2000, AXLE_H), 0.0),
    "wheel_right": ((0.0, -M.TRACK / 2000, AXLE_H), 0.0),
    "fork_front": ((M.CASTER_X / 1000, 0.0, M.CASTER_H / 1000), 0.0),
    "fork_rear": ((-M.CASTER_X / 1000, 0.0, M.CASTER_H / 1000), math.pi),
    "cwheel_front": (((M.CASTER_X - M.CASTER_TRAIL) / 1000, 0.0, M.CASTER_WD / 2000), 0.0),
    "cwheel_rear": ((-(M.CASTER_X - M.CASTER_TRAIL) / 1000, 0.0, M.CASTER_WD / 2000), math.pi),
    "bumper_left": ((0.0, 0.0, 0.0), 0.0),
    "bumper_right": ((0.0, 0.0, 0.0), 0.0),
}


def ros(p):
    return (p[0] / 1000, -p[2] / 1000, p[1] / 1000)


def local(frame, p):
    (ox, oy, oz), yaw = FRAMES[frame]
    dx, dy, dz = p[0] - ox, p[1] - oy, p[2] - oz
    c, s = math.cos(-yaw), math.sin(-yaw)
    return (c * dx - s * dy, s * dx + c * dy, dz)


def boxes_of(pr):
    lo, hi = pr["min"], pr["max"]
    a, b = ros(lo), ros(hi)
    centre = tuple((u + v) / 2 for u, v in zip(a, b))
    size = tuple(abs(u - v) for u, v in zip(a, b))
    return [("box", centre, size)]


def cyls_of(pr):
    """Lathe -> cylinders, one per stretch of constant outer radius; inner stretches are dropped later."""
    ax, o, prof = pr["axis"], pr["origin"], pr["profile"]
    segs = []
    for (r0, a0), (r1, a1) in zip(prof, prof[1:] + prof[:1]):
        if r0 == r1 and r0 > 0 and a0 != a1:
            segs.append((r0, min(a0, a1), max(a0, a1)))
    if not segs:  # e.g. the zig-zag spring: one cylinder at the largest radius
        rs = [p[0] for p in prof]; as_ = [p[1] for p in prof]
        segs = [(max(rs), min(as_), max(as_))]
    # drop stretches that sit inside a larger one
    keep = [s for s in segs if not any(t is not s and t[0] >= s[0] and t[1] <= s[1] and t[2] >= s[2]
                                       and (t[0], t[1], t[2]) != (s[0], s[1], s[2]) for t in segs)]
    out = []
    idx = {"x": 0, "y": 1, "z": 2}[ax]
    for r, a0, a1 in keep:
        c = list(o); c[idx] += (a0 + a1) / 2
        out.append(("cyl_" + ax, ros(c), (r / 1000, (a1 - a0) / 1000)))
    return out


def shapes_of(pr):
    if pr["t"] == "box":
        return boxes_of(pr)
    if pr["t"] == "lathe":
        return cyls_of(pr)
    if pr["t"] == "p2020":
        x, y, z = pr["origin"]
        return boxes_of(dict(min=[x - 10, y, z - 10], max=[x + 10, y + pr["h"], z + 10]))
    raise ValueError(pr["t"])


def assign(pid, pr):
    """Which link frame carries this prim."""
    if pid in ("wheel_l", "wheel_r"):
        return "wheel_left" if pid.endswith("l") else "wheel_right"
    if pid in ("motor_l", "motor_r"):
        return "arm_left" if pid.endswith("l") else "arm_right"
    if pid in ("caster_f", "caster_r"):
        end = "front" if pid.endswith("f") else "rear"
        nm = pr.get("name") or ""
        if nm.startswith("Fork"):
            return "fork_" + end
        if nm.startswith(("Wheel", "Axle")):
            return "cwheel_" + end
        return "base"
    if pid == "bumper":
        zc = (pr["min"][2] + pr["max"][2]) / 2
        return "bumper_left" if zc < 0 else "bumper_right"   # model Z < 0 is the robot's left
    return "base"


colors, visuals = {}, {k: {} for k in FRAMES}


def mat(rgb):
    key = tuple(round(c, 3) for c in rgb)
    if key not in colors:
        colors[key] = f"rv_c{len(colors)}"
    return colors[key]


def add(frame, kind, centre, size, rgb, label):
    x, y, z = local(frame, centre)
    yaw = FRAMES[frame][1]
    if kind == "box":
        sx, sy, sz = size
        geom = f'<box size="{sx:.4f} {sy:.4f} {sz:.4f}"/>'
        rpy = "0 0 0"
    else:
        r, length = size
        geom = f'<cylinder radius="{r:.4f}" length="{length:.4f}"/>'
        rpy = {"cyl_y": "0 0 0", "cyl_x": "0 1.5708 0", "cyl_z": "1.5708 0 0"}[kind]  # model axis -> ROS
    name = mat(rgb)
    visuals[frame].setdefault(name, (rgb, []))[1].append(
        f'      <visual name="{label}"><origin xyz="{x:.4f} {y:.4f} {z:.4f}" rpy="{rpy}"/>'
        f'<geometry>{geom}</geometry><material name="{name}"><color rgba="{rgb[0]} {rgb[1]} {rgb[2]} 1"/>'
        '</material></visual>')
    del yaw


def slug(s):
    return "".join(ch if ch.isalnum() else "_" for ch in s.lower()).strip("_")[:40]


# decks: the lower deck has the two wheel wells, so it is five boxes; the top deck is one
plate_rgb = M.plates()[0].get("color") or [0.78, 0.8, 0.85]
L, W, WX, WZ = M.PLATE_L / 2, M.PLATE_W / 2, M.WELL_HALF, M.WELL_Z
lower = [(-L, L, -WZ, WZ), (WX, L, WZ, W), (-L, -WX, WZ, W), (WX, L, -W, -WZ), (-L, -WX, -W, -WZ)]
for i, (x0, x1, z0, z1) in enumerate(lower):
    for kind, c, s in boxes_of(dict(min=[x0, M.BASE_Y, z0], max=[x1, M.BASE_TOP, z1])):
        add("base", kind, c, s, plate_rgb, f"lower_deck_{i}")
for kind, c, s in boxes_of(dict(min=[-L, M.DECK_Y, -W], max=[L, M.DECK_TOP, W])):
    add("base", kind, c, s, plate_rgb, "top_deck")

n = 0
for p in M.PARTS:
    if p.get("hidden"):
        continue
    for pr in p["prims"]:
        frame = assign(p["id"], pr)
        for kind, c, s in shapes_of(pr):
            if min(s) <= 0:
                continue
            n += 1
            add(frame, kind, c, s, pr.get("color") or [0.6, 0.6, 0.6], f"{p['id']}_{slug(pr.get('name') or 'body')}_{n}")

out = ['<?xml version="1.0"?>',
       "<!-- GENERATED by tools/gen_body_xacro.py from old-designs/Interconnect_RevB/model_b.py. Do not edit by hand.",
       "     RViz colours a whole link with one material, so each colour gets its own small link, fixed to the",
       "     link that carries it (Gazebo merges them back into that link). -->",
       '<robot xmlns:xacro="http://www.ros.org/wiki/xacro">']
for frame, groups in visuals.items():
    out.append(f'  <xacro:macro name="rv_visual_links_{frame}" params="parent">')
    for name, (rgb, vis) in groups.items():
        link = f"vis_{frame}_{name[3:]}"
        out.append(f'    <joint name="{link}_joint" type="fixed"><parent link="${{parent}}"/><child link="{link}"/>'
                   '<origin xyz="0 0 0" rpy="0 0 0"/></joint>')
        out.append(f'    <link name="{link}">')
        out += vis
        out.append("    </link>")
    out.append("  </xacro:macro>")
out.append("</robot>")
dst = Path(__file__).resolve().parent.parent / "urdf" / "roamview_body.xacro"
dst.write_text("\n".join(out) + "\n")
nv = sum(len(v) for g in visuals.values() for _, v in g.values())
nl = sum(len(g) for g in visuals.values())
print(f"wrote {dst} ({nv} visuals in {nl} colour links, {len(colors)} colours)")
for k, g in visuals.items():
    print(f"  {k:13s} {sum(len(v) for _, v in g.values()):4d} visuals, {len(g)} links")
