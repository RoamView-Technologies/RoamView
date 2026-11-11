"""Presentation renders of the full RV-100 assembly: still images, a turntable video and a 3D model.

    python presentation_views.py            # everything, into Simulation/Presentation
    python presentation_views.py --no-video # skip the turntable

Writes:
    RV-100_hero_front.png, RV-100_hero_rear.png     4K, transparent background
    RV-100_labelled_front.png, _rear.png            4K, call-outs with leader lines, white background
    RV-100_exploded.png                             4K, exploded by module, transparent background
    RV-100_views.png                                front, side, top and rear orthographic views with overall sizes
    RV-100_turntable.mp4                            1080p, 12 s, one turn
    RV-100_RoamView.glb                             3D model for PowerPoint (Insert > 3D Models), Y up, front toward +Z

Geometry is the per-component STLs from export_fov_stl.py, so it matches the SolidWorks assembly.
Needs numpy, trimesh, pyvista, pillow and imageio-ffmpeg (the %LOCALAPPDATA%\\rvcv venv has them).
"""
import argparse
import glob
import math
import os
import sys

import numpy as np
import pyvista as pv
import trimesh
from PIL import Image, ImageDraw, ImageFont
import vtk

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from export_urdf import ROOT, STL_DIR, SW_TO_ROS, parse_name  # noqa: E402
from bay_video import COLOUR, hex_rgb, translate, cam_at  # noqa: E402

OUT = os.path.join(ROOT, "Simulation", "Presentation")
METAL = {"RV-410", "RV-420", "RV-430", "RV-431", "RV-432", "RV-433", "RV-434", "RV-435", "RV-502"}


def load_parts():
    parts = []
    for path in sorted(glob.glob(os.path.join(STL_DIR, "RV-100 - *.STL"))):
        pn, module = parse_name(path)
        if pn not in COLOUR:
            continue
        tm = trimesh.load(path, force="mesh")
        tm.apply_transform(SW_TO_ROS)
        parts.append(dict(pn=pn, module=module.split("_")[0], tm=tm, mesh=pv.wrap(tm),
                          centre=tm.bounds.mean(axis=0), rgb=hex_rgb(COLOUR[pn])))
    return parts


def explode(p):
    """Offset (m) of a part in the exploded view."""
    pn, mod, c = p["pn"], p["module"], p["centre"]
    sx, sy = np.sign(c[0]), np.sign(c[1])
    if mod == "RV-140":
        return (0, 0, 0.42 if pn not in ("RV-420",) else 0.30)
    if pn in ("RV-430", "RV-551") or mod == "RV-170":
        return (0, 0, 0.26)
    if pn == "RV-313":
        return (0, 0.22 * sy, 0)
    if pn == "RV-314":
        return (0.20, 0, 0)
    if pn == "RV-305":
        return (0.34, 0, 0)
    if pn == "RV-315":
        return (-0.26, 0, 0)
    if mod == "RV-130":
        return (-0.48, 0, 0)
    if mod == "RV-110":
        return (0, 0.28 * sy, 0)
    if mod == "RV-120":
        return (0.05, 0, -0.10)
    if mod == "RV-125":
        return (-0.05, 0, -0.10)
    if mod in ("RV-150", "RV-160") or pn == "RV-532":
        return (0, 0, 0.12)
    if mod == "RV-180":
        return (0.12 * sx, 0.04 * sy, 0)
    if mod in ("RV-182", "RV-185"):
        return (0.27, 0, 0)
    if pn == "RV-433":
        return (-0.10, 0, 0)
    if pn == "RV-431":
        return (0, 0, -0.04)
    return (0, 0, 0)


def plotter(size, transparent=True, ortho=False):
    pl = pv.Plotter(off_screen=True, window_size=size)
    pl.set_background("white")
    pl.enable_lightkit()
    if ortho:
        pl.enable_parallel_projection()
    return pl


def add_parts(pl, parts, offsets=None, ghost=False):
    for p in parts:
        m = p["mesh"]
        if offsets is not None:
            m = m.translate(offsets(p), inplace=False)
        metal = p["pn"] in METAL
        if ghost:
            pl.add_mesh(m, color="#9aa3ad", opacity=0.10, smooth_shading=True, ambient=0.3, diffuse=0.5)
            continue
        pl.add_mesh(m, color=p["rgb"], smooth_shading=True, split_sharp_edges=True, feature_angle=35,
                    ambient=0.18, diffuse=0.78, specular=0.45 if metal else 0.18, specular_power=30 if metal else 12)


def finish(pl, path, transparent=True, pad=40):
    pl.enable_anti_aliasing("ssaa")
    img = pl.screenshot(None, transparent_background=transparent, return_img=True)
    pl.close()
    im = Image.fromarray(img)
    if transparent:                                            # crop to the robot with a small margin
        bbox = im.getchannel("A").getbbox()
        if bbox:
            x0, y0, x1, y1 = bbox
            im = im.crop((max(0, x0 - pad), max(0, y0 - pad), min(im.width, x1 + pad), min(im.height, y1 + pad)))
    im.save(path)
    print("wrote", os.path.basename(path), im.size)


def hero(parts, path, az, elev, size=(3840, 2160)):
    pl = plotter(size)
    add_parts(pl, parts)
    pos, foc = cam_at((0.0, 0.0, 0.25), 2.0, elev, az)
    pl.camera.position, pl.camera.focal_point, pl.camera.up = pos, foc, (0, 0, 1)
    pl.camera.view_angle = 24
    finish(pl, path)


def exploded(parts, path, size=(3840, 2400)):
    pl = plotter(size)
    add_parts(pl, parts, explode)
    pos, foc = cam_at((-0.04, 0.0, 0.30), 2.9, 22, 38)
    pl.camera.position, pl.camera.focal_point, pl.camera.up = pos, foc, (0, 0, 1)
    pl.camera.view_angle = 26
    finish(pl, path)


CALLOUTS_FRONT = [   # (part number, which instance: +1/-1 on y or None, title, detail)
    ("RV-552", None, "7 in touchscreen", "Raspberry Pi Touch Display 2, RV-552"),
    ("RV-511", None, "Depth camera, tilted 5° down", "Luxonis OAK-D Pro W, RV-511"),
    ("RV-420", None, "Sensor tower", "20 mm aluminium T-slot, RV-420"),
    ("RV-512", None, "360° LiDAR", "LDROBOT STL-19P on printed pedestal, RV-512"),
    ("RV-430", None, "Deck plate", "Aluminium, hole grid for add-ons, RV-430"),
    ("RV-313", 1, "Body skins", "Printed PETG over a 2020 frame, RV-313/314/315"),
    ("RV-503", 1, "Drive module", "70:1 gearmotor inside a 144 mm wheel, RV-110"),
    ("RV-305", None, "Bumper", "TPU, presses two bump switches, RV-305"),
    ("RV-513", 1, "Cliff sensor", "ToF, one at each corner, RV-513"),
    ("RV-306", None, "Charge pads", "Brass, meet the dock contacts, RV-435"),
]
CALLOUTS_REAR = [
    ("RV-552", None, "7 in touchscreen", "Tilted up toward the user, RV-552"),
    ("RV-551", None, "Emergency stop", "Cuts motor power, RV-551"),
    ("RV-512", None, "360° LiDAR", "LDROBOT STL-19P, RV-512"),
    ("RV-316", None, "Battery drawer", "RV-130"),
    ("RV-315", None, "Rear skin", "Printed PETG, RV-315"),
    ("RV-503", -1, "Drive module", "Track 365 mm, RV-110"),
    ("RV-513", -1, "Rear cliff sensor", "ToF, RV-513"),
]


def font(size, bold=False):
    for name in (("segoeuib.ttf" if bold else "segoeui.ttf"), ("arialbd.ttf" if bold else "arial.ttf")):
        try:
            return ImageFont.truetype(os.path.join(os.environ.get("WINDIR", "C:/Windows"), "Fonts", name), size)
        except OSError:
            pass
    return ImageFont.load_default()


def labelled(parts, path, callouts, az, elev, size=(3840, 2160), focal=(0.0, 0.0, 0.25), dist=2.25, ghost=None,
             offsets=None, dims=(), extra=None, legend=()):
    """Robot in the middle, call-outs in two columns with leader lines to a visible point on each part."""
    W, H = size
    f1, f2 = font(54, True), font(38)
    meas = ImageDraw.Draw(Image.new("RGB", (8, 8)))
    tw = {t: max(meas.textlength(t, font=f1), meas.textlength(d, font=f2)) for _, _, t, d in callouts}

    def render(dist):
        pl = plotter(size)
        if ghost:
            pl.enable_depth_peeling(number_of_peels=6)
            add_parts(pl, ghost, ghost=True)
        add_parts(pl, parts, offsets)
        if extra:
            extra(pl)
        pos, foc = cam_at(focal, dist, elev, az)
        pl.camera.position, pl.camera.focal_point, pl.camera.up = pos, foc, (0, 0, 1)
        pl.camera.view_angle = 24
        pl.enable_anti_aliasing("ssaa")
        img = Image.fromarray(pl.screenshot(None, return_img=True)).convert("RGB")
        cam_pos = np.array(pos)
        coord = vtk.vtkCoordinate()
        coord.SetCoordinateSystemToWorld()
        anchors = []
        for pn, side, title, detail in callouts:
            if not isinstance(pn, str):                         # an explicit point, e.g. on a wire
                coord.SetValue(*pn)
                x, y = coord.GetComputedDoubleDisplayValue(pl.renderer)
                anchors.append(dict(x=x, y=H - y, title=title, detail=detail))
                continue
            cands = [p for p in parts if p["pn"] == pn and (side is None or np.sign(p["centre"][1]) == side)]
            best = None
            for p in cands:
                off = np.asarray(offsets(p) if offsets else (0, 0, 0), float)
                v = p["tm"].vertices + off
                k = int(np.argmin(np.linalg.norm(v - cam_pos, axis=1)))
                pt = v[k] + 0.25 * (p["centre"] + off - v[k])       # a little in from the nearest corner
                d = np.linalg.norm(pt - cam_pos)
                if best is None or d < best[0]:
                    best = (d, pt)
            coord.SetValue(*best[1])
            x, y = coord.GetComputedDoubleDisplayValue(pl.renderer)
            anchors.append(dict(x=x, y=H - y, title=title, detail=detail))
        dim_px = []
        for a3, b3, txt in dims:
            ends = []
            for q in (a3, b3):
                coord.SetValue(*q)
                x, y = coord.GetComputedDoubleDisplayValue(pl.renderer)
                ends.append((x, H - y))
            dim_px.append((ends, txt))
        pl.close()
        return img, anchors, dim_px

    for _ in range(6):                     # pull the camera back until the label columns clear the geometry
        img, anchors, dim_px = render(dist)
        dark = np.asarray(img).min(axis=2) < 240
        ink = np.argwhere(dark.any(axis=0))
        rows = np.argwhere(dark.any(axis=1))
        if len(rows) and (rows.min() < 30 or rows.max() > H - 30 - 90 * bool(legend)):
            dist *= 1.15
            continue
        gx0, gx1 = (int(ink.min()), int(ink.max())) if len(ink) else (0, W)
        gc = (gx0 + gx1) / 2
        lw = max([tw[a["title"]] for a in anchors if a["x"] < gc] or [0])
        rw = max([tw[a["title"]] for a in anchors if a["x"] >= gc] or [0])
        lo, hi = 110 + lw + 110, W - 110 - rw - 110
        if gx1 - gx0 <= hi - lo:
            shift = int((lo + hi) / 2 - gc)
            moved = Image.new("RGB", img.size, "white")
            moved.paste(img, (shift, 0))
            img = moved
            for a in anchors:
                a["x"] += shift
            dim_px = [([(x + shift, y) for x, y in ends], t) for ends, t in dim_px]
            cx = gc + shift
            break
        dist *= (gx1 - gx0) / (hi - lo) * 1.04
    else:
        cx = W / 2
    draw = ImageDraw.Draw(img)
    for (pa, pb), txt in dim_px:
        draw.line([pa, pb], fill="#2563eb", width=5)
        d = np.subtract(pb, pa) / max(1e-6, np.hypot(*np.subtract(pb, pa)))
        n = np.array([-d[1], d[0]])
        for q, sgn in ((pa, 1), (pb, -1)):                     # arrow heads
            tip = np.array(q)
            draw.polygon([tuple(tip), tuple(tip + sgn * 34 * d + 13 * n), tuple(tip + sgn * 34 * d - 13 * n)],
                         fill="#2563eb")
        mid = (np.array(pa) + np.array(pb)) / 2
        fd = font(46, True)
        tw = draw.textlength(txt, font=fd)
        draw.rectangle([mid[0] - tw / 2 - 14, mid[1] + 14, mid[0] + tw / 2 + 14, mid[1] + 80], fill="white")
        draw.text((mid[0] - tw / 2, mid[1] + 18), txt, font=fd, fill="#2563eb")
    gap = 128
    for left in (True, False):
        items = sorted([a for a in anchors if (a["x"] < cx) == left], key=lambda a: a["y"])
        ys = []
        for a in items:                                       # keep labels near their anchor, no overlaps
            y = max(a["y"] - 40, (ys[-1] + gap) if ys else 90)
            ys.append(y)
        over = (ys[-1] + gap) - (H - 60) if ys else 0
        if over > 0:
            ys = [y - over for y in ys]
        col_w = max([max(draw.textlength(a["title"], font=f1), draw.textlength(a["detail"], font=f2))
                     for a in items] or [0])
        for a, y in zip(items, ys):                           # one aligned column per side
            w = max(draw.textlength(a["title"], font=f1), draw.textlength(a["detail"], font=f2))
            x0 = 110 if left else W - 110 - col_w
            w = w if left else col_w
            draw.text((x0, y), a["title"], font=f1, fill="#111827")
            draw.text((x0, y + 62), a["detail"], font=f2, fill="#4b5563")
            ex = x0 + w + 30 if left else x0 - 30
            ey = y + 40
            knee = ex + (60 if left else -60)
            draw.line([(ex, ey), (knee, ey), (a["x"], a["y"])], fill="#374151", width=5, joint="curve")
            r = 13
            draw.ellipse([a["x"] - r, a["y"] - r, a["x"] + r, a["y"] + r], fill="#111827", outline="white", width=4)
    if legend:                                                # wire colour key, bottom left
        fl = font(40)
        y0 = H - 90 - 74 * len(legend)
        draw.text((110, y0 - 70), "Wiring", font=font(46, True), fill="#111827")
        for i, (cols, text) in enumerate(legend):
            y = y0 + 74 * i
            for k, c in enumerate(cols):
                draw.rounded_rectangle([110 + k * 30, y + 8, 110 + k * 30 + 22, y + 44], radius=5, fill=c,
                                       outline="#9ca3af", width=2)
            draw.text((110 + 30 * 6 + 20, y + 2), text, font=fl, fill="#374151")
    img.save(path)
    print("wrote", os.path.basename(path), img.size)


def ortho_views(parts, path):
    """Four orthographic views on one sheet, each with its overall size."""
    V = np.vstack([p["tm"].vertices for p in parts])
    lo, hi = V.min(axis=0), V.max(axis=0)
    L, W, H = (hi - lo) * 1000
    views = [("Front", (1, 0, 0), (0, 0, 1), f"width {W:.0f} mm"),
             ("Side (left)", (0, 1, 0), (0, 0, 1), f"length {L:.0f} mm, height {H:.0f} mm"),
             ("Top", (0, 0, 1), (1, 0, 0), f"{L:.0f} x {W:.0f} mm"),
             ("Rear", (-1, 0, 0), (0, 0, 1), "battery drawer face")]
    tiles = []
    c = (lo + hi) / 2
    for name, d, up, note in views:
        pl = plotter((1600, 1600), ortho=True)
        add_parts(pl, parts)
        d = np.array(d, float)
        pl.camera.focal_point = c
        pl.camera.position = c + d * 3
        pl.camera.up = up
        pl.camera.parallel_scale = 0.31
        pl.add_text(name, position=(40, 1520), font_size=26, color="#111827", font="arial")
        pl.add_text(note, position=(40, 40), font_size=18, color="#4b5563", font="arial")
        pl.enable_anti_aliasing("ssaa")
        tiles.append(Image.fromarray(pl.screenshot(None, return_img=True)))
        pl.close()
    sheet = Image.new("RGB", (3200, 3200), "white")
    for i, t in enumerate(tiles):
        sheet.paste(t, ((i % 2) * 1600, (i // 2) * 1600))
    sheet.save(path)
    print("wrote", os.path.basename(path), sheet.size, f"overall {L:.0f} x {W:.0f} x {H:.0f} mm")


def turntable(parts, path, seconds=12, fps=30, size=(1920, 1080)):
    pl = plotter(size)
    add_parts(pl, parts)
    pl.add_mesh(pv.Disc(center=(0, 0, -0.0005), inner=0, outer=0.42, c_res=120), color="#eceff3", ambient=0.4)
    pl.enable_anti_aliasing("ssaa")
    pl.open_movie(path, framerate=fps, quality=9, macro_block_size=8)
    n = seconds * fps
    for k in range(n):
        pos, foc = cam_at((0.0, 0.0, 0.24), 2.1, 18, 35 + 360 * k / n)
        pl.camera.position, pl.camera.focal_point, pl.camera.up = pos, foc, (0, 0, 1)
        pl.camera.view_angle = 24
        pl.write_frame()
    pl.close()
    print("wrote", os.path.basename(path), f"{n} frames")


def glb(parts, path):
    """One mesh per colour, PBR materials, glTF axes (Y up), robot front toward +Z."""
    to_gltf = np.array([[0, 1, 0, 0], [0, 0, 1, 0], [1, 0, 0, 0], [0, 0, 0, 1.0]])   # ROS x,y,z -> glTF z,x,y
    scene = trimesh.Scene()
    by_colour = {}
    for p in parts:
        tm = p["tm"]
        if len(tm.faces) > 80000:                              # vendor wheels: thin out the tread for PowerPoint
            d = pv.wrap(tm).triangulate().decimate(1 - 80000 / len(tm.faces))
            tm = trimesh.Trimesh(d.points, d.faces.reshape(-1, 4)[:, 1:], process=False)
        by_colour.setdefault((COLOUR[p["pn"]], p["pn"] in METAL), []).append(tm)
    for (hexc, metal), meshes in by_colour.items():
        m = trimesh.util.concatenate(meshes)
        m.apply_transform(to_gltf)
        rgb = list((hex_rgb(hexc) * 255).astype(int)) + [255]
        m.visual = trimesh.visual.TextureVisuals(material=trimesh.visual.material.PBRMaterial(
            baseColorFactor=rgb, metallicFactor=0.6 if metal else 0.0, roughnessFactor=0.45 if metal else 0.7))
        scene.add_geometry(m, node_name=f"RV100_{hexc[1:]}")
    scene.export(path)
    print("wrote", os.path.basename(path), f"{os.path.getsize(path) / 1e6:.1f} MB")


DRIVE = ("RV-110", "RV-160")                 # drive modules and the motor driver bay
RUNNING = ("RV-110", "RV-120", "RV-125", "RV-160")

# Wiring of the drivetrain, routed by hand through the bay (mm, robot frame). The DFR0601 terminal positions
# are approximate: motor outputs on the edge facing the axle, supply on the outboard edge, logic header at the front.
WIRES = [
    # name, waypoints (mm), wire colours, radius (mm), spacing (mm)
    ("m1", [(62, 112, 53), (48, 112, 66), (24, 106, 84), (6, 101, 84), (0, 101, 82)],
     ("#d62828", "#111111"), 1.3, 2.8),
    ("m2", [(62, 88, 53), (48, 70, 74), (30, 30, 90), (14, -40, 90), (4, -96, 84), (0, -101, 82)],
     ("#d62828", "#111111"), 1.3, 2.8),
    ("enc1", [(0, 101, 74), (-12, 86, 74), (-26, 40, 74), (-26, 0, 70), (-25, -14, 64.5)],
     ("#16a34a", "#2563eb", "#eab308", "#f5f5f5"), 0.65, 1.4),
    ("enc2", [(0, -101, 74), (-12, -86, 74), (-24, -58, 70), (-25, -44, 64.5)],
     ("#16a34a", "#2563eb", "#eab308", "#f5f5f5"), 0.65, 1.4),
    ("ctrl", [(-23, -26, 64.5), (-4, -4, 78), (40, 40, 76), (96, 72, 66), (108, 92, 56)],
     ("#f97316", "#a855f7", "#0ea5e9", "#f97316", "#a855f7", "#0ea5e9"), 0.6, 1.3),
    ("supply", [(85, 124, 53), (82, 132, 72), (60, 134, 100), (30, 128, 122)],
     ("#d62828", "#111111"), 1.8, 3.8),
]


LEGEND = [
    (("#d62828", "#111111"), "Motor leads M1 and M2, driver to each motor"),
    (("#16a34a", "#2563eb", "#eab308", "#f5f5f5"), "Encoder cables, motor to ESP32 (GND, 3.3 V, A, B)"),
    (("#f97316", "#a855f7", "#0ea5e9"), "Control signals, ESP32 to driver (PWM, INA, INB per motor)"),
    (("#d62828", "#111111"), "Battery supply"),
]


def wire_point(name, frac):
    """A point part-way along a wire route (m), for call-out anchors."""
    pts = np.array(next(w[1] for w in WIRES if w[0] == name), float) / 1000
    sp = pv.Spline(pts, 200).points
    return tuple(sp[int(frac * (len(sp) - 1))])


def add_wires(pl):
    for name, pts, colours, r, gap in WIRES:
        P = np.array(pts, float) / 1000
        spl = pv.Spline(P, 240).points
        t = np.gradient(spl, axis=0)
        t /= np.linalg.norm(t, axis=1)[:, None]
        n1 = np.cross(t, [0, 0, 1])
        n1 /= np.maximum(np.linalg.norm(n1, axis=1)[:, None], 1e-9)
        n2 = np.cross(t, n1)
        k = len(colours)
        cols = 2 if k > 2 else k
        rows = math.ceil(k / cols)
        for i, c in enumerate(colours):                       # wires side by side, two rows for bundles
            a = (i % cols - (cols - 1) / 2) * gap / 1000
            b = ((i // cols) - (rows - 1) / 2) * gap / 1000
            line = pv.lines_from_points(spl + a * n1 + b * n2)
            pl.add_mesh(line.tube(radius=r / 1000, n_sides=14), color=c, smooth_shading=True, ambient=0.25,
                        specular=0.35)


def drivetrain(parts, o):
    drive = [p for p in parts if p["module"] in DRIVE or p["pn"] == "RV-522"]
    tr = 0.1825
    wiring_callouts = [
        ("RV-531", None, "Motor driver", "DFRobot DFR0601, 2 x 12 A, on printed mount RV-312"),
        ("RV-522", None, "ESP32 controller", "Closes the wheel speed loop, RV-522"),
        (wire_point("m1", 0.45), None, "Motor leads, left (M1)", "Red and black, screw terminals"),
        (wire_point("m2", 0.55), None, "Motor leads, right (M2)", "Red and black, screw terminals"),
        (wire_point("enc1", 0.5), None, "Encoder cable", "A, B, 3.3 V, GND to ESP32 interrupt pins"),
        (wire_point("ctrl", 0.55), None, "Control signals", "PWM, INA, INB per motor, 3.3 V logic"),
        (wire_point("supply", 0.9), None, "Battery supply", "Battery Pack"),
    ]
    labelled(drive, o("RV-100_drivetrain.png"), [
        ("RV-503", 1, "Left wheel", "RV-503"),
        ("RV-501", -1, "Gearmotor with encoder", "Pololu 37D 70:1, 150 rpm, 64 CPR, RV-501"),
        ("RV-434", -1, "Motor bracket", "3 mm 5052 aluminium, safety factor 3.4, RV-434"),
        ("RV-503", -1, "Right wheel", "Mirror of the left module, RV-110"),
        wiring_callouts[0], wiring_callouts[1],
    ], az=-30, elev=42, focal=(0.02, 0.0, 0.07), dist=1.0, extra=add_wires, legend=LEGEND,
        dims=[((0.0, -tr, 0.0), (0.0, tr, 0.0), "track 365 mm")])

    labelled(drive, o("RV-100_drivetrain_top.png"), wiring_callouts + [
        ("RV-501", -1, "Gearmotor (right)", "Leads leave from the encoder end"),
        ("RV-503", 1, "Left wheel", "144 mm"),
    ], az=-90, elev=88, focal=(0.02, 0.0, 0.07), dist=1.0, extra=add_wires)

    left = [p for p in parts if p["module"] == "RV-110" and p["centre"][1] > 0]
    step = {"RV-501": -0.12, "RV-434": 0.0, "RV-502": 0.09, "RV-503": 0.20}
    labelled(left, o("RV-100_drivetrain_exploded.png"), [
        ("RV-501", None, "Gearmotor", "Pololu 37D 70:1, 12 V, encoder on the back, RV-501"),
        ("RV-434", None, "Motor bracket", "Bent 5052 sheet, bolts to the frame rail, RV-434"),
        ("RV-502", None, "Hub", "goBILDA 1309 sonic hub, 6 mm D-shaft, RV-502"),
        ("RV-503", None, "Wheel", "144 x 52 mm, bolts to the hub, RV-503"),
    ], az=24, elev=16, focal=(0.0, 0.20, 0.075), dist=1.15, offsets=lambda p: (0, step.get(p["pn"], 0.0), 0))

    run = [p for p in parts if p["module"] in RUNNING or p["pn"] == "RV-522"]
    rest = [p for p in parts if not (p["module"] in RUNNING or p["pn"] == "RV-522")]
    labelled(run, o("RV-100_running_gear.png"), [
        ("RV-503", 1, "Drive modules", "Differential drive, track 365 mm, RV-110"),
        ("RV-554", 1, "Sprung front caster", "2 Lee Spring springs, 10 mm travel, RV-120"),
        ("RV-504", None, "Swivel casters", "goBILDA 2 in, front and rear, RV-504"),
        ("RV-310", None, "Rigid rear caster", "Fixed riser, RV-125"),
        ("RV-531", None, "Motor driver", "DFR0601, wired to both motors"),
        ("RV-522", None, "ESP32 controller", "Encoders in, PWM out"),
    ], az=35, elev=30, focal=(0.0, 0.0, 0.09), dist=1.45, ghost=rest, extra=add_wires, legend=LEGEND)


SAFETY = ("RV-180", "RV-182")                  # cliff sensor modules and bumper switch modules


def safety(parts, o):
    """Where the cliff sensors, bumper and bump switches sit: those parts solid, the rest of the robot ghosted."""
    solid = [p for p in parts if p["module"] in SAFETY or p["pn"] in ("RV-305",)]
    rest = [p for p in parts if p not in solid]
    no_bumper = [p for p in solid if p["pn"] != "RV-305"]
    cliff = lambda x, y: (x * 0.217, y * 0.105, 0.073)            # noqa: E731  ToF board centres (m)
    labelled(solid, o("RV-100_safety_front.png"), [
        ("RV-305", None, "Front bumper", "TPU, floats 4 mm on two guide pins, RV-305"),
        (cliff(1, 1), None, "Cliff sensor, front left", "VL53L4CD ToF in a bumper window, aimed 50° down"),
        (cliff(1, -1), None, "Cliff sensor, front right", "Watches the floor ahead of the right wheel"),
    ], az=32, elev=14, focal=(0.05, 0.0, 0.12), dist=1.5, ghost=rest)
    labelled(no_bumper, o("RV-100_safety_switches.png"), [
        ("RV-553", 1, "Bump switch, left", "Omron D2F-01L-D, pressed when the bumper moves back"),
        ("RV-553", -1, "Bump switch, right", "Wired straight to the ESP32 for an instant stop"),
        ("RV-317", 1, "Switch bracket", "Slotted, so the click point is set at assembly, RV-317"),
        ("RV-307", 1, "Cliff sensor cradle", "Holds the board at 50°, one per corner, RV-307"),
    ], az=28, elev=26, focal=(0.16, 0.0, 0.08), dist=0.9, ghost=rest)
    labelled(solid, o("RV-100_safety_rear.png"), [
        (cliff(-1, 1), None, "Cliff sensor, rear left", "Covers reversing: Nav2 back-up and undocking"),
        (cliff(-1, -1), None, "Cliff sensor, rear right", "Same ToF sensor and cradle as the front"),
    ], az=-145, elev=16, focal=(-0.05, 0.0, 0.12), dist=1.5, ghost=rest)
    labelled(solid, o("RV-100_safety_top.png"), [
        (cliff(1, 1), None, "Front left cliff", "x +217, y +105 mm"),
        (cliff(1, -1), None, "Front right cliff", "x +217, y -105 mm"),
        (cliff(-1, 1), None, "Rear left cliff", "x -217, y +105 mm"),
        (cliff(-1, -1), None, "Rear right cliff", "x -217, y -105 mm"),
        ("RV-305", None, "Bumper", "Front only, full width"),
    ], az=-90, elev=88, focal=(0.0, 0.0, 0.10), dist=1.6, ghost=rest)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--out", default=OUT)
    ap.add_argument("--no-video", action="store_true")
    ap.add_argument("--drivetrain", action="store_true", help="only the drivetrain views")
    ap.add_argument("--safety", action="store_true", help="only the cliff sensor and bumper views")
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    parts = load_parts()
    print(len(parts), "parts")
    o = lambda f: os.path.join(a.out, f)  # noqa: E731
    if a.drivetrain:
        drivetrain(parts, o)
        return
    if a.safety:
        safety(parts, o)
        return
    hero(parts, o("RV-100_hero_front.png"), az=35, elev=18)
    hero(parts, o("RV-100_hero_rear.png"), az=-140, elev=22)
    labelled(parts, o("RV-100_labelled_front.png"), CALLOUTS_FRONT, az=35, elev=18)
    labelled(parts, o("RV-100_labelled_rear.png"), CALLOUTS_REAR, az=-140, elev=24)
    exploded(parts, o("RV-100_exploded.png"))
    drivetrain(parts, o)
    ortho_views(parts, o("RV-100_views.png"))
    glb(parts, o("RV-100_RoamView.glb"))
    if not a.no_video:
        turntable(parts, o("RV-100_turntable.mp4"))


if __name__ == "__main__":
    main()
