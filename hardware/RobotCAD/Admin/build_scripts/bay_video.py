"""Render a guided tour of the RV-100 body bay: the shell opens, then each subsystem inside is shown in turn.

    python bay_video.py              # writes Simulation/Bay/RV-100_bay_tour.mp4
    python bay_video.py --stills     # also saves one PNG per step

Needs numpy, trimesh, pyvista and imageio-ffmpeg (the %LOCALAPPDATA%\\rvcv venv has them).
Geometry is the per-component STLs that export_fov_stl.py writes (Simulation/FOV/parts), in the CAD's
assembly frame, so every part sits exactly where the CAD puts it. Motions shown (drawer stroke, spring
travel, deck lift) are for illustration; the drawer and spring strokes match the globals.
"""
import argparse
import glob
import math
import os
import sys

import numpy as np
import pyvista as pv
import trimesh

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from export_urdf import ROOT, STL_DIR, SW_TO_ROS, parse_name, read_globals  # noqa: E402

FPS = 30
G = read_globals()
mm = 1e-3
DRAWER_OUT = 0.25            # m, of the 333 mm stroke
SPRING_TRAVEL = (G["SPRING_L"] - G["SPRING_SEAT"]) * mm

COLOUR = {
    "RV-301": "#e9e6df", "RV-302": "#d9d5cc", "RV-303": "#e9e6df", "RV-304": "#e9e6df", "RV-305": "#2b2b2e",
    "RV-306": "#e9e6df", "RV-307": "#e9e6df", "RV-308": "#e9e6df", "RV-309": "#d9d5cc", "RV-310": "#e9e6df",
    "RV-311": "#e9e6df", "RV-312": "#e9e6df", "RV-313": "#e9e6df", "RV-314": "#e9e6df", "RV-315": "#e9e6df",
    "RV-316": "#3a3f47", "RV-317": "#e9e6df",
    "RV-410": "#9aa3ad", "RV-420": "#9aa3ad", "RV-430": "#56606c", "RV-431": "#b8bec6", "RV-432": "#b8bec6",
    "RV-433": "#b8bec6", "RV-434": "#b8bec6", "RV-435": "#d8a53f",
    "RV-501": "#4a4f57", "RV-502": "#aab0b8", "RV-503": "#1f1f22", "RV-504": "#2d2f33", "RV-511": "#33363b",
    "RV-512": "#202124", "RV-513": "#1d4ed8", "RV-514": "#55595f", "RV-521": "#1f8a4c", "RV-522": "#1e4fa3",
    "RV-531": "#c0392b", "RV-532": "#8b5cf6", "RV-541": "#f08c2e", "RV-551": "#d62828", "RV-552": "#111214",
    "RV-553": "#111214", "RV-554": "#7a7f87",
}
SKINS = {"RV-313", "RV-314", "RV-315"}
SHELL = SKINS | {"RV-305", "RV-410", "RV-431"}
DRAWER = {"RV-316", "RV-432", "RV-541"}
GREY = np.array([0.80, 0.82, 0.85])


def hex_rgb(h):
    return np.array([int(h[i:i + 2], 16) / 255 for i in (1, 3, 5)])


def smooth(t):
    t = min(1.0, max(0.0, t))
    return t * t * (3 - 2 * t)


def translate(v):
    T = np.eye(4)
    T[:3, 3] = v
    return T


def rot_y_about(angle, centre):
    c, s = math.cos(angle), math.sin(angle)
    R = np.array([[c, 0, s, 0], [0, 1, 0, 0], [-s, 0, c, 0], [0, 0, 0, 1]])
    return translate(centre) @ R @ translate(-np.asarray(centre))


def scale_z_about(s, z0):
    S = np.diag([1, 1, s, 1.0])
    return translate((0, 0, z0)) @ S @ translate((0, 0, -z0))


def cam_at(focal, dist, elev, az):
    e, a = math.radians(elev), math.radians(az)
    f = np.asarray(focal, float)
    return f + dist * np.array([math.cos(e) * math.cos(a), math.cos(e) * math.sin(a), math.sin(e)]), f


# --------------------------------------------------------------------------------------------- the tour
def steps():
    """Each step: title, text, seconds, camera (focal, dist, elev, az0, az1), highlight set, labels, flags."""
    return [
        dict(title="RoamView RV-100", text="420 x 340 mm body, 8.67 kg. The shell comes off to show the bay.",
             t=3.0, cam=((0, 0, 0.22), 1.75, 24, -140, -128), hl=None, labels=[], open=0),
        dict(title="Opening the shell", text="The deck lifts with the sensor tower, LiDAR and E-stop; the skins and "
             "bumper move clear.", t=3.5, cam=((0, 0, 0.12), 1.25, 42, -128, -112), hl=None, labels=[], open=1),
        dict(title="Battery drawer (RV-130)", text="The 4S Li-ion pack rides in a sheet-metal tray on two guides and "
             "slides out the back. Its XT60 mates blind when the drawer closes.",
             t=5.0, cam=((-0.22, 0, 0.10), 1.05, 32, 150, 160), hl={"RV-541", "RV-432", "RV-316", "RV-433"},
             labels=[("RV-541", "4S battery (placeholder envelope)"), ("RV-433", "drawer guides")], open=1,
             drawer=(0.0, 1.8, None)),
        dict(title="Compute bay (RV-150)", text="Raspberry Pi 5 with active cooler and the ESP32 board share one "
             "printed tray under the battery, near the middle of the robot.",
             t=4.5, cam=((-0.05, 0, 0.05), 0.70, 52, -150, -135), hl={"RV-521", "RV-522", "RV-311"},
             labels=[("RV-521", "Raspberry Pi 5"), ("RV-522", "ESP32 board")], open=1, drawer=(None, None, None)),
        dict(title="Power PCB (RV-532, to be designed)", text="Reserved 95 x 120 x 40 mm envelope: E-stop relay, "
             "3x INA260 current sensors, 2x 5 V buck, ideal diodes, bulk capacitor.",
             t=4.5, cam=((0.03, 0, 0.05), 0.75, 48, -70, -55), hl={"RV-532"},
             labels=[("RV-532", "power PCB envelope")], open=1, drawer=(None, None, 3.3)),
        dict(title="Motor driver (RV-160)", text="DFRobot DFR0601 dual driver, 12 A per channel, on a printed mount "
             "close to the left motor so the high-current wires stay short.",
             t=4.0, cam=((0.08, 0.08, 0.06), 0.75, 42, 20, 35), hl={"RV-531", "RV-312"},
             labels=[("RV-531", "DFR0601 motor driver")], open=1),
        dict(title="Drive modules (RV-110)", text="Pololu 70:1 gearmotors with encoders, on 5052 brackets tucked "
             "inside the 144 mm wheels. Track 365 mm.",
             t=4.5, cam=((0, 0.12, 0.08), 0.85, 34, 55, 85), hl={"RV-501", "RV-434", "RV-502", "RV-503"},
             labels=[("RV-501", "gearmotor + encoder"), ("RV-503", "144 mm wheel")], open=1, spin=True),
        dict(title="Sprung front caster (RV-120)", text="The carriage rides on two guide pins and two Lee Spring "
             f"springs: {SPRING_TRAVEL / mm:.0f} mm travel, 10.7 N preload. The rear caster (RV-125) is rigid.",
             t=5.0, cam=((0.13, 0, 0.045), 0.52, 8, -72, -55), hl={"RV-308", "RV-309", "RV-554", "front-504"},
             labels=[("RV-554", "springs"), ("front-504", "2 in caster")], open=1, bounce=True),
        dict(title="Front sensors and dock pads", text="Cliff ToF sensors at all four corners, two bumper "
             "microswitches behind the TPU bumper, and the brass charge pads under the nose.",
             t=5.0, cam=((0.21, 0, 0.06), 0.72, 18, 25, -15), hl={"RV-513", "RV-307", "RV-553", "RV-317", "RV-435",
                                                                   "RV-306"},
             labels=[("RV-513", "cliff sensors"), ("RV-553", "bump switches"), ("RV-435", "charge pads")], open=1),
        dict(title="The whole bay", text="Heavy parts sit low: battery, motors and electronics are all under the deck.",
             t=5.0, cam=((0, 0, 0.10), 1.25, 45, -110, -60), hl=None,
             labels=[("RV-541", "battery"), ("RV-531", "motor driver"), ("RV-532", "power PCB"),
                     ("RV-501", "gearmotor"), ("RV-554", "front caster")], open=1),
        dict(title="Closing up", text="Deck, skins and bumper go back on.", t=4.0,
             cam=((0, 0, 0.22), 1.75, 24, -60, -40), hl=None, labels=[], open=0),
    ]


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--out", default=os.path.join(ROOT, "Simulation", "Bay", "RV-100_bay_tour.mp4"))
    ap.add_argument("--stills", action="store_true")
    ap.add_argument("--size", default="1920x1080")
    a = ap.parse_args()
    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    W, H = (int(v) for v in a.size.split("x"))

    # ---- parts
    parts = []
    for path in sorted(glob.glob(os.path.join(STL_DIR, "RV-100 - *.STL"))):
        pn, module = parse_name(path)
        if pn not in COLOUR:
            continue
        tm = trimesh.load(path, force="mesh")
        tm.apply_transform(SW_TO_ROS)
        key = "front-504" if (pn == "RV-504" and module.startswith("RV-120")) else pn
        deck = pn in ("RV-430", "RV-551") or module.startswith(("RV-140", "RV-170"))
        parts.append(dict(pn=pn, key=key, module=module, mesh=pv.wrap(tm), deck=deck,
                          centre=tm.bounds.mean(axis=0), rgb=hex_rgb(COLOUR[pn])))
    print(f"{len(parts)} parts loaded")

    pl = pv.Plotter(off_screen=True, window_size=(W, H))
    pl.set_background("#eef1f5", top="#ffffff")
    pl.enable_depth_peeling(number_of_peels=8)
    floor = pv.Plane(center=(0, 0, -0.0005), direction=(0, 0, 1), i_size=4, j_size=4)
    pl.add_mesh(floor, color="#d9dde3", ambient=0.25)
    grid = pv.merge([pv.Line((v, -2, 0), (v, 2, 0)) for v in np.arange(-2, 2.01, 0.1)] +
                    [pv.Line((-2, v, 0), (2, v, 0)) for v in np.arange(-2, 2.01, 0.1)])
    pl.add_mesh(grid, color="#c9ced6", line_width=1)
    for p in parts:
        p["actor"] = pl.add_mesh(p["mesh"], color=p["rgb"], ambient=0.22, diffuse=0.75, specular=0.2,
                                 smooth_shading=False)

    # labels: one actor per (step, label), toggled on and off
    S = steps()
    label_actors = []
    for i, st in enumerate(S):
        acts = []
        for key, text in st["labels"]:
            ps = [p for p in parts if p["key"] == key]
            c = max(ps, key=lambda p: p["centre"][2])["centre"] if key != "RV-501" else ps[0]["centre"]
            act = pl.add_point_labels([c + np.array([0, 0, 0.012])], [text], font_size=16, point_size=10,
                                      point_color="#111827", text_color="#111827", shape="rounded_rect",
                                      shape_color="#ffffff", shape_opacity=0.85, always_visible=True,
                                      name=f"lab{i}_{key}")
            act.visibility = False
            acts.append(act)
        label_actors.append(acts)

    head = pl.add_text("", position=(36, H - 78), font_size=22, color="#111827", font="arial")
    body = pl.add_text("", position=(36, H - 116), font_size=12, color="#374151", font="arial")
    count = pl.add_text("", position=(W - 150, H - 70), font_size=12, color="#6b7280", font="arial")
    pl.add_text("RV-100 CAD, Revision A. Placeholder envelopes: battery (RV-541) and power PCB (RV-532).",
                position=(36, 20), font_size=9, color="#6b7280", font="arial")
    for t_act, opac in ((head, 0.88), (body, 0.88), (count, 0.0)):
        tp = t_act.GetTextProperty()
        tp.SetBackgroundColor(1, 1, 1)
        tp.SetBackgroundOpacity(opac)
    cam = pl.camera

    # ---- render
    pl.open_movie(a.out, framerate=FPS, quality=8, macro_block_size=8)
    open_state, drawer_x, prev_cam, wheel = 0.0, 0.0, None, 0.0
    spring_top = max(p["mesh"].bounds[5] for p in parts if p["pn"] == "RV-554")
    spring_len = spring_top - min(p["mesh"].bounds[4] for p in parts if p["pn"] == "RV-554")
    axle_z = G["AXLE_Z"] * mm
    total = 0
    for i, st in enumerate(S):
        n = int(st["t"] * FPS)
        open0, drawer0 = open_state, drawer_x
        for acts_i, acts in enumerate(label_actors):
            for act in acts:
                act.visibility = acts_i == i
        head.SetInput(st["title"])
        body.SetInput(st["text"])
        count.SetInput(f"{i + 1} / {len(S)}")
        start_cam = prev_cam
        for f in range(n):
            t = f / FPS
            u = f / max(1, n - 1)
            # shell
            open_state = open0 + (st["open"] - open0) * smooth(t / 2.5)
            # drawer: (out at, out by, in at) in seconds
            d = st.get("drawer")
            if d:
                out_at, out_by, in_at = d
                if out_at is not None:
                    drawer_x = drawer0 + (DRAWER_OUT - drawer0) * smooth((t - out_at) / (out_by - out_at))
                if in_at is not None and t >= in_at:
                    drawer_x = DRAWER_OUT * (1 - smooth((t - in_at) / 1.2))
            bounce = SPRING_TRAVEL * 0.5 * (1 - math.cos(2 * math.pi * t / 1.6)) if st.get("bounce") and \
                t < 3.2 else 0.0
            if st.get("spin"):
                wheel += 2 * math.pi * 0.35 / FPS
            # part transforms and colours
            hl = st["hl"]
            for p in parts:
                M = np.eye(4)
                op = 1.0
                if p["deck"]:
                    M = translate((0, 0, 0.32 * open_state))
                    op = 1 - 0.88 * open_state
                elif p["pn"] in SKINS or p["pn"] == "RV-305":
                    c = p["centre"]
                    dirn = np.array([np.sign(c[0]) if abs(c[0]) > 0.2 else 0, np.sign(c[1]) if abs(c[1]) > 0.14
                                     else 0, 0])
                    M = translate(dirn * 0.14 * open_state)
                    op = 1 - 0.9 * open_state
                elif p["pn"] in ("RV-410", "RV-431"):
                    op = 1 - 0.7 * open_state
                elif p["pn"] == "RV-503":
                    op = 1 - (0.55 if st.get("spin") else 0.75) * open_state
                if p["pn"] in DRAWER:
                    M = translate((-drawer_x, 0, 0)) @ M
                if p["key"] in ("front-504",) or (p["pn"] == "RV-309"):
                    M = translate((0, 0, bounce)) @ M
                if p["pn"] == "RV-554" and bounce:
                    M = scale_z_about((spring_len - bounce) / spring_len, spring_top) @ M
                if p["pn"] in ("RV-503", "RV-502") and wheel:
                    M = rot_y_about(wheel, (0, 0, axle_z)) @ M
                p["actor"].user_matrix = M
                p["actor"].prop.opacity = op
                if hl is None or p["pn"] in SHELL or p["deck"]:
                    rgb, amb = p["rgb"], 0.22
                elif p["key"] in hl or p["pn"] in hl:
                    rgb, amb = p["rgb"], 0.40
                else:
                    rgb, amb = p["rgb"] * 0.3 + GREY * 0.7, 0.30
                p["actor"].prop.color = rgb
                p["actor"].prop.ambient = amb
            # camera: glide from the last view for 1.5 s, then drift
            focal, dist, elev, az0, az1 = st["cam"]
            pos, foc = cam_at(focal, dist, elev, az0 + (az1 - az0) * u)
            if start_cam is not None:
                k = smooth(t / 1.5)
                pos = start_cam[0] * (1 - k) + pos * k
                foc = start_cam[1] * (1 - k) + foc * k
            cam.position, cam.focal_point, cam.up = pos, foc, (0, 0, 1)
            cam.clipping_range = (0.01, 20)
            pl.write_frame()
            total += 1
            if a.stills and f == int(n * 0.7):
                pl.screenshot(os.path.splitext(a.out)[0] + f"_step{i + 1:02d}.png")
        prev_cam = (np.array(cam.position), np.array(cam.focal_point))
    pl.close()
    print(f"{total} frames, {total / FPS:.1f} s\nwrote {a.out}")


if __name__ == "__main__":
    main()
