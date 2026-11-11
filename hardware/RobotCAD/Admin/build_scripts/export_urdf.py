"""Export the RV-100 CAD as a URDF for ROS 2 (RViz, Gazebo, Nav2).

    python export_urdf.py              # uses the component STLs that export_fov_stl.py wrote
    python export_urdf.py --export     # re-export those STLs from SolidWorks first

Needs numpy, trimesh and pyvista (the %LOCALAPPDATA%\\rvcv venv has them). SolidWorks is only needed with --export.

Inputs (all from RobotCAD):
    Simulation/FOV/parts/*.STL   one STL per component, RV-100 assembly frame, mm
    Drawings/RV-100_BOM.csv      part masses as SolidWorks reports them (overrides included)
    Admin/RV_globals.txt         wheel, caster, spring and sensor dimensions
Outputs (in the ROS package, default ros/src/roamview_sim):
    urdf/roamview_rv100.urdf
    meshes/rv100/<link>_<colour>.stl   merged per link and colour, decimated, metres, in the link frame

Frames follow REP 103/105. base_footprint is the floor under the drive axle centre, which is the CAD origin;
base_link sits on the axle line. SolidWorks X (left), Y (up), Z (forward) become ROS y, z, x.
Each link's mass is the sum of its BOM part masses; its centre of mass and inertia come from the part
meshes (bounding boxes for meshes that are not closed solids), scaled to those masses.
Casters are frictionless spheres. The front one rides on a prismatic joint sprung by the two Lee Spring
LC 032E 05 S springs, with the preload and 10 mm travel taken from the SPRING_* globals.
"""
import argparse
import csv
import glob
import math
import os
import re
import subprocess
import sys
import xml.etree.ElementTree as ET

import numpy as np
import pyvista as pv
import trimesh

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))                      # RobotCAD
STL_DIR = os.path.join(ROOT, "Simulation", "FOV", "parts")
BOM = os.path.join(ROOT, "Drawings", "RV-100_BOM.csv")
GLOBALS = os.path.join(ROOT, "Admin", "RV_globals.txt")
PKG_DIR = os.path.join(os.path.dirname(ROOT), "ros", "src", "roamview_sim")

SW_TO_ROS = np.eye(4)
SW_TO_ROS[:3, :3] = np.array([[0, 0, 1], [1, 0, 0], [0, 1, 0]]) * 1e-3    # mm, SW axes -> m, ROS axes

COLOURS = {                                  # BOM group -> (name, rgba)
    "MANUFACTURED - 3D print": ("petg", (0.86, 0.86, 0.83, 1)),
    "MANUFACTURED - sheet metal": ("alu", (0.75, 0.77, 0.80, 1)),
    "MANUFACTURED - weldment": ("alu", (0.75, 0.77, 0.80, 1)),
    "PURCHASED": ("dark", (0.18, 0.18, 0.20, 1)),
    "RESERVED (buy / design)": ("reserved", (0.48, 0.31, 0.69, 0.6)),
}
TPU = ("tpu", (0.12, 0.12, 0.12, 1))          # the bumper


# ----------------------------------------------------------------------------------------------------- inputs
def read_globals(path=GLOBALS):
    """Same rules as rv_sw.read_globals, without importing SolidWorks: a global may use any other global."""
    pending = {}
    for line in open(path, encoding="utf-8"):
        if "=" in line:
            name, expr = line.split("=", 1)
            pending[name.strip().strip('"')] = expr.strip()
    vals = {}
    while pending:
        done = [n for n, e in pending.items() if all(r in vals for r in re.findall(r'"([^"]+)"', e))]
        if not done:
            raise ValueError(f"globals that cannot be solved: {sorted(pending)}")
        for n in done:
            e = re.sub(r'"([^"]+)"', lambda m: repr(vals[m.group(1)]), pending.pop(n))
            vals[n] = float(eval(e, {"__builtins__": {}}, vars(math)))
    return vals


def read_bom(path=BOM):
    """PartNo -> (unit mass kg, BOM group)."""
    out = {}
    for r in csv.DictReader(open(path, encoding="utf-8")):
        if r["Unit_mass_g"]:
            out[r["PartNo"]] = (float(r["Unit_mass_g"]) / 1000, r["Group"])
    return out


def parse_name(path):
    """'RV-100 - RV-110_DriveModule-1 RV-501_Gearmotor_..-1.STL' -> ('RV-501', 'RV-110_DriveModule-1')."""
    toks = os.path.splitext(os.path.basename(path))[0].split(" ")
    return toks[-1].split("_")[0], (toks[2] if len(toks) > 3 else "")


# ------------------------------------------------------------------------------------------------ mass props
def mass_props(mesh, mass):
    """Centre of mass and inertia about it (link-aligned axes) for a part of the given mass."""
    if mesh.is_volume and mesh.volume > 1e-9:
        return mesh.center_mass, mesh.moment_inertia * (mass / mesh.mass), True
    lo, hi = mesh.bounds
    d = hi - lo
    inertia = mass / 12 * np.diag([d[1] ** 2 + d[2] ** 2, d[0] ** 2 + d[2] ** 2, d[0] ** 2 + d[1] ** 2])
    return (lo + hi) / 2, inertia, False


def combine(items):
    """[(mass, com, inertia about com)] -> total mass, com, inertia about that com (parallel axis)."""
    m_tot = sum(m for m, _, _ in items)
    com = sum(m * c for m, c, _ in items) / m_tot
    inertia = np.zeros((3, 3))
    for m, c, i in items:
        d = c - com
        inertia += i + m * (d @ d * np.eye(3) - np.outer(d, d))
    return m_tot, com, inertia


def bbox(meshes):
    lo = np.min([m.bounds[0] for m in meshes], axis=0)
    hi = np.max([m.bounds[1] for m in meshes], axis=0)
    return lo, hi


def contact_point(mesh, wheel_d):
    """Centre of a caster wheel: the mesh vertices in the lowest half wheel give x, y; z is the radius."""
    v = mesh.vertices[mesh.vertices[:, 2] < mesh.bounds[0][2] + wheel_d / 2]
    return np.array([v[:, 0].mean(), v[:, 1].mean(), mesh.bounds[0][2] + wheel_d / 2])


# ------------------------------------------------------------------------------------------------------ mesh
def write_mesh(mesh, path, max_tris):
    faces = np.hstack([np.full((len(mesh.faces), 1), 3), mesh.faces]).ravel()
    pd = pv.PolyData(np.asarray(mesh.vertices, dtype=float), faces)
    if pd.n_cells > max_tris:
        pd = pd.decimate(1 - max_tris / pd.n_cells)
    pd.save(path, binary=True)
    return pd.n_cells


# ------------------------------------------------------------------------------------------------------ urdf
def f(v):
    return " ".join(f"{x:.6g}" for x in v)


def sub(parent, tag, **attrs):
    return ET.SubElement(parent, tag, {k: str(v) for k, v in attrs.items()})


def add_inertial(link, mass, com_local, inertia):
    el = sub(link, "inertial")
    sub(el, "origin", xyz=f(com_local), rpy="0 0 0")
    sub(el, "mass", value=f"{mass:.5g}")
    i = inertia
    sub(el, "inertia", ixx=f"{i[0, 0]:.6g}", iyy=f"{i[1, 1]:.6g}", izz=f"{i[2, 2]:.6g}",
        ixy=f"{i[0, 1]:.6g}", ixz=f"{i[0, 2]:.6g}", iyz=f"{i[1, 2]:.6g}")


def add_joint(robot, name, jtype, parent, child, xyz, rpy=(0, 0, 0), axis=None, limit=None, damping=None):
    j = sub(robot, "joint", name=name, type=jtype)
    sub(j, "parent", link=parent)
    sub(j, "child", link=child)
    sub(j, "origin", xyz=f(xyz), rpy=f(rpy))
    if axis is not None:
        sub(j, "axis", xyz=f(axis))
    if limit:
        sub(j, "limit", **limit)
    if damping is not None:
        sub(j, "dynamics", damping=damping, friction=0)
    return j


def add_box(link, kind, lo, hi, origin):
    el = sub(link, kind)
    sub(el, "origin", xyz=f((lo + hi) / 2 - origin), rpy="0 0 0")
    sub(sub(el, "geometry"), "box", size=f(hi - lo))


def gazebo(robot, reference=None, **tags):
    g = sub(robot, "gazebo", **({"reference": reference} if reference else {}))
    for k, v in tags.items():
        sub(g, k).text = str(v)
    return g


def check_tree(robot):
    links = [l.get("name") for l in robot.findall("link")]
    assert len(links) == len(set(links)), "duplicate link names"
    children = set()
    for j in robot.findall("joint"):
        p, c = j.find("parent").get("link"), j.find("child").get("link")
        assert p in links and c in links, f"joint {j.get('name')} points at a missing link"
        assert c not in children, f"{c} has two parents"
        children.add(c)
    roots = [l for l in links if l not in children]
    assert roots == ["base_footprint"], f"tree roots: {roots}"
    return len(links), len(robot.findall("joint"))


# ------------------------------------------------------------------------------------------------------ main
def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--export", action="store_true", help="re-export the component STLs from SolidWorks first")
    ap.add_argument("--sw-python", default="python", help="Python with pywin32, for --export")
    ap.add_argument("--stl-dir", default=STL_DIR)
    ap.add_argument("--pkg-dir", default=PKG_DIR, help="ROS package that receives urdf/ and meshes/")
    ap.add_argument("--pkg-name", default="roamview_sim")
    ap.add_argument("--max-tris", type=int, default=40000, help="triangle cap per visual mesh file")
    a = ap.parse_args()

    if a.export:
        subprocess.run([a.sw_python, os.path.join(HERE, "export_fov_stl.py")], check=True)

    G, bom = read_globals(), read_bom()
    mm = 1e-3
    axle_z, wheel_r, wheel_w = G["AXLE_Z"] * mm, G["WHEEL_D"] / 2 * mm, G["WHEEL_W"] * mm
    caster_d, deck_z = G["CASTER_D"] * mm, G["DECK_Z"] * mm

    # ---- load every component and give it to a link
    parts, skipped = [], []
    for path in sorted(glob.glob(os.path.join(a.stl_dir, "RV-100 - *.STL"))):
        pn, module = parse_name(path)
        if pn not in bom:
            skipped.append(os.path.basename(path))
            continue
        mesh = trimesh.load(path, force="mesh")
        mesh.apply_transform(SW_TO_ROS)
        mass, group = bom[pn]
        centre = mesh.bounds.mean(axis=0)
        if pn in ("RV-502", "RV-503"):
            link = "left_wheel" if centre[1] > 0 else "right_wheel"
        elif module.startswith("RV-120") and pn in ("RV-309", "RV-504"):
            link = "caster_front_carriage"                        # sprung: carriage plate + caster
        elif pn == "RV-305":
            link = "bumper_link"
        else:
            link = "base_link"
        colour = TPU if pn == "RV-305" else COLOURS.get(group, COLOURS["PURCHASED"])
        parts.append(dict(pn=pn, module=module, mesh=mesh, mass=mass, link=link, colour=colour))
    if not parts:
        sys.exit(f"no component STLs in {a.stl_dir}: run with --export")

    def of(link, pn=None, module=""):
        return [p for p in parts if p["link"] == link and (pn is None or p["pn"] == pn)
                and p["module"].startswith(module)]

    # ---- link frames (positions in base_footprint = CAD origin, all axis-aligned)
    origin = {"base_footprint": np.zeros(3), "base_link": np.array([0, 0, axle_z])}
    for side in ("left", "right"):
        lo, hi = bbox([p["mesh"] for p in of(f"{side}_wheel", "RV-503")])
        origin[f"{side}_wheel"] = (lo + hi) / 2
    lo, hi = bbox([p["mesh"] for p in of("caster_front_carriage", "RV-309")])
    origin["caster_front_carriage"] = np.array([(lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, lo[2]])
    origin["caster_front_ball"] = contact_point(of("caster_front_carriage", "RV-504")[0]["mesh"], caster_d)
    origin["caster_rear_ball"] = contact_point(of("base_link", "RV-504", "RV-125")[0]["mesh"], caster_d)
    lo, hi = bbox([p["mesh"] for p in of("bumper_link")])
    origin["bumper_link"] = (lo + hi) / 2

    robot = ET.Element("robot", name="roamview")
    robot.append(ET.Comment(" GENERATED by RobotCAD/Admin/build_scripts/export_urdf.py from the RV-100 CAD. "
                            "Do not edit by hand. "))
    for name, rgba in {c for p in parts for c in [p["colour"]]}:
        sub(sub(robot, "material", name=f"rv_{name}"), "color", rgba=f(rgba))

    # ---- links with geometry
    mesh_dir = os.path.join(a.pkg_dir, "meshes", "rv100")
    os.makedirs(mesh_dir, exist_ok=True)
    for old in glob.glob(os.path.join(mesh_dir, "*.stl")):
        os.remove(old)
    links, report, all_items, open_meshes = {}, [], [], []
    for name in ("base_footprint", "base_link", "left_wheel", "right_wheel", "caster_front_carriage",
                 "caster_front_ball", "caster_rear_ball", "bumper_link"):
        links[name] = sub(robot, "link", name=name)
    for name in ("base_link", "left_wheel", "right_wheel", "caster_front_carriage", "bumper_link"):
        mine = [p for p in parts if p["link"] == name]
        items = []
        for p in mine:
            c, i, closed = mass_props(p["mesh"], p["mass"])
            items.append((p["mass"], c, i))
            if not closed:
                open_meshes.append(p["pn"])
        all_items += items
        m, com, inertia = combine(items)
        add_inertial(links[name], m, com - origin[name], inertia)
        tris = 0
        for cname, rgba in sorted({p["colour"] for p in mine}):
            merged = trimesh.util.concatenate([p["mesh"] for p in mine if p["colour"][0] == cname])
            merged.apply_translation(-origin[name])
            fname = f"{name}_{cname}.stl"
            tris += write_mesh(merged, os.path.join(mesh_dir, fname), a.max_tris)
            vis = sub(links[name], "visual")
            sub(vis, "origin", xyz="0 0 0", rpy="0 0 0")
            sub(sub(vis, "geometry"), "mesh", filename=f"package://{a.pkg_name}/meshes/rv100/{fname}")
            sub(vis, "material", name=f"rv_{cname}")
        report.append((name, len(mine), m, tris))

    # ---- collisions: simple shapes so Gazebo stays fast
    base = [p["mesh"] for p in parts if p["link"] == "base_link" and p["pn"] != "RV-504"
            and not p["module"].startswith(("RV-120", "RV-125"))]
    body = [m for m in base if m.bounds.mean(axis=0)[2] < deck_z]
    upper = [m for m in base if m.bounds.mean(axis=0)[2] >= deck_z]
    for group in (body, upper):
        add_box(links["base_link"], "collision", *bbox(group), origin["base_link"])
    add_box(links["bumper_link"], "collision", *bbox([p["mesh"] for p in of("bumper_link")]),
            origin["bumper_link"])
    for side in ("left", "right"):
        col = sub(links[f"{side}_wheel"], "collision")
        sub(col, "origin", xyz="0 0 0", rpy=f"{math.pi / 2:.6g} 0 0")
        sub(sub(col, "geometry"), "cylinder", radius=f"{wheel_r:.6g}", length=f"{wheel_w:.6g}")
    for end in ("front", "rear"):
        col = sub(links[f"caster_{end}_ball"], "collision")
        sub(col, "origin", xyz="0 0 0", rpy="0 0 0")
        sub(sub(col, "geometry"), "sphere", radius=f"{caster_d / 2:.6g}")
        add_inertial(links[f"caster_{end}_ball"], 1e-3, np.zeros(3), np.eye(3) * 1e-7)   # massless contact

    # ---- joints
    o = origin
    add_joint(robot, "base_joint", "fixed", "base_footprint", "base_link", o["base_link"])
    for side in ("left", "right"):
        add_joint(robot, f"{side}_wheel_joint", "continuous", "base_link", f"{side}_wheel",
                  o[f"{side}_wheel"] - o["base_link"], axis=(0, 1, 0),
                  limit=dict(effort=2.45, velocity=15.7), damping=0.005)   # Pololu 4754: 2.45 N m peak, 150 rpm
    k = 2 * G["SPRING_K"] * 1000                                        # two springs, N/m
    preload = 2 * G["SPRING_K"] * (G["SPRING_FL"] - G["SPRING_L"])     # N at rest
    travel = (G["SPRING_L"] - G["SPRING_SEAT"]) * mm                    # carriage gap, m
    add_joint(robot, "caster_front_spring_joint", "prismatic", "base_link", "caster_front_carriage",
              o["caster_front_carriage"] - o["base_link"], axis=(0, 0, 1),
              limit=dict(lower=0, upper=f"{travel:.4g}", effort=200, velocity=1), damping=30)
    add_joint(robot, "caster_front_ball_joint", "fixed", "caster_front_carriage", "caster_front_ball",
              o["caster_front_ball"] - o["caster_front_carriage"])
    add_joint(robot, "caster_rear_ball_joint", "fixed", "base_link", "caster_rear_ball",
              o["caster_rear_ball"] - o["base_link"])
    add_joint(robot, "bumper_joint", "fixed", "base_link", "bumper_link", o["bumper_link"] - o["base_link"])

    # ---- sensor frames (no geometry)
    def frame(name, xyz, rpy=(0, 0, 0), parent="base_link"):
        sub(robot, "link", name=name)
        add_joint(robot, f"{name}_joint", "fixed", parent, name, np.asarray(xyz) - o[parent], rpy)

    frame("lidar_link", (G["LIDAR_X"] * mm, 0, G["SCAN_Z"] * mm))
    oak = of("base_link", "RV-511")[0]["mesh"]
    lo, hi = oak.bounds
    tilt = math.radians(G["OAK_TILT"])
    frame("camera_link", (hi[0], (lo[1] + hi[1]) / 2, (lo[2] + hi[2]) / 2), (0, tilt, 0))
    sub(robot, "link", name="camera_optical_frame")
    add_joint(robot, "camera_optical_joint", "fixed", "camera_link", "camera_optical_frame", (0, 0, 0),
              (-math.pi / 2, 0, -math.pi / 2))
    frame("imu_link", (lo + hi) / 2, (0, tilt, 0))                 # BNO086 inside the OAK
    pitch = math.radians(90 - G["CLIFF_ANG"])                     # x axis = sensing direction
    for name, sx, sy in (("front_left", 1, 1), ("front_right", 1, -1), ("rear_left", -1, 1), ("rear_right", -1, -1)):
        frame(f"cliff_{name}_link", (sx * G["CLIFF_X"] * mm, sy * G["CLIFF_Y"] * mm, G["CLIFF_Z"] * mm),
              (0, pitch, 0 if sx > 0 else math.pi))
    frame("dock_contact_link", (G["CONTACT_X"] * mm, 0, G["CONTACT_Z"] * mm))

    # ---- Gazebo: friction, spring, drive
    for side in ("left", "right"):
        gazebo(robot, f"{side}_wheel", mu1=0.7, mu2=0.7)
    for end in ("front", "rear"):
        gazebo(robot, f"caster_{end}_ball", mu1=0.0, mu2=0.0)
    gazebo(robot, "caster_front_spring_joint", springStiffness=f"{k:.6g}", springReference=f"{-preload / k:.6g}")
    track = o["left_wheel"][1] - o["right_wheel"][1]
    g = gazebo(robot)
    dd = sub(g, "plugin", filename="gz-sim-diff-drive-system", name="gz::sim::systems::DiffDrive")
    for tag, val in (("left_joint", "left_wheel_joint"), ("right_joint", "right_wheel_joint"),
                     ("wheel_separation", f"{track:.6g}"), ("wheel_radius", f"{wheel_r:.6g}"),
                     ("topic", "/model/roamview/cmd_vel"), ("odom_topic", "/model/roamview/odometry"),
                     ("frame_id", "odom"), ("child_frame_id", "base_footprint"), ("odom_publish_frequency", 50)):
        sub(dd, tag).text = str(val)
    js = sub(g, "plugin", filename="gz-sim-joint-state-publisher-system",
             name="gz::sim::systems::JointStatePublisher")
    sub(js, "topic").text = "/model/roamview/joint_states"

    # ---- write and report
    n_links, n_joints = check_tree(robot)
    ET.indent(robot, "  ")
    out = os.path.join(a.pkg_dir, "urdf", "roamview_rv100.urdf")
    os.makedirs(os.path.dirname(out), exist_ok=True)
    ET.ElementTree(robot).write(out, encoding="utf-8", xml_declaration=True)

    m, com, _ = combine(all_items)
    print(f"{len(parts)} components -> {n_links} links, {n_joints} joints")
    for name, n, mass, tris in report:
        print(f"  {name:24s} {n:3d} parts {mass:7.3f} kg {tris:7d} triangles")
    print(f"total mass {m:.2f} kg, COM x {com[0] / mm:.1f} y {com[1] / mm:.1f} z {com[2] / mm:.1f} mm "
          f"(the CAD reports 8.67 kg at -2.0, 0.1, 145.9)")
    print(f"wheels: track {track / mm:.1f} mm (globals {G['TRACK']:.0f}), axle z {o['left_wheel'][2] / mm:.1f} mm "
          f"(globals {G['AXLE_Z']:.0f}), radius {wheel_r / mm:.0f} mm")
    print(f"front spring: k {k:.0f} N/m, preload {preload:.1f} N, travel {travel / mm:.0f} mm")
    if open_meshes:
        print(f"bounding-box inertia (mesh not a closed solid): {', '.join(sorted(set(open_meshes)))}")
    if skipped:
        print(f"not in the BOM, left out: {', '.join(skipped)}")
    cmake = os.path.join(a.pkg_dir, "CMakeLists.txt")
    if os.path.exists(cmake) and not re.search(r"install\(DIRECTORY[^)]*\bmeshes\b", open(cmake).read()):
        print("NOTE: add 'meshes' to install(DIRECTORY ...) in CMakeLists.txt so package:// finds the meshes")
    print(f"wrote {out}\n      {mesh_dir}")


if __name__ == "__main__":
    main()
