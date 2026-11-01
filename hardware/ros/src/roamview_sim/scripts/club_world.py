#!/usr/bin/env python3
"""Writes a warehouse club world for Gazebo with your settings. The launch file runs this for you; you can also
run it by hand and open the result with `gz sim`:

    ros2 run roamview_sim club_world.py --lighting night --people 20 --clutter high --seed 4 -o ~/club.sdf
    GZ_SIM_RESOURCE_PATH=$(ros2 pkg prefix roamview_sim)/share/roamview_sim/models gz sim ~/club.sdf

Settings
  --lighting  day | bright | dim | night   store lights and the light Gazebo adds (night = emergency lights only)
  --shadows   on | off                     shadows cost GPU time; off is faster on a weak PC
  --people    0 .. 40                      shoppers walking loops (first 8 follow hand-made routes)
  --clutter   none | low | medium | high, or a number of items   loose cartons, carts, pallets... next to shelves
  --seed      N                            same seed = same people routes and clutter
  --hazards   on | off                     the 4 fixed test hazards (pallet jack, empty pallet, wet-floor sign, cart)
  --rtf       0.1 .. 1                     real-time factor: slow the simulation down on a slow PC
  --render-engine ogre2 | ogre             sensors' render engine (use ogre on WSL, see the README)

Only the Python standard library is used, so it runs anywhere ROS 2 runs.
"""
import argparse
import heapq
import json
import math
import os
import random
import sys

LIGHTING = {
    # ambient, key light (from above, casts shadows), fill, bounce (lights the roof deck), sky seen through doors
    "day": dict(ambient=0.48, key=0.82, fill=0.28, bounce=0.22, sky=(0.62, 0.66, 0.70), deck="deck_day",
                fixtures=("fixtures_a_on", "fixtures_b_on"), emergency=False),
    "bright": dict(ambient=0.56, key=1.0, fill=0.34, bounce=0.26, sky=(0.70, 0.74, 0.78), deck="deck_day",
                   fixtures=("fixtures_a_on", "fixtures_b_on"), emergency=False),
    "dim": dict(ambient=0.24, key=0.40, fill=0.13, bounce=0.10, sky=(0.35, 0.38, 0.42), deck="deck_night",
                fixtures=("fixtures_a_on", "fixtures_b_off"), emergency=False),
    "night": dict(ambient=0.05, key=0.0, fill=0.03, bounce=0.02, sky=(0.02, 0.03, 0.05), deck="deck_night",
                  fixtures=("fixtures_a_off", "fixtures_b_off"), emergency=True),
}
CLUTTER_LEVELS = {"none": 0, "low": 15, "medium": 40, "high": 90}
# prop, weight, half width across the shelf face (m)
CLUTTER_MIX = [("carton", 30, 0.15), ("carton_stack", 8, 0.15), ("spill", 8, 0.45), ("water_bundle", 8, 0.14),
               ("pallet", 8, 0.5), ("cart", 12, 0.31), ("wet_floor_sign", 6, 0.16), ("platform_truck", 8, 0.3),
               ("ladder", 4, 0.34), ("bin", 3, 0.28), ("pallet_jack", 5, 0.28)]
EMERGENCY_LIGHTS = [(x, y) for y in (17.0, 81.0) for x in (10.0, 40.0, 65.0, 90.0, 120.0)] + \
                   [(20.0, 49.0), (65.0, 49.0), (110.0, 49.0), (11.0, 92.0), (120.0, 92.0)]
WALK = "https://fuel.gazebosim.org/1.0/Mingfei/models/actor/tip/files/meshes/walk.dae"


def _f(v):
    return f"{v:.4f}".rstrip("0").rstrip(".") if isinstance(v, float) else str(v)


def _pose(*v):
    return " ".join(_f(float(x)) for x in v)


def _box_collision(name, c):
    x0, y0, z0, x1, y1, z1 = c
    return (f'<collision name="{name}"><pose>{_pose((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2, 0, 0, 0)}</pose>'
            f'<geometry><box><size>{_pose(abs(x1 - x0), abs(y1 - y0), abs(z1 - z0))}</size></box></geometry>'
            f'</collision>')


def store_visuals(manifest, lighting="day", uri_prefix="model://warehouse_club/"):
    L = LIGHTING[lighting]
    wanted = {"static", L["deck"], *L["fixtures"]}
    out = []
    for m in manifest["meshes"]:
        if m["group"] in wanted:
            out.append(f'<visual name="{m["name"]}"><cast_shadows>{"true" if m["shadows"] else "false"}</cast_shadows>'
                       f'<geometry><mesh><uri>{uri_prefix}{m["uri"]}</uri></mesh></geometry></visual>')
    return out


def store_model(manifest, lighting="day", uri_prefix="model://warehouse_club/"):
    vis = "\n        ".join(store_visuals(manifest, lighting, uri_prefix))
    cols = "\n        ".join(_box_collision(f"c{k}", c) for k, c in enumerate(manifest["collisions"]))
    return f"""    <model name="warehouse_club">
      <static>true</static>
      <link name="store">
        {vis}
        {cols}
      </link>
    </model>
"""


def store_model_sdf(manifest, lighting="day", uri_prefix="model://warehouse_club/"):
    return '<?xml version="1.0"?>\n<sdf version="1.9">\n' + store_model(manifest, lighting, uri_prefix) + "</sdf>\n"


def prop_model(manifest, prop, name, x, y, yaw, uri_prefix="model://warehouse_club/"):
    p = manifest["props"][prop]
    cols = "".join(_box_collision(f"c{k}", c) for k, c in enumerate(p["collisions"]))
    static = p["mass"] is None
    inertial = ""
    if not static:
        x0, y0, z0, x1, y1, z1 = p["collisions"][0]
        m, a, b, c = p["mass"], x1 - x0, y1 - y0, z1 - z0
        inertial = (f'<inertial><pose>{_pose((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2, 0, 0, 0)}</pose>'
                    f'<mass>{_f(float(m))}</mass><inertia><ixx>{m * (b * b + c * c) / 12:.5f}</ixx>'
                    f'<iyy>{m * (a * a + c * c) / 12:.5f}</iyy><izz>{m * (a * a + b * b) / 12:.5f}</izz>'
                    f'<ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>')
    return f"""    <model name="{name}">
      <static>{"true" if static else "false"}</static>
      <pose>{_pose(x, y, 0.0 if static else 0.002, 0, 0, yaw)}</pose>
      <link name="link">{inertial}
        <visual name="v"><geometry><mesh><uri>{uri_prefix}{p["uri"]}</uri></mesh></geometry></visual>
        {cols}
      </link>
    </model>
"""


def header(name, lighting, shadows, rtf=1.0, render_engine="ogre2"):
    L = LIGHTING[lighting]
    a = L["ambient"]
    sky = " ".join(_f(v) for v in L["sky"]) + " 1"
    sh = "true" if shadows else "false"
    s = f"""<?xml version="1.0"?>
<!-- GENERATED by club_world.py: lighting={lighting}, shadows={sh}, rtf={rtf}, render engine {render_engine}.
     Edit the settings, not this file. -->
<sdf version="1.9">
  <world name="{name}">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>{_f(float(rtf))}</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>{render_engine}</render_engine>
    </plugin>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    <plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"/>
    <gravity>0 0 -9.81</gravity>
    <scene>
      <ambient>{_f(a)} {_f(a)} {_f(a * 1.02)} 1</ambient>
      <background>{sky}</background>
      <shadows>{sh}</shadows>
      <grid>false</grid>
    </scene>
"""
    if L["key"] > 0:
        k = L["key"]
        s += f"""    <!-- light from above: the roof casts no shadow, so this reads as the ceiling fixtures' light -->
    <light type="directional" name="key">
      <cast_shadows>{sh}</cast_shadows>
      <pose>0 0 30 0 0 0</pose>
      <diffuse>{_f(k)} {_f(k * 0.99)} {_f(k * 0.95)} 1</diffuse>
      <specular>0.25 0.25 0.25 1</specular>
      <direction>-0.16 0.11 -1.0</direction>
    </light>
"""
    s += f"""    <light type="directional" name="fill">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 30 0 0 0</pose>
      <diffuse>{_f(L["fill"])} {_f(L["fill"] * 1.03)} {_f(L["fill"] * 1.1)} 1</diffuse>
      <specular>0 0 0 1</specular>
      <direction>0.5 -0.4 -0.6</direction>
    </light>
    <light type="directional" name="bounce">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 0 0 0 0</pose>
      <diffuse>{_f(L["bounce"])} {_f(L["bounce"])} {_f(L["bounce"] * 1.04)} 1</diffuse>
      <specular>0 0 0 1</specular>
      <direction>0.1 0.05 1.0</direction>
    </light>
"""
    if L["emergency"]:
        for k, (x, y) in enumerate(EMERGENCY_LIGHTS):
            s += f"""    <light type="point" name="emergency_{k}">
      <cast_shadows>false</cast_shadows>
      <pose>{_pose(x, y, 7.0, 0, 0, 0)}</pose>
      <diffuse>0.55 0.55 0.5 1</diffuse>
      <specular>0 0 0 1</specular>
      <attenuation><range>22</range><constant>0.3</constant><linear>0.03</linear><quadratic>0.006</quadratic></attenuation>
    </light>
"""
    return s


# ---- people ------------------------------------------------------------------------------------------------------
def actor(name, route, speed):
    t, pts = 0.0, []
    for i, (x, y) in enumerate(route[:-1]):
        nx, ny = route[i + 1]
        yaw = math.atan2(ny - y, nx - x)
        if pts:
            pts.append((t, x, y, pts[-1][3]))                  # arrive still facing the old way ...
            t += 0.5                                            # ... then turn on the spot
        pts.append((t, x, y, yaw))
        t += math.hypot(nx - x, ny - y) / speed
    x, y = route[-1]
    pts.append((t, x, y, pts[-1][3]))
    wp = "".join(f"\n          <waypoint><time>{tt:.2f}</time><pose>{_pose(px, py, 1.0, 0, 0, yw)}</pose></waypoint>"
                 for tt, px, py, yw in pts)
    return f"""    <actor name="{name}">
      <skin><filename>{WALK}</filename><scale>1.0</scale></skin>
      <animation name="walk"><filename>{WALK}</filename><interpolate_x>true</interpolate_x></animation>
      <script>
        <loop>true</loop>
        <auto_start>true</auto_start>
        <trajectory id="0" type="walk" tension="0.6">{wp}
        </trajectory>
      </script>
    </actor>
"""


def random_loop(nodes, edges, rng, min_len=120.0, max_len=320.0):
    adj = {}
    for a, b in edges:
        d = math.dist(nodes[a], nodes[b])
        adj.setdefault(a, []).append((b, d))
        adj.setdefault(b, []).append((a, d))
    start = rng.choice(sorted(adj))
    path, length, prev = [start], 0.0, None
    target = rng.uniform(min_len, max_len) * 0.6
    while length < target:
        opts = [(n, d) for n, d in adj[path[-1]] if n != prev] or adj[path[-1]]
        n, d = rng.choice(opts)
        prev = path[-1]
        path.append(n)
        length += d
    dist, back, heap = {path[-1]: 0.0}, {}, [(0.0, path[-1])]   # shortest way home
    while heap:
        du, u = heapq.heappop(heap)
        if u == start:
            break
        if du > dist.get(u, 1e18):
            continue
        for v, d in adj[u]:
            if du + d < dist.get(v, 1e18):
                dist[v], back[v] = du + d, u
                heapq.heappush(heap, (du + d, v))
    home, u = [], start
    while u != path[-1]:
        home.append(u)
        u = back[u]
    path += home[::-1]
    pts = [tuple(nodes[i]) for i in path]
    clean = [pts[0]]                                          # drop points in the middle of straight runs
    for i in range(1, len(pts) - 1):
        (x0, y0), (x1, y1), (x2, y2) = clean[-1], pts[i], pts[i + 1]
        if abs((x1 - x0) * (y2 - y1) - (y1 - y0) * (x2 - x1)) > 1e-6 or (x1 - x0) * (x2 - x1) + (y1 - y0) * (y2 - y1) < 0:
            clean.append(pts[i])
    clean.append(pts[-1])
    return clean


def people_actors(manifest, n, rng):
    out = []
    for k in range(n):
        if k < len(manifest["shoppers"]):
            route = [tuple(p) for p in manifest["shoppers"][k]]
        else:
            route = random_loop(manifest["walk_nodes"], manifest["walk_edges"], rng)
        out.append(actor(f"shopper_{k + 1}", route, speed=rng.uniform(0.85, 1.3)))
    return out


# ---- clutter ---------------------------------------------------------------------------------------------------------
def clutter_models(manifest, count, rng, uri_prefix):
    spots = list(manifest["clutter_spots"])
    rng.shuffle(spots)
    placed, out = [], []
    kinds = [k for k, w, _ in CLUTTER_MIX for _ in range(w)]
    half = {k: h for k, _, h in CLUTTER_MIX}
    n = 0
    for x, y, yaw, clear, zone in spots:
        if n >= count:
            break
        if any(math.hypot(x - px, y - py) < 2.5 for px, py in placed):
            continue
        kind = rng.choice(kinds)
        if half[kind] > clear - 0.05:
            kind = "carton"
        yaw_j = yaw + rng.uniform(-0.25, 0.25)
        if kind == "spill":
            for j in range(3):
                dx, dy = (j - 1) * 0.5 + rng.uniform(-0.03, 0.03), rng.uniform(-0.08, 0.08)
                c, s = math.cos(yaw), math.sin(yaw)
                out.append(prop_model(manifest, f"carton_{rng.randrange(18):02d}", f"clutter_{n}_{j}",
                                      x + c * dx - s * dy, y + s * dx + c * dy, yaw + rng.uniform(-0.3, 0.3), uri_prefix))
        elif kind == "carton":
            out.append(prop_model(manifest, f"carton_{rng.randrange(18):02d}", f"clutter_{n}", x, y, yaw_j, uri_prefix))
        else:
            if kind in ("cart", "platform_truck", "pallet_jack") and rng.random() < 0.5:
                yaw_j += math.pi
            out.append(prop_model(manifest, kind, f"clutter_{n}", x, y, yaw_j, uri_prefix))
        placed.append((x, y))
        n += 1
    return out, n


def parse_count(v, table):
    if isinstance(v, int):
        return v
    s = str(v).strip().lower()
    if s in table:
        return table[s]
    if s in ("true", "yes", "on"):
        return table.get("true", 8)
    if s in ("false", "no", "off", ""):
        return 0
    return max(0, int(float(s)))


def world_sdf(manifest, lighting="day", shadows=True, people=0, clutter="none", seed=1, hazards=True,
              uri_prefix="model://warehouse_club/", name="warehouse_club", extra="", rtf=1.0, render_engine="ogre2"):
    if lighting not in LIGHTING:
        raise ValueError(f"lighting must be one of {', '.join(LIGHTING)}")
    rng = random.Random(seed)
    n_people = min(40, parse_count(people, {"true": 8}))
    n_clutter = parse_count(clutter, CLUTTER_LEVELS)
    s = header(name, lighting, shadows, rtf, render_engine)
    s += store_model(manifest, lighting, uri_prefix)
    if hazards:
        for k, h in enumerate(manifest["hazards"]):
            s += prop_model(manifest, h["prop"], f"hazard_{k + 1}_{h['prop']}", h["x"], h["y"], h["yaw"], uri_prefix)
    items, placed = clutter_models(manifest, n_clutter, random.Random(seed * 7919 + 1), uri_prefix)
    s += "".join(items)
    s += "".join(people_actors(manifest, n_people, random.Random(seed * 104729 + 3)))
    s += extra + "  </world>\n</sdf>\n"
    summary = dict(lighting=lighting, shadows=shadows, people=n_people, clutter=placed, hazards=bool(hazards), seed=seed,
                   rtf=float(rtf), engine=render_engine)
    return s, summary


def on_off(v):
    return str(v).strip().lower() in ("1", "true", "on", "yes")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    here = os.path.dirname(os.path.abspath(__file__))
    ap.add_argument("--manifest", default=None, help="models/warehouse_club/manifest.json (found automatically)")
    ap.add_argument("--lighting", default="day", choices=list(LIGHTING))
    ap.add_argument("--shadows", default="on")
    ap.add_argument("--people", default="0")
    ap.add_argument("--clutter", default="none")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--hazards", default="on")
    ap.add_argument("--rtf", type=float, default=1.0, help="real-time factor Gazebo aims for (0.5 = half speed)")
    ap.add_argument("--render-engine", default="ogre2", choices=["ogre2", "ogre"],
                    help="engine for the robot's sensors: ogre2 (best look) or ogre (WSL: ogre2 leaks memory there)")
    ap.add_argument("-o", "--output", default="warehouse_club_custom.sdf")
    a = ap.parse_args(argv)
    path = a.manifest
    if path is None:
        for cand in (os.path.join(here, "..", "..", "share", "roamview_sim", "models", "warehouse_club", "manifest.json"),
                     os.path.join(here, "..", "models", "warehouse_club", "manifest.json")):
            if os.path.exists(cand):
                path = cand
                break
    if path is None:
        sys.exit("manifest.json not found: pass --manifest")
    with open(path) as f:
        manifest = json.load(f)
    text, summary = world_sdf(manifest, a.lighting, on_off(a.shadows), a.people, a.clutter, a.seed, on_off(a.hazards),
                              rtf=a.rtf, render_engine=a.render_engine)
    with open(os.path.expanduser(a.output), "w") as f:
        f.write(text)
    print(f"wrote {a.output}: {summary}")


if __name__ == "__main__":
    main()
