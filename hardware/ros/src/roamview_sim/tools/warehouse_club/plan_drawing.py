#!/usr/bin/env python3
"""Draws the warehouse club plan (to scale) from club_layout.py and checks routes against obstacles."""
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, FancyArrowPatch
from shapely.geometry import LineString, Point, box
from shapely.ops import unary_union

import club_layout as L

INK, PAPER = "#1d2a33", "#f7f8f6"
C = dict(rack="#e07b28", rack_fill="#f6d9bf", table="#9aa7b1", checkout="#44525c", cooler="#7fb3d5",
         freezer="#4f8fc0", case="#8e6fb3", wall=INK, office="#f3ecd2", receiving="#e5e5e5", fresh="#e3f0f8",
         front="#eef2ee", route="#1f5fbf", shopper="#2e8b57", hazard="#c0392b", dock="#2e8b57", column="#20262b",
         food="#c9b28a", corral="#8a8f94")

obstacles = []   # shapely boxes of everything a robot cannot drive through


def rect(ax, x0, y0, w, h, fc, ec=None, lw=0.6, z=2, obstacle=True, **kw):
    ax.add_patch(Rectangle((x0, y0), w, h, facecolor=fc, edgecolor=ec or fc, linewidth=lw, zorder=z, **kw))
    if obstacle:
        obstacles.append(box(x0, y0, x0 + w, y0 + h))


fig = plt.figure(figsize=(17, 11), dpi=150, facecolor=PAPER)
ax = fig.add_axes([0.03, 0.06, 0.70, 0.86])
ax.set_facecolor(PAPER)
ax.set_aspect("equal")
ax.set_xlim(-6, L.W + 6)
ax.set_ylim(-9, L.D + 7)
ax.axis("off")

# zones (floor tints, not obstacles)
rect(ax, 0, 0, L.W, 14, C["front"], obstacle=False, z=0)
rect(ax, L.OFFICE[0], L.OFFICE[2], L.OFFICE[1] - L.OFFICE[0], L.OFFICE[3] - L.OFFICE[2], C["office"], obstacle=False, z=0)
rect(ax, L.RECEIVING[0], L.RECEIVING[2], L.RECEIVING[1] - L.RECEIVING[0], L.RECEIVING[3] - L.RECEIVING[2], C["receiving"],
     obstacle=False, z=0, hatch="////")
rect(ax, 40, 84, 70, 16, C["fresh"], obstacle=False, z=0)

# outer walls with door gaps
def wall_segments(side, length, doors):
    cuts = sorted((a, b) for s, a, b, _ in doors if s == side)
    segs, pos = [], 0.0
    for a, b in cuts:
        segs.append((pos, a)); pos = b
    segs.append((pos, length))
    return segs

T = L.WALL
for a, b in wall_segments("S", L.W, L.DOORS):
    rect(ax, a, -T, b - a, T, C["wall"], z=5)
for a, b in wall_segments("N", L.W, L.DOORS):
    rect(ax, a, L.D, b - a, T, C["wall"], z=5)
rect(ax, -T, -T, T, L.D + 2 * T, C["wall"], z=5)
rect(ax, L.W, -T, T, L.D + 2 * T, C["wall"], z=5)

# inner walls: office (east wall with door), receiving (south wall with door, west wall), prep room
ox1 = L.OFFICE[1]
rect(ax, ox1, L.OFFICE[2], 0.2, L.OFFICE_DOOR[0] - L.OFFICE[2], C["wall"], z=5)
rect(ax, ox1, L.OFFICE_DOOR[1], 0.2, L.D - L.OFFICE_DOOR[1], C["wall"], z=5)
rect(ax, 0, L.OFFICE[2], ox1, 0.2, C["wall"], z=5)
rx0 = L.RECEIVING[0]
rect(ax, rx0, L.RECEIVING[2], L.RECEIVING_DOOR[0] - rx0, 0.2, C["wall"], z=5)
rect(ax, L.RECEIVING_DOOR[1], L.RECEIVING[2], L.W - L.RECEIVING_DOOR[1], 0.2, C["wall"], z=5)
rect(ax, rx0, L.RECEIVING[2], 0.2, L.D - L.RECEIVING[2], C["wall"], z=5)
px0, px1, py = L.PREP_ROOM_WALL
rect(ax, px0, py, px1 - px0, 0.2, C["wall"], z=5)
rect(ax, px0, py, 0.2, L.D - py, C["wall"], z=5)

# racking
def rack(x0, x1, y0, y1):
    rect(ax, x0, y0, x1 - x0, y1 - y0, C["rack_fill"], C["rack"], lw=0.8, z=3)
    y = y0 + L.BAY
    while y < y1 - 0.1:
        ax.plot([x0, x1], [y, y], color=C["rack"], lw=0.35, zorder=3.5)
        y += L.BAY
    if x1 - x0 > 2:
        ax.plot([(x0 + x1) / 2] * 2, [y0, y1], color=C["rack"], lw=0.35, zorder=3.5)

for x in L.WEST_ROWS + L.EAST_ROWS:
    for y0, y1 in L.ROW_SEGMENTS:
        rack(x, x + L.RACK_DEPTH, y0, y1)
for x0, x1, y0, y1 in L.PERIMETER_RACKS:
    for s0, s1 in L.ROW_SEGMENTS:
        rack(x0, x1, s0, s1)

# centre tables
tw, th = L.TABLE_SIZE
for x, y in L.TABLES:
    rect(ax, x, y, tw, th, C["table"], z=3)

# front of store
for x, y in L.CHECKOUTS:
    rect(ax, x, y, 0.9, 5.0, C["checkout"], z=3)
    rect(ax, x - 0.9, y + 3.2, 0.6, 0.6, C["checkout"], z=3)       # register stand
for x0, y0, x1, y1 in L.CART_CORRALS:
    rect(ax, x0, y0, x1 - x0, y1 - y0, C["corral"], z=3)
sx0, sx1, sy0, sy1 = L.SERVICE_DESK
rect(ax, sx0, sy0, sx1 - sx0, sy1 - sy0, C["checkout"], z=3)
for x, y in L.FOOD_TABLES:
    rect(ax, x, y, 1.2, 0.8, C["food"], z=3)
fx0, fx1, fy0, fy1 = L.FOOD_COUNTER
rect(ax, fx0, fy0, fx1 - fx0, fy1 - fy0, C["checkout"], z=3)
rect(ax, L.RECEIPT_PODIUM[0] - 0.3, L.RECEIPT_PODIUM[1] - 0.3, 0.6, 0.6, C["checkout"], z=3)
mx0, mx1, my0, my1, _ = L.ENTRY_MAT
rect(ax, mx0, my0, mx1 - mx0, my1 - my0, "#6d6250", obstacle=False, z=1, alpha=0.5)

# back of store
cx0, cx1, cy0, cy1, _ = L.COOLER_WALL
rect(ax, cx0, cy0, cx1 - cx0, cy1 - cy0, C["cooler"], "#3c7aa6", z=3)
for x, y in L.FREEZER_CHESTS:
    rect(ax, x, y, 4.5, 1.1, C["freezer"], z=3)
for x, y in L.SERVICE_CASES:
    rect(ax, x, y, 4.2, 1.0, C["case"], z=3)
# office furniture and receiving pallets (indicative)
for i in range(3):
    rect(ax, 4 + 5.5 * i, 95.5, 3.0, 1.4, C["table"], z=3)
rect(ax, 3.0, 85.2, 4.0, 1.6, C["food"], z=3)
for i in range(4):
    for j in range(2):
        rect(ax, 112 + 4.2 * i, 88.5 + 3.0 * j + (5.0 if j else 0), 1.2, 1.0, "#b5895a", z=3)
# charging dock
dx, dy = L.CHARGING_DOCK
rect(ax, dx - 0.3, dy - 0.4, 0.6, 0.8, C["dock"], z=6, obstacle=False)
ax.annotate("Charging dock (AprilTag)", (dx + 0.3, dy + 0.3), (8.2, 93.9), fontsize=7.5, color=C["dock"],
            va="center", arrowprops=dict(arrowstyle="-", color=C["dock"], lw=0.8), zorder=8)

# columns
for x, y in L.COLUMNS:
    rect(ax, x - 0.25, y - 0.25, 0.5, 0.5, C["column"], z=6)

# dock edges (drop to the truck apron)
for s, a, b, name in L.DOORS:
    if s == "N":
        ax.plot([a, b], [L.D + 0.15] * 2, color=C["hazard"], lw=3.2, zorder=7, solid_capstyle="butt")
        rect(ax, a, L.D + T, b - a, 2.6, "#d9d4c7", obstacle=False, z=0)
ax.text(119.0, L.D + 3.3, "Truck apron, 1.22 m below the floor", ha="center", fontsize=7.5, color=C["hazard"])

# routes
def draw_route(pts, color, lw, ls, z):
    xs, ys = zip(*pts)
    ax.plot(xs, ys, color=color, lw=lw, ls=ls, zorder=z)

draw_route(L.PATROL, C["route"], 1.6, (0, (5, 2.5)), 9)
for i in range(0, len(L.PATROL) - 1, 3):
    (x0, y0), (x1, y1) = L.PATROL[i], L.PATROL[i + 1]
    if (x1 - x0) ** 2 + (y1 - y0) ** 2 > 16:
        ax.add_patch(FancyArrowPatch((x0, y0), ((x0 + x1) / 2, (y0 + y1) / 2), arrowstyle="-|>",
                                     mutation_scale=11, color=C["route"], lw=0, zorder=9))
for route in L.SHOPPERS:
    draw_route(route, C["shopper"], 1.0, "-", 8)
    ax.plot(*route[0], marker="o", color=C["shopper"], ms=4.5, zorder=8)

# spawn points
for name, (x, y, yaw) in L.SPAWNS.items():
    ax.plot(x, y, marker="^", color=C["route"], ms=8, mec="white", mew=0.8, zorder=10)

# hazards
for k, (name, (x, y)) in enumerate(L.HAZARDS, 1):
    ax.plot(x, y, marker="o", ms=11, color=C["hazard"], mec="white", mew=1.0, zorder=11)
    ax.text(x, y, str(k), color="white", fontsize=7, ha="center", va="center", fontweight="bold", zorder=12)

# labels
lab = dict(fontsize=9, color=INK, ha="center", va="center", zorder=12, fontweight="bold")
ax.text(24.2, 61.5, "Pallet racking, 5.5 m\n(grocery, household)", rotation=90, **lab)
ax.text(105.8, 34.5, "Pallet racking, 5.5 m\n(bulk, paper, drinks)", rotation=90, **lab)
ax.text(65.0, 49.0, "Cross aisle 4 m", fontsize=8, color=INK, ha="center", va="center", zorder=12,
        bbox=dict(fc=PAPER, ec="none", pad=1.2))
ax.text(65.0, 75.9, "Display tables 0.8 m (electronics, clothing, books)", fontsize=8.5, color=INK, ha="center",
        zorder=12, bbox=dict(fc=PAPER, ec="none", pad=1.2))
ax.text(65.0, 17.0, "Front racetrack aisle, 6 m", fontsize=8, color=INK, ha="center", va="center", zorder=12,
        bbox=dict(fc=PAPER, ec="none", pad=1.0))
ax.text(65.0, 81.0, "Back racetrack aisle, 6 m", fontsize=8, color=INK, ha="center", va="center", zorder=12,
        bbox=dict(fc=PAPER, ec="none", pad=1.0))
ax.text(66.0, 11.2, "14 checkout lanes", fontsize=8.5, color="white", ha="center", va="center", zorder=12,
        bbox=dict(fc=C["checkout"], ec="none", pad=1.5))
ax.text(8.5, 11.2, "Food court", fontsize=8.5, color=INK, ha="center", zorder=12)
ax.text(104.0, 7.0, "Carts", fontsize=7.5, color=INK, ha="center", zorder=12)
ax.text(123.8, 8.3, "Membership desk", fontsize=7.5, color=INK, ha="center", zorder=12)
ax.text(65.0, 99.0 - 0.2, "Glass-door fridges (dairy)", fontsize=7.5, color=INK, ha="center", va="center", zorder=12)
ax.text(65.0, 91.8, "Freezer chests 0.9 m", fontsize=7.5, color=INK, ha="center", zorder=12)
ax.text(100.0, 89.4, "Meat and bakery cases", fontsize=7.5, color=INK, ha="center", zorder=12)
ax.text(100.0, 97.5, "Prep room", fontsize=7.5, color=INK, ha="center", zorder=12)
ax.text(127.6, 92.0, "Receiving", fontsize=9, color=INK, ha="center", va="center", rotation=90, zorder=12,
        fontweight="bold", bbox=dict(fc=C["receiving"], ec="none", pad=1.2))
ax.text(11.0, 98.5, "Office and break room", fontsize=8.5, color=INK, ha="center", va="center", zorder=12)
for s, a, b, name in L.DOORS:
    y = -2.1 if s == "S" else L.D + 1.5
    if s == "S":
        ax.text((a + b) / 2, y, name, fontsize=8, color=INK, ha="center", va="center", zorder=12, fontweight="bold")
        ax.add_patch(FancyArrowPatch(((a + b) / 2, -4.3 if name == "Entrance" else -0.6),
                                     ((a + b) / 2, -0.6 if name == "Entrance" else -4.3),
                                     arrowstyle="-|>", mutation_scale=12, color=INK, lw=1.2, zorder=12))

# dimensions, scale bar, north arrow
def dim(x0, y0, x1, y1, text, off):
    ax.annotate("", (x0, y0), (x1, y1), arrowprops=dict(arrowstyle="<->", color=INK, lw=0.8))
    ax.text((x0 + x1) / 2 + off[0], (y0 + y1) / 2 + off[1], text, fontsize=9, ha="center", va="center", color=INK)
dim(0, -7.2, L.W, -7.2, "130 m", (0, -1.3))
dim(L.W + 3.6, 0, L.W + 3.6, L.D, "100 m", (1.6, 0))
for i in range(5):
    rect(ax, 2 + 2 * i, -7.9, 2, 0.7, INK if i % 2 == 0 else PAPER, INK, lw=0.6, obstacle=False, z=12)
ax.text(7, -6.4, "10 m", fontsize=8, ha="center", color=INK)
ax.add_patch(FancyArrowPatch((L.W + 3.6, L.D + 2), (L.W + 3.6, L.D + 6.5), arrowstyle="-|>", mutation_scale=14,
                             color=INK, lw=1.4, zorder=12))
ax.text(L.W + 3.6, L.D + 1.2, "N", fontsize=10, ha="center", va="center", fontweight="bold", color=INK)
fig.text(0.03, 0.955, "Warehouse club world for Gazebo: proposed layout", fontsize=18, fontweight="bold", color=INK)
fig.text(0.03, 0.93, "To scale, 130 x 100 m sales building, 11 m clear height. Generic and unbranded. "
         "Origin at the south-west inside corner, x east, y north.", fontsize=10.5, color="#4b5a64")

# legend / notes column
lx = 0.755
fig.text(lx, 0.89, "Key", fontsize=13, fontweight="bold", color=INK)
items = [(C["rack_fill"], C["rack"], "Pallet racking, 2.7 m bays, 3.6 m aisles"),
         (C["table"], C["table"], "Display tables"),
         (C["checkout"], C["checkout"], "Counters, checkouts, desks"),
         (C["cooler"], "#3c7aa6", "Glass-door fridges"), (C["freezer"], C["freezer"], "Freezer chests"),
         (C["case"], C["case"], "Service cases"), (C["column"], C["column"], "Steel columns (with bollards)"),
         (C["food"], C["food"], "Food court and break-room tables")]
for k, (fc, ec, text) in enumerate(items):
    y = 0.86 - k * 0.026
    fig.patches.append(Rectangle((lx, y - 0.006), 0.016, 0.013, transform=fig.transFigure, facecolor=fc, edgecolor=ec))
    fig.text(lx + 0.022, y, text, fontsize=9.5, color=INK, va="center")
y = 0.86 - len(items) * 0.026
lines = [(C["route"], (0, (5, 2.5)), "Patrol route (Nav2 waypoints)"),
         (C["shopper"], "-", "Shopper walking routes (people:=true)"),
         (C["hazard"], "-", "Dock edges: 1.22 m drop")]
for k, (col, ls, text) in enumerate(lines):
    yy = y - k * 0.026
    fig.lines.append(plt.Line2D([lx, lx + 0.016], [yy, yy], transform=fig.transFigure, color=col, ls=ls, lw=2))
    fig.text(lx + 0.022, yy, text, fontsize=9.5, color=INK, va="center")
yy = y - len(lines) * 0.026
fig.lines.append(plt.Line2D([lx + 0.008], [yy], transform=fig.transFigure, marker="^", color=C["route"], ms=8))
fig.text(lx + 0.022, yy, "Spawn points: charging dock, entrance, receiving", fontsize=9.5, color=INK, va="center")
yy -= 0.045
fig.text(lx, yy, "Things RoamView should notice", fontsize=13, fontweight="bold", color=INK)
for k, (name, _) in enumerate(L.HAZARDS, 1):
    y2 = yy - 0.03 - (k - 1) * 0.026
    fig.lines.append(plt.Line2D([lx + 0.008], [y2], transform=fig.transFigure, marker="o", ms=11, color=C["hazard"]))
    fig.text(lx + 0.008, y2, str(k), fontsize=7, color="white", ha="center", va="center", fontweight="bold")
    fig.text(lx + 0.022, y2, name, fontsize=9.5, color=INK, va="center")
y3 = yy - 0.03 - len(L.HAZARDS) * 0.026 - 0.02
notes = ["Also: 100 m long aisles (hard for LiDAR SLAM),",
         "glass fridge doors, carts with thin wire frames,",
         "pallets below the 286 mm LiDAR plane."]
for k, t in enumerate(notes):
    fig.text(lx, y3 - k * 0.022, t, fontsize=9.5, color="#4b5a64")

# ---- route checks -----------------------------------------------------------------------------
solid = unary_union(obstacles)
problems = []
for name, pts, r in [("patrol", L.PATROL, 0.30)] + [(f"shopper {i + 1}", p, 0.30) for i, p in enumerate(L.SHOPPERS)]:
    for a, b in zip(pts, pts[1:]):
        hit = LineString([a, b]).buffer(r).intersection(solid)
        if not hit.is_empty and hit.area > 1e-3:
            problems.append(f"{name} {a}->{b}: {hit.area:.2f} m2 near "
                            f"{tuple(round(v, 1) for v in hit.centroid.coords[0])}")
for name, (x, y, _) in L.SPAWNS.items():
    if Point(x, y).buffer(0.25).intersects(solid):
        problems.append(f"spawn {name} inside an obstacle")
print("\n".join(problems) if problems else "routes clear")
out = Path(sys.argv[1] if len(sys.argv) > 1 else "warehouse_club_plan.png")
fig.savefig(out, facecolor=PAPER)
print("wrote", out)
