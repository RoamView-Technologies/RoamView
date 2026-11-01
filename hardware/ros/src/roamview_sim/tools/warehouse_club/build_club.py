#!/usr/bin/env python3
"""Builds the warehouse club world for Gazebo from club_layout.py (generic, unbranded, real scale).

    python3 tools/warehouse_club/build_club.py [OUT] [--stock normal|full|sparse] [--seed 7]

OUT defaults to the roamview_sim package (two folders up). Needs numpy, pillow, opencv (python3-opencv) and shapely.

Writes, under OUT:
  models/warehouse_club/meshes/*.gltf + .bin   store meshes, lighting variants and loose props (glTF 2.0)
  models/warehouse_club/textures/               shared textures (procedural, no brands)
  models/warehouse_club/model.sdf, model.config the store as one static model (day lighting)
  models/warehouse_club/manifest.json           everything the launch-time world generator needs
  maps/warehouse_club.{pgm,yaml}                occupancy map cut from the meshes at LiDAR height (286 mm)
  maps/warehouse_club_keepout.{pgm,yaml}        Nav2 keepout mask: dock edges, doorways, outside
  config/warehouse_club_patrol.yaml             patrol waypoints for scripts/patrol.py
"""
import argparse
import json
import math
import shutil
from pathlib import Path

import numpy as np

import club_geom as G
import club_layout as L
import club_props as P
import club_textures
from club_geom import MAT
from gltf_writer import Builder, box, cylinder, quad, quad_xy

W, D, H = L.W, L.D, L.H
ROOF_Z = H
LIDAR_Z = 0.286
STOCK = dict(full=dict(empty=(0.01, 0.03), picked=0.2), normal=dict(empty=(0.06, 0.12), picked=0.55),
             sparse=dict(empty=(0.28, 0.4), picked=0.85))


class Scene:
    """Named meshes plus the collision boxes that go with them."""

    def __init__(self):
        self.meshes, self.meta, self.cols = {}, {}, []

    def mesh(self, name, group="static", shadows=True, lidar=True):
        """group: 'static' (always in the world) or a variant tag (deck_day, fixtures_a_on, ...).
        lidar: include in the LiDAR-height slice that makes the map."""
        if name not in self.meshes:
            self.meshes[name] = Builder()
            self.meta[name] = dict(group=group, shadows=shadows, lidar=lidar)
        return self.meshes[name]

    def col(self, boxes):
        self.cols += [tuple(round(float(v), 4) for v in bx) for bx in boxes]


# ---- shell -----------------------------------------------------------------------------------------------------
def ext_wall(b, axis, t0, t1, a0, a1, height, openings, inner):
    """Outer wall between t0..t1 across its thickness, running a0..a1; inner face finished like the sales floor
    (block wainscot, metal panels), outer face metal panels. openings: [(start, end, top)]."""
    cols, p = [], a0
    spans = []
    for s, e, top in sorted(openings):
        if s > p:
            spans.append((p, s, 0.0))
        spans.append((s, e, top))
        p = e
    if p < a1:
        spans.append((p, a1, 0.0))
    outer = {"n": "s", "s": "n", "e": "w", "w": "e"}[inner]
    ends = "ew" if axis == "x" else "ns"
    for s, e, z0 in spans:
        if axis == "x":
            x0, y0, x1, y1 = s, t0, e, t1
        else:
            x0, y0, x1, y1 = t0, s, t1, e
        if z0 < 2.4:
            box(b, MAT["cmu"], x0, y0, z0, x1, y1, 2.4, tile=(2.4, 1.2), origin=(x0, y0, 0), faces=inner)
        box(b, MAT["wall"], x0, y0, max(2.4, z0), x1, y1, height, tile=(1.0, 2.0), origin=(x0, y0, 0), faces=inner)
        box(b, MAT["wall"], x0, y0, z0, x1, y1, height, tile=(1.0, 2.0), origin=(x0, y0, 0), faces=outer)
        box(b, MAT["dark"], x0, y0, z0, x1, y1, height, faces=ends + "tb")
        cols.append((x0, y0, z0, x1, y1, height))
    return cols


def shell(S):
    T = L.WALL
    fl = S.mesh("floor")
    G.floor(fl, -T, -6.0, W + T, D + T)                                   # sales floor + sidewalk, one slab
    mx0, mx1, my0, my1, mh = L.ENTRY_MAT
    box(fl, MAT["mat"], mx0, my0, 0, mx1, my1, mh, tile=(1.0, 1.0))
    S.col([(-T, -6.0, -0.3, W + T, D + T, 0.0), (mx0, my0, 0, mx1, my1, mh)])

    w = S.mesh("walls")
    doors_s = [(a, b, 3.0) for s, a, b, _ in L.DOORS if s == "S"]
    doors_n = [(a, b, 3.0) for s, a, b, _ in L.DOORS if s == "N"]
    S.col(ext_wall(w, "x", -T, 0.0, -T, W + T, ROOF_Z, doors_s, inner="n"))
    S.col(ext_wall(w, "x", D, D + T, -T, W + T, ROOF_Z, doors_n, inner="s"))
    S.col(ext_wall(w, "y", -T, 0.0, 0.0, D, ROOF_Z, [], inner="e"))
    S.col(ext_wall(w, "y", W, W + T, 0.0, D, ROOF_Z, [], inner="w"))

    # entrance and exit: door frames, sliding glass leaves parked open inside
    for s, a, b_, name in L.DOORS:
        if s != "S":
            continue
        for xf in (a, b_ - 0.1):
            box(w, MAT["black"], xf, -T, 0, xf + 0.1, 0.0, 3.0)
        box(w, MAT["black"], a, -T, 2.9, b_, 0.0, 3.0, faces="b")
        for x0, x1 in ((a - 1.45, a + 0.05), (b_ - 0.05, b_ + 1.45)):
            quad(w, MAT["glass"], (x0, 0.05, 0.02), (x1, 0.05, 0.02), (x1, 0.05, 2.9), (x0, 0.05, 2.9))
            for xf in (x0, x1 - 0.05):
                box(w, MAT["black"], xf, 0.03, 0.02, xf + 0.05, 0.08, 2.9)
            box(w, MAT["black"], x0, 0.03, 2.85, x1, 0.08, 2.92)
            box(w, MAT["black"], x0, 0.03, 0.0, x1, 0.08, 0.1)
        if name == "Exit":
            box(w, P.lit_sign("dept_exit", 0.55), (a + b_) / 2 - 1.6, 0.0, 3.15, (a + b_) / 2 + 1.6, 0.04, 3.75,
                tile=(3.2, 0.6), origin=((a + b_) / 2 + 1.6, 0.0, 3.15), faces="n")
        else:
            box(w, P.lit_sign("dept_entrance", 0.35), (a + b_) / 2 - 2.0, -T - 0.04, 3.3, (a + b_) / 2 + 2.0, -T,
                4.05, tile=(4.0, 0.75), origin=((a + b_) / 2 - 2.0, -T - 0.04, 3.3), faces="s")

    # roof: deck (lighting variants), joists per tile (culling), LED high-bays in two switchable groups
    for night in (False, True):
        P.deck(S.mesh("deck_night" if night else "deck_day", group="deck_night" if night else "deck_day",
                      shadows=False, lidar=False), -T, -T, W + T, D + T, ROOF_Z, night=night)
    xs = [0.0, 32.4, 64.8, 97.2, W]
    ys = [0.0, 33.4, 66.8, D]
    for i in range(4):
        for j in range(3):
            P.joists(S.mesh(f"roof_{i}{j}", shadows=False, lidar=False), xs[i], ys[j], xs[i + 1], ys[j + 1], ROOF_Z)
    spots = P.fixture_spots(W, D)
    for grp in "ab":
        for lit in (True, False):
            tag = f"fixtures_{grp}_{'on' if lit else 'off'}"
            P.fixtures(S.mesh(tag, group=tag, shadows=False, lidar=False), [s for s in spots if s[2] == grp],
                       ROOF_Z, lit=lit)
    return spots


def outside(S):
    """Sidewalk and parking in front, truck court 1.22 m down behind the dock doors, a parked trailer."""
    T = L.WALL
    o = S.mesh("outside")
    quad_xy(o, MAT["asphalt"], -30.0, -70.0, W + 30.0, -6.0, -0.15, tile=(8.0, 8.0))
    quad(o, P.MAT2["sidewalk"], (-T, -6.0, -0.15), (W + T, -6.0, -0.15), (W + T, -6.0, 0.0), (-T, -6.0, 0.0),
         uv=((0, 0.025), (22, 0.025), (22, 0), (0, 0)))
    for row_y in (-14.0, -31.0):                                 # two double rows of parking stalls
        for k in range(46):
            x = 3.0 + k * 2.75
            quad_xy(o, P.MAT2["line_white"], x - 0.05, row_y - 5.5, x + 0.05, row_y + 5.5, -0.148)
        quad_xy(o, P.MAT2["line_white"], 3.0, row_y - 0.05, 3.0 + 45 * 2.75, row_y + 0.05, -0.148)
    for xc in (25.0, 115.0):                                     # crosswalks to the doors
        for k in range(8):
            quad_xy(o, P.MAT2["line_white"], xc - 3.0, -8.0 - k * 0.8, xc + 3.0, -7.6 - k * 0.8, -0.148)
    S.col([(-30.0, -70.0, -0.45, W + 30.0, -6.0, -0.15)])
    for s, a, b_, _ in L.DOORS:
        if s == "S":
            for xb in (a - 0.8, b_ + 0.8):
                S.col(P.bollard(o, xb, -1.0))
    # north truck court
    zc = -L.DOCK_DROP
    quad_xy(o, MAT["asphalt"], -T, D + T, W + T, D + 26.0, zc, tile=(8.0, 8.0))
    U = (W + 2 * T) / 2.4
    quad(o, MAT["cmu"], (W + T, D + T, zc), (-T, D + T, zc), (-T, D + T, 0.0), (W + T, D + T, 0.0),
         uv=((0, L.DOCK_DROP / 1.2), (U, L.DOCK_DROP / 1.2), (U, 0), (0, 0)))
    quad(o, MAT["cmu"], (-T, D + 26.0, zc), (W + T, D + 26.0, zc), (W + T, D + 26.0, 0.0), (-T, D + 26.0, 0.0),
         uv=((0, L.DOCK_DROP / 1.2), (U, L.DOCK_DROP / 1.2), (U, 0), (0, 0)))
    quad_xy(o, P.MAT2["foliage"], -30.0, D + 26.0, W + 30.0, D + 60.0, 0.0, tile=(4.0, 4.0))
    S.col([(-T, D + T, zc - 0.3, W + T, D + 26.0, zc), (-T, D + 26.0, zc, W + T, D + 26.3, 0.0)])
    S.col(P.trailer(o, 101.0, D + 10.0, zc, length=16.0))


# ---- racking ---------------------------------------------------------------------------------------------------------
def racks(S, rng, stock):
    grocery, snacks, pet = [5, 6, 7, 8, 15], [12, 13, 17], [14]
    drinks, paper, house = [2, 3, 4], [0, 1, 16], [9, 10, 11]
    bays = int(round((L.ROW_SEGMENTS[0][1] - L.ROW_SEGMENTS[0][0]) / L.BAY))
    for k, x in enumerate(L.WEST_ROWS):
        for si, (y0, y1) in enumerate(L.ROW_SEGMENTS):
            prods = grocery if si == 0 else (snacks if k < 5 else pet + snacks)
            G.rack_run(S.mesh(f"racks_w{k}_{'sn'[si]}"), x, y0, bays, double=True, rng=rng, products=prods,
                       mix=(0.04, 0.0), **stock)
            S.col([(x - 0.1, y0 - 0.28, 0, x + L.RACK_DEPTH + 0.1, y1 + 0.28, 2.2)])
    for k, x in enumerate(L.EAST_ROWS):
        for si, (y0, y1) in enumerate(L.ROW_SEGMENTS):
            prods, mix = (drinks, (0.35, 0.0)) if si == 0 else (paper + house, (0.0, 0.3))
            G.rack_run(S.mesh(f"racks_e{k}_{'sn'[si]}"), x, y0, bays, double=True, rng=rng, products=prods, mix=mix,
                       **stock)
            S.col([(x - 0.1, y0 - 0.28, 0, x + L.RACK_DEPTH + 0.1, y1 + 0.28, 2.2)])
    for si, (y0, y1) in enumerate(L.ROW_SEGMENTS):
        G.rack_run(S.mesh(f"racks_pw_{'sn'[si]}"), 0.1, y0, bays, double=False, face_west=False, rng=rng,
                   products=house if si == 0 else pet, mix=(0.0, 0.0), **stock)
        S.col([(0.0, y0 - 0.28, 0, 0.1 + G.FRAME_D + 0.1, y1 + 0.28, 2.2)])
        G.rack_run(S.mesh(f"racks_pe_{'sn'[si]}"), W - 1.2, y0, bays, double=False, face_west=True, rng=rng,
                   products=paper if si == 0 else drinks, mix=(0.0, 0.25) if si == 0 else (0.3, 0.0), **stock)
        S.col([(W - 1.2 - 0.1, y0 - 0.28, 0, W, y1 + 0.28, 2.2)])


def endcaps(S, rng):
    """Promotional pallets in front of rack ends along the racetracks (1.7 m between the end guard and the
    racetrack). Placed at random, so the store is not the same turned round: AMCL needs that to tell the halves
    apart."""
    b = S.mesh("endcaps")
    rows = [(x, x + L.RACK_DEPTH) for x in L.WEST_ROWS + L.EAST_ROWS]
    y_front, y_back = L.ROW_SEGMENTS[0][0], L.ROW_SEGMENTS[1][1]
    for x0, x1 in rows:
        for end, yy in (("front", y_front - 0.35 - 1.0), ("back", y_back + 0.35)):
            r = rng.random()
            if r < 0.3:
                continue
            n = 1 if r < 0.65 else 2
            for k in range(n):
                px = (x0 + x1) / 2 - 0.6 if n == 1 else x0 + 0.05 + k * 1.25
                G.pallet_box(b, px, yy, 0.0, dx=1.2, dy=1.0, fork_faces="ns")
                p = int(rng.integers(18))
                hgt = 0.6 + 0.3 * int(rng.integers(1, 4))
                m = G.carton_mat(p) if rng.random() < 0.7 else (MAT["water"] if rng.random() < 0.5 else MAT["towel"])
                tile = (0.4, 0.3) if m.name.startswith("carton") else ((0.52, 0.28) if m is MAT["water"] else (0.48, 0.48))
                box(b, m, px + 0.03, yy + 0.03, G.PALLET_H, px + 1.17, yy + 0.97, G.PALLET_H + hgt, tile=tile,
                    origin=(px, yy, G.PALLET_H))
                S.col([(px, yy, 0, px + 1.2, yy + 1.0, G.PALLET_H + hgt)])


# ---- centre tables ------------------------------------------------------------------------------------------------------
def tables(S, rng):
    tw, tl = L.TABLE_SIZE
    for x, y in L.TABLES:
        south = y < 47
        west = x < 65
        kind = ("electronics" if west else "clothing") if south else ("books" if west else "cartons")
        name = f"tables_{'s' if south else 'n'}{'w' if west else 'e'}"
        S.col(P.display_table(S.mesh(name), x, y, kind, rng, w=tw, l=tl, h=L.TABLE_H))


# ---- front of store ------------------------------------------------------------------------------------------------------
def front(S, rng):
    c = S.mesh("front_checkouts")
    for i, (x, y) in enumerate(L.CHECKOUTS):
        S.col(P.checkout(c, x, y, i + 1))
    S.col(P.podium(c, *L.RECEIPT_PODIUM))

    f = S.mesh("front_food")
    for x, y in L.FOOD_TABLES:
        S.col(P.food_table(f, x, y))
    S.col(P.food_counter(f, *L.FOOD_COUNTER))
    box(f, MAT["stainless"], 16.4, 11.6, 0.1, 17.6, 12.2, 1.0, tile=(1.0, 0.5))           # condiment station
    box(f, MAT["black"], 16.43, 11.63, 0, 17.57, 12.17, 0.1)
    box(f, P.MAT2["white_top"], 16.35, 11.55, 1.0, 17.65, 12.25, 1.03)
    S.col([(16.4, 11.6, 0, 17.6, 12.2, 1.03)])
    for xb, yb in ((17.8, 2.4), (17.8, 8.4)):                                               # bins
        cylinder(f, MAT["dark"], xb, yb, 0, 0.95, 0.28, segments=18)
        S.col([(xb - 0.28, yb - 0.28, 0, xb + 0.28, yb + 0.28, 0.95)])
    P.sign2(f, 8.5, 12.9, 3.6, 6.0, 1.1, "dept_food_court")
    P.hang(f, 8.5, 12.9, 4.7, x_off=(-2.5, 2.5))

    e = S.mesh("front_east")
    for x0, y0, x1, y1 in L.CART_CORRALS:
        S.col(P.cart_corral(e, x0, y0, x1, y1, ncarts=7))
    S.col(P.membership_desk(e, *L.SERVICE_DESK))
    for k in range(5):                                                                      # carts by the entrance
        G.cart(e, 109.2, 6.6 + k * 0.33, yaw=math.pi / 2)
    S.col([(108.85, 6.4, 0, 109.55, 9.3, 0.2)])                       # cart frames (LiDAR sees posts only)


# ---- back of store ---------------------------------------------------------------------------------------------------------
def back(S):
    d = S.mesh("back_dairy")
    cx0, cx1, cy0, cy1, ch = L.COOLER_WALL
    S.col(P.fridge_wall(d, cx0, cx1, cy0, cy1, h=ch))
    for xs in (52.0, 65.0, 78.0):
        box(d, P.lit_sign("dept_dairy", 0.3), xs - 2.25, cy0 - 0.04, 2.55, xs + 2.25, cy0, 3.3, tile=(4.5, 0.75),
            origin=(xs - 2.25, cy0 - 0.04, 2.55), faces="s")
        box(d, MAT["dark"], xs - 2.25, cy0 - 0.04, 2.55, xs + 2.25, cy0, 3.3, faces="ewtb")
    for x, y in L.FREEZER_CHESTS:
        S.col(P.freezer_chest(d, x, y))
    P.sign2(d, 65.0, 86.0, 4.6, 7.5, 1.4, "dept_frozen")
    P.hang(d, 65.0, 86.0, 6.0, x_off=(-3.2, 3.2))

    f = S.mesh("back_fresh")
    for k, (x, y) in enumerate(L.SERVICE_CASES):
        S.col(P.service_case(f, x, y, "meat" if k < 2 else "bakery"))
    px0, px1, py = L.PREP_ROOM_WALL
    S.col(P.wall_run(f, "x", py, px0, px1, 3.6, openings=[]))
    S.col(P.wall_run(f, "y", px0, py, D, 3.6, openings=[]))
    for xd in (97.0, 103.6):                                                    # closed swing doors (visual)
        box(f, MAT["grey"], xd, py - 0.02, 0.0, xd + 1.8, py, 2.2, tile=(0.9, 2.2), faces="s")
        for xw in (xd + 0.3, xd + 1.2):
            box(f, MAT["glass"], xw, py - 0.025, 1.3, xw + 0.3, py - 0.02, 1.7, faces="s")
        box(f, MAT["dark"], xd + 0.89, py - 0.03, 0.0, xd + 0.91, py - 0.02, 2.2, faces="s")
    P.sign2(f, 95.5, 90.0, 3.3, 5.0, 0.95, "dept_meat")
    P.hang(f, 95.5, 90.0, 4.25, x_off=(-2.0, 2.0))
    P.sign2(f, 105.5, 90.0, 3.3, 5.0, 0.95, "dept_bakery")
    P.hang(f, 105.5, 90.0, 4.25, x_off=(-2.0, 2.0))


# ---- office -------------------------------------------------------------------------------------------------------------------
def office(S):
    o = S.mesh("office")
    ox0, ox1, oy0, oy1 = L.OFFICE
    quad_xy(o, P.MAT2["office_tile"], ox0, oy0 + 0.2, ox1, oy1, 0.003, tile=(1.2, 1.2))
    S.col(P.wall_run(o, "x", oy0, ox0, ox1 + 0.2, 3.0, openings=[]))
    d0, d1 = L.OFFICE_DOOR
    S.col(P.wall_run(o, "y", ox1, oy0 + 0.2, oy1, 3.0, openings=[(d0, d1, 2.4)]))
    for yd in (d0, d1 - 0.08):                                                  # door frame
        box(o, MAT["dark"], ox1 - 0.02, yd, 0, ox1 + 0.22, yd + 0.08, 2.4)
    box(o, MAT["dark"], ox1 - 0.02, d0, 2.32, ox1 + 0.22, d1, 2.4, faces="b")
    G.sign_board(o, ox1 + 0.24, (d0 + d1) / 2, 2.5, 1.4, 0.35, "sign_office", facing="e")
    for i in range(3):
        S.col(P.desk_pair(o, 4.0 + 5.5 * i, 95.5))
    S.col(P.break_table(o, 3.0, 85.2))
    S.col(P.lockers(o, 10.0, oy0 + 0.2, 20.0, oy0 + 0.7))
    box(o, MAT["laminate"], 0.0, 85.0, 0.1, 0.65, 88.4, 0.9, tile=(1.2, 0.6))                 # kitchenette
    box(o, P.MAT2["white_top"], 0.0, 85.0, 0.9, 0.68, 88.4, 0.94)
    box(o, MAT["black"], 0.0, 85.03, 0, 0.62, 88.37, 0.1)
    box(o, MAT["white"], 0.0, 88.5, 0, 0.72, 89.2, 1.8)                                       # fridge
    box(o, MAT["dark"], 0.72, 88.55, 1.0, 0.74, 88.6, 1.5)
    S.col([(0.0, 85.0, 0, 0.68, 88.4, 0.94), (0.0, 88.5, 0, 0.74, 89.2, 1.8)])
    box(o, MAT["white"], 6.0, oy1 - 0.03, 1.0, 10.0, oy1, 2.2, faces="s")                     # whiteboard
    box(o, MAT["grey"], 5.95, oy1 - 0.04, 0.98, 10.05, oy1, 1.0)
    S.col(P.charging_dock(o, *L.CHARGING_DOCK))


# ---- receiving -------------------------------------------------------------------------------------------------------------------
def receiving(S, rng):
    r = S.mesh("receiving")
    rx0, rx1, ry0, ry1 = L.RECEIVING
    wall_h = ROOF_Z - 0.8
    S.col(P.wall_run(r, "x", ry0, rx0, rx1, wall_h, openings=[(L.RECEIVING_DOOR[0], L.RECEIVING_DOOR[1], 4.2)]))
    S.col(P.wall_run(r, "y", rx0, ry0 + 0.2, ry1, wall_h, openings=[]))
    a, b_ = L.RECEIVING_DOOR
    box(r, MAT["dock_door"], a, ry0 + 0.2, 4.2, b_, ry0 + 0.9, 4.9, tile=(4.0, 0.7), origin=(a, ry0 + 0.2, 4.2),
        faces="nb")                                                              # rolled-up door drum
    for xf in (a, b_ - 0.15):                                                   # jamb guards
        box(r, MAT["yellow"], xf, ry0 - 0.1, 0, xf + 0.15, ry0 + 0.3, 1.2, tile=(0.3, 0.3))
        S.col([(xf, ry0 - 0.1, 0, xf + 0.15, ry0 + 0.3, 1.2)])
    box(r, tex_sign("dept_receiving"), 116.0, ry0 - 0.04, 5.0, 122.0, ry0, 6.1, tile=(6.0, 1.1),
        origin=(116.0, ry0 - 0.04, 5.0), faces="s")
    box(r, MAT["dark"], 116.0, ry0 - 0.04, 5.0, 122.0, ry0, 6.1, faces="ewtb")
    for s, a_, b2, _ in L.DOORS:
        if s == "N":
            S.col(P.dock_door_open(r, a_, b2, wall_y=D, thick=L.WALL))
    # loaded pallets waiting to go out, a stack of empties, forklift, jack, a baler
    for i in range(4):
        for yy in (88.5, 96.5):
            x0 = 112.0 + 4.2 * i
            G.pallet_box(r, x0, yy, 0.0)
            hgt = 1.1 + 0.8 * rng.random()
            m = G.wrap_mat(int(rng.integers(6)))
            box(r, m, x0 + 0.03, yy + 0.03, G.PALLET_H, x0 + 1.17, yy + 0.97, G.PALLET_H + hgt, tile=(0.6, 0.5),
                origin=(x0, yy, G.PALLET_H))
            S.col([(x0, yy, 0, x0 + 1.2, yy + 1.0, G.PALLET_H + hgt)])
    G.pallet_stack(r, 127.8, 95.5, 12)
    S.col([(127.8, 95.5, 0, 129.0, 96.5, 12 * G.PALLET_H)])
    S.col(P.forklift(r, 127.4, 92.5, yaw=math.pi))
    G.pallet_jack(r, 111.2, 91.5, yaw=math.pi / 2)
    S.col([(110.9, 91.2, 0, 111.5, 92.9, 0.55)])
    box(r, MAT["grey"], 126.5, 98.0, 0, 128.5, 99.6, 2.6, tile=(1.0, 1.0))                    # baler
    box(r, MAT["stripes"], 126.49, 97.99, 0.9, 128.51, 99.61, 1.05, tile=(0.6, 0.15), faces="nsew")
    S.col([(126.5, 98.0, 0, 128.5, 99.6, 2.6)])
    for y_line in (86.0,):                                                                    # painted walkway
        quad_xy(r, P.MAT2["line_yellow"], rx0 + 0.3, y_line - 0.05, rx1 - 0.3, y_line + 0.05, 0.002)
    quad_xy(r, P.MAT2["line_yellow"], 111.0, 97.25, 127.0, 97.35, 0.002)                   # stay-back line


def tex_sign(name):
    return G.tex_mat(name, rough=0.5)


# ---- structure and signs -----------------------------------------------------------------------------------------------------------
def columns(S):
    c = S.mesh("columns")
    rack_x = set(round(x + L.RACK_DEPTH / 2, 3) for x in L.WEST_ROWS[::2] + L.EAST_ROWS[::2])
    for x, y in L.COLUMNS:
        if round(x, 3) in rack_x:
            box(c, MAT["grey"], x - 0.125, y - 0.125, 0, x + 0.125, y + 0.125, ROOF_Z, tile=(0.25, 1.0))
        else:
            G.column(c, x, y, ROOF_Z, guard=True)
            S.col([(x - 0.3, y - 0.3, 0, x + 0.3, y + 0.3, ROOF_Z)])


AISLES = [4.6, 12.2, 18.2, 24.2, 30.2, 36.2, 93.8, 99.8, 105.8, 111.8, 117.8, 125.4]


def signs(S):
    s = S.mesh("signs", lidar=False)
    for n, x in enumerate(AISLES, start=1):
        for y in (L.ROW_SEGMENTS[0][0] - 0.8, L.ROW_SEGMENTS[1][1] + 0.8):
            P.sign2(s, x, y, 6.0, 1.0, 1.25, f"aisle_{n:02d}")
            P.hang(s, x, y, 7.25, x_off=(-0.4, 0.4))
    banners = [(23.0, 21.0, "dept_grocery"), (23.0, 77.0, "dept_snacks"), (107.0, 21.0, "dept_beverages"),
               (107.0, 77.0, "dept_paper"), (57.2, 21.0, "dept_electronics"), (75.2, 21.0, "dept_clothing"),
               (57.2, 77.0, "dept_books"), (75.2, 77.0, "dept_seasonal"), (4.6, 49.0, "dept_household"),
               (125.4, 49.0, "dept_pet")]
    for x, y, name in banners:
        w = 5.0 if x < 6 or x > 124 else 7.5
        P.sign2(s, x, y, 7.8, w, 1.4, name)
        P.hang(s, x, y, 9.2, x_off=(-w / 2 + 0.5, w / 2 - 0.5))


# ---- loose props: separate meshes, placed by the world generator (hazards, clutter) ---------------------------------------------
def props():
    """Each prop: builder, collision boxes (local), mass (None = static)."""
    out = {}
    for i in range(len(club_textures.PRODUCTS)):
        b = Builder()
        box(b, G.carton_mat(i), -0.2, -0.15, 0, 0.2, 0.15, 0.3, tile=(0.4, 0.3), origin=(-0.2, -0.15, 0))
        out[f"carton_{i:02d}"] = (b, [(-0.2, -0.15, 0, 0.2, 0.15, 0.3)], 6.0)
    b = Builder()
    box(b, MAT["water"], -0.26, -0.14, 0, 0.26, 0.14, 0.28, tile=(0.52, 0.28), origin=(-0.26, -0.14, 0))
    out["water_bundle"] = (b, [(-0.26, -0.14, 0, 0.26, 0.14, 0.28)], 12.0)
    b = Builder()
    G.cart(b, -0.5, 0.0)
    out["cart"] = (b, [(-0.55, -0.31, 0.0, 0.5, 0.31, 0.2), (-0.5, -0.31, 0.42, 0.5, 0.31, 0.92)], None)
    b = Builder()
    G.pallet_detailed(b, -0.6, -0.5, 0.0)
    out["pallet"] = (b, [(-0.6, -0.5, 0, 0.6, 0.5, G.PALLET_H)], None)
    b = Builder()
    G.wet_floor_sign(b, 0.0, 0.0)
    out["wet_floor_sign"] = (b, [(-0.16, -0.15, 0, 0.16, 0.15, 0.6)], 1.2)
    b = Builder()
    G.pallet_jack(b, 0.0, 0.0)
    out["pallet_jack"] = (b, [(0.0, -0.28, 0, 1.15, -0.12, 0.075), (0.0, 0.12, 0, 1.15, 0.28, 0.075),
                              (-0.25, -0.28, 0, 0.0, 0.28, 0.55), (-0.62, -0.18, 0.55, -0.1, 0.18, 1.25)], None)
    b = Builder()
    legs = P.rolling_ladder(b, 0.0, 0.0)
    out["ladder"] = (b, legs, None)
    b = Builder()                                                            # platform truck (flat cart)
    box(b, MAT["grey"], -0.6, -0.3, 0.16, 0.6, 0.3, 0.2, tile=(0.6, 0.3))
    box(b, MAT["rubber"], -0.6, -0.3, 0.2, 0.6, 0.3, 0.205, faces="t")
    for xx in (-0.5, 0.5):
        for yy in (-0.22, 0.22):
            box(b, MAT["rubber"], xx - 0.05, yy - 0.02, 0, xx + 0.05, yy + 0.02, 0.1)
            G.bar(b, MAT["grey"], (xx, yy, 0.1), (xx, yy, 0.16), 0.02)
    for yy in (-0.27, 0.27):
        G.bar(b, MAT["black"], (-0.6, yy, 0.2), (-0.62, yy, 0.95), 0.03)
    G.bar(b, MAT["black"], (-0.62, -0.28, 0.95), (-0.62, 0.28, 0.95), 0.035)
    out["platform_truck"] = (b, [(-0.62, -0.3, 0, 0.6, 0.3, 0.21)], None)
    b = Builder()
    cylinder(b, MAT["blue"], 0, 0, 0, 0.9, 0.28, segments=18)
    cylinder(b, MAT["dark"], 0, 0, 0.9, 0.93, 0.3, segments=18)
    out["bin"] = (b, [(-0.28, -0.28, 0, 0.28, 0.28, 0.93)], None)
    b = Builder()
    p = 13
    box(b, G.carton_mat(p), -0.2, -0.15, 0, 0.2, 0.15, 0.9, tile=(0.4, 0.3), origin=(-0.2, -0.15, 0))
    out["carton_stack"] = (b, [(-0.2, -0.15, 0, 0.2, 0.15, 0.9)], None)
    return out


HAZARDS = [  # (prop, x, y, yaw, note)
    ("pallet_jack", 24.2, 38.0, math.pi, "Pallet jack, forks down (75 mm): below the LiDAR plane"),
    ("pallet", 56.0, 49.0, 0.0, "Empty pallet (144 mm): below the LiDAR plane"),
    ("wet_floor_sign", 96.0, 16.0, 0.4, "Wet-floor sign"),
    ("cart", 99.8, 60.0, 1.3, "Abandoned cart: LiDAR sees only four thin posts"),
]


# ---- LiDAR-height slice -> occupancy map -----------------------------------------------------------------------------------------------------
def slice_segments(builders, z):
    segs = []
    for b in builders:
        for name, bt in b.batches.items():
            if not bt.P:
                continue
            Pp, _, _, F = bt.arrays()
            V = Pp[F]                                          # (m, 3, 3)
            d = V[..., 2] - z
            m = (d.min(1) < 0) & (d.max(1) > 0)
            if not m.any():
                continue
            V, d = V[m], d[m]
            pts = []
            for i, j in ((0, 1), (1, 2), (2, 0)):
                di, dj = d[:, i], d[:, j]
                cross = (di * dj) < 0
                t = np.where(cross, di / np.where(cross, di - dj, 1.0), 0.0)
                p = V[:, i, :2] + (V[:, j, :2] - V[:, i, :2]) * t[:, None]
                pts.append((cross, p))
            # each crossing triangle has exactly two crossing edges
            c = np.stack([pts[k][0] for k in range(3)], 1)
            pp = np.stack([pts[k][1] for k in range(3)], 1)
            ok = c.sum(1) == 2
            idx = np.argsort(~c[ok], axis=1, kind="stable")[:, :2]
            sel = np.take_along_axis(pp[ok], idx[..., None], axis=1)
            segs.append(sel)
    return np.concatenate(segs) if segs else np.zeros((0, 2, 2))


def make_map(S, out, res=0.05, origin=(-1.0, -1.0), size=(132.0, 102.0)):
    import cv2
    nx, ny = int(round(size[0] / res)), int(round(size[1] / res))
    occ = np.zeros((ny, nx), np.uint8)                       # row 0 = y origin (flipped when saved)
    builders = [S.meshes[n] for n, m in S.meta.items() if m["lidar"] and m["group"] == "static"]
    nseg = 0
    for z in (LIDAR_Z - 0.03, LIDAR_Z, LIDAR_Z + 0.03):
        segs = slice_segments(builders, z)
        nseg += len(segs)
        px = np.round((segs - np.array(origin)) / res).astype(np.int32)
        for (x0, y0), (x1, y1) in px:
            cv2.line(occ, (int(x0), int(y0)), (int(x1), int(y1)), 255, 1, lineType=8)
    free = np.zeros((ny + 2, nx + 2), np.uint8)
    work = occ.copy()
    seeds = [(x, y) for x, y, _ in L.SPAWNS.values()] + [(65.0, 17.0), (65.0, 81.0), (65.0, 49.0)]
    for sx, sy in seeds:
        cx, cy = int((sx - origin[0]) / res), int((sy - origin[1]) / res)
        if work[cy, cx] == 0:
            cv2.floodFill(work, free, (cx, cy), 128, flags=4)
    reach = work == 128
    grid = np.full((ny, nx), 0, np.uint8)                    # 0 = occupied
    grid[reach] = 254                                        # free
    yy, xx = np.mgrid[0:ny, 0:nx]
    wx, wy = origin[0] + (xx + 0.5) * res, origin[1] + (yy + 0.5) * res
    outside = (wx < -L.WALL) | (wx > W + L.WALL) | (wy < -L.WALL) | (wy > D + L.WALL)
    grid[outside] = 205                                      # unknown beyond the outer walls
    img = np.flipud(grid)
    maps = out / "maps"
    maps.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(maps / "warehouse_club.pgm"), img)
    (maps / "warehouse_club.yaml").write_text(
        "image: warehouse_club.pgm\nmode: trinary\nresolution: %.3f\norigin: [%.3f, %.3f, 0.0]\nnegate: 0\n"
        "occupied_thresh: 0.65\nfree_thresh: 0.196\n" % (res, origin[0], origin[1]))
    # keepout mask: dock edges (inside and out), the two doorways and outside the building
    mask = np.full((ny, nx), 254, np.uint8)
    keep = outside.copy()
    zones = []
    for s, a, b_, name in L.DOORS:
        if s == "N":
            zones.append((a - 0.8, D - 2.4, b_ + 0.8, D + 1.0, f"{name}: 1.22 m dock edge"))
        else:
            zones.append((a - 0.3, -1.0, b_ + 0.3, 0.6, f"{name} doorway"))
    for x0, y0, x1, y1, _ in zones:
        keep |= (wx >= x0) & (wx <= x1) & (wy >= y0) & (wy <= y1)
    mask[keep] = 0
    cv2.imwrite(str(maps / "warehouse_club_keepout.pgm"), np.flipud(mask))
    (maps / "warehouse_club_keepout.yaml").write_text(
        "image: warehouse_club_keepout.pgm\nmode: trinary\nresolution: %.3f\norigin: [%.3f, %.3f, 0.0]\n"
        "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n" % (res, origin[0], origin[1]))
    return grid, zones, nseg


def clutter_spots(grid, zones, res=0.05, origin=(-1.0, -1.0), walk_edges=(), rng=None, cols=()):
    """Floor spots next to shelves and walls where a loose item can sit without closing the aisle:
    0.45-0.8 m from the nearest obstacle, a clear corridor of 1.5 m or more beside it, away from waypoints,
    spawns, keepout zones and the walking routes. Yaw lines the item up with the nearest face."""
    import cv2
    from shapely.geometry import LineString, Point
    free = (grid == 254).astype(np.uint8)
    dist = cv2.distanceTransform(free, cv2.DIST_L2, 5) * res
    gy, gx = np.gradient(cv2.GaussianBlur(dist, (0, 0), 3))
    big = cv2.dilate(dist, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (101, 101)))   # max clearance within 2.5 m
    keep_away = [Point(x, y).buffer(1.6) for x, y in L.PATROL] + [Point(x, y).buffer(2.5) for x, y, _ in L.SPAWNS.values()]
    keep_away += [Point(x, y).buffer(2.0) for _, x, y, _, _ in HAZARDS]
    from shapely.ops import unary_union
    from shapely.prepared import prep
    from shapely.geometry import box as sbox
    solid = [sbox(c[0], c[1], c[3], c[4]).buffer(0.45) for c in cols if c[2] < 1.0 and c[5] > 0.01
             and (c[3] - c[0]) < 60 and (c[4] - c[1]) < 60]      # not the floor slabs
    avoid = prep(unary_union(keep_away + solid + [LineString(e).buffer(0.9) for e in walk_edges]))
    spots = []
    step = int(0.5 / res)
    ny, nx = grid.shape
    for r in range(0, ny, step):
        for c in range(0, nx, step):
            dv = dist[r, c]
            if not (0.45 <= dv <= 0.8) or big[r, c] < 1.5:
                continue
            x, y = origin[0] + (c + 0.5) * res, origin[1] + (r + 0.5) * res
            if any(x0 <= x <= x1 and y0 <= y <= y1 for x0, y0, x1, y1, _ in zones):
                continue
            if avoid.contains(Point(x, y)):
                continue
            yaw = math.atan2(gy[r, c], gx[r, c]) + math.pi / 2       # long side along the face
            zone = ("office" if x < 22.2 and y > 84 else "receiving" if x > 110 and y > 84 else
                    "front" if y < 14 else "back" if y > 84 else "store")
            spots.append((round(x, 2), round(y, 2), round(yaw, 3), round(float(dv), 2), zone))
    return spots


# ---- shopper walk graph -------------------------------------------------------------------------------------------------------------------
def walk_graph(obstacles_cols):
    """Aisle centre lines as a graph for extra shoppers. Edges that pass too close to hazards are dropped."""
    from shapely.geometry import LineString, Point, box as sbox
    from shapely.ops import unary_union
    xs_full = [4.6, 12.2, 18.2, 24.2, 30.2, 36.2, 44.3, 85.7, 93.8, 99.8, 105.8, 111.8, 117.8, 125.4]
    xs_tables = [50.6, 54.2, 61.4, 65.0, 68.6, 75.8, 79.4]
    ys = [17.0, 49.0, 81.0]
    nodes, edges = {}, []

    def nid(x, y):
        key = (round(x, 2), round(y, 2))
        if key not in nodes:
            nodes[key] = len(nodes)
        return nodes[key]

    for x in xs_full + xs_tables:
        pts = [nid(x, y) for y in ys]
        edges += list(zip(pts, pts[1:]))
    allx = sorted(xs_full + xs_tables)
    for y in ys:
        pts = [nid(x, y) for x in allx]
        edges += list(zip(pts, pts[1:]))
    # back area in front of the freezers, and the front of store behind the checkouts
    back = [nid(x, 86.5) for x in (44.3, 50.0, 65.0, 86.0)]
    edges += list(zip(back, back[1:])) + [(nid(44.3, 81.0), back[0]), (nid(85.7, 81.0), nid(86.0, 86.5))]
    front = [nid(x, 12.5) for x in (19.5, 29.5)]
    edges += [(front[0], front[1]), (nid(18.2, 17.0), front[0])]
    xy = {v: k for k, v in nodes.items()}
    solid = unary_union([sbox(c[0], c[1], c[3], c[4]) for c in obstacles_cols if c[2] < 1.5 and c[5] > 0.3])
    hazards = [Point(x, y).buffer(1.0) for _, x, y, _, _ in HAZARDS]
    good = []
    for a, b in edges:
        seg = LineString([xy[a], xy[b]])
        if seg.length < 0.01 or seg.buffer(0.35).intersects(solid) or any(seg.intersects(h) for h in hazards):
            continue
        good.append((a, b))
    return [list(xy[i]) for i in range(len(xy))], good


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("out", type=Path, nargs="?", default=Path(__file__).resolve().parents[2],
                    help="where models/, maps/, config/, worlds/ go (default: the roamview_sim package)")
    ap.add_argument("--stock", choices=list(STOCK), default="normal", help="how full the racks are")
    ap.add_argument("--seed", type=int, default=7, help="random seed for products and loads")
    a = ap.parse_args()
    out = a.out
    rng = np.random.default_rng(a.seed)
    model = out / "models" / "warehouse_club"
    mesh_dir = model / "meshes"
    if mesh_dir.exists():
        shutil.rmtree(mesh_dir)
    (mesh_dir / "props").mkdir(parents=True)
    tex = club_textures.make_all(model)                     # model/textures/*

    S = Scene()
    shell(S)
    outside(S)
    racks(S, rng, STOCK[a.stock])
    endcaps(S, rng)
    tables(S, rng)
    front(S, rng)
    back(S)
    office(S)
    receiving(S, rng)
    columns(S)
    signs(S)

    total_tris, total_bytes, meshes = 0, 0, []
    for name, b in S.meshes.items():
        n = b.triangles()
        size = b.write(mesh_dir / f"{name}.gltf", texture_dir_rel="../textures")
        total_tris += n
        total_bytes += size
        meshes.append(dict(name=name, uri=f"meshes/{name}.gltf", **S.meta[name], triangles=int(n)))
    prop_meta = {}
    for name, (b, cols, mass) in props().items():
        b.write(mesh_dir / "props" / f"{name}.gltf", texture_dir_rel="../../textures")
        prop_meta[name] = dict(uri=f"meshes/props/{name}.gltf", collisions=[list(c) for c in cols], mass=mass,
                               triangles=int(b.triangles()))

    grid, zones, nseg = make_map(S, out)
    nodes, edges = walk_graph(S.cols)
    routes = [(nodes[i], nodes[j]) for i, j in edges]
    routes += [(p, q) for r in L.SHOPPERS for p, q in zip(r, r[1:])]
    spots = clutter_spots(grid, zones, walk_edges=routes, rng=rng, cols=S.cols)

    manifest = dict(
        name="warehouse_club", size=[W, D, H], stock=a.stock, seed=a.seed, lidar_height=LIDAR_Z,
        meshes=meshes, collisions=[list(c) for c in S.cols], props=prop_meta,
        hazards=[dict(prop=p, x=x, y=y, yaw=yaw, note=n) for p, x, y, yaw, n in HAZARDS],
        spawns={k: list(v) for k, v in L.SPAWNS.items()}, patrol=[list(p) for p in L.PATROL],
        shoppers=[[list(p) for p in r] for r in L.SHOPPERS], walk_nodes=nodes, walk_edges=[list(e) for e in edges],
        clutter_spots=spots, keepout=[dict(box=[x0, y0, x1, y1], note=n) for x0, y0, x1, y1, n in zones],
        office_door=list(L.OFFICE_DOOR), charging_dock=list(L.CHARGING_DOCK))
    (model / "manifest.json").write_text(json.dumps(manifest, separators=(",", ":")))
    (model / "model.config").write_text("""<?xml version="1.0"?>
<model>
  <name>warehouse_club</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <author><name>RoamView team (ENGR 490)</name></author>
  <description>Generic warehouse club, 130 x 100 m, 11 m clear height: racking, tables, checkouts, food court,
  fridges, freezers, service cases, office with charging dock, receiving with two open dock doors (1.22 m drop).
  Day lighting; worlds made by club_world.py can change lighting, people and clutter.</description>
</model>
""")
    import sys
    for cand in (Path(__file__).resolve().parent, Path(__file__).resolve().parents[2] / "scripts"):
        if (cand / "club_world.py").exists():
            sys.path.insert(0, str(cand))
    import club_world
    (model / "model.sdf").write_text(club_world.store_model_sdf(manifest, lighting="day", uri_prefix="model://warehouse_club/"))
    worlds = out / "worlds"
    worlds.mkdir(parents=True, exist_ok=True)
    for fname, kw in (("warehouse_club.sdf", dict()), ("warehouse_club_people.sdf", dict(people=8, clutter="low"))):
        text, _ = club_world.world_sdf(manifest, **kw)
        (worlds / fname).write_text(text)
    cfg = out / "config"
    cfg.mkdir(parents=True, exist_ok=True)
    lines = ["# Patrol for scripts/patrol.py: charging dock -> west aisles -> front racetrack -> centre -> receiving",
             "# -> east aisles -> back to the dock. x, y in the map frame (m); yaw faces the next waypoint.",
             "frame_id: map", "waypoints:"]
    pts = L.PATROL
    for i, (x, y) in enumerate(pts):
        nx_, ny_ = pts[i + 1] if i + 1 < len(pts) else pts[i - 1]
        yaw = math.atan2(ny_ - y, nx_ - x) if i + 1 < len(pts) else 0.0
        lines.append(f"  - {{x: {x:.2f}, y: {y:.2f}, yaw: {yaw:.3f}}}")
    lines.append("# spawn presets (sim.launch.py spawn:=..., nav.launch.py spawn:=..., patrol.py start:=...)")
    lines.append("spawns:")
    for k, (x, y, yaw) in L.SPAWNS.items():
        lines.append(f"  {k}: [{x}, {y}, {yaw}]")
    (cfg / "warehouse_club_patrol.yaml").write_text("\n".join(lines) + "\n")
    print(f"meshes: {len(meshes)} files, {total_tris:,} triangles, {total_bytes / 1e6:.1f} MB; props: {len(prop_meta)}")
    print(f"collision boxes: {len(S.cols)}; map slice segments: {nseg:,}; walk graph: {len(nodes)} nodes, "
          f"{len(edges)} edges; clutter spots: {len(spots)}")


if __name__ == "__main__":
    main()
