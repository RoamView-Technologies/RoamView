"""Store fixtures for the full warehouse club: tables, checkouts, carts, food court, fridges, freezers, service
cases, office, receiving, dock doors, forklift, outside. Real sizes in metres; every builder adds to a Builder.

Each builder that the robot can touch returns the collision boxes (x0, y0, z0, x1, y1, z1) that go with it.
"""
import math

import numpy as np

import club_geom as G
from club_geom import MAT, bar, carton_mat, merge, tex_mat, transform, wrap_mat
from gltf_writer import Builder, Material, box, cylinder, quad, quad_xy

MAT2 = dict(
    office_tile=tex_mat("office_tile", 0.45),
    books=tex_mat("books", 0.8),
    menu=tex_mat("menu_board", 0.4, emissive=(0.55, 0.55, 0.55), glow=True),
    case_meat=tex_mat("case_meat", 0.5, emissive=(0.25, 0.25, 0.25)),
    case_bakery=tex_mat("case_bakery", 0.5, emissive=(0.25, 0.25, 0.25)),
    trailer=tex_mat("trailer_side", 0.6),
    red_top=Material("red_laminate", color=(0.70, 0.10, 0.09, 1), roughness=0.45),
    white_top=Material("white_laminate", color=(0.92, 0.92, 0.9, 1), roughness=0.4),
    sidewalk=Material("sidewalk", texture="concrete_floor.jpg", color=(1.06, 1.06, 1.06, 1), roughness=0.9),
    line_white=Material("paint_white", color=(0.92, 0.92, 0.9, 1), roughness=0.7),
    line_yellow=Material("paint_yellow", color=(0.95, 0.75, 0.08, 1), roughness=0.7),
    foliage=Material("foliage", color=(0.22, 0.36, 0.18, 1), roughness=0.9),
    tag_board=tex_mat("apriltag_0", 0.6, ext="png"),
)


def lit_sign(name, strength=0.45):
    return tex_mat(name, rough=0.5, emissive=(strength, strength, strength), glow=True)


def sign2(b, x, y, z, w, h, name, thick=0.05, lit=False):
    """Double-sided sign across y (printed on its south and north faces), bottom edge at z."""
    m = lit_sign(name) if lit else tex_mat(name, rough=0.5)
    x0, x1, y0, y1 = x - w / 2, x + w / 2, y - thick / 2, y + thick / 2
    box(b, m, x0, y0, z, x1, y1, z + h, tile=(w, h), origin=(x0, y0, z), faces="s")
    box(b, m, x0, y0, z, x1, y1, z + h, tile=(w, h), origin=(x1, y0, z), faces="n")
    box(b, MAT["dark"], x0, y0, z, x1, y1, z + h, faces="ewtb")


def hang(b, x, y, z_top, x_off=(-0.4, 0.4), ceiling=10.2):
    """Two thin hanger rods from the joists down to a sign top at z_top."""
    for dx in x_off:
        bar(b, MAT["dark"], (x + dx, y, z_top), (x + dx, y, ceiling), 0.012)


# ---- centre of store: display tables -------------------------------------------------------------------------
def display_table(b, x, y, kind, rng, w=1.2, l=2.4, h=0.8):
    """Skirted display table (solid to the floor, so LiDAR and bumper agree) with products on top."""
    box(b, MAT["black"], x + 0.03, y + 0.03, 0.0, x + w - 0.03, y + l - 0.03, h - 0.04, faces="nsew")
    box(b, MAT["laminate"], x, y, h - 0.04, x + w, y + l, h, tile=(1.2, 0.6))
    z = h
    if kind == "clothing":
        for i in range(2):
            for j in range(4):
                sh = 0.12 + 0.22 * rng.random()
                x0, y0 = x + 0.08 + i * 0.54, y + 0.08 + j * 0.57
                box(b, MAT["clothing"], x0, y0, z, x0 + 0.48, y0 + 0.5, z + sh, tile=(0.5, 0.5),
                    origin=(x0, y0, z), faces="nsewt")
    elif kind == "electronics":
        for j in range(4):
            e = int(rng.integers(4))
            m = tex_mat(f"electronics_{e}", rough=0.5)
            if e == 0:                                  # big display boxes standing up, facing the aisles
                x0, y0 = x + 0.25, y + 0.1 + j * 0.58
                box(b, m, x0, y0, z, x0 + 0.7, y0 + 0.14, z + 0.5, tile=(0.14, 0.5), faces="ew")
                box(b, m, x0, y0, z, x0 + 0.7, y0 + 0.14, z + 0.5, tile=(0.7, 0.5), origin=(x0, y0, z), faces="ns")
                box(b, MAT["dark"], x0, y0, z, x0 + 0.7, y0 + 0.14, z + 0.5, faces="t")
            else:                                       # smaller boxes in stacks
                for i in range(2):
                    n = int(rng.integers(2, 5))
                    x0, y0 = x + 0.1 + i * 0.52, y + 0.1 + j * 0.58
                    box(b, m, x0, y0, z, x0 + 0.46, y0 + 0.38, z + 0.11 * n, tile=(0.46, 0.33),
                        origin=(x0, y0, z), faces="nsewt")
    elif kind == "books":
        for i in range(3):
            for j in range(6):
                sh = 0.1 + 0.25 * rng.random()
                x0, y0 = x + 0.08 + i * 0.36, y + 0.1 + j * 0.37
                box(b, MAT2["books"], x0, y0, z, x0 + 0.3, y0 + 0.3, z + sh, tile=(0.5, 0.3), origin=(x0, y0, z),
                    faces="nsewt")
    else:                                               # seasonal cartons, a half pallet's worth
        p = int(rng.integers(18))
        box(b, carton_mat(p), x + 0.1, y + 0.2, z, x + w - 0.1, y + l - 0.2, z + 0.6 + 0.3 * rng.integers(2),
            tile=(0.4, 0.3), origin=(x + 0.1, y + 0.2, z))
    return [(x, y, 0, x + w, y + l, h)]


# ---- front of store ----------------------------------------------------------------------------------------------
def checkout(b, x, y, lane):
    """Checkout lane counter from (x, y), 0.9 x 5.0 m along +y: bagging, scanner, belt; a card-terminal stand and
    the lit lane number on a pole."""
    st, dk = MAT["stainless"], MAT["dark"]
    box(b, MAT["black"], x + 0.03, y + 0.03, 0, x + 0.87, y + 4.97, 0.1, faces="nsew")
    box(b, st, x, y, 0.1, x + 0.9, y + 5.0, 0.86, tile=(1.0, 0.5), faces="nsew")
    box(b, st, x, y, 0.86, x + 0.9, y + 1.4, 0.88, tile=(1.0, 0.5), faces="t")        # bagging
    box(b, MAT["black"], x + 0.1, y + 1.4, 0.86, x + 0.8, y + 2.1, 0.885, faces="t")    # scanner glass
    box(b, MAT["black"], x + 0.3, y + 1.68, 0.88, x + 0.6, y + 1.82, 1.12)              # scanner tower
    box(b, MAT["belt"], x + 0.06, y + 2.1, 0.86, x + 0.84, y + 4.95, 0.88, tile=(0.6, 1.2), faces="t")
    for xr in (x, x + 0.84):
        box(b, st, xr, y + 2.1, 0.86, xr + 0.06, y + 4.95, 0.91, tile=(1.0, 0.5))
    # cashier screen
    bar(b, dk, (x + 0.75, y + 2.25, 0.9), (x + 0.75, y + 2.25, 1.2), 0.03)
    box(b, MAT["screen"], x + 0.73, y + 2.05, 1.2, x + 0.77, y + 2.45, 1.48)
    # customer card terminal stand
    sx, sy = x - 0.7, y + 3.3
    box(b, MAT["dark"], sx, sy, 0, sx + 0.4, sy + 0.4, 1.0, tile=(0.4, 1.0))
    box(b, MAT["black"], sx + 0.1, sy + 0.08, 1.0, sx + 0.3, sy + 0.32, 1.12)
    # lane light
    bar(b, dk, (x + 0.45, y + 4.9, 0.9), (x + 0.45, y + 4.9, 2.2), 0.045)
    m = lit_sign(f"lane_{lane:02d}", 0.6)
    lx0, ly0 = x + 0.45 - 0.18, y + 4.9 - 0.06
    box(b, m, lx0, ly0, 2.2, lx0 + 0.36, ly0 + 0.12, 2.56, tile=(0.36, 0.36), origin=(lx0, ly0, 2.2), faces="s")
    box(b, m, lx0, ly0, 2.2, lx0 + 0.36, ly0 + 0.12, 2.56, tile=(0.36, 0.36), origin=(lx0 + 0.36, ly0, 2.2), faces="n")
    box(b, dk, lx0, ly0, 2.2, lx0 + 0.36, ly0 + 0.12, 2.56, faces="ewtb")
    return [(x, y, 0, x + 0.9, y + 5.0, 0.9), (sx, sy, 0, sx + 0.4, sy + 0.4, 1.0)]


def cart_corral(b, x0, y0, x1, y1, ncarts=7):
    """Rails around a row of nested carts; open at the north end."""
    g = MAT["grey"]
    for xr in (x0, x1):
        for z in (0.28, 0.95):                          # the low rail sits in the LiDAR plane
            bar(b, g, (xr, y0, z), (xr, y1, z), 0.05)
        for yp in (y0, (y0 + y1) / 2, y1):
            bar(b, g, (xr, yp, 0), (xr, yp, 0.97), 0.06)
    for z in (0.28, 0.95):
        bar(b, g, (x0, y0, z), (x1, y0, z), 0.05)
    xc = (x0 + x1) / 2
    for k in range(ncarts):
        G.cart(b, xc, y0 + 0.35 + k * 0.33, yaw=math.pi / 2)
    return [(x0 - 0.04, y0 - 0.04, 0, x1 + 0.04, y1 + 0.04, 1.0)]


def food_table(b, x, y):
    """Picnic table with attached benches on two steel legs (the robot fits under the benches, as in real life)."""
    box(b, MAT2["red_top"], x, y, 0.72, x + 1.2, y + 0.8, 0.76)
    for yb in (y - 0.42, y + 0.92):
        box(b, MAT2["red_top"], x, yb, 0.43, x + 1.2, yb + 0.3, 0.46)
    cols = []
    for xx in (x + 0.2, x + 1.0):
        bar(b, MAT["grey"], (xx, y + 0.4, 0.0), (xx, y + 0.4, 0.72), 0.07)
        bar(b, MAT["grey"], (xx, y - 0.3, 0.42), (xx, y + 1.1, 0.42), 0.05)
        bar(b, MAT["grey"], (xx, y + 0.4, 0.42), (xx, y + 0.4, 0.42), 0.05)
        cylinder(b, MAT["grey"], xx, y + 0.4, 0.0, 0.015, 0.2, segments=12)
        cols.append((xx - 0.04, y + 0.36, 0, xx + 0.04, y + 0.44, 0.72))
    cols += [(x, y, 0.72, x + 1.2, y + 0.8, 0.76), (x, y - 0.42, 0.42, x + 1.2, y + 1.22, 0.46)]
    return cols


def food_counter(b, x0, x1, y0, y1):
    """Service counter along the west wall with two lit menu boards above it."""
    box(b, MAT["stainless"], x0, y0, 0.1, x1, y1, 1.0, tile=(1.0, 0.5), faces="nsew")
    box(b, MAT["black"], x0 + 0.03, y0, 0, x1, y1, 0.1, faces="nsew")
    box(b, MAT2["white_top"], x0 - 0.02, y0 - 0.02, 1.0, x1 + 0.05, y1 + 0.02, 1.05)
    for yc in (y0 + 1.2, y0 + 5.0):
        box(b, MAT["dark"], x0 + 0.1, yc, 1.05, x0 + 0.7, yc + 0.8, 1.75)               # drink machines
        box(b, MAT["red"], x0 + 0.69, yc + 0.05, 1.2, x0 + 0.71, yc + 0.75, 1.7)
    m = MAT2["menu"]
    for yc in (y0 + 0.9, y0 + 4.4):
        box(b, m, 0.0, yc, 2.3, 0.06, yc + 3.3, 3.4, tile=(3.3, 1.1), origin=(0.0, yc, 2.3), faces="e")
        box(b, MAT["dark"], 0.0, yc, 2.3, 0.06, yc + 3.3, 3.4, faces="nstb")
    return [(x0, y0, 0, x1 + 0.05, y1, 1.05)]


def membership_desk(b, x0, x1, y0, y1):
    box(b, MAT["laminate"], x0, y0, 0.1, x1, y1, 1.05, tile=(1.2, 0.6), faces="nsew")
    box(b, MAT["black"], x0 + 0.03, y0 + 0.03, 0, x1 - 0.03, y1 - 0.03, 0.1, faces="nsew")
    box(b, MAT2["white_top"], x0 - 0.03, y0 - 0.05, 1.05, x1 + 0.03, y1 + 0.03, 1.09)
    for k in range(4):
        xm = x0 + 1.2 + k * 2.3
        bar(b, MAT["dark"], (xm, y1 - 0.4, 1.09), (xm, y1 - 0.4, 1.25), 0.03)
        box(b, MAT["screen"], xm - 0.22, y1 - 0.43, 1.25, xm + 0.22, y1 - 0.40, 1.52)
    sign2(b, (x0 + x1) / 2, (y0 + y1) / 2, 3.1, 6.0, 1.1, "dept_service")
    hang(b, (x0 + x1) / 2, (y0 + y1) / 2, 4.2, x_off=(-2.5, 2.5))
    return [(x0, y0, 0, x1, y1, 1.09)]


def podium(b, x, y):
    box(b, MAT["dark"], x - 0.3, y - 0.3, 0, x + 0.3, y + 0.3, 1.05, tile=(0.6, 1.0))
    box(b, MAT2["white_top"], x - 0.33, y - 0.33, 1.05, x + 0.33, y + 0.33, 1.08)
    return [(x - 0.3, y - 0.3, 0, x + 0.3, y + 0.3, 1.08)]


def bollard(b, x, y, h=1.0, r=0.08):
    cylinder(b, MAT["yellow"], x, y, 0, h, r, segments=14)
    return [(x - r, y - r, 0, x + r, y + r, h)]


# ---- back of store ------------------------------------------------------------------------------------------------
def fridge_wall(b, x0, x1, y0, y1, h=2.3, door=0.76):
    """Glass-door reach-in fridges, front at y0 facing south; lit shelves behind the glass."""
    n = int(round((x1 - x0) / door))
    door = (x1 - x0) / n
    box(b, MAT["white"], x0, y0 + 0.1, 0, x1, y1, h, faces="ewt")
    quad(b, MAT["fridge_back"], (x0, y0 + 0.55, 0.15), (x1, y0 + 0.55, 0.15), (x1, y0 + 0.55, 2.15),
         (x0, y0 + 0.55, 2.15), uv=((0, 1), (n, 1), (n, 0), (0, 0)))
    for xs in (x0, x1):                                     # side walls of the lit interior
        box(b, MAT["white"], xs - 0.01, y0, 0.15, xs + 0.01, y0 + 0.55, 2.15)
    quad(b, MAT["glass"], (x0, y0 + 0.02, 0.15), (x1, y0 + 0.02, 0.15), (x1, y0 + 0.02, 2.13),
         (x0, y0 + 0.02, 2.13))
    box(b, MAT["black"], x0, y0, 0, x1, y0 + 0.1, 0.15, faces="stn")
    box(b, MAT["black"], x0, y0, 2.13, x1, y0 + 0.1, 2.3, faces="stb")
    box(b, MAT["light"], x0, y0 + 0.03, 2.1, x1, y0 + 0.06, 2.13, faces="b")
    for k in range(n + 1):
        xm = x0 + k * door
        box(b, MAT["black"], xm - 0.025, y0, 0.15, xm + 0.025, y0 + 0.06, 2.13, faces="sew")
        if k < n:
            hx = xm + door - 0.09
            bar(b, MAT["chrome"], (hx, y0 - 0.03, 0.85), (hx, y0 - 0.03, 1.75), 0.022)
    return [(x0, y0, 0, x1, y1, h)]


def freezer_chest(b, x, y, l=4.5, w=1.1, h=0.9):
    """Open-top chest freezer with sliding glass lids and a bumper rail."""
    box(b, MAT["white"], x, y, 0.1, x + l, y + w, h - 0.04, faces="nsew")
    box(b, MAT["black"], x + 0.03, y + 0.03, 0, x + l - 0.03, y + w - 0.03, 0.1, faces="nsew")
    t = 0.06
    for (a0, b0, a1, b1) in ((x, y, x + l, y + t), (x, y + w - t, x + l, y + w), (x, y, x + t, y + w),
                             (x + l - t, y, x + l, y + w), (x, y + w / 2 - 0.02, x + l, y + w / 2 + 0.02)):
        box(b, MAT["stainless"], a0, b0, h - 0.04, a1, b1, h, tile=(1.0, 0.5))
    box(b, MAT["freezer_in"], x + t, y + t, 0.1, x + l - t, y + w - t, 0.62, tile=(1.1, 0.28),
        origin=(x + t, y + t, 0.1), faces="t")
    for (y0_, y1_) in ((y + t, y + w / 2 - 0.02), (y + w / 2 + 0.02, y + w - t)):
        quad(b, MAT["glass"], (x + t, y0_, h - 0.015), (x + l - t, y0_, h - 0.015), (x + l - t, y1_, h - 0.015),
             (x + t, y1_, h - 0.015))
    box(b, MAT["grey"], x - 0.03, y - 0.03, 0.22, x + l + 0.03, y + w + 0.03, 0.32, tile=(1.0, 0.5), faces="nsewt")
    return [(x - 0.03, y - 0.03, 0, x + l + 0.03, y + w + 0.03, h)]


def service_case(b, x, y, kind, l=4.2, d=1.0):
    """Curved-glass service case (meat or bakery), customer side at y facing south."""
    m = MAT2["case_meat"] if kind == "meat" else MAT2["case_bakery"]
    box(b, MAT["stainless"], x, y, 0.12, x + l, y + d, 0.82, tile=(1.0, 0.5), faces="s")
    box(b, MAT["white"], x, y, 0.12, x + l, y + d, 0.82, faces="ewn")
    box(b, MAT["black"], x + 0.03, y + 0.03, 0, x + l - 0.03, y + d, 0.12, faces="sew")
    box(b, m, x + 0.04, y + 0.12, 0.8, x + l - 0.04, y + 0.9, 0.84, tile=(2.0, 0.78), origin=(x, y + 0.12, 0.8), faces="t")
    quad(b, m, (x + 0.04, y + 0.62, 0.84), (x + l - 0.04, y + 0.62, 0.84), (x + l - 0.04, y + 0.62, 1.22),
         (x + 0.04, y + 0.62, 1.22), uv=((0, 1), (l / 2.0, 1), (l / 2.0, 0), (0, 0)))
    quad(b, MAT["glass"], (x, y + 0.02, 0.84), (x + l, y + 0.02, 0.84), (x + l, y + 0.38, 1.36), (x, y + 0.38, 1.36))
    box(b, MAT["stainless"], x, y + 0.36, 1.34, x + l, y + d, 1.4, tile=(1.0, 0.5))
    box(b, MAT["light"], x, y + 0.4, 1.33, x + l, y + 0.6, 1.34, faces="b")
    for xs in (x, x + l - 0.04):
        box(b, MAT["stainless"], xs, y, 0.82, xs + 0.04, y + d, 1.34, tile=(1.0, 0.5))
    return [(x, y, 0, x + l, y + d, 1.4)]


# ---- walls with both faces finished ---------------------------------------------------------------------------------
def partition(b, x0, y0, x1, y1, height, wainscot=2.4):
    """Interior wall: painted block to `wainscot` and metal panels above, on both long faces."""
    along_x = (x1 - x0) >= (y1 - y0)
    faces = "ns" if along_x else "ew"
    ends = "ew" if along_x else "ns"
    if wainscot > 0:
        box(b, MAT["cmu"], x0, y0, 0, x1, y1, min(wainscot, height), tile=(2.4, 1.2), faces=faces)
    if height > wainscot:
        box(b, MAT["wall"], x0, y0, max(0, wainscot), x1, y1, height, tile=(1.0, 2.0), faces=faces)
    box(b, MAT["dark"], x0, y0, 0, x1, y1, height, faces=ends + "t")
    return [(x0, y0, 0, x1, y1, height)]


def wall_run(b, axis, pos, a0, a1, height, openings=(), facing=None, thick=0.2, head=None):
    """A partition along x (axis='x': y = pos .. pos + thick) or along y, with door openings [(start, end, top)].
    Returns collision boxes."""
    cols, cuts, p = [], sorted(openings), a0
    for s, e, top in cuts:
        if s > p:
            cols += _seg(b, axis, pos, p, s, 0, height, thick)
        cols += _seg(b, axis, pos, s, e, top, height, thick)          # lintel over the opening
        p = e
    if p < a1:
        cols += _seg(b, axis, pos, p, a1, 0, height, thick)
    return cols


def _seg(b, axis, pos, s, e, z0, z1, thick):
    if z1 - z0 < 0.01:
        return []
    if axis == "x":
        x0, y0, x1, y1 = s, pos, e, pos + thick
    else:
        x0, y0, x1, y1 = pos, s, pos + thick, e
    faces = "ns" if axis == "x" else "ew"
    ends = "ew" if axis == "x" else "ns"
    if z0 < 2.4:
        box(b, MAT["cmu"], x0, y0, z0, x1, y1, min(2.4, z1), tile=(2.4, 1.2), origin=(x0, y0, 0), faces=faces)
    if z1 > 2.4:
        box(b, MAT["wall"], x0, y0, max(2.4, z0), x1, y1, z1, tile=(1.0, 2.0), origin=(x0, y0, 0), faces=faces)
    box(b, MAT["dark"], x0, y0, z0, x1, y1, z1, faces=ends + "tb")
    return [(x0, y0, z0, x1, y1, z1)]


# ---- office --------------------------------------------------------------------------------------------------------
def office_chair(b, x, y, yaw=0.0):
    L = Builder()
    for k in range(5):
        a = k * 2 * math.pi / 5
        bar(L, MAT["black"], (0, 0, 0.06), (0.3 * math.cos(a), 0.3 * math.sin(a), 0.04), 0.03)
        box(L, MAT["rubber"], 0.3 * math.cos(a) - 0.025, 0.3 * math.sin(a) - 0.025, 0,
            0.3 * math.cos(a) + 0.025, 0.3 * math.sin(a) + 0.025, 0.05)
    bar(L, MAT["chrome"], (0, 0, 0.06), (0, 0, 0.45), 0.045)
    box(L, MAT["black"], -0.24, -0.24, 0.45, 0.24, 0.24, 0.52)
    box(L, MAT["black"], -0.3, -0.22, 0.55, -0.25, 0.22, 1.0)
    bar(L, MAT["black"], (-0.2, 0, 0.5), (-0.28, 0, 0.6), 0.04)
    merge(b, L, transform(yaw, (x, y, 0)))
    return [(x - 0.05, y - 0.05, 0, x + 0.05, y + 0.05, 0.5)]


def desk_pair(b, x, y, l=3.0, d=1.4):
    """Two rows of desks back to back (panel ends to the floor), a privacy screen, monitors and chairs."""
    cols = []
    box(b, MAT["laminate"], x, y, 0.72, x + l, y + d, 0.75, tile=(1.2, 0.6))
    for xe in (x, x + l - 0.04):
        box(b, MAT["dark"], xe, y, 0, xe + 0.04, y + d, 0.72)
        cols.append((xe, y, 0, xe + 0.04, y + d, 0.72))
    box(b, MAT["dark"], x + 0.04, y + d / 2 - 0.02, 0.2, x + l - 0.04, y + d / 2 + 0.02, 0.72)
    cols.append((x, y + d / 2 - 0.02, 0.2, x + l, y + d / 2 + 0.02, 0.72))
    box(b, MAT["grey"], x, y + d / 2 - 0.02, 0.75, x + l, y + d / 2 + 0.02, 1.15, tile=(1.0, 0.5))
    for side in (0, 1):
        ym = y + d / 2 + (0.18 if side else -0.18)
        for k in range(2):
            xm = x + 0.75 + k * 1.5
            bar(b, MAT["dark"], (xm, ym, 0.75), (xm, ym, 0.9), 0.03)
            y_front = ym + (0.02 if side else -0.02)
            box(b, MAT["screen"], xm - 0.28, min(ym, y_front), 0.9, xm + 0.28, max(ym, y_front), 1.22)
            yc = y - 0.35 if side == 0 else y + d + 0.35
            cols += office_chair(b, xm, yc, yaw=math.pi / 2 if side == 0 else -math.pi / 2)
    return cols


def break_table(b, x, y, l=4.0, d=1.6):
    box(b, MAT2["white_top"], x, y, 0.72, x + l, y + d, 0.75)
    cols = []
    for xx in (x + 0.1, x + l - 0.1):
        for yy in (y + 0.1, y + d - 0.1):
            bar(b, MAT["chrome"], (xx, yy, 0), (xx, yy, 0.72), 0.045)
            cols.append((xx - 0.03, yy - 0.03, 0, xx + 0.03, yy + 0.03, 0.72))
    for k in range(3):
        xc = x + 0.7 + k * 1.3
        for yc, yaw in ((y - 0.35, math.pi / 2), (y + d + 0.35, -math.pi / 2)):
            cols += office_chair(b, xc, yc, yaw)
    return cols


def lockers(b, x0, y0, x1, y1, h=1.9):
    box(b, MAT["grey"], x0, y0, 0, x1, y1, h, tile=(0.3, 1.9))
    n = int((x1 - x0) / 0.38)
    for k in range(1, n):
        xg = x0 + k * (x1 - x0) / n
        box(b, MAT["dark"], xg - 0.006, y1, 0.05, xg + 0.006, y1 + 0.005, h - 0.05, faces="n")
    box(b, MAT["dark"], x0, y1, 0.95, x1, y1 + 0.005, 0.96, faces="n")
    return [(x0, y0, 0, x1, y1, h)]


def charging_dock(b, x, y):
    """RoamView charging dock against the west wall at (x, y): base plate, tower with contacts, AprilTag
    (tag36h11 id 0, black square 200 mm, centre 250 mm up) printed on a board facing east."""
    box(b, MAT["dark"], 0.0, y - 0.3, 0, x + 0.25, y + 0.3, 0.015)
    box(b, MAT["black"], 0.0, y - 0.25, 0, 0.14, y + 0.25, 0.45)
    for dy in (-0.07, 0.07):
        box(b, MAT["chrome"], 0.14, y + dy - 0.02, 0.06, 0.16, y + dy + 0.02, 0.1)
    s = 0.25                                                 # board: 200 mm tag + white margin
    box(b, MAT2["tag_board"], 0.14, y - s / 2, 0.125, 0.145, y + s / 2, 0.125 + s, tile=(s, s),
        origin=(0.14, y - s / 2, 0.125), faces="e")
    box(b, MAT["light"], 0.02, y - 0.1, 0.45, 0.1, y + 0.1, 0.46, faces="t")
    return [(0.0, y - 0.25, 0, 0.16, y + 0.25, 0.45)]


# ---- receiving and docks ------------------------------------------------------------------------------------------------
def forklift(b, x, y, yaw=0.0):
    """Counterbalance forklift, forks down, local +x = forks."""
    L = Builder()
    yl = MAT["yellow"]
    box(L, yl, -1.9, -0.55, 0.25, -0.35, 0.55, 1.05, tile=(0.5, 0.5))              # body
    box(L, MAT["dark"], -2.25, -0.55, 0.3, -1.9, 0.55, 1.15)                       # counterweight
    box(L, MAT["black"], -1.5, -0.35, 1.05, -0.9, 0.35, 1.2)                       # seat base
    box(L, MAT["black"], -1.45, -0.25, 1.2, -1.1, 0.25, 1.3)
    box(L, MAT["black"], -1.5, -0.25, 1.3, -1.42, 0.25, 1.75)
    for yy in (-0.62, 0.62):
        cylinder(L, MAT["rubber"], -0.55, 0.36, yy - 0.12, yy + 0.12, 0.36, segments=18, axis="y")
        cylinder(L, MAT["rubber"], -1.75, 0.27, yy - 0.1, yy + 0.1, 0.27, segments=16, axis="y")
    for xx in (-1.75, -0.45):                                                       # overhead guard posts
        for yy in (-0.5, 0.5):
            bar(L, MAT["dark"], (xx, yy, 1.05 if xx < -1 else 1.0), (xx, yy, 2.15), 0.06)
    box(L, MAT["dark"], -1.85, -0.55, 2.15, -0.35, 0.55, 2.22)
    for yy in (-0.35, 0.35):                                                         # mast
        box(L, MAT["dark"], -0.3, yy - 0.06, 0.05, -0.18, yy + 0.06, 2.3)
    box(L, MAT["dark"], -0.3, -0.41, 2.2, -0.18, 0.41, 2.3)
    box(L, MAT["grey"], -0.18, -0.5, 0.05, -0.12, 0.5, 1.0)                          # carriage
    for yy in (-0.3, 0.3):
        box(L, MAT["dark"], -0.12, yy - 0.06, 0.02, 1.05, yy + 0.06, 0.07)          # forks
    bar(L, MAT["black"], (-1.0, 0, 1.25), (-0.8, 0, 1.45), 0.04)
    cylinder(L, MAT["black"], -0.8, 0, 1.45, 1.48, 0.16, segments=16)
    merge(b, L, transform(yaw, (x, y, 0)))
    c, s = math.cos(yaw), math.sin(yaw)
    pts = [(x + c * px - s * py, y + s * px + c * py) for px, py in ((-2.25, -0.75), (1.05, -0.75), (1.05, 0.75), (-2.25, 0.75))]
    xs, ys = [p[0] for p in pts], [p[1] for p in pts]
    return [(min(xs), min(ys), 0, max(xs), max(ys), 1.2)]


def dock_door_open(b, x0, x1, wall_y=100.0, thick=0.3, top=3.0):
    """Open sectional dock door: the door runs back under the roof on tracks; steel leveler in the floor, a striped
    dock edge, rubber bumpers and a warning sign. The floor ends at the outer wall face: 1.22 m drop outside."""
    w = x1 - x0
    box(b, MAT["dock_door"], x0, wall_y - 3.6, top + 0.2, x1, wall_y - 0.1, top + 0.25, tile=(w, 3.5),
        origin=(x0, wall_y - 3.6, top + 0.2), faces="b")
    box(b, MAT["white"], x0, wall_y - 3.6, top + 0.2, x1, wall_y - 0.1, top + 0.25, faces="tnsew")
    for xt in (x0 - 0.05, x1):
        box(b, MAT["grey"], xt, wall_y - 3.7, top + 0.1, xt + 0.05, wall_y, top + 0.3)
        box(b, MAT["grey"], xt, wall_y - 0.05, 0, xt + 0.05, wall_y, top + 0.3)
    box(b, MAT["grey"], x0 + 0.2, wall_y - 2.4, 0, x1 - 0.2, wall_y + thick, 0.003, tile=(1.0, 1.0), faces="t")
    box(b, MAT["stripes"], x0, wall_y + thick - 0.25, 0, x1, wall_y + thick, 0.004, tile=(0.6, 0.15), faces="t")
    for xb in (x0 + 0.1, x1 - 0.35):
        box(b, MAT["rubber"], xb, wall_y + thick, -0.62, xb + 0.25, wall_y + thick + 0.12, -0.1)
    xs = x1 + 0.5
    box(b, tex_mat("sign_dock", 0.5), xs, wall_y - 0.02, 1.6, xs + 1.0, wall_y, 1.85, tile=(1.0, 0.25),
        origin=(xs, wall_y - 0.02, 1.6), faces="s")
    cols = []
    for xg in (x0 - 0.375, x1 + 0.375):                                                  # guard posts
        cols += bollard(b, xg, wall_y - 0.4, h=1.2, r=0.1)
    return cols


def trailer(b, x0, y0, z_ground, length=16.0, width=2.6, yaw_x=True):
    """Parked semi-trailer along +x from (x0, y0) on ground level z_ground."""
    zf, zt = z_ground + 1.15, z_ground + 4.05
    x1, y1 = x0 + length, y0 + width
    m = MAT2["trailer"]
    box(b, m, x0, y0, zf, x1, y1, zt, tile=(8.0, 2.9), origin=(x0, y0, zf), faces="ns")
    box(b, MAT["white"], x0, y0, zf, x1, y1, zt, faces="ewt")
    box(b, MAT["dark"], x0 + 0.3, y0 + 0.2, z_ground + 0.9, x1 - 0.3, y1 - 0.2, zf)
    for xa in (x1 - 1.6, x1 - 2.9):
        for yy in (y0 + 0.35, y1 - 0.35):
            cylinder(b, MAT["rubber"], xa, z_ground + 0.5, yy - 0.28, yy + 0.28, 0.5, segments=18, axis="y")
    for yy in (y0 + 0.5, y1 - 0.5):
        bar(b, MAT["grey"], (x0 + 2.0, yy, z_ground), (x0 + 2.0, yy, zf), 0.1)
    return [(x0, y0, z_ground, x1, y1, zt)]


# ---- ceiling: joists per tile (for culling), deck and light fixtures as swappable variants ----------------------------
LIGHT_OFF = Material("led_panel_off", color=(0.74, 0.76, 0.78, 1), roughness=0.35)
DECK_NIGHT = Material("ceiling_deck_night", texture="ceiling_deck.jpg", roughness=0.7)


def joists(b, x0, y0, x1, y1, z, joist_every=1.8, main_every=12.0, x_max=130.0):
    """Open-web steel joists running north-south and red sprinkler mains inside one roof tile; global grids so
    neighbouring tiles line up."""
    zc, depth = z - 0.02, 0.75
    k0 = int(math.ceil(x0 / joist_every - 1e-9))
    for k in range(k0, int(x1 / joist_every) + 1):
        x = k * joist_every
        if not (x0 <= x < x1) or x <= 0.0 or x >= x_max:
            continue
        box(b, MAT["white"], x - 0.04, y0, zc - 0.06, x + 0.04, y1, zc, tile=(1, 1))
        box(b, MAT["white"], x - 0.04, y0, zc - depth, x + 0.04, y1, zc - depth + 0.06, tile=(1, 1))
        for j in range(int(math.floor(y0 / 1.2)), int(math.ceil(y1 / 1.2))):
            ya, yb = max(y0, j * 1.2), min(y1, (j + 1) * 1.2)
            if yb - ya < 0.05:
                continue
            za, zb = (zc - depth + 0.06, zc - 0.06) if j % 2 == 0 else (zc - 0.06, zc - depth + 0.06)
            bar(b, MAT["white"], (x, ya, za), (x, yb, zb), 0.025)
    for m in range(0, 20):
        yy = 3 + m * main_every
        if y0 <= yy < y1:
            bar(b, MAT["pipe_red"], (x0, yy, zc - depth - 0.1), (x1, yy, zc - depth - 0.1), 0.1)


def fixture_spots(W=130.0, D=100.0, light_dx=6.0, light_dy=7.5):
    """LED high-bay positions: group 'a' on even rows, 'b' on odd rows (dim lighting turns 'b' off)."""
    spots = []
    j = 0
    while light_dy / 2 + j * light_dy < D:
        i = 0
        while light_dx / 2 + i * light_dx < W:
            spots.append((light_dx / 2 + i * light_dx, light_dy / 2 + j * light_dy, "a" if j % 2 == 0 else "b"))
            i += 1
        j += 1
    return spots


def fixtures(b, spots, z, lit=True):
    zc, depth = z - 0.02, 0.75
    lamp = MAT["light"] if lit else LIGHT_OFF
    for lx, ly, _ in spots:
        cylinder(b, lamp, lx, ly, zc - depth - 0.55, zc - depth - 0.5, 0.26, segments=20)
        cylinder(b, MAT["grey"], lx, ly, zc - depth - 0.5, zc - depth - 0.3, 0.22, segments=16)
        bar(b, MAT["dark"], (lx, ly, zc - depth - 0.3), (lx, ly, zc - depth), 0.01)


def deck(b, x0, y0, x1, y1, z, night=False):
    quad_xy(b, DECK_NIGHT if night else MAT["ceiling"], x0, y0, x1, y1, z, tile=(1.2, 1.2), up=False)


def rolling_ladder(b, x, y, yaw=0.0, h=2.0):
    """Rolling warehouse safety ladder: steps, handrails, casters (thin legs: LiDAR sees little of it)."""
    L = Builder()
    g = MAT["grey"]
    n = 8
    for k in range(1, n + 1):
        z = k * h / n
        xs = 0.9 - k * 0.9 / n
        box(L, MAT["deck"], xs - 0.14, -0.3, z - 0.02, xs + 0.02, 0.3, z, tile=(0.3, 0.3))
    for yy in (-0.32, 0.32):
        bar(L, g, (0.95, yy, 0.05), (0.0, yy, h), 0.035)
        bar(L, g, (-0.3, yy, 0.05), (-0.1, yy, h), 0.035)
        bar(L, g, (0.0, yy, h), (0.0, yy, h + 0.9), 0.03)
        bar(L, g, (0.95, yy, 0.05), (-0.3, yy, 0.05), 0.035)
        bar(L, g, (0.4, yy, 1.0), (0.4, yy, 1.9), 0.025)
        bar(L, g, (0.4, yy, 1.9), (0.0, yy, h + 0.9), 0.025)
        for xx in (0.95, -0.3):
            box(L, MAT["rubber"], xx - 0.04, yy - 0.02, 0, xx + 0.04, yy + 0.02, 0.07)
    box(L, MAT["deck"], -0.3, -0.3, h - 0.02, 0.0, 0.3, h, tile=(0.3, 0.3))
    merge(b, L, transform(yaw, (x, y, 0)))
    c, s = math.cos(yaw), math.sin(yaw)
    cols = []
    for px, py in ((0.95, -0.32), (0.95, 0.32), (-0.3, -0.32), (-0.3, 0.32)):
        wx, wy = x + c * px - s * py, y + s * px + c * py
        cols.append((wx - 0.04, wy - 0.04, 0, wx + 0.04, wy + 0.04, 1.0))
    return cols
