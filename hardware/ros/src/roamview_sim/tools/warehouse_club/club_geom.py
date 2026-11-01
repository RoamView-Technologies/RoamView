"""Geometry builders for the warehouse club: racking, pallets, loads, carts, equipment, building shell.

All sizes in metres at real scale. Builders add to a gltf_writer.Builder; composite objects that need a
rotation are built in local coordinates and merged with a transform.
"""
import math

import numpy as np

from gltf_writer import Builder, Material, box, cylinder, quad_xy


# ---- materials ----------------------------------------------------------------------------------------
def M(name, tex=None, color=(1, 1, 1, 1), rough=0.8, metal=0.0, **kw):
    return Material(name, color=color, texture=tex and f"{tex}.jpg", roughness=rough, metallic=metal, **kw)


MAT = dict(
    floor=M("concrete", "concrete_floor", rough=0.32),
    wall=M("wall_panel", "wall_panel", rough=0.6),
    cmu=M("cmu", "cmu_block", rough=0.85),
    upright=M("upright", "upright_blue", rough=0.45, metal=0.2),
    beam=M("beam", "beam_orange", rough=0.4, metal=0.2),
    orange=M("steel_orange", "steel_orange", rough=0.4, metal=0.2),
    blue=M("steel_blue", "steel_blue", rough=0.45, metal=0.2),
    grey=M("steel_grey", "steel_grey", rough=0.5, metal=0.15),
    yellow=M("steel_yellow", "steel_yellow", rough=0.45, metal=0.1),
    deck=M("wire_deck", "wire_deck", rough=0.6, metal=0.25),
    wood=M("pallet_wood", "pallet_wood", rough=0.9),
    mat=M("rubber_mat", "rubber_mat", rough=0.95),
    asphalt=M("asphalt", "asphalt", rough=0.95),
    stainless=M("stainless", "stainless", rough=0.35, metal=0.2),
    belt=M("belt", "belt", rough=0.9),
    laminate=M("laminate", "wood_laminate", rough=0.55),
    fridge_back=M("fridge_interior", "fridge_interior", rough=0.7, emissive=(0.35, 0.36, 0.38)),
    freezer_in=M("freezer_contents", "freezer_contents", rough=0.7),
    ceiling=M("ceiling_deck", "ceiling_deck", rough=0.7, emissive=(0.18, 0.18, 0.19)),
    dock_door=M("dock_door", "dock_door", rough=0.6),
    stripes=M("hazard_stripes", "hazard_stripes", rough=0.6),
    clothing=M("clothing", "clothing", rough=0.95),
    water=M("water_bundle", "water_bundle", rough=0.25),
    towel=M("towel_bundle", "towel_bundle", rough=0.5),
    black=M("black_plastic", color=(0.05, 0.05, 0.06, 1), rough=0.6),
    white=M("white_paint", color=(0.93, 0.93, 0.92, 1), rough=0.6),
    dark=M("dark_grey", color=(0.22, 0.23, 0.25, 1), rough=0.6),
    red=M("red_plastic", color=(0.72, 0.08, 0.07, 1), rough=0.5),
    chrome=M("chrome", color=(0.78, 0.8, 0.83, 1), rough=0.25, metal=0.3),
    rubber=M("rubber_black", color=(0.04, 0.04, 0.04, 1), rough=0.95),
    glass=Material("glass", color=(0.75, 0.85, 0.9, 0.22), roughness=0.05, metallic=0.0, alpha_blend=True,
                   double_sided=True),
    light=Material("led_panel", color=(1, 1, 1, 1), roughness=0.4, emissive=(1.0, 1.0, 0.96)),
    screen=Material("screen", color=(0.1, 0.25, 0.45, 1), roughness=0.2, emissive=(0.08, 0.2, 0.35)),
    pipe_red=M("sprinkler_red", color=(0.62, 0.06, 0.05, 1), rough=0.5, metal=0.3),
)


def carton_mat(i):
    return M(f"carton_{i:02d}", f"carton_{i:02d}", rough=0.88)


def wrap_mat(i):
    return M(f"wrap_{i:02d}", f"wrap_{i:02d}", rough=0.35)


def tex_mat(name, rough=0.7, emissive=None, ext="jpg", glow=False):
    """Material with texture <name>.<ext>. glow: lit from inside (emissive with the texture's own colours)."""
    return Material(name, texture=f"{name}.{ext}", roughness=rough, emissive=emissive, emissive_map=glow)


# ---- transforms -----------------------------------------------------------------------------------------
def transform(yaw=0.0, t=(0, 0, 0)):
    c, s = math.cos(yaw), math.sin(yaw)
    T = np.eye(4)
    T[:3, :3] = [[c, -s, 0], [s, c, 0], [0, 0, 1]]
    T[:3, 3] = t
    return T


def merge(dst, src, T):
    R, t = T[:3, :3], T[:3, 3]
    for name, b in src.batches.items():
        if not b.P:
            continue
        P, N, U, F = b.arrays()
        dst.batch(src.materials[name]).add(P @ R.T + t, N @ R.T, U, F)


def bar(builder, mat, p0, p1, w, h=None):
    """Square-ish bar from p0 to p1 (any direction), cross-section w x h."""
    h = h or w
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    d = p1 - p0
    L = float(np.linalg.norm(d))
    if L < 1e-6:
        return
    a = d / L
    up = np.array([0, 0, 1.0]) if abs(a[2]) < 0.9 else np.array([1.0, 0, 0])
    s = np.cross(a, up); s /= np.linalg.norm(s)
    u = np.cross(s, a)
    local = Builder()
    box(local, mat, 0, -w / 2, -h / 2, L, w / 2, h / 2, tile=(1.0, 1.0))
    T = np.eye(4)
    T[:3, 0], T[:3, 1], T[:3, 2], T[:3, 3] = a, s, u, p0
    merge(builder, local, T)


# ---- pallets and loads ------------------------------------------------------------------------------------
PALLET_H = 0.144


def pallet_simple(b, x0, y0, z0, dx=1.2, dy=1.0):
    """Pallet as seen from the sides: top and bottom deck, three stringers (fork openings on the dx faces)."""
    w = MAT["wood"]
    box(b, w, x0, y0, z0 + 0.122, x0 + dx, y0 + dy, z0 + 0.144, tile=(1.0, 0.25))
    box(b, w, x0, y0, z0, x0 + dx, y0 + dy, z0 + 0.022, tile=(1.0, 0.25))
    for k in range(3):
        yy = y0 + k * (dy - 0.1) / 2
        box(b, w, x0, yy, z0 + 0.022, x0 + dx, yy + 0.1, z0 + 0.122, tile=(1.0, 0.25))


def pallet_detailed(b, x0, y0, z0, dx=1.2, dy=1.0):
    """Empty pallet with separate deck boards (for pallets lying on the floor)."""
    w = MAT["wood"]
    for k in range(7):
        xx = x0 + k * (dx - 0.1) / 6
        box(b, w, xx, y0, z0 + 0.122, xx + 0.1, y0 + dy, z0 + 0.144, tile=(1.0, 0.25))
    for k in range(3):
        xx = x0 + k * (dx - 0.14) / 2
        box(b, w, xx, y0, z0, xx + 0.14, y0 + dy, z0 + 0.022, tile=(1.0, 0.25))
        yy = y0 + k * (dy - 0.1) / 2
        for j in range(3):
            xb = x0 + j * (dx - 0.14) / 2
            box(b, w, xb, yy, z0 + 0.022, xb + 0.14, yy + 0.1, z0 + 0.122, tile=(1.0, 0.25))


def pallet_box(b, x0, y0, z0, dx=1.2, dy=1.0, fork_faces="ew"):
    """A pallet drawn as one box (fork openings printed on the faces in `fork_faces`): 36 triangles instead of 180."""
    x1, y1, z1 = x0 + dx, y0 + dy, z0 + PALLET_H
    other = "ns" if fork_faces == "ew" else "ew"
    wf, ws = (dy, dx) if fork_faces == "ew" else (dx, dy)
    box(b, tex_mat("pallet_front", 0.9), x0, y0, z0, x1, y1, z1, tile=(wf, PALLET_H), origin=(x0, y0, z0), faces=fork_faces)
    box(b, tex_mat("pallet_side", 0.9), x0, y0, z0, x1, y1, z1, tile=(ws, PALLET_H), origin=(x0, y0, z0), faces=other)
    box(b, MAT["wood"], x0, y0, z0, x1, y1, z1, tile=(1.0, 0.25), faces="t")


def pallet_stack(b, x0, y0, n, dx=1.2, dy=1.0, fork_faces="ew"):
    """n empty pallets stacked: one box, the pallet print repeats every 144 mm."""
    x1, y1, z1 = x0 + dx, y0 + dy, n * PALLET_H
    other = "ns" if fork_faces == "ew" else "ew"
    wf, ws = (dy, dx) if fork_faces == "ew" else (dx, dy)
    box(b, tex_mat("pallet_front", 0.9), x0, y0, 0, x1, y1, z1, tile=(wf, PALLET_H), origin=(x0, y0, 0), faces=fork_faces)
    box(b, tex_mat("pallet_side", 0.9), x0, y0, 0, x1, y1, z1, tile=(ws, PALLET_H), origin=(x0, y0, 0), faces=other)
    box(b, MAT["wood"], x0, y0, 0, x1, y1, z1, tile=(1.0, 0.25), faces="t")


def load(b, mat, x0, y0, z0, x1, y1, z1, carton=(0.4, 0.3), rng=None, deplete=0.0):
    """A stack of cartons: one box whose texture repeats once per carton. deplete removes a front-top chunk
    (a partly picked pallet on the sales floor)."""
    if deplete > 0 and rng is not None and (z1 - z0) > 2 * carton[1]:
        cut_z = z1 - carton[1] * int(1 + rng.integers(0, max(1, int((z1 - z0) / carton[1]) - 1)))
        box(b, mat, x0, y0, z0, x1, y1, cut_z, tile=carton, origin=(x0, y0, z0))
        # remaining top layer only at the back half
        xm = (x0 + x1) / 2
        back_x0, back_x1 = (xm, x1) if deplete > 0 else (x0, x1)
        box(b, mat, back_x0, y0, cut_z, back_x1, y1, z1, tile=carton, origin=(x0, y0, z0))
    else:
        box(b, mat, x0, y0, z0, x1, y1, z1, tile=carton, origin=(x0, y0, z0))


# ---- racking --------------------------------------------------------------------------------------------
FRAME_D, FLUE, COL, BEAM_H, BEAM_W = 1.07, 0.26, 0.076, 0.11, 0.05
LEVELS = [1.75, 3.30, 4.80]            # beam tops
RACK_H = 5.5
PAL_W = 1.0                            # pallet width along the beam


def rack_frame_line(b, xs, y, height=RACK_H):
    """One frame line across a row: columns at the given x's (front/back pairs) plus bracing in each frame."""
    up = MAT["upright"]
    for x in xs:
        box(b, up, x, y - COL / 2, 0, x + COL, y + COL / 2, height, tile=(COL, 0.6), origin=(x, y - COL / 2, 0))
    grey = MAT["blue"]
    for xa, xb in zip(xs[0::2], xs[1::2]):       # each single frame: front column xa, back column xb
        xa_in, xb_in = xa + COL, xb
        zs = [0.15, 1.35, 2.55, 3.75, 4.95]
        for z in (0.15, height - 0.25):
            bar(b, grey, (xa_in, y, z), (xb_in, y, z), 0.035)
        for k in range(len(zs) - 1):
            p0 = (xa_in, y, zs[k]) if k % 2 == 0 else (xb_in, y, zs[k])
            p1 = (xb_in, y, zs[k + 1]) if k % 2 == 0 else (xa_in, y, zs[k + 1])
            bar(b, grey, p0, p1, 0.03)


def rack_run(b, x0, y0, bays, double=True, rng=None, products=None, face_west=True, levels=LEVELS,
             height=RACK_H, floor_pick=True, protect=True, mix=(0.12, 0.08), empty=(0.06, 0.12), picked=0.55,
             deep=0.3):
    """A run of selective pallet racking along +y from y0: `bays` bays of 2.7 m.
    double: back-to-back pair (2.4 m deep) whose two sides face -x and +x. Otherwise a single rack 1.07 m deep
    facing -x (face_west) or +x. Loads: floor level = partly picked sales pallets, upper levels = wrapped overstock.
    mix: share of floor pallets that are (water bundles, paper-towel bundles); empty: share of empty pallet positions
    (floor, upper levels); picked: share of floor pallets that are partly picked; deep: share of those picked right
    down to the pallet at the aisle side (what makes a rack face look different along the aisle to a LiDAR)."""
    rng = rng or np.random.default_rng(0)
    products = products or list(range(18))
    if double:
        racks = [(x0, "w"), (x0 + FRAME_D + FLUE, "e")]
    else:
        racks = [(x0, "w" if face_west else "e")]
    xs = []
    for rx, _ in racks:
        xs += [rx, rx + FRAME_D - COL]
    bay = 2.7
    for k in range(bays + 1):
        rack_frame_line(b, xs, y0 + k * bay, height)
    for rx, face in racks:
        fx_front = rx if face == "w" else rx + FRAME_D          # aisle-side face of this rack
        for k in range(bays):
            ya, yb = y0 + k * bay + COL / 2, y0 + (k + 1) * bay - COL / 2
            for z in levels:
                for bx in (rx, rx + FRAME_D - BEAM_W):
                    box(b, MAT["beam"], bx, ya, z - BEAM_H, bx + BEAM_W, yb, z, tile=(2.7, BEAM_H), origin=(bx, ya, z - BEAM_H))
                box(b, MAT["deck"], rx + BEAM_W, ya, z - 0.03, rx + FRAME_D - BEAM_W, yb, z, tile=(0.3, 0.3))
            # loads: two pallet positions per bay per level
            level_bottoms = [0.0] + list(levels)
            for li, zb in enumerate(level_bottoms):
                ztop_clear = (levels[li] - BEAM_H - 0.08) if li < len(levels) else height - 0.05
                for p in range(2):
                    if rng.random() < (empty[0] if li == 0 else empty[1]):
                        continue                                  # empty position
                    py0 = ya + 0.12 + p * (PAL_W + 0.26)
                    over = 0.065
                    px0 = rx - over if face == "w" else rx - (1.2 - FRAME_D - over)
                    if li == 0:
                        pallet_simple(b, px0, py0, zb)          # seen up close: real boards and openings
                    else:
                        pallet_box(b, px0, py0, zb)
                    lz0 = zb + PALLET_H
                    hmax = ztop_clear - lz0
                    if hmax < 0.3:
                        continue
                    hgt = hmax * (0.75 + 0.25 * rng.random()) if li < len(levels) else min(hmax, 0.6 + 0.5 * rng.random())
                    prod = products[int(rng.integers(len(products)))]
                    inset = 0.03
                    lx0, lx1 = px0 + inset, px0 + 1.2 - inset
                    ly0, ly1 = py0 + inset, py0 + PAL_W - inset
                    if li == 0 and floor_pick:
                        kind = rng.random()
                        if kind < mix[0]:
                            mat, cart = MAT["water"], (0.52, 0.28)
                        elif kind < mix[0] + mix[1]:
                            mat, cart = MAT["towel"], (0.48, 0.48)
                        else:
                            mat, cart = carton_mat(prod), (0.4, 0.3)
                        # sales floor: the aisle side is partly picked
                        dep = rng.random() < picked
                        if dep:
                            cut = lz0 + max(cart[1], round((hgt * (0.35 + 0.4 * rng.random())) / cart[1]) * cart[1])
                            front = (lx0, lx0 + 0.45) if face == "w" else (lx1 - 0.45, lx1)
                            back = (lx0 + 0.45, lx1) if face == "w" else (lx0, lx1 - 0.45)
                            if rng.random() >= deep:
                                box(b, mat, front[0], ly0, lz0, front[1], ly1, cut, tile=cart, origin=(lx0, ly0, lz0))
                            box(b, mat, back[0], ly0, lz0, back[1], ly1, lz0 + hgt, tile=cart, origin=(lx0, ly0, lz0))
                        else:
                            box(b, mat, lx0, ly0, lz0, lx1, ly1, lz0 + hgt, tile=cart, origin=(lx0, ly0, lz0))
                    else:
                        wm = wrap_mat(int(rng.integers(6))) if rng.random() < 0.7 else carton_mat(prod)
                        box(b, wm, lx0, ly0, lz0, lx1, ly1, lz0 + hgt, tile=(0.6, 0.5), origin=(lx0, ly0, lz0))
        if protect:
            # column guards on the aisle-side columns, end-of-row guards
            for k in range(bays + 1):
                y = y0 + k * bay
                gx0 = fx_front - 0.1 if face == "w" else fx_front - COL - 0.02
                box(b, MAT["yellow"], gx0, y - 0.07, 0, gx0 + 0.12 + COL, y + 0.07, 0.3, tile=(0.3, 0.3))
    if protect:
        x_min = racks[0][0] - 0.1
        x_max = racks[-1][0] + FRAME_D + 0.1
        for ye, sgn in ((y0, -1), (y0 + bays * bay, 1)):
            yo = ye + sgn * 0.18
            box(b, MAT["yellow"], x_min, min(yo, yo + sgn * 0.1), 0, x_max, max(yo, yo + sgn * 0.1), 0.45, tile=(0.5, 0.45))
            for xg in (x_min, x_max - 0.1):
                box(b, MAT["yellow"], xg, min(ye, yo), 0, xg + 0.1, max(ye, yo + sgn * 0.1), 0.45, tile=(0.5, 0.45))
    return xs


# ---- equipment ---------------------------------------------------------------------------------------------
def cart(b, x, y, yaw=0.0):
    """Warehouse-club cart: 1.05 m long wire basket on a frame, 4 casters, handle bar (local x forward)."""
    L = Builder()
    w, ln, bz0, bz1 = 0.62, 1.0, 0.42, 0.92
    g, c = MAT["grey"], MAT["chrome"]
    # base frame and casters
    for yy in (-w / 2 + 0.04, w / 2 - 0.04):
        bar(L, g, (0.05, yy, 0.16), (ln, yy, 0.16), 0.025)
    for xx in (0.1, ln - 0.08):
        for yy in (-w / 2 + 0.06, w / 2 - 0.06):
            box(L, MAT["rubber"], xx - 0.05, yy - 0.02, 0.0, xx + 0.05, yy + 0.02, 0.1)
            bar(L, g, (xx, yy, 0.1), (xx, yy, 0.16), 0.02)
    # lower tray
    for k in range(6):
        yy = -w / 2 + 0.05 + k * (w - 0.1) / 5
        bar(L, c, (0.08, yy, 0.18), (ln - 0.05, yy, 0.18), 0.008)
    # basket: vertical posts, top rail, wires on the sides and front
    corners = [(0.0, -w / 2), (ln, -w / 2), (ln, w / 2), (0.0, w / 2)]
    for (xa, ya), (xb, yb) in zip(corners, corners[1:] + corners[:1]):
        bar(L, c, (xa, ya, bz1), (xb, yb, bz1), 0.014)
        bar(L, c, (xa, ya, bz0), (xb, yb, bz0), 0.012)
        n = int(max(abs(xb - xa), abs(yb - ya)) / 0.09)
        for k in range(1, n):
            t = k / n
            px, py = xa + t * (xb - xa), ya + t * (yb - ya)
            bar(L, c, (px, py, bz0), (px, py, bz1), 0.006)
        for zz in (bz0 + 0.16, bz0 + 0.33):
            bar(L, c, (xa, ya, zz), (xb, yb, zz), 0.006)
    for xx, yy in corners:
        bar(L, g, (xx, yy, 0.16), (xx, yy, bz1), 0.02)
    # basket floor
    for k in range(8):
        yy = -w / 2 + 0.04 + k * (w - 0.08) / 7
        bar(L, c, (0.0, yy, bz0), (ln, yy, bz0), 0.006)
    # handle with red grip
    bar(L, g, (-0.05, -w / 2, bz1), (-0.12, -w / 2, 1.02), 0.02)
    bar(L, g, (-0.05, w / 2, bz1), (-0.12, w / 2, 1.02), 0.02)
    bar(L, MAT["red"], (-0.12, -w / 2 - 0.02, 1.02), (-0.12, w / 2 + 0.02, 1.02), 0.035)
    merge(b, L, transform(yaw, (x, y, 0)))


def pallet_jack(b, x, y, yaw=0.0):
    """Manual pallet jack, forks down (75 mm), local +x = fork direction."""
    L = Builder()
    for yy in (-0.28, 0.12):
        box(L, MAT["orange"], 0.0, yy, 0.0, 1.15, yy + 0.16, 0.075, tile=(0.5, 0.075))
        box(L, MAT["rubber"], 1.02, yy + 0.03, 0.0, 1.12, yy + 0.13, 0.02)
    box(L, MAT["orange"], -0.25, -0.28, 0.0, 0.0, 0.28, 0.2)
    box(L, MAT["dark"], -0.22, -0.08, 0.2, -0.06, 0.08, 0.55)
    cylinder(L, MAT["rubber"], -0.14, 0.09, -0.06, 0.06, 0.09, segments=14, axis="y")
    bar(L, MAT["dark"], (-0.14, 0, 0.55), (-0.55, 0, 1.2), 0.035)
    bar(L, MAT["black"], (-0.55, -0.16, 1.2), (-0.55, 0.16, 1.2), 0.04)
    merge(b, L, transform(yaw, (x, y, 0)))


def wet_floor_sign(b, x, y, yaw=0.0):
    L = Builder()
    ang = math.radians(14)
    hgt, wid = 0.62, 0.3
    for sgn in (1, -1):
        tb = Builder()
        box(tb, tex_mat("wet_floor_panel", rough=0.4, ext="png"), -0.006, -wid / 2, 0, 0.006, wid / 2, hgt, tile=(wid, hgt),
            origin=(0, -wid / 2, 0))
        # rotate about y so the top of the panel at x = sgn * 0.075 leans back toward x = 0
        a = -sgn * ang
        R = np.eye(4)
        R[:3, :3] = [[math.cos(a), 0, math.sin(a)], [0, 1, 0], [-math.sin(a), 0, math.cos(a)]]
        merge(L, tb, transform(0, (sgn * hgt * math.sin(ang), 0, 0)) @ R)
    box(L, MAT["yellow"], -0.025, -wid / 2 + 0.02, hgt * math.cos(ang) - 0.03, 0.025, wid / 2 - 0.02,
        hgt * math.cos(ang) + 0.01)
    merge(b, L, transform(yaw, (x, y, 0)))


def column(b, x, y, height, guard=True):
    box(b, MAT["grey"], x - 0.18, y - 0.18, 0, x + 0.18, y + 0.18, height, tile=(0.36, 1.0))
    if guard:
        box(b, MAT["yellow"], x - 0.3, y - 0.3, 0, x + 0.3, y + 0.3, 1.2, tile=(0.6, 1.2))
        box(b, MAT["stripes"], x - 0.301, y - 0.301, 1.0, x + 0.301, y + 0.301, 1.2, tile=(0.6, 0.15))


def sign_board(b, x, y, z, w, h, name, facing="s", thick=0.04, ext="jpg"):
    """Flat sign (texture `name`) whose printed face looks toward `facing` (n/s/e/w)."""
    m = tex_mat(name, rough=0.5, ext=ext)
    if facing in "ns":
        box(b, m, x - w / 2, y - thick / 2, z, x + w / 2, y + thick / 2, z + h, tile=(w, h),
            origin=(x - w / 2, y - thick / 2, z), faces=facing)
        box(b, MAT["dark"], x - w / 2, y - thick / 2, z, x + w / 2, y + thick / 2, z + h, faces="ewtb" + ("n" if facing == "s" else "s"))
    else:
        box(b, m, x - thick / 2, y - w / 2, z, x + thick / 2, y + w / 2, z + h, tile=(w, h),
            origin=(x - thick / 2, y - w / 2 if facing == "e" else y + w / 2, z), faces=facing)
        box(b, MAT["dark"], x - thick / 2, y - w / 2, z, x + thick / 2, y + w / 2, z + h, faces="nstb" + ("w" if facing == "e" else "e"))


# ---- building shell -----------------------------------------------------------------------------------------
def floor(b, x0, y0, x1, y1, z=0.0):
    quad_xy(b, MAT["floor"], x0, y0, x1, y1, z, tile=(6.0, 6.0))


def wall_x(b, x, y0, y1, height, facing="e", thick=0.3):
    """Wall along y whose finished face is the plane x = `x`, looking toward `facing` ("e" or "w");
    the wall body lies behind it. Painted block wainscot to 2.4 m, metal panels above."""
    x0, x1 = (x - thick, x) if facing == "e" else (x, x + thick)
    box(b, MAT["cmu"], x0, y0, 0, x1, y1, 2.4, tile=(2.4, 1.2), faces=facing)
    box(b, MAT["wall"], x0, y0, 2.4, x1, y1, height, tile=(1.0, 2.0), faces=facing)
    box(b, MAT["dark"], x0, y0, 0, x1, y1, height, faces="nstb" + ("w" if facing == "e" else "e"))


def wall_y(b, y, x0, x1, height, facing="s", thick=0.3):
    """Wall along x whose finished face is the plane y = `y`, looking toward `facing` ("n" or "s")."""
    y0, y1 = (y, y + thick) if facing == "s" else (y - thick, y)
    box(b, MAT["cmu"], x0, y0, 0, x1, y1, 2.4, tile=(2.4, 1.2), faces=facing)
    box(b, MAT["wall"], x0, y0, 2.4, x1, y1, height, tile=(1.0, 2.0), faces=facing)
    box(b, MAT["dark"], x0, y0, 0, x1, y1, height, faces="ewtb" + ("n" if facing == "s" else "s"))


def ceiling(b, x0, y0, x1, y1, z, joist_every=1.8, along="y", lights=True, light_dx=6.0, light_dy=7.5):
    """Roof deck, open-web steel joists, sprinkler mains and LED high-bay fixtures."""
    quad_xy(b, MAT["ceiling"], x0, y0, x1, y1, z, tile=(1.2, 1.2), up=False)
    zc, depth = z - 0.02, 0.75
    if along == "y":
        n = int((x1 - x0) / joist_every)
        for k in range(1, n + 1):
            x = x0 + k * joist_every
            box(b, MAT["white"], x - 0.04, y0, zc - 0.06, x + 0.04, y1, zc, tile=(1, 1))
            box(b, MAT["white"], x - 0.04, y0, zc - depth, x + 0.04, y1, zc - depth + 0.06, tile=(1, 1))
            yy, up = y0, True
            while yy < y1 - 0.01:
                y2 = min(y1, yy + 1.2)
                za, zb = (zc - depth + 0.06, zc - 0.06) if up else (zc - 0.06, zc - depth + 0.06)
                bar(b, MAT["white"], (x, yy, za), (x, y2, zb), 0.025)
                yy, up = y2, not up
    if lights:
        ly = y0 + light_dy / 2
        while ly < y1:
            lx = x0 + light_dx / 2
            while lx < x1:
                cylinder(b, MAT["light"], lx, ly, zc - depth - 0.55, zc - depth - 0.5, 0.26, segments=20)
                cylinder(b, MAT["grey"], lx, ly, zc - depth - 0.5, zc - depth - 0.3, 0.22, segments=16)
                bar(b, MAT["dark"], (lx, ly, zc - depth - 0.3), (lx, ly, zc - depth), 0.01)
                lx += light_dx
            ly += light_dy
    # sprinkler main
    for yy in np.arange(y0 + 3, y1, 12.0):
        bar(b, MAT["pipe_red"], (x0, yy, zc - depth - 0.1), (x1, yy, zc - depth - 0.1), 0.1)
