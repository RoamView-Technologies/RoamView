"""
3D cable router for the RoamView interconnect model.

Occupancy grid (2 mm) built from the exact part envelopes and plates; cables may
only cross a plate through its cable pass-throughs. A* (numba) finds each route,
routes already laid attract later ones (bundling) and repel overlap, then each
route is shortened where that lowers cost, filleted to a bend radius and resampled.
"""
import math
import numpy as np
import numba
import shapely
from shapely.ops import unary_union
from scipy import ndimage

import model_b as M
import rules_b

STEP = 2.0
X0, X1 = -172.0, 176.0
Y0, Y1 = 30.0, 276.0
Z0, Z1 = -140.0, 140.0
NX = int(round((X1 - X0) / STEP)) + 1
NY = int(round((Y1 - Y0) / STEP)) + 1
NZ = int(round((Z1 - Z0) / STEP)) + 1
CLR = 3.0            # clearance used for routing (cable radius + margin)

xs = X0 + STEP * np.arange(NX)
ys = Y0 + STEP * np.arange(NY)
zs = Z0 + STEP * np.arange(NZ)


def idx(v, lo):
    return int(round((v - lo) / STEP))


# ------------------------------------------------------------------ geometry
def lathe_radius_fn(profile):
    """Return R(a): max radius of the closed profile polygon at axial position a."""
    pts = [tuple(p) for p in profile]
    edges = list(zip(pts, pts[1:] + pts[:1]))

    def R(a, tol=0.0):
        best = -1.0
        for (r1, a1), (r2, a2) in edges:
            if abs(a2 - a1) < 1e-9:
                if abs(a - a1) <= tol + 1e-6:
                    best = max(best, r1, r2)
            elif min(a1, a2) - 1e-6 <= a <= max(a1, a2) + 1e-6:
                best = max(best, r1 + (r2 - r1) * (a - a1) / (a2 - a1))
        return best
    return R


def rasterize_prims(prims, grid, c):
    """Mark grid cells whose centre is within ~c of any primitive."""
    for p in prims:
        if p["t"] == "box" or p["t"] == "p2020":
            if p["t"] == "box":
                lo = np.array(p["min"]) - c
                hi = np.array(p["max"]) + c
            else:
                o = np.array(p["origin"])
                lo = o + np.array([-10, 0, -10]) - c
                hi = o + np.array([10, p["h"], 10]) + c
            i0, i1 = max(0, math.ceil((lo[0] - X0) / STEP)), min(NX - 1, math.floor((hi[0] - X0) / STEP))
            j0, j1 = max(0, math.ceil((lo[1] - Y0) / STEP)), min(NY - 1, math.floor((hi[1] - Y0) / STEP))
            k0, k1 = max(0, math.ceil((lo[2] - Z0) / STEP)), min(NZ - 1, math.floor((hi[2] - Z0) / STEP))
            if i0 <= i1 and j0 <= j1 and k0 <= k1:
                grid[i0:i1 + 1, j0:j1 + 1, k0:k1 + 1] = True
        elif p["t"] == "lathe":
            o = np.array(p["origin"], float)
            prof = np.array(p["profile"], float)
            rmax = prof[:, 0].max()
            amin, amax = prof[:, 1].min(), prof[:, 1].max()
            R = lathe_radius_fn(p["profile"])
            if p["axis"] == "y":
                j0 = max(0, math.ceil((o[1] + amin - c - Y0) / STEP)); j1 = min(NY - 1, math.floor((o[1] + amax + c - Y0) / STEP))
                i0 = max(0, math.ceil((o[0] - rmax - c - X0) / STEP)); i1 = min(NX - 1, math.floor((o[0] + rmax + c - X0) / STEP))
                k0 = max(0, math.ceil((o[2] - rmax - c - Z0) / STEP)); k1 = min(NZ - 1, math.floor((o[2] + rmax + c - Z0) / STEP))
                if i0 > i1 or j0 > j1 or k0 > k1:
                    continue
                XX, ZZ = np.meshgrid(xs[i0:i1 + 1] - o[0], zs[k0:k1 + 1] - o[2], indexing="ij")
                rad = np.hypot(XX, ZZ)
                for j in range(j0, j1 + 1):
                    a = ys[j] - o[1]
                    rr = max(R(a + d, 0.0) for d in np.linspace(-c, c, 7))
                    if rr < 0:
                        continue
                    grid[i0:i1 + 1, j, k0:k1 + 1] |= rad <= rr + c
            else:  # axis z
                k0 = max(0, math.ceil((o[2] + amin - c - Z0) / STEP)); k1 = min(NZ - 1, math.floor((o[2] + amax + c - Z0) / STEP))
                i0 = max(0, math.ceil((o[0] - rmax - c - X0) / STEP)); i1 = min(NX - 1, math.floor((o[0] + rmax + c - X0) / STEP))
                j0 = max(0, math.ceil((o[1] - rmax - c - Y0) / STEP)); j1 = min(NY - 1, math.floor((o[1] + rmax + c - Y0) / STEP))
                if i0 > i1 or j0 > j1 or k0 > k1:
                    continue
                XX, YY = np.meshgrid(xs[i0:i1 + 1] - o[0], ys[j0:j1 + 1] - o[1], indexing="ij")
                rad = np.hypot(XX, YY)
                for k in range(k0, k1 + 1):
                    a = zs[k] - o[2]
                    rr = max(R(a + d, 0.0) for d in np.linspace(-c, c, 7))
                    if rr < 0:
                        continue
                    grid[i0:i1 + 1, j0:j1 + 1, k] |= rad <= rr + c


def plate_columns(plate, c):
    """Boolean (NX, NZ): True where a cable may NOT pass through this plate."""
    outline = plate["poly"]
    solid = shapely.Polygon(outline.exterior)            # outline without its holes
    passable = unary_union(plate["holes"]["pass_through"]).buffer(-c)
    XX, ZZ = np.meshgrid(xs, zs, indexing="ij")
    pts = shapely.points(XX.ravel(), ZZ.ravel())
    near_solid = shapely.dwithin(solid, pts, c) if hasattr(shapely, "dwithin") else (shapely.distance(solid, pts) <= c)
    inside_pass = shapely.contains(passable, pts) if not passable.is_empty else np.zeros(len(pts), bool)
    return (near_solid & ~inside_pass).reshape(NX, NZ)


def build_grids(c=CLR, hidden_ok=False):
    real = np.zeros((NX, NY, NZ), bool)
    for p in M.PARTS:
        if p.get("hidden") and not hidden_ok:
            continue
        rasterize_prims(p["prims"], real, c)
        if p.get("moving") and p["id"].startswith("motor_"):
            for deg in (M.BUMP_DEG, M.DROOP_DEG):             # the drive module's swept volume
                rasterize_prims(rules_b.rotated_prims(p["prims"], deg), real, c)
    for pl in M.plates():
        cols = plate_columns(pl, c)
        j0 = max(0, math.ceil((pl["y0"] - c - Y0) / STEP)); j1 = min(NY - 1, math.floor((pl["y1"] + c - Y0) / STEP))
        for j in range(j0, j1 + 1):
            real[:, j, :] |= cols
    XX, YY, ZZ = np.meshgrid(xs, ys, zs, indexing="ij")
    keep = M.keepouts(XX, YY, ZZ)
    return real, keep


# ------------------------------------------------------------------ A*
NB = np.array([(dx, dy, dz) for dx in (-1, 0, 1) for dy in (-1, 0, 1) for dz in (-1, 0, 1)
               if (dx, dy, dz) != (0, 0, 0)], np.int64)
NBL = np.sqrt((NB ** 2).sum(1)).astype(np.float64)


HF = np.sqrt((NB[:, 0] ** 2 + NB[:, 2] ** 2)).astype(np.float64) / NBL          # horizontal fraction
DG = np.array([0.0 if (a == 0 or b == 0) else 1.0 - 1.0 / math.sqrt(2.0) for a, b in zip(NB[:, 0], NB[:, 2])])
ORTHO = 1.6   # penalty on diagonal travel in the X-Z plane (keeps harness runs square to the plates)


@numba.njit(cache=True)
def move_mult(air_u, air_v, h, dg, ortho):
    a = 0.5 * (air_u + air_v)
    return (1.0 + ortho * dg) * (1.0 + a * (0.2 + 1.8 * h))


@numba.njit(cache=True)
def _astar2(cost, air, blocked, s, t, NB, NBL, HF, DG, ortho, step, hmin):
    nx, ny, nz = cost.shape
    N = nx * ny * nz
    g = np.full(N, 1e30)
    par = np.full(N, -1, np.int64)
    closed = np.zeros(N, np.uint8)
    si = (s[0] * ny + s[1]) * nz + s[2]
    ti = (t[0] * ny + t[1]) * nz + t[2]
    g[si] = 0.0
    heap = [(0.0, si)]
    while len(heap) > 0:
        f, u = heapq_pop(heap)
        if closed[u]:
            continue
        if u == ti:
            break
        closed[u] = 1
        ux = u // (ny * nz)
        uy = (u // nz) % ny
        uz = u % nz
        cu = cost[ux, uy, uz]
        au = air[ux, uy, uz]
        for k in range(26):
            vx = ux + NB[k, 0]
            vy = uy + NB[k, 1]
            vz = uz + NB[k, 2]
            if vx < 0 or vy < 0 or vz < 0 or vx >= nx or vy >= ny or vz >= nz:
                continue
            if blocked[vx, vy, vz]:
                continue
            v = (vx * ny + vy) * nz + vz
            if closed[v]:
                continue
            nc = g[u] + NBL[k] * step * 0.5 * (cu + cost[vx, vy, vz]) * move_mult(au, air[vx, vy, vz], HF[k], DG[k], ortho)
            if nc < g[v]:
                g[v] = nc
                par[v] = u
                dx = (vx - t[0]); dy = (vy - t[1]); dz = (vz - t[2])
                h = math.sqrt(dx * dx + dy * dy + dz * dz) * step * hmin
                heapq_push(heap, (nc + h, v))
    if g[ti] >= 1e29:
        return np.empty((0, 3), np.int64)
    path = []
    u = ti
    while u != -1:
        path.append(u)
        u = par[u]
    out = np.empty((len(path), 3), np.int64)
    for i in range(len(path)):
        u = path[len(path) - 1 - i]
        out[i, 0] = u // (ny * nz)
        out[i, 1] = (u // nz) % ny
        out[i, 2] = u % nz
    return out


@numba.njit(cache=True)
def _astar(cost, blocked, s, t, NB, NBL, step, hmin):
    nx, ny, nz = cost.shape
    N = nx * ny * nz
    g = np.full(N, 1e30)
    par = np.full(N, -1, np.int64)
    closed = np.zeros(N, np.uint8)
    si = (s[0] * ny + s[1]) * nz + s[2]
    ti = (t[0] * ny + t[1]) * nz + t[2]
    g[si] = 0.0
    heap = [(0.0, si)]
    while len(heap) > 0:
        f, u = heapq_pop(heap)
        if closed[u]:
            continue
        if u == ti:
            break
        closed[u] = 1
        ux = u // (ny * nz)
        uy = (u // nz) % ny
        uz = u % nz
        cu = cost[ux, uy, uz]
        for k in range(26):
            vx = ux + NB[k, 0]
            vy = uy + NB[k, 1]
            vz = uz + NB[k, 2]
            if vx < 0 or vy < 0 or vz < 0 or vx >= nx or vy >= ny or vz >= nz:
                continue
            if blocked[vx, vy, vz]:
                continue
            v = (vx * ny + vy) * nz + vz
            if closed[v]:
                continue
            nc = g[u] + NBL[k] * step * 0.5 * (cu + cost[vx, vy, vz])
            if nc < g[v]:
                g[v] = nc
                par[v] = u
                dx = (vx - t[0]); dy = (vy - t[1]); dz = (vz - t[2])
                h = math.sqrt(dx * dx + dy * dy + dz * dz) * step * hmin
                heapq_push(heap, (nc + h, v))
    if g[ti] >= 1e29:
        return np.empty((0, 3), np.int64)
    path = []
    u = ti
    while u != -1:
        path.append(u)
        u = par[u]
    out = np.empty((len(path), 3), np.int64)
    for i in range(len(path)):
        u = path[len(path) - 1 - i]
        out[i, 0] = u // (ny * nz)
        out[i, 1] = (u // nz) % ny
        out[i, 2] = u % nz
    return out


@numba.njit(cache=True)
def heapq_push(h, item):
    h.append(item)
    i = len(h) - 1
    while i > 0:
        p = (i - 1) >> 1
        if h[i][0] < h[p][0]:
            h[i], h[p] = h[p], h[i]
            i = p
        else:
            break


@numba.njit(cache=True)
def heapq_pop(h):
    last = h.pop()
    if len(h) == 0:
        return last
    top = h[0]
    h[0] = last
    i = 0
    n = len(h)
    while True:
        l = 2 * i + 1
        r = l + 1
        m = i
        if l < n and h[l][0] < h[m][0]:
            m = l
        if r < n and h[r][0] < h[m][0]:
            m = r
        if m == i:
            break
        h[i], h[m] = h[m], h[i]
        i = m
    return top


# ------------------------------------------------------------------ helpers
def to_cell(p):
    return np.array([idx(p[0], X0), idx(p[1], Y0), idx(p[2], Z0)])


def to_mm(c):
    return np.array([X0 + STEP * c[0], Y0 + STEP * c[1], Z0 + STEP * c[2]], float)


def nearest_free(p, blocked, max_r=6):
    c0 = to_cell(p)
    best, bd = None, 1e9
    for r in range(0, max_r + 1):
        for dx in range(-r, r + 1):
            for dy in range(-r, r + 1):
                for dz in range(-r, r + 1):
                    c = c0 + [dx, dy, dz]
                    if (c < 0).any() or c[0] >= NX or c[1] >= NY or c[2] >= NZ:
                        continue
                    if not blocked[tuple(c)]:
                        d = np.linalg.norm(to_mm(c) - p)
                        if d < bd:
                            best, bd = c, d
        if best is not None:
            return best
    return None


def seg_ok(a, b, blocked):
    n = max(2, int(np.linalg.norm(b - a) / 0.8))
    for t in np.linspace(0, 1, n):
        c = to_cell(a + (b - a) * t)
        if (c < 0).any() or c[0] >= NX or c[1] >= NY or c[2] >= NZ or blocked[tuple(c)]:
            return False
    return True


AIR = None


def seg_cost(a, b, cost):
    d = b - a
    L = np.linalg.norm(d)
    if L < 1e-9:
        return 0.0
    h = math.hypot(d[0], d[2]) / L
    hx, hz = abs(d[0]), abs(d[2])
    dg = 0.0 if max(hx, hz) < 1e-9 else (1.0 - max(hx, hz) / math.hypot(hx, hz))
    n = max(2, int(L / 1.0))
    tot = 0.0
    for t in np.linspace(0, 1, n):
        c = tuple(np.clip(to_cell(a + d * t), 0, [NX - 1, NY - 1, NZ - 1]))
        air = AIR[c] if AIR is not None else 0.0
        tot += cost[c] * (1.0 + ORTHO * dg) * (1.0 + air * (0.2 + 1.8 * h))
    return tot / n * L


def shorten(pts, blocked, cost, max_len=70.0):
    """Greedy any-angle pruning: replace a run by a straight segment when that is
    collision-free and does not cost more than the run it replaces."""
    out = [pts[0]]
    i = 0
    n = len(pts)
    while i < n - 1:
        best = i + 1
        run = 0.0
        for j in range(i + 2, n):
            run += seg_cost(pts[j - 1], pts[j], cost)
            if np.linalg.norm(pts[j] - pts[i]) > max_len:
                break
            if seg_ok(pts[i], pts[j], blocked):
                sub = sum(seg_cost(pts[k], pts[k + 1], cost) for k in range(i, j))
                if seg_cost(pts[i], pts[j], cost) <= sub * 1.001:
                    best = j
        out.append(pts[best])
        i = best
    return out


def fillet(pts, R):
    """Round every interior corner of a polyline with radius <= R; return dense points."""
    pts = [np.asarray(p, float) for p in pts]
    # drop duplicates
    q = [pts[0]]
    for p in pts[1:]:
        if np.linalg.norm(p - q[-1]) > 1e-6:
            q.append(p)
    pts = q
    if len(pts) < 3:
        return resample(pts, 1.5)
    out = [pts[0]]
    for i in range(1, len(pts) - 1):
        a, b, c = pts[i - 1], pts[i], pts[i + 1]
        u = (a - b); lu = np.linalg.norm(u); u /= lu
        v = (c - b); lv = np.linalg.norm(v); v /= lv
        cosang = np.clip(np.dot(u, v), -1, 1)
        ang = math.acos(cosang)
        if ang > math.pi - 1e-3 or ang < 1e-3:
            out.append(b)
            continue
        t = math.tan(ang / 2)
        r = min(R, 0.45 * min(lu, lv) * t)
        d = r / t
        p1 = b + u * d
        p2 = b + v * d
        bis = (u + v); bis /= np.linalg.norm(bis)
        center = b + bis * (r / math.sin(ang / 2))
        e1 = p1 - center; e2 = p2 - center
        theta = math.pi - ang
        n = max(3, int(theta * r / 1.5) + 2)
        e1n = e1 / np.linalg.norm(e1)
        w = e2 - np.dot(e2, e1n) * e1n
        wn = np.linalg.norm(w)
        if wn < 1e-9:
            out.append(b)
            continue
        w /= wn
        for k in range(n):
            th = theta * k / (n - 1)
            out.append(center + r * (math.cos(th) * e1n + math.sin(th) * w))
    out.append(pts[-1])
    return resample(out, 1.5)


def resample(pts, ds):
    pts = np.array(pts, float)
    seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    s = np.concatenate([[0], np.cumsum(seg)])
    if s[-1] < 1e-9:
        return pts[:1]
    n = max(2, int(math.ceil(s[-1] / ds)) + 1)
    t = np.linspace(0, s[-1], n)
    return np.stack([np.interp(t, s, pts[:, k]) for k in range(3)], 1)


def length(pts):
    return float(np.linalg.norm(np.diff(np.asarray(pts), axis=0), axis=1).sum())


def rmf_offsets(center, n0):
    """Rotation-minimising frame normals along a polyline (double reflection)."""
    P = np.asarray(center)
    T = np.gradient(P, axis=0)
    T /= np.linalg.norm(T, axis=1, keepdims=True)
    N = np.zeros_like(P)
    n = n0 - np.dot(n0, T[0]) * T[0]
    if np.linalg.norm(n) < 1e-6:
        n = np.cross(T[0], [0, 1, 0])
        if np.linalg.norm(n) < 1e-6:
            n = np.cross(T[0], [1, 0, 0])
    N[0] = n / np.linalg.norm(n)
    for i in range(len(P) - 1):
        v1 = P[i + 1] - P[i]
        c1 = np.dot(v1, v1)
        if c1 < 1e-12:
            N[i + 1] = N[i]
            continue
        rL = N[i] - (2 / c1) * np.dot(v1, N[i]) * v1
        tL = T[i] - (2 / c1) * np.dot(v1, T[i]) * v1
        v2 = T[i + 1] - tL
        c2 = np.dot(v2, v2)
        N[i + 1] = rL - (2 / c2) * np.dot(v2, rL) * v2 if c2 > 1e-12 else rL
        N[i + 1] /= np.linalg.norm(N[i + 1])
    return N


def blend_end(path, target, tdir, span=14.0, at_start=True):
    """Bend the first/last `span` mm of a path onto a terminal at `target` leaving along tdir."""
    P = np.array(path, float)
    if not at_start:
        P = P[::-1]
    s = np.concatenate([[0], np.cumsum(np.linalg.norm(np.diff(P, axis=0), axis=1))])
    lead = [target + tdir * min(t, span) for t in s]
    out = P.copy()
    for i, t in enumerate(s):
        if t > span:
            break
        w = 1 - (0.5 - 0.5 * math.cos(math.pi * t / span))   # 1 at the terminal, 0 at span
        out[i] = w * lead[i] + (1 - w) * P[i]
    out[0] = target
    return out if at_start else out[::-1]
