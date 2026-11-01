"""Minimal glTF 2.0 writer: one mesh per file, one primitive per material, textures as shared external files.

Coordinates are written as given (Gazebo reads them with z up, as tested). UVs follow glTF: (0, 0) is the
top-left corner of the image, v grows downwards.
"""
import json
import struct
from pathlib import Path

import numpy as np


class Material:
    def __init__(self, name, color=(1, 1, 1, 1), texture=None, roughness=0.8, metallic=0.0, emissive=None,
                 alpha_blend=False, double_sided=False, emissive_map=False):
        self.name, self.color, self.texture = name, tuple(color), texture
        self.roughness, self.metallic, self.emissive = roughness, metallic, emissive
        self.alpha_blend, self.double_sided = alpha_blend, double_sided
        self.emissive_map = emissive_map          # glow with the base texture (lit signs, screens)


class Batch:
    """Geometry that shares one material."""

    def __init__(self):
        self.P, self.N, self.U, self.F, self.n = [], [], [], [], 0

    def add(self, P, N, U, F):
        P, N, U, F = (np.asarray(a) for a in (P, N, U, F))
        self.P.append(P.astype(np.float32)); self.N.append(N.astype(np.float32))
        self.U.append(U.astype(np.float32)); self.F.append((F + self.n).astype(np.uint32))
        self.n += len(P)

    def arrays(self):
        return (np.concatenate(self.P), np.concatenate(self.N), np.concatenate(self.U), np.concatenate(self.F))


class Builder:
    def __init__(self):
        self.batches = {}      # material name -> Batch
        self.materials = {}    # material name -> Material

    def batch(self, mat):
        if mat.name not in self.materials:
            self.materials[mat.name] = mat
            self.batches[mat.name] = Batch()
        return self.batches[mat.name]

    def triangles(self):
        return sum(len(np.concatenate(b.F)) for b in self.batches.values() if b.F)

    def write(self, path, texture_dir_rel="../textures"):
        path = Path(path)
        bin_parts, views, accessors, prims = [], [], [], []
        offset = 0

        def push(data, target, comp, count, typ, minmax=None):
            nonlocal offset
            raw = data.tobytes()
            pad = (-len(raw)) % 4
            bin_parts.append(raw + b"\0" * pad)
            views.append(dict(buffer=0, byteOffset=offset, byteLength=len(raw), target=target))
            acc = dict(bufferView=len(views) - 1, componentType=comp, count=count, type=typ)
            if minmax is not None:
                acc["min"], acc["max"] = minmax
            accessors.append(acc)
            offset += len(raw) + pad
            return len(accessors) - 1

        mats, textures, images, image_index = [], [], [], {}
        for name, b in self.batches.items():
            if not b.P:
                continue
            P, N, U, F = b.arrays()
            ip = push(P, 34962, 5126, len(P), "VEC3", ([float(v) for v in P.min(0)], [float(v) for v in P.max(0)]))
            inn = push(N, 34962, 5126, len(N), "VEC3")
            iu = push(U, 34962, 5126, len(U), "VEC2")
            ii = push(F.reshape(-1), 34963, 5125, F.size, "SCALAR")
            m = self.materials[name]
            pbr = dict(baseColorFactor=list(m.color), metallicFactor=m.metallic, roughnessFactor=m.roughness)
            if m.texture:
                if m.texture not in image_index:
                    images.append(dict(uri=f"{texture_dir_rel}/{m.texture}"))
                    textures.append(dict(source=len(images) - 1, sampler=0))
                    image_index[m.texture] = len(textures) - 1
                pbr["baseColorTexture"] = dict(index=image_index[m.texture])
            md = dict(name=name, pbrMetallicRoughness=pbr, doubleSided=m.double_sided)
            if m.emissive:
                md["emissiveFactor"] = list(m.emissive)
                if m.emissive_map and m.texture:
                    md["emissiveTexture"] = dict(index=image_index[m.texture])
            if m.alpha_blend:
                md["alphaMode"] = "BLEND"
            mats.append(md)
            prims.append(dict(attributes=dict(POSITION=ip, NORMAL=inn, TEXCOORD_0=iu), indices=ii,
                              material=len(mats) - 1))
        blob = b"".join(bin_parts)
        bin_name = path.with_suffix(".bin").name
        (path.parent / bin_name).write_bytes(blob)
        doc = dict(asset=dict(version="2.0", generator="roamview_sim club generator"), scene=0,
                   scenes=[dict(nodes=[0])], nodes=[dict(mesh=0, name=path.stem)],
                   meshes=[dict(name=path.stem, primitives=prims)], materials=mats,
                   buffers=[dict(uri=bin_name, byteLength=len(blob))], bufferViews=views, accessors=accessors)
        if textures:
            doc["textures"], doc["images"] = textures, images
            doc["samplers"] = [dict(magFilter=9729, minFilter=9987, wrapS=10497, wrapT=10497)]
        path.write_text(json.dumps(doc, separators=(",", ":")))
        return len(blob)


# ---------------------------------------------------------------------------------------------------
# primitives
# ---------------------------------------------------------------------------------------------------
def box(builder, mat, x0, y0, z0, x1, y1, z1, tile=(1.0, 1.0), origin=None, faces="all"):
    """Axis-aligned box. tile = (width, height) in metres covered by one copy of the texture; text on side
    faces reads correctly from outside. origin = (x, y, z) where texture tiles start (default: min corner).
    faces: 'all' or a string of face letters from 'nsewtb' (north +y, south -y, east +x, west -x, top, bottom)."""
    x0, x1 = min(x0, x1), max(x0, x1); y0, y1 = min(y0, y1), max(y0, y1); z0, z1 = min(z0, z1), max(z0, z1)
    ox, oy, oz = origin if origin is not None else (x0, y0, z0)
    tw, th = tile
    P, N, U, F = [], [], [], []

    def quad(verts, normal, uv):
        base = len(P)
        P.extend(verts); N.extend([normal] * 4); U.extend(uv)
        F.extend([(base, base + 1, base + 2), (base, base + 2, base + 3)])

    top_v = lambda z: (oz + th * np.ceil((z1 - oz) / th - 1e-9) - z) / th   # v measured down from a tile top
    sel = "nsewtb" if faces == "all" else faces
    if "s" in sel:   # -y, viewer looks +y: u = x
        quad([(x0, y0, z0), (x1, y0, z0), (x1, y0, z1), (x0, y0, z1)], (0, -1, 0),
             [((x0 - ox) / tw, top_v(z0)), ((x1 - ox) / tw, top_v(z0)), ((x1 - ox) / tw, top_v(z1)), ((x0 - ox) / tw, top_v(z1))])
    if "n" in sel:   # +y, viewer looks -y: u = -x
        quad([(x1, y1, z0), (x0, y1, z0), (x0, y1, z1), (x1, y1, z1)], (0, 1, 0),
             [((ox - x1) / tw, top_v(z0)), ((ox - x0) / tw, top_v(z0)), ((ox - x0) / tw, top_v(z1)), ((ox - x1) / tw, top_v(z1))])
    if "w" in sel:   # -x, viewer looks +x: u = -y
        quad([(x0, y1, z0), (x0, y0, z0), (x0, y0, z1), (x0, y1, z1)], (-1, 0, 0),
             [((oy - y1) / tw, top_v(z0)), ((oy - y0) / tw, top_v(z0)), ((oy - y0) / tw, top_v(z1)), ((oy - y1) / tw, top_v(z1))])
    if "e" in sel:   # +x, viewer looks -x: u = y
        quad([(x1, y0, z0), (x1, y1, z0), (x1, y1, z1), (x1, y0, z1)], (1, 0, 0),
             [((y0 - oy) / tw, top_v(z0)), ((y1 - oy) / tw, top_v(z0)), ((y1 - oy) / tw, top_v(z1)), ((y0 - oy) / tw, top_v(z1))])
    if "t" in sel:
        quad([(x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)], (0, 0, 1),
             [((x0 - ox) / tw, (oy - y0) / th), ((x1 - ox) / tw, (oy - y0) / th), ((x1 - ox) / tw, (oy - y1) / th), ((x0 - ox) / tw, (oy - y1) / th)])
    if "b" in sel:
        quad([(x0, y1, z0), (x1, y1, z0), (x1, y0, z0), (x0, y0, z0)], (0, 0, -1),
             [((x0 - ox) / tw, (y1 - oy) / th), ((x1 - ox) / tw, (y1 - oy) / th), ((x1 - ox) / tw, (y0 - oy) / th), ((x0 - ox) / tw, (y0 - oy) / th)])
    builder.batch(mat).add(P, N, U, F)


def cylinder(builder, mat, cx, cy, z0, z1, r, segments=16, axis="z", caps=True, tile=(1.0, 1.0)):
    """Cylinder along z (or 'x' / 'y' with cx, cy the other two coordinates and z0..z1 along the axis)."""
    a = np.linspace(0, 2 * np.pi, segments + 1)
    P, N, U, F = [], [], [], []
    for k in range(segments + 1):
        c, s = np.cos(a[k]), np.sin(a[k])
        for z in (z0, z1):
            P.append((c * r, s * r, z)); N.append((c, s, 0)); U.append((a[k] * r / tile[0], (z1 - z) / tile[1]))
    for k in range(segments):
        i = 2 * k
        F += [(i, i + 2, i + 3), (i, i + 3, i + 1)]
    if caps:
        for z, nz in ((z0, -1), (z1, 1)):
            base = len(P)
            P.append((0, 0, z)); N.append((0, 0, nz)); U.append((0.5, 0.5))
            for k in range(segments + 1):
                P.append((np.cos(a[k]) * r, np.sin(a[k]) * r, z)); N.append((0, 0, nz))
                U.append((0.5 + 0.5 * np.cos(a[k]), 0.5 + 0.5 * np.sin(a[k])))
            for k in range(segments):
                F.append((base, base + 1 + k, base + 2 + k) if nz > 0 else (base, base + 2 + k, base + 1 + k))
    P, N = np.array(P, float), np.array(N, float)
    if axis == "x":
        P = P[:, [2, 0, 1]]; N = N[:, [2, 0, 1]]
        P += (0, cx, cy)
    elif axis == "y":
        P = P[:, [0, 2, 1]]; N = N[:, [0, 2, 1]]
        P += (cx, 0, cy)
        F = [(f[0], f[2], f[1]) for f in F]
    else:
        P += (cx, cy, 0)
    builder.batch(mat).add(P, N, np.array(U, float), np.array(F, int))


def quad(builder, mat, p0, p1, p2, p3, uv=((0, 1), (1, 1), (1, 0), (0, 0))):
    """Any planar quad: p0 bottom-left, p1 bottom-right, p2 top-right, p3 top-left, counter-clockwise seen
    from the front (the side the normal points to)."""
    P = np.array([p0, p1, p2, p3], float)
    n = np.cross(P[1] - P[0], P[2] - P[0])
    n /= np.linalg.norm(n)
    builder.batch(mat).add(P, np.repeat(n[None, :], 4, 0), np.array(uv, float), np.array([(0, 1, 2), (0, 2, 3)]))


def quad_xy(builder, mat, x0, y0, x1, y1, z, tile=(1.0, 1.0), up=True):
    """Horizontal rectangle (floor, ceiling) with world-anchored tiling."""
    P = [(x0, y0, z), (x1, y0, z), (x1, y1, z), (x0, y1, z)]
    U = [(x0 / tile[0], -y0 / tile[1]), (x1 / tile[0], -y0 / tile[1]), (x1 / tile[0], -y1 / tile[1]), (x0 / tile[0], -y1 / tile[1])]
    F = [(0, 1, 2), (0, 2, 3)] if up else [(0, 2, 1), (0, 3, 2)]
    builder.batch(mat).add(P, [(0, 0, 1 if up else -1)] * 4, U, F)
