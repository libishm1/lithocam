# Copyright 2026 The lithocam authors. SPDX-License-Identifier: Apache-2.0
"""Mesh-based stock removal with Manifold (github.com/elalish/manifold, Apache-2.0): diamond-wire cuts including
ruled faces, with the stock, the blank and the removed pieces as watertight meshes.

Planar supporting faces are exact plane splits (kerf = the slab of one wire diameter on the waste side). Ruled
faces are swept: the wire is a cylinder of the wire radius on each kerf-offset ruling, extended past the part, and
the sweep between consecutive rulings is the convex hull of the two wire positions; the union of those hulls is
subtracted from the stock. Rulings are extrapolated one step past both ends of the face so the sweep leaves the
stock. Between rulings the hull is a piecewise approximation of the ruled sweep: the gouge it causes is measured
(target - blank) and shrinks with more rulings (`n_rulings`).

Needs manifold3d and OCP (for the target's B-rep and tessellation).
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional, Sequence

import manifold3d as mf
import numpy as np

from . import ruled as R
from .mesh import triangles_from_shape


def manifold_from_triangles(tris, decimals: int = 6) -> "mf.Manifold":
    """Weld a triangle soup (as tessellated per face) into a closed Manifold."""
    pts = np.asarray(tris, dtype=np.float64).reshape(-1, 3)
    key = np.round(pts, decimals)
    uniq, inv = np.unique(key, axis=0, return_inverse=True)
    tv = inv.reshape(-1, 3).astype(np.uint32)
    tv = tv[(tv[:, 0] != tv[:, 1]) & (tv[:, 1] != tv[:, 2]) & (tv[:, 0] != tv[:, 2])]
    m = mf.Manifold(mf.Mesh(vert_properties=uniq.astype(np.float32), tri_verts=tv))
    if m.status() != mf.Error.NoError:
        raise ValueError("tessellation is not a closed manifold: {}".format(m.status()))
    return m


def to_arrays(m: "mf.Manifold"):
    mesh = m.to_mesh()
    return np.asarray(mesh.vert_properties)[:, :3], np.asarray(mesh.tri_verts)


def write_stl(m: "mf.Manifold", path: str) -> None:
    v, t = to_arrays(m)
    with open(path, "w", encoding="ascii") as f:
        f.write("solid lithocam\n")
        for a, b, c in t:
            f.write("facet normal 0 0 0\n outer loop\n")
            for p in (v[a], v[b], v[c]):
                f.write("  vertex {:.6f} {:.6f} {:.6f}\n".format(*p))
            f.write(" endloop\nendfacet\n")
        f.write("endsolid lithocam\n")


def _frame(d):
    d = d / np.linalg.norm(d)
    a = np.array([1.0, 0, 0]) if abs(d[0]) < 0.9 else np.array([0, 1.0, 0])
    x = np.cross(a, d)
    x /= np.linalg.norm(x)
    y = np.cross(d, x)
    return x, y, d


def wire_cylinder(p0, p1, r: float, segments: int = 48) -> "mf.Manifold":
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    L = float(np.linalg.norm(p1 - p0))
    x, y, z = _frame(p1 - p0)
    M = np.column_stack([x, y, z, p0])            # 3x4: columns = new axes, last = translation
    return mf.Manifold.cylinder(L, r, r, segments).transform(M)


def _exit(p, d, lo, hi):
    """Distance along unit d from p to leave the box [lo, hi] (0 when p is already outside along d)."""
    t = np.inf
    for k in range(3):
        if abs(d[k]) > 1e-12:
            t = min(t, ((hi[k] if d[k] > 0 else lo[k]) - p[k]) / d[k])
    return max(0.0, t)


def ruled_sweep(rulings, r: float, extend: float, segments: int = 48, box=None) -> "mf.Manifold":
    """Union of convex hulls of consecutive wire cylinders along the kerf-offset rulings (ordered along the sweep),
    each ruling extended by `extend` at both ends; one extrapolated ruling added before the first and after the
    last."""
    P0 = np.array([ru.p0 for ru in rulings], float)
    P1 = np.array([ru.p1 for ru in rulings], float)
    N = np.array([ru.normal for ru in rulings], float)
    if len(P0) > 1:
        mids = (P0 + P1) / 2
        c = mids - mids.mean(axis=0)
        order = np.argsort(c @ np.linalg.svd(c, full_matrices=False)[2][0])
        P0, P1, N = P0[order], P1[order], N[order]
        # orient every ruling the same way as the first
        flip = np.einsum("ij,j->i", P1 - P0, P1[0] - P0[0]) < 0
        P0[flip], P1[flip] = P1[flip].copy(), P0[flip].copy()
        P0 = np.vstack([2 * P0[0] - P0[1], P0, 2 * P0[-1] - P0[-2]])
        P1 = np.vstack([2 * P1[0] - P1[1], P1, 2 * P1[-1] - P1[-2]])
        N = np.vstack([N[0], N, N[-1]])
    cyl = []
    for a, b, n in zip(P0, P1, N):
        d = (b - a) / np.linalg.norm(b - a)
        ea = eb = extend
        if box is not None:                              # run each end just past the stock's boundary
            ea = _exit(a + n * r, -d, box[0], box[1]) + extend
            eb = _exit(b + n * r, d, box[0], box[1]) + extend
        cyl.append(wire_cylinder(a + n * r - d * ea, b + n * r + d * eb, r, segments))
    hulls = [mf.Manifold.batch_hull([cyl[i], cyl[i + 1]]) for i in range(len(cyl) - 1)] or cyl
    return mf.Manifold.batch_boolean(hulls, mf.OpType.Add)


@dataclass
class MeshSimReport:
    stock_volume: float
    target_volume: float
    blank_volume: float = 0.0
    to_mill_volume: float = 0.0
    gouge_volume: float = 0.0
    kerf_volume: float = 0.0
    waste_volume: float = 0.0
    cuts: List[tuple] = field(default_factory=list)      # (face, kind, kerf, waste)
    skipped: List[tuple] = field(default_factory=list)
    stages: List[tuple] = field(default_factory=list)    # (label, stock manifold, removed manifold)

    @property
    def balance(self) -> float:
        return self.stock_volume - (self.blank_volume + self.kerf_volume + self.waste_volume)


def _keep_part(pieces, target):
    best, bv = None, -1.0
    for p in pieces:
        v = (p ^ target).volume()
        if v > bv:
            best, bv = p, v
    return best


def simulate(target_shape, rules: Optional[R.Rules] = None, margin: float = 20.0, n_rulings: int = 96,
             segments: int = 48, record: bool = False) -> MeshSimReport:
    """Each wire runs just past the stock (exit distance + 5). The hull of consecutive skew rulings bulges into the
    part in proportion to the wire length: on the twisted column (30 degree twist, 900 mm) one side's sweep overlaps
    the part by 0.28 / 0.16 / 0.09 percent at 24 / 48 / 96 rulings with 30 mm extensions, and the full simulation
    (exit-length wires at the corners, 96 rulings) gouges 0.48 percent. A real straight wire on exact rulings does
    not gouge; this is the mesh approximation's error. Planar cuts are exact."""
    rules = rules or R.Rules()
    rules_dense = R.Rules(**{**rules.__dict__, "n_rulings": n_rulings})
    rep = R.analyse_shape(target_shape, rules_dense)
    scale = rep.scale
    target = manifold_from_triangles(triangles_from_shape(target_shape, deflection=5e-4 * scale))
    lo, hi = np.asarray(target.bounding_box()[:3]), np.asarray(target.bounding_box()[3:])
    stock = mf.Manifold.cube(tuple(hi - lo + 2 * margin)).translate(tuple(lo - margin))
    out = MeshSimReport(stock.volume(), target.volume())
    if record:
        out.stages.append(("stock", stock, None))
    tv = np.asarray(target.to_mesh().vert_properties)[:, :3]
    d = rules.wire_diameter
    r = d / 2
    done = set()
    planar = [f for f in rep.faces if f.label == "wire" and f.surface == "plane" and f.rulings]
    ruled_faces = [f for f in rep.faces if f.label == "wire" and f.surface != "plane" and f.rulings]
    for f in planar:
        n = np.asarray(f.rulings[0].normal)
        p = np.asarray(f.rulings[0].p0)
        if np.max((tv - p) @ n) > 1e-6 * scale:
            out.skipped.append((f.index, "plane cuts through the part: internal or concave contour"))
            continue
        off = float(p @ n)
        key = tuple(np.round(n, 9)) + (round(off, 6),)
        if key in done:
            continue
        done.add(key)
        keep = stock.trim_by_plane(tuple(-n), -off)               # side n . x <= off
        rest = stock - keep
        kerf = rest.trim_by_plane(tuple(-n), -(off + d))          # off < n . x <= off + d
        vk, vw = kerf.volume(), rest.volume() - kerf.volume()
        out.cuts.append((f.index, "plane", vk, vw))
        out.kerf_volume += vk
        out.waste_volume += vw
        if record:
            out.stages.append(("plane cut (face {})".format(f.index), keep, rest))
        stock = keep
    for f in ruled_faces:
        bb = stock.bounding_box()
        sweep = ruled_sweep(f.rulings, r, extend=5.0, segments=segments, box=(np.asarray(bb[:3]), np.asarray(bb[3:])))
        kerf = stock ^ sweep
        rest = stock - sweep
        pieces = rest.decompose()
        keep = _keep_part(pieces, target)
        if keep is None or len(pieces) < 2:
            out.skipped.append((f.index, "ruled sweep did not separate the stock"))
            continue
        vk = kerf.volume()
        vw = rest.volume() - keep.volume()
        out.cuts.append((f.index, "ruled", vk, vw))
        out.kerf_volume += vk
        out.waste_volume += vw
        if record:
            out.stages.append(("ruled cut (face {})".format(f.index), keep, stock - keep))
        stock = keep
    out.blank_volume = stock.volume()
    out.to_mill_volume = (stock - target).volume()
    out.gouge_volume = (target - stock).volume()
    out.blank = stock
    out.target = target
    return out


def simulate_step(path: str, **kw) -> MeshSimReport:
    return simulate(R.read_step(path), **kw)
