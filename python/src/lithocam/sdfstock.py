# Copyright 2026 The lithocam authors. SPDX-License-Identifier: Apache-2.0
"""Stock removal on a dense sampled signed distance field (numpy only).

The stock is a float32 grid of signed distance, negative inside the material. A cut subtracts a tool with
`sdf = max(sdf, -tool_sdf)` over the tool's bounding box only. Any tool pose works: 3+2, 5-axis, wire and disc cuts
(unlike a single-direction Z-map), and one cut costs the size of the tool, not of a mesh boolean.

Method lineage (named works only): signed and adaptive distance fields for machining simulation (Frisken, Perry,
Rockwood and Jones, 2000); surface nets (Gibson, 1998) for the mesh; dexel and tri-dexel stock models (Van Hook,
1986) for the mesh scan.

Approximations, all stated here once:
* The grid stores a NARROW BAND: values are clamped to +-band cells (default 3). The sign is exact; the distance
  is exact only inside the band. A cut updates the tool box grown by the band, so the band stays consistent.
* `from_mesh` takes the distance as the signed minimum of three axis-aligned distances to the nearest triangle
  crossing. That is an upper bound of the true distance, exact on the surface crossings. A first-order correction
  (division by the gradient norm, applied near the surface) restores unit slope for the volume estimate.
* Capsule sweeps (`cut_ball`) are exact. Flat and bull-nose end mills, discs and moving wires are swept by
  sampling tool poses at spacing <= cell/2 along the motion; the scallop between samples is below cell/8 for a
  tool of radius >= 2 cells moving along a line, and shrinks as the cell shrinks.
* Removed volume uses the smoothed occupancy clamp(0.5 - sdf/cell, 0, 1), exact for planar faces and sub-voxel
  accurate on curved ones.
* Memory is dense: nx*ny*nz*4 bytes.
"""
from __future__ import annotations

import math
from typing import Callable, Optional, Sequence, Tuple

import numpy as np

_F32 = np.float32


def _v3(a) -> np.ndarray:
    return np.asarray(a, dtype=np.float64).reshape(3)


def _unit(a) -> np.ndarray:
    a = _v3(a)
    n = float(np.linalg.norm(a))
    if n == 0.0:
        raise ValueError("zero-length direction")
    return a / n


# ---------------------------------------------------------------- exact tool SDFs on broadcast components

def _round_cyl(qx, qy, qz, ax, hc, hl, rr, c):
    """Rounded cylinder about the axis `ax` through the origin: centre height hc along ax, half-length hl, radius rr,
    edge rounding c (c = 0: sharp flat cylinder; c = rr: capsule-like ends). q is relative to the tool origin."""
    h = qx * ax[0] + (qy * ax[1] + qz * ax[2])
    r2 = qx * qx + (qy * qy + qz * qz) - h * h
    rad = np.sqrt(np.maximum(r2, 0.0))
    dx = rad - (rr - c)
    dy = np.abs(h - hc) - (hl - c)
    out = np.hypot(np.maximum(dx, 0.0), np.maximum(dy, 0.0)) + np.minimum(np.maximum(dx, dy), 0.0)
    return out - c


def _capsule(qx, qy, qz, a, b, r):
    """Capsule between points a and b (q is in absolute coordinates)."""
    d = b - a
    dd = float(d @ d)
    px, py, pz = qx - a[0], qy - a[1], qz - a[2]
    if dd < 1e-30:
        return np.sqrt(px * px + (py * py + pz * pz)) - r
    t = np.clip((px * d[0] + (py * d[1] + pz * d[2])) / dd, 0.0, 1.0)
    rx, ry, rz = px - t * d[0], py - t * d[1], pz - t * d[2]
    return np.sqrt(rx * rx + (ry * ry + rz * rz)) - r


def _clip_segment(a: np.ndarray, b: np.ndarray, lo: np.ndarray, hi: np.ndarray):
    """Part of segment ab inside the box [lo, hi] (slab clipping); None when outside."""
    d = b - a
    t0, t1 = 0.0, 1.0
    for k in range(3):
        if abs(d[k]) < 1e-15:
            if a[k] < lo[k] or a[k] > hi[k]:
                return None
        else:
            u0, u1 = (lo[k] - a[k]) / d[k], (hi[k] - a[k]) / d[k]
            if u0 > u1:
                u0, u1 = u1, u0
            t0, t1 = max(t0, u0), min(t1, u1)
            if t0 > t1:
                return None
    return a + t0 * d, a + t1 * d


class SdfStock:
    """Dense sampled signed distance stock. `sdf[i, j, k]` is the value at `origin + (i, j, k) * cell`."""

    def __init__(self, sdf: np.ndarray, origin, cell: float, band: float = 3.0):
        self.sdf = np.ascontiguousarray(sdf, dtype=_F32)
        self.origin = _v3(origin)
        self.cell = float(cell)
        self.band = float(band)

    # ------------------------------------------------------------ constructors
    @staticmethod
    def _grid_for(lo, hi, cell, pad):
        lo, hi = _v3(lo), _v3(hi)
        origin = lo - pad * cell
        shape = tuple(int(math.ceil((hi[k] - origin[k]) / cell - 1e-9)) + 1 + pad for k in range(3))
        return origin, shape

    @classmethod
    def from_box(cls, lo, hi, cell: float, pad: int = 2, band: float = 3.0) -> "SdfStock":
        lo, hi = _v3(lo), _v3(hi)
        origin, shape = cls._grid_for(lo, hi, cell, pad)
        xs = [origin[k] + cell * np.arange(shape[k]) for k in range(3)]
        c, h = (lo + hi) / 2, (hi - lo) / 2
        qx = np.abs(xs[0] - c[0])[:, None, None] - h[0]
        qy = np.abs(xs[1] - c[1])[None, :, None] - h[1]
        qz = np.abs(xs[2] - c[2])[None, None, :] - h[2]
        outside = np.sqrt(np.maximum(qx, 0) ** 2 + np.maximum(qy, 0) ** 2 + np.maximum(qz, 0) ** 2)
        inside = np.minimum(np.maximum(qx, np.maximum(qy, qz)), 0.0)
        s = np.clip(outside + inside, -band * cell, band * cell)
        return cls(s, origin, cell, band)

    @classmethod
    def from_mesh(cls, V, F, cell: float, pad: int = 2, band: float = 3.0) -> "SdfStock":
        """Closed triangle mesh to a narrow-band SDF. Inside/outside by ray parity along x, y and z (a tri-dexel
        scan); distance as the signed minimum of the three axis-aligned distances to the nearest crossing.
        Narrow-band approximation: an upper bound of the true distance, exact on surface crossings, then divided
        by its gradient norm near the surface. A mesh that is not closed gives wrong parity along the open lines."""
        V = np.asarray(V, dtype=np.float64).reshape(-1, 3)
        F = np.asarray(F, dtype=np.int64).reshape(-1, 3)
        origin, shape = cls._grid_for(V.min(0), V.max(0), cell, pad)
        big = np.full(shape, np.inf)
        votes = np.zeros(shape, dtype=np.int8)
        for axis in range(3):
            d, ins = _axis_scan(V, F, origin, cell, shape, axis)
            big = np.minimum(big, d)
            votes += ins.astype(np.int8)
        inside = votes >= 2
        raw = np.where(inside, -1.0, 1.0) * np.minimum(big, band * cell)
        g = np.sqrt(sum(np.square(x) for x in np.gradient(raw, cell)))
        near = np.abs(raw) < 2.0 * cell
        raw = np.where(near, raw / np.clip(g, 1.0, 2.0), raw)
        return cls(raw, origin, cell, band)

    # ------------------------------------------------------------ queries
    @property
    def shape(self) -> Tuple[int, int, int]:
        return tuple(self.sdf.shape)

    def _occ(self, a: np.ndarray) -> np.ndarray:
        return np.clip(0.5 - a / _F32(self.cell), 0.0, 1.0)

    def volume(self) -> float:
        return float(self._occ(self.sdf).sum(dtype=np.float64)) * self.cell ** 3

    def occupancy(self) -> np.ndarray:
        return self.sdf < 0

    def section(self, axis: int, value: float) -> np.ndarray:
        """Boolean image of the material on the plane axis = value (linear blend of the two nearest slices);
        the image axes are the remaining two axes in increasing order."""
        f = (value - self.origin[axis]) / self.cell
        n = self.sdf.shape[axis]
        f = min(max(f, 0.0), n - 1.0)
        i0 = int(math.floor(f))
        i1 = min(i0 + 1, n - 1)
        w = f - i0
        a = np.take(self.sdf, i0, axis=axis)
        b = np.take(self.sdf, i1, axis=axis)
        return ((1 - w) * a + w * b) < 0

    # ------------------------------------------------------------ cutting core
    def _cut(self, lo, hi, fn: Callable) -> float:
        """Apply sdf = max(sdf, -fn(x, y, z)) on the grid cells in [lo, hi] grown by the band; return the volume
        removed. fn receives broadcastable 1D coordinate arrays (nx,1,1), (1,ny,1), (1,1,nz)."""
        m = (self.band + 1.0) * self.cell
        i0 = np.maximum(np.floor((lo - m - self.origin) / self.cell).astype(int), 0)
        i1 = np.minimum(np.ceil((hi + m - self.origin) / self.cell).astype(int) + 1, self.shape)
        if np.any(i1 <= i0):
            return 0.0
        sl = (slice(i0[0], i1[0]), slice(i0[1], i1[1]), slice(i0[2], i1[2]))
        sub = self.sdf[sl]
        xs = self.origin[0] + self.cell * np.arange(i0[0], i1[0])
        ys = self.origin[1] + self.cell * np.arange(i0[1], i1[1])
        zs = self.origin[2] + self.cell * np.arange(i0[2], i1[2])
        t = fn(xs[:, None, None], ys[None, :, None], zs[None, None, :])
        new = np.maximum(sub, -t.astype(_F32, copy=False))
        removed = (self._occ(sub).sum(dtype=np.float64) - self._occ(new).sum(dtype=np.float64)) * self.cell ** 3
        self.sdf[sl] = new
        return float(removed)

    def _grid_box(self):
        return self.origin, self.origin + self.cell * (np.array(self.shape) - 1)

    def _n_samples(self, dist: float) -> int:
        return max(int(math.ceil(dist / (0.5 * self.cell))), 1) + 1

    # ------------------------------------------------------------ tools
    def cut_ball(self, p0, p1, r: float) -> float:
        """Ball-end sweep: exact capsule between the two tip centres. Returns the removed volume."""
        p0, p1 = _v3(p0), _v3(p1)
        glo, ghi = self._grid_box()
        pieces = max(int(math.ceil(float(np.linalg.norm(p1 - p0)) / (12 * self.cell))), 1)
        total = 0.0
        for i in range(pieces):
            a = p0 + (p1 - p0) * (i / pieces)
            b = p0 + (p1 - p0) * ((i + 1) / pieces)
            seg = _clip_segment(a, b, glo - r, ghi + r)
            if seg is None:
                continue
            a, b = seg
            total += self._cut(np.minimum(a, b) - r, np.maximum(a, b) + r,
                               lambda x, y, z, a=a, b=b: _capsule(x, y, z, a, b, r))
        return total

    def _sweep_round_cyl(self, tips, axes, hc, hl, rr, c) -> float:
        """Apply a rounded-cylinder tool at each sampled pose (tip, unit axis). Tool origin = tip; the body is
        centred at height hc along its axis with half-length hl and radius rr."""
        total = 0.0
        for tip, ax in zip(tips, axes):
            e = np.sqrt(np.maximum(1.0 - ax * ax, 0.0)) * rr
            c0, c1 = tip + ax * (hc - hl), tip + ax * (hc + hl)
            lo, hi = np.minimum(c0, c1) - e, np.maximum(c0, c1) + e
            total += self._cut(lo, hi, lambda x, y, z, tip=tip, ax=ax: _round_cyl(
                x - tip[0], y - tip[1], z - tip[2], ax, hc, hl, rr, c))
        return total

    @staticmethod
    def _blend_axes(a0, a1, n):
        if a1 is None:
            return np.tile(_unit(a0), (n, 1))
        t = np.linspace(0.0, 1.0, n)[:, None]
        v = (1 - t) * _unit(a0) + t * _unit(a1)
        return v / np.linalg.norm(v, axis=1, keepdims=True)

    def cut_cylinder(self, p0, p1, axis, r: float, length: float, corner_r: float = 0.0, axis1=None) -> float:
        """Flat (corner_r = 0) or bull-nose end mill. The tip moves p0 -> p1 and the body extends from the tip along
        +axis for `length`. Poses are sampled along the segment at spacing <= cell/2 (approximation). With `axis1`
        the axis is interpolated (and renormalised) from `axis` to `axis1` along the move (5-axis)."""
        p0, p1 = _v3(p0), _v3(p1)
        n = self._n_samples(float(np.linalg.norm(p1 - p0)))
        tips = p0 + np.linspace(0, 1, n)[:, None] * (p1 - p0)
        axes = self._blend_axes(axis, axis1, n)
        c = min(max(float(corner_r), 0.0), r, length / 2)
        return self._sweep_round_cyl(tips, axes, length / 2, length / 2, r, c)

    def cut_disc(self, center0, center1, normal, R: float, kerf: float, normal1=None) -> float:
        """Saw blade: a thin cylinder of radius R and thickness kerf (axis = normal) whose centre moves
        center0 -> center1. Sampled along the move at spacing <= cell/2 (approximation)."""
        c0, c1 = _v3(center0), _v3(center1)
        n = self._n_samples(float(np.linalg.norm(c1 - c0)))
        cs = c0 + np.linspace(0, 1, n)[:, None] * (c1 - c0)
        axes = self._blend_axes(normal, normal1, n)
        return self._sweep_round_cyl(cs, axes, 0.0, kerf / 2, R, 0.0)

    def cut_wire(self, a0, b0, a1, b1, r: float) -> float:
        """Wire of radius r, a segment moving from (a0, b0) to (a1, b1) by linear interpolation of both ends.
        Each sampled position is cut as capsules of pieces of at most 16 cells along the wire; samples are spaced
        <= cell/2 by the larger end displacement (approximation). Pieces outside the grid are skipped."""
        a0, b0, a1, b1 = _v3(a0), _v3(b0), _v3(a1), _v3(b1)
        n = self._n_samples(max(float(np.linalg.norm(a1 - a0)), float(np.linalg.norm(b1 - b0))))
        glo, ghi = self._grid_box()
        total = 0.0
        for s in np.linspace(0.0, 1.0, n):
            a, b = a0 + s * (a1 - a0), b0 + s * (b1 - b0)
            seg = _clip_segment(a, b, glo - r, ghi + r)
            if seg is None:
                continue
            a, b = seg
            k = max(int(math.ceil(float(np.linalg.norm(b - a)) / (16 * self.cell))), 1)
            for i in range(k):
                pa = a + (b - a) * (i / k)
                pb = a + (b - a) * ((i + 1) / k)
                total += self._cut(np.minimum(pa, pb) - r, np.maximum(pa, pb) + r,
                                   lambda x, y, z, pa=pa, pb=pb: _capsule(x, y, z, pa, pb, r))
        return total

    def cut_path(self, kind: str, poses, active: Optional[Sequence[bool]] = None, **dims) -> float:
        """Cut along consecutive poses; returns the total removed volume.
        kind "ball":     poses (N, 3) tip centres; dims r.
        kind "cylinder": poses (N, 3) tips; dims axis (3,) or axes (N, 3), r, length, corner_r=0.
        kind "disc":     poses (N, 3) centres; dims normal (3,) or normals (N, 3), R, kerf.
        kind "wire":     poses (N, 2, 3) wire end pairs; dims r.
        `active` is an optional bool per move (N-1): false moves (rapids) are skipped."""
        P = np.asarray(poses, dtype=np.float64)
        total = 0.0
        for i in range(len(P) - 1):
            if active is not None and not active[i]:
                continue
            if kind == "ball":
                total += self.cut_ball(P[i], P[i + 1], dims["r"])
            elif kind == "cylinder":
                ax = dims["axes"][i] if "axes" in dims else dims["axis"]
                ax1 = dims["axes"][i + 1] if "axes" in dims else None
                total += self.cut_cylinder(P[i], P[i + 1], ax, dims["r"], dims["length"],
                                           dims.get("corner_r", 0.0), ax1)
            elif kind == "disc":
                nm = dims["normals"][i] if "normals" in dims else dims["normal"]
                nm1 = dims["normals"][i + 1] if "normals" in dims else None
                total += self.cut_disc(P[i], P[i + 1], nm, dims["R"], dims["kerf"], nm1)
            elif kind == "wire":
                total += self.cut_wire(P[i, 0], P[i, 1], P[i + 1, 0], P[i + 1, 1], dims["r"])
            else:
                raise ValueError("unknown tool kind: {}".format(kind))
        return total

    # ------------------------------------------------------------ surface
    def mesh(self) -> Tuple[np.ndarray, np.ndarray]:
        """Surface nets: one vertex per grid cell the surface crosses, at the mean of that cell's edge zero
        crossings; one quad (two triangles) per sign-changing grid edge. Normals point out of the material.
        Watertight when the level set lies inside the padded grid."""
        s = self.sdf
        neg = s < 0
        nx, ny, nz = s.shape
        if min(s.shape) < 2:
            return np.zeros((0, 3)), np.zeros((0, 3), dtype=np.int64)
        cnt = np.zeros((nx - 1, ny - 1, nz - 1), dtype=np.uint8)
        for dx in (0, 1):
            for dy in (0, 1):
                for dz in (0, 1):
                    cnt += neg[dx:nx - 1 + dx, dy:ny - 1 + dy, dz:nz - 1 + dz]
        mixed = (cnt > 0) & (cnt < 8)
        ci = np.nonzero(mixed)
        M = len(ci[0])
        if M == 0:
            return np.zeros((0, 3)), np.zeros((0, 3), dtype=np.int64)
        vid = np.full(mixed.shape, -1, dtype=np.int64)
        vid[mixed] = np.arange(M)
        corners = [(x, y, z) for x in (0, 1) for y in (0, 1) for z in (0, 1)]
        edges = [(a, b) for a in corners for b in corners
                 if a < b and sum(abs(a[k] - b[k]) for k in range(3)) == 1]
        acc = np.zeros((M, 3))
        num = np.zeros(M)
        for oa, ob in edges:
            sa = s[ci[0] + oa[0], ci[1] + oa[1], ci[2] + oa[2]].astype(np.float64)
            sb = s[ci[0] + ob[0], ci[1] + ob[1], ci[2] + ob[2]].astype(np.float64)
            cr = (sa < 0) != (sb < 0)
            t = np.where(cr, sa / np.where(cr, sa - sb, 1.0), 0.0)
            for k in range(3):
                acc[:, k] += np.where(cr, oa[k] + t * (ob[k] - oa[k]), 0.0)
            num += cr
        V = self.origin + self.cell * (np.stack(ci, axis=1) + acc / num[:, None])
        tris = []
        for a in range(3):
            b, c = (a + 1) % 3, (a + 2) % 3
            lo_sl = [slice(None)] * 3
            hi_sl = [slice(None)] * 3
            lo_sl[a], hi_sl[a] = slice(0, s.shape[a] - 1), slice(1, s.shape[a])
            n0, n1 = neg[tuple(lo_sl)], neg[tuple(hi_sl)]
            ei = list(np.nonzero(n0 != n1))
            keep = (ei[b] >= 1) & (ei[b] <= s.shape[b] - 2) & (ei[c] >= 1) & (ei[c] <= s.shape[c] - 2)
            ei = [x[keep] for x in ei]
            low_in = n0[tuple(ei)]
            quad = []
            for db, dc in ((-1, -1), (0, -1), (0, 0), (-1, 0)):
                idx = [None, None, None]
                idx[a], idx[b], idx[c] = ei[a], ei[b] + db, ei[c] + dc
                quad.append(vid[tuple(idx)])
            q = np.stack(quad, axis=1)
            q = np.where(low_in[:, None], q, q[:, [0, 3, 2, 1]])
            tris.append(q[:, [0, 1, 2]])
            tris.append(q[:, [0, 2, 3]])
        F = np.concatenate(tris, axis=0)
        return V, F

    def write_stl(self, path: str) -> int:
        """Binary STL of mesh(); returns the triangle count."""
        V, F = self.mesh()
        T = V[F]
        nrm = np.cross(T[:, 1] - T[:, 0], T[:, 2] - T[:, 0])
        ln = np.linalg.norm(nrm, axis=1, keepdims=True)
        nrm = np.where(ln > 0, nrm / np.where(ln > 0, ln, 1), 0)
        rec = np.zeros(len(F), dtype=[("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")])
        rec["n"], rec["v"] = nrm, T
        with open(path, "wb") as f:
            f.write(b"lithocam sdfstock".ljust(80, b" "))
            f.write(np.uint32(len(F)).tobytes())
            f.write(rec.tobytes())
        return int(len(F))


# ---------------------------------------------------------------- tri-dexel scan of a closed mesh

def _axis_scan(V, F, origin, cell, shape, axis):
    """Crossings of all grid lines parallel to `axis` with the triangles. Returns (distance to the nearest
    crossing along the line, inf where the line has none; inside flag by crossing parity), both in grid layout.
    Lines are shifted by a tiny fixed offset so that axis-aligned meshes never hit an edge exactly."""
    u, v = (axis + 1) % 3, (axis + 2) % 3
    nu, nv, na = shape[u], shape[v], shape[axis]
    A = V[F]
    pu = (A[..., u] - origin[u]) / cell - 1.7e-3
    pv = (A[..., v] - origin[v]) / cell - 2.9e-3
    pz = (A[..., axis] - origin[axis]) / cell
    i0 = np.maximum(np.ceil(pu.min(1)).astype(np.int64), 0)
    i1 = np.minimum(np.floor(pu.max(1)).astype(np.int64), nu - 1)
    j0 = np.maximum(np.ceil(pv.min(1)).astype(np.int64), 0)
    j1 = np.minimum(np.floor(pv.max(1)).astype(np.int64), nv - 1)
    wi, wj = np.maximum(i1 - i0 + 1, 0), np.maximum(j1 - j0 + 1, 0)
    cnt = wi * wj
    tri = np.nonzero(cnt > 0)[0]
    lids, ts = [], []
    if len(tri):
        csum = np.cumsum(cnt[tri])
        start = 0
        while start < len(tri):
            base = csum[start - 1] if start else 0
            stop = int(np.searchsorted(csum, base + 4_000_000, side="right"))
            stop = max(stop, start + 1)
            tt = tri[start:stop]
            c = cnt[tt]
            tot = int(c.sum())
            rep = np.repeat(np.arange(len(tt)), c)
            loc = np.arange(tot) - np.repeat(np.cumsum(c) - c, c)
            w = wj[tt][rep]
            ii = i0[tt][rep] + loc // w
            jj = j0[tt][rep] + loc % w
            a0, a1, a2 = (pu[tt, 0][rep], pv[tt, 0][rep]), (pu[tt, 1][rep], pv[tt, 1][rep]), (pu[tt, 2][rep], pv[tt, 2][rep])
            det = (a1[0] - a0[0]) * (a2[1] - a0[1]) - (a1[1] - a0[1]) * (a2[0] - a0[0])
            ok = np.abs(det) > 1e-14
            det = np.where(ok, det, 1.0)
            w0 = ((a1[0] - ii) * (a2[1] - jj) - (a1[1] - jj) * (a2[0] - ii)) / det
            w1 = ((a2[0] - ii) * (a0[1] - jj) - (a2[1] - jj) * (a0[0] - ii)) / det
            w2 = 1.0 - w0 - w1
            ok &= (w0 >= 0) & (w1 >= 0) & (w2 >= 0)
            z = w0 * pz[tt, 0][rep] + w1 * pz[tt, 1][rep] + w2 * pz[tt, 2][rep]
            lids.append((ii * nv + jj)[ok])
            ts.append(z[ok])
            start = stop
    lid = np.concatenate(lids) if lids else np.zeros(0, dtype=np.int64)
    t = np.concatenate(ts) if ts else np.zeros(0)
    K = na + 8.0
    key = lid * K + np.clip(t, -3.5, na + 2.5) + 4.0
    key.sort()
    L = nu * nv
    line_start = np.searchsorted(key, np.arange(L) * K)
    line_end = np.searchsorted(key, (np.arange(L) + 1) * K)
    sk = (np.arange(L)[:, None] * K + np.arange(na)[None, :] + 4.0).ravel()
    idx = np.searchsorted(key, sk)
    line = np.repeat(np.arange(L), na)
    inside = ((idx - line_start[line]) & 1).astype(bool)
    d = np.full(len(sk), np.inf)
    if len(key):
        lo_ok = idx - 1 >= line_start[line]
        hi_ok = idx < line_end[line]
        dl = np.where(lo_ok, sk - key[np.clip(idx - 1, 0, len(key) - 1)], np.inf)
        dh = np.where(hi_ok, key[np.clip(idx, 0, len(key) - 1)] - sk, np.inf)
        d = np.minimum(dl, dh)
    perm = [u, v, axis]
    back = np.argsort(perm)
    d = (d * cell).reshape(nu, nv, na).transpose(back)
    inside = inside.reshape(nu, nv, na).transpose(back)
    return d, inside
