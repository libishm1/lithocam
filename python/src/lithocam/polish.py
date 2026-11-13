# Copyright 2026 The lithocam authors. SPDX-License-Identifier: Apache-2.0
"""Polishing: constant-contact pad paths, a Preston removal model and grit sequences. numpy only.

Units: millimetres, seconds inside the model; feed in mm/min and spindle speed in rev/min at the API. Pressure is in
N/mm^2 (= MPa) and the Preston coefficient k_p in mm^2/N, so k_p * p * v has the unit of a speed (mm/s).

No process constant is built in. Preston's coefficient, pad pressure, pad speed and the grit values are parameters, or
come from `lithocam.cutting_data` rows (the only polishing row today is the granite pad rpm). Force control is
hardware-gated and dormant: the pressure and force on a move are simulation attributes, never a command.

Pad contact model
-----------------
The pad is a flat disc of radius Rp, its axis along the surface normal (tool axis = surface normal, 5-axis). The
tool tip is the contact point on the surface. The pad is taken as compliant: it presses with the uniform pressure p
on every surface point within Rp of the contact point (3D distance). A rigid flat pad on a curved surface touches
only at its centre; the compliant model is the usual one for foam backed diamond pads and it is the stated limit
of every number here.

Preston's law
-------------
    dh/dt = k_p * p * v

with v the relative speed between pad and surface at the point. In the tangent frame of the pad (x along the feed,
y = axis x x, rotation about the axis at omega, feed speed f) a surface point at offset (x, y) from the contact point
sees the pad surface move with

    v = ( (f - omega*y)^2 + (omega*x)^2 )^(1/2)

The depth removed at a surface point is the sum over the path of k_p * p * v * dt, dt = ds / f for the path element
ds while the point lies inside the footprint. The dwell map is the same sum with k_p * p * v replaced by 1 (seconds
under the pad).

Uniform removal
---------------
Zig-zag lanes of pitch s with alternating direction. A single pass leaves the lateral profile w(y) (integral of
k_p p v dt over the chord of the footprint at lateral offset y; mirrored on the reverse pass because the pad frame
flips). The lane sum W(y) = sum_n w(+-(y - n s)) is periodic with period 2 s. The ripple

    (max W - min W) / mean W

is evaluated over one period. `choose_stepover` returns the largest s (at most 2 Rp) whose ripple is at most the
tolerance. The feed then scales the whole map: mean depth = k_p p v_mean A / (s f), A = pi Rp^2 and v_mean the mean
of v over the footprint, which `feed_for_depth` solves for a target depth. Ripple does not depend on k_p or p.

Move contract (same as studio.cam)
----------------------------------
A move is (kind, point, axis[, attrs]) with kind rapid, plunge or feed, point the tool tip in mm and axis the unit tool
axis (tip to holder). The 4th element is a dict on plunge and feed moves:
    force_n        pad force = pressure * pi * Rp^2, N (simulation attribute)
    pressure_mpa   pad pressure, N/mm^2
    grit           grit label of the step
    rpm            pad spindle speed, rev/min
    feed_mm_min    feed
    water          water setting text
program.py and post.py should pass a 4th element through unchanged, emit S from rpm and F from feed_mm_min,
and treat force_n and pressure_mpa as comments or a side file for simulation; no force command is sent while
hardware is dormant.
"""
from __future__ import annotations

import math
import re
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Sequence, Tuple

import numpy as np

from . import cutting_data

ENTRY_HEIGHT = 2.0    # mm above the pass start where the plunge begins (the same value studio.cam uses)
Move = tuple


# ----------------------------------------------------------------------------------------------------------------
# surface access: vertical (or any `up`) projection onto a triangle patch, smooth normals
# ----------------------------------------------------------------------------------------------------------------
def _unit(v):
    v = np.asarray(v, float)
    n = np.linalg.norm(v)
    if n < 1e-15:
        raise ValueError("zero vector")
    return v / n


def vertex_normals(tris: np.ndarray) -> np.ndarray:
    """Area weighted vertex normals (n, 3, 3) of a triangle soup, welded by position (1e-6 mm)."""
    tris = np.asarray(tris, float).reshape(-1, 3, 3)
    V = tris.reshape(-1, 3)
    _, inv = np.unique(np.round(V, 6), axis=0, return_inverse=True)
    inv = inv.reshape(-1)
    fn = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])      # length = 2 * area
    acc = np.zeros((inv.max() + 1, 3))
    for k in range(3):
        np.add.at(acc, inv.reshape(-1, 3)[:, k], fn)
    nn = np.linalg.norm(acc, axis=1, keepdims=True)
    acc = acc / np.maximum(nn, 1e-300)
    return acc[inv].reshape(-1, 3, 3)


class Surface:
    """A triangle patch seen along `up`: `project` returns the highest surface point and its smooth normal."""

    def __init__(self, tris, up=(0.0, 0.0, 1.0)):
        self.tris = np.asarray(tris, float).reshape(-1, 3, 3)
        self.up = _unit(up)
        helper = np.array([1.0, 0, 0]) if abs(self.up[0]) < 0.9 else np.array([0.0, 1, 0])
        self.e1 = _unit(helper - self.up * (helper @ self.up))
        self.e2 = np.cross(self.up, self.e1)
        self.vn = vertex_normals(self.tris)
        self.uv = np.stack([self.tris @ self.e1, self.tris @ self.e2], axis=-1)       # (T, 3, 2)
        self.lo, self.hi = self.uv.min(axis=1), self.uv.max(axis=1)
        self.h = self.tris @ self.up

    def to_uv(self, pts):
        pts = np.asarray(pts, float)
        return np.stack([pts @ self.e1, pts @ self.e2], axis=-1)

    def from_uv(self, uv, h=0.0):
        uv = np.asarray(uv, float)
        return uv[..., 0:1] * self.e1 + uv[..., 1:2] * self.e2 + np.asarray(h)[..., None] * self.up

    def project(self, uv, chunk: int = 256):
        """(points (m,3), normals (m,3), hit (m,) bool). Misses are nan with hit False."""
        uv = np.asarray(uv, float).reshape(-1, 2)
        m = len(uv)
        P = np.full((m, 3), np.nan)
        N = np.full((m, 3), np.nan)
        hit = np.zeros(m, bool)
        A, B, C = self.uv[:, 0], self.uv[:, 1], self.uv[:, 2]
        d0, d1 = B - A, C - A
        den = d0[:, 0] * d1[:, 1] - d0[:, 1] * d1[:, 0]
        ok_t = np.abs(den) > 1e-12                     # vertical triangles have no projection
        eps = 1e-9
        for s in range(0, m, chunk):
            q = uv[s:s + chunk]
            cand = ((q[:, None, 0] >= self.lo[None, :, 0] - eps) & (q[:, None, 0] <= self.hi[None, :, 0] + eps) &
                    (q[:, None, 1] >= self.lo[None, :, 1] - eps) & (q[:, None, 1] <= self.hi[None, :, 1] + eps) &
                    ok_t[None, :])
            ip, it = np.nonzero(cand)
            if not len(ip):
                continue
            r = q[ip] - A[it]
            l1 = (r[:, 0] * d1[it, 1] - r[:, 1] * d1[it, 0]) / den[it]
            l2 = (d0[it, 0] * r[:, 1] - d0[it, 1] * r[:, 0]) / den[it]
            l0 = 1.0 - l1 - l2
            ins = (l0 >= -eps) & (l1 >= -eps) & (l2 >= -eps)
            ip, it, l0, l1, l2 = ip[ins], it[ins], l0[ins], l1[ins], l2[ins]
            if not len(ip):
                continue
            hh = l0 * self.h[it, 0] + l1 * self.h[it, 1] + l2 * self.h[it, 2]
            order = np.lexsort((-hh, ip))              # per point, highest first
            ip, it, l0, l1, l2, hh = ip[order], it[order], l0[order], l1[order], l2[order], hh[order]
            first = np.r_[True, ip[1:] != ip[:-1]]
            ip, it, l0, l1, l2, hh = ip[first], it[first], l0[first], l1[first], l2[first], hh[first]
            pts = (l0[:, None] * self.tris[it, 0] + l1[:, None] * self.tris[it, 1] + l2[:, None] * self.tris[it, 2])
            nrm = (l0[:, None] * self.vn[it, 0] + l1[:, None] * self.vn[it, 1] + l2[:, None] * self.vn[it, 2])
            nrm /= np.maximum(np.linalg.norm(nrm, axis=1, keepdims=True), 1e-300)
            nrm[nrm @ self.up < 0] *= -1.0             # the side the projection ray comes from
            P[s + ip], N[s + ip], hit[s + ip] = pts, nrm, True
        return P, N, hit


def surface_samples(tris, n: int, seed: int = 0):
    """n area weighted random points on the triangles and their smooth normals: (points (n,3), normals (n,3))."""
    tris = np.asarray(tris, float).reshape(-1, 3, 3)
    vn = vertex_normals(tris)
    area = 0.5 * np.linalg.norm(np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0]), axis=1)
    rng = np.random.default_rng(seed)
    idx = rng.choice(len(tris), size=n, p=area / area.sum())
    r1, r2 = rng.random(n), rng.random(n)
    f = r1 + r2 > 1.0
    r1[f], r2[f] = 1.0 - r1[f], 1.0 - r2[f]
    w = np.stack([1.0 - r1 - r2, r1, r2], axis=1)
    P = np.einsum("nk,nkj->nj", w, tris[idx])
    N = np.einsum("nk,nkj->nj", w, vn[idx])
    N /= np.linalg.norm(N, axis=1, keepdims=True)
    return P, N


# ----------------------------------------------------------------------------------------------------------------
# paths
# ----------------------------------------------------------------------------------------------------------------
def stepover_from_overlap(pad_radius: float, overlap: float) -> float:
    """Lane pitch from the pad radius and an overlap ratio: s = 2 Rp (1 - overlap), overlap in [0, 1)."""
    if not 0.0 <= overlap < 1.0:
        raise ValueError("overlap must be in [0, 1)")
    return 2.0 * pad_radius * (1.0 - overlap)


def _split_hits(P, N, hit):
    """Runs of consecutive hits as (points, normals) pairs."""
    out, i, n = [], 0, len(hit)
    while i < n:
        if not hit[i]:
            i += 1
            continue
        j = i
        while j < n and hit[j]:
            j += 1
        out.append((P[i:j], N[i:j]))
        i = j
    return out


def surface_passes(tris, *, pad_radius: float, stepover: float = 0.0, overlap: float = 0.5, step: float = 0.0,
                   pattern: str = "zigzag", up=(0.0, 0.0, 1.0), angle_deg: float = 0.0,
                   region=None) -> List[Tuple[np.ndarray, np.ndarray]]:
    """Contact point passes over a patch: a list of (points (k,3), normals (k,3)).

    The patch must be single valued along `up` (a height field in that direction). Lanes are straight in the plane
    normal to `up`, then projected onto the surface, so `stepover` is the plan pitch (the pitch on the surface is
    larger by 1/cos(slope); lower the overlap ratio on steep patches, the coverage check reports the result).
    stepover 0 takes it from `overlap`. step is the plan point spacing (0 = Rp / 4). pattern is "zigzag"
    (lanes at angle_deg from the first plan axis, alternating direction; the pitch is reduced so that the lanes
    divide the extent evenly) or "spiral" (Archimedean, outward from the plan centre, radial pitch = stepover).
    region is None (the whole patch), a plan box (umin, umax, vmin, vmax) in the surface frame, or a callable
    f(points (m,3)) -> bool mask keeping the contact points."""
    Rp = float(pad_radius)
    if Rp <= 0:
        raise ValueError("pad_radius must be positive")
    s = float(stepover) if stepover and stepover > 0 else stepover_from_overlap(Rp, overlap)
    ds = float(step) if step and step > 0 else Rp / 4.0
    surf = Surface(tris, up)
    lo, hi = surf.uv.reshape(-1, 2).min(axis=0), surf.uv.reshape(-1, 2).max(axis=0)
    if isinstance(region, (tuple, list)) and len(region) == 4:
        lo = np.maximum(lo, [region[0], region[2]])
        hi = np.minimum(hi, [region[1], region[3]])
    ca, sa = math.cos(math.radians(angle_deg)), math.sin(math.radians(angle_deg))
    ax = np.array([ca, sa])
    ay = np.array([-sa, ca])
    passes = []
    if pattern == "zigzag":
        corners = np.array([[lo[0], lo[1]], [hi[0], lo[1]], [hi[0], hi[1]], [lo[0], hi[1]]])
        a, b = corners @ ax, corners @ ay
        n_lane = max(2, int(math.ceil((b.max() - b.min()) / s)) + 1)
        for k, v in enumerate(np.linspace(b.min(), b.max(), n_lane)):
            nu = max(2, int(math.ceil((a.max() - a.min()) / ds)) + 1)
            u = np.linspace(a.min(), a.max(), nu)
            uv = u[:, None] * ax + v * ay
            inside = np.all((uv >= lo - 1e-9) & (uv <= hi + 1e-9), axis=1)
            if not inside.any():
                continue
            P, N, hit = surf.project(uv[inside])
            runs = _split_hits(P, N, hit)
            if k % 2:
                runs = [(p[::-1], q[::-1]) for p, q in reversed(runs)]
            passes += runs
    elif pattern == "spiral":
        c = 0.5 * (lo + hi)
        rmax = 0.5 * float(np.linalg.norm(hi - lo))
        # arc length parametrisation of r = s * theta / (2 pi)
        pitch = s / (2.0 * math.pi)
        theta, th = [0.0], 0.0
        while pitch * th < rmax:
            th += 0.5 * ds / max(math.hypot(pitch * th, pitch), 1e-9)
            theta.append(th)
        theta = np.array(theta)
        r = pitch * theta
        uv = c + np.stack([r * np.cos(theta), r * np.sin(theta)], axis=1)
        P, N, hit = surf.project(uv)
        passes = _split_hits(P, N, hit)
    else:
        raise ValueError("pattern must be 'zigzag' or 'spiral'")
    if callable(region):
        kept = []
        for P, N in passes:
            mk = np.asarray(region(P), bool)
            kept += _split_hits(P, N, mk)
        passes = kept
    return [(P, N) for P, N in passes if len(P) >= 1]


def passes_to_moves(passes, *, clear_z: float, attrs: Optional[dict] = None) -> List[Move]:
    """Moves for the passes: rapid over the start, rapid down to ENTRY_HEIGHT along the axis, plunge, feed, rapid up.
    `attrs` (copied per move) is the 4th element of every plunge and feed move."""
    zax = np.array([0.0, 0.0, 1.0])
    moves: List[Move] = []
    for P, N in passes:
        moves.append(("rapid", np.array([P[0, 0], P[0, 1], clear_z]), zax))
        moves.append(("rapid", P[0] + N[0] * ENTRY_HEIGHT, N[0]))
        moves.append(("plunge", P[0].copy(), N[0], dict(attrs or {})))
        for p, n in zip(P[1:], N[1:]):
            moves.append(("feed", p.copy(), n, dict(attrs or {})))
        moves.append(("rapid", np.array([P[-1, 0], P[-1, 1], clear_z]), zax))
    return moves


def _pad_radius(tool, pad_radius):
    if pad_radius:
        return float(pad_radius)
    name = getattr(getattr(tool, 'kind', None), 'name', None)
    if name is not None and name != 'PolishingPad':
        raise ValueError('polishing needs a PolishingPad tool (got {})'.format(name))
    return float(tool.diameter) / 2.0


def _attrs(Rp, grit=None, pressure=None, rpm=None, feed=None, water=None):
    a = {}
    if grit is not None:
        a["grit"] = grit
    if pressure is not None:
        a["pressure_mpa"] = float(pressure)
        a["force_n"] = float(pressure) * math.pi * Rp * Rp
    if rpm is not None:
        a["rpm"] = float(rpm)
    if feed is not None:
        a["feed_mm_min"] = float(feed)
    if water is not None:
        a["water"] = water
    return a


def polish(tris, mesh, tool, stock, *, stepover=0.0, overlap=0.5, step=0.0, pattern="zigzag", angle_deg=0.0,
           pressure=None, rpm=None, feed=None, grit=None, water=None, clearance=20.0, up=(0.0, 0.0, 1.0),
           region=None, pad_radius=0.0, **_) -> List[Move]:
    """One polishing pass over the patch (cam.py contract: f(tris, mesh, tool, stock, **params) -> moves).

    The pad stays on the surface with its axis along the surface normal. pressure (N/mm^2), rpm, feed (mm/min),
    grit and water go into the 4th element of the plunge and feed moves when given; no value is assumed. `mesh` is
    not used (the patch is read from `tris`)."""
    Rp = _pad_radius(tool, pad_radius)
    tris = np.asarray(tris, float).reshape(-1, 3, 3)
    passes = surface_passes(tris, pad_radius=Rp, stepover=stepover, overlap=overlap, step=step, pattern=pattern,
                            up=up, angle_deg=angle_deg, region=region)
    zmax = float(stock["zmax"]) if stock is not None else float(tris[..., 2].max())
    return passes_to_moves(passes, clear_z=zmax + clearance, attrs=_attrs(Rp, grit, pressure, rpm, feed, water))


# ----------------------------------------------------------------------------------------------------------------
# removal model
# ----------------------------------------------------------------------------------------------------------------
def _feed_segments(moves):
    """(start, end, axis0, axis1, attrs) of every feed move; the start is the previous plunge or feed point."""
    out, prev, prev_ax = [], None, None
    for m in moves:
        kind, pt = m[0], np.asarray(m[1], float)
        ax = np.asarray(m[2], float) if len(m) > 2 and m[2] is not None else np.array([0.0, 0.0, 1.0])
        at = m[3] if len(m) > 3 else {}
        if kind == "feed" and prev is not None:
            out.append((prev, pt, prev_ax, ax, at))
        prev, prev_ax = (pt, ax) if kind in ("plunge", "feed") else (None, None)
    return out


class _Grid:
    def __init__(self, pts, cell):
        self.cell = cell
        self.pts = pts
        key = np.floor(pts / cell).astype(np.int64)
        self.d: Dict[tuple, List[int]] = {}
        for i, k in enumerate(map(tuple, key)):
            self.d.setdefault(k, []).append(i)
        self.d = {k: np.array(v) for k, v in self.d.items()}

    def near(self, c, r):
        k0 = np.floor((c - r) / self.cell).astype(int)
        k1 = np.floor((c + r) / self.cell).astype(int)
        out = []
        for i in range(k0[0], k1[0] + 1):
            for j in range(k0[1], k1[1] + 1):
                for k in range(k0[2], k1[2] + 1):
                    v = self.d.get((i, j, k))
                    if v is not None:
                        out.append(v)
        if not out:
            return np.zeros(0, int)
        idx = np.concatenate(out)
        keep = np.linalg.norm(self.pts[idx] - c, axis=1) <= r
        return idx[keep]


def _accumulate(moves, samples, *, pad_radius, weight, pressure, rpm, feed, rel_speed, ds, rotation):
    """Sum over the path of weight(p, v) * dt at every sample. weight is a function of (pressure, v)."""
    S = np.asarray(samples, float).reshape(-1, 3)
    out = np.zeros(len(S))
    Rp = float(pad_radius)
    grid = _Grid(S, Rp)
    ds = float(ds) if ds else Rp / 50.0
    for a, b, ax0, ax1, at in _feed_segments(moves):
        L = float(np.linalg.norm(b - a))
        if L < 1e-12:
            continue
        p = at.get("pressure_mpa", pressure)
        rp = at.get("rpm", rpm)
        fd = at.get("feed_mm_min", feed)
        if p is None or fd is None or (rel_speed is None and rp is None):
            raise ValueError("pressure, feed and rpm (or rel_speed) must be given as arguments or on the moves")
        f = float(fd) / 60.0
        om = float(rotation) * 2.0 * math.pi * float(rp) / 60.0 if rp is not None else 0.0
        n = max(1, int(math.ceil(L / ds)))
        t = (np.arange(n) + 0.5) / n
        tang = (b - a) / L
        dt = L / n / f
        for ti in t:
            c = a + ti * (b - a)
            axis = (1 - ti) * ax0 + ti * ax1
            axis /= np.linalg.norm(axis)
            idx = grid.near(c, Rp)
            if not len(idx):
                continue
            d = S[idx] - c
            tx = tang - axis * (tang @ axis)
            nx = np.linalg.norm(tx)
            tx = tx / nx if nx > 1e-12 else tang
            ty = np.cross(axis, tx)
            x, y = d @ tx, d @ ty
            if rel_speed is not None:
                v = np.full(len(idx), float(rel_speed))
            else:
                v = np.sqrt((f - om * y) ** 2 + (om * x) ** 2)
            out[idx] += weight(p, v) * dt
    return out


def predict_removal(moves, samples, *, pad_radius: float, k_p: float, pressure: Optional[float] = None,
                    rpm: Optional[float] = None, feed: Optional[float] = None, rel_speed: Optional[float] = None,
                    ds: float = 0.0, rotation: int = 1) -> np.ndarray:
    """Predicted removal depth (mm) at each sample point: sum of k_p * p * v * dt over the path.

    p, rpm and feed come from the arguments, or per move from the 4th element (pressure_mpa, rpm, feed_mm_min), which
    takes precedence. rel_speed (mm/s) replaces the pad speed model by a constant relative speed (orbital pads,
    scaling tests). ds is the path discretisation (0 = Rp / 50); the footprint test uses the 3D distance to the
    contact point."""
    kp = float(k_p)
    return _accumulate(moves, samples, pad_radius=pad_radius, weight=lambda p, v: kp * p * v, pressure=pressure,
                       rpm=rpm, feed=feed, rel_speed=rel_speed, ds=ds, rotation=rotation)


def dwell_map(moves, samples, *, pad_radius: float, feed: Optional[float] = None, ds: float = 0.0) -> np.ndarray:
    """Seconds each sample lies under the pad footprint (feed in mm/min, or per move feed_mm_min)."""
    return _accumulate(moves, samples, pad_radius=pad_radius, weight=lambda p, v: np.ones_like(v),
                       pressure=1.0, rpm=0.0, feed=feed, rel_speed=1.0, ds=ds, rotation=1)


def predict_by_grit(steps: Sequence[dict], samples, *, pad_radius: float, k_p, pressure=None, ds: float = 0.0,
                    rotation: int = 1) -> Dict[str, np.ndarray]:
    """Removal depth per grit from the steps of `polish_steps`. k_p is a number or a dict grit -> coefficient
    (coefficients differ by grit and material; none is built in). Returns {grit label: depth array}."""
    out = {}
    for st in steps:
        g = str(st["grit"])
        kp = k_p[g] if isinstance(k_p, dict) else k_p
        out[g] = predict_removal(st["moves"], samples, pad_radius=pad_radius, k_p=kp, pressure=pressure, ds=ds,
                                 rotation=rotation)
    return out


def coverage(moves, samples, pad_radius: float) -> float:
    """Largest distance from a sample to its nearest contact point, as a fraction of the pad radius (<= 1 means every
    sample lies inside some footprint)."""
    pts = np.array([np.asarray(m[1], float) for m in moves if m[0] in ("plunge", "feed")])
    S = np.asarray(samples, float).reshape(-1, 3)
    worst = 0.0
    for s in range(0, len(S), 512):
        d = np.linalg.norm(S[s:s + 512, None, :] - pts[None, :, :], axis=2).min(axis=1)
        worst = max(worst, float(d.max()))
    return worst / float(pad_radius)


# ----------------------------------------------------------------------------------------------------------------
# uniform removal on a flat strip: stepover and feed
# ----------------------------------------------------------------------------------------------------------------
def lane_profile(y, pad_radius, rpm, feed, *, direction: int = 1, rotation: int = 1, nx: int = 400):
    """Removal per unit (k_p * p) at lateral offset y from the centre line of one straight pass: the integral of
    v dt across the footprint chord. direction -1 is the reverse pass (the pad frame is mirrored)."""
    y = np.atleast_1d(np.asarray(y, float)) * direction
    om = rotation * 2.0 * math.pi * rpm / 60.0
    f = feed / 60.0
    Rp = pad_radius
    half = np.sqrt(np.maximum(Rp * Rp - y * y, 0.0))
    u = (np.arange(nx) + 0.5) / nx * 2.0 - 1.0                      # (-1, 1)
    x = half[:, None] * u[None, :]
    v = np.sqrt((f - om * y[:, None]) ** 2 + (om * x) ** 2)
    return v.sum(axis=1) * (2.0 * half / nx) / f


def ripple(stepover, pad_radius, rpm, feed, *, alternate: bool = True, rotation: int = 1, ny: int = 200) -> float:
    """(max - min) / mean of the lane sum over one period for lanes of pitch `stepover`."""
    s = float(stepover)
    period = 2.0 * s if alternate else s
    y = (np.arange(ny) + 0.5) / ny * period
    nl = int(math.ceil(pad_radius / s)) + 2
    total = np.zeros(ny)
    for n in range(-nl, nl + 2 + int(alternate) * nl):
        d = -1 if (alternate and n % 2) else 1
        total += lane_profile(y - n * s, pad_radius, rpm, feed, direction=d, rotation=rotation)
    return float((total.max() - total.min()) / total.mean())


def choose_stepover(pad_radius: float, rpm: float, feed: float, tol: float, *, alternate: bool = True,
                    rotation: int = 1, n_trial: int = 200, min_ratio: float = 0.05, margin: float = 0.05
                    ) -> Tuple[float, float]:
    """Largest lane pitch s <= 2 Rp whose ripple is at most tol, returns (stepover, ripple at s).

    The ripple of a lane sum changes quickly with the pitch, so s must keep the ripple within tol over the
    whole band s (1 - margin) .. s (1 + margin): a pitch that is only good at one value is rejected, which
    leaves room for the slope (surface pitch = plan pitch / cos) and for rounding of the path. Trial pitches run
    from 2 Rp down to min_ratio * 2 Rp in n_trial steps; ValueError when none meets tol."""
    for r in np.linspace(1.0, min_ratio, n_trial):
        s = 2.0 * pad_radius * float(r)
        rp = ripple(s, pad_radius, rpm, feed, alternate=alternate, rotation=rotation)
        if rp > tol:
            continue
        if all(ripple(s * (1.0 + k * margin), pad_radius, rpm, feed, alternate=alternate, rotation=rotation) <= tol
               for k in (-1.0, -0.5, 0.5, 1.0)):
            return s, rp
    raise ValueError("no stepover down to {:.3g} Rp gives a ripple of {:.3g} or less".format(min_ratio * 2, tol))

def mean_footprint_speed(pad_radius: float, rpm: float, feed: float, rotation: int = 1, n_r: int = 60,
                         n_t: int = 90) -> float:
    """Mean of v over the pad footprint (mm/s), v = sqrt((f - omega y)^2 + (omega x)^2)."""
    om = rotation * 2.0 * math.pi * rpm / 60.0
    f = feed / 60.0
    r = (np.arange(n_r) + 0.5) / n_r * pad_radius
    t = (np.arange(n_t) + 0.5) / n_t * 2.0 * math.pi
    R, T = np.meshgrid(r, t, indexing="ij")
    v = np.sqrt((f - om * R * np.sin(T)) ** 2 + (om * R * np.cos(T)) ** 2)
    return float((v * R).sum() / R.sum())          # area weighted: dA = r dr dtheta


def feed_for_depth(depth: float, pad_radius: float, rpm: float, k_p: float, pressure: float, stepover: float,
                   rotation: int = 1, iters: int = 30) -> float:
    """Feed (mm/min) giving the mean depth `depth` on lanes of pitch `stepover`: depth = k_p p v_mean A / (s f)."""
    A = math.pi * pad_radius ** 2
    f = 1.0
    for _ in range(iters):
        v = mean_footprint_speed(pad_radius, rpm, f * 60.0, rotation)
        f = k_p * pressure * v * A / (stepover * depth)
    return f * 60.0


def plan_uniform(pad_radius: float, rpm: float, k_p: float, pressure: float, depth: float, tol: float, *,
                 rotation: int = 1) -> dict:
    """Stepover and feed for a mean removal `depth` with lane-sum ripple at most `tol`. The stepover search depends
    on the feed, the feed on the stepover, so the pair is iterated to a fixed point."""
    f = 600.0
    for _ in range(8):
        s, rp = choose_stepover(pad_radius, rpm, f, tol, rotation=rotation)
        f2 = feed_for_depth(depth, pad_radius, rpm, k_p, pressure, s, rotation)
        if abs(f2 - f) < 1e-6 * f:
            f = f2
            break
        f = f2
    s, rp = choose_stepover(pad_radius, rpm, f, tol, rotation=rotation)
    return {"stepover": s, "feed_mm_min": f, "ripple": rp,
            "overlap": 1.0 - s / (2.0 * pad_radius)}


# ----------------------------------------------------------------------------------------------------------------
# grit sequences
# ----------------------------------------------------------------------------------------------------------------
@dataclass
class GritStep:
    grit: str                     # label: a grit number as text, a band such as "50-100", or "buff"
    rpm: float
    feed: float                   # mm/min
    water: str
    k_p: Optional[float] = None   # mm^2/N, Preston coefficient for this grit when the user gives one
    source: str = ""              # where rpm came from


def _pad_rows(material: str, quantity: str):
    """polishing_pad rows of a material as cutting_data entries. The unified material model (lithocam.materials)
    resolves names and aliases when it is present; cutting_data is the fallback and the source of the rows."""
    try:
        from . import materials
        m = materials.get(material)
        return [cutting_data.Entry(r["material"], r["tool"], r["quantity"], r["low"], r["high"], r["unit"],
                                   r["source"], r.get("source_type", ""), "page", r.get("size", ""),
                                   r.get("note", "")) for r in m.grit_rows if r["quantity"] == quantity]
    except (ImportError, KeyError):
        return cutting_data.lookup(material, "polishing_pad", quantity)


def sourced_value(material: str, quantity: str) -> Optional[Tuple[float, str, str]]:
    """(low end value, unit, source) of a polishing_pad row (feed, water, ...) or None: no row, no value."""
    rows = _pad_rows(material, quantity)
    if not rows:
        return None
    e = min(rows, key=lambda r: r.lo)
    return e.lo, e.unit, e.source


def sourced_bands(material: str) -> List[dict]:
    """Ordered grit bands with a pad rpm band from the cutting_data polishing_pad rpm rows of `material`.

    The row note lists 'grit rpm-rpm' groups separated by semicolons; the groups keep their order (coarse to fine).
    Returns [{label, rpm_lo, rpm_hi, source}]; [] when the material has no such row or the note has no groups."""
    out = []
    for e in _pad_rows(material, "rpm"):
        for seg in e.note.split(";"):
            m = re.match(r"\s*(.+?)\s+(\d+)\s*-\s*(\d+)\s*$", seg)
            if not m:
                continue
            for lab in re.split(r"\s+and\s+", re.sub(r"\s*grit\s*", " ", m.group(1)).strip()):
                lab = lab.strip()
                if lab.lower().startswith("buff"):
                    lab = "buff"
                out.append({"label": lab, "rpm_lo": float(m.group(2)), "rpm_hi": float(m.group(3)),
                            "source": e.source})
    return out


def _per(val, label, what):
    if isinstance(val, dict):
        if label not in val:
            raise ValueError("{} has no value for grit '{}'".format(what, label))
        return val[label]
    if val is None:
        raise ValueError("{} must be given (no sourced value exists for this material)".format(what))
    return val


def grit_sequence(material: str, *, grits: Optional[Sequence] = None, feed=None, water=None, rpm=None, k_p=None,
                  rpm_pick: float = 0.5) -> List[GritStep]:
    """Ordered grit steps for a material.

    grits None: the sourced bands of the material (cutting_data), coarse to fine; only granite has them today.
    grits given: the user's order is kept; a numeric grit takes its rpm band from the sourced band that contains it,
    else rpm must be given. rpm_pick in [0, 1] places the rpm inside a sourced band (0 low, 1 high; 0.5 is the
    midpoint, a choice and not a sourced value). feed (mm/min), water, rpm and k_p are a scalar or a dict keyed by the
    grit label. Feed and water come from a sourced polishing_pad row of the material when one exists (none does
    today), else they must be given; a missing one raises ValueError.
    No grit sequence is invented: a material without sourced bands needs `grits`."""
    bands = sourced_bands(material)
    if grits is None:
        if not bands:
            raise ValueError("no sourced grit sequence for '{}': pass grits=[...]".format(material))
        labels = [b["label"] for b in bands]
    else:
        labels = [str(g) for g in grits]
    fd, wt = feed, water
    if fd is None:                                   # a sourced pad feed row, only in mm/min
        sv = sourced_value(material, "feed")
        fd = sv[0] if sv and sv[1] == "mm/min" else None
    if wt is None:
        sv = sourced_value(material, "water")
        wt = "{:g} {}".format(sv[0], sv[1]) if sv else None
    steps = []
    for lab in labels:
        band = next((b for b in bands if b["label"] == lab), None)
        if band is None:
            try:
                g = float(lab)
            except ValueError:
                g = None
            if g is not None:
                for b in bands:
                    m = re.match(r"^(\d+)(?:-(\d+))?$", b["label"])
                    if m and float(m.group(1)) <= g <= float(m.group(2) or m.group(1)):
                        band = b
                        break
        if rpm is not None:
            r, src = float(_per(rpm, lab, "rpm")), "user"
        elif band is not None:
            r = band["rpm_lo"] + float(rpm_pick) * (band["rpm_hi"] - band["rpm_lo"])
            src = band["source"]
        else:
            raise ValueError("no sourced rpm for grit '{}' of '{}': pass rpm".format(lab, material))
        kp = _per(k_p, lab, "k_p") if k_p is not None else None
        steps.append(GritStep(lab, r, float(_per(fd, lab, "feed")), str(_per(wt, lab, "water")), kp, src))
    return steps


def polish_steps(tris, mesh, tool, stock, *, steps: Sequence[GritStep], pressure, tools: Optional[dict] = None,
                 cross: bool = False, cross_angle_deg: float = 90.0, **path_params) -> List[dict]:
    """One polishing pass per grit, each its own step with a tool change (a new pad per grit).

    Returns [{index, grit, tool_change True, tool, moves, rpm, feed_mm_min, water, pressure_mpa, force_n}]. `tools`
    maps a grit label to its pad tool (default: `tool`). pressure is a scalar or a dict by grit (N/mm^2). With cross
    True every second grit runs its lanes cross_angle_deg from the previous one. path_params go to `polish`
    (stepover, overlap, step, pattern, up, region, clearance, ...). The caller inserts the tool change
    (M6 and the pad's tool number) before the step's moves."""
    out = []
    base_angle = float(path_params.pop("angle_deg", 0.0))
    for i, st in enumerate(steps):
        t = (tools or {}).get(st.grit, tool)
        p = pressure[st.grit] if isinstance(pressure, dict) else pressure
        if p is None:
            raise ValueError("pressure must be given for grit '{}'".format(st.grit))
        ang = base_angle + (cross_angle_deg if (cross and i % 2) else 0.0)
        mv = polish(tris, mesh, t, stock, pressure=p, rpm=st.rpm, feed=st.feed, grit=st.grit, water=st.water,
                    angle_deg=ang, **path_params)
        Rp = _pad_radius(t, path_params.get("pad_radius", 0.0))
        out.append({"index": i, "grit": st.grit, "tool_change": True, "tool": t, "moves": mv, "rpm": st.rpm,
                    "feed_mm_min": st.feed, "water": st.water, "pressure_mpa": float(p),
                    "force_n": float(p) * math.pi * Rp * Rp})
    return out
