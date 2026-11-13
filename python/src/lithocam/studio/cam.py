# Copyright 2026 The lithocam authors. SPDX-License-Identifier: Apache-2.0
"""Toolpath generators the studio runs today, on the C++ kernel (_lithocam): face, plane (z-level) roughing,
raster, waterline and steep / shallow finishing over a triangle mesh with any tool that has a drop-cutter model.

Units: millimetres, in the part frame (the work frame of the setup). A path is a list of moves; each move is
(kind, point[, axis]) with kind "rapid", "plunge" or "feed" and point the tool tip.

Allowance: the tip is lifted by the allowance. That is exact on horizontal floors and approximate on slopes; the
Z-map simulation shows the real leftover.
"""
from __future__ import annotations

import math
from pathlib import Path
from typing import Dict, List, Tuple

import numpy as np

from .core import lc

Move = Tuple[str, np.ndarray]


def load_stl(path: str | Path) -> np.ndarray:
    """Triangles (n, 3, 3) of an STL file, in the file's units (taken as mm)."""
    from lithocam.viz import read_stl
    return np.asarray(read_stl(str(path)), dtype=float).reshape(-1, 3, 3)


# ------------------------------------------------------------------------------------------------------------------
# Height fields: a relief stored as a regular grid of heights (examples/data/*.npz), turned into a closed slab.
#
# File format (.npz, numpy only): "heights" uint16 (rows, cols); row j is y = j * dy, column i is x = i * dx, row 0 at
# y = 0 (an image view needs a vertical flip). Height in mm = heights * z_scale_mm (z_scale_mm = relief_mm / 65535).
# Metadata: width_mm (x extent), height_mm (y extent), relief_mm, z_scale_mm, base_mm (slab under the relief),
# source, credit, license (strings). Only plain arrays are stored, so np.load(..., allow_pickle=False) reads it.

def _sampled(n: int, stride: int) -> np.ndarray:
    """Every stride-th index of 0..n-1, always with the last one (the grid keeps its full extent)."""
    idx = np.arange(0, n, max(int(stride), 1))
    return idx if idx[-1] == n - 1 else np.r_[idx, n - 1]


def heightfield_tris(H: np.ndarray, width_mm: float, depth_mm: float = 0.0, base_mm: float = 35.0,
                     stride: int = 1) -> np.ndarray:
    """A closed slab (n, 3, 3) from a height field: the relief top z = base_mm + H, four side walls and a flat
    bottom at z = 0. H (rows, cols) holds heights in mm over the rectangle [0, width_mm] x [0, depth_mm] (depth_mm
    is the plan extent along y; 0 = width_mm * (rows - 1) / (cols - 1), square pixels). stride > 1 keeps every
    stride-th row and column (and the last), a coarser part for planning. Every edge is shared by exactly two
    triangles and the normals point out."""
    H = np.asarray(H, dtype=float)
    if H.ndim != 2 or min(H.shape) < 2:
        raise ValueError("height field must be a 2D grid of at least 2 x 2 (got shape {})".format(H.shape))
    if not np.all(np.isfinite(H)):
        raise ValueError("height field has non-finite values")
    if base_mm <= 0:
        raise ValueError("base_mm must be > 0 (got {})".format(base_mm))
    ny0, nx0 = H.shape
    depth_mm = float(depth_mm) if depth_mm and depth_mm > 0 else float(width_mm) * (ny0 - 1) / (nx0 - 1)
    jj, ii = _sampled(ny0, stride), _sampled(nx0, stride)
    Z = float(base_mm) + H[np.ix_(jj, ii)]
    xs = ii * (float(width_mm) / (nx0 - 1))
    ys = jj * (depth_mm / (ny0 - 1))
    ny, nx = Z.shape
    X, Y = np.meshgrid(xs, ys)
    top = np.stack([X, Y, Z], axis=-1)                       # (ny, nx, 3)
    a, b = top[:-1, :-1], top[:-1, 1:]                       # quad corners: a (j, i), b (j, i+1)
    c, d = top[1:, 1:], top[1:, :-1]                         # c (j+1, i+1), d (j+1, i)
    up = np.concatenate([np.stack([a, b, c], -2).reshape(-1, 3, 3),     # counter-clockwise seen from +z
                         np.stack([a, c, d], -2).reshape(-1, 3, 3)])
    # the boundary of the top, counter-clockwise seen from +z: y = 0, x = max, y = max (back), x = 0 (back)
    ring = np.concatenate([top[0, :-1], top[:-1, -1], top[-1, :0:-1], top[:0:-1, 0]])
    nxt = np.roll(ring, -1, axis=0)
    lo_r, lo_n = ring.copy(), nxt.copy()
    lo_r[:, 2] = lo_n[:, 2] = 0.0
    walls = np.concatenate([np.stack([ring, lo_r, lo_n], 1), np.stack([ring, lo_n, nxt], 1)])
    centre = np.array([xs[-1] / 2, ys[-1] / 2, 0.0])
    bottom = np.stack([np.broadcast_to(centre, lo_r.shape), lo_n, lo_r], 1)   # clockwise from +z: faces down
    return np.concatenate([up, walls, bottom])


def load_heightfield(path: str | Path) -> Tuple[np.ndarray, Dict[str, object]]:
    """(heights in mm (rows, cols), metadata) of a height-field .npz (format above)."""
    with np.load(str(path), allow_pickle=False) as z:
        if "heights" not in z:
            raise ValueError("{}: no 'heights' array (keys: {})".format(path, ", ".join(z.files)))
        q = np.asarray(z["heights"])
        meta = {k: (z[k].item() if z[k].ndim == 0 else z[k]) for k in z.files if k != "heights"}
    for k in ("width_mm", "relief_mm"):
        if k not in meta:
            raise ValueError("{}: metadata '{}' missing".format(path, k))
    scale = float(meta.get("z_scale_mm", float(meta["relief_mm"]) / 65535.0))
    H = q.astype(float) * scale if q.dtype.kind in "ui" else q.astype(float)
    return H, meta


def load_part(path: str | Path, stride: int = 1, base_mm: float = 0.0) -> np.ndarray:
    """Triangles (n, 3, 3) mm of a part file: an STL as stored, or a height-field .npz as a closed slab (relief on
    a base of base_mm, else the file's base_mm, else 35 mm). stride > 1 decimates the height-field grid."""
    p = Path(path)
    ext = p.suffix.lower()
    if ext == ".stl":
        return load_stl(p)
    if ext == ".npz":
        H, meta = load_heightfield(p)
        base = float(base_mm) if base_mm and base_mm > 0 else float(meta.get("base_mm", 35.0))
        return heightfield_tris(H, float(meta["width_mm"]), float(meta.get("height_mm", 0.0)), base, stride)
    raise ValueError("unsupported part file '{}': use .stl or a height-field .npz".format(p.name))


def trimesh(tris: np.ndarray):
    out = []
    for t in tris:
        tri = lc.Tri()
        a, b, c = lc.P3(), lc.P3(), lc.P3()
        a.x, a.y, a.z = map(float, t[0])
        b.x, b.y, b.z = map(float, t[1])
        c.x, c.y, c.z = map(float, t[2])
        tri.a, tri.b, tri.c = a, b, c
        out.append(tri)
    return lc.TriMesh(out, 0.0)


def stock_box(tris: np.ndarray, margin: float = 5.0) -> Dict[str, float]:
    lo, hi = tris.reshape(-1, 3).min(0), tris.reshape(-1, 3).max(0)
    return {"xmin": lo[0] - margin, "xmax": hi[0] + margin, "ymin": lo[1] - margin, "ymax": hi[1] + margin,
            "zmin": lo[2], "zmax": hi[2] + margin}


def _params(tol: float, floor: float, radius: float):
    p = lc.PathParams()
    p.max_step = max(min(radius / 2, 2.0), 0.05)
    p.min_step = 0.01
    p.tol = max(tol, 1e-4)
    p.floor = floor
    return p


def _zigzag(tool, mesh, rect, stepover, tol, floor) -> List[np.ndarray]:
    cutter = lc.cutter_of(tool)
    passes = lc.zigzag(cutter, mesh, rect[0], rect[1], rect[2], rect[3], stepover, _params(tol, floor, cutter.radius))
    return [np.array([[c.x, c.y, c.z if c.contact else floor] for c in ps]) for ps in passes if len(ps)]


ENTRY_HEIGHT = 2.0   # mm above the pass start where the plunge at plunge feed begins


def _link(passes: List[np.ndarray], clear_z: float) -> List[Move]:
    """Links between passes: rapid at the clearance plane over the pass start, rapid down to ENTRY_HEIGHT above
    it, plunge to the start at the plunge feed, feed along the pass, straight retract to the clearance plane.
    Rapids are posted as straight lines (not joint moves), so the tool never sweeps sideways near the stone."""
    moves: List[Move] = []
    for ps in passes:
        if len(ps) == 0:
            continue
        s = ps[0]
        moves.append(("rapid", np.array([s[0], s[1], clear_z])))
        entry = np.array([s[0], s[1], min(clear_z, s[2] + ENTRY_HEIGHT)])
        if entry[2] > s[2] + 1e-9:
            moves.append(("rapid", entry))
        moves.append(("plunge", s))
        moves += [("feed", p) for p in ps[1:]]
        moves.append(("rapid", np.array([ps[-1, 0], ps[-1, 1], clear_z])))
    return moves


MAX_FEED_SEGMENT = 50.0   # mm: longest feed segment simplify merges points into
MAX_FEED_TURN = 5.0       # deg: largest tool-axis turn over one merged feed segment (5-axis)


def simplify(moves: List[Move], tol: float, ang_tol_deg: float = 0.5,
             max_segment: float = MAX_FEED_SEGMENT, max_turn_deg: float = MAX_FEED_TURN) -> List[Move]:
    """Drop feed points that lie within `tol` (mm) of the straight line between their kept neighbours, with the
    tool axis within `ang_tol_deg` (Douglas-Peucker per feed run). Rapids, plunges and run ends are kept, so the
    posted path stays within the step tolerance of the computed CL path. A merged segment is split (at its middle
    point) while longer than `max_segment` mm or while the tool axis turns more than `max_turn_deg` over it: the
    planner solves IK per kept point and treats a joint jump of more than 10 deg inside a cut as a break, and a
    robot's joints turn that far over a few hundred mm of straight line or a large axis turn."""
    out: List[Move] = []
    run: List[Move] = []

    cmax = math.cos(math.radians(max_turn_deg)) if max_turn_deg > 0 else -2.0

    def densify(kept):
        """Points on the straight move between kept neighbours (tip linear, axis interpolated, as the controller's
        linear move runs it) wherever a move is longer than max_segment or turns the axis more than max_turn_deg:
        where the CL path itself has no point in between, e.g. a ball rolling over a part's edge."""
        res = kept[:1]
        for m0, m1 in zip(kept, kept[1:]):
            n = 1
            L = float(np.linalg.norm(m1[1] - m0[1]))
            if max_segment > 0:
                n = max(n, int(math.ceil(L / max_segment - 1e-9)))
            if len(m0) > 2 and len(m1) > 2 and max_turn_deg > 0 and float(m0[2] @ m1[2]) < cmax:
                ang = math.degrees(math.acos(max(-1.0, min(1.0, float(m0[2] @ m1[2])))))
                n = max(n, int(math.ceil(ang / max_turn_deg - 1e-9)))
            for i in range(1, n):
                f = i / n
                p = m0[1] + (m1[1] - m0[1]) * f
                if len(m0) > 2 and len(m1) > 2:
                    a = m0[2] * (1 - f) + m1[2] * f
                    res.append(("feed", p, a / np.linalg.norm(a)) + tuple(m1[3:]))   # per-point attributes carry on
                else:
                    res.append(("feed", p))
            res.append(m1)
        return res

    def flush():
        if len(run) <= 2:
            out.extend(densify(run))
            return
        P = np.array([m[1] for m in run], dtype=float)
        A = np.array([m[2] for m in run], dtype=float) if len(run[0]) > 2 else None
        keep = np.zeros(len(P), bool)
        keep[0] = keep[-1] = True
        stack = [(0, len(P) - 1)]
        ca = math.cos(math.radians(ang_tol_deg))
        cturn = math.cos(math.radians(max_turn_deg)) if max_turn_deg > 0 else -2.0
        while stack:
            a, b = stack.pop()
            if b <= a + 1:
                continue
            d = P[b] - P[a]
            L = float(d @ d)
            seg = P[a + 1:b] - P[a]
            t = np.clip(seg @ d / L, 0, 1) if L > 0 else np.zeros(len(seg))
            dist = np.linalg.norm(seg - t[:, None] * d, axis=1)
            if A is not None:   # interpolated axis vs the actual axis
                Ai = A[a] + (A[b] - A[a]) * t[:, None]
                Ai /= np.linalg.norm(Ai, axis=1, keepdims=True)
                bad = np.einsum("ij,ij->i", Ai, A[a + 1:b]) < ca
                dist = np.where(bad, np.inf, dist)
            k = int(np.argmax(dist))
            too_long = max_segment > 0 and L > max_segment * max_segment
            too_turned = A is not None and float(A[a] @ A[b]) < cturn
            if dist[k] <= tol and (too_long or too_turned):
                k = (b - a) // 2 - 1                      # split at the middle point
                dist[k] = np.inf
            if dist[k] > tol:
                keep[a + 1 + k] = True
                stack += [(a, a + 1 + k), (a + 1 + k, b)]
        out.extend(densify([m for m, kk in zip(run, keep) if kk]))

    for m in moves:
        if m[0] == "feed":
            run.append(m)
        else:
            flush()
            run = []
            out.append(m)
            if m[0] == "plunge":
                run = [m]          # a pass starts at its plunge point
                out.pop()
    flush()
    return out


def _clamp_runs(ps: np.ndarray, level: float, top: float) -> List[np.ndarray]:
    """Split a pass where the tip would be above the stock top (air): only cut where material is."""
    runs, cur = [], []
    for p in ps:
        if p[2] < top - 1e-9:
            cur.append(p)
        elif cur:
            runs.append(np.array(cur))
            cur = []
    if cur:
        runs.append(np.array(cur))
    return runs


_EDGE_INSET = 1e-3   # mm


def _part_rect(lo, hi, R):
    """The zigzag rectangle: the part's xy box grown by the cutter radius, less 1 micrometre. On the grown box itself
    the first and last passes hold the cutter exactly tangent to the part's extreme edge, and the native zigzag
    does not terminate there (seen on a slab's straight top edge, 2026-11-14); the inset cuts nothing."""
    return (lo[0] - R + _EDGE_INSET, hi[0] + R - _EDGE_INSET, lo[1] - R + _EDGE_INSET, hi[1] + R - _EDGE_INSET)


# ------------------------------------------------------------------------------------------------------------------
# Boundary: keep the passes inside an xy rectangle or polygon (a relief panel's border, not the slab sides)

def parse_boundary(b):
    """The boundary as a counter-clockwise polygon (n, 2) in mm, or None. Accepts None or "" (no boundary), a
    rectangle written "xmin,ymin,xmax,ymax" (4 numbers), a polygon written "x,y; x,y; x,y" (3 or more points), a dict
    with xmin, ymin, xmax, ymax, or an array: 4 numbers = rectangle, (n, 2) = polygon."""
    if b is None:
        return None
    if isinstance(b, dict):
        b = [b["xmin"], b["ymin"], b["xmax"], b["ymax"]]
    if isinstance(b, str):
        text = b.replace("\n", ";").strip()
        if not text:
            return None
        if ";" in text:
            pts = [[float(v) for v in s.split(",")] for s in text.split(";") if s.strip()]
            if len(pts) < 3 or any(len(q) != 2 for q in pts):
                raise ValueError("a boundary polygon needs at least 3 points written 'x,y; x,y; x,y' (mm)")
            P = np.asarray(pts, dtype=float)
        else:
            v = [float(s) for s in text.split(",") if s.strip()]
            if len(v) != 4:
                raise ValueError("a boundary rectangle is 'xmin,ymin,xmax,ymax' (4 numbers, mm); a polygon is "
                                 "'x,y; x,y; x,y'")
            b = v
    if not isinstance(b, str):
        A = np.asarray(b, dtype=float)
        if A.size == 0:
            return None
        if A.ndim == 1 and A.size == 4:
            x0, y0, x1, y1 = (float(v) for v in A)
            if not (x1 > x0 and y1 > y0):
                raise ValueError("boundary rectangle needs xmax > xmin and ymax > ymin")
            P = np.array([[x0, y0], [x1, y0], [x1, y1], [x0, y1]])
        elif A.ndim == 2 and A.shape[1] == 2 and len(A) >= 3:
            P = A
        else:
            raise ValueError("a boundary is 4 numbers (rectangle) or at least 3 xy points (polygon)")
    area = 0.5 * float(np.sum(P[:, 0] * np.roll(P[:, 1], -1) - np.roll(P[:, 0], -1) * P[:, 1]))
    if abs(area) < 1e-9:
        raise ValueError("the boundary has no area")
    return P if area > 0 else P[::-1]


def _in_poly(P: np.ndarray, x: float, y: float) -> bool:
    inside = False
    n = len(P)
    for k in range(n):
        x0, y0 = P[k]
        x1, y1 = P[(k + 1) % n]
        if (y0 > y) != (y1 > y) and x < x0 + (y - y0) * (x1 - x0) / (y1 - y0):
            inside = not inside
    return inside


def boundary_rect(rect, poly):
    """The zigzag rectangle (xmin, xmax, ymin, ymax) cut down to the boundary's bounding box."""
    if poly is None:
        return rect
    lo, hi = poly.min(0), poly.max(0)
    r = (max(rect[0], lo[0]), min(rect[1], hi[0]), max(rect[2], lo[1]), min(rect[3], hi[1]))
    if not (r[1] > r[0] and r[3] > r[2]):
        raise ValueError("the boundary lies outside the part: nothing to cut")
    return r


def clip_polyline(P: np.ndarray, poly, zfun=None) -> List[np.ndarray]:
    """The parts of the polyline P (n, 3) inside the polygon `poly`, each with its ends on the boundary. A crossing
    takes its z from `zfun(x, y)` when given, else linearly between the two points. poly None returns [P]."""
    P = np.asarray(P, dtype=float)
    if poly is None or len(P) == 0:
        return [P] if len(P) else []
    n = len(poly)
    runs: List[List[np.ndarray]] = []
    cur: List[np.ndarray] = []

    def z_at(p, q, t):
        xy = p[:2] + (q[:2] - p[:2]) * t
        return float(zfun(xy[0], xy[1])) if zfun is not None else float(p[2] + (q[2] - p[2]) * t)

    if len(P) == 1:
        return [P] if _in_poly(poly, P[0, 0], P[0, 1]) else []
    for a, b in zip(P[:-1], P[1:]):
        ts = [0.0, 1.0]
        d = b[:2] - a[:2]
        for k in range(n):
            e0, e1 = poly[k], poly[(k + 1) % n]
            e = e1 - e0
            den = d[0] * e[1] - d[1] * e[0]
            if abs(den) < 1e-14:
                continue
            w = e0 - a[:2]
            t = (w[0] * e[1] - w[1] * e[0]) / den
            s = (w[0] * d[1] - w[1] * d[0]) / den
            if 0.0 < t < 1.0 and -1e-12 <= s <= 1.0 + 1e-12:
                ts.append(float(t))
        ts = sorted(set(ts))
        for t0, t1 in zip(ts[:-1], ts[1:]):
            tm = 0.5 * (t0 + t1)
            m = a[:2] + d * tm
            if not _in_poly(poly, m[0], m[1]):
                if cur:
                    runs.append(cur)
                    cur = []
                continue
            p0 = a + (b - a) * t0 if t0 > 0 else a.copy()
            p1 = b.copy() if t1 >= 1.0 else a + (b - a) * t1
            if t0 > 0:
                p0[2] = z_at(a, b, t0)
            if t1 < 1.0:
                p1[2] = z_at(a, b, t1)
            if not cur:
                cur = [p0]
            cur.append(p1)
    if cur:
        runs.append(cur)
    return [np.array(r) for r in runs if len(r) >= 2]


def clip_passes(passes: List[np.ndarray], poly, zfun=None) -> List[np.ndarray]:
    if poly is None:
        return passes
    out: List[np.ndarray] = []
    for ps in passes:
        out += clip_polyline(ps, poly, zfun)
    return out


def raster(tris, mesh, tool, stock, stepover=2.0, scallop=0.0, allowance=0.0, tolerance=0.01, clearance=20.0,
           boundary=None, **_) -> List[Move]:
    """Zigzag finishing. boundary (see parse_boundary): the tool tip stays inside this xy rectangle or polygon, so the
    passes stop at a panel's border instead of running down the stock sides."""
    R = tool.diameter / 2
    if scallop > 0 and tool.kind == lc.ToolKind.BallEndMill:
        stepover = min(stepover, lc.max_stepover(R, scallop)) if stepover > 0 else lc.max_stepover(R, scallop)
    lo, hi = tris.reshape(-1, 3).min(0), tris.reshape(-1, 3).max(0)
    poly = parse_boundary(boundary)
    rect = boundary_rect(_part_rect(lo, hi, R), poly)
    passes = _zigzag(tool, mesh, rect, max(stepover, 0.05), tolerance, lo[2])
    if poly is not None:
        cutter = lc.cutter_of(tool)

        def zfun(x, y):
            z = lc.drop_cutter(cutter, mesh, float(x), float(y))
            return z if math.isfinite(z) else lo[2]
        passes = clip_passes(passes, poly, zfun)
    passes = [p + np.array([0, 0, allowance]) for p in passes]
    return _link(passes, stock["zmax"] + clearance)


def rough_plane(tris, mesh, tool, stock, stepdown=5.0, stepover=4.0, allowance=0.5, tolerance=0.01, clearance=20.0,
                **_) -> List[Move]:
    lo = tris.reshape(-1, 3).min(0)
    rect = (stock["xmin"], stock["xmax"], stock["ymin"], stock["ymax"])
    base = _zigzag(tool, mesh, rect, max(stepover, 0.05), tolerance, lo[2])
    top = stock["zmax"]
    # every tip is max(level, surface + allowance): at and below the lowest surface tip a level only repeats the
    # surface-following pass, so the levels stop there (a slab on a base is not re-cut down to its bottom)
    floor = lo[2]
    if base:
        floor = max(floor, min(float(ps[:, 2].min()) for ps in base) + allowance)
    levels = []
    z = top - stepdown
    while z > floor + 1e-9:
        levels.append(z)
        z -= stepdown
    if not levels or levels[-1] > floor + 1e-9:
        levels.append(floor)
    moves: List[Move] = []
    for level in levels:
        cut = []
        for ps in base:
            tip = ps.copy()
            tip[:, 2] = np.maximum(level, ps[:, 2] + allowance)
            cut += _clamp_runs(tip, level, top)
        moves += _link(cut, top + clearance)
    return moves


def face(tris, mesh, tool, stock, depth=1.0, stepover=0.0, allowance=0.0, tolerance=0.01, clearance=20.0,
         **_) -> List[Move]:
    R = tool.diameter / 2
    so = stepover if stepover > 0 else 0.7 * tool.diameter
    rect = (stock["xmin"] - R, stock["xmax"] + R, stock["ymin"] - R, stock["ymax"] + R)
    level = stock["zmax"] - depth
    base = _zigzag(tool, mesh, rect, so, tolerance, level)
    cut = []
    for ps in base:
        tip = ps.copy()
        tip[:, 2] = np.maximum(level, ps[:, 2] + allowance)
        cut.append(tip)
    return _link(cut, stock["zmax"] + clearance)


def _limit_axis_rate(A: np.ndarray, C: np.ndarray, rate: float) -> np.ndarray:
    """Turn the axis at most `rate` radians per mm of centre travel, forward then backward along the pass (the
    ball centre path is unchanged, so this never gouges; it only spreads a sudden normal change, like the edge
    of a part, over a length the robot can follow)."""
    if rate <= 0 or len(A) < 2:
        return A
    A = A.copy()
    ds = np.r_[0.0, np.linalg.norm(np.diff(C, axis=0), axis=1)]
    for order in (range(1, len(A)), range(len(A) - 2, -1, -1)):
        for k in order:
            j = k - 1 if order.step == 1 else k + 1
            lim = rate * max(ds[k] if order.step == 1 else ds[j], 1e-6)
            c = float(np.clip(A[k] @ A[j], -1.0, 1.0))
            ang = math.acos(c)
            if ang > lim:
                w = A[k] - A[j] * c
                w /= max(np.linalg.norm(w), 1e-12)
                A[k] = math.cos(lim) * A[j] + math.sin(lim) * w
    return A


def surfacing5(tris, mesh, tool, stock, stepover=2.0, lead=15.0, max_tilt=45.0, smooth=7, allowance=0.0,
               tolerance=0.01, clearance=20.0, axis_rate=2.0, boundary=None, **_) -> List[Move]:
    """Simultaneous 5-axis ball-end finishing. Moves are (kind, tip point mm, tool axis) in the setup frame; the
    axis points from the tip to the holder.

    The ball centre runs on the drop-cutter offset surface, so the cut surface does not depend on the axis: the
    path is gouge free for the ball whatever the tilt. The axis is the offset surface's normal (equal to the part
    normal at the contact) leaned back against the feed by `lead`, so the contact moves off the ball's tip where
    the cutting speed is zero; then clamped to `max_tilt` from the setup z axis. Only the shank and holder change
    with the axis: the robot planner checks them for collision."""
    if tool.kind != lc.ToolKind.BallEndMill:
        raise ValueError("5-axis surfacing needs a ball-end mill (got {})".format(lc.tool_kind_name(tool.kind)))
    R = tool.diameter / 2
    cutter = lc.cutter_of(tool)
    lo, hi = tris.reshape(-1, 3).min(0), tris.reshape(-1, 3).max(0)
    poly = parse_boundary(boundary)
    rect = boundary_rect(_part_rect(lo, hi, R), poly)
    passes = _zigzag(tool, mesh, rect, max(stepover, 0.05), tolerance, lo[2])
    d = max(0.25, min(1.0, R / 4))
    floor = lo[2]

    def h(x, y):
        z = lc.drop_cutter(cutter, mesh, float(x), float(y))
        return z if math.isfinite(z) else floor
    passes = clip_passes(passes, poly, h)

    lam, tmax = math.radians(lead), math.radians(max_tilt)
    zax = np.array([0.0, 0.0, 1.0])
    moves: List[Move] = []
    clear_z = stock["zmax"] + clearance
    for ps in passes:
        if len(ps) < 2:
            continue
        C = ps + np.array([0.0, 0.0, R])                                   # ball centres
        N = np.array([[-(h(x + d, y) - h(x - d, y)) / (2 * d), -(h(x, y + d) - h(x, y - d)) / (2 * d), 1.0]
                      for x, y, _ in ps])
        N /= np.linalg.norm(N, axis=1, keepdims=True)
        if smooth > 1:                                                     # moving average along the pass
            k = int(smooth)
            pad = np.pad(N, ((k // 2, k - 1 - k // 2), (0, 0)), mode="edge")
            N = np.stack([np.convolve(pad[:, j], np.ones(k) / k, mode="valid") for j in range(3)], 1)
            N /= np.linalg.norm(N, axis=1, keepdims=True)
        F = np.gradient(C, axis=0)
        A = []
        for n, f in zip(N, F):
            f = f - n * (f @ n)
            nf = np.linalg.norm(f)
            a = n if nf < 1e-9 else math.cos(lam) * n - math.sin(lam) * f / nf
            ang = math.acos(max(-1.0, min(1.0, a @ zax)))
            if ang > tmax:                                                 # pull back towards the setup axis
                w = a - zax * (a @ zax)
                a = math.cos(tmax) * zax + math.sin(tmax) * w / max(np.linalg.norm(w), 1e-12)
            A.append(a / np.linalg.norm(a))
        A = _limit_axis_rate(np.asarray(A), C, math.radians(axis_rate))
        C = C + N * allowance
        tips = C - R * A
        moves.append(("rapid", np.array([tips[0, 0], tips[0, 1], clear_z]), zax))
        moves.append(("rapid", tips[0] + A[0] * ENTRY_HEIGHT, A[0]))
        moves.append(("plunge", tips[0], A[0]))
        moves += [("feed", t, a) for t, a in zip(tips[1:], A[1:])]
        moves.append(("rapid", np.array([tips[-1, 0], tips[-1, 1], clear_z]), zax))
    return moves


# ------------------------------------------------------------------------------------------------------------------
# Waterline (constant-Z finishing) and steep / shallow finishing, on the drop-cutter height field.
#
# h(x, y) is the drop-cutter tip height: the lowest tip at (x, y) that does not cut the part. A tip at height z over
# (x, y) gouges exactly when h(x, y) > z. The waterline at level z is the boundary of {h > z}: there the cutter
# touches the part at height z and cuts nothing below it. Method:
#   1. sample h on a regular xy grid (`cell`) once; the grid reaches R + 2 cells past the part, where h = -inf;
#   2. per level, marching squares on {h > z} gives closed cycles of grid edges (saddles resolved by h at the
#      cell centre);
#   3. each crossing is bisected along its grid edge with drop_cutter and kept on the allowed side (h <= z), until
#      the bracket is at most `tolerance` wide and h is within `tolerance` below z (or the bracket is 1e-6 mm wide,
#      at a vertical step of h such as a flat cutter on a wall);
#   4. each chord between loop points is checked at its quarter points; where h exceeds z by more than
#      `tolerance` and no allowed point lies within `tolerance` across the chord (a wall, where h steps up), a new
#      contour point is bisected in next to it. The straight moves then cut into the part by at most `tolerance`,
#      measured vertically or horizontally.
# Every posted point satisfies h <= tip z (no gouge) and lies within `tolerance` of the true contour.
# Features of h narrower than `cell` can be missed by the grid; `cell` defaults to R / 5, at most 0.5 mm.

_EPS = 1e-6       # mm: h within this of a level counts as on it (flat floors exactly at a level are not loops)
_WMIN = 1e-6      # mm: smallest bisection bracket


def _offset_cutter(cutter, a: float):
    """The cutter grown by `a` (radius, and corner for a bull nose). drop(grown) + a is the tip height that keeps
    the tool `a` clear of the part: exact for a ball (offset sphere), conservative-approximate for the others."""
    if a == 0:
        return cutter
    if cutter.kind not in (lc.CutterKind.Flat, lc.CutterKind.Ball, lc.CutterKind.Bull, lc.CutterKind.Cone):
        # profile and disc cutters carry segments, thickness and heading; a field-by-field copy would drop them
        raise ValueError("stock to leave (allowance) is supported for flat, ball, bull and cone cutters only; "
                         "use allowance 0 with a {} cutter".format(str(cutter.kind).split(".")[-1]))
    c = lc.Cutter()
    c.kind = cutter.kind
    c.radius = cutter.radius + a
    c.corner = cutter.corner + a if cutter.kind == lc.CutterKind.Bull else cutter.corner
    c.half_angle = cutter.half_angle
    return c


class _Field:
    """Drop-cutter height field of one tool over one part, with its grid samples and slope."""

    def __init__(self, tris, mesh, tool, allowance=0.0, cell=0.0):
        base = lc.cutter_of(tool)
        self.cutter = _offset_cutter(base, float(allowance))
        self.mesh, self.a = mesh, float(allowance)
        self.R = float(self.cutter.radius)
        self.lo, self.hi = tris.reshape(-1, 3).min(0), tris.reshape(-1, 3).max(0)
        self.floor = float(self.lo[2]) + self.a          # the part's lowest point, lifted by the allowance
        cell = float(cell) if cell and cell > 0 else min(0.5, max(base.radius / 5.0, 0.02))
        m = self.R + 2 * cell
        span = max(self.hi[0] - self.lo[0] + 2 * m, self.hi[1] - self.lo[1] + 2 * m)
        cell = max(cell, span / 2000.0)                 # at most 2000 x 2000 samples
        self.cell = cell
        self.xs = self.lo[0] - m + cell * np.arange(int(math.ceil((self.hi[0] - self.lo[0] + 2 * m) / cell)) + 1)
        self.ys = self.lo[1] - m + cell * np.arange(int(math.ceil((self.hi[1] - self.lo[1] + 2 * m) / cell)) + 1)
        self.H = np.array([[self(x, y) for x in self.xs] for y in self.ys])
        self._slope = None

    def __call__(self, x, y) -> float:
        z = lc.drop_cutter(self.cutter, self.mesh, float(x), float(y))
        return z + self.a if math.isfinite(z) else -math.inf

    def slope(self) -> np.ndarray:
        """Slope of h in degrees per grid sample. Where nothing is under the cutter h is taken as the floor minus
        the radius (where a cutter rolling off the part's edge ends up)."""
        if self._slope is None:
            Hf = np.where(np.isfinite(self.H), self.H, self.floor - self.R)
            gy, gx = np.gradient(Hf, self.cell)
            self._slope = np.degrees(np.arctan(np.hypot(gx, gy)))
        return self._slope

    def steep_mask(self, threshold_deg: float) -> np.ndarray:
        """Samples on steep ground: slope >= threshold and the tip at or above the part's floor (a cutter rolling
        off the part's outer edge is not on a steep wall; the raster family covers it)."""
        return (self.slope() >= threshold_deg) & (self.H >= self.floor - _EPS)

    def lookup(self, mask: np.ndarray, x, y) -> np.ndarray:
        i = np.clip(np.rint((np.asarray(x) - self.xs[0]) / self.cell).astype(int), 0, len(self.xs) - 1)
        j = np.clip(np.rint((np.asarray(y) - self.ys[0]) / self.cell).astype(int), 0, len(self.ys) - 1)
        return mask[j, i]


def _dilate(mask: np.ndarray, k: int) -> np.ndarray:
    """Grow a boolean grid mask by a disc of k samples."""
    if k <= 0:
        return mask
    ny, nx = mask.shape
    out = mask.copy()
    for dj in range(-k, k + 1):
        for di in range(-k, k + 1):
            if di * di + dj * dj > k * k:
                continue
            out[max(dj, 0):ny + min(dj, 0), max(di, 0):nx + min(di, 0)] |= \
                mask[max(-dj, 0):ny + min(-dj, 0), max(-di, 0):nx + min(-di, 0)]
    return out


def _bisect(f, a, ha, b, z, tol):
    """a allowed (h(a) <= z), b forbidden (h(b) > z): shrink the bracket; return the allowed end and its h."""
    while True:
        w = math.hypot(b[0] - a[0], b[1] - a[1])
        if w <= _WMIN or (w <= tol and z - ha <= tol):
            return a, ha
        m = ((a[0] + b[0]) / 2, (a[1] + b[1]) / 2)
        hm = f(*m)
        if hm > z + _EPS:
            b = m
        else:
            a, ha = m, hm


def _chord(f, p, q, z, tol, reach, depth=0):
    """Contour points to insert between loop points p and q (x, y, h, fx, fy) so the straight move between them
    does not gouge by more than tol. reach: largest search distance for an allowed point."""
    d = (q[0] - p[0], q[1] - p[1])
    L = math.hypot(*d)
    if depth > 12 or L <= tol:
        return []
    n = (-d[1] / L, d[0] / L)
    w = None
    for t in (0.5, 0.25, 0.75):
        x, y = p[0] + t * d[0], p[1] + t * d[1]
        if f(x, y) <= z + tol:
            continue                                     # within tol vertically
        e = 0.9 * tol
        if f(x + e * n[0], y + e * n[1]) <= z + _EPS or f(x - e * n[0], y - e * n[1]) <= z + _EPS:
            continue                                     # within tol horizontally (a wall: h steps up)
        w = (x, y)
        break
    if w is None:
        return []
    s = max(L / 8, tol)
    while s <= reach:
        for sg in (1.0, -1.0):
            c = (w[0] + sg * s * n[0], w[1] + sg * s * n[1])
            hc = f(*c)
            if hc <= z + _EPS:
                a, ha = _bisect(f, c, hc, w, z, tol)
                fx, fy = w[0] - c[0], w[1] - c[1]
                fl = math.hypot(fx, fy) or 1.0
                r = (a[0], a[1], ha, fx / fl, fy / fl)
                return _chord(f, p, r, z, tol, reach, depth + 1) + [r] + _chord(f, r, q, z, tol, reach, depth + 1)
        s *= 2
    return []


def _level_loops(fld: _Field, z: float, tol: float) -> List[np.ndarray]:
    """Closed waterline loops at level z: arrays (n, 5) of x, y, h, and the unit xy direction towards the
    material (h > z) at each point; the first point is not repeated."""
    f, H, xs, ys, cell = fld, fld.H, fld.xs, fld.ys, fld.cell
    F = H > z + _EPS
    F[0, :] = F[-1, :] = F[:, 0] = F[:, -1] = False
    hx = F[:, :-1] != F[:, 1:]          # edge (i, j)-(i+1, j)
    vy = F[:-1, :] != F[1:, :]          # edge (i, j)-(i, j+1)
    cells = hx[:-1, :] | hx[1:, :] | vy[:, :-1] | vy[:, 1:]
    adj: Dict[tuple, list] = {}

    def link(e1, e2):
        adj.setdefault(e1, []).append(e2)
        adj.setdefault(e2, []).append(e1)

    for j, i in zip(*np.nonzero(cells)):
        e = [(0, i, j), (1, i + 1, j), (0, i, j + 1), (1, i, j)]          # bottom, right, top, left
        c = [F[j, i], F[j, i + 1], F[j + 1, i + 1], F[j + 1, i]]           # corners 0..3 counter-clockwise
        crossed = [k for k in range(4) if c[k] != c[(k + 1) % 4]]
        if len(crossed) == 2:
            link(e[crossed[0]], e[crossed[1]])
        elif len(crossed) == 4:                                            # saddle: the centre decides
            centre = f(xs[i] + cell / 2, ys[j] + cell / 2) > z + _EPS
            for k in range(4):
                if c[k] != centre:                                         # cut off the corner unlike the centre
                    link(e[(k - 1) % 4], e[k])

    def point(edge):
        kind, i, j = edge
        p0 = (xs[i], ys[j])
        p1 = (xs[i + 1], ys[j]) if kind == 0 else (xs[i], ys[j + 1])
        h0 = H[j, i]
        h1 = H[j, i + 1] if kind == 0 else H[j + 1, i]
        if F[j, i]:
            p0, p1, h0, h1 = p1, p0, h1, h0
        a, ha = _bisect(f, p0, h0, p1, z, tol)
        fx, fy = p1[0] - p0[0], p1[1] - p0[1]
        fl = math.hypot(fx, fy)
        return (a[0], a[1], ha, fx / fl, fy / fl)

    loops, seen = [], set()
    for s in adj:
        if s in seen:
            continue
        cyc, prev, cur = [s], None, s
        seen.add(s)
        while True:
            nb = adj[cur]
            nxt = nb[1] if nb[0] == prev else nb[0]
            if nxt == s:
                break
            cyc.append(nxt)
            seen.add(nxt)
            prev, cur = cur, nxt
        pts = [point(e) for e in cyc]
        dense = []
        for k in range(len(pts)):
            dense.append(pts[k])
            dense += _chord(f, pts[k], pts[(k + 1) % len(pts)], z, tol, 2 * cell)
        loops.append(np.array(dense, dtype=float))
    return loops


def _orient(loop: np.ndarray, climb: bool) -> np.ndarray:
    """Climb milling with a clockwise spindle (M3) keeps the material on the tool's right: clockwise around an
    island, counter-clockwise inside a pocket. Conventional is the reverse."""
    P = loop[:, :2]
    t = np.roll(P, -1, axis=0) - np.roll(P, 1, axis=0)
    right = float(np.sum(t[:, 0] * loop[:, 4] - t[:, 1] * loop[:, 3])) < 0   # cross(t, material dir) < 0
    return loop if right == climb else loop[::-1]


def _levels(top: float, bottom: float, stepdown: float) -> List[float]:
    out, z = [], top - stepdown
    while z > bottom + _EPS:
        out.append(z)
        z -= stepdown
    out.append(bottom)
    return out


def _nearest_order(runs: List[Tuple[np.ndarray, bool]], start) -> List[np.ndarray]:
    """Order runs nearest-next from `start`. Closed runs start at their point nearest the tool and end there
    again; open runs keep their direction (climb / conventional)."""
    out, pos, todo = [], np.asarray(start, float)[:2], list(runs)
    while todo:
        best = None
        for n, (r, closed) in enumerate(todo):
            if closed:
                d = np.linalg.norm(r[:, :2] - pos, axis=1)
                k = int(np.argmin(d))
                cand = (float(d[k]), n, k)
            else:
                cand = (float(np.linalg.norm(r[0, :2] - pos)), n, 0)
            if best is None or cand < best:
                best = cand
        _, n, k = best
        r, closed = todo.pop(n)
        if closed:
            r = np.vstack([np.roll(r, -k, axis=0), np.roll(r, -k, axis=0)[:1]])
        out.append(r)
        pos = r[-1, :2]
    return out


def _clip_closed(loop: np.ndarray, keep: np.ndarray) -> List[Tuple[np.ndarray, bool]]:
    if keep.all():
        return [(loop, True)]
    if not keep.any():
        return []
    k = int(np.argmin(keep))                     # first excluded point: start after it
    loop, keep = np.roll(loop, -k, axis=0), np.roll(keep, -k)
    return [(r, False) for r in _runs(loop, keep)]


def _runs(P: np.ndarray, keep: np.ndarray) -> List[np.ndarray]:
    out, cur = [], []
    for p, k in zip(P, keep):
        if k:
            cur.append(p)
        elif cur:
            out.append(np.array(cur))
            cur = []
    if cur:
        out.append(np.array(cur))
    return [r for r in out if len(r) >= 2]


def _waterline_runs(fld: _Field, stepdown, tolerance, direction="climb", mask=None) -> List[np.ndarray]:
    """Tip runs (n, 3) of the waterline, top level first, nearest-next within a level; closed loops repeat their
    first point at the end. `mask`: grid mask the points must lie on (None = everywhere)."""
    finite = fld.H[np.isfinite(fld.H)]
    if finite.size == 0:
        return []
    top = float(finite.max())
    climb = str(direction).lower() != "conventional"
    tol = max(float(tolerance), 1e-4)
    out: List[np.ndarray] = []
    pos = np.array([fld.xs[0], fld.ys[0]])
    for z in _levels(top, fld.floor, max(float(stepdown), 0.01)):
        runs = []
        for lp in _level_loops(fld, z, tol):
            lp = _orient(lp, climb)
            tip = np.column_stack([lp[:, 0], lp[:, 1], np.maximum(z, lp[:, 2])])
            if mask is None:
                runs.append((tip, True))
            else:
                runs += _clip_closed(tip, fld.lookup(mask, tip[:, 0], tip[:, 1]))
        for r in _nearest_order(runs, pos):
            out.append(r)
            pos = r[-1, :2]
    return out


def waterline(tris, mesh, tool, stock, stepdown=1.0, allowance=0.0, tolerance=0.01, clearance=20.0,
              min_slope_deg=0.0, overlap=0.0, direction="climb", cell=0.0, boundary=None, **_) -> List[Move]:
    """Constant-Z finishing loops on the drop-cutter contour of every level from the part's top down to its floor,
    `stepdown` apart. min_slope_deg > 0 keeps only the parts of the loops on ground at least that steep (the steep
    region grown by `overlap` mm). direction: "climb" or "conventional" for a clockwise (M3) spindle."""
    fld = _Field(tris, mesh, tool, allowance, cell)
    mask = None
    if min_slope_deg > 0:
        mask = _dilate(fld.steep_mask(float(min_slope_deg)), int(math.ceil(max(overlap, 0.0) / fld.cell)))
    runs = _waterline_runs(fld, stepdown, tolerance, direction, mask)
    runs = _clip_runs(runs, parse_boundary(boundary))
    return _link(runs, stock["zmax"] + clearance)


def _clip_runs(runs: List[np.ndarray], poly) -> List[np.ndarray]:
    """Waterline and raster runs cut to the boundary. A closed loop that crosses it is opened at the crossings."""
    if poly is None:
        return runs
    out: List[np.ndarray] = []
    for r in runs:
        out += clip_polyline(r, poly)
    return out


def steep_shallow_paths(tris, mesh, tool, stock, threshold_deg=40.0, overlap=1.0, stepover=2.0, stepdown=0.0,
                        allowance=0.0, tolerance=0.01, direction="climb", cell=0.0, boundary=None, **_):
    """The two families of steep / shallow finishing as tip runs: (waterline runs on steep ground, raster runs on
    shallow ground). Slope is that of the drop-cutter height field (for a ball, the part's slope at the contact).
    Each family covers its region grown by `overlap` mm, so the two overlap in a band 2 * overlap wide.
    stepdown <= 0 picks stepover * tan(threshold): on the shallowest steep ground the waterline levels are then
    `stepover` apart horizontally, like the raster passes."""
    th = min(max(float(threshold_deg), 1.0), 89.0)
    if stepdown <= 0:
        stepdown = stepover * math.tan(math.radians(th))
    fld = _Field(tris, mesh, tool, allowance, cell)
    k = int(math.ceil(max(overlap, 0.0) / fld.cell))
    steep = fld.steep_mask(th)
    wl = _waterline_runs(fld, stepdown, tolerance, direction, _dilate(steep, k))
    shallow = _dilate(~steep, k)
    R = tool.diameter / 2
    lo, hi = fld.lo, fld.hi
    poly = parse_boundary(boundary)
    passes = lc.zigzag(fld.cutter, mesh, *boundary_rect(_part_rect(lo, hi, R), poly), max(stepover, 0.05),
                       _params(tolerance, lo[2], fld.cutter.radius))
    ra = []
    for ps in passes:
        if not len(ps):
            continue
        P = np.array([[c.x, c.y, c.z + fld.a] for c in ps])
        contact = np.array([bool(c.contact) for c in ps])
        ra += _runs(P, contact & fld.lookup(shallow, P[:, 0], P[:, 1]))
    return _clip_runs(wl, poly), _clip_runs(ra, poly)


def steep_shallow(tris, mesh, tool, stock, threshold_deg=40.0, overlap=1.0, stepover=2.0, stepdown=0.0,
                  allowance=0.0, tolerance=0.01, clearance=20.0, direction="climb", cell=0.0, boundary=None,
                  **_) -> List[Move]:
    """Steep / shallow finishing: waterline loops on ground steeper than `threshold_deg`, raster along x on the
    rest, with an `overlap` mm band cut by both. Waterline first, then raster. boundary (see parse_boundary): the
    tool tip stays inside this xy rectangle or polygon."""
    wl, ra = steep_shallow_paths(tris, mesh, tool, stock, threshold_deg, overlap, stepover, stepdown, allowance,
                                 tolerance, direction, cell, boundary=boundary)
    return _link(wl + ra, stock["zmax"] + clearance)


GENERATORS = {"raster": raster, "rough_plane": rough_plane, "face": face, "surfacing5": surfacing5,
              "waterline": waterline, "steep_shallow": steep_shallow}


def path_length(moves: List[Move]) -> Tuple[float, float]:
    """Feed and rapid lengths, mm."""
    feed = rapid = 0.0
    for (k0, p0, *_), (k1, p1, *_) in zip(moves, moves[1:]):
        d = float(np.linalg.norm(p1 - p0))
        if k1 in ("tap_in", "tap_out") or k1 == "feed" and k0 in ("feed", "plunge") or k1 == "plunge":
            feed += d
        else:
            rapid += d
    return feed, rapid


def zmap_check(tris, mesh, tool, moves: List[Move], stock, cell: float = 1.0):
    """Cut the stock along the feed moves in a Z-map; return (deviation stats over the part's xy box, zmap)."""
    zm = lc.make_stock(stock["xmin"], stock["xmax"], stock["ymin"], stock["ymax"], cell, stock["zmax"])
    passes, cur = [], []
    for k, p, *_ in moves:
        if k in ("feed", "plunge"):
            c = lc.CLPoint()
            c.x, c.y, c.z, c.contact = float(p[0]), float(p[1]), float(p[2]), True
            cur.append(c)
        elif cur:
            passes.append(cur)
            cur = []
    if cur:
        passes.append(cur)
    lc.cut_passes(zm, lc.cutter_of(tool), passes)
    target = lc.target_heights(mesh, zm)
    lo, hi = tris.reshape(-1, 3).min(0), tris.reshape(-1, 3).max(0)
    return lc.deviation(zm, target, lo[0], hi[0], lo[1], hi[1]), zm


# The hole cycles, rest machining, pencil and adaptive raster live in ops.py and register themselves here. Imported
# last: ops imports this module, and when ops is imported first it updates GENERATORS itself.
from . import ops as _ops  # noqa: E402,F401
