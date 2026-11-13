# Copyright 2026 The lithocam authors. SPDX-License-Identifier: Apache-2.0
"""Toolpath generators the studio runs today, on the C++ kernel (_lithocam): face, plane (z-level) roughing and
raster finishing over a triangle mesh with any tool that has a drop-cutter model.

Units: millimetres, in the part frame (the work frame of the setup). A path is a list of moves; each move is
(kind, point) with kind "rapid" or "feed" and point the tool tip.

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


def _link(passes: List[np.ndarray], clear_z: float) -> List[Move]:
    """Rapid up to the clearance plane between passes, feed along each pass."""
    moves: List[Move] = []
    for ps in passes:
        if len(ps) == 0:
            continue
        moves.append(("rapid", np.array([ps[0, 0], ps[0, 1], clear_z])))
        moves += [("feed", p) for p in ps]
        moves.append(("rapid", np.array([ps[-1, 0], ps[-1, 1], clear_z])))
    return moves


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


def raster(tris, mesh, tool, stock, stepover=2.0, scallop=0.0, allowance=0.0, tolerance=0.01, clearance=20.0,
           **_) -> List[Move]:
    R = tool.diameter / 2
    if scallop > 0 and tool.kind == lc.ToolKind.BallEndMill:
        stepover = min(stepover, lc.max_stepover(R, scallop)) if stepover > 0 else lc.max_stepover(R, scallop)
    lo, hi = tris.reshape(-1, 3).min(0), tris.reshape(-1, 3).max(0)
    rect = (lo[0] - R, hi[0] + R, lo[1] - R, hi[1] + R)
    passes = _zigzag(tool, mesh, rect, max(stepover, 0.05), tolerance, lo[2])
    passes = [p + np.array([0, 0, allowance]) for p in passes]
    return _link(passes, stock["zmax"] + clearance)


def rough_plane(tris, mesh, tool, stock, stepdown=5.0, stepover=4.0, allowance=0.5, tolerance=0.01, clearance=20.0,
                **_) -> List[Move]:
    lo = tris.reshape(-1, 3).min(0)
    rect = (stock["xmin"], stock["xmax"], stock["ymin"], stock["ymax"])
    base = _zigzag(tool, mesh, rect, max(stepover, 0.05), tolerance, lo[2])
    top = stock["zmax"]
    levels = []
    z = top - stepdown
    while z > lo[2] - 1e-9:
        levels.append(z)
        z -= stepdown
    if not levels or levels[-1] > lo[2] + 1e-9:
        levels.append(lo[2])
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
               tolerance=0.01, clearance=20.0, axis_rate=2.0, **_) -> List[Move]:
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
    rect = (lo[0] - R, hi[0] + R, lo[1] - R, hi[1] + R)
    passes = _zigzag(tool, mesh, rect, max(stepover, 0.05), tolerance, lo[2])
    d = max(0.25, min(1.0, R / 4))
    floor = lo[2]

    def h(x, y):
        z = lc.drop_cutter(cutter, mesh, float(x), float(y))
        return z if math.isfinite(z) else floor

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
        moves += [("feed", t, a) for t, a in zip(tips, A)]
        moves.append(("rapid", np.array([tips[-1, 0], tips[-1, 1], clear_z]), zax))
    return moves


GENERATORS = {"raster": raster, "rough_plane": rough_plane, "face": face, "surfacing5": surfacing5}


def path_length(moves: List[Move]) -> Tuple[float, float]:
    """Feed and rapid lengths, mm."""
    feed = rapid = 0.0
    for (k0, p0, *_), (k1, p1, *_) in zip(moves, moves[1:]):
        d = float(np.linalg.norm(p1 - p0))
        if k1 == "feed" and k0 == "feed":
            feed += d
        else:
            rapid += d
    return feed, rapid


def zmap_check(tris, mesh, tool, moves: List[Move], stock, cell: float = 1.0):
    """Cut the stock along the feed moves in a Z-map; return (deviation stats over the part's xy box, zmap)."""
    zm = lc.make_stock(stock["xmin"], stock["xmax"], stock["ymin"], stock["ymax"], cell, stock["zmax"])
    passes, cur = [], []
    for k, p, *_ in moves:
        if k == "feed":
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
