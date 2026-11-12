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


GENERATORS = {"raster": raster, "rough_plane": rough_plane, "face": face}


def path_length(moves: List[Move]) -> Tuple[float, float]:
    """Feed and rapid lengths, mm."""
    feed = rapid = 0.0
    for (k0, p0), (k1, p1) in zip(moves, moves[1:]):
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
    for k, p in moves:
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
