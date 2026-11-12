# Copyright 2026 The lithocam authors. SPDX-License-Identifier: Apache-2.0
"""Example: 2.5-axis pocketing with z-level roughing and a finishing pass, verified in the Z-map simulation.

    python examples/pocket_2p5d.py <dir with the _lithocam module> <out dir>

Part (build123d): a 200 x 150 x 40 block with a 140 x 90 pocket (corner R15) 20 deep and a D30 island standing
10 mm proud of the pocket floor.

Roughing: flat end mill R5, step-down 5 mm, 0.3 mm allowance. At each level the tool tip runs at
max(level, drop-cutter height + allowance), so it never gouges; outside the pocket it rides on the block top.
Finishing: the same flat end mill, a zig-zag at the drop-cutter height (no allowance), stepover 8 mm.

Writes pocket.stl (part), pocket_stock.stl (simulated stock), pocket_dev.png, pocket.json.
"""
import json
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "python" / "src"))
sys.path.insert(0, sys.argv[1])
OUT = Path(sys.argv[2])
OUT.mkdir(parents=True, exist_ok=True)

import _lithocam as lc  # noqa: E402
from build123d import Align, Box, BuildSketch, Cylinder, Plane, Pos, RectangleRounded, export_step, extrude  # noqa: E402
from lithocam.mesh import triangles_from_shape, write_stl_ascii  # noqa: E402
from lithocam.ruled import read_step  # noqa: E402

# ---------------------------------------------------------------- part
MIN = (Align.MIN, Align.MIN, Align.MIN)
part = Box(200, 150, 40, align=MIN)
with BuildSketch(Plane.XY.offset(20)) as sk:
    RectangleRounded(140, 90, 15)
part -= Pos(100, 75, 0) * extrude(sk.sketch, amount=25)
part += Pos(130, 75, 20) * Cylinder(15, 10, align=(Align.CENTER, Align.CENTER, Align.MIN))
export_step(part, str(OUT / "pocket.step"))
tris = triangles_from_shape(read_step(str(OUT / "pocket.step")), deflection=0.02)
write_stl_ascii(tris, str(OUT / "pocket.stl"), "pocket")


def P(p):
    q = lc.P3()
    q.x, q.y, q.z = p
    return q


def T(t):
    r = lc.Tri()
    r.a, r.b, r.c = P(t[0]), P(t[1]), P(t[2])
    return r


mesh = lc.TriMesh([T(t) for t in tris], 0.0)
flat = lc.Cutter()
flat.kind, flat.radius = lc.CutterKind.Flat, 5.0
pp = lc.PathParams()
pp.max_step, pp.min_step, pp.tol, pp.floor = 2.0, 0.05, 0.01, 0.0
X0, X1, Y0, Y1 = 20.0, 180.0, 20.0, 130.0         # around the pocket


def clamp(passes, level, allowance):
    out = []
    for p in passes:
        q = []
        for c in p:
            n = lc.CLPoint()
            n.x, n.y = c.x, c.y
            n.z = max(level, c.z + allowance) if c.contact else max(level, 40.0)
            n.contact = True
            q.append(n)
        out.append(q)
    return out


def level_contours(cutter, mesh, z, x0, x1, y0, y1, step=0.5, tol=1e-4):
    """Tool-centre loops at height z: where the drop-cutter height crosses z on fine x- and y-fibres (bisection),
    chained into loops by nearest neighbour. A waterline by fibres, built on drop-cutter (milling kernel M4 seed)."""
    pts = []

    def h(x, y):
        return lc.drop_cutter(cutter, mesh, x, y)

    def scan(fixed, a0, a1, along_x):
        n = int((a1 - a0) / step) + 1
        prev = None
        for i in range(n + 1):
            a = a0 + i * step
            x, y = (a, fixed) if along_x else (fixed, a)
            below = h(x, y) <= z + tol
            if prev is not None and below != prev[1]:
                lo, hi = prev[0], a                      # bisection on the crossing
                for _ in range(40):
                    m = 0.5 * (lo + hi)
                    xm, ym = (m, fixed) if along_x else (fixed, m)
                    if (h(xm, ym) <= z + tol) == prev[1]:
                        lo = m
                    else:
                        hi = m
                c = 0.5 * (lo + hi)
                pts.append((c, fixed) if along_x else (fixed, c))
            prev = (a, below)

    for k in range(int((y1 - y0) / step) + 1):
        scan(y0 + k * step, x0, x1, True)
    for k in range(int((x1 - x0) / step) + 1):
        scan(x0 + k * step, y0, y1, False)
    pts = np.array(pts)
    loops, used = [], np.zeros(len(pts), bool)
    for s in range(len(pts)):
        if used[s]:
            continue
        loop, cur = [s], s
        used[s] = True
        while True:
            d = np.hypot(pts[:, 0] - pts[cur, 0], pts[:, 1] - pts[cur, 1])
            d[used] = np.inf
            j = int(np.argmin(d))
            if not np.isfinite(d[j]) or d[j] > 2.5 * step:
                break
            loop.append(j)
            used[j] = True
            cur = j
        if len(loop) > 8:
            closed = np.hypot(*(pts[loop[-1]] - pts[loop[0]])) <= 2.5 * step
            loops.append(pts[loop + [loop[0]]] if closed else pts[loop])   # never jump across to close
    return loops


def as_passes(loops, z):
    out = []
    for lp in loops:
        q = []
        for x, y in lp:
            c = lc.CLPoint()
            c.x, c.y, c.z, c.contact = float(x), float(y), z, True
            q.append(c)
        out.append(q)
    return out


t0 = time.time()
base = lc.zigzag(flat, mesh, X0, X1, Y0, Y1, 8.0, pp)           # drop-cutter heights along the raster
rough = []
for level in (35.0, 30.0, 25.0, 20.0):
    rough.append(clamp(base, level, 0.3))
finish = clamp(base, -1e9, 0.0)
# contour finishing at the floor and at the island top: the tool centre one radius off every wall
walls = level_contours(flat, mesh, 20.0, X0, X1, Y0, Y1)
contour = as_passes(walls, 20.0)
t1 = time.time()

stock = lc.make_stock(X0 - 10, X1 + 10, Y0 - 10, Y1 + 10, 0.25, 40.0)
snaps = []
for lv in rough:
    lc.cut_passes(stock, flat, lv)
    snaps.append(np.array(stock.z).reshape(stock.ny, stock.nx).copy())
lc.cut_passes(stock, flat, finish)
lc.cut_passes(stock, flat, contour)
t2 = time.time()
target = lc.target_heights(mesh, stock)
S = np.array(stock.z).reshape(stock.ny, stock.nx)
Tg = np.array(target.z).reshape(stock.ny, stock.nx)
dev = S - Tg
# cells on a vertical wall (target jumps by > 1 mm to a neighbour): their vertical ray reads the wall's top edge
# while the tool touches the wall itself, so they are excluded from the gouge measure and counted separately
jump = np.zeros_like(Tg, bool)
for ax in (0, 1):
    dz = np.abs(np.diff(Tg, axis=ax)) > 1.0
    if ax == 0:
        jump[1:, :] |= dz
        jump[:-1, :] |= dz
    else:
        jump[:, 1:] |= dz
        jump[:, :-1] |= dz
lc.write_stl(lc.zmap_mesh(stock, 0.0), str(OUT / "pocket_stock.stl"))
xs = stock.x0 + np.arange(stock.nx) * stock.cell
ys = stock.y0 + np.arange(stock.ny) * stock.cell
X, Y = np.meshgrid(xs, ys)
# floor region away from walls and island by more than the tool radius
floor = (np.abs(Tg - 20.0) < 1e-9) & (np.hypot(X - 130, Y - 75) > 15 + 5.5) & (np.abs(X - 100) < 70 - 5.5) & \
        (np.abs(Y - 75) < 45 - 5.5)
res = {"part": "200x150x40 block, 140x90 R15 pocket 20 deep, D30 island 10 high", "cutter": "flat R5",
       "rough_levels": [35, 30, 25, 20], "allowance": 0.3, "stepover": 8.0,
       "passes_per_level": len(base), "cl_points_per_level": sum(len(p) for p in base),
       "min_dev_off_walls": float(np.nanmin(dev[~jump])), "max_dev": float(np.nanmax(dev)),
       "wall_cells": int(jump.sum()), "wall_cells_below_target": int(((dev < -0.01) & jump).sum()),
       "floor_max_leftover": float(np.nanmax(dev[floor])), "floor_min": float(np.nanmin(dev[floor])),
       "after_roughing_floor_leftover": float(np.nanmax((snaps[-1] - Tg)[floor])),
       "contour_loops": len(walls), "contour_points": int(sum(len(w) for w in walls)),
       "floor_cells_uncut_gt_0p01": int(((S - Tg) > 0.01)[np.abs(Tg - 20) < 1e-9].sum()),
       "floor_cells": int((np.abs(Tg - 20) < 1e-9).sum()),
       "seconds": {"paths": round(t1 - t0, 2), "simulate": round(t2 - t1, 2)}}
(OUT / "pocket.json").write_text(json.dumps(res, indent=1))
np.savez_compressed(OUT / "pocket_sim.npz", snaps=np.array(snaps + [S]), target=Tg,
                    extent=np.array([xs[0], xs[-1], ys[0], ys[-1]]))
print(json.dumps(res, indent=1))
