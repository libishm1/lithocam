# Copyright 2026 The lithocam authors. SPDX-License-Identifier: Apache-2.0
"""Data for the docs GIFs (run with a Python that has OCP and the _lithocam module):

    python docs/media/make_gif_data.py <dir with _lithocam module> <out dir>

Writes ruled_<part>.json (per-face triangles, labels, rulings), wire_<part>.json (stage meshes of the wire
simulation) and mill_tread.npz (Z-map snapshots of a finishing pass).
"""
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "python" / "src"))
sys.path.insert(0, sys.argv[1])
OUT = Path(sys.argv[2])
OUT.mkdir(parents=True, exist_ok=True)

import _lithocam as lc  # noqa: E402
from lithocam import ruled, wiresim  # noqa: E402
from lithocam.mesh import triangles_from_shape  # noqa: E402

DATA = ROOT / "python" / "tests" / "data"

# 0) a demonstration block with features big enough to see in a GIF (build123d)
from build123d import Align, Box, Cylinder, Pos, chamfer, export_step, Axis  # noqa: E402
MIN = (Align.MIN, Align.MIN, Align.MIN)
blk = Box(300, 200, 60, align=MIN)
blk = chamfer(blk.edges().filter_by(Axis.X).sort_by(Axis.Z)[-1], 12)            # top front... one top x-edge
blk -= Pos(80, 100, 20) * Cylinder(20, 41, align=(Align.CENTER, Align.CENTER, Align.MIN))   # blind hole
blk -= Pos(220, 100, -1) * Cylinder(30, 62, align=(Align.CENTER, Align.CENTER, Align.MIN))  # through hole
blk -= Pos(40, 160, 40) * Box(130, 8, 21, align=MIN)                                        # narrow slot
demo = OUT / "label_demo.step"
export_step(blk, str(demo))

# 1) ruled labels with rulings, per face
for name, path in (("twisted_column", DATA / "step_ruled" / "twisted_column.step"),
                   ("label_demo", demo)):
    shape = ruled.read_step(str(path))
    rep = ruled.analyse_shape(shape)
    faces = ruled.faces_of(shape)
    items = []
    for f in rep.faces:
        tris = triangles_from_shape(faces[f.index], deflection=0.002 * rep.scale)
        items.append({"label": f.label, "tris": tris, "rulings": [[r.p0, r.p1] for r in f.rulings]})
    (OUT / "ruled_{}.json".format(name)).write_text(json.dumps(items))
    print("ruled", name, rep.counts())

# 2) wire simulation stages
for name in ("window_sill", "cladding_panel"):
    shape = ruled.read_step(str(DATA / "step" / (name + ".step")))
    rep = wiresim.simulate(shape, margin=40.0, record=True)
    scale = ruled.analyse_shape(shape).scale
    stages = []
    for label, stock, waste in rep.stages:
        stages.append({"label": label, "stock": triangles_from_shape(stock, 0.003 * scale),
                       "waste": triangles_from_shape(waste, 0.003 * scale) if waste is not None else []})
    target = triangles_from_shape(shape, 0.003 * scale)
    (OUT / "wire_{}.json".format(name)).write_text(json.dumps({
        "stages": stages, "target": target, "kerf": rep.kerf_volume, "waste": rep.waste_volume,
        "to_mill": rep.to_mill_volume, "stock": rep.stock_volume, "target_volume": rep.target_volume}))
    print("wire", name, len(stages), "stages")

# 3) milling: Z-map snapshots while a ball finishes part of the stair tread
tris = triangles_from_shape(ruled.read_step(str(DATA / "step" / "stair_tread.step")), 0.02)


def P(p):
    q = lc.P3()
    q.x, q.y, q.z = p
    return q


def T(t):
    r = lc.Tri()
    r.a, r.b, r.c = P(t[0]), P(t[1]), P(t[2])
    return r


mesh = lc.TriMesh([T(t) for t in tris], 0.0)
cut = lc.Cutter()
cut.kind, cut.radius = lc.CutterKind.Ball, 3.0
pp = lc.PathParams()
pp.max_step, pp.min_step, pp.tol = 2.0, 0.02, 0.005
X0, X1, Y0, Y1 = 30.0, 90.0, 10.0, 60.0
so = lc.max_stepover(3.0, 0.2)
passes = lc.zigzag(cut, mesh, X0 - 4, X1 + 4, Y0 - 4, Y1 + 4, so, pp)
stock = lc.make_stock(X0, X1, Y0, Y1, 0.15, 41.0)
snaps = []
step = max(1, len(passes) // 24)
for k, p in enumerate(passes):
    lc.cut_passes(stock, cut, [p])
    if k % step == 0 or k == len(passes) - 1:
        snaps.append(np.array(stock.z).reshape(stock.ny, stock.nx).copy())
target = np.array(lc.target_heights(mesh, stock).z).reshape(stock.ny, stock.nx)
np.savez_compressed(OUT / "mill_tread.npz", snaps=np.array(snaps), target=target,
                    extent=np.array([X0, X0 + (stock.nx - 1) * stock.cell, Y0, Y0 + (stock.ny - 1) * stock.cell]),
                    passes=len(passes), stepover=so)
print("mill", len(snaps), "snapshots of", len(passes), "passes")
