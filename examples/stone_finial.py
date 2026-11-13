# Copyright 2026 The lithocam authors. SPDX-License-Identifier: Apache-2.0
"""A stone finial made by lithocam: two-view diamond-wire blank, 3+2 indexed roughing and 5-setup ball surfacing,
the same pipeline as examples/bunny_wire_and_mill.py on a part that ships under Apache-2.0.

    python examples/stone_finial.py OUT_DIR [--cuts twoview|hull] [--robots ur10e,kr120] [--run]

The part is generated here, not stored: a square two-tier plinth, a round base moulding, a five-lobed shaft that
swells like a baluster and twists 90 degrees over its height, a bead, a neck and a ball. Units mm, z up, the plinth
on z = 0. Every ring of the surface is star-shaped around the z axis and the profile never turns back on itself,
so the mesh is closed and free of self-intersections by construction; manifold3d confirms it before anything is
written.

Writes to OUT_DIR:
  finial.stl                 the part (binary STL, written at run time)
  finial_cuts_<robot>.json   the wire cuts as ruled faces
  tools.csv                  the cutters and the wire (example sizes)
  finial_<robot>.lcs.json    studio stations: program WireBlank, then Mill
  finial_wire_stock.stl      the stock after the wire cuts (Manifold)
With --run it also expands and posts both programs (Tesseract when installed); planning can take minutes.

Open a station:  python -m lithocam.studio OUT_DIR/finial_ur10e.lcs.json
"""
import argparse
import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "python" / "src"))
sys.path.insert(0, str(HERE))

import bunny_wire_and_mill as pipeline  # noqa: E402  (shared wire-blank and station code)

N_AROUND = 240          # samples per ring: a multiple of 8 (square corners) and of LOBES
LOBES = 5


def _smooth(t):
    t = min(max(t, 0.0), 1.0)
    return t * t * (3 - 2 * t)


def finial_rings():
    """The profile as a list of rings (z, shape, radius, lobe amplitude, twist in radians). shape 'square': radius
    is the half side; 'round': the mean radius, modulated by the lobes."""
    R = []
    # plinth: two square tiers with a 5 mm ledge
    R += [(0.0, "square", 55.0, 0.0, 0.0), (26.0, "square", 55.0, 0.0, 0.0),
          (26.0, "square", 50.0, 0.0, 0.0), (34.0, "square", 50.0, 0.0, 0.0)]
    # round base moulding: a torus bead from r 42 to r 40, peak about 46
    for k in range(17):
        t = k / 16
        R.append((34.0 + 12.0 * t, "round", 40.0 + 2.0 * (1 - t) + 5.0 * math.sin(math.pi * t), 0.0, 0.0))
    # cove into the shaft
    for k in range(1, 9):
        t = k / 8
        R.append((46.0 + 6.0 * t, "round", 40.0 - 10.0 * _smooth(t), 0.0, 0.0))
    # lobed, twisted baluster shaft
    z0, z1, twist = 52.0, 170.0, math.radians(90.0)
    n = 80
    for k in range(1, n + 1):
        t = k / n
        r = 30.0 + 9.0 * math.sin(math.pi * t ** 0.8) - 8.0 * t
        a = 0.14 * _smooth(t / 0.08) * _smooth((1 - t) / 0.08)
        R.append((z0 + (z1 - z0) * t, "round", r, a, twist * t))
    # bead
    for k in range(1, 9):
        t = k / 8
        R.append((170.0 + 6.0 * t, "round", 22.0 + 3.0 * math.sin(math.pi * t), 0.0, twist))
    # neck
    for k in range(1, 7):
        t = k / 6
        R.append((176.0 + 6.0 * t, "round", 22.0 - 7.0 * _smooth(t), 0.0, twist))
    # ball, radius 24, meeting the neck at r = 15
    rb, r_neck = 24.0, 15.0
    zc = 182.0 + math.sqrt(rb * rb - r_neck * r_neck)
    p0 = -math.acos(r_neck / rb)
    m = 40
    for k in range(1, m):
        p = p0 + (math.pi / 2 - p0) * k / m
        R.append((zc + rb * math.sin(p), "round", rb * math.cos(p), 0.0, twist))
    return R, zc + rb


def finial_mesh():
    """Vertices (mm) and outward triangles of the finial."""
    rings, z_top = finial_rings()
    th = 2 * np.pi * np.arange(N_AROUND) / N_AROUND
    c, s = np.cos(th), np.sin(th)
    V = [np.array([[0.0, 0.0, 0.0]])]
    for z, shape, r, a, tw in rings:
        if shape == "square":
            rr = r / np.maximum(np.abs(c), np.abs(s))
        else:
            rr = r * (1 + a * np.cos(LOBES * (th - tw)))
        V.append(np.c_[rr * c, rr * s, np.full(N_AROUND, z)])
    V.append(np.array([[0.0, 0.0, z_top]]))
    V = np.vstack(V)
    n, nr = N_AROUND, len(rings)
    ring = lambda i, j: 1 + i * n + (j % n)
    F = [(0, ring(0, j + 1), ring(0, j)) for j in range(n)]                       # bottom, facing down
    for i in range(nr - 1):
        for j in range(n):
            a, b, cc, d = ring(i, j), ring(i, j + 1), ring(i + 1, j + 1), ring(i + 1, j)
            F += [(a, b, cc), (a, cc, d)]
    top = len(V) - 1
    F += [(top, ring(nr - 1, j), ring(nr - 1, j + 1)) for j in range(n)]          # top, facing up
    return V, np.asarray(F, dtype=np.int64)


def main():
    ap = argparse.ArgumentParser(description="Procedural stone finial: wire blank, then 3+2 milling.")
    ap.add_argument("out")
    ap.add_argument("--cuts", choices=["twoview", "hull"], default="twoview",
                    help="twoview: side and front silhouette profiles (default); hull: planes on the convex hull")
    ap.add_argument("--robots", default="ur10e,kr120", help="comma-separated: ur10e, kr120")
    ap.add_argument("--run", action="store_true", help="also expand, plan and post the programs")
    a = ap.parse_args()
    robots = [r.strip() for r in a.robots.split(",") if r.strip()]
    bad = [r for r in robots if r not in pipeline.CELLS]
    if bad:
        ap.error("unknown robot(s): {}".format(", ".join(bad)))

    from lithocam import meshfix
    V, F = finial_mesh()
    rep = meshfix.check_closed(V, F)
    if not (rep["closed"] and rep["oriented"] and rep["manifold_ok"] and rep["euler"] == 2 and rep["volume"] > 0):
        raise SystemExit("finial mesh is not a closed manifold: {}".format(rep))
    size = V.max(0) - V.min(0)
    print("finial {:.1f} x {:.1f} x {:.1f} mm, {} triangles, volume {:.3f} dm3 (manifold3d {}, genus {})".format(
        *size, rep["faces"], rep["volume"] / 1e6, rep["manifold_status"], rep["manifold_genus"]))
    print("marble at 2.7 g/cm3: {:.1f} kg".format(rep["volume"] / 1e3 * 2.7 / 1e3))
    pipeline.build_stations(V, F, a.out, "finial", a.cuts, robots=robots)
    if a.run:
        pipeline.run(Path(a.out), "finial", robots)


if __name__ == "__main__":
    main()
