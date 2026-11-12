# Copyright 2026 The lithocam authors. SPDX-License-Identifier: Apache-2.0
"""Write a drop-cutter test case: a wavy terrain mesh with ridges, plus query points. Usage: make_case.py outdir [seed]"""
import math
import random
import sys
from pathlib import Path

out = Path(sys.argv[1])
out.mkdir(parents=True, exist_ok=True)
rnd = random.Random(int(sys.argv[2]) if len(sys.argv) > 2 else 7)
G, S = 40, 2.5


def h(i, j):
    x, y = i * S, j * S
    return 6 * math.sin(0.11 * x) * math.cos(0.07 * y) + (4.0 if 40 < x < 46 else 0.0) + 0.02 * x


tris = []
for i in range(G):
    for j in range(G):
        p = [(i * S, j * S, h(i, j)), ((i + 1) * S, j * S, h(i + 1, j)), ((i + 1) * S, (j + 1) * S, h(i + 1, j + 1)),
             (i * S, (j + 1) * S, h(i, j + 1))]
        tris.append((p[0], p[1], p[2]))
        tris.append((p[0], p[2], p[3]))
for _ in range(40):            # loose triangles above the terrain: spikes and slanted plates
    c = (rnd.uniform(5, 95), rnd.uniform(5, 95), rnd.uniform(5, 15))
    tris.append(tuple((c[0] + rnd.uniform(-6, 6), c[1] + rnd.uniform(-6, 6), c[2] + rnd.uniform(-4, 4))
                      for _ in range(3)))
with open(out / "tris.txt", "w") as f:
    for t in tris:
        f.write(" ".join("{:.9f}".format(v) for p in t for v in p) + "\n")
with open(out / "points.txt", "w") as f:
    for _ in range(3000):
        f.write("{:.9f} {:.9f}\n".format(rnd.uniform(3, 97), rnd.uniform(3, 97)))
print(len(tris), "triangles, 3000 points ->", out)
