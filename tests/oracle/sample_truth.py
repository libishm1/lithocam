# Copyright 2026 The lithocam authors. SPDX-License-Identifier: Apache-2.0
"""Independent dense-sampling drop-cutter height for chosen query points (numpy), to referee disagreements.

    python sample_truth.py tris.txt points.txt cone 3 0.5235987755982988 2131 811 [...]
    python sample_truth.py tris.txt points.txt ball 3 0 17

Samples every triangle under the cutter on an n x n barycentric grid (n = 400) and prints the maximum of
p.z - f(d). The result is a lower bound that converges to the exact height as n grows.
"""
import math
import sys

import numpy as np

tris = np.loadtxt(sys.argv[1]).reshape(-1, 3, 3)
pts = np.loadtxt(sys.argv[2])
kind, R, extra = sys.argv[3], float(sys.argv[4]), float(sys.argv[5])
idx = [int(a) for a in sys.argv[6:]]


def f(d):
    if kind == "flat":
        return np.zeros_like(d)
    if kind == "ball":
        return R - np.sqrt(np.maximum(0, R * R - d * d))
    if kind == "bull":
        a = R - extra
        e = np.maximum(0, d - a)
        return extra - np.sqrt(np.maximum(0, extra * extra - e * e))
    return d / math.tan(extra)


n = 400
u, v = np.meshgrid(np.linspace(0, 1, n + 1), np.linspace(0, 1, n + 1))
m = (u + v) <= 1
u, v = u[m], v[m]
for i in idx:
    x, y = pts[i]
    best = -math.inf
    for t in tris:
        if np.min(np.hypot(t[:, 0] - x, t[:, 1] - y)) > R + np.max(np.ptp(t[:, :2], axis=0)):
            continue
        P = t[0] + np.outer(u, t[1] - t[0]) + np.outer(v, t[2] - t[0])
        d = np.hypot(P[:, 0] - x, P[:, 1] - y)
        k = d <= R
        if k.any():
            best = max(best, float(np.max(P[k, 2] - f(d[k]))))
    print("point {} sampled {:.6f}".format(i, best))
