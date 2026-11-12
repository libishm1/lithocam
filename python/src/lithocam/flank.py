# Copyright 2026 The lithocam authors. SPDX-License-Identifier: Apache-2.0
"""5-axis flank (swarf) milling of ruled faces: CL frames, material-removal simulation, and the robot path.

A cylindrical cutter of radius r lies with its axis on a ruling of the face, offset r along the outward normal, so
its side touches the face along the whole ruling (lean/Lithocam/WireTube.lean proves the planar case touches and
does not gouge; the same offset is used here). The cutter sweeps across the rulings; position and axis direction
both change: five axes. The spin about the tool axis is free and is the robot's redundancy.

A flute of length `flute` shorter than the ruling is handled in bands along the ruling, each band swept across
the face. The tool tip points along -axis, from the shank side (`shank_side` = the end of the ruling that is free
for the tool, found by lithocam.ruled).

Material removal is simulated with Manifold: the union of convex hulls of the tool cylinders at consecutive
rulings, subtracted from the stock (as in lithocam.meshsim; the hull of skew rulings over-cuts by a measured small
amount that shrinks with more rulings).
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import List, Sequence

import numpy as np


@dataclass
class FlankFrame:
    tip: np.ndarray      # tool tip centre (model units)
    axis: np.ndarray     # unit tool axis, from tip towards the shank
    band: int
    step: int


def order_rulings(rulings):
    """Rulings sorted along the sweep and oriented consistently: arrays P0, P1, N."""
    P0 = np.array([r.p0 for r in rulings], float)
    P1 = np.array([r.p1 for r in rulings], float)
    N = np.array([r.normal for r in rulings], float)
    mids = (P0 + P1) / 2
    c = mids - mids.mean(axis=0)
    order = np.argsort(c @ np.linalg.svd(c, full_matrices=False)[2][0])
    P0, P1, N = P0[order], P1[order], N[order]
    flip = np.einsum("ij,j->i", P1 - P0, P1[0] - P0[0]) < 0
    P0[flip], P1[flip] = P1[flip].copy(), P0[flip].copy()
    return P0, P1, N


def flank_frames(rulings, tool_radius: float, flute: float, overlap: float = 0.1, shank_at_p1: bool = True,
                 layers: Sequence[float] = (0.0,)) -> List[FlankFrame]:
    """CL frames layer by layer (extra radial offsets, outermost first; 0 is the finishing pass), band by band
    along the rulings, zig-zag across the face. Outer layers extend past the ruling ends by their offset so the
    corners of a box blank are cleared too."""
    P0, P1, N = order_rulings(rulings)
    frames = []
    band_id = 0
    for layer in sorted(layers, reverse=True):
        A = P0 - (P1 - P0) / np.linalg.norm(P1 - P0, axis=1)[:, None] * layer
        B = P1 + (P1 - P0) / np.linalg.norm(P1 - P0, axis=1)[:, None] * layer
        L = float(np.mean(np.linalg.norm(B - A, axis=1)))
        nb = max(1, int(np.ceil((L - overlap * flute) / ((1 - overlap) * flute))))
        for b in range(nb):
            s0 = min(b * (1 - overlap) * flute, max(0.0, L - flute)) / L   # band start along the ruling (0..1)
            idx = range(len(A)) if b % 2 == 0 else range(len(A) - 1, -1, -1)
            for k, i in enumerate(idx):
                a, c, n = A[i], B[i], N[i]
                d = (c - a) / np.linalg.norm(c - a)
                start = a + (c - a) * s0 + n * (tool_radius + layer)
                if shank_at_p1:
                    tip, axis = start, d                  # tip at the band start, shank towards the far end
                else:
                    tip, axis = start + d * flute, -d
                frames.append(FlankFrame(tip, axis, band_id, k))
            band_id += 1
    return frames


def stock_depth(rulings, lo, hi) -> float:
    """Largest distance from the face to the stock box boundary along the outward normals (model units)."""
    best = 0.0
    for r in rulings:
        for p in (np.asarray(r.p0, float), np.asarray(r.p1, float), (np.asarray(r.p0) + np.asarray(r.p1)) / 2):
            n = np.asarray(r.normal, float)
            t = np.inf
            for k in range(3):
                if abs(n[k]) > 1e-12:
                    t = min(t, ((hi[k] if n[k] > 0 else lo[k]) - p[k]) / n[k])
            best = max(best, t if np.isfinite(t) else 0.0)
    return best


def simulate(stock, target, frames: Sequence[FlankFrame], tool_radius: float, flute: float, segments: int = 40):
    """Remove the flank sweeps from a Manifold stock. Returns (new stock, removed volume, gouge volume)."""
    import manifold3d as mf
    from .meshsim import wire_cylinder

    hulls = []
    by_band = {}
    for f in frames:
        by_band.setdefault(f.band, []).append(f)
    for fs in by_band.values():
        cyl = [wire_cylinder(f.tip, f.tip + f.axis * flute, tool_radius, segments) for f in fs]
        hulls += [mf.Manifold.batch_hull([cyl[i], cyl[i + 1]]) for i in range(len(cyl) - 1)]
    sweep = mf.Manifold.batch_boolean(hulls, mf.OpType.Add)
    new = stock - sweep
    return new, stock.volume() - new.volume(), (target - new).volume()
