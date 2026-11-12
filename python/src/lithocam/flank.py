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


def flank_frames(rulings, tool_radius: float, flute: float, overlap: float = 0.1, shank_at_p1: bool = True
                 ) -> List[FlankFrame]:
    """CL frames band by band: the tool axis on the offset ruling, the tip inside each band, zig-zag across."""
    P0, P1, N = order_rulings(rulings)
    L = float(np.mean(np.linalg.norm(P1 - P0, axis=1)))
    nb = max(1, int(np.ceil((L - overlap * flute) / ((1 - overlap) * flute))))
    frames = []
    for b in range(nb):
        s0 = min(b * (1 - overlap) * flute, max(0.0, L - flute)) / L       # band start along the ruling (0..1)
        idx = range(len(P0)) if b % 2 == 0 else range(len(P0) - 1, -1, -1)
        for k, i in enumerate(idx):
            a, c, n = P0[i], P1[i], N[i]
            d = (c - a) / np.linalg.norm(c - a)
            off = n * tool_radius
            start = a + (c - a) * s0 + off
            if shank_at_p1:
                tip, axis = start, d                      # tip at the band start, shank towards p1
            else:
                tip, axis = start + d * flute, -d
            frames.append(FlankFrame(tip, axis, b, k))
    return frames


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
