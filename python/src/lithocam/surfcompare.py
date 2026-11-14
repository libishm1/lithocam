# Copyright 2026 The lithocam authors. SPDX-License-Identifier: Apache-2.0
"""Compare a machined (or scanned) surface with the design: ICP registration and 3D distances, numpy only.

* `GridNN`: nearest neighbours by a hashed uniform grid. A neighbour closer than the cell size is exact (every point
  within that distance lies in the 27 cells around the query); queries whose best distance is larger are searched
  again on a grid twice as coarse, until the best distance is within the cell size, so every answer is exact.
* `distances(A, B, nB)`: per point of A, the distance to surface B: the nearest sample of B refined to the tangent
  plane of B there (point to plane, signed along B's normal: + outside / above the design, - inside / below). The
  plane step removes the sampling error of the nearest point (half the sample spacing) to second order in the
  curvature.
* `icp(A, B, nB)`: point-to-plane ICP (Chen and Medioni 1992, doi 10.1016/0262-8856(92)90066-C; linearised small
  rotation, Low 2004 TR04-004) with trimming of the worst pairs (Chetverikov et al. 2002, doi
  10.1109/ICPR.2002.1047431). It separates a rigid misplacement from the shape error.
* `hausdorff(A, nA, B, nB)`: directed distances both ways, the symmetric Hausdorff distance (the larger directed
  maximum) and robust summaries (mean, RMS, 95th and 99th percentiles), as in mesh comparison practice (Cignon et
  al. 1998, Metro, doi 10.1111/1467-8659.00236).

Height fields (Z-map runs, relief designs) become point clouds with normals through `heightfield_points`.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, Optional, Tuple

import numpy as np


# ---------------------------------------------------------------- nearest neighbours
class GridNN:
    def __init__(self, P: np.ndarray, cell: float):
        self.P = np.asarray(P, dtype=float)
        self.cell = float(cell)
        self.o = self.P.min(0) - 2 * self.cell
        k = self._keys(np.floor((self.P - self.o) / self.cell).astype(np.int64))
        order = np.argsort(k, kind="stable")
        self.order = order
        ks = k[order]
        self.ukeys, self.start, self.count = np.unique(ks, return_index=True, return_counts=True)
        self._coarse: Optional[GridNN] = None

    @staticmethod
    def _keys(c: np.ndarray) -> np.ndarray:
        c = c + (1 << 20)
        return (c[:, 0] << 42) | (c[:, 1] << 21) | c[:, 2]

    def query(self, Q: np.ndarray, chunk: int = 200000) -> Tuple[np.ndarray, np.ndarray]:
        """(distance, index into P) of the nearest point of P for every row of Q; exact."""
        Q = np.asarray(Q, dtype=float)
        d = np.full(len(Q), np.inf)
        idx = np.full(len(Q), -1, dtype=np.int64)
        for a in range(0, len(Q), chunk):
            dd, ii = self._query_ring(Q[a: a + chunk])
            d[a: a + chunk], idx[a: a + chunk] = dd, ii
        far = d > self.cell
        if far.any():
            if self._coarse is None:
                self._coarse = GridNN(self.P, self.cell * 2.0)
            dd, ii = self._coarse.query(Q[far], chunk)
            d[far], idx[far] = dd, ii
        return d, idx

    def _query_ring(self, Q: np.ndarray):
        c = np.floor((Q - self.o) / self.cell).astype(np.int64)
        best = np.full(len(Q), np.inf)
        bi = np.full(len(Q), -1, dtype=np.int64)
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for dz in (-1, 0, 1):
                    k = self._keys(c + np.array([dx, dy, dz]))
                    pos = np.searchsorted(self.ukeys, k)
                    pos = np.minimum(pos, len(self.ukeys) - 1)
                    hit = self.ukeys[pos] == k
                    if not hit.any():
                        continue
                    qi = np.flatnonzero(hit)
                    st, cnt = self.start[pos[qi]], self.count[pos[qi]]
                    for j in range(int(cnt.max())):
                        m = cnt > j
                        q2 = qi[m]
                        pi = self.order[st[m] + j]
                        dist = np.linalg.norm(self.P[pi] - Q[q2], axis=1)
                        better = dist < best[q2]
                        best[q2[better]] = dist[better]
                        bi[q2[better]] = pi[better]
        return best, bi


# ---------------------------------------------------------------- surfaces
def heightfield_points(H: np.ndarray, cell: float, x0: float = 0.0, y0: float = 0.0, mask=None,
                       stride: int = 1) -> Tuple[np.ndarray, np.ndarray]:
    """(points (n, 3), unit normals (n, 3)) of a height field H[j, i] at (x0 + i*cell, y0 + j*cell); cells that are
    not finite or outside `mask` are left out."""
    H = np.asarray(H, dtype=float)
    gy, gx = np.gradient(np.where(np.isfinite(H), H, np.nan), cell)
    ny_, nx_ = H.shape
    X, Y = np.meshgrid(x0 + np.arange(nx_) * cell, y0 + np.arange(ny_) * cell)
    ok = np.isfinite(H) & np.isfinite(gx) & np.isfinite(gy)
    if mask is not None:
        ok &= mask
    if stride > 1:
        s = np.zeros_like(ok)
        s[::stride, ::stride] = True
        ok &= s
    N = np.dstack([-gx, -gy, np.ones_like(H)])[ok]
    N /= np.linalg.norm(N, axis=1, keepdims=True)
    return np.column_stack([X[ok], Y[ok], H[ok]]), N


def distances(A: np.ndarray, B: np.ndarray, nB: np.ndarray, nn: Optional[GridNN] = None,
              cell: Optional[float] = None, plane: bool = True) -> Tuple[np.ndarray, np.ndarray]:
    """(signed distance, unsigned distance) from every point of A to surface B (samples B with unit normals nB).
    Signed: along B's normal at the nearest sample (+ above or outside). With plane=True the unsigned distance is the
    point-to-plane distance at the nearest sample (never more than the point-to-point distance)."""
    if nn is None:
        nn = GridNN(B, cell or _spacing(B))
    d, i = nn.query(A)
    s = np.einsum("ij,ij->i", A - B[i], nB[i])
    u = np.minimum(np.abs(s), d) if plane else d
    return np.sign(s) * u, u


def _spacing(P: np.ndarray) -> float:
    ext = np.ptp(P, axis=0)
    area = max(ext[0] * ext[1], ext[0] * ext[2], ext[1] * ext[2], 1e-9)
    return max(2.0 * math.sqrt(area / max(len(P), 1)), 1e-6)


def _rot(w: np.ndarray) -> np.ndarray:
    a = float(np.linalg.norm(w))
    if a < 1e-15:
        return np.eye(3)
    k = w / a
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(a) * K + (1 - math.cos(a)) * K @ K


@dataclass
class IcpResult:
    T: np.ndarray                    # 4x4: A moved by T lies on B
    rms_before: float
    rms_after: float
    iterations: int
    converged: bool
    translation_mm: float
    rotation_deg: float
    history: list = field(default_factory=list)


def icp(A: np.ndarray, B: np.ndarray, nB: np.ndarray, max_iter: int = 50, trim: float = 0.9, tol: float = 1e-7,
        samples: int = 60000, seed: int = 0, cell: Optional[float] = None) -> IcpResult:
    """Point-to-plane ICP of A onto surface B. Each iteration pairs a random subsample of A with the nearest samples
    of B, keeps the `trim` fraction with the smallest residuals and solves the linearised 6x6 system."""
    rng = np.random.default_rng(seed)
    sub = A if len(A) <= samples else A[rng.choice(len(A), samples, replace=False)]
    nn = GridNN(B, cell or _spacing(B))
    T = np.eye(4)
    hist = []
    converged = False
    for it in range(max_iter):
        P = sub @ T[:3, :3].T + T[:3, 3]
        _, i = nn.query(P)
        q, n = B[i], nB[i]
        r = np.einsum("ij,ij->i", P - q, n)
        keep = np.argsort(np.abs(r))[: max(6, int(trim * len(r)))]
        P, n, r = P[keep], n[keep], r[keep]
        hist.append(float(np.sqrt(np.mean(r * r))))
        J = np.hstack([np.cross(P, n), n])
        x = np.linalg.lstsq(J, -r, rcond=None)[0]
        dT = np.eye(4)
        dT[:3, :3] = _rot(x[:3])
        dT[:3, 3] = x[3:]
        T = dT @ T
        if np.linalg.norm(x) < tol:
            converged = True
            break
    P = sub @ T[:3, :3].T + T[:3, 3]
    _, u_after = distances(P, B, nB, nn)
    _, u_before = distances(sub, B, nB, nn)
    ang = math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(T[:3, :3]) - 1) / 2))))
    return IcpResult(T, float(np.sqrt(np.mean(u_before ** 2))), float(np.sqrt(np.mean(u_after ** 2))), it + 1,
                     converged, float(np.linalg.norm(T[:3, 3])), ang, hist)


def _summary(u: np.ndarray, s: Optional[np.ndarray] = None) -> Dict[str, float]:
    out = dict(max=float(u.max()), mean=float(u.mean()), rms=float(np.sqrt(np.mean(u * u))),
               p95=float(np.percentile(u, 95)), p99=float(np.percentile(u, 99)), n=int(len(u)))
    if s is not None:
        out.update(max_above=float(max(s.max(), 0.0)), max_below=float(max(-s.min(), 0.0)))
    return out


def hausdorff(A: np.ndarray, nA: np.ndarray, B: np.ndarray, nB: np.ndarray) -> Dict[str, object]:
    """Directed distances A->B and B->A (point to plane, see `distances`), their summaries and the symmetric
    Hausdorff distance max(max A->B, max B->A). A is the result (machined or scanned), B the design."""
    sAB, uAB = distances(A, B, nB)
    sBA, uBA = distances(B, A, nA)
    return {"A_to_B": _summary(uAB, sAB), "B_to_A": _summary(uBA, -sBA), "hausdorff": float(max(uAB.max(), uBA.max())),
            "signed_A_to_B": sAB}


def main(argv=None):
    """python -m lithocam.surfcompare RUN.npz [--step K] [--window x0,y0,x1,y1] [--stride S] [--no-icp] [--json OUT]
    RUN.npz: a Z-map run saved with cell, x0, y0, target and heights (studio.process.zmap_stock)."""
    import argparse
    import json
    ap = argparse.ArgumentParser(description=main.__doc__)
    ap.add_argument("run")
    ap.add_argument("--step", type=int, default=-1, help="index of the saved step (default the last)")
    ap.add_argument("--window", default="", help="x0,y0,x1,y1 mm")
    ap.add_argument("--stride", type=int, default=1)
    ap.add_argument("--no-icp", action="store_true")
    ap.add_argument("--json", default="")
    a = ap.parse_args(argv)
    z = np.load(a.run, allow_pickle=False)
    cell, x0, y0 = float(z["cell"]), float(z["x0"]), float(z["y0"])
    H, Tg = z["heights"][a.step].astype(float), z["target"].astype(float)
    mask = np.isfinite(H) & np.isfinite(Tg)
    if a.window:
        w = [float(v) for v in a.window.split(",")]
        X, Y = np.meshgrid(x0 + np.arange(H.shape[1]) * cell, y0 + np.arange(H.shape[0]) * cell)
        mask &= (X >= w[0]) & (X <= w[2]) & (Y >= w[1]) & (Y <= w[3])
    A, nA = heightfield_points(H, cell, x0, y0, mask, a.stride)
    B, nB = heightfield_points(Tg, cell, x0, y0, mask, a.stride)
    vert = (H - Tg)[mask]
    out = {"points": [len(A), len(B)], "vertical": _summary(np.abs(vert), vert)}
    h = hausdorff(A, nA, B, nB)
    out["as_machined"] = {k: v for k, v in h.items() if k != "signed_A_to_B"}
    if not a.no_icp:
        r = icp(A, B, nB)
        A2 = A @ r.T[:3, :3].T + r.T[:3, 3]
        nA2 = nA @ r.T[:3, :3].T
        h2 = hausdorff(A2, nA2, B, nB)
        out["icp"] = dict(iterations=r.iterations, converged=r.converged, translation_mm=r.translation_mm,
                          rotation_deg=r.rotation_deg, rms_before=r.rms_before, rms_after=r.rms_after)
        out["after_icp"] = {k: v for k, v in h2.items() if k != "signed_A_to_B"}
    print(json.dumps(out, indent=1))
    if a.json:
        with open(a.json, "w", encoding="utf-8") as f:
            json.dump(out, f, indent=1)


if __name__ == "__main__":
    main()
