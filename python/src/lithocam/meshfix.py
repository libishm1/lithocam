# Copyright 2026 The lithocam authors. SPDX-License-Identifier: Apache-2.0
"""Close the holes of a triangle mesh so that it bounds a solid.

    V, F = fill_holes(V, F)          # closed, consistently oriented, positive volume
    report = check_closed(V, F)      # edges, Euler characteristic, signed volume, manifold3d status

Scanned parts (the Stanford bunny, photogrammetry of a block) have open boundaries: the scanner never saw the
underside. The milling and wire steps need a solid. `fill_holes` finds every boundary loop and closes it:

- a loop that is close to planar is triangulated by ear clipping in its best-fit plane;
- a loop that is not planar, or whose projection is not a simple polygon, is closed by a fan to its centroid
  (one new vertex per loop).

The faces are first oriented consistently across shared edges, and the result is flipped if its volume is negative.
Vertices that no face uses are dropped, so the returned vertex indices can differ from the input.
"""
from __future__ import annotations

from collections import defaultdict, deque
from typing import Dict, List, Tuple

import numpy as np

__all__ = ["fill_holes", "boundary_loops", "orient_faces", "signed_volume", "edge_counts", "check_closed",
           "to_manifold"]


def _compact(V, F):
    used = np.unique(F)
    remap = -np.ones(len(V), dtype=np.int64)
    remap[used] = np.arange(len(used))
    return V[used], remap[F]


def edge_counts(F) -> Dict[Tuple[int, int], int]:
    """Number of faces on each undirected edge (key: (min, max) vertex index)."""
    F = np.asarray(F, dtype=np.int64)
    e = np.sort(np.concatenate([F[:, [0, 1]], F[:, [1, 2]], F[:, [2, 0]]]), axis=1)
    keys, counts = np.unique(e, axis=0, return_counts=True)
    return {(int(a), int(b)): int(c) for (a, b), c in zip(keys, counts)}


def orient_faces(F):
    """Flip faces so that every edge shared by two faces is used in opposite directions (breadth-first over each
    connected patch; the first face of a patch keeps its orientation). Edges with more than two faces are skipped."""
    F = np.array(F, dtype=np.int64, copy=True)
    nf = len(F)
    edge_faces = defaultdict(list)
    for f in range(nf):
        a, b, c = F[f]
        for u, v in ((a, b), (b, c), (c, a)):
            edge_faces[(u, v) if u < v else (v, u)].append(f)
    seen = np.zeros(nf, dtype=bool)
    for start in range(nf):
        if seen[start]:
            continue
        seen[start] = True
        q = deque([start])
        while q:
            f = q.popleft()
            a, b, c = F[f]
            for u, v in ((a, b), (b, c), (c, a)):
                fs = edge_faces[(u, v) if u < v else (v, u)]
                if len(fs) != 2:
                    continue
                g = fs[0] if fs[1] == f else fs[1]
                if seen[g]:
                    continue
                ga, gb, gc = F[g]
                if (u, v) in ((ga, gb), (gb, gc), (gc, ga)):     # same direction: g is flipped relative to f
                    F[g] = (ga, gc, gb)
                seen[g] = True
                q.append(g)
    return F


def boundary_loops(F) -> List[List[int]]:
    """Closed loops of boundary half-edges. Each loop lists vertices in the direction of its faces' half-edges
    (v[i] -> v[i+1] is an edge of an existing face). A vertex where two loops touch is split between them."""
    F = np.asarray(F, dtype=np.int64)
    half = set()
    for a, b, c in F:
        half.update(((int(a), int(b)), (int(b), int(c)), (int(c), int(a))))
    out_edges = defaultdict(list)
    for u, v in half:
        if (v, u) not in half:
            out_edges[u].append(v)
    loops = []
    for start in sorted(out_edges):
        while out_edges[start]:
            loop = [start]
            v = out_edges[start].pop()
            guard = 0
            while v != start:
                loop.append(v)
                if not out_edges[v]:
                    raise ValueError("open boundary chain at vertex {} (non-manifold input)".format(v))
                v = out_edges[v].pop()
                guard += 1
                if guard > len(half):
                    raise ValueError("boundary walk did not close")
            loops.append(_split_pinches(loop))
    return [l for group in loops for l in group]


def _split_pinches(loop):
    """Split a loop that passes a vertex twice into simple loops."""
    out, stack, pos = [], [], {}
    for v in loop:
        if v in pos:
            k = pos[v]
            out.append(stack[k:])
            for w in stack[k + 1:]:
                pos.pop(w, None)
            stack = stack[:k + 1]
        else:
            pos[v] = len(stack)
            stack.append(v)
    if len(stack) >= 3:
        out.append(stack)
    return [l for l in out if len(l) >= 3]


def _plane_frame(P):
    c = P.mean(0)
    _, s, vt = np.linalg.svd(P - c)
    return c, vt[0], vt[1], vt[2], s


def _ear_clip(Q, eps):
    """Ear clipping of a counter-clockwise simple polygon Q (n x 2). Returns triangles as index triples in the
    polygon's order, or None if no ear can be found (the projection is not simple). Among the valid ears it clips
    the one with the largest smallest angle, which keeps the triangles well shaped."""
    idx = list(range(len(Q)))
    tris = []

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    def min_angle(a, b, c):
        best = np.pi
        for p, q, r in ((a, b, c), (b, c, a), (c, a, b)):
            u, v = q - p, r - p
            nu, nv = np.linalg.norm(u), np.linalg.norm(v)
            if nu < 1e-300 or nv < 1e-300:
                return 0.0
            best = min(best, np.arccos(np.clip(u @ v / (nu * nv), -1.0, 1.0)))
        return best

    while len(idx) > 3:
        n = len(idx)
        best, best_k = -1.0, -1
        for k in range(n):
            i0, i1, i2 = idx[k - 1], idx[k], idx[(k + 1) % n]
            a, b, c = Q[i0], Q[i1], Q[i2]
            if cross(a, b, c) <= eps:                     # reflex or collinear corner
                continue
            ok = True
            for j in idx:
                if j in (i0, i1, i2):
                    continue
                p = Q[j]
                if cross(a, b, p) >= -eps and cross(b, c, p) >= -eps and cross(c, a, p) >= -eps:
                    ok = False                            # inside or on the candidate ear
                    break
            if not ok:
                continue
            score = min_angle(a, b, c) if n <= 400 else 1.0
            if score > best:
                best, best_k = score, k
                if n > 400:
                    break
        if best_k < 0:
            return None
        k = best_k
        tris.append((idx[k - 1], idx[k], idx[(k + 1) % n]))
        idx.pop(k)
    a, b, c = Q[idx[0]], Q[idx[1]], Q[idx[2]]
    if cross(a, b, c) <= eps:
        return None
    tris.append(tuple(idx))
    return tris


def _close_loop(V, loop, planar_tol):
    """Triangles (and new vertices) that close one boundary loop. The loop is reversed so that the new faces use
    each boundary edge in the opposite direction of the existing face."""
    R = list(reversed(loop))
    P = V[R]
    c, e1, e2, nrm, s = _plane_frame(P)
    size = max(float(s[0]), 1e-12) / np.sqrt(len(P))     # rms extent of the loop in its plane
    flat = float(np.max(np.abs((P - c) @ nrm)))
    if len(R) == 3:
        return [tuple(R)], []
    if flat <= planar_tol * size:
        Q = np.c_[(P - c) @ e1, (P - c) @ e2]
        area = 0.5 * float(np.sum(Q[:, 0] * np.roll(Q[:, 1], -1) - np.roll(Q[:, 0], -1) * Q[:, 1]))
        if area < 0:
            Q[:, 1] *= -1.0                               # view the plane from the other side: R is CCW
        scale = float(np.max(np.ptp(Q, axis=0)))
        tris = _ear_clip(Q, eps=1e-12 * scale * scale)
        if tris is not None:
            return [(R[i], R[j], R[k]) for i, j, k in tris], []
    return None, c                                        # fan to the centroid


def fill_holes(V, F, planar_tol: float = 0.35):
    """Close every boundary loop of the mesh (V: n x 3 floats, F: m x 3 vertex indices).

    `planar_tol` is the largest out-of-plane distance of a loop, relative to its rms in-plane extent, for which ear
    clipping is used; less planar loops get a centroid fan. Returns (V, F) as numpy arrays: closed (every edge on
    exactly two faces when the input is manifold), consistently oriented, outward normals (positive volume),
    without unused vertices."""
    V = np.asarray(V, dtype=np.float64)
    F = np.asarray(F, dtype=np.int64).reshape(-1, 3)
    F = F[(F[:, 0] != F[:, 1]) & (F[:, 1] != F[:, 2]) & (F[:, 2] != F[:, 0])]   # drop collapsed faces
    V, F = _compact(V, F)
    F = orient_faces(F)
    new_v, new_f = [], []
    for loop in boundary_loops(F):
        tris, centre = _close_loop(V, loop, planar_tol)
        if tris is not None:
            new_f += tris
            continue
        ci = len(V) + len(new_v)
        new_v.append(centre)
        R = list(reversed(loop))
        new_f += [(R[i], R[(i + 1) % len(R)], ci) for i in range(len(R))]
    if new_v:
        V = np.vstack([V, np.asarray(new_v)])
    if new_f:
        F = np.vstack([F, np.asarray(new_f, dtype=np.int64)])
    if signed_volume(V, F) < 0:
        F = F[:, [0, 2, 1]]
    return V, F


def signed_volume(V, F) -> float:
    V = np.asarray(V, dtype=np.float64)
    F = np.asarray(F, dtype=np.int64)
    a, b, c = V[F[:, 0]], V[F[:, 1]], V[F[:, 2]]
    return float(np.einsum("ij,ij->i", a, np.cross(b, c)).sum() / 6.0)


def to_manifold(V, F):
    """manifold3d Manifold of the mesh (status NoError when the mesh is closed and oriented)."""
    import manifold3d as mf
    mesh = mf.Mesh(vert_properties=np.asarray(V, dtype=np.float32),
                   tri_verts=np.asarray(F, dtype=np.uint32))
    return mf.Manifold(mesh)


def check_closed(V, F) -> dict:
    """Closure report: every edge on two faces, opposite directions, Euler characteristic V - E + F, genus,
    signed volume and the manifold3d status."""
    F = np.asarray(F, dtype=np.int64)
    counts = edge_counts(F)
    half = np.concatenate([F[:, [0, 1]], F[:, [1, 2]], F[:, [2, 0]]])
    _, hc = np.unique(half, axis=0, return_counts=True)
    nv = len(np.unique(F))
    chi = nv - len(counts) + len(F)
    m = to_manifold(V, F)
    return {"vertices": nv, "edges": len(counts), "faces": len(F), "euler": chi,
            "closed": all(c == 2 for c in counts.values()), "oriented": bool(np.all(hc == 1)),
            "boundary_edges": sum(1 for c in counts.values() if c == 1),
            "volume": signed_volume(V, F), "manifold_status": str(m.status()), "manifold_ok": m.status().name ==
            "NoError", "manifold_genus": int(m.genus()), "manifold_volume": float(m.volume())}
