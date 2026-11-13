# Copyright 2026 The lithocam authors. SPDX-License-Identifier: Apache-2.0
"""meshfix.fill_holes: cubes with faces removed, a sphere with a cut cap, and the Stanford bunny when it is in the
user cache (examples/bunny_wire_and_mill.py --fetch puts it there; the test never downloads)."""
import os
from pathlib import Path

import numpy as np
import pytest

pytest.importorskip("manifold3d")

from lithocam import meshfix  # noqa: E402


def grid_cube(n=1, size=10.0):
    """Closed cube, each face an n x n grid of quads split into triangles, outward normals. Returns V, F, and the
    face index (0..5) of every triangle."""
    idx, V = {}, []

    def vid(p):
        k = tuple(np.round(p, 9))
        if k not in idx:
            idx[k] = len(V)
            V.append(p)
        return idx[k]

    F, side = [], []
    t = np.linspace(0.0, size, n + 1)
    for s, (ax, sign) in enumerate([(0, -1), (0, 1), (1, -1), (1, 1), (2, -1), (2, 1)]):
        u, v = [a for a in range(3) if a != ax]
        for i in range(n):
            for j in range(n):
                q = []
                for di, dj in ((0, 0), (1, 0), (1, 1), (0, 1)):
                    p = np.zeros(3)
                    p[ax] = size if sign > 0 else 0.0
                    p[u], p[v] = t[i + di], t[j + dj]
                    q.append(vid(p))
                tri = [(q[0], q[1], q[2]), (q[0], q[2], q[3])]
                for a, b, c in tri:
                    F.append((a, b, c))
                    side.append(s)
    V = np.array(V)
    F = meshfix.orient_faces(np.array(F))
    if meshfix.signed_volume(V, F) < 0:
        F = F[:, [0, 2, 1]]
    return V, F, np.array(side)


def uv_sphere(r=20.0, nu=48, nv=24):
    """Closed UV sphere with poles, outward normals."""
    V = [(0.0, 0.0, -r)]
    for i in range(1, nv):
        th = np.pi * i / nv - np.pi / 2
        for j in range(nu):
            ph = 2 * np.pi * j / nu
            V.append((r * np.cos(th) * np.cos(ph), r * np.cos(th) * np.sin(ph), r * np.sin(th)))
    V.append((0.0, 0.0, r))
    V = np.array(V)
    ring = lambda i, j: 1 + (i - 1) * nu + (j % nu)
    F = [(0, ring(1, j + 1), ring(1, j)) for j in range(nu)]
    for i in range(1, nv - 1):
        for j in range(nu):
            a, b, c, d = ring(i, j), ring(i, j + 1), ring(i + 1, j + 1), ring(i + 1, j)
            F += [(a, b, c), (a, c, d)]
    top = len(V) - 1
    F += [(top, ring(nv - 1, j), ring(nv - 1, j + 1)) for j in range(nu)]
    F = np.array(F)
    assert meshfix.signed_volume(V, F) > 0
    return V, F


def assert_closed(V, F, chi=2):
    rep = meshfix.check_closed(V, F)
    assert rep["closed"], rep                     # every edge on exactly two faces
    assert rep["oriented"], rep                   # and used once in each direction
    assert rep["boundary_edges"] == 0
    assert rep["euler"] == chi, rep               # V - E + F
    assert rep["manifold_ok"], rep                # manifold3d: Manifold(Mesh) status NoError
    assert rep["volume"] > 0
    assert rep["manifold_volume"] == pytest.approx(rep["volume"], rel=1e-4)
    return rep


def test_closed_cube_is_reported_closed():
    V, F, _ = grid_cube(1)
    rep = assert_closed(V, F)
    assert rep["volume"] == pytest.approx(1000.0)


@pytest.mark.parametrize("removed", [(5,), (0, 1), (1, 3, 5)])
def test_cube_with_faces_removed(removed):
    """One face (planar square), two opposite faces (two loops, a tube), three faces at a corner (a skew hexagon:
    centroid fan, and the corner vertex is dropped)."""
    V, F, side = grid_cube(1)
    keep = ~np.isin(side, removed)
    V2, F2 = meshfix.fill_holes(V, F[keep])
    assert len(meshfix.boundary_loops(F[keep])) == (2 if removed == (0, 1) else 1)
    rep = assert_closed(V2, F2)
    if len(removed) < 3:
        assert rep["volume"] == pytest.approx(1000.0)     # planar holes are refilled exactly
    else:
        assert 0 < rep["volume"] < 1000.0                  # the fan cuts the missing corner off


def test_cube_with_one_triangle_removed():
    V, F, _ = grid_cube(1)
    V2, F2 = meshfix.fill_holes(V, F[1:])
    rep = assert_closed(V2, F2)
    assert rep["volume"] == pytest.approx(1000.0)


def test_grid_cube_face_with_collinear_boundary():
    """A 4 x 4 grid face removed: the loop has 16 vertices, 3 collinear runs on each side. Ear clipping must not
    emit a zero-area triangle and must cover the face exactly."""
    V, F, side = grid_cube(4)
    V2, F2 = meshfix.fill_holes(V, F[side != 4])
    rep = assert_closed(V2, F2)
    assert rep["volume"] == pytest.approx(1000.0)
    a, b, c = V2[F2[:, 0]], V2[F2[:, 1]], V2[F2[:, 2]]
    assert np.min(np.linalg.norm(np.cross(b - a, c - a), axis=1)) > 1e-9
    assert len(V2) == len(V) - 9                           # the 9 inner grid vertices are unused and dropped;
                                                           # planar loop: no centroid vertex added


def test_sphere_with_cut_cap():
    """Remove every face above z = 0.6 r: one circular planar loop."""
    V, F = uv_sphere()
    cz = V[F][:, :, 2].mean(1)
    Fo = F[cz < 0.6 * 20.0]
    loops = meshfix.boundary_loops(Fo)
    assert len(loops) == 1
    V2, F2 = meshfix.fill_holes(V, Fo)
    rep = assert_closed(V2, F2)
    assert rep["volume"] < meshfix.signed_volume(V, F)


def test_sphere_with_tilted_cut_and_flipped_input():
    """A tilted cut (loop is not planar on the ring grid) and every face flipped: the result is re-oriented to a
    positive volume."""
    V, F = uv_sphere()
    c = V[F].mean(1)
    Fo = F[c @ np.array([0.3, 0.0, 1.0]) < 12.0][:, [0, 2, 1]]
    V2, F2 = meshfix.fill_holes(V, Fo)
    assert_closed(V2, F2)


def test_orient_faces_repairs_mixed_orientation():
    V, F = uv_sphere()
    G = F.copy()
    G[::3] = G[::3][:, [0, 2, 1]]
    V2, F2 = meshfix.fill_holes(V, G)
    assert_closed(V2, F2)
    assert meshfix.signed_volume(V2, F2) == pytest.approx(meshfix.signed_volume(V, F))


def _cached_bunny():
    if os.name == "nt" and os.environ.get("LOCALAPPDATA"):
        d = Path(os.environ["LOCALAPPDATA"]) / "lithocam" / "data"
    else:
        d = Path(os.environ.get("XDG_CACHE_HOME") or Path.home() / ".cache") / "lithocam"
    return d / "stanford_bunny" / "bun_zipper.ply"


@pytest.mark.skipif(not _cached_bunny().is_file(), reason="Stanford bunny not in the user cache (run the bunny "
                                                          "example with --fetch)")
def test_stanford_bunny_closes():
    import importlib.util
    spec = importlib.util.spec_from_file_location(
        "bunny_example", Path(__file__).resolve().parents[2] / "examples" / "bunny_wire_and_mill.py")
    ex = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(ex)
    V, F = ex.read_ply(_cached_bunny())
    assert len(meshfix.boundary_loops(meshfix.orient_faces(F))) == 5
    V2, F2 = meshfix.fill_holes(V, F)
    assert_closed(V2, F2)
