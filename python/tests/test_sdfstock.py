# Copyright 2026 The lithocam authors. SPDX-License-Identifier: Apache-2.0
import math
import os
import time

import manifold3d as mf
import numpy as np

from lithocam.sdfstock import SdfStock

LO, HI = (0.0, 0.0, 0.0), (40.0, 30.0, 20.0)
BOX_V = 40.0 * 30.0 * 20.0


def rel(a, b):
    return abs(a - b) / abs(b)


def mesh_stats(V, F):
    used = np.unique(F)
    e = np.sort(np.concatenate([F[:, [0, 1]], F[:, [1, 2]], F[:, [2, 0]]]), axis=1)
    ne = len(np.unique(e, axis=0))
    T = V[F]
    sv = float(np.einsum("ij,ij->i", T[:, 0], np.cross(T[:, 1], T[:, 2])).sum() / 6.0)
    return len(used) - ne + len(F), sv


def test_box_volume():
    for lo in ((0, 0, 0), (0.3, 0.17, 0.41)):
        hi = tuple(np.array(lo) + (40, 30, 20))
        s = SdfStock.from_box(lo, hi, 1.0)
        assert rel(s.volume(), BOX_V) < 0.005
        assert s.occupancy().sum() > 0


def test_ball_slot_convergence():
    r, L = 4.0, 24.0
    exact = math.pi * r * r * L + 4.0 / 3.0 * math.pi * r ** 3
    errs = []
    for cell in (1.0, 0.5, 0.25):
        s = SdfStock.from_box(LO, HI, cell)
        v0 = s.volume()
        removed = s.cut_ball((8, 15, 10), (32, 15, 10), r)
        errs.append(rel(removed, exact))
        assert rel(v0 - s.volume(), removed) < 1e-4
    print("ball slot errors (cell 1, .5, .25):", errs)
    assert errs[1] < 0.02 and errs[2] < 0.02
    assert errs[2] < errs[1] < errs[0] + 1e-9


def test_half_capsule_slot():
    r, L = 4.0, 24.0
    exact = 0.5 * (math.pi * r * r * L + 4.0 / 3.0 * math.pi * r ** 3)
    s = SdfStock.from_box(LO, HI, 0.5)
    removed = s.cut_ball((8, 15, 20), (32, 15, 20), r)
    print("half capsule error:", rel(removed, exact))
    assert rel(removed, exact) < 0.02


def test_flat_slot_vs_manifold():
    r, depth = 4.0, 15.0
    s = SdfStock.from_box(LO, HI, 0.5)
    removed = s.cut_cylinder((10, 15, 5), (30, 15, 5), (0, 0, 1), r, 30.0)
    cyl = mf.Manifold.cylinder(30.0, r, r, 256)
    hull = mf.Manifold.batch_hull([cyl.translate((10, 15, 5)), cyl.translate((30, 15, 5))])
    box = mf.Manifold.cube((40, 30, 20))
    exact = (hull ^ box).volume()
    print("flat slot: sdf {:.1f} manifold {:.1f} err {:.4f}".format(removed, exact, rel(removed, exact)))
    assert rel(removed, exact) < 0.02
    assert abs(depth * (2 * r * 20 + math.pi * r * r) - exact) / exact < 0.01


def test_bull_nose_removes_less_than_flat():
    a = SdfStock.from_box(LO, HI, 0.5)
    b = SdfStock.from_box(LO, HI, 0.5)
    va = a.cut_cylinder((10, 15, 5), (30, 15, 5), (0, 0, 1), 4.0, 30.0)
    vb = b.cut_cylinder((10, 15, 5), (30, 15, 5), (0, 0, 1), 4.0, 30.0, corner_r=1.5)
    assert vb < va
    # the bull-nose corner relief for one pass: area (1 - pi/4) c^2 along the length
    expect = (1 - math.pi / 4) * 1.5 ** 2 * 2 * 20
    assert va - vb > 0.5 * expect


def test_tilted_cylinder_5axis_volume():
    s = SdfStock.from_box((0, 0, 0), (40, 40, 40), 0.5)
    ax = np.array([0.5, 0.0, 1.0])
    ax /= np.linalg.norm(ax)
    # one pose pair at the same point: a single tilted cylinder fully inside; length 10 from the tip
    removed = s.cut_cylinder((15, 20, 12), (15, 20, 12), ax, 3.0, 10.0)
    exact = math.pi * 9.0 * 10.0
    assert rel(removed, exact) < 0.03


def test_from_mesh_sphere_and_cube():
    sph = mf.Manifold.sphere(10.0, 128)
    m = sph.to_mesh()
    V, F = np.asarray(m.vert_properties)[:, :3].astype(float), np.asarray(m.tri_verts).astype(np.int64)
    s = SdfStock.from_mesh(V, F, 0.5)
    print("sphere from_mesh volume err:", rel(s.volume(), sph.volume()))
    assert rel(s.volume(), sph.volume()) < 0.02
    Vm, Fm = s.mesh()
    chi, sv = mesh_stats(Vm, Fm)
    assert chi == 2
    assert sv > 0 and rel(sv, sph.volume()) < 0.03

    cube = mf.Manifold.cube((20, 20, 20)).translate((-9.87, -10.13, -10.31))
    m = cube.to_mesh()
    V, F = np.asarray(m.vert_properties)[:, :3].astype(float), np.asarray(m.tri_verts).astype(np.int64)
    s = SdfStock.from_mesh(V, F, 0.5)
    print("cube from_mesh volume err:", rel(s.volume(), cube.volume()))
    assert rel(s.volume(), cube.volume()) < 0.02
    Vm, Fm = s.mesh()
    chi, sv = mesh_stats(Vm, Fm)
    assert chi == 2
    assert sv > 0 and rel(sv, cube.volume()) < 0.03


def test_disc_cut():
    R, kerf, d = 15.0, 2.0, 5.0
    s = SdfStock.from_box((0, 0, 0), (80, 40, 40), 0.5)
    removed = s.cut_disc((20, 20, 40 + d), (60, 20, 40 + d), (0, 1, 0), R, kerf)
    seg = R * R * math.acos(d / R) - d * math.sqrt(R * R - d * d)
    exact = kerf * (seg + 40.0 * (R - d))
    print("disc cut: sdf {:.1f} analytic {:.1f} err {:.4f}".format(removed, exact, rel(removed, exact)))
    assert rel(removed, exact) < 0.03


def test_wire_cut():
    r = 1.0
    s = SdfStock.from_box((0, 0, 0), (40, 40, 40), 0.5)
    # a wire along y, from x = 10 to x = 30 at z = 20: swept strip, length 40 in y, fully through the box
    removed = s.cut_wire((10, -5, 20), (10, 45, 20), (30, -5, 20), (30, 45, 20), r)
    exact = 40.0 * (2 * r * 20.0 + math.pi * r * r)
    print("wire cut: sdf {:.1f} analytic {:.1f} err {:.4f}".format(removed, exact, rel(removed, exact)))
    assert rel(removed, exact) < 0.03


def test_cut_path_and_stl(tmp_path):
    s = SdfStock.from_box((0, 0, 0), (30, 30, 20), 0.5)
    poses = np.array([[5, 5, 19], [25, 5, 19], [25, 10, 19], [5, 10, 19]], float)
    tot = s.cut_path("ball", poses, r=2.0)
    assert tot > 0
    path = str(tmp_path / "s.stl")
    n = s.write_stl(path)
    assert os.path.getsize(path) == 84 + 50 * n
    V, F = s.mesh()
    chi, sv = mesh_stats(V, F)
    assert chi == 2 and sv > 0
    assert s.section(2, 19.0).any() and s.section(2, 19.0).shape == s.shape[:2]


def test_timing_20000_ball_moves():
    cell = 1.0
    s = SdfStock.from_box((0, 0, 0), (116, 116, 116), cell)
    assert s.shape[0] >= 120
    rng = np.random.default_rng(1)
    p = np.array([30.0, 30.0, 80.0])
    t0 = time.time()
    removed = 0.0
    for _ in range(20000):
        q = np.clip(p + rng.normal(0, 1.0, 3), 10.0, 106.0)
        removed += s.cut_ball(p, q, 3.0)
        p = q
    dt = time.time() - t0
    print("20000 ball moves on {} grid: {:.2f} s, {:.0f} us/move, removed {:.0f}".format(s.shape, dt, dt / 20000 * 1e6, removed))
    assert removed > 0
