# Copyright 2026 The lithocam authors. SPDX-License-Identifier: Apache-2.0
"""GridNN against brute force, distances and Hausdorff on shapes with known answers, ICP recovering a rigid motion."""
import math

import numpy as np
import pytest

from lithocam import surfcompare as sc


def _sphere(r, n=6000, seed=0):
    g = np.random.default_rng(seed).normal(size=(n, 3))
    u = g / np.linalg.norm(g, axis=1, keepdims=True)
    return r * u, u


def test_gridnn_is_exact_against_brute_force_including_far_queries():
    rng = np.random.default_rng(1)
    P = rng.uniform(0, 10, (3000, 3))
    Q = np.vstack([rng.uniform(0, 10, (500, 3)), rng.uniform(30, 40, (20, 3))])   # some far outside the cloud
    d, i = sc.GridNN(P, 0.5).query(Q)
    D = np.linalg.norm(Q[:, None, :] - P[None], axis=2)
    assert np.allclose(d, D.min(1)) and np.allclose(D[np.arange(len(Q)), i], D.min(1))


def test_offset_spheres_give_the_offset_both_ways():
    A, nA = _sphere(50.5, n=40000, seed=2)
    B, nB = _sphere(50.0, n=40000, seed=3)
    h = sc.hausdorff(A, nA, B, nB)
    assert h["A_to_B"]["mean"] == pytest.approx(0.5, abs=0.02)            # point to plane removes sampling error
    assert h["A_to_B"]["max_above"] == pytest.approx(0.5, abs=0.05) and h["A_to_B"]["max_below"] < 0.05
    assert h["hausdorff"] == pytest.approx(0.5, abs=0.1)


def test_hausdorff_sees_a_bump_the_mean_hides():
    x = np.arange(0, 100, 0.5)
    X, Y = np.meshgrid(x, x)
    D = np.zeros_like(X)
    M = D + 3.0 * np.exp(-((X - 50) ** 2 + (Y - 50) ** 2) / 4.0)            # a 3 mm bump of stone left
    A, nA = sc.heightfield_points(M, 0.5)
    B, nB = sc.heightfield_points(D, 0.5)
    h = sc.hausdorff(A, nA, B, nB)
    assert h["hausdorff"] > 2.5 and h["A_to_B"]["mean"] < 0.05


def test_icp_recovers_a_rigid_motion():
    x = np.arange(0, 120, 1.0)
    X, Y = np.meshgrid(x, x)
    Z = 8 * np.sin(X / 17.0) * np.cos(Y / 23.0) + 0.02 * X
    B, nB = sc.heightfield_points(Z, 1.0)
    R = sc._rot(np.radians([0.4, -0.3, 0.8]))
    t = np.array([0.6, -0.4, 0.3])
    A = B[::3] @ R.T + t
    r = sc.icp(A, B, nB, max_iter=80)
    A2 = A @ r.T[:3, :3].T + r.T[:3, 3]
    assert r.rms_before > 0.2 and r.rms_after < 0.01
    assert np.allclose(A2, B[::3], atol=0.05)
