# Copyright 2026 The lithocam authors. SPDX-License-Identifier: Apache-2.0
"""Studio kinematics: URDF forward kinematics against the published UR10e DH parameters, Jacobian against finite
differences, and pose solving and jogging round trips on both cells."""
import math
from pathlib import Path

import pytest

np = pytest.importorskip("numpy")
from lithocam.studio.kinematics import load_urdf, rotation_error  # noqa: E402

CELLS = Path(__file__).resolve().parents[2] / "cells"


def ur10e_dh_fk(q):
    """Universal Robots' published UR10e DH parameters (standard DH), base_link to tool0."""
    d = [0.1807, 0, 0, 0.17415, 0.11985, 0.11655]
    a = [0, -0.6127, -0.57155, 0, 0, 0]
    alpha = [math.pi / 2, 0, 0, math.pi / 2, -math.pi / 2, 0]
    T = np.eye(4)
    for i in range(6):
        ct, st, ca, sa = math.cos(q[i]), math.sin(q[i]), math.cos(alpha[i]), math.sin(alpha[i])
        T = T @ np.array([[ct, -st * ca, st * sa, a[i] * ct], [st, ct * ca, -ct * sa, a[i] * st],
                          [0, sa, ca, d[i]], [0, 0, 0, 1]])
    return T


@pytest.fixture(scope="module")
def ur():
    return load_urdf(CELLS / "ur10e.urdf")


@pytest.fixture(scope="module")
def kr():
    return load_urdf(CELLS / "kr120.urdf")


def test_chain_and_limits(ur, kr):
    assert ur.dof == 6 and kr.dof == 6
    assert np.all(ur.lower < ur.upper) and np.all(kr.lower < kr.upper)


def test_ur10e_matches_dh(ur):
    # the ROS UR description puts base_link rotated by pi about z relative to the DH base frame
    Rz = np.diag([-1.0, -1.0, 1.0, 1.0])
    rng = np.random.default_rng(1)
    for _ in range(200):
        q = rng.uniform(-math.pi, math.pi, 6)
        T = ur.tcp(q)
        D = Rz @ ur10e_dh_fk(q)
        assert np.allclose(T[:3, 3], D[:3, 3], atol=1e-9)
        # tool0 in the ROS description and the DH flange frame differ only by a fixed rotation
        assert np.allclose(T[:3, 2], D[:3, 2], atol=1e-9)


@pytest.mark.parametrize("name", ["ur", "kr"])
def test_jacobian_matches_finite_differences(name, request):
    r = request.getfixturevalue(name)
    rng = np.random.default_rng(2)
    for _ in range(30):
        q = rng.uniform(r.lower / 2, r.upper / 2)
        J = r.jacobian(q)
        T0 = r.tcp(q)
        h = 1e-7
        for i in range(r.dof):
            dq = np.zeros(r.dof)
            dq[i] = h
            T1 = r.tcp(q + dq)
            v = (T1[:3, 3] - T0[:3, 3]) / h
            w = rotation_error(T0[:3, :3], T1[:3, :3]) / h
            assert np.allclose(J[:3, i], v, atol=1e-5)
            assert np.allclose(J[3:, i], w, atol=1e-5)


@pytest.mark.parametrize("name", ["ur", "kr"])
def test_solve_pose_round_trip(name, request):
    r = request.getfixturevalue(name)
    rng = np.random.default_rng(3)
    solved = 0
    for _ in range(60):
        q = rng.uniform(r.lower / 3, r.upper / 3)
        target = r.tcp(q)
        seed = q + rng.normal(0, 0.15, r.dof)
        qs, ok = r.solve_pose(target, seed)
        if ok:
            solved += 1
            T = r.tcp(qs)
            assert np.linalg.norm(T[:3, 3] - target[:3, 3]) < 1e-5
            assert np.linalg.norm(rotation_error(T[:3, :3], target[:3, :3])) < 1e-5
    assert solved >= 55


@pytest.mark.parametrize("name", ["ur", "kr"])
@pytest.mark.parametrize("frame", ["base", "tool"])
def test_jog_moves_the_tcp_by_the_step(name, frame, request):
    r = request.getfixturevalue(name)
    q = (r.lower + r.upper) / 2 + np.array([0.3, -0.6, 0.9, -0.4, 0.5, 0.2])[: r.dof] * 0.5
    T0 = r.tcp(q)
    q1, ok = r.jog(q, (0.01, -0.005, 0.008, 0, 0, 0), frame)
    assert ok
    T1 = r.tcp(q1)
    want = np.array([0.01, -0.005, 0.008])
    if frame == "tool":
        want = T0[:3, :3] @ want
    assert np.allclose(T1[:3, 3] - T0[:3, 3], want, atol=1e-5)
    assert np.linalg.norm(rotation_error(T0[:3, :3], T1[:3, :3])) < 1e-5
    q2, ok = r.jog(q, (0, 0, 0, 0, 0, math.radians(5)), frame)
    assert ok
    T2 = r.tcp(q2)
    assert np.allclose(T2[:3, 3], T0[:3, 3], atol=1e-5)
    assert abs(np.linalg.norm(rotation_error(T0[:3, :3], T2[:3, :3])) - math.radians(5)) < 1e-5
