# Copyright 2026 The lithocam authors. SPDX-License-Identifier: Apache-2.0
"""Robot cells in Tesseract: kinematics checks and wire-ruling reachability (needs tesseract-robotics)."""
import math
from pathlib import Path

import pytest

np = pytest.importorskip("numpy")
robot = pytest.importorskip("lithocam.robot", reason="tesseract-robotics not installed", exc_type=ImportError)

DATA = Path(__file__).resolve().parent / "data"
UR = dict(d1=0.1807, a2=-0.6127, a3=-0.57155, d4=0.17415, d5=0.11985, d6=0.11655)


@pytest.fixture(scope="module")
def ur():
    return robot.Cell("ur10e")


@pytest.fixture(scope="module")
def kr():
    return robot.Cell("kr120")


def test_ur10e_zero_pose_matches_published_dh(ur):
    """At zero joints the UR flange sits at (-(a2+a3), d4+d6, d1-d5) in the ROS base_link frame."""
    p = ur.fk(np.zeros(6))[:3, 3]
    assert p == pytest.approx([-(UR["a2"] + UR["a3"]), UR["d4"] + UR["d6"], UR["d1"] - UR["d5"]], abs=1e-9)


def test_kr120_zero_pose(kr):
    """KR120 R2500 PRO at zero joints has the upper arm horizontal (A2 = -90 degrees is upright), so the flange is at
    x = 0.35 + 1.15 + 1.0 + 0.215, z = 0.675 - 0.041."""
    p = kr.fk(np.zeros(6))[:3, 3]
    assert p == pytest.approx([0.35 + 1.15 + 1.0 + 0.215, 0.0, 0.675 - 0.041], abs=1e-9)


@pytest.mark.parametrize("name", ["ur10e", "kr120"])
def test_verified_ik_round_trip(name, ur, kr):
    """Every returned IK solution reproduces the pose and lies inside the joint limits."""
    cell = ur if name == "ur10e" else kr
    rng = np.random.default_rng(11)
    found = 0
    for _ in range(60):
        q = cell.limits[:, 0] * 0.8 + rng.random(6) * (cell.limits[:, 1] - cell.limits[:, 0]) * 0.8
        T = cell.fk(q)
        sols = cell.ik(T, q)
        found += bool(sols)
        for s in sols:
            assert np.linalg.norm(cell.fk(s) - T) < 1e-6
            assert cell.within_limits(s)
    assert found == 60


def test_wire_target_puts_the_wire_on_the_kerf_offset_line():
    p0, p1, n = np.array([0.5, 0.0, 0.1]), np.array([0.5, 0.3, 0.1]), np.array([0.0, 0.0, 1.0])
    for roll in (-0.7, 0.0, 0.9):
        T = robot.wire_target(p0, p1, n, 0.0035, roll, 0.25)
        R, t = T[:3, :3], T[:3, 3]
        assert R @ R.T == pytest.approx(np.eye(3), abs=1e-12)
        wire_mid = t + R[:, 2] * 0.25
        assert wire_mid == pytest.approx(0.5 * (p0 + p1) + n * 0.0035, abs=1e-12)   # kerf offset by the radius
        assert abs(R[:, 0] @ (p1 - p0)) == pytest.approx(np.linalg.norm(p1 - p0), abs=1e-12)   # wire along x


def test_twisted_column_fully_reachable_on_both_robots(ur, kr):
    rep = robot.load_ruled_json(str(DATA / "ruled_json" / "twisted_column.json"))
    assert robot.wire_reach(ur, rep, (0.5, -0.3, 0.0), bow_depth=0.25).fraction == 1.0
    assert robot.wire_reach(kr, rep, (1.4, -0.5, 0.0), bow_depth=0.6).fraction == 1.0


def test_part_far_out_of_reach_is_unreachable(ur):
    rep = robot.load_ruled_json(str(DATA / "ruled_json" / "twisted_column.json"))
    assert robot.wire_reach(ur, rep, (5.0, 0.0, 0.0)).fraction == 0.0


def test_far_side_face_is_out_of_reach(ur):
    """The panel spans x = 0.5 .. 1.1 m. On its far side (normal +x) the bow's flange would sit at x = 1.35 m,
    beyond the UR10e's 1.3 m reach; the near side (normal -x) is fully reachable. (No collision check yet, so the
    bottom face is kinematically reachable too.)"""
    rep = robot.load_ruled_json(str(DATA / "ruled_json" / "cladding_panel.json"))
    out = robot.wire_reach(ur, rep, (0.5, -0.3, 0.0), bow_depth=0.25)
    by_face = {f.index: f for f in out.faces}
    far = [f.index for f in rep.faces if f.label == "wire" and f.rulings and f.rulings[0].normal[0] > 0.99]
    near = [f.index for f in rep.faces if f.label == "wire" and f.rulings and f.rulings[0].normal[0] < -0.99]
    assert far and all(by_face[i].reachable == 0 for i in far)
    assert near and all(by_face[i].reachable == by_face[i].rulings for i in near)
