# Copyright 2026 The lithocam authors. SPDX-License-Identifier: Apache-2.0
"""The TCP handle: projection, picking and drags keep the grabbed point under the mouse."""
import math

import numpy as np
import pytest

from lithocam.studio.gizmo import Camera, Gizmo, rot


def cam():
    return Camera(eye=np.array([2.0, -1.5, 1.4]), center=np.array([0.6, 0.0, 0.3]), up=np.array([0, 0, 1.0]),
                  fov_y_deg=40.0, width=1200, height=800)


def test_project_ray_round_trip():
    c = cam()
    rng = np.random.default_rng(1)
    for _ in range(50):
        p = c.center + rng.uniform(-0.4, 0.4, 3)
        px, _ = c.project(p)
        e, d = c.ray(*px)
        # p lies on the ray
        t = (p - e) @ d
        assert np.linalg.norm(e + t * d - p) < 1e-9


def frame():
    F = np.eye(4)
    F[:3, :3] = rot([0.3, 0.2, 1.0], 0.7) @ np.diag([1.0, -1.0, -1.0])
    F[:3, 3] = [0.6, 0.1, 0.35]
    return F


def test_pick_axis_ring_centre():
    c, g = cam(), Gizmo()
    g.frame = frame()
    o, _ = c.project(g.frame[:3, 3])
    assert g.pick(c, *o) == ("plane", -1)
    L = g.length(c)
    for k in range(3):
        tip, _ = c.project(g.frame[:3, 3] + np.eye(3)[:, k] * L * 0.8)
        assert g.pick(c, *tip) == ("axis", k)
    # a point on the z ring away from the axes
    p = g.frame[:3, 3] + L * g.RING * (np.cos(0.8) * np.eye(3)[:, 0] + np.sin(0.8) * np.eye(3)[:, 1])
    px, _ = c.project(p)
    assert g.pick(c, *px) == ("ring", 2)
    far, _ = c.project(g.frame[:3, 3] + np.array([0.0, 0.0, 3 * L]))
    assert g.pick(c, far[0] + 300, far[1]) is None


@pytest.mark.parametrize("k", [0, 1, 2])
def test_axis_drag_follows_mouse(k):
    c, g = cam(), Gizmo()
    g.frame = frame()
    L = g.length(c)
    a = np.eye(3)[:, k]
    start = g.frame[:3, 3] + a * L * 0.7
    px0, _ = c.project(start)
    assert g.begin(c, *px0)
    target = start + a * 0.05
    px1, _ = c.project(target)
    F = g.drag(c, *px1)
    assert np.allclose(F[:3, 3], frame()[:3, 3] + a * 0.05, atol=1e-6)
    assert np.allclose(F[:3, :3], frame()[:3, :3])
    g.end()


def test_snap_and_plane_drag():
    c, g = cam(), Gizmo()
    g.frame = frame()
    g.snap_m = 0.01
    o, _ = c.project(g.frame[:3, 3])
    assert g.begin(c, *o)
    F = g.drag(c, o[0] + 37, o[1] - 11)
    t = F[:3, 3] - frame()[:3, 3]
    assert np.allclose(t / 0.01, np.round(t / 0.01), atol=1e-9)
    f = c.basis()[0]
    assert abs(t @ f) < 1e-9 or g.snap_m > 0        # in the view plane before snapping


def test_ring_drag_angle():
    c, g = cam(), Gizmo()
    g.frame = frame()
    L = g.length(c)
    R = L * g.RING
    a0, a1 = 0.6, 0.6 + math.radians(30)
    ex, ey = np.eye(3)[:, 0], np.eye(3)[:, 1]
    p0 = g.frame[:3, 3] + R * (math.cos(a0) * ex + math.sin(a0) * ey)
    p1 = g.frame[:3, 3] + R * (math.cos(a1) * ex + math.sin(a1) * ey)
    px0, _ = c.project(p0)
    assert g.begin(c, *px0) and g.mode == ("ring", 2)
    F = g.drag(c, *c.project(p1)[0])
    Rz = F[:3, :3] @ frame()[:3, :3].T
    assert np.allclose(Rz, rot([0, 0, 1], math.radians(30)), atol=1e-6)
    assert np.allclose(F[:3, 3], frame()[:3, 3])
