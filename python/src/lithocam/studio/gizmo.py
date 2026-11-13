# Copyright 2026 The lithocam authors. SPDX-License-Identifier: Apache-2.0
"""Mouse manipulation of a 6-DoF frame (the robot TCP): pure math, no Qt, so it is testable.

The handle has three axis arrows (drag = translate along that axis), three rings (drag = rotate about that
axis) and a centre square (drag = translate in the plane facing the camera). The picking is done in screen space
against the projected handle geometry, so it does not depend on the renderer's picking.

Drags are exact in the sense that the handle point under the mouse stays under the mouse:
  axis   the closest point of the axis line to the mouse ray;
  plane  the intersection of the mouse ray with the plane through the origin facing the camera;
  ring   the angle swept by the ray's intersection with the ring plane.
Holding a snap step rounds translations and angles to it.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np


@dataclass
class Camera:
    eye: np.ndarray
    center: np.ndarray
    up: np.ndarray
    fov_y_deg: float
    width: int
    height: int

    def basis(self):
        f = self.center - self.eye
        f = f / np.linalg.norm(f)
        r = np.cross(f, self.up)
        if np.linalg.norm(r) < 1e-9:
            r = np.cross(f, [0.0, 1.0, 0.0])
        r = r / np.linalg.norm(r)
        u = np.cross(r, f)
        return f, r, u

    def project(self, p) -> Tuple[np.ndarray, float]:
        """Pixel coordinates (x right, y down) and depth along the view direction."""
        f, r, u = self.basis()
        d = np.asarray(p, dtype=float) - self.eye
        z = float(d @ f)
        if z <= 1e-9:
            return np.array([np.nan, np.nan]), z
        t = math.tan(math.radians(self.fov_y_deg) / 2)
        a = self.width / max(self.height, 1)
        x = (d @ r) / (z * t * a)
        y = (d @ u) / (z * t)
        return np.array([(x + 1) * self.width / 2, (1 - y) * self.height / 2]), z

    def ray(self, px: float, py: float) -> Tuple[np.ndarray, np.ndarray]:
        f, r, u = self.basis()
        t = math.tan(math.radians(self.fov_y_deg) / 2)
        a = self.width / max(self.height, 1)
        x = (2 * px / self.width - 1) * t * a
        y = (1 - 2 * py / self.height) * t
        d = f + x * r + y * u
        return self.eye.copy(), d / np.linalg.norm(d)

    def pixel_size(self, p) -> float:
        """World length of one pixel at the depth of p."""
        _, z = self.project(p)
        return 2 * max(z, 1e-6) * math.tan(math.radians(self.fov_y_deg) / 2) / max(self.height, 1)


def rot(axis, angle) -> np.ndarray:
    a = np.asarray(axis, dtype=float)
    a = a / np.linalg.norm(a)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(angle) * K + (1 - math.cos(angle)) * K @ K


def _seg_dist(p, a, b) -> float:
    ab = b - a
    L = float(ab @ ab)
    t = 0.0 if L < 1e-12 else min(1.0, max(0.0, float((p - a) @ ab) / L))
    return float(np.linalg.norm(p - (a + t * ab)))


def ring_points(T: np.ndarray, axis: int, radius: float, n: int = 64) -> np.ndarray:
    i, j = [k for k in range(3) if k != axis]
    a = np.linspace(0, 2 * math.pi, n + 1)
    return T[:3, 3] + radius * (np.cos(a)[:, None] * T[:3, i] + np.sin(a)[:, None] * T[:3, j])


class Gizmo:
    """frame: the pose being manipulated (4x4). axes: rotation whose columns are the handle axes (base, tool or
    work frame orientation); the handle sits at the frame origin. size: arrow length in pixels."""

    RING = 0.72          # ring radius as a fraction of the arrow length
    PICK_PX = 9.0

    def __init__(self, size_px: float = 110.0):
        self.size_px = size_px
        self.frame = np.eye(4)
        self.axes = np.eye(3)
        self.mode: Optional[Tuple[str, int]] = None
        self._start = None
        self.snap_m = 0.0
        self.snap_rad = 0.0

    # geometry
    def handle_frame(self) -> np.ndarray:
        T = np.eye(4)
        T[:3, :3] = self.axes
        T[:3, 3] = self.frame[:3, 3]
        return T

    def length(self, cam: Camera) -> float:
        return self.size_px * cam.pixel_size(self.frame[:3, 3])

    def pick(self, cam: Camera, px: float, py: float) -> Optional[Tuple[str, int]]:
        H = self.handle_frame()
        L = self.length(cam)
        m = np.array([px, py], dtype=float)
        o2, _ = cam.project(H[:3, 3])
        if np.all(np.isfinite(o2)) and np.linalg.norm(m - o2) < self.PICK_PX * 1.4:
            return ("plane", -1)
        best, hit = self.PICK_PX, None
        for k in range(3):
            a2, _ = cam.project(H[:3, 3] + H[:3, k] * L * 0.25)
            b2, _ = cam.project(H[:3, 3] + H[:3, k] * L)
            if np.all(np.isfinite(a2)) and np.all(np.isfinite(b2)):
                d = _seg_dist(m, a2, b2)
                if d < best:
                    best, hit = d, ("axis", k)
        if hit:
            return hit
        for k in range(3):
            pts = [cam.project(p)[0] for p in ring_points(H, k, L * self.RING, 64)]
            for a2, b2 in zip(pts[:-1], pts[1:]):
                if np.all(np.isfinite(a2)) and np.all(np.isfinite(b2)):
                    d = _seg_dist(m, a2, b2)
                    if d < best:
                        best, hit = d, ("ring", k)
        return hit

    # drag
    def begin(self, cam: Camera, px: float, py: float) -> bool:
        self.mode = self.pick(cam, px, py)
        if self.mode is None:
            return False
        self._start = (self.frame.copy(), self.handle_frame(), self._grab(cam, px, py, self.handle_frame()))
        return self._start[2] is not None

    def _grab(self, cam, px, py, H):
        e, d = cam.ray(px, py)
        o = H[:3, 3]
        kind, k = self.mode
        if kind == "axis":
            a = H[:3, k]
            w0 = o - e
            b, c = float(a @ d), float(d @ d)
            den = c - b * b
            if abs(den) < 1e-9:
                return None
            return float((b * (d @ w0) - c * (a @ w0)) / den)       # parameter along the axis
        n = cam.basis()[0] if kind == "plane" else H[:3, k]
        den = float(d @ n)
        if abs(den) < 1e-9:
            return None
        p = e + d * float((o - e) @ n) / den
        return p - o

    def drag(self, cam: Camera, px: float, py: float) -> Optional[np.ndarray]:
        """New frame for the mouse at (px, py), or None when the drag is degenerate (axis seen end-on)."""
        if self.mode is None or self._start is None:
            return None
        F0, H0, g0 = self._start
        g = self._grab(cam, px, py, H0)
        if g is None:
            return None
        kind, k = self.mode
        F = F0.copy()
        if kind == "axis":
            t = g - g0
            if self.snap_m > 0:
                t = round(t / self.snap_m) * self.snap_m
            F[:3, 3] = F0[:3, 3] + H0[:3, k] * t
        elif kind == "plane":
            t = g - g0
            if self.snap_m > 0:
                t = np.round(t / self.snap_m) * self.snap_m
            F[:3, 3] = F0[:3, 3] + t
        else:
            a = H0[:3, k]
            v0, v1 = g0 - a * (g0 @ a), g - a * (g @ a)
            if np.linalg.norm(v0) < 1e-12 or np.linalg.norm(v1) < 1e-12:
                return None
            ang = math.atan2(float(np.cross(v0, v1) @ a), float(v0 @ v1))
            if self.snap_rad > 0:
                ang = round(ang / self.snap_rad) * self.snap_rad
            F[:3, :3] = rot(a, ang) @ F0[:3, :3]
        self.frame = F
        return F

    def end(self):
        self.mode = None
        self._start = None
