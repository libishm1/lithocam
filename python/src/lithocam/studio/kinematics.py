# Copyright 2026 The lithocam authors. SPDX-License-Identifier: Apache-2.0
"""Robot kinematics for the studio, from the cell URDFs (cells/*.urdf), in numpy only.

- Forward kinematics of every link from the joint values.
- The geometric Jacobian of the TCP (tool0, or tool0 times a TCP offset).
- Damped least-squares steps for jogging in Cartesian space (base or tool frame) and an iterative pose solver.

This module needs no Tesseract, so the studio executable stays small. Collision checking stays in lithocam.robot
(Tesseract); the studio says so where it shows a pose.
"""
from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np


def rpy_matrix(rpy: Sequence[float]) -> np.ndarray:
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def transform(xyz: Sequence[float] = (0, 0, 0), rpy: Sequence[float] = (0, 0, 0)) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = rpy_matrix(rpy)
    T[:3, 3] = xyz
    return T


def matrix_rpy(R: np.ndarray) -> Tuple[float, float, float]:
    """Roll, pitch, yaw (URDF convention R = Rz(y) Ry(p) Rx(r)) of a rotation matrix."""
    sp = -R[2, 0]
    if abs(sp) < 1 - 1e-12:
        return math.atan2(R[2, 1], R[2, 2]), math.asin(sp), math.atan2(R[1, 0], R[0, 0])
    # gimbal lock: roll and yaw share one axis; put it all in yaw
    return 0.0, math.copysign(math.pi / 2, sp), math.atan2(-R[0, 1], R[1, 1])


def axis_angle(axis: Sequence[float], angle: float) -> np.ndarray:
    a = np.asarray(axis, dtype=float)
    a = a / np.linalg.norm(a)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(angle) * K + (1 - math.cos(angle)) * K @ K


def rotation_error(R_have: np.ndarray, R_want: np.ndarray) -> np.ndarray:
    """Rotation vector (world frame) taking R_have to R_want."""
    Re = R_want @ R_have.T
    c = max(-1.0, min(1.0, (np.trace(Re) - 1) / 2))
    angle = math.acos(c)
    if angle < 1e-12:
        return np.zeros(3)
    v = np.array([Re[2, 1] - Re[1, 2], Re[0, 2] - Re[2, 0], Re[1, 0] - Re[0, 1]])
    if math.pi - angle < 1e-6:   # near pi: axis from the symmetric part
        B = (Re + np.eye(3)) / 2
        axis = np.sqrt(np.maximum(np.diag(B), 0))
        k = int(np.argmax(axis))
        axis = B[:, k] / math.sqrt(max(B[k, k], 1e-300))
        return axis / np.linalg.norm(axis) * angle
    return v / (2 * math.sin(angle)) * angle


@dataclass
class Visual:
    mesh: Optional[str]                 # absolute path of a mesh file, or None for a primitive
    origin: np.ndarray                  # 4x4 in the link frame
    scale: Tuple[float, float, float] = (1.0, 1.0, 1.0)
    box: Optional[Tuple[float, float, float]] = None
    cylinder: Optional[Tuple[float, float]] = None   # radius, length


@dataclass
class Joint:
    name: str
    kind: str                           # revolute, continuous, prismatic, fixed
    parent: str
    child: str
    origin: np.ndarray
    axis: np.ndarray
    lower: float = -math.pi
    upper: float = math.pi


@dataclass
class Robot:
    name: str
    joints: List[Joint]
    visuals: Dict[str, List[Visual]]
    root: str
    tcp_link: str = "tool0"
    tcp_offset: np.ndarray = field(default_factory=lambda: np.eye(4))   # TCP in the tcp_link frame
    active: List[Joint] = field(default_factory=list)                   # moving joints from root to tcp_link

    @property
    def dof(self) -> int:
        return len(self.active)

    @property
    def lower(self) -> np.ndarray:
        return np.array([j.lower for j in self.active])

    @property
    def upper(self) -> np.ndarray:
        return np.array([j.upper for j in self.active])

    # ---------------------------------------------------------------- forward kinematics
    def _joint_tf(self, j: Joint, q: float) -> np.ndarray:
        if j.kind in ("revolute", "continuous"):
            M = np.eye(4)
            M[:3, :3] = axis_angle(j.axis, q)
            return j.origin @ M
        if j.kind == "prismatic":
            M = np.eye(4)
            M[:3, 3] = j.axis / np.linalg.norm(j.axis) * q
            return j.origin @ M
        return j.origin

    def link_transforms(self, q: Sequence[float]) -> Dict[str, np.ndarray]:
        qmap = {j.name: float(v) for j, v in zip(self.active, q)}
        tf = {self.root: np.eye(4)}
        pending = list(self.joints)
        while pending:   # joints in any order: resolve parents first
            rest = []
            for j in pending:
                if j.parent in tf:
                    tf[j.child] = tf[j.parent] @ self._joint_tf(j, qmap.get(j.name, 0.0))
                else:
                    rest.append(j)
            if len(rest) == len(pending):
                break
            pending = rest
        return tf

    def tcp(self, q: Sequence[float]) -> np.ndarray:
        return self.link_transforms(q)[self.tcp_link] @ self.tcp_offset

    def jacobian(self, q: Sequence[float]) -> np.ndarray:
        """6 x dof geometric Jacobian of the TCP in the base frame: rows (v, w)."""
        tf = self.link_transforms(q)
        p = (tf[self.tcp_link] @ self.tcp_offset)[:3, 3]
        J = np.zeros((6, self.dof))
        for i, j in enumerate(self.active):
            Tj = tf[j.parent] @ j.origin
            z = Tj[:3, :3] @ (j.axis / np.linalg.norm(j.axis))
            if j.kind == "prismatic":
                J[:3, i] = z
            else:
                J[:3, i] = np.cross(z, p - Tj[:3, 3])
                J[3:, i] = z
        return J

    # ---------------------------------------------------------------- inverse kinematics (local, damped)
    def clamp(self, q: np.ndarray) -> np.ndarray:
        return np.minimum(np.maximum(q, self.lower), self.upper)

    def twist_step(self, q: Sequence[float], twist: np.ndarray, damping: float = 0.02) -> np.ndarray:
        """Joint values after one damped least-squares step for a small base-frame twist (dx, dy, dz, rx, ry, rz)."""
        J = self.jacobian(q)
        JJt = J @ J.T + damping ** 2 * np.eye(6)
        dq = J.T @ np.linalg.solve(JJt, np.asarray(twist, dtype=float))
        return self.clamp(np.asarray(q, dtype=float) + dq)

    def solve_pose(self, target: np.ndarray, seed: Sequence[float], tol_pos: float = 1e-6, tol_rot: float = 1e-6,
                   iterations: int = 200, damping: float = 0.01) -> Tuple[np.ndarray, bool]:
        """Iterate damped least squares from `seed` towards the TCP pose `target`. Returns (q, converged)."""
        q = np.asarray(seed, dtype=float).copy()
        for _ in range(iterations):
            T = self.tcp(q)
            ep = target[:3, 3] - T[:3, 3]
            er = rotation_error(T[:3, :3], target[:3, :3])
            if np.linalg.norm(ep) < tol_pos and np.linalg.norm(er) < tol_rot:
                return q, True
            step = np.concatenate([ep, er])
            n = np.linalg.norm(step[:3]) + 0.3 * np.linalg.norm(step[3:])
            if n > 0.05:   # limit the step for robustness far from the target
                step *= 0.05 / n
            q = self.twist_step(q, step, damping)
        T = self.tcp(q)
        ok = np.linalg.norm(target[:3, 3] - T[:3, 3]) < tol_pos * 10 and \
            np.linalg.norm(rotation_error(T[:3, :3], target[:3, :3])) < tol_rot * 10
        return q, bool(ok)

    def jog(self, q: Sequence[float], delta: Sequence[float], frame: str = "base") -> Tuple[np.ndarray, bool]:
        """Move the TCP by (dx, dy, dz) metres and (rx, ry, rz) radians in the base or tool frame."""
        T = self.tcp(q)
        d = np.asarray(delta, dtype=float)
        target = T.copy()
        if frame == "tool":
            target[:3, 3] = T[:3, 3] + T[:3, :3] @ d[:3]
            if np.linalg.norm(d[3:]) > 0:
                target[:3, :3] = T[:3, :3] @ axis_angle(d[3:], np.linalg.norm(d[3:]))
        else:
            target[:3, 3] = T[:3, 3] + d[:3]
            if np.linalg.norm(d[3:]) > 0:   # rotate about the TCP, axes of the base frame
                target[:3, :3] = axis_angle(d[3:], np.linalg.norm(d[3:])) @ T[:3, :3]
        return self.solve_pose(target, q)

    def manipulability(self, q: Sequence[float]) -> float:
        J = self.jacobian(q)
        return float(math.sqrt(max(np.linalg.det(J @ J.T), 0.0)))


def load_urdf(path: str | Path, tcp_link: str = "tool0") -> Robot:
    path = Path(path)
    root = ET.parse(path).getroot()
    base = path.parent

    def origin_of(el) -> np.ndarray:
        o = el.find("origin") if el is not None else None
        if o is None:
            return np.eye(4)
        xyz = [float(v) for v in o.get("xyz", "0 0 0").split()]
        rpy = [float(v) for v in o.get("rpy", "0 0 0").split()]
        return transform(xyz, rpy)

    def resolve(fn: str) -> str:
        if fn.startswith("package://"):
            rel = fn[len("package://"):]
            pkg, _, rest = rel.partition("/")
            return str((base / rest) if (base / rest).exists() else (base.parent / pkg / rest))
        return str((base / fn).resolve())

    visuals: Dict[str, List[Visual]] = {}
    for link in root.findall("link"):
        vs = []
        for v in link.findall("visual"):
            g = v.find("geometry")
            if g is None:
                continue
            m, b, c = g.find("mesh"), g.find("box"), g.find("cylinder")
            if m is not None:
                sc = tuple(float(s) for s in m.get("scale", "1 1 1").split())
                vs.append(Visual(resolve(m.get("filename")), origin_of(v), sc))
            elif b is not None:
                vs.append(Visual(None, origin_of(v), box=tuple(float(s) for s in b.get("size").split())))
            elif c is not None:
                vs.append(Visual(None, origin_of(v), cylinder=(float(c.get("radius")), float(c.get("length")))))
        visuals[link.get("name")] = vs
    joints = []
    for j in root.findall("joint"):
        ax = j.find("axis")
        lim = j.find("limit")
        joints.append(Joint(
            name=j.get("name"), kind=j.get("type"), parent=j.find("parent").get("link"),
            child=j.find("child").get("link"), origin=origin_of(j),
            axis=np.array([float(s) for s in ax.get("xyz").split()]) if ax is not None else np.array([0, 0, 1.0]),
            lower=float(lim.get("lower", -math.pi)) if lim is not None else -math.pi,
            upper=float(lim.get("upper", math.pi)) if lim is not None else math.pi))
    children = {j.child for j in joints}
    roots = [n for n in visuals if n not in children]
    rname = "world" if "world" in roots else (roots[0] if roots else joints[0].parent)
    robot = Robot(root.get("name", path.stem), joints, visuals, rname, tcp_link)
    # moving joints on the chain root -> tcp_link
    by_child = {j.child: j for j in joints}
    chain, link = [], tcp_link
    while link in by_child:
        j = by_child[link]
        if j.kind != "fixed":
            chain.append(j)
        link = j.parent
    robot.active = list(reversed(chain))
    return robot
