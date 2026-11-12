# Copyright 2026 The lithocam authors. SPDX-License-Identifier: Apache-2.0
"""Process planner: a ladder graph over the free choices of a CAM path (Descartes-style).

A process path fixes where the tool must be; it leaves some freedom: the roll of a diamond wire about itself, the
spin of a spindle about its axis, the IK branch, and joint wrap-around. Each path point is a rung; each rung holds
the joint configurations of every candidate pose and every IK solution that is inside the limits and collision
free. Dynamic programming picks one node per rung minimising the summed squared joint steps, with no step larger
than `max_step` (a jump would sweep the tool through the part). Where no edge passes, the path breaks there and a
new run starts, and the break is reported.

Backends:
  TesseractBackend  analytic/numeric IK branches and collision checks on the cell meshes (lithocam.robot.Cell);
  NumpyBackend      the studio's damped least-squares IK from several seeds; no collision checks.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional, Sequence

import numpy as np


class NumpyBackend:
    name = "numpy (no collision checks)"
    checks_collision = False

    def __init__(self, robot, seeds: Sequence[Sequence[float]] = ()):
        self.robot = robot                      # studio kinematics; tcp_offset must be identity (tool0 targets)
        self.seeds = [np.asarray(s, dtype=float) for s in seeds]

    def ik(self, T: np.ndarray, seed: Optional[np.ndarray] = None) -> List[np.ndarray]:
        out = []
        for s in ([seed] if seed is not None else []) + self.seeds:
            q, ok = self.robot.solve_pose(T, s)
            if ok and not any(np.allclose(q, o, atol=1e-6) for o in out):
                out.append(q)
        return out

    def in_collision(self, q) -> bool:
        return False


class TesseractBackend:
    name = "Tesseract (IK branches, collision on cell meshes)"
    checks_collision = True

    def __init__(self, cell, wraps: bool = True):
        self.cell = cell
        self.wraps = wraps

    def ik(self, T: np.ndarray, seed: Optional[np.ndarray] = None) -> List[np.ndarray]:
        sols = self.cell.ik(T, seed)
        if self.wraps:
            sols = [w for s in sols for w in self.cell.wrap_variants(s)]
        return sols

    def in_collision(self, q) -> bool:
        return self.cell.in_collision(q)


class CellBackend(TesseractBackend):
    """A Tesseract cell set up for one station: the spindle stack on tool0 (collision geometry; the cutter itself is
    not, it is meant to touch the stone), the part from its STL, and the turntable. Needs tesseract-robotics."""

    def __init__(self, robot_name: str, cells_dir, spindle=None, part_stl: str = "", work=None,
                 turntable=None):
        from lithocam.robot import Cell
        cell = Cell(robot_name, cells_dir=cells_dir)
        super().__init__(cell)
        self.work = np.eye(4) if work is None else np.asarray(work, dtype=float)
        if spindle is not None:
            ox, oy = spindle.axis_offset
            cell.attach_tool("spindle", [(d / 1000, z0 / 1000, ln / 1000, ox / 1000, oy / 1000)
                                         for _, d, z0, ln in spindle.parts() if d > 0 and ln > 0])
        self.has_part = bool(part_stl)
        if part_stl:
            cell.add_mesh_obstacle("part", part_stl, 1e-3, tuple(self.work[:3, 3]), 0.0)
        if turntable:
            d, h = float(turntable.get("diameter", 0.8)), float(turntable.get("height", 0.04))
            o = self.work[:3, 3]
            cell.add_box_obstacle("turntable", (d, d, h), (o[0], o[1], o[2] - h / 2 - 0.002))

    def set_tool_mode(self, mode: str):
        """'wire': the bow collides, the spindle does not; 'spindle': the other way round."""
        from tesseract_robotics.tesseract_environment import ChangeLinkCollisionEnabledCommand
        self.cell.set_wire_tool(mode == "wire")
        if "spindle" in [str(n) for n in self.cell.env.getLinkNames()]:
            self.cell.env.applyCommand(ChangeLinkCollisionEnabledCommand("spindle", mode != "wire"))
        self.cell._mgr = None

    def place_part(self, work, yaw: float):
        """Turn the part obstacle with the turntable."""
        if not self.has_part:
            return
        from lithocam.robot import _iso
        from tesseract_robotics.tesseract_environment import ChangeJointOriginCommand
        c, s = math.cos(yaw), math.sin(yaw)
        T = np.eye(4)
        T[:3, :3] = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])
        T[:3, 3] = np.asarray(work, dtype=float)[:3, 3]
        self.cell.env.applyCommand(ChangeJointOriginCommand("part_joint", _iso(T)))
        self.cell._mgr = None


def make_backend(station, cells_dir):
    """CellBackend when tesseract-robotics imports, else None (the caller falls back to NumpyBackend)."""
    try:
        import tesseract_robotics  # noqa: F401
    except ImportError:
        return None
    stl = station.part_path if station.part_path and str(station.part_path).lower().endswith(".stl") else ""
    return CellBackend(station.robot, cells_dir, station.spindle_model(), stl, station.work_frame(),
                       station.turntable)


@dataclass
class LadderResult:
    joints: List[np.ndarray]                 # one configuration per solved rung, in rung order
    rungs: List[int]                         # rung index of each configuration
    choice: List[int]                        # chosen candidate pose per solved rung
    missing: List[int] = field(default_factory=list)   # rungs with no collision-free solution
    breaks: List[int] = field(default_factory=list)    # rungs where the path had to restart (a jump)
    max_step: float = 0.0
    nodes: int = 0
    collisions_rejected: int = 0


def ladder(rungs: Sequence[Sequence[np.ndarray]], backend, seed: Optional[Sequence[float]] = None,
           max_step: float = math.radians(25), check_collision: bool = True) -> LadderResult:
    """rungs[k] = candidate tool0 poses (4x4) of path point k. Returns the least-motion joint path."""
    seed = None if seed is None else np.asarray(seed, dtype=float)
    layers = []                                          # per rung: (Q (n, dof), cand idx (n,))
    rejected = 0
    prev_hint = seed
    for poses in rungs:
        qs, idx = [], []
        for c, T in enumerate(poses):
            for q in backend.ik(T, prev_hint):
                if check_collision and backend.checks_collision and backend.in_collision(q):
                    rejected += 1
                    continue
                qs.append(q)
                idx.append(c)
        if qs:
            Q = np.asarray(qs)
            layers.append((Q, np.asarray(idx)))
            prev_hint = Q[0] if prev_hint is None else Q[np.argmin(np.abs(Q - prev_hint).max(1))]
        else:
            layers.append((None, None))
    res = LadderResult([], [], [], nodes=sum(len(L[0]) for L in layers if L[0] is not None),
                       collisions_rejected=rejected)
    k = 0
    start = seed
    while k < len(layers):
        if layers[k][0] is None:
            res.missing.append(k)
            k += 1
            continue
        # one run of consecutive rungs connected by steps <= max_step
        Q0 = layers[k][0]
        cost = np.zeros(len(Q0)) if start is None else np.sum((Q0 - start) ** 2, axis=1)
        back = []
        run = [k]
        j = k + 1
        while j < len(layers) and layers[j][0] is not None:
            Qa, Qb = layers[run[-1]][0], layers[j][0]
            d = Qb[None, :, :] - Qa[:, None, :]
            step = np.abs(d).max(axis=2)
            w = np.where(step <= max_step, np.sum(d * d, axis=2), np.inf)
            tot = cost[:, None] + w
            arg = np.argmin(tot, axis=0)
            new = tot[arg, np.arange(len(Qb))]
            if not np.isfinite(new).any():
                break                                    # no continuation: break here
            back.append(arg)
            cost = new
            run.append(j)
            j += 1
        # backtrack
        node = int(np.argmin(cost))
        picks = [node]
        for arg in reversed(back):
            node = int(arg[node])
            picks.append(node)
        picks.reverse()
        for r, n in zip(run, picks):
            res.joints.append(layers[r][0][n])
            res.rungs.append(r)
            res.choice.append(int(layers[r][1][n]))
        start = res.joints[-1]
        if j < len(layers) and layers[j][0] is not None:
            res.breaks.append(j)
        k = j
    steps = [float(np.max(np.abs(res.joints[i] - res.joints[i - 1]))) for i in range(1, len(res.joints))]
    res.max_step = max(steps) if steps else 0.0
    return res
