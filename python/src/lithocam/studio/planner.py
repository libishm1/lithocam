# Copyright 2026 The lithocam authors. SPDX-License-Identifier: Apache-2.0
"""Process planner: a ladder graph over the free choices of a CAM path (Descartes-style).

A process path fixes where the tool must be; it leaves some freedom: the roll of a diamond wire about itself, the
spin of a spindle about its axis, the IK branch, and joint wrap-around. Each path point is a rung; each rung holds
the joint configurations of every candidate pose and every IK solution that is inside the limits and collision
free. Dynamic programming picks one node per rung minimising the summed squared joint steps, with no step larger
than `max_step` (a jump would sweep the tool through the part). Where no edge passes, the path breaks there and a
new run starts, and the break is reported.

External axis. A turntable driven as a coordinated axis (E1) adds one more free choice per rung: each candidate pose
carries the table angle it was built for (`ext`), the graph nodes are (6 arm joints, table angle), and an edge also
needs the table step within `ext_max_step`. `TableChain` is the 7-axis chain (arm + table rotary) the candidates
come from: the part frame turns with the table angle about the work frame's z axis.

Backends:
  TesseractBackend  analytic/numeric IK branches and collision checks on the cell meshes (lithocam.robot.Cell);
  NumpyBackend      the studio's damped least-squares IK from several seeds; no collision checks.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional, Sequence

import numpy as np


def pruned_wraps(q, limits=None, hint=None) -> List[np.ndarray]:
    """At most two of the configurations equal to q modulo 2 pi per joint: the one nearest the hint (the previous
    path point) and the one nearest zero. Joints with +-360 degree ranges give up to 2^6 wrap variants per IK
    branch; the ladder only ever uses the ones near its neighbours, so enumerating all of them is wasted work
    (that enumeration made a two-view bunny wire plan run for hours)."""
    q = np.asarray(q, dtype=float)
    if limits is None:
        return [q]
    lo, hi = np.asarray(limits, dtype=float)[:, 0], np.asarray(limits, dtype=float)[:, 1]

    def nearest(target):
        w = q + 2 * math.pi * np.round((target - q) / (2 * math.pi))
        for _ in range(2):                                   # pull back inside the limits
            w = np.where(w > hi + 1e-9, w - 2 * math.pi, w)
            w = np.where(w < lo - 1e-9, w + 2 * math.pi, w)
        w = np.where((w < lo - 1e-9) | (w > hi + 1e-9), q, w)
        return w

    out = [nearest(np.zeros_like(q))]
    if hint is not None:
        h = nearest(np.asarray(hint, dtype=float))
        if not np.allclose(h, out[0], atol=1e-9):
            out.insert(0, h)
    return out


def rot_z4(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    T = np.eye(4)
    T[:2, :2] = [[c, -s], [s, c]]
    return T


class TableChain:
    """Arm plus turntable: a 7-axis chain q7 = (q1..q6, e1) for planning, e1 in radians.

    The turntable turns about the z axis of the work frame W (robot base frame, metres) through its origin; the part
    frame at table angle e1 is P(e1) = W Rz(e1). The arm is `robot` (studio kinematics, tcp_offset used as set).
    Forward kinematics give the TCP in the robot base (independent of e1) and in the part frame (dependent on it)."""

    def __init__(self, robot, work: np.ndarray):
        self.robot = robot
        self.work = np.asarray(work, dtype=float)

    @property
    def dof(self) -> int:
        return self.robot.dof + 1

    def part_frame(self, e1: float) -> np.ndarray:
        return self.work @ rot_z4(float(e1))

    def fk(self, q7) -> np.ndarray:
        """TCP in the robot base frame."""
        return self.robot.tcp(np.asarray(q7, dtype=float)[:-1])

    def fk_part(self, q7) -> np.ndarray:
        """TCP in the part frame at the table angle q7[-1]."""
        q7 = np.asarray(q7, dtype=float)
        return np.linalg.inv(self.part_frame(q7[-1])) @ self.robot.tcp(q7[:-1])

    def to_base(self, T_part: np.ndarray, e1: float) -> np.ndarray:
        """A pose given in the part frame, in the robot base frame at table angle e1."""
        return self.part_frame(e1) @ np.asarray(T_part, dtype=float)

    def to_part(self, T_base: np.ndarray, e1: float) -> np.ndarray:
        return np.linalg.inv(self.part_frame(e1)) @ np.asarray(T_base, dtype=float)


def collides(backend, q, e1=None) -> bool:
    """backend.in_collision with the table angle when the backend models the table as a joint."""
    if e1 is not None and getattr(backend, "table_axis", False):
        return bool(backend.in_collision(q, e1))
    return bool(backend.in_collision(q))


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
            lim = getattr(self.cell, "limits", None)
            out = []
            for s in sols:
                for w in pruned_wraps(s, lim, seed):
                    if not any(np.allclose(w, o, atol=1e-9) for o in out):
                        out.append(w)
            sols = out
        return sols

    def in_collision(self, q, e1=None) -> bool:
        return self.cell.in_collision(q) if e1 is None else self.cell.in_collision(q, ext=[e1])


class CellBackend(TesseractBackend):
    """A Tesseract cell set up for one station: the spindle stack on tool0 (collision geometry; the cutter itself is
    not, it is meant to touch the stone), the part from its STL, and the turntable. Needs tesseract-robotics.

    With the turntable driven as E1 (turntable["driven"] == "e1") the table is a revolute joint in the Tesseract
    scene graph (lithocam.robot.Cell.add_rotary_table) carrying the table top and the part, so every collision
    check turns the part to the sample's table angle (`table_axis` is True, in_collision takes the angle).
    Otherwise the part is a fixed obstacle that place_part moves to each setup's angle."""

    table_axis = False

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
        self.e1 = 0.0
        if turntable and str(turntable.get("driven", "operator")) == "e1":
            from .program import table_settings
            ts = table_settings(turntable)
            d, h = float(turntable.get("diameter", 0.8)), float(turntable.get("height", 0.04))
            lo, hi = (math.radians(v) for v in ts["e1_limits_deg"])
            link = cell.add_rotary_table("turntable_e1", tuple(self.work[:3, 3]), lo, hi,
                                         math.radians(ts["e1_speed_deg_s"]), disc=(d, h))
            if part_stl:
                cell.add_mesh_obstacle("part", part_stl, 1e-3, (0.0, 0.0, 0.0), 0.0, parent=link)
                cell.allow_collision("part", link, "stands on the table")
            self.table_axis = True
            return
        if part_stl:
            cell.add_mesh_obstacle("part", part_stl, 1e-3, tuple(self.work[:3, 3]), 0.0)
        if turntable:
            d, h = float(turntable.get("diameter", 0.8)), float(turntable.get("height", 0.04))
            o = self.work[:3, 3]
            cell.add_box_obstacle("turntable", (d, d, h), (o[0], o[1], o[2] - h / 2 - 0.002))

    def in_collision(self, q, e1=None) -> bool:
        if self.table_axis:
            return self.cell.in_collision(q, ext=[self.e1 if e1 is None else e1])
        return self.cell.in_collision(q)

    def set_tool_mode(self, mode: str):
        """'wire': the bow collides, the spindle does not; 'spindle': the other way round."""
        from tesseract_robotics.tesseract_environment import ChangeLinkCollisionEnabledCommand
        self.cell.set_wire_tool(mode == "wire")
        if "spindle" in [str(n) for n in self.cell.env.getLinkNames()]:
            self.cell.env.applyCommand(ChangeLinkCollisionEnabledCommand("spindle", mode != "wire"))
        self.cell._mgr = None

    def place_part(self, work, yaw: float):
        """Turn the part obstacle with the turntable (with the table as a joint: set its angle)."""
        if self.table_axis:
            self.e1 = float(yaw)
            return
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
    ext: List[float] = field(default_factory=list)     # external-axis value (table angle, rad) per solved rung
    max_ext_step: float = 0.0
    node: List[int] = field(default_factory=list)      # node row per solved rung in its layer (internal)


def ladder(rungs: Sequence[Sequence[np.ndarray]], backend, seed: Optional[Sequence[float]] = None,
           max_step: float = math.radians(25), check_collision: bool = True, ext=None, ext_seed=None,
           ext_max_step: float = math.inf, ext_weight: float = 1.0) -> LadderResult:
    """rungs[k] = candidate tool0 poses (4x4) of path point k. Returns the least-motion joint path.

    `ext[k][c]`: the external-axis value (turntable angle, radians) candidate c of rung k was built for; None = no
    external axis. With it a node is (arm joints, table angle): an edge needs the arm step <= max_step and the table
    step <= ext_max_step, and costs the summed squared arm steps plus ext_weight times the squared table step.
    `ext_seed` is the table angle before the first rung.

    Collision checks are lazy: the graph is built from IK alone, the least-motion path is found, only the nodes on
    it are checked, colliding nodes are removed and the search repeats until the path is clear. A collision check
    costs far more than a graph edge, and most nodes are never on the best path."""
    seed = None if seed is None else np.asarray(seed, dtype=float)
    layers = []                                          # per rung: (Q (n, dof), cand idx (n,), table (n,)|None)
    prev_hint = seed
    for k, poses in enumerate(rungs):
        qs, idx = [], []
        for c, T in enumerate(poses):
            for q in backend.ik(T, prev_hint):
                qs.append(q)
                idx.append(c)
        if qs:
            Q = np.asarray(qs)
            I = np.asarray(idx)
            layers.append((Q, I, None if ext is None else np.asarray(ext[k], dtype=float)[I]))
            prev_hint = Q[0] if prev_hint is None else Q[np.argmin(np.abs(Q - prev_hint).max(1))]
        else:
            layers.append((None, None, None))
    nodes = sum(len(L[0]) for L in layers if L[0] is not None)
    lazy = check_collision and backend.checks_collision
    checked = {}                                         # (rung, joint bytes, table) -> collides
    rejected = 0
    eseed = None if ext is None or ext_seed is None else float(ext_seed)
    # edge weights between rungs j-1 and j, computed once on the full layers and sliced to the nodes still alive:
    # the lazy collision loop re-solves the graph many times and the weights do not change
    full = list(layers)
    alive = [None if L[0] is None else np.arange(len(L[0])) for L in layers]
    wcache = {}

    def edge(j):
        if j not in wcache:
            wcache[j] = _edge_w(full[j - 1][0], full[j][0], full[j - 1][2], full[j][2], max_step, ext_max_step,
                                ext_weight)
        return wcache[j][np.ix_(alive[j - 1], alive[j])]

    for _ in range(500):
        res = _solve_layers(layers, seed, max_step, eseed, ext_max_step, ext_weight, edge)
        if not lazy:
            break
        bad = {}
        for r, q, n in zip(res.rungs, res.joints, res.node):
            e = None if ext is None else float(layers[r][2][n])
            key = (r, np.asarray(q, dtype=float).tobytes(), e)
            if key not in checked:
                checked[key] = collides(backend, q, e)
            if checked[key]:
                bad.setdefault(r, []).append(n)
        if not bad:
            break
        for r, rows in bad.items():
            Q, idx, E = layers[r]
            keep = np.setdiff1d(np.arange(len(Q)), rows)
            rejected += len(rows)
            layers[r] = (Q[keep], idx[keep], None if E is None else E[keep]) if len(keep) else (None, None, None)
            alive[r] = alive[r][keep] if len(keep) else None
    res.nodes = nodes
    res.collisions_rejected = rejected
    return res


def _edge_w(Qa, Qb, Ea, Eb, max_step, ext_max_step=math.inf, ext_weight=1.0) -> np.ndarray:
    """(na, nb) cost of every edge between two rungs; inf where a step is above its limit."""
    d = Qb[None, :, :] - Qa[:, None, :]
    step = np.abs(d).max(axis=2)
    if Ea is not None and Eb is not None:
        de = Eb[None, :] - Ea[:, None]
        return np.where((step <= max_step) & (np.abs(de) <= ext_max_step + 1e-12),
                        np.sum(d * d, axis=2) + ext_weight * de * de, np.inf)
    return np.where(step <= max_step, np.sum(d * d, axis=2), np.inf)


def _solve_layers(layers, seed, max_step, ext_seed=None, ext_max_step=math.inf, ext_weight=1.0,
                  edge=None) -> LadderResult:
    """Least-cost runs over the layers. `edge(j)`: the weight matrix between layers j-1 and j (cached by the
    caller); None = computed here."""
    res = LadderResult([], [], [])
    k = 0
    start = seed
    estart = ext_seed
    while k < len(layers):
        if layers[k][0] is None:
            res.missing.append(k)
            k += 1
            continue
        # one run of consecutive rungs connected by steps <= max_step
        Q0 = layers[k][0]
        cost = np.zeros(len(Q0)) if start is None else np.sum((Q0 - start) ** 2, axis=1)
        if layers[k][2] is not None and estart is not None:
            cost = cost + ext_weight * (layers[k][2] - estart) ** 2
        back = []
        run = [k]
        j = k + 1
        while j < len(layers) and layers[j][0] is not None:
            Qb = layers[j][0]
            w = edge(j) if edge is not None else _edge_w(layers[run[-1]][0], Qb, layers[run[-1]][2], layers[j][2],
                                                         max_step, ext_max_step, ext_weight)
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
            res.node.append(int(n))
            if layers[r][2] is not None:
                res.ext.append(float(layers[r][2][n]))
        start = res.joints[-1]
        if res.ext:
            estart = res.ext[-1]
        if j < len(layers) and layers[j][0] is not None:
            res.breaks.append(j)
        k = j
    steps = [float(np.max(np.abs(res.joints[i] - res.joints[i - 1]))) for i in range(1, len(res.joints))]
    res.max_step = max(steps) if steps else 0.0
    if len(res.ext) > 1:
        res.max_ext_step = float(np.max(np.abs(np.diff(res.ext))))
    return res
