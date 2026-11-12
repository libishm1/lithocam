# Copyright 2026 The lithocam authors. SPDX-License-Identifier: Apache-2.0
"""Station and program model of the studio, and its expansion into a joint trajectory and post records.

A station holds one robot cell, a work frame with a part and a stock, the tool library and programs. A program is
an ordered list of steps (catalog.StepSpec ids with parameters). Expansion walks the steps:

  movej        joint interpolation to the stored joints;
  movel        TCP pose interpolation (straight line, rotation about a fixed axis), damped least-squares IK;
  machining    the generator's CL path (mm, work frame), tool axis along -z of the work frame, IK point by point
               seeded with the previous solution; the approach comes in from the clearance plane;
  others       post records only (tool, speed, I/O, pause, comment).

Everything is pure Python (numpy + the C++ core) so it is testable without Qt.
"""
from __future__ import annotations

import json
import math
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from .. import post
from . import catalog, spindles
from .kinematics import Robot, axis_angle, load_urdf, rotation_error, transform

def _cells_dir() -> Path:
    """LITHOCAM_CELLS, else the bundle's cells/ in a frozen executable, else the repo's cells/."""
    import os
    import sys
    if os.environ.get("LITHOCAM_CELLS"):
        return Path(os.environ["LITHOCAM_CELLS"])
    if getattr(sys, "frozen", False):
        return Path(getattr(sys, "_MEIPASS", Path(sys.executable).parent)) / "cells"
    return Path(__file__).resolve().parents[4] / "cells"


CELLS = _cells_dir()
ROBOTS = {"ur10e": ("UR10e", "ur10e.urdf", "urscript"), "kr120": ("KUKA KR120 R2500 PRO", "kr120.urdf", "krl")}
# Controller base frame in the URDF base_link frame. UR controllers' "base" is base_link turned by pi about z
# (ur_description); KUKA and ABB writers post joint targets only, so their base frame is not needed yet.
CONTROLLER_BASE = {"ur10e": np.diag([-1.0, -1.0, 1.0, 1.0])}
POST_OPTIONS = {"ur10e": {"cut_motion": "movep", "blend": 0.0005}, "kr120": {}}
def post_options(robot: str, dialect: str) -> Dict[str, object]:
    """Writer options for a cell: cutting motion, blend and the controller base frame (URScript only)."""
    if dialect != "urscript" or robot not in CONTROLLER_BASE:
        return {}
    return dict(POST_OPTIONS.get(robot, {}), base_frame=CONTROLLER_BASE[robot])


HOME = {"ur10e": [0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0], "kr120": [0.0, -1.5708, 1.5708, 0.0, 0.5, 0.0]}


@dataclass
class Step:
    kind: str
    params: Dict[str, object] = field(default_factory=dict)
    label: str = ""

    @property
    def spec(self) -> catalog.StepSpec:
        return catalog.BY_ID[self.kind]

    def title(self) -> str:
        if self.label:
            return self.label
        t = self.spec.label
        tool = self.params.get("tool")
        return "{} [{}]".format(t, tool) if tool else t


@dataclass
class Program:
    name: str = "Main"
    steps: List[Step] = field(default_factory=list)


@dataclass
class Station:
    robot: str = "ur10e"
    work_xyz: List[float] = field(default_factory=lambda: [0.7, 0.0, 0.0])   # m, robot base frame
    work_rpy: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])   # deg
    part_path: str = ""
    stock: Optional[Dict[str, float]] = None                                  # mm, work frame
    tool_library: str = ""                                                    # CSV; "" = built-in survey tools
    tool_length: float = 0.0                                                  # mm; 0 = from the tool
    spindle: Optional[Dict[str, object]] = None                               # None = the cell's default preset
    ruled_path: str = ""          # ruled-face JSON of the part (python -m lithocam.ruled part.step --json)
    turntable: Optional[Dict[str, float]] = None   # {"diameter": m, "height": m}: the part sits on a rotary table
    programs: List[Program] = field(default_factory=lambda: [Program()])

    def work_frame(self) -> np.ndarray:
        return transform(self.work_xyz, [math.radians(a) for a in self.work_rpy])

    def spindle_model(self) -> "spindles.Spindle":
        if self.spindle:
            return spindles.from_dict(self.spindle)
        return spindles.PRESETS[spindles.DEFAULT_FOR_ROBOT.get(self.robot, "ur10e_example")]

    def tool_tcp(self, tool) -> np.ndarray:
        """Cutter tip in the flange frame (metres): the spindle stack plus the cutter's stickout, or the tool
        length override measured from the flange."""
        if self.tool_length > 0:
            T = np.eye(4)
            T[2, 3] = self.tool_length / 1000
            return T
        return self.spindle_model().tcp(spindles.stickout(tool))

    def to_json(self) -> str:
        return json.dumps(asdict(self), indent=1)

    @staticmethod
    def load(path) -> "Station":
        """Read a station file; relative part, ruled and tool-library paths are relative to the file."""
        path = Path(path)
        st = Station.from_json(path.read_text(encoding="utf-8"))
        for k in ("part_path", "ruled_path", "tool_library"):
            v = getattr(st, k)
            if v and not Path(v).is_absolute():
                setattr(st, k, str((path.parent / v).resolve()))
        return st

    @staticmethod
    def from_json(text: str) -> "Station":
        d = json.loads(text)
        progs = [Program(p["name"], [Step(**s) for s in p["steps"]]) for p in d.pop("programs", [])]
        st = Station(**d)
        st.programs = progs or [Program()]
        return st


def load_robot(name: str) -> Robot:
    return load_urdf(CELLS / ROBOTS[name][1])


def down_pose(work: np.ndarray, p_mm: Sequence[float]) -> np.ndarray:
    """TCP pose with the tool axis along -z of the work frame (x kept), at a work-frame point in mm."""
    T = np.eye(4)
    T[:3, :3] = work[:3, :3] @ np.diag([1.0, -1.0, -1.0])
    T[:3, 3] = work[:3, :3] @ (np.asarray(p_mm, dtype=float) / 1000) + work[:3, 3]
    return T


def interp_pose(A: np.ndarray, B: np.ndarray, t: float) -> np.ndarray:
    T = np.eye(4)
    T[:3, 3] = (1 - t) * A[:3, 3] + t * B[:3, 3]
    w = rotation_error(A[:3, :3], B[:3, :3])
    a = np.linalg.norm(w)
    T[:3, :3] = (axis_angle(w, a * t) if a > 1e-12 else np.eye(3)) @ A[:3, :3]
    return T


@dataclass
class Segment:
    step: int                       # index of the program step
    label: str
    q: np.ndarray                   # (n, dof) joint samples
    feed: np.ndarray                # (n,) bool: cutting move
    ok: np.ndarray                  # (n,) bool: IK converged
    tcp: np.ndarray                 # (n, 3) TCP positions, metres, robot base frame
    tcp_offset: np.ndarray = field(default_factory=lambda: np.eye(4))   # active tool TCP in the flange frame
    tool: str = ""                  # active tool id
    yaw: float = 0.0                # turntable angle (radians) during this segment


@dataclass
class Expansion:
    segments: List[Segment]
    records: List[post.Rec]
    messages: List[str]

    def joints(self) -> np.ndarray:
        qs = [s.q for s in self.segments if len(s.q)]
        return np.vstack(qs) if qs else np.zeros((0, 6))

    @property
    def unreachable(self) -> int:
        return int(sum((~s.ok).sum() for s in self.segments))


def lc_kinds_without_shank():
    """Tool kinds that do not sit in a collet (arbor-mounted or not rotary)."""
    from .core import lc
    K = lc.ToolKind
    return (K.DiscSaw, K.DiamondWire, K.MultiWireSaw, K.GangSaw, K.ChainSaw, K.ChainMortiser, K.BandSaw,
            K.PlanerHead, K.Waterjet, K.Knife, K.BushHammer, K.Chisel, K.Splitter, K.FlameTorch, K.AggregateHead,
            K.FaceMill, K.CupWheel, K.ProfileWheel, K.PolishingPad, K.AbrasiveBrush, K.GrindingDisc)


def solve_start(robot: Robot, target: np.ndarray, q_now: Sequence[float], tries: int = 24,
                seed: int = 7) -> Tuple[np.ndarray, bool]:
    """Pose solve from the current joints, then from home and random seeds; keeps the solution closest to q_now."""
    rng = np.random.default_rng(seed)
    best, best_d = None, np.inf
    q_now = np.asarray(q_now, dtype=float)
    seeds = [q_now] + [np.asarray(h, dtype=float) for h in HOME.values() if len(h) == robot.dof]
    seeds += [rng.uniform(robot.lower / 2, robot.upper / 2) for _ in range(tries)]
    for s in seeds:
        q, ok = robot.solve_pose(target, s)
        if ok:
            d = float(np.linalg.norm(q - q_now))
            if d < best_d:
                best, best_d = q, d
                if d < 0.5:
                    break
    return (best, True) if best is not None else (q_now, False)


WIRE_BOW = {"ur10e": 0.25, "kr120": 0.6}   # wire depth along tool0 z of each cell's bow saw (cells/make_cells.py)


def wire_rungs(face, work: np.ndarray, yaw: float, wire_radius: float, bow: float, rolls_deg: Sequence[float],
               lead: bool = True):
    """Candidate tool0 poses per ruling (one per roll) for a ruled face given in mm in the part frame, on a
    turntable at `yaw`; rulings ordered along the sweep, with one lead-in and one lead-out ruling outside the face."""
    from ..wirepath import order_rulings, turned_face, wire_target
    f = turned_face(face, yaw)
    rs = order_rulings(f.rulings)
    R, o = work[:3, :3], work[:3, 3]
    lines = [(R @ (np.asarray(r.p0) / 1000) + o, R @ (np.asarray(r.p1) / 1000) + o, R @ np.asarray(r.normal))
             for r in rs]
    if lead and len(lines) > 1:
        a, b = lines[0], lines[1]
        lines = [(2 * a[0] - b[0], 2 * a[1] - b[1], a[2])] + lines
        a, b = lines[-1], lines[-2]
        lines.append((2 * a[0] - b[0], 2 * a[1] - b[1], a[2]))
    return [[wire_target(p0, p1, n, wire_radius, math.radians(rd), bow) for rd in rolls_deg] for p0, p1, n in lines]


def _wire_step(i, st, p, station, robot, tools, backend, q, add, recs, msgs, yaw):
    """Diamond-wire sweep over the ruled faces labelled 'wire': per face the turntable turns the face towards the
    robot, then the ladder planner picks roll and IK branch per ruling."""
    from ..wirepath import facing_yaw, load_ruled_json
    from .planner import NumpyBackend, ladder
    if not station.ruled_path:
        msgs.append("step {}: wire cut needs the part's ruled-face JSON (Station.ruled_path)".format(i + 1))
        return q
    rep = load_ruled_json(station.ruled_path)
    want = str(p.get("faces", "all"))
    faces = [f for f in rep.faces if f.label == "wire" and len(f.rulings) >= 2]
    if want != "all":
        keep = {int(s) for s in want.replace(";", ",").split(",") if s.strip()}
        faces = [f for f in faces if f.index in keep]
    tid = str(p.get("tool") or "")
    tool = tools.get(tid)
    wire_r = (tool.diameter / 2000) if tool is not None and tool.diameter > 0 else 0.0035
    bow = WIRE_BOW.get(station.robot, 0.25)
    robot.tcp_offset = np.eye(4)                          # wire targets are tool0 poses (the bow is in the URDF)
    if tid:
        recs.append(post.Rec("tool", {"id": tid, "tcp": np.eye(4).tolist()}))
    be = backend or NumpyBackend(robot, [HOME.get(station.robot, [0.0] * robot.dof)])
    if hasattr(be, "set_tool_mode"):
        be.set_tool_mode("wire")
    rolls = [float(v) for v in str(p.get("rolls", ",".join(str(a) for a in range(-90, 91, 10)))).split(",")]
    max_step = math.radians(float(p.get("max_step_deg", 30.0)))
    lead = bool(p.get("lead", True))
    work = station.work_frame()
    for f in faces:
        nz = float(np.mean([np.asarray(r.normal) for r in f.rulings], axis=0)[2])
        if station.turntable is not None and nz < -0.7:
            msgs.append("step {}: wire face {} faces down onto the table; cut it before the part is set down".format(
                i + 1, f.index))
            continue
        if station.turntable is not None and p.get("auto_table", True) and nz < 0.7:   # top faces: keep the angle
            yaw[0] = facing_yaw(f, work[:3, 3])
            recs.append(post.Rec("table", {"deg": math.degrees(yaw[0])}))
            if hasattr(be, "place_part"):
                be.place_part(work, yaw[0])
        rungs = wire_rungs(f, work, yaw[0], wire_r, bow, rolls, lead)
        res = ladder(rungs, be, seed=q, max_step=max_step)
        if lead and len(rungs) > 3:   # lead rulings are optional: drop one that only connects through a jump
            drop_in = 1 in res.breaks or 0 in res.missing
            drop_out = (len(rungs) - 1) in res.breaks or (len(rungs) - 1) in res.missing
            if drop_in or drop_out:
                rungs = rungs[1 if drop_in else 0: len(rungs) - 1 if drop_out else len(rungs)]
                res = ladder(rungs, be, seed=q, max_step=max_step)
                msgs.append("step {}: wire face {}: lead-{} dropped (it only connects through a jump)".format(
                    i + 1, f.index, "in and lead-out" if drop_in and drop_out else ("in" if drop_in else "out")))
        if not res.joints:
            msgs.append("step {}: wire face {}: no reachable ruling".format(i + 1, f.index))
            continue
        qa = np.asarray(res.joints)
        # approach: joint move from the current configuration to the first ruling
        n0 = max(2, int(np.max(np.abs(qa[0] - q)) / math.radians(2)) + 1)
        appr = [q + (qa[0] - q) * t for t in np.linspace(0, 1, n0)]
        add(i, "{} face {} approach".format(st.title(), f.index), appr, [False] * n0, [True] * n0)
        add(i, "{} face {}".format(st.title(), f.index), qa, [True] * len(qa), [True] * len(qa))
        recs.append(post.Rec("comment", {"text": "wire face {}: {} rulings, table {:.1f} deg".format(
            f.index, len(qa), math.degrees(yaw[0]))}))
        recs.append(post.Rec("movej", {"q": qa[0].tolist()}))
        for qq in qa[1:]:
            recs.append(post.Rec("movej", {"q": qq.tolist(), "cut": True}))
        msgs.append("step {}: wire face {}: {} of {} rulings, {} missing, {} breaks, largest step {:.1f} deg, "
                    "{} nodes, {} colliding nodes rejected ({})".format(
                        i + 1, f.index, len(qa), len(rungs), len(res.missing), len(res.breaks),
                        math.degrees(res.max_step), res.nodes, res.collisions_rejected, be.name))
        q = qa[-1]
    return q


def expand(station: Station, robot: Robot, program: Program, tools: Dict[str, object], part=None,
           q0: Optional[Sequence[float]] = None, max_feed_points: int = 4000, progress=None,
           backend=None) -> Expansion:
    """Expand a program. `part` = (tris, trimesh) in mm, needed by machining steps. `tools` maps id -> Tool.
    `backend`: a planner backend (studio.planner) for IK branches and collision checks; None = numpy IK only."""
    from . import cam

    work = station.work_frame()
    q = np.asarray(q0 if q0 is not None else HOME.get(station.robot, [0.0] * robot.dof), dtype=float)
    segs: List[Segment] = []
    recs: List[post.Rec] = [post.Rec("start", {"name": "".join(c if c.isalnum() else "_" for c in program.name)})]
    msgs: List[str] = []
    tool = None

    sp = station.spindle_model()
    yaw = [0.0]                     # turntable angle, radians (a list so the helpers can update it)

    def mount(i, tid):
        """Put the cutter in the spindle: TCP, tool record (with the spindle mass), collet check."""
        t = tools[tid]
        robot.tcp_offset = station.tool_tcp(t)
        recs.append(post.Rec("tool", {"id": tid, "tcp": robot.tcp_offset.tolist(), "mass": sp.mass_kg}))
        shank = float(getattr(t, "shank_diameter", 0.0) or 0.0) or float(getattr(t, "diameter", 0.0) or 0.0)
        if sp.max_collet_mm and shank > sp.max_collet_mm + 1e-9 and t.kind not in (
                lc_kinds_without_shank()):
            msgs.append("step {}: {} shank {:g} mm exceeds the {} collet ({:g} mm max)".format(
                i + 1, tid, shank, sp.collet, sp.max_collet_mm))
        return t

    def add(i, label, qs, feed, ok):
        qs = np.asarray(qs, dtype=float).reshape(-1, robot.dof)
        tcp = np.array([robot.tcp(x)[:3, 3] for x in qs]) if len(qs) else np.zeros((0, 3))
        segs.append(Segment(i, label, qs, np.asarray(feed, bool), np.asarray(ok, bool), tcp,
                            robot.tcp_offset.copy(), getattr(tool, "id", "") if tool is not None else "", yaw[0]))

    for i, st in enumerate(program.steps):
        spec = catalog.BY_ID.get(st.kind)
        if spec is None:
            msgs.append("step {}: unknown kind {}".format(i + 1, st.kind))
            continue
        p = dict(catalog.default_params(spec), **st.params)
        if progress:
            progress(i, len(program.steps), st.title())
        if st.kind in ("set_tool", "tool_change"):
            tid = str(p.get("tool", ""))
            if tid in tools:
                tool = mount(i, tid)
            else:
                msgs.append("step {}: tool '{}' is not in the library".format(i + 1, tid))
        elif st.kind == "movej":
            target = np.radians(np.asarray(p["joints"], dtype=float)) if p.get("joints") is not None else q
            n = max(2, int(np.max(np.abs(target - q)) / math.radians(2)) + 1)
            qs = [q + (target - q) * t for t in np.linspace(0, 1, n)]
            add(i, st.title(), qs, [False] * n, [True] * n)
            recs.append(post.Rec("movej", {"q": target.tolist()}))
            q = target
        elif st.kind == "movel":
            if p.get("pose") is None:
                msgs.append("step {}: linear move without a pose".format(i + 1))
                continue
            B = np.asarray(p["pose"], dtype=float)
            A = robot.tcp(q)
            n = max(2, int(np.linalg.norm(B[:3, 3] - A[:3, 3]) / 0.005) + 1)
            qs, oks = [], []
            for t in np.linspace(0, 1, n):
                q, ok = robot.solve_pose(interp_pose(A, B, t), q)
                qs.append(q)
                oks.append(ok)
            add(i, st.title(), qs, [False] * n, oks)
            recs.append(post.Rec("movel", {"q": q.tolist(), "pose": B.tolist()}))
            if not all(oks):
                msgs.append("step {}: {} of {} linear samples did not converge".format(i + 1, n - sum(oks), n))
        elif st.kind == "speed":
            recs.append(post.Rec("speed", {"linear": p["linear"], "joint": p["joint"] / 100}))
        elif st.kind == "pause":
            recs.append(post.Rec("pause", {"seconds": p["seconds"]}))
        elif st.kind == "io":
            recs.append(post.Rec("io_set" if p["mode"] == "set" else "io_wait",
                                 {"port": p["port"], "value": p["value"]}))
        elif st.kind == "gripper":
            recs.append(post.Rec("gripper", {"width": p["width"], "force": p["force"]}))
        elif st.kind == "comment":
            recs.append(post.Rec("comment", {"text": p["text"]}))
        elif st.kind == "call":
            recs.append(post.Rec("call", {"name": p["name"]}))
        elif st.kind == "setup":
            if station.turntable is not None:
                yaw[0] = math.radians(float(p.get("yaw", 0.0)))
                recs.append(post.Rec("table", {"deg": math.degrees(yaw[0])}))
        elif st.kind == "wire_ruled":
            q = _wire_step(i, st, p, station, robot, tools, backend, q, add, recs, msgs, yaw)
        elif st.kind in cam.GENERATORS:
            tid = str(p.get("tool") or "")
            if tid:
                if tid not in tools:
                    msgs.append("step {}: tool '{}' is not in the library".format(i + 1, tid))
                    continue
                tool = mount(i, tid)
            if tool is None or part is None or station.stock is None:
                msgs.append("step {}: {} needs a part, a stock and a tool".format(i + 1, spec.label))
                continue
            tris, mesh = part
            try:
                moves = cam.GENERATORS[st.kind](tris, mesh, tool, station.stock,
                                                **{k: v for k, v in p.items() if k != "tool"})
            except Exception as e:   # noqa: BLE001 - report generator errors in the program log
                msgs.append("step {}: {} failed: {}".format(i + 1, spec.label, e))
                continue
            if len(moves) > max_feed_points:   # thin the robot path; the CL path stays full for display and posting
                keep = np.unique(np.r_[np.linspace(0, len(moves) - 1, max_feed_points).astype(int),
                                       [k for k, (kind, _) in enumerate(moves) if kind == "rapid"]])
                rmoves = [moves[k] for k in keep]
            else:
                rmoves = moves
            Rz = np.eye(4)
            Rz[:3, :3] = np.array([[math.cos(yaw[0]), -math.sin(yaw[0]), 0], [math.sin(yaw[0]), math.cos(yaw[0]), 0],
                                   [0, 0, 1.0]])
            wk = work @ Rz                                   # the work frame turned with the table
            first = down_pose(wk, rmoves[0][1])
            q, ok = solve_start(robot, first, q)
            if not ok:
                msgs.append("step {}: the start of {} is out of reach".format(i + 1, spec.label))
            qs, feeds, oks = [], [], []
            for kind, pt in rmoves:
                q, ok = robot.solve_pose(down_pose(wk, pt), q)
                qs.append(q)
                feeds.append(kind == "feed")
                oks.append(ok)
            if backend is not None and getattr(backend, "checks_collision", False):
                if hasattr(backend, "set_tool_mode"):
                    backend.set_tool_mode("spindle")
                if hasattr(backend, "place_part"):
                    backend.place_part(work, yaw[0])
                hits = [k for k, qq in enumerate(qs) if backend.in_collision(qq)]
                for k in hits:
                    oks[k] = False
                msgs.append("step {}: collision check ({}): {} of {} samples in contact".format(
                    i + 1, backend.name, len(hits), len(qs)))
            add(i, st.title(), qs, feeds, oks)
            if p.get("rpm") or getattr(tool, "spindle_rpm", 0):
                recs.append(post.Rec("spindle", {"rpm": p.get("rpm") or tool.spindle_rpm}))
            recs.append(post.Rec("comment", {"text": "{} with {} ({} points)".format(spec.label, tid or "tool",
                                                                                   len(rmoves))}))
            for (kind, pt), qq in zip(rmoves, qs):
                if kind == "feed":   # cutting: pose of the cutter tip in the robot base (URDF) frame
                    recs.append(post.Rec("movel", {"q": qq.tolist(), "pose": down_pose(wk, pt).tolist(),
                                                   "cut": True}))
                else:
                    recs.append(post.Rec("movej", {"q": qq.tolist()}))
            bad = len(oks) - sum(oks)
            if bad:
                msgs.append("step {}: {} of {} points did not converge (reach or singularity)".format(
                    i + 1, bad, len(oks)))
            feed_len, rapid_len = cam.path_length(moves)
            msgs.append("step {}: {}: {} CL points, feed {:.0f} mm, rapid {:.0f} mm".format(
                i + 1, spec.label, len(moves), feed_len, rapid_len))
        else:
            recs.append(post.Rec("comment", {"text": "{} ({}): no generator yet".format(spec.label, spec.status)}))
            msgs.append("step {}: {} is {}; posted as a comment".format(i + 1, spec.label, spec.status))
    recs.append(post.Rec("end"))
    return Expansion(segs, recs, msgs)
