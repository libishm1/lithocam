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
ROBOTS = {"ur10e": ("UR10e", "ur10e.urdf", "urscript"), "kr120": ("KUKA KR120 R2500 PRO", "kr120.urdf", "krl"),
          # dimensions from the public KUKA KR 500 FORTEC manual (cells/make_cells.py); KRL post
          "kr480_r3330_mt": ("KUKA KR 480 R3330 MT", "kr480_r3330_mt.urdf", "krl"),
          "kr500_r2830_mt": ("KUKA KR 500 R2830 MT", "kr500_r2830_mt.urdf", "krl")}
# Controller base frame in the URDF base_link frame. UR controllers' "base" is base_link turned by pi about z
# (ur_description); KUKA and ABB writers post joint targets only, so their base frame is not needed yet.
CONTROLLER_BASE = {"ur10e": np.diag([-1.0, -1.0, 1.0, 1.0])}
POST_OPTIONS = {"ur10e": {"cut_motion": "movep", "blend": 0.0005}, "kr120": {}, "kr480_r3330_mt": {}, "kr500_r2830_mt": {}}
def post_options(robot: str, dialect: str) -> Dict[str, object]:
    """Writer options for a cell: cutting motion, blend and the controller base frame (URScript only)."""
    if dialect != "urscript" or robot not in CONTROLLER_BASE:
        return {}
    return dict(POST_OPTIONS.get(robot, {}), base_frame=CONTROLLER_BASE[robot])


HOME = {"ur10e": [0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0], "kr120": [0.0, -1.5708, 1.5708, 0.0, 0.5, 0.0],
        "kr480_r3330_mt": [0.0, -1.5708, 1.5708, 0.0, 0.5, 0.0], "kr500_r2830_mt": [0.0, -1.5708, 1.5708, 0.0, 0.5, 0.0]}


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
    material: str = "granite"     # workpiece material class (lithocam.cutting_data.MATERIALS)
    rapid_speed: float = 100.0    # mm/s for links and retracts near the stone (a cell setting; set it for your cell)
    robot_factor: float = 1.0     # feed factor for robot stiffness (user-set; no published value)
    ruled_path: str = ""          # ruled-face JSON of the part (python -m lithocam.ruled part.step --json)
    # {"diameter": m, "height": m}: the part sits on a rotary table. "driven" says who turns it: "operator" (default:
    # the program stops and asks), "e1" (a coordinated external axis E1 of a KUKA or ABB controller; with it
    # "e1_limits_deg", "e1_speed_deg_s" and "e1_max_step_deg", placeholders until set for the cell) or "io" (a PLC
    # turns it on an output request and reports done on an input: "io" holds the actions, see table_io_ops)
    turntable: Optional[Dict[str, object]] = None
    # cell I/O definition (lithocam.cell_io JSON): "" = none (process records post as comments, with a warning);
    # "default" = the cell's shipped placeholder file cells/<robot>_io.json; else a path
    cell_io: str = ""
    programs: List[Program] = field(default_factory=lambda: [Program()])
    # named robot positions: {"name", "joints" (deg), "pose" (4x4 tool pose, m, robot base), "tool" (id or "")}
    targets: List[Dict[str, object]] = field(default_factory=list)

    def target(self, name: str) -> Optional[Dict[str, object]]:
        return next((t for t in self.targets if t.get("name") == name), None)

    def add_target(self, joints_rad, pose, tool: str = "", name: str = "") -> Dict[str, object]:
        """Store a position under a new or given name (an existing name is overwritten)."""
        if not name:
            k = len(self.targets) + 1
            while self.target("T{}".format(k)) is not None:
                k += 1
            name = "T{}".format(k)
        t = {"name": name, "joints": [math.degrees(v) for v in joints_rad],
             "pose": np.asarray(pose, dtype=float).tolist(), "tool": tool}
        old = self.target(name)
        if old is not None:
            old.update(t)
            return old
        self.targets.append(t)
        return t

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
        """Read a station file; relative part, ruled, tool-library and cell I/O paths are relative to the file."""
        path = Path(path)
        st = Station.from_json(path.read_text(encoding="utf-8"))
        for k in ("part_path", "ruled_path", "tool_library", "cell_io"):
            v = getattr(st, k)
            if v and v != "default" and not Path(v).is_absolute():
                setattr(st, k, str((path.parent / v).resolve()))
        return st

    @staticmethod
    def from_json(text: str) -> "Station":
        d = json.loads(text)
        progs = [Program(p["name"], [Step(**s) for s in p["steps"]]) for p in d.pop("programs", [])]
        from dataclasses import fields as _fields
        known = {f.name for f in _fields(Station)}
        st = Station(**{k: v for k, v in d.items() if k in known})   # extra keys (a title for menus) are kept out
        st.programs = progs or [Program()]
        return st


def load_robot(name: str) -> Robot:
    return load_urdf(CELLS / ROBOTS[name][1])


# ---------------------------------------------------------------- turntable modes
TABLE_MODES = ("operator", "e1", "io")
# E1 axis values used when the station does not set them. PLACEHOLDERS: no value comes from a real table; set the
# travel, speed and the largest table step per wire ruling for your cell.
TABLE_E1_DEFAULTS = {"e1_limits_deg": [-360.0, 360.0], "e1_speed_deg_s": 30.0, "e1_max_step_deg": 10.0}
# dialects that post a coordinated external axis E1 (KRL E6AXIS/E6POS, RAPID extax eax_a)
E1_DIALECTS = ("krl", "rapid")


def table_settings(turntable: Optional[Dict[str, object]]) -> Dict[str, object]:
    """The turntable's mode and E1 axis values with the placeholder defaults filled in; "placeholders" lists the
    E1 keys the station did not set."""
    tt = dict(turntable or {})
    out = {"driven": str(tt.get("driven", "operator") or "operator")}
    out["placeholders"] = [k for k in TABLE_E1_DEFAULTS if tt.get(k) is None]
    for k, v in TABLE_E1_DEFAULTS.items():
        out[k] = tt[k] if tt.get(k) is not None else v
    out["e1_limits_deg"] = [float(v) for v in out["e1_limits_deg"]]
    out["e1_speed_deg_s"] = float(out["e1_speed_deg_s"])
    out["e1_max_step_deg"] = float(out["e1_max_step_deg"])
    out["io"] = tt.get("io")
    return out


def check_table(station: "Station") -> Tuple[str, List[str], List[str]]:
    """(mode used, errors, warnings) for the station's turntable. The mode used is "" without a turntable. A mode the
    cell cannot run is an error (posting is refused) and the expansion goes on in operator mode."""
    if station.turntable is None:
        return "", [], []
    ts = table_settings(station.turntable)
    mode = ts["driven"]
    errors: List[str] = []
    warns: List[str] = []
    dialect = ROBOTS.get(station.robot, ("", "", ""))[2]
    if mode not in TABLE_MODES:
        errors.append("turntable driven '{}' is not one of {}".format(mode, ", ".join(TABLE_MODES)))
        return "operator", errors, warns
    if mode == "e1":
        if station.robot == "ur10e" or dialect not in E1_DIALECTS:
            errors.append("the {} has no external axis: its {} program cannot drive the turntable as a coordinated "
                          "axis E1. Set the turntable to driven 'operator' (the program stops for the operator) or "
                          "'io' (a PLC turns it through the cell I/O).".format(
                              ROBOTS.get(station.robot, (station.robot,))[0], dialect or "controller"))
            return "operator", errors, warns
        lo, hi = ts["e1_limits_deg"]
        if not lo < hi or ts["e1_speed_deg_s"] <= 0 or ts["e1_max_step_deg"] <= 0:
            errors.append("turntable E1: limits must be [lo, hi] with lo < hi, speed and max step > 0")
            return "operator", errors, warns
        if ts["placeholders"]:
            warns.append("WARNING: turntable E1 {} are PLACEHOLDER values ({}): set them for your cell".format(
                ", ".join(ts["placeholders"]), ", ".join("{} {}".format(k, ts[k]) for k in ts["placeholders"])))
        warns.append("WARNING: E1 = the turntable angle about +z of the work frame, 0 = the part as loaded; the "
                     "controller's external-axis configuration (KUKA $EX_AX_NUM and $ETx machine data, ABB mechanical "
                     "unit) is not written by lithocam and must match it")
    if mode == "io":
        try:
            table_io_ops(station, 0.0)
        except ValueError as e:
            errors.append(str(e))
            return "operator", errors, warns
    return mode, errors, warns


def table_io_ops(station: "Station", deg: float) -> List[list]:
    """Neutral I/O operations (lithocam.cell_io) that ask the cell PLC to turn the table to `deg` and wait for it.

    turntable["io"] = {"angle": {"register": name|index, "type": "float"|"int", "per_deg": 1.0} (optional: the target
    angle written to a register or group output), "request": [actions] (cell I/O actions, e.g. a digital output
    pulse), "done": {"di": port, "value": true} (the PLC's in-position input), "release": [actions] (optional, after
    done)}. Ports are checked for the station's controller like a cell I/O file. No value comes from a real cell."""
    from .. import cell_io
    io = (station.turntable or {}).get("io")
    dialect = ROBOTS.get(station.robot, ("", "", "urscript"))[2]
    if not isinstance(io, dict) or not io.get("request") or not isinstance(io.get("done"), dict):
        raise ValueError("turntable driven 'io' needs turntable.io with 'request' actions and a 'done' input "
                         "(cell I/O action syntax), set for your cell")
    bad = set(io) - {"angle", "request", "done", "release", "note"}
    if bad:
        raise ValueError("turntable.io: unknown key(s) {}".format(sorted(bad)))
    ops: List[list] = [["comment", "turntable to {:.3f} deg by the cell PLC".format(float(deg))]]
    a = io.get("angle")
    if a is not None:
        if not isinstance(a, dict) or "register" not in a or set(a) - {"register", "type", "per_deg", "note"}:
            raise ValueError("turntable.io.angle needs {'register': ..., 'type': 'float'|'int', 'per_deg': ...}")
        typ = a.get("type", "float")
        v = float(deg) * float(a.get("per_deg", 1.0))
        cell_io._check_action("turntable.io.angle", {"register": a["register"], "value": v, "type": typ}, dialect)
        ops.append(["register", a["register"], int(round(v)) if typ == "int" else v, typ])
    cell_io._check_actions("turntable.io.request", io["request"], dialect)
    cell_io._check_input("turntable.io.done", io["done"], dialect)
    cell_io._check_actions("turntable.io.release", io.get("release", []), dialect)
    acts = cell_io.CellIO({"controller": dialect})
    fmt = {"rpm": "0", "tool": "", "pocket": ""}
    ops += [list(o) for o in acts._acts(io["request"], fmt)]
    ops.append(["wait_di", io["done"]["di"], bool(io["done"].get("value", True))])
    ops += [list(o) for o in acts._acts(io.get("release", []), fmt)]
    return ops


def table_record(station: "Station", mode: str, deg: float) -> post.Rec:
    """The table record of a setup or face change: operator mode keeps the plain {"deg"} record."""
    if mode == "e1":
        return post.Rec("table", {"deg": float(deg), "driven": "e1"})
    if mode == "io":
        return post.Rec("table", {"deg": float(deg), "driven": "io", "ops": table_io_ops(station, deg)})
    return post.Rec("table", {"deg": float(deg)})


def down_pose(work: np.ndarray, p_mm: Sequence[float]) -> np.ndarray:
    """TCP pose with the tool axis along -z of the work frame (x kept), at a work-frame point in mm."""
    T = np.eye(4)
    T[:3, :3] = work[:3, :3] @ np.diag([1.0, -1.0, -1.0])
    T[:3, 3] = work[:3, :3] @ (np.asarray(p_mm, dtype=float) / 1000) + work[:3, 3]
    return T


def tool_pose(work: np.ndarray, p_mm: Sequence[float], axis=None, spin: float = 0.0) -> np.ndarray:
    """TCP pose at a work-frame point (mm). `axis`: unit tool axis in the work frame, from the tip to the holder
    (None = the work z axis, as down_pose). `spin` (rad) turns the tool about its axis; a spindle tool cuts the
    same at every spin, so the robot planner chooses it."""
    if axis is None:
        T = down_pose(work, p_mm)
    else:
        a = np.asarray(axis, dtype=float)
        a = a / np.linalg.norm(a)
        z = -(work[:3, :3] @ a)                              # tool z points from the flange to the tip
        xr = work[:3, 0] - z * (work[:3, 0] @ z)
        if np.linalg.norm(xr) < 1e-6:
            xr = work[:3, 1] - z * (work[:3, 1] @ z)
        x = xr / np.linalg.norm(xr)
        T = np.eye(4)
        T[:3, :3] = np.column_stack([x, np.cross(z, x), z])
        T[:3, 3] = work[:3, :3] @ (np.asarray(p_mm, dtype=float) / 1000) + work[:3, 3]
    if spin:
        T = T.copy()
        T[:3, :3] = T[:3, :3] @ axis_angle(np.array([0.0, 0.0, 1.0]), spin)
    return T


def _solve_five_axis(robot: Robot, backend, wk, rmoves, q, spins):
    """Joints per move with the spin about the tool axis left to the planner.

    With a collision backend: a ladder graph (studio.planner) over spins x IK branches, collision checks lazy on
    the cell meshes (robot links, spindle, part). Without one: damped least squares, trying the spins nearest the
    previous one first. Returns (joints, ok flags, chosen spin per move)."""
    from .planner import ladder
    if backend is not None and getattr(backend, "checks_collision", False):
        inv_tcp = np.linalg.inv(robot.tcp_offset)
        rungs = [[tool_pose(wk, m[1], m[2] if len(m) > 2 else None, s) @ inv_tcp for s in spins] for m in rmoves]
        res = ladder(rungs, backend, seed=q, max_step=math.radians(10))
        by = {r: (qq, c) for r, qq, c in zip(res.rungs, res.joints, res.choice)}
        qs, oks, chosen = [], [], []
        last = np.asarray(q, dtype=float)
        for k in range(len(rmoves)):
            if k in by:
                last, c = np.asarray(by[k][0]), by[k][1]
                qs.append(last)
                oks.append(True)
                chosen.append(spins[c])
            else:
                qs.append(last)
                oks.append(False)
                chosen.append(chosen[-1] if chosen else 0.0)
        return qs, oks, chosen
    qs, oks, chosen = [], [], []
    s_prev = 0.0
    m0 = rmoves[0]
    q, ok = solve_start(robot, tool_pose(wk, m0[1], m0[2] if len(m0) > 2 else None, 0.0), q)
    prev_kind = "rapid"
    for m in rmoves:
        order = sorted(spins, key=lambda s: abs(math.remainder(s - s_prev, 2 * math.pi)))
        free = m[0] != "feed" or prev_kind != "feed"       # moves into or out of a cut are joint moves
        best = None
        for s in order[:4]:
            T = tool_pose(wk, m[1], m[2] if len(m) > 2 else None, s)
            qq, ok = robot.solve_pose(T, q)
            if not ok and free:
                qq, ok = solve_start(robot, T, q, tries=8)
            if ok and (free or float(np.max(np.abs(qq - q))) < math.radians(20)):
                best = (qq, s)
                break
        prev_kind = m[0]
        if best is None:
            # no continuous solution: mark the point, but move on with the nearest solution so the next points
            # are judged from where the robot would be, not from a stale configuration
            cands = [robot.solve_pose(tool_pose(wk, m[1], m[2] if len(m) > 2 else None, s), q) for s in order[:4]]
            cands = [c for c, ok in cands if ok]
            if cands:
                q = min(cands, key=lambda c: float(np.max(np.abs(c - q))))
            qs.append(q)
            oks.append(False)
            chosen.append(s_prev)
        else:
            q, s_prev = best
            qs.append(q)
            oks.append(True)
            chosen.append(s_prev)
    return qs, oks, chosen


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
    yaw: float = 0.0                # turntable angle (radians) at the start of this segment
    e1: Optional[np.ndarray] = None   # (n,) turntable angle per sample (radians); it changes within a segment only
    #                                   when the table is driven as E1


@dataclass
class Expansion:
    segments: List[Segment]
    records: List[post.Rec]
    messages: List[str]
    errors: List[str] = field(default_factory=list)    # posting is refused while any error stands
    key: str = ""                                       # hash of the station and program it was made from

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


class ProcessState:
    """Spindle, wire motor, water and tool-change records of one expansion, mapped at post time through the
    station's cell I/O definition (lithocam.cell_io; the expansion carries it in an io_map record).

    The spindle (disc and milling steps) or the wire motor (wire steps) and the water go on before the first cut of
    a step; a changed spindle speed posts as a new spindle record; drives and water go off before a tool change,
    before an operator turns the table, and before the program end. A tool_change record (the cell's mode, manual
    without a cell I/O) is emitted whenever the mounted tool changes, the first tool included."""

    def __init__(self, station: "Station", recs: List[post.Rec], msgs: List[str], errors: List[str]):
        from .. import cell_io
        self.recs, self.msgs, self.robot = recs, msgs, station.robot
        self.io = None
        self.warned = False
        self.tool: Optional[str] = None
        self.on = {"spindle": False, "wire": False, "water": False}
        self.rpm = 0.0
        src = (station.cell_io or "").strip()
        if not src:
            return
        path = cell_io.default_path(station.robot, CELLS) if src == "default" else Path(src)
        try:
            self.io = cell_io.load(path)
        except (OSError, ValueError) as e:
            errors.append("cell I/O {}: {}".format(path, e))
            return
        native = ROBOTS.get(station.robot, ("", "", ""))[2]
        if self.io.controller != native:
            msgs.append("WARNING: cell I/O {} is for {} programs; the {} posts {}: its process records post as "
                        "comments".format(path, self.io.controller, station.robot, native))
        recs.append(post.Rec("io_map", {"path": str(path), "io": self.io.data}))
        msgs.extend("WARNING: " + w for w in self.io.warnings)

    def _warn(self):
        if self.io is None and not self.warned:
            self.warned = True
            self.msgs.append("WARNING: no cell I/O definition loaded (Station.cell_io): spindle, water and tool "
                             "change post as comments and operator pauses. Select cells/{}_io.json (placeholder "
                             "values) and set it for your cell.".format(self.robot))

    def tool_change(self, tid: str):
        if not tid or tid == self.tool:
            return
        self.all_off()
        self._warn()
        mode = self.io.tool_mode() if self.io is not None else "manual"
        self.recs.append(post.Rec("tool_change", {"id": tid, "mode": mode}))
        self.tool = tid

    def drive_on(self, drive: str, rpm: float = 0.0, step: int = -1):
        """`drive`: "spindle" or "wire"."""
        self._warn()
        other = "wire" if drive == "spindle" else "spindle"
        if self.on[other]:
            self.recs.append(post.Rec(other, {"rpm": 0.0, "on": False}))
            self.on[other] = False
        rpm = float(rpm or 0.0)
        if not self.on[drive] or (drive == "spindle" and rpm and abs(rpm - self.rpm) > 1e-9):
            self.recs.append(post.Rec(drive, {"rpm": rpm, "on": True}))
            if drive == "spindle" and not rpm:
                self.msgs.append("step {}: spindle on without a speed: set the rpm on the step or the tool".format(
                    step + 1))
            self.on[drive] = True
            self.rpm = rpm if drive == "spindle" else self.rpm
        if not self.on["water"]:
            self.recs.append(post.Rec("water", {"on": True}))
            self.on["water"] = True

    def all_off(self):
        for k in ("spindle", "wire"):
            if self.on[k]:
                self.recs.append(post.Rec(k, {"rpm": 0.0, "on": False}))
                self.on[k] = False
        if self.on["water"]:
            self.recs.append(post.Rec("water", {"on": False}))
            self.on["water"] = False
        self.rpm = 0.0


WIRE_BOW = {"ur10e": 0.25, "kr120": 0.6, "kr480_r3330_mt": 0.6, "kr500_r2830_mt": 0.6}   # wire depth along tool0 z of each cell's bow saw (cells/make_cells.py)


def wire_rungs(face, work: np.ndarray, yaw: float, wire_radius: float, bow: float, rolls_deg: Sequence[float],
               lead: bool = True):
    """Candidate tool0 poses per ruling (one per roll) for a ruled face given in mm in the part frame, on a
    turntable at `yaw`; rulings ordered along the sweep, with one lead-in and one lead-out ruling outside the face."""
    from ..wirepath import order_rulings, turned_face, wire_target
    f = turned_face(face, yaw)
    rs = list(f.rulings) if getattr(f, "ordered", False) else order_rulings(f.rulings)   # profiles keep their order
    lead = lead and not getattr(f, "ordered", False)                                      # profiles carry their leads
    R, o = work[:3, :3], work[:3, 3]
    lines = [(R @ (np.asarray(r.p0) / 1000) + o, R @ (np.asarray(r.p1) / 1000) + o, R @ np.asarray(r.normal))
             for r in rs]
    if lead and len(lines) > 1:
        a, b = lines[0], lines[1]
        lines = [(2 * a[0] - b[0], 2 * a[1] - b[1], a[2])] + lines
        a, b = lines[-1], lines[-2]
        lines.append((2 * a[0] - b[0], 2 * a[1] - b[1], a[2]))
    return [[wire_target(p0, p1, n, wire_radius, math.radians(rd), bow) for rd in rolls_deg] for p0, p1, n in lines]


def wire_rungs_table(face, work: np.ndarray, yaw_order: float, yaws: Sequence[float], wire_radius: float, bow: float,
                     rolls_deg: Sequence[float], lead: bool = True):
    """Candidates over turntable angles as well as rolls: rungs[k] holds one tool0 pose per (yaw, roll), ext[k] the
    yaw (radians) of each. The rulings are ordered once, at `yaw_order`, so every angle sweeps them in one order."""
    from ..wirepath import order_rulings, turned_face, wire_target
    tf = turned_face(face, yaw_order)
    if getattr(tf, "ordered", False):
        order = list(range(len(tf.rulings)))
    else:
        pos = {id(r): k for k, r in enumerate(tf.rulings)}
        order = [pos[id(r)] for r in order_rulings(tf.rulings)]
    lead = lead and not getattr(tf, "ordered", False)
    R, o = work[:3, :3], work[:3, 3]
    per_yaw = []
    for y in yaws:
        fy = turned_face(face, y)
        rs = [fy.rulings[k] for k in order]
        lines = [(R @ (np.asarray(r.p0) / 1000) + o, R @ (np.asarray(r.p1) / 1000) + o, R @ np.asarray(r.normal))
                 for r in rs]
        if lead and len(lines) > 1:
            a, b = lines[0], lines[1]
            lines = [(2 * a[0] - b[0], 2 * a[1] - b[1], a[2])] + lines
            a, b = lines[-1], lines[-2]
            lines.append((2 * a[0] - b[0], 2 * a[1] - b[1], a[2]))
        per_yaw.append([[wire_target(p0, p1, n, wire_radius, math.radians(rd), bow) for rd in rolls_deg]
                        for p0, p1, n in lines])
    rungs = [[T for py in per_yaw for T in py[k]] for k in range(len(per_yaw[0]))]
    ext = [[float(y) for y in yaws for _ in rolls_deg] for _ in range(len(rungs))]
    return rungs, ext


def table_window(centre: float, current: float, window_deg: float, step_deg: float, limits_deg) -> List[float]:
    """Table angles (radians) around `centre` within +-window_deg every step_deg, the centre moved by whole turns
    to the one nearest the current angle that lies inside the limits; angles outside the limits are dropped."""
    lo, hi = (math.radians(v) for v in limits_deg)
    c = centre + 2 * math.pi * round((current - centre) / (2 * math.pi))
    cands = [c + 2 * math.pi * k for k in range(-3, 4)]
    inside = [v for v in cands if lo - 1e-9 <= v <= hi + 1e-9]
    if inside:
        c = min(inside, key=lambda v: abs(v - current))
    n = int(math.floor(window_deg / step_deg + 1e-9)) if step_deg > 0 else 0
    out = [c + math.radians(step_deg * k) for k in range(-n, n + 1)]
    return [v for v in out if lo - 1e-9 <= v <= hi + 1e-9]


def _wire_step(i, st, p, station, robot, tools, backend, q, add, recs, msgs, yaw, errors, proc=None, mode=""):
    """Diamond-wire sweep over the ruled faces labelled 'wire': per face the turntable turns the face towards the
    robot, then the ladder planner picks roll and IK branch per ruling.

    With the table driven as E1 (mode "e1") the table angle is a free choice per ruling as well: candidates every
    table_step_deg (default 10) within +-table_window_deg (default 30) of the facing angle, with the table step per
    ruling at most the turntable's e1_max_step_deg; a table step costs table_weight (default 0.1) times an arm joint
    step of the same angle, so the planner turns the table rather than the arm. The wire moves between rulings in the
    part frame while the table turns, and every motion record carries the table angle."""
    from ..wirepath import facing_yaw, load_ruled_json
    from .planner import NumpyBackend, TableChain, collides, ladder
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
        if proc is not None:
            proc.tool_change(tid)
        recs.append(post.Rec("tool", {"id": tid, "tcp": np.eye(4).tolist()}))
    be = backend or NumpyBackend(robot, [HOME.get(station.robot, [0.0] * robot.dof)])
    if hasattr(be, "set_tool_mode"):
        be.set_tool_mode("wire")
    rolls = [float(v) for v in str(p.get("rolls", ",".join(str(a) for a in range(-90, 91, 10)))).split(",")]
    max_step = math.radians(float(p.get("max_step_deg", 30.0)))
    lead = bool(p.get("lead", True))
    work = station.work_frame()
    set_feed = float(p.get("feed") or getattr(tool, "feed", 0) or 0)      # mm/min of the wire across the stone
    from .. import cutting_data
    rate = cutting_data.lookup(station.material, "diamond_wire", "cut_rate")
    e1 = mode == "e1" and station.turntable is not None
    ts = table_settings(station.turntable) if e1 else None
    chain = TableChain(robot, work)
    for f in faces:
        nz = float(np.mean([np.asarray(r.normal) for r in f.rulings], axis=0)[2])
        if station.turntable is not None and nz < -0.7:
            msgs.append("step {}: wire face {} faces down onto the table; cut it before the part is set down".format(
                i + 1, f.index))
            continue
        turns = station.turntable is not None and (
            getattr(f, "yaw_deg", None) is not None or (p.get("auto_table", True) and nz < 0.7))
        if e1:
            # the table is a coordinated axis: no stop, drives stay on; the angle is chosen per ruling below
            if getattr(f, "yaw_deg", None) is not None:
                centre = math.radians(float(f.yaw_deg))
            elif p.get("auto_table", True) and nz < 0.7:
                centre = facing_yaw(f, work[:3, 3])
            else:
                centre = yaw[0]                               # top faces: around the present angle
        else:
            if proc is not None and turns:
                proc.all_off()                                # the operator turns the table: drives off first
            if station.turntable is not None and getattr(f, "yaw_deg", None) is not None:   # the face names it
                yaw[0] = math.radians(float(f.yaw_deg))
                recs.append(table_record(station, mode, float(f.yaw_deg)))
                if hasattr(be, "place_part"):
                    be.place_part(work, yaw[0])
            elif station.turntable is not None and p.get("auto_table", True) and nz < 0.7:   # top: keep the angle
                yaw[0] = facing_yaw(f, work[:3, 3])
                recs.append(table_record(station, mode, math.degrees(yaw[0])))
                if hasattr(be, "place_part"):
                    be.place_part(work, yaw[0])
        wire_feed = set_feed
        if wire_feed <= 0 and rate:
            # sawing rates are per area: feed = rate / cut height; the cut height is the median ruling length
            L = float(np.median([np.linalg.norm(np.asarray(r.p1) - np.asarray(r.p0)) for r in f.rulings])) / 1000
            e = min(rate, key=lambda x: x.lo)
            wire_feed = e.lo / max(L, 1e-3) * 1000 / 60          # m2/h / m -> m/h -> mm/min
            recs.append(post.Rec("comment", {"text": "wire feed {:.0f} mm/min = {:g} m2/h ({}, {} cutting rate) over "
                                             "a {:.0f} mm cut; source {}".format(wire_feed, e.lo, e.material, e.note
                                                                                or "lower", L * 1000, e.source)}))
            msgs.append("step {}: wire face {}: feed {:.0f} mm/min from the {} sawing rate {:g} m2/h ({})".format(
                i + 1, f.index, wire_feed, e.material, e.lo, e.source_type))
        if wire_feed <= 0:
            errors.append("step {}: wire face {} has no feed: set the step's feed (mm/min) or the wire tool's feed; "
                          "there is no sawing rate for '{}' in the cutting data".format(i + 1, f.index,
                                                                                      station.material))
        if e1:
            yaws = table_window(centre, yaw[0], float(p.get("table_window_deg", 30.0)),
                                float(p.get("table_step_deg", 10.0)), ts["e1_limits_deg"])
            if not yaws:
                errors.append("step {}: wire face {}: no table angle within the E1 limits {} deg".format(
                    i + 1, f.index, ts["e1_limits_deg"]))
                continue
            rungs, ext = wire_rungs_table(f, work, yaws[len(yaws) // 2], yaws, wire_r, bow, rolls, lead)
            lkw = dict(ext_seed=yaw[0], ext_max_step=math.radians(ts["e1_max_step_deg"]),
                       ext_weight=float(p.get("table_weight", 0.1)))
        else:
            rungs, ext = wire_rungs(f, work, yaw[0], wire_r, bow, rolls, lead), None
            lkw = {}
        res = ladder(rungs, be, seed=q, max_step=max_step, ext=ext, **lkw)
        if lead and len(rungs) > 3:   # lead rulings are optional: drop one that only connects through a jump
            drop_in = 1 in res.breaks or 0 in res.missing
            drop_out = (len(rungs) - 1) in res.breaks or (len(rungs) - 1) in res.missing
            if drop_in or drop_out:
                a_, b_ = 1 if drop_in else 0, len(rungs) - 1 if drop_out else len(rungs)
                rungs = rungs[a_:b_]
                ext = ext[a_:b_] if ext is not None else None
                res = ladder(rungs, be, seed=q, max_step=max_step, ext=ext, **lkw)
                msgs.append("step {}: wire face {}: lead-{} dropped (it only connects through a jump)".format(
                    i + 1, f.index, "in and lead-out" if drop_in and drop_out else ("in" if drop_in else "out")))
        if not res.joints:
            msgs.append("step {}: wire face {}: no reachable ruling".format(i + 1, f.index))
            continue
        qa = np.asarray(res.joints)
        # the wire cuts between rulings too: fill each gap with straight-line poses (tool0 interpolated between the
        # chosen ruling poses) so the joint change per sample stays <= 2 deg, and post them as linear cuts
        poses = [rungs[r][c] for r, c in zip(res.rungs, res.choice)]
        E = np.asarray(res.ext) if e1 else np.full(len(qa), yaw[0])
        sq, sp_, sok, scut, se = [qa[0]], [poses[0]], [True], [False], [E[0]]
        fast = 0
        for k in range(1, len(qa)):
            joined = res.rungs[k] == res.rungs[k - 1] + 1 and res.rungs[k] not in res.breaks
            n = max(1, int(math.ceil(max(np.max(np.abs(qa[k] - qa[k - 1])), abs(E[k] - E[k - 1]))
                                     / math.radians(2))))
            if e1 and joined:                                 # interpolate in the part frame: the table turns
                Pa, Pb = chain.to_part(poses[k - 1], E[k - 1]), chain.to_part(poses[k], E[k])
                if wire_feed > 0 and abs(E[k] - E[k - 1]) > 1e-12:
                    wa = Pa[:3, 3] + Pa[:3, 2] * bow
                    wb = Pb[:3, 3] + Pb[:3, 2] * bow
                    dt = max(float(np.linalg.norm(wb - wa)) * 1000 / (wire_feed / 60.0), 1e-9)   # s
                    if math.degrees(abs(E[k] - E[k - 1])) / dt > ts["e1_speed_deg_s"] + 1e-9:
                        fast += 1
            qq = sq[-1]
            for t in np.linspace(0, 1, n + 1)[1:]:
                e = float(E[k - 1] + (E[k] - E[k - 1]) * t)
                if t < 1 and joined:
                    T = (chain.to_base(interp_pose(Pa, Pb, float(t)), e) if e1
                         else interp_pose(poses[k - 1], poses[k], float(t)))
                    qq, ok = robot.solve_pose(T, qq)
                    ok = ok and float(np.max(np.abs(qq - sq[-1]))) <= math.radians(4)
                elif t < 1:                                   # a break: joint move, the wire leaves the cut
                    qq, ok, T = qa[k - 1] + (qa[k] - qa[k - 1]) * t, True, None
                else:
                    qq, ok, T, e = qa[k], True, poses[k], float(E[k])
                if ok and backend is not None and getattr(backend, "checks_collision", False) and 0 < t < 1:
                    ok = not collides(backend, qq, e if e1 else None)
                sq.append(qq)
                sp_.append(T)
                sok.append(ok)
                scut.append(joined)
                se.append(e)
        sq = np.asarray(sq)
        se = np.asarray(se, dtype=float)
        bad = len(sok) - int(sum(sok))
        if e1:
            recs.append(table_record(station, mode, math.degrees(se[0])))
        add(i, "{} face {}".format(st.title(), f.index), sq, scut, sok, se if e1 else None)
        recs.append(post.Rec("comment", {"text": "wire face {}: {} rulings, {} samples, table {}".format(
            f.index, len(qa), len(sq), "E1 {:.1f} to {:.1f} deg (coordinated)".format(
                math.degrees(se.min()), math.degrees(se.max())) if e1 else "{:.1f} deg".format(math.degrees(yaw[0])))}))
        if proc is not None:
            proc.drive_on("wire", 0.0, i)                     # wire motor and water before the first cut
        x1 = (lambda k: {"e1": math.degrees(se[k])}) if e1 else (lambda k: {})
        recs.append(post.Rec("movej", dict({"q": sq[0].tolist()}, **x1(0))))
        if wire_feed > 0:
            recs.append(post.Rec("speed", {"linear": wire_feed / 60.0}))
        for k, (qq, T, c) in enumerate(zip(sq[1:], sp_[1:], scut[1:]), start=1):
            if c and T is not None:
                recs.append(post.Rec("movel", dict({"q": qq.tolist(), "pose": np.asarray(T).tolist(), "cut": True,
                                                    "v": wire_feed / 60.0 if wire_feed > 0 else None}, **x1(k))))
            else:
                recs.append(post.Rec("movej", dict({"q": qq.tolist()}, **x1(k))))
        if bad:
            msgs.append("step {}: wire face {}: {} of {} samples between rulings did not solve or collide".format(
                i + 1, f.index, bad, len(sq)))
        msgs.append("step {}: wire face {}: {} of {} rulings, {} missing, {} breaks, largest ruling step {:.1f} "
                    "deg, {} nodes, {} colliding nodes rejected ({})".format(
                        i + 1, f.index, len(qa), len(rungs), len(res.missing), len(res.breaks),
                        math.degrees(res.max_step), res.nodes, res.collisions_rejected, be.name))
        if e1:
            msgs.append("step {}: wire face {}: table E1 chosen per ruling from {} angles ({:.1f} to {:.1f} deg); "
                        "used {:.1f} to {:.1f} deg, largest table step {:.1f} deg per ruling (limit {:g})".format(
                            i + 1, f.index, len(yaws), math.degrees(min(yaws)), math.degrees(max(yaws)),
                            math.degrees(se.min()), math.degrees(se.max()), math.degrees(res.max_ext_step),
                            ts["e1_max_step_deg"]))
            if fast:
                msgs.append("WARNING: step {}: wire face {}: at {} ruling gaps the table turns faster than "
                            "e1_speed_deg_s {:g} at the wire feed; lower the feed or the table step".format(
                                i + 1, f.index, fast, ts["e1_speed_deg_s"]))
            yaw[0] = float(se[-1])
            if hasattr(be, "place_part"):
                be.place_part(work, yaw[0])
        q = sq[-1]
    return q


def _sample_links(moves, step_mm: float):
    """Insert points every `step_mm` along rapid and plunge links (not before the first move), so a link is a
    straight line in the simulation as it is in the posted program."""
    out = list(moves[:1])
    for a, b in zip(moves, moves[1:]):
        if b[0] in ("rapid", "plunge"):
            pa, pb = np.asarray(a[1], dtype=float), np.asarray(b[1], dtype=float)
            n = int(math.ceil(np.linalg.norm(pb - pa) / step_mm))
            for t in np.linspace(0, 1, n + 1)[1:-1]:
                if len(b) > 2:
                    aa = np.asarray(a[2] if len(a) > 2 else b[2], dtype=float)
                    ax = aa + (np.asarray(b[2], dtype=float) - aa) * t
                    out.append((b[0], pa + (pb - pa) * t, ax / np.linalg.norm(ax)))
                else:
                    out.append((b[0], pa + (pb - pa) * t))
        out.append(b)
    return out


def _polygon(text: str):
    pts = [[float(v) for v in s.split(",")] for s in str(text).replace("\n", ";").split(";") if s.strip()]
    if len(pts) < 3 or any(len(p_) != 2 for p_ in pts):
        raise ValueError("a polygon needs at least 3 points written as 'x,y; x,y; x,y' (mm)")
    P = np.asarray(pts, dtype=float)
    a = 0.5 * float(np.sum(P[:, 0] * np.roll(P[:, 1], -1) - np.roll(P[:, 0], -1) * P[:, 1]))
    return P if a > 0 else P[::-1]                         # counter-clockwise


def _disc_step(i, st, p, station, robot, tool, backend, q, add, recs, msgs, errors, yaw, sp, proc=None):
    """Saw cuts from lithocam.discsaw, run by the robot: the blade centre is the TCP, the spindle axis is the
    blade normal; the blade is round, so the spin about its axis is left to the planner like a 5-axis spindle."""
    from .. import discsaw
    s = station.stock
    if s is None:
        msgs.append("step {}: the disc saw needs a stock (slab top and thickness)".format(i + 1))
        return q
    top, thick = float(s["zmax"]), float(s["zmax"] - s["zmin"])
    D = float(tool.diameter)
    kerf = float(getattr(tool, "kerf", 0.0) or 0.0) or float(getattr(tool, "segment_height", 0.0) or 0.0)
    if D <= 0 or kerf <= 0:
        errors.append("step {}: tool {} needs a diameter and a kerf".format(i + 1, tool.id))
        return q
    try:
        if st.kind == "disc_cut":
            text = str(p.get("outline") or "")
            outline = _polygon(text) if text.strip() else np.array(
                [[s["xmin"], s["ymin"]], [s["xmax"], s["ymin"]], [s["xmax"], s["ymax"]], [s["xmin"], s["ymax"]]])
            bev = [float(v) for v in str(p.get("bevel", "0")).replace(";", ",").split(",") if v.strip()] or [0.0]
            rep = {}
            cuts = discsaw.profile_cuts(outline, top=top, thickness=thick, blade_diameter=D, kerf=kerf,
                                        flange_diameter=float(p.get("flange_diameter", 100.0)),
                                        stepdown=float(p.get("stepdown", 10.0)),
                                        depth=float(p["depth"]) if float(p.get("depth") or 0) > 0 else None,
                                        bevel_deg=bev[0] if len(bev) == 1 else bev, report=rep)
            for e, k, why in rep.get("skipped", []):
                msgs.append("step {}: edge {} pass {} left uncut: {}".format(i + 1, e, k, why))
            for w in discsaw.uncut_wedges(cuts, outline):
                msgs.append("step {}: uncut wedge for a follow-up tool: {}".format(i + 1, w))
        else:
            region = _polygon(p.get("region") or "")
            ca = str(p.get("cross_angle") or "").strip()
            cuts = discsaw.comb_cuts(region, top=top, thickness=thick, blade_diameter=D, kerf=kerf,
                                     flange_diameter=float(p.get("flange_diameter", 40.0)),
                                     pitch=float(p.get("pitch", 15.0)), stepdown=float(p.get("stepdown", 5.0)),
                                     depth=float(p.get("depth", 10.0)), angle_deg=float(p.get("angle", 0.0)),
                                     cross_angle_deg=float(ca) if ca else None)
    except ValueError as e:
        errors.append("step {}: {}".format(i + 1, e))
        return q
    feed = float(p.get("feed") or getattr(tool, "feed", 0) or 0)
    rpm = float(p.get("rpm") or getattr(tool, "spindle_rpm", 0) or 0)
    if not feed or not rpm:
        from .. import cutting_data
        sg = cutting_data.suggest(tool, station.material, float(p.get("stepdown", 0) or 0), sp.max_rpm,
                                  station.robot_factor)
        feed, rpm = feed or sg.feed, rpm or sg.rpm
        msgs.append("step {}: cutting data for {} in {}: {}".format(i + 1, tool.id, station.material, sg.text()))
    if feed <= 0:
        errors.append("step {}: the saw cut has no feed; set it on the step or the tool".format(i + 1))
    if proc is not None:
        proc.drive_on("spindle", rpm, i)                      # spindle and water before the first cut
    elif rpm:
        recs.append(post.Rec("spindle", {"rpm": rpm}))
    if feed > 0:
        recs.append(post.Rec("speed", {"linear": feed / 60.0}))
    work = station.work_frame()
    Rz = np.eye(4)
    Rz[:3, :3] = axis_angle(np.array([0.0, 0.0, 1.0]), yaw[0])
    wk = work @ Rz
    if backend is not None and getattr(backend, "checks_collision", False):
        if hasattr(backend, "set_tool_mode"):
            backend.set_tool_mode("spindle")
    ns = max(1, int(p.get("spins", 8)))
    spins = [2 * math.pi * k / ns for k in range(ns)]
    step_mm = 5.0
    moves = []
    for c in cuts:
        n = -np.asarray(c.normal, dtype=float)                 # tool axis (tip to holder) = minus the blade normal
        a, b = np.asarray(c.start, dtype=float), np.asarray(c.end, dtype=float)
        if len(c.lead_in):
            moves.append(("rapid", np.asarray(c.lead_in, dtype=float), n))
        m = max(1, int(math.ceil(np.linalg.norm(b - a) / step_mm)))
        moves += [("feed", a + (b - a) * t, n) for t in np.linspace(0, 1, m + 1)]
        if len(c.lead_out):
            moves.append(("rapid", np.asarray(c.lead_out, dtype=float), n))
    qs, oks, chosen = _solve_five_axis(robot, backend, wk, moves, q, spins)
    add(i, st.title(), qs, [m[0] == "feed" for m in moves], oks)
    recs.append(post.Rec("comment", {"text": "{}: {} cuts with {} (D {:g} mm, kerf {:g} mm)".format(
        st.title(), len(cuts), tool.id, D, kerf)}))
    for m, qq, sn in zip(moves, qs, chosen):
        T = tool_pose(wk, m[1], m[2], sn)
        if m[0] == "feed":
            recs.append(post.Rec("movel", {"q": np.asarray(qq).tolist(), "pose": T.tolist(), "cut": True,
                                           "v": feed / 60.0 if feed > 0 else None}))
        else:
            recs.append(post.Rec("movej", {"q": np.asarray(qq).tolist()}))
    bad = len(oks) - int(sum(oks))
    msgs.append("step {}: {}: {} cuts, {} robot samples, {} unreachable or colliding".format(
        i + 1, st.spec.label, len(cuts), len(qs), bad))
    return qs[-1] if qs else q


def expand(station: Station, robot: Robot, program: Program, tools: Dict[str, object], part=None,
           q0: Optional[Sequence[float]] = None, max_feed_points: int = 4000, progress=None,
           backend=None) -> Expansion:
    """Expand a program. `part` = (tris, trimesh) in mm, needed by machining steps. `tools` maps id -> Tool.
    `backend`: a planner backend (studio.planner) for IK branches and collision checks; None = numpy IK only."""
    from . import cam
    from .planner import collides

    work = station.work_frame()
    q = np.asarray(q0 if q0 is not None else HOME.get(station.robot, [0.0] * robot.dof), dtype=float)
    segs: List[Segment] = []
    recs: List[post.Rec] = [post.Rec("start", {"name": "".join(c if c.isalnum() else "_" for c in program.name)})]
    msgs: List[str] = []
    errors: List[str] = []
    tool = None
    # who turns the table: "" (no table), "operator", "e1" (coordinated external axis) or "io" (PLC through I/O)
    table_mode, terr, twarn = check_table(station)
    errors.extend(terr)
    msgs.extend(twarn)
    if table_mode == "e1" and backend is not None and getattr(backend, "checks_collision", False) and \
            not getattr(backend, "table_axis", False):
        msgs.append("WARNING: the collision backend has no table joint; the part is checked at each setup's angle "
                    "only (build the backend from the station with the table driven 'e1')")
    proc = ProcessState(station, recs, msgs, errors)   # spindle, water and tool-change records via the cell I/O

    sp = station.spindle_model()
    yaw = [0.0]                     # turntable angle, radians (a list so the helpers can update it)
    tilt = [0.0]                    # 3+2 tool tilt about the table x axis, radians
    framed = {}                     # tilt -> (tris, trimesh, stock) of the part seen from that setup

    def in_setup_frame(F):
        """The part and its stock box expressed in the setup frame F (rotation only)."""
        key = round(float(tilt[0]), 9)
        if key not in framed:
            tris, _ = part
            tf = (tris.reshape(-1, 3) @ F[:3, :3]).reshape(-1, 3, 3)
            s = station.stock
            c = np.array([[x, y, z] for x in (s["xmin"], s["xmax"]) for y in (s["ymin"], s["ymax"])
                          for z in (s["zmin"], s["zmax"])]) @ F[:3, :3]
            box = {"xmin": c[:, 0].min(), "xmax": c[:, 0].max(), "ymin": c[:, 1].min(), "ymax": c[:, 1].max(),
                   "zmin": c[:, 2].min(), "zmax": c[:, 2].max()}
            framed[key] = (tf, cam.trimesh(tf), box)
        return framed[key]

    def mount(i, tid):
        """Put the cutter in the spindle: TCP, tool record (with the spindle mass), collet check."""
        t = tools[tid]
        robot.tcp_offset = station.tool_tcp(t)
        proc.tool_change(tid)                          # drives off, then the change (only when the tool changes)
        recs.append(post.Rec("tool", {"id": tid, "tcp": robot.tcp_offset.tolist(), "mass": sp.mass_kg}))
        shank = float(getattr(t, "shank_diameter", 0.0) or 0.0) or float(getattr(t, "diameter", 0.0) or 0.0)
        if sp.max_collet_mm and shank > sp.max_collet_mm + 1e-9 and t.kind not in (
                lc_kinds_without_shank()):
            msgs.append("step {}: {} shank {:g} mm exceeds the {} collet ({:g} mm max)".format(
                i + 1, tid, shank, sp.collet, sp.max_collet_mm))
        return t

    def table_now() -> float:
        """Table angle at the end of the last segment (the start angle before any)."""
        last = next((s for s in reversed(segs) if len(s.q)), None)
        if last is None:
            return 0.0
        return float(last.e1[-1]) if last.e1 is not None and len(last.e1) else float(last.yaw)

    def hit(x, e):
        """Collision at joints x and table angle e (the angle counts only with the table driven as E1)."""
        return collides(backend, x, e if table_mode == "e1" else None)

    def add(i, label, qs, feed, ok, e1=None):
        """A segment; `e1`: table angle per sample (radians), None = the present table angle throughout."""
        qs = np.asarray(qs, dtype=float).reshape(-1, robot.dof)
        feed, ok = list(np.asarray(feed, bool)), list(np.asarray(ok, bool))
        es = np.full(len(qs), yaw[0]) if e1 is None else np.asarray(e1, dtype=float).reshape(len(qs))
        if len(qs) > 1:
            # joint moves inside a segment (rapids, moves into and out of a cut) are sampled at <= 2 deg like the
            # transitions between segments, so the simulation shows and checks the motion the controller makes
            Q, Fd, Ok, Es = [qs[0]], [feed[0]], [ok[0]], [es[0]]
            for k in range(1, len(qs)):
                jump = float(np.max(np.abs(qs[k] - qs[k - 1])))
                if not feed[k] and max(jump, abs(es[k] - es[k - 1])) > math.radians(2) + 1e-9:
                    n = int(math.ceil(max(jump, abs(es[k] - es[k - 1])) / math.radians(2)))
                    for t in np.linspace(0, 1, n + 1)[1:-1]:
                        x = qs[k - 1] + (qs[k] - qs[k - 1]) * t
                        e = es[k - 1] + (es[k] - es[k - 1]) * t
                        good = True
                        if backend is not None and getattr(backend, "checks_collision", False):
                            good = not hit(x, e)
                        Q.append(x)
                        Fd.append(False)
                        Ok.append(good)
                        Es.append(e)
                elif feed[k] and jump > math.radians(10):
                    ok[k] = False                         # a jump inside a cut: the tool would leave the path
                Q.append(qs[k])
                Fd.append(feed[k])
                Ok.append(ok[k])
                Es.append(es[k])
            qs, feed, ok, es = np.asarray(Q), Fd, Ok, np.asarray(Es, dtype=float)
        last = next((s for s in reversed(segs) if len(s.q)), None)
        prev = last.q[-1] if last is not None else None
        if prev is not None and len(qs):
            # the controller moves in joint space between the end of one segment and the start of the next:
            # simulate that motion at <= 2 deg per sample and check it like any other motion. With the table as
            # E1 the table turns in the same move (coordinated), so its angle is interpolated too.
            pe = float(last.e1[-1]) if last.e1 is not None and len(last.e1) else last.yaw
            if table_mode != "e1":
                pe = float(es[0])
            jump = max(float(np.max(np.abs(qs[0] - prev))), abs(float(es[0]) - pe))
            if jump > math.radians(2) + 1e-9:
                n = int(math.ceil(jump / math.radians(2))) + 1
                ts_ = np.linspace(0, 1, n)[1:-1]
                tq = [prev + (qs[0] - prev) * t for t in ts_]
                te = [pe + (float(es[0]) - pe) * t for t in ts_]
                tok = [True] * len(tq)
                if tq and backend is not None and getattr(backend, "checks_collision", False):
                    hits = [hit(x, e) for x, e in zip(tq, te)]
                    tok = [not h for h in hits]
                    if any(hits):
                        msgs.append("step {}: the joint move into '{}' collides at {} of {} samples".format(
                            i + 1, label, sum(hits), len(tq)))
                if tq:
                    ttcp = np.array([robot.tcp(x)[:3, 3] for x in tq])
                    segs.append(Segment(i, "transition to " + label, np.asarray(tq), np.zeros(len(tq), bool),
                                        np.asarray(tok, bool), ttcp, robot.tcp_offset.copy(),
                                        getattr(tool, "id", "") if tool is not None else "",
                                        te[0] if table_mode == "e1" else yaw[0], np.asarray(te, dtype=float)))
        tcp = np.array([robot.tcp(x)[:3, 3] for x in qs]) if len(qs) else np.zeros((0, 3))
        segs.append(Segment(i, label, qs, np.asarray(feed, bool), np.asarray(ok, bool), tcp,
                            robot.tcp_offset.copy(), getattr(tool, "id", "") if tool is not None else "",
                            float(es[0]) if len(es) and table_mode == "e1" else yaw[0], es))

    for i, st in enumerate(program.steps):
        spec = catalog.BY_ID.get(st.kind)
        if spec is None:
            msgs.append("step {}: unknown kind {}".format(i + 1, st.kind))
            continue
        p = dict(catalog.default_params(spec), **st.params)
        if progress:
            progress(i, len(program.steps), st.title())
        if st.kind in ("movej", "movel") and p.get("target"):
            tg = station.target(str(p["target"]))
            if tg is None:
                msgs.append("step {}: target '{}' does not exist".format(i + 1, p["target"]))
                continue
            p = dict(p, joints=tg["joints"], pose=tg["pose"])
        if st.kind in ("set_tool", "tool_change"):
            tid = str(p.get("tool", ""))
            if tid in tools:
                tool = mount(i, tid)
            else:
                msgs.append("step {}: tool '{}' is not in the library".format(i + 1, tid))
        elif st.kind == "movej":
            target = np.radians(np.asarray(p["joints"], dtype=float)) if p.get("joints") is not None else q
            pe = table_now() if table_mode == "e1" else yaw[0]
            n = max(2, int(max(np.max(np.abs(target - q)), abs(yaw[0] - pe)) / math.radians(2)) + 1)
            qs = [q + (target - q) * t for t in np.linspace(0, 1, n)]
            add(i, st.title(), qs, [False] * n, [True] * n,
                np.linspace(pe, yaw[0], n) if table_mode == "e1" else None)
            recs.append(post.Rec("movej", {"q": target.tolist(), "vj": float(p.get("speed") or 0) / 100 or None}))
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
            add(i, st.title(), qs, [False] * n, oks,
                np.linspace(table_now(), yaw[0], n) if table_mode == "e1" else None)
            recs.append(post.Rec("movel", {"q": q.tolist(), "pose": B.tolist(), "v": float(p.get("speed") or 0) or None,
                                           "r": float(p.get("blend") or 0) / 1000}))
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
                if table_mode != "e1":
                    proc.all_off()                     # the operator or the PLC turns the table: drives off first
                yaw[0] = math.radians(float(p.get("yaw", 0.0)))
                recs.append(table_record(station, table_mode, math.degrees(yaw[0])))
                if table_mode == "e1" and backend is not None and hasattr(backend, "place_part"):
                    backend.place_part(work, yaw[0])   # the next moves turn the table with the arm (coordinated)
            tilt[0] = math.radians(float(p.get("tilt", 0.0)))
            if tilt[0]:
                recs.append(post.Rec("comment", {"text": "3+2 setup: tool tilted {:.1f} deg about the table x axis"
                                                 .format(math.degrees(tilt[0]))}))
        elif st.kind == "wire_ruled":
            q = _wire_step(i, st, p, station, robot, tools, backend, q, add, recs, msgs, yaw, errors, proc,
                           table_mode)
        elif st.kind in ("disc_cut", "disc_comb"):
            tid = str(p.get("tool") or "")
            if tid not in tools:
                msgs.append("step {}: disc saw needs a disc tool from the library".format(i + 1))
                continue
            tool = mount(i, tid)
            q = _disc_step(i, st, p, station, robot, tool, backend, q, add, recs, msgs, errors, yaw, sp, proc)
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
            F = np.eye(4)
            ct, sn = math.cos(tilt[0]), math.sin(tilt[0])
            F[:3, :3] = np.array([[1, 0, 0], [0, ct, -sn], [0, sn, ct]])   # setup frame: tilted about table x
            if tilt[0]:
                tris, mesh, stock_f = in_setup_frame(F)
            else:
                (tris, mesh), stock_f = part, station.stock
            try:
                moves = cam.GENERATORS[st.kind](tris, mesh, tool, stock_f,
                                                **{k: v for k, v in p.items() if k != "tool"})
            except Exception as e:   # noqa: BLE001 - report generator errors in the program log
                msgs.append("step {}: {} failed: {}".format(i + 1, spec.label, e))
                continue
            # every posted point: the CL path simplified only within the step tolerance (never thinned by count),
            # and the links between passes sampled as straight lines so the simulation shows what is posted
            tol = max(float(p.get("tolerance", 0.01) or 0.01), 1e-4)
            rmoves = _sample_links(cam.simplify(moves, tol), 5.0)
            if len(rmoves) > max_feed_points:
                msgs.append("step {}: {} robot points after simplification (tolerance {:g} mm); planning takes "
                            "longer".format(i + 1, len(rmoves), tol))
            Rz = np.eye(4)
            Rz[:3, :3] = np.array([[math.cos(yaw[0]), -math.sin(yaw[0]), 0], [math.sin(yaw[0]), math.cos(yaw[0]), 0],
                                   [0, 0, 1.0]])
            wk = work @ Rz @ F                               # the work frame turned with the table, then tilted
            five = any(len(m) > 2 for m in rmoves)
            if backend is not None and getattr(backend, "checks_collision", False):
                if hasattr(backend, "set_tool_mode"):
                    backend.set_tool_mode("spindle")
                if hasattr(backend, "place_part"):
                    backend.place_part(work, yaw[0])
            if five:
                ns = max(1, int(p.get("spins", 12)))
                spins = [2 * math.pi * k / ns for k in range(ns)]
                qs, oks, chosen = _solve_five_axis(robot, backend, wk, rmoves, q, spins)
                poses = [tool_pose(wk, m[1], m[2] if len(m) > 2 else None, s) for m, s in zip(rmoves, chosen)]
                feeds = [m[0] in ("feed", "plunge") for m in rmoves]
                if qs:
                    q = qs[-1]
                msgs.append("step {}: 5-axis: {} points, spin chosen per point from {} candidates ({})".format(
                    i + 1, len(qs), ns, getattr(backend, "name", "numpy IK") if backend else "numpy IK"))
            else:
                first = down_pose(wk, rmoves[0][1])
                q, ok = solve_start(robot, first, q)
                if not ok:
                    msgs.append("step {}: the start of {} is out of reach".format(i + 1, spec.label))
                qs, feeds, oks, poses = [], [], [], []
                for m in rmoves:
                    T = down_pose(wk, m[1])
                    q, ok = robot.solve_pose(T, q)
                    qs.append(q)
                    feeds.append(m[0] in ("feed", "plunge"))
                    oks.append(ok)
                    poses.append(T)
                if backend is not None and getattr(backend, "checks_collision", False):
                    hits = [k for k, qq in enumerate(qs) if backend.in_collision(qq)]
                    for k in hits:
                        oks[k] = False
                    msgs.append("step {}: collision check ({}): {} of {} samples in contact".format(
                        i + 1, backend.name, len(hits), len(qs)))
            add(i, st.title(), qs, feeds, oks)
            rpm = float(p.get("rpm") or getattr(tool, "spindle_rpm", 0) or 0)
            feed = float(p.get("feed") or getattr(tool, "feed", 0) or 0)
            if not rpm or not feed:   # sourced starting values for the material (lithocam.cutting_data)
                from .. import cutting_data
                sg = cutting_data.suggest(tool, station.material, float(p.get("stepdown", 0) or 0),
                                          sp.max_rpm, station.robot_factor)
                rpm = rpm or sg.rpm
                feed = feed or sg.feed
                recs.append(post.Rec("comment", {"text": "cutting data ({}): {}; {}; sources: {}".format(
                    station.material, sg.text(), "; ".join(sg.notes) or "-", ", ".join(sg.sources) or "none")}))
                msgs.append("step {}: cutting data for {} in {}: {} ({} source{})".format(
                    i + 1, tid, station.material, sg.text(), len(sg.sources), "" if len(sg.sources) == 1 else "s"))
            proc.drive_on("spindle", rpm, i)          # spindle and water before the first cut
            if feed <= 0:
                errors.append("step {}: {} has no feed: the step, the tool {} and the cutting data for {} give none. "
                              "Set a feed on the step or the tool.".format(i + 1, spec.label, tid or "(none)",
                                                                           station.material))
            else:
                recs.append(post.Rec("speed", {"linear": feed / 60.0}))
            recs.append(post.Rec("comment", {"text": "{} with {} ({} points)".format(spec.label, tid or "tool",
                                                                                   len(rmoves))}))
            plunge = float(p.get("plunge_feed") or getattr(tool, "plunge_feed", 0) or 0) or feed / 2
            if not (p.get("plunge_feed") or getattr(tool, "plunge_feed", 0)):
                recs.append(post.Rec("comment", {"text": "plunge at half the feed (no plunge feed on the step or "
                                                         "the tool)"}))
            for k, (m, qq, T) in enumerate(zip(rmoves, qs, poses)):
                if m[0] in ("feed", "plunge"):   # cutting: pose of the cutter tip in the robot base (URDF) frame
                    recs.append(post.Rec("movel", {"q": np.asarray(qq).tolist(), "pose": np.asarray(T).tolist(),
                                                   "cut": True, "v": (feed if m[0] == "feed" else plunge) / 60.0}))
                elif k == 0:                     # the approach from wherever the robot is: a joint move
                    recs.append(post.Rec("movej", {"q": np.asarray(qq).tolist()}))
                else:                            # links and retracts: straight lines at the cell's rapid speed
                    recs.append(post.Rec("movel", {"q": np.asarray(qq).tolist(), "pose": np.asarray(T).tolist(),
                                                   "v": float(station.rapid_speed)}))
            bad = len(oks) - sum(oks)
            if bad:
                msgs.append("step {}: {} of {} points unreachable, at a singularity or colliding".format(
                    i + 1, bad, len(oks)))
            feed_len, rapid_len = cam.path_length(moves)
            msgs.append("step {}: {}: {} CL points, feed {:.0f} mm, rapid {:.0f} mm".format(
                i + 1, spec.label, len(moves), feed_len, rapid_len))
        else:
            recs.append(post.Rec("comment", {"text": "{} ({}): no generator yet".format(spec.label, spec.status)}))
            msgs.append("step {}: {} is {}; posted as a comment".format(i + 1, spec.label, spec.status))
    proc.all_off()                                     # spindle, wire motor and water off before the program end
    recs.append(post.Rec("end"))
    if table_mode == "e1":
        # every motion carries the table angle: the one its step chose (wire), else the last table record's
        cur = 0.0
        for r in recs:
            if r.kind == "table":
                cur = float(r.data.get("deg", 0.0))
            elif r.kind in ("movej", "movel"):
                if r.data.get("e1") is None:
                    r.data["e1"] = cur
                else:
                    cur = float(r.data["e1"])
    for e in errors:
        msgs.append("ERROR " + e)
    return Expansion(segs, recs, msgs, errors, station_key(station, program))


def station_key(station: Station, program: Program) -> str:
    """Hash of everything an expansion depends on, so a stale result is never posted."""
    import hashlib
    blob = station.to_json() + "\n" + program.name + "\n" + json.dumps(
        [[s.kind, s.params, s.label] for s in program.steps], sort_keys=True, default=str)
    return hashlib.sha1(blob.encode("utf-8")).hexdigest()
