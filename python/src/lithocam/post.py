# Copyright 2026 The lithocam authors. SPDX-License-Identifier: Apache-2.0
"""Post-processing: a neutral record stream and one writer per controller dialect.

Layer 1 (records) holds what to do, never how a controller spells it. Layer 4 (writers) only formats; it does no
geometry. Joint values come from lithocam's own kinematics (verified IK), so every writer can post joint targets and
the controller runs the configuration lithocam checked. Design notes: docs/POSTPROCESSORS.md.

Record kinds (`Rec.kind`, data keys):
  start(name) end() comment(text) tool(id, tcp: 4x4 in metres, mass kg) speed(linear mm/s, joint 0..1)
  movej(q rad) movel(q rad, pose 4x4 m in the robot base frame) pause(seconds; -1 = wait for the operator)
  io_set(port, value) io_wait(port, value) gripper(width mm, force N) call(name)
  process records, mapped through the cell I/O definition (lithocam.cell_io):
  spindle(rpm, on) water(on) air(on) wire(on, rpm) tool_change(id, mode: none|manual|changer) dwell(seconds)
  io_map(path, io): the cell I/O definition the expansion loaded (writers use it unless the `io` option is given)
  table(deg; driven: absent = the operator turns it, "e1" = coordinated external axis, "io" = a PLC with `ops`)

Turntable as external axis E1: with the table driven "e1" every movej/movel record carries "e1" (degrees). KRL writes
it in the target aggregates (PTP {E6AXIS: A1 .., A6 .., E1 ..}, LIN {X .., C .., E1 ..}); RAPID in the external-axis
part of jointtargets and robtargets ([e1, 9E9, 9E9, 9E9, 9E9, 9E9], eax_a). The table moves with the arm in the same
motion; no stop. URScript has no external axis and refuses such records. The controller's external-axis setup (KUKA
$EX_AX_NUM and $ETx machine data, ABB mechanical unit and its activation) is cell configuration that lithocam does not
write: the programs say so in a PLACEHOLDER comment. `external_axes` reads E1 back.

Dialects: "urscript" (Universal Robots), "krl" (KUKA KRC), "rapid" (ABB IRC5), "cl" (neutral text, one record per
line; also the round-trip format for tests), "gcode" (ISO milling G-code for a 3-axis or 5-axis machine described
by a MillMachine JSON; see `write_gcode`).

Without a cell I/O definition the robot writers post process records as comments (a tool change also as an
operator pause); the studio expansion adds a warning. Robot programs from these writers are for simulation
(URSim, KUKA.OfficeLite, RobotStudio) until a hardware gate is passed. Joint-direction conventions of each cell URDF
must match its controller; check every new cell in its simulator first.
"""
from __future__ import annotations

import json
import math
from dataclasses import asdict, dataclass, field, fields
from pathlib import Path
from typing import Callable, Dict, List, Optional, Sequence, Tuple

import numpy as np

from . import cell_io

PROCESS_KINDS = ("spindle", "water", "air", "wire", "tool_change")


@dataclass
class Rec:
    kind: str
    data: Dict[str, object] = field(default_factory=dict)


def rotvec(R: np.ndarray) -> np.ndarray:
    c = max(-1.0, min(1.0, (np.trace(R) - 1) / 2))
    a = math.acos(c)
    if a < 1e-12:
        return np.zeros(3)
    if math.pi - a < 1e-6:
        B = (R + np.eye(3)) / 2
        k = int(np.argmax(np.diag(B)))
        v = B[:, k] / math.sqrt(max(B[k, k], 1e-300))
        return v / np.linalg.norm(v) * a
    return np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]]) / (2 * math.sin(a)) * a


def zyx_deg(R: np.ndarray):
    """KUKA A, B, C (rotations about Z, then Y, then X), degrees."""
    b = math.asin(max(-1.0, min(1.0, -R[2, 0])))
    if abs(math.cos(b)) > 1e-9:
        a, c = math.atan2(R[1, 0], R[0, 0]), math.atan2(R[2, 1], R[2, 2])
    else:
        a, c = math.atan2(-R[0, 1], R[1, 1]), 0.0
    return math.degrees(a), math.degrees(b), math.degrees(c)


def quat(R: np.ndarray):
    """(w, x, y, z), w >= 0."""
    t = np.trace(R)
    if t > 0:
        s = math.sqrt(t + 1) * 2
        q = [s / 4, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s]
    else:
        i = int(np.argmax(np.diag(R)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = math.sqrt(1 + R[i, i] - R[j, j] - R[k, k]) * 2
        q = [0.0] * 4
        q[0] = (R[k, j] - R[j, k]) / s
        q[1 + i] = s / 4
        q[1 + j] = (R[j, i] + R[i, j]) / s
        q[1 + k] = (R[k, i] + R[i, k]) / s
    q = np.array(q)
    return q if q[0] >= 0 else -q


def blend_radii(recs: List[Rec], blend: float) -> List[float]:
    """Blend radius (m) per record. Only between two moves with known TCP positions (data "pose"), and at most
    40% of the shorter neighbouring segment, so neighbouring blends never overlap (UR's rule for blended moves).
    0 at the first and last move, next to non-motion records and where a move has no pose."""
    out = [0.0] * len(recs)
    if blend <= 0:
        return out
    moves = [k for k, r in enumerate(recs) if r.kind in ("movej", "movel")]
    for a, k, b in zip(moves, moves[1:], moves[2:]):
        if b - a != 2 or not all(recs[i].data.get("pose") is not None for i in (a, k, b)):
            continue   # something happens between the moves (I/O, tool change): stop exactly
        p = [np.asarray(recs[i].data["pose"], dtype=float)[:3, 3] for i in (a, k, b)]
        seg = min(np.linalg.norm(p[1] - p[0]), np.linalg.norm(p[2] - p[1]))
        out[k] = float(min(blend, 0.4 * seg))
    return out


# ---------------------------------------------------------------- process records through the cell I/O
def _process(recs: List[Rec], opt: Dict, dialect: str):
    """(Process state, note): the cell I/O from the `io` option, else from the io_map record; a definition for
    another controller is ignored with a note."""
    io = opt.get("io")
    if io is None:
        m = next((r for r in recs if r.kind == "io_map"), None)
        if m is not None and m.data.get("io") is not None:
            io = cell_io.from_dict(m.data["io"], str(m.data.get("path", "")))
    io = cell_io.coerce(io)
    note = None
    if io is not None and io.controller != dialect:
        note = "cell I/O '{}' is for {} programs, not {}: process records post as comments".format(
            io.name, io.controller, dialect)
        io = None
    if note and isinstance(opt.get("warnings"), list):
        opt["warnings"].append(note)
    return cell_io.Process(io), note


def _start_ops(proc, note) -> list:
    ops = [("comment", note)] if note else []
    if proc.io is not None:
        ops += proc.io.start_ops()
    return ops


def _end_ops(proc, opt) -> list:
    """Safety net: drives and switches still on at the end are turned off, with a warning."""
    ops = []
    for k in proc.still_on():
        w = "WARNING: {} still on at the program end; turned off here".format(k)
        if isinstance(opt.get("warnings"), list):
            opt["warnings"].append(w)
        ops.append(("comment", w))
        ops += proc.ops(k, {"on": False, "rpm": 0})
    return ops


def _text(s) -> str:
    return str(s).replace("\n", " ").replace('"', "'")


def _uses_e1(recs: List[Rec]) -> bool:
    return any((r.kind in ("movej", "movel") and r.data.get("e1") is not None) or
               (r.kind == "table" and r.data.get("driven") == "e1") for r in recs)


def _e1(d) -> Optional[float]:
    v = d.get("e1")
    return None if v is None else float(v)


def _ur_ops(ops) -> List[str]:
    out = []
    for op in ops:
        k = op[0]
        if k == "do":
            out.append("  set_standard_digital_out({}, {})".format(int(op[1]), bool(op[2])))
        elif k == "ao":
            out.append("  set_standard_analog_out({}, {:.4f})".format(int(op[1]), float(op[2])))
        elif k == "wait_di":
            neg = "not " if op[2] else ""
            out += ["  while {}get_standard_digital_in({}):".format(neg, int(op[1])), "    sync()", "  end"]
        elif k == "register":
            fn = "write_output_float_register" if op[3] == "float" else "write_output_integer_register"
            out.append("  {}({}, {})".format(fn, op[1], op[2]))
        elif k == "call":
            out.append("  " + str(op[1]))
        elif k == "dwell":
            out.append("  sleep({:.3f})".format(float(op[1])))
        elif k == "message":
            out.append('  popup("lithocam: {}", blocking=True)'.format(_text(op[1])))
        else:   # comment (and mcode, which validation keeps out of URScript definitions)
            out.append("  # " + _text(op[1]))
    return out


def _krl_ops(ops) -> List[str]:
    out = []
    for op in ops:
        k = op[0]
        if k == "do":
            out.append("  $OUT[{}] = {}".format(int(op[1]), "TRUE" if op[2] else "FALSE"))
        elif k == "ao":
            out.append("  $ANOUT[{}] = {:.4f}".format(int(op[1]), float(op[2])))
        elif k == "wait_di":
            out.append("  WAIT FOR $IN[{}] == {}".format(int(op[1]), "TRUE" if op[2] else "FALSE"))
        elif k == "register":
            out.append("  {} = {}".format(op[1], op[2]))
        elif k == "call":
            out.append("  " + str(op[1]))
        elif k == "dwell":
            out.append("  WAIT SEC {:.3f}".format(float(op[1])))
        elif k == "message":
            out += ["  ; " + _text(op[1]), "  HALT"]
        else:
            out.append("  ; " + _text(op[1]))
    return out


def _rapid_sig(prefix: str, port) -> str:
    return str(port) if isinstance(port, str) else "{}_{}".format(prefix, int(port))


def _rapid_ops(ops) -> List[str]:
    out = []
    for op in ops:
        k = op[0]
        if k == "do":
            out.append("    SetDO {}, {};".format(_rapid_sig("DO", op[1]), 1 if op[2] else 0))
        elif k == "ao":
            out.append("    SetAO {}, {:.4f};".format(_rapid_sig("AO", op[1]), float(op[2])))
        elif k == "wait_di":
            out.append("    WaitDI {}, {};".format(_rapid_sig("DI", op[1]), 1 if op[2] else 0))
        elif k == "register":
            out.append("    SetGO {}, {};".format(op[1], op[2]))
        elif k == "call":
            t = str(op[1]).strip()
            out.append("    " + (t if t.endswith(";") else t + ";"))
        elif k == "dwell":
            out.append("    WaitTime {:.3f};".format(float(op[1])))
        elif k == "message":
            out += ['    TPWrite "{}";'.format(_text(op[1])[:80]), "    Stop;"]
        else:
            out.append("    ! " + _text(op[1]))
    return out


# ---------------------------------------------------------------- writers
def _urscript(recs: List[Rec], opt: Dict) -> str:
    name = "lithocam"
    out: List[str] = []
    acc_j, acc_l = opt.get("joint_accel", 1.0), opt.get("linear_accel", 0.5)
    vj = opt.get("joint_speed_max", 1.0) * 0.5      # rad/s
    vl = 0.1                                        # m/s
    blend = opt.get("blend", 0.0)
    fmt = lambda q: "[" + ", ".join("{:.6f}".format(float(v)) for v in q) + "]"   # noqa: E731
    B_inv = np.linalg.inv(np.asarray(opt.get("base_frame", np.eye(4)), dtype=float))
    cut_motion = opt.get("cut_motion", "movel")
    radii = blend_radii(recs, blend)
    proc, note = _process(recs, opt, "urscript")
    if _uses_e1(recs):
        raise ValueError("URScript has no external axis: these records drive the turntable as E1. A UR cell turns "
                         "the table by the operator (table driven 'operator') or by a PLC through I/O ('io').")

    def pose_p(T):
        T = B_inv @ np.asarray(T, dtype=float)
        return "p[{:.6f}, {:.6f}, {:.6f}, {:.6f}, {:.6f}, {:.6f}]".format(*T[:3, 3], *rotvec(T[:3, :3]))

    for k, r in enumerate(recs):
        d = r.data
        if r.kind == "start":
            name = str(d.get("name", name))
            out.append("def {}():".format(name))
            out += _ur_ops(_start_ops(proc, note))
        elif r.kind == "end":
            out += _ur_ops(_end_ops(proc, opt))
            out += ["end", "{}()".format(name)]
        elif r.kind in PROCESS_KINDS:
            out += _ur_ops(proc.ops(r.kind, d))
        elif r.kind == "dwell":
            out.append("  sleep({:.3f})".format(float(d.get("seconds", 0.0))))
        elif r.kind == "comment":
            out.append("  # " + str(d.get("text", "")).replace("\n", " "))
        elif r.kind == "tool":
            T = np.asarray(d["tcp"], dtype=float)
            p = "p[{:.6f}, {:.6f}, {:.6f}, {:.6f}, {:.6f}, {:.6f}]".format(*T[:3, 3], *rotvec(T[:3, :3]))
            out.append("  set_tcp({})  # tool {}".format(p, d.get("id", "")))
            if d.get("mass"):
                out.append("  set_payload({:.3f})".format(float(d["mass"])))
            else:
                out.append("  # payload not set: enter the spindle and cutter mass before running")
        elif r.kind == "speed":
            if d.get("linear"):
                vl = float(d["linear"]) / 1000
            if d.get("joint"):
                vj = opt.get("joint_speed_max", 1.0) * float(d["joint"])
        elif r.kind == "movej":
            v = opt.get("joint_speed_max", 1.0) * float(d["vj"]) if d.get("vj") else vj
            rr = float(d["r"]) if d.get("r") is not None else radii[k]
            out.append("  movej({}, a={}, v={:.4f}, r={:.4f})".format(fmt(d["q"]), acc_j, v, rr))
        elif r.kind == "movel":
            v = float(d["v"]) / 1000 if d.get("v") else vl          # record speed in mm/s
            rr = float(d["r"]) if d.get("r") is not None else radii[k]
            if cut_motion == "movep" and d.get("cut") and d.get("pose") is not None:
                # constant tool speed through the cut; target pose in the controller base frame
                out.append("  movep({}, a={}, v={:.4f}, r={:.4f})  # q {}".format(pose_p(d["pose"]), acc_l, v,
                                                                               rr, fmt(d["q"])))
            else:   # joint target: linear in tool space, ends in the verified configuration
                out.append("  movel({}, a={}, v={:.4f}, r={:.4f})".format(fmt(d["q"]), acc_l, v, rr))
        elif r.kind == "pause":
            s = float(d.get("seconds", -1))
            out.append("  sleep({:.3f})".format(s) if s >= 0 else '  popup("lithocam: paused, press Continue", '
                                                                       'blocking=True)')
        elif r.kind == "io_set":
            out.append("  set_standard_digital_out({}, {})".format(int(d["port"]), bool(d.get("value", True))))
        elif r.kind == "io_wait":
            neg = "not " if d.get("value", True) else ""   # wait while the input differs from the wanted value
            out += ["  while {}get_standard_digital_in({}):".format(neg, int(d["port"])), "    sync()", "  end"]
        elif r.kind == "table" and d.get("driven") == "io":   # a PLC turns it: request, wait for in position
            out += _ur_ops(d.get("ops") or [])
        elif r.kind == "table":   # the UR has no table axis: the operator (or a PLC) turns it
            out.append('  popup("lithocam: turn the table to {:.1f} deg, then Continue", blocking=True)'.format(
                float(d.get("deg", 0.0))))
        elif r.kind == "gripper":
            out.append(opt.get("gripper_call", "  # gripper width {width} mm force {force} N: set the cell's gripper "
                                               "call").format(**{"width": d.get("width", 0), "force": d.get("force", 0)}))
        elif r.kind == "call":
            out.append("  {}()".format(d.get("name", "")))
    return "\n".join(out) + "\n"


def _krl(recs: List[Rec], opt: Dict) -> str:
    out: List[str] = []
    deg = lambda q: ", ".join("A{} {:.4f}".format(i + 1, math.degrees(float(v))) for i, v in enumerate(q))  # noqa
    approx = " C_PTP" if opt.get("blend", 0.0) > 0 else ""
    cur_cp = [None]
    proc, note = _process(recs, opt, "krl")
    e1 = _uses_e1(recs)
    ext = lambda d: ", E1 {:.4f}".format(_e1(d)) if _e1(d) is not None else ""   # noqa: E731
    for r in recs:
        d = r.data
        if r.kind == "start":
            out += ["&ACCESS RVP", "DEF {}()".format(d.get("name", "lithocam")), "  ;FOLD INI",
                    "  BAS(#INITMOV, 0)", "  ;ENDFOLD",
                    "  $BASE = {X 0, Y 0, Z 0, A 0, B 0, C 0}  ; LIN targets are in the robot root frame"]
            if e1:
                out += ["  ; E1 = turntable angle (deg) about +z of the work frame, 0 = the part as loaded; it moves in",
                        "  ; the same PTP/LIN as the arm. PLACEHOLDER: the external axis ($EX_AX_NUM, $ETx machine",
                        "  ; data, E1 zero and direction) is cell configuration, set for your cell; LIN targets stay",
                        "  ; in the robot root frame ($BASE not coupled to the table), sampled densely by lithocam"]
            if opt.get("blend", 0.0) > 0:
                out.append("  $APO.CPTP = {:.0f}".format(min(100.0, opt["blend"])))
                out.append("  $APO.CDIS = {:.1f}".format(float(opt["blend"])))
            out += _krl_ops(_start_ops(proc, note))
        elif r.kind == "end":
            out += _krl_ops(_end_ops(proc, opt))
            out.append("END")
        elif r.kind in PROCESS_KINDS:
            out += _krl_ops(proc.ops(r.kind, d))
        elif r.kind == "dwell":
            out.append("  WAIT SEC {:.3f}".format(float(d.get("seconds", 0.0))))
        elif r.kind == "comment":
            out.append("  ; " + str(d.get("text", "")).replace("\n", " "))
        elif r.kind == "tool":
            T = np.asarray(d["tcp"], dtype=float)
            a, b, c = zyx_deg(T[:3, :3])
            x, y, z = T[:3, 3] * 1000
            out.append("  $TOOL = {{X {:.3f}, Y {:.3f}, Z {:.3f}, A {:.4f}, B {:.4f}, C {:.4f}}}  ; tool {}".format(
                x, y, z, a, b, c, d.get("id", "")))
        elif r.kind == "speed":
            if d.get("joint"):
                pct = max(1, min(100, round(float(d["joint"]) * 100)))
                out += ["  $VEL_AXIS[{}] = {}".format(i, pct) for i in range(1, 7)]
            if d.get("linear"):
                out.append("  $VEL.CP = {:.4f}".format(float(d["linear"]) / 1000))
                cur_cp[0] = float(d["linear"]) / 1000
        elif r.kind == "movel" and d.get("pose") is not None:
            # straight line in Cartesian space. KRL evaluates Status and Turn only for PTP; a LIN keeps the
            # configuration of its start point, which a PTP to verified joints set. Pose: TCP in the robot root.
            T = np.asarray(d["pose"], dtype=float)
            a, b, c = zyx_deg(T[:3, :3])
            x, y, z = T[:3, 3] * 1000
            if d.get("v"):
                v = float(d["v"]) / 1000
                if v != cur_cp[0]:
                    out.append("  $VEL.CP = {:.4f}".format(v))
                    cur_cp[0] = v
            cdis = " C_DIS" if (d.get("r") or opt.get("blend", 0.0)) else ""
            out.append("  LIN {{X {:.3f}, Y {:.3f}, Z {:.3f}, A {:.4f}, B {:.4f}, C {:.4f}{}}}{}  ; q [{}]".format(
                x, y, z, a, b, c, ext(d), cdis, ", ".join("{:.6f}".format(float(v)) for v in d["q"])))
        elif r.kind in ("movej", "movel"):   # joint targets: the verified configuration
            if d.get("vj"):
                pct = max(1, min(100, round(float(d["vj"]) * 100)))
                out += ["  $VEL_AXIS[{}] = {}".format(i, pct) for i in range(1, 7)]
            if _e1(d) is not None:
                out.append("  PTP {{E6AXIS: {}{}}}{}".format(deg(d["q"]), ext(d), approx))
            else:
                out.append("  PTP {{AXIS: {}}}{}".format(deg(d["q"]), approx))
        elif r.kind == "pause":
            s = float(d.get("seconds", -1))
            out.append("  WAIT SEC {:.3f}".format(s) if s >= 0 else "  HALT")
        elif r.kind == "io_set":
            out.append("  $OUT[{}] = {}".format(int(d["port"]), "TRUE" if d.get("value", True) else "FALSE"))
        elif r.kind == "io_wait":
            out.append("  WAIT FOR $IN[{}] == {}".format(int(d["port"]), "TRUE" if d.get("value", True) else "FALSE"))
        elif r.kind == "table" and d.get("driven") == "e1":
            out.append("  ; turntable E1 to {:.3f} deg with the next motion (coordinated, no stop)".format(
                float(d.get("deg", 0.0))))
        elif r.kind == "table" and d.get("driven") == "io":
            out += _krl_ops(d.get("ops") or [])
        elif r.kind == "table":
            out.append("  ; turntable to {:.3f} deg (external axis E1 once the cell defines it)".format(
                float(d.get("deg", 0.0))))
            out.append("  HALT")
        elif r.kind == "gripper":
            out.append("  ; gripper width {} mm force {} N".format(d.get("width", 0), d.get("force", 0)))
        elif r.kind == "call":
            out.append("  {}()".format(d.get("name", "")))
    return "\n".join(out) + "\n"


def _rapid(recs: List[Rec], opt: Dict) -> str:
    head: List[str] = []
    body: List[str] = []
    name = "lithocam"
    tool = "tool0"
    speed = "v100"
    zone = "fine" if opt.get("blend", 0.0) <= 0 else "z{}".format(int(max(1, min(200, opt["blend"]))))
    deg = lambda q: ", ".join("{:.4f}".format(math.degrees(float(v))) for v in q)  # noqa: E731
    n_speed = 0
    speeds: Dict[float, str] = {}
    lin_conf = [False]
    proc, note = _process(recs, opt, "rapid")
    e1 = _uses_e1(recs)

    def extax(d) -> str:
        v = _e1(d)
        return "[{}, 9E9, 9E9, 9E9, 9E9, 9E9]".format("9E9" if v is None else "{:.4f}".format(v))

    for r in recs:
        d = r.data
        if r.kind == "start":
            name = str(d.get("name", name))
            if e1:
                body += ["    ! eax_a = turntable angle (deg) about +z of the work frame, 0 = the part as loaded; it",
                         "    ! moves in the same MoveAbsJ/MoveL as the arm. PLACEHOLDER: the table's mechanical unit,",
                         "    ! its activation (ActUnit) and calibration are cell configuration, set for your cell"]
            body += _rapid_ops(_start_ops(proc, note))
        elif r.kind == "end":
            body += _rapid_ops(_end_ops(proc, opt))
        elif r.kind in PROCESS_KINDS:
            body += _rapid_ops(proc.ops(r.kind, d))
        elif r.kind == "dwell":
            body.append("    WaitTime {:.3f};".format(float(d.get("seconds", 0.0))))
        elif r.kind == "comment":
            body.append("    ! " + str(d.get("text", "")).replace("\n", " "))
        elif r.kind == "tool":
            T = np.asarray(d["tcp"], dtype=float)
            q = quat(T[:3, :3])
            tool = "t_" + "".join(ch if ch.isalnum() else "_" for ch in str(d.get("id", "tool")))
            head.append("  PERS tooldata {} := [TRUE, [[{:.3f}, {:.3f}, {:.3f}], [{:.6f}, {:.6f}, {:.6f}, {:.6f}]], "
                        "[{:.3f}, [0, 0, 1], [1, 0, 0, 0], 0, 0, 0]];".format(
                            tool, *(T[:3, 3] * 1000), *q, max(float(d.get("mass", 1.0)), 0.001)))
        elif r.kind == "speed":
            n_speed += 1
            speed = "v_lc{}".format(n_speed)
            lin = float(d.get("linear") or 100.0)
            head.append("  VAR speeddata {} := [{:.1f}, 500, 5000, 1000];".format(speed, lin))
        elif r.kind == "movel" and d.get("pose") is not None:
            # straight line to a robtarget (TCP in the robot base, wobj0). Quadrants cf1, cf4, cf6 come from the
            # verified joints; cfx is not derived, so configuration monitoring is off for linear moves and the
            # move keeps the configuration that the joint move before it set.
            if not lin_conf[0]:
                body.append("    ConfL \\Off;")
                lin_conf[0] = True
            T = np.asarray(d["pose"], dtype=float)
            qq = quat(T[:3, :3])
            qd = [math.degrees(float(v)) for v in d["q"]]
            cf = [int(math.floor(qd[j] / 90.0)) for j in (0, 3, 5)]
            sp = speed
            if d.get("v"):
                key = round(float(d["v"]), 3)
                if key not in speeds:
                    speeds[key] = "v_lcf{}".format(len(speeds) + 1)
                    head.append("  VAR speeddata {} := [{:.3f}, 500, 5000, 1000];".format(speeds[key], key))
                sp = speeds[key]
            body.append("    MoveL [[{:.3f}, {:.3f}, {:.3f}], [{:.6f}, {:.6f}, {:.6f}, {:.6f}], [{}, {}, {}, 0], "
                        "{}], {}, {}, {};  ! q [{}]".format(
                            *(T[:3, 3] * 1000), *qq, *cf, extax(d), sp, zone, tool,
                            ", ".join("{:.6f}".format(float(v)) for v in d["q"])))
        elif r.kind in ("movej", "movel"):
            body.append("    MoveAbsJ [[{}], {}], {}, {}, {};".format(deg(d["q"]), extax(d), speed, zone, tool))
        elif r.kind == "pause":
            s = float(d.get("seconds", -1))
            body.append("    WaitTime {:.3f};".format(s) if s >= 0 else "    Stop;")
        elif r.kind == "io_set":
            body.append("    SetDO DO_{}, {};".format(int(d["port"]), 1 if d.get("value", True) else 0))
        elif r.kind == "io_wait":
            body.append("    WaitDI DI_{}, {};".format(int(d["port"]), 1 if d.get("value", True) else 0))
        elif r.kind == "table" and d.get("driven") == "e1":
            body.append("    ! turntable eax_a to {:.3f} deg with the next motion (coordinated, no stop)".format(
                float(d.get("deg", 0.0))))
        elif r.kind == "table" and d.get("driven") == "io":
            body += _rapid_ops(d.get("ops") or [])
        elif r.kind == "table":
            body.append("    ! turntable to {:.3f} deg (external axis once the cell defines it)".format(
                float(d.get("deg", 0.0))))
            body.append("    Stop;")
        elif r.kind == "gripper":
            body.append("    ! gripper width {} mm force {} N".format(d.get("width", 0), d.get("force", 0)))
        elif r.kind == "call":
            body.append("    {};".format(d.get("name", "")))
    return "\n".join(["MODULE {}".format(name)] + head + ["  PROC main()"] + body + ["  ENDPROC", "ENDMODULE"]) + "\n"


def _cl(recs: List[Rec], opt: Dict) -> str:
    def enc(v):
        if isinstance(v, np.ndarray):
            return v.tolist()
        return v
    return "\n".join(r.kind + " " + json.dumps({k: enc(v) for k, v in r.data.items()}) for r in recs) + "\n"


def parse_cl(text: str) -> List[Rec]:
    out = []
    for line in text.splitlines():
        if line.strip():
            k, _, rest = line.partition(" ")
            out.append(Rec(k, json.loads(rest)))
    return out


# ================================================================ G-code milling post
# Positions are the cutter tip in the part (work) frame, mm; the tool axis points from the tip to the holder. Both
# come from the pose of each motion record: tip = W^-1 pose, axis = minus the pose z axis (W = `work_frame`, the work
# frame in the robot base, metres). Records without a pose (robot joint moves) become a safe approach: retract to
# safe_z, turn the rotary axes, move over the next point.
#
# 5-axis kinematics. Two rotary axes, primary (on the machine frame) and secondary (carried by the primary), each
# with a unit direction (positive turn by the right-hand rule). The tool points along machine +Z at zero rotation.
#   table: the axes turn the part. R1(a1) R2(a2) v = Z for a part-frame tool axis v. Machine position of a part
#          point P: o1 + R1 [(o2 - o1) + R2 (P - o2)], o1, o2 = pivot points at zero rotation (machine coordinates
#          relative to the program zero).
#   head:  the axes turn the spindle. R1(a1) R2(a2) Z = v. Programmed point at zero rotation = the tip:
#          X = tip - p1 - R1 [(p2 - p1) - R2 p2], p1, p2 = pivots from the tip (pivot from the gauge point + tool length).
# Both reduce to R1 R2 p = q, solved in closed form (Paden-Kahan subproblem 2: rotation about two axes): zero, one or
# two solutions. Choice: each solution is unwound to the angles nearest the previous point (rotary limits respected),
# and the one with the smallest largest rotary change wins. Pole: when the tool axis lies within `pole_deg` of the
# axis whose angle becomes undefined (table AC: the C axis), that angle is held and the other one is solved for the
# nearest reachable tool axis (deviation at most pole_deg). Across a run of such points the held angle turns
# linearly, by tip path length, from its value before the run to the value the first point after the run needs, so
# the rotary axis never jumps at the exit of the pole cone. A cutting move that still needs a rotary turn above
# `rewind_deg` (a limited axis out of travel) gets a retract, turn and re-entry. With `tcp` the program carries the
# tip position and the angles (G43.4-style tool centre point control); without it the machine coordinates.
class GcodeError(ValueError):
    pass


_KINEMATICS = ("3axis", "table", "head")
_GC_RESERVED = set("GMFNPDIJKOHSTLQRE")


@dataclass
class MillMachine:
    """A milling machine, or a robot controller that reads G-code. Lengths mm, angles degrees. The example files in
    examples/gcode_machines hold placeholders."""
    name: str = "generic_3axis"
    kinematics: str = "3axis"                       # "3axis" | "table" | "head"
    axes: Dict[str, str] = field(default_factory=lambda: {"x": "X", "y": "Y", "z": "Z"})   # + "primary", "secondary"
    primary_axis: List[float] = field(default_factory=lambda: [1.0, 0.0, 0.0])
    secondary_axis: List[float] = field(default_factory=lambda: [0.0, 0.0, 1.0])
    table_pivots: Dict[str, List[float]] = field(default_factory=lambda: {"primary": [0.0, 0.0, 0.0],
                                                                          "secondary": [0.0, 0.0, 0.0]})
    head_pivots: Dict[str, List[float]] = field(default_factory=lambda: {"primary": [0.0, 0.0, 0.0],
                                                                         "secondary": [0.0, 0.0, 0.0]})
    limits: Dict[str, List[float]] = field(default_factory=dict)    # axis letter -> [lo, hi]; absent = no limit
    prefer_primary_sign: int = 1                    # tie-break between the two solutions at the first point
    tcp: bool = False                               # tool centre point control on the controller
    tcp_on: Optional[str] = "G43.4 H{pocket}"
    length_comp: Optional[str] = "G43 H{pocket}"    # without TCP
    comp_cancel: Optional[str] = "G49"
    feed_mode: str = "G94"                          # "G94" mm/min, "G93" inverse time
    rotary_feed_deg_min: Optional[float] = None     # for G93 blocks that only turn
    arcs: bool = False                              # fit G2/G3 to the CL points (3-axis only)
    arc_tol: float = 0.005
    arc_min_radius: float = 0.5
    arc_max_radius: float = 5000.0
    pole_deg: float = 0.5
    vertical_tol_deg: float = 0.01                  # 3-axis: largest tool-axis tilt accepted as vertical
    max_rotary_step_deg: Optional[float] = None     # linearise cutting moves (no TCP) above this rotary step
    rewind_deg: float = 90.0
    safe_z: float = 50.0
    decimals: int = 3
    rotary_decimals: int = 4
    dwell_in_ms: bool = False
    line_numbers: int = 0
    max_lines: Optional[int] = None
    split: str = "subprograms"                      # "subprograms" (main calls M98 P) | "files"
    program_number: Optional[int] = None
    sub_number: int = 1000
    header: List[str] = field(default_factory=lambda: ["G17 G21 G90 G40 G80", "G54"])
    footer: List[str] = field(default_factory=lambda: ["M30"])
    io: Optional[Dict] = None                       # cell I/O definition (controller "gcode") for M-codes
    placeholder: bool = True
    notes: str = ""

    # ---- JSON
    def to_dict(self) -> dict:
        return asdict(self)

    @staticmethod
    def from_dict(d: dict) -> "MillMachine":
        known = {f.name for f in fields(MillMachine)}
        bad = set(d) - known
        if bad:
            raise GcodeError("unknown machine definition field(s): {}".format(sorted(bad)))
        m = MillMachine(**d)
        m.validate()
        return m

    @staticmethod
    def load(path) -> "MillMachine":
        return MillMachine.from_dict(json.loads(Path(path).read_text(encoding="utf-8")))

    def save(self, path) -> None:
        Path(path).write_text(json.dumps(self.to_dict(), indent=2) + "\n", encoding="utf-8")

    # ---- checks
    @property
    def five(self) -> bool:
        return self.kinematics in ("table", "head")

    def validate(self) -> None:
        if self.kinematics not in _KINEMATICS:
            raise GcodeError("kinematics must be one of {}".format(_KINEMATICS))
        need = ["x", "y", "z"] + (["primary", "secondary"] if self.five else [])
        for a in need:
            if a not in self.axes:
                raise GcodeError("axes needs a letter for '{}'".format(a))
        for a, L in self.axes.items():
            if a not in ("x", "y", "z", "primary", "secondary"):
                raise GcodeError("unknown axis '{}' (use x, y, z, primary, secondary)".format(a))
            if not (isinstance(L, str) and len(L) == 1 and L.isalpha() and L.isupper()) or L in _GC_RESERVED:
                raise GcodeError("axis letter {!r} for {} must be one capital letter, not one of {}".format(
                    L, a, "".join(sorted(_GC_RESERVED))))
        if len(set(self.axes.values())) != len(self.axes):
            raise GcodeError("two axes share a letter")
        if self.five:
            w1, w2 = (np.asarray(v, dtype=float) for v in (self.primary_axis, self.secondary_axis))
            if np.linalg.norm(w1) < 1e-9 or np.linalg.norm(w2) < 1e-9:
                raise GcodeError("rotary axis directions must be non-zero")
            if abs(abs(w1 @ w2) / np.linalg.norm(w1) / np.linalg.norm(w2) - 1) < 1e-9:
                raise GcodeError("the two rotary axes are parallel")
            for key in ("primary", "secondary"):
                for piv in (self.table_pivots, self.head_pivots):
                    if len(piv.get(key, [])) != 3:
                        raise GcodeError("pivots need 3 coordinates for {}".format(key))
        elif self.tcp:
            raise GcodeError("tcp needs a 5-axis kinematics (table or head)")
        for L, lim in self.limits.items():
            if L not in self.axes.values():
                raise GcodeError("limit for {}, which is not an axis letter of this machine".format(L))
            if len(lim) != 2 or not lim[0] < lim[1]:
                raise GcodeError("limit of {} must be [lo, hi] with lo < hi".format(L))
        if self.feed_mode not in ("G94", "G93"):
            raise GcodeError("feed_mode must be G94 or G93")
        if self.split not in ("subprograms", "files"):
            raise GcodeError("split must be subprograms or files")
        if self.arcs and self.five:
            raise GcodeError("arcs are fitted for 3-axis posts only; set arcs false")
        if self.max_lines is not None and self.max_lines < 30:
            raise GcodeError("max_lines must be at least 30")
        if not 0 <= self.decimals <= 6 or not 0 <= self.rotary_decimals <= 6:
            raise GcodeError("decimals must be 0..6")
        if self.pole_deg < 0 or self.vertical_tol_deg < 0 or self.rewind_deg <= 0:
            raise GcodeError("pole_deg and vertical_tol_deg must be >= 0, rewind_deg > 0")
        if self.io is not None:
            io = cell_io.from_dict(self.io)
            if io.controller != "gcode":
                raise GcodeError("the machine's io must be a cell I/O definition with controller gcode")

    # ---- kinematics
    def _w(self):
        w1 = np.asarray(self.primary_axis, dtype=float)
        w2 = np.asarray(self.secondary_axis, dtype=float)
        return w1 / np.linalg.norm(w1), w2 / np.linalg.norm(w2)

    def _pq(self, v):
        z = np.array([0.0, 0.0, 1.0])
        return (np.asarray(v, dtype=float), z) if self.kinematics == "table" else (z, np.asarray(v, dtype=float))

    def axis_of(self, a1: float = 0.0, a2: float = 0.0) -> np.ndarray:
        """Tool axis (tip to holder) in the part frame at rotary angles a1, a2 (deg): forward kinematics."""
        z = np.array([0.0, 0.0, 1.0])
        if not self.five:
            return z
        w1, w2 = self._w()
        R1, R2 = _rotm(w1, a1), _rotm(w2, a2)
        return R2.T @ R1.T @ z if self.kinematics == "table" else R1 @ R2 @ z

    def _head_p(self, L: float):
        add = np.array([0.0, 0.0, float(L)])
        return (np.asarray(self.head_pivots["primary"], dtype=float) + add,
                np.asarray(self.head_pivots["secondary"], dtype=float) + add)

    def to_machine(self, tip, a1: float = 0.0, a2: float = 0.0, tool_length: float = 0.0) -> np.ndarray:
        """Programmed linear axes for a tip (part frame, mm) at angles a1, a2."""
        tip = np.asarray(tip, dtype=float)
        if not self.five or self.tcp:
            return tip.copy()
        w1, w2 = self._w()
        R1, R2 = _rotm(w1, a1), _rotm(w2, a2)
        if self.kinematics == "table":
            o1, o2 = (np.asarray(self.table_pivots[k], dtype=float) for k in ("primary", "secondary"))
            return o1 + R1 @ ((o2 - o1) + R2 @ (tip - o2))
        p1, p2 = self._head_p(tool_length)
        return tip - p1 - R1 @ ((p2 - p1) - R2 @ p2)

    def from_machine(self, X, a1: float = 0.0, a2: float = 0.0, tool_length: float = 0.0) -> np.ndarray:
        """Tip in the part frame from programmed linear axes and angles: the read-back forward kinematics."""
        X = np.asarray(X, dtype=float)
        if not self.five or self.tcp:
            return X.copy()
        w1, w2 = self._w()
        R1, R2 = _rotm(w1, a1), _rotm(w2, a2)
        if self.kinematics == "table":
            o1, o2 = (np.asarray(self.table_pivots[k], dtype=float) for k in ("primary", "secondary"))
            return o2 + R2.T @ (R1.T @ (X - o1) - (o2 - o1))
        p1, p2 = self._head_p(tool_length)
        return X + p1 + R1 @ ((p2 - p1) - R2 @ p2)

    def solutions(self, v) -> List[Tuple[float, float]]:
        """All (a1, a2) in degrees that point the tool along v (part frame); empty if unreachable."""
        w1, w2 = self._w()
        p, q = self._pq(v)
        return _sp2(w1, w2, p, q)

    def pole(self, v) -> Optional[int]:
        """0 or 1 when the primary or secondary angle is undefined (within pole_deg) for tool axis v."""
        w1, w2 = self._w()
        p, q = self._pq(v)
        lim = max(self.pole_deg, 1e-7)
        if _par_deg(p, w2) < lim:
            return 1
        if _par_deg(q, w1) < lim:
            return 0
        return None

    def held(self, v, h: int, H: float) -> Optional[Tuple[float, float]]:
        """Angles with axis h held at H (deg), the other solved for the nearest reachable tool axis."""
        w1, w2 = self._w()
        p, q = self._pq(v)
        if h == 1:
            t1 = _sp1(w1, _rotm(w2, H) @ p, q)
            return None if t1 is None else (t1, H)
        t2 = _sp1(w2, p, _rotm(w1, -H) @ q)
        return None if t2 is None else (H, t2)

    def _fit(self, letter: str, a: float, ref: float) -> Optional[float]:
        lim = self.limits.get(letter)
        b = a + 360.0 * round((ref - a) / 360.0)
        if lim is None:
            return b
        lo, hi = lim
        if lo - 1e-9 <= b <= hi + 1e-9:
            return b
        ks = [a + 360.0 * k for k in range(-4, 5) if lo - 1e-9 <= a + 360.0 * k <= hi + 1e-9]
        return min(ks, key=lambda x: abs(x - ref)) if ks else None

    def choose(self, sols, prev) -> Optional[Tuple[float, float]]:
        L1, L2 = self.axes["primary"], self.axes["secondary"]
        best = None
        for t1, t2 in sols:
            u1, u2 = self._fit(L1, t1, prev[0]), self._fit(L2, t2, prev[1])
            if u1 is None or u2 is None:
                continue
            cost = max(abs(u1 - prev[0]), abs(u2 - prev[1])) - 1e-6 * self.prefer_primary_sign * np.sign(u1)
            if best is None or cost < best[0]:
                best = (cost, (u1, u2))
        return best[1] if best else None


def _rotm(w, deg: float) -> np.ndarray:
    w = np.asarray(w, dtype=float)
    a = math.radians(deg)
    K = np.array([[0.0, -w[2], w[1]], [w[2], 0.0, -w[0]], [-w[1], w[0], 0.0]])
    return np.eye(3) + math.sin(a) * K + (1 - math.cos(a)) * (K @ K)


def _sp1(w, p, c) -> Optional[float]:
    """Angle (deg) that turns p about the unit axis w onto c (projections); None when either is along w."""
    u = p - w * (w @ p)
    v = c - w * (w @ c)
    if np.linalg.norm(u) < 1e-12 or np.linalg.norm(v) < 1e-12:
        return None
    return math.degrees(math.atan2(float(w @ np.cross(u, v)), float(u @ v)))


def _sp2(w1, w2, p, q) -> List[Tuple[float, float]]:
    """(a1, a2) deg with R(w1, a1) R(w2, a2) p = q (Paden-Kahan subproblem 2)."""
    d = float(w1 @ w2)
    den = d * d - 1.0
    al = (d * float(w2 @ p) - float(w1 @ q)) / den
    be = (d * float(w1 @ q) - float(w2 @ p)) / den
    n = np.cross(w1, w2)
    g2 = (float(p @ p) - al * al - be * be - 2 * al * be * d) / float(n @ n)
    if g2 < -1e-9:
        return []
    g = math.sqrt(max(g2, 0.0))
    out = []
    for s in ((1.0, -1.0) if g > 1e-12 else (1.0,)):
        c = al * w1 + be * w2 + s * g * n
        t2, t1 = _sp1(w2, p, c), _sp1(w1, c, q)
        if t1 is not None and t2 is not None:
            out.append((t1, t2))
    return out


def _par_deg(a, b) -> float:
    """Angle between two lines (deg, 0..90)."""
    c = abs(float(a @ b)) / max(np.linalg.norm(a) * np.linalg.norm(b), 1e-300)
    return math.degrees(math.acos(min(1.0, c)))


def _ang_deg(a, b) -> float:
    c = float(a @ b) / max(np.linalg.norm(a) * np.linalg.norm(b), 1e-300)
    return math.degrees(math.acos(max(-1.0, min(1.0, c))))


def solve_rotary(mc: MillMachine, pts) -> Tuple[List[Tuple[float, float]], List[float], List[bool]]:
    """Rotary angles per point. `pts`: (tip mm, axis, cutting) in the part frame. Returns (angles, tool-axis
    deviation in deg, rewind flags). Method in the comment block above MillMachine."""
    n = len(pts)
    out: List[Tuple[float, float]] = [(0.0, 0.0)] * n
    dev = [0.0] * n
    rew = [False] * n
    prev = (0.0, 0.0)
    poles = [mc.pole(a) for _, a, _ in pts]
    i = 0
    while i < n:
        if poles[i] is None:
            s = mc.choose(mc.solutions(pts[i][1]), prev)
            if s is None:
                raise GcodeError("point {}: tool axis {} is out of the rotary travel".format(
                    i, np.round(pts[i][1], 4).tolist()))
            if i > 0 and pts[i][2] and max(abs(s[0] - prev[0]), abs(s[1] - prev[1])) > mc.rewind_deg:
                rew[i] = True
            out[i], prev = s, s
            dev[i] = _ang_deg(mc.axis_of(*s), pts[i][1])
            i += 1
            continue
        h = poles[i]
        j = i
        while j < n and poles[j] == h:
            j += 1
        start = end = prev[h]
        if j < n and poles[j] is None:
            s_after = mc.choose(mc.solutions(pts[j][1]), prev)
            if s_after is not None:
                end = s_after[h]
        k0 = i - 1 if i > 0 else i                    # anchor: the point before the run
        k1 = j if j < n else n - 1                    # the first point after the run (or the last point)
        cum = {k0: 0.0}
        for k in range(k0 + 1, k1 + 1):
            cum[k] = cum[k - 1] + float(np.linalg.norm(np.asarray(pts[k][0]) - np.asarray(pts[k - 1][0])))
        total = cum[k1]
        for k in range(i, j):
            t = cum[k] / total if total > 1e-12 else (k - k0) / max(k1 - k0, 1)
            H = start + (end - start) * min(1.0, t)
            s = mc.held(pts[k][1], h, H)
            if s is None:
                s = prev
            other = 1 - h
            L = mc.axes["primary" if other == 0 else "secondary"]
            f = mc._fit(L, s[other], prev[other])
            if f is None:
                raise GcodeError("point {}: no rotary position in travel near the pole".format(k))
            s = (f, H) if other == 0 else (H, f)
            out[k], prev = s, s
            dev[k] = _ang_deg(mc.axis_of(*s), pts[k][1])
        i = j
    return out, dev, rew


def cl_points(recs: List[Rec], work_frame=None) -> List[Tuple[np.ndarray, np.ndarray, bool]]:
    """(tip mm, tool axis, cutting) of every motion record with a pose, in the work frame."""
    Wi = np.linalg.inv(np.asarray(work_frame if work_frame is not None else np.eye(4), dtype=float))
    out = []
    for r in recs:
        if r.kind in ("movel", "movej") and r.data.get("pose") is not None:
            T = Wi @ np.asarray(r.data["pose"], dtype=float)
            out.append((T[:3, 3] * 1000.0, -T[:3, 2] / np.linalg.norm(T[:3, 2]), bool(r.data.get("cut"))))
    return out


def _gc_comment(text) -> str:
    return "(" + str(text).replace("(", "[").replace(")", "]").replace("\n", " ") + ")"


def _gc_ops(ops, mc: MillMachine, warn: List[str]) -> List[str]:
    from .wiresaw_cnc import _fnum
    out = []
    for op in ops:
        k = op[0]
        if k in ("mcode", "call"):
            out.append(str(op[1]))
        elif k == "dwell":
            out.append("G4 P" + _fnum(float(op[1]) * (1000.0 if mc.dwell_in_ms else 1.0), 3))
        elif k == "message":
            out += [_gc_comment(op[1]), "M0"]
        elif k == "comment":
            out.append(_gc_comment(op[1]))
        else:
            w = "{} {} is not posted in G-code".format(k, op[1])
            warn.append(w)
            out.append(_gc_comment(w))
    return out


def _arc_fit(P: np.ndarray, tol: float, rmin: float, rmax: float):
    """Circle through P[0], the middle and the last point in a principal plane, if every point lies on it within
    tol, the sweep is monotone and below 359 deg, and each chord stays within tol of the arc. Returns
    (plane, centre, ccw, sweep rad, radius) or None. Planes: G17 (x, y) normal +z, G18 (z, x) normal +y,
    G19 (y, z) normal +x; ccw = counter-clockwise seen from the positive end of the normal (G3)."""
    for plane, (a, b, k) in (("G17", (0, 1, 2)), ("G18", (2, 0, 1)), ("G19", (1, 2, 0))):
        if np.max(np.abs(P[:, k] - P[0, k])) > 1e-6:
            continue
        U = P[:, [a, b]]
        p0, pm, pe = U[0], U[len(U) // 2], U[-1]
        dd = 2 * (p0[0] * (pm[1] - pe[1]) + pm[0] * (pe[1] - p0[1]) + pe[0] * (p0[1] - pm[1]))
        if abs(dd) < 1e-12:
            return None
        s0, sm, se = p0 @ p0, pm @ pm, pe @ pe
        c = np.array([(s0 * (pm[1] - pe[1]) + sm * (pe[1] - p0[1]) + se * (p0[1] - pm[1])) / dd,
                      (s0 * (pe[0] - pm[0]) + sm * (p0[0] - pe[0]) + se * (pm[0] - p0[0])) / dd])
        r = float(np.linalg.norm(p0 - c))
        if not rmin <= r <= rmax:
            return None
        if np.max(np.abs(np.linalg.norm(U - c, axis=1) - r)) > tol:
            return None
        th = np.arctan2(U[:, 1] - c[1], U[:, 0] - c[0])
        d = np.diff(th)
        e0 = (U[0] - c)
        e1 = (U[1] - c)
        ccw = (e0[0] * e1[1] - e0[1] * e1[0]) > 0
        d = np.mod(d, 2 * math.pi) if ccw else -np.mod(-d, 2 * math.pi)
        if np.any(np.abs(d) < 1e-9) or np.any(np.abs(d) >= math.pi):
            return None
        sweep = float(np.sum(d))
        if abs(sweep) > math.radians(359.0):
            return None
        if np.max(r * (1 - np.cos(np.abs(d) / 2))) > tol:
            return None
        c3 = P[0].astype(float).copy()
        c3[a], c3[b] = c[0], c[1]
        return plane, c3, bool(ccw), sweep, r
    return None


@dataclass
class _GItem:
    lines: List[str]
    state: Dict


def _gc_machine(opt: Dict) -> MillMachine:
    m = opt.get("machine")
    if m is None:
        return MillMachine()
    if isinstance(m, MillMachine):
        m.validate()
        return m
    if isinstance(m, dict):
        return MillMachine.from_dict(m)
    return MillMachine.load(m)


def _gc_io(recs: List[Rec], opt: Dict, mc: MillMachine, warn: List[str]):
    """The I/O for G-code: the `io` option, else the machine's own io, else an io_map record for gcode."""
    io = cell_io.coerce(opt.get("io"))
    if io is None and mc.io is not None:
        io = cell_io.from_dict(mc.io, "machine " + mc.name)
    if io is None:
        m = next((r for r in recs if r.kind == "io_map"), None)
        if m is not None and (m.data.get("io") or {}).get("controller") == "gcode":
            io = cell_io.from_dict(m.data["io"], str(m.data.get("path", "")))
    if io is not None and io.controller != "gcode":
        warn.append("cell I/O '{}' is for {} programs, not gcode: process records post as comments".format(
            io.name, io.controller))
        io = None
    if io is None:
        warn.append("no cell I/O for G-code: spindle, water and tool change post as comments")
    return io


def _gc_build(recs: List[Rec], mc: MillMachine, W, io, tool_length: Optional[float], warn: List[str]):
    """Body items of a G-code program (no header or footer)."""
    from .wiresaw_cnc import _fnum, _num
    dec, rdec = mc.decimals, mc.rotary_decimals
    Lx, Ly, Lz = mc.axes["x"], mc.axes["y"], mc.axes["z"]
    L1, L2 = mc.axes.get("primary"), mc.axes.get("secondary")
    if mc.kinematics == "head" and not mc.tcp and tool_length is None:
        raise GcodeError("a head machine without TCP needs the tool length (option tool_length, mm)")
    tl = float(tool_length or 0.0)
    Wi = np.linalg.inv(np.asarray(W, dtype=float))
    # tips and axes of every pose record, then the rotary solution over all of them
    pidx, pts, note = {}, [], ""
    for k, r in enumerate(recs):
        if r.kind == "comment":
            note = str(r.data.get("text", ""))
        if r.kind in ("movel", "movej") and r.data.get("pose") is not None:
            T = Wi @ np.asarray(r.data["pose"], dtype=float)
            tip, ax = T[:3, 3] * 1000.0, -T[:3, 2] / np.linalg.norm(T[:3, 2])
            if not mc.five:
                tilt = _ang_deg(ax, np.array([0.0, 0.0, 1.0]))
                if tilt > mc.vertical_tol_deg:
                    raise GcodeError("record {} (after '{}'): the tool axis is {:.3f} deg from vertical; a 3-axis "
                                     "machine posts only steps with a vertical tool. Post this step on a 5-axis "
                                     "machine definition.".format(k, note[:60], tilt))
            pidx[k] = len(pts)
            pts.append((tip, ax, bool(r.data.get("cut"))))
    if mc.five:
        ang, dev, rew = solve_rotary(mc, pts)
        if dev and max(dev) > 1e-6:
            warn.append("tool axis held near the pole: largest deviation {:.4f} deg (pole_deg {:g})".format(
                max(dev), mc.pole_deg))
    else:
        ang, dev, rew = [(0.0, 0.0)] * len(pts), [0.0] * len(pts), [False] * len(pts)

    proc = cell_io.Process(io)
    items: List[_GItem] = []
    st = {"last": {}, "tip": None, "ang": (0.0, 0.0), "cut": False, "feed": None, "rpm": 0.0, "pocket": None,
          "tool": "", "comp": None, "on": dict(proc.on)}
    feed_default = [None]
    need_approach = [True]

    def snap():
        s = dict(st)
        s["last"] = dict(st["last"])
        s["on"] = dict(proc.on)
        return s

    def add(lines):
        if lines:
            items.append(_GItem(list(lines), snap()))

    def ra(a):
        return round(float(a), rdec)

    def words(X, a, force_all=False):
        vals = [(Lx, _num(X[0], dec)), (Ly, _num(X[1], dec)), (Lz, _num(X[2], dec))]
        if mc.five:
            vals += [(L1, _num(a[0], rdec)), (L2, _num(a[1], rdec))]
        w = [L + s for L, s in vals if st["last"].get(L) != s]
        if not w or force_all:
            w = [L + s for L, s in vals]
        for L, s in vals:
            st["last"][L] = s
        return w

    def fword(feed, dist, drot):
        if mc.feed_mode == "G94":
            if st["feed"] != feed:
                st["feed"] = feed
                return ["F" + _fnum(feed, 3)]
            return []
        st["feed"] = feed
        if dist > 1e-9:
            t = dist / feed
        elif drot > 1e-9:
            if not mc.rotary_feed_deg_min:
                raise GcodeError("G93 with a rotary-only move needs rotary_feed_deg_min in the machine definition")
            t = drot / float(mc.rotary_feed_deg_min)
        else:
            t = 1e-6
        return ["F" + _fnum(1.0 / t, 5)]

    def approach(tip, a, why):
        a = (ra(a[0]), ra(a[1]))
        X = mc.to_machine(tip, a[0], a[1], tl)
        lines = [_gc_comment(why), "G0 {}{}".format(Lz, _num(mc.safe_z, dec))]
        st["last"][Lz] = _num(mc.safe_z, dec)
        if mc.five:
            lines.append("G0 {}{} {}{}".format(L1, _num(a[0], rdec), L2, _num(a[1], rdec)))
            st["last"][L1], st["last"][L2] = _num(a[0], rdec), _num(a[1], rdec)
        lines.append("G0 {}{} {}{}".format(Lx, _num(X[0], dec), Ly, _num(X[1], dec)))
        st["last"][Lx], st["last"][Ly] = _num(X[0], dec), _num(X[1], dec)
        st["cut"] = False
        return lines

    def move(rec, k):
        i = pidx[k]
        tip, ax, cut = pts[i]
        a = (ra(ang[i][0]), ra(ang[i][1]))
        lines = []
        if need_approach[0]:
            lines += approach(tip, a, "approach")
            need_approach[0] = False
        elif rew[i]:
            warn.append("rotary rewind at record {}: retract, turn, re-enter".format(k))
            lines += approach(tip, a, "rewind: rotary axis out of travel, retract and turn")
        if not cut:
            X = mc.to_machine(tip, a[0], a[1], tl)
            lines.append(" ".join(["G0"] + words(X, a)))
            st["cut"] = False
        else:
            v = rec.data.get("v")
            feed = float(v) * 60.0 if v else feed_default[0]
            if not feed or feed <= 0:
                raise GcodeError("record {}: a cutting move without a feed".format(k))
            prev_tip, prev_a = st["tip"], st["ang"]
            subs = [(tip, a)]
            if mc.five and not mc.tcp and mc.max_rotary_step_deg and prev_tip is not None:
                step = max(abs(a[0] - prev_a[0]), abs(a[1] - prev_a[1]))
                nsub = int(math.ceil(step / mc.max_rotary_step_deg)) if step > mc.max_rotary_step_deg else 1
                if nsub > 1:   # tip on the straight line, angles interpolated: the machine follows the line
                    subs = [(prev_tip + (tip - prev_tip) * (s / nsub),
                             (ra(prev_a[0] + (a[0] - prev_a[0]) * s / nsub),
                              ra(prev_a[1] + (a[1] - prev_a[1]) * s / nsub))) for s in range(1, nsub + 1)]
            p_tip, p_a = prev_tip, prev_a
            for s_tip, s_a in subs:
                X = mc.to_machine(s_tip, s_a[0], s_a[1], tl)
                dist = float(np.linalg.norm(s_tip - p_tip)) if p_tip is not None else 0.0
                drot = max(abs(s_a[0] - p_a[0]), abs(s_a[1] - p_a[1]))
                w = words(X, s_a)
                lines.append(" ".join(["G1"] + w + fword(feed, dist, drot)))
                p_tip, p_a = s_tip, s_a
            st["cut"] = True
        st["tip"], st["ang"] = tip, a
        return lines

    def arc_run(k):
        """Fit an arc from the current tip through the following cutting records (same feed, nothing between)."""
        if not mc.arcs or st["tip"] is None or need_approach[0] or not st["cut"]:
            return None
        v = recs[k].data.get("v")
        run = []
        j = k
        while j < len(recs) and len(run) < 500:
            r = recs[j]
            if r.kind != "movel" or not r.data.get("cut") or j not in pidx or r.data.get("v") != v:
                break
            run.append(j)
            j += 1
        if len(run) < 2:
            return None
        P = np.array([st["tip"]] + [pts[pidx[j]][0] for j in run])
        best = None
        e = 2
        while e < len(P):
            f = _arc_fit(P[:e + 1], mc.arc_tol, mc.arc_min_radius, mc.arc_max_radius)
            if f is None:
                break
            best = (e, f)
            e += 1
        if best is None:
            return None
        e, (plane, c, ccw, sweep, rad) = best
        feed = float(v) * 60.0 if v else feed_default[0]
        if not feed or feed <= 0:
            raise GcodeError("record {}: a cutting move without a feed".format(k))
        start = np.array([float(st["last"][Lx]), float(st["last"][Ly]), float(st["last"][Lz])])
        end = P[e]
        off = c - start
        ow = {"G17": [("I", 0), ("J", 1)], "G18": [("I", 0), ("K", 2)], "G19": [("J", 1), ("K", 2)]}[plane]
        w = words(end, (0.0, 0.0))
        line = " ".join([plane, "G3" if ccw else "G2"] + w + [L + _num(off[ix], dec) for L, ix in ow] +
                        fword(feed, abs(sweep) * rad, 0.0))
        st["tip"] = end
        st["cut"] = True
        return run[e - 1], [line]

    k = 0
    while k < len(recs):
        r = recs[k]
        d = r.data
        if r.kind == "start":
            add([_gc_comment("program {}".format(d.get("name", "lithocam")))])
        elif r.kind == "end":
            lines = _gc_ops(_end_ops(proc, {"warnings": warn}), mc, warn)
            lines.append("G0 {}{}".format(Lz, _num(mc.safe_z, dec)))
            if st["comp"] and mc.comp_cancel:
                lines.append(mc.comp_cancel)
            add(lines)
        elif r.kind in ("comment", "tool"):
            add([_gc_comment(d.get("text", "tool {}".format(d.get("id", ""))))])
        elif r.kind in PROCESS_KINDS:
            if r.kind == "spindle":
                st["rpm"] = float(d.get("rpm") or 0.0)
            if r.kind == "spindle" and d.get("reverse") and d.get("on", True):
                # tapping: M4 runs the spindle counter-clockwise (reverse); M3 again for the next forward run
                lines = ["M4 S{:g}".format(st["rpm"])]
                proc.on["spindle"] = True
                st["rev"] = True
            elif r.kind == "spindle" and st.get("rev") and d.get("on", True) and (d.get("rpm") or 0) > 0:
                lines = ["M3 S{:g}".format(st["rpm"])]
                proc.on["spindle"] = True
                st["rev"] = False
            else:
                if r.kind == "spindle":
                    st["rev"] = False
                ops = proc.ops(r.kind, d)
                lines = _gc_ops(ops, mc, warn)
            if r.kind == "tool_change":
                tid = str(d.get("id", ""))
                st["tool"] = tid
                pk = io.pocket(tid) if io is not None else None
                st["pocket"] = pk
                tmpl = mc.tcp_on if mc.tcp else mc.length_comp
                if tmpl:
                    if "{pocket}" in tmpl and pk is None:
                        w = "no tool length compensation for {}: no pocket number in the cell I/O".format(tid)
                        warn.append(w)
                        lines.append(_gc_comment(w))
                    else:
                        lines.append(tmpl.format(pocket=pk, tool=tid))
                        st["comp"] = lines[-1]
                need_approach[0] = True
            add(lines)
        elif r.kind == "dwell":
            add(_gc_ops([("dwell", float(d.get("seconds", 0.0)))], mc, warn))
        elif r.kind == "pause":
            s = float(d.get("seconds", -1))
            add(_gc_ops([("dwell", s)] if s >= 0 else [("message", "paused: continue when ready")], mc, warn))
        elif r.kind == "speed":
            if d.get("linear"):
                feed_default[0] = float(d["linear"]) * 60.0
        elif r.kind in ("movel", "movej") and k in pidx:
            fit = arc_run(k) if r.kind == "movel" and d.get("cut") else None
            if fit is not None:
                k_last, lines = fit
                add(lines)
                k = k_last + 1
                continue
            add(move(r, k))
        elif r.kind in ("movel", "movej"):
            need_approach[0] = True       # a robot joint move: the machine retracts and approaches the next point
        elif r.kind == "io_map":
            pass
        else:
            w = "{} record not posted in G-code".format(r.kind)
            warn.append(w)
            add([_gc_comment(w)])
        k += 1
    return items


def _gc_header(mc: MillMachine, io, name: str, k: int, n: int, number: Optional[int]) -> List[str]:
    h = []
    if number is not None:
        h.append("O{:04d}".format(number))
    h.append(_gc_comment("lithocam {} file {}/{}".format(name, k, n)))
    h.append(_gc_comment("machine {} kinematics {} tcp {}".format(mc.name, mc.kinematics, "on" if mc.tcp else "off")))
    if mc.placeholder:
        h.append(_gc_comment("WARNING: PLACEHOLDER machine definition, set every value for your machine"))
    if io is not None and io.placeholder:
        h.append(_gc_comment("WARNING: PLACEHOLDER I/O values, set for your machine"))
    h += list(mc.header)
    h.append("G21 G90 " + mc.feed_mode)
    return h


def _gc_resume(mc: MillMachine, io, s: Dict, warn: List[str]) -> List[str]:
    """Start of a continuation file: tool length, drives and water back on, approach to the last position."""
    from .wiresaw_cnc import _fnum, _num
    out = []
    if s.get("comp"):
        out.append(s["comp"])
    if io is not None:
        for drive in ("spindle", "wire"):
            if s["on"].get(drive):
                out += _gc_ops(io.drive_on(drive, s["rpm"] if drive == "spindle" else 0.0, False, s["tool"]), mc,
                               warn)
        for sw in ("water", "air"):
            if s["on"].get(sw):
                out += _gc_ops(io.switch(sw, True), mc, warn)
    elif any(s["on"].values()):
        out.append(_gc_comment("drives were on: no cell I/O to restart them"))
    last = s["last"]
    Lx, Ly, Lz = mc.axes["x"], mc.axes["y"], mc.axes["z"]
    if all(L in last for L in (Lx, Ly, Lz)):
        out.append("G0 {}{}".format(Lz, _num(mc.safe_z, mc.decimals)))
        if mc.five:
            L1, L2 = mc.axes["primary"], mc.axes["secondary"]
            out.append("G0 {}{} {}{}".format(L1, last.get(L1, "0"), L2, last.get(L2, "0")))
        out.append("G0 {}{} {}{}".format(Lx, last[Lx], Ly, last[Ly]))
        if s["cut"] and s["feed"]:
            pre = ["G94"] if mc.feed_mode == "G93" else []
            out += pre + ["G1 {}{} F{}".format(Lz, last[Lz], _fnum(s["feed"], 3))]
            if mc.feed_mode == "G93":
                out.append("G93")
        else:
            out.append("G0 {}{}".format(Lz, last[Lz]))
    return out


def _gc_stop(mc: MillMachine, io, s: Dict, warn: List[str]) -> List[str]:
    out = []
    for k in ("spindle", "wire"):
        if s["on"].get(k):
            out += _gc_ops(io.drive_off(k) if io is not None else cell_io.no_io_ops(k, {"on": False}), mc, warn)
    for k in ("water", "air"):
        if s["on"].get(k):
            out += _gc_ops(io.switch(k, False) if io is not None else cell_io.no_io_ops(k, {"on": False}), mc, warn)
    out.append("G0 {}{}".format(mc.axes["z"], "{:.{}f}".format(mc.safe_z, mc.decimals)))
    if s.get("comp") and mc.comp_cancel:
        out.append(mc.comp_cancel)
    return out


def write_gcode(recs: List[Rec], machine=None, work_frame=None, io=None, tool_length: Optional[float] = None,
                warnings: Optional[List[str]] = None, name: Optional[str] = None) -> List[str]:
    """ISO G-code of a record stream for a milling machine, one text per file. With `max_lines` the program is split:
    "subprograms" gives a main program (M98 P calls) followed by the subprograms; "files" gives standalone files,
    each continuation restarting the drives and approaching the last position from safe_z.
    `machine`: MillMachine, dict or JSON path (default: generic 3-axis). `work_frame`: 4x4 work frame in the robot
    base (m) that the record poses are expressed against. `io`: cell I/O for the M-codes (default: the machine's io).
    `tool_length`: mm, for a head machine without TCP."""
    from .wiresaw_cnc import _numbered
    warn = warnings if warnings is not None else []
    mc = _gc_machine({"machine": machine})
    recs = list(recs)
    if not recs or recs[0].kind != "start":
        recs.insert(0, Rec("start", {"name": "lithocam"}))
    if recs[-1].kind != "end":
        recs.append(Rec("end"))
    name = name or str(recs[0].data.get("name", "lithocam"))
    io_ = _gc_io(recs, {"io": io}, mc, warn)
    W = np.asarray(work_frame if work_frame is not None else np.eye(4), dtype=float)
    items = _gc_build(recs, mc, W, io_, tool_length, warn)
    if mc.max_lines is None:
        lines = _gc_header(mc, io_, name, 1, 1, mc.program_number)
        for it in items:
            lines += it.lines
        lines += mc.footer
        return ["\n".join(_numbered(lines, mc.line_numbers)) + "\n"]
    # split
    parts: List[List[_GItem]] = []
    if mc.split == "subprograms":
        cap = mc.max_lines - 3
    else:
        worst = len(_gc_header(mc, io_, name, 99, 99, mc.program_number)) + len(mc.footer)
        cap = mc.max_lines - worst - 24       # room for the resume and stop blocks
    if cap < 4:
        raise GcodeError("max_lines {} is too small for this machine's header and footer".format(mc.max_lines))
    cur: List[_GItem] = []
    used = 0
    for it in items:
        if len(it.lines) > cap:
            raise GcodeError("one block group has {} lines, more than the split allows".format(len(it.lines)))
        if cur and used + len(it.lines) > cap:
            cut = len(cur) - 1
            if mc.split == "files":   # prefer a split where the tool is out of the cut
                for jj in range(len(cur) - 1, max(-1, len(cur) - 1 - 200), -1):
                    if not cur[jj].state["cut"]:
                        cut = jj
                        break
            parts.append(cur[:cut + 1])
            cur = cur[cut + 1:]
            used = sum(len(x.lines) for x in cur)
        cur.append(it)
        used += len(it.lines)
    if cur:
        parts.append(cur)
    texts = []
    if mc.split == "subprograms":
        main = _gc_header(mc, io_, name, 1, len(parts) + 1, mc.program_number)
        main += ["M98 P{}".format(mc.sub_number + k) for k in range(1, len(parts) + 1)] + list(mc.footer)
        if len(main) > mc.max_lines:
            raise GcodeError("the main program needs {} lines for {} subprograms, more than max_lines".format(
                len(main), len(parts)))
        texts.append("\n".join(_numbered(main, mc.line_numbers)) + "\n")
        for k, part in enumerate(parts, start=1):
            lines = ["O{}".format(mc.sub_number + k), _gc_comment("{} part {}/{}".format(name, k, len(parts)))]
            for it in part:
                lines += it.lines
            lines.append("M99")
            texts.append("\n".join(_numbered(lines, mc.line_numbers)) + "\n")
        return texts
    for k, part in enumerate(parts, start=1):
        num = None if mc.program_number is None else mc.program_number + k - 1
        lines = _gc_header(mc, io_, name, k, len(parts), num)
        if k > 1:
            lines += [_gc_comment("continue from file {}".format(k - 1))] + _gc_resume(mc, io_, parts[k - 2][-1].state,
                                                                                      warn)
        for it in part:
            lines += it.lines
        if k < len(parts):
            lines += _gc_stop(mc, io_, part[-1].state, warn)
        lines += list(mc.footer)
        if len(lines) > mc.max_lines:
            raise GcodeError("file {} has {} lines, more than max_lines {}".format(k, len(lines), mc.max_lines))
        texts.append("\n".join(_numbered(lines, mc.line_numbers)) + "\n")
    return texts


def _gcode(recs: List[Rec], opt: Dict) -> str:
    texts = write_gcode(recs, opt.get("machine"), opt.get("work_frame"), opt.get("io"), opt.get("tool_length"),
                        opt.get("warnings"))
    return "".join(texts)


# ---------------------------------------------------------------- G-code read-back
@dataclass
class GPoint:
    kind: str                     # "rapid" | "line" | "arc"
    tip: np.ndarray               # part frame, mm
    axis: np.ndarray              # tool axis, tip to holder
    angles: Tuple[float, float]
    file: int
    block: int
    arc: Optional[Tuple] = None   # (centre, normal, start, ccw) for G2/G3


def gcode_points(texts, machine=None, tool_length: Optional[float] = None) -> List[GPoint]:
    """Tip and tool axis after every motion block, by forward kinematics of the programmed axes. `texts`: one
    program or a list of files in run order (a subprogram split: main first, then the subprograms)."""
    from .wiresaw_cnc import parse_gcode
    mc = _gc_machine({"machine": machine})
    if isinstance(texts, str):
        texts = [texts]
    L = mc.axes
    pos: Dict[str, float] = {}
    mode, plane = None, 17
    out: List[GPoint] = []
    tl = float(tool_length or 0.0)
    normals = {17: np.array([0.0, 0.0, 1.0]), 18: np.array([0.0, 1.0, 0.0]), 19: np.array([1.0, 0.0, 0.0])}
    for fi, text in enumerate(texts):
        for b in parse_gcode(text):
            for g in b.g:
                if g in (0, 1, 2, 3):
                    mode = int(g)
                elif g in (17, 18, 19):
                    plane = int(g)
            letters = set(L.values())
            if not any(k in letters for k in b.words) or mode is None:
                continue
            start = dict(pos)
            for k, v in b.words.items():
                if k in letters:
                    pos[k] = v
            if not all(L[a] in pos for a in ("x", "y", "z")):
                continue
            a1 = pos.get(L.get("primary", ""), 0.0) if mc.five else 0.0
            a2 = pos.get(L.get("secondary", ""), 0.0) if mc.five else 0.0
            X = np.array([pos[L["x"]], pos[L["y"]], pos[L["z"]]])
            tip = mc.from_machine(X, a1, a2, tl)
            arc = None
            if mode in (2, 3):
                s = np.array([start.get(L["x"], 0.0), start.get(L["y"], 0.0), start.get(L["z"], 0.0)])
                c = s + np.array([b.words.get("I", 0.0), b.words.get("J", 0.0), b.words.get("K", 0.0)])
                n = normals[plane]
                c = c - n * (n @ (c - s))      # the offset normal to the plane is zero
                arc = (c, n, s, mode == 3)
            out.append(GPoint({0: "rapid", 1: "line"}.get(mode, "arc"), tip, mc.axis_of(a1, a2), (a1, a2), fi,
                              b.index, arc))
    return out


def _on_arc(g: GPoint, p, tol: float) -> Optional[float]:
    c, n, s, ccw = g.arc
    p = np.asarray(p, dtype=float)
    if abs(n @ (p - s)) > tol:
        return None
    r = np.linalg.norm(s - c)
    v = (p - c) - n * (n @ (p - c))
    err = abs(np.linalg.norm(v) - r)
    if err > tol:
        return None

    def ang(x):   # angle from the start, along the arc direction, 0..2pi
        a, b = s - c, x - c
        t = math.atan2(float(n @ np.cross(a, b)), float(a @ b))
        t = t if ccw else -t
        return t % (2 * math.pi)
    return err if ang(p) <= ang(g.tip) + 1e-9 else None


@dataclass
class GcodeCheck:
    n: int
    matched: int
    max_tip_err: float            # mm
    max_axis_err: float           # deg
    skipped_blocks: int           # motion blocks between matches (approach, rewind, linearisation)
    tip_errors: List[float] = field(default_factory=list)
    axis_errors: List[float] = field(default_factory=list)

    @property
    def ok(self) -> bool:
        return self.matched == self.n


def compare_gcode(recs: List[Rec], texts, machine=None, work_frame=None, tool_length: Optional[float] = None,
                  tol_mm: Optional[float] = None, tol_deg: Optional[float] = None, max_skip: int = 64) -> GcodeCheck:
    """Static verification gate for G-code: every pose record must reappear, in order, as a motion block end (or a
    point on an arc block) within tol_mm and tol_deg. Extra blocks between (approach, rewind, linearisation) are
    allowed. Defaults: tol_mm = 2 units of the last decimal; tol_deg = pole_deg + 1e-3 (5-axis) or
    vertical_tol_deg + 1e-6 (3-axis)."""
    mc = _gc_machine({"machine": machine})
    tol_mm = tol_mm if tol_mm is not None else 2.0 * 10.0 ** (-mc.decimals)
    tol_deg = tol_deg if tol_deg is not None else ((mc.pole_deg + 1e-3) if mc.five else mc.vertical_tol_deg + 1e-6)
    exp = cl_points(recs, work_frame)
    got = gcode_points(texts, mc, tool_length)
    j = 0
    chk = GcodeCheck(len(exp), 0, 0.0, 0.0, 0)
    for tip, ax, _ in exp:
        jj = j
        while jj < len(got) and jj - j <= max_skip:
            g = got[jj]
            e, a = float(np.linalg.norm(g.tip - tip)), _ang_deg(g.axis, ax)
            if e <= tol_mm and a <= tol_deg:
                chk.matched += 1
                chk.skipped_blocks += jj - j
                chk.tip_errors.append(e)
                chk.axis_errors.append(a)
                j = jj + 1
                break
            if g.arc is not None:
                ea = _on_arc(g, tip, tol_mm + mc.arc_tol)
                if ea is not None:
                    chk.matched += 1
                    chk.skipped_blocks += jj - j
                    chk.tip_errors.append(ea)
                    chk.axis_errors.append(a)
                    j = jj
                    break
            jj += 1
    chk.max_tip_err = max(chk.tip_errors, default=0.0)
    chk.max_axis_err = max(chk.axis_errors, default=0.0)
    return chk


WRITERS: Dict[str, Callable[[List[Rec], Dict], str]] = {"urscript": _urscript, "krl": _krl, "rapid": _rapid,
                                                         "cl": _cl, "gcode": _gcode}
EXTENSIONS = {"urscript": ".script", "krl": ".src", "rapid": ".mod", "cl": ".cl", "gcode": ".nc"}


def write(recs: List[Rec], dialect: str, **options) -> str:
    """Program text for one controller dialect. Options: blend (m for URScript, mm for KRL/RAPID zones),
    joint_speed_max (rad/s), joint_accel, linear_accel, gripper_call (format string with {width} and {force}),
    io (cell I/O: lithocam.cell_io.CellIO, dict or path; default: the io_map record of the expansion), warnings (a
    list the writer appends to). G-code: machine, work_frame, tool_length (see write_gcode); a split program comes
    back as the files joined; use write_files for one text per file."""
    if dialect not in WRITERS:
        raise ValueError("unknown dialect '{}'; known: {}".format(dialect, ", ".join(WRITERS)))
    recs = list(recs)
    if not recs or recs[0].kind != "start":
        recs.insert(0, Rec("start", {"name": "lithocam"}))
    if recs[-1].kind != "end":
        recs.append(Rec("end"))
    return WRITERS[dialect](recs, options)


def write_files(recs: List[Rec], dialect: str, **options) -> List[str]:
    """One text per program file: the G-code split, or a single text for the robot dialects."""
    if dialect == "gcode":
        return write_gcode(recs, options.get("machine"), options.get("work_frame"), options.get("io"),
                           options.get("tool_length"), options.get("warnings"))
    return [write(recs, dialect, **options)]


def joint_targets(text: str, dialect: str) -> List[List[float]]:
    """Joint targets (radians) read back from a posted program, for the static verification gate."""
    import re
    out = []
    if dialect == "urscript":   # movej/movel joint targets, and the verified joints noted on each movep
        for m in re.finditer(r"move[jl]\(\[([^\]]*)\]|movep\(p\[[^\]]*\][^#\n]*# q \[([^\]]*)\]", text):
            out.append([float(v) for v in (m.group(1) or m.group(2)).split(",")])
    elif dialect == "krl":   # PTP axis targets (E1 left out), and the verified joints noted on each LIN
        for m in re.finditer(r"PTP \{(?:E6)?AXIS: ([^}]*)\}|LIN \{[^}]*\}[^;\n]*; q \[([^\]]*)\]", text):
            if m.group(1):
                out.append([math.radians(float(p.split()[1])) for p in m.group(1).split(",")
                            if p.split()[0].startswith("A")])
            else:
                out.append([float(v) for v in m.group(2).split(",")])
    elif dialect == "rapid":
        for m in re.finditer(r"MoveAbsJ \[\[([^\]]*)\]|MoveL .*! q \[([^\]]*)\]", text):
            if m.group(1):
                out.append([math.radians(float(v)) for v in m.group(1).split(",")])
            else:
                out.append([float(v) for v in m.group(2).split(",")])
    elif dialect == "cl":
        out = [list(r.data["q"]) for r in parse_cl(text) if r.kind in ("movej", "movel")]
    return out


def external_axes(text: str, dialect: str) -> List[Optional[float]]:
    """E1 (degrees) of every motion read back from a posted program, in the order joint_targets reads the motions;
    None where a motion has no E1 (KRL without E1, RAPID 9E9). URScript has no external axis: all None."""
    import re
    out: List[Optional[float]] = []
    if dialect == "urscript":
        return [None] * len(joint_targets(text, dialect))
    if dialect == "krl":
        for m in re.finditer(r"PTP \{(?:E6)?AXIS: ([^}]*)\}|LIN \{([^}]*)\}[^;\n]*; q \[[^\]]*\]", text):
            body = m.group(1) if m.group(1) is not None else m.group(2)
            e = [float(p.split()[1]) for p in body.split(",") if p.split() and p.split()[0] == "E1"]
            out.append(e[0] if e else None)
    elif dialect == "rapid":
        pat = (r"MoveAbsJ \[\[[^\]]*\], \[([^\]]*)\]\]|"
               r"MoveL \[\[[^\]]*\], \[[^\]]*\], \[[^\]]*\], \[([^\]]*)\]\].*! q \[[^\]]*\]")
        for m in re.finditer(pat, text):
            v = (m.group(1) if m.group(1) is not None else m.group(2)).split(",")[0].strip()
            out.append(None if v.upper() == "9E9" else float(v))
    elif dialect == "cl":
        out = [None if r.data.get("e1") is None else float(r.data["e1"]) for r in parse_cl(text)
               if r.kind in ("movej", "movel")]
    return out
