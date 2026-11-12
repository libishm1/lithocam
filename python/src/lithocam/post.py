# Copyright 2026 The lithocam authors. SPDX-License-Identifier: Apache-2.0
"""Post-processing: a neutral record stream and one writer per controller dialect.

Layer 1 (records) holds what to do, never how a controller spells it. Layer 4 (writers) only formats; it does no
geometry. Joint values come from lithocam's own kinematics (verified IK), so every writer can post joint targets and
the controller runs the configuration lithocam checked. Design notes: docs/POSTPROCESSORS.md.

Record kinds (`Rec.kind`, data keys):
  start(name) end() comment(text) tool(id, tcp: 4x4 in metres, mass kg) speed(linear mm/s, joint 0..1)
  movej(q rad) movel(q rad, pose 4x4 m in the robot base frame) pause(seconds; -1 = wait for the operator)
  io_set(port, value) io_wait(port, value) spindle(rpm) gripper(width mm, force N) call(name)

Dialects: "urscript" (Universal Robots), "krl" (KUKA KRC), "rapid" (ABB IRC5), "cl" (neutral text, one record per
line; also the round-trip format for tests).

Robot programs from these writers are for simulation (URSim, KUKA.OfficeLite, RobotStudio) until a hardware gate is
passed. Joint-direction conventions of each cell URDF must match its controller; check every new cell in its
simulator first.
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Sequence

import numpy as np


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


# ---------------------------------------------------------------- writers
def _urscript(recs: List[Rec], opt: Dict) -> str:
    name = "lithocam"
    out: List[str] = []
    acc_j, acc_l = opt.get("joint_accel", 1.0), opt.get("linear_accel", 0.5)
    vj = opt.get("joint_speed_max", 1.0) * 0.5      # rad/s
    vl = 0.1                                        # m/s
    blend = opt.get("blend", 0.0)
    fmt = lambda q: "[" + ", ".join("{:.6f}".format(float(v)) for v in q) + "]"   # noqa: E731
    for r in recs:
        d = r.data
        if r.kind == "start":
            name = str(d.get("name", name))
            out.append("def {}():".format(name))
        elif r.kind == "end":
            out += ["end", "{}()".format(name)]
        elif r.kind == "comment":
            out.append("  # " + str(d.get("text", "")).replace("\n", " "))
        elif r.kind == "tool":
            T = np.asarray(d["tcp"], dtype=float)
            p = "p[{:.6f}, {:.6f}, {:.6f}, {:.6f}, {:.6f}, {:.6f}]".format(*T[:3, 3], *rotvec(T[:3, :3]))
            out.append("  set_tcp({})  # tool {}".format(p, d.get("id", "")))
            if d.get("mass"):
                out.append("  set_payload({:.3f})".format(float(d["mass"])))
        elif r.kind == "speed":
            if d.get("linear"):
                vl = float(d["linear"]) / 1000
            if d.get("joint"):
                vj = opt.get("joint_speed_max", 1.0) * float(d["joint"])
        elif r.kind == "movej":
            out.append("  movej({}, a={}, v={:.4f}, r={})".format(fmt(d["q"]), acc_j, vj, d.get("blend", blend)))
        elif r.kind == "movel":   # movel with joint targets: linear in tool space, ends in the verified configuration
            out.append("  movel({}, a={}, v={:.4f}, r={})".format(fmt(d["q"]), acc_l, vl, d.get("blend", blend)))
        elif r.kind == "pause":
            s = float(d.get("seconds", -1))
            out.append("  sleep({:.3f})".format(s) if s >= 0 else '  popup("lithocam: paused, press Continue", '
                                                                       'blocking=True)')
        elif r.kind == "io_set":
            out.append("  set_standard_digital_out({}, {})".format(int(d["port"]), bool(d.get("value", True))))
        elif r.kind == "io_wait":
            neg = "not " if d.get("value", True) else ""   # wait while the input differs from the wanted value
            out += ["  while {}get_standard_digital_in({}):".format(neg, int(d["port"])), "    sync()", "  end"]
        elif r.kind == "spindle":
            out.append("  # spindle {} rev/min: map to the cell's spindle I/O".format(d.get("rpm", 0)))
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
    for r in recs:
        d = r.data
        if r.kind == "start":
            out += ["&ACCESS RVP", "DEF {}()".format(d.get("name", "lithocam")), "  ;FOLD INI",
                    "  BAS(#INITMOV, 0)", "  ;ENDFOLD"]
            if opt.get("blend", 0.0) > 0:
                out.append("  $APO.CPTP = {:.0f}".format(min(100.0, opt["blend"])))
        elif r.kind == "end":
            out.append("END")
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
        elif r.kind in ("movej", "movel"):   # joint targets: the verified configuration
            out.append("  PTP {{AXIS: {}}}{}".format(deg(d["q"]), approx))
        elif r.kind == "pause":
            s = float(d.get("seconds", -1))
            out.append("  WAIT SEC {:.3f}".format(s) if s >= 0 else "  HALT")
        elif r.kind == "io_set":
            out.append("  $OUT[{}] = {}".format(int(d["port"]), "TRUE" if d.get("value", True) else "FALSE"))
        elif r.kind == "io_wait":
            out.append("  WAIT FOR $IN[{}] == {}".format(int(d["port"]), "TRUE" if d.get("value", True) else "FALSE"))
        elif r.kind == "spindle":
            out.append("  ; spindle {} rev/min: map to the cell's spindle I/O".format(d.get("rpm", 0)))
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
    for r in recs:
        d = r.data
        if r.kind == "start":
            name = str(d.get("name", name))
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
        elif r.kind in ("movej", "movel"):
            body.append("    MoveAbsJ [[{}], [9E9, 9E9, 9E9, 9E9, 9E9, 9E9]], {}, {}, {};".format(
                deg(d["q"]), speed, zone, tool))
        elif r.kind == "pause":
            s = float(d.get("seconds", -1))
            body.append("    WaitTime {:.3f};".format(s) if s >= 0 else "    Stop;")
        elif r.kind == "io_set":
            body.append("    SetDO DO_{}, {};".format(int(d["port"]), 1 if d.get("value", True) else 0))
        elif r.kind == "io_wait":
            body.append("    WaitDI DI_{}, {};".format(int(d["port"]), 1 if d.get("value", True) else 0))
        elif r.kind == "spindle":
            body.append("    ! spindle {} rev/min: map to the cell's spindle I/O".format(d.get("rpm", 0)))
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


WRITERS: Dict[str, Callable[[List[Rec], Dict], str]] = {"urscript": _urscript, "krl": _krl, "rapid": _rapid,
                                                         "cl": _cl}
EXTENSIONS = {"urscript": ".script", "krl": ".src", "rapid": ".mod", "cl": ".cl"}


def write(recs: List[Rec], dialect: str, **options) -> str:
    """Program text for one controller dialect. Options: blend (m for URScript, mm for KRL/RAPID zones),
    joint_speed_max (rad/s), joint_accel, linear_accel, gripper_call (format string with {width} and {force})."""
    if dialect not in WRITERS:
        raise ValueError("unknown dialect '{}'; known: {}".format(dialect, ", ".join(WRITERS)))
    recs = list(recs)
    if not recs or recs[0].kind != "start":
        recs.insert(0, Rec("start", {"name": "lithocam"}))
    if recs[-1].kind != "end":
        recs.append(Rec("end"))
    return WRITERS[dialect](recs, options)


def joint_targets(text: str, dialect: str) -> List[List[float]]:
    """Joint targets (radians) read back from a posted program, for the static verification gate."""
    import re
    out = []
    if dialect == "urscript":
        for m in re.finditer(r"move[jl]\(\[([^\]]*)\]", text):
            out.append([float(v) for v in m.group(1).split(",")])
    elif dialect == "krl":
        for m in re.finditer(r"PTP \{AXIS: ([^}]*)\}", text):
            out.append([math.radians(float(p.split()[1])) for p in m.group(1).split(",")])
    elif dialect == "rapid":
        for m in re.finditer(r"MoveAbsJ \[\[([^\]]*)\]", text):
            out.append([math.radians(float(v)) for v in m.group(1).split(",")])
    elif dialect == "cl":
        out = [list(r.data["q"]) for r in parse_cl(text) if r.kind in ("movej", "movel")]
    return out
