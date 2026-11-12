# Copyright 2026 The lithocam authors. SPDX-License-Identifier: Apache-2.0
"""Robot program export: URScript (Universal Robots) and KRL (KUKA) from joint paths.

Both exporters write joint-space moves (movej / PTP AXIS), so what the controller executes is exactly the
configuration lithocam verified (no controller-side IK branch choice). Units: radians in, the controller's units
out (URScript radians, KRL degrees). Pure Python; no robot connection.

These programs are for simulation (URSim, KUKA.OfficeLite) until a hardware gate is passed.
"""
from __future__ import annotations

import math
from typing import Iterable, Optional, Sequence


def urscript(joint_path: Iterable[Sequence[float]], name: str = "lithocam", speed: float = 0.5, accel: float = 0.8,
             blend: float = 0.0, home: Optional[Sequence[float]] = None, digital_out: Optional[int] = None) -> str:
    """URScript program: optional home move, then movej through every configuration.

    speed rad/s, accel rad/s^2, blend radius metres (0 = stop at each point).
    digital_out: switch this standard output on during the path (for example the wire drive), off at the end.
    """
    lines = ["def {}():".format(name)]
    fmt = lambda q: "[" + ", ".join("{:.6f}".format(v) for v in q) + "]"   # noqa: E731
    if home is not None:
        lines.append("  movej({}, a={}, v={})".format(fmt(home), accel, speed))
    if digital_out is not None:
        lines.append("  set_standard_digital_out({}, True)".format(digital_out))
    qs = list(joint_path)
    for i, q in enumerate(qs):
        r = blend if 0 < i < len(qs) - 1 else 0.0
        lines.append("  movej({}, a={}, v={}, r={})".format(fmt(q), accel, speed, r))
    if digital_out is not None:
        lines.append("  set_standard_digital_out({}, False)".format(digital_out))
    if home is not None:
        lines.append("  movej({}, a={}, v={})".format(fmt(home), accel, speed))
    lines.append("end")
    lines.append("{}()".format(name))
    return "\n".join(lines) + "\n"


def krl(joint_path: Iterable[Sequence[float]], name: str = "lithocam", vel_percent: int = 20,
        home: Optional[Sequence[float]] = None, output: Optional[int] = None) -> str:
    """KRL .src program with PTP {AXIS: A1 .. A6} moves in degrees. Pair with a .dat if your controller needs one."""
    deg = lambda q: ", ".join("A{} {:.4f}".format(i + 1, math.degrees(v)) for i, v in enumerate(q))  # noqa: E731
    lines = ["&ACCESS RVP", "DEF {}()".format(name), "  ;FOLD INI", "  BAS(#INITMOV, 0)", "  ;ENDFOLD",
             "  $VEL_AXIS[1] = {0}".format(vel_percent)]
    for i in range(2, 7):
        lines.append("  $VEL_AXIS[{}] = {}".format(i, vel_percent))
    if home is not None:
        lines.append("  PTP {{AXIS: {}}}".format(deg(home)))
    if output is not None:
        lines.append("  $OUT[{}] = TRUE".format(output))
    for q in joint_path:
        lines.append("  PTP {{AXIS: {}}} C_PTP".format(deg(q)))
    if output is not None:
        lines.append("  $OUT[{}] = FALSE".format(output))
    if home is not None:
        lines.append("  PTP {{AXIS: {}}}".format(deg(home)))
    lines.append("END")
    return "\n".join(lines) + "\n"
