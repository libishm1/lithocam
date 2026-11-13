# Copyright 2026 The lithocam authors. SPDX-License-Identifier: Apache-2.0
"""Cell I/O definition: how one controller drives the process hardware of one cell.

A cell I/O file (JSON, schema "lithocam.cell_io/1") says how the controller turns the spindle on and off and sets its
speed, switches water and air, changes tools, runs a wire saw motor, and which inputs to wait for. The post writers
(lithocam.post) turn the process records (spindle, water, air, wire, tool_change) into controller statements through
this file. Without a file the writers post comments and the expansion warns.

Layout (every key except "controller" is optional; "note" may appear in any object):

  {"schema": "lithocam.cell_io/1", "name": "...", "robot": "ur10e", "controller": "urscript|krl|rapid|gcode",
   "placeholder": true,                      # the values are not from a real cell; posts carry a warning
   "ready_inputs": [{"di": 0, "value": true}],          # waited for once, at program start
   "fault_input": {"di": 1, "ok": false},               # waited for before every drive start
   "spindle": {"start": [action...], "stop": [action...], "speed": speed, "ready": {"di": 2, "value": true},
               "dwell_s": 2.0, "max_rpm": 12000},
   "wire":    (same keys as spindle; speed optional),
   "water":   {"on": [action...], "off": [action...]},
   "air":     {"on": [action...], "off": [action...]},
   "tool_change": {"mode": "none|manual|changer", "message": "Put tool {tool} in the spindle",
                   "actions": [action...], "pockets": {"W7": 1}, "ready": {"di": 3, "value": true}}}

Actions (one operation key each):
  {"do": port, "value": true}        digital output        URScript set_standard_digital_out, KRL $OUT, RAPID SetDO
  {"ao": port, "value": 0.5}         analog output         set_standard_analog_out, $ANOUT, SetAO
  {"wait_di": port, "value": true}   wait for an input     while/get_standard_digital_in, WAIT FOR $IN, WaitDI
  {"register": r, "value": 1}        fieldbus register     write_output_integer_register, KRL variable, SetGO
  {"call": "text {rpm}"}             function / subprogram call line (controller plug-in function, KRL or RAPID routine)
  {"mcode": "T{pocket} M6"}          G-code line (controller gcode only)
  {"dwell": 1.5}                     wait, seconds
Templates may use {rpm}, {tool} and {pocket}.

Speed (spindle.speed, wire.speed):
  {"kind": "analog", "port": 0, "volts_per_rpm": v, "offset_volts": 0.0, "full_scale_volts": 10.0,
   "output": "fraction|volts"}     value = (offset + rpm * v) / full_scale for "fraction" (UR 0..1, KUKA $ANOUT -1..1),
                                   offset + rpm * v for "volts" (RAPID analog signal in volts)
  {"kind": "register", "register": r, "per_rpm": 1.0, "type": "int|float"}
  {"kind": "call", "template": "spindle_speed({rpm})"}
  {"kind": "word", "template": "S{rpm}"}             G-code speed word
A null scaling (volts_per_rpm or full_scale_volts) means "not set": the speed posts as a comment with a warning.
"""
from __future__ import annotations

import json
import string
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Tuple, Union

SCHEMA = "lithocam.cell_io/1"
CONTROLLERS = ("urscript", "krl", "rapid", "gcode")
TOOL_MODES = ("none", "manual", "changer")
FIELDS = {"rpm", "tool", "pocket"}
_OPS = ("do", "ao", "wait_di", "register", "call", "mcode", "dwell")
_TOP = {"schema", "name", "robot", "controller", "placeholder", "note", "ready_inputs", "fault_input", "spindle",
        "wire", "water", "air", "tool_change"}
_DRIVE = {"start", "stop", "speed", "ready", "dwell_s", "max_rpm", "note"}
_SWITCH = {"on", "off", "note"}
_TOOL = {"mode", "message", "actions", "pockets", "ready", "note"}

Op = Tuple   # neutral operation: ("do", port, bool), ("ao", port, float), ("wait_di", port, bool),
#              ("register", reg, value, type), ("call", text), ("mcode", text), ("dwell", s), ("message", text),
#              ("comment", text)


class CellIOError(ValueError):
    pass


def _err(where: str, text: str):
    raise CellIOError("cell I/O {}: {}".format(where, text))


def _check_template(where: str, t) -> None:
    if not isinstance(t, str) or not t.strip():
        _err(where, "needs a non-empty text")
    try:
        names = {f for _, f, _, _ in string.Formatter().parse(t) if f is not None}
    except ValueError as e:
        _err(where, "bad template '{}': {}".format(t, e))
    bad = names - FIELDS
    if bad:
        _err(where, "unknown template field(s) {} in '{}'; use {}".format(sorted(bad), t, sorted(FIELDS)))


def _check_port(where: str, port, controller: str, kind: str) -> None:
    if controller == "gcode":
        _err(where, "the gcode controller has no {} ports here; use mcode actions".format(kind))
    if controller == "rapid":
        if isinstance(port, str) and port.replace("_", "a").isalnum() and not port[0].isdigit():
            return
        if isinstance(port, int) and not isinstance(port, bool) and port >= 0:
            return
        _err(where, "RAPID {} must be a signal name or an index >= 0, got {!r}".format(kind, port))
    if not isinstance(port, int) or isinstance(port, bool):
        _err(where, "{} {} port must be an integer, got {!r}".format(controller, kind, port))
    if controller == "urscript":
        hi = 1 if kind == "analog" else 7
        if not 0 <= port <= hi:
            _err(where, "URScript standard {} port {} is outside 0..{}".format(kind, port, hi))
    if controller == "krl" and port < 1:
        _err(where, "KRL {} index {} must be >= 1".format(kind, port))


def _check_action(where: str, a, controller: str) -> None:
    if not isinstance(a, dict):
        _err(where, "an action is an object, got {!r}".format(a))
    ops = [k for k in a if k in _OPS]
    extra = set(a) - set(_OPS) - {"value", "type", "note"}
    if len(ops) != 1 or extra:
        _err(where, "an action needs exactly one of {} (got keys {})".format(_OPS, sorted(a)))
    op = ops[0]
    if op == "do":
        _check_port(where, a["do"], controller, "digital")
        if not isinstance(a.get("value"), bool):
            _err(where, "a digital output needs value true or false")
    elif op == "wait_di":
        _check_port(where, a["wait_di"], controller, "digital")
        if not isinstance(a.get("value", True), bool):
            _err(where, "wait_di value must be true or false")
    elif op == "ao":
        _check_port(where, a["ao"], controller, "analog")
        if not isinstance(a.get("value"), (int, float)) or isinstance(a.get("value"), bool):
            _err(where, "an analog output needs a numeric value")
    elif op == "register":
        if controller == "gcode":
            _err(where, "registers are not posted in G-code")
        r = a["register"]
        if controller == "urscript" and not (isinstance(r, int) and not isinstance(r, bool) and r >= 0):
            _err(where, "a URScript output register is an index >= 0")
        if controller in ("krl", "rapid") and not (isinstance(r, str) and r.strip()):
            _err(where, "a {} register is the name of the mapped variable or group signal".format(controller))
        if not isinstance(a.get("value"), (int, float)) or isinstance(a.get("value"), bool):
            _err(where, "a register action needs a numeric value")
        if a.get("type", "int") not in ("int", "float"):
            _err(where, "register type must be int or float")
    elif op == "call":
        _check_template(where, a["call"])
    elif op == "mcode":
        if controller != "gcode":
            _err(where, "mcode actions are for the gcode controller only")
        _check_template(where, a["mcode"])
    elif op == "dwell":
        if not isinstance(a["dwell"], (int, float)) or a["dwell"] < 0:
            _err(where, "dwell must be seconds >= 0")


def _check_actions(where: str, acts, controller: str) -> None:
    if not isinstance(acts, list):
        _err(where, "must be a list of actions")
    for k, a in enumerate(acts):
        _check_action("{}[{}]".format(where, k), a, controller)


def _check_input(where: str, d, controller: str, value_key: str = "value") -> None:
    if not isinstance(d, dict) or "di" not in d:
        _err(where, "needs an object with 'di'")
    if set(d) - {"di", value_key, "note"}:
        _err(where, "unknown key(s) {}".format(sorted(set(d) - {"di", value_key, "note"})))
    if controller == "gcode":
        _err(where, "input waits are not posted in G-code; leave them out of a gcode definition")
    _check_port(where, d["di"], controller, "digital")
    if not isinstance(d.get(value_key, True), bool):
        _err(where, "{} must be true or false".format(value_key))


def _check_speed(where: str, s, controller: str) -> None:
    if s is None:
        return
    if not isinstance(s, dict) or s.get("kind") not in ("analog", "register", "call", "word"):
        _err(where, "speed needs kind analog, register, call or word")
    k = s["kind"]
    allowed = {"analog": {"kind", "port", "volts_per_rpm", "offset_volts", "full_scale_volts", "output", "note"},
               "register": {"kind", "register", "per_rpm", "type", "note"},
               "call": {"kind", "template", "note"}, "word": {"kind", "template", "note"}}[k]
    if set(s) - allowed:
        _err(where, "unknown key(s) {} for speed kind {}".format(sorted(set(s) - allowed), k))
    if k == "analog":
        _check_port(where, s.get("port"), controller, "analog")
        for key in ("volts_per_rpm", "full_scale_volts"):
            v = s.get(key)
            if v is not None and (not isinstance(v, (int, float)) or v <= 0):
                _err(where, "{} must be a positive number or null (not set)".format(key))
        if s.get("output", "fraction") not in ("fraction", "volts"):
            _err(where, "analog output must be fraction or volts")
    elif k == "register":
        _check_action(where, {"register": s.get("register"), "value": 0, "type": s.get("type", "int")}, controller)
        if not isinstance(s.get("per_rpm", 1.0), (int, float)):
            _err(where, "per_rpm must be a number")
    elif k == "word":
        if controller != "gcode":
            _err(where, "speed kind word is for the gcode controller only")
        _check_template(where, s.get("template"))
    else:
        _check_template(where, s.get("template"))


def validate(d: Dict) -> List[str]:
    """Check a cell I/O dict. Raises CellIOError on a structural fault; returns warnings (placeholder values,
    unset speed scaling)."""
    if not isinstance(d, dict):
        _err("file", "must be a JSON object")
    if set(d) - _TOP:
        _err("file", "unknown key(s) {}".format(sorted(set(d) - _TOP)))
    if d.get("schema", SCHEMA) != SCHEMA:
        _err("file", "schema must be {}".format(SCHEMA))
    c = d.get("controller")
    if c not in CONTROLLERS:
        _err("file", "controller must be one of {}, got {!r}".format(CONTROLLERS, c))
    warn: List[str] = []
    if d.get("placeholder", False):
        warn.append("cell I/O '{}' holds PLACEHOLDER values: set every port, scaling and dwell for your cell before "
                    "running".format(d.get("name", "")))
    for k, r in enumerate(d.get("ready_inputs") or []):
        _check_input("ready_inputs[{}]".format(k), r, c)
    if d.get("fault_input") is not None:
        _check_input("fault_input", d["fault_input"], c, value_key="ok")
    for drive in ("spindle", "wire"):
        s = d.get(drive)
        if s is None:
            continue
        if not isinstance(s, dict) or set(s) - _DRIVE:
            _err(drive, "unknown key(s) {}".format(sorted(set(s) - _DRIVE) if isinstance(s, dict) else s))
        _check_actions(drive + ".start", s.get("start", []), c)
        _check_actions(drive + ".stop", s.get("stop", []), c)
        _check_speed(drive + ".speed", s.get("speed"), c)
        if s.get("ready") is not None:
            _check_input(drive + ".ready", s["ready"], c)
        for key in ("dwell_s", "max_rpm"):
            v = s.get(key)
            if v is not None and (not isinstance(v, (int, float)) or v < 0):
                _err(drive, "{} must be a number >= 0 or null".format(key))
        sp = s.get("speed") or {}
        if sp.get("kind") == "analog" and (sp.get("volts_per_rpm") is None or
                                           (sp.get("output", "fraction") == "fraction" and
                                            sp.get("full_scale_volts") is None)):
            warn.append("{} speed scaling is not set: the speed posts as a comment".format(drive))
        if not s.get("start"):
            warn.append("{} has no start actions: its start posts as a comment".format(drive))
    for sw in ("water", "air"):
        s = d.get(sw)
        if s is None:
            continue
        if not isinstance(s, dict) or set(s) - _SWITCH:
            _err(sw, "unknown key(s) {}".format(sorted(set(s) - _SWITCH) if isinstance(s, dict) else s))
        _check_actions(sw + ".on", s.get("on", []), c)
        _check_actions(sw + ".off", s.get("off", []), c)
    tc = d.get("tool_change")
    if tc is not None:
        if not isinstance(tc, dict) or set(tc) - _TOOL:
            _err("tool_change", "unknown key(s) {}".format(sorted(set(tc) - _TOOL) if isinstance(tc, dict) else tc))
        if tc.get("mode", "manual") not in TOOL_MODES:
            _err("tool_change", "mode must be one of {}".format(TOOL_MODES))
        if tc.get("message") is not None:
            _check_template("tool_change.message", tc["message"])
        _check_actions("tool_change.actions", tc.get("actions", []), c)
        if tc.get("ready") is not None:
            _check_input("tool_change.ready", tc["ready"], c)
        for tid, pk in (tc.get("pockets") or {}).items():
            if not isinstance(pk, int) or isinstance(pk, bool) or pk < 0:
                _err("tool_change.pockets", "pocket of {} must be an integer >= 0".format(tid))
        if tc.get("mode") == "changer" and not tc.get("actions"):
            _err("tool_change", "mode changer needs actions")
    return warn


@dataclass
class CellIO:
    data: Dict
    path: str = ""
    warnings: List[str] = field(default_factory=list)

    @property
    def controller(self) -> str:
        return self.data["controller"]

    @property
    def placeholder(self) -> bool:
        return bool(self.data.get("placeholder", False))

    @property
    def name(self) -> str:
        return str(self.data.get("name") or Path(self.path).stem or "cell I/O")

    def tool_mode(self) -> str:
        tc = self.data.get("tool_change")
        return tc.get("mode", "manual") if tc else "manual"

    def pocket(self, tool: str) -> Optional[int]:
        tc = self.data.get("tool_change") or {}
        return (tc.get("pockets") or {}).get(tool)

    # -------------------------------------------------------- record -> neutral operations
    def start_ops(self) -> List[Op]:
        """Operations at program start: a banner and the ready inputs."""
        ops: List[Op] = [("comment", "cell I/O: {}{}".format(self.name, " ({})".format(self.path) if self.path
                                                              else ""))]
        if self.placeholder:
            ops.append(("comment", "WARNING: PLACEHOLDER I/O values; set for your cell before running"))
        for r in self.data.get("ready_inputs") or []:
            ops.append(("wait_di", r["di"], bool(r.get("value", True))))
        return ops

    def _acts(self, acts, fmt: Dict) -> List[Op]:
        out: List[Op] = []
        for a in acts or []:
            if "do" in a:
                out.append(("do", a["do"], bool(a["value"])))
            elif "ao" in a:
                out.append(("ao", a["ao"], float(a["value"])))
            elif "wait_di" in a:
                out.append(("wait_di", a["wait_di"], bool(a.get("value", True))))
            elif "register" in a:
                out.append(("register", a["register"], a["value"], a.get("type", "int")))
            elif "call" in a:
                out.append(("call", a["call"].format(**fmt)))
            elif "mcode" in a:
                out.append(("mcode", a["mcode"].format(**fmt)))
            elif "dwell" in a:
                out.append(("dwell", float(a["dwell"])))
        return out

    def speed_ops(self, drive: str, rpm: float) -> List[Op]:
        s = (self.data.get(drive) or {})
        sp = s.get("speed")
        mx = s.get("max_rpm")
        if mx and rpm > mx + 1e-9:
            raise CellIOError("{} speed {:g} rev/min is above the cell's max_rpm {:g}".format(drive, rpm, mx))
        if sp is None:
            return [("comment", "{} {:g} rev/min: no speed output in the cell I/O".format(drive, rpm))]
        k = sp["kind"]
        fmt = {"rpm": _num(rpm), "tool": "", "pocket": ""}
        if k == "analog":
            vpr, fs = sp.get("volts_per_rpm"), sp.get("full_scale_volts")
            out = sp.get("output", "fraction")
            if vpr is None or (out == "fraction" and fs is None):
                return [("comment", "{} {:g} rev/min: speed scaling not set for your cell".format(drive, rpm))]
            volts = float(sp.get("offset_volts", 0.0) or 0.0) + rpm * float(vpr)
            if out == "volts":
                return [("ao", sp["port"], volts)]
            f = volts / float(fs)
            lo = 0.0 if self.controller == "urscript" else -1.0
            if not lo - 1e-12 <= f <= 1.0 + 1e-12:
                raise CellIOError("{} speed {:g} rev/min needs analog value {:.4f}, outside {:g}..1".format(
                    drive, rpm, f, lo))
            return [("ao", sp["port"], f)]
        if k == "register":
            v = rpm * float(sp.get("per_rpm", 1.0))
            typ = sp.get("type", "int")
            return [("register", sp["register"], int(round(v)) if typ == "int" else v, typ)]
        if k == "call":
            return [("call", sp["template"].format(**fmt))]
        return [("mcode", sp["template"].format(**fmt))]

    def drive_on(self, drive: str, rpm: float, running: bool, tool: str = "") -> List[Op]:
        """Start (or re-speed, when `running`) the spindle or the wire motor."""
        s = self.data.get(drive)
        if s is None:
            return [("comment", "{} on {:g} rev/min: the cell I/O has no {} section".format(drive, rpm, drive))]
        ops: List[Op] = []
        fi = self.data.get("fault_input")
        if fi is not None and not running:     # order: no fault, speed reference, start, at speed, dwell
            ops.append(("wait_di", fi["di"], bool(fi.get("ok", False))))
        if rpm and rpm > 0:
            ops += self.speed_ops(drive, rpm)
        if running:
            return ops
        fmt = {"rpm": _num(rpm or 0), "tool": tool, "pocket": "" if self.pocket(tool) is None else self.pocket(tool)}
        start = self._acts(s.get("start"), fmt)
        ops += start or [("comment", "{} start: no start actions in the cell I/O".format(drive))]
        if s.get("ready") is not None:
            ops.append(("wait_di", s["ready"]["di"], bool(s["ready"].get("value", True))))
        if s.get("dwell_s"):
            ops.append(("dwell", float(s["dwell_s"])))
        return ops

    def drive_off(self, drive: str) -> List[Op]:
        s = self.data.get(drive)
        if s is None:
            return [("comment", "{} off: the cell I/O has no {} section".format(drive, drive))]
        fmt = {"rpm": "0", "tool": "", "pocket": ""}
        return self._acts(s.get("stop"), fmt) or [("comment", "{} stop: no stop actions in the cell I/O".format(
            drive))]

    def switch(self, what: str, on: bool) -> List[Op]:
        s = self.data.get(what)
        if s is None:
            return [("comment", "{} {}: the cell I/O has no {} section".format(what, "on" if on else "off", what))]
        fmt = {"rpm": "0", "tool": "", "pocket": ""}
        return self._acts(s.get("on" if on else "off"), fmt) or [("comment", "{} {}: no actions".format(
            what, "on" if on else "off"))]

    def tool_change(self, tool: str, mode: Optional[str] = None) -> List[Op]:
        tc = self.data.get("tool_change") or {}
        mode = mode or tc.get("mode", "manual")
        pk = self.pocket(tool)
        fmt = {"rpm": "0", "tool": tool, "pocket": "" if pk is None else pk}
        if mode == "none":
            return [("comment", "tool {} (no tool change: the cell holds one tool)".format(tool))]
        if mode == "changer":
            if not tc.get("actions"):
                raise CellIOError("tool change to {}: mode changer but no changer actions".format(tool))
            if pk is None and any("{pocket}" in str(a.get("call", a.get("mcode", ""))) for a in tc["actions"]):
                raise CellIOError("tool change to {}: no pocket for this tool in tool_change.pockets".format(tool))
            ops = [("comment", "tool change to {}".format(tool))] + self._acts(tc["actions"], fmt)
            if tc.get("ready") is not None:
                ops.append(("wait_di", tc["ready"]["di"], bool(tc["ready"].get("value", True))))
            return ops
        msg = (tc.get("message") or "Put tool {tool} in the spindle, then continue").format(**fmt)
        return [("comment", "tool change to {}: manual".format(tool)), ("message", msg)]


def _num(v: float) -> str:
    s = "{:.3f}".format(float(v))
    return s.rstrip("0").rstrip(".") if "." in s else s


def from_dict(d: Dict, path: str = "") -> CellIO:
    warn = validate(d)
    return CellIO(dict(d), str(path), warn)


def load(path: Union[str, Path]) -> CellIO:
    p = Path(path)
    try:
        d = json.loads(p.read_text(encoding="utf-8"))
    except json.JSONDecodeError as e:
        raise CellIOError("cell I/O {}: not valid JSON ({})".format(p, e))
    return from_dict(d, str(p))


def coerce(io) -> Optional[CellIO]:
    """CellIO from a CellIO, a dict or a path; None stays None."""
    if io is None or isinstance(io, CellIO):
        return io
    if isinstance(io, dict):
        return from_dict(io)
    return load(io)


def default_path(robot: str, cells_dir: Union[str, Path]) -> Path:
    """The shipped placeholder file of a cell: cells/<robot>_io.json."""
    return Path(cells_dir) / "{}_io.json".format(robot)


def no_io_ops(kind: str, data: Dict) -> List[Op]:
    """Operations when no cell I/O is loaded: comments, and an operator pause for a tool change."""
    if kind == "spindle":
        if data.get("on", (data.get("rpm") or 0) > 0):
            return [("comment", "spindle {} rev/min: map to the cell's spindle I/O".format(data.get("rpm", 0)))]
        return [("comment", "spindle off: map to the cell's spindle I/O")]
    if kind in ("water", "air", "wire"):
        return [("comment", "{} {}: map to the cell's {} I/O".format(kind, "on" if data.get("on", True) else "off",
                                                                     kind))]
    if kind == "tool_change":
        t = data.get("id", "")
        if data.get("mode") == "none":
            return [("comment", "tool {} (no tool change)".format(t))]
        return [("comment", "tool change to {}: no cell I/O, the operator changes it".format(t)),
                ("message", "Put tool {} in the spindle, then continue".format(t))]
    return []


class Process:
    """Turns process records into neutral operations with spindle, wire, water and air state, for one writer run.
    The state lets a second spindle record with a new speed post only the speed."""

    def __init__(self, io: Optional[CellIO]):
        self.io = io
        self.on = {"spindle": False, "wire": False, "water": False, "air": False}
        self.tool = ""

    def ops(self, kind: str, data: Dict) -> List[Op]:
        io = self.io
        if kind == "io_map":
            return io.start_ops() if io is not None else []
        if kind in ("spindle", "wire"):
            rpm = float(data.get("rpm") or 0.0)
            on = bool(data.get("on", rpm > 0 if kind == "spindle" else True))
            was = self.on[kind]
            self.on[kind] = on
            if io is None:
                return no_io_ops(kind, dict(data, on=on))
            if on:
                return io.drive_on(kind, rpm, was, self.tool)
            return io.drive_off(kind) if was else [("comment", "{} already off".format(kind))]
        if kind in ("water", "air"):
            on = bool(data.get("on", True))
            self.on[kind] = on
            return no_io_ops(kind, data) if io is None else io.switch(kind, on)
        if kind == "tool_change":
            self.tool = str(data.get("id", ""))
            return no_io_ops(kind, data) if io is None else io.tool_change(self.tool, data.get("mode"))
        return []

    def still_on(self) -> List[str]:
        return [k for k, v in self.on.items() if v]
