# Copyright 2026 The lithocam authors. SPDX-License-Identifier: Apache-2.0
"""Headless programming API: build, check, simulate and post a robot program from Python, with no window.

Written for scripts and for language models that write code. Every call checks its input and, when something is
wrong, raises ApiError with a message that names the bad value and lists the valid ones, so a caller can correct
itself from the message alone.

    from lithocam.studio import api
    s = api.Session("ur10e", material="granite")
    s.load_part("dome.stl")                       # mm, z up; the stock box follows the part
    s.set_work_frame([0.7, 0.0, 0.0])             # metres, robot base frame
    s.add_target("approach", joints=[0, -90, 90, -90, -90, 0])
    p = s.program("Main")
    p.add("set_tool", tool="W7")
    p.add("movej", target="approach")
    p.add("raster", tool="W7", stepover=4.0, feed=600.0)
    r = s.simulate("Main")                        # r.ok, r.errors, r.messages, r.unreachable
    text = s.post("Main", "urscript")             # refuses while r.errors stand
    s.save("station.lcs.json")                    # the studio opens the same file

Discover what exists with api.steps(), api.step("raster"), s.tools(), api.robots(), api.dialects().
Units: lengths in mm and angles in degrees in step parameters and targets given by joints; frames and poses in
metres (4x4 matrices). The station file is the program: everything a Session holds is in Session.station.
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Sequence

import numpy as np

from .. import post
from . import catalog, program, sheet


class ApiError(ValueError):
    """Bad input. The message says what is wrong and what is allowed."""


# ---------------------------------------------------------------- discovery
def robots() -> Dict[str, str]:
    """Robot ids and names."""
    return {k: v[0] for k, v in program.ROBOTS.items()}


def dialects() -> List[str]:
    return list(post.WRITERS)


def steps(category: Optional[str] = None) -> List[Dict[str, object]]:
    """The step catalogue: id, label, category, status (done / partial / planned), tool family, parameters."""
    return [step(s.id) for s in catalog.STEPS if category is None or s.category == category]


def step(kind: str) -> Dict[str, object]:
    spec = _spec(kind)
    return {"id": spec.id, "label": spec.label, "category": spec.category, "status": spec.status,
            "tool_family": spec.tool_family, "doc": spec.doc,
            "params": [{"key": k, "label": lab, "type": t, "default": d, "unit": u} for k, lab, t, d, u in spec.params]}


def _spec(kind: str) -> catalog.StepSpec:
    if kind not in catalog.BY_ID:
        near = [s.id for s in catalog.STEPS if kind.lower() in s.id or kind.lower() in s.label.lower()]
        raise ApiError("unknown step '{}'. {}Valid steps: {}".format(
            kind, "Did you mean: {}. ".format(", ".join(near)) if near else "", ", ".join(catalog.BY_ID)))
    return catalog.BY_ID[kind]


# ---------------------------------------------------------------- program and session
@dataclass
class Result:
    ok: bool
    errors: List[str]
    messages: List[str]
    unreachable: int
    samples: int
    max_joint_step_deg: float
    expansion: object = field(repr=False, default=None)

    def summary(self) -> str:
        head = "OK" if self.ok else "NOT OK"
        return "{}: {} samples, {} unreachable or colliding, largest joint step {:.2f} deg, {} error(s)".format(
            head, self.samples, self.unreachable, self.max_joint_step_deg, len(self.errors))


class ProgramBuilder:
    """Adds checked steps to one program of a Session."""

    def __init__(self, session: "Session", prog: program.Program):
        self.session = session
        self.prog = prog

    @property
    def name(self) -> str:
        return self.prog.name

    def add(self, kind: str, label: str = "", at: Optional[int] = None, **params) -> int:
        """Append (or insert at index `at`) a step; returns its index. Unknown parameters, wrong types, unknown
        tools, tools that do not fit the step and unknown targets raise ApiError."""
        spec = _spec(kind)
        st = program.Step(kind, self.session._check_params(spec, params), label)
        if at is None:
            self.prog.steps.append(st)
            return len(self.prog.steps) - 1
        if not 0 <= at <= len(self.prog.steps):
            raise ApiError("at={} is outside 0..{}".format(at, len(self.prog.steps)))
        self.prog.steps.insert(at, st)
        return at

    def remove(self, index: int):
        self._index(index)
        del self.prog.steps[index]

    def set(self, index: int, **params):
        """Change parameters of an existing step."""
        st = self.prog.steps[self._index(index)]
        st.params.update(self.session._check_params(st.spec, params, partial=True))

    def move(self, index: int, to: int):
        st = self.prog.steps.pop(self._index(index))
        self.prog.steps.insert(max(0, min(to, len(self.prog.steps))), st)

    def list(self) -> List[Dict[str, object]]:
        return [{"index": i, "kind": s.kind, "title": s.title(), "params": dict(s.params)}
                for i, s in enumerate(self.prog.steps)]

    def _index(self, index: int) -> int:
        if not -len(self.prog.steps) <= index < len(self.prog.steps):
            raise ApiError("step index {} is outside 0..{}".format(index, len(self.prog.steps) - 1))
        return index % len(self.prog.steps)


class Session:
    def __init__(self, robot: str = "ur10e", material: str = "granite", station: Optional[program.Station] = None,
                 tool_library: str = ""):
        if station is None:
            if robot not in program.ROBOTS:
                raise ApiError("unknown robot '{}'. Valid robots: {}".format(robot, ", ".join(program.ROBOTS)))
            station = program.Station(robot=robot, material=material, programs=[])
            station.work_xyz = [1.6, 0.0, 0.3] if robot.startswith("kr") else [0.7, 0.0, 0.0]
        self.station = station
        self.part = None                       # (tris mm, TriMesh)
        self._robot = None
        self._tools = None
        if tool_library:
            self.load_tools(tool_library)
        if station.part_path and Path(station.part_path).exists():
            self.load_part(station.part_path, keep_stock=True)

    # files
    @staticmethod
    def open(path) -> "Session":
        return Session(station=program.Station.load(path))

    def save(self, path) -> str:
        """Write the station file; part, ruled-face and tool-library paths are stored relative to it."""
        path = Path(path)
        st = program.Station.from_json(self.station.to_json())
        for k in ("part_path", "ruled_path", "tool_library"):
            v = getattr(st, k)
            if v:
                try:
                    setattr(st, k, str(Path(v).resolve().relative_to(path.resolve().parent)).replace("\\", "/"))
                except ValueError:
                    pass
        path.write_text(st.to_json(), encoding="utf-8")
        return str(path)

    def to_dict(self) -> Dict[str, object]:
        return json.loads(self.station.to_json())

    # cell
    @property
    def robot(self):
        if self._robot is None or self._robot_name != self.station.robot:
            self._robot = program.load_robot(self.station.robot)
            self._robot_name = self.station.robot
        return self._robot

    def set_work_frame(self, xyz: Sequence[float], rpy_deg: Sequence[float] = (0.0, 0.0, 0.0)):
        """Work frame origin (m) and roll-pitch-yaw (deg) in the robot base frame."""
        self.station.work_xyz = _vec(xyz, 3, "xyz")
        self.station.work_rpy = _vec(rpy_deg, 3, "rpy_deg")

    def set_turntable(self, diameter: float = 0.8, height: float = 0.04, enabled: bool = True):
        self.station.turntable = {"diameter": float(diameter), "height": float(height)} if enabled else None

    def set_material(self, material: str):
        from .. import cutting_data
        known = sorted({e.material for e in cutting_data.TABLE})
        if material not in known:
            raise ApiError("unknown material '{}'. Known: {}".format(material, ", ".join(known)))
        self.station.material = material

    def load_part(self, path, stock_margin: float = 5.0, keep_stock: bool = False, stride: int = 1):
        """Load the part: an STL (mm, z up) or a height-field .npz (a relief slab, see studio.cam.load_part;
        stride > 1 plans on a coarser grid). The stock box follows the part unless keep_stock and one is set."""
        from . import cam
        p = Path(path)
        if not p.exists():
            raise ApiError("part file not found: {}".format(p))
        if p.suffix.lower() not in (".stl", ".npz"):
            raise ApiError("part file must be .stl or a height-field .npz (got '{}')".format(p.name))
        if int(stride) < 1:
            raise ApiError("stride must be an integer >= 1 (got {!r})".format(stride))
        try:
            tris = cam.load_part(p, stride=int(stride))
        except ValueError as e:
            raise ApiError(str(e)) from None
        self.part = (tris, cam.trimesh(tris))
        self.station.part_path = str(p.resolve())
        if not (keep_stock and self.station.stock):
            self.station.stock = cam.stock_box(tris, stock_margin)

    def set_stock(self, xmin, xmax, ymin, ymax, zmin, zmax):
        if not (xmin < xmax and ymin < ymax and zmin < zmax):
            raise ApiError("stock box needs min < max on every axis")
        self.station.stock = dict(xmin=xmin, xmax=xmax, ymin=ymin, ymax=ymax, zmin=zmin, zmax=zmax)

    # tools
    def load_tools(self, csv_path):
        from .core import lc
        self._tools = {t.id: t for t in lc.read_tool_library(str(csv_path)).tools}
        self.station.tool_library = str(Path(csv_path).resolve())

    def _tool_map(self):
        if self._tools is None:
            from .core import lc
            if self.station.tool_library:
                self._tools = {t.id: t for t in lc.read_tool_library(self.station.tool_library).tools}
            else:
                self._tools = {t.id: t for t in list(lc.stone_tool_library().tools) +
                               list(lc.wood_tool_library().tools)}
        return self._tools

    def tools(self, family: str = "") -> List[Dict[str, object]]:
        """Tools in the library (id, name, kind, diameter mm, operations); `family` filters by operation."""
        from .core import lc
        out = []
        for t in self._tool_map().values():
            ops = [str(o) for o in lc.tool_operations(t.kind)]
            if family and family not in ops:
                continue
            out.append({"id": t.id, "name": t.name, "kind": lc.tool_kind_name(t.kind),
                        "diameter": float(t.diameter), "operations": ops})
        return out

    # targets
    def add_target(self, name: str, joints: Optional[Sequence[float]] = None, pose=None) -> Dict[str, object]:
        """A named position: joints in degrees, or a TCP pose (4x4, m, robot base) solved by IK from home."""
        if not name or not str(name).replace("_", "").isalnum():
            raise ApiError("target names use letters, digits and _ (got '{}')".format(name))
        r = self.robot
        if joints is not None:
            q = np.radians(_vec(joints, r.dof, "joints"))
            if np.any(q < r.lower - 1e-9) or np.any(q > r.upper + 1e-9):
                raise ApiError("joints {} are outside the limits {} .. {} deg".format(
                    list(np.round(np.degrees(q), 2)), list(np.round(np.degrees(r.lower), 1)),
                    list(np.round(np.degrees(r.upper), 1))))
            T = r.tcp(q)
        elif pose is not None:
            T = np.asarray(pose, dtype=float)
            if T.shape != (4, 4):
                raise ApiError("pose must be a 4x4 matrix (metres, robot base frame)")
            q, ok = program.solve_start(r, T, program.HOME.get(self.station.robot, [0.0] * r.dof))
            if not ok:
                raise ApiError("pose of target '{}' is out of reach".format(name))
        else:
            raise ApiError("give joints (deg) or pose (4x4, m)")
        return self.station.add_target(q, T, "", name)

    def targets(self) -> List[Dict[str, object]]:
        return list(self.station.targets)

    # programs
    def program(self, name: str = "Main") -> ProgramBuilder:
        """The program with this name, created when missing."""
        for p in self.station.programs:
            if p.name == name:
                return ProgramBuilder(self, p)
        p = program.Program(name, [])
        self.station.programs.append(p)
        return ProgramBuilder(self, p)

    def programs(self) -> List[str]:
        return [p.name for p in self.station.programs]

    def _prog(self, name) -> program.Program:
        for p in self.station.programs:
            if p.name == name:
                return p
        raise ApiError("no program '{}'. Programs: {}".format(name, ", ".join(self.programs()) or "(none)"))

    def _check_params(self, spec, params, partial=False) -> Dict[str, object]:
        from .core import lc
        known = {k: (t, d, u) for k, _, t, d, u in spec.params}
        if spec.tool_family:
            known.setdefault("tool", ("tool", "", ""))
        out = {}
        for k, v in params.items():
            if k not in known:
                raise ApiError("step '{}' has no parameter '{}'. Parameters: {}".format(
                    spec.id, k, ", ".join("{} ({}{})".format(n, t, ", " + u if u else "")
                                          for n, (t, _, u) in known.items()) or "(none)"))
            t = known[k][0]
            if t == "float":
                v = _num(v, k)
            elif t == "int":
                if isinstance(v, bool) or int(_num(v, k)) != _num(v, k):
                    raise ApiError("{} must be an integer (got {!r})".format(k, v))
                v = int(v)
            elif t == "bool":
                if not isinstance(v, bool):
                    raise ApiError("{} must be true or false (got {!r})".format(k, v))
            elif t.startswith("choice:"):
                ch = t[7:].split("|")
                if v not in ch:
                    raise ApiError("{} must be one of {} (got {!r})".format(k, ", ".join(ch), v))
            elif t == "joints":
                v = None if v is None else _vec(v, self.robot.dof, k)
            elif t == "pose":
                if v is not None:
                    v = np.asarray(v, dtype=float)
                    if v.shape != (4, 4):
                        raise ApiError("{} must be a 4x4 matrix (metres, robot base frame)".format(k))
                    v = v.tolist()
            elif t == "target":
                if v and self.station.target(str(v)) is None:
                    raise ApiError("no target '{}'. Targets: {}".format(
                        v, ", ".join(x["name"] for x in self.station.targets) or "(none; use add_target)"))
            elif t == "tool":
                tools = self._tool_map()
                if v and v not in tools:
                    raise ApiError("no tool '{}' in the library. Use Session.tools() to list them".format(v))
                fam = spec.tool_family
                if v and fam and fam != "any" and fam not in [str(o) for o in lc.tool_operations(tools[v].kind)]:
                    fits = [t["id"] for t in self.tools(fam)]
                    raise ApiError("tool {} cannot do '{}' (needs a '{}' tool). Tools that fit: {}".format(
                        v, spec.id, fam, ", ".join(fits[:30]) or "(none in this library)"))
            out[k] = v
        return out

    # run
    def simulate(self, program_name: str = "Main", collisions: bool = False, q0=None) -> Result:
        """Expand the program into joint motion; collisions=True plans with Tesseract (needs tesseract-robotics)."""
        prog = self._prog(program_name)
        st = self.station
        needs_part = any(s.kind in _generators() for s in prog.steps)
        if needs_part and self.part is None:
            raise ApiError("program '{}' has machining steps; load a part first (load_part)".format(program_name))
        backend = None
        if collisions:
            from .planner import make_backend
            backend = make_backend(st, program.CELLS)
            if backend is None:
                raise ApiError("collisions=True needs tesseract-robotics (pip install tesseract-robotics)")
        ex = program.expand(st, self.robot, prog, self._tool_map(), self.part, q0, backend=backend)
        q = ex.joints()
        step = float(np.degrees(np.max(np.abs(np.diff(q, axis=0))))) if len(q) > 1 else 0.0
        self._last = ex
        return Result(not ex.errors and ex.unreachable == 0, list(ex.errors), list(ex.messages), ex.unreachable,
                      len(q), step, ex)

    def post(self, program_name: str = "Main", dialect: str = "", path=None, allow_unreachable: bool = False
             ) -> str:
        """Program text for the robot controller. Simulates first; refuses while errors stand or (unless
        allow_unreachable) any sample is unreachable or colliding. The joint read-back check must pass."""
        native = program.ROBOTS[self.station.robot][2]
        dialect = dialect or native
        if dialect not in post.WRITERS:
            raise ApiError("unknown dialect '{}'. Valid: {}".format(dialect, ", ".join(post.WRITERS)))
        if dialect not in (native, "cl"):
            raise ApiError("the {} runs '{}' programs; '{}' is for another controller. Use '{}' or 'cl' (neutral "
                           "text)".format(program.ROBOTS[self.station.robot][0], native, dialect, native))
        r = self.simulate(program_name)
        if r.errors:
            raise ApiError("not posted, fix first:\n  " + "\n  ".join(r.errors))
        if r.unreachable and not allow_unreachable:
            raise ApiError("not posted: {} samples are unreachable or colliding (see Result.messages)".format(
                r.unreachable))
        ex = r.expansion
        text = post.write(ex.records, dialect, **program.post_options(self.station.robot, dialect))
        back = post.joint_targets(text, dialect)
        want = [x.data["q"] for x in ex.records if x.kind in ("movej", "movel")]
        if len(back) != len(want) or (back and np.max(np.abs(np.asarray(back) - np.asarray(want))) > 1e-5):
            raise ApiError("internal: the posted joint targets do not read back equal to the plan")
        want_e1 = [x.data.get("e1") for x in ex.records if x.kind in ("movej", "movel")]
        if any(v is not None for v in want_e1):               # the table is a coordinated axis: E1 reads back too
            got = post.external_axes(text, dialect)
            if len(got) != len(want_e1) or any((g is None) != (w is None) or (g is not None and abs(g - w) > 1e-3)
                                               for g, w in zip(got, want_e1)):
                raise ApiError("internal: the posted E1 values do not read back equal to the plan")
        if path:
            Path(path).write_text(text, encoding="utf-8")
        return text

    def setup_sheet(self, program_name: str = "Main", stock: bool = False, path=None) -> str:
        """Markdown setup sheet of the program: per cutting step the tool, feed, spindle speed, path lengths,
        estimated time and, with stock=True, the removed volume (a stock simulation, which takes a while). Simulates
        first; a program with errors still gets a sheet, with the errors listed. See studio.sheet."""
        r = self.simulate(program_name)
        run = None
        if stock:
            from . import process
            run = process.simulate_stock(self.station, self.robot, r.expansion, self._tool_map())
        text = sheet.setup_sheet(self.station, r.expansion, self._prog(program_name), self._tool_map(), run)
        if path:
            Path(path).write_text(text, encoding="utf-8")
        return text


WIRE_FRAMES = [("z", "y", "-x"), ("z", "x", "y"), ("z", "-y", "x"), ("z", "-x", "-y")]


def export_wire_cnc(session: "Session", machine_path, out_dir, feed: float, faces: str = "all",
                    frame: Optional[Sequence[str]] = None, offset: Sequence[float] = (200.0, 0.0, 1000.0),
                    dxf: bool = True) -> Dict[str, object]:
    """CNC diamond wire saw export (lithocam.wiresaw_cnc) of the station's ruled faces labelled 'wire': one job per
    face group, ISO G-code files (+ DXF of the wire-centre path). The machine definition JSON names the layout
    (xy, xyc, xyuv), axis letters, kerf mode (CAM offset or G41/G42) and M-codes. Every file is read back and
    checked (wire start before the first cut, wire stop at the end, positive feeds)."""
    from .. import wiresaw_cnc as wc
    from ..wirepath import load_ruled_json
    if not session.station.ruled_path:
        raise ApiError("the station has no ruled-face JSON (Station.ruled_path)")
    try:
        mc = wc.MachineDef.load(machine_path)
    except (OSError, ValueError) as e:
        raise ApiError("machine definition {}: {}".format(machine_path, e)) from None
    rep = load_ruled_json(session.station.ruled_path)
    fs = [f for f in rep.faces if f.label == "wire" and len(f.rulings) >= 2]
    if faces != "all":
        keep = {int(s) for s in str(faces).replace(";", ",").split(",") if s.strip()}
        fs = [f for f in fs if f.index in keep]
    if not fs:
        raise ApiError("no wire faces selected")
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    written, summaries, refused = [], [], []
    for f in fs:
        plan, why = None, []
        # frame: which part axes become (trolley, depth, height); without one, the clamping orientations that make
        # the rulings advance along the trolley are tried in turn (the part is re-clamped between faces)
        for fr in ([tuple(frame)] if frame else WIRE_FRAMES):
            try:
                setup = wc.ruled_face_setup(f, mc, frame=fr, offset=tuple(offset), feed=float(feed),
                                            name="face{}".format(f.index))
                plan = wc.plan_job(wc.WireJob(name="face{}".format(f.index), setups=[setup], feed=float(feed)), mc)
                summaries.append(dict(plan.summary(), face=f.index, clamped=",".join(fr)))
                break
            except wc.WireSawError as e:
                why.append("{}: {}".format(",".join(fr), e))
        if plan is None:
            refused.append("face {}: {}".format(f.index, " | ".join(why)))
            continue
        try:
            for k, text in enumerate(wc.gcode_files(plan)):
                wc.check_program(text, mc, "face {}".format(f.index))
                fn = out / "{}_face{}{}.nc".format(mc.name, f.index, "" if k == 0 else "_{}".format(k + 1))
                fn.write_text(text, encoding="utf-8")
                written.append(str(fn))
            if dxf:
                fn = out / "{}_face{}.dxf".format(mc.name, f.index)
                fn.write_text(wc.write_dxf(plan), encoding="utf-8")
                written.append(str(fn))
        except wc.WireSawError as e:
            refused.append("face {}: {}".format(f.index, e))
    return {"files": written, "summary": summaries, "refused": refused}


def _generators():
    from . import cam
    return set(cam.GENERATORS) | {"wire_ruled", "polish"}


def _num(v, name) -> float:
    if isinstance(v, bool) or not isinstance(v, (int, float, np.floating, np.integer)):
        raise ApiError("{} must be a number (got {!r})".format(name, v))
    if not math.isfinite(float(v)):
        raise ApiError("{} must be finite".format(name))
    return float(v)


def _vec(v, n, name) -> List[float]:
    try:
        out = [float(x) for x in v]
    except (TypeError, ValueError):
        raise ApiError("{} must be a list of {} numbers (got {!r})".format(name, n, v)) from None
    if len(out) != n:
        raise ApiError("{} needs {} values (got {})".format(name, n, len(out)))
    return out


def station_schema() -> Dict[str, object]:
    """JSON Schema of the station file (the program format the studio and this API share)."""
    step_ids = list(catalog.BY_ID)
    mat4 = {"type": "array", "items": {"type": "array", "items": {"type": "number"}, "minItems": 4, "maxItems": 4},
            "minItems": 4, "maxItems": 4}
    return {
        "$schema": "https://json-schema.org/draft/2020-12/schema",
        "title": "lithocam station",
        "type": "object",
        "required": ["robot", "programs"],
        "properties": {
            "robot": {"enum": list(program.ROBOTS)},
            "work_xyz": {"type": "array", "items": {"type": "number"}, "minItems": 3, "maxItems": 3,
                         "description": "work frame origin, m, robot base"},
            "work_rpy": {"type": "array", "items": {"type": "number"}, "minItems": 3, "maxItems": 3,
                         "description": "deg"},
            "part_path": {"type": "string"}, "ruled_path": {"type": "string"}, "tool_library": {"type": "string"},
            "stock": {"type": ["object", "null"], "properties": {k: {"type": "number"} for k in
                                                                 ("xmin", "xmax", "ymin", "ymax", "zmin", "zmax")}},
            "tool_length": {"type": "number"}, "spindle": {"type": ["object", "string", "null"]},
            "material": {"type": "string"}, "robot_factor": {"type": "number"},
            "turntable": {"type": ["object", "null"]},
            "targets": {"type": "array", "items": {"type": "object", "required": ["name", "joints", "pose"],
                                                   "properties": {"name": {"type": "string"},
                                                                  "joints": {"type": "array", "items": {"type": "number"}},
                                                                  "pose": mat4, "tool": {"type": "string"}}}},
            "programs": {"type": "array", "items": {
                "type": "object", "required": ["name", "steps"],
                "properties": {"name": {"type": "string"}, "steps": {"type": "array", "items": {
                    "type": "object", "required": ["kind"],
                    "properties": {"kind": {"enum": step_ids}, "params": {"type": "object"},
                                   "label": {"type": "string"}}}}}}},
        },
    }


def main(argv=None) -> int:
    """python -m lithocam.studio.api steps|schema|check STATION [PROGRAM]|post STATION PROGRAM DIALECT OUT|
    sheet STATION PROGRAM OUT.md"""
    import sys
    a = list(sys.argv[1:] if argv is None else argv)
    try:
        if not a or a[0] == "steps":
            print(json.dumps(steps(), indent=1, default=str))
        elif a[0] == "schema":
            print(json.dumps(station_schema(), indent=1))
        elif a[0] == "check":
            s = Session.open(a[1])
            names = a[2:] or s.programs()
            import time
            out = {}
            for n in names:
                t0 = time.perf_counter()
                out[n] = s.simulate(n).summary() + ", {:.1f} s".format(time.perf_counter() - t0)
            print(json.dumps(out, indent=1))
        elif a[0] == "post":
            s = Session.open(a[1])
            s.post(a[2], a[3], path=a[4])
            print(json.dumps({"written": a[4]}))
        elif a[0] == "sheet":
            if len(a) != 4:
                raise ApiError("sheet needs STATION PROGRAM OUT.md")
            Session.open(a[1]).setup_sheet(a[2], path=a[3])
            print(json.dumps({"written": a[3]}))
        else:
            raise ApiError("commands: steps, schema, check STATION [PROGRAM], post STATION PROGRAM DIALECT OUT, "
                           "sheet STATION PROGRAM OUT.md")
    except ApiError as e:
        print(json.dumps({"error": str(e)}))
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
