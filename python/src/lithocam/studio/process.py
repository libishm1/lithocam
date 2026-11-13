# Copyright 2026 The lithocam authors. SPDX-License-Identifier: Apache-2.0
"""Stock removal along an expanded program, and a process check of forces, deflections and the remaining part.

simulate_stock cuts a narrow-band SDF stock (lithocam.sdfstock) with the active tool at every feed sample of an
expansion. The tool pose is the robot's forward kinematics with the segment's TCP offset, expressed in mm in the work
frame turned with the table (the part turns with the turntable). Tools by kind:

  ball end mill        capsule between consecutive ball centres (tip + R * axis, axis = -tool z), plus the fluted
                       cylinder above the ball centre (flute length from studio.spindles.cutter_parts)
  flat / bull nose     rounded cylinder of the flute length, swept tip to tip (axis interpolated for 5-axis)
  disc saw             thin cylinder: blade normal = tool z, centre = TCP, thickness = kerf
  diamond wire         a wire of the tool radius along tool0 x, at the bow depth along tool z (studio.program
                       WIRE_BOW), between the pulleys of the cell's bow saw (cells/make_cells.py WIRE_TOOLS)

process_check turns the removed volume per step into a specific-energy milling force (lithocam.interaction), the
robot's TCP deflection under it (joint stiffness), diamond-wire forces and bow, and a voxel FEA strength check of the
final stock clamped at its base.

No material constant lives in this module. Each constant comes from the caller or from a sourced row of
lithocam.materials_mechanics for the station's material (the row and the end of its range are recorded); a check whose
constant is missing is skipped with a message naming it.

Approximations, stated once:
* Only samples flagged as feed are cut; rapids, plunges and joint moves are not. A sample whose IK did not converge
  is cut where the forward kinematics puts the tool.
* One force resultant per step from the removed volume of that step (interaction.milling_forces).
* The part check loads the FINAL stock with the peak step force at that step's contact point: a screening check, not
  the stress history.
"""
from __future__ import annotations

import importlib.util
import math
import time
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Sequence, Tuple

import numpy as np

from .. import interaction, materials_mechanics
from ..sdfstock import SdfStock
from . import program, spindles

# ---------------------------------------------------------------- tool models

BALL, CYLINDER, DISC, WIRE = "ball", "cylinder", "disc", "wire"


def _kind_name(tool) -> str:
    k = getattr(tool, "kind", None)
    return str(getattr(k, "name", k))


def wire_bow_geometry(robot_name: str, robot=None) -> Tuple[float, float, str]:
    """(half span mm, bow depth mm, source text) of the cell's diamond-wire bow saw.

    The half span is read from cells/make_cells.py WIRE_TOOLS (pulley centres at x = -half and +half); when that
    file is not shipped, from the pulley visuals of the robot URDF. The depth is studio.program.WIRE_BOW."""
    depth = program.WIRE_BOW.get(robot_name)
    if depth is None:
        raise ValueError("no wire bow depth for robot '{}' (studio.program.WIRE_BOW)".format(robot_name))
    path = program.CELLS / "make_cells.py"
    if path.exists():
        spec = importlib.util.spec_from_file_location("lithocam_make_cells", str(path))
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        t = mod.WIRE_TOOLS.get(robot_name)
        if t is not None:
            return float(t["half"]) * 1000, float(depth) * 1000, \
                "cells/make_cells.py WIRE_TOOLS['{}']: half {:g} m, depth {:g} m; wire depth WIRE_BOW {:g} m".format(
                    robot_name, t["half"], t["depth"], depth)
    if robot is not None:
        xs = [abs(v.origin[0, 3]) for name in ("wiresaw_pulley_a", "wiresaw_pulley_b")
              for v in robot.visuals.get(name, [])]
        if xs:
            return max(xs) * 1000, float(depth) * 1000, "URDF pulley visuals: half {:g} m; WIRE_BOW {:g} m".format(
                max(xs), depth)
    raise ValueError("no wire bow span for robot '{}' (cells/make_cells.py WIRE_TOOLS)".format(robot_name))


def tool_model(tool, robot_name: str = "", robot=None) -> Dict[str, object]:
    """Cutting model of a library tool for the SDF stock (mm). Raises ValueError for tools without one."""
    from .core import lc
    K = lc.ToolKind
    D = float(getattr(tool, "diameter", 0.0) or 0.0)
    flutes = [p for p in spindles.cutter_parts(tool, spindles.stickout(tool)) if p[0] == "flutes"]
    length = flutes[0][3] if flutes else spindles.stickout(tool)
    if tool.kind == K.BallEndMill:
        if D <= 0:
            raise ValueError("{}: ball end mill without a diameter".format(tool.id))
        return {"kind": BALL, "r": D / 2, "D": D, "flute": max(length - D / 2, 0.0)}
    if tool.kind in (K.FlatEndMill, K.BullNoseMill):
        cr = float(getattr(tool, "corner_radius", 0.0) or 0.0) if tool.kind == K.BullNoseMill else 0.0
        if D <= 0 or length <= 0:
            raise ValueError("{}: end mill without a diameter or length".format(tool.id))
        return {"kind": CYLINDER, "r": D / 2, "length": length, "corner_r": cr, "D": D}
    if tool.kind == K.DiscSaw:
        kerf = float(getattr(tool, "kerf", 0.0) or 0.0) or float(getattr(tool, "segment_height", 0.0) or 0.0)
        if D <= 0 or kerf <= 0:
            raise ValueError("{}: disc saw needs a diameter and a kerf".format(tool.id))
        return {"kind": DISC, "R": D / 2, "kerf": kerf, "D": D}
    if tool.kind in (K.DiamondWire, K.MultiWireSaw):
        if D <= 0:
            raise ValueError("{}: diamond wire without a diameter".format(tool.id))
        half, depth, src = wire_bow_geometry(robot_name, robot)
        return {"kind": WIRE, "r": D / 2, "half": half, "depth": depth, "D": D, "bow_source": src}
    raise ValueError("{}: no stock-removal model for a {}".format(tool.id, _kind_name(tool)))


def cut_step(stock: SdfStock, model: Dict[str, object], P0: np.ndarray, P1: np.ndarray) -> float:
    """Cut one move between two tool poses (4x4, work frame, translation in mm). Returns the removed volume, mm3."""
    k = model["kind"]
    a0, a1 = -P0[:3, 2], -P1[:3, 2]                      # tool axis, tip to holder
    if k == BALL:
        r = model["r"]
        c0, c1 = P0[:3, 3] + r * a0, P1[:3, 3] + r * a1
        v = stock.cut_ball(c0, c1, r)
        if model.get("flute", 0.0) > 0:                  # the fluted cylinder above the ball centre cuts too
            v += stock.cut_cylinder(c0, c1, a0, r, model["flute"], 0.0, a1)
        return v
    if k == CYLINDER:
        return stock.cut_cylinder(P0[:3, 3], P1[:3, 3], a0, model["r"], model["length"], model["corner_r"], a1)
    if k == DISC:
        return stock.cut_disc(P0[:3, 3], P1[:3, 3], P0[:3, 2], model["R"], model["kerf"], P1[:3, 2])
    if k == WIRE:
        (wa0, wb0), (wa1, wb1) = wire_ends(model, P0), wire_ends(model, P1)
        return stock.cut_wire(wa0, wb0, wa1, wb1, model["r"])
    raise ValueError("unknown tool model " + str(k))


def wire_ends(model, P: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """Ends of the cutting wire (between the pulleys) for a tool0 pose P (mm)."""
    h, d = model["half"], model["depth"]
    return P[:3, :3] @ np.array([-h, 0.0, d]) + P[:3, 3], P[:3, :3] @ np.array([h, 0.0, d]) + P[:3, 3]


def wire_contact(stock: SdfStock, a: np.ndarray, b: np.ndarray) -> float:
    """Length (mm) of the wire segment ab inside the stock: midpoints at spacing <= cell/2, nearest-sample sign."""
    L = float(np.linalg.norm(b - a))
    n = max(int(math.ceil(L / (0.5 * stock.cell))), 1)
    t = (np.arange(n) + 0.5) / n
    p = a + t[:, None] * (b - a)
    idx = np.round((p - stock.origin) / stock.cell).astype(np.int64)
    inside = np.all((idx >= 0) & (idx < np.array(stock.shape)), axis=1)
    occ = np.zeros(n, bool)
    ii = idx[inside]
    occ[inside] = stock.sdf[ii[:, 0], ii[:, 1], ii[:, 2]] < 0
    return float(occ.sum()) * L / n


# ---------------------------------------------------------------- stock run

@dataclass
class StepCut:
    index: int                 # global sample index (into Expansion.joints())
    segment: int
    tool: str
    kind: str
    removed: float             # mm3
    length: float              # tool reference point travel, mm
    p0: np.ndarray             # tool tip (ball: tip) at the start / end, work frame mm
    p1: np.ndarray
    axis: np.ndarray           # tool axis tip -> holder at the end (disc: blade normal)
    q: np.ndarray              # joints at the end sample
    contact: float = 0.0       # wire: length in the stock before the cut, mm
    wire_span: float = 0.0     # wire: pulley span, mm
    ok: bool = True            # IK converged at the end sample


@dataclass
class StockRun:
    frames: List[Tuple[int, np.ndarray, np.ndarray]]   # (sample index, V mm, F) in the work frame; index -1 = uncut
    frame_volumes: List[float]                          # stock volume at each frame, mm3
    steps: List[StepCut]
    removed: np.ndarray                                 # removed volume per step, mm3
    total_removed: float
    stock: SdfStock                                     # final stock
    initial_volume: float
    cell: float
    seconds: float
    messages: List[str] = field(default_factory=list)

    def frame_at(self, sample: int) -> int:
        """Position in `frames` of the latest snapshot at or before `sample`."""
        idx = [f[0] for f in self.frames]
        return max(int(np.searchsorted(idx, sample, side="right")) - 1, 0)


def _turned_work(station: program.Station, yaw: float) -> np.ndarray:
    Rz = np.eye(4)
    c, s = math.cos(yaw), math.sin(yaw)
    Rz[:3, :3] = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    return station.work_frame() @ Rz


def tool_pose_mm(robot, seg: program.Segment, q, W_inv: np.ndarray) -> np.ndarray:
    """TCP pose of sample q in the turned work frame, translation in mm (FK with the segment's TCP offset)."""
    T = W_inv @ (robot.link_transforms(q)[robot.tcp_link] @ seg.tcp_offset)
    T = T.copy()
    T[:3, 3] *= 1000.0
    return T


def stock_box(station: program.Station, cell_mm: float) -> SdfStock:
    s = station.stock
    if s is None:
        raise ValueError("the station has no stock")
    return SdfStock.from_box([s["xmin"], s["ymin"], s["zmin"]], [s["xmax"], s["ymax"], s["zmax"]], cell_mm)


def simulate_stock(station: program.Station, robot, expansion: program.Expansion, tools: Dict[str, object],
                   cell_mm: float = 2.0, snapshots: int = 40,
                   progress: Optional[Callable[[int, int, str], None]] = None) -> StockRun:
    """Cut the station's stock box along every feed sample of the expansion. See the module text."""
    t0 = time.perf_counter()
    stock = stock_box(station, cell_mm)
    v0 = stock.volume()
    msgs: List[str] = []
    models: Dict[str, Optional[Dict[str, object]]] = {}
    # the cut list: (global index, segment, k) of every feed sample with a previous sample in its segment
    plan = []
    base = 0
    for si, seg in enumerate(expansion.segments):
        n = len(seg.q)
        if n and seg.feed.any():
            if seg.tool not in models:
                t = tools.get(seg.tool)
                try:
                    if t is None:
                        raise ValueError("tool '{}' is not in the library".format(seg.tool))
                    models[seg.tool] = tool_model(t, station.robot, robot)
                    if models[seg.tool]["kind"] == WIRE:
                        msgs.append("wire bow: " + models[seg.tool]["bow_source"])
                except ValueError as e:
                    models[seg.tool] = None
                    msgs.append("stock: {}; its cuts are not simulated".format(e))
            if models[seg.tool] is not None:
                plan += [(base + k, si, k) for k in range(1, n) if seg.feed[k]]
        base += n
    total = len(plan)
    snap_at = set(np.unique(np.round(np.linspace(0, total - 1, max(int(snapshots), 1))).astype(int)).tolist()) \
        if total else set()
    V, F = stock.mesh()
    frames = [(-1, V, F)]
    vols = [v0]
    steps: List[StepCut] = []
    cache: Dict[Tuple[int, int], np.ndarray] = {}
    W_inv: Dict[int, np.ndarray] = {}
    bad = 0

    def pose(si, k):
        key = (si, k)
        if key not in cache:
            seg = expansion.segments[si]
            if si not in W_inv:
                W_inv[si] = np.linalg.inv(_turned_work(station, seg.yaw))
            cache[key] = tool_pose_mm(robot, seg, seg.q[k], W_inv[si])
        return cache[key]

    for n_done, (gi, si, k) in enumerate(plan):
        seg = expansion.segments[si]
        m = models[seg.tool]
        P0, P1 = pose(si, k - 1), pose(si, k)
        cache.pop((si, k - 1), None)
        contact = 0.0
        if m["kind"] == WIRE:
            contact = wire_contact(stock, *wire_ends(m, P1))
        removed = cut_step(stock, m, P0, P1)
        ok = bool(seg.ok[k])
        bad += 0 if ok else 1
        ax = P1[:3, 2].copy() if m["kind"] == DISC else -P1[:3, 2]
        steps.append(StepCut(gi, si, seg.tool, m["kind"], removed, float(np.linalg.norm(P1[:3, 3] - P0[:3, 3])),
                             P0[:3, 3].copy(), P1[:3, 3].copy(), ax, np.asarray(seg.q[k], float).copy(), contact,
                             2 * m["half"] if m["kind"] == WIRE else 0.0, ok))
        if n_done in snap_at:
            V, F = stock.mesh()
            frames.append((gi, V, F))
            vols.append(stock.volume())
        if progress and (n_done % 200 == 0 or n_done == total - 1):
            progress(n_done + 1, total, "stock: {}/{} cuts".format(n_done + 1, total))
    removed = np.array([s.removed for s in steps])
    if bad:
        msgs.append("stock: {} cut samples had no converged IK; cut where the forward kinematics puts the tool".format(
            bad))
    msgs.append("stock: {} cuts on a {:g} mm grid {}; removed {:.0f} mm3 of {:.0f} mm3 ({:.1f} s)".format(
        total, cell_mm, "x".join(str(v) for v in stock.shape), float(removed.sum()), v0, time.perf_counter() - t0))
    return StockRun(frames, vols, steps, removed, float(removed.sum()), stock, v0, cell_mm,
                    time.perf_counter() - t0, msgs)


# ---------------------------------------------------------------- constants

@dataclass(frozen=True)
class ConstantSpec:
    key: str
    unit: str
    quantity: Optional[str]      # materials_mechanics quantity, None = the caller must give it
    end: str = "low"             # which end of the sourced ranges: the conservative one for the check
    by_robot: bool = False       # looked up by the robot name, not the material
    used_for: str = ""


CONSTANTS: List[ConstantSpec] = [
    ConstantSpec("specific_energy", "J/mm3", "specific_cutting_energy", "high", used_for="milling and wire forces"),
    ConstantSpec("radial_ratio", "Fr/Ft", None, used_for="milling force"),
    ConstantSpec("axial_ratio", "Fa/Ft", None, used_for="milling force"),
    ConstantSpec("joint_stiffness", "Nm/rad", "joint_stiffness", "low", by_robot=True, used_for="TCP deflection"),
    ConstantSpec("E", "GPa", "youngs_modulus", "low", used_for="part check"),
    ConstantSpec("nu", "-", "poissons_ratio", "low", used_for="part check"),
    ConstantSpec("tensile_strength", "MPa", "tensile_strength", "low", used_for="part check"),
    ConstantSpec("wire_tension", "N", "wire_tension", "low", used_for="wire bow"),
    ConstantSpec("normal_ratio", "Fn/Ft", "force_ratio", "high", used_for="wire force"),
    ConstantSpec("wire_speed", "m/s", "wire_speed", "low", used_for="wire force"),
]
# optional: cutter bending (stepped cantilever) needs the cutter's modulus and the flute diameter factor
OPTIONAL = [ConstantSpec("tool_E", "GPa", None, used_for="cutter bending"),
            ConstantSpec("flute_factor", "-", None, used_for="cutter bending")]
SPECS = {c.key: c for c in CONSTANTS + OPTIONAL}


@dataclass
class Sourced:
    value: float
    unit: str
    text: str


def sourced_constants(material: str, robot_name: str = "") -> Dict[str, Sourced]:
    """Constants with a sourced row in lithocam.materials_mechanics: rows of exactly this material first, else rows
    whose material contains it; the conservative end of the ranges (ConstantSpec.end); the rows are named."""
    out: Dict[str, Sourced] = {}
    for c in CONSTANTS:
        if c.quantity is None:
            continue
        name = (robot_name if c.by_robot else material).strip().lower()
        if not name:
            continue
        rows = [r for r in materials_mechanics.TABLE if r["quantity"] == c.quantity and r["low"] is not None
                and r["unit"] == c.unit]
        exact = [r for r in rows if r["material"].lower() == name]
        rows = exact or [r for r in rows if name in r["material"].lower()]
        if not rows:
            continue
        if c.end == "high":
            r = max(rows, key=lambda x: x["high"])
            v = float(r["high"])
        else:
            r = min(rows, key=lambda x: x["low"])
            v = float(r["low"])
        out[c.key] = Sourced(v, c.unit, "{} of {} row{}: '{}' {:g}-{:g} {}, {} ({}); {}".format(
            c.end + " end", len(rows), "" if len(rows) == 1 else "s", r["material"], r["low"], r["high"], r["unit"],
            r["source_type"], r["read"], r["source"]))
    return out


def resolve_constants(material: str, robot_name: str, given: Optional[Dict[str, object]] = None):
    """(values, sources): given values win, then sourced rows; a key given as None or "" is left out (the caller
    refuses the sourced row); absent keys are missing."""
    given = dict(given or {})
    vals: Dict[str, float] = {}
    srcs: Dict[str, str] = {}
    sourced = sourced_constants(material, robot_name)
    for k in SPECS:
        if k in given and (given[k] is None or given[k] == ""):
            continue
        if k in given:
            vals[k] = float(given[k])
            srcs[k] = "given by the caller"
        elif k in sourced:
            vals[k] = sourced[k].value
            srcs[k] = sourced[k].text
    return vals, srcs


# ---------------------------------------------------------------- part check

@dataclass
class PartResult:
    max_von_mises: float          # MPa
    max_principal: float          # MPa
    safety_factor: float          # tensile strength / max principal
    flagged_count: int
    peak_point: np.ndarray        # mm, work frame, centre of the peak voxel
    flagged_points: np.ndarray    # (m, 3) mm, centres of voxels above the tensile strength
    voxels: int
    cell: float                   # FEA voxel size, mm
    origin: np.ndarray            # corner of the FEA grid, mm
    von_mises: np.ndarray         # (nx, ny, nz), NaN on empty voxels
    principal: np.ndarray         # (nx, ny, nz), NaN on empty voxels
    force: np.ndarray             # N, on the part
    point: np.ndarray             # mm
    converged: bool
    iterations: int
    seconds: float


def coarsen(stock: SdfStock, max_voxels: int = 20000):
    """Occupancy voxels of the stock, merged f x f x f until at most max_voxels are occupied (a block is material
    when at least half of it is). Returns (occupancy, grid corner mm, voxel size mm). A sample at origin + i*cell is
    the centre of voxel i."""
    occ = stock.occupancy()
    f = 1
    while True:
        if f == 1:
            o = occ
        else:
            s = [int(math.ceil(n / f)) * f for n in occ.shape]
            p = np.zeros(s, np.float32)
            p[:occ.shape[0], :occ.shape[1], :occ.shape[2]] = occ
            o = p.reshape(s[0] // f, f, s[1] // f, f, s[2] // f, f).mean(axis=(1, 3, 5)) >= 0.5
        if int(o.sum()) <= max_voxels or f > 64:
            break
        f += 1
    corner = stock.origin - 0.5 * stock.cell
    return o, corner, stock.cell * f


def check_part(stock: SdfStock, force, point, E_mpa: float, nu: float, tensile_mpa: float,
               load_radius: Optional[float] = None, max_voxels: int = 20000, tol: float = 1e-6,
               max_iter: int = 4000) -> PartResult:
    """Voxel FEA of the stock (N, mm, MPa): clamped on its bottom face, `force` (N) spread over the surface nodes
    within load_radius of `point` (mm; grown until a surface node is found, last the nearest node)."""
    from ..fea import VoxelFEA
    t0 = time.perf_counter()
    occ, corner, h = coarsen(stock, max_voxels)
    if not occ.any():
        raise ValueError("no material left for the part check")
    m = VoxelFEA(occ, corner, h, E_mpa, nu)
    zs = np.nonzero(occ.any(axis=(0, 1)))[0]
    z_fix = corner[2] + zs[0] * h + 0.25 * h
    fixed = m.empty_fixed()
    m.fix_where(fixed, lambda x, y, z: z <= z_fix)
    loads = m.empty_loads()
    r = float(load_radius) if load_radius else 2.0 * h
    for _ in range(4):
        try:
            m.add_sphere_surface_load(loads, point, r, force)
            break
        except ValueError:
            r *= 2
    else:
        m.add_point_load(loads, point, force)
    res = m.solve(loads, fixed, tol=tol, max_iter=max_iter)
    pr, vm = res.max_principal, res.von_mises
    mx = float(pr.max())
    centres = corner + (m.el_ijk + 0.5) * h
    flag = pr > tensile_mpa
    return PartResult(float(vm.max()), mx, float(tensile_mpa / mx) if mx > 0 else float("inf"), int(flag.sum()),
                      centres[int(np.argmax(pr))], centres[flag], m.n_el, h, corner,
                      m.element_field_to_grid(vm), m.element_field_to_grid(pr), np.asarray(force, float),
                      np.asarray(point, float), bool(res.converged), int(res.iterations), time.perf_counter() - t0)


def ramp(t: np.ndarray) -> np.ndarray:
    """Blue (0) to green to red (1), as fea.VoxelFEA.voxel_colors."""
    t = np.clip(np.asarray(t, float), 0.0, 1.0)
    return np.stack([t, 1 - np.abs(2 * t - 1), 1 - t], axis=1)


def stress_colors(V: np.ndarray, part: PartResult, which: str = "principal",
                  vmax: Optional[float] = None) -> Tuple[np.ndarray, float]:
    """Per-vertex colours of mesh vertices V (mm) from the part check's voxel stress (blue 0 to red vmax, default the
    largest value). Empty voxels take the largest neighbouring value. Returns (colours (n, 3), vmax)."""
    g = np.array(part.principal if which == "principal" else part.von_mises, dtype=float)
    g = np.where(np.isnan(g), np.nan, np.maximum(g, 0.0))
    for _ in range(3):                                   # fill a few empty layers from the material
        pad = np.pad(g, 1, constant_values=np.nan)
        nb = np.stack([pad[2:, 1:-1, 1:-1], pad[:-2, 1:-1, 1:-1], pad[1:-1, 2:, 1:-1], pad[1:-1, :-2, 1:-1],
                       pad[1:-1, 1:-1, 2:], pad[1:-1, 1:-1, :-2]])
        with np.errstate(all="ignore"):
            fill = np.nanmax(np.where(np.isnan(nb), -np.inf, nb), axis=0)
        fill = np.where(np.isfinite(fill), fill, np.nan)
        g = np.where(np.isnan(g), fill, g)
    idx = np.floor((np.asarray(V, float) - part.origin) / part.cell).astype(np.int64)
    idx = np.clip(idx, 0, np.array(g.shape) - 1)
    val = g[idx[:, 0], idx[:, 1], idx[:, 2]]
    val = np.where(np.isnan(val), 0.0, val)
    top = float(vmax) if vmax else float(np.nanmax(g)) if np.isfinite(np.nanmax(g)) else 0.0
    return ramp(val / top if top > 0 else np.zeros_like(val)), top


# ---------------------------------------------------------------- process check

@dataclass
class ProcessReport:
    constants: Dict[str, float]
    sources: Dict[str, str]
    messages: List[str]
    steps: List[Dict[str, object]]
    part: Optional[PartResult] = None
    colors: Optional[np.ndarray] = None       # per vertex of the final stock mesh (run.frames[-1])
    color_max: float = 0.0
    seconds: float = 0.0

    def text(self) -> str:
        return "\n".join(self.messages)


def _feeds_from_records(expansion: program.Expansion) -> Dict[Tuple[float, ...], Tuple[Optional[float], float]]:
    """Joint vector -> (feed mm/s, spindle rpm) of every cutting move in the post records."""
    out = {}
    rpm = 0.0
    for r in expansion.records:
        if r.kind == "spindle":
            rpm = float(r.data.get("rpm") or 0.0)
        elif r.kind == "movel" and r.data.get("cut"):
            v = r.data.get("v")
            out[tuple(float(x) for x in r.data["q"])] = (float(v) if v else None, rpm)
    return out


def _missing(vals, keys) -> List[str]:
    return [k for k in keys if k not in vals]


def process_check(station: program.Station, run: StockRun, expansion: program.Expansion, robot,
                  tools: Dict[str, object], constants: Optional[Dict[str, object]] = None,
                  max_voxels: int = 20000, progress: Optional[Callable[[str], None]] = None) -> ProcessReport:
    """Per-step forces, deflections and wire loads, and a part check of the final stock. See the module text.

    constants: {key: value} in the units of CONSTANTS (J/mm3, GPa, MPa, N, m/s, Nm/rad, ratios); absent keys come
    from sourced rows for the station material and robot; a check whose constant is still missing is skipped."""
    t0 = time.perf_counter()
    vals, srcs = resolve_constants(station.material, station.robot, constants)
    msgs: List[str] = ["Process check: {} ({}), {} cut steps".format(station.material, station.robot,
                                                                     len(run.steps))]
    for k in SPECS:
        if k in vals:
            msgs.append("  {} = {:g} {}: {}".format(k, vals[k], SPECS[k].unit, srcs[k]))
    feeds = _feeds_from_records(expansion)
    mill_need = _missing(vals, ["specific_energy", "radial_ratio", "axial_ratio"])
    wire_need = _missing(vals, ["specific_energy", "wire_speed", "normal_ratio"])
    if mill_need:
        msgs.append("SKIPPED milling force: missing {}".format(", ".join(mill_need)))
    if "joint_stiffness" not in vals:
        msgs.append("SKIPPED TCP deflection: missing joint_stiffness (no sourced row for '{}')".format(station.robot))
    bend_need = _missing(vals, ["tool_E", "flute_factor"])
    if bend_need:
        msgs.append("SKIPPED cutter bending: missing {}".format(", ".join(bend_need)))
    has_wire = any(s.kind == WIRE for s in run.steps)
    if has_wire and wire_need:
        msgs.append("SKIPPED wire force: missing {}".format(", ".join(wire_need)))
    if has_wire and "wire_tension" not in vals:
        msgs.append("SKIPPED wire bow: missing wire_tension")
    no_feed = no_rpm = plunges = 0
    rows: List[Dict[str, object]] = []
    saved_tcp = robot.tcp_offset
    try:
        for s in run.steps:
            row: Dict[str, object] = {"index": s.index, "kind": s.kind, "tool": s.tool, "removed_mm3": s.removed}
            v, rpm = feeds.get(tuple(float(x) for x in s.q), (None, 0.0))
            row["feed_mm_s"], row["rpm"] = v, rpm
            if v and s.length > 0:
                row["time_s"] = s.length / v
            seg = expansion.segments[s.segment]
            tool = tools.get(s.tool)
            W = _turned_work(station, seg.yaw)
            if s.kind in (BALL, CYLINDER, DISC):
                if not mill_need:
                    if not v:
                        no_feed += 1
                    elif not rpm:
                        no_rpm += 1
                    elif s.length > 0 and s.removed > 0:
                        try:
                            mf = interaction.milling_forces(
                                s.removed * 1e-9, s.length / v, vals["specific_energy"] * 1e9,
                                float(tool.diameter) / 1000, rpm, s.p1 - s.p0, s.axis, vals["radial_ratio"],
                                vals["axial_ratio"])
                        except ValueError:
                            plunges += 1
                            mf = None
                        if mf is not None:
                            row.update(Ft=mf.Ft, Fr=mf.Fr, Fa=mf.Fa, F=mf.force_on_tool, power_W=mf.power,
                                       Fmag=float(np.linalg.norm(mf.force_on_tool)))
                            if "joint_stiffness" in vals:
                                robot.tcp_offset = seg.tcp_offset
                                J = robot.jacobian(s.q)
                                k = np.full(robot.dof, vals["joint_stiffness"])
                                Fb = W[:3, :3] @ mf.force_on_tool
                                d = interaction.tcp_deflection(J, k, np.r_[Fb, 0.0, 0.0, 0.0])
                                row["deflection_mm"] = float(np.linalg.norm(d[:3])) * 1000
                            if not bend_need and s.kind != DISC:
                                parts = spindles.cutter_parts(tool, spindles.stickout(tool))
                                shank = [p for p in parts if p[0] == "shank"]
                                flute = [p for p in parts if p[0] == "flutes"][0]
                                Fl = float(np.linalg.norm(mf.force_on_tool - (mf.force_on_tool @ s.axis) * s.axis))
                                row["bending_mm"] = float(interaction.stepped_cantilever_tip_deflection(
                                    Fl, vals["tool_E"] * 1e3, shank[0][1] if shank else flute[1],
                                    shank[0][3] if shank else 0.0, vals["flute_factor"], flute[3], flute[1]))
            elif s.kind == WIRE and s.contact > 0:
                if not wire_need:
                    if not v:
                        no_feed += 1
                    else:
                        wf = interaction.wire_forces(vals["specific_energy"] * 1e9, float(tool.diameter) / 1000,
                                                     s.contact / 1000, v / 1000, vals["wire_speed"],
                                                     vals["normal_ratio"])
                        d = s.p1 - s.p0
                        n = d / np.linalg.norm(d) if np.linalg.norm(d) > 0 else np.zeros(3)
                        row.update(Ft=wf.Ft, Fn=wf.Fn, power_W=wf.power, Fmag=float(math.hypot(wf.Ft, wf.Fn)),
                                   F=-wf.Fn * n, contact_mm=s.contact)
                        if "wire_tension" in vals:
                            b = interaction.wire_bow(wf.Fn, s.wire_span / 1000, min(s.contact, s.wire_span) / 1000,
                                                     vals["wire_tension"])
                            row.update(bow_mm=b.max_deflection * 1000, lag_deg=math.degrees(b.lag_angle))
            rows.append(row)
    finally:
        robot.tcp_offset = saved_tcp
    if no_feed:
        msgs.append("  {} steps without a feed in the post records: no force".format(no_feed))
    if no_rpm:
        msgs.append("  {} steps without a spindle speed: no milling force".format(no_rpm))
    if plunges:
        msgs.append("  {} steps feed along the tool axis (plunge): the tangential direction is undefined, no force"
                    .format(plunges))
    times = [r["time_s"] for r in rows if "time_s" in r]
    if times:
        msgs.append("  cutting time {:.1f} s over {} timed steps".format(sum(times), len(times)))
    forced = [r for r in rows if "Fmag" in r]
    peak = max(forced, key=lambda r: r["Fmag"]) if forced else None
    if peak is not None:
        msgs.append("  peak force {:.1f} N at sample {} ({} {}, removed {:.2f} mm3, feed {:.1f} mm/s, {} rpm), "
                    "peak power {:.0f} W".format(peak["Fmag"], peak["index"], peak["kind"], peak["tool"],
                                                 peak["removed_mm3"], peak["feed_mm_s"] or 0, peak["rpm"] or 0,
                                                 max(r.get("power_W", 0.0) for r in forced)))
    dfl = [r["deflection_mm"] for r in rows if "deflection_mm" in r]
    if dfl:
        msgs.append("  TCP deflection (joint stiffness only): max {:.4f} mm, median {:.4f} mm".format(
            max(dfl), float(np.median(dfl))))
    bnd = [r["bending_mm"] for r in rows if "bending_mm" in r]
    if bnd:
        msgs.append("  cutter bending: max {:.4f} mm".format(max(bnd)))
    bows = [r for r in rows if "bow_mm" in r]
    if bows:
        b = max(bows, key=lambda r: r["bow_mm"])
        msgs.append("  wire: max Fn {:.1f} N, max bow {:.2f} mm, lag {:.2f} deg (contact {:.0f} mm)".format(
            max(r["Fn"] for r in bows), b["bow_mm"], b["lag_deg"], b["contact_mm"]))
        steep = sum(1 for r in bows if math.tan(math.radians(r["lag_deg"])) > 0.1)
        if steep:
            msgs.append("  WARNING wire: {} of {} steps have an end slope above 0.1, outside the small-slope model of "
                        "interaction.wire_bow; the bow values there only say the wire cannot hold that load at this "
                        "feed".format(steep, len(bows)))
    rep = ProcessReport(vals, srcs, msgs, rows)
    part_need = _missing(vals, ["E", "nu", "tensile_strength"])
    if part_need:
        msgs.append("SKIPPED part check: missing {}".format(", ".join(part_need)))
    elif peak is None:
        msgs.append("SKIPPED part check: no step force (the force checks were skipped or found no cut)")
    else:
        if progress:
            progress("part check (voxel FEA) ...")
        s = next(x for x in run.steps if x.index == peak["index"])
        F_part = -np.asarray(peak["F"], float)                 # the part carries the reaction of the tool force
        tool = tools.get(s.tool)
        rad = max(float(getattr(tool, "diameter", 0.0) or 0.0) / 2, 2 * run.cell)
        try:
            part = check_part(run.stock, F_part, s.p1, vals["E"] * 1e3, vals["nu"], vals["tensile_strength"],
                              load_radius=rad, max_voxels=max_voxels)
        except ValueError as e:
            msgs.append("SKIPPED part check: {}".format(e))
        else:
            rep.part = part
            V = run.frames[-1][1]
            rep.colors, rep.color_max = stress_colors(V, part, vmax=vals["tensile_strength"])
            msgs.append("  part check: {} voxels of {:g} mm, clamped at the base, {:.1f} N at ({:.1f}, {:.1f}, {:.1f})"
                        " mm; max von Mises {:.3f} MPa, max principal {:.3f} MPa, safety factor {:.3g} on {:g} MPa; "
                        "{} voxels above the strength; peak at ({:.1f}, {:.1f}, {:.1f}) mm; CG {} in {} iterations "
                        "({:.1f} s)".format(part.voxels, part.cell, float(np.linalg.norm(F_part)), *s.p1,
                                            part.max_von_mises, part.max_principal, part.safety_factor,
                                            vals["tensile_strength"], part.flagged_count, *part.peak_point,
                                            "converged" if part.converged else "NOT converged", part.iterations,
                                            part.seconds))
            msgs.append("  colours: max principal stress, blue 0 MPa to red at the tensile strength {:g} MPa and above"
                        .format(rep.color_max))
    rep.seconds = time.perf_counter() - t0
    msgs.append("Process check done in {:.1f} s".format(rep.seconds))
    return rep
