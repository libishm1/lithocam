# Copyright 2026 The lithocam authors. SPDX-License-Identifier: Apache-2.0
"""Unified material model, engagement-based feed limit and diamond tool wear.

Every physical value comes from a sourced row (cutting_data.TABLE or materials_mechanics.TABLE, with its source
shown) or from the caller. A missing value makes a check say so and skip. Nothing is defaulted silently. This
module defines no physical constant. See docs/MATERIALS.md for the models and their limits.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np

from . import cutting_data, interaction, materials_mechanics

# ------------------------------------------------------------------ names

# name -> (family, materials_mechanics name terms, cutting_data material, aliases)
_DEFS: Dict[str, Tuple[str, List[str], str, List[str]]] = {
    "granite": ("stone", ["granite"], "granite", []),
    "marble": ("stone", ["marble"], "marble", []),
    "limestone": ("stone", ["limestone"], "limestone", []),
    "sandstone": ("stone", ["sandstone"], "sandstone", []),
    "travertine": ("stone", ["travertine"], "travertine", []),
    "quartzite": ("stone", ["quartzite"], "quartzite", []),
    "slate": ("stone", ["slate"], "slate", []),
    "basalt": ("stone", ["basalt"], "basalt", []),
    "andesite": ("stone", ["andesite"], "andesite", []),
    "gabbro": ("stone", ["gabbro"], "gabbro", []),
    "diorite": ("stone", ["diorite"], "diorite", []),
    "quartz_engineered": ("stone", ["quartz"], "quartz_engineered", ["engineered quartz", "quartz"]),
    "oak": ("wood", ["oak"], "hardwood", ["white oak", "red oak"]),
    "beech": ("wood", ["beech"], "hardwood", ["european beech", "american beech"]),
    "pine": ("wood", ["pine"], "softwood", ["scots pine", "white pine"]),
    "spruce": ("wood", ["spruce"], "softwood", ["norway spruce", "sitka spruce"]),
    "hardwood": ("wood", ["hardwood"], "hardwood", []),
    "softwood": ("wood", ["softwood"], "softwood", []),
    "plywood": ("wood", ["plywood"], "plywood", []),
    "mdf": ("wood", ["mdf"], "mdf", []),
    "particleboard": ("wood", ["particleboard"], "particleboard", ["chipboard"]),
    "hpl": ("wood", ["hpl"], "hpl", []),
}
_ALIAS = {a.replace(" ", "_"): n for n, (_, _, _, al) in _DEFS.items() for a in al + [n]}

# quantity: preferred unit (rows in other units are not mixed in) and the conservative end of a range
_UNIT = {"youngs_modulus": "GPa", "density": "kg/m3", "specific_cutting_energy": "J/mm3",
         "specific_cutting_force": "N/mm2", "tensile_strength": "MPa", "compressive_strength": "MPa",
         "flexural_strength": "MPa", "wire_tension": "N", "wire_speed": "m/s", "g_ratio": "mm3/mm3",
         "wear_per_area": "mm/m2"}
_CONSERVATIVE = {"specific_cutting_energy": "high", "specific_cutting_force": "high", "force_ratio": "high",
                 "wear_per_area": "high", "wear_ratio": "low", "g_ratio": "low"}
# the quantity names materials_mechanics rows must use for the wear model (docs/MATERIALS.md)
WEAR_QUANTITIES = ("g_ratio", "wear_per_area")
MECH_QUANTITIES = ["youngs_modulus", "poissons_ratio", "density", "tensile_strength", "compressive_strength",
                   "flexural_strength", "specific_cutting_energy", "specific_cutting_force", "sawing_rate",
                   "wire_tension", "wire_speed", "bead_normal_force", "bead_tangential_force", "force_ratio",
                   "g_ratio", "wear_per_area"]
CUT_QUANTITIES = ["surface_speed", "rpm", "feed", "plunge_feed", "chip_load", "depth", "water", "cut_rate",
                  "tension"]
GRIT_TOOL = "polishing_pad"


def _norm(name: str) -> str:
    return name.strip().lower().replace("-", "_").replace(" ", "_")


def _row_mech(r: Dict) -> Dict:
    d = dict(r)
    d["table"] = "materials_mechanics"
    d["tool"] = ""
    d["size"] = ""
    return d


def _row_cut(e: cutting_data.Entry) -> Dict:
    return dict(material=e.material, quantity=e.quantity, low=e.lo, high=e.hi, unit=e.unit, source=e.source,
                source_type=e.source_type, read="page read" if e.confidence == "page" else "snippet", note=e.note,
                table="cutting_data", tool=e.tool, size=e.size)


def row_text(row: Dict) -> str:
    """One line naming a row and its source."""
    lo, hi = row["low"], row["high"]
    rng = "{:g}".format(lo) if lo == hi else "{:g}-{:g}".format(lo, hi)
    where = row["table"] + (" " + row["tool"] if row.get("tool") else "") + (" " + row["size"] if row.get("size") else "")
    return "'{}' {} {} {}, {} ({}), {}; {}".format(row["material"], row["quantity"], rng, row["unit"],
                                                  row["source_type"], row["read"], row["source"], where)


def _has_value(r: Dict) -> bool:
    return r["low"] is not None and r["high"] is not None


def _is_page(r: Dict) -> bool:
    return r["read"].startswith("page")


# --------------------------------------------------------------- Material


@dataclass
class Material:
    name: str
    family: str                                   # stone | wood
    mechanical: Dict[str, List[Dict]]             # quantity -> rows (rows with low None record "nothing found")
    cutting: List[Dict]                           # cutting_data rows for the material
    grit_rows: List[Dict]                         # polishing pad rows (grit sequence lives in the notes)
    water_rows: List[Dict]                        # water need rows
    sources: List[str]
    cutting_material: str = ""

    def rows(self, quantity: str, tool: Optional[str] = None, diameter_mm: float = 0.0) -> List[Dict]:
        """Rows for a quantity, valued ones only. Cutting quantities need `tool` (a tool kind name)."""
        out = [r for r in self.mechanical.get(quantity, []) if _has_value(r)
               and (quantity not in _UNIT or r["unit"] == _UNIT[quantity])]
        if tool:
            cut = [r for r in self.cutting if r["quantity"] == quantity and r["tool"] == tool
                   and _fits_row(r, diameter_mm)]
            if diameter_mm > 0 and any(r["size"] for r in cut):
                # sized rows: keep the nearest size only, as cutting_data.suggest does; unsized rows go last
                def dist(r):
                    s = cutting_data._size_mm(r["size"]) if r["size"] else None
                    return abs(s - diameter_mm) if s is not None else float("inf")
                best = min(dist(r) for r in cut)
                cut = [r for r in cut if dist(r) == best] if best != float("inf") else cut
            out += cut
        return out

    def quantities(self) -> List[str]:
        return sorted(set(self.mechanical) | {r["quantity"] for r in self.cutting})


def _fits_row(r: Dict, d_mm: float) -> bool:
    if d_mm <= 0 or not r.get("size"):
        return True
    e = cutting_data.Entry(r["material"], r["tool"], r["quantity"], r["low"], r["high"], r["unit"], r["source"],
                           r["source_type"], "page", r["size"], r["note"])
    return cutting_data._fits(e, d_mm)


def _mech_rows(terms: Sequence[str]) -> Dict[str, List[Dict]]:
    out: Dict[str, List[Dict]] = {}
    qs = sorted({r["quantity"] for r in materials_mechanics.TABLE})
    for q in qs:
        rows = [r for r in materials_mechanics.TABLE if r["quantity"] == q]
        exact = [r for r in rows if r["material"].lower() in terms]
        hit = exact or [r for r in rows if any(t in r["material"].lower() for t in terms)]
        # rows that only record a miss stay listed so the report can say "no value found"
        if hit:
            out[q] = [_row_mech(r) for r in hit]
    return out


def _build(name: str) -> Material:
    family, terms, cmat, _ = _DEFS[name]
    mech = _mech_rows(terms)
    cut = [_row_cut(e) for e in cutting_data.TABLE if e.material == cmat]
    srcs = sorted({r["source"] for rs in mech.values() for r in rs if r["source"] and _has_value(r)}
                  | {r["source"] for r in cut})
    return Material(name, family, mech, cut, [r for r in cut if r["tool"] == GRIT_TOOL],
                    [r for r in cut if r["quantity"] == "water"], srcs, cmat)


def available() -> List[str]:
    """Names with at least one valued row in either table. This is the studio's material selector list."""
    out = []
    for n in _DEFS:
        m = _build(n)
        if m.cutting or any(_has_value(r) for rs in m.mechanical.values() for r in rs):
            out.append(n)
    return out


def get(name: str) -> Material:
    """The material by name or alias (granite, marble, limestone, sandstone, travertine, slate, quartzite,
    basalt, oak, beech, pine, spruce, mdf, ...). KeyError names the choices when no row exists."""
    key = _ALIAS.get(_norm(name))
    if key is None:
        raise KeyError("unknown material '{}'; available: {}".format(name, ", ".join(available())))
    m = _build(key)
    if not (m.cutting or any(_has_value(r) for rs in m.mechanical.values() for r in rs)):
        raise KeyError("no sourced row for '{}' in cutting_data or materials_mechanics; available: {}".format(
            key, ", ".join(available())))
    return m


def _material(material) -> Optional[Material]:
    if isinstance(material, Material):
        return material
    try:
        return get(material)
    except KeyError:
        return None


def value(material, quantity: str, which: str = "conservative", tool: Optional[str] = None,
          diameter_mm: float = 0.0) -> Optional[Tuple[float, Dict]]:
    """(value, source row) or None when no sourced row has a value.

    which: 'low' or 'high' end of the sourced ranges, or 'conservative' (the end that gives the safer check:
    high for cutting energy and force ratio, low otherwise; see _CONSERVATIVE). The row is the one the value
    came from. Cutting quantities (feed, rpm, chip_load, water, ...) need `tool`, a tool kind name. Rows in
    another unit than the quantity's preferred one are left out, so units never mix."""
    m = _material(material)
    if m is None:
        return None
    rows = m.rows(quantity, tool, diameter_mm)
    if not rows:
        return None
    end = _CONSERVATIVE.get(quantity, "low") if which == "conservative" else which
    if end not in ("low", "high"):
        raise ValueError("which must be 'conservative', 'low' or 'high'")
    if end == "high":
        r = max(rows, key=lambda x: (x["high"], _is_page(x)))
        return float(r["high"]), r
    r = min(rows, key=lambda x: (x["low"], not _is_page(x)))
    return float(r["low"]), r


# ----------------------------------------------------------------- report


def status(m: Material, quantity: str) -> Tuple[str, str]:
    """('sourced' | 'snippet' | 'missing', detail). sourced: at least one page-read row with a value."""
    rows = [r for r in m.mechanical.get(quantity, []) if _has_value(r)] + \
           [r for r in m.cutting if r["quantity"] == quantity]
    if not rows:
        miss = m.mechanical.get(quantity, [])
        return "missing", ("a row records that nothing was found" if miss else "no row")
    tools = sorted({r["tool"] for r in rows if r["tool"]})
    page = [r for r in rows if _is_page(r)]
    detail = "{} row{}{}".format(len(rows), "" if len(rows) == 1 else "s", ", tools: " + ", ".join(tools) if tools else "")
    return ("sourced" if page else "snippet"), detail


def report_table(names: Optional[Iterable[str]] = None) -> List[Dict]:
    out = []
    for n in (list(names) if names else available()):
        m = get(n)
        for q in MECH_QUANTITIES + CUT_QUANTITIES:
            s, d = status(m, q)
            out.append(dict(material=n, family=m.family, quantity=q, status=s, detail=d))
        out.append(dict(material=n, family=m.family, quantity="grit_sequence",
                        status=("sourced" if any(_is_page(r) for r in m.grit_rows) else
                                "snippet" if m.grit_rows else "missing"),
                        detail="{} polishing_pad row(s); the grit steps are in the row notes".format(len(m.grit_rows))))
    return out


def report(names: Optional[Iterable[str]] = None) -> str:
    """Per material, which quantities are sourced (page read), snippet-only or missing."""
    lines = []
    tab = report_table(names)
    for n in dict.fromkeys(t["material"] for t in tab):
        rows = [t for t in tab if t["material"] == n]
        lines.append("{} ({})".format(n, rows[0]["family"]))
        for key, label in (("sourced", "sourced"), ("snippet", "snippet only"), ("missing", "missing")):
            qs = [t["quantity"] for t in rows if t["status"] == key]
            lines.append("  {}: {}".format(label, ", ".join(qs) if qs else "none"))
    return "\n".join(lines)


# -------------------------------------------------------------- feed limit


@dataclass
class Engagement:
    """Cut cross-section of one step. ae radial and ap axial engagement in mm; either may be None when only the
    area is known (the power and force checks need only the area)."""
    ae: Optional[float] = None
    ap: Optional[float] = None
    area_mm2: Optional[float] = None
    notes: List[str] = field(default_factory=list)

    def __post_init__(self):
        if self.area_mm2 is None and self.ae is not None and self.ap is not None:
            self.area_mm2 = float(self.ae) * float(self.ap)


def engagement_from_stock(removed_mm3: float, step_length_mm: float, tool_diameter_mm: float = 0.0,
                          ap_mm: Optional[float] = None, ae_mm: Optional[float] = None) -> Engagement:
    """Engagement of one step from the stock simulation (studio/process StepCut.removed and .length).

    The removed volume divided by the path length of the step is the cut cross-section A = ae * ap. A alone fixes
    only the product, so give one of ap (axial depth, from the program) or ae (stepover); the other follows. With
    neither, only the area is returned. Limits: the removed volume is voxel-quantised (error about one cell per
    face), so short steps are noisy; sum several steps first. Overlap with earlier passes is already out of the
    stock, which is what is wanted: it is the material actually cut."""
    if step_length_mm <= 0:
        raise ValueError("step_length_mm must be > 0")
    if removed_mm3 < 0:
        raise ValueError("removed_mm3 must be >= 0")
    A = float(removed_mm3) / float(step_length_mm)
    eng = Engagement(area_mm2=A)
    if A <= 0:
        eng.notes.append("no material removed in this step")
        return eng
    if ap_mm and ap_mm > 0:
        eng.ap, eng.ae = float(ap_mm), A / float(ap_mm)
    elif ae_mm and ae_mm > 0:
        eng.ae, eng.ap = float(ae_mm), A / float(ae_mm)
    else:
        eng.notes.append("ae and ap not separated: give ap_mm or ae_mm")
    if tool_diameter_mm and eng.ae is not None and eng.ae > tool_diameter_mm * (1 + 1e-9):
        eng.notes.append("ae {:.2f} mm exceeds the tool diameter {:g} mm; the removed volume includes cuts not under "
                         "the tool (check ap or the sample spacing)".format(eng.ae, tool_diameter_mm))
    return eng


def engagements_from_steps(steps: Sequence, tool_diameters: Optional[Dict[str, float]] = None,
                           ap_mm: Optional[float] = None, ae_mm: Optional[float] = None) -> List[Engagement]:
    """One Engagement per StepCut-like object (attributes removed, length, tool). tool_diameters maps tool name to
    diameter in mm. Zero-length steps give an empty Engagement with a note."""
    out = []
    for s in steps:
        if s.length <= 0:
            out.append(Engagement(notes=["zero-length step"]))
            continue
        out.append(engagement_from_stock(s.removed, s.length, (tool_diameters or {}).get(s.tool, 0.0), ap_mm, ae_mm))
    return out


@dataclass
class Candidate:
    name: str                 # power | tool_deflection | robot_deflection | chip_load | sourced_feed
    feed: float               # mm/min
    source: str
    note: str = ""


@dataclass
class FeedLimit:
    feed: Optional[float]                       # mm/min, the smallest candidate; None if no check could run
    binding: str                                # name of the binding candidate, or "none"
    candidates: List[Candidate] = field(default_factory=list)
    skipped: List[Tuple[str, str]] = field(default_factory=list)   # (check, missing constant)
    warnings: List[str] = field(default_factory=list)

    def badge(self, programmed_feed: float) -> str:
        """Short label for a step: how the programmed feed compares with the limit."""
        if self.feed is None:
            return "no limit (" + "; ".join("{}: needs {}".format(a, b) for a, b in self.skipped) + ")"
        r = programmed_feed / self.feed
        if r <= 1.0:
            return "ok, {:.0f}% of the {} limit".format(100 * r, self.binding)
        return "over the {} limit by {:.0f}%".format(self.binding, 100 * (r - 1))

    def text(self) -> str:
        lines = ["feed limit: " + ("none" if self.feed is None else "{:.0f} mm/min, bound by {}".format(
            self.feed, self.binding))]
        for c in self.candidates:
            lines.append("  {}: {:.0f} mm/min. {} {}".format(c.name, c.feed, c.note, c.source).rstrip())
        for a, b in self.skipped:
            lines.append("  skipped {}: missing {}".format(a, b))
        for w in self.warnings:
            lines.append("  warning: " + w)
        return "\n".join(lines)


def _given(constants: Dict[str, Any], key: str):
    """(present, value): a key given as None or '' is a refusal and counts as absent."""
    if key in constants:
        v = constants[key]
        if v is None or v == "":
            return True, None
        return True, float(v)
    return False, None


def _constant(constants, mat, key, quantity=None, which="conservative", unit_scale=1.0):
    """(value, source text) from the caller, else a sourced row, else None."""
    present, v = _given(constants, key)
    if present:
        return (v, "given by the caller") if v is not None else None
    if quantity and mat is not None:
        got = value(mat, quantity, which)
        if got:
            return got[0] * unit_scale, row_text(got[1])
    return None


def feed_limit(tool, material, engagement, rpm: float, spindle=None, robot: Optional[Dict] = None,
               limits: Optional[Dict[str, float]] = None, constants: Optional[Dict[str, Any]] = None,
               geometry: Optional[Dict[str, float]] = None, which: str = "conservative",
               power_fraction: Optional[float] = None) -> FeedLimit:
    """Highest feed the checks allow for one step, and which check binds.

    tool: object with kind (a tool kind name or the lithocam enum), diameter [mm], flutes.
    material: name, Material or None (all constants then come from `constants`).
    engagement: Engagement or (ae, ap) in mm. rpm: spindle speed.
    spindle: object or dict with power_w and max_rpm (0 = not stated).
    robot: optional dict with J (6xn or 3xn, linear rows in m/rad), joint_stiffness (n, Nm/rad) or robot_name (a
      materials_mechanics joint_stiffness row, low end), and optional force_dirs (t_hat, r_hat, a_hat in the base
      frame; without them the largest linear-compliance eigenvalue bounds the deflection).
    limits: tool_deflection_mm and robot_deflection_mm, both user limits.
    constants: caller values, keys specific_energy [J/mm3], radial_ratio, axial_ratio [Fr/Ft, Fa/Ft], tool_E
      [GPa], flute_factor, chip_load [mm/tooth], joint_stiffness. A key given as None refuses the sourced row.
    geometry: shank_diameter_mm, shank_length_mm, flute_length_mm of the cutter (stepped cantilever).
    power_fraction: share of the rated spindle power usable at the cutter; None = the rated power, said in the note.

    Models: P = u A vf / 60 (A = ae ap mm2, u J/mm3, vf mm/min), Ft = P / vc, vc = pi D rpm / 60000 m/s. Power
    limit vf = 60 P / (u A). Deflections are linear in the feed, so the limit is the allowed deflection divided by
    the deflection per mm/min. Chip load: vf = rpm x flutes x chip load x depth_factor(ap / D) (wood).
    Static, linear, quasi-steady; no chatter, no thermal limit, no wear. See docs/MATERIALS.md."""
    constants = dict(constants or {})
    limits = dict(limits or {})
    mat = _material(material) if material is not None else None
    kind = cutting_data.tool_kind_label(tool)
    D = float(getattr(tool, "diameter", 0.0) or 0.0)
    flutes = int(getattr(tool, "flutes", 0) or 0)
    out = FeedLimit(None, "none")
    if rpm <= 0:
        raise ValueError("rpm must be > 0")
    if material is not None and mat is None:
        out.warnings.append("material '{}' has no sourced row; only caller constants are used".format(material))
    if not isinstance(engagement, Engagement):
        ae, ap = engagement
        engagement = Engagement(ae, ap)
    A = engagement.area_mm2
    out.warnings += list(engagement.notes)
    sp_power = _attr(spindle, "power_w")
    sp_rpm = _attr(spindle, "max_rpm")
    if sp_rpm and rpm > sp_rpm:
        out.warnings.append("rpm {:.0f} exceeds the spindle maximum {:.0f}".format(rpm, sp_rpm))
    if A is None or A <= 0:
        out.skipped.append(("all force checks", "engagement area (ae and ap, or removed volume per length)"))
        A = None
    vc = math.pi * D * rpm / 60000.0 if D > 0 else 0.0
    # specific energy: user, then a specific_cutting_energy row, then specific_cutting_force x 1e-3 (N/mm2 = N mm/mm3)
    u = _constant(constants, mat, "specific_energy", "specific_cutting_energy", which)
    if u is None and not _given(constants, "specific_energy")[0]:
        f = _constant(constants, mat, "specific_force", "specific_cutting_force", which)
        if f is not None:
            u = (f[0] * 1e-3, "specific cutting force x 1e-3 (N/mm2 to J/mm3): " + f[1])
    # ---- power
    if A and u and sp_power and vc > 0:
        P = sp_power * (power_fraction if power_fraction else 1.0)
        out.candidates.append(Candidate("power", interaction.power_limited_feed(P, u[0], A, 1.0), u[1],
                                        "u {:g} J/mm3, {:.0f} W{}".format(
                                            u[0], P, "" if power_fraction else ", rated power, no drive loss applied")))
    elif A:
        miss = [n for n, v in (("specific cutting energy (sourced row or constants['specific_energy'])", u),
                               ("spindle power_w", sp_power), ("tool diameter", vc)) if not v]
        out.skipped.append(("power", "; ".join(miss)))
    # ---- chip load (wood) and sourced feed range
    cl = _constant(constants, None, "chip_load")
    cl_src = None
    if cl is None and not _given(constants, "chip_load")[0] and mat is not None:
        got = value(mat, "chip_load", which, tool=kind, diameter_mm=D)
        if got:
            cl, cl_src = (got[0], row_text(got[1])), True
    if cl is not None and flutes > 0:
        f = 1.0
        note = "rpm x {} flutes x {:g} mm/tooth".format(flutes, cl[0])
        if D > 0 and engagement.ap:
            f = cutting_data.depth_factor(engagement.ap / D)
            note += " x {:.2f} (depth factor, ap/D {:.2f})".format(f, engagement.ap / D)
        else:
            out.warnings.append("chip-load depth factor not applied: ap or tool diameter unknown")
        out.candidates.append(Candidate("chip_load", cutting_data.chip_load_feed(rpm, flutes, cl[0]) * f, cl[1], note))
    else:
        out.skipped.append(("chip_load", "a chip-load row for {} {} or constants['chip_load']{}".format(
            (mat.cutting_material if mat else material), kind, "" if flutes else "; tool flutes")))
    fr = value(mat, "feed", which, tool=kind, diameter_mm=D) if mat is not None else None
    if fr:
        out.candidates.append(Candidate("sourced_feed", fr[0], row_text(fr[1]),
                                        "starting range, {} end".format("low" if which != "high" else "high")))
    else:
        out.skipped.append(("sourced_feed", "a feed row for {} {}".format(mat.cutting_material if mat else material, kind)))
    # ---- deflections need the force per unit feed
    rr = _constant(constants, None, "radial_ratio")
    ar = _constant(constants, None, "axial_ratio")
    Ft1 = interaction.tangential_force_per_feed(u[0], A, vc) if (A and u and vc > 0) else None
    # tool bending
    lim_t = limits.get("tool_deflection_mm")
    need = []
    if not lim_t:
        need.append("limits['tool_deflection_mm']")
    if Ft1 is None:
        need.append("force per feed (specific energy, area, diameter)")
    if not rr:
        need.append("constants['radial_ratio'] (Fr/Ft)")
    tE = _constant(constants, None, "tool_E")
    ff = _constant(constants, None, "flute_factor")
    g = geometry or {}
    if not tE:
        need.append("constants['tool_E'] (GPa)")
    if not ff:
        need.append("constants['flute_factor']")
    for k in ("shank_diameter_mm", "shank_length_mm", "flute_length_mm"):
        if not g.get(k):
            need.append("geometry['{}']".format(k))
    if not need:
        F1 = Ft1 * math.sqrt(1.0 + rr[0] ** 2)
        d1 = interaction.stepped_cantilever_tip_deflection(F1, tE[0] * 1000.0, g["shank_diameter_mm"],
                                                           g["shank_length_mm"], ff[0], g["flute_length_mm"], D)
        out.candidates.append(Candidate("tool_deflection", lim_t / d1, "limit given by the caller",
                                        "deflection {:.3g} mm per mm/min, limit {:g} mm; tool E, flute factor, "
                                        "radial ratio from the caller".format(d1, lim_t)))
    else:
        out.skipped.append(("tool_deflection", "; ".join(need)))
    # robot
    lim_r = limits.get("robot_deflection_mm")
    need = []
    if not lim_r:
        need.append("limits['robot_deflection_mm']")
    if Ft1 is None:
        need.append("force per feed (specific energy, area, diameter)")
    if not rr:
        need.append("constants['radial_ratio'] (Fr/Ft)")
    if not ar:
        need.append("constants['axial_ratio'] (Fa/Ft)")
    robot = robot or {}
    J = robot.get("J")
    if J is None:
        need.append("robot['J'] (Jacobian, linear rows in m/rad)")
    kq, kq_src = robot.get("joint_stiffness"), "given by the caller"
    if kq is None:
        present, v = _given(constants, "joint_stiffness")
        if present and v is not None:
            kq = [v] * (np.asarray(J).shape[1] if J is not None else 0)
    if kq is None and robot.get("robot_name"):
        rows = [r for r in materials_mechanics.lookup(robot["robot_name"], "joint_stiffness") if r["low"] is not None
                and r["unit"] == "Nm/rad"]
        if rows:
            r = min(rows, key=lambda x: x["low"])
            kq = [float(r["low"])] * (np.asarray(J).shape[1] if J is not None else 0)
            kq_src = row_text(_row_mech(r)) + " (one value for every joint, low end)"
    if kq is None or len(np.atleast_1d(kq)) == 0:
        need.append("joint stiffness (robot['joint_stiffness'] or a sourced row for robot['robot_name'])")
    if not need:
        C = interaction.robot_linear_compliance(J, np.atleast_1d(np.asarray(kq, float)))
        dirs = robot.get("force_dirs")
        if dirs is not None:
            t, r_, a = (np.asarray(v, float) for v in dirs)
            f = Ft1 * (t + rr[0] * r_ + ar[0] * a)
            d1 = float(np.linalg.norm(C @ f)) * 1000.0
            how = "force along the given directions"
        else:
            lam = float(np.linalg.eigvalsh(C).max())
            d1 = lam * Ft1 * math.sqrt(1.0 + rr[0] ** 2 + ar[0] ** 2) * 1000.0
            how = "bound: largest linear compliance eigenvalue, any force direction"
        out.candidates.append(Candidate("robot_deflection", lim_r / d1, kq_src,
                                        "deflection {:.3g} mm per mm/min, limit {:g} mm; {}".format(d1, lim_r, how)))
    else:
        out.skipped.append(("robot_deflection", "; ".join(need)))
    if out.candidates:
        b = min(out.candidates, key=lambda c: c.feed)
        out.feed, out.binding = b.feed, b.name
    return out


def _attr(obj, name):
    if obj is None:
        return 0.0
    if isinstance(obj, dict):
        return float(obj.get(name) or 0.0)
    return float(getattr(obj, name, 0.0) or 0.0)


# ------------------------------------------------------------------- wear

SAW_KINDS = ("disc_saw", "diamond_wire", "multi_wire_saw")


@dataclass
class Wear:
    model: str = ""                                   # g_ratio | per_area | none
    tool_volume_mm3: Optional[float] = None           # diamond segment volume worn (g_ratio model)
    height_loss_mm: Optional[float] = None            # segment height worn (per_area model)
    life_fraction: Optional[float] = None             # share of the usable segment consumed; needs a usable amount
    sources: Dict[str, str] = field(default_factory=dict)
    skipped: List[Tuple[str, str]] = field(default_factory=list)


def wear(tool_kind, material, removed_mm3: Optional[float] = None, cut_area_mm2: Optional[float] = None,
         constants: Optional[Dict[str, Any]] = None, which: str = "conservative") -> Wear:
    """Diamond tool wear of one cut from a wear-ratio model.

    G-ratio model (grinding, milling, drilling): worn diamond volume = removed stone volume / G, with
    G = constants['g_ratio'] [mm3 stone per mm3 segment]. Per-area model (saws and wires): segment height loss
    = cut area x k, with k = constants['wear_per_area'] [mm per m2 of cut face]. Life fraction needs the usable
    amount: constants['usable_volume_mm3'] (G-ratio model) or constants['usable_height_mm'] (per-area model).
    All of these come from the caller or from materials_mechanics rows named g_ratio and wear_per_area; no such
    row exists yet, so a missing one is reported and the wear is None. Saw kinds prefer the per-area model when
    both constants exist. Limits: the ratio is not constant (it falls as the bond glazes or the stone changes,
    and it depends on speed, feed, water and grit); it applies to the conditions it was measured in."""
    constants = dict(constants or {})
    kind = cutting_data.tool_kind_label(tool_kind)
    mat = _material(material) if material is not None else None
    out = Wear(model="none")
    g = _constant(constants, mat, "g_ratio", "g_ratio", which)
    k = _constant(constants, mat, "wear_per_area", "wear_per_area", which)
    area_ok = k is not None and cut_area_mm2 is not None
    vol_ok = g is not None and removed_mm3 is not None
    use_area = area_ok and (kind in SAW_KINDS or not vol_ok)
    use_vol = vol_ok and not use_area
    if use_area:
        if k[0] < 0:
            raise ValueError("wear_per_area must be >= 0")
        out.model = "per_area"
        out.height_loss_mm = float(cut_area_mm2) / 1e6 * k[0]
        out.sources["wear_per_area"] = k[1]
        h = _constant(constants, None, "usable_height_mm")
        if h and h[0] > 0:
            out.life_fraction = out.height_loss_mm / h[0]
        else:
            out.skipped.append(("life fraction", "constants['usable_height_mm']"))
    elif use_vol:
        if g[0] <= 0:
            raise ValueError("g_ratio must be > 0")
        out.model = "g_ratio"
        out.tool_volume_mm3 = float(removed_mm3) / g[0]
        out.sources["g_ratio"] = g[1]
        v = _constant(constants, None, "usable_volume_mm3")
        if v and v[0] > 0:
            out.life_fraction = out.tool_volume_mm3 / v[0]
        else:
            out.skipped.append(("life fraction", "constants['usable_volume_mm3']"))
    else:
        if kind in SAW_KINDS:
            out.skipped.append(("wear", "wear_per_area (mm/m2; sourced row or constants) and the cut area"
                                if k is None or cut_area_mm2 is None else "a cut area"))
        else:
            out.skipped.append(("wear", "g_ratio (mm3/mm3; sourced row or constants) and the removed volume"
                                if g is None or removed_mm3 is None else "a removed volume"))
    return out


@dataclass
class LifeReport:
    per_step: List[Optional[float]]         # life fraction per step (None where the step could not be evaluated)
    cumulative: List[float]                 # running total over the program, per step order
    per_tool: Dict[str, float]              # life fraction used by each tool name
    program_fraction: float                 # total over the program (steps that could be evaluated)
    programs_per_tool_life: Optional[float]   # 1 / worst per-tool fraction; None when nothing was evaluated
    evaluated: int = 0
    skipped: List[Tuple[str, str]] = field(default_factory=list)
    sources: Dict[str, str] = field(default_factory=dict)

    def text(self) -> str:
        if not self.evaluated:
            return "tool life: not evaluated (" + "; ".join("{}: needs {}".format(a, b) for a, b in self.skipped) + ")"
        s = "tool life: {:.2%} of the usable segment used by this program".format(self.program_fraction)
        if self.programs_per_tool_life:
            s += ", about {:.1f} runs per tool life at the most worn tool".format(self.programs_per_tool_life)
        return s + "; {} of {} steps evaluated".format(self.evaluated, len(self.per_step))


def step_cut_area_mm2(step) -> Optional[float]:
    """Cut face area of a wire step: contact length x path length (StepCut.contact, .length). None for others."""
    c = float(getattr(step, "contact", 0.0) or 0.0)
    L = float(getattr(step, "length", 0.0) or 0.0)
    return c * L if c > 0 and L > 0 else None


def tool_life(steps: Sequence, material, tool_kinds: Optional[Dict[str, str]] = None,
              constants: Optional[Dict[str, Any]] = None, which: str = "conservative") -> LifeReport:
    """Wear over a program. steps: StepCut-like objects (removed mm3, length mm, tool name, optional contact mm).
    tool_kinds maps a tool name to its kind name; without it the step's own `kind` attribute is used. Per-step
    life fractions add up per tool name; programs_per_tool_life = 1 / the largest per-tool fraction. constants
    may map to a dict per tool name: {'saw1': {...}}; keys not matching a tool name are shared by all."""
    constants = dict(constants or {})
    per: Dict[str, float] = {}
    out_steps: List[Optional[float]] = []
    cum: List[float] = []
    skipped: List[Tuple[str, str]] = []
    srcs: Dict[str, str] = {}
    total = 0.0
    n_ok = 0
    names = {getattr(s, "tool", "") for s in steps}
    for s in steps:
        name = getattr(s, "tool", "")
        kind = (tool_kinds or {}).get(name) or getattr(s, "kind", "") or ""
        c = {k: v for k, v in constants.items() if k not in names or not isinstance(v, dict)}
        if isinstance(constants.get(name), dict):
            c.update(constants[name])
        w = wear(kind, material, getattr(s, "removed", None), step_cut_area_mm2(s), c, which)
        if w.life_fraction is None:
            out_steps.append(None)
            for sk in w.skipped:
                if sk not in skipped:
                    skipped.append(sk)
        else:
            out_steps.append(w.life_fraction)
            per[name] = per.get(name, 0.0) + w.life_fraction
            total += w.life_fraction
            n_ok += 1
            srcs.update(w.sources)
        cum.append(total)
    worst = max(per.values()) if per else 0.0
    return LifeReport(out_steps, cum, per, total, (1.0 / worst) if worst > 0 else None, n_ok, skipped, srcs)
