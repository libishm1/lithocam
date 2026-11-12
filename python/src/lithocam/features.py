# Copyright 2026 The lithocam authors. SPDX-License-Identifier: Apache-2.0
"""Stone feature input: quiddity B-rep recognition mapped to stone operations.

quiddity (github.com/pzfreo/quiddity, Apache-2.0) recognises machining features in a STEP B-rep and names them by
geometry: a kerf slot comes back as a pocket, a drip groove as a channel, a sink cutout as a slot. This module turns
those records into stone operations with a process route (drill, saw disc, profile wheel, contour cut, router,
lathe), and adds three things quiddity does not do for stone work:

1. a priority rule, so overlapping descriptive records (plates, step levels, risers, section recesses, bosses on a
   turned body, concave fillets that are the corners of a cutout) do not become extra operations;
2. pattern grouping of equal parallel grooves and slots at a constant pitch;
3. a revision diff in file coordinates. quiddity's own `correspond` aligns the two parts first, so a symmetric
   stone panel comes back ambiguous; most panels are symmetric.

quiddity runs in-process when the `quiddity` wheel imports (Python 3.11 or newer). Otherwise the `quiddity`
executable is called as a subprocess: set LITHOCAM_QUIDDITY to its path, or put it on PATH. That second path lets
Rhino 8 (CPython 3.9) use this module.

The routes are a starting rule table, not a process plan. Every record that no rule covers is kept as `unmapped`,
so nothing disappears silently.

    python -m lithocam.features part.step [--json ops.json] [--diff revised.step]
"""
from __future__ import annotations

import json
import math
import os
import shutil
import subprocess
from dataclasses import asdict, dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

SCHEMA_PREFIX = "quiddity-rust/recognition/"
SCHEMA_TESTED = "quiddity-rust/recognition/6"

# Routes
DRILL = "drill"
SAW_DISC = "saw_disc"
PROFILE_WHEEL = "profile_wheel"
CONTOUR_CUT = "contour_cut"      # waterjet, wire or CNC router following a closed contour
ROUTER = "router"                # CNC router or milling with an end tool
LATHE = "lathe"
UNMAPPED = "unmapped"

# Families that describe faces already explained by a feature. They become operations only when no kept
# operation covers their faces.
DESCRIPTIVE = {"plates", "step_levels", "risers", "section_recesses", "flats", "pads"}


@dataclass
class Rules:
    """Thresholds for the rule table, in model units (mm)."""

    narrow_max: float = 12.0          # widest slot or groove a saw disc cuts in one pass
    kerf_depth_ratio: float = 2.0     # a narrow recess this many widths deep or deeper is a kerf slot, else a groove
    core_drill_min: float = 20.0      # holes at or above this diameter take a core drill
    pattern_tol: float = 0.05         # size and pitch tolerance for pattern grouping
    diff_match_max: float = 50.0      # largest move still matched as the same feature in a revision diff
    diff_tol: float = 1e-3            # smallest reported parameter change


@dataclass
class StoneOp:
    kind: str                         # stone name: anchor_hole, kerf_slot, drip_groove, rebate, cutout, ...
    route: str                        # one of the route constants
    at: Tuple[float, float, float]    # a representative point in file coordinates
    params: Dict[str, object]
    faces: List[int]
    source: List[str]                 # quiddity record ids
    evidence: str                     # quiddity evidence status of the source record
    note: str = ""


@dataclass
class FeatureSet:
    ops: List[StoneOp]
    patterns: List[Dict[str, object]]
    dropped: List[Dict[str, object]]  # records absorbed by the priority rule, with the reason
    unexplained_faces: List[int]      # faces quiddity could not assign to a feature or the stock body
    face_count: int
    schema: str
    source_sha256: str = ""            # of the file quiddity read (the canonicalised copy when edges were rewritten)
    warnings: List[str] = field(default_factory=list)
    canonicalised_edges: int = 0       # straight edges rewritten from degree-1 B-splines to LINE before recognition

    def to_json(self) -> str:
        return json.dumps(asdict(self), indent=1)


# ---------------------------------------------------------------- running quiddity

def run_quiddity(step_path: str, exe: Optional[str] = None, timeout: float = 120.0) -> dict:
    """Recognise a STEP file. Returns quiddity's parsed JSON (the family lists plus `document`)."""
    if exe is None:
        try:
            import quiddity  # noqa: PLC0415  (optional dependency)
            return json.loads(quiddity.recognise(str(step_path)))
        except ImportError:
            pass
    exe = exe or os.environ.get("LITHOCAM_QUIDDITY") or shutil.which("quiddity")
    if not exe:
        raise RuntimeError("quiddity not found: pip install quiddity (Python 3.11+), or set LITHOCAM_QUIDDITY "
                           "to the quiddity executable")
    # pip installs `quiddity` as a launcher for its own Python. A host Python (Rhino 8 sets PYTHONHOME and
    # PYTHONPATH) must not leak into it, or the launcher loads the wrong standard library.
    env = {k: v for k, v in os.environ.items() if not k.upper().startswith("PYTHON")}
    res = subprocess.run([exe, str(step_path)], capture_output=True, text=True, timeout=timeout, env=env)
    if res.returncode != 0:
        raise RuntimeError("quiddity failed ({}): {}".format(res.returncode, res.stderr.strip()[:500]))
    return json.loads(res.stdout)


def recognise(step_path: str, rules: Optional[Rules] = None, exe: Optional[str] = None,
              canonicalise: bool = True) -> FeatureSet:
    """Recognise a STEP file and map it to stone operations.

    With `canonicalise` (the default), straight edges stored as degree-1 B-splines (Rhino's STEP writer does this
    for every straight edge) are first rewritten as exact LINEs in a temporary copy; see lithocam.step_canon.
    """
    n = 0
    path = str(step_path)
    tmp = None
    if canonicalise:
        from .step_canon import canonical_lines  # noqa: PLC0415
        with open(path, encoding="utf-8", errors="replace") as f:
            text, n = canonical_lines(f.read())
        if n:
            import tempfile  # noqa: PLC0415
            fd, tmp = tempfile.mkstemp(suffix=".step", prefix="lithocam_canon_")
            with os.fdopen(fd, "w", encoding="utf-8", newline="\n") as f:
                f.write(text)
            path = tmp
    try:
        fs = stone_ops(run_quiddity(path, exe=exe), rules)
    finally:
        if tmp:
            os.remove(tmp)
    fs.canonicalised_edges = n
    return fs


# ---------------------------------------------------------------- geometry helpers

_AX = {"x": 0, "y": 1, "z": 2}


def _box(rec: dict) -> Optional[List[float]]:
    bk = rec["parameters"].get("body_key")
    return list(bk[:6]) if isinstance(bk, list) and len(bk) >= 6 else None


def _span_centre(p: dict) -> Tuple[float, float, float]:
    """Centre of a pocket, channel or slot record from its long, width and depth ranges."""
    c = [0.0, 0.0, 0.0]
    la, wa = _AX[p["long_axis"]], _AX[p["width_axis"]]
    da = 3 - la - wa
    c[la] = (p["lo"] + p["hi"]) / 2.0
    c[wa] = p["w_center"]
    c[da] = (p["d_lo"] + p["d_hi"]) / 2.0
    return (c[0], c[1], c[2])


def _point(rec: dict) -> Tuple[float, float, float]:
    p = rec["parameters"]
    if rec["family"] == "holes" and p.get("bottom") == "through":
        # mid-depth, so the drilling side the writer happened to pick does not move the hole
        loc, ax, dep = p["location"], p["axis"], float(p["depth"])
        return tuple(float(loc[i]) + float(ax[i]) * dep / 2.0 for i in range(3))
    for key in ("location", "at", "axis_point", "center", "origin"):
        v = p.get(key)
        if isinstance(v, list) and len(v) == 3:
            return (float(v[0]), float(v[1]), float(v[2]))
    if {"long_axis", "width_axis", "lo", "hi", "w_center", "d_lo", "d_hi"} <= set(p):
        return _span_centre(p)
    return (math.nan, math.nan, math.nan)


def _section_extents(section: Sequence[Sequence[float]]) -> Tuple[float, float]:
    us = [q[0] for q in section]
    vs = [q[1] for q in section]
    a, b = max(us) - min(us), max(vs) - min(vs)
    return (max(a, b), min(a, b))


def _spans_body(rec: dict, axis: int, lo: float, hi: float, tol: float = 1e-6) -> Optional[bool]:
    box = _box(rec)
    if box is None:
        return None
    return lo <= box[axis] + tol and hi >= box[axis + 3] - tol


# ---------------------------------------------------------------- the rule table

def _map_record(rec: dict, rules: Rules) -> Tuple[Optional[str], Optional[str], Dict[str, object], str]:
    """Returns (kind, route, params, note). kind None means the record is descriptive or a pattern."""
    fam, p = rec["family"], rec["parameters"]
    if fam == "holes":
        d = float(p["diameter"])
        through = p.get("bottom") == "through"
        tool = "core_drill" if d >= rules.core_drill_min else "twist_or_diamond_drill"
        return ("through_hole" if through else "anchor_hole", DRILL,
                {"diameter": d, "depth": p["depth"], "through": through, "axis": p.get("axis"), "tool": tool}, "")
    if fam in ("pockets", "prismatic_pockets"):
        # quiddity names the same recess `pockets` or `prismatic_pockets` depending on how the edges are encoded,
        # so the stone kind follows the shape, not the family name.
        if fam == "pockets":
            w, ln, dep = float(p["width"]), float(p["length"]), float(p["depth"])
        else:
            ln, w = _section_extents(p["section"])
            dep = float(p["depth"])
        return _narrow_recess(w, ln, dep, p.get("corner_radius"), rules)
    if fam == "channels":
        w, dep = float(p["width"]), float(p["d_hi"]) - float(p["d_lo"])
        da = 3 - _AX[p["long_axis"]] - _AX[p["width_axis"]]
        under = da == 2 and p.get("open_sign") == -1
        route = SAW_DISC if w <= rules.narrow_max else ROUTER
        return ("drip_groove" if under else "through_groove", route,
                {"width": w, "depth": dep, "length": float(p["hi"]) - float(p["lo"]), "long_axis": p["long_axis"]},
                "underside groove read as a drip in file orientation (z up)" if under else "")
    if fam == "slots":
        la, wa = _AX[p["long_axis"]], _AX[p["width_axis"]]
        da = 3 - la - wa
        through = _spans_body(rec, da, float(p["d_lo"]), float(p["d_hi"]))
        cr = p.get("corner_radius")
        prm = {"length": p["length"], "width": p["width"], "corner_radius": cr, "through": through,
               "min_tool_radius": cr}
        if through:
            return ("cutout", CONTOUR_CUT, prm, "closed contour; the corner radius bounds the tool or jet radius")
        return ("pocket", ROUTER, prm, "")
    if fam == "through_steps":
        ln, w = _section_extents(p["section"])
        return ("rebate", SAW_DISC, {"length": p["length"], "section": p["section"], "legs": [ln, w]},
                "two crossing disc passes")
    if fam == "chamfers":
        return ("edge_chamfer", PROFILE_WHEEL, {"leg1": p["leg1"], "leg2": p["leg2"], "angle": p["angle"],
                                                "axis": p.get("axis")}, "")
    if fam == "fillets":
        if p.get("side") == "convex":
            return ("edge_round", PROFILE_WHEEL, {"radius": p["radius"], "axis": p.get("axis")}, "")
        return ("inside_radius", ROUTER, {"radius": p["radius"], "axis": p.get("axis")}, "")
    if fam == "turned_bodies":
        return ("turned_profile", LATHE, {"length": p["length"], "od": p["od"], "axis": p.get("axis"),
                                          "segments": len(p.get("profile", []))}, "")
    if fam == "bosses":
        return ("boss", ROUTER, {"diameter": p["diameter"], "height": p["height"]}, "")
    if fam.endswith("_patterns") or fam == "fingerprints":
        return (None, None, {}, "pattern")
    if fam in DESCRIPTIVE:
        return (None, None, {}, "descriptive")
    return (fam, UNMAPPED, {k: v for k, v in p.items() if k != "body_key"}, "no stone rule for this family yet")


def _narrow_recess(w: float, ln: float, dep: float, corner_radius, rules: Rules):
    """A stopped rectangular recess. Narrow and deep is a kerf slot; narrow and shallow is a groove."""
    prm = {"width": w, "length": ln, "depth": dep}
    if w <= rules.narrow_max and dep >= rules.kerf_depth_ratio * w:
        return ("kerf_slot", SAW_DISC, prm, "a disc leaves run-out arcs at the ends; square ends need a router")
    if w <= rules.narrow_max:
        return ("stopped_groove", ROUTER, prm, "stopped ends are square; a saw disc would leave run-out arcs")
    prm["corner_radius"] = corner_radius
    return ("pocket", ROUTER, prm, "")


# Primary families are mapped first; their faces then absorb descriptive records.
_PRIMARY_ORDER = ["turned_bodies", "slots", "holes", "pockets", "channels", "prismatic_pockets", "through_steps",
                  "chamfers", "fillets", "bosses"]


def stone_ops(qdoc: dict, rules: Optional[Rules] = None) -> FeatureSet:
    rules = rules or Rules()
    doc = qdoc["document"] if "document" in qdoc else qdoc
    schema = doc.get("schema_version", "")
    warnings = []
    if not schema.startswith(SCHEMA_PREFIX):
        raise ValueError("not a quiddity recognition document: schema_version={!r}".format(schema))
    if schema != SCHEMA_TESTED:
        warnings.append("schema {} differs from the tested {}".format(schema, SCHEMA_TESTED))
    recs = doc["records"]
    order = {f: i for i, f in enumerate(_PRIMARY_ORDER)}
    recs = sorted(recs, key=lambda r: (order.get(r["family"], len(order)), r["id"]))

    fa = doc.get("face_accounting") or []
    stock = {i for i, a in enumerate(fa) if isinstance(a, dict) and a.get("group") == "body"}
    unexplained = [i for i, a in enumerate(fa) if isinstance(a, dict) and a.get("group") == "unexplained"]

    ops: List[StoneOp] = []
    dropped: List[Dict[str, object]] = []
    covered = set(stock)             # stock-body faces count as explained for descriptive records
    turned_faces = set()
    cutouts: List[Tuple[dict, StoneOp]] = []
    for rec in recs:
        fam, faces = rec["family"], list(rec["faces"])
        kind, route, prm, note = _map_record(rec, rules)
        if kind is None:
            if note == "pattern":
                continue
            if faces and set(faces) <= covered:
                dropped.append({"id": rec["id"], "reason": "faces already explained by a kept operation or the stock"})
                continue
            if fam == "plates":
                # a stone slab is a plate: its broad faces are the sawn stock, not an operation
                covered |= set(faces)
                dropped.append({"id": rec["id"], "reason": "slab faces (sawn stock)"})
                continue
            dropped.append({"id": rec["id"], "reason": "descriptive record with faces not covered: check by hand",
                            "faces": faces})
            continue
        if fam == "bosses" and set(faces) <= turned_faces:
            dropped.append({"id": rec["id"], "reason": "part of the turned profile"})
            continue
        if fam == "fillets" and rec["parameters"].get("side") == "concave":
            host = _cutout_corner(rec, cutouts)
            if host is not None:
                host.faces = sorted(set(host.faces) | set(faces))
                host.source.append(rec["id"])
                dropped.append({"id": rec["id"], "reason": "corner of {}".format(host.source[0])})
                covered |= set(faces)
                continue
        op = StoneOp(kind=kind, route=route, at=_point(rec), params=prm, faces=faces, source=[rec["id"]],
                     evidence=rec.get("evidence_status", ""), note=note)
        ops.append(op)
        covered |= set(faces)
        if fam == "turned_bodies":
            turned_faces |= set(faces)
        if fam == "slots" or (fam == "pockets" and rec["parameters"].get("corner_radius")):
            cutouts.append((rec, op))

    return FeatureSet(ops=ops, patterns=_patterns(doc, ops, rules), dropped=dropped, unexplained_faces=unexplained,
                      face_count=int(doc.get("face_count", len(fa))), schema=schema,
                      source_sha256=doc.get("source_sha256", ""), warnings=warnings)


def _cutout_corner(fillet: dict, cutouts: List[Tuple[dict, StoneOp]]) -> Optional[StoneOp]:
    """A concave fillet whose radius equals a cutout's corner radius and sits inside its plan span."""
    p = fillet["parameters"]
    at = p.get("at")
    for rec, op in cutouts:
        q = rec["parameters"]
        cr = q.get("corner_radius")
        if not cr or abs(float(cr) - float(p["radius"])) > 1e-6 or at is None:
            continue
        la, wa = _AX[q["long_axis"]], _AX[q["width_axis"]]
        half_w = float(q["width"]) / 2.0
        if (float(q["lo"]) - 1e-6 <= at[la] <= float(q["hi"]) + 1e-6
                and abs(at[wa] - float(q["w_center"])) <= half_w + 1e-6):
            return op
    return None


# ---------------------------------------------------------------- patterns

def _patterns(doc: dict, ops: List[StoneOp], rules: Rules) -> List[Dict[str, object]]:
    out: List[Dict[str, object]] = []
    for rec in doc["records"]:
        if rec["family"] == "hole_patterns":
            members = [op.source[0] for op in ops if op.kind in ("anchor_hole", "through_hole")
                       and set(op.faces) & set(rec["faces"])]
            prm = {k: v for k, v in rec["parameters"].items() if k in ("width", "height", "angle", "center")}
            out.append({"kind": "hole_grid", "source": rec["id"], "members": members, "count": len(members),
                        "params": prm})
    # equal parallel grooves and slots at a constant pitch: quiddity reports these one by one
    for kind in ("stopped_groove", "kerf_slot", "through_groove", "drip_groove"):
        group = [op for op in ops if op.kind == kind]
        if len(group) < 3:
            continue
        ref = group[0].params
        same = [op for op in group
                if all(abs(float(op.params[k]) - float(ref[k])) <= rules.pattern_tol
                       for k in ("width", "length", "depth") if k in ref)]
        if len(same) < 3:
            continue
        pts = [op.at for op in same]
        spread = [max(p[i] for p in pts) - min(p[i] for p in pts) for i in range(3)]
        axis = max(range(3), key=lambda i: spread[i])
        if any(spread[i] > rules.pattern_tol for i in range(3) if i != axis):
            continue
        same.sort(key=lambda op: op.at[axis])
        steps = [same[i + 1].at[axis] - same[i].at[axis] for i in range(len(same) - 1)]
        if max(steps) - min(steps) <= rules.pattern_tol:
            out.append({"kind": kind + "_set", "members": [op.source[0] for op in same], "count": len(same),
                        "pitch": round(sum(steps) / len(steps), 6), "axis": "xyz"[axis]})
    return out


# ---------------------------------------------------------------- revision diff in file coordinates

def _num_params(op: StoneOp) -> Dict[str, float]:
    return {k: float(v) for k, v in op.params.items() if isinstance(v, (int, float)) and not isinstance(v, bool)}


def diff(old: FeatureSet, new: FeatureSet, rules: Optional[Rules] = None) -> List[Dict[str, object]]:
    """Match operations of the same kind by nearest position in file coordinates (no alignment step).

    Returns one entry per operation: unchanged, moved, resized (or both), removed or added.
    """
    rules = rules or Rules()
    pairs = []
    for i, a in enumerate(old.ops):
        for j, b in enumerate(new.ops):
            if a.kind == b.kind:
                pairs.append((math.dist(a.at, b.at), i, j))
    pairs.sort()
    used_a, used_b, out = set(), set(), []
    for dist, i, j in pairs:
        if i in used_a or j in used_b or dist > rules.diff_match_max:
            continue
        used_a.add(i)
        used_b.add(j)
        a, b = old.ops[i], new.ops[j]
        pa, pb = _num_params(a), _num_params(b)
        changed = {k: [pa[k], pb[k]] for k in pa.keys() & pb.keys() if abs(pa[k] - pb[k]) > rules.diff_tol}
        moved = dist > rules.diff_tol
        status = "unchanged" if not (moved or changed) else "+".join(
            s for s, f in (("moved", moved), ("resized", bool(changed))) if f)
        out.append({"status": status, "kind": a.kind, "old": a.source[0], "new": b.source[0],
                    "move": [round(b.at[k] - a.at[k], 6) for k in range(3)], "changed": changed})
    out += [{"status": "removed", "kind": a.kind, "old": a.source[0]} for i, a in enumerate(old.ops) if i not in used_a]
    out += [{"status": "added", "kind": b.kind, "new": b.source[0]} for j, b in enumerate(new.ops) if j not in used_b]
    return out


# ---------------------------------------------------------------- CLI

def _table(fs: FeatureSet) -> str:
    rows = ["{:<16} {:<14} {:<34} {}".format("kind", "route", "at", "source")]
    for op in fs.ops:
        at = "({:.1f}, {:.1f}, {:.1f})".format(*op.at)
        rows.append("{:<16} {:<14} {:<34} {}".format(op.kind, op.route, at, ",".join(op.source)))
    for p in fs.patterns:
        rows.append("pattern {}: {} members{}".format(p["kind"], p["count"],
                                                       ", pitch {}".format(p["pitch"]) if "pitch" in p else ""))
    rows.append("faces {} | unexplained {} | dropped {} | line edges canonicalised {}".format(
        fs.face_count, fs.unexplained_faces, [d["id"] for d in fs.dropped], fs.canonicalised_edges))
    rows += ["warning: " + w for w in fs.warnings]
    return "\n".join(rows)


def main(argv: Optional[Sequence[str]] = None) -> int:
    import argparse  # noqa: PLC0415

    ap = argparse.ArgumentParser(prog="python -m lithocam.features", description=__doc__.split("\n\n")[0])
    ap.add_argument("step")
    ap.add_argument("--json", help="write the operations as JSON to this path")
    ap.add_argument("--diff", help="a revised STEP file to compare against, in file coordinates")
    ap.add_argument("--quiddity", help="path to the quiddity executable")
    ap.add_argument("--no-canon", action="store_true", help="skip the straight-edge canonicalisation")
    a = ap.parse_args(argv)
    fs = recognise(a.step, exe=a.quiddity, canonicalise=not a.no_canon)
    print(_table(fs))
    if a.json:
        with open(a.json, "w", encoding="utf-8") as f:
            f.write(fs.to_json())
    if a.diff:
        for d in diff(fs, recognise(a.diff, exe=a.quiddity, canonicalise=not a.no_canon)):
            print(json.dumps(d))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
