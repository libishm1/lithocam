# Copyright 2026 The lithocam authors. SPDX-License-Identifier: Apache-2.0
"""Disc saw toolpaths for a 30 mm granite slab, split over two robots.

KR120, 350 mm blade (kerf 3.2, flange 100): through profile cuts of an L-shaped tread with one reflex corner and a
45 degree mitre on the front edge.
UR10e, 125 mm blade (kerf 2.0, flange 40): two shallow grooves, then comb roughing (parallel and cross) of a rebate
pocket, 12 mm deep.

Usage: python examples/disc_saw_slab.py OUTDIR
Writes OUTDIR/disc_cuts_kr120.json, OUTDIR/disc_cuts_ur10e.json and OUTDIR/disc_saw_summary.txt, and prints the summary.
Needs numpy only (lithocam.discsaw, lithocam.cutting_data).
"""
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "python" / "src"))
from lithocam import cutting_data as cd, discsaw as ds   # noqa: E402

TOP, T = 0.0, 30.0
TREAD = np.array([[0, 0], [600, 0], [600, 300], [300, 300], [300, 500], [0, 500]], float)   # reflex vertex 3
MITRE = [-45.0, 0, 0, 0, 0, 0]          # front edge (edge 0): undercut mitre, sharp top arris
REBATE = np.array([[40, 40], [260, 40], [260, 260], [40, 260]], float)
GROOVES = [np.array([[330, y], [570, y], [570, y + 2.02], [330, y + 2.02]], float) for y in (100.0, 150.0)]
GROOVE_DEPTH, REBATE_DEPTH = 6.0, 12.0

ROBOTS = {
    "kr120": dict(blade_diameter=350.0, kerf=3.2, flange_diameter=100.0, stepdown=10.0),
    "ur10e": dict(blade_diameter=125.0, kerf=2.0, flange_diameter=40.0, stepdown=4.0),
}


def feeds(diameter):
    """rpm and feed from the sourced granite disc-saw entries (lowest of the range), or None when not covered."""
    v = cd.lookup("granite", "disc_saw", "surface_speed")
    f = cd.lookup("granite", "disc_saw", "feed")
    rpm = cd.rpm_from_surface_speed(v[0].lo, diameter) if v else None
    feed = f[0].lo if f else None
    note = "rpm from {:g} m/s ({}); feed {} mm/min is sourced at 6 mm depth only, derate for deeper passes".format(
        v[0].lo, v[0].source, feed) if v and f else "no sourced entry; rpm and feed left unset"
    return rpm, feed, note


def corner_lines(cuts, outline):
    types = ds.corner_types(outline)
    n = len(outline)
    lines = []
    deepest = {}
    for c in cuts:
        if c.kind == "profile" and (c.edge not in deepest or c.depth > deepest[c.edge].depth):
            deepest[c.edge] = c
    for k in range(n):
        prev, nxt = deepest.get((k - 1) % n), deepest.get(k)
        oc = (prev.overcut_end if prev else 0.0) + (nxt.overcut_start if nxt else 0.0)
        un = (prev.uncut_end if prev else 0.0) + (nxt.uncut_start if nxt else 0.0)
        lines.append("  vertex {} {:<6} ({:6.1f}, {:6.1f}): overcut {:6.2f} mm, uncut {:6.2f} mm".format(
            k, types[k], outline[k][0], outline[k][1], oc, un))
    return lines


def main(outdir):
    out = Path(outdir)
    out.mkdir(parents=True, exist_ok=True)
    S = []

    # ---------------- KR120: through profile cuts with the mitre ----------------
    kw = dict(top=TOP, thickness=T, **ROBOTS["kr120"])
    rep = {}
    kr = ds.profile_cuts(TREAD, bevel_deg=MITRE, depth=None, report=rep, **kw)
    r1 = ds.blade_intrudes(kr, TREAD, T, bevels=MITRE)
    rpm, feed, note = feeds(350.0)
    S += ["KR120, 350 mm blade, kerf 3.2, flange 100, stepdown <= 10 mm, 30 mm granite slab",
          "  max depth {:.1f} mm, through depth {:.2f} mm along the blade (mitre edge {:.2f})".format(
              ds.max_cut_depth(175, 50), T + 1.0, T / np.cos(np.radians(45)) + 1.0),
          "  rpm {}, feed {} mm/min; {}".format("unset" if rpm is None else "{:.0f}".format(rpm), feed, note),
          "  {} cuts, {} passes per edge (edge 0 mitre {} deg), skipped {}".format(
              len(kr), max(c.pass_index for c in kr) + 1, MITRE[0], rep["skipped"]),
          "  corners (deepest pass):"] + corner_lines(kr, TREAD)
    wedges = ds.uncut_wedges(kr, TREAD)
    S.append("  uncut wedges for a follow-up tool: " + ("none" if not wedges else "; ".join(
        "edge {} {} at ({:.0f},{:.0f}) length {:.2f} mm".format(w["edge"], w["end"], *w["vertex"], w["length_mm"])
        for w in wedges)))
    S.append("  intrusion check: max intrusion {:.2e} mm (<= 0 within 1e-6: {}), kerf face to edge-line error {:.1e} mm".format(
        r1.max_intrusion, "PASS" if r1.max_intrusion <= 1e-6 else "FAIL", max(r1.face_plane_error.values())))
    ds.cuts_to_json(kr, out / "disc_cuts_kr120.json")

    # ---------------- UR10e: grooves and rebate combing ----------------
    kw = dict(top=TOP, thickness=T, **ROBOTS["ur10e"])
    ur, checks, skipped, n_groove = [], [], [], 0
    for g in GROOVES:
        rp = {}
        cs = ds.comb_cuts(g, depth=GROOVE_DEPTH, pitch=10.0, report=rp, **kw)
        r = ds.blade_intrudes(cs, g, T, solid="outside")
        checks.append(("groove", r.max_intrusion))
        skipped += rp["skipped"]
        ur += cs
        n_groove += len(cs)
    rp = {}
    rb = ds.comb_cuts(REBATE, depth=REBATE_DEPTH, pitch=15.0, angle_deg=0.0, cross_angle_deg=90.0, report=rp, **kw)
    skipped += rp["skipped"]
    checks.append(("rebate comb", ds.blade_intrudes(rb, REBATE, T, solid="outside").max_intrusion))
    ur += rb
    rpm, feed, note = feeds(125.0)
    S += ["", "UR10e, 125 mm blade, kerf 2.0, flange 40, stepdown <= 4 mm",
          "  max depth {:.1f} mm; grooves {:.0f} mm deep, rebate {:.0f} mm deep".format(
              ds.max_cut_depth(62.5, 20), GROOVE_DEPTH, REBATE_DEPTH),
          "  rpm {}, feed {} mm/min; {}".format("unset" if rpm is None else "{:.0f}".format(rpm), feed, note),
          "  {} cuts: {} groove, {} rebate comb (parallel + cross), skipped {}".format(
              len(ur), n_groove, len(rb), skipped),
          "  comb ends stop e(d) short of the pocket wall: uncut {:.2f} mm per end at {:.0f} mm depth".format(
              ds.runout(62.5, REBATE_DEPTH), REBATE_DEPTH)]
    for name, v in checks:
        if v is not None:
            S.append("  intrusion check {}: max intrusion {:.2e} mm ({})".format(name, v, "PASS" if v <= 1e-6 else "FAIL"))
    ds.cuts_to_json(ur, out / "disc_cuts_ur10e.json")

    text = "\n".join(S) + "\n"
    (out / "disc_saw_summary.txt").write_text(text, encoding="utf-8")
    print(text)


if __name__ == "__main__":
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    main(sys.argv[1])
