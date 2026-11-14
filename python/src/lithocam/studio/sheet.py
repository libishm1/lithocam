# Copyright 2026 The lithocam authors. SPDX-License-Identifier: Apache-2.0
"""Setup sheet: a Markdown summary of an expanded program, one row per cutting step.

Columns: step, tool, tool kind, feed (mm/min), spindle (rev/min), feed path and rapid path (mm), estimated time, and
the removed volume (mm3) when a stock run (studio.process.StockRun) is given.

Where the numbers come from. Feed and spindle speed are the ones the expansion posted (the step, else the tool, else
the sourced cutting data); for a step the expander does not run through its milling path (wire, disc) they are the
step's or the tool's own values, and "-" when neither sets one. Path lengths are of the CL path for milling steps and
of the robot's tool point for the others. The time adds every move at its own feed, rapids at the station's rapid
speed (Station.rapid_speed), and the dwells: an estimate with no acceleration, no tool changes and no table turns.
Nothing here is a cutting value of its own.
"""
from __future__ import annotations

from typing import Dict, List, Optional

import numpy as np


def _fmt_time(s: Optional[float]) -> str:
    if s is None:
        return "n/a"
    s = float(s)
    if s < 60:
        return "{:.0f} s".format(s)
    return "{}:{:02d}:{:02d}".format(int(s // 3600), int(s % 3600 // 60), int(round(s % 60)))


def _num(v, digits=0) -> str:
    return "-" if not v else ("{:.%df}" % digits).format(float(v))


def step_rows(station, expansion, program=None, tools: Optional[Dict[str, object]] = None, run=None) -> List[dict]:
    """The sheet rows as dicts: step (1-based), title, tools, kind, feed, rpm, feed_mm, rapid_mm, seconds, removed."""
    from .core import lc
    ops = {o["step"]: o for o in getattr(expansion, "ops", [])}
    by_step: Dict[int, List] = {}
    for sg in expansion.segments:
        if len(sg.q):
            by_step.setdefault(sg.step, []).append(sg)
    removed: Dict[int, float] = {}
    if run is not None:
        for sc in run.steps:
            si = expansion.segments[sc.segment].step
            removed[si] = removed.get(si, 0.0) + float(sc.removed)
    rows = []
    for i in sorted(by_step):
        segs = by_step[i]
        if not any(s.feed.any() for s in segs):
            continue                                   # no cutting move: a joint move or a link, not an operation
        ids = []
        for s in segs:
            if s.tool and s.tool not in ids:
                ids.append(s.tool)
        feed_mm = rapid_mm = 0.0
        for s in segs:
            P = np.asarray(s.tcp, dtype=float) * 1000.0
            d = np.linalg.norm(np.diff(P, axis=0), axis=1) if len(P) > 1 else np.zeros(0)
            f = np.asarray(s.feed, bool)[1:]
            feed_mm += float(d[f].sum())
            rapid_mm += float(d[~f].sum())
        o = ops.get(i)
        st = program.steps[i] if program is not None and i < len(program.steps) else None
        p = dict(st.params) if st is not None else {}
        tl = tools.get(ids[0]) if tools and ids else None
        feed = float(o["feed"]) if o else float(p.get("feed") or getattr(tl, "feed", 0) or 0)
        rpm = float(o["rpm"]) if o else float(p.get("rpm") or getattr(tl, "spindle_rpm", 0) or 0)
        if o:
            feed_mm, rapid_mm, secs = float(o["feed_mm"]), float(o["rapid_mm"]), o["seconds"]
        elif feed > 0:
            secs = feed_mm / feed * 60.0 + rapid_mm / max(float(station.rapid_speed), 1e-9)
        else:
            secs = None
        kind = ""
        if tl is not None:
            try:
                kind = lc.tool_kind_name(tl.kind)
            except Exception:   # noqa: BLE001 - a stand-in tool object
                kind = str(getattr(tl, "kind", ""))
        rows.append({"step": i + 1, "title": st.title() if st is not None else segs[0].label, "tools": ids,
                     "kind": kind, "feed": feed, "rpm": rpm, "feed_mm": feed_mm, "rapid_mm": rapid_mm,
                     "seconds": secs, "removed": removed.get(i) if run is not None else None})
    return rows


def setup_sheet(station, expansion, program=None, tools: Optional[Dict[str, object]] = None, run=None) -> str:
    """Markdown setup sheet of an expansion (see the module text). program: the program it was made from (step titles
    and parameters); tools: id -> Tool (tool kinds); run: a StockRun of the expansion (removed volumes)."""
    rows = step_rows(station, expansion, program, tools, run)
    out = ["# Setup sheet" + (": " + program.name if program is not None else ""), ""]
    out.append("- Robot: {}".format(station.robot))
    out.append("- Material: {}".format(station.material))
    if station.stock:
        s = station.stock
        out.append("- Stock: {:g} x {:g} x {:g} mm (x, y, z)".format(
            s["xmax"] - s["xmin"], s["ymax"] - s["ymin"], s["zmax"] - s["zmin"]))
    out.append("- Work frame: x {:g}, y {:g}, z {:g} m in the robot base frame".format(*station.work_xyz))
    out.append("- Rapid speed for the estimate: {:g} mm/s".format(station.rapid_speed))
    out.append("")
    with_vol = run is not None
    head = ["Step", "Operation", "Tool", "Kind", "Feed (mm/min)", "Spindle (rev/min)", "Feed path (mm)",
            "Rapid path (mm)", "Time"] + (["Removed (mm3)"] if with_vol else [])
    out.append("| " + " | ".join(head) + " |")
    out.append("|" + "|".join("---" for _ in head) + "|")
    tf = tr = ts = tv = 0.0
    unknown = False
    for r in rows:
        cells = [str(r["step"]), r["title"].replace("|", "/"), ", ".join(r["tools"]) or "-", r["kind"] or "-",
                 _num(r["feed"]), _num(r["rpm"]), "{:.0f}".format(r["feed_mm"]), "{:.0f}".format(r["rapid_mm"]),
                 _fmt_time(r["seconds"])]
        if with_vol:
            cells.append("{:.0f}".format(r["removed"] or 0.0))
            tv += r["removed"] or 0.0
        out.append("| " + " | ".join(cells) + " |")
        tf += r["feed_mm"]
        tr += r["rapid_mm"]
        if r["seconds"] is None:
            unknown = True
        else:
            ts += r["seconds"]
    total = ["", "Total", "", "", "", "", "{:.0f}".format(tf), "{:.0f}".format(tr),
             _fmt_time(ts) + (" (without the steps marked n/a)" if unknown else "")]
    if with_vol:
        total.append("{:.0f}".format(tv))
    out.append("| " + " | ".join(total) + " |")
    if getattr(expansion, "errors", None):
        out += ["", "Errors in the program (it is not posted until they are fixed):"] + ["- " + e for e in expansion.errors]
    out += ["", "Times are estimates: every move at its own feed, rapids at the station's rapid speed, plus dwells. "
            "No acceleration, tool changes or table turns are counted. A feed shown as - is not set by the step, "
            "the tool or the cutting data; the time is then n/a."]
    if not with_vol:
        out.append("The removed volume needs a stock simulation of the program.")
    return "\n".join(out) + "\n"
