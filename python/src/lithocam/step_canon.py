# Copyright 2026 The lithocam authors. SPDX-License-Identifier: Apache-2.0
"""Exact canonicalisation of STEP edge curves before feature recognition.

Rhino 8's STEP writer stores every straight edge as a degree-1 B_SPLINE_CURVE_WITH_KNOTS with two control points (or
its rational complex-entity form) and writes no LINE entities. That curve is exactly a line segment. quiddity 0.4.9 recognises channels, through steps and
slots only when the edges are LINEs, so a Rhino-written window sill loses its drip groove and rebate.

`canonical_lines` rewrites each such curve as LINE(start, VECTOR(direction, length)). The edge still runs between
the same two vertices, so the B-rep is unchanged. A curve is rewritten only when it is degree 1, has exactly two
control points and is not referenced by anything that depends on its parameterisation (a PCURVE or a
SURFACE_CURVE); otherwise it is left alone. Faces and surfaces are never touched.

    python -m lithocam.step_canon in.step out.step
"""
from __future__ import annotations

import math
import re
from typing import Dict, List, Tuple

_ENTITY = re.compile(r"#(\d+)\s*=\s*(.*?);(?=\s*(?:#\d+\s*=|ENDSEC))", re.S)
_BSPLINE = re.compile(r"^B_SPLINE_CURVE_WITH_KNOTS\s*\(\s*'[^']*'\s*,\s*1\s*,\s*\(\s*#(\d+)\s*,\s*#(\d+)\s*\)", re.S)
# the rational form is a complex entity; with degree 1 and two control points it is still the straight segment
# between them, whatever the (positive) weights
_BSPLINE_COMPLEX = re.compile(r"^\(\s*BOUNDED_CURVE\s*\(\s*\).*?\bB_SPLINE_CURVE\s*\(\s*1\s*,\s*\(\s*#(\d+)\s*,\s*#(\d+)\s*\)"
                              r".*?\bRATIONAL_B_SPLINE_CURVE\s*\(\s*\(\s*([^,)]+)\s*,\s*([^,)]+)\s*\)\s*\)", re.S)
_POINT = re.compile(r"^CARTESIAN_POINT\s*\(\s*'[^']*'\s*,\s*\(\s*([^)]*)\)\s*\)", re.S)
_PARAM_USERS = ("PCURVE", "SURFACE_CURVE", "SEAM_CURVE", "TRIMMED_CURVE")


def _num(s: str) -> float:
    return float(s.strip().replace("D", "E"))


def _fmt(v: float) -> str:
    s = repr(float(v))
    return s if ("." in s or "e" in s or "E" in s) else s + "."


def canonical_lines(text: str, tol: float = 1e-12) -> Tuple[str, int]:
    """Return (new STEP text, number of curves rewritten)."""
    ents: Dict[int, Tuple[int, int, str]] = {}
    for m in _ENTITY.finditer(text):
        ents[int(m.group(1))] = (m.start(), m.end(), m.group(2).strip())
    if not ents:
        return text, 0
    pts: Dict[int, List[float]] = {}
    for k, (_, _, body) in ents.items():
        pm = _POINT.match(body)
        if pm:
            pts[k] = [_num(x) for x in pm.group(1).split(",")]
    blocked = set()
    for _, _, body in ents.values():
        head = body.split("(", 1)[0].strip()
        if head in _PARAM_USERS:
            blocked.update(int(r) for r in re.findall(r"#(\d+)", body))
    next_id = max(ents) + 1
    edits = []                         # (start, end, replacement)
    extra = []
    count = 0
    for k, (s, e, body) in sorted(ents.items()):
        if k in blocked:
            continue
        bm = _BSPLINE.match(body)
        if not bm:
            bm = _BSPLINE_COMPLEX.match(body)
            if not bm or not (_num(bm.group(3)) > 0 and _num(bm.group(4)) > 0):
                continue
        a, b = int(bm.group(1)), int(bm.group(2))
        if a not in pts or b not in pts or len(pts[a]) != 3 or len(pts[b]) != 3:
            continue
        d = [pts[b][i] - pts[a][i] for i in range(3)]
        ln = math.sqrt(sum(x * x for x in d))
        if ln <= tol:
            continue
        u = [x / ln for x in d]
        did, vid = next_id, next_id + 1
        next_id += 2
        extra.append("#{}=DIRECTION('',({},{},{}));".format(did, *(_fmt(x) for x in u)))
        extra.append("#{}=VECTOR('',#{},{});".format(vid, did, _fmt(ln)))
        edits.append((s, e, "#{}=LINE('',#{},#{});".format(k, a, vid)))
        count += 1
    if not count:
        return text, 0
    out, pos = [], 0
    for s, e, rep in edits:
        out.append(text[pos:s])
        out.append(rep)
        pos = e + 1                    # the regex stops before ';'
    out.append(text[pos:])
    new = "".join(out)
    i = new.rfind("ENDSEC", 0, new.rfind("END-ISO-10303-21"))
    new = new[:i] + "\n".join(extra) + "\n" + new[i:]
    return new, count


def main(argv=None) -> int:
    import argparse  # noqa: PLC0415

    ap = argparse.ArgumentParser(prog="python -m lithocam.step_canon", description=__doc__.split("\n\n")[0])
    ap.add_argument("src")
    ap.add_argument("dst")
    a = ap.parse_args(argv)
    with open(a.src, encoding="utf-8", errors="replace") as f:
        text = f.read()
    new, n = canonical_lines(text)
    with open(a.dst, "w", encoding="utf-8", newline="\n") as f:
        f.write(new)
    print("{} degree-1 B-spline edges rewritten as LINE".format(n))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
