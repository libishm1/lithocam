# Copyright 2026 The lithocam authors. SPDX-License-Identifier: Apache-2.0
"""Straight-edge canonicalisation (lithocam.step_canon) and its effect on recognition of Rhino-written STEP."""
import json
import math
import re
from collections import Counter
from pathlib import Path

import pytest

from lithocam.features import recognise, stone_ops
from lithocam.step_canon import canonical_lines

DATA = Path(__file__).resolve().parent / "data"
HDR = "ISO-10303-21;\nHEADER;\nENDSEC;\nDATA;\n"
END = "ENDSEC;\nEND-ISO-10303-21;\n"


def ents(text):
    return {int(m.group(1)): m.group(2).strip()
            for m in re.finditer(r"#(\d+)\s*=\s*(.*?);(?=\s*(?:#\d+\s*=|ENDSEC))", text, re.S)}


def point(e, k):
    return [float(v) for v in re.search(r"\(([^()]*)\)\s*\)$", e[k]).group(1).split(",")]


def test_simple_and_rational_forms_become_exact_lines():
    text = (HDR
            + "#1=CARTESIAN_POINT('',(1.,2.,3.));\n#2=CARTESIAN_POINT('',(4.,6.,3.));\n"
            + "#3=B_SPLINE_CURVE_WITH_KNOTS('',1,(#1,#2),.UNSPECIFIED.,.F.,.F.,(2,\n2),(0.,5.),.UNSPECIFIED.);\n"
            + "#4=( BOUNDED_CURVE() B_SPLINE_CURVE(1,(#2,#1),.UNSPECIFIED.,.F.,.F.) B_SPLINE_CURVE_WITH_KNOTS((2,2),"
              "(1.,21.),.UNSPECIFIED.) CURVE() GEOMETRIC_REPRESENTATION_ITEM() RATIONAL_B_SPLINE_CURVE((1.,1.)) "
              "REPRESENTATION_ITEM('') );\n"
            + "#5=EDGE_CURVE('',#9,#9,#3,.T.);\n#6=EDGE_CURVE('',#9,#9,#4,.T.);\n" + END)
    new, n = canonical_lines(text)
    assert n == 2
    e = ents(new)
    for k, (a, b) in ((3, (1, 2)), (4, (2, 1))):
        m = re.match(r"LINE\('',#(\d+),#(\d+)\)", e[k])
        assert m and int(m.group(1)) == a
        vec = re.match(r"VECTOR\('',#(\d+),([^)]+)\)", e[int(m.group(2))])
        d = point(e, int(vec.group(1)))
        length = float(vec.group(2))
        pa, pb = point(e, a), point(e, b)
        end = [pa[i] + d[i] * length for i in range(3)]
        assert end == pytest.approx(pb, abs=1e-12)          # the line ends exactly on the second control point
        assert math.hypot(*d) == pytest.approx(1.0, abs=1e-15)
    assert "EDGE_CURVE('',#9,#9,#3,.T.)" in new                # edges keep their ids and vertices


def test_leaves_curves_that_cannot_be_rewritten():
    text = (HDR
            + "#1=CARTESIAN_POINT('',(0.,0.,0.));\n#2=CARTESIAN_POINT('',(1.,0.,0.));\n"
              "#7=CARTESIAN_POINT('',(1.,1.,0.));\n"
            + "#3=B_SPLINE_CURVE_WITH_KNOTS('',1,(#1,#2),.UNSPECIFIED.,.F.,.F.,(2,2),(0.,1.),.UNSPECIFIED.);\n"
            + "#4=PCURVE('',#9,#3);\n"                                              # parameterisation is used
            + "#5=B_SPLINE_CURVE_WITH_KNOTS('',2,(#1,#7,#2),.UNSPECIFIED.,.F.,.F.,(3,3),(0.,1.),.UNSPECIFIED.);\n"
            + "#6=B_SPLINE_CURVE_WITH_KNOTS('',1,(#1,#1),.UNSPECIFIED.,.F.,.F.,(2,2),(0.,1.),.UNSPECIFIED.);\n"
            + END)
    new, n = canonical_lines(text)
    assert n == 0 and new == text


@pytest.mark.parametrize("name", ["cladding_panel", "window_sill", "worktop", "stair_tread"])
def test_rhino_written_parts_rewrite_every_straight_edge(name):
    text = (DATA / "step_rhino" / (name + ".step")).read_text(encoding="utf-8")
    before = Counter(re.findall(r"=\s*(LINE|B_SPLINE_CURVE_WITH_KNOTS)\s*\(", text))
    new, n = canonical_lines(text)
    assert before["LINE"] == 0 and n > 0                         # Rhino writes no LINE entities
    assert Counter(re.findall(r"=\s*(LINE)\s*\(", new))["LINE"] == n
    assert not re.search(r"B_SPLINE_CURVE_WITH_KNOTS\s*\(\s*'[^']*'\s*,\s*1\s*,\s*\(\s*#\d+\s*,\s*#\d+\s*\)", new)


def key(fs):
    return sorted((op.kind, op.route, tuple(round(v, 3) for v in op.at),
                   tuple(sorted((k, round(v, 3)) for k, v in op.params.items()
                                if isinstance(v, (int, float)) and not isinstance(v, bool))))
                  for op in fs.ops)


def golden(name):
    return stone_ops(json.loads((DATA / "quiddity" / (name + ".json")).read_text(encoding="utf-8")))


@pytest.mark.parametrize("name", ["cladding_panel", "window_sill", "worktop", "stair_tread"])
def test_rhino_part_gives_the_same_operations_as_build123d(name):
    assert key(golden("rhino_" + name)) == key(golden(name))


def test_known_limits_are_still_limits():
    """Rhino writes cones and general revolves, and a cutter extruded as one surface, as B-spline faces.

    quiddity 0.4.9 reports those faces as freeform. If a quiddity upgrade fixes this, these asserts fail: then move
    the parts into the test above.
    """
    bal = golden("rhino_baluster")
    assert not any(op.kind == "turned_profile" for op in bal.ops)
    assert any(op.kind == "freeform_surfaces" for op in bal.ops)
    wt = golden("rhino_worktop_one_face_cutter")
    assert not any(op.kind == "cutout" for op in wt.ops)


@pytest.mark.parametrize("name", ["cladding_panel", "window_sill", "worktop", "stair_tread"])
def test_end_to_end_rhino_step(name):
    try:
        fs = recognise(str(DATA / "step_rhino" / (name + ".step")))
    except RuntimeError as e:
        pytest.skip(str(e))
    assert fs.canonicalised_edges > 0
    assert key(fs) == key(golden(name))
