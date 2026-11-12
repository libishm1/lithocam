# Copyright 2026 The lithocam authors. SPDX-License-Identifier: Apache-2.0
"""Smoke and agreement tests for the generated `_lithocam` Python module.

    python bindings/test_bindings.py <dir containing _lithocam*.pyd/.so> [build/oracle dir]
"""
import math
import sys

sys.path.insert(0, sys.argv[1])
import _lithocam as lc  # noqa: E402

# scallop: R 5, stepover 4 -> 5 - sqrt(25 - 4)
assert abs(lc.scallop_height(5.0, 4.0) - (5 - math.sqrt(21))) < 1e-12
assert abs(lc.scallop_height(5.0, lc.max_stepover(5.0, 0.2)) - 0.2) < 1e-12

# kerf around a 100 x 50 rectangle: path never inside part + kerf/2
part = []
for x, y in ((0, 0), (100, 0), (100, 50), (0, 50)):
    v = lc.Vec2()
    v.x, v.y = x, y
    part.append(v)
path = lc.kerf_path_convex(part, 3.0, 0.05)
assert len(path) > 4
assert min(lc.signed_distance_convex(part, p) for p in path) >= 1.5 - 1e-9


def P(x, y, z):
    p = lc.P3()
    p.x, p.y, p.z = x, y, z
    return p


def T(a, b, c):
    t = lc.Tri()
    t.a, t.b, t.c = a, b, c
    return t


cut = lc.Cutter()
cut.kind, cut.radius = lc.CutterKind.Ball, 4.0
spike = T(P(0, 0, 5), P(50, 0, -5), P(0, 50, -5))
assert abs(lc.drop_triangle(cut, spike, -3, 0) - (5 + math.sqrt(7) - 4)) < 1e-12
mesh = lc.TriMesh([spike], 0.0)
assert abs(lc.drop_cutter(cut, mesh, -3, 0) - (5 + math.sqrt(7) - 4)) < 1e-12

# agreement with the C++ CLI on the oracle case, when present
if len(sys.argv) > 2:
    import pathlib
    o = pathlib.Path(sys.argv[2])
    tris = []
    for line in (o / "tris.txt").read_text().splitlines():
        v = [float(s) for s in line.split()]
        tris.append(T(P(*v[0:3]), P(*v[3:6]), P(*v[6:9])))
    m = lc.TriMesh(tris, 0.0)
    pts = [tuple(float(s) for s in l.split()) for l in (o / "points.txt").read_text().splitlines()]
    cut.kind, cut.radius = lc.CutterKind.Bull, 3.0
    cut.corner = 1.0
    ref = [float(s) for s in (o / "lc_bull_py.txt").read_text().split()] if (o / "lc_bull_py.txt").exists() else None
    got = [lc.drop_cutter(cut, m, x, y) for x, y in pts]
    if ref:
        worst = max(abs(a - b) for a, b in zip(got, ref))
        print("python vs C++ CLI (bull R3 c1), points", len(got), "max diff", worst)
        assert worst < 1e-12

# tool library: starter set, drop-cutter mapping, CSV round trip, errors as Python exceptions
lib = lc.stone_tool_library()
wood = lc.wood_tool_library()
lib.tools = list(lib.tools) + list(wood.tools)
assert len(lib.tools) == 63 and all(lc.validate_tool(t) == "" for t in lib.tools)
c = lc.cutter_of(lc.find_tool(lib, "W7"))
assert c.kind == lc.CutterKind.Ball and abs(c.radius - 3.175) < 1e-12
assert lc.tool_kind_from_name(lc.tool_kind_name(lc.ToolKind.Engraver)) == lc.ToolKind.Engraver
assert len(lc.all_tool_kinds()) == 38 and lc.tool_kind_envelope(lc.ToolKind.CupWheel) == "R-ANNULUS"
assert [t.id for t in lc.tools_for_operation(lib, "wire_cut")] == ["S17", "S18"]
back = lc.parse_tool_library(lc.format_tool_library(lib))
assert [t.id for t in back.tools] == [t.id for t in lib.tools]
assert not lc.has_cutter(lc.find_tool(lib, "S15"))
for bad in (lambda: lc.cutter_of(lc.find_tool(lib, "S15")), lambda: lc.tool_kind_from_name("spoon")):
    try:
        bad()
        raise AssertionError("expected an exception")
    except ValueError:
        pass
try:
    lc.find_tool(lib, "nope")
    raise AssertionError("expected an exception")
except IndexError:
    pass
print("bindings OK")
