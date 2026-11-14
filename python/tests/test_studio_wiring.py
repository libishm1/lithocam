# Copyright 2026 The lithocam authors. SPDX-License-Identifier: Apache-2.0
"""Hole cycles, rest, pencil, adaptive raster, polishing, disc cuts on ruled faces, the boundary of finishing passes,
spindle reverse for tapping, tool wear and feed limits, and the table angle per sample, wired into the studio
(program expansion, posting, process check). No Qt."""
import copy
import math
from pathlib import Path

import pytest

np = pytest.importorskip("numpy")
try:
    from lithocam.studio.core import lc  # noqa: F401
except ImportError:
    pytest.skip("_lithocam not built (bindings/); set LITHOCAM_CORE", allow_module_level=True)
pytest.importorskip("manifold3d")

from lithocam import cell_io, post  # noqa: E402
from lithocam.studio import api, cam, catalog, ops, process, program  # noqa: E402

from test_studio_ops import dome, plate_with_holes, pocket  # noqa: E402

RULED = Path(__file__).parent / "data" / "ruled_json"
KR_IO = Path(__file__).parents[2] / "cells" / "kr120_io.json"


def _tool(tools, tid, kind, d, **kw):
    t = lc.Tool()
    t.id, t.name, t.kind, t.diameter = tid, tid, kind, d
    for k, v in kw.items():
        setattr(t, k, v)
    tools[tid] = t


@pytest.fixture(scope="module")
def tools():
    out = {t.id: t for t in list(lc.stone_tool_library().tools) + list(lc.wood_tool_library().tools)}
    _tool(out, "TAP6", lc.ToolKind.Tap, 6.0, thread_pitch=1.0)
    _tool(out, "DR6", lc.ToolKind.Drill, 6.0, spindle_rpm=2000, feed=100)
    return out


def _station(part_tris, robot="kr120", work=(1.6, 0.0, 0.3), **kw):
    return program.Station(robot=robot, work_xyz=list(work), stock=cam.stock_box(part_tris, 3.0), **kw)


def _io_with_reverse():
    d = cell_io.load(KR_IO).data
    d = copy.deepcopy(d)
    d["spindle"]["reverse"] = {"start": [{"do": 4, "value": True}], "stop": [{"do": 4, "value": False}]}
    return d


def _hole_program(extra=()):
    return program.Program("Holes", [
        program.Step("set_tool", {"tool": "DR6"}),
        program.Step("drill", {"tool": "DR6", "points": "15,20; 45,20", "depth": 6, "peck": 3, "dwell": 0.5,
                               "feed": 100, "rpm": 2000}),
        program.Step("tap", {"tool": "TAP6", "points": "15,20", "depth": 5, "pitch": 1.0, "rpm": 300, "dwell": 0.2}),
    ] + list(extra))


# ---------------------------------------------------------------- 1. operations in the catalogue and the expander
def test_ops_generators_are_registered_and_catalogued():
    assert set(ops.GENERATORS) <= set(cam.GENERATORS)
    for k in ops.GENERATORS:
        spec = catalog.BY_ID[k]
        assert spec.status == "done"
    assert {s.id for s in catalog.steps_in("Holes")} >= {"drill", "core_drill", "tap", "bore"}
    assert {"rest", "pencil", "adaptive_raster"} <= {s.id for s in catalog.steps_in("3-axis")}
    assert catalog.BY_ID["polish"].category == "Finishing" and catalog.BY_ID["disc_ruled"].category == "Stone"
    # every parameter the generators read is a catalogue parameter (the api refuses unknown ones)
    s = api.Session("kr120")
    for k in ("drill", "core_drill", "tap", "bore", "rest", "pencil", "adaptive_raster", "polish", "disc_ruled"):
        s._check_params(catalog.BY_ID[k], {key: d for key, _, _, d, _ in catalog.BY_ID[k].params})


def test_drill_tap_expand_with_dwell_and_no_motion_for_the_stop(tools):
    plate = plate_with_holes()
    part = (plate, cam.trimesh(plate))
    st = _station(plate)
    ex = program.expand(st, program.load_robot("kr120"), _hole_program(), tools, part)
    assert ex.unreachable == 0 and not ex.errors, ex.messages
    dw = [r.data["seconds"] for r in ex.records if r.kind == "dwell"]
    assert dw == [0.5, 0.5, 0.2]
    # a dwell is a record, not a sample: nothing moves while it runs
    recs = ex.records
    for k, r in enumerate(recs):
        if r.kind == "dwell":
            assert recs[k - 1].kind in ("movel", "movej")
    # tap feeds are the move's own: pitch x rpm = 300 mm/min = 5 mm/s
    taps = [r for r in recs if r.kind == "movel" and r.data.get("cut") and abs(r.data["v"] - 5.0) < 1e-9]
    assert len(taps) == 2                                         # in and out
    sp = [r.data for r in recs if r.kind == "spindle" and r.data.get("on")]
    assert [bool(d.get("reverse")) for d in sp if d["rpm"] == 300.0] == [False, True]
    assert any("Tap" in m and "CL points" in m for m in ex.messages)


def test_tap_out_needs_a_spindle_reverse_output_in_robot_posts(tools):
    plate = plate_with_holes()
    part = (plate, cam.trimesh(plate))
    ex = program.expand(_station(plate), program.load_robot("kr120"), _hole_program(), tools, part)
    for dialect in ("krl", "rapid", "urscript"):
        with pytest.raises(ValueError, match="spindle reverse"):
            post.write(ex.records, dialect)
    # a cell I/O that defines the reverse output: the posts run, the reverse output is switched around the way out
    io = _io_with_reverse()
    text = post.write(ex.records, "krl", io=io)
    assert "$OUT[4] = TRUE" in text and "$OUT[4] = FALSE" in text
    assert "WAIT SEC 0.500" in text and "WAIT SEC 0.200" in text


def test_dwell_posts_in_every_dialect_and_gcode_reverses_with_m4_m3(tools):
    plate = plate_with_holes()
    part = (plate, cam.trimesh(plate))
    st = _station(plate)
    prog = program.Program("D", [program.Step("set_tool", {"tool": "DR6"}),
                                 program.Step("drill", {"tool": "DR6", "points": "15,20", "depth": 6, "dwell": 0.5,
                                                        "feed": 100, "rpm": 2000})])
    ex = program.expand(st, program.load_robot("kr120"), prog, tools, part)
    assert "WAIT SEC 0.500" in post.write(ex.records, "krl")
    assert "WaitTime 0.500;" in post.write(ex.records, "rapid")
    assert "sleep(0.500)" in post.write(program.expand(
        _station(plate, "ur10e", (0.55, 0.15, 0.05)), program.load_robot("ur10e"), prog, tools, part).records,
        "urscript")
    # G-code: a controller that reverses by M4: tap_out posts M4 S, the next forward move M3 S again
    ex = program.expand(st, program.load_robot("kr120"), _hole_program(), tools, part)
    mach = Path(__file__).parents[2] / "examples" / "gcode_machines" / "generic_3axis.json"
    texts = post.write_files(ex.records, "gcode", machine=str(mach), work_frame=st.work_frame())
    g = "\n".join(texts)
    assert "G4 P0.5" in g and "G4 P0.2" in g
    lines = [ln.strip() for ln in g.splitlines()]
    k4 = [i for i, ln in enumerate(lines) if "M4 S300" in ln]
    assert k4 and any("S300" in ln and "M4" not in ln for ln in lines[:k4[0]]), g[:600]   # in forward, out reversed
    assert any(ln.startswith("N") and "G1" in ln and "Z" in ln for ln in lines[k4[0]:])   # the move out follows


def test_cell_io_reverse_validates_and_stops_with_the_spindle():
    d = _io_with_reverse()
    cell_io.from_dict(d)
    bad = copy.deepcopy(d)
    bad["spindle"]["reverse"] = {"start": [{"do": 99, "value": True}, {"nonsense": 1}]}
    with pytest.raises(ValueError):
        cell_io.from_dict(bad)
    proc = cell_io.Process(cell_io.from_dict(d))
    proc.ops("spindle", {"rpm": 300.0, "on": True})
    r = proc.ops("spindle", {"rpm": 300.0, "on": True, "reverse": True})
    stop, start = d["spindle"]["stop"][0], d["spindle"]["start"][0]
    assert ("do", stop["do"], stop["value"]) in r and ("do", 4, True) in r      # forward stops, reverse starts
    f = proc.ops("spindle", {"rpm": 300.0, "on": True})
    assert ("do", 4, False) in f and ("do", start["do"], start["value"]) in f   # reverse stops, forward starts again
    proc.ops("spindle", {"rpm": 300.0, "on": True, "reverse": True})
    off = proc.ops("spindle", {"rpm": 0.0, "on": False})
    assert ("do", 4, False) in off and "spindle" not in proc.still_on()


def test_core_drill_and_bore_expand(tools):
    plate = plate_with_holes()
    part = (plate, cam.trimesh(plate))
    st = _station(plate)
    prog = program.Program("H", [
        program.Step("core_drill", {"tool": "S3", "points": "15,20", "depth": 8, "peck": 4, "feed": 60, "rpm": 800}),
        program.Step("bore", {"tool": "W2", "points": "", "diameter": 0.0, "depth": 0.0, "pitch": 2.0,
                              "feed": 300, "rpm": 3000, "tolerance": 0.05, "diameter_range": "7,9"}),
    ])
    ex = program.expand(st, program.load_robot("kr120"), prog, tools, part)
    assert not ex.errors and ex.unreachable == 0, ex.messages
    assert any("Core drill" in m and "CL points" in m for m in ex.messages)
    assert any("Bore" in m and "CL points" in m for m in ex.messages)       # the detected blind hole d 8


def test_rest_takes_the_previous_milling_tool_and_an_empty_region_is_a_message(tools):
    t = pocket()
    part = (t, cam.trimesh(t))
    st = _station(t)
    prog = program.Program("R", [
        program.Step("raster", {"tool": "S2", "stepover": 8.0, "feed": 800, "rpm": 6000, "tolerance": 0.05}),
        program.Step("rest", {"tool": "S1", "stepover": 3.0, "feed": 500, "rpm": 8000, "tolerance": 0.05}),
        program.Step("rest", {"tool": "S2", "stepover": 3.0, "feed": 500, "rpm": 8000, "tolerance": 0.05}),
    ])
    ex = program.expand(st, program.load_robot("kr120"), prog, tools, part)
    assert "step 2: rest: leftover from a Z-map of steps 1..1 at 1 mm" in ex.messages, ex.messages
    assert any("step 2: Rest machining: " in m and "CL points" in m for m in ex.messages)
    assert not ex.errors
    # with an explicit radius equal to the tool's own, nothing is left: an empty move list is accepted
    prog.steps[2].params["previous_tool_radius"] = 25.4
    ex = program.expand(st, program.load_robot("kr120"), prog, tools, part)
    assert any("step 3" in m and "nothing to cut" in m for m in ex.messages), ex.messages
    assert not ex.errors


def test_rest_from_a_leftover_stock_and_without_any_previous_tool(tools):
    t = pocket()
    part = (t, cam.trimesh(t))
    st = _station(t)
    prog = program.Program("R", [program.Step("rest", {"tool": "S1", "stepover": 3.0, "feed": 500, "rpm": 8000})])
    ex = program.expand(st, program.load_robot("kr120"), prog, tools, part)
    assert any("failed" in m and "previous_tool_radius" in m for m in ex.messages)       # says what is missing
    xs = np.arange(-5.0, 56.0, 1.0)
    Z = np.full((len(xs), len(xs)), 20.0)                                               # a block, uncut
    ex = program.expand(st, program.load_robot("kr120"), prog, tools, part, leftover=(xs, xs, Z))
    assert any("Rest machining" in m and "CL points" in m for m in ex.messages), ex.messages


def test_pencil_and_adaptive_raster_expand(tools):
    t = pocket()
    part = (t, cam.trimesh(t))
    st = _station(t)
    prog = program.Program("P", [
        program.Step("pencil", {"tool": "W7", "feed": 500, "rpm": 8000, "tolerance": 0.05}),
        program.Step("adaptive_raster", {"tool": "W7", "scallop": 0.05, "feed": 500, "rpm": 8000, "tolerance": 0.05}),
    ])
    ex = program.expand(st, program.load_robot("kr120"), prog, tools, part)
    assert not ex.errors, ex.messages
    assert any("Pencil" in m and "CL points" in m for m in ex.messages)
    assert any("adaptive" in m.lower() and "CL points" in m for m in ex.messages)


# ---------------------------------------------------------------- 2. polishing
def _polish_prog(**kw):
    p = {"tool": "S8", "pressure": 0.05, "feed": 1500.0, "water": "flood", "overlap": 0.3, "tolerance": 0.05}
    p.update(kw)
    return program.Program("Pol", [program.Step("polish", p)])


def test_polish_runs_the_sourced_granite_sequence_with_a_pad_change_per_grit(tools):
    t = dome()
    part = (t, cam.trimesh(t))
    st = _station(t, "kr120", (1.6, 0.0, 0.3), material="granite")
    ex = program.expand(st, program.load_robot("kr120"), _polish_prog(), tools, part)
    assert not ex.errors, ex.errors
    changes = [r for r in ex.records if r.kind == "tool_change"]
    assert len(changes) == 5                                       # 50-100, 200-400, 800-1500, 3000, buff
    comments = [r.data["text"] for r in ex.records if r.kind == "comment"]
    assert sum(c.startswith("polish grit") and "simulation attributes" in c for c in comments) == 5
    assert any(c.startswith("pass: grit 50-100, pressure 0.05 MPa") for c in comments)
    assert any("grit buff" in c for c in comments)
    rpms = [r.data["rpm"] for r in ex.records if r.kind == "spindle" and r.data.get("on")]
    assert len(rpms) == 5 and rpms == sorted(rpms) or len(set(rpms)) >= 4                 # S per grit from the bands
    feeds = {round(r.data["v"], 6) for r in ex.records if r.kind == "movel" and r.data.get("cut")}
    assert feeds == {1500.0 / 60.0, 1500.0 / 120.0}                  # the feed, and the half-feed plunge
    assert any("5-axis" in m for m in ex.messages)                  # the pad axis follows the normal: the planner spins
    assert any("polish: 5 grit step" in m for m in ex.messages)


def test_polish_refuses_without_a_sourced_sequence_or_a_pressure(tools):
    t = dome()
    part = (t, cam.trimesh(t))
    st = _station(t, "kr120", (1.6, 0.0, 0.3), material="marble")
    ex = program.expand(st, program.load_robot("kr120"), _polish_prog(), tools, part)
    assert any("no sourced grit sequence for 'marble'" in e for e in ex.errors), ex.errors
    ok = program.expand(st, program.load_robot("kr120"), _polish_prog(grits="400, 800", rpm=1200.0), tools, part)
    assert not ok.errors, ok.errors
    assert len([r for r in ok.records if r.kind == "tool_change"]) == 2
    st.material = "granite"
    ex = program.expand(st, program.load_robot("kr120"), _polish_prog(pressure=0.0), tools, part)
    assert any("pressure must be given" in e for e in ex.errors)
    ex = program.expand(st, program.load_robot("kr120"), _polish_prog(feed=0.0), tools, part)
    assert any("feed must be given" in e for e in ex.errors)


# ---------------------------------------------------------------- 3. disc saw on ruled faces
def test_disc_ruled_reports_feasibility_per_face_and_cuts(tools):
    t = pocket()
    st = program.Station(robot="kr120", work_xyz=[1.2, -0.25, 0.3], stock={"xmin": -400, "xmax": 400, "ymin": -400,
                                                                         "ymax": 400, "zmin": -400, "zmax": 400},
                         ruled_path=str(RULED / "twisted_column.json"))
    prog = program.Program("S", [program.Step("disc_ruled", {"tool": "SAW350", "faces": "0", "tol": 0.5,
                                                             "stepdown": 40.0, "feed": 100.0, "rpm": 700.0})])
    lib = dict(tools)
    _tool(lib, "SAW350", lc.ToolKind.DiscSaw, 350.0, kerf=3.2, segment_height=3.2, spindle_rpm=700.0)
    ex = program.expand(st, program.load_robot("kr120"), prog, lib, None)
    face = [m for m in ex.messages if "disc face 0" in m]
    assert face and "rulings cuttable" in face[0] and "twist" in face[0], ex.messages
    assert any("cuts with SAW350" in (r.data.get("text") or "") for r in ex.records if r.kind == "comment")
    assert ex.segments and sum(len(s.q) for s in ex.segments) > 20


def test_disc_ruled_without_a_ruled_file_is_a_message(tools):
    st = program.Station(robot="kr120", stock={"xmin": 0, "xmax": 1, "ymin": 0, "ymax": 1, "zmin": 0, "zmax": 1})
    lib = dict(tools)
    _tool(lib, "SAW350", lc.ToolKind.DiscSaw, 350.0, kerf=3.2)
    ex = program.expand(st, program.load_robot("kr120"), program.Program("S", [
        program.Step("disc_ruled", {"tool": "SAW350"})]), lib, None)
    assert any("ruled-face JSON" in m for m in ex.messages)


# ---------------------------------------------------------------- 4. boundary of the finishing passes
def test_boundary_parses_and_clips():
    assert cam.parse_boundary("") is None and cam.parse_boundary(None) is None
    P = cam.parse_boundary("0,0,10,5")
    assert P.shape == (4, 2) and np.allclose(P[0], [0, 0]) and np.allclose(P[2], [10, 5])
    tri = cam.parse_boundary("0,0; 10,0; 0,10")
    assert len(tri) == 3
    with pytest.raises(ValueError):
        cam.parse_boundary("0,0,10")
    run = np.array([[-5.0, 2.0, 0.0], [15.0, 2.0, 20.0]])
    c = cam.clip_polyline(run, P)
    assert len(c) == 1 and np.allclose(c[0][:, 0], [0.0, 10.0]) and np.allclose(c[0][:, 2], [5.0, 15.0])
    # a loop inside is kept whole; a polyline that leaves and returns splits in two
    inside = np.array([[1.0, 1.0, 0], [9.0, 1.0, 0], [9.0, 4.0, 0], [1.0, 1.0, 0]])
    assert len(cam.clip_polyline(inside, P)) == 1
    vee = np.array([[1.0, 1.0, 0], [5.0, 8.0, 0], [9.0, 1.0, 0]])
    assert len(cam.clip_polyline(vee, P)) == 2


@pytest.mark.parametrize("kind", ["raster", "steep_shallow", "waterline"])
def test_boundary_keeps_every_pass_inside(kind, tools):
    t = dome()
    mesh = cam.trimesh(t)
    stock = cam.stock_box(t, 3.0)
    box = (-10.0, -8.0, 12.0, 9.0)
    kw = dict(stepover=3.0, tolerance=0.05) if kind != "waterline" else dict(stepdown=1.0, tolerance=0.05)
    full = cam.GENERATORS[kind](t, mesh, tools["W7"], stock, **kw)
    cut = cam.GENERATORS[kind](t, mesh, tools["W7"], stock, boundary="{:g},{:g},{:g},{:g}".format(*box), **kw)
    feed = [m[1] for m in cut if m[0] in ("feed", "plunge")]
    assert feed and len(feed) < len([m for m in full if m[0] in ("feed", "plunge")])
    P = np.array(feed)
    assert P[:, 0].min() >= box[0] - 1e-9 and P[:, 0].max() <= box[2] + 1e-9
    assert P[:, 1].min() >= box[1] - 1e-9 and P[:, 1].max() <= box[3] + 1e-9


def test_boundary_on_surfacing5_and_a_polygon(tools):
    t = dome()
    mesh = cam.trimesh(t)
    stock = cam.stock_box(t, 3.0)
    mv = cam.surfacing5(t, mesh, tools["W7"], stock, stepover=4.0, tolerance=0.05,
                        boundary="-10,-5; 10,-5; 0,12")
    R = tools["W7"].diameter / 2
    C = np.array([(m[1] + R * m[2])[:2] for m in mv if m[0] == "feed"])         # the ball centres: what is clipped
    assert len(C) > 10
    poly = cam.parse_boundary("-10,-5; 10,-5; 0,12")

    def edge_dist(x, y):
        d = []
        for k in range(len(poly)):
            a, b = poly[k], poly[(k + 1) % len(poly)]
            u = np.clip(((x - a[0]) * (b[0] - a[0]) + (y - a[1]) * (b[1] - a[1])) / ((b - a) @ (b - a)), 0, 1)
            d.append(math.hypot(x - a[0] - u * (b[0] - a[0]), y - a[1] - u * (b[1] - a[1])))
        return min(d)
    assert all(cam._in_poly(poly, x, y) or edge_dist(x, y) < 1e-6 for x, y in C)


# ---------------------------------------------------------------- 5. tool wear, feed limits, the table per sample
@pytest.fixture(scope="module")
def milled(tools):
    t = dome()
    part = (t, cam.trimesh(t))
    st = program.Station(robot="ur10e", work_xyz=[0.55, 0.15, 0.05], stock=cam.stock_box(t, 3.0))
    prog = program.Program("Dome", [
        program.Step("set_tool", {"tool": "W7"}),
        program.Step("movej", {"joints": None}),
        program.Step("raster", {"tool": "W7", "stepover": 6.0, "tolerance": 0.05, "feed": 600.0, "rpm": 6000.0}),
    ])
    robot = program.load_robot("ur10e")
    ex = program.expand(st, robot, prog, tools, part)
    run = process.simulate_stock(st, robot, ex, tools, cell_mm=2.0, snapshots=4)
    return st, robot, ex, run, prog


def test_wear_is_reported_per_tool_only_with_constants(milled, tools):
    st, robot, ex, run, prog = milled
    # granite has a sourced G-ratio since the 2026-11-14 constants review: refuse it (None) to test the skip path
    rep = process.process_check(st, run, ex, robot, tools, {"specific_energy": 1.0, "radial_ratio": 0.0,
                                                            "axial_ratio": 0.0, "g_ratio": None}, prog=prog)
    assert rep.life is not None and rep.life.evaluated == 0
    assert any("tool life: not evaluated" in m and "g_ratio" in m for m in rep.messages), rep.messages
    rep = process.process_check(st, run, ex, robot, tools, {"specific_energy": 1.0, "radial_ratio": 0.0,
                                                            "axial_ratio": 0.0, "g_ratio": 50.0,
                                                            "usable_volume_mm3": 2000.0}, prog=prog)
    assert rep.life.evaluated > 0
    want = run.total_removed / 50.0 / 2000.0
    assert rep.life.program_fraction == pytest.approx(want, rel=1e-9)
    assert rep.life.per_tool["W7"] == pytest.approx(want, rel=1e-9)
    assert any(r.get("life_fraction") for r in rep.steps)
    assert any("tool W7:" in m for m in rep.messages)
    assert rep.wear_by_step[2] == pytest.approx(want, rel=1e-9)           # the raster step of the program
    assert any("program step 3:" in m for m in rep.messages)
    # a refused key (given as None) is not filled from a row, and the constants dialog lists the new keys
    keys = {c.key for c in process.CONSTANTS + process.OPTIONAL}
    assert {"g_ratio", "wear_per_area", "usable_volume_mm3", "usable_height_mm", "tool_E", "flute_factor",
            "radial_ratio", "axial_ratio", "tool_deflection_limit_mm", "robot_deflection_limit_mm"} <= keys


def test_feed_limit_per_cutting_step_names_the_binding_limit_and_the_skips(milled, tools):
    st, robot, ex, run, prog = milled
    fl = process.step_feed_limits(st, run, ex, robot, tools, prog, {"specific_energy": 30.0})
    assert list(fl) == [2]                                       # the raster step
    sf = fl[2]
    assert sf.programmed == pytest.approx(600.0) and sf.rpm == 6000.0
    assert sf.limit.skipped                                        # tool bending and robot deflection need limits
    assert any(a == "tool_deflection" for a, _ in sf.limit.skipped)
    assert "feed limit:" in sf.tooltip() and "skipped" in sf.tooltip()
    assert sf.badge()
    st2 = copy.deepcopy(st)
    st2.spindle = {"id": "t", "name": "t", "power_w": 2000.0, "max_rpm": 24000.0}
    fl = process.step_feed_limits(st2, run, ex, robot, tools, prog, {"specific_energy": 30.0})
    assert fl[2].limit.binding in ("power", "sourced_feed", "chip_load")
    assert fl[2].limit.feed is not None


def test_stock_uses_the_table_angle_of_each_sample(milled, tools):
    st, robot, ex, run, prog = milled
    # the same expansion with the table angle turning inside the segment: the samples cut where the part has turned
    ex2 = copy.deepcopy(ex)
    for seg in ex2.segments:
        assert seg.e1 is not None and len(seg.e1) == len(seg.q)          # every segment carries it per sample
    seg = next(s for s in ex2.segments if s.feed.any())
    seg.e1 = seg.e1 + np.linspace(0.0, math.radians(40.0), len(seg.q))
    st2 = copy.deepcopy(st)
    st2.turntable = {"diameter": 0.8, "height": 0.04}
    run_a = process.simulate_stock(st2, robot, ex, tools, cell_mm=2.0, snapshots=2)
    run_b = process.simulate_stock(st2, robot, ex2, tools, cell_mm=2.0, snapshots=2)
    assert abs(run_a.total_removed - run_b.total_removed) > 1.0          # a turning table changes what is cut
    # a constant e1 equal to the yaw gives the old result
    ex3 = copy.deepcopy(ex)
    for s in ex3.segments:
        s.e1 = np.full(len(s.q), s.yaw)
    run_c = process.simulate_stock(st2, robot, ex3, tools, cell_mm=2.0, snapshots=2)
    assert run_c.total_removed == pytest.approx(run_a.total_removed, rel=1e-12)


def test_ur10e_cannot_drive_the_table_as_e1_and_says_why():
    st = program.Station(robot="ur10e", turntable={"diameter": 0.8, "height": 0.04, "driven": "e1"})
    mode, errs, _ = program.check_table(st)
    assert mode == "operator" and errs and "no external axis" in errs[0]
    ok = program.Station(robot="kr120", turntable={"diameter": 0.8, "height": 0.04, "driven": "e1"})
    mode, errs, warns = program.check_table(ok)
    assert mode == "e1" and not errs and any("PLACEHOLDER" in w for w in warns)
    ts = program.table_settings(ok.turntable)
    assert set(ts["placeholders"]) == {"e1_limits_deg", "e1_speed_deg_s", "e1_max_step_deg"}
    spec = catalog.BY_ID["wire_ruled"]
    keys = {k for k, *_ in spec.params}
    assert {"table_window_deg", "table_step_deg", "table_weight"} <= keys


@pytest.mark.parametrize("name", ["disc_saw_kr120", "disc_comb_ur10e"])
def test_disc_example_stations_reach_every_sample(name):
    s = api.Session.open(Path(__file__).parents[2] / "examples" / "studio" / (name + ".lcs.json"))
    r = s.simulate(s.programs()[0])
    assert r.ok and r.unreachable == 0 and not r.errors, r.summary()
