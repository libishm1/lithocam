# Copyright 2026 The lithocam authors. SPDX-License-Identifier: Apache-2.0
"""Engraving, chamfering, flat lands, ramp and helix entries, and the setup sheet (studio.ops, studio.cam,
studio.sheet, studio.api). No Qt."""
import math

import numpy as np
import pytest

try:
    from lithocam.studio.core import lc  # noqa: F401
except ImportError:
    pytest.skip("_lithocam not built (bindings/); set LITHOCAM_CORE", allow_module_level=True)
mf = pytest.importorskip("manifold3d")

from lithocam.studio import api, cam, catalog, ops, program, sheet  # noqa: E402

from test_studio_ops import _tris, surface  # noqa: E402

TOL = 0.01


@pytest.fixture(scope="module")
def tools():
    return {t.id: t for t in list(lc.stone_tool_library().tools) + list(lc.wood_tool_library().tools)}


def plate(x=60.0, y=40.0, z=10.0):
    return _tris(mf.Manifold.cube([x, y, z]))


def stepped():
    """60 x 40 base 10 high with a 30 x 40 x 5 step on its left half: lands at z = 15 (x 0..30) and z = 10."""
    return _tris(mf.Manifold.cube([60, 40, 10]) + mf.Manifold.cube([30, 40, 5]).translate([0, 0, 10]))


def prepared(t, margin=3.0):
    return t, cam.trimesh(t), cam.stock_box(t, margin)


def zmap_reader(tool, moves, stock, cell=0.25):
    xs, ys, Z = ops.zmap_leftover([(tool, moves)], stock, cell)

    def at(x, y):
        return float(Z[int(round((y - ys[0]) / cell)), int(round((x - xs[0]) / cell))])
    return at


def cutting(moves):
    return [m for m in moves if m[0] in ("feed", "plunge")]


# ---------------------------------------------------------------- catalogue
def test_new_steps_are_done_with_defaults_and_tooltips():
    for sid in ("engrave", "chamfer", "flat_land"):
        spec = catalog.BY_ID[sid]
        assert spec.status == "done" and spec.doc and sid in cam.GENERATORS
    for sid in ("face", "rough_plane", "flat_land"):
        keys = {k for k, *_ in catalog.BY_ID[sid].params}
        assert {"entry", "ramp_angle", "helix_radius", "helix_pitch"} <= keys
        assert catalog.default_params(catalog.BY_ID[sid])["entry"] == "plunge"
        assert catalog.default_params(catalog.BY_ID[sid])["ramp_angle"] == 3.0
    s = api.Session("kr120")
    for sid in ("engrave", "chamfer", "flat_land", "face", "rough_plane"):
        s._check_params(catalog.BY_ID[sid], {k: d for k, _, _, d, _ in catalog.BY_ID[sid].params})


# ---------------------------------------------------------------- engraving
def test_parse_paths():
    P = ops.parse_paths("0,0; 10,0; 10,5 | 1,1; 2,2")
    assert [len(p) for p in P] == [3, 2] and P[0][2].tolist() == [10.0, 5.0]
    assert ops.parse_paths("") == [] and ops.parse_paths(None) == []
    with pytest.raises(ValueError, match="at least 2 points"):
        ops.parse_paths("1,1")
    with pytest.raises(ValueError, match="x,y"):
        ops.parse_paths("1,1,1; 2,2")


def test_engrave_depth_from_width_follows_the_cone_angle(tools):
    assert ops.engrave_depth(tools["S12"], 2.0) == pytest.approx(1.0)                  # 90 degree V: depth = w / 2
    assert ops.engrave_depth(tools["W9"], 2.0) == pytest.approx(1.0 / math.tan(math.radians(30)))
    with pytest.raises(ValueError, match="exceeds the tool diameter"):
        ops.engrave_depth(tools["S12"], 7.0)
    with pytest.raises(ValueError, match="tip angle"):
        ops.engrave_depth(tools["W1"], 1.0)


def test_engrave_cuts_the_groove_width_on_a_flat_top(tools):
    t, mesh, stock = prepared(plate())
    tool = tools["S12"]
    mv = ops.engrave(t, mesh, tool, stock, paths="10,10; 50,10 | 10,30; 50,30", width=2.0)
    cut = cutting(mv)
    assert cut and all(m[1][2] == pytest.approx(9.0) for m in cut)                      # 1 mm below the top
    assert sum(m[0] == "plunge" for m in mv) == 2                                       # two polylines
    zm = zmap_reader(tool, mv, stock)

    def z(x, y):                                  # the stone: the Z-map below the part's top (the stock above is air)
        return min(zm(x, y), 10.0)
    assert z(30, 10) == pytest.approx(9.0, abs=1e-6)
    assert z(30, 10.5) == pytest.approx(9.5, abs=0.02)                                  # the flank, 45 degrees
    assert z(30, 11.0) == pytest.approx(10.0, abs=0.02)                                 # groove edge: width 2 mm
    assert z(30, 11.5) == pytest.approx(10.0)                                           # untouched beyond it
    assert zm(30, 20) == pytest.approx(stock["zmax"])


def test_engrave_follows_a_sloped_top_at_constant_depth(tools):
    t = surface(lambda x, y: 0.01 * x * x + 0 * y, half=20.0, n=80, base=30.0)         # a parabolic trough
    _, mesh, stock = prepared(t)
    mv = ops.engrave(t, mesh, tools["S12"], stock, paths="-10,0; 10,0", depth=0.8, step=0.5, tolerance=TOL)
    cut = cutting(mv)
    assert len(cut) > 5
    for _, p, *_ in cut:
        assert p[2] == pytest.approx(ops.surface_z(mesh, p[0], p[1]) - 0.8, abs=1e-6)
    # between the posted points the straight move stays within the tolerance of the constant depth
    for (_, a, *_), (_, b, *_) in zip(cut[:-1], cut[1:]):
        m = (a + b) / 2
        assert abs(m[2] - (ops.surface_z(mesh, m[0], m[1]) - 0.8)) <= TOL + 0.005
    flat = ops.engrave(t, mesh, tools["S12"], stock, paths="-10,0; 10,0", depth=0.8, step=0.5)
    assert len(cutting(flat)) >= len(cut)


def test_engrave_skips_where_the_part_is_absent_and_refuses_bad_input(tools):
    t, mesh, stock = prepared(plate())
    mv = ops.engrave(t, mesh, tools["S12"], stock, paths="-20,10; 20,10", depth=1.0)   # starts off the part
    xs = [m[1][0] for m in cutting(mv)]
    assert min(xs) >= -1e-6                                                             # only over the part
    with pytest.raises(ValueError, match="single-stroke font"):
        ops.engrave(t, mesh, tools["S12"], stock, paths="1,1; 5,5", depth=1.0, text="LITHO", height=5.0)
    with pytest.raises(ValueError, match="paths"):
        ops.engrave(t, mesh, tools["S12"], stock, depth=1.0)
    with pytest.raises(ValueError, match="width"):
        ops.engrave(t, mesh, tools["S12"], stock, paths="1,1; 5,5")
    # a ball mill engraves by depth
    assert cutting(ops.engrave(t, mesh, tools["W7"], stock, paths="5,5; 20,5", depth=0.5))


# ---------------------------------------------------------------- chamfer
def test_top_outline_is_the_highest_flat_top_with_the_material_on_the_right():
    t = _tris(mf.Manifold.cube([40, 30, 5]) + mf.Manifold.cube([20, 30, 5]).translate([0, 0, 5]))
    loops = ops.top_outline(t)
    assert len(loops) == 1
    L = loops[0]
    assert np.allclose(L[0], L[-1]) and np.allclose(L[:, 2], 10.0)
    assert L[:, 0].max() == pytest.approx(20.0) and L[:, 1].max() == pytest.approx(30.0)
    area = 0.5 * np.sum(L[:-1, 0] * L[1:, 1] - L[1:, 0] * L[:-1, 1])
    assert area == pytest.approx(-600.0)                                                # clockwise: climb milling
    with pytest.raises(ValueError, match="no upward face"):
        ops.top_outline(np.array([[[0, 0, 0], [1, 0, 0], [0, 1, 1.0]]]) * 1.0, 0.1)


def test_chamfer_along_the_top_outline_cuts_a_45_degree_face(tools):
    t, mesh, stock = prepared(plate(40.0, 30.0, 10.0))
    tool = tools["S12"]                                                                 # 90 degree V, d 6
    mv = ops.chamfer(t, mesh, tool, stock, width=1.0)
    cut = cutting(mv)
    assert all(m[1][2] == pytest.approx(9.0) for m in cut)                              # tip width / tan(45) below
    on_edge = [min(abs(p[0]), abs(p[0] - 40), abs(p[1]), abs(p[1] - 30)) < 1e-6 for _, p, *_ in cut]
    assert all(on_edge)
    zm = zmap_reader(tool, mv, stock)

    def z(x, y):                                  # the stone: the Z-map below the part's top (the stock above is air)
        return min(zm(x, y), 10.0)
    assert z(0.5, 15) == pytest.approx(9.5, abs=0.02)                                   # chamfer face at 45 degrees
    assert z(1.0, 15) == pytest.approx(10.0, abs=0.02)                                  # its width on the top
    assert z(15, 0.5) == pytest.approx(9.5, abs=0.02)
    assert z(0.0, 15) == pytest.approx(9.0, abs=0.02)                                   # the tip sits on the wall
    assert z(20, 15) == pytest.approx(10.0)                                             # the middle of the top is left
    # a 60 degree bit: the depth grows as width / tan(30)
    mv2 = ops.chamfer(t, mesh, tools["W9"], stock, width=1.0)
    assert all(m[1][2] == pytest.approx(10.0 - 1.0 / math.tan(math.radians(30))) for m in cutting(mv2))


def test_chamfer_on_given_edges_follows_a_sloped_top_and_checks_the_width(tools):
    t = surface(lambda x, y: 0.1 * x + 0 * y, half=20.0, n=40, base=30.0)
    _, mesh, stock = prepared(t)
    mv = ops.chamfer(t, mesh, tools["S12"], stock, width=1.5, paths="-10,0; 10,0")
    for _, p, *_ in cutting(mv):
        assert p[2] == pytest.approx(ops.surface_z(mesh, p[0], p[1]) - 1.5, abs=0.005)
    with pytest.raises(ValueError, match="exceeds the tool radius"):
        ops.chamfer(t, mesh, tools["S12"], stock, width=3.5, paths="-10,0; 10,0")
    with pytest.raises(ValueError, match="width"):
        ops.chamfer(t, mesh, tools["S12"], stock, width=0.0)
    with pytest.raises(ValueError, match="tip angle"):
        ops.chamfer(t, mesh, tools["W1"], stock, width=1.0)


# ---------------------------------------------------------------- flat lands
def test_flat_lands_groups_the_horizontal_faces_by_height():
    g = ops.flat_lands(stepped())
    assert [round(z, 6) for z, _ in g] == [15.0, 10.0]
    assert ops.flat_lands(np.array([[[0, 0, 0], [1, 0, 0], [0, 1, 0.5]]], float), 1.0) == []   # a 27 degree slope


def test_flat_land_rides_at_each_land_height_inset_from_walls_without_gouging(tools):
    t, mesh, stock = prepared(stepped())
    tool = tools["W2"]                                                                  # flat d 6.35
    cutter = lc.cutter_of(tool)
    mv = ops.flat_land(t, mesh, tool, stock, stepover=3.0)
    cut = cutting(mv)
    assert cut
    assert {round(m[1][2], 6) for m in cut} == {10.0, 15.0}
    for _, p, *_ in cut:
        assert lc.drop_cutter(cutter, mesh, float(p[0]), float(p[1])) <= p[2] + 0.02    # never below the part
    low = [p for _, p, *_ in cut if p[2] < 12]
    assert min(p[0] for p in low) >= 30.0 + tool.diameter / 2 - 0.5                     # a radius off the step wall
    z = zmap_reader(tool, mv, stock, 0.5)
    assert z(50, 20) == pytest.approx(10.0, abs=0.02)                                   # lower land cut
    assert z(59.75, 0.25) == pytest.approx(10.0, abs=0.02)                              # the open edge: overhang
    assert z(10, 20) == pytest.approx(15.0, abs=0.02)                                   # upper land cut
    assert z(0.25, 39.75) == pytest.approx(15.0, abs=0.02)
    assert z(30.5, 20) == pytest.approx(10.0, abs=0.02)                                 # the inset tool reaches the wall
    # allowance lifts every pass
    assert all(m[1][2] in (pytest.approx(10.5), pytest.approx(15.5))
               for m in cutting(ops.flat_land(t, mesh, tool, stock, stepover=3.0, allowance=0.5)))


def test_flat_land_refuses_a_ball_mill_and_a_part_without_lands(tools):
    t, mesh, stock = prepared(stepped())
    with pytest.raises(ValueError, match="flat or bull"):
        ops.flat_land(t, mesh, tools["W7"], stock)
    dome = np.array([[[0, 0, 0], [10, 0, 0], [0, 10, 8.0]], [[10, 0, 0], [10, 10, 8.0], [0, 10, 8.0]]], float)
    assert ops.flat_land(dome, cam.trimesh(dome), tools["W2"], cam.stock_box(dome, 3.0)) == []


# ---------------------------------------------------------------- entries: plunge, ramp, helix
def _first_entry(mv):
    """(rapid end above the pass start, the cutting moves of the first pass)."""
    i = next(k for k, m in enumerate(mv) if m[0] == "rapid" and k > 0 and mv[k - 1][0] == "rapid")
    j = next(k for k in range(i + 1, len(mv)) if mv[k][0] == "rapid")
    return mv[i][1], [m for m in mv[i + 1:j]]


def test_ramp_entry_descends_at_the_angle_and_never_below_the_pass(tools):
    t, mesh, stock = prepared(plate())
    tool = tools["W1"]
    level = stock["zmax"] - 1.0
    mv = cam.face(t, mesh, tool, stock, depth=1.0, stepover=8.0, entry="ramp", ramp_angle=3.0)
    a, first = _first_entry(mv)
    assert a[2] == pytest.approx(level + cam.ENTRY_HEIGHT)
    assert not any(m[0] == "plunge" for m in mv)                                        # no vertical plunge at all
    b = first[0][1]
    assert (a[2] - b[2]) / math.hypot(*(b[:2] - a[:2])) == pytest.approx(math.tan(math.radians(3.0)), rel=1e-6)
    # the ramp reaches the pass height after ENTRY_HEIGHT / tan(angle) of travel, on the line
    pts = [a] + [m[1] for m in first]
    k = next(i for i, p in enumerate(pts) if p[2] <= level + 1e-9)
    run = sum(math.hypot(*(pts[i + 1][:2] - pts[i][:2])) for i in range(k))
    assert run == pytest.approx(cam.ENTRY_HEIGHT / math.tan(math.radians(3.0)), rel=1e-6)
    assert all(p[2] >= level - 1e-9 for _, p, *_ in cutting(mv))                        # none below the pass z
    # the pass itself is unchanged: its last point is the plunge version's last point
    ref = cam.face(t, mesh, tool, stock, depth=1.0, stepover=8.0)
    assert np.allclose(cutting(mv)[-1][1], cutting(ref)[-1][1])


def test_ramp_bounces_along_a_short_pass_and_stays_on_it():
    ps = np.array([[0.0, 0.0, 5.0], [5.0, 0.0, 5.0]])
    ent = cam.Entry("ramp", angle=3.0)
    mv = cam._ramp_in(ps, 50.0, ent)
    P = np.array([m[1] for m in mv])
    assert np.all(P[:, 1] == 0) and P[:, 0].min() >= 0 and P[:, 0].max() <= 5.0         # inside the pass
    assert np.all(P[:, 2] >= 5.0 - 1e-12) and np.allclose(P[-1], ps[0])                 # above the pass, ends at its start
    dz = -np.diff(np.r_[5.0 + cam.ENTRY_HEIGHT, P[:, 2]])
    dl = np.abs(np.diff(np.r_[0.0, P[:, 0]]))
    descending = dz > 1e-9
    assert np.allclose(dz[descending] / dl[descending], math.tan(math.radians(3.0)), rtol=1e-6)


def test_ramp_follows_a_pass_that_rises_and_falls():
    ps = np.array([[0.0, 0.0, 5.0], [10.0, 0.0, 5.0], [20.0, 0.0, 8.0], [30.0, 0.0, 8.0]])
    mv = cam._ramp_in(ps, 50.0, cam.Entry("ramp", angle=10.0))
    for _, p, *_ in mv:                                                                 # above the pass height at p's x
        assert p[2] >= np.interp(p[0], ps[:, 0], ps[:, 2]) - 1e-9


def test_helix_entry_stays_in_the_region_above_the_floor_and_descends(tools):
    t, mesh, stock = prepared(plate())
    tool = tools["W1"]
    level = stock["zmax"] - 1.0
    R = tool.diameter / 2
    mv = cam.face(t, mesh, tool, stock, depth=1.0, stepover=8.0, entry="helix", helix_radius=3.0, helix_pitch=1.0)
    start, moves = _first_entry(mv)
    helix = np.array([m[1] for m in moves])
    assert start[2] == pytest.approx(level + cam.ENTRY_HEIGHT)
    n = next(k for k, p in enumerate(helix) if abs(p[2] - level) < 1e-9)                # where the descent ends
    d = helix[:n + 1]
    assert np.all(np.diff(d[:, 2]) <= 1e-9) and np.all(helix[:, 2] >= level - 1e-9)
    c = d[:, :2].mean(axis=0)
    rad = np.hypot(*(d[:, :2] - c).T)
    assert rad.max() == pytest.approx(3.0, abs=0.05) and rad.min() > 2.9                # a circle of radius 3
    turns = (start[2] - level) / 1.0
    assert (len(d) / math.ceil(turns)) > 8                                              # sampled finely
    rect = (stock["xmin"] - R, stock["xmax"] + R, stock["ymin"] - R, stock["ymax"] + R)
    assert d[:, 0].min() >= rect[0] - 1e-6 and d[:, 0].max() <= rect[1] + 1e-6
    assert d[:, 1].min() >= rect[2] - 1e-6 and d[:, 1].max() <= rect[3] + 1e-6
    assert not any(m[0] == "plunge" for m in mv)


def test_entries_never_go_below_the_part_on_a_relief(tools):
    from test_studio_ops import dome
    t = dome(24, 60.0, 12.0)
    t[:, :, 2] += 8.0
    _, mesh, stock = prepared(t)
    tool = tools["W1"]
    cutter = lc.cutter_of(tool)
    for entry in ("ramp", "helix"):
        mv = cam.rough_plane(t, mesh, tool, stock, stepdown=4.0, stepover=6.0, allowance=0.5, entry=entry,
                             helix_radius=2.0)
        assert cutting(mv)
        for _, p, *_ in cutting(mv):
            h = lc.drop_cutter(cutter, mesh, float(p[0]), float(p[1]))
            assert not math.isfinite(h) or p[2] >= h - 1e-6                              # the tool never gouges
        if entry == "helix":
            assert not any(m[0] == "plunge" for m in mv)


def test_bad_entry_is_refused(tools):
    t, mesh, stock = prepared(plate())
    with pytest.raises(ValueError, match="entry must be one of"):
        cam.face(t, mesh, tools["W1"], stock, entry="spiral")
    with pytest.raises(ValueError, match="between 0 and 90"):
        cam.face(t, mesh, tools["W1"], stock, entry="ramp", ramp_angle=0.0)
    with pytest.raises(ValueError, match="does not fit"):
        cam.face(t, mesh, tools["W1"], stock, entry="helix", helix_radius=500.0)


def test_flat_land_accepts_the_entries(tools):
    t, mesh, stock = prepared(stepped())
    for entry in ("ramp", "helix"):
        mv = ops.flat_land(t, mesh, tools["W2"], stock, stepover=3.0, entry=entry, helix_radius=1.5)
        assert cutting(mv) and all(m[1][2] >= 10.0 - 1e-9 for m in cutting(mv))


# ---------------------------------------------------------------- program and setup sheet
def _session(tmp_path, tools_ids=("S12", "W1")):
    s = api.Session("kr120", material="granite")
    t = plate(40.0, 30.0, 10.0)
    cam_stl = tmp_path / "plate.stl"
    import struct
    with open(cam_stl, "wb") as f:
        f.write(b"\0" * 80 + struct.pack("<I", len(t)))
        for tri in t:
            f.write(struct.pack("<12fH", 0, 0, 0, *[c for p in tri for c in p], 0))
    s.load_part(cam_stl, stock_margin=3.0)
    s.set_work_frame([1.6, 0.0, 0.3])
    return s


def test_the_new_steps_run_in_a_program_and_the_setup_sheet_reports_them(tmp_path):
    s = _session(tmp_path)
    p = s.program("Main")
    p.add("face", tool="W1", depth=1.0, stepover=8.0, entry="ramp", feed=800.0, rpm=6000.0, tolerance=0.05)
    p.add("flat_land", tool="W1", stepover=8.0, feed=800.0, rpm=6000.0, tolerance=0.05)
    p.add("chamfer", tool="S12", width=1.0, feed=400.0, rpm=8000.0, tolerance=0.05)
    p.add("engrave", tool="S12", paths="10,15; 30,15", width=2.0, feed=300.0, rpm=8000.0, tolerance=0.05)
    r = s.simulate("Main")
    assert not r.errors, r.messages
    for label in ("Face", "Flat lands", "Chamfer", "Engrave"):
        assert any(label in m and "CL points" in m for m in r.messages), (label, r.messages)
    md = s.setup_sheet("Main")
    rows = [ln for ln in md.splitlines() if ln.startswith("| ") and not ln.startswith("| Step")]
    assert len(rows) == 5                                                              # 4 steps and the total
    assert "| W1 |" in md and "| S12 |" in md and "flat_end_mill" in md and "engraver" in md
    assert "| 800 | 6000 |" in md and "| 400 | 8000 |" in md
    assert "Removed" not in md and "needs a stock simulation" in md
    assert not any("n/a" in ln for ln in rows)                                         # every feed is known: times given
    # the same sheet from the module, with the rows as numbers
    rows = sheet.step_rows(s.station, r.expansion, s._prog("Main"), s._tool_map())
    assert [x["step"] for x in rows] == [1, 2, 3, 4]
    assert all(x["seconds"] > 0 and x["feed_mm"] > 0 for x in rows)
    t_face = rows[0]["seconds"]
    assert t_face >= rows[0]["feed_mm"] / 800.0 * 60.0                                  # feed time alone is a lower bound


def test_sheet_with_a_stock_run_has_the_removed_volume_and_the_cli_writes_it(tmp_path, capsys):
    s = _session(tmp_path)
    p = s.program("Main")
    p.add("face", tool="W1", depth=2.0, stepover=8.0, feed=800.0, rpm=6000.0, tolerance=0.05)
    out = tmp_path / "sheet.md"
    md = s.setup_sheet("Main", stock=True, path=out)
    assert out.read_text(encoding="utf-8") == md and "Removed (mm3)" in md
    vol = float(md.splitlines()[[i for i, ln in enumerate(md.splitlines()) if ln.startswith("| 1 |")][0]]
                .split("|")[-2])
    assert vol > 1000.0                                                                # a 2 mm face over 46 x 36 mm
    f = s.save(tmp_path / "st.lcs.json")
    out2 = tmp_path / "sheet2.md"
    assert api.main(["sheet", str(f), "Main", str(out2)]) == 0
    assert '"written"' in capsys.readouterr().out
    text = out2.read_text(encoding="utf-8")
    assert text.startswith("# Setup sheet: Main") and "| 1 | Face [W1] |" in text
    assert api.main(["sheet", str(f)]) == 2                                            # wrong arguments: refused


def test_a_step_without_a_feed_shows_a_dash_and_no_time(tmp_path):
    s = _session(tmp_path)
    s.program("Main").add("face", tool="W1", depth=1.0, stepover=8.0, tolerance=0.05)   # no feed anywhere
    md = s.setup_sheet("Main")
    assert "n/a" in md or "Errors in the program" in md
