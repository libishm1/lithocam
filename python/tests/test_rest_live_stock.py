# Copyright 2026 The lithocam authors. SPDX-License-Identifier: Apache-2.0
"""Rest machining from the live stock: the stock simulation before the step, else a Z-map of the earlier milling
steps, else the previous tool radius, and the log line that says which. No Qt."""
import numpy as np
import pytest

try:
    from lithocam.studio.core import lc  # noqa: F401
except ImportError:
    pytest.skip("_lithocam not built (bindings/); set LITHOCAM_CORE", allow_module_level=True)
mf = pytest.importorskip("manifold3d")

from lithocam.studio import cam, livestock, ops, process, program  # noqa: E402

from test_studio_ops import _tris  # noqa: E402

FLAT, BALL = "W1", "W7"        # flat end mill d 12.7, ball end mill d 6.35 of the wood library
ROUGH = {"tool": FLAT, "stepdown": 5.0, "stepover": 4.0, "allowance": 0.0, "feed": 800, "rpm": 6000,
         "tolerance": 0.05}
REST = {"tool": BALL, "stepover": 3.0, "feed": 500, "rpm": 8000, "tolerance": 0.05}


def slotted_block():
    """60 x 40 x 20 block with an 8 mm wide slot along x (x 10..50, y 16..24), floor at z = 6. The flat mill is wider
    than the slot and rides over it; the ball mill fits."""
    return _tris(mf.Manifold.cube([60, 40, 20]) - mf.Manifold.cube([40, 8, 15]).translate([10, 16, 6]))


@pytest.fixture(scope="module")
def tools():
    return {t.id: t for t in list(lc.stone_tool_library().tools) + list(lc.wood_tool_library().tools)}


@pytest.fixture(scope="module")
def part():
    t = slotted_block()
    return t, cam.trimesh(t)


def _station(t):
    return program.Station(robot="kr120", work_xyz=[1.6, 0.0, 0.3], stock=cam.stock_box(t, 3.0))


def _prog(*steps):
    return program.Program("R", [program.Step(k, dict(p)) for k, p in steps])


def _expand(part, tools, prog, **kw):
    t, mesh = part
    return program.expand(_station(t), program.load_robot("kr120"), prog, tools, (t, mesh), **kw)


def test_zmap_leftover_region_is_where_the_rough_tool_left_material(part, tools):
    t, mesh = part
    stock = _station(t).stock
    moves = cam.rough_plane(t, mesh, tools[FLAT], stock, stepdown=5.0, stepover=4.0, allowance=0.0)
    xs, ys, Z = ops.zmap_leftover([(tools[FLAT], moves)], stock, 1.0)
    assert Z.shape == (len(ys), len(xs)) and xs[1] - xs[0] == pytest.approx(1.0)
    fld, mask = ops.rest_region(t, mesh, tools[BALL], leftover=(xs, ys, Z))
    assert mask.any()
    X, Y = np.meshgrid(fld.xs, fld.ys)
    over_part = (X >= 0.0) & (X <= 60.0) & (Y >= 0.0) & (Y <= 40.0)       # not the samples off the part's edge
    m = mask & over_part
    assert m.any()
    # every region sample over the part is inside the slot (the ball centre is at least its radius from the walls)
    assert np.all((Y[m] > 16.0) & (Y[m] < 24.0) & (X[m] > 10.0) & (X[m] < 50.0))
    # there the flat tool left the stock at the part's top, 14 mm above the slot floor the ball reaches
    top = ops._stock_top((xs, ys, Z), fld.xs, fld.ys)
    assert np.all(top[m] > 19.0)
    assert np.any(fld.H[m] < 7.0)              # the slot's centre line, where the ball reaches the floor
    assert np.all(fld.H[m] < top[m] - 0.05)    # everywhere the ball tip can go lower than the stock left
    # and the flat tool cut the rest of the top: no leftover over the plate beside the slot
    assert not (mask & over_part & (Y < 10.0)).any()


def test_expand_uses_a_zmap_of_the_earlier_steps_and_logs_it(part, tools):
    ex = _expand(part, tools, _prog(("rough_plane", ROUGH), ("rest", REST)))
    assert not ex.errors, ex.messages
    assert "step 2: rest: leftover from a Z-map of steps 1..1 at 1 mm" in ex.messages
    assert any("step 2: Rest machining: " in m and "CL points" in m for m in ex.messages), ex.messages
    ex = _expand(part, tools, _prog(("rough_plane", ROUGH), ("rest", dict(REST, leftover_cell=2.0))))
    assert "step 2: rest: leftover from a Z-map of steps 1..1 at 2 mm" in ex.messages


def test_the_same_tool_twice_gives_the_empty_region_message(part, tools):
    ex = _expand(part, tools, _prog(("rough_plane", ROUGH), ("rest", dict(REST, tool=FLAT))))
    assert any("step 2: rest: the region is empty" in m for m in ex.messages), ex.messages
    assert any("step 2" in m and "nothing to cut" in m for m in ex.messages)
    assert not ex.errors


def test_without_stock_or_milling_steps_the_radius_or_an_error(part, tools):
    ex = _expand(part, tools, _prog(("rest", dict(REST, previous_tool_radius=6.35))))
    assert "step 1: rest: previous tool radius 6.35 mm (no stock)" in ex.messages
    assert any("Rest machining" in m and "CL points" in m for m in ex.messages), ex.messages
    ex = _expand(part, tools, _prog(("rest", REST)))                         # nothing to compare with
    assert any("step 1: Rest machining failed" in m and "previous_tool_radius" in m for m in ex.messages)
    # a program step with an explicit radius ignores the Z-map of the earlier steps
    ex = _expand(part, tools, _prog(("rough_plane", ROUGH), ("rest", dict(REST, previous_tool_radius=6.35))))
    assert "step 2: rest: previous tool radius 6.35 mm (no stock)" in ex.messages
    assert not any("Z-map" in m for m in ex.messages)


def test_the_stock_run_is_used_before_the_rest_step(part, tools):
    t, mesh = part
    prog = _prog(("rough_plane", ROUGH), ("rest", REST))
    st = _station(t)
    robot = program.load_robot("kr120")
    ex = _expand(part, tools, prog)
    run = process.simulate_stock(st, robot, ex, tools, cell_mm=2.0, snapshots=2)
    old_sig = livestock.signature(st, prog)
    live = livestock.LiveStock(run, ex, tools, st, robot, old_sig, livestock.signature(st, prog))
    # the stock before step 1 (the rest step, index 1) is what the rough step alone leaves; before step 0 it is the
    # uncut box; the run's own final stock is smaller, the rest step having cut too
    rough_only = process.simulate_stock(st, robot, _expand(part, tools, _prog(("rough_plane", ROUGH))), tools,
                                        cell_mm=2.0, snapshots=2)
    before_rest = live(1)
    assert before_rest.volume() == pytest.approx(rough_only.stock.volume(), rel=1e-6)
    assert live(0).volume() == pytest.approx(run.initial_volume, rel=1e-9)
    assert run.stock.volume() < before_rest.volume() < run.initial_volume
    ex2 = _expand(part, tools, prog, leftover=live)
    assert "step 2: rest: leftover from the stock simulation before step 2" in ex2.messages, ex2.messages
    assert not any("Z-map" in m for m in ex2.messages)
    assert any("step 2: Rest machining: " in m and "CL points" in m for m in ex2.messages), ex2.messages


def test_a_stale_run_is_not_used(part, tools):
    t, mesh = part
    prog = _prog(("rough_plane", ROUGH), ("rest", REST))
    st = _station(t)
    robot = program.load_robot("kr120")
    ex = _expand(part, tools, prog)
    run = process.simulate_stock(st, robot, ex, tools, cell_mm=2.0, snapshots=2)
    old_sig = livestock.signature(st, prog)
    edited = _prog(("rough_plane", dict(ROUGH, stepover=3.0)), ("rest", REST))      # the step before the rest changed
    live = livestock.LiveStock(run, ex, tools, st, robot, old_sig, livestock.signature(st, edited))
    assert live(1) is None and live(0) is not None
    ex2 = _expand(part, tools, edited, leftover=live)
    assert "step 2: rest: leftover from a Z-map of steps 1..1 at 1 mm" in ex2.messages
    # a step the run does not have
    assert livestock.LiveStock(run, ex, tools, st, robot, old_sig, old_sig)(5) is None
