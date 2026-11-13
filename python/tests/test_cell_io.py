# Copyright 2026 The lithocam authors. SPDX-License-Identifier: Apache-2.0
"""Cell I/O definitions: the shipped placeholder files validate; every robot dialect maps spindle, water, air, wire
and tool-change records through the definition, in the right order around the cuts; without a definition the
writers post comments and the expansion warns."""
import copy
import json
import math
from pathlib import Path

import pytest

np = pytest.importorskip("numpy")

from lithocam import cell_io, post  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]
CELLS = ROOT / "cells"
NATIVE = {"ur10e": "urscript", "kr120": "krl", "kr480_r3330_mt": "krl", "kr500_r2830_mt": "krl"}

# a filled-in definition per controller (test values, not a real cell)
FULL = {
    "urscript": {"controller": "urscript", "ready_inputs": [{"di": 5, "value": True}],
                 "fault_input": {"di": 6, "ok": False},
                 "spindle": {"start": [{"do": 0, "value": True}], "stop": [{"do": 0, "value": False}],
                             "speed": {"kind": "analog", "port": 0, "volts_per_rpm": 10.0 / 12000,
                                       "full_scale_volts": 10.0, "output": "fraction"},
                             "ready": {"di": 2, "value": True}, "dwell_s": 1.5, "max_rpm": 12000},
                 "water": {"on": [{"do": 1, "value": True}], "off": [{"do": 1, "value": False}]},
                 "air": {"on": [{"do": 2, "value": True}], "off": [{"do": 2, "value": False}]},
                 "wire": {"start": [{"do": 3, "value": True}], "stop": [{"do": 3, "value": False}]},
                 "tool_change": {"mode": "manual", "message": "Put {tool} in"}},
    "krl": {"controller": "krl", "fault_input": {"di": 6, "ok": False},
            "spindle": {"start": [{"do": 11, "value": True}], "stop": [{"do": 11, "value": False}],
                        "speed": {"kind": "analog", "port": 1, "volts_per_rpm": 10.0 / 24000,
                                  "full_scale_volts": 10.0},
                        "ready": {"di": 12, "value": True}, "dwell_s": 2.0},
            "water": {"on": [{"do": 13, "value": True}], "off": [{"do": 13, "value": False}]},
            "tool_change": {"mode": "changer", "actions": [{"call": "TOOL_CHANGE({pocket})"}],
                            "pockets": {"W7": 4}, "ready": {"di": 14, "value": True}}},
    "rapid": {"controller": "rapid",
              "spindle": {"start": [{"do": "doSpindleRun", "value": True}],
                          "stop": [{"do": "doSpindleRun", "value": False}],
                          "speed": {"kind": "analog", "port": "aoSpindle", "volts_per_rpm": 0.0005,
                                    "output": "volts"},
                          "ready": {"di": "diAtSpeed", "value": True}, "dwell_s": 1.0},
              "water": {"on": [{"do": 7, "value": True}], "off": [{"do": 7, "value": False}]},
              "tool_change": {"mode": "manual"}},
}


def pose(x):
    T = np.diag([1.0, -1.0, -1.0, 1.0])
    T[:3, 3] = [0.5 + x, 0.1, 0.2]
    return T.tolist()


def cut_records(tool="W7"):
    q = [0.1, -1.2, 1.3, -0.4, 0.5, 0.6]
    r = [post.Rec("start", {"name": "t"}), post.Rec("tool_change", {"id": tool}),
         post.Rec("tool", {"id": tool, "tcp": np.eye(4).tolist(), "mass": 2.0}),
         post.Rec("spindle", {"rpm": 6000.0, "on": True}), post.Rec("water", {"on": True}),
         post.Rec("movej", {"q": q})]
    r += [post.Rec("movel", {"q": q, "pose": pose(0.01 * k), "cut": True, "v": 10.0}) for k in range(3)]
    r += [post.Rec("spindle", {"rpm": 0.0, "on": False}), post.Rec("water", {"on": False}), post.Rec("end")]
    return r


def first(lines, pred, start=0):
    return next(k for k in range(start, len(lines)) if pred(lines[k]))


def last(lines, pred):
    return max(k for k, l in enumerate(lines) if pred(l))


@pytest.mark.parametrize("robot", sorted(NATIVE))
def test_shipped_files_validate_and_say_placeholder(robot):
    io = cell_io.load(CELLS / "{}_io.json".format(robot))
    assert io.controller == NATIVE[robot] and io.placeholder
    assert any("PLACEHOLDER" in w for w in io.warnings)
    raw = json.loads((CELLS / "{}_io.json".format(robot)).read_text(encoding="utf-8"))
    assert "set for your cell" in raw["note"]
    for sec in ("spindle", "water", "air", "wire"):          # every section carries the marker
        assert "set for your cell" in json.dumps(raw[sec])
    assert io.tool_mode() == "manual"


def test_example_machine_io_validates():
    for f in ("generic_3axis", "generic_5axis_ac"):
        m = json.loads((ROOT / "examples" / "gcode_machines" / (f + ".json")).read_text(encoding="utf-8"))
        assert cell_io.from_dict(m["io"]).controller == "gcode"


@pytest.mark.parametrize("bad,match", [
    ({"controller": "fanuc"}, "controller"),
    ({"controller": "urscript", "spindl": {}}, "unknown key"),
    ({"controller": "urscript", "spindle": {"start": [{"do": 9, "value": True}]}}, "outside 0..7"),
    ({"controller": "urscript", "spindle": {"start": [{"mcode": "M3"}]}}, "gcode controller only"),
    ({"controller": "krl", "water": {"on": [{"do": 0, "value": True}]}}, ">= 1"),
    ({"controller": "krl", "spindle": {"start": [{"call": "SPIN({speed})"}]}}, "unknown template field"),
    ({"controller": "rapid", "tool_change": {"mode": "changer"}}, "needs actions"),
    ({"controller": "gcode", "fault_input": {"di": 1, "ok": False}}, "not posted in G-code"),
    ({"controller": "urscript", "spindle": {"speed": {"kind": "analog", "port": 0, "volts_per_rpm": -1}}},
     "positive"),
])
def test_validation_refuses_bad_definitions(bad, match):
    with pytest.raises(cell_io.CellIOError, match=match):
        cell_io.from_dict(bad)


def test_urscript_maps_the_process_around_the_cuts():
    text = post.write(cut_records(), "urscript", io=FULL["urscript"])
    L = text.splitlines()
    cut0 = first(L, lambda l: l.strip().startswith("movel("))
    cut1 = last(L, lambda l: l.strip().startswith("movel("))
    ao = L.index("  set_standard_analog_out(0, 0.5000)")       # 6000 rev/min * 10/12000 V / 10 V
    on = L.index("  set_standard_digital_out(0, True)")
    assert first(L, lambda l: "get_standard_digital_in(6)" in l) < ao < on      # fault clear, speed, then start
    assert on < first(L, lambda l: "get_standard_digital_in(2)" in l) < L.index("  sleep(1.500)") < cut0
    assert L.index("  set_standard_digital_out(1, True)") < cut0
    assert L.index("  set_standard_digital_out(0, False)") > cut1 and L.index("  set_standard_digital_out(1, False)") > cut1
    assert first(L, lambda l: "get_standard_digital_in(5)" in l) < first(L, lambda l: "popup(" in l)  # ready first
    assert 'popup("lithocam: Put W7 in", blocking=True)' in text
    assert "map to the cell's spindle I/O" not in text
    assert len(post.joint_targets(text, "urscript")) == 4


def test_krl_and_rapid_map_the_process_around_the_cuts():
    krl = post.write(cut_records(), "krl", io=FULL["krl"]).splitlines()
    c0 = first(krl, lambda l: l.strip().startswith("LIN "))
    c1 = last(krl, lambda l: l.strip().startswith("LIN "))
    assert krl.index("  WAIT FOR $IN[6] == FALSE") < krl.index("  $ANOUT[1] = 0.2500") < \
        krl.index("  $OUT[11] = TRUE") < krl.index("  WAIT FOR $IN[12] == TRUE") < krl.index("  WAIT SEC 2.000") < c0
    assert krl.index("  $OUT[13] = TRUE") < c0 and krl.index("  $OUT[11] = FALSE") > c1 and \
        krl.index("  $OUT[13] = FALSE") > c1
    assert krl.index("  TOOL_CHANGE(4)") < krl.index("  WAIT FOR $IN[14] == TRUE") < krl.index("  $OUT[11] = TRUE")
    rapid = post.write(cut_records(), "rapid", io=FULL["rapid"]).splitlines()
    c0 = first(rapid, lambda l: l.strip().startswith("MoveL "))
    c1 = last(rapid, lambda l: l.strip().startswith("MoveL "))
    assert rapid.index("    SetAO aoSpindle, 3.0000;") < rapid.index("    SetDO doSpindleRun, 1;") < \
        rapid.index("    WaitDI diAtSpeed, 1;") < rapid.index("    WaitTime 1.000;") < c0
    assert rapid.index("    SetDO DO_7, 1;") < c0 and rapid.index("    SetDO doSpindleRun, 0;") > c1
    assert rapid.index("    Stop;") < c0                       # manual tool change: TPWrite + Stop
    for d in ("krl", "rapid"):
        assert len(post.joint_targets(post.write(cut_records(), d, io=FULL[d]), d)) == 4


def test_no_io_posts_comments_and_a_manual_change():
    for d, spin, stop in (("urscript", "  # spindle 6000.0 rev/min: map to the cell's spindle I/O", "popup("),
                          ("krl", "  ; spindle 6000.0 rev/min: map to the cell's spindle I/O", "  HALT"),
                          ("rapid", "    ! spindle 6000.0 rev/min: map to the cell's spindle I/O", "    Stop;")):
        text = post.write(cut_records(), d)
        assert spin in text.splitlines() and "water on: map to the cell's water I/O" in text
        assert stop in text and "the operator changes it" in text


def test_state_speed_change_safety_net_mismatch_and_limits():
    io = cell_io.from_dict(FULL["urscript"])
    recs = cut_records()[:-3] + [post.Rec("spindle", {"rpm": 9000.0, "on": True}), post.Rec("end")]
    warn = []
    text = post.write(recs, "urscript", io=io, warnings=warn)
    L = text.splitlines()
    assert L.count("  set_standard_digital_out(0, True)") == 1     # running: only the new speed
    assert "  set_standard_analog_out(0, 0.7500)" in L
    assert any("still on at the program end" in w for w in warn)   # off added before the end
    assert L.index("  set_standard_digital_out(0, False)") < L.index("end")
    # above max_rpm: refused
    with pytest.raises(cell_io.CellIOError, match="max_rpm"):
        post.write(cut_records()[:3] + [post.Rec("spindle", {"rpm": 15000.0})], "urscript", io=io)
    # a KRL definition given to the URScript writer: ignored with a note
    warn = []
    text = post.write(cut_records(), "urscript", io=FULL["krl"], warnings=warn)
    assert "is for krl programs" in text and "map to the cell's spindle I/O" in text and warn
    # wire, air and a register speed; a call-type (plug-in function) spindle
    d = copy.deepcopy(FULL["urscript"])
    d["spindle"]["speed"] = {"kind": "register", "register": 24, "per_rpm": 1.0}
    d["spindle"]["start"] = [{"call": "my_spindle_start({rpm})"}]
    recs = [post.Rec("start", {}), post.Rec("spindle", {"rpm": 4000.0}), post.Rec("wire", {"on": True}),
            post.Rec("air", {"on": True}), post.Rec("air", {"on": False}), post.Rec("end")]
    L = post.write(recs, "urscript", io=d).splitlines()
    assert "  write_output_integer_register(24, 4000)" in L and "  my_spindle_start(4000)" in L
    assert "  set_standard_digital_out(3, True)" in L and "  set_standard_digital_out(2, True)" in L
    assert L.index("  set_standard_digital_out(3, False)") < L.index("end")        # safety net turns the wire off


# ---------------------------------------------------------------- the studio expansion (needs the C++ core)
@pytest.fixture(scope="module")
def studio():
    try:
        from lithocam.studio.core import lc
    except ImportError:
        pytest.skip("_lithocam not built (bindings/); set LITHOCAM_CORE")
    from lithocam.studio import cam, program
    from test_studio_program import dome, raster_program
    t = dome()
    tools = {x.id: x for x in list(lc.stone_tool_library().tools) + list(lc.wood_tool_library().tools)}
    def fed():   # the shared dome program, with a feed on the raster step
        prog = raster_program()
        prog.steps[3].params["feed"] = 600.0
        return prog
    return program, cam, (t, cam.trimesh(t)), tools, fed


def _io_file(tmp_path, robot, d):
    p = tmp_path / "{}_io.json".format(robot)
    p.write_text(json.dumps(d), encoding="utf-8")
    return str(p)


def test_expansion_without_io_warns_and_keeps_comments(studio):
    program, cam, part, tools, raster_program = studio
    st = program.Station(robot="ur10e", work_xyz=[0.55, 0.15, 0.05], stock=cam.stock_box(part[0], 3.0))
    ex = program.expand(st, program.load_robot("ur10e"), raster_program(), tools, part)
    assert any(m.startswith("WARNING: no cell I/O definition loaded") for m in ex.messages)
    assert not any(r.kind == "io_map" for r in ex.records)
    kinds = [r.kind for r in ex.records]
    assert kinds.index("tool_change") < kinds.index("tool")
    text = post.write(ex.records, "urscript")
    assert "map to the cell's spindle I/O" in text and "map to the cell's water I/O" in text


@pytest.mark.parametrize("robot,work", [("ur10e", [0.55, 0.15, 0.05]), ("kr120", [1.6, 0.0, 0.3])])
def test_expansion_orders_process_records_and_posts_them(studio, tmp_path, robot, work):
    program, cam, part, tools, raster_program = studio
    dialect = NATIVE[robot]
    st = program.Station(robot=robot, work_xyz=work, stock=cam.stock_box(part[0], 3.0),
                         cell_io=_io_file(tmp_path, robot, FULL[dialect]))
    ex = program.expand(st, program.load_robot(robot), raster_program(), tools, part)
    assert not ex.errors and ex.records[1].kind == "io_map"
    assert not any("no cell I/O" in m for m in ex.messages)
    k = [r.kind for r in ex.records]
    cuts = [i for i, r in enumerate(ex.records) if r.kind == "movel" and r.data.get("cut")]
    spin_on = next(i for i, r in enumerate(ex.records) if r.kind == "spindle" and r.data["on"])
    spin_off = next(i for i, r in enumerate(ex.records) if r.kind == "spindle" and not r.data["on"])
    water_on = next(i for i, r in enumerate(ex.records) if r.kind == "water" and r.data["on"])
    water_off = next(i for i, r in enumerate(ex.records) if r.kind == "water" and not r.data["on"])
    assert k.index("tool_change") < k.index("tool") < spin_on < water_on < cuts[0]
    assert cuts[-1] < spin_off < k.index("end") and cuts[-1] < water_off < k.index("end")
    assert ex.records[spin_on].data["rpm"] > 0
    text = post.write(ex.records, dialect, **program.post_options(robot, dialect))   # io from the io_map record
    rec_q = [r.data["q"] for r in ex.records if r.kind in ("movej", "movel")]
    back = post.joint_targets(text, dialect)
    assert len(back) == len(rec_q) and np.max(np.abs(np.asarray(back) - np.asarray(rec_q))) < 1e-5
    L = text.splitlines()
    if dialect == "urscript":
        c0 = first(L, lambda l: l.strip().startswith("movep("))
        c1 = last(L, lambda l: l.strip().startswith("movep("))
        on, off = "  set_standard_digital_out(0, True)", "  set_standard_digital_out(0, False)"
        won, woff = "  set_standard_digital_out(1, True)", "  set_standard_digital_out(1, False)"
    else:
        c0 = first(L, lambda l: l.strip().startswith("LIN "))
        c1 = last(L, lambda l: l.strip().startswith("LIN "))
        on, off, won, woff = "  $OUT[11] = TRUE", "  $OUT[11] = FALSE", "  $OUT[13] = TRUE", "  $OUT[13] = FALSE"
    assert L.index(on) < c0 < c1 < L.index(off) and L.index(won) < c0 < c1 < L.index(woff)


def test_shipped_default_file_posts_placeholder_outputs_and_warns(studio):
    program, cam, part, tools, raster_program = studio
    st = program.Station(robot="ur10e", work_xyz=[0.55, 0.15, 0.05], stock=cam.stock_box(part[0], 3.0),
                         cell_io="default")
    ex = program.expand(st, program.load_robot("ur10e"), raster_program(), tools, part)
    assert any("PLACEHOLDER" in m for m in ex.messages) and any("speed scaling is not set" in m for m in ex.messages)
    text = post.write(ex.records, "urscript")
    assert "WARNING: PLACEHOLDER I/O values" in text
    assert "speed scaling not set for your cell" in text and "set_standard_digital_out(0, True)" in text
    # a definition that does not load is an error: posting is refused
    st.cell_io = str(CELLS / "missing_io.json")
    ex = program.expand(st, program.load_robot("ur10e"), raster_program(), tools, part)
    assert ex.errors and "cell I/O" in ex.errors[0]
