# Copyright 2026 The lithocam authors. SPDX-License-Identifier: Apache-2.0
"""ROBOmove readers and the Quarra Stone Q-LAB import (docs/ROBOMOVE_IMPORT.md).

The Q-LAB files have no licence, so none of them is in this repository: the tests that read them skip when the local
clone (LITHOCAM_QLAB or the default path, see lithocam.robomove) is absent. The synthetic tests below use made-up
text and always run."""
import math
import re

import numpy as np
import pytest

from lithocam import robomove as rm

core = pytest.importorskip("lithocam.studio.core", reason="C++ core not built", exc_type=ImportError)
lc = core.lc

SYNTH_TOOLS = """<ROBOMOVE><TOOL_LIST><SZ>1</SZ>
<0><ID>7</ID><NAME>12mm BALL</NAME><TOOL_TYPE>1</TOOL_TYPE><TCP_TYPE>0</TCP_TYPE><SPINDLE_ASSOC>-1</SPINDLE_ASSOC>
<VALUES>12 40 40 0 0 0 150 0 0 0</VALUES></0>
<1><ID>8</ID><NAME>500mm DISK</NAME><TOOL_TYPE>2</TOOL_TYPE><TCP_TYPE>0</TCP_TYPE><SPINDLE_ASSOC>-1</SPINDLE_ASSOC>
<VALUES>50 160 500 0 0 0 100 6 0 0</VALUES></1>
</TOOL_LIST></ROBOMOVE>"""


def test_tag_parser_reads_numeric_tags_and_ignores_declared_size(tmp_path):
    p = tmp_path / "T_x.ini"
    p.write_text(SYNTH_TOOLS)
    tools = rm.read_tools(p)
    assert len(tools) == 2 and rm.declared_tool_count(p) == 1      # SZ says 1, the file holds 2
    assert tools[1].tool_type == 2 and tools[1].values[2] == 500


def test_synthetic_mapping_and_round_trip(tmp_path):
    p = tmp_path / "T_x.ini"
    p.write_text(SYNTH_TOOLS)
    tools, maps = rm.to_lithocam_tools(rm.read_tools(p), prefix="S")
    assert [t.kind for t in tools] == [lc.ToolKind.BallEndMill, lc.ToolKind.DiscSaw]
    assert tools[0].diameter == 12 and tools[0].corner_radius == 6 and tools[0].stickout == 150
    assert tools[1].diameter == 500 and tools[1].kerf == 6
    assert all(t.source == rm.CREDIT for t in tools)
    assert any(f == "v6" for f, _, _ in maps[1].unmapped)          # the disc-plane offset is reported, not guessed
    lib = lc.ToolLibrary()
    lib.tools = tools
    out = tmp_path / "lib.csv"
    assert lc.write_tool_library(lib, str(out))
    back = lc.read_tool_library(str(out)).tools
    assert [(t.id, t.kind, t.diameter) for t in back] == [(t.id, t.kind, t.diameter) for t in tools]


def test_iv_reader_units_and_faces(tmp_path):
    p = tmp_path / "a.iv"
    p.write_text("#Inventor V1.0 ascii\nSeparator { Units { units MILLIMETERS } Material { diffuseColor 1 0 0 }\n"
                 "Coordinate3 { point [ 0 0 0, 1000 0 0, 0 1000 0, 1000 1000 0 ] }\n"
                 "FaceSet { numVertices [ 3, 3 ] } }\n")
    (s,) = rm.read_iv(p)
    assert s.vertices.max() == pytest.approx(1.0) and len(s.triangles) == 2 and s.color == (1.0, 0.0, 0.0)


# ---------------------------------------------------------------- cells (no clone needed)
def test_fortec_cells_registered_with_krl_post():
    from lithocam.studio import program
    for k, label in (("kr480_r3330_mt", "KR 480 R3330 MT"), ("kr500_r2830_mt", "KR 500 R2830 MT")):
        assert label in program.ROBOTS[k][0] and program.ROBOTS[k][2] == "krl"
        assert len(program.HOME[k]) == 6


@pytest.mark.parametrize("name,reach_mm", [("kr500_r2830_mt", 2826.0), ("kr480_r3330_mt", 3326.0)])
def test_stretched_pose_reach_matches_datasheet(name, reach_mm):
    """KUKA gives the maximum reach to the A4/A5 intersection. At zero joints the arm is stretched, so the wrist
    centre (flange minus the 290 mm wrist length) is at that distance from axis 1: within 2 percent."""
    robot = pytest.importorskip("lithocam.robot", reason="tesseract-robotics not installed", exc_type=ImportError)
    T = robot.Cell(name).fk(np.zeros(6))
    wrist = T[:3, 3] - 0.290 * T[:3, 2]    # tool0 z is the flange axis
    reach = math.hypot(wrist[0], wrist[1]) * 1000
    assert reach == pytest.approx(reach_mm, rel=0.02)


# ---------------------------------------------------------------- the clone
@pytest.fixture(scope="module")
def clone():
    root = rm.qlab_root()
    if root is None:
        pytest.skip("Q-LAB clone not found (set LITHOCAM_QLAB)")
    return root


@pytest.fixture(scope="module")
def backups(clone):
    b = rm.read_all(clone)
    assert b
    return b


def test_every_file_is_read_or_reported(clone):
    inv = rm.inventory(clone)
    assert inv
    assert [r for r, st, _ in inv if st == "unread"] == []
    # binary files that are only partly understood are labelled, not silently accepted
    assert {st for _, st, _ in inv} <= {"parsed", "partial"}
    assert all(st == "partial" for r, st, _ in inv if r.lower().endswith((".dll", ".dwp")))


def test_parse_all_tool_tables_and_end_effectors(clone, backups):
    tabled = [b for b in backups if b.tools]
    assert len(tabled) >= 3
    for b in tabled:
        assert all(len(t.values) == 10 for t in b.tools), b.name
        assert all(t.tool_type in rm.TOOL_TYPES for t in b.tools)
        assert all(t.spindle_assoc == -1 for t in b.tools)
    assert all(b.end_effectors for b in backups)
    for b in backups:
        e = rm.spindle_end_effector(b)
        assert e is not None and len(e.tcp) == 7 and e.spindle_id == 1


def test_rfconf_tables(backups):
    for b in backups:
        assert {"ROTATING_TABLE", "FIXED_TABLE_1", "FIXED_TABLE_2"} <= set(b.tables), b.folder
        assert all(t.euler_config == 1 for t in b.tables.values())


def test_tool_value_roles_hold_on_every_table(backups):
    """The evidence behind VALUE_ROLES: a disc's v2 is its diameter, a milling tool's v0 is its diameter, v9 is 0."""
    n_disc = 0
    for b in backups:
        for t in b.tools:
            assert t.values[9] == 0
            m = re.match(r"(\d+(?:\.\d+)?)\s*mm", t.name)
            if t.tool_type == 2 and m and "DISK" in t.name.upper():
                assert t.values[2] == pytest.approx(float(m.group(1)), rel=0.02)
                n_disc += 1
    assert n_disc >= 4


def test_map_all_tables_and_disc_saws(backups):
    for b in backups:
        if not b.tools:
            continue
        tools, maps = rm.to_lithocam_tools(b.tools, prefix=b.folder.name + ":")
        assert len(maps) == len(b.tools)
        assert len({t.id for t in tools}) == len(tools)
        assert all(t.source == rm.CREDIT for t in tools)
        assert all(m.problems == "" for m in maps), [(m.tool.name, m.problems) for m in maps if m.problems]
        for m in maps:
            if re.search(r"\d+\s*mm\s+DISK", m.tool.name, re.I):
                assert m.kind == "disc_saw"
                nominal = float(re.match(r"(\d+)", m.tool.name).group(1))
                (t,) = [x for x in tools if x.id == m.lithocam_id]
                assert t.kind == lc.ToolKind.DiscSaw and t.diameter == pytest.approx(nominal, rel=0.02)
        lib = lc.ToolLibrary()
        lib.tools = tools
    # one combined library round trip through the CSV writer and reader
    allt = []
    for b in backups:
        if b.tools:
            allt += rm.to_lithocam_tools(b.tools, prefix=b.folder.name + ":")[0]
    lib = lc.ToolLibrary()
    lib.tools = allt
    text = lc.format_tool_library(lib)
    back = lc.parse_tool_library(text).tools
    assert [(t.id, t.name, t.kind, t.source) for t in back] == [(t.id, t.name, t.kind, t.source) for t in allt]
    assert [round(t.diameter, 6) for t in back] == [round(t.diameter, 6) for t in allt]
    assert sum(t.kind == lc.ToolKind.DiscSaw for t in back) >= 10


def test_spindle_presets_from_end_effectors(backups):
    from lithocam.studio import spindles
    keys = [k for k in spindles.PRESETS if k.startswith("qlab_")]
    assert {"qlab_kr480mt_d", "qlab_kr480mt_m", "qlab_kr500"} <= set(keys)
    cb = next(b for b in backups if b.folder.name == "KR480MT-D")
    ee = rm.spindle_end_effector(cb)
    sp = spindles.PRESETS["qlab_kr480mt_d"]
    assert rm.CREDIT in sp.source
    # the cutter axis is the end-effector TCP z axis: with pitch -90 it points along -x of the flange frame
    T = sp.tcp(300.0)
    assert T[:3, 2] == pytest.approx([-1, 0, 0], abs=1e-9)
    assert T[:3, 3] == pytest.approx([ee.tcp[0] / 1000 - 0.3, ee.tcp[1] / 1000, ee.tcp[2] / 1000], abs=1e-9)
    assert spindles.from_dict(spindles.to_dict(sp)).tcp_flange == sp.tcp_flange


def test_kr500_link_meshes_agree_with_the_datasheet_cell(clone):
    """The Q-LAB KR500 link meshes (home pose, metres) put the wrist centre 55 mm above the A3 axis at
    c1 + c2 = 2.345 m, the sign cells/make_cells.py takes for its KR 480 and KR 500 cells."""
    d = next(p for p in rm.find_backups(clone) if p.name == "CellKR500_QUARRASTONE_00")
    v = np.vstack([s.vertices for s in rm.read_iv(d / "link_4.iv")])
    z_mid = 0.5 * (v[:, 2].min() + v[:, 2].max())
    assert z_mid == pytest.approx(1.045 + 1.300 + 0.055, abs=0.02)
