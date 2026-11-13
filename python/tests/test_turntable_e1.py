# Copyright 2026 The lithocam authors. SPDX-License-Identifier: Apache-2.0
"""Turntable as a coordinated external axis E1: the 7-axis chain (arm + table), the ladder with the table angle as
a free choice per rung, KRL and RAPID posting and read-back of E1, the PLC (I/O) mode, operator mode unchanged, the
UR10e refusing E1, and the table as a revolute joint in the Tesseract scene graph turning the part."""
import math
from pathlib import Path

import pytest

np = pytest.importorskip("numpy")

from lithocam import post  # noqa: E402
from lithocam.studio import planner, program  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]


# ---------------------------------------------------------------- chain
def test_table_chain_fk_turns_the_part_frame():
    robot = program.load_robot("kr120")
    work = program.Station(robot="kr120", work_xyz=[1.4, -0.5, 0.1], work_rpy=[0, 0, 20]).work_frame()
    chain = planner.TableChain(robot, work)
    assert chain.dof == 7
    q = np.radians([10.0, -80.0, 100.0, 15.0, 40.0, -30.0])
    for e1 in (0.0, 0.7, -2.1):
        q7 = np.r_[q, e1]
        T = chain.fk(q7)
        assert np.allclose(T, robot.tcp(q))                        # the table does not move the arm
        Tp = chain.fk_part(q7)
        assert np.allclose(chain.to_base(Tp, e1), T)               # part frame <-> base frame
        assert np.allclose(chain.part_frame(e1)[:3, 3], work[:3, 3])   # the table axis runs through the work origin
    # turning the table by d turns the tool, seen from the part, by -d about the table axis
    d = 0.4
    A, B = chain.fk_part(np.r_[q, 0.3]), chain.fk_part(np.r_[q, 0.3 + d])
    Rz = planner.rot_z4(-d)
    assert np.allclose(B, Rz @ A, atol=1e-12)
    # a pose fixed in the part needs the same arm pose only at the same table angle
    Tp = chain.fk_part(np.r_[q, 0.0])
    assert np.allclose(chain.to_base(Tp, 0.0), robot.tcp(q))
    assert not np.allclose(chain.to_base(Tp, 0.2), robot.tcp(q))


# ---------------------------------------------------------------- ladder with the table as a free choice
class ArcBackend:
    """Synthetic arm with one joint that must point at a part point: pose = (part angle, table angle); the arm angle
    is their sum (the table turns the point). No collisions."""
    name = "synthetic"
    checks_collision = False

    def ik(self, T, seed=None):
        phi, e = T
        return [np.array([phi + e])]

    def in_collision(self, q):
        return False


def _arc_rungs(yaws, n=9, step_deg=12.0):
    phis = [math.radians(step_deg * k) for k in range(n)]
    rungs = [[(-ph, y) for y in yaws] for ph in phis]
    ext = [[y for y in yaws] for _ in phis]
    return rungs, ext


def test_ladder_with_table_candidates_has_fewer_breaks_than_a_fixed_table():
    max_step = math.radians(10)
    fixed, _ = _arc_rungs([0.0])
    res0 = planner.ladder(fixed, ArcBackend(), seed=np.zeros(1), max_step=max_step)
    assert len(res0.breaks) == len(fixed) - 1                     # every 12 deg step is a jump
    yaws = [math.radians(v) for v in range(-60, 61, 6)]
    rungs, ext = _arc_rungs(yaws)
    res = planner.ladder(rungs, ArcBackend(), seed=np.zeros(1), max_step=max_step, ext=ext, ext_seed=0.0,
                         ext_max_step=math.radians(10))
    assert len(res.breaks) < len(res0.breaks) and not res.breaks and not res.missing
    assert len(res.ext) == len(rungs)
    assert res.max_step <= max_step + 1e-12 and res.max_ext_step <= math.radians(10) + 1e-12
    # the table step limit binds: with 2 deg table steps the 12 deg sweep cannot be absorbed
    res2 = planner.ladder(rungs, ArcBackend(), seed=np.zeros(1), max_step=max_step, ext=ext, ext_seed=0.0,
                          ext_max_step=math.radians(1.0))
    assert len(res2.breaks) == len(res0.breaks)


def test_ladder_without_ext_is_unchanged():
    rungs, _ = _arc_rungs([0.0, 0.1])
    a = planner.ladder(rungs, ArcBackend(), seed=np.zeros(1), max_step=math.radians(10))
    assert a.ext == [] and a.max_ext_step == 0.0


def test_table_window_respects_limits_and_turns():
    ys = program.table_window(math.radians(170), math.radians(-175), 30, 10, [-230, 230])
    assert len(ys) == 7
    assert abs(math.degrees(ys[3]) - (-190)) < 1e-9               # the centre moved a whole turn to the present angle
    ys = program.table_window(math.radians(170), math.radians(-175), 30, 10, [-200, 200])
    assert [round(math.degrees(v)) for v in ys] == [-200, -190, -180, -170, -160]   # outside the travel: dropped
    ys = program.table_window(0.0, 0.0, 30, 10, [-15, 15])
    assert [round(math.degrees(v)) for v in ys] == [-10, 0, 10]


# ---------------------------------------------------------------- posting
def _e1_records():
    q = [0.1, -1.2, 1.3, 0.2, 0.5, -0.3]
    T = np.eye(4)
    T[:3, 3] = [1.2, -0.3, 0.6]
    return [post.Rec("start", {"name": "e1test"}),
            post.Rec("table", {"deg": 30.0, "driven": "e1"}),
            post.Rec("movej", {"q": q, "e1": 30.0}),
            post.Rec("movel", {"q": [v + 0.01 for v in q], "pose": T.tolist(), "cut": True, "v": 5.0, "e1": 32.5}),
            post.Rec("movel", {"q": [v + 0.02 for v in q], "pose": T.tolist(), "cut": True, "v": 5.0, "e1": -12.25}),
            post.Rec("movej", {"q": q, "e1": 0.0}),
            post.Rec("end")]


@pytest.mark.parametrize("dialect", ["krl", "rapid", "cl"])
def test_e1_posts_and_reads_back(dialect):
    recs = _e1_records()
    text = post.write(recs, dialect)
    qs = [r.data["q"] for r in recs if r.kind in ("movej", "movel")]
    es = [r.data["e1"] for r in recs if r.kind in ("movej", "movel")]
    back = post.joint_targets(text, dialect)
    assert len(back) == len(qs)
    assert np.allclose(back, qs, atol=1e-5)
    e_back = post.external_axes(text, dialect)
    assert np.allclose(e_back, es, atol=1e-4)
    if dialect == "krl":
        assert "PTP {E6AXIS: A1" in text and ", E1 30.0000}" in text
        assert "LIN {X 1200.000, Y -300.000, Z 600.000" in text and ", E1 32.5000}" in text
        assert "HALT" not in text and "PLACEHOLDER" in text
    if dialect == "rapid":
        assert "[30.0000, 9E9, 9E9, 9E9, 9E9, 9E9]" in text and "[-12.2500, 9E9, 9E9, 9E9, 9E9, 9E9]" in text
        assert "Stop;" not in text and "PLACEHOLDER" in text


def test_urscript_refuses_e1():
    with pytest.raises(ValueError, match="no external axis"):
        post.write(_e1_records(), "urscript")


def test_operator_records_post_as_before():
    """Without E1 the writers keep the operator stop and the plain joint aggregates."""
    q = [0.1, -1.2, 1.3, 0.2, 0.5, -0.3]
    recs = [post.Rec("start", {"name": "op"}), post.Rec("table", {"deg": 90.0}), post.Rec("movej", {"q": q}),
            post.Rec("end")]
    krl = post.write(recs, "krl")
    assert "  ; turntable to 90.000 deg (external axis E1 once the cell defines it)\n  HALT" in krl
    assert "PTP {AXIS: A1 5.7296, A2 -68.7549, A3 74.4845, A4 11.4592, A5 28.6479, A6 -17.1887}" in krl
    assert "E1" not in krl.replace("external axis E1 once", "")
    rapid = post.write(recs, "rapid")
    assert "    Stop;" in rapid and "MoveAbsJ [[5.7296, -68.7549, 74.4845, 11.4592, 28.6479, -17.1887], " \
                                    "[9E9, 9E9, 9E9, 9E9, 9E9, 9E9]]" in rapid
    ur = post.write(recs, "urscript")
    assert 'popup("lithocam: turn the table to 90.0 deg, then Continue", blocking=True)' in ur
    assert post.external_axes(krl, "krl") == [None] and post.external_axes(rapid, "rapid") == [None]


# ---------------------------------------------------------------- station modes
def _column_station(**tt):
    st = program.Station.load(ROOT / "examples" / "studio" / "wire_turntable_kr120.lcs.json")
    st.turntable = dict(st.turntable, **tt)
    return st


def _setup_program():
    return program.Program("T", [program.Step("movej", {"joints": [0.0, -90.0, 90.0, 0.0, 28.6, 0.0]}),
                                 program.Step("setup", {"yaw": 45.0}),
                                 program.Step("movej", {"joints": [10.0, -80.0, 90.0, 0.0, 28.6, 0.0]})])


def test_ur10e_refuses_e1_with_a_clear_message():
    st = program.Station.load(ROOT / "examples" / "studio" / "wire_turntable_ur10e.lcs.json")
    st.turntable = dict(st.turntable, driven="e1")
    mode, errors, _ = program.check_table(st)
    assert mode == "operator" and errors and "UR10e has no external axis" in errors[0]
    ex = program.expand(st, program.load_robot("ur10e"), _setup_program(), {}, None)
    assert any("no external axis" in e for e in ex.errors)
    assert not any(r.kind in ("movej", "movel") and "e1" in r.data for r in ex.records)


def test_operator_mode_records_unchanged_and_default_is_operator():
    robot = program.load_robot("kr120")
    a = program.expand(_column_station(), robot, _setup_program(), {}, None)
    b = program.expand(_column_station(driven="operator"), program.load_robot("kr120"), _setup_program(), {}, None)
    assert [(r.kind, r.data) for r in a.records] == [(r.kind, r.data) for r in b.records]
    tab = [r for r in a.records if r.kind == "table"]
    assert tab and tab[0].data == {"deg": 45.0}
    assert not any("e1" in r.data for r in a.records)
    assert "HALT" in post.write(a.records, "krl")


def test_e1_setup_turn_is_coordinated_with_the_next_motion():
    st = _column_station(driven="e1", e1_limits_deg=[-180, 180], e1_speed_deg_s=20, e1_max_step_deg=8)
    ex = program.expand(st, program.load_robot("kr120"), _setup_program(), {}, None)
    assert not ex.errors
    moves = [r for r in ex.records if r.kind in ("movej", "movel")]
    assert [m.data["e1"] for m in moves] == [0.0, 45.0]
    text = post.write(ex.records, "krl")
    assert "HALT" not in text and "E1 45.0000" in text
    assert post.external_axes(text, "krl") == [0.0, 45.0]
    # the simulation turns the table during the coordinated move, in step with the arm
    seg = [s for s in ex.segments if s.e1 is not None and len(s.e1) > 1 and abs(s.e1[-1] - s.e1[0]) > 1e-9]
    assert seg and abs(math.degrees(seg[-1].e1[-1]) - 45.0) < 1e-9
    steps = np.abs(np.diff(np.concatenate([s.e1 for s in ex.segments])))
    assert steps.max() <= math.radians(2) + 1e-9


def test_e1_placeholders_warn():
    ex = program.expand(_column_station(driven="e1"), program.load_robot("kr120"), _setup_program(), {}, None)
    assert any("PLACEHOLDER" in m and "e1_limits_deg" in m for m in ex.messages)


IO = {"angle": {"register": "TABLE_DEG", "type": "float"}, "request": [{"do": 7, "value": True}],
      "done": {"di": 8, "value": True}, "release": [{"do": 7, "value": False}]}


def test_io_mode_requests_the_plc_and_waits():
    ex = program.expand(_column_station(driven="io", io=IO), program.load_robot("kr120"), _setup_program(), {},
                        None)
    assert not ex.errors
    text = post.write(ex.records, "krl")
    seq = ["  TABLE_DEG = 45.0", "  $OUT[7] = TRUE", "  WAIT FOR $IN[8] == TRUE", "  $OUT[7] = FALSE"]
    pos = [text.index(s) for s in seq]
    assert pos == sorted(pos) and "HALT" not in text
    bad = program.expand(_column_station(driven="io"), program.load_robot("kr120"), _setup_program(), {}, None)
    assert any("turntable driven 'io' needs" in e for e in bad.errors)


# ---------------------------------------------------------------- wire planning with E1 (Tesseract)
def test_e1_wire_face_on_the_column_kr120():
    """One twisted-column face with the table as E1: every ruling solved without a break, the table angle varies
    per ruling within the window, each cut pose seen from the turned part puts the wire on the ruled face, and the
    KRL post reads back the joints and E1 of the expansion."""
    pytest.importorskip("tesseract_robotics")
    pytest.importorskip("lithocam.studio.core")
    from lithocam.wirepath import load_ruled_json
    st = _column_station(driven="e1", e1_limits_deg=[-360, 360], e1_speed_deg_s=30, e1_max_step_deg=10)
    for s in st.programs[0].steps:
        if s.kind == "wire_ruled":
            s.params.update(faces="0", table_weight=1.0)   # a dearer table: it turns step by step
    robot = program.load_robot("kr120")
    be = planner.make_backend(st, program.CELLS)
    assert be.table_axis
    ex = program.expand(st, robot, st.programs[0], {}, None, backend=be)
    assert not ex.errors
    face = [m for m in ex.messages if "wire face 0:" in m and "rulings" in m and "breaks" in m]
    assert face and " 0 breaks" in face[0] and " 0 missing" in face[0]
    moves = [r for r in ex.records if r.kind in ("movej", "movel")]
    assert all(r.data.get("e1") is not None for r in moves)
    cuts = [r for r in moves if r.data.get("cut")]
    es = [r.data["e1"] for r in cuts]
    assert max(es) - min(es) > 1.0                                  # the table turns during the face
    assert np.max(np.abs(np.diff(es))) <= 10.0 + 1e-9               # within e1_max_step_deg between samples
    # each cut pose, seen from the part turned to its E1, puts the wire on the face (between two rulings)
    rep = load_ruled_json(st.ruled_path)
    f = next(f for f in rep.faces if f.index == 0)
    P0 = np.array([r.p0 for r in f.rulings]) / 1000
    P1 = np.array([r.p1 for r in f.rulings]) / 1000
    P0 = np.vstack([2 * P0[0] - P0[1], P0, 2 * P0[-1] - P0[-2]])   # with the lead-in and lead-out lines
    P1 = np.vstack([2 * P1[0] - P1[1], P1, 2 * P1[-1] - P1[-2]])
    chain = planner.TableChain(robot, st.work_frame())
    bow = program.WIRE_BOW["kr120"]
    worst = 0.0
    for r in cuts:
        T = chain.to_part(np.asarray(r.data["pose"]), math.radians(r.data["e1"]))
        mid = T[:3, 3] + T[:3, 2] * bow
        u = (P1 - P0) / np.linalg.norm(P1 - P0, axis=1)[:, None]
        v = mid - P0
        dist = np.linalg.norm(v - (v * u).sum(1)[:, None] * u, axis=1)
        worst = max(worst, float(dist.min()))
        assert np.allclose(robot.tcp(r.data["q"]), r.data["pose"], atol=1e-5)
    spacing = float(np.median(np.linalg.norm(np.diff((P0 + P1) / 2, axis=0), axis=1)))
    assert worst < 0.6 * spacing             # within half a ruling gap; a wrong table angle would be off by metres
    text = post.write(ex.records, "krl")
    assert np.allclose(post.joint_targets(text, "krl"), [r.data["q"] for r in moves], atol=1e-5)
    assert np.allclose(post.external_axes(text, "krl"), [r.data["e1"] for r in moves], atol=1e-4)
    assert "HALT" not in text


# ---------------------------------------------------------------- the table as a joint in Tesseract
def test_tesseract_table_joint_turns_the_part():
    pytest.importorskip("tesseract_robotics")
    from lithocam.robot import Cell
    c = Cell("kr120")
    c.set_wire_tool(False)
    link = c.add_rotary_table("turntable_e1", (1.4, 0.0, 0.0), disc=(0.8, 0.04))
    c.add_box_obstacle("part", (0.2, 0.2, 1.0), (-0.6, 0.0, 0.5), parent=link)   # 0.6 m off the table axis
    c.allow_collision("part", link)
    T = np.eye(4)
    T[:3, :3] = np.diag([1.0, -1.0, -1.0])
    T[:3, 3] = [0.8, 0.0, 0.95]                                    # the flange over the part at table angle 0
    q = c.ik(T, np.radians([0, -90, 90, 0, 28.6, 0]))[0]
    assert c.in_collision(q, ext=[0.0])
    assert not c.in_collision(q, ext=[math.pi / 2])
    assert not c.in_collision(q, ext=[math.pi])
    assert not c.in_collision(q)                                   # keeps the last table angle
