# Copyright 2026 The lithocam authors. SPDX-License-Identifier: Apache-2.0
"""G-code milling post: machine definitions, 3-axis and 5-axis (AC table, head) kinematics, read-back of tip and
tool axis, pole handling, unwinding, arcs, process M-codes and program split."""
import math
from pathlib import Path

import pytest

np = pytest.importorskip("numpy")

from lithocam import post  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]
MACHINES = ROOT / "examples" / "gcode_machines"


def pose(tip_mm, axis):
    """Robot-style record pose (m): z axis from the flange to the tip = minus the tool axis."""
    a = np.asarray(axis, dtype=float)
    a = a / np.linalg.norm(a)
    z = -a
    x = np.cross([0.0, 1.0, 0.0], z)
    if np.linalg.norm(x) < 1e-6:
        x = np.cross([1.0, 0.0, 0.0], z)
    x /= np.linalg.norm(x)
    T = np.eye(4)
    T[:3, :3] = np.column_stack([x, np.cross(z, x), z])
    T[:3, 3] = np.asarray(tip_mm, dtype=float) / 1000
    return T


def recs_from(points, feed=600.0, rpm=8000.0, tool="T1", lead=True):
    """Records of one cutting step: tool change, spindle and water on, an approach joint move, then the points
    (tip mm, axis, cut) as movel records; spindle and water off at the end."""
    r = [post.Rec("start", {"name": "t"}), post.Rec("tool_change", {"id": tool, "mode": "manual"}),
         post.Rec("tool", {"id": tool, "tcp": np.eye(4).tolist()}),
         post.Rec("spindle", {"rpm": rpm, "on": True}), post.Rec("water", {"on": True})]
    if lead:
        r.append(post.Rec("movej", {"q": [0.0] * 6}))
    for tip, ax, cut in points:
        r.append(post.Rec("movel", {"q": [0.0] * 6, "pose": pose(tip, ax).tolist(), "cut": cut,
                                    "v": feed / 60.0 if cut else 100.0}))
    r += [post.Rec("spindle", {"rpm": 0.0, "on": False}), post.Rec("water", {"on": False}), post.Rec("end")]
    return r


def ac_machine(**kw):
    d = post.MillMachine.load(MACHINES / "generic_5axis_ac.json").to_dict()
    d.update({"table_pivots": {"primary": [0.0, 30.0, -120.0], "secondary": [5.0, -10.0, -80.0]},
              "feed_mode": "G94", "max_rotary_step_deg": None})
    d.update(kw)
    return post.MillMachine.from_dict(d)


def test_example_machines_load_and_bad_definitions_are_refused():
    m3 = post.MillMachine.load(MACHINES / "generic_3axis.json")
    m5 = post.MillMachine.load(MACHINES / "generic_5axis_ac.json")
    assert m3.kinematics == "3axis" and m5.kinematics == "table" and m3.placeholder and m5.placeholder
    d = m3.to_dict()
    for bad in ({"arcs": True, "kinematics": "table", "axes": m5.axes}, {"tcp": True}, {"axes": {"x": "X", "y": "Y",
                "z": "F"}}, {"split": "chunks"}, {"max_lines": 5}, {"what": 1}):
        with pytest.raises(post.GcodeError):
            post.MillMachine.from_dict(dict(d, **bad))
    with pytest.raises(post.GcodeError):
        post.MillMachine.from_dict(dict(m5.to_dict(), secondary_axis=[2.0, 0.0, 0.0]))   # parallel to A


@pytest.mark.parametrize("kind", ["table", "head"])
def test_rotary_inverse_and_forward_kinematics_agree(kind):
    rng = np.random.default_rng(3)
    if kind == "table":
        mc = ac_machine()
    else:   # fork head: C on the ram (about z), A on the C (about x), pivots above the gauge point
        mc = ac_machine(kinematics="head", primary_axis=[0.0, 0.0, 1.0], secondary_axis=[1.0, 0.0, 0.0],
                        axes={"x": "X", "y": "Y", "z": "Z", "primary": "C", "secondary": "A"},
                        head_pivots={"primary": [0.0, 0.0, 150.0], "secondary": [0.0, 2.0, 120.0]})
    for _ in range(200):
        v = rng.normal(size=3)
        v[2] = abs(v[2]) + 0.05
        v /= np.linalg.norm(v)
        sols = mc.solutions(v)
        assert len(sols) == 2
        for a1, a2 in sols:
            assert np.linalg.norm(mc.axis_of(a1, a2) - v) < 1e-9
        tip = rng.uniform(-100, 100, 3)
        a1, a2 = sols[0]
        X = mc.to_machine(tip, a1, a2, tool_length=40.0)
        assert np.linalg.norm(mc.from_machine(X, a1, a2, tool_length=40.0) - tip) < 1e-9
    # at zero rotation the programmed point is the tip
    assert np.allclose(mc.to_machine([1.0, 2.0, 3.0], 0.0, 0.0, 40.0), [1.0, 2.0, 3.0])


def test_three_axis_reads_back_equal_and_refuses_a_tilted_tool():
    pts = [((x, y, 5.0 + 0.01 * x), (0, 0, 1), True) for y in (0.0, 4.0) for x in np.linspace(0, 40, 21)]
    pts = [((0, 0, 30.0), (0, 0, 1), False)] + pts
    recs = recs_from(pts)
    mc = post.MillMachine.load(MACHINES / "generic_3axis.json")
    texts = post.write_gcode(recs, mc)
    assert len(texts) == 1
    chk = post.compare_gcode(recs, texts, mc)
    assert chk.ok and chk.max_tip_err <= 1e-3 * math.sqrt(3) / 2 + 1e-12 and chk.max_axis_err < 1e-9
    blocks = [l.split(" ", 1)[1] if l.startswith("N") else l for l in texts[0].splitlines()]
    assert sum(1 for l in blocks if l.startswith("G1 ")) == sum(1 for p in pts if p[2])   # one block per cut
    tilted = recs_from([((0, 0, 5.0), (0.1, 0, 1), True)])
    with pytest.raises(post.GcodeError, match="vertical"):
        post.write_gcode(tilted, mc)
    # the work frame: poses given in a robot base frame read back in the work frame
    W = np.eye(4)
    W[:3, :3] = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1.0]])
    W[:3, 3] = [0.6, 0.1, 0.05]
    moved = [post.Rec(r.kind, dict(r.data, pose=(W @ np.asarray(r.data["pose"])).tolist()))
             if r.data.get("pose") is not None else r for r in recs]
    t2 = post.write_gcode(moved, mc, work_frame=W)
    assert t2 == texts


def test_process_m_codes_wrap_the_cuts():
    pts = [((0, 0, 20.0), (0, 0, 1), False)] + [((x, 0, 2.0), (0, 0, 1), True) for x in range(0, 50, 5)]
    recs = recs_from(pts, rpm=9000)
    mc = post.MillMachine.load(MACHINES / "generic_3axis.json")
    text = post.write_gcode(recs, mc)[0]
    lines = [l.split(" ", 1)[1] if l.startswith("N") else l for l in text.splitlines()]
    cuts = [k for k, l in enumerate(lines) if l.startswith("G1 ")]
    assert lines.index("S9000") < lines.index("M3") < cuts[0] < lines.index("M5")
    assert lines.index("M8") < cuts[0] and lines.index("M9") > cuts[-1]
    assert "M0" in lines and lines.index("M0") < cuts[0]          # manual tool change: operator stop
    assert any("PLACEHOLDER" in l for l in lines)
    # a changer with pockets: T and H words; no cell I/O at all: comments and a warning
    io = dict(mc.io, tool_change={"mode": "changer", "actions": [{"mcode": "T{pocket} M6"}], "pockets": {"T1": 7}})
    recs_c = [post.Rec(r.kind, dict(r.data, mode="changer")) if r.kind == "tool_change" else r for r in recs]
    text = post.write_gcode(recs_c, mc, io=io)[0]
    assert "T7 M6" in text and "G43 H7" in text and "G49" in text
    warn = []
    bare = post.MillMachine.from_dict(dict(mc.to_dict(), io=None))
    text = post.write_gcode(recs, bare, warnings=warn)[0]
    bare_lines = [l.split(" ", 1)[1] if l.startswith("N") else l for l in text.splitlines()]
    assert "M3" not in bare_lines and "map to the cell's spindle I/O" in text
    assert any("no cell I/O" in w for w in warn)


def test_arcs_fit_and_read_back():
    R = 25.0
    mc = post.MillMachine.from_dict(dict(post.MillMachine.load(MACHINES / "generic_3axis.json").to_dict(),
                                         arcs=True, line_numbers=0))

    def circle(n, plane="xy", cw=False):
        ts = np.linspace(0, 1.5 * math.pi, n) * (-1 if cw else 1)
        if plane == "xy":
            P = [(R * math.cos(t), R * math.sin(t), -1.0) for t in ts]
        else:   # a circle in the z-x plane (G18), the tool still vertical
            P = [(R * math.cos(t), 3.0, -30.0 + R * math.sin(t)) for t in ts]
        return [((0, 0, 20.0), (0, 0, 1), False), (P[0][:2] + (20.0,), (0, 0, 1), False)] + \
            [(p, (0, 0, 1), True) for p in P]

    for plane, cw, word, g in (("xy", False, "G3", "G17"), ("xy", True, "G2", "G17"), ("zx", False, None, "G18")):
        recs = recs_from(circle(400, plane, cw))
        text = post.write_gcode(recs, mc)[0]
        arcs = [l for l in text.splitlines() if l.split()[:2] in ([g, "G2"], [g, "G3"])]
        assert 1 <= len(arcs) <= 3, (plane, cw)
        if word:
            assert all(word in l.split() for l in arcs)
        chk = post.compare_gcode(recs, text, mc)
        assert chk.ok and chk.max_tip_err < 5e-3, (plane, cw)
        assert len(text.splitlines()) < 60                     # 400 points became a few arc blocks
    # a coarse polygon (chord error above arc_tol) stays as straight lines
    coarse = post.write_gcode(recs_from(circle(60)), mc)[0]
    assert not any(l.split()[:2] == ["G17", "G3"] for l in coarse.splitlines())


def _pole_path():
    """A pass whose tool axis tilts through vertical (the C pole) and then sweeps round in azimuth."""
    pts = [((-30.0, 0.0, 30.0), (0, 0, 1), False)]
    for k, x in enumerate(np.linspace(-30, 30, 61)):
        tilt = math.radians(x)                     # -30..30 deg through 0, in the x-z plane
        pts.append(((x, 0.0, 0.0), (math.sin(tilt), 0.0, math.cos(tilt)), True))
    for k, t in enumerate(np.linspace(0, 4 * math.pi, 181)[1:]):   # two full turns of azimuth at 30 deg tilt
        s = math.radians(30)
        pts.append(((30.0 + 5 * math.sin(t), 5 * (1 - math.cos(t)), 0.0),
                    (math.sin(s) * math.cos(t), math.sin(s) * math.sin(t), math.cos(s)), True))
    return pts


def test_pole_is_held_and_c_unwinds_without_jumps():
    pts = _pole_path()
    mc = ac_machine()
    ang, dev, rew = post.solve_rotary(mc, pts)
    A, C = np.array([a for a, _ in ang]), np.array([c for _, c in ang])
    assert max(dev) <= mc.pole_deg + 1e-9
    assert np.max(np.abs(np.diff(C[1:]))) < 6.0              # no C flip at the exit of the pole cone
    assert np.max(np.abs(np.diff(A[1:]))) < 2.0
    assert abs(abs(C[-1] - C[62]) - 720.0) < 5.0              # two turns of azimuth: C unwound past 360
    assert not any(rew)
    for (tip, ax, _), (a, c), d in zip(pts, ang, dev):
        assert abs(math.degrees(math.acos(min(1.0, mc.axis_of(a, c) @ ax))) - d) < 1e-6
    # with C limited to +-200 deg the second turn needs a rewind: retract, turn, re-enter
    lim = ac_machine(limits={"C": [-200.0, 200.0]})
    ang2, _, rew2 = post.solve_rotary(lim, pts)
    assert any(rew2) and all(-200 - 1e-9 <= c <= 200 + 1e-9 for _, c in ang2)
    text = post.write_gcode(recs_from(pts), lim)[0]
    assert "rewind" in text
    assert post.compare_gcode(recs_from(pts), text, lim).ok


def test_five_axis_tcp_and_linearisation_read_back():
    pts = _pole_path()
    recs = recs_from(pts)
    for kw in ({"tcp": False}, {"tcp": True}, {"tcp": False, "max_rotary_step_deg": 1.0, "feed_mode": "G93",
                                                  "rotary_feed_deg_min": 3600.0}):
        mc = ac_machine(**kw)
        io = dict(mc.io, tool_change={"mode": "manual", "pockets": {"T1": 1}})
        text = post.write_gcode(recs, mc, io=io)[0]
        chk = post.compare_gcode(recs, text, mc, max_skip=400)
        assert chk.ok, kw
        assert chk.max_tip_err < 2e-3 and chk.max_axis_err <= mc.pole_deg + 1e-3
        assert ("G43.4 H1" in text) == kw["tcp"] and ("G43 H1" in text) == (not kw["tcp"])
        if kw["tcp"]:   # tool centre point control: the program carries the tip itself
            for (tip, _, _), g in zip([p for p in pts if p[2]], [g for g in post.gcode_points(text, mc)
                                                                 if g.kind == "line"]):
                assert np.max(np.abs(g.tip - np.asarray(tip))) <= 5e-4 + 1e-12
        if kw.get("feed_mode") == "G93":
            g1 = [l for l in text.splitlines() if l.startswith("G1 ")]
            assert all(" F" in l for l in g1)
            steps = []
            last = None
            for g in post.gcode_points(text, mc):
                if g.kind == "line" and last is not None:
                    steps.append(max(abs(g.angles[0] - last[0]), abs(g.angles[1] - last[1])))
                last = g.angles
            assert max(steps) <= 1.0 + 1e-3


def test_split_subprograms_and_files_keep_the_path():
    pts = [((0, 0, 30.0), (0, 0, 1), False)]
    for y in range(0, 60, 3):
        pts.append(((0, y, 10.0), (0, 0, 1), False))
        pts += [((x, y, 2.0 + 0.001 * x * y), (0, 0, 1), True) for x in range(0, 100, 2)]
        pts.append(((98, y, 10.0), (0, 0, 1), False))
    recs = recs_from(pts)
    base = post.MillMachine.load(MACHINES / "generic_3axis.json").to_dict()
    whole = post.write_gcode(recs, base)[0]
    ref = [g.tip for g in post.gcode_points(whole, base)]
    for split in ("subprograms", "files"):
        mc = dict(base, max_lines=150, split=split)
        texts = post.write_gcode(recs, mc)
        assert len(texts) > 4
        assert all(t.count("\n") <= 150 for t in texts)
        chk = post.compare_gcode(recs, texts, mc)
        assert chk.ok, split
        got = [g.tip for g in post.gcode_points(texts, mc)]
        if split == "subprograms":
            assert texts[0].count("M98 P") == len(texts) - 1 and all("M99" in t for t in texts[1:])
            assert len(got) == len(ref) and np.max(np.abs(np.array(got) - np.array(ref))) == 0.0
        else:
            # every continuation file restarts spindle and water before its first cut and comes back to the last
            # position of the file before it
            for k in range(1, len(texts)):
                ls = [l.split(" ", 1)[1] if l.startswith("N") else l for l in texts[k].splitlines()]
                first_cut = next(i for i, l in enumerate(ls) if l.startswith("G1 "))
                assert "M3" in ls[:first_cut] and "M8" in ls[:first_cut]
                prev = post.gcode_points(texts[k - 1], mc)        # its last point is the stop retract
                here = post.gcode_points(texts[k], mc)            # [0] over the point at safe Z, [1] back on it
                assert np.linalg.norm(here[1].tip - prev[-2].tip) < 1e-9
            # the reference points appear in order in the split read-back
            j = 0
            for p in ref:
                while j < len(got) and np.linalg.norm(got[j] - p) > 1e-9:
                    j += 1
                assert j < len(got)
                j += 1


# ---------------------------------------------------------------- the studio's dome programs (needs the C++ core)
@pytest.fixture(scope="module")
def dome_setup():
    try:
        from lithocam.studio.core import lc
    except ImportError:
        pytest.skip("_lithocam not built (bindings/); set LITHOCAM_CORE")
    from lithocam.studio import cam
    from test_studio_program import dome
    t = dome()
    lib = lc.stone_tool_library()
    tools = {x.id: x for x in list(lib.tools) + list(lc.wood_tool_library().tools)}
    return (t, cam.trimesh(t)), tools, lc


def test_dome_raster_posts_three_axis_and_reads_back_equal(dome_setup):
    from lithocam.studio import cam, program
    part, tools, _ = dome_setup
    st = program.Station(robot="ur10e", work_xyz=[0.55, 0.15, 0.05], work_rpy=[0.0, 0.0, 20.0],
                         stock=cam.stock_box(part[0], 3.0))
    prog = program.Program("P", [program.Step("raster", {"tool": "W7", "stepover": 6.0, "feed": 600.0,
                                                         "tolerance": 0.01})])
    ex = program.expand(st, program.load_robot("ur10e"), prog, tools, part)
    W = st.work_frame()
    mc = post.MillMachine.load(MACHINES / "generic_3axis.json")
    texts = post.write_gcode(ex.records, mc, work_frame=W)
    chk = post.compare_gcode(ex.records, texts, mc, work_frame=W)
    cl = post.cl_points(ex.records, W)
    assert chk.ok and chk.n == len(cl) and chk.max_tip_err <= 1e-3 and chk.max_axis_err < 1e-6
    blocks = [l.split(" ", 1)[1] for l in texts[0].splitlines() if l.startswith("N")]
    assert sum(1 for l in blocks if l.startswith("G1 ")) == sum(1 for p in cl if p[2])
    # the CL tips are the generator's points in the work frame
    moves = [m for m in cam.simplify(cam.raster(part[0], part[1], tools["W7"], st.stock, stepover=6.0,
                                                tolerance=0.01), 0.01) if m[0] in ("feed", "plunge")]
    cuts = [p for p, _, c in cl if c]
    assert len(cuts) == len(moves) and max(np.linalg.norm(np.asarray(m[1]) - p) for m, p in zip(moves, cuts)) < 1e-9


def test_dome_surfacing5_posts_on_an_ac_table_and_reads_back(dome_setup):
    """The 5-axis dome surfacing step: A/C from the tool axis, read back by forward kinematics of A, C and XYZ
    through the pivots, reproduces every tip and tool axis; the 3-axis machine refuses the step."""
    from lithocam.studio import cam, program
    part, tools, lc = dome_setup
    st = program.Station(robot="ur10e", work_xyz=[0.55, 0.15, 0.05], stock=cam.stock_box(part[0], 3.0))
    ball = [t for t in tools.values() if t.kind == lc.ToolKind.BallEndMill and 4 <= t.diameter <= 12][0]
    prog = program.Program("F5", [program.Step("surfacing5", {"tool": ball.id, "stepover": 6.0, "feed": 600.0,
                                                              "spins": 8})])
    ex = program.expand(st, program.load_robot("ur10e"), prog, tools, part)
    W = st.work_frame()
    cl = post.cl_points(ex.records, W)
    assert sum(1 for _, a, c in cl if c and a[2] < math.cos(math.radians(5))) > 20     # tilted cutting points
    with pytest.raises(post.GcodeError, match="vertical"):
        post.write_gcode(ex.records, post.MillMachine.load(MACHINES / "generic_3axis.json"), work_frame=W)
    for kw in ({}, {"max_rotary_step_deg": 2.0, "feed_mode": "G93", "rotary_feed_deg_min": 3600.0}, {"tcp": True}):
        mc = ac_machine(**kw)
        warn = []
        texts = post.write_gcode(ex.records, mc, work_frame=W, warnings=warn)
        chk = post.compare_gcode(ex.records, texts, mc, work_frame=W, max_skip=400)
        assert chk.ok, (kw, chk.n, chk.matched)
        assert chk.max_tip_err <= 2e-3, kw
        assert chk.max_axis_err <= mc.pole_deg + 1e-3, kw
        ang, dev, _ = post.solve_rotary(mc, cl)
        off_pole = [e for e, d in zip(chk.axis_errors, dev) if d < 1e-9]
        assert off_pole and max(off_pole) < 1e-3, kw          # away from the pole: rounding only
        C = np.array([c for _, c in ang])
        cut = np.array([c for _, _, c in cl])
        both = cut[1:] & cut[:-1]
        # no solution flip inside a cut (a flip turns C by about 180 deg); near the apex the surface normal swings
        # in azimuth a few degrees from vertical, so C steps of tens of degrees there are real
        assert np.max(np.abs(np.diff(C))[both]) < 90.0, kw
        if kw.get("max_rotary_step_deg"):                      # linearised: every block turns at most the step
            g = post.gcode_points(texts, mc)
            steps = [max(abs(b.angles[0] - a.angles[0]), abs(b.angles[1] - a.angles[1]))
                     for a, b in zip(g, g[1:]) if b.kind == "line"]
            assert max(steps) <= kw["max_rotary_step_deg"] + 1e-3
