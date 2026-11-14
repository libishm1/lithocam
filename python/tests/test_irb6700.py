# Copyright 2026 The lithocam authors. SPDX-License-Identifier: Apache-2.0
"""ABB IRB 6700-200/2.60 cell (ROS-Industrial abb_irb6700_support, Apache-2.0): model data against the ABB product
specification numbers in the URDF, registration in the studio, numpy and Tesseract IK round trips, collision at the
home pose, the RAPID post with its read-back, and the turntable as external axis E1."""
import math
import struct
from pathlib import Path

import pytest

np = pytest.importorskip("numpy")
from lithocam import cell_io, post  # noqa: E402
from lithocam.studio import program, spindles  # noqa: E402
from lithocam.studio.kinematics import load_urdf, rotation_error  # noqa: E402

CELLS = Path(__file__).resolve().parents[2] / "cells"
# ABB IRB 6700 product specification, 200/2.60 variant (the xacro's limits equal it): range deg, speed deg/s
SPEC = [(-170, 170, 110), (-65, 85, 110), (-180, 70, 110), (-300, 300, 190), (-130, 130, 150), (-360, 360, 210)]


@pytest.fixture(scope="module")
def arm():
    return load_urdf(CELLS / "irb6700.urdf")


# ---------------------------------------------------------------- model data
def test_limits_speeds_and_link_lengths_match_the_spec(arm):
    assert arm.dof == 6
    assert np.degrees(arm.lower) == pytest.approx([s[0] for s in SPEC])
    assert np.degrees(arm.upper) == pytest.approx([s[1] for s in SPEC])
    import re
    text = (CELLS / "irb6700.urdf").read_text(encoding="utf-8")
    vel = [float(v) for v in re.findall(r'<limit effort="0" lower="[^"]+" upper="[^"]+" velocity="([^"]+)"', text)][:6]
    assert np.degrees(vel) == pytest.approx([s[2] for s in SPEC], abs=1e-4)
    # zero pose: upper arm up, forearm along +x, wrist 0.2 m above the A3 axis: flange at 0.32 + 1.1425 + 0.2, 0.78 + 1.125 + 0.2
    T = arm.tcp(np.zeros(6))
    assert T[:3, 3] == pytest.approx([0.32 + 1.1425 + 0.2, 0.0, 0.78 + 1.125 + 0.2], abs=1e-9)
    assert T[:3, 2] == pytest.approx([1.0, 0.0, 0.0], abs=1e-9)          # tool0 z is the flange axis, here along +x


def test_reach_matches_the_model_name(arm):
    """The 2.60 of 200/2.60 is the reach in metres to the wrist centre (A4/A5 intersection): stretched out, the
    wrist centre is 0.32 + 1.125 + hypot(1.1425, 0.2) = 2.605 m from axis 1, within 0.5 percent."""
    best = 0.0
    for q2 in np.radians(np.linspace(0, 85, 18)):
        for q3 in np.radians(np.linspace(-180, 70, 126)):
            T = arm.tcp([0, q2, q3, 0, 0, 0])
            wrist = T[:3, 3] - 0.2 * T[:3, 2]          # flange minus the 0.2 m wrist length
            best = max(best, math.hypot(wrist[0], wrist[1]))
    assert best == pytest.approx(2.60, rel=0.005)


def test_vendored_meshes_carry_licence_and_credit():
    d = CELLS / "meshes" / "irb6700"
    assert "Apache License" in (d / "LICENSE").read_text(encoding="utf-8")
    assert "ROS-Industrial abb_irb6700_support, Apache-2.0" in (d / "NOTICE").read_text(encoding="utf-8")
    readme = (CELLS.parent / "README.md").read_text(encoding="utf-8")
    assert "ABB IRB 6700 model from ROS-Industrial abb_irb6700_support, Apache-2.0" in readme
    for n in ("base_link", "link_1", "link_2", "link_3", "link_4", "link_5", "link_6"):
        for f in (d / (n + ".stl"), d / "visual" / (n + ".stl")):
            b = f.read_bytes()
            assert 84 + 50 * struct.unpack("<I", b[80:84])[0] == len(b), f      # binary STL, whole file


# ---------------------------------------------------------------- registration
def test_every_mesh_the_urdf_names_exists(arm):
    import re
    names = re.findall(r'package://cells/(meshes/[^"]+)', (CELLS / "irb6700.urdf").read_text(encoding="utf-8"))
    assert len(names) == 14 and all((CELLS / n).is_file() for n in names)         # 7 links, collision and visual
    assert all(Path(v.mesh).is_file() for vs in arm.visuals.values() for v in vs if v.mesh)   # the studio's loader


def test_registered_in_the_studio_and_the_api():
    from lithocam.studio import api
    assert program.ROBOTS["irb6700"] == ("ABB IRB 6700-200/2.60", "irb6700.urdf", "rapid")
    assert api.robots()["irb6700"] == "ABB IRB 6700-200/2.60"
    assert "irb6700" in api.station_schema()["properties"]["robot"]["enum"]
    assert len(program.HOME["irb6700"]) == 6 and program.WIRE_BOW["irb6700"] == 0.6
    assert program.POST_OPTIONS["irb6700"] == {}
    assert program.load_robot("irb6700").dof == 6
    sp = program.Station(robot="irb6700").spindle_model()
    assert sp.id == "irb6700_example" and sp.source == "example"
    assert spindles.DEFAULT_FOR_ROBOT["irb6700"] == "irb6700_example"


def test_shipped_cell_io_is_a_rapid_placeholder():
    io = cell_io.load(CELLS / "irb6700_io.json")
    assert io.controller == "rapid" and io.placeholder and io.tool_mode() == "manual"
    assert any("PLACEHOLDER" in w for w in io.warnings)


# ---------------------------------------------------------------- IK
def test_numpy_ik_round_trip_2000_poses(arm):
    """The studio's IK (damped least squares on the URDF chain, seeded from a nearby configuration as the planner
    does) reproduces 2000 random reachable poses: position within 0.01 mm and orientation within 0.01 degree."""
    rng = np.random.default_rng(6700)
    lo, hi = arm.lower, arm.upper
    mid, half = (lo + hi) / 2, (hi - lo) / 2
    worst_p = worst_r = 0.0
    solved = first_try = 0
    unsolved = []
    for _ in range(2000):
        q = mid + rng.uniform(-0.7, 0.7, 6) * half
        target = arm.tcp(q)
        for attempt in range(8):             # the planner restarts from further seeds when one does not converge
            qs, ok = arm.solve_pose(target, q + rng.normal(0, min(0.15 * (attempt + 1), 0.6), 6))
            if ok:
                first_try += attempt == 0
                break
        if not ok:
            unsolved.append(abs(math.degrees(q[4])))
            continue
        solved += 1
        T = arm.tcp(qs)
        worst_p = max(worst_p, float(np.linalg.norm(T[:3, 3] - target[:3, 3])))
        worst_r = max(worst_r, float(np.linalg.norm(rotation_error(T[:3, :3], target[:3, :3]))))
    print("irb6700 numpy IK: solved {}/2000 ({} on the first seed), max position error {:.3e} mm, max orientation "
          "error {:.3e} deg".format(solved, first_try, worst_p * 1000, math.degrees(worst_r)))
    print("unsolved poses, |A5| in degrees:", np.round(unsolved, 2).tolist())
    assert first_try >= 1950 and solved >= 1995
    assert all(a < 3.0 for a in unsolved)        # a pose that never converges has A5 within 3 degrees of the wrist singularity
    assert worst_p * 1000 < 0.01 and math.degrees(worst_r) < 0.01


robot = pytest.importorskip("lithocam.robot", reason="tesseract-robotics not installed", exc_type=ImportError)


@pytest.fixture(scope="module")
def cell():
    return robot.Cell("irb6700")


def test_tesseract_cell_loads_and_opw_matches_fk(cell):
    assert cell.joint_names == ["joint_{}".format(i) for i in range(1, 7)]
    assert np.degrees(cell.limits[:, 0]) == pytest.approx([s[0] for s in SPEC])
    assert cell.fk(np.zeros(6))[:3, 3] == pytest.approx([1.6625, 0.0, 2.105], abs=1e-9)
    rng = np.random.default_rng(3)
    analytic = total = 0
    for _ in range(100):
        q = cell.limits[:, 0] + rng.random(6) * (cell.limits[:, 1] - cell.limits[:, 0])
        T = cell.fk(q)
        sols = cell._raw_ik(cell.kin, T, q)            # the OPW solver alone, before the verification filter
        total += len(sols)
        analytic += sum(np.linalg.norm(cell.fk(s) - T) < 1e-6 for s in sols)
        verified = cell.ik(T, q)
        assert verified
        for s in verified:
            assert np.linalg.norm(cell.fk(s) - T) < 1e-6 and cell.within_limits(s)
    assert analytic == total and total > 100          # every analytic solution is a true one


def test_home_is_clear_of_the_floor_and_of_itself(cell):
    from tesseract_robotics.tesseract_environment import ChangeLinkCollisionEnabledCommand  # noqa: F401
    q = np.asarray(program.HOME["irb6700"])
    assert cell.within_limits(q)
    for wire in (True, False):
        cell.set_wire_tool(wire)
        assert not cell.in_collision(q)
    cell.set_wire_tool(True)
    cell.add_box_obstacle("floor", (8.0, 8.0, 0.2), (0.0, 0.0, -0.1 - 0.002))      # top 2 mm below the base
    assert not cell.in_collision(q)
    assert cell.in_collision(np.radians([0.0, 45.0, 70.0, 0.0, 0.0, 0.0]))          # forearm and bow driven into the floor


# ---------------------------------------------------------------- post and E1
def _dome(path, n=20, size=60.0, h=12.0):
    xs = np.linspace(-size / 2, size / 2, n + 1)
    Z = lambda x, y: h * max(0.0, 1 - (x * x + y * y) / (size / 2) ** 2)   # noqa: E731
    tris = []
    for i in range(n):
        for j in range(n):
            a, b = (xs[i], xs[j], Z(xs[i], xs[j])), (xs[i + 1], xs[j], Z(xs[i + 1], xs[j]))
            c, d = (xs[i + 1], xs[j + 1], Z(xs[i + 1], xs[j + 1])), (xs[i], xs[j + 1], Z(xs[i], xs[j + 1]))
            tris += [(a, b, c), (a, c, d)]
    with open(path, "wb") as f:
        f.write(b"\0" * 80 + struct.pack("<I", len(tris)))
        for t in tris:
            f.write(struct.pack("<12fH", 0, 0, 0, *[c for p in t for c in p], 0))


def test_program_simulates_posts_rapid_and_reads_back(tmp_path):
    pytest.importorskip("lithocam.studio.core", reason="_lithocam not built", exc_type=ImportError)
    from lithocam.studio import api
    stl = tmp_path / "dome.stl"
    _dome(stl)
    s = api.Session("irb6700", material="granite")
    s.load_part(stl, stock_margin=3.0)
    s.set_work_frame([1.6, 0.0, 0.3])
    home = np.degrees(program.HOME["irb6700"]).tolist()
    s.add_target("approach", joints=home)
    p = s.program("Main")
    p.add("set_tool", tool="W7")
    p.add("movej", target="approach", speed=20.0)
    p.add("raster", tool="W7", stepover=8.0, feed=600.0)
    p.add("movej", target="approach")
    r = s.simulate("Main")
    assert r.ok, r.summary() + "\n" + "\n".join(r.messages)
    text = s.post("Main", "rapid")
    assert "MODULE" in text and "MoveAbsJ" in text and "MoveL" in text
    with pytest.raises(api.ApiError, match="another controller"):
        s.post("Main", "krl")
    # read-back: the joints in the RAPID text are the joints of the simulated expansion
    qs = [rec.data["q"] for rec in r.expansion.records if rec.kind in ("movej", "movel")]
    back = post.joint_targets(text, "rapid")
    assert len(back) == len(qs) and np.allclose(back, qs, atol=1e-5)
    f = s.save(tmp_path / "st.lcs.json")
    s2 = api.Session.open(f)
    assert s2.station.robot == "irb6700" and s2.simulate("Main").samples == r.samples


def _e1_station():
    return program.Station(robot="irb6700", work_xyz=[1.8, 0.0, 0.3],
                           turntable={"diameter": 0.8, "height": 0.04, "driven": "e1", "e1_limits_deg": [-360, 360],
                                      "e1_speed_deg_s": 30, "e1_max_step_deg": 10})


def test_turntable_as_e1_posts_rapid_extax_and_reads_back():
    st = _e1_station()
    mode, errs, warns = program.check_table(program.Station(robot="irb6700", turntable={
        "diameter": 0.8, "height": 0.04, "driven": "e1"}))
    assert mode == "e1" and not errs and any("PLACEHOLDER" in w for w in warns)
    prog = program.Program("T", [program.Step("movej", {"joints": [0.0, 0.0, 0.0, 0.0, 30.0, 0.0]}),
                                 program.Step("setup", {"yaw": 45.0}),
                                 program.Step("movej", {"joints": [10.0, 0.0, 0.0, 0.0, 30.0, 0.0]})])
    ex = program.expand(st, program.load_robot("irb6700"), prog, {}, None)
    assert not ex.errors, ex.errors
    moves = [r for r in ex.records if r.kind in ("movej", "movel")]
    assert [m.data["e1"] for m in moves] == [0.0, 45.0]
    text = post.write(ex.records, "rapid")
    assert "[45.0000, 9E9, 9E9, 9E9, 9E9, 9E9]" in text and "Stop;" not in text and "PLACEHOLDER" in text
    assert np.allclose(post.joint_targets(text, "rapid"), [m.data["q"] for m in moves], atol=1e-5)
    assert post.external_axes(text, "rapid") == [0.0, 45.0]


def test_turntable_joint_turns_the_part_in_the_irb6700_scene():
    from lithocam.studio import planner
    st = _e1_station()
    be = planner.make_backend(st, program.CELLS)
    assert be is not None and be.table_axis
    c = be.cell
    c.set_wire_tool(False)
    link = "turntable_e1_link"
    c.add_box_obstacle("post", (0.2, 0.2, 1.0), (-0.6, 0.0, 0.8), parent=link)       # 0.6 m off the table axis
    c.allow_collision("post", link)
    T = np.eye(4)
    T[:3, :3] = np.diag([1.0, -1.0, -1.0])
    T[:3, 3] = [1.2, 0.0, 1.2]                                                         # the flange over the post at angle 0
    sols = c.ik(T, np.radians([0, 0, 0, 0, 30, 0]))
    assert sols
    q = sols[0]
    assert c.in_collision(q, ext=[0.0])
    assert not c.in_collision(q, ext=[math.pi])
