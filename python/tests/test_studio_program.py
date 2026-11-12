# Copyright 2026 The lithocam authors. SPDX-License-Identifier: Apache-2.0
"""Studio program expansion and posting: a raster-finishing program on a small dome, run on the UR10e and the
KR120 cells; the posted joint targets read back equal the expansion's; posts round-trip through the CL dialect."""
import math

import pytest

np = pytest.importorskip("numpy")
try:
    from lithocam.studio.core import lc  # noqa: F401  (the compiled core, built from bindings/)
except ImportError:
    pytest.skip("_lithocam not built (bindings/); set LITHOCAM_CORE", allow_module_level=True)

from lithocam import post  # noqa: E402
from lithocam.studio import cam, program  # noqa: E402
from lithocam.studio.kinematics import rotation_error  # noqa: E402


def dome(n=24, size=60.0, h=12.0):
    xs = np.linspace(-size / 2, size / 2, n + 1)
    Z = lambda x, y: h * max(0.0, 1 - (x * x + y * y) / (size / 2) ** 2)   # noqa: E731
    tris = []
    for i in range(n):
        for j in range(n):
            a = (xs[i], xs[j], Z(xs[i], xs[j]))
            b = (xs[i + 1], xs[j], Z(xs[i + 1], xs[j]))
            c = (xs[i + 1], xs[j + 1], Z(xs[i + 1], xs[j + 1]))
            d = (xs[i], xs[j + 1], Z(xs[i], xs[j + 1]))
            tris += [(a, b, c), (a, c, d)]
    return np.array(tris, dtype=float)


@pytest.fixture(scope="module")
def part():
    t = dome()
    return t, cam.trimesh(t)


@pytest.fixture(scope="module")
def tools():
    lib = lc.stone_tool_library()
    wood = lc.wood_tool_library()
    return {t.id: t for t in list(lib.tools) + list(wood.tools)}


def raster_program():
    return program.Program("Dome", [
        program.Step("set_tool", {"tool": "W7"}),
        program.Step("movej", {"joints": None}),
        program.Step("speed", {"linear": 50.0, "joint": 30.0}),
        program.Step("raster", {"tool": "W7", "stepover": 6.0, "tolerance": 0.05}),
        program.Step("io", {"mode": "set", "port": 2, "value": False}),
        program.Step("pause", {"seconds": 0.5}),
        program.Step("polish", {}),
    ])


@pytest.mark.parametrize("robot_name,work", [("ur10e", [0.55, 0.15, 0.05]), ("kr120", [1.6, 0.0, 0.3])])
def test_raster_program_expands_and_posts(robot_name, work, part, tools):
    st = program.Station(robot=robot_name, work_xyz=work, stock=cam.stock_box(part[0], 3.0))
    robot = program.load_robot(robot_name)
    ex = program.expand(st, robot, raster_program(), tools, part)
    assert ex.unreachable == 0, ex.messages
    seg = [s for s in ex.segments if s.feed.any()][0]
    # every robot sample puts the TCP on the CL point with the tool axis down
    moves = cam.raster(part[0], part[1], tools["W7"], st.stock, stepover=6.0, tolerance=0.05)
    assert len(seg.q) == len(moves)
    W = st.work_frame()
    for (kind, p), q in zip(moves[::25], seg.q[::25]):
        T = robot.tcp(q)
        want = program.down_pose(W, p)
        assert np.linalg.norm(T[:3, 3] - want[:3, 3]) < 1e-5
        assert np.linalg.norm(rotation_error(T[:3, :3], want[:3, :3])) < 1e-5
    # posting: joint targets read back equal the records, for every dialect
    rec_q = [r.data["q"] for r in ex.records if r.kind in ("movej", "movel")]
    for dialect in ("urscript", "krl", "rapid", "cl"):
        text = post.write(ex.records, dialect)
        back = post.joint_targets(text, dialect)
        assert len(back) == len(rec_q), dialect
        tol = 1e-6 if dialect == "urscript" else (1e-12 if dialect == "cl" else math.radians(1e-4))
        assert np.max(np.abs(np.asarray(back) - np.asarray(rec_q))) < tol, dialect
    assert any("polish" in m.lower() or "Polish" in m for m in ex.messages)


def test_cl_round_trip_and_tool_frames():
    T = np.eye(4)
    T[:3, :3] = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1.0]])
    T[:3, 3] = [0.01, -0.02, 0.15]
    recs = [post.Rec("start", {"name": "t"}), post.Rec("tool", {"id": "B10", "tcp": T.tolist()}),
            post.Rec("movej", {"q": [0.1, -1.2, 1.3, -0.4, 0.5, 0.6]}), post.Rec("end")]
    back = post.parse_cl(post.write(recs, "cl"))
    assert [r.kind for r in back] == [r.kind for r in recs]
    ur = post.write(recs, "urscript")
    assert "set_tcp(p[0.010000, -0.020000, 0.150000, 0.000000, 0.000000, 1.570796])" in ur
    krl = post.write(recs, "krl")
    assert "$TOOL = {X 10.000, Y -20.000, Z 150.000, A 90.0000, B -0.0000, C 0.0000}" in krl or \
        "$TOOL = {X 10.000, Y -20.000, Z 150.000, A 90.0000, B 0.0000, C 0.0000}" in krl
    rapid = post.write(recs, "rapid")
    w, x, y, z = post.quat(T[:3, :3])
    assert abs(w - math.cos(math.pi / 4)) < 1e-12 and abs(z - math.sin(math.pi / 4)) < 1e-12
    assert "PERS tooldata t_B10" in rapid


def test_rough_plane_stays_above_the_part_and_inside_the_stock(part, tools):
    tris, mesh = part
    stock = cam.stock_box(tris, 3.0)
    tool = tools["W2"]   # flat spiral up-cut
    moves = cam.rough_plane(tris, mesh, tool, stock, stepdown=4.0, stepover=3.0, allowance=0.5)
    feed = np.array([p for k, p in moves if k == "feed"])
    assert len(feed) > 0
    assert feed[:, 2].max() < stock["zmax"]
    # no feed point gouges: the tip is at or above the drop-cutter height plus the allowance
    c = lc.cutter_of(tool)
    for p in feed[::7]:
        h = lc.drop_cutter(c, mesh, float(p[0]), float(p[1]))
        assert p[2] >= h + 0.5 - 1e-9 or not math.isfinite(h)


def test_station_json_round_trip():
    st = program.Station(stock={"xmin": 0, "xmax": 1, "ymin": 0, "ymax": 1, "zmin": 0, "zmax": 1},
                         programs=[raster_program()])
    back = program.Station.from_json(st.to_json())
    assert back.to_json() == st.to_json()
