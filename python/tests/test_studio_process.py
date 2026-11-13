# Copyright 2026 The lithocam authors. SPDX-License-Identifier: Apache-2.0
"""Stock removal along an expanded program and the process check (studio.process), without Qt.

Material values in these tests come from lithocam.materials_mechanics rows; the milling force ratios are set to 0
by the caller (tangential force only, a lower bound), never a material value."""
import math

import pytest

np = pytest.importorskip("numpy")
try:
    from lithocam.studio.core import lc  # noqa: F401
except ImportError:
    pytest.skip("_lithocam not built (bindings/); set LITHOCAM_CORE", allow_module_level=True)

from lithocam import materials_mechanics  # noqa: E402
from lithocam.sdfstock import SdfStock  # noqa: E402
from lithocam.studio import cam, process, program  # noqa: E402

from test_studio_program import dome  # noqa: E402


def row(material, quantity):
    rows = [r for r in materials_mechanics.lookup(material, quantity)
            if r["material"] == material and r["low"] is not None]
    assert rows, (material, quantity)
    return rows[0]


@pytest.fixture(scope="module")
def setup():
    tris = dome()
    part = (tris, cam.trimesh(tris))
    lib = {t.id: t for t in list(lc.stone_tool_library().tools) + list(lc.wood_tool_library().tools)}
    st = program.Station(robot="ur10e", work_xyz=[0.55, 0.15, 0.05], stock=cam.stock_box(tris, 3.0))
    prog = program.Program("Dome", [
        program.Step("set_tool", {"tool": "W7"}),
        program.Step("movej", {"joints": None}),
        program.Step("raster", {"tool": "W7", "stepover": 6.0, "tolerance": 0.05, "feed": 600.0}),
    ])
    robot = program.load_robot("ur10e")
    ex = program.expand(st, robot, prog, lib, part)
    assert not ex.errors and ex.unreachable == 0, ex.messages
    run = process.simulate_stock(st, robot, ex, lib, cell_mm=2.0, snapshots=12)
    return st, robot, ex, lib, run


def test_raster_removes_the_volume_of_a_hand_run(setup):
    st, robot, ex, lib, run = setup
    assert run.total_removed > 1000.0
    # the same cuts written out by hand on a fresh SdfStock: FK with the segment TCP, work frame turned by the yaw
    s = st.stock
    hand = SdfStock.from_box([s["xmin"], s["ymin"], s["zmin"]], [s["xmax"], s["ymax"], s["zmax"]], 2.0)
    v0 = hand.volume()
    tool = lib["W7"]
    R = tool.diameter / 2
    flute = [p for p in process.spindles.cutter_parts(tool, process.spindles.stickout(tool)) if p[0] == "flutes"][0][3]
    removed = 0.0
    for seg in ex.segments:
        if not seg.feed.any():
            continue
        c, sn = math.cos(seg.yaw), math.sin(seg.yaw)
        Rz = np.eye(4)
        Rz[:2, :2] = [[c, -sn], [sn, c]]
        Winv = np.linalg.inv(st.work_frame() @ Rz)
        robot.tcp_offset = seg.tcp_offset
        prev = None
        for k, q in enumerate(seg.q):
            T = Winv @ robot.tcp(q)
            axis = -T[:3, 2]
            centre = T[:3, 3] * 1000 + R * axis
            if seg.feed[k] and prev is not None:
                removed += hand.cut_ball(prev[0], centre, R)
                removed += hand.cut_cylinder(prev[0], centre, prev[1], R, flute - R, 0.0, axis)
            prev = (centre, axis)
    assert abs(run.total_removed - removed) <= 0.01 * removed
    assert abs((v0 - hand.volume()) - removed) <= 0.01 * removed
    assert abs((run.initial_volume - run.stock.volume()) - run.total_removed) <= 0.01 * run.total_removed


def test_snapshots_shrink_monotonically(setup):
    run = setup[4]
    idx = [f[0] for f in run.frames]
    assert idx[0] == -1 and idx == sorted(idx) and len(run.frames) == 13
    v = run.frame_volumes
    assert all(b <= a + 1e-6 for a, b in zip(v, v[1:]))
    assert v[-1] < v[0] - 1000.0
    assert run.frame_at(-1) == 0 and run.frame_at(10 ** 9) == len(run.frames) - 1
    assert run.frame_at(idx[3]) == 3 and run.frame_at(idx[3] - 1) == 2
    V, F = run.frames[-1][1], run.frames[-1][2]
    assert len(V) and F.max() < len(V)


def test_disc_cut_removes_kerf_times_groove_area():
    # a blade of radius 30 mm and kerf 3.6 mm, normal along y, cutting a 6 mm deep groove right through a slab
    L, W, H, depth, Rb, kerf = 60.0, 20.0, 20.0, 6.0, 30.0, 3.6
    stock = SdfStock.from_box([0, -W / 2, 0], [L, W / 2, H], 0.5)
    model = {"kind": process.DISC, "R": Rb, "kerf": kerf}
    P0, P1 = np.eye(4), np.eye(4)
    for P in (P0, P1):
        P[:3, :3] = np.array([[1.0, 0, 0], [0, 0, 1.0], [0, -1.0, 0]])     # tool z = +y, the blade normal
    zc = H + Rb - depth
    P0[:3, 3], P1[:3, 3] = [-Rb - 5, 0, zc], [L + Rb + 5, 0, zc]
    removed = process.cut_step(stock, model, P0, P1)
    want = kerf * L * depth
    assert abs(removed - want) <= 0.03 * want, (removed, want)
    m = process.tool_model([t for t in lc.wood_tool_library().tools if t.id == "W27"][0])
    assert m["kind"] == process.DISC and m["R"] == 125.0 and m["kerf"] == 3.2


def test_process_check_refuses_missing_constants(setup):
    st, robot, ex, lib, run = setup
    bare = program.Station(**{k: getattr(st, k) for k in ("robot", "work_xyz", "stock")}, material="unobtainium")
    rep = process.process_check(bare, run, ex, robot, lib, {})
    text = rep.text()
    assert "SKIPPED milling force: missing specific_energy, radial_ratio, axial_ratio" in text
    assert "SKIPPED part check: missing E, nu, tensile_strength" in text
    assert "SKIPPED TCP deflection: missing joint_stiffness" in text
    assert rep.part is None and not any("Fmag" in r for r in rep.steps)
    # granite has sourced rows, but no row gives the milling force ratios: still skipped, and named
    rep = process.process_check(st, run, ex, robot, lib, {})
    assert "missing radial_ratio, axial_ratio" in rep.text() and rep.part is None
    assert "tensile_strength" in rep.sources and "materials" not in rep.sources["tensile_strength"]


def test_process_check_runs_with_given_constants(setup):
    st, robot, ex, lib, run = setup
    k = row("KUKA KR240-2 (PROXY for a KR120-class arm, not a KR120)", "joint_stiffness")
    c = {"specific_energy": row("granite", "specific_cutting_energy")["low"],
         "E": row("granite", "youngs_modulus")["low"], "nu": row("granite", "poissons_ratio")["low"],
         "tensile_strength": row("granite", "tensile_strength")["low"], "joint_stiffness": k["low"],
         "radial_ratio": 0.0, "axial_ratio": 0.0}
    rep = process.process_check(st, run, ex, robot, lib, c)
    assert all(rep.sources[key] == "given by the caller" for key in c)
    forced = [r for r in rep.steps if "Fmag" in r]
    assert len(forced) > 0.5 * len(run.steps)
    tool = lib["W7"]
    for r in forced[::17]:                     # Ft = u * MRR / v_c, written out
        s = next(x for x in run.steps if x.index == r["index"])
        dt = s.length / r["feed_mm_s"]
        ft = c["specific_energy"] * 1e9 * s.removed * 1e-9 / dt / (math.pi * tool.diameter / 1000 * r["rpm"] / 60)
        assert abs(r["Ft"] - ft) <= 1e-9 * max(ft, 1.0) and r["Fr"] == 0.0 and r["Fa"] == 0.0
        assert r["deflection_mm"] > 0
    assert rep.part is not None and rep.part.max_principal > 0 and rep.part.voxels <= 20000
    assert rep.colors is not None and rep.colors.shape == (len(run.frames[-1][1]), 3)
    assert "SKIPPED milling" not in rep.text() and "SKIPPED part check" not in rep.text()


def test_part_check_flags_the_root_of_a_thin_fin():
    # voxels of 1 mm: a fin 5 mm thick, 29 mm wide and 40 mm tall (z 5.5 to 45.5) on a 29 x 29 x 5 mm base;
    # a side load at the fin top
    cell = 1.0
    gx = np.arange(-16, 16.01, cell)
    gz = np.arange(-1, 48.01, cell)
    X, Y, Z = np.meshgrid(gx, gx, gz, indexing="ij")
    base = (np.abs(X) < 15) & (np.abs(Y) < 15) & (Z > 0) & (Z < 6)
    fin = (np.abs(X) < 2.5) & (np.abs(Y) < 15) & (Z > 0) & (Z < 46)
    stock = SdfStock(np.where(base | fin, -cell, cell), [gx[0], gx[0], gz[0]], cell)
    E = row("granite", "youngs_modulus")["low"] * 1e3              # GPa -> MPa
    nu = row("granite", "poissons_ratio")["low"]
    ft = row("granite", "tensile_strength")["low"]
    res = process.check_part(stock, [30.0, 0.0, 0.0], [0.0, 0.0, 45.5], E, nu, ft, load_radius=3.0)
    assert res.converged
    assert res.flagged_count > 0
    assert 4.0 <= res.peak_point[2] <= 5.5 + 0.25 * 40.0, res.peak_point     # at the root
    assert res.peak_point[0] < 0                                            # on the tension side
    assert res.flagged_points[:, 2].max() < 6.0 + 0.5 * 40.0                # only the lower half of the fin
    assert res.safety_factor < 1.0
