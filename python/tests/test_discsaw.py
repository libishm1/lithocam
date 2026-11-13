# Copyright 2026 The lithocam authors. SPDX-License-Identifier: Apache-2.0
"""Disc saw toolpaths: kerf face on the edge line, no intrusion into the part, corner policy, bevel, combing, refusal."""
import json
import math

import numpy as np
import pytest

from lithocam import discsaw as ds

R, KERF, TOP, T = 175.0, 3.2, 0.0, 30.0
KW = dict(top=TOP, thickness=T, blade_diameter=350.0, kerf=KERF, flange_diameter=100.0, stepdown=10.0)
L_SHAPE = np.array([[0, 0], [600, 0], [600, 300], [300, 300], [300, 500], [0, 500]], float)   # reflex at index 3


def lowest_point(c: ds.Cut, which: str):
    g = -np.cross(np.asarray(c.normal), np.asarray(c.direction))     # in-plane down
    return np.asarray(getattr(c, which)) + c.radius * g


def star(rng, n=None):
    n = n or int(rng.integers(5, 13))
    while True:                      # keep every angular gap below 160 degrees so the origin is inside (star-shaped)
        ang = np.sort(rng.uniform(0, 2 * math.pi, n))
        if np.max(np.diff(np.append(ang, ang[0] + 2 * math.pi))) < math.radians(160):
            break
    r = rng.uniform(90, 260, n)
    return np.stack([r * np.cos(ang), r * np.sin(ang)], axis=1)


def l_shape(rng):
    w, h = rng.uniform(250, 600), rng.uniform(250, 600)
    a, b = rng.uniform(0.3, 0.7) * w, rng.uniform(0.3, 0.7) * h
    return np.array([[0, 0], [w, 0], [w, b], [a, b], [a, h], [0, h]], float)


def test_runout_formula():
    assert ds.runout(175, 20) == pytest.approx(math.sqrt(2 * 175 * 20 - 400))
    assert ds.runout(175, 0) == 0.0
    # the rim really is at the top surface e(d) from the lowest point: centre is R - d above
    d = 31.0
    assert math.hypot(ds.runout(175, d), 175 - d) == pytest.approx(175.0)


def test_kerf_face_on_edge_line_and_midplane_offset():
    cuts = ds.profile_cuts(L_SHAPE, depth=20.0, **KW)
    assert cuts
    n = len(L_SHAPE)
    for c in cuts:
        a = np.append(L_SHAPE[c.edge], TOP)
        b = np.append(L_SHAPE[(c.edge + 1) % n], TOP)
        N = np.asarray(c.normal)
        face_pt = np.asarray(c.start) - 0.5 * KERF * N
        assert abs((a - face_pt) @ N) < 1e-9 and abs((b - face_pt) @ N) < 1e-9
        assert abs((np.asarray(c.start) - a) @ N - 0.5 * KERF) < 1e-9      # mid-plane is kerf/2 to the waste side
        assert abs(np.linalg.norm(N) - 1) < 1e-12
    rep = ds.blade_intrudes(cuts, L_SHAPE, T)
    assert max(rep.face_plane_error.values()) < 1e-9


def test_pose_axes():
    c = ds.profile_cuts(L_SHAPE, depth=20.0, **KW)[0]
    Tm = ds.disc_pose(c, 0.5)
    Rm = Tm[:3, :3]
    assert np.allclose(Rm @ Rm.T, np.eye(3), atol=1e-12) and np.linalg.det(Rm) == pytest.approx(1.0)
    assert np.allclose(Rm[:, 2], c.normal) and np.allclose(Rm[:, 0], c.direction)
    assert np.allclose(Tm[:3, 3], 0.5 * (np.asarray(c.start) + np.asarray(c.end)))


def test_convex_corners_reach_full_depth_and_overcut():
    d = 20.0
    e = ds.runout(R, d)
    cuts = [c for c in ds.profile_cuts(L_SHAPE, depth=d, corner_clearance=2.0, **KW) if c.pass_index == 1]
    types = ds.corner_types(L_SHAPE)
    n = len(L_SHAPE)
    for c in cuts:
        L = np.linalg.norm(L_SHAPE[(c.edge + 1) % n] - L_SHAPE[c.edge])
        a = np.append(L_SHAPE[c.edge], TOP)
        t = np.asarray(c.direction)
        u_end = (lowest_point(c, "end") - a) @ t
        u_start = (lowest_point(c, "start") - a) @ t
        if types[(c.edge + 1) % n] == "convex":
            assert u_end >= L - 1e-9                       # lowest point reaches the corner: full depth there
            assert u_end - L == pytest.approx(ds.runout(R, c.depth) + 2.0)
            assert c.overcut_end == pytest.approx(ds.runout(R, c.depth) + 2.0)
        if types[c.edge] == "convex":
            assert u_start <= 1e-9
            assert -u_start == pytest.approx(ds.runout(R, c.depth) + 2.0)
        # depth of the lowest point is d below the top surface
        assert lowest_point(c, "start")[2] == pytest.approx(TOP - c.depth)
    assert e > 0


def test_reflex_corner_uncut_equals_runout_and_never_enters():
    d = 20.0
    cuts = ds.profile_cuts(L_SHAPE, depth=d, **KW)
    n = len(L_SHAPE)
    deepest = [c for c in cuts if c.pass_index == max(x.pass_index for x in cuts)]
    e = ds.runout(R, d)
    by_edge = {c.edge: c for c in deepest}
    # reflex vertex 3 joins edge 2 (end) and edge 3 (start)
    assert by_edge[2].uncut_end == pytest.approx(e, abs=1e-6)
    assert by_edge[3].uncut_start == pytest.approx(e, abs=1e-6)
    assert by_edge[2].overcut_end == 0.0 and by_edge[3].overcut_start == 0.0
    for c in (by_edge[2],):
        a = np.append(L_SHAPE[2], TOP)
        L = np.linalg.norm(L_SHAPE[3] - L_SHAPE[2])
        u_end = (lowest_point(c, "end") - a) @ np.asarray(c.direction)
        assert u_end == pytest.approx(L - e, abs=1e-6)
    w = ds.uncut_wedges(cuts, L_SHAPE)
    assert {(x["edge"], x["end"]) for x in w} == {(2, "end"), (3, "start")}
    assert all(abs(x["length_mm"] - e) < 1e-6 for x in w)
    rep = ds.blade_intrudes(cuts, L_SHAPE, T)
    assert rep.max_intrusion <= 1e-7


def test_stepdown_passes_and_through_depth():
    cuts = ds.profile_cuts(L_SHAPE, **KW)            # through
    depths = sorted({round(c.depth, 9) for c in cuts})
    assert depths[-1] == pytest.approx(T + 1.0)
    assert all(b - a <= 10.0 + 1e-9 for a, b in zip([0.0] + depths, depths))
    assert all(c.through for c in cuts)
    rep = ds.blade_intrudes(cuts, L_SHAPE, T)
    assert rep.max_intrusion <= 1e-7


def test_leads_are_above_the_slab():
    for c in ds.profile_cuts(L_SHAPE, **KW):
        for key in ("lead_in", "lead_out"):
            lp = np.asarray(getattr(c, key))
            g = -np.cross(np.asarray(c.normal), np.asarray(c.direction))
            assert (lp + c.radius * g)[2] > TOP + 1.0     # lowest point is clear of the top surface


def test_refuses_when_blade_too_small():
    small = dict(KW, blade_diameter=100.0, flange_diameter=60.0)       # reach 50 - 30 - 1 = 19 mm < 31 mm
    with pytest.raises(ValueError, match="cannot be cut through"):
        ds.profile_cuts(L_SHAPE, **small)
    with pytest.raises(ValueError, match="reaches at most"):
        ds.profile_cuts(L_SHAPE, depth=25.0, **small)
    assert ds.profile_cuts(L_SHAPE, depth=15.0, **small) is not None
    with pytest.raises(ValueError, match="cannot be cut through"):
        ds.comb_cuts(np.array([[0, 0], [90, 0], [90, 90], [0, 90.0]]), pitch=10, **small)
    with pytest.raises(ValueError, match="counter-clockwise"):
        ds.profile_cuts(L_SHAPE[::-1], depth=10.0, **KW)


def test_bevel_mitre_front_edge():
    for bev in (45.0, -45.0, 30.0):
        betas = [bev, 0, 0, 0, 0, 0]
        cuts = ds.profile_cuts(L_SHAPE, bevel_deg=betas, depth=None, **KW)
        c0 = [c for c in cuts if c.edge == 0]
        assert c0 and all(c.bevel == bev for c in c0)
        cb, sb = math.cos(math.radians(bev)), math.sin(math.radians(bev))
        N = np.asarray(c0[0].normal)
        assert np.allclose(N, [0, -cb, sb], atol=1e-12)           # edge 0 runs along +x, outward is -y
        # kerf face contains the edge line at the top surface
        for c in c0:
            fp = np.asarray(c.start) - 0.5 * KERF * np.asarray(c.normal)
            for p in ([0, 0, TOP], [600, 0, TOP]):
                assert abs((np.array(p) - fp) @ np.asarray(c.normal)) < 1e-9
        # depth is measured along the blade: through depth = t / cos(bev) + 1
        assert max(c.depth for c in c0) == pytest.approx(T / cb + 1.0)
        # the lowest point sits d*cos(bev) below the top and d*sin(bev) outward
        c = c0[-1]
        lp = lowest_point(c, "start") - 0.5 * KERF * N          # lowest point of the part-side face
        assert lp[2] == pytest.approx(TOP - c.depth * cb)
        rep = ds.blade_intrudes(cuts, L_SHAPE, T, bevels=betas)
        assert rep.max_intrusion <= 1e-6, (bev, rep.max_intrusion)
    # the flange limit tightens with bevel: a slab that just cuts through vertically does not at 60 degrees
    tight = dict(KW, blade_diameter=250.0, flange_diameter=100.0)       # reach 125 - 50 - 1 = 74 mm
    assert ds.profile_cuts(L_SHAPE, depth=None, **tight)
    with pytest.raises(ValueError):
        ds.profile_cuts(L_SHAPE, bevel_deg=[60, 0, 0, 0, 0, 0], depth=None, **dict(tight, blade_diameter=190.0))


def _run_random(polys, rng, bevel=False):
    n_cuts = n_skipped = 0
    for P in polys:
        d = float(rng.uniform(5, 28))
        through = bool(rng.random() < 0.3)
        rep = {}
        try:
            cuts = ds.profile_cuts(P, depth=None if through else d, report=rep, **KW)
        except ValueError as ex:
            assert "collinear" in str(ex) or "spike" in str(ex)
            continue
        n_cuts += len(cuts)
        n_skipped += len(rep["skipped"])
        r = ds.blade_intrudes(cuts, P, T, samples=5)
        assert r.max_intrusion <= 1e-6, (P.tolist(), d, r.max_intrusion)
        for c in cuts:
            assert c.overcut_mm >= 0 and c.uncut_mm >= 0
    return n_cuts, n_skipped


def test_random_star_polygons_never_intruded():
    rng = np.random.default_rng(20261113)
    cuts, skipped = _run_random([star(rng) for _ in range(250)], rng)
    assert cuts > 500


def test_random_l_polygons_never_intruded():
    rng = np.random.default_rng(7)
    cuts, skipped = _run_random([l_shape(rng) for _ in range(150)], rng)
    assert cuts > 300


def test_random_bevelled_polygons_never_intruded():
    rng = np.random.default_rng(99)
    done = 0
    for _ in range(40):
        P = l_shape(rng)
        betas = rng.choice([0.0, 0.0, 20.0, -20.0, 45.0, -30.0], size=len(P))
        try:
            cuts = ds.profile_cuts(P, depth=float(rng.uniform(8, 25)), bevel_deg=betas, **KW)
        except ValueError:
            continue
        r = ds.blade_intrudes(cuts, P, T, samples=5, bevels=betas)
        assert r.max_intrusion <= 1e-6, (P.tolist(), betas.tolist(), r.max_intrusion)
        done += 1
    assert done >= 30


def test_intrusion_check_detects_a_bad_cut():
    cuts = ds.profile_cuts(L_SHAPE, depth=20.0, **KW)
    bad = []
    for c in cuts:
        if c.edge == 2 and c.pass_index == max(x.pass_index for x in cuts):
            ext = 0.5 * ds.runout(R, c.depth) * np.asarray(c.direction)      # run the blade on into the corner
            c = ds.Cut(**{**c.__dict__, "end": tuple(np.asarray(c.end) + 2 * ext)})
        bad.append(c)
    assert ds.blade_intrudes(bad, L_SHAPE, T).max_intrusion > 1.0


REGION = np.array([[40, 40], [260, 40], [260, 260], [40, 260]], float)


def _comb_inside(cuts, region, kerf, R, tol=1e-6):
    n = len(region)
    A, B = region, np.roll(region, -1, axis=0)
    for c in cuts:
        t = np.asarray(c.direction)[:2]
        for p in (np.asarray(c.start)[:2], np.asarray(c.end)[:2]):
            for sgn in (-1, 0, 1):
                q = p + sgn * 0.5 * kerf * np.array([t[1], -t[0]])
                e = ds.runout(R, c.depth)
                for ext in (-e, e):
                    qq = q + ext * t
                    sd = ds._sd_polygon(region, qq[None, :])[0]
                    assert sd <= tol, (sd, c.edge)                     # inside the region (or on the wall)


def test_comb_stays_inside_region_minus_half_kerf():
    rep = {}
    cuts = ds.comb_cuts(REGION, depth=12.0, pitch=15.0, report=rep, **KW)
    assert cuts and all(c.kind == "comb" for c in cuts)
    for c in cuts:
        t = np.asarray(c.direction)[:2]
        for p in (np.asarray(c.start)[:2], np.asarray(c.end)[:2]):
            # wall distance of the slot centre line is at least kerf/2
            assert min(p[0] - 40, 260 - p[0], p[1] - 40, 260 - p[1]) >= 0.5 * KERF - 1e-6
        assert c.uncut_start == pytest.approx(ds.runout(R, c.depth), abs=1e-9)
    _comb_inside(cuts, REGION, KERF, R)
    r = ds.blade_intrudes(cuts, REGION, T, solid="outside")
    assert r.max_intrusion <= 1e-6
    # alternate direction (zig-zag)
    firsts = {c.edge: c for c in cuts if c.pass_index == 0}
    assert np.dot(firsts[0].direction, firsts[1].direction) < 0


def test_comb_cross_and_angled_and_pitch():
    cuts = ds.comb_cuts(REGION, depth=12.0, pitch=20.0, angle_deg=0.0, cross_angle_deg=90.0, **KW)
    dirs = {tuple(np.round(np.abs(c.direction), 6)) for c in cuts}
    assert (1.0, 0.0, 0.0) in dirs and (0.0, 1.0, 0.0) in dirs
    r = ds.blade_intrudes(cuts, REGION, T, solid="outside")
    assert r.max_intrusion <= 1e-6
    ys = sorted({round(c.start[1], 6) for c in cuts if abs(c.direction[1]) < 1e-9 and c.pass_index == 0})
    assert np.allclose(np.diff(ys), 20.0)
    ang = ds.comb_cuts(REGION, depth=12.0, pitch=25.0, angle_deg=30.0, **KW)
    assert ang and ds.blade_intrudes(ang, REGION, T, solid="outside").max_intrusion <= 1e-6


def test_comb_concave_region():
    region = L_SHAPE * 0.9 + 20
    cuts = ds.comb_cuts(region, depth=10.0, pitch=18.0, angle_deg=17.0, cross_angle_deg=107.0, **KW)
    assert cuts
    assert ds.blade_intrudes(cuts, region, T, solid="outside").max_intrusion <= 1e-6


def test_json_roundtrip():
    cuts = ds.profile_cuts(L_SHAPE, depth=20.0, **KW) + ds.comb_cuts(REGION, depth=12.0, pitch=20.0, **KW)
    text = ds.cuts_to_json(cuts)
    assert json.loads(text)["format"] == "lithocam.discsaw.v1"
    back = ds.load(text)
    assert back == cuts


# ---------------------------------------------------------------- ruled faces (T7)
from pathlib import Path
from types import SimpleNamespace as NS

RULED = Path(__file__).parent / "data" / "ruled_json"


def hypar_face(c, X=100.0, Z=100.0, n=9):
    """Horizontal rulings z = const of the surface y = x z / c, outward normal +y. The middle ruling (z = 0) has
    the tangent plane turning about it by atan(x / c): spread 2 atan(X / c)."""
    rs = []
    for z in np.linspace(-Z, Z, n):
        nm = np.array([-z / c, 1.0, 0.0])
        rs.append(NS(p0=[-X, -X * z / c, z], p1=[X, X * z / c, z], normal=list(nm / np.linalg.norm(nm)),
                     tool_free=[True, True]))
    return NS(rulings=rs)


def plane_face(n=5, length=300.0, pitch=40.0):
    return NS(rulings=[NS(p0=[0.0, 0.0, -k * pitch], p1=[length, 0.0, -k * pitch], normal=[0.0, 1.0, 0.0],
                          tool_free=[True, True]) for k in range(n)])


BIG = dict(blade_diameter=1000.0, flange_diameter=100.0)        # flange limit 449 mm, never binds in the hypar tests


def test_plane_all_rulings_cuttable_at_full_depth():
    f = plane_face()
    rep = ds.ruled_disc_feasibility(f, blade_diameter=350.0, flange_diameter=100.0, tol=0.1)
    full = ds.max_cut_depth(175.0, 50.0, 1.0, 0.0)
    assert rep.all_cuttable and rep.n_cuttable == 5
    assert rep.max_depth == pytest.approx(full)                      # 124 mm: set by the flange only
    for r in rep.rulings:
        assert r.twist_deg == pytest.approx(0.0, abs=1e-9)
        assert math.isinf(r.curvature_depth)
        assert r.bevel_deg == pytest.approx(0.0, abs=1e-6) and r.tilt_deg == pytest.approx(0.0, abs=1e-6)
    cuts = ds.ruled_disc_cuts(f, blade_diameter=350.0, flange_diameter=100.0, kerf=3.2, stepdown=31.0, tol=0.1)
    assert len(cuts) == 5 * 4 and all(c.kind == "ruling" for c in cuts)
    chk = ds.ruled_blade_intrudes(cuts, f)
    print("plane intrusion", chk.max_intrusion, "excess", chk.max_excess)
    assert chk.max_intrusion <= 1e-9 and chk.max_excess <= 1e-9
    for c in cuts:                                                   # the part-side face holds the ruling line
        N, T = np.asarray(c.normal), np.asarray(c.direction)
        g = np.cross(T, N)
        assert abs(N @ T) < 1e-12 and g[2] == pytest.approx(-1.0)
        low = np.asarray(c.start) + c.radius * g
        assert low[2] == pytest.approx(-40.0 * c.edge - c.depth)     # lowest point depth below the ruling
        assert (np.asarray(c.start) - 0.5 * c.kerf * N)[1] == pytest.approx(0.0, abs=1e-9)   # face plane y = 0


def test_plane_full_depth_request_beyond_flange_refused():
    rep = ds.ruled_disc_feasibility(plane_face(), blade_diameter=350.0, flange_diameter=100.0, tol=0.1, depth=130.0)
    assert not rep.all_cuttable and "flange" in rep.rulings[0].reason


@pytest.mark.parametrize("c", [600.0, 300.0, 150.0])
def test_hypar_depth_falls_with_twist_as_derived(c):
    tol = 0.5
    rep = ds.ruled_disc_feasibility(hypar_face(c), tol=tol, **BIG)
    mid = [r for r in rep.rulings if r.index == 4][0]
    S = 2.0 * math.atan(100.0 / c)                                    # twist spread of the middle ruling
    D = tol / math.tan(S)                                             # curvature depth, intrusion_tol = 0
    print("hypar c", c, "twist deg", mid.twist_deg, "depth", mid.max_depth, "derived", D)
    assert math.radians(mid.twist_deg) == pytest.approx(S, rel=1e-6)
    assert mid.curvature_depth == pytest.approx(D, rel=1e-6)
    assert mid.max_depth == pytest.approx(D, rel=1e-3)                # the sheet check agrees at this depth
    assert mid.cuttable


def test_hypar_depth_decreases_monotonically_and_dies_beyond_90_degrees():
    ds_ = [ds.ruled_disc_feasibility(hypar_face(c), tol=0.5, **BIG).max_depth for c in (1200.0, 600.0, 300.0, 150.0)]
    assert all(a > b for a, b in zip(ds_, ds_[1:]))
    rep = ds.ruled_disc_feasibility(hypar_face(75.0), tol=0.5, **BIG)      # middle spread 106 deg: no depth works
    assert not rep.all_cuttable and not rep.rulings[4].cuttable and rep.rulings[4].max_depth == 0.0


def test_hypar_intrusion_tolerance_raises_depth_at_equal_tol():
    S = 2.0 * math.atan(100.0 / 300.0)
    a = ds.ruled_disc_feasibility(hypar_face(300.0), tol=0.5, **BIG).rulings[4]
    b = ds.ruled_disc_feasibility(hypar_face(300.0), tol=0.5, intrusion_tol=0.5, **BIG).rulings[4]
    assert a.curvature_depth == pytest.approx(0.5 / math.tan(S))
    assert b.curvature_depth == pytest.approx(0.5 / math.tan(S / 2))


def test_hypar_tilt_range_at_shallower_depth():
    c, tol = 300.0, 0.5
    S = 2.0 * math.atan(100.0 / c)
    D = 0.9 * tol / math.tan(S)                                       # 90 percent of the curvature depth
    r = ds.ruled_disc_feasibility(hypar_face(c), tol=tol, depth=D, **BIG).rulings[4]
    width = math.degrees(math.atan(tol / D) - S)
    assert r.cuttable
    assert r.tilt_max_deg - r.tilt_min_deg == pytest.approx(width, rel=1e-6)
    assert r.tilt_deg == pytest.approx(0.5 * (r.tilt_min_deg + r.tilt_max_deg), abs=0.05)
    assert r.tilt_min_deg <= r.tilt_deg <= r.tilt_max_deg


def rotate_cut_about_ruling(c, ang_deg):
    """The same cut with its blade plane turned about the ruling line (a deliberately wrong posture)."""
    from dataclasses import replace
    T, N = np.asarray(c.direction), np.asarray(c.normal)
    g = np.cross(T, N)
    a = math.radians(ang_deg)

    def rot(v):
        return v * math.cos(a) + np.cross(T, v) * math.sin(a) + T * (T @ v) * (1 - math.cos(a))
    base = np.asarray(c.start) + (c.radius - c.depth) * g - 0.5 * c.kerf * N

    def pt(p):
        return tuple(base + rot(np.asarray(p) - base))
    return replace(c, normal=tuple(rot(N)), start=pt(c.start), end=pt(c.end), lead_in=pt(c.lead_in),
                   lead_out=pt(c.lead_out))


@pytest.mark.parametrize("c", [600.0, 300.0])
def test_hypar_generated_passes_do_not_intrude(c):
    f = hypar_face(c)
    rep = {}
    cuts = ds.ruled_disc_cuts(f, kerf=3.2, stepdown=0.5, tol=0.5, report=rep, **BIG)
    assert cuts and not rep["skipped"]
    chk = ds.ruled_blade_intrudes(cuts, f)
    print("hypar", c, "n cuts", len(cuts), "intrusion", chk.max_intrusion, "excess", chk.max_excess)
    assert chk.max_intrusion <= 1e-6
    assert chk.max_excess <= 0.5 + 1e-6
    # control: turn the blades by 3 degrees about their rulings and the check sees the part being cut
    bad = ds.ruled_blade_intrudes([rotate_cut_about_ruling(x, 3.0) for x in cuts], f)
    bad2 = ds.ruled_blade_intrudes([rotate_cut_about_ruling(x, -3.0) for x in cuts], f)
    assert max(bad.max_intrusion, bad2.max_intrusion) > 0.01
    assert max(bad.max_excess, bad2.max_excess) > 0.5


def test_twisted_column_passes():
    f = ds.load_face(RULED / "twisted_column.json", 0)
    rep = {}
    kw = dict(blade_diameter=350.0, flange_diameter=100.0, tol=0.5)
    fe = ds.ruled_disc_feasibility(f, **kw)
    print(fe.summary())
    assert fe.all_cuttable and fe.n_cuttable == 9
    assert 9.0 < fe.min_twist_deg < fe.max_twist_deg < 10.5          # a few degrees of twist per 300 mm ruling
    cuts = ds.ruled_disc_cuts(f, kerf=3.2, stepdown=1.0, report=rep, **kw)
    assert len({c.edge for c in cuts}) == 9 and not rep["skipped"]
    chk = ds.ruled_blade_intrudes(cuts, f)
    print("column intrusion", chk.max_intrusion, "excess", chk.max_excess, "n", len(cuts))
    assert chk.max_intrusion <= 1e-6 and chk.max_excess <= 0.5 + 1e-6
    for c in cuts:                                                   # blade normal is perpendicular to the cut
        assert abs(np.dot(c.normal, c.direction)) < 1e-12
    # a deeper request than the curvature allows is refused per ruling
    deep = ds.ruled_disc_feasibility(f, depth=20.0, **kw)
    assert deep.n_cuttable == 0 and "curvature" in deep.rulings[0].reason
    # json round trip keeps the ruling cuts
    assert ds.load(ds.cuts_to_json(cuts))[0].kind == "ruling"


def test_column_horizontal_faces_refused_vertical_planar_face_cut():
    for idx in (4, 5):                                               # the horizontal end faces: a disc cannot lie flat
        f = ds.load_face(RULED / "twisted_column.json", idx)
        fe = ds.ruled_disc_feasibility(f, blade_diameter=350.0, flange_diameter=100.0, tol=0.1)
        assert fe.n_cuttable == 0 and "lean more than 75" in fe.rulings[0].reason
    f = ds.load_face(RULED / "cladding_panel.json", 0)               # vertical plane, vertical rulings: refused too
    assert ds.ruled_disc_feasibility(f, blade_diameter=350.0, flange_diameter=100.0, tol=0.1).n_cuttable == 0

def test_vertical_rulings_refused():
    f = NS(rulings=[NS(p0=[0, 0, 0], p1=[0, 0, 300], normal=[0, 1, 0], tool_free=[True, True]),
                    NS(p0=[40, 0, 0], p1=[40, 0, 300], normal=[0, 1, 0], tool_free=[True, True])])
    rep = ds.ruled_disc_feasibility(f, blade_diameter=350.0, flange_diameter=100.0, tol=0.1)
    assert rep.n_cuttable == 0 and "vertical" in rep.rulings[0].reason


def test_blocked_end_leaves_uncut_wedge():
    f = plane_face(n=2)
    for r in f.rulings:
        r.tool_free = [False, True]
    cuts = ds.ruled_disc_cuts(f, blade_diameter=350.0, flange_diameter=100.0, kerf=3.2, stepdown=124.0, tol=0.1)
    c = cuts[0]
    assert c.uncut_mm == pytest.approx(ds.runout(175.0, c.depth))     # blocked end: the lowest point stops e(d) short\n    assert c.overcut_mm > 0 and min(c.uncut_start, c.uncut_end) == 0.0
