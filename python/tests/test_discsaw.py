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
