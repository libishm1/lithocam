# Copyright 2026 The lithocam authors. SPDX-License-Identifier: Apache-2.0
"""Polishing: pad axis = surface normal, coverage, uniform removal, linear Preston scaling, grit sequences."""
import math
from types import SimpleNamespace as NS

import numpy as np
import pytest

from lithocam import polish as pl


def grid_patch(f, nx, ny, x0, x1, y0, y1):
    """Height field z = f(x, y) as triangles (n, 3, 3), counter-clockwise from +z."""
    xs, ys = np.linspace(x0, x1, nx), np.linspace(y0, y1, ny)
    X, Y = np.meshgrid(xs, ys)
    Z = f(X, Y)
    P = np.stack([X, Y, Z], -1)
    a, b, c, d = P[:-1, :-1], P[:-1, 1:], P[1:, 1:], P[1:, :-1]
    return np.concatenate([np.stack([a, b, c], -2).reshape(-1, 3, 3), np.stack([a, c, d], -2).reshape(-1, 3, 3)])


R_SPH = 100.0
SPHERE = grid_patch(lambda x, y: np.sqrt(R_SPH ** 2 - x ** 2 - y ** 2), 161, 161, -50, 50, -50, 50)   # cap, polar angle <= 45 deg
R_CYL = 100.0
CYL = grid_patch(lambda x, y: np.sqrt(R_CYL ** 2 - x ** 2), 161, 21, -70, 70, -60, 60)               # axis along y
FLAT = grid_patch(lambda x, y: 0 * x + 5.0, 3, 3, 0, 120, 0, 100)
PAD = NS(kind=NS(name="PolishingPad"), diameter=40.0)
STOCK = {"zmax": 200.0}
RP = 20.0


def contact_moves(moves):
    return [m for m in moves if m[0] in ("plunge", "feed")]


def test_axis_is_surface_normal_on_sphere():
    mv = pl.polish(SPHERE, None, PAD, STOCK, overlap=0.6)
    cm = contact_moves(mv)
    assert len(cm) > 100
    P = np.array([m[1] for m in cm])
    A = np.array([m[2] for m in cm])
    true_n = P / np.linalg.norm(P, axis=1, keepdims=True)
    err = np.degrees(np.arccos(np.clip(np.sum(A * true_n, axis=1), -1, 1)))
    print("sphere axis error max deg", err.max())
    assert err.max() < 1.0 and np.median(err) < 0.3
    assert np.allclose(np.linalg.norm(A, axis=1), 1.0)
    # contact points lie on the sphere
    assert np.abs(np.linalg.norm(P, axis=1) - R_SPH).max() < 0.05


def test_axis_is_surface_normal_on_cylinder():
    mv = pl.polish(CYL, None, PAD, STOCK, overlap=0.6, pattern="zigzag")
    cm = contact_moves(mv)
    P = np.array([m[1] for m in cm])
    A = np.array([m[2] for m in cm])
    true_n = np.stack([P[:, 0], 0 * P[:, 0], P[:, 2]], 1)
    true_n /= np.linalg.norm(true_n, axis=1, keepdims=True)
    err = np.degrees(np.arccos(np.clip(np.sum(A * true_n, axis=1), -1, 1)))
    print("cylinder axis error max deg", err.max())
    assert err.max() < 1.0 and np.median(err) < 0.3
    # the axis never has a component along the cylinder axis y
    assert np.abs(A[:, 1]).max() < 1e-6


@pytest.mark.parametrize("surf,kw", [(SPHERE, {}), (CYL, {}), (SPHERE, {"pattern": "spiral"}), (FLAT, {})])
def test_coverage_every_sample_within_pad_radius(surf, kw):
    mv = pl.polish(surf, None, PAD, STOCK, overlap=0.6, **kw)
    S, _ = pl.surface_samples(surf, 4000, seed=3)
    cov = pl.coverage(mv, S, RP)
    print("coverage max distance / Rp", round(cov, 3), kw)
    assert cov <= 1.0


def test_stepover_from_overlap():
    assert pl.stepover_from_overlap(20.0, 0.25) == pytest.approx(30.0)
    with pytest.raises(ValueError):
        pl.stepover_from_overlap(20.0, 1.0)


def test_move_contract_and_attributes():
    mv = pl.polish(FLAT, None, PAD, STOCK, pressure=0.02, rpm=1500, feed=600, grit="200", water="flood")
    assert {m[0] for m in mv} == {"rapid", "plunge", "feed"}
    pm = [m for m in mv if m[0] == "plunge"][0]
    assert pm[3]["force_n"] == pytest.approx(0.02 * math.pi * RP ** 2)
    assert pm[3]["grit"] == "200" and pm[3]["rpm"] == 1500
    assert all(len(m) == 3 for m in mv if m[0] == "rapid")


def test_wrong_tool_kind_refused():
    with pytest.raises(ValueError):
        pl.polish(FLAT, None, NS(kind=NS(name="BallEndMill"), diameter=10.0), STOCK)


def test_region_box_limits_the_path():
    mv = pl.polish(FLAT, None, PAD, STOCK, overlap=0.6, region=(0, 40, 0, 100))
    P = np.array([m[1] for m in contact_moves(mv)])
    assert P[:, 0].max() <= 40.0 + 1e-6


# ---------------------------------------------------------------- removal model
def interior(S, margin, box):
    x0, x1, y0, y1 = box
    return (S[:, 0] > x0 + margin) & (S[:, 0] < x1 - margin) & (S[:, 1] > y0 + margin) & (S[:, 1] < y1 - margin)


def ripple_of(d):
    return (d.max() - d.min()) / d.mean()


def test_uniform_on_flat_within_tolerance():
    rpm, feed, kp, p, tol = 1500.0, 1200.0, 1e-6, 0.02, 0.05
    s, rp_pred = pl.choose_stepover(RP, rpm, feed, tol)
    print("chosen stepover", s, "predicted ripple", rp_pred, "overlap", 1 - s / (2 * RP))
    assert rp_pred <= tol and s <= 2 * RP
    mv = pl.polish(FLAT, None, PAD, STOCK, stepover=s, step=1.0, pressure=p, rpm=rpm, feed=feed)
    rng = np.random.default_rng(1)
    S = np.column_stack([rng.uniform(0, 120, 1500), rng.uniform(0, 100, 1500), np.full(1500, 5.0)])
    S = S[interior(S, RP + 1.0, (0, 120, 0, 100))]
    d = pl.predict_removal(mv, S, pad_radius=RP, k_p=kp, pressure=p, rpm=rpm, feed=feed)
    r = ripple_of(d)
    print("flat measured ripple", r, "n", len(S))
    assert r <= tol + 0.02          # discretisation of the lane edge and of the path add a little to the 1D value
    # a wider stepover is visibly less uniform
    mv2 = pl.polish(FLAT, None, PAD, STOCK, stepover=2 * RP, step=1.0, pressure=p, rpm=rpm, feed=feed)
    d2 = pl.predict_removal(mv2, S, pad_radius=RP, k_p=kp, pressure=p, rpm=rpm, feed=feed)
    assert ripple_of(d2) > 3 * r


def test_uniform_on_curved_patch_within_tolerance():
    rpm, feed, kp, p, tol = 1500.0, 1200.0, 1e-6, 0.02, 0.05
    s, _ = pl.choose_stepover(RP, rpm, feed, tol)
    # a gentle curved patch: sphere of radius 600 over 120 x 100 (slope < 10 degrees)
    patch = grid_patch(lambda x, y: np.sqrt(600.0 ** 2 - (x - 60) ** 2 - (y - 50) ** 2), 41, 41, 0, 120, 0, 100)
    s_plan = s * 0.99               # plan pitch shrinks with the slope: surface pitch = plan / cos(slope)
    mv = pl.polish(patch, None, PAD, STOCK, stepover=s_plan, step=1.0, pressure=p, rpm=rpm, feed=feed)
    S, _ = pl.surface_samples(patch, 6000, seed=5)
    S = S[interior(S, RP + 2.0, (0, 120, 0, 100))]
    d = pl.predict_removal(mv, S, pad_radius=RP, k_p=kp, pressure=p, rpm=rpm, feed=feed)
    r = ripple_of(d)
    print("curved measured ripple", r, "n", len(S))
    assert r <= tol + 0.03


def test_mean_depth_formula_matches_map():
    rpm, kp, p = 1500.0, 1e-6, 0.02
    plan = pl.plan_uniform(RP, rpm, kp, p, depth=0.002, tol=0.05)
    print("plan", plan)
    mv = pl.polish(FLAT, None, PAD, STOCK, stepover=plan["stepover"], step=1.0, pressure=p, rpm=rpm,
                   feed=plan["feed_mm_min"])
    rng = np.random.default_rng(2)
    S = np.column_stack([rng.uniform(0, 120, 1200), rng.uniform(0, 100, 1200), np.full(1200, 5.0)])
    S = S[interior(S, RP + 1.0, (0, 120, 0, 100))]
    d = pl.predict_removal(mv, S, pad_radius=RP, k_p=kp, pressure=p, rpm=rpm, feed=plan["feed_mm_min"])
    print("mean depth", d.mean(), "target 0.002")
    assert d.mean() == pytest.approx(0.002, rel=0.05)


def test_preston_scales_linearly():
    mv = pl.polish(FLAT, None, PAD, STOCK, overlap=0.5, step=2.0)
    S, _ = pl.surface_samples(FLAT, 300, seed=9)
    base = dict(pad_radius=RP, k_p=1e-6, pressure=0.02, rel_speed=500.0, feed=900.0)
    d0 = pl.predict_removal(mv, S, **base)
    assert d0.max() > 0
    d_k = pl.predict_removal(mv, S, **{**base, "k_p": 3e-6})
    d_p = pl.predict_removal(mv, S, **{**base, "pressure": 0.06})
    d_v = pl.predict_removal(mv, S, **{**base, "rel_speed": 1500.0})
    d_t = pl.predict_removal(mv, S, **{**base, "feed": 300.0})        # a third of the feed = three times the dwell
    for d in (d_k, d_p, d_v, d_t):
        assert np.allclose(d, 3.0 * d0, rtol=1e-9, atol=1e-15)
    dw0 = pl.dwell_map(mv, S, pad_radius=RP, feed=900.0)
    dw1 = pl.dwell_map(mv, S, pad_radius=RP, feed=300.0)
    assert np.allclose(dw1, 3.0 * dw0)
    # with a constant relative speed the removal is exactly k_p * p * v * dwell
    assert np.allclose(d0, 1e-6 * 0.02 * 500.0 * dw0)


def test_dwell_of_a_single_straight_pass():
    # one pass along x through the centre of a point cloud on a line across the path: dwell = chord / feed
    mv = [("rapid", np.array([-100.0, 0, 50]), np.array([0, 0, 1.0])),
          ("plunge", np.array([-100.0, 0, 0]), np.array([0, 0, 1.0]), {}),
          ("feed", np.array([100.0, 0, 0]), np.array([0, 0, 1.0]), {})]
    S = np.array([[0.0, y, 0.0] for y in (0.0, 10.0, 19.0)])
    dw = pl.dwell_map(mv, S, pad_radius=RP, feed=600.0, ds=0.2)
    chord = 2 * np.sqrt(RP ** 2 - S[:, 1] ** 2)
    assert np.allclose(dw, chord / 10.0, rtol=2e-2)


# ---------------------------------------------------------------- grit sequences
def test_granite_sourced_bands_and_rpm():
    bands = pl.sourced_bands("granite")
    assert [b["label"] for b in bands] == ["50-100", "200-400", "800-1500", "3000", "buff"]
    assert (bands[0]["rpm_lo"], bands[0]["rpm_hi"]) == (1500.0, 2000.0)
    assert (bands[-1]["rpm_lo"], bands[-1]["rpm_hi"]) == (3000.0, 3500.0)
    seq = pl.grit_sequence("granite", feed=600.0, water="flood")
    assert [s.grit for s in seq] == ["50-100", "200-400", "800-1500", "3000", "buff"]
    assert seq[0].rpm == 1750.0 and seq[0].source.startswith("https://slabwise")


def test_user_grits_pick_band_rpm_else_refuse():
    seq = pl.grit_sequence("granite", grits=[100, 400, 1500], feed={"100": 500, "400": 600, "1500": 700},
                           water="flood", rpm_pick=0.0)
    assert [s.rpm for s in seq] == [1500.0, 2000.0, 2500.0]
    assert [s.feed for s in seq] == [500.0, 600.0, 700.0]
    with pytest.raises(ValueError):
        pl.grit_sequence("marble", feed=500, water="flood")                       # no sourced sequence
    with pytest.raises(ValueError):
        pl.grit_sequence("marble", grits=[100, 400], feed=500, water="flood")     # no sourced rpm
    with pytest.raises(ValueError):
        pl.grit_sequence("granite", grits=[100], water="flood")                   # feed missing
    seq = pl.grit_sequence("wood", grits=[120, 240], feed=300, water="none", rpm=900)
    assert [s.rpm for s in seq] == [900.0, 900.0]


def test_polish_steps_one_pass_per_grit_with_tool_change():
    seq = pl.grit_sequence("granite", grits=[100, 400], feed=600.0, water="flood")
    steps = pl.polish_steps(FLAT, None, PAD, STOCK, steps=seq, pressure=0.02, overlap=0.6, cross=True)
    assert [s["grit"] for s in steps] == ["100", "400"]
    assert all(s["tool_change"] for s in steps)
    assert steps[0]["moves"][2][3]["grit"] == "100" and steps[1]["moves"][2][3]["grit"] == "400"
    # second grit runs across the first
    d0 = np.array(steps[0]["moves"][3][1]) - np.array(steps[0]["moves"][2][1])
    d1 = np.array(steps[1]["moves"][3][1]) - np.array(steps[1]["moves"][2][1])
    assert abs(d0[:2] @ d1[:2]) < 1e-6 * np.linalg.norm(d0) * np.linalg.norm(d1) + 1e-9
    S, _ = pl.surface_samples(FLAT, 200, seed=1)
    by = pl.predict_by_grit(steps, S, pad_radius=RP, k_p={"100": 2e-6, "400": 1e-6})
    assert set(by) == {"100", "400"} and by["100"].max() > 0
    with pytest.raises(ValueError):
        pl.polish_steps(FLAT, None, PAD, STOCK, steps=seq, pressure=None)
