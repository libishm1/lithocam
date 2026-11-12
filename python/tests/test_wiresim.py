# Copyright 2026 The lithocam authors. SPDX-License-Identifier: Apache-2.0
"""Diamond-wire cutting simulation (planar supporting cuts) against hand calculations (needs OCP)."""
import math
from pathlib import Path

import pytest

wiresim = pytest.importorskip("lithocam.wiresim", reason="OCP (OpenCascade) not installed", exc_type=ImportError)

DATA = Path(__file__).resolve().parent / "data" / "step"


def run(name):
    return wiresim.simulate_step(str(DATA / (name + ".step")))


def check_balance(r):
    assert r.gouge_volume == pytest.approx(0.0, abs=1e-6 * r.target_volume)
    assert r.balance == pytest.approx(0.0, abs=1e-6 * r.stock_volume)
    assert r.blank_volume - r.to_mill_volume == pytest.approx(r.target_volume, rel=1e-9)


def test_cladding_panel_leaves_holes_and_kerf_slots_for_milling():
    r = run("cladding_panel")
    check_balance(r)
    holes = 4 * math.pi * 5 ** 2 * 20
    slots = 2 * 100 * 15 * 3
    assert r.to_mill_volume == pytest.approx(holes + slots, rel=1e-6)
    assert len(r.cuts) == 6 and not r.skipped


def test_window_sill_leaves_groove_and_rebate():
    r = run("window_sill")
    check_balance(r)
    assert r.to_mill_volume == pytest.approx(10 * 10 * 1200 + 20 * 15 * 1200, rel=1e-6)
    assert any("internal or concave" in s for _, s in r.skipped)


def test_stair_tread_leaves_grooves_and_nosing_corner():
    r = run("stair_tread")
    check_balance(r)
    grooves = 3 * 5 * 3 * 900
    nosing = (10 * 10 - math.pi * 10 ** 2 / 4) * 1000
    assert r.to_mill_volume == pytest.approx(grooves + nosing, rel=1e-6)


def test_kerf_is_the_wire_slab():
    """Each cut removes a positive kerf slab and a positive waste piece, and the volumes balance."""
    r = run("cladding_panel")
    first = r.cuts[0]
    assert first.kerf_volume > 0 and first.waste_volume > 0
    assert r.kerf_volume + r.waste_volume + r.blank_volume == pytest.approx(r.stock_volume, rel=1e-9)
