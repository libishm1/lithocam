# Copyright 2026 The lithocam authors. SPDX-License-Identifier: Apache-2.0
"""Mesh-based wire simulation (Manifold) against the exact OCP simulation and hand calculations."""
import math
from pathlib import Path

import pytest

meshsim = pytest.importorskip("lithocam.meshsim", reason="manifold3d or OCP not installed", exc_type=ImportError)

DATA = Path(__file__).resolve().parent / "data"


def test_planar_cuts_match_the_exact_simulation():
    r = meshsim.simulate_step(str(DATA / "step" / "cladding_panel.step"))
    # exact (OCP) values: blank 600 x 400 x 30, kerf 4,379,200, waste 8,132,800 (margin 20, wire 7)
    assert r.blank_volume == pytest.approx(600 * 400 * 30, rel=1e-9)
    assert r.kerf_volume == pytest.approx(4379200, rel=1e-9)
    assert r.waste_volume == pytest.approx(8132800, rel=1e-9)
    holes, slots = 4 * math.pi * 25 * 20, 2 * 100 * 15 * 3
    assert r.to_mill_volume == pytest.approx(holes + slots, rel=2e-3)     # holes tessellated
    assert r.gouge_volume < 1.0 and abs(r.balance) < 1e-6 * r.stock_volume


def test_ruled_cuts_finish_the_twisted_column_with_a_small_gouge():
    r = meshsim.simulate_step(str(DATA / "step_ruled" / "twisted_column.step"))
    assert sum(1 for c in r.cuts if c[1] == "ruled") == 4
    assert r.to_mill_volume < 1e-4 * r.target_volume               # nothing left to mill
    assert r.gouge_volume < 6e-3 * r.target_volume                 # hull approximation: 0.48% measured, 96 rulings
    assert abs(r.balance) < 1e-6 * r.stock_volume
