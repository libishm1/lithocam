# Copyright 2026 The lithocam authors. SPDX-License-Identifier: Apache-2.0
"""Cutting data: every entry is sourced and well formed; conversions; suggestions are the conservative end of the
sourced ranges, capped by the spindle, with the chip-load formula and depth rule for wood."""
import math

import pytest

from lithocam import cutting_data as cd


def test_conversions_and_depth_rule():
    assert abs(cd.rpm_from_surface_speed(25, 400) - 25 * 60000 / (math.pi * 400)) < 1e-9
    assert abs(cd.surface_speed(cd.rpm_from_surface_speed(31.4, 120), 120) - 31.4) < 1e-9
    assert cd.chip_load_feed(18000, 2, 0.229) == pytest.approx(8244.0)
    assert cd.depth_factor(1.0) == 1.0 and cd.depth_factor(2.0) == 0.75 and cd.depth_factor(3.0) == 0.5
    assert cd.depth_factor(5.0) == 0.5


def test_every_entry_is_sourced_and_well_formed():
    for e in cd.TABLE:
        assert e.material in cd.MATERIALS, e
        assert e.source and e.source_type and e.confidence in ("page", "snippet"), e
        assert 0 <= e.lo <= e.hi, e
        assert e.unit


def test_tool_kinds_exist():
    try:
        from lithocam.studio.core import lc
    except ImportError:
        pytest.skip("_lithocam not built")
    names = {lc.tool_kind_name(k) for k in lc.all_tool_kinds()}
    assert {e.tool for e in cd.TABLE} <= names


def test_suggestions():
    try:
        from lithocam.studio.core import lc
    except ImportError:
        pytest.skip("_lithocam not built")
    tools = {t.id: t for t in list(lc.stone_tool_library().tools) + list(lc.wood_tool_library().tools)}
    # 1/4 in spiral up-cut in hardwood: 18000 rpm basis, Rennie 6.35 mm chip load lower bound, 2 flutes
    s = cd.suggest(tools["W2"], "hardwood")
    assert s.rpm == 18000 and s.feed == pytest.approx(18000 * 2 * 0.229)
    # capped by a 12000 rpm spindle: the feed follows the capped rpm
    s = cd.suggest(tools["W2"], "hardwood", max_rpm=12000)
    assert s.rpm == 12000 and s.feed == pytest.approx(12000 * 2 * 0.229)
    # depth 2 x D: chip load x 0.75
    s = cd.suggest(tools["W2"], "hardwood", depth_mm=2 * tools["W2"].diameter)
    assert s.feed == pytest.approx(18000 * 2 * 0.229 * 0.75)
    # 400 mm bridge-saw blade in granite: rpm from the lower surface speed (25 m/s), feed 200 mm/min, water
    s = cd.suggest(tools["S15"], "granite")
    assert s.rpm == pytest.approx(cd.rpm_from_surface_speed(25, 400)) and s.feed == 200 and s.water == 22.7
    # nothing sourced: zeros, never invented
    s = cd.suggest(tools["S24"], "granite")
    assert s.rpm == 0 and s.feed == 0 and s.text() == "no sourced data"
