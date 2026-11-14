# Copyright 2026 The lithocam authors. SPDX-License-Identifier: Apache-2.0
"""Sourced constants of the 2026-11-14 review, and exact robot matching for joint stiffness."""
import pytest

from lithocam import materials_mechanics as mm

process = pytest.importorskip("lithocam.studio.process")


def test_kr120_gets_its_proxy_row_not_the_cobot_row():
    c = process.sourced_constants("granite", "kr120")
    assert c["joint_stiffness"].value == pytest.approx(1.01e5)
    assert "KR210" in c["joint_stiffness"].text


def test_robot_names_never_match_by_substring():
    assert mm.robot_stiffness_rows("ur10") == []           # the cobot row once named "UR10" in its text
    assert mm.robot_stiffness_rows("ur10e") == []          # no sourced UR10e set exists
    assert "joint_stiffness" not in process.sourced_constants("granite", "ur10e")
    for key, row in mm.ROBOT_STIFFNESS_ROW.items():
        rows = mm.robot_stiffness_rows(key)
        assert len(rows) == 1 and rows[0]["material"] == row and rows[0]["low"] > 0


def test_granite_wear_constants_are_sourced():
    c = process.sourced_constants("granite", "kr120")
    assert c["g_ratio"].value == pytest.approx(5032)             # low end: the conservative tool life
    assert c["wear_per_area"].value == pytest.approx(0.013462)   # high end
    assert "doi.org" in c["g_ratio"].text and "doi.org" in c["wear_per_area"].text


def test_every_new_row_names_a_source():
    for r in mm.TABLE:
        if r["low"] is not None:
            assert r["source"], r["material"]
