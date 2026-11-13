# Copyright 2026 The lithocam authors. SPDX-License-Identifier: Apache-2.0
"""Unified material model, engagement-based feed limit and wear model. Synthetic constants are given by the test
(never a physical claim); sourced rows are only checked for presence, source and units."""
import math
from types import SimpleNamespace as NS

import numpy as np
import pytest

from lithocam import cutting_data as cd
from lithocam import interaction as ia
from lithocam import materials as mt
from lithocam import materials_mechanics as mm

TOOL = NS(kind="flat_end_mill", diameter=10.0, flutes=2)
SPINDLE = {"power_w": 1000.0, "max_rpm": 24000.0}


# ------------------------------------------------------------------ M1

def test_get_and_aliases():
    assert mt.get("Granite").name == "granite"
    assert mt.get("white oak").name == "oak"
    assert mt.get("quartz engineered").name == "quartz_engineered"
    assert mt.get("MDF").family == "wood"
    with pytest.raises(KeyError) as e:
        mt.get("unobtainium")
    assert "available" in str(e.value)
    for n in ("slate", "basalt"):
        if n not in mt.available():
            with pytest.raises(KeyError):
                mt.get(n)


def test_available_has_rows():
    names = mt.available()
    assert {"granite", "marble", "oak", "mdf"} <= set(names)
    for n in names:
        m = mt.get(n)
        assert m.cutting or any(r["low"] is not None for rs in m.mechanical.values() for r in rs)


def test_value_ends_and_source_row():
    v, row = mt.value("granite", "specific_cutting_energy")        # conservative = high end
    rows = [r for r in mm.TABLE if r["material"] == "granite" and r["quantity"] == "specific_cutting_energy"
            and r["low"] is not None]
    assert v == max(r["high"] for r in rows)
    assert row["source"] and row["unit"] == "J/mm3" and row["table"] == "materials_mechanics"
    assert mt.value("granite", "specific_cutting_energy", "low")[0] == min(r["low"] for r in rows)
    assert mt.value("oak", "youngs_modulus")[0] == min(r["low"] for r in mm.lookup("oak", "youngs_modulus")
                                                       if r["low"] is not None)
    with pytest.raises(ValueError):
        mt.value("granite", "specific_cutting_energy", "mid")


def test_value_missing_is_none_and_cutting_needs_tool():
    assert mt.value("granite", "density") is None                 # a row records that nothing was found
    assert mt.value("mdf", "youngs_modulus") is None
    assert mt.value("nonsense", "rpm", tool="flat_end_mill") is None
    assert mt.value("mdf", "chip_load") is None                   # no tool given
    v, row = mt.value("mdf", "chip_load", tool="flat_end_mill", diameter_mm=10)
    assert row["table"] == "cutting_data" and row["unit"] == "mm/tooth" and row["source"]
    e = cd.pick("mdf", "flat_end_mill", "chip_load", 10)
    assert v == e.lo


def test_units_do_not_mix():
    # the Carrara density rows are g/cm3 and kg/m3; only the preferred unit is used
    got = mt.value("marble", "density")
    assert got is None or got[1]["unit"] == "kg/m3"


def test_report_statuses():
    tab = {(t["material"], t["quantity"]): t["status"] for t in mt.report_table()}
    assert tab[("granite", "specific_cutting_energy")] == "sourced"
    assert tab[("granite", "density")] == "missing"
    assert tab[("marble", "youngs_modulus")] == "snippet"
    assert tab[("granite", "g_ratio")] == "missing"
    txt = mt.report(["granite", "oak"])
    assert "granite (stone)" in txt and "oak (wood)" in txt and "missing:" in txt


# ------------------------------------------------------------------ M2

def test_power_limit_scales_as_p_over_u_ae_ap():
    c = {"specific_energy": 2.0}
    f = mt.feed_limit(TOOL, None, (2.0, 5.0), 10000, SPINDLE, constants=c)
    assert f.binding == "power"
    assert f.feed == pytest.approx(60.0 * 1000.0 / (2.0 * 2.0 * 5.0))
    f2 = mt.feed_limit(TOOL, None, (4.0, 5.0), 10000, SPINDLE, constants=c)
    assert f2.feed == pytest.approx(f.feed / 2)
    f3 = mt.feed_limit(TOOL, None, (2.0, 5.0), 10000, {"power_w": 2000.0}, constants=c)
    assert f3.feed == pytest.approx(2 * f.feed)
    f4 = mt.feed_limit(TOOL, None, (2.0, 5.0), 10000, SPINDLE, constants={"specific_energy": 4.0})
    assert f4.feed == pytest.approx(f.feed / 2)
    assert "rated power" in f.candidates[0].note


def test_power_fraction_derates():
    c = {"specific_energy": 2.0}
    a = mt.feed_limit(TOOL, None, (2.0, 5.0), 10000, SPINDLE, constants=c)
    b = mt.feed_limit(TOOL, None, (2.0, 5.0), 10000, SPINDLE, constants=c, power_fraction=0.5)
    assert b.feed == pytest.approx(a.feed / 2)


def _defl_setup(limit_mm):
    geo = dict(shank_diameter_mm=10.0, shank_length_mm=30.0, flute_length_mm=20.0)
    const = {"specific_energy": 0.01, "radial_ratio": 0.5, "tool_E": 600.0, "flute_factor": 0.8}
    return geo, const, {"tool_deflection_mm": limit_mm}


def test_tool_deflection_limit_matches_interaction_functions():
    geo, const, lim = _defl_setup(0.05)
    big = {"power_w": 1e9}
    f = mt.feed_limit(TOOL, None, (2.0, 5.0), 10000, big, limits=lim, constants=const, geometry=geo)
    c = [x for x in f.candidates if x.name == "tool_deflection"][0]
    vf = c.feed
    vc = math.pi * 10.0 * 10000 / 60000.0
    Ft = 0.01 * 10.0 * vf / 60.0 / vc
    F = Ft * math.sqrt(1 + 0.25)
    d = ia.stepped_cantilever_tip_deflection(F, 600e3, 10.0, 30.0, 0.8, 20.0, 10.0)
    assert d == pytest.approx(0.05)
    # deflection is linear in the feed: halving the limit halves the feed
    f2 = mt.feed_limit(TOOL, None, (2.0, 5.0), 10000, big, limits={"tool_deflection_mm": 0.025}, constants=const,
                       geometry=geo)
    assert [x for x in f2.candidates if x.name == "tool_deflection"][0].feed == pytest.approx(vf / 2)


def _robot():
    J = np.zeros((6, 3))
    J[:3, 0] = [0.8, 0, 0]
    J[:3, 1] = [0, 0.5, 0]
    J[:3, 2] = [0, 0, 0.3]
    return {"J": J, "joint_stiffness": [1e5, 2e5, 4e5]}


def test_robot_deflection_bound_and_directions():
    const = {"specific_energy": 0.01, "radial_ratio": 0.5, "axial_ratio": 0.2}
    lim = {"robot_deflection_mm": 0.1}
    big = {"power_w": 1e9}
    f = mt.feed_limit(TOOL, None, (2.0, 5.0), 10000, big, robot=_robot(), limits=lim, constants=const)
    c = [x for x in f.candidates if x.name == "robot_deflection"][0]
    vc = math.pi * 10.0 * 10000 / 60000.0
    Ft1 = 0.01 * 10.0 / 60.0 / vc
    C = ia.cartesian_compliance(_robot()["J"], _robot()["joint_stiffness"])[:3, :3]
    lam = np.linalg.eigvalsh(C).max()
    d1 = lam * Ft1 * math.sqrt(1 + 0.25 + 0.04) * 1000
    assert c.feed == pytest.approx(0.1 / d1)
    assert "bound" in c.note
    # given directions can only be less severe than the bound
    r = _robot()
    r["force_dirs"] = ([1, 0, 0], [0, 1, 0], [0, 0, 1])
    g = mt.feed_limit(TOOL, None, (2.0, 5.0), 10000, big, robot=r, limits=lim, constants=const)
    cg = [x for x in g.candidates if x.name == "robot_deflection"][0]
    assert cg.feed >= c.feed - 1e-9


def test_binding_constraint_switches():
    geo, const, lim = _defl_setup(0.05)
    const = dict(const, axial_ratio=0.2, chip_load=0.1)
    rob = _robot()
    lim = dict(lim, robot_deflection_mm=0.05)
    cand = {}

    def names(power, **kw):
        f = mt.feed_limit(TOOL, None, (2.0, 5.0), 10000, {"power_w": power}, robot=rob, limits=lim,
                          constants=const, geometry=geo, **kw)
        return f, {c.name: c.feed for c in f.candidates}
    f, d = names(1.0)                      # tiny spindle: power binds
    assert f.binding == "power" and set(d) == {"power", "chip_load", "tool_deflection", "robot_deflection"}
    f, d = names(1e12)                     # huge spindle: power never binds
    assert f.binding in ("tool_deflection", "robot_deflection", "chip_load")
    assert f.feed == min(d.values()) and f.binding == min(d, key=d.get)
    # chip load binds when its value is small
    f = mt.feed_limit(TOOL, None, (2.0, 5.0), 10000, {"power_w": 1e12}, robot=rob, limits=lim,
                      constants=dict(const, chip_load=1e-6), geometry=geo)
    assert f.binding == "chip_load" and f.feed == pytest.approx(10000 * 2 * 1e-6 * cd.depth_factor(0.5))
    # deflection binds when the limit is tight and chip load is large
    tight = {"tool_deflection_mm": 1e-6, "robot_deflection_mm": 10.0}
    f = mt.feed_limit(TOOL, None, (2.0, 5.0), 10000, {"power_w": 1e12}, robot=rob, limits=tight,
                      constants=dict(const, chip_load=100.0), geometry=geo)
    assert f.binding == "tool_deflection"
    tight = {"tool_deflection_mm": 10.0, "robot_deflection_mm": 1e-6}
    f = mt.feed_limit(TOOL, None, (2.0, 5.0), 10000, {"power_w": 1e12}, robot=rob, limits=tight,
                      constants=dict(const, chip_load=100.0), geometry=geo)
    assert f.binding == "robot_deflection"


def test_chip_load_wood_from_sourced_row_and_depth_factor():
    f = mt.feed_limit(TOOL, "hardwood", (3.0, 10.0), 18000, SPINDLE, constants={"specific_energy": 1e-9})
    c = [x for x in f.candidates if x.name == "chip_load"][0]
    e = cd.pick("hardwood", "flat_end_mill", "chip_load", 10)
    assert c.feed == pytest.approx(cd.chip_load_feed(18000, 2, e.lo) * cd.depth_factor(1.0))
    assert e.source in c.source
    f2 = mt.feed_limit(TOOL, "hardwood", (3.0, 20.0), 18000, SPINDLE, constants={"specific_energy": 1e-9})
    c2 = [x for x in f2.candidates if x.name == "chip_load"][0]
    assert c2.feed == pytest.approx(c.feed * 0.75)


def test_sourced_energy_used_for_stone_and_marked():
    f = mt.feed_limit(TOOL, "granite", (2.0, 5.0), 10000, SPINDLE)
    u, row = mt.value("granite", "specific_cutting_energy")
    p = [c for c in f.candidates if c.name == "power"][0]
    assert p.feed == pytest.approx(60 * 1000 / (u * 10.0))
    assert row["source"] in p.source


def test_wood_uses_specific_cutting_force_when_no_energy_row():
    f = mt.feed_limit(TOOL, "hardwood", (3.0, 10.0), 18000, SPINDLE)
    u, row = mt.value("hardwood", "specific_cutting_force")
    p = [c for c in f.candidates if c.name == "power"][0]
    assert p.feed == pytest.approx(60 * 1000 / (u * 1e-3 * 30.0))
    assert "N/mm2 to J/mm3" in p.source


def test_missing_constants_skip_and_say_so():
    f = mt.feed_limit(TOOL, "marble", (2.0, 5.0), 10000, SPINDLE)    # no marble energy row
    assert f.feed is not None and f.binding == "sourced_feed"
    sk = dict(f.skipped)
    assert "specific cutting energy" in sk["power"]
    assert "tool_E" in sk["tool_deflection"] and "limits['tool_deflection_mm']" in sk["tool_deflection"]
    assert "robot['J']" in sk["robot_deflection"]
    bare = mt.feed_limit(TOOL, None, (2.0, 5.0), 10000, None)
    assert bare.feed is None and bare.binding == "none"
    assert "no limit" in bare.badge(1000.0)
    assert "spindle power_w" in dict(bare.skipped)["power"]


def test_refusal_of_sourced_row():
    f = mt.feed_limit(TOOL, "granite", (2.0, 5.0), 10000, SPINDLE, constants={"specific_energy": None})
    assert "power" in dict(f.skipped)
    assert all(c.name != "power" for c in f.candidates)


def test_unknown_material_warns():
    f = mt.feed_limit(TOOL, "unobtainium", (2.0, 5.0), 10000, SPINDLE, constants={"specific_energy": 1.0})
    assert any("no sourced row" in w for w in f.warnings) and f.binding == "power"


def test_rpm_above_spindle_warns_and_bad_rpm_raises():
    f = mt.feed_limit(TOOL, None, (2.0, 5.0), 30000, SPINDLE, constants={"specific_energy": 1.0})
    assert any("exceeds the spindle maximum" in w for w in f.warnings)
    with pytest.raises(ValueError):
        mt.feed_limit(TOOL, None, (2.0, 5.0), 0, SPINDLE)


def test_badge():
    f = mt.feed_limit(TOOL, None, (2.0, 5.0), 10000, SPINDLE, constants={"specific_energy": 2.0})
    assert f.badge(f.feed / 2).startswith("ok")
    assert "over the power limit by 100%" == f.badge(2 * f.feed)
    assert "power" in f.text()


def test_engagement_from_stock():
    e = mt.engagement_from_stock(500.0, 50.0, 10.0, ap_mm=5.0)
    assert e.area_mm2 == pytest.approx(10.0) and e.ap == 5.0 and e.ae == pytest.approx(2.0)
    e = mt.engagement_from_stock(500.0, 50.0, 10.0, ae_mm=2.0)
    assert e.ap == pytest.approx(5.0)
    e = mt.engagement_from_stock(500.0, 50.0, 10.0)
    assert e.ae is None and e.area_mm2 == pytest.approx(10.0) and e.notes
    e = mt.engagement_from_stock(5000.0, 50.0, 10.0, ap_mm=5.0)
    assert any("exceeds the tool diameter" in n for n in e.notes)
    with pytest.raises(ValueError):
        mt.engagement_from_stock(1.0, 0.0)
    # area alone is enough for the power limit
    f = mt.feed_limit(TOOL, None, mt.engagement_from_stock(500.0, 50.0), 10000, SPINDLE,
                      constants={"specific_energy": 2.0})
    assert f.feed == pytest.approx(60 * 1000 / (2.0 * 10.0))


def test_engagements_from_steps():
    steps = [NS(removed=100.0, length=10.0, tool="t"), NS(removed=0.0, length=0.0, tool="t")]
    es = mt.engagements_from_steps(steps, {"t": 10.0}, ap_mm=5.0)
    assert es[0].ae == pytest.approx(2.0) and es[1].notes


# ------------------------------------------------------------------ M3

def test_wear_g_ratio():
    w = mt.wear("flat_end_mill", None, removed_mm3=1000.0, constants={"g_ratio": 50.0, "usable_volume_mm3": 2.0})
    assert w.model == "g_ratio" and w.tool_volume_mm3 == pytest.approx(20.0)
    assert w.life_fraction == pytest.approx(10.0)
    assert w.sources["g_ratio"] == "given by the caller"
    w = mt.wear("flat_end_mill", None, removed_mm3=1000.0, constants={"g_ratio": 50.0})
    assert w.life_fraction is None and ("life fraction", "constants['usable_volume_mm3']") in w.skipped


def test_wear_per_area_for_saws():
    w = mt.wear("diamond_wire", None, removed_mm3=1.0, cut_area_mm2=2e6,
                constants={"wear_per_area": 0.01, "g_ratio": 5.0, "usable_height_mm": 0.5})
    assert w.model == "per_area" and w.height_loss_mm == pytest.approx(0.02)
    assert w.life_fraction == pytest.approx(0.04)


def test_wear_missing_constants_skip_never_default():
    w = mt.wear("flat_end_mill", "granite", removed_mm3=1000.0)
    assert w.model == "none" and w.tool_volume_mm3 is None and w.life_fraction is None
    assert "g_ratio" in w.skipped[0][1]
    w = mt.wear("disc_saw", "granite", cut_area_mm2=1e6)
    assert w.model == "none" and "wear_per_area" in w.skipped[0][1]
    w = mt.wear("flat_end_mill", None, removed_mm3=1.0, constants={"g_ratio": None})
    assert w.model == "none"
    with pytest.raises(ValueError):
        mt.wear("flat_end_mill", None, removed_mm3=1.0, constants={"g_ratio": 0.0})


def test_wear_uses_sourced_row_when_one_exists(monkeypatch):
    row = dict(material="granite", quantity="g_ratio", low=100.0, high=200.0, unit="mm3/mm3", source="test source",
               source_type="paper", read="page read", note="")
    monkeypatch.setattr(mm, "TABLE", mm.TABLE + [row])
    w = mt.wear("flat_end_mill", "granite", removed_mm3=1000.0)
    assert w.tool_volume_mm3 == pytest.approx(1000.0 / 100.0)          # conservative = low G (more wear)
    assert "test source" in w.sources["g_ratio"]
    w = mt.wear("flat_end_mill", "granite", removed_mm3=1000.0, which="high")
    assert w.tool_volume_mm3 == pytest.approx(5.0)


def test_tool_life_per_step_and_program():
    steps = [NS(removed=100.0, length=10.0, tool="a", kind="flat_end_mill", contact=0.0),
             NS(removed=300.0, length=10.0, tool="a", kind="flat_end_mill", contact=0.0),
             NS(removed=200.0, length=10.0, tool="b", kind="flat_end_mill", contact=0.0)]
    const = {"g_ratio": 100.0, "usable_volume_mm3": 40.0}
    r = mt.tool_life(steps, None, constants=const)
    assert r.per_step == pytest.approx([0.025, 0.075, 0.05])
    assert r.per_tool == pytest.approx({"a": 0.1, "b": 0.05})
    assert r.cumulative == pytest.approx([0.025, 0.1, 0.15])
    assert r.program_fraction == pytest.approx(0.15) and r.programs_per_tool_life == pytest.approx(10.0)
    assert r.evaluated == 3 and "15.00%" in r.text()
    # per-tool constants
    r2 = mt.tool_life(steps, None, constants={"g_ratio": 100.0, "a": {"usable_volume_mm3": 20.0},
                                              "usable_volume_mm3": 40.0})
    assert r2.per_tool["a"] == pytest.approx(0.2) and r2.per_tool["b"] == pytest.approx(0.05)


def test_tool_life_wire_area_and_missing():
    steps = [NS(removed=10.0, length=100.0, tool="w", kind="diamond_wire", contact=50.0)]
    r = mt.tool_life(steps, None, constants={"wear_per_area": 0.02, "usable_height_mm": 0.001})
    assert mt.step_cut_area_mm2(steps[0]) == 5000.0
    assert r.per_step[0] == pytest.approx(5000 / 1e6 * 0.02 / 0.001)
    r = mt.tool_life(steps, "granite")
    assert r.evaluated == 0 and r.per_step == [None] and r.programs_per_tool_life is None
    assert "not evaluated" in r.text() and r.skipped
