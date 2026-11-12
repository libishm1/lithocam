# Copyright 2026 The lithocam authors. SPDX-License-Identifier: Apache-2.0
"""Ruled-surface extraction and wire / flank / point labels (needs OCP: pip install cadquery-ocp)."""
import math
from collections import Counter
from pathlib import Path

import pytest

ruled = pytest.importorskip("lithocam.ruled", reason="OCP (OpenCascade) not installed", exc_type=ImportError)

DATA = Path(__file__).resolve().parent / "data"


@pytest.fixture(scope="module")
def reports():
    out = {}
    for name in ("cladding_panel", "worktop", "window_sill"):
        out[name] = ruled.analyse_step(str(DATA / "step" / (name + ".step")))
    for name in ("twisted_column", "cone_frustum", "freeform_loft"):
        out[name] = ruled.analyse_step(str(DATA / "step_ruled" / (name + ".step")))
    return out


def labels(rep):
    return Counter((f.surface, f.label) for f in rep.faces)


def test_cladding_panel(reports):
    rep = reports["cladding_panel"]
    # 6 outer faces by wire; 4 blind-hole walls by flank (open at the back); the hole floors and all
    # kerf-slot faces (3 mm wide, narrower than the 7 mm wire and the 10 mm tool) are point
    assert labels(rep) == Counter({("plane", "wire"): 6, ("cylinder", "flank"): 4, ("plane", "point"): 14})


def test_worktop_through_cutout_is_wire(reports):
    assert labels(reports["worktop"]) == Counter({("plane", "wire"): 10, ("cylinder", "wire"): 5})


def test_window_sill_all_wire(reports):
    rep = reports["window_sill"]
    assert all(f.label == "wire" for f in rep.faces)
    assert max(f.max_ruling for f in rep.faces) == pytest.approx(1200.0, abs=1e-3)   # the drip groove runs through


def test_twisted_column_sides_are_exactly_ruled(reports):
    rep = reports["twisted_column"]
    sides = [f for f in rep.faces if f.surface == "bspline"]
    assert len(sides) == 4 and all(f.ruled == "exact" and f.label == "wire" for f in sides)


def test_cone_rulings_are_generators(reports):
    cone = next(f for f in reports["cone_frustum"].faces if f.surface == "cone")
    assert cone.label == "wire"
    assert cone.max_ruling == pytest.approx(math.hypot(500.0, 100.0), rel=1e-6)   # slant height
    for r in cone.rulings:       # every ruling runs from the D400 base circle to the D200 top circle
        z = sorted((r.p0, r.p1), key=lambda p: p[2])
        assert math.hypot(z[0][0], z[0][1]) == pytest.approx(200.0, abs=1e-6)
        assert math.hypot(z[1][0], z[1][1]) == pytest.approx(100.0, abs=1e-6)


def test_freeform_sides_are_not_ruled(reports):
    rep = reports["freeform_loft"]
    curved = [f for f in rep.faces if f.surface == "bspline"]
    assert curved and all(f.ruled == "no" and f.label == "point" for f in curved)


def test_rulings_lie_on_the_face_and_in_the_trim(reports):
    """Ruling midpoints of exact faces sit on the part surface: the solid classifier says ON."""
    from OCP.BRepClass3d import BRepClass3d_SolidClassifier
    from OCP.gp import gp_Pnt
    from OCP.TopAbs import TopAbs_ON
    shape = ruled.read_step(str(DATA / "step" / "cladding_panel.step"))
    rep = reports["cladding_panel"]
    for f in rep.faces:
        for r in f.rulings:
            m = gp_Pnt(*[(r.p0[k] + r.p1[k]) / 2 for k in range(3)])
            assert BRepClass3d_SolidClassifier(shape, m, 1e-3).State() == TopAbs_ON


def test_wire_too_short_falls_back(reports):
    rep = ruled.analyse_step(str(DATA / "step" / "window_sill.step"), ruled.Rules(wire_length_max=500.0))
    long_faces = [f for f in rep.faces if f.max_ruling > 500.0]
    assert long_faces and all(f.label != "wire" or f.max_ruling <= 500.0 for f in rep.faces)


def test_thicker_wire_blocks_narrow_features(reports):
    """A 12 mm wire no longer fits the 10 mm drip groove, so the groove's inner faces stop being wire."""
    thin = reports["window_sill"]
    thick = ruled.analyse_step(str(DATA / "step" / "window_sill.step"), ruled.Rules(wire_diameter=12.0))
    assert sum(f.label == "wire" for f in thick.faces) < sum(f.label == "wire" for f in thin.faces)
