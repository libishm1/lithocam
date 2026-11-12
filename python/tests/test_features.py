# Copyright 2026 The lithocam authors. SPDX-License-Identifier: Apache-2.0
"""Stone feature mapping on five dimension-stone parts (exact B-rep fixtures from tests/data/make_parts.py).

The mapping tests read recorded quiddity documents (tests/data/quiddity/*.json), so they run without quiddity.
The end-to-end test runs quiddity on the STEP files when it is installed and checks it still agrees.
"""
import json
from collections import Counter
from pathlib import Path

import pytest

from lithocam.features import (CONTOUR_CUT, DRILL, LATHE, PROFILE_WHEEL, ROUTER, SAW_DISC, UNMAPPED, Rules, diff,
                               run_quiddity, stone_ops)

DATA = Path(__file__).resolve().parent / "data"
PARTS = ["baluster", "cladding_panel", "stair_tread", "window_sill", "worktop"]


def load(name):
    return stone_ops(json.loads((DATA / "quiddity" / (name + ".json")).read_text(encoding="utf-8")))


def kinds(fs):
    return Counter((op.kind, op.route) for op in fs.ops)


def test_cladding_panel():
    fs = load("cladding_panel")
    assert kinds(fs) == Counter({("anchor_hole", DRILL): 4, ("kerf_slot", SAW_DISC): 2})
    holes = sorted(op.at[:2] for op in fs.ops if op.kind == "anchor_hole")
    assert holes == [(100.0, 80.0), (100.0, 320.0), (500.0, 80.0), (500.0, 320.0)]
    for op in fs.ops:
        if op.kind == "anchor_hole":
            assert op.params["diameter"] == 10.0 and op.params["depth"] == 20.0 and not op.params["through"]
        else:
            assert (op.params["width"], op.params["depth"], op.params["length"]) == (3.0, 15.0, 100.0)
    grid = [p for p in fs.patterns if p["kind"] == "hole_grid"]
    assert len(grid) == 1 and grid[0]["count"] == 4


def test_window_sill():
    fs = load("window_sill")
    assert kinds(fs) == Counter({("drip_groove", SAW_DISC): 1, ("rebate", SAW_DISC): 1,
                                 ("edge_chamfer", PROFILE_WHEEL): 1})
    drip = next(op for op in fs.ops if op.kind == "drip_groove")
    assert (drip.params["width"], drip.params["depth"], drip.params["length"]) == (10.0, 10.0, 1200.0)
    assert drip.at[1] == 25.0                       # 20 from the front edge, 10 wide


def test_worktop_cutout_absorbs_corner_fillets_and_plates():
    fs = load("worktop")
    assert kinds(fs) == Counter({("cutout", CONTOUR_CUT): 1, ("through_hole", DRILL): 1})
    cut = next(op for op in fs.ops if op.kind == "cutout")
    assert cut.params["through"] is True and cut.params["corner_radius"] == 25.0
    assert (cut.params["length"], cut.params["width"]) == (500.0, 400.0)
    assert sum(1 for s in cut.source if s.startswith("fillets/")) == 4
    tap = next(op for op in fs.ops if op.kind == "through_hole")
    assert tap.params["diameter"] == 35.0 and tap.params["tool"] == "core_drill"
    dropped = {d["id"].split("/")[0] for d in fs.dropped}
    assert {"plates", "step_levels", "risers", "fillets"} <= dropped
    assert all("check by hand" not in d["reason"] for d in fs.dropped)


def test_stair_tread_groove_set():
    fs = load("stair_tread")
    assert kinds(fs) == Counter({("stopped_groove", ROUTER): 3, ("edge_round", PROFILE_WHEEL): 1})
    sets = [p for p in fs.patterns if p["kind"] == "stopped_groove_set"]
    assert len(sets) == 1
    assert sets[0]["count"] == 3 and sets[0]["pitch"] == pytest.approx(15.0) and sets[0]["axis"] == "y"
    nose = next(op for op in fs.ops if op.kind == "edge_round")
    assert nose.params["radius"] == 10.0


def test_baluster_is_one_lathe_op():
    fs = load("baluster")
    assert kinds(fs) == Counter({("turned_profile", LATHE): 1})
    op = fs.ops[0]
    assert op.params["length"] == 380.0 and op.params["od"] == 120.0
    assert sorted(d["id"] for d in fs.dropped) == ["bosses/0", "bosses/1", "bosses/2"]


@pytest.mark.parametrize("name", PARTS)
def test_every_recognised_face_is_accounted(name):
    """Each face is in a kept op, the stock, a dropped record, or quiddity's unexplained list. Nothing is unmapped."""
    raw = json.loads((DATA / "quiddity" / (name + ".json")).read_text(encoding="utf-8"))
    fs = stone_ops(raw)
    assert not [op for op in fs.ops if op.route == UNMAPPED]
    rec_faces = {f for r in raw["document"]["records"] for f in r["faces"]}
    op_faces = {f for op in fs.ops for f in op.faces}
    dropped_ids = {d["id"] for d in fs.dropped}
    dropped_faces = {f for r in raw["document"]["records"] if r["id"] in dropped_ids for f in r["faces"]}
    pattern_faces = {f for r in raw["document"]["records"] if r["family"].endswith("_patterns") for f in r["faces"]}
    assert rec_faces <= op_faces | dropped_faces | pattern_faces


def test_revision_diff_in_file_frame():
    """Rev B: left anchor holes +20 in x; kerf slots 100 -> 120 long about the same centre.

    quiddity's correspond() aligns the parts first and returns every hole and slot as ambiguous on this symmetric
    panel. The file-frame diff resolves all six.
    """
    d = diff(load("cladding_panel"), load("cladding_panel_revB"))
    status = Counter((x["kind"], x["status"]) for x in d)
    assert status == Counter({("anchor_hole", "unchanged"): 2, ("anchor_hole", "moved"): 2,
                              ("kerf_slot", "resized"): 2})
    for x in d:
        if x["status"] == "moved":
            assert x["move"] == [20.0, 0.0, 0.0]
        if x["status"] == "resized":
            assert x["changed"] == {"length": [100.0, 120.0]}


def test_unknown_family_is_kept_as_unmapped():
    doc = {"document": {"schema_version": "quiddity-rust/recognition/6", "face_count": 1, "face_accounting": [{}],
                        "records": [{"id": "gusset_ribs/0", "family": "gusset_ribs", "faces": [0],
                                     "parameters": {"at": [1.0, 2.0, 3.0], "thickness": 4.0},
                                     "evidence_status": "verified"}]}}
    fs = stone_ops(doc)
    assert [(op.kind, op.route, op.at) for op in fs.ops] == [("gusset_ribs", UNMAPPED, (1.0, 2.0, 3.0))]


def test_rejects_other_documents():
    with pytest.raises(ValueError):
        stone_ops({"document": {"schema_version": "something-else/1", "records": []}})


def test_narrow_threshold_switches_route():
    fs = stone_ops(json.loads((DATA / "quiddity" / "cladding_panel.json").read_text(encoding="utf-8")),
                   Rules(narrow_max=2.0))
    assert {op.kind for op in fs.ops if op.source[0].startswith("pockets/")} == {"pocket"}


@pytest.mark.parametrize("name", PARTS + ["cladding_panel_revB"])
def test_end_to_end_matches_recording(name):
    try:
        live = run_quiddity(str(DATA / "step" / (name + ".step")))
    except RuntimeError as e:
        pytest.skip(str(e))
    rec = json.loads((DATA / "quiddity" / (name + ".json")).read_text(encoding="utf-8"))
    a, b = stone_ops(live), stone_ops(rec)
    assert [(o.kind, o.route, o.at) for o in a.ops] == [(o.kind, o.route, o.at) for o in b.ops]
