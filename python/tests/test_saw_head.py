# Copyright 2026 The lithocam authors. SPDX-License-Identifier: Apache-2.0
"""Right-angle saw heads: TCP and solids, the arm approach angle of disc steps, the ladder's node cost."""
import math
from pathlib import Path

import numpy as np
import pytest

from lithocam.studio import planner, spindles

EX = Path(__file__).resolve().parents[2] / "examples" / "studio"


def test_angle_head_tcp_turns_the_cutter_axis_to_flange_x():
    sp = spindles.preset("kr120_saw_head")
    T = sp.tcp(40.0)
    assert np.allclose(T[:3, 2], [1.0, 0.0, 0.0], atol=1e-12)          # cutter axis along the flange x axis
    assert T[0, 3] * 1000 == pytest.approx(sp.gauge() + 40.0)
    assert T[2, 3] * 1000 == pytest.approx(sp.head_drop)
    assert sp.parts() == []                                             # not a coaxial stack
    names = [s[0] for s in sp.solids(spindles.cutter_parts(type("T", (), dict(
        diameter=350.0, kerf=3.2, bore_diameter=50.0, kind="ToolKind.DiscSaw"))(), 40.0))]
    assert names[:4] == ["plate", "column", "body", "nose"] and names[-1] == "flutes"


def test_blade_is_drawn_at_its_kerf_centred_on_the_tcp():
    blade = type("T", (), dict(diameter=125.0, kerf=2.0, bore_diameter=22.23, kind="ToolKind.DiscSaw"))()
    parts = spindles.cutter_parts(blade, 40.0)
    name, d, z0, ln = parts[-1]
    assert (name, d, ln) == ("flutes", 125.0, 2.0) and z0 + ln / 2 == pytest.approx(40.0)


def test_coaxial_solids_match_the_old_parts():
    sp = spindles.preset("cm_pds")
    sol = sp.solids()
    for (n, d, z0, ln), (n2, d2, ln2, F) in zip(sp.parts(), sol):
        assert (n, d, ln) == (n2, d2, ln2) and F[2, 3] == pytest.approx(z0) and np.allclose(F[:3, :3], np.eye(3))


def test_ladder_node_cost_moves_the_path_off_a_penalised_node():
    class B:
        checks_collision = False

        def ik(self, T, hint):
            return [np.zeros(6), np.full(6, 0.01)]        # node 0 is closer to the seed

    rungs = [[np.eye(4)]] * 3
    base = planner.ladder(rungs, B(), seed=np.zeros(6))
    assert all(np.allclose(q, 0) for q in base.joints)
    pen = planner.ladder(rungs, B(), seed=np.zeros(6), node_cost=lambda Q: np.where(Q[:, 0] == 0, 1.0, 0.0))
    assert all(np.allclose(q, 0.01) for q in pen.joints)


def test_ur10e_comb_with_the_angle_head_comes_in_at_45_deg_and_keeps_the_wrist_clear():
    api = pytest.importorskip("lithocam.studio.api")
    s = api.Session.open(EX / "disc_comb_ur10e.lcs.json")
    assert s.station.spindle == "ur10e_saw_head"
    r = s.simulate("Saw")
    assert r.ok and r.unreachable == 0
    assert any("arm comes in 45 deg" in m for m in r.messages)
    robot = s.robot
    q = [np.asarray(x.data["q"], float) for x in r.expansion.records if x.kind == "movel" and x.data.get("cut")]
    assert min(abs(math.sin(v[4])) for v in q) > 0.5                    # wrist more than 30 deg from singular
    seg = [g for g in r.expansion.segments if g.feed.any()][0]
    F = robot.link_transforms(seg.q[int(np.argmax(seg.feed))])[robot.tcp_link]
    ang = math.degrees(math.acos(-F[2, 2]))                              # flange z from straight down
    assert abs(ang - 45.0) < 15.0
