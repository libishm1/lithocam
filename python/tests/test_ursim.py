# Copyright 2026 The lithocam authors. SPDX-License-Identifier: Apache-2.0
"""The offline parts of the URSim check: waypoints, the simulation copy, chunks and the ordered matcher."""
import numpy as np

from lithocam import ursim

PROG = """def Saw():
  movej([0.000000, -1.570796, 1.570796, -1.570796, -1.570796, 0.000000], a=1.0, v=0.5000, r=0.0000)
  popup("lithocam: Put tool SAW125 in the spindle, then continue", blocking=True)
  set_tcp(p[0.187000, 0.000000, 0.090000, 0.000000, 1.570796, 0.000000])  # tool SAW125
  movep(p[-0.49, 0.47, 0.08, -0.7, -1.7, -1.7], a=0.5, v=0.0033, r=0.0000)  # q [0.1, -1.1, 2.0, -1.7, -2.0, -0.5]
  movep(p[-0.50, 0.47, 0.08, -0.7, -1.7, -1.7], a=0.5, v=0.0033, r=0.0005)  # q [0.2, -1.1, 2.0, -1.7, -2.0, -0.5]
  movel([0.3, -1.1, 2.0, -1.7, -2.0, -0.5], a=0.5, v=0.1000, r=0.0005)
  movel([0.4, -1.1, 2.0, -1.7, -2.0, -0.5], a=0.5, v=0.1000, r=0.0000)
end
Saw()
"""


def test_waypoints_take_the_joint_lists_and_the_q_notes():
    W = ursim.waypoints(PROG)
    assert W.shape == (5, 6)
    assert np.allclose(W[:, 0], [0.0, 0.1, 0.2, 0.3, 0.4])


def test_sim_copy_comments_popups_and_scales_only_linear_speeds():
    s = ursim.sim_copy(PROG, speed_scale=30.0, max_speed=0.25)
    assert "# [simulation copy] popup(" in s
    assert "v=0.0990" in s                        # 0.0033 x 30
    assert "v=0.2500" in s                        # 0.1 x 30 capped at 0.25
    assert "v=0.5000" in s                        # movej untouched
    assert np.allclose(ursim.waypoints(s), ursim.waypoints(PROG))


def test_chunks_repeat_the_tcp_and_keep_every_motion_line():
    progs = ursim.chunks(PROG, moves_per_chunk=2)
    assert len(progs) == 3
    assert all(p.startswith("def lc_chunk_{}():".format(k)) and p.rstrip().endswith("lc_chunk_{}()".format(k))
               for k, p in enumerate(progs))
    assert "set_tcp(" in progs[1] and "set_tcp(" in progs[2]
    assert np.allclose(np.concatenate([ursim.waypoints(p) for p in progs]), ursim.waypoints(PROG))


def test_ordered_matcher_does_not_jump_to_a_later_pass_with_the_same_joints():
    # two identical passes (as comb slots repeat a joint pattern), the plan covers both in order
    t = np.linspace(0, 1, 400)
    one = np.stack([t, np.zeros_like(t), t * 0.5, np.zeros_like(t), np.zeros_like(t), np.zeros_like(t)], 1)
    rec = np.concatenate([one, one[::-1], one])
    plan = np.concatenate([one[::40], one[::-1][::40], one[::40]])
    e = ursim.compare(plan, rec, tol_deg=0.5)
    assert e.max() < 0.2
    # a waypoint the robot never reaches is reported, and the pointer does not run ahead
    bad = plan.copy()
    bad[5] += np.radians(5.0)
    e2 = ursim.compare(bad, rec, tol_deg=0.5)
    assert e2[5] > 4.0 and np.delete(e2, 5).max() < 0.2
