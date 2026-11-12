# Copyright 2026 The lithocam authors. SPDX-License-Identifier: Apache-2.0
"""Ladder planner on a synthetic backend where the answer is known: least motion through the candidates, colliding
nodes skipped, a break where no step under the limit exists, and agreement with brute force on small ladders."""
import itertools
import math

import pytest

np = pytest.importorskip("numpy")
from lithocam.studio.planner import ladder  # noqa: E402


class Fake:
    """Poses are just labels; ik returns the configurations listed for each label."""
    name = "fake"
    checks_collision = True

    def __init__(self, table, colliding=()):
        self.table = table
        self.colliding = [np.asarray(c, float) for c in colliding]

    def ik(self, T, seed=None):
        return [np.asarray(q, float) for q in self.table[T]]

    def in_collision(self, q):
        return any(np.allclose(q, c) for c in self.colliding)


def test_least_motion_and_collision_skip():
    table = {"a0": [[0.0], [1.0]], "b0": [[0.1], [0.9]], "c0": [[0.2], [2.0]]}
    res = ladder([["a0"], ["b0"], ["c0"]], Fake(table), max_step=0.5)
    assert [float(q[0]) for q in res.joints] == [0.0, 0.1, 0.2] and not res.breaks
    res = ladder([["a0"], ["b0"], ["c0"]], Fake(table, colliding=[[0.1]]), max_step=1.5)
    assert [float(q[0]) for q in res.joints] == [1.0, 0.9, 0.2]
    assert res.collisions_rejected == 1


def test_break_where_no_step_fits():
    table = {"a": [[0.0]], "b": [[0.1]], "c": [[2.0]], "d": [[2.1]]}
    res = ladder([["a"], ["b"], ["c"], ["d"]], Fake(table), max_step=math.radians(30))
    assert res.breaks == [2] and len(res.joints) == 4
    res = ladder([["a"], ["x"], ["d"]], Fake(dict(table, x=[])), max_step=10)
    assert res.missing == [1]


def test_matches_brute_force_on_random_ladders():
    rng = np.random.default_rng(5)
    for _ in range(60):
        n, k = int(rng.integers(2, 5)), int(rng.integers(1, 4))
        table = {(r, c): [rng.uniform(-1, 1, 2).tolist() for _ in range(int(rng.integers(1, 3)))]
                 for r in range(n) for c in range(k)}
        rungs = [[(r, c) for c in range(k)] for r in range(n)]
        res = ladder(rungs, Fake(table), max_step=10.0)
        nodes = [[np.asarray(q) for c in range(k) for q in table[(r, c)]] for r in range(n)]
        best = min(sum(float(np.sum((p[i + 1] - p[i]) ** 2)) for i in range(n - 1))
                   for p in itertools.product(*nodes))
        got = sum(float(np.sum((res.joints[i + 1] - res.joints[i]) ** 2)) for i in range(n - 1))
        assert abs(got - best) < 1e-9
