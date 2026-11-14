# Copyright 2026 The lithocam authors. SPDX-License-Identifier: Apache-2.0
"""The stock as it stands just before a program step, from a stock run (studio.process.StockRun).

Rest machining needs the material the earlier steps left. When the studio holds a stock run of the previous
simulation, `LiveStock` replays its recorded cuts (ball and cylinder tools, discs) up to a step, on a fresh stock box
of the same cell size, and hands that SdfStock to `program.expand(..., leftover=...)`. The run keeps one cut per
feed sample (tip, end axis, tool), which is all those tool models need. Wire cuts are not replayed: they need the
full tool pose, which the run does not keep.

A run belongs to the program it was made from. `LiveStock` serves a step only when every step before it, and the
station, are unchanged since that run (compare `signature`), so an edited program never gets a stale stock.
"""
from __future__ import annotations

import dataclasses
import json
from typing import Dict, List, Optional

import numpy as np

from . import process, program


def signature(station: "program.Station", prog: "program.Program") -> List[str]:
    """Per-step fingerprints of a program plus one of the station without its programs: the first entry. Two
    signatures agree up to step i when [:i + 1] of them are equal."""
    base = dataclasses.replace(station, programs=[]).to_json()
    return [base] + [json.dumps([s.kind, s.params, s.label], sort_keys=True, default=str) for s in prog.steps]


def _frame(z_axis: np.ndarray, p: np.ndarray) -> np.ndarray:
    """A 4x4 pose with the given z axis (any x and y) at the point p."""
    z = np.asarray(z_axis, dtype=float)
    z = z / np.linalg.norm(z)
    a = np.array([1.0, 0.0, 0.0]) if abs(z[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    x = np.cross(a, z)
    x /= np.linalg.norm(x)
    T = np.eye(4)
    T[:3, 0], T[:3, 1], T[:3, 2], T[:3, 3] = x, np.cross(z, x), z, p
    return T


def stock_before_step(run, expansion, tools: Dict[str, object], station, robot, step: int):
    """An SdfStock: the station's stock box cut by every recorded cut of the steps before program step `step`.
    Wire cuts and tools without a stock model are skipped."""
    stock = process.stock_box(station, run.cell)
    models: Dict[str, Optional[dict]] = {}
    last_axis: Dict[int, np.ndarray] = {}
    for sc in run.steps:
        seg = expansion.segments[sc.segment]
        if seg.step >= step:
            break
        if seg.tool not in models:
            try:
                models[seg.tool] = process.tool_model(tools[seg.tool], station.robot, robot)
            except (KeyError, ValueError):
                models[seg.tool] = None
        m = models[seg.tool]
        if m is None or m["kind"] == process.WIRE:
            continue
        a1 = np.asarray(sc.axis, dtype=float)
        a0 = last_axis.get(sc.segment, a1)
        last_axis[sc.segment] = a1
        sign = 1.0 if m["kind"] == process.DISC else -1.0       # disc: blade normal = tool z; else tool z = -axis
        process.cut_step(stock, m, _frame(sign * a0, sc.p0), _frame(sign * a1, sc.p1))
    return stock


class LiveStock:
    """Callable for `program.expand(leftover=...)`: LiveStock(run, expansion, tools, station, robot, old_sig, new_sig)
    returns the stock before step i, or None when step i cannot use the run (the steps before it changed, or the
    run has no such step)."""

    def __init__(self, run, expansion, tools, station, robot, old_sig: List[str], new_sig: List[str]):
        self.args = (run, expansion, tools, station, robot)
        self.old, self.new = old_sig, new_sig
        self._cache: Dict[int, object] = {}

    def usable(self, i: int) -> bool:
        return i + 1 <= len(self.old) - 1 and i + 1 <= len(self.new) - 1 and self.old[:i + 1] == self.new[:i + 1]

    def __call__(self, i: int):
        if not self.usable(i):
            return None
        if i not in self._cache:
            self._cache[i] = stock_before_step(*self.args, i)
        return self._cache[i]
