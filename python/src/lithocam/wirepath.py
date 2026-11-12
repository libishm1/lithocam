# Copyright 2026 The lithocam authors. SPDX-License-Identifier: Apache-2.0
"""Diamond-wire targets and turntable helpers, numpy only (shared by lithocam.robot and the studio).

A wire saw on the flange (cells/make_cells.py) carries the wire along tool x at `bow_depth` along tool z. For a
ruling p0 -> p1 of a ruled face with outward normal n, the wire runs along the ruling, offset by the wire radius
(kerf compensation) outwards, and the bow reaches over the cut (tool z against n), turned by a roll about the wire.
"""
from __future__ import annotations

import json
import math
from pathlib import Path
from types import SimpleNamespace as NS
from typing import Sequence

import numpy as np


def rot_x(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], dtype=np.float64)


def rot_z(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


def wire_target(p0, p1, normal, wire_radius: float, roll: float, bow_depth: float) -> np.ndarray:
    """tool0 pose (4x4) that puts the wire (tool x axis, at bow_depth along tool z) on the kerf-offset ruling.

    Units are metres. The tool z axis points against the face normal (the bow reaches over the cut), turned by
    `roll` about the wire.
    """
    p0, p1, n = (np.asarray(v, dtype=np.float64) for v in (p0, p1, normal))
    x = p1 - p0
    x /= np.linalg.norm(x)
    z = -(n - np.dot(n, x) * x)
    z /= np.linalg.norm(z)
    y = np.cross(z, x)
    R = np.column_stack([x, y, z]) @ rot_x(roll)
    mid = 0.5 * (p0 + p1) + n * wire_radius
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = mid - R[:, 2] * bow_depth          # tool0 sits bow_depth behind the wire along tool z
    return T


def turned_face(face, yaw: float):
    """The face's rulings turned by yaw (radians) about the part's z axis (a turntable under the part)."""
    Rz = rot_z(yaw)
    rs = [NS(p0=(Rz @ np.asarray(r.p0)).tolist(), p1=(Rz @ np.asarray(r.p1)).tolist(),
             normal=(Rz @ np.asarray(r.normal)).tolist()) for r in face.rulings]
    return NS(index=face.index, label=face.label, rulings=rs)


def facing_yaw(face, placement: Sequence[float]) -> float:
    """Turntable angle that points the face's mean outward normal at the robot base (horizontal component)."""
    n = np.mean([np.asarray(r.normal) for r in face.rulings], axis=0)
    want = -np.asarray(placement, float)[:2]
    return math.atan2(want[1], want[0]) - math.atan2(n[1], n[0])


def order_rulings(rulings):
    """Rulings sorted along the sweep: by the projection of their midpoints on the main direction of the midpoints."""
    rs = list(rulings)
    if len(rs) < 2:
        return rs
    mids = np.array([(np.asarray(r.p0) + np.asarray(r.p1)) / 2 for r in rs])
    c = mids - mids.mean(axis=0)
    sweep = np.linalg.svd(c, full_matrices=False)[2][0]
    return [rs[i] for i in np.argsort(c @ sweep)]


def load_ruled_json(path: str):
    """Read `python -m lithocam.ruled ... --json` output into objects with the attributes wire_reach uses.

    This keeps the OCP environment (ruled) and the Tesseract environment (robot) separate.
    """
    d = json.loads(Path(path).read_text(encoding="utf-8"))
    faces = [NS(index=f["index"], label=f["label"], rulings=[NS(**r) for r in f["rulings"]]) for f in d["faces"]]
    return NS(faces=faces, rules=NS(**d["rules"]), scale=d["scale"])
