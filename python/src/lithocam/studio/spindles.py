# Copyright 2026 The lithocam authors. SPDX-License-Identifier: Apache-2.0
"""Spindles and tool drives on the robot flange, holding the cutter.

The stack along the flange z axis, from the flange face outwards:

    adapter plate  ->  spindle body  ->  nose / collet nut  ->  cutter (shank in the collet, then the flutes)

The TCP is the cutter tip: plate + body + nose + the cutter's stickout from the collet face. The spindle axis may be
offset sideways from the flange axis (side-mounted spindles); the TCP keeps the spindle axis as its z.

Presets marked "example" are placeholder geometry to edit; presets from a datasheet name their source. lithocam
does not invent speeds or powers: those fields are 0 (= not stated) unless a source gives them.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass, field
from typing import Dict, List, Optional, Tuple

import numpy as np


@dataclass
class Spindle:
    id: str
    name: str
    plate_thickness: float = 10.0     # mm, adapter plate on the flange
    plate_diameter: float = 90.0
    body_diameter: float = 80.0       # mm
    body_length: float = 180.0
    nose_diameter: float = 40.0       # collet nut
    nose_length: float = 30.0
    axis_offset: Tuple[float, float] = (0.0, 0.0)   # mm, spindle axis from the flange axis (x, y)
    collet: str = "ER20"
    max_collet_mm: float = 13.0       # largest shank the collet takes
    max_rpm: float = 0.0              # 0 = not stated
    power_w: float = 0.0
    mass_kg: float = 0.0
    control: str = ""                 # how the robot program commands it (I/O, analog, URCap, VFD)
    source: str = "example"

    def gauge(self) -> float:
        """Flange face to collet face, mm."""
        return self.plate_thickness + self.body_length + self.nose_length

    def tcp(self, tool_stickout_mm: float) -> np.ndarray:
        """TCP (cutter tip) in the flange frame, metres."""
        T = np.eye(4)
        T[0, 3], T[1, 3] = self.axis_offset[0] / 1000, self.axis_offset[1] / 1000
        T[2, 3] = (self.gauge() + tool_stickout_mm) / 1000
        return T

    def parts(self) -> List[Tuple[str, float, float, float]]:
        """(name, diameter mm, z0 mm, length mm) of the coaxial cylinders, in the flange frame along z."""
        z = 0.0
        out = [("plate", self.plate_diameter, z, self.plate_thickness)]
        z += self.plate_thickness
        out.append(("body", self.body_diameter, z, self.body_length))
        z += self.body_length
        out.append(("nose", self.nose_diameter, z, self.nose_length))
        return out


def cutter_parts(tool, stickout_mm: float) -> List[Tuple[str, float, float, float]]:
    """(name, diameter mm, z0 mm from the collet face, length mm): the shank, then the cutting part to the tip."""
    D = float(getattr(tool, "diameter", 0.0) or 0.0)
    flute = float(getattr(tool, "flute_length", 0.0) or 0.0)
    flute = min(flute if flute > 0 else stickout_mm, stickout_mm)
    shank_d = float(getattr(tool, "shank_diameter", 0.0) or 0.0) or min(D, 12.0) or D
    shank_len = max(stickout_mm - flute, 0.0)
    out = []
    if shank_len > 0:
        out.append(("shank", shank_d, 0.0, shank_len))
    out.append(("flutes", D, shank_len, flute))
    return out


def stickout(tool, default: float = 40.0) -> float:
    """Cutter length out of the collet, mm: the tool's stickout, else flute length + 15 mm, else `default`."""
    s = float(getattr(tool, "stickout", 0.0) or 0.0)
    if s > 0:
        return s
    f = float(getattr(tool, "flute_length", 0.0) or 0.0)
    return f + 15.0 if f > 0 else default


PRESETS: Dict[str, Spindle] = {
    # Datasheet values where the source states them; body geometry is estimated (sources give no dimensions).
    "cm_pds": Spindle("cm_pds", "Compact Modul CM-PDS, ER20 (UR URCap spindle)",
                      plate_thickness=12, plate_diameter=80, body_diameter=70, body_length=190, nose_diameter=35,
                      nose_length=28, collet="ER20", max_collet_mm=13.0, max_rpm=12000, power_w=1250,
                      control="URCap program node (speed, start/stop); ISO 9409-1-50-4-M6 flange; 230 VAC drive",
                      source="UR marketplace listing CM-PDS (power, speed, ER20 2-13 mm, flange); body size "
                             "estimated"),
    "hx3c": Spindle("hx3c", "Hiteco Robotech HX 3C, HSK63F (robot electrospindle)",
                    plate_thickness=30, plate_diameter=220, body_diameter=160, body_length=360, nose_diameter=63,
                    nose_length=60, collet="HSK63F", max_collet_mm=25.0, max_rpm=24000, power_w=20000, mass_kg=25.0,
                    control="VFD and EtherCAT; automatic pneumatic tool change; sensors S1/S2/S3",
                    source="EtherCAT product listing Hiteco HX 3C (20 kW S1, 24,000 rpm, HSK63F, 25 kg, liquid "
                           "cooled); body size estimated"),
    "ur10e_example":Spindle("ur10e_example", "Light electrospindle, ER11 (example size, UR10e)",
                             plate_thickness=12, plate_diameter=80, body_diameter=62, body_length=170,
                             nose_diameter=22, nose_length=20, collet="ER11", max_collet_mm=7.0,
                             control="digital out on / analog speed (set per cell)"),
    "kr120_example": Spindle("kr120_example", "Milling electrospindle, ER32 (example size, KR120)",
                             plate_thickness=25, plate_diameter=200, body_diameter=150, body_length=330,
                             nose_diameter=50, nose_length=45, collet="ER32", max_collet_mm=20.0,
                             control="VFD: $OUT start, $ANOUT speed (set per cell)"),
}
DEFAULT_FOR_ROBOT = {"ur10e": "cm_pds", "kr120": "hx3c"}


def to_dict(s: Spindle) -> Dict:
    return asdict(s)


def from_dict(d: Dict) -> Spindle:
    d = dict(d)
    d["axis_offset"] = tuple(d.get("axis_offset", (0.0, 0.0)))
    return Spindle(**d)
