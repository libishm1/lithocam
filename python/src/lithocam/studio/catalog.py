# Copyright 2026 The lithocam authors. SPDX-License-Identifier: Apache-2.0
"""Catalogue of program steps for the studio palette: robot instructions and machining operations,
stone operations first. Status follows docs/OPERATIONS.md:

  done      the studio generates a path for it now;
  partial   the kernel exists, the studio keeps the step and posts a comment until its generator lands;
  planned   listed so programs can be laid out now; posts a comment.

`tool_family` is the operation family used to filter the tool library (lithocam tool_operations).
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Tuple

# (key, label, kind, default, unit). kind: "float", "int", "bool", "text", "choice:a|b|c", "tool", "joints", "pose"
Param = Tuple[str, str, str, object, str]


@dataclass(frozen=True)
class StepSpec:
    id: str
    label: str
    category: str
    glyph: str                    # short text drawn on the icon
    status: str = "planned"
    tool_family: str = ""         # "" = no tool
    params: Tuple[Param, ...] = field(default_factory=tuple)
    doc: str = ""


CATEGORIES: List[Tuple[str, str]] = [   # (name, colour)
    ("Robot", "#3d7bd9"),
    ("Setup", "#6c6f7d"),
    ("Stone", "#b5651d"),
    ("2.5-axis", "#2f9e44"),
    ("3-axis", "#1c7ed6"),
    ("5-axis", "#ae3ec9"),
    ("Holes", "#e8590c"),
    ("Finishing", "#c2255c"),
]

_CUT = (("stepdown", "Step-down", "float", 5.0, "mm"), ("stepover", "Stepover", "float", 4.0, "mm"),
        ("allowance", "Stock to leave", "float", 0.0, "mm"), ("feed", "Feed", "float", 0.0, "mm/min (0 = tool)"),
        ("rpm", "Spindle", "float", 0.0, "rev/min (0 = tool)"))
_LINK = (("clearance", "Clearance height", "float", 20.0, "mm above stock"),
         ("tolerance", "Path tolerance", "float", 0.01, "mm"))

STEPS: List[StepSpec] = [
    # ---------------------------------------------------------------- robot instructions
    StepSpec("movej", "Joint move", "Robot", "J", "done",
             params=(("target", "Target", "target", "", "name; overrides joints"),
                     ("joints", "Joints", "joints", None, "deg"), ("speed", "Speed", "float", 50.0, "% of max"))),
    StepSpec("movel", "Linear move", "Robot", "L", "done",
             params=(("target", "Target", "target", "", "name; overrides pose"),
                     ("pose", "TCP pose", "pose", None, "mm, deg"), ("speed", "Speed", "float", 100.0, "mm/s"),
                     ("blend", "Blend radius", "float", 0.0, "mm"))),
    StepSpec("set_tool", "Set tool", "Robot", "T", "done", "any", params=(("tool", "Tool", "tool", "", ""),)),
    StepSpec("set_frame", "Set reference", "Robot", "F", "partial",
             params=(("frame", "Frame", "choice:work|base", "work", ""),)),
    StepSpec("speed", "Set speed", "Robot", "v", "done",
             params=(("linear", "Linear", "float", 100.0, "mm/s"), ("joint", "Joint", "float", 50.0, "%"))),
    StepSpec("pause", "Pause", "Robot", "||", "done", params=(("seconds", "Seconds (-1 = wait)", "float", -1.0, "s"),)),
    StepSpec("io", "Set / wait I/O", "Robot", "IO", "done",
             params=(("mode", "Mode", "choice:set|wait", "set", ""), ("port", "Port", "int", 0, ""),
                     ("value", "Value", "bool", True, ""))),
    StepSpec("gripper", "Gripper", "Robot", "G", "partial",
             params=(("width", "Width", "float", 80.0, "mm"), ("force", "Force", "float", 40.0, "N"))),
    StepSpec("call", "Program call", "Robot", "()", "partial", params=(("name", "Program", "text", "", ""),)),
    StepSpec("comment", "Comment", "Robot", "#", "done", params=(("text", "Text", "text", "", ""),)),
    # ---------------------------------------------------------------- setup
    StepSpec("setup", "Setup: work frame and stock", "Setup", "S", "done",
             params=(("stock_margin", "Stock margin", "float", 5.0, "mm"), ("yaw", "Turntable", "float", 0.0, "deg"),
                     ("tilt", "Tool tilt (3+2)", "float", 0.0, "deg about the table x axis"))),
    StepSpec("tool_change", "Tool change", "Setup", "TC", "done", "any", params=(("tool", "Tool", "tool", "", ""),)),
    # ---------------------------------------------------------------- stone (the niche, first)
    StepSpec("wire_ruled", "Diamond wire: ruled surface", "Stone", "W", "partial", "wire_cut",
             params=(("feed", "Feed", "float", 0.0, "mm/min across the stone (0 = tool)"),
                     ("faces", "Faces", "text", "all", "all, or face indices 1,4,7"),
                     ("rolls", "Wire rolls tried", "text", ",".join(str(a) for a in range(-90, 91, 10)), "deg"),
                     ("max_step_deg", "Largest joint step between rulings", "float", 30.0, "deg"),
                     ("lead", "Lead-in and lead-out rulings", "bool", True, ""),
                     ("auto_table", "Turn the table to face each face", "bool", True, "")),
             doc="Sweeps the diamond wire over the part's ruled faces (Station.ruled_path, from lithocam.ruled)."),
    StepSpec("wire_planar", "Diamond wire: planar cut", "Stone", "W|", "partial", "wire_cut"),
    StepSpec("disc_cut", "Disc saw: profile with kerf", "Stone", "D", "partial", "disc_cut",
             (("outline", "Outline", "text", "", "x,y; x,y; ... mm, CCW, work frame (empty = stock box)"),
              ("depth", "Depth", "float", 0.0, "mm along the blade (0 = through)"),
              ("stepdown", "Step-down", "float", 10.0, "mm"),
              ("bevel", "Bevel", "text", "0", "deg, one value or one per edge"),
              ("flange_diameter", "Flange diameter", "float", 100.0, "mm"),
              ("feed", "Feed", "float", 0.0, "mm/min (0 = tool / cutting data)"),
              ("rpm", "Spindle", "float", 0.0, "rev/min (0 = tool / cutting data)"),
              ("spins", "Blade spins tried", "int", 8, "per point (planner)")),
             doc="Kerf-compensated saw cuts around a part outline (lithocam.discsaw): the blade's part-side face on "
                 "each edge, full depth into convex corners, stops short of reflex corners and reports the uncut "
                 "wedge."),
    StepSpec("disc_comb", "Disc saw: comb roughing", "Stone", "Dc", "partial", "disc_cut",
             (("region", "Region", "text", "", "x,y; x,y; ... mm, CCW, work frame"),
              ("pitch", "Pitch", "float", 15.0, "mm"), ("depth", "Depth", "float", 10.0, "mm"),
              ("stepdown", "Step-down", "float", 5.0, "mm"), ("angle", "Angle", "float", 0.0, "deg"),
              ("cross_angle", "Cross angle", "text", "", "deg, empty = none"),
              ("flange_diameter", "Flange diameter", "float", 40.0, "mm"),
              ("feed", "Feed", "float", 0.0, "mm/min (0 = tool / cutting data)"),
              ("rpm", "Spindle", "float", 0.0, "rev/min (0 = tool / cutting data)"),
              ("spins", "Blade spins tried", "int", 8, "per point (planner)")),
             doc="Parallel saw kerfs across a region (and an optional cross pass) to break out stone before "
                 "finishing."),
    StepSpec("edge_profile", "Edge profile wheel", "Stone", "E", "planned", "edge_profile", _LINK),
    StepSpec("polish", "Polish (grit sequence)", "Stone", "P", "planned", "polish"),
    StepSpec("bush_hammer", "Bush-hammer texture", "Stone", "BH", "planned", "texture"),
    StepSpec("split", "Split (plug and feather)", "Stone", "Sp", "planned", "split"),
    StepSpec("waterjet", "Waterjet cut", "Stone", "WJ", "planned", "jet_cut"),
    StepSpec("calibrate", "Calibrate slab", "Stone", "Ca", "planned", "calibrate", _CUT + _LINK),
    # ---------------------------------------------------------------- 2.5-axis
    StepSpec("face", "Face", "2.5-axis", "Fa", "done", "face",
             (("depth", "Depth", "float", 1.0, "mm"),) + _CUT + _LINK),
    StepSpec("profile", "Profile (2D contour)", "2.5-axis", "Pr", "partial", "profile", _CUT + _LINK),
    StepSpec("pocket", "Pocket", "2.5-axis", "Pk", "partial", "pocket", _CUT + _LINK),
    StepSpec("slot", "Slot", "2.5-axis", "Sl", "planned", "slot", _CUT + _LINK),
    StepSpec("chamfer", "Chamfer", "2.5-axis", "Ch", "planned", "chamfer", _LINK),
    StepSpec("engrave", "Engrave / lettering", "2.5-axis", "En", "planned", "engrave", _LINK),
    StepSpec("adaptive", "Adaptive clearing", "2.5-axis", "Ad", "planned", "pocket", _CUT + _LINK),
    # ---------------------------------------------------------------- 3-axis
    StepSpec("rough_plane", "Plane roughing (z-level)", "3-axis", "R", "done", "raster", _CUT + _LINK),
    StepSpec("raster", "Raster finishing", "3-axis", "Ra", "done", "raster",
             (("stepover", "Stepover", "float", 2.0, "mm"), ("scallop", "Scallop (ball)", "float", 0.0, "mm, 0 = off"),
              ("allowance", "Stock to leave", "float", 0.0, "mm"), ("feed", "Feed", "float", 0.0, "mm/min"),
              ("rpm", "Spindle", "float", 0.0, "rev/min")) + _LINK),
    StepSpec("waterline", "Waterline", "3-axis", "Wl", "partial", "waterline", _CUT + _LINK),
    StepSpec("rest", "Rest machining", "3-axis", "Re", "planned", "rest", _CUT + _LINK),
    StepSpec("pencil", "Pencil / corners", "3-axis", "Pe", "planned", "rest", _LINK),
    StepSpec("spiral", "Spiral / radial finishing", "3-axis", "Sp", "planned", "raster", _LINK),
    # ---------------------------------------------------------------- 5-axis and robot
    StepSpec("indexed", "3+2 indexed setup", "5-axis", "3+2", "partial", "raster",
             (("tilt", "Tilt", "float", 0.0, "deg"), ("yaw", "Turntable", "float", 0.0, "deg"))),
    StepSpec("flank", "Flank (swarf)", "5-axis", "Fl", "partial", "flank",
             doc="lithocam.flank; needs a STEP part with ruled faces (OCP)"),
    StepSpec("surfacing5", "5-axis surfacing (ball, lead angle)", "5-axis", "5D", "partial", "raster",
             (("stepover", "Stepover", "float", 2.0, "mm"), ("lead", "Lead angle", "float", 15.0,
                                                              "deg, tool leans back from the surface normal"),
              ("max_tilt", "Largest tilt from the setup axis", "float", 45.0, "deg"),
              ("smooth", "Axis smoothing", "int", 7, "points"),
              ("axis_rate", "Largest axis turn", "float", 2.0, "deg per mm of travel"),
              ("allowance", "Stock to leave", "float", 0.0, "mm"),
              ("spins", "Spindle spins tried", "int", 12, "per point (planner)"),
              ("feed", "Feed", "float", 0.0, "mm/min"), ("rpm", "Spindle", "float", 0.0, "rev/min")) + _LINK,
             doc="Simultaneous 5-axis ball finishing: the ball centre follows the drop-cutter offset surface (gouge "
                 "free whatever the axis); the axis is the surface normal leaned back by the lead angle so the ball "
                 "never cuts on its zero-speed tip; the planner picks the spin about the axis per point."),
    StepSpec("rotary4", "4-axis rotary", "5-axis", "4D", "planned", "raster", _LINK),
    # ---------------------------------------------------------------- holes
    StepSpec("drill", "Drill", "Holes", "Dr", "planned", "drill",
             (("depth", "Depth", "float", 10.0, "mm"), ("peck", "Peck", "float", 0.0, "mm, 0 = off")) + _LINK),
    StepSpec("core_drill", "Core drill", "Holes", "Co", "planned", "bore", _LINK),
    StepSpec("tap", "Tap", "Holes", "Tp", "planned", "tap", _LINK),
    StepSpec("thread_mill", "Thread mill", "Holes", "Th", "planned", "thread", _LINK),
    # ---------------------------------------------------------------- finishing
    StepSpec("sand", "Sand / brush", "Finishing", "Sa", "planned", "polish"),
    StepSpec("flame", "Flame finish", "Finishing", "Fl", "planned", "flame"),
]

BY_ID: Dict[str, StepSpec] = {s.id: s for s in STEPS}
CATEGORY_COLOUR: Dict[str, str] = dict(CATEGORIES)


def default_params(spec: StepSpec) -> Dict[str, object]:
    return {k: d for k, _, _, d, _ in spec.params}


def steps_in(category: str) -> List[StepSpec]:
    return [s for s in STEPS if s.category == category]
