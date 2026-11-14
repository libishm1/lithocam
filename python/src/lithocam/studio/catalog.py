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
# how a pass enters the stone (cam.Entry): plunge, a ramp along the pass, or a helix; the ramp angle, helix radius and
# helix pitch are example values, to be set for the tool and the stone
_ENTRY = (("entry", "Entry", "choice:plunge|ramp|helix", "plunge", "plunge: vertical; ramp: along the pass; helix: a circle"),
          ("ramp_angle", "Ramp angle", "float", 3.0, "deg (ramp entry; an example value)"),
          ("helix_radius", "Helix radius", "float", 0.0, "mm, 0 = the tool radius (helix entry)"),
          ("helix_pitch", "Helix pitch", "float", 1.0, "mm per turn at most (helix entry; an example value)"))

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
    StepSpec("wire_ruled", "Diamond wire: ruled surface", "Stone", "W", "done", "wire_cut",
             params=(("feed", "Feed", "float", 0.0, "mm/min across the stone (0 = tool)"),
                     ("faces", "Faces", "text", "all", "all, or face indices 1,4,7"),
                     ("rolls", "Wire rolls tried", "text", ",".join(str(a) for a in range(-90, 91, 10)), "deg"),
                     ("max_step_deg", "Largest joint step between rulings", "float", 30.0, "deg"),
                     ("lead", "Lead-in and lead-out rulings", "bool", True, ""),
                     ("auto_table", "Turn the table to face each face", "bool", True, ""),
                     ("table_window_deg", "Table window (E1)", "float", 30.0,
                      "deg either side of the facing angle; the table driven as E1 only"),
                     ("table_step_deg", "Table angle step (E1)", "float", 10.0,
                      "deg between candidate table angles; E1 only"),
                     ("table_weight", "Table step weight (E1)", "float", 0.1,
                      "cost of a table step relative to an arm joint step of the same angle; E1 only")),
             doc="Sweeps the diamond wire over the part's ruled faces (Station.ruled_path, from lithocam.ruled)."),
    StepSpec("wire_planar", "Diamond wire: planar cut", "Stone", "W|", "partial", "wire_cut"),
    StepSpec("disc_cut", "Disc saw: profile with kerf", "Stone", "D", "done", "disc_cut",
             (("outline", "Outline", "text", "", "x,y; x,y; ... mm, CCW, work frame (empty = stock box)"),
              ("depth", "Depth", "float", 0.0, "mm along the blade (0 = through)"),
              ("stepdown", "Step-down", "float", 10.0, "mm"),
              ("bevel", "Bevel", "text", "0", "deg, one value or one per edge"),
              ("flange_diameter", "Flange diameter", "float", 100.0, "mm"),
              ("feed", "Feed", "float", 0.0, "mm/min (0 = tool / cutting data)"),
              ("rpm", "Spindle", "float", 0.0, "rev/min (0 = tool / cutting data)"),
              ("spins", "Blade spins tried", "int", 8, "per point (planner)"),
              ("approach_deg", "Arm approach from vertical", "float", 45.0, "deg (angle head only)")),
             doc="Kerf-compensated saw cuts around a part outline (lithocam.discsaw): the blade's part-side face on "
                 "each edge, full depth into convex corners, stops short of reflex corners and reports the uncut "
                 "wedge."),
    StepSpec("disc_comb", "Disc saw: comb roughing", "Stone", "Dc", "done", "disc_cut",
             (("region", "Region", "text", "", "x,y; x,y; ... mm, CCW, work frame"),
              ("pitch", "Pitch", "float", 15.0, "mm"), ("depth", "Depth", "float", 10.0, "mm"),
              ("stepdown", "Step-down", "float", 5.0, "mm"), ("angle", "Angle", "float", 0.0, "deg"),
              ("cross_angle", "Cross angle", "text", "", "deg, empty = none"),
              ("flange_diameter", "Flange diameter", "float", 40.0, "mm"),
              ("feed", "Feed", "float", 0.0, "mm/min (0 = tool / cutting data)"),
              ("rpm", "Spindle", "float", 0.0, "rev/min (0 = tool / cutting data)"),
              ("spins", "Blade spins tried", "int", 8, "per point (planner)"),
              ("approach_deg", "Arm approach from vertical", "float", 45.0, "deg (angle head only)")),
             doc="Parallel saw kerfs across a region (and an optional cross pass) to break out stone before "
                 "finishing."),
    StepSpec("disc_ruled", "Disc saw: ruled faces", "Stone", "Dr", "done", "disc_cut",
             (("faces", "Faces", "text", "all", "all, or face indices 1,4,7 of Station.ruled_path"),
              ("labels", "Face labels", "text", "", "labels of the faces to cut, e.g. wire, flank (empty = all)"),
              ("tol", "Left-over stock allowed", "float", 0.5, "mm between the flat blade and the face"),
              ("intrusion_tol", "Intrusion allowed", "float", 0.0, "mm into the part (0 = none)"),
              ("depth", "Depth", "float", 0.0, "mm along the blade (0 = the deepest safe pass of each ruling)"),
              ("stepdown", "Step-down", "float", 10.0, "mm"),
              ("flange_diameter", "Flange diameter", "float", 100.0, "mm"),
              ("feed", "Feed", "float", 0.0, "mm/min (0 = tool / cutting data)"),
              ("rpm", "Spindle", "float", 0.0, "rev/min (0 = tool / cutting data)"),
              ("spins", "Blade spins tried", "int", 8, "per point (planner)"),
              ("approach_deg", "Arm approach from vertical", "float", 45.0, "deg (angle head only)")),
             doc="Saw cuts along the rulings of the part's ruled faces (lithocam.discsaw.ruled_disc_cuts). The log "
                 "says per face how many rulings can be cut, the depth, the twist and the tilt; a ruling that cannot "
                 "be cut is left out with its reason. The blade spin is chosen by the planner."),
    StepSpec("edge_profile", "Edge profile wheel", "Stone", "E", "planned", "edge_profile", _LINK),
    StepSpec("bush_hammer", "Bush-hammer texture", "Stone", "BH", "planned", "texture"),
    StepSpec("split", "Split (plug and feather)", "Stone", "Sp", "planned", "split"),
    StepSpec("waterjet", "Waterjet cut", "Stone", "WJ", "planned", "jet_cut"),
    StepSpec("calibrate", "Calibrate slab", "Stone", "Ca", "planned", "calibrate", _CUT + _LINK),
    # ---------------------------------------------------------------- 2.5-axis
    StepSpec("face", "Face", "2.5-axis", "Fa", "done", "face",
             (("depth", "Depth", "float", 1.0, "mm"),) + _CUT + _ENTRY + _LINK,
             doc="Zig-zag at the stock top minus the depth. The entry is a plunge, a ramp or a helix."),
    StepSpec("profile", "Profile (2D contour)", "2.5-axis", "Pr", "partial", "profile", _CUT + _LINK),
    StepSpec("pocket", "Pocket", "2.5-axis", "Pk", "partial", "pocket", _CUT + _LINK),
    StepSpec("slot", "Slot", "2.5-axis", "Sl", "planned", "slot", _CUT + _LINK),
    StepSpec("chamfer", "Chamfer", "2.5-axis", "Ch", "done", "chamfer",
             (("width", "Chamfer width", "float", 0.0, "mm on the top face, required; at most the tool radius"),
              ("paths", "Edges", "text", "", "x,y; x,y | x,y; ... mm (empty = the outline of the part's highest flat top)"),
              ("max_slope_deg", "Flat top within", "float", 1.0, "deg of +z (the outline search)"),
              ("step", "Sample spacing", "float", 0.5, "mm"),
              ("feed", "Feed", "float", 0.0, "mm/min (0 = tool / cutting data)"),
              ("rpm", "Spindle", "float", 0.0, "rev/min (0 = tool / cutting data)")) + _LINK,
             doc="A V-bit or chamfer bit runs along the top edge of the part (or the edges you give) at the chamfer "
                 "width; the tip sits on the wall below the edge. Given edges follow a non-flat top."),
    StepSpec("engrave", "Engrave / lettering", "2.5-axis", "En", "done", "engrave",
             (("paths", "Paths", "text", "", "x,y; x,y | x,y; ... mm, polylines separated by |"),
              ("width", "Groove width", "float", 0.0, "mm at the surface; sets the depth from the V angle (0 = use depth)"),
              ("depth", "Depth", "float", 0.0, "mm below the top surface (used when the width is 0)"),
              ("text", "Text", "text", "", "not available: lithocam ships no single-stroke font; give the strokes as paths"),
              ("step", "Sample spacing", "float", 0.5, "mm"),
              ("feed", "Feed", "float", 0.0, "mm/min (0 = tool / cutting data)"),
              ("rpm", "Spindle", "float", 0.0, "rev/min (0 = tool / cutting data)")) + _LINK,
             doc="V-bit engraving along polylines. The tip follows the part's top surface at the depth, or at the "
                 "depth that gives the groove width. Letters need a single-stroke font, which is not shipped: "
                 "give the strokes as paths."),
    StepSpec("flat_land", "Flat lands (horizontal areas)", "2.5-axis", "FL", "done", "pocket",
             (("max_slope_deg", "Flat within", "float", 1.0, "deg of +z"),
              ("stepover", "Stepover", "float", 0.0, "mm, 0 = 0.7 x the tool diameter"),
              ("allowance", "Stock to leave", "float", 0.0, "mm"),
              ("cell", "Grid cell", "float", 0.0, "mm, 0 = from the tool"),
              ("feed", "Feed", "float", 0.0, "mm/min (0 = tool / cutting data)"),
              ("rpm", "Spindle", "float", 0.0, "rev/min (0 = tool / cutting data)")) + _ENTRY + _LINK,
             doc="Finishes the horizontal faces of the part with a flat or bull end mill: a zig-zag at the height of "
                 "each land, highest first. The tool stays a radius from walls that rise above the land and may "
                 "overhang an open edge."),
    StepSpec("adaptive", "Adaptive clearing", "2.5-axis", "Ad", "planned", "pocket", _CUT + _LINK),
    # ---------------------------------------------------------------- 3-axis
    StepSpec("rough_plane", "Plane roughing (z-level)", "3-axis", "R", "done", "raster", _CUT + _ENTRY + _LINK,
             doc="Z-level roughing over the stock. The entry is a plunge, a ramp or a helix."),
    StepSpec("raster", "Raster finishing", "3-axis", "Ra", "done", "raster",
             (("stepover", "Stepover", "float", 2.0, "mm"), ("scallop", "Scallop (ball)", "float", 0.0, "mm, 0 = off"),
              ("allowance", "Stock to leave", "float", 0.0, "mm"), ("feed", "Feed", "float", 0.0, "mm/min"),
              ("rpm", "Spindle", "float", 0.0, "rev/min"), ("boundary", "Boundary", "text", "", "xmin,ymin,xmax,ymax mm, or a polygon x,y; x,y; x,y (empty = whole part)")) + _LINK),
    StepSpec("waterline", "Waterline", "3-axis", "Wl", "done", "waterline",
             (("stepdown", "Step-down", "float", 1.0, "mm between levels"),
              ("allowance", "Stock to leave", "float", 0.0, "mm"),
              ("direction", "Direction", "choice:climb|conventional", "climb", "clockwise spindle (M3)"),
              ("feed", "Feed", "float", 0.0, "mm/min (0 = tool)"), ("rpm", "Spindle", "float", 0.0,
                                                                     "rev/min (0 = tool)"), ("boundary", "Boundary", "text", "", "xmin,ymin,xmax,ymax mm, or a polygon x,y; x,y; x,y (empty = whole part)")) + _LINK,
             doc="Constant-Z finishing loops on the drop-cutter contour of each level, from the part's top down to "
                 "its floor; every point is within the path tolerance of the contour and never below the "
                 "drop-cutter height."),
    StepSpec("steep_shallow", "Steep and shallow finishing", "3-axis", "SS", "done", "waterline",
             (("threshold_deg", "Steep from", "float", 40.0, "deg of slope"),
              ("overlap", "Overlap", "float", 1.0, "mm, each region grown by this"),
              ("stepover", "Stepover", "float", 2.0, "mm (raster; waterline gap on the shallowest steep slope)"),
              ("stepdown", "Step-down", "float", 0.0, "mm, 0 = stepover x tan(threshold)"),
              ("allowance", "Stock to leave", "float", 0.0, "mm"),
              ("feed", "Feed", "float", 0.0, "mm/min (0 = tool)"),
              ("rpm", "Spindle", "float", 0.0, "rev/min (0 = tool)"), ("boundary", "Boundary", "text", "", "xmin,ymin,xmax,ymax mm, or a polygon x,y; x,y; x,y (empty = whole part)")) + _LINK,
             doc="Splits the part by the slope of the drop-cutter height field: waterline loops on steep ground, "
                 "raster along x on shallow ground, with an overlap band cut by both."),
    StepSpec("rest", "Rest machining", "3-axis", "Re", "done", "rest",
             (("previous_tool_radius", "Previous tool radius", "float", 0.0,
               "mm, 0 = the stock simulation, else a Z-map of the earlier milling steps"),
              ("threshold", "Leftover threshold", "float", 0.05, "mm the previous tool left"),
              ("mode", "Mode", "choice:raster|waterline", "raster", ""),
              ("stepover", "Stepover", "float", 1.0, "mm (raster)"),
              ("stepdown", "Step-down", "float", 1.0, "mm (waterline)"),
              ("overlap", "Overlap", "float", 0.5, "mm, the region is grown by this"),
              ("allowance", "Stock to leave", "float", 0.0, "mm"),
              ("direction", "Direction", "choice:climb|conventional", "climb", "clockwise spindle (M3)"),
              ("cell", "Grid cell", "float", 0.0, "mm, 0 = from the tool"),
              ("leftover_cell", "Z-map cell", "float", 1.0, "mm, the Z-map of the earlier milling steps"),
              ("low_links", "Low links", "bool", False, "nearest run first; links 2 mm above the stone left, "
                                                        "not at the clearance plane"),
              ("feed", "Feed", "float", 0.0, "mm/min (0 = tool / cutting data)"),
              ("rpm", "Spindle", "float", 0.0, "rev/min (0 = tool / cutting data)")) + _LINK,
             doc="Cuts only where the earlier steps left more than the threshold (docs/OPERATIONS_HOLES_REST.md). "
                 "The material comes from the last stock simulation (the state before this step), else from a "
                 "Z-map of the earlier milling steps, else from the previous tool radius; the log says which."),
    StepSpec("pencil", "Pencil / corners", "3-axis", "Pe", "done", "raster",
             (("min_angle_deg", "Crease angle", "float", 15.0, "deg of slope change that counts as a crease"),
              ("min_length", "Shortest path", "float", 0.0, "mm"),
              ("allowance", "Stock to leave", "float", 0.0, "mm"),
              ("cell", "Grid cell", "float", 0.0, "mm, 0 = from the tool"),
              ("feed", "Feed", "float", 0.0, "mm/min (0 = tool / cutting data)"),
              ("rpm", "Spindle", "float", 0.0, "rev/min (0 = tool / cutting data)")) + _LINK,
             doc="Traces the concave creases a ball-end mill touches at two or more points (ball tools only)."),
    StepSpec("adaptive_raster", "Raster, curvature-adaptive stepover", "3-axis", "Ar", "done", "raster",
             (("scallop", "Scallop", "float", 0.0, "mm, required: the largest scallop height"),
              ("adaptive", "Adapt to curvature", "bool", True, ""),
              ("stepover", "Constant stepover", "float", 0.0, "mm, 0 = from the scallop"),
              ("window", "Rim window", "float", 0.0, "mm, 0 = the tool radius"),
              ("min_stepover", "Smallest stepover", "float", 0.0, "mm"),
              ("allowance", "Stock to leave", "float", 0.0, "mm"),
              ("cell", "Grid cell", "float", 0.0, "mm, 0 = from the tool"),
              ("feed", "Feed", "float", 0.0, "mm/min (0 = tool / cutting data)"),
              ("rpm", "Spindle", "float", 0.0, "rev/min (0 = tool / cutting data)")) + _LINK,
             doc="Raster along x with the cross-feed spacing set from the local curvature so the scallop stays at "
                 "or below the limit (ball-end mills only)."),
    StepSpec("spiral", "Spiral / radial finishing", "3-axis", "Sp", "planned", "raster", _LINK),
    # ---------------------------------------------------------------- 5-axis and robot
    StepSpec("indexed", "3+2 indexed setup", "5-axis", "3+2", "partial", "raster",
             (("tilt", "Tilt", "float", 0.0, "deg"), ("yaw", "Turntable", "float", 0.0, "deg"))),
    StepSpec("flank", "Flank (swarf)", "5-axis", "Fl", "partial", "flank",
             doc="lithocam.flank; needs a STEP part with ruled faces (OCP)"),
    StepSpec("surfacing5", "5-axis surfacing (ball, lead angle)", "5-axis", "5D", "done", "raster",
             (("stepover", "Stepover", "float", 2.0, "mm"), ("lead", "Lead angle", "float", 15.0,
                                                              "deg, tool leans back from the surface normal"),
              ("max_tilt", "Largest tilt from the setup axis", "float", 45.0, "deg"),
              ("smooth", "Axis smoothing", "int", 7, "points"),
              ("axis_rate", "Largest axis turn", "float", 2.0, "deg per mm of travel"),
              ("allowance", "Stock to leave", "float", 0.0, "mm"),
              ("spins", "Spindle spins tried", "int", 12, "per point (planner)"),
              ("feed", "Feed", "float", 0.0, "mm/min"), ("rpm", "Spindle", "float", 0.0, "rev/min"),
              ("boundary", "Boundary", "text", "", "xmin,ymin,xmax,ymax mm, or a polygon x,y; x,y; x,y (empty = whole part)")) + _LINK,
             doc="Simultaneous 5-axis ball finishing: the ball centre follows the drop-cutter offset surface (gouge "
                 "free whatever the axis); the axis is the surface normal leaned back by the lead angle so the ball "
                 "never cuts on its zero-speed tip; the planner picks the spin about the axis per point."),
    StepSpec("rotary4", "4-axis rotary", "5-axis", "4D", "planned", "raster", _LINK),
    # ---------------------------------------------------------------- holes
    StepSpec("drill", "Drill", "Holes", "Dr", "done", "drill",
             (("points", "Hole points", "text", "", "x,y[,z_top]; ... mm (empty = the vertical holes found in the part)"),
              ("depth", "Depth", "float", 10.0, "mm below each hole top (0 = the detected depth)"),
              ("peck", "Peck", "float", 0.0, "mm, 0 = one plunge"),
              ("peck_mode", "Peck mode", "choice:full|chip", "full", "full: out to the R plane; chip: a short break"),
              ("chip_retract", "Chip-break retract", "float", 0.0, "mm (chip mode)"),
              ("retract_height", "R plane above the stock", "float", 2.0, "mm"),
              ("dwell", "Dwell at the bottom", "float", 0.0, "s"), ("diameter_range", "Hole diameters", "text", "", "min,max mm of the holes found in the part (empty = any)"),
              ("feed", "Feed", "float", 0.0, "mm/min (0 = tool / cutting data)"), ("rpm", "Spindle", "float", 0.0,
                                                                  "rev/min (0 = tool / cutting data)"),
              ("clearance", "Clearance height", "float", 20.0, "mm above stock")),
             doc="Drill cycle with pecks (docs/OPERATIONS_HOLES_REST.md). Peck, dwell and speeds are yours to set; "
                 "none is built in."),
    StepSpec("core_drill", "Core drill", "Holes", "Co", "done", "drill",
             (("points", "Hole points", "text", "", "x,y[,z_top]; ... mm (empty = the vertical holes found in the part)"),
              ("depth", "Depth", "float", 10.0, "mm below each hole top (0 = the detected depth)"),
              ("peck", "Peck", "float", 0.0, "mm, required: every peck ends in a clearing retract"),
              ("retract_height", "R plane above the stock", "float", 2.0, "mm"),
              ("dwell", "Dwell at the bottom", "float", 0.0, "s"),
              ("orbit_radius", "Entry orbit radius", "float", 0.0, "mm, 0 = off (a helix on the first peck)"),
              ("orbit_turns", "Entry orbit turns", "float", 1.0, ""),
              ("tolerance", "Path tolerance", "float", 0.01, "mm"), ("diameter_range", "Hole diameters", "text", "", "min,max mm of the holes found in the part (empty = any)"),
              ("feed", "Feed", "float", 0.0, "mm/min (0 = tool / cutting data)"), ("rpm", "Spindle", "float", 0.0,
                                                                  "rev/min (0 = tool / cutting data)"),
              ("clearance", "Clearance height", "float", 20.0, "mm above stock")),
             doc="Stone core drilling: a clearing retract per peck, optional entry orbit."),
    StepSpec("tap", "Tap", "Holes", "Tp", "done", "tap",
             (("points", "Hole points", "text", "", "x,y[,z_top]; ... mm (empty = the vertical holes found in the part)"),
              ("depth", "Depth", "float", 10.0, "mm below each hole top (0 = the detected depth)"),
              ("pitch", "Pitch", "float", 0.0, "mm per revolution, required"),
              ("rpm", "Spindle", "float", 0.0, "rev/min, required; the feed is pitch x rpm"),
              ("dwell", "Dwell at the bottom", "float", 0.0, "s"),
              ("retract_height", "R plane above the stock", "float", 2.0, "mm"), ("diameter_range", "Hole diameters", "text", "", "min,max mm of the holes found in the part (empty = any)"),
              ("clearance", "Clearance height", "float", 20.0, "mm above stock")),
             doc="Rigid tapping: in at pitch x rpm, then out with the spindle reversed. A robot cell needs a spindle "
                 "reverse output in its cell I/O; G-code posts M4 and M3."),
    StepSpec("bore", "Bore (helical)", "Holes", "Bo", "done", "pocket",
             (("points", "Hole points", "text", "", "x,y[,z_top]; ... mm (empty = the vertical holes found in the part)"),
              ("diameter", "Hole diameter", "float", 0.0, "mm (0 = the detected diameter)"),
              ("depth", "Depth", "float", 10.0, "mm below each hole top (0 = the detected depth)"),
              ("pitch", "Descent per turn", "float", 1.0, "mm"),
              ("tolerance", "Path tolerance", "float", 0.01, "mm chord sag"),
              ("retract_height", "R plane above the stock", "float", 2.0, "mm"), ("diameter_range", "Hole diameters", "text", "", "min,max mm of the holes found in the part (empty = any)"),
              ("feed", "Feed", "float", 0.0, "mm/min (0 = tool / cutting data)"), ("rpm", "Spindle", "float", 0.0,
                                                                  "rev/min (0 = tool / cutting data)"),
              ("clearance", "Clearance height", "float", 20.0, "mm above stock")),
             doc="An end mill bores a hole larger than itself on a helix, then one full circle at the bottom."),
    StepSpec("thread_mill", "Thread mill", "Holes", "Th", "planned", "thread", _LINK),
    # ---------------------------------------------------------------- finishing
    StepSpec("polish", "Polish (grit sequence)", "Finishing", "Po", "done", "polish",
             (("grits", "Grit sequence", "text", "",
               "e.g. 50-100, 200-400, 800-1500 (empty = the sourced sequence of the material)"),
              ("pressure", "Pad pressure", "float", 0.0, "MPa (N/mm2), required; a simulation attribute, no force "
                                                          "command is sent"),
              ("feed", "Feed", "float", 0.0, "mm/min, required (no sourced polishing feed)"),
              ("rpm", "Pad speed", "float", 0.0, "rev/min (0 = the sourced band of each grit)"),
              ("rpm_pick", "Place in the band", "float", 0.5, "0 = low end, 1 = high end of a sourced rpm band"),
              ("water", "Water", "text", "", "text posted per grit, e.g. flood (empty = from a sourced row)"),
              ("overlap", "Pad overlap", "float", 0.5, "ratio"),
              ("stepover", "Stepover", "float", 0.0, "mm, 0 = from the pad radius and the overlap"),
              ("pattern", "Pattern", "choice:zigzag|spiral", "zigzag", ""),
              ("angle_deg", "Lane angle", "float", 0.0, "deg"),
              ("cross", "Cross every second grit", "bool", False, ""),
              ("cross_angle_deg", "Cross angle", "float", 90.0, "deg"),
              ("grit_tools", "Pad per grit", "text", "", "grit=tool; grit=tool (empty = the step's tool for all)"),
              ("region", "Region", "text", "", "umin,umax,vmin,vmax mm in plan (empty = the whole part)"),
              ("spins", "Spindle spins tried", "int", 12, "per point (planner)"),
              ("clearance", "Clearance height", "float", 20.0, "mm above stock"),
              ("tolerance", "Path tolerance", "float", 0.01, "mm")),
             doc="One pad pass per grit with a pad change before each (lithocam.polish). The pad axis follows the "
                 "surface normal like 5-axis surfacing; the planner chooses the spin. Pressure and grit go to the "
                 "program as comments; force control stays off."),
    StepSpec("sand", "Sand / brush", "Finishing", "Sa", "planned", "polish"),
    StepSpec("flame", "Flame finish", "Finishing", "Fl", "planned", "flame"),
]

BY_ID: Dict[str, StepSpec] = {s.id: s for s in STEPS}
# generator steps whose tool needs no drop-cutter model (drills, taps, pads): any library tool of the family fits
NO_CUTTER_MODEL = {"drill", "core_drill", "tap", "polish"}
CATEGORY_COLOUR: Dict[str, str] = dict(CATEGORIES)


def default_params(spec: StepSpec) -> Dict[str, object]:
    return {k: d for k, _, _, d, _ in spec.params}


def steps_in(category: str) -> List[StepSpec]:
    return [s for s in STEPS if s.category == category]
