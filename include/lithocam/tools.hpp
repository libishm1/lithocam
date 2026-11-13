// Copyright 2026 The lithocam authors. SPDX-License-Identifier: Apache-2.0
//
// Tool library: cutting tools for stone and wood with their dimensions, holder, cutting data and the operations they
// fit, plus the mapping from a tool to the drop-cutter's `Cutter` (lithocam/dropcutter.hpp). The kinds cover the 60
// tool types of the stone and wood tooling survey (docs/TOOLING.md, rows S1-S25 and W1-W35).
//
// Units: lengths in mm, angles in degrees, spindle speed in rev/min, surface speed in m/s, feeds in mm/min,
// coolant in l/min, pressure in MPa, abrasive flow in g/s, impact rate in Hz. A cutting-data field of 0 means
// "not set": lithocam does not invent feeds and speeds; take them from the tool maker for the material being cut.
//
// Drop-cutter mapping (cutter_of). The model must never sit lower than the real tool, or a path computed with it
// gouges. Each mapping is exact or conservative:
//   FlatEndMill, FaceMill, GrindingDisc   Flat, radius D/2                               exact
//   BallEndMill                          Ball, radius D/2                               exact
//   BullNoseMill                         Bull, radius D/2, corner r                     exact
//   Drill, Engraver, ChamferMill         Cone, radius D/2, half angle = tip_angle / 2   exact when tip_diameter = 0
//                                        Profile (flat of diameter tip_diameter, then the flank) exact when tip_diameter > 0
//   TaperedBallMill                      Profile: tip radius corner_radius (arc), optional flat of diameter
//                                        tip_diameter, flank taper_angle from the axis up to D/2   exact when the
//                                        flute covers the cone, conservative when it is shorter
//   CarvingBurr                          Profile from `profile` (flame, ball, cone, cylinder) with radius D/2 and head
//                                        length flute_length; exact for that nominal shape
//   CoreDrill, CupWheel                  Flat, radius D/2                               conservative (ignores the bore)
// Every other kind has no drop-cutter model yet, and cutter_of rejects it:
//   - burrs with another profile name, and step drills (a stepped profile is not convex);
//   - flat-bottom drills: the centre spur projects below the rim;
//   - undercut tools (dovetail, T-slot, keyhole) need full-profile collision;
//   - compliant pads and brushes need a force model;
//   - side-cutting profile tools, discs, wires, chains, blades, jets, knives, impact tools, wedges, torches and
//     aggregate heads are not vertical-axis drop-cutter tools.
//
// File format (read_tool_library / write_tool_library): CSV with a header row naming the fields of `Tool`. Columns
// may come in any order; missing columns take the defaults; unknown columns are ignored; lines starting with '#'
// are comments. Text fields may be double-quoted ("" inside quotes is one quote).
#pragma once

#include <string>
#include <vector>

#include "lithocam/dropcutter.hpp"

namespace lithocam {

enum class ToolKind {
    // vertical-axis milling and grinding
    FlatEndMill,       // finger bit, drum router, straight/spiral/compression/O-flute router bit, PCD end mill
    BallEndMill,       // ball nose, core box
    BullNoseMill,
    TaperedBallMill,   // conical ball (carving)
    FaceMill,          // face mill, calibrating wheel, spoilboard surfacing cutter
    GrindingDisc,      // rigid metal-bond pad or disc
    CupWheel,
    PolishingPad,      // flexible resin pad, edge polishing wheel
    AbrasiveBrush,     // Frankfurt, Fickert, round brush
    ChamferMill,
    Engraver,          // V-bit
    CarvingBurr,       // sintered or plated burr with a free tip profile
    DovetailMill,
    TSlotMill,
    KeyholeMill,
    EdgeProfileBit,    // piloted router bit: roundover, cove, ogee, bead, rabbet, flush trim
    ProfileWheel,      // stone edge-profile wheel, insert profile cutterhead
    // holes
    Drill,             // twist or V-point
    FlatBottomDrill,   // Forstner, hinge boring, brad-point blind-hole bit
    StepDrill,         // countersink-counterbore combination
    CoreDrill,         // core bit, plug cutter
    Tap,
    ThreadMill,
    // sawing and cutting
    DiscSaw,           // segmented diamond blade, toothed wood blade
    DiamondWire,
    MultiWireSaw,
    GangSaw,           // frame saw with reciprocating blades
    ChainSaw,          // quarry arm or robotic chainsaw
    ChainMortiser,     // slot-cutter chain
    BandSaw,
    PlanerHead,        // straight-knife or helical insert cutterhead (horizontal axis)
    Waterjet,          // abrasive waterjet
    Knife,             // oscillating tangential or drag knife
    // impact, splitting, thermal
    BushHammer,        // pneumatic head or rotary plate
    Chisel,            // pneumatic chisel, carving gouge
    Splitter,          // plug and feather, hydraulic wedge
    FlameTorch,
    // holders
    AggregateHead,     // angle head re-orienting another tool
};

struct Tool {
    std::string id;                  // unique key in a library, e.g. "T12"
    std::string name;                // free text shown in a GUI
    ToolKind kind = ToolKind::FlatEndMill;
    std::string material;            // cutting material: "diamond", "PCD", "carbide", "HSS", ...
    std::string bond;                // diamond bond: "sintered", "electroplated", "vacuum-brazed", "resin", "metal"
    std::string grit;                // grit or grit sequence, e.g. "50/60" or "50,100,200,400,800,1500,3000"
    std::string profile;             // named edge or tip profile, e.g. "full_bullnose", "ogee", "roundover", "flame"
    std::string helix;               // "up", "down", "compression", "straight", "o_flute", "chipbreaker"
    std::string point;               // drill point: "cone", "brad", "forstner", "flat"
    std::string rotation;            // "right" (default when empty) or "left"
    std::string thread;              // tool mounting thread: "1/2 gas", "5/8-11", "M14"
    std::string source;              // where the data comes from (survey row, catalogue)
    // Geometry
    double diameter = 0.0;           // cutting diameter (disc, wheel, wire bead, jet focusing tube, hole for a splitter)
    double corner_radius = 0.0;      // bull corner; tapered-ball tip radius; profile radius of an edge tool
    double tip_angle = 0.0;          // included angle of a Drill point, Engraver V, ChamferMill or StepDrill countersink
    double tip_diameter = 0.0;       // flat at the tip of an Engraver or ChamferMill (0 = pointed)
    double taper_angle = 0.0;        // side angle of a tapered ball or a dovetail flank, from the axis
    double flute_length = 0.0;       // cutting length (disc, wheel: rim width; T-slot: cutting height)
    double neck_diameter = 0.0;      // T-slot, keyhole and dovetail neck
    double pilot_diameter = 0.0;     // bearing of a piloted bit; drill of a step drill
    double spur_length = 0.0;        // centre spur below the rim of a flat-bottom drill
    double shank_diameter = 0.0;
    double stickout = 0.0;           // gauge length from the holder face to the tip
    double holder_diameter = 0.0;
    double holder_length = 0.0;
    double thread_pitch = 0.0;       // Tap, ThreadMill
    double bore_diameter = 0.0;      // core drill or cup wheel inner diameter; disc, wheel or head arbor bore
    double segment_height = 0.0;     // diamond segment or resin layer height
    double kerf = 0.0;               // cut width (saw, wire, chain, jet, knife, chisel edge, torch)
    double length = 0.0;             // blade, bar, arm or chain length; overall length of a boring pin
    double rake_angle = 0.0;         // rake or hook angle; may be negative
    double stroke = 0.0;             // reciprocating or impact stroke
    double impact_rate = 0.0;        // impacts or oscillations per second
    double standoff = 0.0;           // jet or torch standoff
    double orifice_diameter = 0.0;   // waterjet orifice
    double pressure = 0.0;           // waterjet pressure
    double abrasive_flow = 0.0;      // waterjet abrasive flow
    double output_angle = 0.0;       // aggregate head output axis to the spindle axis
    int flutes = 0;                  // flutes, teeth, segments, knives, inserts, impact points or wires
    // Cutting data (0 = not set)
    double spindle_rpm = 0.0;
    double surface_speed = 0.0;      // m/s (wire and chain speed)
    double feed = 0.0;               // mm/min
    double plunge_feed = 0.0;        // mm/min
    double max_stepdown = 0.0;       // axial depth per pass, mm
    double max_stepover = 0.0;       // radial width per pass, mm
    double coolant_flow = 0.0;       // l/min; diamond tools on stone run wet
};

struct ToolLibrary {
    std::vector<Tool> tools;
};

// Kind names as written in library files: "flat_end_mill", "ball_end_mill", ...
std::string tool_kind_name(ToolKind kind);
// Inverse of tool_kind_name; throws std::invalid_argument for an unknown name.
ToolKind tool_kind_from_name(const std::string& name);
// Every kind, in declaration order (for dropdowns).
std::vector<ToolKind> all_tool_kinds();
// Human-readable label, e.g. "Piloted edge-profile router bit".
std::string tool_kind_label(ToolKind kind);
// "stone", "wood" or "both".
std::string tool_kind_domain(ToolKind kind);
// Cutting-envelope class from docs/TOOLING.md: "R-FLAT", "R-BALL", "R-TORUS", "R-CONE", "R-TBALL", "R-ANNULUS",
// "R-PROFILE-SIDE", "R-PROFILE-TIP", "R-UNDERCUT", "R-STEP", "R-COMPLIANT", "H-CYL", "DISC", "WIRE", "WIRE-ARRAY",
// "BLADE-RECIP", "BLADE-BAND", "CHAIN", "KNIFE", "IMPACT-POINT", "IMPACT-EDGE", "WEDGE", "JET", "THERMAL", "HOLDER".
std::string tool_kind_envelope(ToolKind kind);

// Empty when the tool is consistent; otherwise the first problem found, in words.
std::string validate_tool(const Tool& tool);

// True when cutter_of accepts the tool.
bool has_cutter(const Tool& tool);
// Drop-cutter model of the tool (see the table above); throws std::invalid_argument when there is none.
Cutter cutter_of(const Tool& tool);
// Saw-slot drop-cutter of a DiscSaw: radius diameter / 2, thickness kerf (the slot width), `heading` the cut direction
// (atan2(dy, dx) of the pass). Throws std::invalid_argument for another kind or a missing diameter or kerf.
Cutter disc_cutter_of(const Tool& tool, double heading);

// Operation families the tool kind fits, e.g. "face", "pocket", "raster", "drill", "edge_profile", "polish",
// "slab_saw", "wire_cut", "plane", "texture", "split". tool_operation_names lists every family any kind uses.
std::vector<std::string> tool_operations(ToolKind kind);
std::vector<std::string> tool_operation_names();
// Tools of the library that fit an operation family, in library order.
std::vector<Tool> tools_for_operation(const ToolLibrary& lib, const std::string& operation);
std::vector<Tool> tools_of_kind(const ToolLibrary& lib, ToolKind kind);
// Tool with the given id; throws std::out_of_range when it is not in the library.
Tool find_tool(const ToolLibrary& lib, const std::string& id);

// Deepest a tool can reach below a flat top before its holder touches it: the stickout (0 when not set).
double holder_clearance(const Tool& tool);

// CSV reading and writing (format above). parse/read throw std::runtime_error on an unreadable file or a malformed
// row (the message names the line); write_tool_library returns false when the file cannot be written.
ToolLibrary parse_tool_library(const std::string& csv_text);
std::string format_tool_library(const ToolLibrary& lib);
ToolLibrary read_tool_library(const std::string& path);
bool write_tool_library(const ToolLibrary& lib, const std::string& path);

// One tool per row of the tooling survey (docs/TOOLING.md), ids S1..S25 and W1..W35 (plus a few "b" variants).
// Sizes and cutting data are those the survey's sources state; where a source states none, the name says
// "example size" and the geometry is a plausible placeholder to edit.
ToolLibrary stone_tool_library();
ToolLibrary wood_tool_library();

}  // namespace lithocam
