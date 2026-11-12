// Copyright 2026 The lithocam authors. SPDX-License-Identifier: Apache-2.0
//
// Tool library: cutting tools with their dimensions, holder, cutting data and the operations they fit, plus the
// mapping from a tool to the drop-cutter's `Cutter` (lithocam/dropcutter.hpp).
//
// Lengths are in mm, angles in degrees, spindle speed in rev/min, feeds in mm/min. A cutting-data field of 0 means
// "not set": lithocam does not invent feeds and speeds; take them from the tool maker for the stone being cut.
//
// Drop-cutter mapping (cutter_of). The model must never sit lower than the real tool, or a path computed with it
// gouges. Each mapping is exact or conservative:
//   FlatEndMill, FaceMill        Flat,  radius D/2                                  exact
//   BallEndMill                  Ball,  radius D/2                                  exact
//   BullNoseMill                 Bull,  radius D/2, corner r                        exact
//   Drill, Engraver, ChamferMill Cone,  radius D/2, half angle = tip_angle / 2      exact when tip_diameter = 0
//   CoreDrill                    Flat,  radius D/2                                  conservative (ignores the bore)
// A cone tool with a flat tip (tip_diameter > 0) has no drop-cutter model yet: the pointed cone would sit below the
// real flat tip and gouge, so cutter_of rejects it. Taps, thread mills, disc saws, profile wheels and diamond wire
// are not vertical-axis drop-cutter tools; their operations use their own kernels (wire: lithocam.wiresim/meshsim).
//
// File format (read_tool_library / write_tool_library): CSV with a header row naming the fields below. Columns may
// come in any order; missing columns take the defaults; unknown columns are ignored; lines starting with '#' are
// comments. Text fields may be double-quoted ("" inside quotes is one quote).
#pragma once

#include <string>
#include <vector>

#include "lithocam/dropcutter.hpp"

namespace lithocam {

enum class ToolKind {
    FlatEndMill,
    BallEndMill,
    BullNoseMill,
    FaceMill,
    ChamferMill,
    Engraver,
    Drill,
    CoreDrill,
    Tap,
    ThreadMill,
    ProfileWheel,
    DiscSaw,
    DiamondWire,
};

struct Tool {
    std::string id;                  // unique key in a library, e.g. "T12"
    std::string name;                // free text shown in a GUI
    ToolKind kind = ToolKind::FlatEndMill;
    std::string material;            // cutting material, e.g. "diamond electroplated", "PCD", "carbide"
    // Geometry
    double diameter = 0.0;           // cutting diameter (disc, wheel, wire: outer diameter)
    double corner_radius = 0.0;      // BullNoseMill corner; ProfileWheel edge radius
    double tip_angle = 0.0;          // included angle of a Drill point, Engraver V or ChamferMill cone
    double tip_diameter = 0.0;       // flat at the tip of an Engraver or ChamferMill (0 = pointed)
    double flute_length = 0.0;       // cutting length (disc, wheel: rim width; wire: unused)
    double shank_diameter = 0.0;
    double stickout = 0.0;           // gauge length from the holder face to the tip
    double holder_diameter = 0.0;
    double holder_length = 0.0;
    double thread_pitch = 0.0;       // Tap, ThreadMill
    double bore_diameter = 0.0;      // CoreDrill inner diameter; DiscSaw arbor bore
    int flutes = 0;                  // 0 for segmented or plated tools
    // Cutting data (0 = not set)
    double spindle_rpm = 0.0;        // DiamondWire: wire speed in m/s goes in surface_speed instead
    double surface_speed = 0.0;      // m/s
    double feed = 0.0;               // mm/min
    double plunge_feed = 0.0;        // mm/min
    double max_stepdown = 0.0;       // axial depth per pass, mm
    double max_stepover = 0.0;       // radial width per pass, mm
    double coolant_flow = 0.0;       // l/min; diamond tools on stone run wet
};

struct ToolLibrary {
    std::vector<Tool> tools;
};

// Kind names as written in library files and shown in a GUI: "flat_end_mill", "ball_end_mill", ...
std::string tool_kind_name(ToolKind kind);
// Inverse of tool_kind_name; throws std::invalid_argument for an unknown name.
ToolKind tool_kind_from_name(const std::string& name);
// Every kind, in declaration order (for dropdowns).
std::vector<ToolKind> all_tool_kinds();

// Empty when the tool is consistent; otherwise the first problem found, in words.
std::string validate_tool(const Tool& tool);

// True when cutter_of accepts the tool.
bool has_cutter(const Tool& tool);
// Drop-cutter model of the tool (see the table above); throws std::invalid_argument when there is none.
Cutter cutter_of(const Tool& tool);

// Operation families the tool kind fits: "face", "profile", "pocket", "slot", "raster", "waterline", "rest",
// "flank", "drill", "bore", "tap", "thread", "chamfer", "engrave", "disc_cut", "profile_wheel", "wire_cut".
std::vector<std::string> tool_operations(ToolKind kind);
// Tools of the library that fit an operation family, in library order.
std::vector<Tool> tools_for_operation(const ToolLibrary& lib, const std::string& operation);
std::vector<Tool> tools_of_kind(const ToolLibrary& lib, ToolKind kind);
// Tool with the given id; throws std::out_of_range when it is not in the library.
Tool find_tool(const ToolLibrary& lib, const std::string& id);

// Deepest a tool can reach below a flat top before its holder touches it: the stickout (0 when not set).
double holder_clearance(const Tool& tool);

// CSV reading and writing (format above). read_tool_library throws std::runtime_error on an unreadable file or a
// malformed row (the message names the line); write_tool_library returns false when the file cannot be written.
ToolLibrary parse_tool_library(const std::string& csv_text);
std::string format_tool_library(const ToolLibrary& lib);
ToolLibrary read_tool_library(const std::string& path);
bool write_tool_library(const ToolLibrary& lib, const std::string& path);

// A starter library of common stone-working tool sizes (geometry only; cutting data left unset).
ToolLibrary stone_tool_library();

}  // namespace lithocam
