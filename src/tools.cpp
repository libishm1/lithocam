// Copyright 2026 The lithocam authors. SPDX-License-Identifier: Apache-2.0
#include "lithocam/tools.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <functional>
#include <set>
#include <sstream>
#include <stdexcept>

namespace lithocam {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kIn = 25.4;

enum class Model { None, Flat, Ball, Bull, Cone, FlatConservative };

struct KindInfo {
    ToolKind kind;
    const char* name;
    const char* label;
    const char* domain;
    const char* envelope;
    Model model;
    bool needs_diameter;
    const char* operations;   // space-separated families
};

// One row per kind: adding a kind means adding a row here and an enumerator in tools.hpp.
const KindInfo kKinds[] = {
    {ToolKind::FlatEndMill, "flat_end_mill", "Flat end mill / finger bit / straight or spiral router bit", "both",
     "R-FLAT", Model::Flat, true, "face profile pocket slot raster waterline rest flank"},
    {ToolKind::BallEndMill, "ball_end_mill", "Ball end mill / ball nose / core box", "both", "R-BALL", Model::Ball,
     true, "raster waterline rest pocket carve engrave"},
    {ToolKind::BullNoseMill, "bull_nose_mill", "Bull-nose (torus) mill", "both", "R-TORUS", Model::Bull, true,
     "face pocket raster waterline rest flank"},
    {ToolKind::TaperedBallMill, "tapered_ball_mill", "Tapered ball nose (conical ball)", "both", "R-TBALL",
     Model::None, true, "raster rest carve engrave"},
    {ToolKind::FaceMill, "face_mill", "Face mill / calibrating wheel / surfacing cutter", "both", "R-FLAT",
     Model::Flat, true, "face calibrate"},
    {ToolKind::GrindingDisc, "grinding_disc", "Rigid grinding disc (metal-bond pad)", "stone", "R-FLAT", Model::Flat,
     true, "grind face"},
    {ToolKind::CupWheel, "cup_wheel", "Diamond cup wheel", "stone", "R-ANNULUS", Model::FlatConservative, true,
     "grind face"},
    {ToolKind::PolishingPad, "polishing_pad", "Flexible polishing pad / edge polishing wheel", "stone", "R-COMPLIANT",
     Model::None, true, "polish"},
    {ToolKind::AbrasiveBrush, "abrasive_brush", "Abrasive brush (Frankfurt, Fickert, round)", "stone", "R-COMPLIANT",
     Model::None, true, "texture polish"},
    {ToolKind::ChamferMill, "chamfer_mill", "Chamfer mill / bevel bit", "both", "R-CONE", Model::Cone, true,
     "chamfer engrave"},
    {ToolKind::Engraver, "engraver", "V-bit engraver", "both", "R-CONE", Model::Cone, true, "engrave chamfer carve"},
    {ToolKind::CarvingBurr, "carving_burr", "Carving burr (free tip profile)", "both", "R-PROFILE-TIP", Model::None,
     true, "carve engrave texture"},
    {ToolKind::DovetailMill, "dovetail_mill", "Dovetail bit", "wood", "R-UNDERCUT", Model::None, true,
     "dovetail joinery"},
    {ToolKind::TSlotMill, "t_slot_mill", "T-slot / slot cutter", "wood", "R-UNDERCUT", Model::None, true,
     "undercut_slot slot"},
    {ToolKind::KeyholeMill, "keyhole_mill", "Keyhole bit", "wood", "R-UNDERCUT", Model::None, true, "undercut_slot"},
    {ToolKind::EdgeProfileBit, "edge_profile_bit",
     "Piloted edge-profile router bit (roundover, cove, ogee, bead, rabbet, flush trim)", "both", "R-PROFILE-SIDE",
     Model::None, true, "edge_profile profile"},
    {ToolKind::ProfileWheel, "profile_wheel", "Edge-profile wheel / insert profile cutterhead", "both",
     "R-PROFILE-SIDE", Model::None, true, "edge_profile chamfer joinery"},
    {ToolKind::Drill, "drill", "Twist or V-point drill", "both", "R-CONE", Model::Cone, true, "drill"},
    {ToolKind::FlatBottomDrill, "flat_bottom_drill", "Forstner / hinge boring / brad-point blind-hole bit", "wood",
     "R-FLAT", Model::None, true, "drill bore"},
    {ToolKind::StepDrill, "step_drill", "Countersink-counterbore combination drill", "wood", "R-STEP", Model::None,
     true, "drill bore"},
    {ToolKind::CoreDrill, "core_drill", "Core drill / plug cutter", "both", "R-ANNULUS", Model::FlatConservative,
     true, "drill bore"},
    {ToolKind::Tap, "tap", "Tap", "both", "R-PROFILE-SIDE", Model::None, true, "tap"},
    {ToolKind::ThreadMill, "thread_mill", "Thread mill", "both", "R-PROFILE-SIDE", Model::None, true, "thread"},
    {ToolKind::DiscSaw, "disc_saw", "Circular saw blade (segmented diamond or toothed)", "both", "DISC", Model::None,
     true, "disc_cut slot slab_saw"},
    {ToolKind::DiamondWire, "diamond_wire", "Diamond wire", "stone", "WIRE", Model::None, true, "wire_cut quarry"},
    {ToolKind::MultiWireSaw, "multi_wire_saw", "Multi-wire saw", "stone", "WIRE-ARRAY", Model::None, true,
     "slab_saw wire_cut"},
    {ToolKind::GangSaw, "gang_saw", "Gang saw / frame saw", "stone", "BLADE-RECIP", Model::None, false, "slab_saw"},
    {ToolKind::ChainSaw, "chain_saw", "Chain saw (quarry arm, robotic chainsaw)", "both", "CHAIN", Model::None, false,
     "quarry rough_cut"},
    {ToolKind::ChainMortiser, "chain_mortiser", "Slot-cutter chain / chain mortiser", "wood", "CHAIN", Model::None,
     false, "mortise joinery"},
    {ToolKind::BandSaw, "band_saw", "Band saw", "wood", "BLADE-BAND", Model::None, false, "ruled_cut rough_cut"},
    {ToolKind::PlanerHead, "planer_head", "Planer head (straight knives or helical inserts)", "wood", "H-CYL",
     Model::None, true, "plane"},
    {ToolKind::Waterjet, "waterjet", "Abrasive waterjet", "both", "JET", Model::None, true, "jet_cut texture"},
    {ToolKind::Knife, "knife", "Oscillating tangential or drag knife", "wood", "KNIFE", Model::None, false,
     "sheet_cut"},
    {ToolKind::BushHammer, "bush_hammer", "Bush hammer (pneumatic head or rotary plate)", "stone", "IMPACT-POINT",
     Model::None, true, "texture"},
    {ToolKind::Chisel, "chisel", "Pneumatic chisel / carving gouge", "both", "IMPACT-EDGE", Model::None, false,
     "carve texture"},
    {ToolKind::Splitter, "splitter", "Splitter (plug and feather, hydraulic wedge)", "stone", "WEDGE", Model::None,
     true, "split quarry"},
    {ToolKind::FlameTorch, "flame_torch", "Flame-finishing torch", "stone", "THERMAL", Model::None, false,
     "flame texture"},
    {ToolKind::AggregateHead, "aggregate_head", "Aggregate (angle) head", "both", "HOLDER", Model::None, false,
     "angled_drill angled_mill"},
};

const KindInfo& info(ToolKind kind) {
    for (const auto& k : kKinds)
        if (k.kind == kind) return k;
    throw std::invalid_argument("unknown tool kind");
}

std::vector<std::string> split_words(const char* s) {
    std::vector<std::string> out;
    std::istringstream in(s);
    for (std::string w; in >> w;) out.push_back(w);
    return out;
}

// ---- CSV fields: one table drives parse and format
struct Field {
    const char* name;
    bool text;   // text fields keep their spaces; numbers are trimmed
    std::function<std::string(const Tool&)> get;
    std::function<void(Tool&, const std::string&)> set;
};

std::string num(double v) {
    std::ostringstream o;
    o.precision(17);
    o << v;
    return o.str();
}

double to_double(const std::string& s, const char* field) {
    if (s.empty()) return 0.0;
    char* end = nullptr;
    const double v = std::strtod(s.c_str(), &end);
    if (end == s.c_str() || *end != '\0' || !std::isfinite(v))
        throw std::invalid_argument(std::string("field '") + field + "' is not a number: '" + s + "'");
    return v;
}

#define LC_TEXT(f) Field{#f, true, [](const Tool& t) { return t.f; }, [](Tool& t, const std::string& v) { t.f = v; }}
#define LC_NUM(f)                                                            \
    Field{#f, false, [](const Tool& t) { return num(t.f); },                 \
          [](Tool& t, const std::string& v) { t.f = to_double(v, #f); }}

const std::vector<Field>& fields() {
    static const std::vector<Field> fs = {
        LC_TEXT(id),
        LC_TEXT(name),
        Field{"kind", false, [](const Tool& t) { return tool_kind_name(t.kind); },
              [](Tool& t, const std::string& v) { t.kind = tool_kind_from_name(v); }},
        LC_TEXT(material),
        LC_TEXT(bond),
        LC_TEXT(grit),
        LC_TEXT(profile),
        LC_TEXT(helix),
        LC_TEXT(point),
        LC_TEXT(rotation),
        LC_TEXT(thread),
        LC_TEXT(source),
        LC_NUM(diameter),
        LC_NUM(corner_radius),
        LC_NUM(tip_angle),
        LC_NUM(tip_diameter),
        LC_NUM(taper_angle),
        LC_NUM(flute_length),
        LC_NUM(neck_diameter),
        LC_NUM(pilot_diameter),
        LC_NUM(spur_length),
        LC_NUM(shank_diameter),
        LC_NUM(stickout),
        LC_NUM(holder_diameter),
        LC_NUM(holder_length),
        LC_NUM(thread_pitch),
        LC_NUM(bore_diameter),
        LC_NUM(segment_height),
        LC_NUM(kerf),
        LC_NUM(length),
        LC_NUM(rake_angle),
        LC_NUM(stroke),
        LC_NUM(impact_rate),
        LC_NUM(standoff),
        LC_NUM(orifice_diameter),
        LC_NUM(pressure),
        LC_NUM(abrasive_flow),
        LC_NUM(output_angle),
        Field{"flutes", false, [](const Tool& t) { return std::to_string(t.flutes); },
              [](Tool& t, const std::string& v) {
                  const double d = to_double(v, "flutes");
                  if (d != std::floor(d) || d < 0 || d > 100000)
                      throw std::invalid_argument("field 'flutes' is not a count: '" + v + "'");
                  t.flutes = static_cast<int>(d);
              }},
        LC_NUM(spindle_rpm),
        LC_NUM(surface_speed),
        LC_NUM(feed),
        LC_NUM(plunge_feed),
        LC_NUM(max_stepdown),
        LC_NUM(max_stepover),
        LC_NUM(coolant_flow),
    };
    return fs;
}

#undef LC_TEXT
#undef LC_NUM

// Split one CSV record starting at `pos`; quoted fields may hold commas, quotes ("") and newlines.
std::vector<std::string> csv_record(const std::string& s, std::size_t& pos) {
    std::vector<std::string> out;
    std::string cur;
    bool quoted = false, any = false;
    while (pos < s.size()) {
        const char c = s[pos++];
        any = true;
        if (quoted) {
            if (c == '"') {
                if (pos < s.size() && s[pos] == '"') {
                    cur += '"';
                    ++pos;
                } else {
                    quoted = false;
                }
            } else {
                cur += c;
            }
        } else if (c == '"') {
            quoted = true;
        } else if (c == ',') {
            out.push_back(cur);
            cur.clear();
        } else if (c == '\n') {
            break;
        } else if (c != '\r') {
            cur += c;
        }
    }
    if (quoted) throw std::invalid_argument("unterminated quoted field");
    if (any) out.push_back(cur);
    return out;
}

std::string trim(const std::string& s) {
    const auto a = s.find_first_not_of(" \t");
    if (a == std::string::npos) return "";
    return s.substr(a, s.find_last_not_of(" \t") - a + 1);
}

std::string csv_quote(const std::string& s) {
    if (s.find_first_of(",\"\n\r") == std::string::npos && trim(s) == s && (s.empty() || s[0] != '#')) return s;
    std::string q = "\"";
    for (char c : s) q += c == '"' ? std::string("\"\"") : std::string(1, c);
    return q + "\"";
}

}  // namespace

std::string tool_kind_name(ToolKind kind) { return info(kind).name; }
std::string tool_kind_label(ToolKind kind) { return info(kind).label; }
std::string tool_kind_domain(ToolKind kind) { return info(kind).domain; }
std::string tool_kind_envelope(ToolKind kind) { return info(kind).envelope; }

ToolKind tool_kind_from_name(const std::string& name) {
    for (const auto& k : kKinds)
        if (name == k.name) return k.kind;
    throw std::invalid_argument("unknown tool kind '" + name + "'");
}

std::vector<ToolKind> all_tool_kinds() {
    std::vector<ToolKind> out;
    for (const auto& k : kKinds) out.push_back(k.kind);
    return out;
}

std::string validate_tool(const Tool& t) {
    if (t.id.empty()) return "id is empty";
    const double vals[] = {t.diameter,        t.corner_radius,   t.tip_angle,      t.tip_diameter,   t.taper_angle,
                           t.flute_length,    t.neck_diameter,   t.pilot_diameter, t.spur_length,    t.shank_diameter,
                           t.stickout,        t.holder_diameter, t.holder_length,  t.thread_pitch,   t.bore_diameter,
                           t.segment_height,  t.kerf,            t.length,         t.stroke,         t.impact_rate,
                           t.standoff,        t.orifice_diameter, t.pressure,      t.abrasive_flow,  t.output_angle,
                           t.spindle_rpm,     t.surface_speed,   t.feed,           t.plunge_feed,    t.max_stepdown,
                           t.max_stepover,    t.coolant_flow};
    for (double v : vals)
        if (!std::isfinite(v) || v < 0) return "a dimension or cutting value is negative or not finite";
    if (!std::isfinite(t.rake_angle) || std::fabs(t.rake_angle) >= 90) return "rake angle must be in (-90, 90)";
    if (t.flutes < 0) return "flutes is negative";
    const KindInfo& k = info(t.kind);
    if (k.needs_diameter && !(t.diameter > 0)) return "diameter must be positive";
    const double R = t.diameter / 2;
    if (t.bore_diameter > 0 && t.bore_diameter >= t.diameter) return "bore must be smaller than the diameter";
    if (t.corner_radius > R && t.diameter > 0) return "corner or profile radius exceeds diameter / 2";
    if (t.tip_angle >= 180) return "tip angle must be below 180 degrees";
    if (t.taper_angle >= 90) return "taper angle must be below 90 degrees";
    if (t.output_angle > 180) return "output angle must be at most 180 degrees";
    if (t.pilot_diameter > t.diameter && t.diameter > 0) return "pilot is larger than the cutter";
    switch (t.kind) {
        case ToolKind::BullNoseMill:
            if (!(t.corner_radius > 0)) return "bull-nose corner radius must be positive";
            break;
        case ToolKind::BallEndMill:
            if (t.corner_radius != 0 && std::fabs(t.corner_radius - R) > 1e-9 * R)
                return "a ball end mill's corner radius is diameter / 2 (or leave it 0)";
            break;
        case ToolKind::TaperedBallMill:
            if (!(t.corner_radius > 0) || t.corner_radius >= R) return "tapered-ball tip radius must be in (0, D/2)";
            if (!(t.taper_angle > 0)) return "tapered ball needs a taper angle";
            break;
        case ToolKind::DovetailMill:
            if (!(t.taper_angle > 0)) return "dovetail needs a flank angle";
            if (t.neck_diameter >= t.diameter) return "dovetail neck must be narrower than the head";
            break;
        case ToolKind::TSlotMill:
        case ToolKind::KeyholeMill:
            if (!(t.neck_diameter > 0) || t.neck_diameter >= t.diameter)
                return "neck diameter must be in (0, head diameter)";
            break;
        case ToolKind::StepDrill:
            if (!(t.pilot_diameter > 0) || t.pilot_diameter >= t.diameter)
                return "step drill: drill diameter (pilot) must be in (0, counterbore diameter)";
            break;
        case ToolKind::Tap:
        case ToolKind::ThreadMill:
            if (!(t.thread_pitch > 0)) return "thread pitch must be positive";
            break;
        default:
            break;
    }
    if (k.model == Model::Cone) {
        if (!(t.tip_angle > 0)) return "tip angle must be in (0, 180) degrees";
        if (t.tip_diameter >= t.diameter) return "tip diameter must be smaller than the diameter";
    } else if (t.tip_diameter != 0) {
        return "tip diameter applies to engravers, chamfer mills and drills only";
    }
    if (t.shank_diameter > 0 && t.holder_diameter > 0 && t.holder_diameter < t.shank_diameter)
        return "holder diameter is smaller than the shank";
    if (t.stickout > 0 && t.flute_length > t.stickout) return "flute length exceeds the stickout";
    if (!t.rotation.empty() && t.rotation != "right" && t.rotation != "left") return "rotation is 'right' or 'left'";
    return "";
}

bool has_cutter(const Tool& t) {
    const Model m = info(t.kind).model;
    if (m == Model::None) return false;
    if (m == Model::Cone && t.tip_diameter > 0) return false;
    return validate_tool(t).empty();
}

Cutter cutter_of(const Tool& t) {
    const std::string problem = validate_tool(t);
    if (!problem.empty()) throw std::invalid_argument("tool " + t.id + ": " + problem);
    const double R = t.diameter / 2;
    switch (info(t.kind).model) {
        case Model::Flat:
        case Model::FlatConservative:
            return Cutter{CutterKind::Flat, R, 0.0, 0.0};
        case Model::Ball:
            return Cutter{CutterKind::Ball, R, 0.0, 0.0};
        case Model::Bull:
            return Cutter{CutterKind::Bull, R, t.corner_radius, 0.0};
        case Model::Cone:
            if (t.tip_diameter > 0)
                throw std::invalid_argument("tool " + t.id +
                                            ": a flat-tipped cone has no drop-cutter model yet (a pointed cone would "
                                            "sit below the real tip and gouge)");
            return Cutter{CutterKind::Cone, R, 0.0, t.tip_angle / 2 * kPi / 180};
        case Model::None:
            break;
    }
    throw std::invalid_argument("tool " + t.id + ": " + tool_kind_name(t.kind) + " (" + info(t.kind).envelope +
                                ") has no drop-cutter model");
}

std::vector<std::string> tool_operations(ToolKind kind) { return split_words(info(kind).operations); }

std::vector<std::string> tool_operation_names() {
    std::vector<std::string> out;
    std::set<std::string> seen;
    for (const auto& k : kKinds)
        for (const auto& op : split_words(k.operations))
            if (seen.insert(op).second) out.push_back(op);
    return out;
}

std::vector<Tool> tools_for_operation(const ToolLibrary& lib, const std::string& operation) {
    std::vector<Tool> out;
    for (const auto& t : lib.tools) {
        const auto ops = tool_operations(t.kind);
        if (std::find(ops.begin(), ops.end(), operation) != ops.end()) out.push_back(t);
    }
    return out;
}

std::vector<Tool> tools_of_kind(const ToolLibrary& lib, ToolKind kind) {
    std::vector<Tool> out;
    for (const auto& t : lib.tools)
        if (t.kind == kind) out.push_back(t);
    return out;
}

Tool find_tool(const ToolLibrary& lib, const std::string& id) {
    for (const auto& t : lib.tools)
        if (t.id == id) return t;
    throw std::out_of_range("no tool with id '" + id + "'");
}

double holder_clearance(const Tool& t) { return t.stickout; }

ToolLibrary parse_tool_library(const std::string& text) {
    ToolLibrary lib;
    std::size_t pos = 0, counted = 0;
    int line = 1;
    std::vector<int> column;   // field index per CSV column, -1 = ignored
    bool have_header = false;
    auto fail = [&](const std::string& why) {
        throw std::runtime_error("tool library line " + std::to_string(line) + ": " + why);
    };
    while (pos < text.size()) {
        const std::size_t start = pos;
        line += static_cast<int>(std::count(text.begin() + static_cast<std::ptrdiff_t>(counted),
                                            text.begin() + static_cast<std::ptrdiff_t>(start), '\n'));
        counted = start;
        if (text[start] == '#') {
            const auto nl = text.find('\n', start);
            pos = nl == std::string::npos ? text.size() : nl + 1;
            continue;
        }
        std::vector<std::string> rec;
        try {
            rec = csv_record(text, pos);
        } catch (const std::invalid_argument& e) {
            fail(e.what());
        }
        bool blank = true;
        for (auto& f : rec)
            if (!trim(f).empty()) blank = false;
        if (blank) continue;
        if (!have_header) {
            bool has_id = false, has_kind = false;
            for (auto& h : rec) {
                const std::string name = trim(h);
                int idx = -1;
                for (std::size_t i = 0; i < fields().size(); ++i)
                    if (name == fields()[i].name) idx = static_cast<int>(i);
                column.push_back(idx);
                has_id = has_id || name == "id";
                has_kind = has_kind || name == "kind";
            }
            if (!has_id || !has_kind) fail("the header needs 'id' and 'kind' columns");
            have_header = true;
            continue;
        }
        Tool t;
        try {
            for (std::size_t i = 0; i < rec.size() && i < column.size(); ++i)
                if (column[i] >= 0) {
                    const Field& f = fields()[static_cast<std::size_t>(column[i])];
                    f.set(t, f.text ? rec[i] : trim(rec[i]));
                }
        } catch (const std::invalid_argument& e) {
            fail(e.what());
        }
        for (const auto& u : lib.tools)
            if (u.id == t.id) fail("duplicate id '" + t.id + "'");
        lib.tools.push_back(t);
    }
    return lib;
}

std::string format_tool_library(const ToolLibrary& lib) {
    std::string out =
        "# lithocam tool library: mm, degrees, rev/min, m/s, mm/min, l/min, MPa, g/s, Hz; 0 = not set\n";
    for (std::size_t i = 0; i < fields().size(); ++i) out += (i ? "," : "") + std::string(fields()[i].name);
    out += "\n";
    for (const auto& t : lib.tools) {
        for (std::size_t i = 0; i < fields().size(); ++i) out += (i ? "," : "") + csv_quote(fields()[i].get(t));
        out += "\n";
    }
    return out;
}

ToolLibrary read_tool_library(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in) throw std::runtime_error("cannot read tool library '" + path + "'");
    std::ostringstream ss;
    ss << in.rdbuf();
    return parse_tool_library(ss.str());
}

bool write_tool_library(const ToolLibrary& lib, const std::string& path) {
    std::ofstream out(path, std::ios::binary);
    if (!out) return false;
    out << format_tool_library(lib);
    return static_cast<bool>(out);
}

// ---- survey libraries (docs/TOOLING.md). Figures marked in the survey as stated by a source are used as given;
// everything else is an "example size" placeholder and the name says so.
namespace {

Tool mk(const char* id, ToolKind kind, const std::string& name, double diameter) {
    Tool t;
    t.id = id;
    t.kind = kind;
    t.name = name;
    t.diameter = diameter;
    std::string row = id;
    while (!row.empty() && std::isalpha(static_cast<unsigned char>(row.back())) && row.size() > 2) row.pop_back();
    t.source = "docs/TOOLING.md " + row;
    return t;
}

void add(ToolLibrary& lib, Tool t, const std::function<void(Tool&)>& more = {}) {
    if (more) more(t);
    lib.tools.push_back(std::move(t));
}

}  // namespace

ToolLibrary stone_tool_library() {
    using K = ToolKind;
    ToolLibrary L;
    add(L, mk("S1", K::FlatEndMill, "Diamond finger bit 3/4 in x 1-1/2 in, 50/60", 0.75 * kIn), [](Tool& t) {
        t.material = "diamond";
        t.grit = "50/60";
        t.flute_length = 1.5 * kIn;
    });
    add(L, mk("S2", K::FlatEndMill, "Drum router 2 in x 2 in, 30/40", 2 * kIn), [](Tool& t) {
        t.material = "diamond";
        t.grit = "30/40";
        t.flute_length = 2 * kIn;
    });
    add(L, mk("S3", K::CoreDrill, "Wet core bit 35 mm, 7 mm segments, L300", 35), [](Tool& t) {
        t.material = "diamond";
        t.segment_height = 7;
        t.length = 300;
        t.thread = "1/2 gas";
    });
    add(L, mk("S4", K::ProfileWheel, "Profile wheel F23 R10, 35 mm bore", 100), [](Tool& t) {
        t.material = "diamond";
        t.profile = "F23";
        t.corner_radius = 10;
        t.bore_diameter = 35;
    });
    add(L, mk("S4b", K::ProfileWheel, "Roundover wheel 115 x 50 x R3, wet only", 115), [](Tool& t) {
        t.material = "diamond";
        t.profile = "roundover";
        t.corner_radius = 3;
        t.bore_diameter = 50;
        t.spindle_rpm = 4500;   // source: 4,500-5,000 rev/min, 200-250 mm/min; lower ends kept
        t.feed = 200;
    });
    add(L, mk("S5", K::EdgeProfileBit, "Full bullnose bit for 3 cm slab, 2 in, top bearing", 2 * kIn), [](Tool& t) {
        t.material = "diamond";
        t.profile = "full_bullnose";
        t.flute_length = 30;
    });
    add(L, mk("S6", K::FaceMill, "Calibrating wheel, 20 mm segments, 50 mm arbor (example diameter)", 200),
        [](Tool& t) {
            t.material = "diamond";
            t.segment_height = 20;
            t.bore_diameter = 50;
        });
    add(L, mk("S7", K::CupWheel, "Turbo cup wheel (example size)", 125), [](Tool& t) {
        t.material = "diamond";
        t.bore_diameter = 100;
        t.thread = "M14";
    });
    add(L, mk("S8", K::PolishingPad, "Resin polishing pad set (example diameter)", 100), [](Tool& t) {
        t.material = "diamond";
        t.bond = "resin";
        t.grit = "50,100,200,400,800,1500,3000";
    });
    add(L, mk("S9", K::GrindingDisc, "Metal-bond grinding pad 30-50 grit (example diameter)", 100), [](Tool& t) {
        t.material = "diamond";
        t.bond = "metal";
        t.grit = "30-50";
    });
    add(L, mk("S10", K::PolishingPad, "Edge polishing wheel 5 in, 5 mm resin", 5 * kIn), [](Tool& t) {
        t.material = "diamond";
        t.bond = "resin";
        t.segment_height = 5;
        t.grit = "50-3000,buff";
    });
    add(L, mk("S11", K::AbrasiveBrush, "SiC abrasive brush (example size)", 100), [](Tool& t) {
        t.material = "silicon carbide";
        t.grit = "24-1000";
    });
    add(L, mk("S12", K::Engraver, "PCD V-bit 90 deg (example diameter)", 6), [](Tool& t) {
        t.material = "PCD";
        t.tip_angle = 90;
    });
    add(L, mk("S13", K::CarvingBurr, "Sintered carving burr, flame (example size)", 6), [](Tool& t) {
        t.material = "diamond";
        t.bond = "sintered";
        t.profile = "flame";
        t.flute_length = 15;
    });
    add(L, mk("S14", K::FlatEndMill, "PCD end mill, two teeth (example diameter)", 20), [](Tool& t) {
        t.material = "PCD";
        t.flutes = 2;
    });
    add(L, mk("S15", K::DiscSaw, "Bridge saw blade 400 mm, 15 mm segments", 400), [](Tool& t) {
        t.material = "diamond";
        t.kerf = 3.6;
        t.segment_height = 15;
        t.bore_diameter = 50;
    });
    add(L, mk("S16", K::GangSaw, "Frame saw with diamond segments (example size)", 0), [](Tool& t) {
        t.material = "diamond";
        t.kerf = 5;
        t.length = 3000;
    });
    add(L, mk("S17", K::DiamondWire, "Quarry diamond wire 11.5 mm, sintered beads", 11.5), [](Tool& t) {
        t.material = "diamond";
        t.bond = "sintered";
    });
    add(L, mk("S18", K::MultiWireSaw, "Multi-wire saw, 7.2 mm beads, 37 beads/m", 7.2), [](Tool& t) {
        t.material = "diamond";
        t.kerf = 7;
    });
    add(L, mk("S19", K::ChainSaw, "Quarry chain saw, 3 m arm, PCD inserts", 0), [](Tool& t) {
        t.material = "PCD on tungsten carbide";
        t.length = 3000;
    });
    add(L, mk("S20", K::Waterjet, "Abrasive waterjet (example nozzle)", 1.0), [](Tool& t) {
        t.material = "garnet";
        t.orifice_diameter = 0.35;
    });
    add(L, mk("S21", K::BushHammer, "Robot pneumatic bush hammer (example head)", 40),
        [](Tool& t) { t.material = "tungsten carbide"; });
    add(L, mk("S22", K::BushHammer, "Bush hammer plate 300 mm, 45 points of 4.7 mm", 300), [](Tool& t) {
        t.material = "tungsten carbide";
        t.flutes = 45;
    });
    add(L, mk("S23", K::Chisel, "Robot pneumatic point chisel (example edge)", 0), [](Tool& t) {
        t.profile = "point";
        t.kerf = 10;
    });
    add(L, mk("S24", K::Splitter, "Plug and feather (example 25 mm hole)", 25), [](Tool& t) { t.length = 150; });
    add(L, mk("S25", K::FlameTorch, "Flame-finishing torch (example width)", 0), [](Tool& t) { t.kerf = 300; });
    return L;
}

ToolLibrary wood_tool_library() {
    using K = ToolKind;
    ToolLibrary L;
    auto carbide = [](Tool& t) { t.material = "carbide"; };
    add(L, mk("W1", K::FlatEndMill, "Straight bit, 2 flutes (example 1/2 in)", 0.5 * kIn), [&](Tool& t) {
        carbide(t);
        t.helix = "straight";
        t.flutes = 2;
    });
    add(L, mk("W2", K::FlatEndMill, "Spiral up-cut (example 1/4 in)", 0.25 * kIn), [&](Tool& t) {
        carbide(t);
        t.helix = "up";
        t.flutes = 2;
    });
    add(L, mk("W3", K::FlatEndMill, "Spiral down-cut (example 1/4 in)", 0.25 * kIn), [&](Tool& t) {
        carbide(t);
        t.helix = "down";
        t.flutes = 2;
    });
    add(L, mk("W4", K::FlatEndMill, "Compression 3/8 in, 0.188 in up-cut zone, 0.875 in cut", 0.375 * kIn),
        [&](Tool& t) {
            carbide(t);
            t.helix = "compression";
            t.flute_length = 0.875 * kIn;
        });
    add(L, mk("W5", K::FlatEndMill, "O-flute, single flute (example 1/4 in)", 0.25 * kIn), [&](Tool& t) {
        carbide(t);
        t.helix = "o_flute";
        t.flutes = 1;
    });
    add(L, mk("W6", K::FlatEndMill, "Chipbreaker roughing spiral (example 1/2 in)", 0.5 * kIn), [&](Tool& t) {
        carbide(t);
        t.helix = "chipbreaker";
    });
    add(L, mk("W7", K::BallEndMill, "Ball nose / core box (example 1/4 in)", 0.25 * kIn), carbide);
    add(L, mk("W8", K::TaperedBallMill, "Tapered ball 1/32 in tip radius, 11 deg included", 0.25 * kIn),
        [&](Tool& t) {
            carbide(t);
            t.corner_radius = kIn / 32;
            t.taper_angle = 5.5;
        });
    add(L, mk("W9", K::Engraver, "V-bit 60 deg (example size)", 0.5 * kIn), [&](Tool& t) {
        carbide(t);
        t.tip_angle = 60;
    });
    add(L, mk("W10", K::FaceMill, "Insert spoilboard surfacing cutter 1-3/4 in, 3 wings", 1.75 * kIn), [&](Tool& t) {
        carbide(t);
        t.flutes = 3;
        t.shank_diameter = 0.5 * kIn;
    });
    add(L, mk("W11", K::DovetailMill, "Dovetail 14 deg, 1/2 in, 0.532 in flutes", 0.5 * kIn), [&](Tool& t) {
        carbide(t);
        t.taper_angle = 14;
        t.flute_length = 0.532 * kIn;
    });
    add(L, mk("W12", K::TSlotMill, "T-slot 1-1/8 in, 3/8 in neck, 5/16 in kerf", 1.125 * kIn), [&](Tool& t) {
        carbide(t);
        t.neck_diameter = 0.375 * kIn;
        t.flute_length = kIn * 5 / 16;
    });
    add(L, mk("W13", K::KeyholeMill, "Keyhole 1/2 in entry, 5/16 in slot", 0.5 * kIn), [&](Tool& t) {
        carbide(t);
        t.neck_diameter = kIn * 5 / 16;
        t.flutes = 1;
    });
    auto edge = [&](const char* id, const char* name, const char* profile, double D, double radius) {
        add(L, mk(id, K::EdgeProfileBit, name, D), [&](Tool& t) {
            carbide(t);
            t.profile = profile;
            t.corner_radius = radius;
        });
    };
    edge("W14", "Rabbeting bit with bearing set (example size)", "rabbet", 1.375 * kIn, 0);
    edge("W15", "Flush trim bit, bearing at tip (example size)", "flush_trim", 0.5 * kIn, 0);
    L.tools.back().pilot_diameter = 0.5 * kIn;
    edge("W16", "Round-over bit R 1/4 in (example size)", "roundover", 1.25 * kIn, 0.25 * kIn);
    edge("W17", "Cove bit R 1/4 in (example size)", "cove", 1.0 * kIn, 0.25 * kIn);
    edge("W18", "Roman ogee bit (example size)", "roman_ogee", 1.25 * kIn, 5 / 32.0 * kIn);
    add(L, mk("W19", K::ChamferMill, "Chamfer bit 45 deg (example size)", 1.25 * kIn), [&](Tool& t) {
        carbide(t);
        t.tip_angle = 90;
    });
    edge("W20", "Bead bit (example size)", "bead", 1.0 * kIn, 0.25 * kIn);
    add(L, mk("W21", K::ProfileWheel, "Lock-miter insert cutterhead (example size)", 120), [&](Tool& t) {
        carbide(t);
        t.profile = "lock_miter";
        t.bore_diameter = 30;
    });
    add(L, mk("W22", K::FlatBottomDrill, "Forstner bit 35 mm, 3/8 in shank", 35), [](Tool& t) {
        t.point = "forstner";
        t.shank_diameter = 0.375 * kIn;
    });
    add(L, mk("W23", K::FlatBottomDrill, "Boring pin, brad point, 10 mm shank, L57.5 (example diameter)", 8),
        [&](Tool& t) {
            carbide(t);
            t.point = "brad";
            t.shank_diameter = 10;
            t.length = 57.5;
            t.rotation = "right";
        });
    add(L, mk("W23b", K::Drill, "Through-hole dowel drill, V point (example size)", 8), [&](Tool& t) {
        carbide(t);
        t.point = "cone";
        t.tip_angle = 60;
        t.shank_diameter = 10;
    });
    add(L, mk("W24", K::FlatBottomDrill, "Hinge boring bit 35 mm, 10 mm shank", 35), [&](Tool& t) {
        carbide(t);
        t.point = "forstner";
        t.shank_diameter = 10;
    });
    add(L, mk("W25", K::StepDrill, "#12 countersink: 7/32 in drill, 7/16 in counterbore", kIn * 7 / 16),
        [](Tool& t) { t.pilot_diameter = kIn * 7 / 32; });
    add(L, mk("W26", K::CoreDrill, "Plug cutter for 3/8 in plugs (example outer diameter)", 16),
        [](Tool& t) { t.bore_diameter = 0.375 * kIn; });
    add(L, mk("W27", K::DiscSaw, "Rip blade, flat-top grind (example size)", 250), [&](Tool& t) {
        carbide(t);
        t.profile = "FTG";
        t.flutes = 24;
        t.kerf = 3.2;
        t.bore_diameter = 30;
    });
    add(L, mk("W27b", K::DiscSaw, "Flangeless saw blade 800 mm (joinery robot)", 800), carbide);
    add(L, mk("W28", K::PlanerHead, "Straight-knife planer head, 20 deg rake (example size)", 75), [](Tool& t) {
        t.material = "HSS";
        t.flutes = 3;
        t.rake_angle = 20;
    });
    add(L, mk("W29", K::PlanerHead, "Helical insert cutterhead (example size)", 75), carbide);
    add(L, mk("W30", K::AggregateHead, "90 deg angle head", 0), [](Tool& t) { t.output_angle = 90; });
    add(L, mk("W31", K::Knife, "Oscillating tangential knife (example blade)", 0), [](Tool& t) {
        t.kerf = 0.6;
        t.length = 25;
    });
    add(L, mk("W32", K::ChainSaw, "Robotic chainsaw (example bar)", 0), [](Tool& t) { t.length = 400; });
    add(L, mk("W33", K::BandSaw, "Robotic band saw (example blade)", 0), [](Tool& t) { t.kerf = 1.2; });
    add(L, mk("W34", K::ChainMortiser, "Guided slot-cutter chain, 145 mm deep", 0), [](Tool& t) { t.length = 145; });
    add(L, mk("W35", K::Chisel, "Carving gouge (example sweep)", 0), [](Tool& t) {
        t.profile = "gouge";
        t.kerf = 12;
    });
    return L;
}

}  // namespace lithocam
