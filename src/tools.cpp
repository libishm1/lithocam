// Copyright 2026 The lithocam authors. SPDX-License-Identifier: Apache-2.0
#include "lithocam/tools.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <functional>
#include <sstream>
#include <stdexcept>

namespace lithocam {

namespace {

constexpr double kPi = 3.14159265358979323846;

enum class Model { None, Flat, Ball, Bull, Cone, FlatConservative };

struct KindInfo {
    ToolKind kind;
    const char* name;
    Model model;
    const char* operations;   // space-separated families
};

// One row per kind: adding a kind means adding a row here and an enumerator in tools.hpp.
const KindInfo kKinds[] = {
    {ToolKind::FlatEndMill, "flat_end_mill", Model::Flat, "face profile pocket slot raster waterline rest flank"},
    {ToolKind::BallEndMill, "ball_end_mill", Model::Ball, "raster waterline rest pocket engrave"},
    {ToolKind::BullNoseMill, "bull_nose_mill", Model::Bull, "face pocket raster waterline rest flank"},
    {ToolKind::FaceMill, "face_mill", Model::Flat, "face"},
    {ToolKind::ChamferMill, "chamfer_mill", Model::Cone, "chamfer engrave"},
    {ToolKind::Engraver, "engraver", Model::Cone, "engrave chamfer"},
    {ToolKind::Drill, "drill", Model::Cone, "drill"},
    {ToolKind::CoreDrill, "core_drill", Model::FlatConservative, "drill bore"},
    {ToolKind::Tap, "tap", Model::None, "tap"},
    {ToolKind::ThreadMill, "thread_mill", Model::None, "thread"},
    {ToolKind::ProfileWheel, "profile_wheel", Model::None, "profile_wheel chamfer"},
    {ToolKind::DiscSaw, "disc_saw", Model::None, "disc_cut slot"},
    {ToolKind::DiamondWire, "diamond_wire", Model::None, "wire_cut"},
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

#define LC_TEXT(f) Field{#f, [](const Tool& t) { return t.f; }, [](Tool& t, const std::string& v) { t.f = v; }}
#define LC_NUM(f)                                                     \
    Field{#f, [](const Tool& t) { return num(t.f); },                 \
          [](Tool& t, const std::string& v) { t.f = to_double(v, #f); }}

const std::vector<Field>& fields() {
    static const std::vector<Field> fs = {
        LC_TEXT(id),
        LC_TEXT(name),
        Field{"kind", [](const Tool& t) { return tool_kind_name(t.kind); },
              [](Tool& t, const std::string& v) { t.kind = tool_kind_from_name(v); }},
        LC_TEXT(material),
        LC_NUM(diameter),
        LC_NUM(corner_radius),
        LC_NUM(tip_angle),
        LC_NUM(tip_diameter),
        LC_NUM(flute_length),
        LC_NUM(shank_diameter),
        LC_NUM(stickout),
        LC_NUM(holder_diameter),
        LC_NUM(holder_length),
        LC_NUM(thread_pitch),
        LC_NUM(bore_diameter),
        Field{"flutes", [](const Tool& t) { return std::to_string(t.flutes); },
              [](Tool& t, const std::string& v) {
                  const double d = to_double(v, "flutes");
                  if (d != std::floor(d) || d < 0 || d > 1000)
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
    if (s.find_first_of(",\"\n\r") == std::string::npos && trim(s) == s) return s;
    std::string q = "\"";
    for (char c : s) q += c == '"' ? std::string("\"\"") : std::string(1, c);
    return q + "\"";
}

bool cone_kind(ToolKind k) { return info(k).model == Model::Cone; }

}  // namespace

std::string tool_kind_name(ToolKind kind) { return info(kind).name; }

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
    const double vals[] = {t.diameter,       t.corner_radius, t.tip_angle,     t.tip_diameter, t.flute_length,
                           t.shank_diameter, t.stickout,      t.holder_diameter, t.holder_length, t.thread_pitch,
                           t.bore_diameter,  t.spindle_rpm,   t.surface_speed, t.feed,         t.plunge_feed,
                           t.max_stepdown,   t.max_stepover,  t.coolant_flow};
    for (double v : vals)
        if (!std::isfinite(v) || v < 0) return "a dimension or cutting value is negative or not finite";
    if (t.flutes < 0) return "flutes is negative";
    if (!(t.diameter > 0)) return "diameter must be positive";
    const double R = t.diameter / 2;
    switch (t.kind) {
        case ToolKind::BullNoseMill:
            if (!(t.corner_radius > 0) || t.corner_radius > R)
                return "bull-nose corner radius must be in (0, diameter / 2]";
            break;
        case ToolKind::BallEndMill:
            if (t.corner_radius != 0 && std::fabs(t.corner_radius - R) > 1e-9 * R)
                return "a ball end mill's corner radius is diameter / 2 (or leave it 0)";
            break;
        case ToolKind::ProfileWheel:
            if (t.corner_radius > R) return "profile radius exceeds the wheel radius";
            break;
        case ToolKind::Tap:
        case ToolKind::ThreadMill:
            if (!(t.thread_pitch > 0)) return "thread pitch must be positive";
            break;
        case ToolKind::CoreDrill:
            if (t.bore_diameter >= t.diameter) return "core drill bore must be smaller than its diameter";
            break;
        case ToolKind::DiscSaw:
            if (t.bore_diameter >= t.diameter) return "arbor bore must be smaller than the disc diameter";
            break;
        default:
            break;
    }
    if (cone_kind(t.kind)) {
        if (!(t.tip_angle > 0 && t.tip_angle < 180)) return "tip angle must be in (0, 180) degrees";
        if (t.tip_diameter >= t.diameter) return "tip diameter must be smaller than the diameter";
    } else if (t.tip_diameter != 0) {
        return "tip diameter applies to engravers and chamfer mills only";
    }
    if (t.shank_diameter > 0 && t.holder_diameter > 0 && t.holder_diameter < t.shank_diameter)
        return "holder diameter is smaller than the shank";
    if (t.stickout > 0 && t.flute_length > t.stickout) return "flute length exceeds the stickout";
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
    throw std::invalid_argument("tool " + t.id + ": " + tool_kind_name(t.kind) +
                                " is not a vertical-axis drop-cutter tool");
}

std::vector<std::string> tool_operations(ToolKind kind) { return split_words(info(kind).operations); }

std::vector<Tool> tools_for_operation(const ToolLibrary& lib, const std::string& operation) {
    std::vector<Tool> out;
    for (const auto& t : lib.tools)
        for (const auto& op : tool_operations(t.kind))
            if (op == operation) {
                out.push_back(t);
                break;
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
    std::size_t pos = 0;
    int line = 0;
    std::vector<int> column;   // field index per CSV column, -1 = ignored
    bool have_header = false;
    std::size_t counted = 0;   // newlines before `counted` are in `line`
    while (pos < text.size()) {
        const std::size_t start = pos;
        line += static_cast<int>(std::count(text.begin() + static_cast<long>(counted),
                                            text.begin() + static_cast<long>(start), '\n'));
        counted = start;
        if (line == 0) line = 1;
        if (text[start] == '#') {
            const auto nl = text.find('\n', start);
            pos = nl == std::string::npos ? text.size() : nl + 1;
            continue;
        }
        std::vector<std::string> rec;
        try {
            rec = csv_record(text, pos);
        } catch (const std::invalid_argument& e) {
            throw std::runtime_error("tool library line " + std::to_string(line) + ": " + e.what());
        }
        bool blank = true;
        for (auto& f : rec)
            if (!trim(f).empty()) blank = false;
        if (blank) continue;
        if (!have_header) {
            for (auto& h : rec) {
                const std::string name = trim(h);
                int idx = -1;
                for (std::size_t i = 0; i < fields().size(); ++i)
                    if (name == fields()[i].name) idx = static_cast<int>(i);
                column.push_back(idx);
            }
            bool has_id = false, has_kind = false;
            for (int c : column) {
                if (c >= 0 && std::string(fields()[c].name) == "id") has_id = true;
                if (c >= 0 && std::string(fields()[c].name) == "kind") has_kind = true;
            }
            if (!has_id || !has_kind) throw std::runtime_error("tool library header needs 'id' and 'kind' columns");
            have_header = true;
            continue;
        }
        Tool t;
        try {
            for (std::size_t i = 0; i < rec.size() && i < column.size(); ++i)
                if (column[i] >= 0) {
                    const bool text_field = std::string(fields()[column[i]].name) == "name" ||
                                            std::string(fields()[column[i]].name) == "material";
                    fields()[column[i]].set(t, text_field ? rec[i] : trim(rec[i]));
                }
        } catch (const std::invalid_argument& e) {
            throw std::runtime_error("tool library line " + std::to_string(line) + ": " + e.what());
        }
        for (const auto& u : lib.tools)
            if (u.id == t.id) throw std::runtime_error("tool library line " + std::to_string(line) +
                                                       ": duplicate id '" + t.id + "'");
        lib.tools.push_back(t);
    }
    return lib;
}

std::string format_tool_library(const ToolLibrary& lib) {
    std::string out = "# lithocam tool library: mm, degrees, rev/min, m/s, mm/min, l/min; 0 = not set\n";
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

ToolLibrary stone_tool_library() {
    auto mill = [](std::string id, std::string name, ToolKind k, double D, double flute, double shank, double stick) {
        Tool t;
        t.id = std::move(id);
        t.name = std::move(name);
        t.kind = k;
        t.material = "diamond";
        t.diameter = D;
        t.flute_length = flute;
        t.shank_diameter = shank;
        t.stickout = stick;
        return t;
    };
    ToolLibrary lib;
    lib.tools.push_back(mill("B6", "ball D6", ToolKind::BallEndMill, 6, 20, 6, 40));
    lib.tools.push_back(mill("B10", "ball D10", ToolKind::BallEndMill, 10, 30, 10, 60));
    lib.tools.push_back(mill("B20", "ball D20", ToolKind::BallEndMill, 20, 40, 20, 90));
    lib.tools.push_back(mill("F12", "finger D12", ToolKind::FlatEndMill, 12, 35, 12, 70));
    lib.tools.push_back(mill("F20", "finger D20", ToolKind::FlatEndMill, 20, 120, 20, 150));
    Tool bull = mill("R25", "bull D25 r5", ToolKind::BullNoseMill, 25, 40, 25, 90);
    bull.corner_radius = 5;
    lib.tools.push_back(bull);
    Tool face = mill("FM80", "face mill D80", ToolKind::FaceMill, 80, 10, 0, 60);
    lib.tools.push_back(face);
    Tool v = mill("V90", "V engraver 90 deg", ToolKind::Engraver, 12, 8, 12, 40);
    v.tip_angle = 90;
    lib.tools.push_back(v);
    Tool ch = mill("C45", "chamfer 90 deg D20", ToolKind::ChamferMill, 20, 10, 20, 50);
    ch.tip_angle = 90;
    lib.tools.push_back(ch);
    Tool dr = mill("D8", "drill D8", ToolKind::Drill, 8, 60, 8, 90);
    dr.tip_angle = 118;
    dr.material = "carbide-tipped";
    lib.tools.push_back(dr);
    Tool core = mill("K30", "core drill D30", ToolKind::CoreDrill, 30, 150, 0, 180);
    core.bore_diameter = 24;
    lib.tools.push_back(core);
    Tool wheel = mill("W100", "profile wheel D100 r10", ToolKind::ProfileWheel, 100, 30, 0, 0);
    wheel.corner_radius = 10;
    lib.tools.push_back(wheel);
    Tool disc = mill("S400", "disc saw D400", ToolKind::DiscSaw, 400, 10, 0, 0);
    disc.bore_diameter = 50;
    lib.tools.push_back(disc);
    Tool wire = mill("WIRE7", "diamond wire 7 mm beads", ToolKind::DiamondWire, 7, 0, 0, 0);
    lib.tools.push_back(wire);
    return lib;
}

}  // namespace lithocam
