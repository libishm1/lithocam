// Copyright 2026 The lithocam authors. SPDX-License-Identifier: Apache-2.0
// Property tests for the tool library: drop-cutter mapping matches each tool's own lower surface and never sits below
// it, CSV round trips are exact, validation accepts generated valid tools and rejects broken ones.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <string>

#include "check.hpp"
#include "lithocam/tools.hpp"

using namespace lithocam;

namespace {

constexpr double kPi = 3.14159265358979323846;

// The real tool's lower surface above its tip at radius d, written from the tool's own fields (not from Cutter).
double tool_surface(const Tool& t, double d) {
    const double R = t.diameter / 2;
    switch (t.kind) {
        case ToolKind::BallEndMill:
            return R - std::sqrt(R * R - d * d);
        case ToolKind::BullNoseMill: {
            const double r = t.corner_radius, flat = R - r;
            if (d <= flat) return 0.0;
            const double e = d - flat;
            return r - std::sqrt(r * r - e * e);
        }
        case ToolKind::Drill:
        case ToolKind::Engraver:
        case ToolKind::ChamferMill: {
            const double tip = t.tip_diameter / 2;
            return d <= tip ? 0.0 : (d - tip) / std::tan(t.tip_angle / 2 * kPi / 180);
        }
        case ToolKind::CoreDrill:   // annulus: nothing under the bore
        case ToolKind::CupWheel:
            return d < t.bore_diameter / 2 ? INFINITY : 0.0;
        default:
            return 0.0;
    }
}

std::string random_text(lcheck::Gen& g) {
    static const std::string alphabet = "abc XYZ,\"'#;=-_0123456789\t";
    std::string s;
    const int n = g.integer(0, 14);
    for (int i = 0; i < n; ++i) s += alphabet[static_cast<std::size_t>(g.integer(0, int(alphabet.size()) - 1))];
    return s;
}

Tool random_tool(lcheck::Gen& g, int serial) {
    const auto kinds = all_tool_kinds();
    Tool t;
    t.id = "T" + std::to_string(serial);
    t.name = random_text(g);
    t.material = random_text(g);
    t.kind = kinds[static_cast<std::size_t>(g.integer(0, int(kinds.size()) - 1))];
    using K = ToolKind;
    const K k = t.kind;
    t.bond = random_text(g);
    t.grit = random_text(g);
    t.profile = random_text(g);
    t.helix = random_text(g);
    t.point = random_text(g);
    t.thread = random_text(g);
    t.source = random_text(g);
    t.rotation = g.integer(0, 2) == 0 ? "" : g.integer(0, 1) ? "left" : "right";
    t.diameter = g.uniform(0.2, 800.0);
    const double R = t.diameter / 2;
    if (k == K::BullNoseMill) t.corner_radius = g.uniform(1e-3, 1.0) * R;
    if (k == K::BallEndMill && g.integer(0, 1)) t.corner_radius = R;
    if (k == K::TaperedBallMill) t.corner_radius = g.uniform(0.01, 0.99) * R;
    if (k == K::ProfileWheel || k == K::EdgeProfileBit) t.corner_radius = g.uniform(0.0, 1.0) * R;
    if (k == K::Drill || k == K::Engraver || k == K::ChamferMill || k == K::StepDrill) {
        t.tip_angle = g.uniform(1.0, 179.0);
        if ((k == K::Engraver || k == K::ChamferMill) && g.integer(0, 2) == 0)
            t.tip_diameter = g.uniform(0.0, 0.9) * t.diameter;
    }
    if (k == K::TaperedBallMill || k == K::DovetailMill) t.taper_angle = g.uniform(0.5, 89.0);
    if (k == K::TSlotMill || k == K::KeyholeMill || k == K::DovetailMill)
        t.neck_diameter = g.uniform(0.05, 0.95) * t.diameter;
    if (k == K::EdgeProfileBit) t.pilot_diameter = g.uniform(0.0, 1.0) * t.diameter;
    if (k == K::StepDrill) t.pilot_diameter = g.uniform(0.05, 0.95) * t.diameter;
    if (k == K::FlatBottomDrill) t.spur_length = g.uniform(0.0, 5.0);
    if (k == K::Tap || k == K::ThreadMill) t.thread_pitch = g.uniform(0.2, 6.0);
    if (k == K::CoreDrill || k == K::DiscSaw || k == K::CupWheel || k == K::ProfileWheel)
        t.bore_diameter = g.uniform(0.0, 0.95) * t.diameter;
    t.segment_height = g.uniform(0.0, 30.0);
    t.kerf = g.uniform(0.0, 50.0);
    t.length = g.uniform(0.0, 5000.0);
    t.rake_angle = g.uniform(-60.0, 60.0);
    t.stroke = g.uniform(0.0, 50.0);
    t.impact_rate = g.uniform(0.0, 200.0);
    t.standoff = g.uniform(0.0, 300.0);
    t.orifice_diameter = g.uniform(0.0, 1.0);
    t.pressure = g.uniform(0.0, 600.0);
    t.abrasive_flow = g.uniform(0.0, 20.0);
    t.output_angle = g.uniform(0.0, 180.0);
    t.flute_length = g.uniform(0.0, 200.0);
    t.stickout = g.integer(0, 1) ? t.flute_length + g.uniform(0.0, 100.0) : 0.0;
    t.shank_diameter = g.uniform(0.0, 50.0);
    t.holder_diameter = t.shank_diameter + g.uniform(0.0, 60.0);
    t.holder_length = g.uniform(0.0, 120.0);
    t.flutes = g.integer(0, 12);
    t.spindle_rpm = g.uniform(0.0, 30000.0);
    t.surface_speed = g.uniform(0.0, 40.0);
    t.feed = g.uniform(0.0, 20000.0);
    t.plunge_feed = g.uniform(0.0, 5000.0);
    t.max_stepdown = g.uniform(0.0, 20.0);
    t.max_stepover = g.uniform(0.0, 20.0);
    t.coolant_flow = g.uniform(0.0, 50.0);
    return t;
}

// format_tool_library is injective (numbers at 17 significant digits, text quoted reversibly), so equal formats
// mean equal tools in every field; a few fields are also compared directly.
bool same(const Tool& a, const Tool& b) {
    ToolLibrary la, lb;
    la.tools.push_back(a);
    lb.tools.push_back(b);
    return format_tool_library(la) == format_tool_library(lb) && a.id == b.id && a.name == b.name &&
           a.kind == b.kind && a.diameter == b.diameter && a.rake_angle == b.rake_angle && a.flutes == b.flutes;
}

template <class E, class F>
bool throws(F f) {
    try {
        f();
    } catch (const E&) {
        return true;
    }
    return false;
}

}  // namespace

int main() {
    lcheck::Gen g(20261112);
    long cases = 0;

    // Kind names round trip; every kind lists at least one operation.
    for (ToolKind k : all_tool_kinds()) {
        lcheck::expect(tool_kind_from_name(tool_kind_name(k)) == k, "kind_name_roundtrip", tool_kind_name(k));
        lcheck::expect(!tool_operations(k).empty(), "kind_has_operations", tool_kind_name(k));
        ++cases;
    }
    lcheck::expect(throws<std::invalid_argument>([] { tool_kind_from_name("spoon"); }), "unknown_kind_throws");

    // Generated tools are valid; the drop-cutter model matches the real lower surface where it is exact and is never
    // below it: f_model(d) <= f_real(d), so the model's tip bound p.z - f(d) is never lower than the real one.
    for (int i = 0; i < 20000; ++i, ++cases) {
        const Tool t = random_tool(g, i);
        const std::string problem = validate_tool(t);
        lcheck::expect(problem.empty(), "generated_tool_valid", t.id + " " + tool_kind_name(t.kind) + ": " + problem);
        if (!has_cutter(t)) {
            lcheck::expect(throws<std::invalid_argument>([&] { cutter_of(t); }), "no_cutter_throws",
                           tool_kind_name(t.kind));
            continue;
        }
        const Cutter c = cutter_of(t);
        lcheck::expect(std::fabs(c.radius - t.diameter / 2) <= 1e-12 * t.diameter, "cutter_radius", t.id);
        const bool exact = t.kind != ToolKind::CoreDrill && t.kind != ToolKind::CupWheel;
        for (int j = 0; j < 16; ++j) {
            const double d = g.uniform(0.0, 1.0) * c.radius;
            const double model = profile(c, d), real = tool_surface(t, d);
            if (exact)
                lcheck::expect(std::fabs(model - real) <= 1e-9 * t.diameter + 1e-9 * std::fabs(real), "cutter_exact",
                               t.id + " " + tool_kind_name(t.kind) + " d=" + std::to_string(d));
            else
                lcheck::expect(model <= real, "cutter_conservative", t.id + " d=" + std::to_string(d));
        }
    }

    // A flat-tipped cone has no model (the pointed cone would gouge); cutter_of refuses it.
    {
        Tool v;
        v.id = "V";
        v.kind = ToolKind::Engraver;
        v.diameter = 10;
        v.tip_angle = 60;
        v.tip_diameter = 0.5;
        lcheck::expect(validate_tool(v).empty() && !has_cutter(v), "flat_tip_cone_no_model");
        lcheck::expect(throws<std::invalid_argument>([&] { cutter_of(v); }), "flat_tip_cone_throws");
        ++cases;
    }

    // Validation rejects broken tools.
    {
        Tool base;
        base.id = "X";
        base.kind = ToolKind::BullNoseMill;
        base.diameter = 20;
        base.corner_radius = 4;
        lcheck::expect(validate_tool(base).empty(), "base_valid", validate_tool(base));
        auto bad = [&](auto mutate, const char* what) {
            Tool t = base;
            mutate(t);
            lcheck::expect(!validate_tool(t).empty(), "rejects", what);
            ++cases;
        };
        bad([](Tool& t) { t.id.clear(); }, "empty id");
        bad([](Tool& t) { t.diameter = 0; }, "zero diameter");
        bad([](Tool& t) { t.diameter = -5; }, "negative diameter");
        bad([](Tool& t) { t.corner_radius = 11; }, "corner > R");
        bad([](Tool& t) { t.corner_radius = 0; }, "bull without corner");
        bad([](Tool& t) { t.feed = NAN; }, "NaN feed");
        bad([](Tool& t) { t.flutes = -1; }, "negative flutes");
        bad([](Tool& t) { t.tip_diameter = 1; }, "tip diameter on a mill");
        bad([](Tool& t) { t.stickout = 10, t.flute_length = 20; }, "flute longer than stickout");
        bad([](Tool& t) { t.shank_diameter = 20, t.holder_diameter = 10; }, "holder thinner than shank");
        bad([](Tool& t) { t.kind = ToolKind::Drill; }, "drill without tip angle");
        bad([](Tool& t) { t.kind = ToolKind::Tap; }, "tap without pitch");
        bad([](Tool& t) { t.kind = ToolKind::CoreDrill, t.corner_radius = 0, t.bore_diameter = 20; }, "core bore = D");
    }

    // CSV round trip is exact, including text with commas, quotes, '#' and tabs.
    for (int i = 0; i < 300; ++i, ++cases) {
        ToolLibrary lib;
        const int n = g.integer(0, 30);
        for (int j = 0; j < n; ++j) lib.tools.push_back(random_tool(g, j));
        const ToolLibrary back = parse_tool_library(format_tool_library(lib));
        bool ok = back.tools.size() == lib.tools.size();
        for (std::size_t j = 0; ok && j < lib.tools.size(); ++j) ok = same(lib.tools[j], back.tools[j]);
        lcheck::expect(ok, "csv_roundtrip", "library " + std::to_string(i));
    }

    // Hand-written files: any column order, missing columns default, unknown columns and comments ignored.
    {
        const std::string csv =
            "# a comment\n"
            "kind,diameter,id,vendor_code,corner_radius\n"
            "\n"
            "bull_nose_mill, 25 ,R25,abc-1,5\n"
            "# another comment\n"
            "ball_end_mill,10,B10,,\n";
        const ToolLibrary lib = parse_tool_library(csv);
        lcheck::expect(lib.tools.size() == 2, "hand_csv_count", std::to_string(lib.tools.size()));
        if (lib.tools.size() == 2) {
            lcheck::expect(lib.tools[0].id == "R25" && lib.tools[0].diameter == 25 && lib.tools[0].corner_radius == 5,
                           "hand_csv_values");
            lcheck::expect(lib.tools[1].kind == ToolKind::BallEndMill && lib.tools[1].feed == 0, "hand_csv_defaults");
        }
        lcheck::expect(throws<std::runtime_error>([] { parse_tool_library("id,kind\nA,spoon\n"); }),
                       "bad_kind_row_throws");
        lcheck::expect(throws<std::runtime_error>([] { parse_tool_library("id,kind,diameter\nA,drill,ten\n"); }),
                       "bad_number_row_throws");
        lcheck::expect(throws<std::runtime_error>([] { parse_tool_library("id,kind\nA,drill\nA,drill\n"); }),
                       "duplicate_id_throws");
        lcheck::expect(throws<std::runtime_error>([] { parse_tool_library("name,diameter\nx,1\n"); }),
                       "header_needs_id_kind");
        bool names_line = false;
        try {
            parse_tool_library("# c\nid,kind\nA,drill\n\nB,spoon\n");
        } catch (const std::runtime_error& e) {
            names_line = std::string(e.what()).find("line 5") != std::string::npos;
        }
        lcheck::expect(names_line, "error_names_line");
        cases += 6;
    }

    // Kind metadata is complete; every operation family is reachable from some kind.
    {
        const auto ops = tool_operation_names();
        for (ToolKind k : all_tool_kinds()) {
            lcheck::expect(!tool_kind_label(k).empty() && !tool_kind_envelope(k).empty(), "kind_metadata",
                           tool_kind_name(k));
            const std::string d = tool_kind_domain(k);
            lcheck::expect(d == "stone" || d == "wood" || d == "both", "kind_domain", tool_kind_name(k));
            for (const auto& op : tool_operations(k))
                lcheck::expect(std::find(ops.begin(), ops.end(), op) != ops.end(), "op_listed", op);
            ++cases;
        }
    }

    // The survey libraries: every row S1..S25 and W1..W35 is present, every tool is valid, ids are unique across both,
    // the source names the row, and lookups agree.
    {
        ToolLibrary all = stone_tool_library();
        for (const auto& t : wood_tool_library().tools) all.tools.push_back(t);
        for (int i = 1; i <= 25; ++i)
            lcheck::expect(!throws<std::out_of_range>([&] { find_tool(all, "S" + std::to_string(i)); }), "row",
                           "S" + std::to_string(i));
        for (int i = 1; i <= 35; ++i)
            lcheck::expect(!throws<std::out_of_range>([&] { find_tool(all, "W" + std::to_string(i)); }), "row",
                           "W" + std::to_string(i));
        for (std::size_t i = 0; i < all.tools.size(); ++i) {
            const Tool& t = all.tools[i];
            lcheck::expect(validate_tool(t).empty(), "survey_valid", t.id + ": " + validate_tool(t));
            lcheck::expect(find_tool(all, t.id).name == t.name, "unique_id", t.id);
            lcheck::expect(t.source.rfind("docs/TOOLING.md ", 0) == 0 && t.id.rfind(t.source.substr(16), 0) == 0,
                           "source_names_row", t.id + " " + t.source);
            ++cases;
        }
        lcheck::expect(throws<std::out_of_range>([&] { find_tool(all, "nope"); }), "find_missing_throws");
        for (const auto& op : tool_operation_names())
            for (const auto& t : tools_for_operation(all, op)) {
                const auto tops = tool_operations(t.kind);
                lcheck::expect(std::find(tops.begin(), tops.end(), op) != tops.end(), "for_operation_consistent",
                               t.id + " " + op);
            }
        int with_cutter = 0;
        for (const auto& t : all.tools) with_cutter += has_cutter(t) ? 1 : 0;
        lcheck::expect(with_cutter >= 20, "survey_cutters", std::to_string(with_cutter));
        const ToolLibrary back = parse_tool_library(format_tool_library(all));
        bool ok = back.tools.size() == all.tools.size();
        for (std::size_t j = 0; ok && j < all.tools.size(); ++j) ok = same(all.tools[j], back.tools[j]);
        lcheck::expect(ok, "survey_csv_roundtrip");
        std::printf("survey library: %zu tools, %d with a drop-cutter model\n", all.tools.size(), with_cutter);
    }

    return lcheck::finish("tools", cases);
}
