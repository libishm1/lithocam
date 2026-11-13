// Copyright 2026 The lithocam authors. SPDX-License-Identifier: Apache-2.0
//
// Property tests for the thin-disc (saw slot) drop-cutter: a disc of radius R and thickness w, axis horizontal and
// perpendicular to the cut direction (heading).
//   closed forms:   horizontal plane, vertex on the rim, face contact (plane rising along the axis), rim contact
//                   (plane rising along the cut), extruded profile (2D case), vertex outside the slab;
//   properties:     no gouge (triangle sampling and rim sampling) and tight contact against an independent
//                   maximiser (tests/test_dropcutter_props.hpp), 12,000 random triangles;
//   invariances:    rotating the mesh about the tip and the heading together keeps the height; a thicker disc is never
//                   lower; lifting the mesh lifts the height; the grid index equals brute force;
//   path drop:      adaptive_line with a disc whose heading follows the pass.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <vector>

#include "lithocam/pathdrop.hpp"
#include "lithocam/tools.hpp"
#include "test_dropcutter_props.hpp"

using namespace lithocam;
using props::kPi;

namespace {

Tri rot(const Tri& t, double x, double y, double th) {
    auto r = [&](const P3& p) {
        double dx = p.x - x, dy = p.y - y;
        return P3{x + dx * std::cos(th) - dy * std::sin(th), y + dx * std::sin(th) + dy * std::cos(th), p.z};
    };
    return {r(t.a), r(t.b), r(t.c)};
}

std::vector<Tri> terrain(int G, int seed) {
    std::vector<Tri> tris;
    auto h = [&](int i, int j) { return 3 * std::sin(0.3 * i + seed) * std::cos(0.2 * j) + 0.1 * ((i * 7 + j * 3) % 5); };
    for (int i = 0; i < G; ++i)
        for (int j = 0; j < G; ++j) {
            P3 p00{double(i), double(j), h(i, j)}, p10{double(i + 1), double(j), h(i + 1, j)};
            P3 p01{double(i), double(j + 1), h(i, j + 1)}, p11{double(i + 1), double(j + 1), h(i + 1, j + 1)};
            tris.push_back({p00, p10, p11});
            tris.push_back({p00, p11, p01});
        }
    return tris;
}

}  // namespace

int main() {
    const auto t0 = std::chrono::steady_clock::now();
    lcheck::Gen g(20261114);
    long cases = 0;

    // ---- closed forms (heading 0: u = x, l = y)
    {
        const double R = 5.0, w = 2.0;
        Cutter d = disc_cutter(R, w, 0.0);
        Tri plane{{-80, -80, 3}, {80, -80, 3}, {0, 80, 3}};
        lcheck::expect(std::fabs(drop_triangle(d, plane, 0, 0) - 3) < 1e-12, "disc_plane");
        // vertex at u = 3, l = 0.5, z = 10: tip = 10 - (R - sqrt(R^2 - 9)) = 9
        Tri spike{{3, 0.5, 10}, {40, 0.5, -100}, {3, 60, -100}};
        lcheck::expect(std::fabs(drop_triangle(d, spike, 0, 0) - 9.0) < 1e-12, "disc_vertex_on_rim");
        // the same vertex outside the slab (l = 1.5 > w / 2) with the whole triangle on that side: no contact
        Tri outside{{3, 1.5, 10}, {40, 1.5, -100}, {3, 60, -100}};
        lcheck::expect(std::isinf(drop_triangle(d, outside, 0, 0)), "disc_outside_slab");
        // face contact: z = 10 + 2 l; at u = 0 the best point is l = +w/2: tip = 12
        Tri face{{-80, -80, -150}, {80, -80, -150}, {0, 80, 170}};
        lcheck::expect(std::fabs(drop_triangle(d, face, 0, 0) - 12.0) < 1e-9, "disc_face_contact");
        // rim contact: z = 10 + b u; tip = 10 + R (sqrt(1 + b^2) - 1)
        const double b = 0.5;
        const double rim_tip = 10 + R * (std::sqrt(1 + b * b) - 1);
        Tri rim{{-80, -80, 10 - 80 * b}, {80, -80, 10 + 80 * b}, {0, 80, 10}};
        lcheck::expect(std::fabs(drop_triangle(d, rim, 0, 0) - rim_tip) < 1e-9, "disc_rim_contact");
        // heading pi/2 turns u into y: the same rim plane written along y
        Tri rim_y{{-80, -80, 10 - 80 * b}, {-80, 80, 10 + 80 * b}, {80, 0, 10}};
        Cutter dy = disc_cutter(R, w, kPi / 2);
        lcheck::expect(std::fabs(drop_triangle(dy, rim_y, 0, 0) - rim_tip) < 1e-9, "disc_heading_quarter_turn");
        // a disc has no rotational profile; invalid dimensions are refused
        bool threw = false;
        try { profile(d, 1.0); } catch (const std::invalid_argument&) { threw = true; }
        lcheck::expect(threw, "disc_profile_throws");
        threw = false;
        try { disc_cutter(5.0, 0.0, 0.0); } catch (const std::invalid_argument&) { threw = true; }
        lcheck::expect(threw, "disc_zero_thickness_throws");
        cases += 9;
        // extruded profile along l (2D case): tip = max over the polyline of z(u) - f(u)
        std::vector<std::pair<double, double>> pl;
        for (int i = 0; i <= 30; ++i) pl.push_back({-12.0 + 0.8 * i, 2 * std::sin(0.7 * i) + 0.05 * i});
        std::vector<Tri> strip;
        for (std::size_t i = 0; i + 1 < pl.size(); ++i) {
            P3 a{pl[i].first, -50, pl[i].second}, bb{pl[i + 1].first, -50, pl[i + 1].second};
            P3 c{pl[i].first, 50, pl[i].second}, dd{pl[i + 1].first, 50, pl[i + 1].second};
            strip.push_back({a, bb, dd});
            strip.push_back({a, dd, c});
        }
        for (double ux : {-3.3, 0.0, 1.7, 4.1}) {
            double ref = -1e300;
            for (std::size_t i = 0; i + 1 < pl.size(); ++i) {
                double u0 = pl[i].first - ux, u1 = pl[i + 1].first - ux;
                double lo = std::max(-R, std::min(u0, u1)), hi = std::min(R, std::max(u0, u1));
                if (lo > hi) continue;
                auto zz = [&](double u) { return pl[i].second + (u - u0) / (u1 - u0) * (pl[i + 1].second - pl[i].second); };
                ref = std::max(ref, props::golden_max([&](double u) { return zz(u) - (R - std::sqrt(R * R - u * u)); },
                                                      lo, hi, 90));
            }
            double z = drop_cutter_brute(d, strip, ux, 0.0);
            lcheck::expect(std::fabs(z - ref) < 1e-9, "disc_extruded_profile", std::to_string(z) + " vs " + std::to_string(ref));
            ++cases;
        }
    }

    // ---- the library's disc saws map to a disc cutter
    {
        ToolLibrary all = stone_tool_library();
        const ToolLibrary w = wood_tool_library();
        all.tools.insert(all.tools.end(), w.tools.begin(), w.tools.end());
        int n = 0;
        for (const auto& t : all.tools) {
            if (t.kind != ToolKind::DiscSaw) continue;
            if (!(t.kerf > 0)) {   // W27b states no kerf: refused, not guessed
                bool refused = false;
                try { disc_cutter_of(t, 0.0); } catch (const std::invalid_argument&) { refused = true; }
                lcheck::expect(refused, "disc_of_tool_without_kerf_throws", t.id);
                continue;
            }
            Cutter c = disc_cutter_of(t, 0.25);
            lcheck::expect(c.kind == CutterKind::Disc && std::fabs(c.radius - t.diameter / 2) < 1e-12 &&
                               std::fabs(c.thickness - t.kerf) < 1e-12 && c.heading == 0.25,
                           "disc_of_tool", t.id);
            ++n;
        }
        lcheck::expect(n == 2, "disc_saws_with_kerf_in_survey", std::to_string(n));   // S15 and W27
        bool threw = false;
        try { disc_cutter_of(find_tool(all, "S1"), 0.0); } catch (const std::invalid_argument&) { threw = true; }
        lcheck::expect(threw, "disc_of_non_saw_throws");
        ++cases;
    }

    // ---- random triangles: no gouge, tight contact
    const int N = 12000;
    props::Tally tl = props::run_disc_cases(20261114, N);
    cases += tl.cases;
    lcheck::expect(tl.failing == 0, "disc_props",
                   "failing " + std::to_string(tl.failing) + " (gouge-tri " + std::to_string(tl.by_bit[0]) +
                       ", gouge-rim " + std::to_string(tl.by_bit[1]) + ", contact " + std::to_string(tl.by_bit[2]) + ")");
    std::printf("disc properties: %ld cases, %ld failing\n", tl.cases, tl.failing);

    // ---- rotation invariance, thickness monotonicity
    {
        long n = 0;
        for (int i = 0; i < 4000; ++i, ++n) {
            Cutter c = props::random_disc(g);
            Tri t = props::rand_tri(g, 12);
            double x = g.uniform(-14, 14), y = g.uniform(-14, 14), th = g.uniform(-kPi, kPi);
            double z0 = drop_triangle(c, t, x, y);
            Cutter cr = c;
            cr.heading += th;
            double z1 = drop_triangle(cr, rot(t, x, y, th), x, y);
            lcheck::expect((std::isinf(z0) && std::isinf(z1)) || std::fabs(z0 - z1) < 1e-8 * (1 + std::fabs(z0)),
                           "disc_rotation_invariant", std::to_string(z0) + " vs " + std::to_string(z1));
            Cutter thick = c;
            thick.thickness = c.thickness * g.uniform(1.0, 3.0);
            double z2 = drop_triangle(thick, t, x, y);
            lcheck::expect(std::isinf(z0) || z2 >= z0 - 1e-9 * (1 + std::fabs(z0)), "disc_thickness_monotone");
        }
        cases += n;
    }

    // ---- terrain meshes: index = brute force (the query square must cover the rectangle), lifting
    {
        long n = 0;
        for (int m = 0; m < 20; ++m) {
            std::vector<Tri> tris = terrain(24, m);
            TriMesh mesh(tris);
            for (int q = 0; q < 60; ++q, ++n) {
                Cutter c = props::random_disc(g);
                double x = g.uniform(-2, 26), y = g.uniform(-2, 26);
                double z0 = drop_cutter(c, mesh, x, y), zb = drop_cutter_brute(c, tris, x, y);
                lcheck::expect((std::isinf(z0) && std::isinf(zb)) || std::fabs(z0 - zb) < 1e-12, "disc_index_equals_brute");
                if (std::isinf(z0)) continue;
                double d = g.uniform(0.0, 5.0);
                std::vector<Tri> up = tris, part = tris;
                for (auto& t : up) { t.a.z += d; t.b.z += d; t.c.z += d; }
                for (std::size_t i = 0; i < part.size(); ++i)
                    if (g.integer(0, 2) == 0) { part[i].a.z += d; part[i].b.z += d; part[i].c.z += d; }
                double zu = drop_cutter_brute(c, up, x, y), zp = drop_cutter_brute(c, part, x, y);
                lcheck::expect(std::fabs(zu - (z0 + d)) < 1e-9 * (1 + std::fabs(z0) + d), "disc_lift_translates");
                lcheck::expect(zp >= z0 - 1e-12 && zp <= zu + 1e-12, "disc_lift_monotone");
            }
        }
        cases += n;
    }

    // ---- path drop: a slot pass with the heading of the pass
    {
        std::vector<Tri> tris = terrain(30, 3);
        TriMesh mesh(tris);
        PathParams p;
        p.max_step = 1.0;
        p.min_step = 0.01;
        p.tol = 0.005;
        p.floor = -50.0;
        long n = 0;
        for (int k = 0; k < 40; ++k, ++n) {
            double x0 = g.uniform(0, 30), y0 = g.uniform(0, 30), ang = g.uniform(-kPi, kPi), len = g.uniform(3, 25);
            double x1 = x0 + len * std::cos(ang), y1 = y0 + len * std::sin(ang);
            Cutter c = disc_cutter(g.uniform(2, 8), g.uniform(0.3, 2.0), std::atan2(y1 - y0, x1 - x0));
            auto pts = adaptive_line(c, mesh, x0, y0, x1, y1, p);
            lcheck::expect(pts.size() >= 2, "disc_path_points");
            double worst = 0.0;
            for (std::size_t i = 0; i < pts.size(); ++i) {
                double z = drop_cutter(c, mesh, pts[i].x, pts[i].y);
                lcheck::expect((std::isinf(z) ? p.floor : z) == pts[i].z, "disc_path_point_is_drop");
                if (i + 1 == pts.size()) break;
                for (int s = 1; s < 8; ++s) {   // the straight run between kept points stays near the true height
                    double f = s / 8.0, x = pts[i].x + f * (pts[i + 1].x - pts[i].x), y = pts[i].y + f * (pts[i + 1].y - pts[i].y);
                    double zt = drop_cutter(c, mesh, x, y), zc = pts[i].z + f * (pts[i + 1].z - pts[i].z);
                    if (!std::isinf(zt) && pts[i].contact && pts[i + 1].contact) worst = std::max(worst, zt - zc);
                }
            }
            lcheck::expect(worst < 0.05, "disc_path_chord_deviation", std::to_string(worst));
        }
        cases += n;
    }

    const double secs = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    std::printf("elapsed %.1f s\n", secs);
    return lcheck::finish("dropcutter_disc", cases);
}
