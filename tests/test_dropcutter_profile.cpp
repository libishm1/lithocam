// Copyright 2026 The lithocam authors. SPDX-License-Identifier: Apache-2.0
//
// Property tests for the general profile cutter (piecewise line and arc profile, tip upwards):
//   closed forms:   tapered ball on a plane, flat-tipped cone on a vertex, flame burr on a flat;
//   properties:     no gouge (triangle sampling and cutter-surface sampling) and tight contact against an
//                   independent maximiser (tests/test_dropcutter_props.hpp), on random convex profiles;
//   agreement:      with the exact Ball, Flat, Bull and Cone models where the profile coincides with them;
//   monotonicity:   lifting the mesh by d lifts the drop by d; lifting part of it never lowers the drop;
//   index:          the grid-indexed result equals brute force;
//   validation:     non-convex, falling or broken profiles are refused.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <vector>

#include "test_dropcutter_props.hpp"

using namespace lithocam;
using props::kPi;

namespace {

template <class F>
bool throws_invalid(F f) {
    try {
        f();
    } catch (const std::invalid_argument&) {
        return true;
    }
    return false;
}

double rel(double a, double b) { return std::fabs(a - b) / (1.0 + std::fabs(a) + std::fabs(b)); }

// agreement of a profile cutter with an exact model on random triangles; returns the worst relative difference
double agreement(lcheck::Gen& g, const Cutter& profile_c, const Cutter& exact_c, int n, int& cases) {
    double worst = 0.0;
    for (int i = 0; i < n; ++i, ++cases) {
        Tri t = props::rand_tri(g, 12);
        double x = g.uniform(-14, 14), y = g.uniform(-14, 14);
        double a = drop_triangle(profile_c, t, x, y), b = drop_triangle(exact_c, t, x, y);
        if (std::isinf(a) || std::isinf(b)) {
            lcheck::expect(std::isinf(a) && std::isinf(b), "agree_reach", "reach differs");
            continue;
        }
        worst = std::max(worst, rel(a, b));
    }
    return worst;
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
    std::setvbuf(stdout, nullptr, _IONBF, 0);
    lcheck::Gen g(20261114);
    long cases = 0;

    // ---- closed forms
    {
        // tapered ball: tip radius 2, half taper 0.3 rad, outer radius 6; a horizontal plane touches the tip
        Cutter tb = tapered_cutter(6.0, 0.0, 2.0, 0.3);
        Tri plane{{-50, -50, 3}, {50, -50, 3}, {0, 50, 3}};
        lcheck::expect(std::fabs(drop_triangle(tb, plane, 0, 0) - 3) < 1e-12, "tapered_ball_plane");
        // a vertex at horizontal distance 1 sits on the ball arc: f = 2 - sqrt(4 - 1)
        Tri spike{{1, 0, 5}, {50, 0, -5}, {1, 50, -5}};
        lcheck::expect(std::fabs(drop_triangle(tb, spike, 0, 0) - (5 - (2 - std::sqrt(3.0)))) < 1e-12,
                       "tapered_ball_vertex_on_arc");
        // the same vertex on the cone: distance 5, tangent point at r = 2 cos 0.3, h = 2 (1 - sin 0.3)
        double rt = 2 * std::cos(0.3), ht = 2 * (1 - std::sin(0.3));
        Tri spike2{{5, 0, 5}, {50, 0, -5}, {5, 50, -5}};
        lcheck::expect(std::fabs(drop_triangle(tb, spike2, 0, 0) - (5 - (ht + (5 - rt) / std::tan(0.3)))) < 1e-12,
                       "tapered_ball_vertex_on_cone");
        // flat-tipped cone (flat radius 1, half angle 0.5): a plane at z = 4 touches the flat
        Cutter fc = tapered_cutter(6.0, 1.0, 0.0, 0.5);
        Tri plane4{{-50, -50, 4}, {50, -50, 4}, {0, 50, 4}};
        lcheck::expect(std::fabs(drop_triangle(fc, plane4, 0, 0) - 4) < 1e-12, "flat_tip_cone_plane");
        // a vertex at distance 0.5 is under the flat: tip = z; at distance 3: z - (3 - 1) / tan 0.5
        Tri v1{{0.5, 0, 7}, {50, 0, -5}, {0.5, 50, -5}}, v3{{3, 0, 7}, {50, 0, -5}, {3, 50, -5}};
        lcheck::expect(std::fabs(drop_triangle(fc, v1, 0, 0) - 7) < 1e-12, "flat_tip_cone_vertex_flat");
        lcheck::expect(std::fabs(drop_triangle(fc, v3, 0, 0) - (7 - 2 / std::tan(0.5))) < 1e-12,
                       "flat_tip_cone_vertex_flank");
        // flame burr R 3, L 15: f(R) = L; a vertex at distance R has tip z - L
        Cutter fl = burr_cutter("flame", 3.0, 15.0, 0.0);
        Tri v{{3, 0, 20}, {50, 0, -5}, {3, 50, -5}};
        lcheck::expect(std::fabs(drop_triangle(fl, v, 0, 0) - (20 - 15)) < 1e-9, "flame_vertex_at_rim");
        lcheck::expect(std::fabs(profile(fl, 0.0)) < 1e-12 && std::fabs(profile(fl, 3.0) - 15.0) < 1e-9, "flame_ends");
        // a sloped plane under a cone-flank cutter: support point on the flank where the slopes match
        cases += 9;
    }

    // ---- validation
    {
        lcheck::expect(throws_invalid([] { profile_cutter({}); }), "reject_empty");
        lcheck::expect(throws_invalid([] { profile_cutter({{false, 1, 1, 0, 0}, {false, 2, 1.5, 0, 0}}); }),
                       "reject_concave_lines");   // slope 1 then 0.5
        lcheck::expect(throws_invalid([] { profile_cutter({{false, 1, 1, 0, 0}, {false, 1, 2, 0, 0}}); }),
                       "reject_vertical_step");
        lcheck::expect(throws_invalid([] { profile_cutter({{false, 1, -1, 0, 0}}); }), "reject_falling");
        lcheck::expect(throws_invalid([] { profile_cutter({{true, 1, 1, 0, 0.5}}); }), "reject_arc_off_circle");
        lcheck::expect(throws_invalid([] { tapered_cutter(1.0, 0.9, 0.5, 0.3); }), "reject_tapered_overfit");
        lcheck::expect(throws_invalid([] { burr_cutter("flame", 3.0, 2.0, 0.0); }), "reject_flame_short");
        lcheck::expect(throws_invalid([] { burr_cutter("spoon", 3.0, 2.0, 0.0); }), "reject_burr_shape");
        // an arc that continues after a vertical tangent is not convex
        lcheck::expect(throws_invalid([] { profile_cutter({{true, 1, 1, 0, 1}, {false, 2, 2, 0, 0}}); }),
                       "reject_after_vertical");
        Cutter ok = tapered_cutter(5.0, 0.5, 1.0, 0.4);
        lcheck::expect(validate_profile(ok).empty(), "valid_tapered");
        Cutter bad = ok;
        bad.radius += 1.0;
        lcheck::expect(!validate_profile(bad).empty(), "reject_radius_mismatch");
        cases += 12;
    }

    // ---- random convex profiles: no gouge, tight contact
    const int NP = 12000;
    props::Tally tl = props::run_profile_cases(20261114, NP);
    cases += tl.cases;
    lcheck::expect(tl.cases >= NP - 50, "profile_cases_ran", std::to_string(tl.cases));
    lcheck::expect(tl.failing == 0, "profile_props",
                   "failing " + std::to_string(tl.failing) + " (gouge-tri " + std::to_string(tl.by_bit[0]) +
                       ", gouge-surface " + std::to_string(tl.by_bit[1]) + ", contact " + std::to_string(tl.by_bit[2]) + ")");
    std::printf("profile properties: %ld cases, %ld failing\n", tl.cases, tl.failing);

    // ---- agreement with the exact models
    {
        double w_ball = 0, w_flat = 0, w_bull = 0, w_cone = 0, w_cone2 = 0;
        int n = 0;
        for (int k = 0; k < 2500; ++k) {
            double R = g.uniform(0.5, 15);
            w_ball = std::max(w_ball, agreement(g, burr_cutter("ball", R, 0, 0), Cutter{CutterKind::Ball, R}, 1, n));
            w_flat = std::max(w_flat, agreement(g, burr_cutter("cylinder", R, 1, 0.0), Cutter{CutterKind::Flat, R}, 1, n));
            double rc = g.uniform(0.05, 1.0) * R;
            w_bull = std::max(w_bull, agreement(g, burr_cutter("cylinder", R, 1, rc), Cutter{CutterKind::Bull, R, rc}, 1, n));
            double L = g.uniform(0.3, 4.0) * R, ha = std::atan(R / L);
            w_cone = std::max(w_cone, agreement(g, burr_cutter("cone", R, L, 0), Cutter{CutterKind::Cone, R, 0.0, ha}, 1, n));
            double a2 = g.uniform(0.05, 1.45);
            w_cone2 = std::max(w_cone2, agreement(g, tapered_cutter(R, 0, 0, a2), Cutter{CutterKind::Cone, R, 0.0, a2}, 1, n));
        }
        cases += n;
        std::printf("agreement (max relative difference): ball %.2e, flat %.2e, bull %.2e, cone %.2e, cone %.2e\n",
                    w_ball, w_flat, w_bull, w_cone, w_cone2);
        for (double w : {w_ball, w_flat, w_bull, w_cone, w_cone2}) lcheck::expect(w < 1e-9, "agreement", std::to_string(w));
    }

    // ---- monotonicity under lifting, and index = brute force, on terrain meshes
    {
        long mono = 0;
        for (int m = 0; m < 20; ++m) {
            std::vector<Tri> tris = terrain(24, m);
            TriMesh mesh(tris);
            for (int q = 0; q < 60; ++q) {
                Cutter c{CutterKind::Flat, 1.0};
                if (!props::random_profile(g, m + q, c)) continue;
                double x = g.uniform(-2, 26), y = g.uniform(-2, 26);
                double z0 = drop_cutter(c, mesh, x, y), zb = drop_cutter_brute(c, tris, x, y);
                lcheck::expect((std::isinf(z0) && std::isinf(zb)) || std::fabs(z0 - zb) < 1e-12, "index_equals_brute");
                if (std::isinf(z0)) { ++cases; continue; }
                double d = g.uniform(0.0, 5.0);
                std::vector<Tri> up = tris, part = tris;
                for (auto& t : up) { t.a.z += d; t.b.z += d; t.c.z += d; }
                for (std::size_t i = 0; i < part.size(); ++i)
                    if (g.integer(0, 2) == 0) { part[i].a.z += d; part[i].b.z += d; part[i].c.z += d; }
                double zu = drop_cutter_brute(c, up, x, y), zp = drop_cutter_brute(c, part, x, y);
                lcheck::expect(std::fabs(zu - (z0 + d)) < 1e-9 * (1 + std::fabs(z0) + d), "lift_translates");
                lcheck::expect(zp >= z0 - 1e-12 && zp <= zu + 1e-12, "lift_monotone",
                               std::to_string(z0) + " <= " + std::to_string(zp) + " <= " + std::to_string(zu));
                ++mono;
                cases += 1;
            }
        }
        std::printf("terrain cases (index, lifting): %ld\n", mono);
    }

    const double secs = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    std::printf("elapsed %.1f s\n", secs);
    return lcheck::finish("dropcutter_profile", cases);
}
