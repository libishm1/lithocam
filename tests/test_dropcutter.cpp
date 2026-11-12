// Copyright 2026 The lithocam authors. SPDX-License-Identifier: Apache-2.0
//
// Property tests for drop-cutter (ball and flat):
//   no_gouge:  no sampled mesh point lies above the cutter's lower surface at the computed height;
//   contact:   the computed height equals the maximum over a dense sampling within the sampling error;
//   index:     the grid-indexed result equals brute force over all triangles;
//   exact:     closed-form cases (flat plane, single vertex, horizontal edge, sloped plane).
#include <cmath>
#include <cstdio>
#include <limits>
#include <vector>

#include "check.hpp"
#include "lithocam/dropcutter.hpp"

using namespace lithocam;

namespace {

double surf(const Cutter& c, double d) { return profile(c, d); }

Cutter rand_cutter(lcheck::Gen& g, int k) {
    double R = g.uniform(0.5, 15);
    switch (k % 4) {
        case 0: return {CutterKind::Flat, R};
        case 1: return {CutterKind::Ball, R};
        case 2: return {CutterKind::Bull, R, g.uniform(0.05, 1.0) * R};
        default: return {CutterKind::Cone, R, 0.0, g.uniform(0.25, 1.4)};
    }
}

// Maximum of p.z - f(d(p)) over a dense sampling of the triangle's points within the disc.
double sampled(const Cutter& c, const Tri& t, double x, double y, int n) {
    double best = -std::numeric_limits<double>::infinity();
    for (int i = 0; i <= n; ++i)
        for (int j = 0; i + j <= n; ++j) {
            double u = double(i) / n, v = double(j) / n, w = 1 - u - v;
            P3 p{w * t.a.x + u * t.b.x + v * t.c.x, w * t.a.y + u * t.b.y + v * t.c.y, w * t.a.z + u * t.b.z + v * t.c.z};
            double d = std::hypot(p.x - x, p.y - y);
            if (d <= c.radius) best = std::max(best, p.z - surf(c, d));
        }
    return best;
}

Tri rand_tri(lcheck::Gen& g, double span) {
    auto rp = [&] { return P3{g.uniform(-span, span), g.uniform(-span, span), g.uniform(-span, span)}; };
    return {rp(), rp(), rp()};
}

}  // namespace

int main() {
    lcheck::Gen g(20261112);
    int cases = 0;

    // exact closed forms
    {
        Tri flat{{-10, -10, 2}, {10, -10, 2}, {0, 10, 2}};
        for (CutterKind k : {CutterKind::Flat, CutterKind::Ball}) {
            Cutter c{k, 3};
            lcheck::expect(std::fabs(drop_triangle(c, flat, 0, 0) - 2) < 1e-12, "horizontal plane under the tip");
        }
        Tri spike{{0, 0, 5}, {50, 0, -5}, {0, 50, -5}};
        Cutter ball{CutterKind::Ball, 4};
        // vertex at horizontal distance 3: tip at 5 + sqrt(16 - 9) - 4
        lcheck::expect(std::fabs(drop_triangle(ball, spike, -3, 0) - (5 + std::sqrt(7.0) - 4)) < 1e-12,
                       "ball on a vertex");
        Cutter flatc{CutterKind::Flat, 4};
        lcheck::expect(std::fabs(drop_triangle(flatc, spike, -3, 0) - 5) < 1e-12, "flat on a vertex");
        // sloped plane z = 0.5 x: ball tip = plane z at (x,y) + R (1/cos - 1); flat tip = plane z at x + R
        Tri slope{{-100, -100, -50}, {100, -100, 50}, {0, 100, 0}};
        double cosa = 1 / std::sqrt(1 + 0.25);
        lcheck::expect(std::fabs(drop_triangle(ball, slope, 10, 0) - (5 + 4 * (1 / cosa - 1))) < 1e-9,
                       "ball on a sloped plane");
        lcheck::expect(std::fabs(drop_triangle(flatc, slope, 10, 0) - (0.5 * 14)) < 1e-9, "flat on a sloped plane");
        // horizontal edge (ridge) along y at x = 0, z = 1, seen at horizontal distance 2 by a ball of R 4
        Tri ridge{{0, -50, 1}, {0, 50, 1}, {-50, 0, -40}};
        lcheck::expect(std::fabs(drop_triangle(ball, ridge, 2, 0) - (1 + std::sqrt(12.0) - 4)) < 1e-9,
                       "ball on an edge");
        cases += 7;
    }

    // random triangles: no gouge and contact against dense sampling
    const int N = 4000, S = 120;
    int contact_fail = 0;
    for (int it = 0; it < N; ++it) {
        Tri t = rand_tri(g, 20);
        Cutter c = rand_cutter(g, it);
        double x = g.uniform(-25, 25), y = g.uniform(-25, 25);
        double z = drop_triangle(c, t, x, y);
        double zs = sampled(c, t, x, y, S);
        if (std::isinf(zs)) {
            // no sampled point inside the disc: the exact answer may still touch a sliver; it must not exceed
            // the triangle's top
            lcheck::expect(z <= std::max({t.a.z, t.b.z, t.c.z}) + 1e-9, "no-sample case bounded");
            ++cases;
            continue;
        }
        lcheck::expect(z >= zs - 1e-9, "no_gouge: a sampled point lies above the cutter");
        // sampling error: one grid step of the triangle times the steepest slope of p.z - f(d)
        double step = std::max({std::hypot(t.b.x - t.a.x, t.b.y - t.a.y), std::hypot(t.c.x - t.a.x, t.c.y - t.a.y),
                                std::hypot(t.c.x - t.b.x, t.c.y - t.b.y)}) / S;
        double zrange = std::max({std::fabs(t.a.z - t.b.z), std::fabs(t.b.z - t.c.z), std::fabs(t.a.z - t.c.z)});
        double slack = 1e-6 + step * (2 + zrange) * 2;
        if (c.kind == CutterKind::Ball) slack += std::sqrt(2 * c.radius * step);
        if (c.kind == CutterKind::Bull) slack += std::sqrt(2 * c.corner * step);
        if (c.kind == CutterKind::Cone) slack += 2 * step / std::tan(c.half_angle);
        if (z - zs > slack) ++contact_fail;
        ++cases;
    }
    lcheck::expect(contact_fail == 0, "contact: computed height above every sampled bound by more than the slack");

    // grid index equals brute force on random terrain meshes
    for (int m = 0; m < 20; ++m) {
        std::vector<Tri> tris;
        const int G = 30;
        auto h = [&](int i, int j) { return 3 * std::sin(0.3 * i + m) * std::cos(0.2 * j) + 0.1 * ((i * 7 + j * 3) % 5); };
        for (int i = 0; i < G; ++i)
            for (int j = 0; j < G; ++j) {
                P3 p00{double(i), double(j), h(i, j)}, p10{double(i + 1), double(j), h(i + 1, j)};
                P3 p01{double(i), double(j + 1), h(i, j + 1)}, p11{double(i + 1), double(j + 1), h(i + 1, j + 1)};
                tris.push_back({p00, p10, p11});
                tris.push_back({p00, p11, p01});
            }
        TriMesh mesh(tris);
        for (int q = 0; q < 200; ++q) {
            Cutter c = rand_cutter(g, q);
            double x = g.uniform(-3, G + 3), y = g.uniform(-3, G + 3);
            double a = drop_cutter(c, mesh, x, y), b = drop_cutter_brute(c, tris, x, y);
            lcheck::expect((std::isinf(a) && std::isinf(b)) || std::fabs(a - b) < 1e-12, "index equals brute force");
            ++cases;
        }
    }
    return lcheck::finish("dropcutter", cases);
}
