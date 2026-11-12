// Copyright 2026 The lithocam authors. SPDX-License-Identifier: Apache-2.0
//
// Property tests for the adaptive path drop-cutter:
//   chord:     between kept points, the true CL height (dense drop-cutter) stays within tol plus a small slack of
//              the straight interpolation, on smooth terrain; on terrain with steps the deviation is reported
//   endpoints: the first and last points are the line's ends
//   economy:   fewer points than a dense pass at min_step
//   zigzag:    passes alternate direction and are `stepover` apart
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>

#include "check.hpp"
#include "lithocam/pathdrop.hpp"
#include "lithocam/scallop.hpp"

using namespace lithocam;

namespace {

std::vector<Tri> terrain(int G, double S, int seed, bool steps) {
    std::vector<Tri> tris;
    auto h = [&](int i, int j) {
        double x = i * S, y = j * S;
        double z = 4 * std::sin(0.13 * x + seed) * std::cos(0.09 * y) + 0.03 * x;
        if (steps && x > 20 && x < 26) z += 3.0;
        return z;
    };
    for (int i = 0; i < G; ++i)
        for (int j = 0; j < G; ++j) {
            P3 a{i * S, j * S, h(i, j)}, b{(i + 1) * S, j * S, h(i + 1, j)};
            P3 c{(i + 1) * S, (j + 1) * S, h(i + 1, j + 1)}, d{i * S, (j + 1) * S, h(i, j + 1)};
            tris.push_back({a, b, c});
            tris.push_back({a, c, d});
        }
    return tris;
}

}  // namespace

int main() {
    lcheck::Gen g(712);
    long cases = 0;
    double worst_smooth = 0, worst_step = 0;
    for (int m = 0; m < 6; ++m) {
        for (bool steps : {false, true}) {
            auto tris = terrain(40, 1.25, m, steps);
            TriMesh mesh(tris);
            for (int k = 0; k < 4; ++k) {
                Cutter c = k == 0 ? Cutter{CutterKind::Ball, 3} : k == 1 ? Cutter{CutterKind::Flat, 2.5}
                         : k == 2 ? Cutter{CutterKind::Bull, 3, 1} : Cutter{CutterKind::Cone, 3, 0, 0.6};
                PathParams p;
                p.max_step = 2.0;
                p.min_step = 0.01;
                p.tol = 0.01;
                double y = g.uniform(5, 45);
                auto pts = adaptive_line(c, mesh, 2, y, 48, y, p);
                lcheck::expect(std::fabs(pts.front().x - 2) < 1e-12 && std::fabs(pts.back().x - 48) < 1e-12,
                               "endpoints");
                lcheck::expect(pts.size() < static_cast<std::size_t>((48 - 2) / p.min_step), "economy");
                // dense check between kept points
                double worst = 0;
                for (std::size_t i = 1; i < pts.size(); ++i) {
                    const CLPoint &a = pts[i - 1], &b = pts[i];
                    for (int s = 1; s < 20; ++s) {
                        double t = s / 20.0;
                        double x = a.x + t * (b.x - a.x);
                        double z = drop_cutter(c, mesh, x, y);
                        worst = std::max(worst, z - (a.z + t * (b.z - a.z)));   // above the chord = gouge risk
                    }
                }
                if (steps) worst_step = std::max(worst_step, worst);
                else {
                    worst_smooth = std::max(worst_smooth, worst);
                    lcheck::expect(worst <= 2 * p.tol, "chord: straight run cuts into the CL surface by more than 2 tol");
                }
                ++cases;
            }
        }
    }
    // zigzag with the scallop-derived stepover
    {
        auto tris = terrain(30, 1.0, 1, false);
        TriMesh mesh(tris);
        Cutter c{CutterKind::Ball, 3};
        double so = max_stepover(3.0, 0.05);
        PathParams p;
        auto passes = zigzag(c, mesh, 1, 29, 1, 29, so, p);
        lcheck::expect(passes.size() >= 2, "zigzag passes");
        for (std::size_t i = 0; i + 1 < passes.size(); ++i) {
            bool fwd = passes[i].front().x < passes[i].back().x, fwd2 = passes[i + 1].front().x < passes[i + 1].back().x;
            lcheck::expect(fwd != fwd2, "zigzag alternates");
            double dy = passes[i + 1].front().y - passes[i].front().y;
            lcheck::expect(i + 2 == passes.size() || std::fabs(dy - so) < 1e-9, "zigzag stepover");
            ++cases;
        }
    }
    std::printf("pathdrop: worst chord excess smooth %.4g, with 3-unit steps %.4g (tol 0.01)\n", worst_smooth, worst_step);
    return lcheck::finish("pathdrop", cases);
}
