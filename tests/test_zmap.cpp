// Copyright 2026 The lithocam authors. SPDX-License-Identifier: Apache-2.0
//
// Z-map stock simulation checked end to end against the verified scallop bound:
//   flat floor, ball cutter, parallel passes at max_stepover(R, h): the leftover peaks at h (scallop theorem) and
//   never goes below the floor (no gouge);
//   a single plunge leaves exactly the cutter profile;
//   drop-cutter + adaptive zig-zag over wavy terrain: no gouge beyond the path tolerance.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <map>
#include <utility>
#include <vector>

#include "check.hpp"
#include "lithocam/pathdrop.hpp"
#include "lithocam/scallop.hpp"
#include "lithocam/zmap.hpp"

using namespace lithocam;

int main() {
    long cases = 0;
    // 1) flat floor at z = 0, ball R 3, scallop h = 0.05
    for (double h : {0.02, 0.05, 0.2}) {
        Cutter ball{CutterKind::Ball, 3.0};
        double so = max_stepover(3.0, h);
        ZMap s = make_stock(0, 40, 0, 40, 0.02, 1.0);
        std::vector<std::vector<CLPoint>> passes;
        for (double y = 0; y <= 40 + 1e-9; y += so) passes.push_back({{-5, y, 0, true}, {45, y, 0, true}});
        cut_passes(s, ball, passes);
        double lo = 1e9, hi = -1e9;
        for (int j = 0; j < s.ny; ++j)
            for (int i = 0; i < s.nx; ++i) {
                double y = s.cy(j);
                if (y < 10 || y > 30) continue;     // away from the raster's edges
                lo = std::min(lo, s.at(i, j));
                hi = std::max(hi, s.at(i, j));
            }
        std::printf("flat floor, scallop target %.3f: leftover min %.2e max %.5f (grid 0.02)\n", h, lo, hi);
        lcheck::expect(lo >= -1e-12, "no gouge on the floor");
        lcheck::expect(hi <= h + 1e-9, "leftover never exceeds the scallop bound");
        lcheck::expect(hi >= h - 2e-3, "leftover reaches the bound within the grid resolution");
        ++cases;
    }
    // 2) a single plunge leaves the profile: cell at distance d has height tip + f(d)
    for (CutterKind k : {CutterKind::Ball, CutterKind::Flat, CutterKind::Bull, CutterKind::Cone}) {
        Cutter c{k, 4.0, 1.5, 0.7};
        ZMap s = make_stock(-6, 6, -6, 6, 0.05, 10.0);
        CLPoint p{0.0, 0.0, 2.0, true};
        cut_move(s, c, p, p);
        double worst = 0;
        for (int j = 0; j < s.ny; ++j)
            for (int i = 0; i < s.nx; ++i) {
                double d = std::hypot(s.cx(i), s.cy(j));
                if (std::fabs(d - 4.0) < 1e-9) continue;   // on the rim: inside/outside depends on rounding
                double expect = d <= 4.0 ? std::min(10.0, 2.0 + profile(c, d)) : 10.0;
                worst = std::max(worst, std::fabs(s.at(i, j) - expect));
            }
        lcheck::expect(worst < 1e-12, "plunge leaves the cutter profile");
        ++cases;
    }
    // 3) terrain: drop-cutter zig-zag leaves no gouge beyond the path tolerance
    std::vector<Tri> tris;
    const int G = 40;
    const double S = 1.0;
    auto hgt = [&](int i, int j) { return 3 * std::sin(0.2 * i) * std::cos(0.15 * j) + 0.05 * i; };
    for (int i = 0; i < G; ++i)
        for (int j = 0; j < G; ++j) {
            P3 a{i * S, j * S, hgt(i, j)}, b{(i + 1) * S, j * S, hgt(i + 1, j)};
            P3 cc{(i + 1) * S, (j + 1) * S, hgt(i + 1, j + 1)}, d{i * S, (j + 1) * S, hgt(i, j + 1)};
            tris.push_back({a, b, cc});
            tris.push_back({a, cc, d});
        }
    TriMesh mesh(tris);
    for (CutterKind k : {CutterKind::Ball, CutterKind::Bull, CutterKind::Flat}) {
        Cutter c{k, 2.0, 0.8, 0.0};
        PathParams pp;
        pp.max_step = 1.0;
        pp.min_step = 0.01;
        pp.tol = 0.01;
        auto passes = zigzag(c, mesh, 0, 40, 0, 40, 0.5, pp);
        ZMap s = make_stock(0, 40, 0, 40, 0.1, 10.0);
        cut_passes(s, c, passes);
        ZMap t = target_heights(mesh, s);
        ZStats st = deviation(s, t, 3, 37, 3, 37);
        std::printf("terrain, cutter %d: stock - target min %.4f max %.4f mean %.4f over %ld cells\n",
                    static_cast<int>(k), st.min_dev, st.max_dev, st.mean_dev, st.cells);
        lcheck::expect(st.min_dev >= -2 * pp.tol, "no gouge beyond 2 x path tolerance");
        ++cases;
    }
    // 4) stock mesh: watertight (every edge in exactly two triangles, opposite directions) and its volume equals
    //    the height field's volume (trapezoid rule over grid squares, split as the mesh is: exact for the mesh)
    {
        Cutter c{CutterKind::Ball, 3.0};
        ZMap s = make_stock(0, 30, 0, 20, 0.25, 10.0);
        for (int k = 0; k < 6; ++k) cut_move(s, c, {2.0, 3.0 + 3 * k, 7.0 + 0.3 * k, true}, {28.0, 4.0 + 3 * k, 6.5, true});
        StockMesh m = zmap_mesh(s, 0.0);
        std::map<std::pair<int, int>, int> edges;
        for (std::size_t k = 0; k < m.triangles.size(); k += 3)
            for (int e = 0; e < 3; ++e) {
                int a = m.triangles[k + e], b = m.triangles[k + (e + 1) % 3];
                edges[{a, b}] += 1;
            }
        bool manifold = true;
        for (const auto& [ab, n] : edges) {
            auto it = edges.find({ab.second, ab.first});
            if (n != 1 || it == edges.end() || it->second != 1) manifold = false;
        }
        lcheck::expect(manifold, "stock mesh: every directed edge once, its reverse once (closed, oriented)");
        double hv = 0;                                   // exact volume of the two-triangle-per-square surface
        for (int j = 0; j + 1 < s.ny; ++j)
            for (int i = 0; i + 1 < s.nx; ++i) {
                double a = s.at(i, j), b = s.at(i + 1, j), cc = s.at(i + 1, j + 1), d = s.at(i, j + 1);
                hv += s.cell * s.cell * ((a + b + cc) / 3 + (a + cc + d) / 3) / 2;
            }
        double mv = mesh_volume(m);
        std::printf("stock mesh: %zu vertices, %zu triangles, volume %.6f vs height field %.6f\n", m.vertices.size(),
                    m.triangles.size() / 3, mv, hv);
        lcheck::expect(std::fabs(mv - hv) < 1e-9 * hv, "stock mesh volume equals the height-field volume");
        ++cases;
    }
    return lcheck::finish("zmap", cases);
}
