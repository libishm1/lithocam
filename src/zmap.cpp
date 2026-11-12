// Copyright 2026 The lithocam authors. SPDX-License-Identifier: Apache-2.0
#include "lithocam/zmap.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace lithocam {

ZMap make_stock(double xmin, double xmax, double ymin, double ymax, double cell, double z_top) {
    ZMap m;
    m.cell = cell;
    m.x0 = xmin;
    m.y0 = ymin;
    m.nx = static_cast<int>(std::floor((xmax - xmin) / cell)) + 1;
    m.ny = static_cast<int>(std::floor((ymax - ymin) / cell)) + 1;
    m.z.assign(static_cast<std::size_t>(m.nx) * m.ny, z_top);
    return m;
}

void cut_move(ZMap& s, const Cutter& c, const CLPoint& a, const CLPoint& b) {
    const double R = c.radius;
    double dx = b.x - a.x, dy = b.y - a.y, dz = b.z - a.z;
    int i0 = std::max(0, static_cast<int>(std::floor((std::min(a.x, b.x) - R - s.x0) / s.cell)));
    int i1 = std::min(s.nx - 1, static_cast<int>(std::ceil((std::max(a.x, b.x) + R - s.x0) / s.cell)));
    int j0 = std::max(0, static_cast<int>(std::floor((std::min(a.y, b.y) - R - s.y0) / s.cell)));
    int j1 = std::min(s.ny - 1, static_cast<int>(std::ceil((std::max(a.y, b.y) + R - s.y0) / s.cell)));
    double qa = dx * dx + dy * dy;
    const double phi = 0.5 * (std::sqrt(5.0) - 1.0);
    for (int j = j0; j <= j1; ++j)
        for (int i = i0; i <= i1; ++i) {
            double x = s.cx(i), y = s.cy(j);
            double fx = a.x - x, fy = a.y - y;
            double lo = 0, hi = 1;
            if (qa > 0) {                            // part of the move whose disc covers (x, y)
                double qb = 2 * (fx * dx + fy * dy), qc = fx * fx + fy * fy - R * R;
                double disc = qb * qb - 4 * qa * qc;
                if (disc < 0) continue;
                double r = std::sqrt(disc);
                lo = std::max(0.0, (-qb - r) / (2 * qa));
                hi = std::min(1.0, (-qb + r) / (2 * qa));
                if (lo > hi) continue;
            } else if (fx * fx + fy * fy > R * R) {
                continue;
            }
            auto g = [&](double t) {
                double px = a.x + t * dx - x, py = a.y + t * dy - y;
                double d = std::min(R, std::sqrt(px * px + py * py));
                return a.z + t * dz + profile(c, d);
            };
            double best;
            if (hi - lo < 1e-15) {
                best = g(lo);
            } else {
                double x1 = hi - phi * (hi - lo), x2 = lo + phi * (hi - lo), g1 = g(x1), g2 = g(x2);
                for (int k = 0; k < 80 && hi - lo > 1e-14; ++k) {
                    if (g1 > g2) { lo = x1; x1 = x2; g1 = g2; x2 = lo + phi * (hi - lo); g2 = g(x2); }
                    else         { hi = x2; x2 = x1; g2 = g1; x1 = hi - phi * (hi - lo); g1 = g(x1); }
                }
                best = std::min({g(lo), g(hi), g1, g2});
            }
            double& h = s.at(i, j);
            if (best < h) h = best;
        }
}

void cut_passes(ZMap& s, const Cutter& c, const std::vector<std::vector<CLPoint>>& passes) {
    for (const auto& p : passes)
        for (std::size_t k = 1; k < p.size(); ++k)
            if (p[k - 1].contact && p[k].contact) cut_move(s, c, p[k - 1], p[k]);
}

ZMap target_heights(const TriMesh& mesh, const ZMap& grid) {
    ZMap t = grid;
    std::vector<std::size_t> cand;
    const double nan = std::numeric_limits<double>::quiet_NaN();
    for (int j = 0; j < grid.ny; ++j)
        for (int i = 0; i < grid.nx; ++i) {
            double x = grid.cx(i), y = grid.cy(j);
            mesh.query(x, y, 0.0, cand);
            double best = -std::numeric_limits<double>::infinity();
            for (std::size_t k : cand) {
                const Tri& tr = mesh.triangles()[k];
                // barycentric in xy
                double d = (tr.b.y - tr.c.y) * (tr.a.x - tr.c.x) + (tr.c.x - tr.b.x) * (tr.a.y - tr.c.y);
                if (std::fabs(d) < 1e-300) continue;
                double l1 = ((tr.b.y - tr.c.y) * (x - tr.c.x) + (tr.c.x - tr.b.x) * (y - tr.c.y)) / d;
                double l2 = ((tr.c.y - tr.a.y) * (x - tr.c.x) + (tr.a.x - tr.c.x) * (y - tr.c.y)) / d;
                double l3 = 1 - l1 - l2;
                const double e = -1e-12;
                if (l1 < e || l2 < e || l3 < e) continue;
                best = std::max(best, l1 * tr.a.z + l2 * tr.b.z + l3 * tr.c.z);
            }
            t.at(i, j) = std::isinf(best) ? nan : best;
        }
    return t;
}

ZStats deviation(const ZMap& s, const ZMap& t, double xmin, double xmax, double ymin, double ymax) {
    ZStats st{std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity(), 0, 0};
    double sum = 0;
    for (int j = 0; j < s.ny; ++j)
        for (int i = 0; i < s.nx; ++i) {
            double x = s.cx(i), y = s.cy(j);
            if (x < xmin || x > xmax || y < ymin || y > ymax) continue;
            double tv = t.at(i, j);
            if (std::isnan(tv)) continue;
            double d = s.at(i, j) - tv;
            st.min_dev = std::min(st.min_dev, d);
            st.max_dev = std::max(st.max_dev, d);
            sum += d;
            ++st.cells;
        }
    st.mean_dev = st.cells ? sum / st.cells : 0;
    return st;
}

}  // namespace lithocam
