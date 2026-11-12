// Copyright 2026 The lithocam authors. SPDX-License-Identifier: Apache-2.0
#include "lithocam/pathdrop.hpp"

#include <algorithm>
#include <cmath>

namespace lithocam {

namespace {

CLPoint at(const Cutter& c, const TriMesh& m, double x, double y, double floor) {
    double z = drop_cutter(c, m, x, y);
    if (std::isinf(z)) return {x, y, floor, false};
    return {x, y, z, true};
}

void refine(const Cutter& c, const TriMesh& m, const CLPoint& a, const CLPoint& b, const PathParams& p, int depth,
            std::vector<CLPoint>& out) {
    double len = std::hypot(b.x - a.x, b.y - a.y);
    // hard floor: min_step / 1000. Where the cutter first meets a sharp edge the CL height rises like a square
    // root, and a chord over even min_step can sag well below it (a gouge of about 0.1 mm at 0.02 mm for a 3 mm
    // ball), so deviation keeps refining below min_step; a contact change alone stops at min_step.
    if (len <= p.min_step * 1e-3 || depth > 60) {
        out.push_back(b);
        return;
    }
    CLPoint mid = at(c, m, 0.5 * (a.x + b.x), 0.5 * (a.y + b.y), p.floor);
    // also probe the quarter points, so a narrow feature between a and mid or mid and b is not missed when the
    // midpoint happens to lie on the chord
    CLPoint q1 = at(c, m, 0.75 * a.x + 0.25 * b.x, 0.75 * a.y + 0.25 * b.y, p.floor);
    CLPoint q3 = at(c, m, 0.25 * a.x + 0.75 * b.x, 0.25 * a.y + 0.75 * b.y, p.floor);
    double dev = std::max({std::fabs(mid.z - 0.5 * (a.z + b.z)), std::fabs(q1.z - (0.75 * a.z + 0.25 * b.z)),
                           std::fabs(q3.z - (0.25 * a.z + 0.75 * b.z))});
    bool contact_change = a.contact != b.contact || a.contact != mid.contact || a.contact != q1.contact ||
                          a.contact != q3.contact;
    if (contact_change ? len <= p.min_step : dev <= p.tol) {
        out.push_back(b);
        return;
    }
    refine(c, m, a, mid, p, depth + 1, out);
    refine(c, m, mid, b, p, depth + 1, out);
}

}  // namespace

std::vector<CLPoint> adaptive_line(const Cutter& cutter, const TriMesh& mesh, double x0, double y0, double x1,
                                   double y1, const PathParams& p) {
    std::vector<CLPoint> out;
    double len = std::hypot(x1 - x0, y1 - y0);
    int n = std::max(1, static_cast<int>(std::ceil(len / p.max_step)));
    CLPoint prev = at(cutter, mesh, x0, y0, p.floor);
    out.push_back(prev);
    for (int i = 1; i <= n; ++i) {
        double t = static_cast<double>(i) / n;
        CLPoint cur = at(cutter, mesh, x0 + t * (x1 - x0), y0 + t * (y1 - y0), p.floor);
        refine(cutter, mesh, prev, cur, p, 0, out);
        prev = cur;
    }
    return out;
}

std::vector<std::vector<CLPoint>> zigzag(const Cutter& cutter, const TriMesh& mesh, double xmin, double xmax,
                                         double ymin, double ymax, double stepover, const PathParams& p) {
    std::vector<std::vector<CLPoint>> passes;
    int n = std::max(1, static_cast<int>(std::ceil((ymax - ymin) / stepover)));
    for (int i = 0; i <= n; ++i) {
        double y = std::min(ymax, ymin + i * stepover);
        if (i % 2 == 0) passes.push_back(adaptive_line(cutter, mesh, xmin, y, xmax, y, p));
        else passes.push_back(adaptive_line(cutter, mesh, xmax, y, xmin, y, p));
    }
    return passes;
}

}  // namespace lithocam
