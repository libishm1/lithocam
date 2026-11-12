// Copyright 2026 The lithocam authors. SPDX-License-Identifier: Apache-2.0
// Property tests mirroring lean/Lithocam/Kerf.lean, extended to convex parts with circumscribed corners.
#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "check.hpp"
#include "lithocam/kerf.hpp"

using lithocam::Vec2;

namespace {

constexpr double kPi = 3.14159265358979323846;

// Random convex counter-clockwise polygon: sorted angles on a random ellipse, rotated and shifted. Sizes in mm.
std::vector<Vec2> random_convex(lcheck::Gen& g) {
    const int n = g.integer(3, 12);
    std::vector<double> t(n);
    for (double& a : t) a = g.uniform(0.0, 2.0 * kPi);
    std::sort(t.begin(), t.end());
    const double A = g.uniform(50.0, 2000.0), B = g.uniform(50.0, 2000.0), rot = g.uniform(0.0, 2.0 * kPi);
    const double cx = g.uniform(-1000.0, 1000.0), cy = g.uniform(-1000.0, 1000.0);
    std::vector<Vec2> p;
    for (double a : t) {
        const double x = A * std::cos(a), y = B * std::sin(a);
        p.push_back({cx + x * std::cos(rot) - y * std::sin(rot), cy + x * std::sin(rot) + y * std::cos(rot)});
    }
    // drop near-duplicate vertices (degenerate edges)
    std::vector<Vec2> q;
    for (const Vec2& v : p)
        if (q.empty() || std::hypot(v.x - q.back().x, v.y - q.back().y) > 1e-3) q.push_back(v);
    if (q.size() > 1 && std::hypot(q.front().x - q.back().x, q.front().y - q.back().y) <= 1e-3) q.pop_back();
    return q;
}

// Negative control: the same path with INSCRIBED corner chords (vertices on the arc). It must gouge.
std::vector<Vec2> inscribed_path(const std::vector<Vec2>& part, double kerf, int m) {
    const double r = kerf / 2.0;
    const std::size_t n = part.size();
    std::vector<Vec2> path;
    for (std::size_t i = 0; i < n; ++i) {
        const Vec2 a = part[i], v = part[(i + 1) % n], w = part[(i + 2) % n];
        const double L1 = std::hypot(v.x - a.x, v.y - a.y), L2 = std::hypot(w.x - v.x, w.y - v.y);
        const Vec2 n1{(v.y - a.y) / L1, -(v.x - a.x) / L1}, n2{(w.y - v.y) / L2, -(w.x - v.x) / L2};
        double a0 = std::atan2(n1.y, n1.x), phi = std::atan2(n2.y, n2.x) - a0;
        while (phi <= 0.0) phi += 2.0 * kPi;
        for (int j = 0; j <= m; ++j) {
            const double t = a0 + phi * j / m;
            path.push_back({v.x + r * std::cos(t), v.y + r * std::sin(t)});
        }
    }
    path.push_back(path.front());
    return path;
}

// Smallest signed distance from the part along every path segment (dense sampling).
double min_clearance(const std::vector<Vec2>& part, const std::vector<Vec2>& path) {
    double dmin = 1e300;
    for (std::size_t i = 0; i + 1 < path.size(); ++i)
        for (int k = 0; k <= 16; ++k) {
            const double t = k / 16.0;
            const Vec2 p{path[i].x + t * (path[i + 1].x - path[i].x), path[i].y + t * (path[i + 1].y - path[i].y)};
            dmin = std::min(dmin, lithocam::signed_distance_convex(part, p));
        }
    return dmin;
}

}  // namespace

int main() {
    lcheck::Gen g(20261112);
    const long N = 20000;
    long gouging_controls = 0, controls = 0;
    for (long c = 0; c < N; ++c) {
        const auto part = random_convex(g);
        if (part.size() < 3) continue;
        const double kerf = g.uniform(0.3, 12.0);          // hot wire 0.3 mm .. diamond wire / disc 12 mm
        const double over = g.uniform(1e-3, 0.5);          // allowed extra waste at a corner, mm
        const double r = kerf / 2.0, scale = 2000.0, tol = 1e-9 * scale;
        const auto path = lithocam::kerf_path_convex(part, kerf, over);
        const std::string tag = "case " + std::to_string(c) + " n=" + std::to_string(part.size()) + " kerf=" + std::to_string(kerf);

        // kerf_strip_outside_part (generalised): the wire centre never comes closer than kerf/2 to the part,
        // so the swept kerf never removes part material.
        const double clear = min_clearance(part, path);
        lcheck::expect(clear >= r - tol, "no_gouge", tag + " clearance=" + std::to_string(clear) + " r=" + std::to_string(r));

        // bounded waste: every path vertex lies within kerf/2 + max_overcut of the part
        double dmax = 0.0;
        for (const Vec2& p : path) dmax = std::max(dmax, lithocam::signed_distance_convex(part, p));
        lcheck::expect(dmax <= r + over + tol, "bounded_overcut", tag + " dmax=" + std::to_string(dmax));

        // kerf_touches_boundary: on the straight edges the kerf reaches the part exactly (clearance == r, no extra waste)
        lcheck::expect(clear <= r + tol, "touches_boundary", tag + " clearance=" + std::to_string(clear));

        // Negative control: inscribed chords at a coarse step gouge by the sagitta r(1 - cos(a/2)).
        if (c % 10 == 0) {
            ++controls;
            if (min_clearance(part, inscribed_path(part, kerf, 2)) < r - 1e-6 * r) ++gouging_controls;
        }
    }
    lcheck::expect(gouging_controls == controls, "negative_control_inscribed_gouges",
                   std::to_string(gouging_controls) + " of " + std::to_string(controls) + " inscribed paths gouged");
    std::printf("negative control: %ld of %ld inscribed-chord paths gouge, as expected\n", gouging_controls, controls);
    return lcheck::finish("kerf", N);
}
