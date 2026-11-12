// Copyright 2026 The lithocam authors. SPDX-License-Identifier: Apache-2.0
#include "lithocam/kerf.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace lithocam {
namespace {

constexpr double kPi = 3.14159265358979323846;

Vec2 sub(Vec2 a, Vec2 b) { return {a.x - b.x, a.y - b.y}; }
Vec2 add(Vec2 a, Vec2 b) { return {a.x + b.x, a.y + b.y}; }
Vec2 mul(double k, Vec2 a) { return {k * a.x, k * a.y}; }
double dot(Vec2 a, Vec2 b) { return a.x * b.x + a.y * b.y; }
double cross(Vec2 a, Vec2 b) { return a.x * b.y - a.y * b.x; }
double norm(Vec2 a) { return std::hypot(a.x, a.y); }

double dist_point_segment(Vec2 p, Vec2 a, Vec2 b) {
    const Vec2 d = sub(b, a);
    const double L2 = dot(d, d);
    double t = L2 > 0.0 ? dot(sub(p, a), d) / L2 : 0.0;
    t = std::clamp(t, 0.0, 1.0);
    return norm(sub(p, add(a, mul(t, d))));
}

// Outward unit normal of edge a->b of a counter-clockwise polygon.
Vec2 outward_normal(Vec2 a, Vec2 b) {
    const Vec2 d = sub(b, a);
    const double L = norm(d);
    return {d.y / L, -d.x / L};
}

}  // namespace

double signed_distance_convex(const std::vector<Vec2>& poly, Vec2 p) {
    const std::size_t n = poly.size();
    double dmin = std::numeric_limits<double>::infinity();
    bool inside = true;
    for (std::size_t i = 0; i < n; ++i) {
        const Vec2 a = poly[i], b = poly[(i + 1) % n];
        dmin = std::min(dmin, dist_point_segment(p, a, b));
        if (cross(sub(b, a), sub(p, a)) < 0.0) inside = false;
    }
    return inside ? -dmin : dmin;
}

std::vector<Vec2> kerf_path_convex(const std::vector<Vec2>& part, double kerf, double max_overcut) {
    const std::size_t n = part.size();
    if (n < 3 || !(kerf > 0.0) || !(max_overcut > 0.0))
        throw std::domain_error("kerf_path_convex: need >= 3 vertices, kerf > 0, max_overcut > 0");
    const double r = kerf / 2.0;
    // Circumscribed step angle a: overshoot r (1/cos(a/2) - 1) <= max_overcut  <=>  a <= 2 acos(r / (r + max_overcut)).
    const double a_max = 2.0 * std::acos(r / (r + max_overcut));

    std::vector<Vec2> path;
    for (std::size_t i = 0; i < n; ++i) {
        const Vec2 v = part[(i + 1) % n];
        const Vec2 n_in = outward_normal(part[i], v);                 // edge i ends at v
        const Vec2 n_out = outward_normal(v, part[(i + 2) % n]);      // edge i+1 starts at v
        path.push_back(add(part[i], mul(r, n_in)));                   // start of the offset edge i
        path.push_back(add(v, mul(r, n_in)));                         // end of the offset edge i: tangent point
        const double a0 = std::atan2(n_in.y, n_in.x);
        double phi = std::atan2(n_out.y, n_out.x) - a0;               // exterior turn at v, in (0, pi) for convex CCW
        while (phi <= 0.0) phi += 2.0 * kPi;
        if (phi >= 2.0 * kPi - 1e-15) phi = 0.0;
        if (phi > 0.0) {
            const int m = std::max(1, static_cast<int>(std::ceil(phi / a_max)));
            const double a = phi / m;
            const double rc = r / std::cos(a / 2.0);                  // circumscribed vertex radius
            for (int j = 0; j < m; ++j) {
                const double t = a0 + (j + 0.5) * a;
                path.push_back(add(v, mul(rc, Vec2{std::cos(t), std::sin(t)})));
            }
        }
    }
    path.push_back(path.front());
    return path;
}

}  // namespace lithocam
