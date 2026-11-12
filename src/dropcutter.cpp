// Copyright 2026 The lithocam authors. SPDX-License-Identifier: Apache-2.0
#include "lithocam/dropcutter.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace lithocam {

namespace {

constexpr double NEG = -std::numeric_limits<double>::infinity();

P3 sub(const P3& a, const P3& b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
P3 cross(const P3& a, const P3& b) { return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x}; }
double dot(const P3& a, const P3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
double norm(const P3& a) { return std::sqrt(dot(a, a)); }

// Is the xy point (px, py) inside the xy projection of the triangle (inclusive)?
bool in_tri_xy(const Tri& t, double px, double py) {
    auto s = [](double ax, double ay, double bx, double by, double cx, double cy) {
        return (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
    };
    double d1 = s(t.a.x, t.a.y, t.b.x, t.b.y, px, py);
    double d2 = s(t.b.x, t.b.y, t.c.x, t.c.y, px, py);
    double d3 = s(t.c.x, t.c.y, t.a.x, t.a.y, px, py);
    const double eps = 1e-12 * (1.0 + std::fabs(px) + std::fabs(py));
    bool neg = d1 < -eps || d2 < -eps || d3 < -eps;
    bool pos = d1 > eps || d2 > eps || d3 > eps;
    return !(neg && pos);
}

// Height of the triangle's plane at (px, py); requires a non-vertical triangle.
double plane_z(const Tri& t, const P3& n, double px, double py) {
    return t.a.z - (n.x * (px - t.a.x) + n.y * (py - t.a.y)) / n.z;
}

double ball_vertex(double R, const P3& v, double x, double y) {
    double d2 = (v.x - x) * (v.x - x) + (v.y - y) * (v.y - y);
    if (d2 > R * R) return NEG;
    return v.z + std::sqrt(R * R - d2) - R;
}

double flat_vertex(double R, const P3& v, double x, double y) {
    double d2 = (v.x - x) * (v.x - x) + (v.y - y) * (v.y - y);
    return d2 <= R * R ? v.z : NEG;
}

// Ball touching the segment a-b in its interior: centre on the vertical line through (x, y) at distance R from the
// segment's line, highest root, foot point inside the segment.
double ball_edge(double R, const P3& a, const P3& b, double x, double y) {
    P3 d = sub(b, a);
    double L = norm(d);
    if (L <= 0) return NEG;
    P3 u{d.x / L, d.y / L, d.z / L};
    double A = 1.0 - u.z * u.z;
    if (A < 1e-14) return NEG;                       // vertical edge: its top vertex is the contact
    double wx = x - a.x, wy = y - a.y;
    double k = wx * u.x + wy * u.y;
    double C = wx * wx + wy * wy - k * k - R * R;    // s^2 A - 2 k uz s + C = 0, s = zc - a.z
    double disc = k * k * u.z * u.z - A * C;
    if (disc < 0) return NEG;
    double s = (k * u.z + std::sqrt(disc)) / A;
    double t = (k + s * u.z) / L;                    // foot point parameter along a-b
    if (t < 0 || t > 1) return NEG;
    return a.z + s - R;
}

// Flat cutter on an edge: the highest end of the part of the segment within the disc.
double flat_edge(double R, const P3& a, const P3& b, double x, double y) {
    double dx = b.x - a.x, dy = b.y - a.y;
    double fx = a.x - x, fy = a.y - y;
    double qa = dx * dx + dy * dy, qb = 2 * (fx * dx + fy * dy), qc = fx * fx + fy * fy - R * R;
    if (qa <= 0) return NEG;                         // vertical edge: vertices cover it
    double disc = qb * qb - 4 * qa * qc;
    if (disc < 0) return NEG;
    double r = std::sqrt(disc);
    double t0 = std::max(0.0, (-qb - r) / (2 * qa)), t1 = std::min(1.0, (-qb + r) / (2 * qa));
    if (t0 > t1) return NEG;
    return std::max(a.z + t0 * (b.z - a.z), a.z + t1 * (b.z - a.z));
}

}  // namespace

double drop_triangle(const Cutter& cutter, const Tri& t, double x, double y) {
    const double R = cutter.radius;
    double best = NEG;
    P3 n = cross(sub(t.b, t.a), sub(t.c, t.a));
    double nl = norm(n);
    if (nl > 0) {
        n = {n.x / nl, n.y / nl, n.z / nl};
        if (n.z < 0) n = {-n.x, -n.y, -n.z};         // upward normal
    }
    if (cutter.kind == CutterKind::Ball) {
        best = std::max({ball_vertex(R, t.a, x, y), ball_vertex(R, t.b, x, y), ball_vertex(R, t.c, x, y),
                         ball_edge(R, t.a, t.b, x, y), ball_edge(R, t.b, t.c, x, y), ball_edge(R, t.c, t.a, x, y)});
        if (nl > 0 && n.z > 1e-12) {
            // facet contact: the contact point is (x, y) - R * (n.x, n.y) on the plane
            double px = x - R * n.x, py = y - R * n.y;
            if (in_tri_xy(t, px, py)) best = std::max(best, plane_z(t, n, px, py) + R * n.z - R);
        }
    } else {
        best = std::max({flat_vertex(R, t.a, x, y), flat_vertex(R, t.b, x, y), flat_vertex(R, t.c, x, y),
                         flat_edge(R, t.a, t.b, x, y), flat_edge(R, t.b, t.c, x, y), flat_edge(R, t.c, t.a, x, y)});
        if (nl > 0 && n.z > 1e-12) {
            double gx = -n.x, gy = -n.y, gl = std::hypot(gx, gy);  // up-slope direction of the plane
            double px = x, py = y;
            if (gl > 1e-15) {
                px += R * gx / gl;
                py += R * gy / gl;
            }
            if (in_tri_xy(t, px, py)) best = std::max(best, plane_z(t, n, px, py));
        }
    }
    return best;
}

TriMesh::TriMesh(std::vector<Tri> tris, double cell) : tris_(std::move(tris)) {
    if (tris_.empty()) return;
    double xmin = 1e300, ymin = 1e300, xmax = -1e300, ymax = -1e300, span = 0;
    for (const Tri& t : tris_) {
        for (const P3* p : {&t.a, &t.b, &t.c}) {
            xmin = std::min(xmin, p->x); xmax = std::max(xmax, p->x);
            ymin = std::min(ymin, p->y); ymax = std::max(ymax, p->y);
        }
        span += std::max({std::fabs(t.a.x - t.b.x), std::fabs(t.b.x - t.c.x), std::fabs(t.a.y - t.b.y),
                          std::fabs(t.b.y - t.c.y)});
    }
    cell_ = cell > 0 ? cell : std::max(span / static_cast<double>(tris_.size()), 1e-9 * (1 + xmax - xmin));
    x0_ = xmin; y0_ = ymin;
    nx_ = std::max(1, static_cast<int>(std::ceil((xmax - xmin) / cell_)) + 1);
    ny_ = std::max(1, static_cast<int>(std::ceil((ymax - ymin) / cell_)) + 1);
    while (static_cast<double>(nx_) * ny_ > 4e6) {     // cap memory for very fine meshes
        cell_ *= 2;
        nx_ = static_cast<int>(std::ceil((xmax - xmin) / cell_)) + 1;
        ny_ = static_cast<int>(std::ceil((ymax - ymin) / cell_)) + 1;
    }
    bins_.assign(static_cast<std::size_t>(nx_) * ny_, {});
    for (std::size_t i = 0; i < tris_.size(); ++i) {
        const Tri& t = tris_[i];
        double bx0 = std::min({t.a.x, t.b.x, t.c.x}), bx1 = std::max({t.a.x, t.b.x, t.c.x});
        double by0 = std::min({t.a.y, t.b.y, t.c.y}), by1 = std::max({t.a.y, t.b.y, t.c.y});
        int i0 = static_cast<int>((bx0 - x0_) / cell_), i1 = static_cast<int>((bx1 - x0_) / cell_);
        int j0 = static_cast<int>((by0 - y0_) / cell_), j1 = static_cast<int>((by1 - y0_) / cell_);
        for (int ii = std::max(0, i0); ii <= std::min(nx_ - 1, i1); ++ii)
            for (int jj = std::max(0, j0); jj <= std::min(ny_ - 1, j1); ++jj)
                bins_[static_cast<std::size_t>(ii) * ny_ + jj].push_back(i);
    }
    stamp_.assign(tris_.size(), 0);
}

void TriMesh::query(double x, double y, double r, std::vector<std::size_t>& out) const {
    out.clear();
    if (tris_.empty()) return;
    if (++tick_ == 0) {
        std::fill(stamp_.begin(), stamp_.end(), 0);
        tick_ = 1;
    }
    int i0 = static_cast<int>(std::floor((x - r - x0_) / cell_)), i1 = static_cast<int>(std::floor((x + r - x0_) / cell_));
    int j0 = static_cast<int>(std::floor((y - r - y0_) / cell_)), j1 = static_cast<int>(std::floor((y + r - y0_) / cell_));
    for (int ii = std::max(0, i0); ii <= std::min(nx_ - 1, i1); ++ii)
        for (int jj = std::max(0, j0); jj <= std::min(ny_ - 1, j1); ++jj)
            for (std::size_t k : bins_[static_cast<std::size_t>(ii) * ny_ + jj])
                if (stamp_[k] != tick_) {
                    stamp_[k] = tick_;
                    out.push_back(k);
                }
}

double drop_cutter(const Cutter& cutter, const TriMesh& mesh, double x, double y) {
    thread_local std::vector<std::size_t> cand;
    mesh.query(x, y, cutter.radius, cand);
    double best = NEG;
    for (std::size_t k : cand) best = std::max(best, drop_triangle(cutter, mesh.triangles()[k], x, y));
    return best;
}

double drop_cutter_brute(const Cutter& cutter, const std::vector<Tri>& tris, double x, double y) {
    double best = NEG;
    for (const Tri& t : tris) best = std::max(best, drop_triangle(cutter, t, x, y));
    return best;
}

}  // namespace lithocam
