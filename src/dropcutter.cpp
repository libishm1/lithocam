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

// Any cutter on a vertex: tip at v.z - f(d).
double gen_vertex(const Cutter& c, const P3& v, double x, double y) {
    double d = std::hypot(v.x - x, v.y - y);
    if (d > c.radius) return NEG;
    return v.z - profile(c, d);
}

// Any cutter on an edge: maximise the concave g(t) = e_z(t) - f(d(t)) over the part of the edge within the disc
// (golden-section search). Endpoints inside the disc are vertex contacts and are covered there too.
double gen_edge(const Cutter& c, const P3& a, const P3& b, double x, double y) {
    const double R = c.radius;
    double dx = b.x - a.x, dy = b.y - a.y;
    double fx = a.x - x, fy = a.y - y;
    double qa = dx * dx + dy * dy, qb = 2 * (fx * dx + fy * dy), qc = fx * fx + fy * fy - R * R;
    if (qa <= 0) return NEG;
    double disc = qb * qb - 4 * qa * qc;
    if (disc < 0) return NEG;
    double r = std::sqrt(disc);
    double lo = std::max(0.0, (-qb - r) / (2 * qa)), hi = std::min(1.0, (-qb + r) / (2 * qa));
    if (lo > hi) return NEG;
    auto g = [&](double t) {
        double px = a.x + t * dx, py = a.y + t * dy, pz = a.z + t * (b.z - a.z);
        double d = std::min(R, std::hypot(px - x, py - y));
        return pz - profile(c, d);
    };
    const double phi = 0.5 * (std::sqrt(5.0) - 1.0);
    double x1 = hi - phi * (hi - lo), x2 = lo + phi * (hi - lo), g1 = g(x1), g2 = g(x2);
    for (int i = 0; i < 200 && hi - lo > 1e-16; ++i) {
        if (g1 < g2) { lo = x1; x1 = x2; g1 = g2; x2 = lo + phi * (hi - lo); g2 = g(x2); }
        else         { hi = x2; x2 = x1; g2 = g1; x1 = hi - phi * (hi - lo); g1 = g(x1); }
    }
    return std::max({g(lo), g(hi), g1, g2});
}

// Facet contact by the support point of the cutter's lower surface in the direction -n (n = upward unit normal).
// Returns the tip height, or NEG when the contact point is outside the triangle.
double gen_facet(const Cutter& c, const Tri& t, const P3& n, double x, double y) {
    const double R = c.radius;
    double hl = std::hypot(n.x, n.y);
    double ux = hl > 1e-15 ? -n.x / hl : 0.0, uy = hl > 1e-15 ? -n.y / hl : 0.0;   // horizontal, down-slope side
    // candidate support points as (horizontal offset along u, height above the tip)
    double off = 0.0, hgt = 0.0;
    switch (c.kind) {
        case CutterKind::Flat: off = R; hgt = 0.0; break;
        case CutterKind::Ball: off = R * hl; hgt = R - R * n.z; break;   // centre - R n, centre at tip + R
        case CutterKind::Bull: {
            double a = R - c.corner, rr = c.corner;
            off = a + rr * hl;
            hgt = rr - rr * n.z;
            break;
        }
        case CutterKind::Cone: {
            // apex (0, 0) or rim point (R, R / tan): whichever is further along -n
            double h = R / std::tan(c.half_angle);
            double s_apex = 0.0, s_rim = -(n.x * ux * R + n.y * uy * R) - n.z * h;
            if (s_rim > s_apex + 1e-15) { off = R; hgt = h; }
            else { off = 0.0; hgt = 0.0; }
            break;
        }
    }
    if (hl <= 1e-15) off = 0.0;                      // horizontal facet: the tip's own (x, y) represents the flat
    double px = x + off * ux, py = y + off * uy;
    if (!in_tri_xy(t, px, py)) return NEG;
    return plane_z(t, n, px, py) - hgt;
}

}  // namespace

double profile(const Cutter& c, double d) {
    const double R = c.radius;
    switch (c.kind) {
        case CutterKind::Flat: return 0.0;
        case CutterKind::Ball: return R - std::sqrt(std::max(0.0, R * R - d * d));
        case CutterKind::Bull: {
            double a = R - c.corner;
            if (d <= a) return 0.0;
            double e = d - a;
            return c.corner - std::sqrt(std::max(0.0, c.corner * c.corner - e * e));
        }
        case CutterKind::Cone: return d / std::tan(c.half_angle);
    }
    return 0.0;
}

double drop_triangle(const Cutter& cutter, const Tri& t, double x, double y) {
    const double R = cutter.radius;
    double best = NEG;
    P3 n = cross(sub(t.b, t.a), sub(t.c, t.a));
    double nl = norm(n);
    if (nl > 0) {
        n = {n.x / nl, n.y / nl, n.z / nl};
        if (n.z < 0) n = {-n.x, -n.y, -n.z};         // upward normal
    }
    if (cutter.kind == CutterKind::Bull || cutter.kind == CutterKind::Cone) {
        best = std::max({gen_vertex(cutter, t.a, x, y), gen_vertex(cutter, t.b, x, y), gen_vertex(cutter, t.c, x, y),
                         gen_edge(cutter, t.a, t.b, x, y), gen_edge(cutter, t.b, t.c, x, y),
                         gen_edge(cutter, t.c, t.a, x, y)});
        if (nl > 0 && n.z > 1e-12) best = std::max(best, gen_facet(cutter, t, n, x, y));
        return best;
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
