// Copyright 2026 The lithocam authors. SPDX-License-Identifier: Apache-2.0
#include "lithocam/dropcutter.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#ifdef LITHOCAM_MUTATE
extern int lithocam_mutation;   // defined by tests/test_dropcutter_mutation.cpp
#endif

namespace lithocam {

namespace {

constexpr double NEG = -std::numeric_limits<double>::infinity();
constexpr double POSINF = std::numeric_limits<double>::infinity();

// Mutation hooks for tests/test_dropcutter_mutation.cpp, which builds this file with LITHOCAM_MUTATE and checks that
// every deliberate defect makes the property tests fail. Compiled out (constant false) in the library.
#ifdef LITHOCAM_MUTATE
inline bool MUT(int k) { return ::lithocam_mutation == k; }
#else
constexpr bool MUT(int) { return false; }
#endif

// ---- general profile (line and arc pieces; see ProfileSeg)

double arc_rho(const ProfileSeg& s) { return std::hypot(s.r - s.cr, s.h - s.ch); }

// f(d) of a validated profile; d is clamped to the last piece's end.
double profile_eval(const std::vector<ProfileSeg>& S, double d) {
    double r0 = 0.0, h0 = 0.0;
    for (std::size_t i = 0; i < S.size(); ++i) {
        const ProfileSeg& s = S[i];
        if (d <= s.r || i + 1 == S.size()) {
            if (d > s.r) d = s.r;
            if (!s.arc) {
                double dr = s.r - r0;
                return dr > 0 ? h0 + (d - r0) * (s.h - h0) / dr : s.h;
            }
            double rho = arc_rho(s), e = d - s.cr;
            return s.ch - std::sqrt(std::max(0.0, rho * rho - e * e));
        }
        r0 = s.r;
        h0 = s.h;
    }
    return 0.0;
}

// Radii of the cutter's support points for a facet whose upward unit normal has horizontal part hl and vertical part
// nz, with s0 = hl / nz. The support point minimises nz f(r) - hl r over r in [0, R]: it is where the profile's
// slope f'(r) first reaches s0 (f' is nondecreasing). A line piece whose slope equals s0 touches along its whole
// generator, so both its ends are returned. An arc piece with s0 inside its slope range touches at the tangent point
// r = cr + rho s0 / sqrt(1 + s0^2). Returns the number of radii written to out[0..1].
int profile_support(const Cutter& c, double s0, double out[2]) {
    double r0 = 0.0, h0 = 0.0;
    for (const ProfileSeg& s : c.segs) {
        double s_start, s_end;
        if (!s.arc) {
            s_start = s_end = (s.h - h0) / (s.r - r0);
        } else {
            double den0 = s.ch - h0, den1 = s.ch - s.h;
            s_start = den0 > 1e-300 ? (r0 - s.cr) / den0 : POSINF;
            s_end = den1 > 1e-300 ? (s.r - s.cr) / den1 : POSINF;
        }
        if (s_end >= s0) {
            if (!s.arc) {
                out[0] = r0;
                if (std::fabs(s_start - s0) <= 1e-12 * (1.0 + s0)) {
                    out[1] = s.r;
                    return 2;
                }
                return 1;
            }
            if (s_start >= s0) {
                out[0] = r0;
                return 1;
            }
            out[0] = std::min(s.r, std::max(r0, s.cr + arc_rho(s) * s0 / std::sqrt(1.0 + s0 * s0)));
            if (MUT(3)) out[0] = s.r;               // mutant: arc support at the arc's end instead of the tangent
            return 1;
        }
        r0 = s.r;
        h0 = s.h;
    }
    out[0] = c.radius;
    return 1;
}

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
    if (c.kind == CutterKind::Profile && MUT(2)) return NEG;   // mutant: no edge contact
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
    if (c.kind == CutterKind::Profile) {
        if (MUT(1)) return NEG;                      // mutant: no facet contact
        double rs[2];
        int cnt = profile_support(c, hl / n.z, rs);
        double best = NEG;
        for (int k = 0; k < cnt; ++k) {
            double px = x + rs[k] * ux, py = y + rs[k] * uy;
            if (in_tri_xy(t, px, py)) best = std::max(best, plane_z(t, n, px, py) - profile_eval(c.segs, rs[k]));
        }
        return best;
    }
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
        case CutterKind::Profile:
        case CutterKind::Disc: break;                // handled above / by disc_triangle
    }
    if (hl <= 1e-15) off = 0.0;                      // horizontal facet: the tip's own (x, y) represents the flat
    double px = x + off * ux, py = y + off * uy;
    if (!in_tri_xy(t, px, py)) return NEG;
    return plane_z(t, n, px, py) - hgt;
}

// ---- thin disc (saw slot): axis horizontal, perpendicular to the cut direction.
//
// With u the horizontal coordinate along the cut direction and l the one along the axis (both from the tip's xy), a
// mesh point p with |l| <= w/2 and |u| <= R forces tip >= p.z - f(u), f(u) = R - sqrt(R^2 - u^2) (the disc's rim, a
// circle in the plane l = const). The mesh points that matter are those of the triangle inside the slab |l| <= w/2:
// a convex polygon P. On the triangle's plane p.z is linear in (l, u), so for fixed u the bound p.z - f(u) is linear in
// l and its maximum over P lies on P's boundary; a vertical triangle's maximum lies on its top edges, also on the
// boundary. The boundary is made of pieces of the triangle's edges (rim-edge contacts) and the chords where the
// triangle crosses the disc's two flat faces l = +-w/2 (face contacts). Along a boundary piece the bound is concave
// in the edge parameter, with the closed-form stationary point u* = R dz sgn(du) / hypot(du, dz) (the rim point whose
// tangent has the piece's slope), clamped to the piece's part inside |u| <= R.
struct DV {
    double u, l, z;
};

double disc_edge(double R, const DV& A, const DV& B) {
    double du = B.u - A.u, dz = B.z - A.z;
    double tlo = 0.0, thi = 1.0;
    if (du == 0.0) {
        if (std::fabs(A.u) > R) return NEG;
    } else {
        double t1 = (-R - A.u) / du, t2 = (R - A.u) / du;
        tlo = std::max(0.0, std::min(t1, t2));
        thi = std::min(1.0, std::max(t1, t2));
        if (tlo > thi) return NEG;
    }
    auto g = [&](double t) {
        double u = A.u + t * du;
        return A.z + t * dz + std::sqrt(std::max(0.0, R * R - u * u)) - R;
    };
    double best = std::max(g(tlo), g(thi));
    double hyp = std::hypot(du, dz);
    if (du != 0.0 && hyp > 0.0 && !MUT(6)) {         // mutant 6: no rim stationary point
        double ustar = R * dz * (du > 0 ? 1.0 : -1.0) / hyp;
        double tstar = (ustar - A.u) / du;
        if (tstar > tlo && tstar < thi) best = std::max(best, g(tstar));
    }
    return best;
}

// Keep the part of the polygon with sign * l <= hw (Sutherland-Hodgman against one half-plane).
int disc_clip(const DV* in, int n, DV* out, double sign, double hw) {
    int m = 0;
    for (int i = 0; i < n; ++i) {
        const DV& p = in[i];
        const DV& q = in[(i + 1) % n];
        double dp = sign * p.l - hw, dq = sign * q.l - hw;   // inside when <= 0
        if (dp <= 0) out[m++] = p;
        if ((dp < 0 && dq > 0) || (dp > 0 && dq < 0)) {
            double t = dp / (dp - dq);
            out[m++] = {p.u + t * (q.u - p.u), p.l + t * (q.l - p.l), p.z + t * (q.z - p.z)};
        }
    }
    return m;
}

double disc_triangle(const Cutter& c, const Tri& t, double x, double y) {
    const double R = c.radius, hw = 0.5 * c.thickness;
    const double ct = std::cos(c.heading), st = std::sin(c.heading);
    auto to = [&](const P3& p) {
        double dx = p.x - x, dy = p.y - y;
        return DV{dx * ct + dy * st, -dx * st + dy * ct, p.z};
    };
    DV a[12] = {to(t.a), to(t.b), to(t.c)}, b[12];
    int n = disc_clip(a, 3, b, 1.0, hw);
    if (n == 0) return NEG;
    if (MUT(7)) {                                    // mutant 7: the second face is ignored
        for (int i = 0; i < n; ++i) a[i] = b[i];
    } else {
        n = disc_clip(b, n, a, -1.0, hw);
        if (n == 0) return NEG;
    }
    double best = NEG;
    if (n == 1) return disc_edge(R, a[0], a[0]);
    for (int i = 0; i < n; ++i) {
        const DV& p = a[i];
        const DV& q = a[(i + 1) % n];
        // mutant 5: no face contact (chords lying in a face plane are skipped)
        if (MUT(5) && std::fabs(std::fabs(p.l) - hw) < 1e-12 && std::fabs(std::fabs(q.l) - hw) < 1e-12) continue;
        best = std::max(best, disc_edge(R, p, q));
    }
    return best;
}

}  // namespace

std::string validate_profile(const Cutter& c) {
    if (c.segs.empty()) return "profile has no segments";
    double r0 = 0.0, h0 = 0.0, prev = 0.0;
    for (const ProfileSeg& s : c.segs) {
        if (!std::isfinite(s.r) || !std::isfinite(s.h) || !std::isfinite(s.cr) || !std::isfinite(s.ch))
            return "profile has a non-finite value";
        const double scale = 1.0 + std::fabs(s.r) + std::fabs(s.h);
        if (!(s.r > r0 + 1e-12 * scale)) return "profile radius must increase along the pieces";
        if (s.h < h0 - 1e-12 * scale) return "profile height must not decrease";
        double s_start, s_end;
        if (!s.arc) {
            s_start = s_end = (s.h - h0) / (s.r - r0);
        } else {
            const double rho = arc_rho(s), rho0 = std::hypot(r0 - s.cr, h0 - s.ch);
            if (!(rho > 0)) return "arc has zero radius";
            if (std::fabs(rho - rho0) > 1e-9 * (1.0 + rho)) return "arc does not start where the previous piece ends";
            if (s.cr > r0 + 1e-9 * scale) return "arc centre lies right of the arc's start (the surface would fall)";
            if (s.h > s.ch + 1e-9 * scale || h0 > s.ch + 1e-9 * scale)
                return "arc must lie below its centre (lower-right quarter of the circle)";
            const double den0 = s.ch - h0, den1 = s.ch - s.h;
            if (!(den0 > 1e-12 * rho)) return "arc starts at its vertical tangent";
            s_start = (r0 - s.cr) / den0;
            s_end = den1 > 1e-12 * rho ? (s.r - s.cr) / den1 : POSINF;
        }
        if (s_start < prev - 1e-9 * (1.0 + (std::isfinite(prev) ? prev : 0.0)))
            return "profile is not convex (slope decreases)";
        prev = s_end;
        r0 = s.r;
        h0 = s.h;
    }
    if (std::fabs(c.radius - r0) > 1e-12 * (1.0 + r0)) return "radius differs from the last piece's end";
    return "";
}

Cutter profile_cutter(const std::vector<ProfileSeg>& segs) {
    Cutter c{CutterKind::Profile, segs.empty() ? 0.0 : segs.back().r};
    c.segs = segs;
    const std::string problem = validate_profile(c);
    if (!problem.empty()) throw std::invalid_argument("profile cutter: " + problem);
    return c;
}

Cutter tapered_cutter(double radius, double flat_radius, double corner, double half_angle) {
    constexpr double kHalfPi = 1.57079632679489661923;
    if (!(radius > 0) || !(flat_radius >= 0) || !(corner >= 0) || !(half_angle > 0 && half_angle < kHalfPi))
        throw std::invalid_argument("tapered cutter: radius > 0, flat_radius >= 0, corner >= 0, 0 < half_angle < pi/2");
    std::vector<ProfileSeg> S;
    double r = 0.0, h = 0.0;
    if (flat_radius > 0) {
        S.push_back({false, flat_radius, 0.0, 0.0, 0.0});
        r = flat_radius;
    }
    const double ca = std::cos(half_angle), sa = std::sin(half_angle);
    if (corner > 0) {
        // arc centre (r, corner); tangent to the flank where the slope is cot(half_angle)
        r = r + corner * ca;
        h = corner * (1.0 - sa);
        S.push_back({true, r, h, S.empty() ? 0.0 : S.back().r, corner});
    }
    if (!(radius > r + 1e-12 * radius))
        throw std::invalid_argument("tapered cutter: the flat and corner arc do not fit inside the radius");
    S.push_back({false, radius, h + (radius - r) * ca / sa, 0.0, 0.0});
    return profile_cutter(S);
}

Cutter burr_cutter(const std::string& shape, double radius, double length, double corner) {
    if (!(radius > 0)) throw std::invalid_argument("burr: radius must be positive");
    if (shape == "ball") return profile_cutter({{true, radius, radius, 0.0, radius}});
    if (!(length > 0)) throw std::invalid_argument("burr: length must be positive");
    if (shape == "flame") {
        if (length < radius) throw std::invalid_argument("burr: a flame (ogive) needs length >= radius");
        const double rho = (radius * radius + length * length) / (2.0 * radius);
        return profile_cutter({{true, radius, length, radius - rho, length}});
    }
    if (shape == "cone") return profile_cutter({{false, radius, length, 0.0, 0.0}});
    if (shape == "cylinder") {
        if (!(corner >= 0) || corner > radius) throw std::invalid_argument("burr: corner must be in [0, radius]");
        std::vector<ProfileSeg> S;
        const double a = radius - corner;
        if (corner > 0) {
            if (a > 0) S.push_back({false, a, 0.0, 0.0, 0.0});
            S.push_back({true, radius, corner, a, corner});
        } else {
            S.push_back({false, radius, 0.0, 0.0, 0.0});
        }
        return profile_cutter(S);
    }
    throw std::invalid_argument("burr: unknown shape '" + shape + "' (flame, ball, cone, cylinder)");
}

Cutter disc_cutter(double radius, double thickness, double heading) {
    if (!(radius > 0) || !(thickness > 0) || !std::isfinite(heading))
        throw std::invalid_argument("disc cutter: radius and thickness must be positive");
    Cutter c{CutterKind::Disc, radius};
    c.thickness = thickness;
    c.heading = heading;
    return c;
}

double profile(const Cutter& c, double d) {
    const double R = c.radius;
    switch (c.kind) {
        case CutterKind::Profile: return profile_eval(c.segs, d);
        case CutterKind::Disc: throw std::invalid_argument("a disc cutter has no rotational profile");
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
    if (cutter.kind == CutterKind::Disc) return disc_triangle(cutter, t, x, y);
    if (cutter.kind == CutterKind::Profile && cutter.segs.empty())
        throw std::invalid_argument("profile cutter has no segments (use profile_cutter)");
    double best = NEG;
    P3 n = cross(sub(t.b, t.a), sub(t.c, t.a));
    double nl = norm(n);
    if (nl > 0) {
        n = {n.x / nl, n.y / nl, n.z / nl};
        if (n.z < 0) n = {-n.x, -n.y, -n.z};         // upward normal
    }
    if (cutter.kind == CutterKind::Bull || cutter.kind == CutterKind::Cone || cutter.kind == CutterKind::Profile) {
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
    // a disc's footprint is a rectangle 2R by w: its half-diagonal bounds the query square
    mesh.query(x, y, cutter.kind == CutterKind::Disc ? std::hypot(cutter.radius, 0.5 * cutter.thickness) : cutter.radius,
               cand);
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
