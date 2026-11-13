// Copyright 2026 The lithocam authors. SPDX-License-Identifier: Apache-2.0
//
// Shared property checks for the general-profile and thin-disc drop-cutters (tests/test_dropcutter_profile.cpp,
// test_dropcutter_disc.cpp, test_dropcutter_mutation.cpp). Every check returns a bit mask of failures so a caller can
// count failing cases without touching the lcheck counters (the mutation test expects failures).
//
//   bit 1  no gouge, triangle sampling: a sampled triangle point lies above the cutter's lower surface
//   bit 2  no gouge, cutter sampling:   a sampled point of the cutter's lower surface lies above the triangle's
//                                       plane where the triangle is overhead, i.e. the signed distance is < -eps
//   bit 4  tight contact:               the height differs from an independent maximiser (nested golden-section
//                                       search over the triangle, a penalty beyond the cutter's reach) by more than eps
#pragma once

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>

#include "check.hpp"
#include "lithocam/dropcutter.hpp"

namespace props {

using namespace lithocam;

constexpr double kPi = 3.14159265358979323846;
constexpr double kNeg = -std::numeric_limits<double>::infinity();
constexpr double kPenalty = 1e6;   // slope of the extension beyond the cutter's reach (reference solver only)

inline Tri rand_tri(lcheck::Gen& g, double span) {
    auto rp = [&] { return P3{g.uniform(-span, span), g.uniform(-span, span), g.uniform(-span, span)}; };
    return {rp(), rp(), rp()};
}

// Upward unit normal; false for a degenerate triangle.
inline bool normal_of(const Tri& t, P3& n) {
    P3 u{t.b.x - t.a.x, t.b.y - t.a.y, t.b.z - t.a.z}, v{t.c.x - t.a.x, t.c.y - t.a.y, t.c.z - t.a.z};
    n = {u.y * v.z - u.z * v.y, u.z * v.x - u.x * v.z, u.x * v.y - u.y * v.x};
    double l = std::sqrt(n.x * n.x + n.y * n.y + n.z * n.z);
    if (!(l > 0)) return false;
    n = {n.x / l, n.y / l, n.z / l};
    if (n.z < 0) n = {-n.x, -n.y, -n.z};
    return true;
}

// Largest |gradient| of the triangle's plane over xy (infinity for a vertical or degenerate triangle).
inline double max_slope(const Tri& t) {
    P3 n;
    if (!normal_of(t, n) || n.z < 1e-9) return std::numeric_limits<double>::infinity();
    return std::hypot(n.x, n.y) / n.z;
}

inline bool inside_xy(const Tri& t, double px, double py, double eps) {
    auto s = [](double ax, double ay, double bx, double by, double cx, double cy) {
        return (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
    };
    double d1 = s(t.a.x, t.a.y, t.b.x, t.b.y, px, py), d2 = s(t.b.x, t.b.y, t.c.x, t.c.y, px, py),
           d3 = s(t.c.x, t.c.y, t.a.x, t.a.y, px, py);
    bool neg = d1 < -eps || d2 < -eps || d3 < -eps, pos = d1 > eps || d2 > eps || d3 > eps;
    return !(neg && pos);
}

inline double plane_at(const Tri& t, const P3& n, double px, double py) {
    return t.a.z - (n.x * (px - t.a.x) + n.y * (py - t.a.y)) / n.z;
}

// Maximise a concave function on [lo, hi] by golden-section search.
inline double golden_max(const std::function<double(double)>& f, double lo, double hi, int iters = 70) {
    const double phi = 0.5 * (std::sqrt(5.0) - 1.0);
    double x1 = hi - phi * (hi - lo), x2 = lo + phi * (hi - lo), f1 = f(x1), f2 = f(x2);
    for (int i = 0; i < iters; ++i) {
        if (f1 < f2) { lo = x1; x1 = x2; f1 = f2; x2 = lo + phi * (hi - lo); f2 = f(x2); }
        else         { hi = x2; x2 = x1; f2 = f1; x1 = hi - phi * (hi - lo); f1 = f(x1); }
    }
    return std::max({f1, f2, f(lo), f(hi)});
}

// ------------------------------------------------------------------ rotationally symmetric profile cutters

inline double f_ext(const Cutter& c, double d) {
    return d <= c.radius ? profile(c, d) : profile(c, c.radius) + kPenalty * (d - c.radius);
}

// Dense sampling of the triangle: max of p.z - f(d) over sampled points within the cutter's reach.
inline double sampled_profile(const Cutter& c, const Tri& t, double x, double y, int n) {
    double best = kNeg;
    for (int i = 0; i <= n; ++i)
        for (int j = 0; i + j <= n; ++j) {
            double u = double(i) / n, v = double(j) / n, w = 1 - u - v;
            double px = w * t.a.x + u * t.b.x + v * t.c.x, py = w * t.a.y + u * t.b.y + v * t.c.y;
            double pz = w * t.a.z + u * t.b.z + v * t.c.z;
            double d = std::hypot(px - x, py - y);
            if (d <= c.radius) best = std::max(best, pz - profile(c, d));
        }
    return best;
}

// Independent maximiser: g(u, v) = p.z - f_ext(d(p)) is concave on the triangle (affine minus convex of convex).
inline double reference_profile(const Cutter& c, const Tri& t, double x, double y) {
    auto g = [&](double u, double v) {
        double w = 1 - u - v;
        double px = w * t.a.x + u * t.b.x + v * t.c.x, py = w * t.a.y + u * t.b.y + v * t.c.y;
        double pz = w * t.a.z + u * t.b.z + v * t.c.z;
        return pz - f_ext(c, std::hypot(px - x, py - y));
    };
    return golden_max([&](double u) { return golden_max([&](double v) { return g(u, v); }, 0.0, 1.0 - u, 60); },
                      0.0, 1.0, 60);
}

// All three checks for one random case. `z` is the height under test (so a mutant can be checked against the truth).
inline int check_profile(const Cutter& c, const Tri& t, double x, double y, double z) {
    int mask = 0;
    const int S = 60;
    double zs = sampled_profile(c, t, x, y, S);
    P3 n;
    const bool plane_ok = normal_of(t, n) && n.z > 1e-9;
    const double zscale = 1.0 + std::fabs(z) + (std::isfinite(zs) ? std::fabs(zs) : 0.0);
    if (std::isfinite(zs) && !(z >= zs - 1e-9 * zscale)) mask |= 1;
    if (!std::isfinite(zs) && std::isfinite(z)) {
        // nothing sampled inside the reach: the exact answer may touch a sliver but cannot exceed the top vertex
        if (z > std::max({t.a.z, t.b.z, t.c.z}) + 1e-9) mask |= 1;
    }
    if (std::isfinite(z) && plane_ok) {
        const int NR = 16, NP = 32;
        const double eps = 1e-9 * (1.0 + std::fabs(z) + std::fabs(t.a.z));
        for (int i = 0; i <= NR && !(mask & 2); ++i) {
            double r = c.radius * double(i) / NR;
            for (int k = 0; k < (i == 0 ? 1 : NP); ++k) {
                double phi = 2 * kPi * k / NP;
                double qx = x + r * std::cos(phi), qy = y + r * std::sin(phi), qz = z + profile(c, r);
                if (inside_xy(t, qx, qy, 1e-12) && plane_at(t, n, qx, qy) > qz + eps) {
                    mask |= 2;
                    break;
                }
            }
        }
    }
    if (std::isfinite(z) && max_slope(t) <= 20.0) {
        double zr = reference_profile(c, t, x, y);
        if (!(std::fabs(z - zr) <= 1e-6 * (1.0 + std::fabs(z)))) mask |= 4;
    }
    return mask;
}

// A random convex profile cutter: kinds 0-6 are named shapes, 7-8 random chains of line and arc pieces.
inline bool random_profile(lcheck::Gen& g, int k, Cutter& out) {
    const double R = g.uniform(0.5, 15);
    try {
        switch (k % 9) {
            case 0: out = tapered_cutter(R, 0.0, g.uniform(0.05, 0.95) * R, g.uniform(0.05, 1.45)); return true;
            case 1: out = tapered_cutter(R, g.uniform(0.05, 0.9) * R, 0.0, g.uniform(0.05, 1.45)); return true;
            case 2: out = tapered_cutter(R, g.uniform(0.05, 0.5) * R, g.uniform(0.02, 0.3) * R, g.uniform(0.05, 1.45)); return true;
            case 3: out = burr_cutter("flame", R, g.uniform(1.0, 4.0) * R, 0.0); return true;
            case 4: out = burr_cutter("ball", R, 0.0, 0.0); return true;
            case 5: out = burr_cutter("cone", R, g.uniform(0.3, 4.0) * R, 0.0); return true;
            case 6: out = burr_cutter("cylinder", R, 1.0, g.uniform(0.0, 1.0) * R); return true;
            default: {
                std::vector<ProfileSeg> S;
                double px = 0, py = 0, th = 0;
                const int n = g.integer(1, 4);
                for (int i = 0; i < n; ++i) {
                    const double maxth = (i == n - 1) ? kPi / 2 : 1.5;
                    const bool arc = g.integer(0, 1) == 1 && th + 0.05 < maxth;
                    if (arc) {
                        double th2 = g.uniform(th + 0.05, maxth), rho = g.uniform(0.3, 6.0);
                        double cx = px - rho * std::sin(th), cy = py + rho * std::cos(th);
                        px = cx + rho * std::sin(th2);
                        py = cy - rho * std::cos(th2);
                        S.push_back({true, px, py, cx, cy});
                        th = th2;
                    } else {
                        double th2 = g.uniform(th, std::min(maxth, 1.5)), l = g.uniform(0.2, 5.0);
                        px += l * std::cos(th2);
                        py += l * std::sin(th2);
                        S.push_back({false, px, py, 0.0, 0.0});
                        th = th2;
                    }
                }
                out = profile_cutter(S);
                return true;
            }
        }
    } catch (const std::invalid_argument&) {
        return false;
    }
}

// Run N random cases and return how many failed (and how many ran). `mutated` runs the library under test; the truth
// is always the same checks.
struct Tally {
    long cases = 0, failing = 0;
    long by_bit[3] = {0, 0, 0};
};

inline Tally run_profile_cases(unsigned long long seed, int N) {
    lcheck::Gen g(seed);
    Tally tl;
    for (int it = 0; it < N; ++it) {
        Cutter c{CutterKind::Flat, 1.0};
        if (!random_profile(g, it, c)) continue;
        Tri t = rand_tri(g, 12);
        double x = g.uniform(-14, 14), y = g.uniform(-14, 14);
        double z = drop_triangle(c, t, x, y);
        int m = check_profile(c, t, x, y, z);
        ++tl.cases;
        if (m) ++tl.failing;
        for (int b = 0; b < 3; ++b)
            if (m & (1 << b)) ++tl.by_bit[b];
    }
    return tl;
}

// ------------------------------------------------------------------ thin disc

inline double f_disc_ext(double R, double u) {
    u = std::fabs(u);
    return u <= R ? R - std::sqrt(std::max(0.0, R * R - u * u)) : R + kPenalty * (u - R);
}

inline void to_local(const Cutter& c, const P3& p, double x, double y, double& u, double& l) {
    double dx = p.x - x, dy = p.y - y, ct = std::cos(c.heading), st = std::sin(c.heading);
    u = dx * ct + dy * st;
    l = -dx * st + dy * ct;
}

inline double sampled_disc(const Cutter& c, const Tri& t, double x, double y, int n) {
    double best = kNeg;
    const double hw = 0.5 * c.thickness;
    for (int i = 0; i <= n; ++i)
        for (int j = 0; i + j <= n; ++j) {
            double a = double(i) / n, b = double(j) / n, w = 1 - a - b;
            P3 p{w * t.a.x + a * t.b.x + b * t.c.x, w * t.a.y + a * t.b.y + b * t.c.y,
                 w * t.a.z + a * t.b.z + b * t.c.z};
            double u, l;
            to_local(c, p, x, y, u, l);
            if (std::fabs(l) <= hw && std::fabs(u) <= c.radius) best = std::max(best, p.z - f_disc_ext(c.radius, u));
        }
    return best;
}

// Independent maximiser: outer golden search over l, inner over the section of the triangle by the plane l = const.
inline double reference_disc(const Cutter& c, const Tri& t, double x, double y) {
    const double hw = 0.5 * c.thickness;
    struct V { double u, l, z; };
    V v[3];
    const P3* ps[3] = {&t.a, &t.b, &t.c};
    for (int i = 0; i < 3; ++i) {
        to_local(c, *ps[i], x, y, v[i].u, v[i].l);
        v[i].z = ps[i]->z;
    }
    double lmin = std::min({v[0].l, v[1].l, v[2].l}), lmax = std::max({v[0].l, v[1].l, v[2].l});
    double lo = std::max(-hw, lmin), hi = std::min(hw, lmax);
    if (lo > hi) return kNeg;
    auto section_max = [&](double l) {
        bool have = false;
        V pa{}, pb{};
        for (int e = 0; e < 3; ++e) {
            const V& A = v[e];
            const V& B = v[(e + 1) % 3];
            if ((A.l - l) * (B.l - l) > 0 || A.l == B.l) continue;
            double s = (l - A.l) / (B.l - A.l);
            V q{A.u + s * (B.u - A.u), l, A.z + s * (B.z - A.z)};
            if (!have) { pa = pb = q; have = true; }
            else {
                if (q.u < pa.u) pa = q;
                if (q.u > pb.u) pb = q;
            }
        }
        if (!have) return kNeg;
        return golden_max(
            [&](double s) {
                double u = pa.u + s * (pb.u - pa.u), z = pa.z + s * (pb.z - pa.z);
                return z - f_disc_ext(c.radius, u);
            },
            0.0, 1.0, 60);
    };
    return golden_max([&](double l) {
        double r = section_max(l);
        return std::isfinite(r) ? r : -1e30;
    }, lo, hi, 60);
}

inline int check_disc(const Cutter& c, const Tri& t, double x, double y, double z) {
    int mask = 0;
    const double hw = 0.5 * c.thickness;
    double zs = sampled_disc(c, t, x, y, 80);
    const double zscale = 1.0 + std::fabs(z) + (std::isfinite(zs) ? std::fabs(zs) : 0.0);
    if (std::isfinite(zs) && !(z >= zs - 1e-9 * zscale)) mask |= 1;
    if (!std::isfinite(zs) && std::isfinite(z) && z > std::max({t.a.z, t.b.z, t.c.z}) + 1e-9) mask |= 1;
    P3 n;
    if (std::isfinite(z) && normal_of(t, n) && n.z > 1e-9) {
        // lower rim of the disc: (u, l, z + f(u)) for |l| <= hw; the triangle must not be above it where overhead
        const double eps = 1e-9 * (1.0 + std::fabs(z) + std::fabs(t.a.z));
        const double ct = std::cos(c.heading), st = std::sin(c.heading);
        for (int i = -24; i <= 24 && !(mask & 2); ++i) {
            double u = c.radius * double(i) / 24;
            for (int k = 0; k <= 6; ++k) {
                double l = hw * (double(k) / 3 - 1);
                double qx = x + u * ct - l * st, qy = y + u * st + l * ct, qz = z + c.radius - std::sqrt(std::max(0.0, c.radius * c.radius - u * u));
                if (inside_xy(t, qx, qy, 1e-12) && plane_at(t, n, qx, qy) > qz + eps) {
                    mask |= 2;
                    break;
                }
            }
        }
    }
    if (std::isfinite(z) && max_slope(t) <= 20.0) {
        double zr = reference_disc(c, t, x, y);
        if (std::isfinite(zr) && zr > -1e29 && !(std::fabs(z - zr) <= 1e-6 * (1.0 + std::fabs(z)))) mask |= 4;
    }
    return mask;
}

inline Cutter random_disc(lcheck::Gen& g) {
    double R = g.uniform(0.5, 15);
    return disc_cutter(R, g.uniform(0.05, 1.2) * R, g.uniform(-kPi, kPi));
}

inline Tally run_disc_cases(unsigned long long seed, int N) {
    lcheck::Gen g(seed);
    Tally tl;
    for (int it = 0; it < N; ++it) {
        Cutter c = random_disc(g);
        Tri t = rand_tri(g, 12);
        double x = g.uniform(-14, 14), y = g.uniform(-14, 14);
        double z = drop_triangle(c, t, x, y);
        int m = check_disc(c, t, x, y, z);
        ++tl.cases;
        if (m) ++tl.failing;
        for (int b = 0; b < 3; ++b)
            if (m & (1 << b)) ++tl.by_bit[b];
    }
    return tl;
}

}  // namespace props
