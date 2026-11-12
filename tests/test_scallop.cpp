// Copyright 2026 The lithocam authors. SPDX-License-Identifier: Apache-2.0
// Property tests mirroring lean/Lithocam/Scallop.lean. Tolerances are scale-aware (relative to the tool radius R).
#include <cmath>
#include <string>

#include "check.hpp"
#include "lithocam/scallop.hpp"

using lithocam::max_stepover;
using lithocam::scallop_height;

int main() {
    lcheck::Gen g(20261112);
    const long N = 200000;
    for (long i = 0; i < N; ++i) {
        const double R = g.uniform(0.1, 60.0);            // tool radius, mm
        const double tol = 1e-12 * R;
        const double h = g.uniform(0.0, R);
        const double smax = max_stepover(R, h);

        // scallop_le_of_stepover_le: s <= s_max(h)  =>  scallop(s) <= h
        const double s = g.uniform(0.0, 1.0) * smax;
        lcheck::expect(scallop_height(R, s) <= h + tol, "scallop_le_of_stepover_le",
                       "R=" + std::to_string(R) + " h=" + std::to_string(h) + " s=" + std::to_string(s));

        // scallop_maxStepover: scallop(s_max(h)) = h  (the bound is exact, not just safe)
        lcheck::expect(std::fabs(scallop_height(R, std::fmin(smax, 2.0 * R)) - h) <= 1e-9 * R, "scallop_maxStepover",
                       "R=" + std::to_string(R) + " h=" + std::to_string(h));

        // scallop_mono: 0 <= s1 <= s2 <= 2R  =>  scallop(s1) <= scallop(s2)
        const double s1 = g.uniform(0.0, 2.0 * R), s2 = g.uniform(s1, 2.0 * R);
        lcheck::expect(scallop_height(R, s1) <= scallop_height(R, s2) + tol, "scallop_mono",
                       "R=" + std::to_string(R) + " s1=" + std::to_string(s1) + " s2=" + std::to_string(s2));

        // scallop_eq_stable: the stable form equals the definition (compared in long double)
        const long double Rl = R, ql = static_cast<long double>(s1) * s1 / 4.0L;
        const long double ref = Rl - std::sqrt(Rl * Rl - ql);
        lcheck::expect(std::fabs(static_cast<long double>(scallop_height(R, s1)) - ref) <= 1e-13L * Rl + 1e-9L * ref,
                       "scallop_eq_stable", "R=" + std::to_string(R) + " s=" + std::to_string(s1));
    }
    // Small stepovers: the naive double formula R - sqrt(R^2 - s^2/4) loses all digits; the stable form must not.
    for (long i = 0; i < 1000; ++i) {
        const double R = g.uniform(1.0, 50.0), s = g.uniform(1e-9, 1e-6) * R;
        const double exact = s * s / (8.0 * R);            // leading term; the next term is O(s^4/R^3)
        lcheck::expect(std::fabs(scallop_height(R, s) - exact) <= 1e-6 * exact, "small_stepover_precision",
                       "R=" + std::to_string(R) + " s=" + std::to_string(s));
    }
    // Domain guards
    bool threw = false;
    try { scallop_height(1.0, 2.5); } catch (const std::exception&) { threw = true; }
    lcheck::expect(threw, "domain_guard", "s > 2R must throw");
    return lcheck::finish("scallop", N + 1001);
}
