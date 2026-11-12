// Copyright 2026 The lithocam authors. SPDX-License-Identifier: Apache-2.0
// Minimal deterministic property-test harness: seeded generator, case counter, first-failure report. No dependencies.
#pragma once

#include <cstdio>
#include <cstdlib>
#include <random>
#include <string>

namespace lcheck {

struct Gen {
    std::mt19937_64 rng;
    explicit Gen(unsigned long long seed) : rng(seed) {}
    double uniform(double lo, double hi) { return std::uniform_real_distribution<double>(lo, hi)(rng); }
    int integer(int lo, int hi) { return std::uniform_int_distribution<int>(lo, hi)(rng); }
};

inline int failures = 0;

inline void expect(bool ok, const std::string& property, const std::string& detail) {
    if (ok) return;
    if (failures < 10) std::fprintf(stderr, "FAIL %s: %s\n", property.c_str(), detail.c_str());
    ++failures;
}

inline void expect(bool ok, const std::string& property) { expect(ok, property, ""); }

inline int finish(const char* suite, long cases) {
    std::printf("%s: %ld cases, %d failures\n", suite, cases, failures);
    return failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}

}  // namespace lcheck
