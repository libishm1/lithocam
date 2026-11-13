// Copyright 2026 The lithocam authors. SPDX-License-Identifier: Apache-2.0
//
// Mutation negative control for the profile and disc drop-cutters. This executable compiles src/dropcutter.cpp with
// LITHOCAM_MUTATE, which lets `lithocam_mutation` switch on one deliberate defect at a time:
//   1  profile: facet contact removed            5  disc: face contact (chord in a face plane) removed
//   2  profile: edge contact removed             6  disc: rim stationary point removed
//   3  profile: arc support taken at the arc end 7  disc: second face ignored (slab one-sided)
// The property checks of tests/test_dropcutter_props.hpp must pass with no defect (0) and fail for every defect.
#include <cstdio>

#include "test_dropcutter_props.hpp"

int lithocam_mutation = 0;

int main() {
    const int N = 2500;
    int bad = 0;
    struct Row { int id; const char* what; bool disc; };
    const Row rows[] = {
        {0, "none (control)", false},       {0, "none (control)", true},
        {1, "profile facet contact removed", false}, {2, "profile edge contact removed", false},
        {3, "profile arc support at the arc end", false},
        {5, "disc face contact removed", true},     {6, "disc rim stationary point removed", true},
        {7, "disc second face ignored", true},
    };
    for (const Row& r : rows) {
        lithocam_mutation = r.id;
        props::Tally t = r.disc ? props::run_disc_cases(7, N) : props::run_profile_cases(7, N);
        std::printf("mutant %d %-40s %s: %ld of %ld cases fail\n", r.id, r.what, r.disc ? "disc   " : "profile", t.failing, t.cases);
        const bool control = r.id == 0;
        if (control ? t.failing != 0 : t.failing == 0) {
            std::fprintf(stderr, "FAIL mutant %d (%s): %s\n", r.id, r.what, control ? "the control must pass" : "the defect was not detected");
            ++bad;
        }
    }
    lithocam_mutation = 0;
    std::printf("dropcutter_mutation: %d of %zu rows wrong\n", bad, sizeof(rows) / sizeof(rows[0]));
    return bad == 0 ? 0 : 1;
}
