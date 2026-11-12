// Copyright 2026 The lithocam authors. SPDX-License-Identifier: Apache-2.0
#include "lithocam/scallop.hpp"

#include <cmath>
#include <stdexcept>

namespace lithocam {

double scallop_height(double R, double s) {
    if (!(R > 0.0) || !(s >= 0.0) || s > 2.0 * R) throw std::domain_error("scallop_height: need R > 0 and 0 <= s <= 2R");
    const double q = s * s / 4.0;
    return q / (R + std::sqrt(R * R - q));
}

double max_stepover(double R, double h) {
    if (!(R > 0.0) || !(h >= 0.0) || h > R) throw std::domain_error("max_stepover: need R > 0 and 0 <= h <= R");
    return 2.0 * std::sqrt(h * (2.0 * R - h));
}

}  // namespace lithocam
