// Copyright 2026 The lithocam authors. SPDX-License-Identifier: Apache-2.0
#pragma once

namespace lithocam {

/// Scallop (cusp) height left between two parallel passes of a ball-end tool on a flat surface.
/// h(s) = R - sqrt(R^2 - s^2/4), for 0 <= s <= 2R.
/// Computed in the cancellation-free form (s^2/4) / (R + sqrt(R^2 - s^2/4)).
/// Lean: Lithocam.scallop, Lithocam.scallop_eq_stable.
/// Throws std::domain_error outside the domain.
double scallop_height(double tool_radius, double stepover);

/// Largest stepover whose scallop does not exceed `scallop_limit`: s_max(h) = 2 sqrt(h (2R - h)), for 0 <= h <= R.
/// Lean: Lithocam.maxStepover, Lithocam.scallop_le_of_stepover_le, Lithocam.scallop_maxStepover.
/// Throws std::domain_error outside the domain.
double max_stepover(double tool_radius, double scallop_limit);

}  // namespace lithocam
