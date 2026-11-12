// Copyright 2026 The lithocam authors. SPDX-License-Identifier: Apache-2.0
#pragma once

#include <vector>

namespace lithocam {

struct Vec2 {
    double x = 0.0, y = 0.0;
};

/// Signed distance from p to a convex polygon given counter-clockwise: positive outside, negative inside.
double signed_distance_convex(const std::vector<Vec2>& poly_ccw, Vec2 p);

/// Wire- or blade-centre path around a convex part (counter-clockwise vertices). The path is offset outward by kerf/2,
/// so the cut (all points within kerf/2 of the path) stays on the waste side.
///
/// Edges: straight offsets. Lean: Lithocam.kerf_strip_outside_part, Lithocam.kerf_touches_boundary.
/// Corners: the arc of radius kerf/2 is replaced by its CIRCUMSCRIBED polygon. Every path segment is tangent to the arc,
/// so no point of the path comes closer than kerf/2 to the part. An inscribed polygon (chords) would gouge the corner
/// by the sagitta. The step angle is chosen so that the extra waste at a corner is at most `max_overcut`.
///
/// Returns a closed path: the last point equals the first.
/// Throws std::domain_error for fewer than 3 vertices, kerf <= 0, or max_overcut <= 0.
std::vector<Vec2> kerf_path_convex(const std::vector<Vec2>& part_ccw, double kerf, double max_overcut);

}  // namespace lithocam
