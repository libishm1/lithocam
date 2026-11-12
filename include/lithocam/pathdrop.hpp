// Copyright 2026 The lithocam authors. SPDX-License-Identifier: Apache-2.0
//
// Path drop-cutter (milling kernel M3): cutter-location (CL) points along lines over a mesh.
//
// adaptive_line samples the drop-cutter height along a straight xy segment and keeps subdividing an interval while
// the height at its midpoint differs from the straight-line interpolation of its ends by more than `tol` (and the
// interval is longer than `min_step`). It starts from a uniform pass at `max_step`. Straight runs between kept
// points then stay within about `tol` of the true CL surface, so a controller interpolating linearly between them
// does not gouge by more than that.
//
// zigzag lays parallel lines along x over a rectangle, `stepover` apart, alternating direction. For a ball cutter
// the stepover can come from the verified scallop bound: max_stepover(R, h) (lithocam/scallop.hpp), exact on a
// flat floor.
#pragma once

#include <vector>

#include "lithocam/dropcutter.hpp"

namespace lithocam {

struct CLPoint {
    double x, y, z;
    bool contact;   // false where no triangle lies under the cutter (z is then the `floor` value)
};

struct PathParams {
    double max_step = 1.0;    // uniform first pass, model units
    double min_step = 0.01;   // stop subdividing below this
    double tol = 0.005;       // allowed deviation of the straight run from the CL surface
    double floor = 0.0;       // z used where nothing is under the cutter
};

std::vector<CLPoint> adaptive_line(const Cutter& cutter, const TriMesh& mesh, double x0, double y0, double x1,
                                   double y1, const PathParams& p);

// Zig-zag raster over [xmin, xmax] x [ymin, ymax] with lines along x; returns one CL polyline per pass.
std::vector<std::vector<CLPoint>> zigzag(const Cutter& cutter, const TriMesh& mesh, double xmin, double xmax,
                                         double ymin, double ymax, double stepover, const PathParams& p);

}  // namespace lithocam
