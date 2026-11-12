// Copyright 2026 The lithocam authors. SPDX-License-Identifier: Apache-2.0
//
// Z-map stock simulation (milling kernel M5, 3-axis): the stock is a height field on a regular xy grid; each
// cutter move lowers every cell to the cutter's swept lower envelope.
//
// Swept envelope of a linear move a -> b at a cell centre (x, y): min over t in [0, 1] of
//     tip_z(t) + f(dist((x, y), p(t)))     for dist <= R,
// where p(t) moves linearly and f is the cutter profile (dropcutter.hpp). tip_z is linear and f(dist(t)) is convex
// (f convex nondecreasing, dist convex in t), so the minimum is found by a golden-section search on the part of
// the move whose disc covers the cell.
//
// target_heights gives the highest mesh point straight above-or-below each cell centre (a vertical ray), so
// stock - target is the leftover material (positive) or a gouge (negative) at each cell.
#pragma once

#include <vector>

#include "lithocam/dropcutter.hpp"
#include "lithocam/pathdrop.hpp"

namespace lithocam {

struct ZMap {
    double x0 = 0, y0 = 0, cell = 1;   // centre of cell (0, 0) and spacing
    int nx = 0, ny = 0;
    std::vector<double> z;             // row-major, index j * nx + i

    double& at(int i, int j) { return z[static_cast<std::size_t>(j) * nx + i]; }
    double at(int i, int j) const { return z[static_cast<std::size_t>(j) * nx + i]; }
    double cx(int i) const { return x0 + i * cell; }
    double cy(int j) const { return y0 + j * cell; }
};

ZMap make_stock(double xmin, double xmax, double ymin, double ymax, double cell, double z_top);

// Lower the stock by one linear cutter move between two CL points (tips).
void cut_move(ZMap& stock, const Cutter& cutter, const CLPoint& a, const CLPoint& b);

// Lower the stock along every pass (consecutive points joined by linear moves; points without contact skipped).
void cut_passes(ZMap& stock, const Cutter& cutter, const std::vector<std::vector<CLPoint>>& passes);

// Highest mesh z on the vertical through each cell centre of `grid` (NaN where no triangle covers the cell).
ZMap target_heights(const TriMesh& mesh, const ZMap& grid);

struct ZStats {
    double min_dev, max_dev, mean_dev;  // stock - target over covered cells
    long cells;
};

ZStats deviation(const ZMap& stock, const ZMap& target, double xmin, double xmax, double ymin, double ymax);

}  // namespace lithocam
