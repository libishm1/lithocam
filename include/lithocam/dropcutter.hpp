// Copyright 2026 The lithocam authors. SPDX-License-Identifier: Apache-2.0
//
// Drop-cutter: the lowest position of a vertical-axis cutter above (x, y) that touches but does not cut into a
// triangle mesh. Written from first principles (lithocam milling kernel, module M2; see docs/MILLING.md):
//
//   The cutter's lower surface at horizontal distance d <= R from its axis lies f(d) above the tip:
//     ball (radius R):  f(d) = R - sqrt(R^2 - d^2)
//     flat (radius R):  f(d) = 0
//   A mesh point p within the cutter's disc forces the tip to z >= p.z - f(d(p)). The drop-cutter height is the
//   maximum of that bound over every mesh point in the disc. Per triangle the maximum lies at
//     ball:  a vertex, an edge contact (ball centre at distance R from the edge's line), or a facet contact
//            (ball centre offset by R along the facet's normal);
//     flat:  a vertex inside the disc, an end of an edge clipped to the disc, or the disc's highest point on the
//            facet plane (the disc edge in the plane's up-slope direction).
//
// Units are those of the mesh. The cutter axis is +z. Returns the tip height, or -infinity when no triangle lies
// under the cutter.
#pragma once

#include <array>
#include <cstddef>
#include <vector>

namespace lithocam {

struct P3 {
    double x, y, z;
};

struct Tri {
    P3 a, b, c;
};

enum class CutterKind { Flat, Ball };

struct Cutter {
    CutterKind kind;
    double radius;
};

// Tip height for one triangle (or -infinity when the triangle is outside the cutter's disc).
double drop_triangle(const Cutter& cutter, const Tri& tri, double x, double y);

// Triangle mesh with a uniform xy grid index over triangle bounding boxes.
class TriMesh {
public:
    explicit TriMesh(std::vector<Tri> tris, double cell = 0.0);   // cell <= 0 picks a size from the mesh
    const std::vector<Tri>& triangles() const { return tris_; }
    // Indices of triangles whose xy bounding box overlaps the square [x-r, x+r] x [y-r, y+r].
    void query(double x, double y, double r, std::vector<std::size_t>& out) const;

private:
    std::vector<Tri> tris_;
    double x0_ = 0, y0_ = 0, cell_ = 1;
    int nx_ = 1, ny_ = 1;
    std::vector<std::vector<std::size_t>> bins_;
    mutable std::vector<unsigned> stamp_;
    mutable unsigned tick_ = 0;
};

// Drop-cutter height at (x, y) over the whole mesh, using the grid index.
double drop_cutter(const Cutter& cutter, const TriMesh& mesh, double x, double y);

// Brute force over all triangles, for tests.
double drop_cutter_brute(const Cutter& cutter, const std::vector<Tri>& tris, double x, double y);

}  // namespace lithocam
