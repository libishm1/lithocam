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
#include <string>
#include <vector>

namespace lithocam {

struct P3 {
    double x, y, z;
};

struct Tri {
    P3 a, b, c;
};

enum class CutterKind { Flat, Ball, Bull, Cone, Profile, Disc };

// One piece of a general cutter profile, from the end of the previous piece (the first starts at the tip, (0, 0)) to
// (r, h): r is the radius from the axis, h the height above the tip.
//   line (arc = false): a straight segment, i.e. a cone frustum (a flat disc when h is unchanged);
//   arc (arc = true):   a circular arc of the circle with centre (cr, ch), i.e. a torus patch (a sphere zone when
//                       cr = 0). Only the lower-right quarter of the circle is allowed (r >= cr, h <= ch), where
//                       the surface rises with r.
struct ProfileSeg {
    bool arc = false;
    double r = 0.0, h = 0.0;
    double cr = 0.0, ch = 0.0;
};

// Lower-surface profiles f(d), 0 <= d <= radius:
//   Flat    0
//   Ball    R - sqrt(R^2 - d^2)
//   Bull    0 for d <= R - r;  r - sqrt(r^2 - (d - (R - r))^2) beyond (corner radius r = `corner`)
//   Cone    d / tan(half_angle)  (the tip is the cone's apex; `half_angle` in radians, 0 < half_angle < pi/2)
//   Profile the piecewise line and arc profile in `segs` (`radius` is the last segment's r)
// Every profile is convex and nondecreasing in d, so along a triangle edge the contact height is a concave
// function of the edge parameter (edge contacts for Bull, Cone and Profile use a golden-section search on it).
//
// Disc is not rotationally symmetric: a thin disc of radius `radius` and thickness `thickness` whose axis is
// horizontal and perpendicular to the cut direction (cos(heading), sin(heading)). The tip is the disc's lowest
// point, on the vertical plane through the disc's mid-plane. `profile` and the Z-map do not apply to a disc.
struct Cutter {
    CutterKind kind;
    double radius;
    double corner = 0.0;       // Bull: corner radius, 0 < corner <= radius
    double half_angle = 0.0;   // Cone: half of the included angle
    std::vector<ProfileSeg> segs;   // Profile: the pieces, from the tip outwards
    double thickness = 0.0;    // Disc: width of the disc along its axis
    double heading = 0.0;      // Disc: cut direction in the xy plane, radians from +x
};

// Empty when the profile cutter is consistent (see ProfileSeg: contiguous, r strictly increasing, nondecreasing
// height, convex, radius equal to the last r); otherwise the first problem found.
std::string validate_profile(const Cutter& cutter);
// Profile cutter from segments (sets `radius`); throws std::invalid_argument when validate_profile objects.
Cutter profile_cutter(const std::vector<ProfileSeg>& segs);
// Tapered cutter of outer radius `radius`: a flat of radius `flat_radius` at the tip (0 for none), then a corner arc
// of radius `corner` (0 for none) tangent to the flat, then a cone whose flank makes `half_angle` (radians, from the
// axis) reaching `radius`. flat_radius = 0 and corner > 0: tapered ball; corner = 0 and flat_radius > 0: tapered
// flat end (a flat-tipped cone); both > 0: bull taper; both 0: a pointed cone. Throws when the corner arc and the
// flank do not fit inside `radius`.
Cutter tapered_cutter(double radius, double flat_radius, double corner, double half_angle);
// Burr head of outer radius `radius` and length `length`: shape "flame" (tangent ogive, needs length >= radius),
// "ball" (hemisphere; length is ignored), "cone" (pointed cone reaching `radius` at height `length`), or "cylinder"
// (flat end with corner radius `corner`, 0 for a sharp edge). Throws std::invalid_argument for another shape or
// dimensions that do not fit.
Cutter burr_cutter(const std::string& shape, double radius, double length, double corner);
// Thin disc for saw slotting. `heading` is the cut direction (use atan2(dy, dx) of the pass). Throws
// std::invalid_argument unless radius and thickness are positive.
Cutter disc_cutter(double radius, double thickness, double heading);

// Height of the cutter's lower surface above its tip at horizontal distance d from the axis (d <= radius).
// Throws std::invalid_argument for a Disc.
double profile(const Cutter& cutter, double d);

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
