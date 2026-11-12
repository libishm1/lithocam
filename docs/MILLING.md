# Milling kernel

lithocam's milling kernel covers the 3-axis surface operations that general CAM libraries such as OpenCAMLib provide. It is written in C++20 from first principles and the published literature, under Apache-2.0. OpenCAMLib's code is not read or copied. OpenCAMLib (LGPL) is used only as a **test oracle**; see `tests/oracle/`.

| Module | State |
|---|---|
| M0 Cutters | flat, ball, bull (torus corner), cone: **done** |
| M1 Mesh index | uniform xy grid over triangle bounding boxes: **done** |
| M2 Drop-cutter | flat, ball, bull, cone: **done** |
| M3 Path drop-cutter, adaptive, zig-zag | **done** |
| M4 Push-cutter, waterline | planned |
| M5 Stock simulation | Z-map (3-axis): **done**; tri-dexel for wire cuts and 5-axis: planned |
| M6 Pocket clearing | planned |

## Drop-cutter (`include/lithocam/dropcutter.hpp`)

The cutter axis is +z. At horizontal distance d from the axis, the cutter's lower surface lies f(d) above the tip:
- ball: f = R - sqrt(R² - d²);
- flat: f = 0.

The drop-cutter height at (x, y) is the maximum of p.z - f(d(p)) over the mesh points under the cutter.

Per triangle the maximum lies at one of these:

| Cutter | Candidates |
|---|---|
| Ball | a vertex; an edge contact (the ball centre at distance R from the edge's line: a quadratic in the centre height, highest root, foot point inside the edge); a facet contact (the centre offset by R along the facet normal, contact point inside the triangle) |
| Flat | a vertex inside the disc; the higher end of an edge clipped to the disc; the disc's highest point on the facet plane (the disc edge in the up-slope direction) |
| Bull, cone | a vertex; an edge contact found by a golden-section search (every profile is convex and nondecreasing, so p.z - f(d) is concave along an edge); a facet contact at the cutter's support point in the direction of the downward facet normal |

## Adaptive path (`pathdrop.hpp`)

The path is sampled at `max_step`. An interval is subdivided while its midpoint or quarter points leave the chord by more than `tol`.

Where the cutter first meets a sharp edge, the CL height rises like a square root. A chord across even 0.02 mm then sags about 0.1 mm below the true path (3 mm ball), and the Z-map simulation showed that as a gouge along the part's edge. Deviation therefore keeps refining down to `min_step / 1000`; a change of contact alone stops at `min_step`.

## Stock simulation (`zmap.hpp`)

The stock is a height field. Each linear cutter move lowers every cell to the swept lower envelope: the minimum over the move of tip height + f(distance). The tip height is linear and f(distance) is convex, so a golden-section search finds the envelope.

`target_heights` casts a vertical ray per cell, so stock - target gives leftover (+) or gouge (-).

**End to end against the scallop theorem** (`tests/test_zmap.cpp`). A flat floor milled by a ball at `max_stepover(R, h)` leaves cusps that peak at h:

| h (mm) | Simulated peak (mm, grid 0.02) | Gouge |
|---|---|---|
| 0.020 | 0.01991 | none |
| 0.050 | 0.04988 | none |
| 0.200 | 0.19996 | none |

A single plunge reproduces the cutter profile to 1e-12 for all four cutter kinds. On wavy terrain, a drop-cutter zig-zag gouges at most 0.012 to 0.014 at a path tolerance of 0.01.

## Evidence

**Property tests** (`tests/test_dropcutter.cpp`, 8,007 cases):
- closed-form cases;
- no gouge: no sampled mesh point lies above the cutter surface;
- contact: the result matches dense sampling within the sampling error;
- the grid index equals brute force.

**Mutation check:**

| Mutant | Failures |
|---|---|
| ball edge contact removed | 364 |
| flat facet contact removed | 74 |

**Differential test against OpenCAMLib 2023.01.11.** Mesh: 3,240 triangles (terrain plus loose triangles). Queries: 3,000 points for each of ball and flat cutters at R 1.5 and 6.

| Cutter | R | max \|difference\| | nan mismatches |
|---|---|---|---|
| ball | 1.5 | 1e-12 | 0 |
| ball | 6 | 1e-12 | 0 |
| flat | 1.5 | 1e-12 | 0 |
| flat | 6 | 1e-12 | 0 |
| bull, corner 1 | 3 | 1.2e-9 | 0 |
| bull, corner 0.5 | 6 | 5.4e-10 | 0 |
| cone, half-angle 45° | 3 | **4.4** (608 of 3,000 points) | 0 |
| cone, half-angle 30° | 6 | **3.4** (673 of 3,000 points) | 0 |

Ball and flat agree at print precision. Bull agrees at the precision of the golden-section edge search.

**The cone does not agree.** At about 20% of the points, OpenCAMLib's `ConeCutter` gives a *lower* tip height than lithocam. An independent dense sampling of every triangle under the cone, written separately from both implementations, agrees with lithocam:

| Point | Sampled | lithocam | OpenCAMLib |
|---|---|---|---|
| 2131 | 4.9721 | 4.9721 | 1.5999 |
| 811 | 7.0013 | 7.0020 | 4.7634 |
| 1393 | 1.8347 | 1.8348 | — |

Passing OpenCAMLib the full included angle, or a quarter of it, does not remove the difference. At those points OpenCAMLib's cone result appears to gouge. Under the clean-room rule its code is not read, so the cause is not known. The case (`tests/oracle/make_case.py`, seed 7) is a candidate for an upstream issue report.
