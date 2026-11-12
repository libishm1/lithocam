# Milling kernel

lithocam's milling kernel covers the 3-axis surface operations that general CAM libraries such as OpenCAMLib provide. It is written in C++20 from first principles and the published literature, under Apache-2.0. OpenCAMLib's code is not read or copied. OpenCAMLib (LGPL) is used only as a **test oracle**; see `tests/oracle/`.

| Module | State |
|---|---|
| M0 Cutters | flat and ball: **done**. Bull and cone: next |
| M1 Mesh index | uniform xy grid over triangle bounding boxes: **done** |
| M2 Drop-cutter | flat and ball: **done** |
| M3 Path drop-cutter, adaptive | next |
| M4 Push-cutter, waterline | planned |
| M5 Dexel stock simulation | planned |
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

The differences are at print precision.
