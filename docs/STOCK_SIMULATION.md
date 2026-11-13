# Stock-removal simulation: libraries and plan

## Today

| Backend | Use | Limits |
|---|---|---|
| Z-map, `zmap.hpp` (own, C++) | 3-axis milling. Exact swept envelope per move, watertight stock mesh. | One height per cell: no undercuts, no 5-axis. |
| Manifold, `meshsim.py` (Apache-2.0) | Wire cuts. Exact plane splits; ruled sweeps as hulls of tool positions. | One boolean per sweep. Fine for tens of sweeps; too slow for the 10^4 to 10^6 moves of a milling program. The hull of skew rulings over-cuts slightly (0.48 % measured on the twisted column). |

## Candidate C++ libraries

Activity and licence as GitHub reported them on 2026-11-12. Licences marked "to confirm" were not detected by GitHub; read the LICENSE file before adding a dependency.

| Library | Licence | Fit for stock removal |
|---|---|---|
| **OpenVDB** (ASF) | Apache-2.0 | Sparse narrow-band level sets. CSG difference of a tool-sweep SDF is fast and local to the moved region. `volumeToMesh` gives an adaptive stock mesh. Distance queries give gouge and leftover maps. Handles 5-axis moves, wire sweeps and undercuts. Accuracy is set by the voxel size. Python bindings exist. Main candidate for the general stock. |
| **Manifold** | Apache-2.0 | Exact, robust and fast mesh booleans. Keep it for exact operations (planar wire cuts, the final part comparison) and for the mesh export of an SDF stock when exactness matters. |
| **Embree** (Intel) | Apache-2.0 | Fast ray casting. Builds dexels and tri-dexels from meshes, and replaces the per-cell triangle query in `target_heights`. |
| **Geogram** | BSD-3-Clause (to confirm) | Exact-predicate mesh booleans, remeshing, Poisson reconstruction. Fallback for large booleans and for cleaning scan meshes (the Stanford bunny is not watertight). |
| **PMP library** | MIT | Mesh processing (decimation, remeshing, smoothing). No booleans. |
| **FCL** | BSD-3-Clause (to confirm) | Collision and distance between meshes. Tesseract already provides collision through Bullet and FCL. |
| CGAL | GPL-3 / LGPL-3 per package (to confirm) | Exact booleans (Nef, corefinement), but the mesh-processing packages are GPL. Not for the Apache core. |
| libigl | MPL-2.0 core with GPL parts (GitHub reports GPL-3.0) | Booleans through CGAL or its own arrangements; slower. Not needed. |
| mcut | LGPL-3 or commercial (to confirm) | Fast mesh cutting. The licence makes it a poor fit for the core. |
| cork | to confirm | Unmaintained since 2020. |
| CAMotics | GPL-2 (to confirm) | A complete open CNC simulator (octree stock, marching cubes). A good reference for behaviour; no code is copied. |
| MillSimSharp (.NET) | MIT | Voxel and SDF stock, dual-contouring export, 3- and 5-axis. A reference design and a second test oracle; its code may be read and adapted with attribution. |

## Recommendation

1. **General stock: OpenVDB.**
   - Each move becomes a tool-sweep SDF: a capsule for ball and flat moves, a convex hull of tool positions for 5-axis and wire moves.
   - That SDF is subtracted from the stock level set (`csgDifference`).
   - The stock mesh comes from `volumeToMesh`.
   - It is bound through rosetta like the rest of the C++ core.
   - Keep the Z-map as the fast exact 3-axis path.
2. **Cross-check every backend on the same jobs**, comparing removed volume, final volume and deviation maps:
   - Z-map (exact for 3-axis);
   - Manifold (exact for planar and analytic sweeps);
   - OpenVDB (resolution-limited);
   - MillSimSharp (voxel).
3. **Embree for ray casting** (dexels, target heights). It speeds up the comparisons without changing their results.
4. **Geogram for mesh repair** of scanned inputs before they enter Manifold.

## Next step

Build an OpenVDB stock prototype, `vdbstock.hpp`, behind a CMake option, and check it against the Z-map and Manifold results already in the tests:
- stair tread scallops;
- the cladding panel's planar wire cuts;
- the twisted column's ruled cuts.

On Windows, OpenVDB comes from vcpkg. In WSL it comes from apt (`libopenvdb-dev`), which needs sudo.

## Benchmark: sampled SDF stock (`sdfstock.py`)

### Method

`SdfStock` keeps the stock as a dense float32 grid of signed distance, negative inside the material. A tool cut is `sdf = max(sdf, -tool_sdf)`, evaluated only inside the tool's box grown by the band. The removed volume of each cut is the change of the smoothed occupancy `clamp(0.5 - sdf/cell, 0, 1)`, which is exact for planar faces and sub-voxel accurate on curved ones. The mesh comes from surface nets. A closed input mesh is scanned along x, y and z (tri-dexel style, ray parity) to get the initial field. numpy only; no per-voxel Python loops.

Lineage, as named works: signed and adaptive distance fields for machining simulation (Frisken et al., 2000); surface nets (Gibson, 1998); dexel and tri-dexel stock models (Van Hook, 1986).

### Why

- The Z-map holds one height per cell. It cannot represent undercuts, 3+2 or 5-axis cuts, wire cuts or disc cuts.
- A Manifold boolean per move is exact but its cost grows with the stock mesh, so it is the wrong tool for the 10^4 to 10^6 moves of a milling program.
- The sampled SDF costs one tool-sized array operation per move, whatever the tool pose, and the stock keeps a fixed memory size.

### Accuracy (measured, `python/tests/test_sdfstock.py`)

| Case | Result |
|---|---|
| Box 40 x 30 x 20, cell 1, aligned and unaligned | volume within 0.5 % |
| Ball-end slot (capsule r 4, length 24), cell 1 / 0.5 / 0.25 | error 0.47 % / 0.084 % / 0.019 % |
| Half capsule cut into the top face, cell 0.5 | 0.61 % |
| Flat-end slot vs Manifold hull boolean (256 segments), cell 0.5 | 0.04 % (3152.6 vs 3153.9) |
| Sphere and cube from a Manifold mesh, cell 0.5 | volume 0.085 % and 0.093 % |
| `mesh()` of both | V - E + F = 2, positive signed volume |
| Saw disc (R 15, kerf 2, cell 0.5) vs analytic | 0.11 % |
| Wire sweep (r 1, cell 0.5) vs analytic | 0.00 % |

The error falls as the cell halves in the ball-slot case.

### Cost (measured, one CPU thread, numpy)

| Run | Time |
|---|---|
| 20,000 short ball moves (r 3 mm), 121^3 grid, cell 1 | 5.4 s, 271 us per move |
| 5,000 short ball moves, 237^3 grid, cell 0.5 (53 MB) | 2.4 s |
| 5,000 flat-end moves (r 3, length 20), 121^3 grid | 8.2 s |
| `mesh()` of the 237^3 stock after the cuts (927,744 triangles) | 1.1 s |
| Manifold, 400 ball moves (hull of two spheres, subtracted) on a 116 mm cube | 1.0 s, stock grown to 24,910 triangles |

The Manifold figure is for a young stock. Its per-move time was about 1 ms over the first 100 moves, 4 ms over moves 100 to 200 and 2.5 ms over the last 200, as the mesh grew. I did not run it to 10^4 moves. The SDF time per move does not depend on the number of earlier moves.

### Limitations

- Only `cut_ball` is an exact sweep. Flat and bull-nose end mills, discs and moving wires are sampled at spacing <= cell/2 along the motion. Check the gap between samples against the tolerance you need.
- The field is dense. Memory is nx*ny*nz*4 bytes: 53 MB at 237^3, 0.5 GB at about 500^3. Pick the cell for the finishing tolerance, or cut a sub-block.
- The field is a narrow band (3 cells by default). The sign is exact everywhere. The distance is exact only inside the band, so distance queries (gouge, leftover) are valid near the surface only.
- `from_mesh` needs a closed mesh. Its distance is an upper bound of the true distance (minimum over three axes), corrected by the gradient norm near the surface. The zero set is exact on the scan lines.
- Sharp edges and corners round off at about one cell. Features thinner than two cells (a thin disc kerf) are resolved poorly; use cell <= kerf/4.
- A tool pose is a rigid sweep of one fixed shape. Tool deflection, runout and cutter-engagement forces are not modelled.
