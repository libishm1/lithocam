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
