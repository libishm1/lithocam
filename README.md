# lithocam

An open, verified CAM layer for stone. lithocam generates the parts of stone toolpaths that general CAM libraries don't:
- wire and disc-saw paths with kerf compensation;
- stone stepover, feed and wear rules;
- engagement maps.

Its output is 5-axis cutter-location data with a tool-spin tolerance and per-point process attributes. Robot motion planners consume that output, for example Tesseract, including arms on a linear track or turntable.

Status: **0.0.1, early.** Main pieces so far:
- verified stepover and kerf, and a Lean-proved wire against a planar face;
- a milling kernel (drop-cutter for four cutter shapes, adaptive zig-zag, Z-map stock simulation), checked against OpenCAMLib;
- ruled-surface extraction for wire saw and flank milling, and a diamond-wire cutting simulation;
- UR10e and KUKA KR120 cells in Tesseract, with a track and turntable;
- Python bindings generated with C++26 reflection.

| | |
|---|---|
| ![ruled labels](docs/media/ruled_labels.gif) | ![wire cuts](docs/media/wire_cuts.gif) |
| Faces labelled wire, flank or point, with their rulings (`lithocam.ruled`) | Squaring a blank with diamond-wire cuts: kerf, waste and what is left for milling (`lithocam.wiresim`) |
| ![milling simulation](docs/media/milling_sim.gif) | ![robot wire sweep](docs/media/robot_wire_sweep.gif) |
| Finishing in the Z-map simulation at a scallop-verified stepover | UR10e wire sweep along a verified joint path (`lithocam.robot`) |

## Contract: every algorithm has a proof and a test

| Module | C++ | Lean 4 (Mathlib) | Property test |
|---|---|---|---|
| Scallop height and stepover bound | `scallop.hpp` | `Lithocam/Scallop.lean`: `scallop_le_of_stepover_le`, `scallop_maxStepover`, `scallop_mono`, `scallop_eq_stable` | `tests/test_scallop.cpp` |
| Kerf offset of a wire or blade | `kerf.hpp` | `Lithocam/Kerf.lean`: `kerf_strip_outside_part`, `kerf_touches_boundary` | `tests/test_kerf.cpp` (convex parts, circumscribed corners, inscribed-chord negative control) |

**How the proofs relate to the code:**
- The Lean statements are over the real numbers. The C++ is checked against the same statements by seeded property tests, with tolerances relative to part size.
- A statement is either proved or listed as a cited axiom. `sorry` is never allowed, and CI checks for it.
- See `docs/VERIFICATION.md` for the theorem-to-test matrix and the planned modules.

## Stone feature input (`python/`)

`lithocam.features` reads a part as a STEP B-rep and lists the stone operations it needs. Recognition is done by [quiddity](https://github.com/pzfreo/quiddity) (Apache-2.0). lithocam adds the stone layer on top:
- **Stone names and routes.** For example, quiddity's `pockets` record becomes a `kerf_slot` cut by saw disc, `channels` becomes a `drip_groove`, and a through `slot` becomes a `cutout` by contour cut, with the corner radius as the tool radius limit.
- **A priority rule.** Plates, risers, section recesses, the corner fillets of a cutout and the bosses of a turned body do not become extra operations.
- **Pattern grouping.** Equal parallel grooves at a constant pitch are grouped into one set.
- **A revision diff in file coordinates.** It handles symmetric panels, which quiddity's own `correspond` reports as ambiguous.
- **A straight-edge canonicalising pass for Rhino STEP** (`lithocam.step_canon`). Rhino 8 writes every straight edge as a degree-1 B-spline, and without the pass quiddity misses grooves, rebates and cutouts.

```sh
pip install -e "python[features,test]"      # quiddity needs Python 3.11+
python -m lithocam.features part.step --diff part_revB.step
```

### Ruled surfaces: wire, flank or point (`lithocam.ruled`)

`lithocam.ruled` finds, for each face, whether it is swept by straight lines (a ruled surface) and along which direction. Those lines are where a diamond wire or the side of a flank-milling tool can lie. It then checks access along each line with exact line-solid intersection and labels the face:
- `wire`: the wire passes through waste only;
- `flank`: the tool can enter from one side;
- `point`: point milling.

Inside corners keep the wire or tool radius, as on a real machine. The extractor needs OpenCascade (`pip install -e "python[ruled]"`).

```sh
python -m lithocam.ruled part.step --wire-diameter 7 --tool-radius 5 --json faces.json
```

Rhino 8: run `rhino/lithocam_features.py` with RunPythonScript. It marks each operation with a dot coloured by route. Rhino's Python 3.9 calls the `quiddity` executable through `LITHOCAM_QUIDDITY`. See `docs/FEATURES.md` for the rule table, the Rhino findings and the known limits.

## Build

```sh
cmake -S . -B build && cmake --build build --config Release && ctest --test-dir build -C Release
cd lean && lake exe cache get && lake build
cd python && pytest
```

## Bindings

The C++ core is written once. Python bindings are generated with [rosetta](https://github.com/rosetta-bindings/rosetta), using C++26 reflection on a clang-p2996 host. The generated sources are then compiled with ordinary compilers, including MSVC for Blender's Python. This is in progress.

## Licence

Apache-2.0. GPL code (for example BlenderCAM or animaquina) is never copied into lithocam. Those projects can use lithocam.
