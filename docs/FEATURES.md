# Stone feature input

`lithocam.features` turns a STEP B-rep into a list of stone operations. Recognition is done by [quiddity](https://github.com/pzfreo/quiddity) 0.4.9 (Apache-2.0, deterministic, Rust). lithocam adds the stone layer on top of it.

## Pipeline

1. **Canonicalise straight edges** (`lithocam.step_canon`, on by default).
   - The input is a degree-1 B-spline edge with two control points, in either the simple or the rational form.
   - It is rewritten as an exact `LINE` between the same two points.
   - The pass skips any curve whose parameterisation is used elsewhere (`PCURVE`, `SURFACE_CURVE`, `SEAM_CURVE`, `TRIMMED_CURVE`) and never touches faces.
2. **Recognise.** quiddity runs in-process when its wheel imports, which needs Python 3.11 or newer. Otherwise the `quiddity` executable is called: set `LITHOCAM_QUIDDITY` or put it on `PATH`. The subprocess gets an environment without `PYTHON*` variables, so a host such as Rhino 8 cannot break pip's launcher.
3. **Map to stone operations** with the rule table below. Records that no rule covers are kept as `unmapped`.
4. **Priority rule.** Descriptive records whose faces are already explained become `dropped` entries with a reason, not operations. These are plates, step levels, risers, section recesses, the bosses of a turned body, and the concave fillets that are the corners of a cutout.
5. **Patterns.**
   - quiddity's `hole_patterns` are kept.
   - Equal parallel grooves or slots at a constant pitch are grouped by lithocam (three or more members).
6. **Revision diff** (`diff(old, new)`).
   - Operations of the same kind are matched by nearest position in file coordinates, with no alignment step.
   - quiddity's `correspond` aligns the two parts first, so on a symmetric panel every feature comes back ambiguous. The file-frame diff resolves them.

## Rule table

| quiddity family | Condition | Stone kind | Route |
|---|---|---|---|
| `holes` | blind | `anchor_hole` | drill |
| `holes` | through | `through_hole` (position at mid-depth) | drill; `core_drill` at 20 mm and over |
| `pockets`, `prismatic_pockets` | width 12 mm or less, depth at least 2 x width | `kerf_slot` | saw_disc |
| `pockets`, `prismatic_pockets` | width 12 mm or less, shallower | `stopped_groove` | router (square ends) |
| `pockets`, `prismatic_pockets` | wider | `pocket` | router |
| `channels` | open downwards (z up) | `drip_groove` | saw_disc, or router when wide |
| `channels` | other | `through_groove` | saw_disc, or router when wide |
| `slots` | through the body | `cutout` (corner fillets absorbed; `min_tool_radius` = corner radius) | contour_cut |
| `slots` | blind | `pocket` | router |
| `through_steps` | | `rebate` | saw_disc (two passes) |
| `chamfers` | | `edge_chamfer` | profile_wheel |
| `fillets` | convex | `edge_round` | profile_wheel |
| `fillets` | concave, not a cutout corner | `inside_radius` | router |
| `turned_bodies` | | `turned_profile` (bosses absorbed) | lathe |
| any other | | the family name | unmapped |

The thresholds are in `Rules`: `narrow_max`, `kerf_depth_ratio`, `core_drill_min`, `pattern_tol`, `diff_match_max` and `diff_tol`. This is a starting rule table, not a process plan.

quiddity names the same recess `pockets` or `prismatic_pockets` depending on how the edges are encoded. The rules therefore read the shape, not the family name.

## Test parts

There are five dimension-stone parts. `tests/data/make_parts.py` builds them in build123d, and `tests/data/make_rhino_parts.py` models them natively in Rhino 8 and writes them with Rhino's STEP writer. The volumes agree: panel 7,184,717 mm3, sill 11,505,000 mm3, worktop 13,991,488 mm3, baluster 2,235,767 mm3.

| Part | build123d STEP | Rhino 8 STEP, raw | Rhino 8 STEP, canonicalised |
|---|---|---|---|
| Cladding panel (4 anchor holes, 2 kerf slots) | all found | all found, slots named `prismatic_pockets` | same as build123d |
| Window sill (drip groove, rebate, chamfer) | all found | chamfer only | same as build123d |
| Worktop (sink cutout R25, tap hole) | all found | round trip: 4 fillets, no cutout | same as build123d (cutter extruded per segment) |
| Stair tread (R10 nosing, 3 stopped grooves) | all found | all found | same as build123d |
| Baluster (turned) | `turned_profile` | 3 bosses + freeform faces | 3 bosses + freeform faces (known limit) |

"Same as build123d" means the same operations, positions and parameters. `tests/test_step_canon.py` checks this from recorded quiddity documents, and end to end when quiddity is installed.

## Known limits

- **Rhino writes cones and general revolved faces as B-spline surfaces.** It writes cylinders and planes analytically. quiddity reports the B-spline faces as `freeform_surfaces`, so a turned baluster modelled in Rhino is not recognised as a turned body.
- **One-surface extrusions.** A cutter extruded from a tangent-continuous curve as one surface becomes one B-spline wall. Extrude per segment, as `ExtrudeCrv` with `SplitAtTangents=Yes` does.
- **Mesh-derived or faceted STEP** is not expected to work. Freeform carving is out of scope for this module.
- **Floors of blind features.** quiddity leaves the floors of blind holes, kerf slots and grooves unexplained. The feature parameters are still correct.

## Upstream notes for quiddity

These come from the measurements above, with the STEP files in `python/tests/data/` as reproducers:

1. Treat degree-1, two-point B-spline edges (simple and rational) as lines. Rhino 8 writes no `LINE` entities. Without this, channels, through steps and slots are lost, and pockets change family.
2. Recognise B-spline surfaces that are exactly planes, cylinders or cones, within tolerance. Rhino writes cones and revolves this way, which hides turned bodies.
3. Account for the floor faces of blind holes, pockets and grooves.
4. Add a pattern record for parallel stopped grooves (quiddity has `section_recess_patterns` for some encodings only).
5. Give `correspond` an option to skip the alignment step, for symmetric parts.
