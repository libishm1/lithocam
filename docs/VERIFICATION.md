# Verification matrix

| # | Statement | Lean | Executable check | Status |
|---|---|---|---|---|
| 1 | Stepover within `2 sqrt(h(2R - h))` leaves a scallop of at most `h` | `scallop_le_of_stepover_le` | `scallop_le_of_stepover_le` | proved, tested |
| 2 | The bound is exact: the scallop at the bound equals `h` | `scallop_maxStepover` | `scallop_maxStepover` | proved, tested |
| 3 | The scallop is monotone in the stepover | `scallop_mono` | `scallop_mono` | proved, tested |
| 4 | The cancellation-free form used in C++ equals the definition | `scallop_eq_stable` | `scallop_eq_stable`, `small_stepover_precision` | proved, tested |
| 5 | Kerf on a straight edge removes no part material | `kerf_strip_outside_part` | `no_gouge` (convex parts) | proved (edge), tested (convex) |
| 6 | Kerf on a straight edge reaches the boundary: no extra waste | `kerf_touches_boundary` | `touches_boundary` | proved, tested |
| 7 | Corner arcs: circumscribed segments keep clearance >= kerf/2; inscribed chords gouge by the sagitta | planned | `no_gouge`, `bounded_overcut`, `negative_control_inscribed_gouges` | tested |
| 8 | A wire of radius r whose centre line runs parallel to a planar face, offset r along the outward normal, cuts no part material (every point within r of the line is outside the face plane) | `wire_tube_outside_part` | `ruled.py` access test (offset ruling); `robot.wire_target` places the wire at the offset (`test_wire_target_puts_the_wire_on_the_kerf_offset_line`) | proved, tested |
| 9 | That wire touches the face along the ruling (no extra waste) | `wire_tube_touches_face` | as above | proved, tested |
| 10 | Drop-cutter: no gouge and contact (flat, ball, bull, cone) | planned | `test_dropcutter.cpp` (8,007 cases, mutation check); OpenCAMLib oracle (flat, ball, bull agree; cone disagreement refereed by independent sampling, `docs/MILLING.md`) | tested |
| 10a | Profile drop-cutter (tapered ball and flat, bull taper, flat-tipped cone, burrs, random line and arc chains): no gouge, tight contact, agreement with the exact ball, flat, bull and cone | planned | `test_dropcutter_profile.cpp` (25,721 cases); `test_dropcutter_mutation.cpp` (3 profile mutants detected, control clean) | tested |
| 10b | Saw-slot disc drop-cutter (thin disc, horizontal axis): no gouge, tight contact, rotation invariance, thickness monotone | planned | `test_dropcutter_disc.cpp` (17,254 cases); `test_dropcutter_mutation.cpp` (3 disc mutants detected, control clean) | tested |
| 11 | Adaptive path: straight runs stay within tolerance of the CL surface | planned | `test_pathdrop.cpp` (worst 1.34 tol on smooth terrain) | tested |

All proved statements use only Lean's standard axioms (`propext`, `Classical.choice`, `Quot.sound`) and contain no `sorry`. CI rejects any `sorry`.

## Planned, in order

1. Ruled wire on curved ruled surfaces: the wire segment lies on a ruling of the target surface, within tolerance (statement 8 covers planar faces).
2. Kerf with taper, and non-convex parts.
3. Curvature-adaptive stepover on curved surfaces, generalising statement 1.
4. Feed limit: engagement at or below its limit, with a monotone force model, keeps force within its bound. Stone constants enter as hypotheses.
5. Time law: velocity, acceleration and jerk limits, checked against Ruckig and TOPP outputs.
6. External axis: the track or turntable composition preserves the TCP pose (6+1 forward kinematics).
7. Drop-cutter vertex, edge and facet cases (statement 10) in Lean.

Floating point is not the reals. Tests use tolerances relative to the part or tool scale and never exact equality.
