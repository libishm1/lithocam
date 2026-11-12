# Verification matrix

| # | Statement | Lean | C++ property test | Status |
|---|---|---|---|---|
| 1 | Stepover within `2 sqrt(h(2R - h))` leaves a scallop of at most `h` | `scallop_le_of_stepover_le` | `scallop_le_of_stepover_le` | proved, tested |
| 2 | The bound is exact: the scallop at the bound equals `h` | `scallop_maxStepover` | `scallop_maxStepover` | proved, tested |
| 3 | The scallop is monotone in the stepover | `scallop_mono` | `scallop_mono` | proved, tested |
| 4 | The cancellation-free form used in C++ equals the definition | `scallop_eq_stable` | `scallop_eq_stable`, `small_stepover_precision` | proved, tested |
| 5 | Kerf on a straight edge removes no part material | `kerf_strip_outside_part` | `no_gouge` (convex parts) | proved (edge), tested (convex) |
| 6 | Kerf on a straight edge reaches the boundary: no extra waste | `kerf_touches_boundary` | `touches_boundary` | proved, tested |
| 7 | Corner arcs: circumscribed segments keep clearance >= kerf/2; inscribed chords gouge by the sagitta | planned | `no_gouge`, `bounded_overcut`, `negative_control_inscribed_gouges` | tested |

## Planned, in order

1. Ruled wire path: each wire segment lies on a ruling of the target ruled surface, within tolerance.
2. Kerf with taper, and non-convex parts.
3. Curvature-adaptive stepover on curved surfaces, generalising statement 1.
4. Feed limit: engagement at or below its limit, with a monotone force model, keeps force within its bound. Stone constants enter as hypotheses.
5. Time law: velocity, acceleration and jerk limits, checked against Ruckig and TOPP outputs.
6. External axis: the track or turntable composition preserves the TCP pose (6+1 forward kinematics).
7. No-gouge for drop-cutter contact, checked against OpenCAMLib.

Floating point is not the reals. Tests use tolerances relative to the part or tool scale and never exact equality.
