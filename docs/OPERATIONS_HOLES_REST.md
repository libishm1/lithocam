# Hole cycles, rest machining, pencil and adaptive stepover

Module: `lithocam/studio/ops.py`. Tests: `python/tests/test_studio_ops.py`. Every generator has the `cam.py`
signature `f(tris, mesh, tool, stock, **params)` and returns moves `(kind, point mm[, axis[, ...]])`.
`ops.GENERATORS` lists them for registration in `cam.GENERATORS`.

## New move kinds

The 3rd element is the tool axis (always +z here). Later elements carry data.

| kind | tuple | meaning |
|---|---|---|
| `dwell` | `(kind, point, axis, seconds)` | stay at `point` for `seconds` |
| `tap_in` | `(kind, point, axis, feed, rpm)` | feed to `point` at `feed` mm/min, spindle forward at `rpm` |
| `tap_out` | `(kind, point, axis, feed, rpm)` | feed to `point` at `feed` mm/min, spindle REVERSED at `rpm` |

`feed = pitch x rpm` (`ops.tap_feed`). All other moves use the existing kinds. Pecks and bores use `rapid`,
`plunge` and `feed` only.

## Generators

All lengths mm. `points`: list of `(x, y)`, `(x, y, z_top)` or dicts (`x, y, z_top, depth, diameter`). `None`
detects vertical cylindrical holes in the part (`ops.detect_holes`: rim edges between an up face and a vertical
wall, circle fit within `tol`, hole not boss, bottom from the wall).

| generator | parameters |
|---|---|
| `drill` | `points, depth, peck=0, peck_mode="full"\|"chip", chip_retract, retract_height=2, dwell=0 s, clearance=20, diameter_range` |
| `core_drill` | as `drill` but `peck > 0` required, always a clearing retract to the R plane per peck; `orbit_radius=0, orbit_turns=1, tolerance` |
| `tap` | `points, depth, pitch (mm/rev), rpm, dwell, retract_height` (both required) |
| `bore` | `points, diameter (0 = detected), depth, pitch (descent per turn), tolerance (chord sag)` |
| `rest` | `previous_tool_radius \| previous_tool \| leftover, threshold=0.05, mode="raster"\|"waterline", stepover, stepdown, overlap, allowance, tolerance, direction, cell` |
| `pencil` | `min_angle_deg=15, cell, min_length, allowance, tolerance` (ball tool only) |
| `adaptive_raster` | `scallop` (required), `adaptive=True, stepover=0 (constant override), window, min_stepover, allowance, tolerance, cell` (ball tool only) |

Helpers with their own results: `rest_region(...) -> (field, mask)`, `pencil_paths(...)`,
`adaptive_raster_paths(...) -> (passes, info)`, `local_stepover(R, scallop, kappa)`.

Hole depth is below each hole's `z_top` (the detected rim, else `stock["zmax"]`). The tip reaches exactly
`z_top - depth`. Pecks are equal steps, none above `peck`. The R plane is `max(z_top, stock zmax) + retract_height`,
so a retract always clears the stock. `chip` mode retracts only `chip_retract` (required > 0).

## What program.py, post.py and catalog.py need

program.py
- Treat `dwell` as a timed stop: no distance, `seconds` added to cycle time, zero joint motion.
- Treat `tap_in` and `tap_out` as feed moves for kinematics and simulation (a straight z move at the given feed).
  The feed comes from the move, not from the operation's feed field. The stock simulation cuts them like feeds.
- `ops.GENERATORS` entries return `[]` when nothing needs cutting (`rest` with an empty region): the expander must
  accept an empty move list.
- Moves may carry 3 or 4+ elements. Code that unpacks `kind, p, *_` is fine. `cam.simplify` passes the new kinds
  through (it only merges runs of `feed`).

post.py
- G-code: `dwell` -> `G4 P<seconds>` (or `G4 S` per controller); `tap_in` -> spindle forward (`M3 S<rpm>`) then
  `G1 Z.. F<feed>`; `tap_out` -> `M4 S<rpm>` then `G1 Z.. F<feed>`, and `M3` again before the next non-tap
  move. Controllers with rigid tapping can fold a `tap_in`/`tap_out` pair into `G84 Z.. R.. F..`.
- Robot posts that cannot reverse the spindle must refuse `tap_out` with a clear error.
- The `dwell` point equals the previous point, so a post that tracks position can ignore it.

catalog.py (operation entries)
- `drill`, `core_drill`, `tap`, `bore`: tool kinds `drill`, `flat_bottom_drill`, `step_drill`, `core_drill` (tap: a
  tap tool, or any drill kind until a tap kind exists; bore: end mills). Parameters as the table above. Defaults
  for `peck`, `dwell`, `pitch`, `rpm` come from the cutting-data tables, not from here.
- `rest`: any tool the part has a drop-cutter model for; needs the previous operation's tool radius (from the
  previous program step) or the simulated stock as `leftover`.
- `pencil`, `adaptive_raster`: ball-end mill only.

## Method notes

- Rest: region where `h_large - h_small > threshold`, `h` the drop-cutter tip height on one grid. Equal tools give
  an empty region.
- Pencil: valley kinks of the offset surface `z = h + R` (slope angle rises by at least `min_angle_deg` across a
  sample), located to cell/16 by sampling across the crease. A vertical wall on a floor is a step of `h`; its low side
  is the valley. Vertical inside corners come out as climbing paths. Smooth surfaces give no path.
- Adaptive stepover: the exact two-circle scallop of a ball on a surface of normal curvature kappa (effective radius
  form by Vickers and Quan, 1989, for small steps). The passes are ball-centre curves, so the centre spacing is the
  contact spacing times (1 + R kappa) in the section, then projected to xy with the surface slope. At equal CENTRE
  spacing a concave surface scallops more than the flat formula says, a convex one less.
- Limit: within max(R, window) of the part's xy box edge the offset surface is affected by the rim, so the stepover
  there repeats the nearest interior value. Steep walls belong to waterline or steep_shallow (`min_stepover` floors the
  spacing).
