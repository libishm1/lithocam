# Polishing

`lithocam.polish` plans pad paths over a surface, predicts the removal with Preston's law and builds grit steps. It is numpy only. Force control is hardware-gated and dormant: pressure and force are simulation attributes on the moves, never a command.

No process constant is built in. The Preston coefficient, pad pressure, pad speed, feed, water and the grit values are parameters or come from `lithocam.cutting_data` rows (through `lithocam.materials` when present). The only polishing row today is the granite pad rpm (Slabwise, snippet confidence).

## Contact model

The pad is a flat disc of radius Rp with its axis on the surface normal (5-axis). The tool tip is the contact point. The pad is compliant: it presses with the uniform pressure p on every surface point within Rp (3D distance) of the contact point. A rigid flat pad on a curved surface touches only at its centre. Every number below is limited by this model.

## Paths

`polish(tris, mesh, tool, stock, **params)` follows the cam.py move contract. `surface_passes` and `passes_to_moves` are the parts.

- The patch must be single valued along `up` (default +z). Lanes are straight in the plane normal to `up` and are projected onto the triangles. Normals are area weighted vertex normals interpolated by barycentric weights.
- `pattern="zigzag"`: lanes at `angle_deg`, alternating direction. The pitch is reduced so the lanes divide the extent evenly.
- `pattern="spiral"`: Archimedean, outward from the plan centre, radial pitch = stepover.
- Stepover `s = 2 Rp (1 - overlap)`, or given directly. It is the plan pitch; on a slope the pitch on the surface is larger by 1/cos(slope). Lower the overlap ratio on steep patches and check `coverage`.
- `region`: a plan box (umin, umax, vmin, vmax) or a function of the contact points returning a mask.
- Moves: rapid over the start at the clearance plane, rapid to 2 mm above the start along the axis, plunge, feed, rapid up. Misses (holes in the patch) split a lane.

## Per-point attributes (4th element)

Plunge and feed moves are `(kind, point, axis, attrs)`. `attrs` holds `force_n` (pressure * pi * Rp^2), `pressure_mpa`, `grit`, `rpm`, `feed_mm_min`, `water`, each only when given.

How program.py and post.py should carry them:

- keep the 4th element when copying or transforming moves (only the point and axis change under a frame transform);
- post `S` from `rpm` and `F` from `feed_mm_min`, a tool change before each grit step, water as a comment or the coolant output;
- write `force_n` and `pressure_mpa` as comments or a side file for simulation. No force command is sent while hardware is dormant.

## Removal model

Preston: `dh/dt = k_p p v`, with k_p in mm^2/N, p in N/mm^2, v in mm/s. In the pad frame (x along the feed, y = axis x x) a surface point at (x, y) from the contact point sees

    v = sqrt( (f - omega y)^2 + (omega x)^2 )

with f the feed in mm/s and omega the pad rotation in rad/s. `predict_removal(moves, samples, ...)` sums `k_p p v dt` over the path (path resampled to Rp/50 by default, `dt = ds / f`). `dwell_map` is the same sum with `k_p p v` replaced by 1 (seconds under the pad). `rel_speed` replaces the speed model by a constant (orbital pads, scaling tests). `predict_by_grit` gives one map per grit step, with k_p per grit.

## Uniform removal (method)

For zig-zag lanes of pitch s with alternating direction, one pass leaves the profile `w(y)` across the footprint chord (`lane_profile`; mirrored on the reverse pass). The lane sum is periodic with period 2 s. The ripple is `(max - min) / mean` over one period (`ripple`). `choose_stepover(Rp, rpm, feed, tol)` returns the largest s <= 2 Rp whose ripple is at most `tol` over the band s (1 +- 5 %). The band matters: with a rotating pad v is about omega r, the profile has a sharp shape and the ripple changes quickly with s.

The feed scales the whole map. Mean depth `= k_p p v_mean A / (s f)` with `A = pi Rp^2` and `v_mean` the mean of v over the footprint. `feed_for_depth` solves it for a target depth. `plan_uniform(Rp, rpm, k_p, pressure, depth, tol)` iterates stepover and feed to a fixed point. The ripple does not depend on k_p or p.

## Grit sequences

`grit_sequence(material, grits=None, feed=..., water=..., rpm=None, k_p=None)`:

- `grits=None`: the ordered bands from the material's polishing_pad rpm row note (granite: `50-100`, `200-400`, `800-1500`, `3000`, `buff`, with pad rpm bands 1500-2000, 2000-2500, 2500-3000, 3000-3500, 3000-3500). Other materials have no sourced sequence and raise ValueError, so no sequence is invented.
- `grits` given: the order is kept. A numeric grit takes its rpm band from the band that contains it, else `rpm` must be given. `rpm_pick` places the rpm inside a band (0.5 is the midpoint, a choice and not a sourced value).
- Feed (mm/min) and water have no sourced polishing row; give them, as a scalar or a dict by grit label. A sourced row is used when one appears.

`polish_steps(tris, mesh, tool, stock, steps=..., pressure=..., tools=None, cross=False)` returns one dict per grit: `moves`, `tool` (a new pad per grit), `tool_change True`, `rpm`, `feed_mm_min`, `water`, `pressure_mpa`, `force_n`. With `cross=True` every second grit runs its lanes `cross_angle_deg` from the previous one.

## Test results (tests/test_polish.py)

| Check | Result |
|---|---|
| Axis = surface normal, sphere cap R100 (polar angle up to 45 deg, 0.62 mm mesh) | max 0.36 deg, median below 0.3 deg |
| Axis = surface normal, cylinder R100 | max 0.35 deg; axis never has a component along the cylinder axis |
| Coverage, overlap 0.6, Rp 20 (max distance to nearest contact point / Rp) | sphere zig-zag 0.41, cylinder 0.40, flat 0.38, sphere spiral 0.98 |
| Uniform removal, flat 120 x 100, Rp 20, 1500 rpm, 1200 mm/min, tol 0.05 | stepover 3.72 mm (overlap 0.91), predicted ripple 0.047, measured 0.056 (interior, 554 samples); stepover 2 Rp gives more than 3 times the ripple |
| Uniform removal, sphere R600 patch (slope below 6 deg) | measured ripple 0.033 (2057 samples) |
| Mean depth from `plan_uniform`, target 0.002 mm | map mean 0.00201 mm |
| Linear scaling | tripling k_p, p, rel_speed or dwell (feed / 3) triples the map; map = k_p p v dwell to 1e-9 |
| Dwell of one straight pass | chord / feed within 2 % |

The measured flat ripple is above the 1D prediction because of the finite lane count and path sampling. The sphere spiral covers with 0.98 Rp at overlap 0.6: lower the overlap ratio on steep spiral patches.

## What the studio needs

- A catalog step for polish that passes `pressure`, `rpm`, `feed`, `overlap` or `stepover`, `pattern`, `angle_deg`, `region` and the grit sequence to `polish_steps`, and a material selector that fills the grit rows.
- Pass-through of the 4th move element in program.py and post.py, and a tool change per grit step.
- Region selection on the part (a mask of triangles or a plan box) and a preview of the dwell or predicted removal map.
- A way to give k_p per material and grit. No sourced values exist yet; leave it empty and the maps stay unavailable rather than guessed.
- Visual validation by Libish: the coverage and removal maps on a real part before any pad path goes to a robot.
