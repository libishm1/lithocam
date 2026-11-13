# Unified materials, feed limit and tool wear

Module: `lithocam.materials`. It joins `cutting_data.TABLE` and `materials_mechanics.TABLE` and adds an
engagement-based feed limit and a diamond wear model. It defines no physical constant. Every value comes from a
sourced row (shown with its source) or from the caller. A missing value makes a check say so and skip.

## M1. Material model

```python
from lithocam import materials as mt
mt.available()                    # names with at least one valued row: the studio's material selector
m = mt.get("oak")                 # aliases: white oak, quartz engineered, chipboard, ...
mt.value("granite", "specific_cutting_energy")                # (19.71, row), conservative end
mt.value("mdf", "chip_load", "low", tool="flat_end_mill", diameter_mm=10)
mt.report()                       # per material: sourced, snippet only, missing
```

`Material` holds `mechanical` (quantity to rows), `cutting` rows, `grit_rows` (polishing pad rows; the grit
steps are in the row notes), `water_rows` and the list of `sources`.

`value(material, quantity, which)` returns `(value, row)` or `None`. `which` is `low`, `high` or `conservative`.
Conservative is the high end for specific cutting energy, specific cutting force, force ratio and wear per area,
and the low end otherwise (low G-ratio means more wear). The row is a dict with `low`, `high`, `unit`, `source`,
`source_type`, `read`, `note`, `table`, `tool`, `size`. Rows in another unit than the quantity's preferred one are
left out, so units never mix (the Carrara density in g/cm3 is skipped; the kg/m3 rows are used). Cutting
quantities need `tool` (a kind name); sized rows use the nearest size, as `cutting_data.suggest` does.

Name matching for `materials_mechanics`: rows whose material equals the name first, else rows that contain it.
So `oak` uses every oak row and takes the low or high end across them. Rows that record a miss (`low` None) are
listed in the report and never returned by `value`.

Wear quantities `g_ratio` (mm3 stone per mm3 diamond segment, unit `mm3/mm3`) and `wear_per_area` (mm of
segment height per m2 of cut face, unit `mm/m2`) are read from `materials_mechanics` rows of those names when a
sourced row exists. None exist yet.

## M2. Feed limit

```python
fl = mt.feed_limit(tool, "granite", (ae, ap), rpm, spindle, robot=..., limits={...}, constants={...}, geometry={...})
fl.feed, fl.binding, fl.candidates, fl.skipped, fl.warnings
fl.badge(programmed_feed)        # "ok, 62% of the power limit" or "over the power limit by 20%"
```

Model, per step (mm, N, W, s):

* Cut area `A = ae * ap` mm2. Power `P = u A vf / 60` with `u` in J/mm3 and `vf` in mm/min.
* Power limit: `vf = 60 P / (u A)`. `P` is the spindle's `power_w` times `power_fraction` (rated power when not
  given, and the candidate note says so; no drive loss is applied).
* Tangential force `Ft = P / vc`, `vc = pi D rpm / 60000` m/s. Forces are linear in `vf`.
* Tool deflection: `F = Ft sqrt(1 + (Fr/Ft)^2)` through `stepped_cantilever_tip_deflection`. Limit is the
  allowed deflection divided by the deflection per mm/min. Needs `radial_ratio`, `tool_E` (GPa), `flute_factor`,
  the shank and flute lengths and diameter (`geometry`) and `limits['tool_deflection_mm']`.
* Robot deflection: `C = J Kq^-1 J^T` through `cartesian_compliance`. With `robot['force_dirs']` the force
  `Ft (t + rr r + ar a)` is applied; without, the largest eigenvalue of the linear block bounds any direction.
  `J` has linear rows in m/rad. `joint_stiffness` is given, or taken from the lowest `materials_mechanics` row for
  `robot['robot_name']` (one value for all joints; those rows are proxies, read the row note). Needs
  `radial_ratio`, `axial_ratio` and `limits['robot_deflection_mm']`.
* Chip load (wood): `vf = rpm * flutes * chip_load * depth_factor(ap / D)`. The chip load is the low end of the
  sourced row for the tool size, or `constants['chip_load']`. Without `ap` the depth factor is not applied and a
  warning says so.
* Sourced feed: the low end of a `feed` row for the tool kind and size, as a ceiling candidate. These are
  starting ranges, not hard limits.
* `u` comes from `constants['specific_energy']` (J/mm3), else a `specific_cutting_energy` row, else a
  `specific_cutting_force` row times 1e-3 (1 N/mm2 = 1e-3 J/mm3). A key given as `None` refuses the sourced
  row, as in `studio/process.resolve_constants`.

The limit is the smallest candidate; `binding` names it. A check without its constants is listed in `skipped`
with the missing item. With no candidate, `feed` is `None`.

Caveat on `u` for stone: the granite rows are wire-saw energies (motor power over cut volume, Khoshouei 2020),
which include wire friction and drive losses. Used for milling they are a conservative proxy, not a milling
measurement. Ratios `radial_ratio` and `axial_ratio` have no sourced row; give them from a test.

Limits: static, linear, quasi-steady. No chatter, thermal limit, tool wear, entry and exit transients, or
pose-dependent stiffness along the step.

### Engagement from the stock

```python
eng = mt.engagement_from_stock(step.removed, step.length, tool_diameter, ap_mm=5.0)   # or ae_mm=
engs = mt.engagements_from_steps(run.steps, {"tool name": diameter}, ap_mm=5.0)
```

`A = removed / length`. `A` fixes only the product, so give `ap_mm` (axial depth from the program) or `ae_mm`
(stepover); the other follows. With neither, only `area_mm2` is returned, which is enough for the power and force
checks. The removed volume is voxel-quantised (about one cell per face), so sum short steps first. `ae` above the
tool diameter is flagged.

## M3. Wear and tool life

```python
w = mt.wear(kind, material, removed_mm3=..., cut_area_mm2=..., constants={"g_ratio": ..., "usable_volume_mm3": ...})
life = mt.tool_life(run.steps, material, tool_kinds={name: kind}, constants={...})
life.per_step, life.per_tool, life.program_fraction, life.programs_per_tool_life, life.text()
```

* G-ratio model (grinding, milling, drilling): worn segment volume = removed volume / G. Life fraction =
  worn volume / `usable_volume_mm3`.
* Per-area model (saws and wires, preferred for `disc_saw`, `diamond_wire`, `multi_wire_saw`): segment height
  loss = cut area (m2) x `wear_per_area`. Life fraction = loss / `usable_height_mm`. For a wire step the cut
  area is contact length x path length (`step_cut_area_mm2`).
* All constants come from the caller or from `g_ratio` and `wear_per_area` rows. No sourced row exists yet, so
  without caller values the result is `None` and `skipped` names the missing item.
* `tool_life` adds per-step fractions per tool name. `programs_per_tool_life = 1 / worst per-tool fraction`.
  Constants may be given per tool name as a nested dict.

Limits: wear ratios are not constants. They change as the bond glazes or the stone varies, and depend on speed,
feed, water, grit and rock abrasivity. Use a ratio only for the conditions it was measured in. Tool breakage,
bond loss, loading and thermal damage are not modelled. The removed volume carries the stock-grid error.

## Studio needs

* Material selector: fill from `materials.available()`; show `materials.get(n).family`; tooltip from
  `materials.report(["granite"])`.
* Feed-limit badge per step: `engagement_from_stock` per `StepCut`, then `feed_limit`, show `badge(feed)` and
  `text()` in a tooltip; list `skipped` so the user sees which constants to supply.
* Wear per step: `tool_life(run.steps, material, tool_kinds, constants)`; add `life.text()` and the per-step
  fractions to the process report; add the `g_ratio`, `wear_per_area` and usable amounts to the constants panel.
