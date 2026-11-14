# lithocam studio

The studio is a desktop program for stone CAM on industrial robots. It holds a cell (robot, spindle, turntable),
a part and its stock, a tool library and programs. It simulates the programs and writes controller code.
Robots: UR10e (URScript), KUKA KR120 R2500, KUKA KR 480 R3330 MT and KR 500 R2830 MT (KRL), and ABB IRB 6700-200/2.60
(key `irb6700`, RAPID; the turntable can be driven as external axis E1). The robot programs are for simulation
until a cell passes its hardware gate.

Start it with `python -m lithocam.studio [station.lcs.json]` (needs the `studio` extra: PySide6).

## Layout

- Top: the Station toolbar (files, Examples, part, views, teaching, Generate, Post) and the Machining toolbar
  (setup and one menu per operation family, collision checks, turntable, post dialect).
- Left: the work tree (station, robot, spindle, work frame, part, stock, targets, programs and their steps) and the
  Objects tabs (operation palette, tool library). Drag palette items onto a program to insert steps; drag steps to
  reorder them.
- Centre: the 3D cell view and the simulation bar.
- Right: the robot panel (joints, TCP pose, Cartesian jog, targets) and the properties of the selected step.
- Bottom: joint trajectory plot, log and posted program.

## Station page

Click the Station item in the work tree. The settings there change the posted program.

- **Work frame**, **robot feed factor**, **tool length override**, **rapid speed near the stone**.
- **Material.** The list is `lithocam.materials.available()`. A name without cutting data is marked. The tooltip is the
  material's `report()`: which quantities are sourced, snippet only or missing.
- **Table driven by.** `Operator` (the program stops while the operator turns the table), `E1` (the table is a
  coordinated external axis of a KUKA or ABB controller) or `PLC through I/O` (a request output and a done input of the
  cell I/O). `E1` is disabled for the UR10e, with the reason in its tooltip: a plain UR has no external axis. A mode the
  cell cannot run is shown under the fields, in red.
- **E1 travel, speed and largest step per ruling.** Fields marked `[placeholder]` hold the defaults of
  `program.TABLE_E1_DEFAULTS` until you set them. None of them comes from a real table. They are enabled in E1 mode.
- **Cell I/O** and **G-code machine.**

Changing the table mode makes the present simulation stale: the collision backend is built from the station on every
simulation, with a table joint when the table is driven as E1.

The wire step has three E1 parameters: `table_window_deg` (candidate table angles either side of the facing angle),
`table_step_deg` and `table_weight` (cost of a table step against an arm joint step of the same angle).

**E1 in the views.** The 3D view turns the part to the table angle of each simulation sample (`Segment.e1`), not to one
angle per segment, so the part turns while the arm moves. The stock simulation cuts at the same per-sample angle. The post
read-back checks E1 as well as the joints (`post.external_axes`) and reports `E1 read-back PASS` or `FAIL`.

## Steps

The palette lists every step with its status: done (the studio generates a path), partial or planned (kept in the
program, posted as a comment). Steps added with the hole, rest and finishing work:

| Step | Palette | What it does |
|---|---|---|
| `drill` | Holes | Peck drilling at the listed points or at the vertical holes found in the part. Peck, dwell, feed and speed are yours to set |
| `core_drill` | Holes | Stone core drilling: a clearing retract per peck (`peck` is required), optional entry orbit |
| `tap` | Holes | Rigid tapping. In at pitch x rpm, dwell, out with the spindle reversed. `pitch` and `rpm` are required |
| `bore` | Holes | An end mill bores a hole larger than itself on a helix |
| `rest` | 3-axis | Cuts only where the earlier steps left more than the threshold: from the last stock simulation, else a Z-map of the earlier milling steps, else `previous_tool_radius` |
| `engrave` | 2.5-axis | V-bit along polylines `x,y; x,y \| x,y; ...`, depth from the groove `width` or a `depth`; the tip follows the top surface |
| `chamfer` | 2.5-axis | V-bit or chamfer bit along the top outline of the part (or given `paths`) at a chamfer `width` |
| `flat_land` | 2.5-axis | Flat or bull end mill on the horizontal faces of the part, one height at a time |
| `pencil` | 3-axis | Traces the concave creases a ball-end mill touches at two or more points |
| `adaptive_raster` | 3-axis | Raster along x with the cross-feed set from the local curvature so the scallop stays below the limit |
| `polish` | Finishing | One pad pass per grit, a pad change before each, the pad axis along the surface normal |
| `disc_ruled` | Stone | Disc-saw cuts along the rulings of the part's ruled faces |

Details of the hole, rest and pencil methods are in `docs/OPERATIONS_HOLES_REST.md`.

**Holes and tapping.** `points` is `x,y[,z_top]; x,y; ...` in mm; empty means the vertical cylindrical holes found in
the part (`diameter_range` limits them). A dwell is a timed stop in the program (`sleep`, `WAIT SEC`, `WaitTime`,
`G4 P`), with no joint motion. A tap feeds at its own feed, pitch x rpm, not at the step feed. The way out needs the
spindle in reverse: a robot post (URScript, KRL, RAPID) refuses with a clear message unless the cell I/O file
defines `spindle.reverse` (`docs/CELL_IO.md`); G-code posts `M4` and `M3`. Posting a program with `tap` on a robot
without that output stays refused until the output is set.

**Rest.** The step takes its material from the first source that applies, and the log says which:

1. `previous_tool_radius` set on the step: "rest: previous tool radius r mm (no stock)".
2. A stock run from the previous simulation (the Stock box): the next Generate gives the step the stock just before it,
   replayed from the run's recorded cuts (`studio/livestock.py`): "rest: leftover from the stock simulation before step
   N". It is used only while the steps before the rest step and the station are unchanged since that run.
3. Otherwise a Z-map of the earlier vertical-axis milling steps of the program, cut from the station stock box at
   `leftover_cell` (default 1 mm): "rest: leftover from a Z-map of steps a..b at 1 mm".
4. With no earlier milling step, the tool of the previous one if any ("previous tool radius r (no stock)"); else the
   step reports what is missing.

A tilted setup skips 2 and 3 (they are height fields). The stock is compared at the tool centre, so the fillet a larger
round tool leaves in an inside corner would be missed; the previous tool's reach is added to sources 2 and 3 for that.
`expand(..., leftover=...)` takes an `SdfStock`, an `(xs, ys, Z)` height field, or a function of the step index. An empty
region (the same tool twice) is a message, not an error.

**Entry moves.** `face`, `rough_plane` and `flat_land` take `entry`: `plunge` (vertical, at the plunge feed), `ramp`
(descends at `ramp_angle` along the pass itself, bouncing along a short pass, then returns at depth to the pass start)
or `helix` (a circle of `helix_radius`, 0 = the tool radius, at most `helix_pitch` per turn, kept inside the stock
region and never below the part). The ramp angle (3 deg) and helix pitch (1 mm) are example values.

**Engrave and chamfer.** `engrave` follows polylines given as text, `x,y; x,y | x,y; ...` in mm. With `width` the depth
comes from the V angle (`width = tip flat + 2 depth tan(half angle)`); otherwise `depth` is used with any tool. The tip
rides the part's top surface (the height straight under it), so it follows a curved top. Lettering (`text`) is refused:
lithocam ships no single-stroke font, so give the strokes as paths. `chamfer` runs the cone along the top outline of the
part's highest flat top, or along `paths`; the chamfer `width` may not exceed the tool radius, and the tip sits on the
wall below the edge. A flat tip leaves a ledge of its radius.

**Flat lands.** `flat_land` groups the faces within `max_slope_deg` of +z by height and zig-zags at each, highest first.
The tool stays a radius off any wall that rises above the land and may overhang an open edge; it never gouges.

**Setup sheet.** The toolbar action Setup sheet (after Generate) writes a Markdown sheet: per cutting step the tool, kind,
feed, spindle speed, feed and rapid path lengths, an estimated time and, with the Stock box on, the removed volume. Feeds
and speeds are the ones posted; the time has no acceleration, tool changes or table turns. From code:
`Session.setup_sheet("Main", stock=False, path=None)`, or
`python -m lithocam.studio.api sheet station.lcs.json Main sheet.md`.

**Polish.** The grit sequence comes from the station material (`lithocam.materials`, sourced for granite only). A
material without a sourced sequence needs `grits` on the step; the step reports the module's message and refuses to
post otherwise. `pressure` (MPa) and `feed` (mm/min) are required: there is no sourced polishing pressure or feed.
`water` is required text. The pad axis follows the surface normal like 5-axis surfacing, so the planner chooses the
spin about the axis (`spins` candidates per point). Per grit the program gets a pad change (a forced tool change, also
for the same tool id), the pad speed (`S`) and the feed (`F`) of that grit, and comments with the grit, the pressure and
the force. The force is a simulation attribute; no force command is sent. `grit_tools` names a pad per grit.

**Disc saw on ruled faces.** `disc_ruled` reads the faces of `Station.ruled_path` (`python -m lithocam.ruled part.step
--json`), restricted by `labels` and `faces`. The log lists per face how many rulings can be cut, the depth, the twist
and the tilt, and why a ruling is left out. The cuts run through the same machinery as `disc_cut`: the blade centre is
the TCP, the spindle axis the blade normal, and the planner chooses the blade spin.

**Boundary.** `raster`, `waterline`, `steep_shallow` and `surfacing5` take `boundary`: `xmin,ymin,xmax,ymax` in mm, or a
polygon `x,y; x,y; x,y`. The tool tip (the ball centre for 5-axis) stays inside it, and passes end on the boundary.
Use it to stop relief finishing at the panel border instead of running down the slab sides. The temple relief
examples set it to the 600 x 600 mm panel.

## Materials, feed limits and tool wear

With a stock run (the Stock checkbox), each milling step in the work tree carries a badge: the
step's feed against its limit (`lithocam.materials.feed_limit`), for example `[ok, 40% of the power limit]` or `[over
the tool_deflection limit by 12%]`. The engagement comes from the removed volume of the step over its path length
(`materials.engagement_from_stock`), with the step-down or the stepover as the second dimension. The tooltip names the
binding limit, every candidate, and the checks that were skipped with the constant each needs. A limit that no
constant allows says `no limit` and lists what is missing.

**Process check** (after the stock run) adds the tool wear of the program: per step and per tool
(`materials.tool_life`), and the feed limit of each step in the report. The constants dialog has a field for each
constant, prefilled only where a source row exists, each labelled with its source:

| Field | Used for |
|---|---|
| `specific_energy`, `radial_ratio`, `axial_ratio` | milling force and the power and deflection limits |
| `tool_E`, `flute_factor` | cutter bending |
| `g_ratio`, `wear_per_area` | wear of milling and drilling tools (mm3 stone per mm3 segment) and of saws and wires (mm per m2) |
| `usable_volume_mm3`, `usable_height_mm` | the life fraction of a tool |
| `tool_deflection_limit_mm`, `robot_deflection_limit_mm` | the deflection limits of the feed limit |

No row exists for the wear constants, the usable amounts and the deflection limits today, so those fields stay empty
and the report says what each check needs. A blank field skips the check.

## Teaching positions

- Press **Move TCP** (key T). A handle appears at the tool tip. Drag an arrow to move along that axis, a ring to
  turn about that axis, the centre square to move in the screen plane. Hold Shift to snap to the jog step.
  The handle axes follow the jog frame (base, tool or work).
- The robot follows by inverse kinematics. A drag that would flip the arm to another configuration is refused, and
  the robot stays at the last reachable pose.
- **Save target** (Ctrl+T) stores the position under a name (T1, T2, ...). Targets are listed in the work tree.
  Double-click one to move the robot there. Its menu adds a joint or linear move to it, updates it from the current
  position, renames or deletes it. Steps that use a target follow a rename. Deleting a target keeps the motion:
  its values are copied into the steps that used it.

## Programs and safety gates

Generate expands the selected program into robot motion. Post writes the controller program. Posting refuses when:

- a cutting step has no feed (the step, the tool and the cutting data for the station material give none);
- the station or the program changed since the last Generate;
- the dialect belongs to another controller than the cell's robot (the API refuses this; the neutral CL dialect is
  always allowed);
- a robot program has a tap and the cell I/O defines no spindle reverse output;
- the table is driven as E1 on a cell without an external axis (the UR10e).

Every joint move between and inside steps is simulated at 2 degree steps and collision-checked when collision
checks are on.

Disc saw links are linear moves: up until the blade clears the stone by `clearance`, across while the blade axis
turns, down to the next lead-in. A disc step keeps one arm configuration from its approach to its last cut, as a
controller does on linear moves. The blade spin follows the turning axis, the long-range wrists start at the wind
nearest the middle of their range, and comb slots keep the spindle on one side. A point that would need another
configuration is reported as unreachable. A non-cutting point more than 5 mm under the stock bottom is an error.

**Saw heads.** A blade cuts a vertical kerf with its spindle axis level. With the spindle in line with the flange
the wrist then reaches down level with the stone, and on the UR10e it passes close to its wrist singularity. Stone
cells mount the spindle at a right angle to the flange instead (the side-mounted spindles of the Q-LAB cells credited
in the README). The presets `kr120_saw_head` and `ur10e_saw_head` (example sizes) are such heads: a column of
`head_drop` mm along the flange axis, then the spindle turned by `head_angle_deg` (90). A station selects one by
key: `"spindle": "kr120_saw_head"`. The head is drawn and collision-checked as its plate, column, body and nose.
With an angle head the blade spin sets where the arm sits, so the disc steps take `approach_deg` (default 45): the
arm comes in that far from vertical. Of the two sides (tilted forward or back along the cut) the planner takes the
one that keeps the wrist further from its singularity, and the collision planner adds a cost to wrist poses within
30 degrees of it. The log names the angle and the wrist margin. With an in-line spindle the parameter has no
effect and the log says so. On the example stations the head keeps the UR10e wrist at least 45 degrees from
singular (it reached 0.1 degree with the in-line spindle). The KR120 wrist stays at least 24 degrees from singular
with collision checks on and 15 degrees without (the faster planner orders the sides but adds no cost).

Milling posts every point the step tolerance needs. Passes start with a plunge at the plunge feed
from 2 mm above the stone and end with a straight retract. Linear cuts post as linear moves on every controller,
with the verified joints noted on each line for the read-back check.

## Programming from code

`lithocam.studio.api` builds, checks, simulates and posts programs with no window. Bad input raises `ApiError`
with a message that names the valid choices, so a script or a language model can correct itself.

```python
from lithocam.studio import api
s = api.Session("ur10e", material="granite")
s.load_part("dome.stl")                     # mm, z up; the stock box follows the part
s.set_work_frame([0.7, 0.0, 0.0])           # metres, robot base frame
s.add_target("approach", joints=[0, -90, 90, -90, -90, 0])
p = s.program("Main")
p.add("set_tool", tool="W7")
p.add("movej", target="approach")
p.add("raster", tool="W7", stepover=4.0, feed=600.0)
r = s.simulate("Main")                      # r.ok, r.errors, r.messages, r.unreachable
print(r.summary())
text = s.post("Main")                       # the cell's own dialect
s.save("station.lcs.json")                  # the studio opens the same file
```

Discovery: `api.steps()`, `api.step("raster")`, `api.robots()`, `api.dialects()`, `s.tools()`, `s.targets()`.
`api.station_schema()` returns the JSON Schema of the station file. `api.export_wire_cnc(...)` writes CNC wire saw
G-code and DXF for the station's wire faces (docs/WIRESAW_CNC.md).

`api.steps()` lists the new steps with their parameters. The check command prints the summary and the time of each
program:

```text
python -m lithocam.studio.api steps
python -m lithocam.studio.api schema
python -m lithocam.studio.api check station.lcs.json [PROGRAM]
python -m lithocam.studio.api post station.lcs.json PROGRAM DIALECT OUT
```

## Recording frames for documentation

`python -m lithocam.studio --record DIR N [--five-axis-demo] [--no-collisions] [--timeout S] [station.json]`
generates the program, hides the docks and saves N frames of the 3D view. `docs/media/frames_to_gif.py DIR OUT.gif`
turns them into a GIF (needs Pillow). Batch modes exit on failure and after the timeout; they never wait for input.
With `--stock` the camera centres on the stock and frames it by its size.

`python -m lithocam.studio --teach-record DIR` records the teaching sequence as whole-window frames: it saves a
target, drags the z arrow, the x arrow and the z ring through the same code as the mouse, saves a second target, adds a
joint and a linear move to the two and goes to each target. DIR also gets the log and the resulting station file.
