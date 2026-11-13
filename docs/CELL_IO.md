# Cell I/O and the G-code milling post

Date: 2026-11-13
Status: implemented, tested in Python against hand-built and studio-expanded programs. Not run on a controller, a
controller simulator or a machine. Every value in the shipped I/O files and machine files is a placeholder.

This page covers two additions to `lithocam.post`:

1. A cell I/O definition (`lithocam.cell_io`) that maps process records (spindle, water, air, wire motor, tool change)
   to controller statements.
2. A G-code dialect for milling on a 3-axis or 5-axis machine, or on a robot controller that reads G-code, driven by a
   machine definition (`post.MillMachine`).

## 1. Process records

The studio expansion (`studio.program.expand`) emits these records. The robot writers map them through the cell I/O.

| Record | Data | Emitted |
|---|---|---|
| `tool_change` | `id`, `mode` (`none`, `manual`, `changer`) | when the mounted tool changes, the first tool included; before the `tool` record |
| `spindle` | `rpm`, `on` | on before the first cut of a milling or disc step; a new record when the speed changes; off before a tool change, a table turn and the program end |
| `wire` | `on` | wire saw motor on before the first cut of a wire step; off as for the spindle |
| `water` | `on` | on with the spindle or wire motor; off with them |
| `air` | `on` | available to programs; the expansion does not emit it yet |
| `dwell` | `seconds` | available to programs; the spin-up dwell comes from the cell I/O at post time |
| `io_map` | `path`, `io` | right after `start` when the station names a cell I/O file |

The mode on a `tool_change` record is the cell's mode at expansion time (`manual` without a cell I/O).

## 2. The cell I/O file

One JSON file per cell, schema `lithocam.cell_io/1`. The loader validates it and raises `CellIOError` on a fault.
Full key list: the module docstring of `python/src/lithocam/cell_io.py`.

```json
{"schema": "lithocam.cell_io/1", "controller": "urscript", "placeholder": true,
 "ready_inputs": [{"di": 0, "value": true}],
 "fault_input": {"di": 1, "ok": false},
 "spindle": {"start": [{"do": 0, "value": true}], "stop": [{"do": 0, "value": false}],
             "speed": {"kind": "analog", "port": 0, "volts_per_rpm": null, "full_scale_volts": null},
             "ready": {"di": 2, "value": true}, "dwell_s": null, "max_rpm": null},
 "water": {"on": [{"do": 1, "value": true}], "off": [{"do": 1, "value": false}]},
 "tool_change": {"mode": "manual", "message": "Put tool {tool} in the spindle, then press Continue"}}
```

Actions and their statements:

| Action | URScript | KRL | RAPID | G-code |
|---|---|---|---|---|
| `{"do": p, "value": b}` | `set_standard_digital_out(p, b)` | `$OUT[p] = TRUE` | `SetDO DO_p, 1;` or a named signal | refused |
| `{"ao": p, "value": v}` | `set_standard_analog_out(p, v)` | `$ANOUT[p] = v` | `SetAO AO_p, v;` | refused |
| `{"wait_di": p, "value": b}` | `while not get_standard_digital_in(p): sync() end` | `WAIT FOR $IN[p] == TRUE` | `WaitDI DI_p, 1;` | refused |
| `{"register": r, "value": v}` | `write_output_integer_register(r, v)` | `name = v` | `SetGO name, v;` | refused |
| `{"call": "text"}` | the line | the line | the line with `;` | the line |
| `{"mcode": "M8"}` | refused | refused | refused | the line |
| `{"dwell": s}` | `sleep(s)` | `WAIT SEC s` | `WaitTime s;` | `G4 P s` |

Templates may use `{rpm}`, `{tool}` and `{pocket}`.

Speed outputs:

- `analog`: value = (offset_volts + rpm x volts_per_rpm) / full_scale_volts with `output: fraction` (URScript 0..1,
  KRL -1..1), or the volts with `output: volts` (a RAPID analog signal in volts). A value outside the range is refused.
- `register`: rpm x per_rpm to a fieldbus register.
- `call`: a function line such as `spindle_speed({rpm})`, for a spindle driven by a controller plug-in function.
- `word`: a G-code speed word such as `S{rpm}`.

A null scaling means "not set". The speed then posts as a comment and the loader warns.

Order of a spindle start: wait for the fault input to read its no-fault level, set the speed, run the start
actions, wait for the at-speed input, dwell. A speed change while running posts the speed only. At the program
start the writers wait for every `ready_inputs` entry. A tool change in `manual` mode posts an operator stop with the
message (URScript popup, KRL HALT, RAPID TPWrite and Stop, G-code M0). `changer` mode posts its actions with the
tool's pocket number. `none` posts a comment.

Safety net: a drive or switch still on at the `end` record is turned off there, with a warning comment.

### Shipped files

`cells/ur10e_io.json` (URScript), `cells/kr120_io.json`, `cells/kr480_r3330_mt_io.json`,
`cells/kr500_r2830_mt_io.json` (KRL). They are marked `"placeholder": true` and every section carries the note
"set for your cell". Port numbers are slots to fill. Speed scaling, dwell and the drive speed limit are null (not
set). Posting with them prints a PLACEHOLDER warning in the program and in the expansion messages.

### Selecting a file

`Station.cell_io`: `""` loads nothing, `"default"` loads `cells/<robot>_io.json`, any other value is a path
(relative to the station file when loaded with `Station.load`). A file that does not load is an expansion error, so
posting is refused. A file for another controller is ignored by the writer, with a note.

### No cell I/O

The writers post the process records as comments (`spindle 6000.0 rev/min: map to the cell's spindle I/O`) and a
tool change as an operator stop. The expansion adds the message
`WARNING: no cell I/O definition loaded (Station.cell_io) ...` once, at the first process record.

## 3. G-code milling post

```python
from lithocam import post
texts = post.write_gcode(records, machine="examples/gcode_machines/generic_5axis_ac.json",
                         work_frame=station.work_frame(), io=None, tool_length=None, warnings=[])
check = post.compare_gcode(records, texts, machine, work_frame=station.work_frame())
assert check.ok
```

`post.write(records, "gcode", machine=..., work_frame=...)` returns the files joined; `post.write_files` returns one
text per file. Positions are the cutter tip in the work frame, mm, from each record pose; the tool axis is minus the
pose z axis. A robot joint move without a pose becomes a safe approach: Z to `safe_z`, rotary axes, XY, then the
next point.

### Machine definition (`MillMachine`, JSON)

| Field | Meaning |
|---|---|
| `kinematics` | `3axis`, `table` (the rotary axes turn the part), `head` (they turn the spindle) |
| `axes` | letters for `x`, `y`, `z`, `primary`, `secondary` |
| `primary_axis`, `secondary_axis` | unit directions at zero rotation; positive turn by the right-hand rule. The secondary is carried by the primary |
| `table_pivots` | a point on each table axis, machine coordinates relative to the program zero |
| `head_pivots` | each head pivot from the spindle gauge point at zero rotation; the tool length adds along +Z |
| `limits` | `{letter: [lo, hi]}`; an absent rotary axis turns without limit |
| `tcp`, `tcp_on`, `length_comp`, `comp_cancel` | TCP mode posts tip and angles after `tcp_on` (default `G43.4 H{pocket}`); without TCP, `length_comp` (`G43 H{pocket}`) and machine coordinates |
| `feed_mode` | `G94` (mm/min) or `G93` (inverse time on every G1; `rotary_feed_deg_min` for rotary-only blocks) |
| `arcs`, `arc_tol`, `arc_min_radius`, `arc_max_radius` | fit G2/G3 (3-axis only) |
| `pole_deg` | half-angle of the pole cone |
| `vertical_tol_deg` | 3-axis: largest tilt accepted as vertical |
| `max_rotary_step_deg` | linearise cutting moves (without TCP) so no block turns more than this |
| `rewind_deg` | a cutting move that needs a larger rotary turn gets a retract, turn and re-entry |
| `safe_z`, `decimals`, `rotary_decimals`, `dwell_in_ms`, `line_numbers`, `header`, `footer` | format |
| `max_lines`, `split`, `program_number`, `sub_number` | program split |
| `io` | a cell I/O definition with controller `gcode` (M-codes) |

Examples: `examples/gcode_machines/generic_3axis.json`, `generic_5axis_ac.json`. Both are placeholders.

### 3-axis

The post accepts only records whose tool axis lies within `vertical_tol_deg` of +Z. Any other record raises
`GcodeError` naming the record, the tilt and the last comment before it.

### 5-axis kinematics

Both machine types reduce to R1(a1) R2(a2) p = q with p, q unit vectors (table: p = part tool axis, q = +Z; head:
p = +Z, q = part tool axis). The post solves it in closed form as a rotation about two axes (Paden-Kahan
subproblem 2). It gives zero, one or two solutions.

- Positions. Table: machine point = o1 + R1 [(o2 - o1) + R2 (P - o2)]. Head: X = tip - p1 - R1 [(p2 - p1) - R2 p2].
  At zero rotation the programmed point is the tip. The machine position uses the rounded angles, so the read-back
  error comes from the linear rounding only.
- Choice. Each solution is unwound to the angles nearest the previous point, inside the limits. The solution with
  the smallest largest rotary change wins. The first point starts from zero angles.
- Pole. When the tool axis lies within `pole_deg` of the axis whose angle becomes undefined (on the AC table, the
  C axis), the post holds that angle and solves the other for the nearest reachable tool axis. The deviation is at
  most `pole_deg`. Across a run of such points the held angle turns linearly, by tip path length, from its value
  before the run to the value the first point after the run needs. The axis does not jump at the exit of the cone.
- Unwinding. An axis without limits keeps turning (C passes 360 deg). A limited axis takes the in-travel angle
  nearest the previous one; if that needs more than `rewind_deg` in a cut, the post retracts to `safe_z`, turns and
  re-enters.
- Linearisation. Without TCP, a cutting move whose rotary change exceeds `max_rotary_step_deg` is split. The tip
  stays on the straight line and the angles are interpolated.

Measured on the studio's dome surfacing step (UR10e station, ball cutter, stepover 6 mm): near the apex the surface
normal passes 2 to 4 deg from vertical, and C steps up to 60.8 deg between two cutting points 1.9 mm apart with
`pole_deg` 0.5 (62 steps above 10 deg). With `pole_deg` 5 the largest step is 26.8 deg and the tool axis deviates up
to 3.8 deg. These are real azimuth changes, not solution flips. Linearisation bounds the step per block; the machine
still has to turn C that far.

### Program split

With `max_lines`, `split: subprograms` writes a main program that calls `M98 P<sub_number + k>` and the subprograms,
each ending in `M99`. `split: files` writes standalone files. Each continuation file restarts the tool length
compensation, the spindle and the water, then approaches the last position from `safe_z`. In `files` mode the split
prefers a point where the tool is out of the cut.

### Read-back gate

`gcode_points` parses the G-code and applies the forward kinematics (rotary angles and XYZ through the pivots, or
TCP) to recover the tip and tool axis after every motion block, arcs included. `compare_gcode` checks that every
pose record reappears in order, within `tol_mm` (default two units of the last decimal) and `tol_deg` (default
`pole_deg` + 0.001 deg on 5-axis). Approach, rewind and linearisation blocks between matches are allowed.
`post.joint_targets(text, "gcode")` returns nothing: G-code carries no robot joints.

## 4. What the studio still needs

- A dialect entry `gcode` in the post box and a machine-file selector for it; the post call passes
  `machine=` and `work_frame=station.work_frame()`; the read-back check uses `compare_gcode`, not `joint_targets`.
- A cell I/O file selector that sets `Station.cell_io` (`""`, `"default"` or a path) and shows the expansion's
  warnings.

## 5. Limits

- No program in this page ran on a controller, a controller simulator or a machine.
- The URScript port ranges checked (digital 0..7, analog 0..1) are the standard I/O only; tool and configurable I/O
  are not modelled.
- Input waits have no timeout.
- The G-code dialect is plain ISO words. Arc direction follows the right-hand rule about +Z (G17), +Y (G18) and
  +X (G19); check it against the control.
- The turntable yaw of a robot station is not carried into the G-code post: the work frame is the part frame.
