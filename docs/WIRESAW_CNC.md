# CNC diamond wire saw export

`lithocam.wiresaw_cnc` writes ISO G-code and DXF for CNC diamond wire saws. One exporter serves three axis layouts. A machine definition (JSON) holds everything that differs between machines. The definitions shipped here are placeholders. No machine manual was used. Every value that matters must be set from the real machine before a program runs on it.

Demo: `python examples/wiresaw_cnc_demo.py OUTDIR`. Tests: `python/tests/test_wiresaw_cnc.py`. The module needs numpy only.

## Layouts

| Layout | Axes (logical) | Cuts |
|---|---|---|
| `xy` | trolley, height | plane profile in the wire plane |
| `xyc` | trolley, height, table | indexed profiles (table fixed per setup) and coordinated turning (table moves with the cut) |
| `xyuv` | trolley, height, height2, optional trolley2 | ruled surfaces: two wire-end carriages, the wire is the straight line between them |

Frame. The wire is a straight horizontal line. The trolley moves it across the stone, the height axis lifts it, and the wire plane is the trolley-height plane. Profiles are given directly in machine coordinates of that plane (x = trolley, y = height). Part-to-machine placement is the caller's job, except for ruled faces, which take a frame (below).

### xy

`Setup2D(profile, waste="outside")`. The profile is a `Profile` of lines and arcs, open or closed. Build it with `Profile.from_points`, `Profile.from_bulge` (DXF bulge convention) or from `Seg` objects. `waste` is `outside` or `inside` for a closed profile (the part is on the other side) or `left` or `right` of the direction of travel.

### xyc, indexed

`Setup2D(profile, table_deg=a)` per setup. A 4-sided column is the same side profile at 0, 90, 180 and 270. Between setups the wire stops, the table turns, and the wire moves to the next lead-in.

### xyc, coordinated

`CoordSetup(profile_sz, deg_per_mm, waste="+s")` for turned shapes such as balusters.

The wire plane contains the table axis. The axis stands vertical at trolley position `table_axis_trolley` (x0). The profile is given as (s, z): radius from the axis and height. The wire is a horizontal line at distance s from the axis, so over a full turn it removes everything beyond radius s. With s following the profile the envelope is the surface of revolution r(z) = s(z). The motion is X = x0 + s, Z = z, C = theta, interpolated linearly in each block. The table angle follows the path length, theta = theta0 + deg_per_mm times the length from the path start, and no block turns the table more than `max_step_deg`. The wire centre follows the profile offset by the wire radius to the waste side (larger s for outside turning). A large angle step makes the envelope a polygon. Wire tilt, wire bow and wire sag are not modelled.

### xyuv

`ruled_face_setup(face, machine, frame, ...)` turns one face of the ruled-face JSON (`lithocam.wirepath.load_ruled_json`) into a `RuledSetup`, rows of (x0, h0, x1, h1).

- `frame` names the part axes that become the machine trolley, depth (along the wire) and height directions, for example `("z", "y", "-x")`. It must be a rotation, so a mirrored part cannot be cut by mistake.
- Each ruling is moved by the wire radius along its outward normal. Then it is cut by the two carriage planes (fixed depth positions, by default the mid depth of the face plus or minus half `carriage_spacing`). The two intersections are the wire-end positions.
- A ruling must run within `max_depth_angle_deg` (default 60) of the depth axis, so the wire can span it.
- If the machine has no `trolley2` axis, the wire must stand in a plane of constant trolley position. The helper and the planner refuse otherwise.
- Rulings are sorted along the trolley. The planner refuses a sequence that is not monotone in trolley position (no going back, no repeated position), and checks every end position against the travel limits and the wire tilt against `max_wire_tilt_deg`.
- Move lines carry all four words: `G1 X.. Y.. U.. V.. F..`.

The twisted column in the demo has rulings that rotate in plan with height. Its faces need a wire tilt of about 28 degrees at the top. The placeholder limit is therefore 30. A machine limited to 20 degrees refuses this part, with a message.

## Machine definition fields

| Field | Meaning |
|---|---|
| `name`, `notes` | free text |
| `layout` | `xy`, `xyc`, `xyuv` |
| `axes` | logical axis to letter. Logical names: `trolley`, `height`, `table`, `height2`, optional `trolley2`. Any capital letter except the program words G M F N P D I J K O. Letters must be unique |
| `limits` | logical axis to [lo, hi]. Absent means unchecked. Checked on every planned position |
| `table_unwind` | `continuous` (the written angle is unbounded, limits apply, the next setup uses the equivalent angle nearest the current one), `wrap360` (written angle in 0..360, a coordinated cut that crosses a multiple of 360 is refused), `rewind` (as continuous, then a rapid back into 0..360 at the end) |
| `table_axis_trolley` | trolley position of the table axis, for coordinated turning |
| `feed_mode` | `G94` (mm/min) or `G93` (inverse time, 1/min) |
| `rotary_feed_deg_min` | table speed. Needed for G93 with a moving table, and used in the time estimate |
| `arc_plane` | `G17`, `G18`, `G19`, or null to flatten arcs to lines |
| `arc_first`, `arc_second` | which logical axis is the first and second axis of the arc plane. Counter-clockwise in (trolley, height) is G3 when trolley is first, else G2 |
| `arc_words` | centre-offset word per logical axis, I, J or K |
| `chord_tol` | chord tolerance when arcs are flattened |
| `kerf_mode` | `cam` or `g41` |
| `d_register` | number after D for G41 and G42 |
| `wire_diameter` | the offset is half of it |
| `offset_join`, `mitre_limit` | `round` or `mitre` joins at outside corners. A mitre longer than the limit (in offsets) falls back to a round join |
| `m_wire_start`, `m_wire_stop`, `m_coolant_on`, `m_coolant_off`, `m_program_end` | M numbers. Coolant and end may be null |
| `header`, `footer` | lines copied into every file |
| `line_numbers` | step of N words, 0 for none |
| `max_lines` | split into files of at most this many lines |
| `decimals` | decimal places of every coordinate |
| `corner_dwell_s`, `dwell_in_ms` | `G4 P` at profile corners, in seconds or milliseconds |
| `corner_angle_deg`, `corner_slowdown_factor`, `corner_slowdown_length` | a turn above the angle is a corner. Feed is multiplied by the factor within the length either side of a corner |
| `lead_in`, `lead_out` | lead lengths in mm. A setup can override them |
| `carriage_spacing`, `max_wire_tilt_deg` | xyuv wire geometry |

Load and save with `MachineDef.load(path)` and `MachineDef.save(path)`. Unknown fields are refused.

## Kerf

`cam`. The exporter offsets the profile by the wire radius to the waste side and writes the wire-centre path. Lines offset by their normal, arcs change radius. Outside corners get a round join (an arc about the corner) or a mitre. Inside corners are trimmed at the intersection of the offset segments. The exporter refuses when a segment vanishes, when the offset path crosses itself, and when any point of it comes closer to the profile than the wire radius. The last two cover slots and notches narrower than the wire diameter.

`g41`. The nominal profile is written. The lead-in block carries `G41 D<n>` when the waste is on the left of travel and `G42 D<n>` when it is on the right. The lead-out block carries `G40`. The controller D register must hold the wire radius, and the file header says so. The lead-in and lead-out must be at least one wire radius long. The exporter still computes the offset path, to run the same refusals and checks. Coordinated turning and ruled surfaces use `cam` only.

Offsets are applied in the wire plane normal to the path. For xyuv the offset is along the face normal, perpendicular to the wire.

### Leads

The lead-in is a straight line tangent to the start of the path. An open profile leaves along the end tangent. A closed profile leaves by retracing its lead-in, so rotate the start (`Profile.rotated_start`) to a convex vertex or an edge where the lead-in lies in waste. The exporter checks that the wire-centre lead lines stay at least one wire radius from the profile. It has no stock model, so rapids between setups are not checked against the stone or the fixture.

## Outputs

G-code, `write_gcode(job, machine)` returns one text per file.

- Header: comments, the `header` lines, then `G21 G90 G94|G93` and the arc plane.
- Order per setup: table rapid, rapid to the lead-in start, wire start, cuts, wire stop. Coolant on at the start, off and program end after the last setup.
- Every motion block carries its G word, every cutting block carries F (G94) or the inverse-time F (G93). There is no default feed. A missing or zero feed raises `FeedError`.
- Each file is a full program with wire start before its first cut and wire stop after its last. `check_program` verifies this on every file written.
- File split. A continuation file starts with the same header, coolant on, wire start if the wire was running, and a `G0` to the last position of the previous file. The split never falls inside a G41 or G42 span. If the span does not fit, the exporter refuses.
- `parse_gcode(text)` gives blocks. `gcode_positions(text, machine)` gives the modal position after every motion block in logical axes, with arc centres. The tests compare these with the planned path to the written decimals.

DXF, `write_dxf(job, machine, lwpolyline=False)` returns text.

- The wire-centre path in the wire plane, one layer per setup (`S1_NAME`). With G41 the nominal path is added on `S1_NAME_PROG`. xyuv writes the two carriage paths on `_END0` and `_END1` in (trolley, height) coordinates. Coordinated turning writes the path in (trolley, height) and drops the table angle.
- Default is R12 (`AC1009`) with LINE and ARC. `lwpolyline=True` writes one LWPOLYLINE with bulges per path (`AC1015`, R13 or later). Arcs stay arcs even when the G-code flattens them.

Planning. `plan_job(job, machine)` returns a `Plan` of moves and events without text. `Plan.summary()` gives the move count, cut length and an estimated cut time (cut length over feed, or table angle over `rotary_feed_deg_min` when slower, plus dwells).

## What was assumed

There is no machine manual behind this. All of the following are assumptions to confirm on the real machine.

1. Words, order and spelling. Absolute metric positions (`G21 G90`), `G0` rapids, `G1` cuts, `G2` and `G3` arcs, one block per line, comments in parentheses. M numbers are placeholders.
2. Wire start and stop, coolant and program end are M-codes, and the wire must run before the first cut.
3. Cutter compensation (`G41`, `G42`, `G40`, `D`) is accepted for a wire, with D holding a radius, and compensation can start on a straight lead-in block. Unconfirmed for stone wire saws. `cam` mode does not depend on it.
4. Feed. G94 gives the cutting speed in the wire plane. With a moving table in G94, controllers differ on how rotary degrees count, so G93 is the portable choice and G94 with coordinated turning should be checked. In G93 the F word is 1 over the block time. Real machines may adapt the feed themselves (wire tension, motor load) and treat F as a limit.
5. Arc planes and the direction of G2 and G3 for the chosen axis pair. Set `arc_first`, `arc_second` and `arc_words` from the manual. If unsure, set `arc_plane` to null.
6. Table. Degrees, which direction is positive, whether the controller wraps at 360, and whether it accepts moves beyond 360. The three unwinding rules cover the usual choices.
7. Coordinated turning. A straight wire at distance s from the table axis makes a cylinder of radius s. The table axis is vertical and in the wire plane. The table angle per mm and the largest step per block are cut-quality choices, not machine facts.
8. xyuv. The two carriages hold the wire ends at fixed depth positions and `carriage_spacing` apart, heights are absolute, and a block moves both ends together so the wire is straight between them. Whether the controller takes absolute or incremental U and V is a global setting there, and this exporter writes absolute.
9. Lead-in and lead-out lengths must exceed the wire bow. The placeholder is 60 mm. No bow model is used.
10. Dwell `G4 P` at corners and the corner slowdown come from wire EDM practice. For stone they are options, off by default.
11. File limits. Some controllers limit program size. `max_lines` splits files. A real limit may also be in bytes or in a native segment format.
12. DXF scale and units are millimetres, and the target software reads R12 LINE and ARC.
13. Travel limits are rectangular and independent per axis. Wire tilt limit is a single angle. Collisions with the stone, fixture, table and frame are not checked.

## Adapting a definition to a real machine

1. Copy the closest `examples/wiresaw_machines/generic_*.json`. Rename it.
2. Set `axes` to the letters of the machine and `limits` to its travel. Add `table_axis_trolley` for turning.
3. From the manual: M numbers, `feed_mode`, whether arcs are interpolated and in which plane, G41 support and the D register. Start with `kerf_mode` `cam` and `arc_plane` null. Change them one at a time.
4. Set the wire diameter to the real wire including the beads.
5. Set `header` and `footer` to the lines the controller requires (program number, safety lines, tool or wire data).
6. Set `max_lines` and `decimals` to what the controller accepts.
7. Export a short profile and read it on the controller simulator or on the machine without wire, stone and spindle power. Check axis directions, the table sense, arc directions and compensation side. Only then cut.
8. Keep the definition under version control next to the programs.

Safety. A generated program is not checked against the stone, the fixture or the machine envelope beyond the travel limits. A person who knows the machine must review the first program of each kind and run it dry.
