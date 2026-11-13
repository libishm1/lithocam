# lithocam studio

The studio is a desktop program for stone CAM on industrial robots. It holds a cell (robot, spindle, turntable),
a part and its stock, a tool library and programs. It simulates the programs and writes controller code.
Robots: UR10e, KUKA KR120 R2500, KUKA KR 480 R3330 MT and KR 500 R2830 MT. The robot programs are for simulation
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
  always allowed).

Every joint move between and inside steps is simulated at 2 degree steps and collision-checked when collision
checks are on. Milling posts every point the step tolerance needs. Passes start with a plunge at the plunge feed
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

Command line:

```
python -m lithocam.studio.api steps
python -m lithocam.studio.api schema
python -m lithocam.studio.api check station.lcs.json [PROGRAM]
python -m lithocam.studio.api post station.lcs.json PROGRAM DIALECT OUT
```

## Recording frames for documentation

`python -m lithocam.studio --record DIR N [--five-axis-demo] [--no-collisions] [--timeout S] [station.json]`
generates the program, hides the docks and saves N frames of the 3D view. `docs/media/frames_to_gif.py DIR OUT.gif`
turns them into a GIF (needs Pillow). Batch modes exit on failure and after the timeout; they never wait for input.
