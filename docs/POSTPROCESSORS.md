# CAM post-processors: a light reading for lithocam

Date: 2026-11-12
Scope: a small reading of about 25 pages and search result sets, from standard web searches only. No PDFs were downloaded and no paywalled text was read. ISO standards were seen only through their public catalogue pages.

Status: reading notes, not validated. The section "What lithocam implements today" at the end maps the notes onto the code.

## Summary

- A post-processor turns machine-neutral cutter-location (CL) data into code for one controller. ISO 3592 defines the CLDATA file format. ISO 4343 defines the post-processor commands carried in CLDATA. G-code (ISO 6983 / RS-274) is the usual output. STEP-NC (ISO 14649) aims to replace that model with feature-based programs.
- Open posts (FreeCAD CAM) and commercial posts share one shape. They read a stream of motion and state records, keep modal state, and write a block only when a value changes. They add header, tool-change and footer blocks from a machine profile.
- 5-axis posts solve tool axis vector to rotary angles per machine type. Most tool axes have two rotary solutions. Near the pole (tool axis parallel to a rotary axis) the angle becomes undefined and small vector changes ask for large rotary moves. Posts must pick a solution, guard travel limits, and linearise between points.
- Controllers with tool centre point control (Fanuc G43.4, Heidenhain M128 / FUNCTION TCPM, Siemens TRAORI) take tool tip plus orientation and do the kinematics themselves. Without TCP the post must emit machine joint coordinates, which ties the program to one machine and one tool length.
- Robot posts receive a pose, the joint solution and a configuration triple per target. They write frame, tool, speed and blending instructions, and split long programs. Configuration data (KUKA Status/Turn, ABB robconf) pins the joint solution so the controller does not choose a different one.
- Post systems keep the machine description out of the post logic. A typical split is a machine definition, a control definition and a post file.
- Verification means running the posted code on a simulator of the same dialect: CAMotics or LinuxCNC for G-code, URSim for URScript, a virtual controller for KRL and RAPID. Reported faults include missing tool length offset, wrong arc plane, and rotary moves that oscillate or flip.

## 1. What a post-processor is and where it sits

A CAM system computes toolpaths in part coordinates. It does not know the target controller. A post-processor converts that neutral output into the program a specific machine accepts.

**Standards in the chain.**

- **APT and CLDATA (ISO 3592).** ISO 3592:2000 specifies the CLDATA reference language and its file layout. Its record types cover post-processor commands, tool position data and part contour description. The name comes from cutter location data. ISO lists the 2000 edition as current, last confirmed in 2021. Source: [ISO 3592:2000 catalogue](https://www.iso.org/standard/33798.html).
- **Post-processor commands (ISO 4343).** ISO 4343:2000 defines the post-processor statements (tool change, spindle, coolant and similar) that a CAM processor writes and a post reads. They are encoded on the 2000-class and 20000-class CLDATA records. The standard requires every ISO NC language processor to produce these records and every post to accept them. Sources: [ISO 4343:2000 catalogue](https://www.iso.org/standard/33799.html), [iTeh preview](https://standards.iteh.ai/catalog/standards/iso/848b5e4b-62f6-4bec-bace-d18b9ddfae7a/iso-4343-2000).
- **G-code (ISO 6983 / RS-274).** The output language for most mills. The NIST RS274/NGC Interpreter Version 3 report (NISTIR 6556, Kramer, Proctor, Messina, 2000) is the public reference. It describes an interpreter that reads NC code and calls a set of canonical machining functions. LinuxCNC descends from this work. Source: [NIST publication page](https://www.nist.gov/publications/nist-rs274ngc-interpreter-version-3?pub_id=823374).
- **STEP-NC (ISO 14649).** A feature- and workingstep-based data model. ISO 14649-17 states that the older language programs the tool centre path relative to machine axes, not the machining process relative to the part, and that vendors extend it beyond its scope. Source: [ISO 14649-17:2020 catalogue](https://standards.iteh.ai/catalog/standards/iso/0eb9e578-4c89-4e13-8983-b85379e189f4/iso-14649-17-2020).

**Where lithocam sits.** lithocam already produces CL data (tip position, tool axis, feed, tool id, operation boundaries). In the ISO model that is the CAM processor side. The new work is the post side.

## 2. Pipeline stages inside a post

**Reference architecture (FreeCAD CAM).** The FreeCAD developer wiki describes the post pipeline in stages. Configuration merges machine, user, job and run settings. Ordering groups operations per fixture and drops repeated tool changes when two operations share a tool. Expansion turns canned drilling cycles into plain moves where needed and splits arcs if the machine requires it. Conversion applies units and precision, formats lines and comments, and suppresses unchanged axis and feed words (modal filtering). It also inserts preamble, postamble, operation and tool-change blocks. A final pass removes redundant moves and adds line numbers. The post returns text and the caller writes the file. Source: [FreeCAD Postprocessing developer wiki (sliptonic mirror)](https://github-wiki-see.page/m/sliptonic/FreeCAD/wiki/Postprocessing). The official FreeCAD wiki page ([CAM Post](https://wiki.freecad.org/CAM_Post)) refused the automated fetch, so this reading relies on the mirror.

**Event model.** A common post design is a script that the kernel calls with one NC record at a time. Entry functions cover the header, each operation section, rapid, linear and circular moves, canned cycles, spindle speed, radius compensation, commands such as spindle stop, and the footer. 5-axis moves arrive with position plus direction components.

**Stage notes with the G-code facts a post must respect.** All items below come from the [LinuxCNC G-code reference](https://linuxcnc.org/docs/stable/html/gcode/g-code.html) unless marked.

- *Parsing CL records.* Read motion, tool, feed, spindle, coolant and cycle records in order. Keep a modal state object.
- *Units.* G20 selects inches and G21 millimetres. Convert once, at the boundary.
- *Work offsets.* G54 to G59.3 select coordinate systems 1 to 9. Offsets are stored on the controller and set with G10 L2. The post names the offset; it does not bake it into coordinates.
- *Tool change and length offset.* G43 applies the tool table length offset (H word or loaded tool). G43.1 and G43.2 apply given or added offsets.
- *Radius compensation.* G41 / G42 offset left / right of the path with an optional D word. LinuxCNC requires a lead-in at least as long as the tool radius and forbids compensation in the YZ plane.
- *Spindle and coolant.* M3/M4/M5 and coolant M codes. A post that leaves out coolant codes forces hand edits.
- *Rapid vs feed.* G0 vs G1 with F. A post should avoid dog-leg rapids that cut across the part.
- *Arcs and arc fitting.* G17/G18/G19 select the XY, ZX or YZ plane, and arcs lie in the active plane. G2 is clockwise viewed from the positive end of the normal axis. Centre format is preferred. LinuxCNC rejects an arc when start and end radii differ by more than 0.05 in / 0.5 mm, or by more than 0.0005 in / 0.005 mm and 0.1% of radius. A post that fits arcs to CL points must stay inside this check.
- *Canned cycles.* G81 to G89 cover drilling, boring and tapping with an R retract plane and optional L repeats. G83 pecks by increment Q and retracts to R between pecks. G84 is right-hand tapping with feed equal to spindle speed times pitch.
- *Line numbers, block format, modal suppression.* Optional N words, fixed decimal precision per address, and change-only output (FreeCAD above).
- *Safe start and end.* A start block that sets plane, units, absolute mode and cancels compensation; an end block that retracts, stops spindle and coolant, and ends the program. A FreeCAD forum thread discusses adding G49 (cancel tool length) to the LinuxCNC post's init block ([FreeCAD forum](https://devtalk.freecad.org/t/linuxcnc-processor-cancel-tool-length-compensation-g49-in-init-g-code/50230)).
- *Program size.* Robot posts split programs by a maximum line count (section 4). This reading found no public numeric limit for CNC controllers; the value belongs in the machine file.

## 3. Multi-axis posts

**Inverse kinematics per machine type.** Lee and She (1997) built a post on homogeneous transformation matrices for three configurations: table-tilting, spindle-tilting, and table/spindle-tilting. They checked it with a trial cut measured on a CMM. Source: Lee R.-S., She C.-H., Int. J. Adv. Manuf. Technol. 13(9):658-665, 1997, [doi:10.1007/BF01350824](https://www.doi.org/10.1007/BF01350824) (abstract only). The LinuxCNC 5-axis kinematics page derives the forward and inverse transforms for table rotary/tilting machines (xyzac-trt, xyzbc-trt) with pivot offsets. It solves one angle from the third row of the orientation equation and the other from a ratio of rows, and states that CAM post-processors usually do this conversion. Tool length enters as an extra Z offset through a HAL pin. Source: [LinuxCNC 5-axis kinematics](https://linuxcnc.org/docs/stable/html/motion/5-axis-kinematics.html). The page does not discuss head-head machines, multiple solutions or singularities.

**Two solutions and the pole.** A 5-axis machine usually has two rotary poses for one tool axis. Picking the pose with the least tool-axis change between points can fall outside travel limits. When the tool axis is parallel to a rotary axis, that axis is singular and small orientation changes demand large rotary moves. A post can limit the singular axis by moving the other rotary axis slightly and accepting a small, bounded tool-axis deviation. It can also add intermediate points so the tool tip stays within a set deviation from the straight line during rotary moves (linearisation). A 2023 paper in *Machines* treats the two-solution case and the infinite-solution case at the singularity ("Rotation-Angle Solution and Singularity Handling of Five-Axis Machine Tools for Dual NURBS Interpolation", *Machines* 11(2):281; seen only as a search snippet, full text not read).

**Rotary wrap and rewinds.** A rewind retracts to a safe position, rotates the axes above the re-entry point, and returns. The post must track the unwrapped angle, choose the shortest path where the axis is continuous, and retract before unwinding where it is not.

**TCP vs machine coordinates.**

- *Fanuc G43.4 / G43.5.* Fanuc America describes tool centre point control as the program stating where the tool should be, with the control computing axis motion and removing kinematic math from the post. Source: [Fanuc America 5-axis](https://www.fanucamerica.com/cnc-applications/5-axis).
- *Heidenhain M128 / FUNCTION TCPM.* Holds the tool tip position while tilt axes move. Heidenhain sells TCPM as a control option. Source: [Heidenhain options and accessories brochure](https://www.heidenhain.de/fileadmin/pdf/en/01_Products/Prospekte/PR_Options_and_Accessories_for_TNC_ID827222_en.pdf).
- *Siemens TRAORI.* Activates orientation transformation. Orientation can be given as machine axis angles (A, B, C), Euler or RPY angles (A2, B2, C2), or a direction vector (A3, B3, C3) pointing from tool tip to tool holder. ORIWKS and ORIVECT select interpolation modes. Source: Siemens 840D sl programming manual, tool orientation section, as hosted on [ManualsLib](https://manualslib.com/manual/1670082/Siemens-Sinumerik-840d-Sl.html?page=333) (third-party host, 2011 edition).
- *LinuxCNC.* The kinematics module does the transform once configured. Forum users describe switching it with custom M codes. This reading did not find a G43.4 equivalent in the official docs.

With TCP the post emits tip position plus orientation and the program survives a tool length change. Without TCP the post emits joint coordinates and must include the tool length and pivot offsets.

**Inverse-time feed.** In G93 mode an F word means the move takes 1/F minutes. Every G1, G2 and G3 line then needs its own F. G0 is unaffected. Source: [LinuxCNC G-code reference](https://linuxcnc.org/docs/stable/html/gcode/g-code.html). Posts use it for simultaneous rotary moves, where a linear feed per minute does not describe the tip speed.

## 4. Robot posts

**Robot post model.** A typical robot post is one class with methods called in program order: program start, set frame, set tool, set speed, set zone (blend radius), joint move, linear move, circular move, set digital output, wait for digital input, message, program finish and program save. The linear move receives the target pose, the joint solution and a configuration triple (rear, lower arm, flip). A maximum line count splits long programs into subprograms called in sequence.

**Per controller.**

- *UR URScript.* `movel` takes a blend radius `r`, where 0 means stop at the point. `servoj` streams joint setpoints at a fixed period; the e-Series manual gives a default `t` of 0.002 s and a `lookahead_time` range of 0.03 to 0.2 s. Sources: [UR script directory](https://www.universal-robots.com/manuals/latest/en/pdf/ps5/script-directory/), [UR servoj article](https://universal-robots.com/articles/ur/programming/servoj-command). lithocam already writes this dialect.
- *KUKA KRL.* PTP, LIN, CIRC and spline motions, approximated with C_DIS and related `$APO` settings. Status and Turn select one axis solution among several; Beckhoff's KUKA-compatible reference points to the KUKA system software manual for detail ([Beckhoff TF5120 structures](https://infosys.beckhoff.com/content/1033/tf5120/13081351563.html)). This reading found no official KUKA manual page; the KRL details here are indicative.
- *ABB RAPID.* A robtarget holds position, quaternion orientation, `robconf` (cf1, cf4, cf6, cfx) and external axes. Each cf value names the 90 degree quadrant of an axis. Zone data sets corner blending for fly-by points. Source: [FZI robot interface notes on ABB](https://fzi-robot-interface-proposal.readthedocs.io/en/latest/industry/ABB.html). An official ABB RAPID reference was not reached in this reading.
- *Fanuc TP.* This reading did not cover TP syntax.

**Robot machining specifics.** For a spindle on a robot, rotation about the tool Z axis is free, so the CL tool axis gives only 5 of 6 pose degrees of freedom. The post or the planner must choose the sixth, and can use it to avoid singularities and joint limits. A 6-axis robot cannot cross a singularity on a linear move.

## 5. What the post must know about the machine

- **FreeCAD CAM.** Machine, user, job and run settings merge into one configuration before the post runs (section 2).
- **Robot posts.** The robot model holds kinematics and limits. The post holds only the dialect. Each robot links to one post.

Common content: axis names and order, kinematic chain with offsets, travel and rate limits, controller dialect, supported functions (TCP, G93, canned cycles, arc planes), header and footer templates, and number formats.

## 6. Verification

- **G-code.** CAMotics simulates 3-axis G-code, shows the toolpath and the cut stock, and exports the result as STL. It is GPL-2.0+. Source: [CAMotics overview (LinuxLinks)](https://www.linuxlinks.com/camotics-3-axis-cnc-gcode-simulator/). LinuxCNC can run with simulated hardware and Vismach models, which its 5-axis page uses to show the kinematics ([LinuxCNC 5-axis kinematics, section 6.1](https://linuxcnc.org/docs/stable/html/motion/5-axis-kinematics.html)). Running the posted file through the target interpreter catches syntax and arc-radius errors.
- **Robots.** URSim emulates a UR controller for offline program tests and runs on Linux or a Linux VM ([MathWorks URSim setup](https://mathworks.com/help/robotics/urseries/ug/setup-ursim-offline-simulator.html)). KUKA OfficeLite and ABB RobotStudio provide virtual controllers for KRL and RAPID. This reading did not reach vendor pages for those two; confirm licensing before planning on them.
- **Reported faults.**
  - Tool length and diameter offsets wrong or generic, needing hand edits; coolant codes missing; head-table machines oscillating around zero on a rotary axis.
  - Rotary travel limits hit when the chosen solution is out of range.
  - Wrong arc plane and wrong units appear in practitioner forums. This reading found no primary source with a measured frequency. The defensive checks are cheap: emit G17/G18/G19 and G20/G21 explicitly in every safe-start block.

## Design for lithocam

*Suggestion only. Not validated. It reflects this reading, not a tested design.*

**Layer 1: CL-data model.** One neutral, ordered record stream, close to ISO 3592/4343 in meaning but stored as plain data (JSON or a typed in-memory list). Record kinds: `ToolChange(id, length, radius)`, `Spindle(rpm, dir)`, `Coolant(mode)`, `Feed(value, mode)`, `Rapid(p)`, `Linear(p, axis?)`, `Arc(p, centre, normal, cw)`, `Cycle(kind, params, points)`, `OpStart(name)` / `OpEnd`, `Comment`, `Dwell`. Positions are tool tip in a named part frame, always millimetres. The tool axis is a unit vector on each 5-axis record. No machine, controller or offset values live here. An APT/CL text writer becomes one more dialect on this stream, which gives a cheap round-trip test.

**Layer 2: machine-definition file.** One file per physical cell, separate from code. Contents: machine kind (3-axis, table-table, head-head, head-table, 6-axis robot, robot plus track or turntable); kinematic chain with pivot offsets; axis names and order; travel, velocity and rate limits; which rotary axes are continuous; supported functions (TCP mode, G93, canned cycles, arc planes, splines); work offset register; robot tool and base frame indices; configuration policy (fixed robconf or S/T, or nearest); maximum program lines; number formats; header and footer templates. Keep the dialect name here so one machine file selects one writer.

**Layer 3: kinematic solver (shared, between layers 1 and 4).** For CNC without TCP: tool axis to rotary angles, both solutions, choose by continuity then limits, detect the pole cone and hold the near-singular axis, unwrap angles, insert retract-rotate-return when a limit forces a rewind, and add intermediate points so the tip deviation stays under a tolerance from the machine file. For CNC with TCP: pass tip and vector through. For robots: reuse the existing lithocam robot path code to pick the free rotation about the tool axis and the joint solution, then attach configuration flags. Every choice is logged per point so the verification layer can show it.

**Layer 4: dialect writers.** One small writer per controller. Each writer only formats; it does no geometry.

- *ISO / Fanuc*: G17-G19, G20/G21, G54-G59, G43 H, G41/G42 optional, G2/G3 centre form, G81-G89, G93 for simultaneous rotary moves, G43.4 when TCP is on.
- *LinuxCNC*: as ISO, with its arc-radius check enforced in the writer, and no G43.4 unless the machine file says the kinematics module handles it.
- *Heidenhain*: conversational blocks, M128 or FUNCTION TCPM for 5-axis.
- *Siemens*: TRAORI with A3/B3/C3 vectors for 5-axis.
- *URScript* (exists), *KRL* (exists), *RAPID* (new): frame and tool setup, move with blend or zone, configuration data per target, split by line limit.
- *APT/CL*: neutral text dump of layer 1 for exchange and debugging.

Each writer keeps a modal state and writes only changed words. Each writer has fixed safe-start and safe-end blocks that set plane, units, absolute mode and cancel compensation explicitly.

**Layer 5: verification.** Three gates, cheapest first. (a) Static: re-parse the posted file and replay it into a CL stream; compare with layer 1 within tolerance. Check limits, arc radii, F present in G93 lines, tool length active before the first feed move. (b) Simulator: CAMotics or LinuxCNC sim for G-code, URSim for URScript, a virtual controller for KRL and RAPID where licensed. (c) Visual: render the replayed path in Rhino coloured by rotary jump size and configuration change, for a person to accept. No code reaches hardware until a hardware gate is opened.

## What lithocam implements today

- **Layer 1 (records):** `lithocam.post.Rec`. Kinds: start, end, comment, tool, speed, movej, movel, pause, io_set, io_wait, spindle, gripper and call. Robot targets carry joint values from lithocam's own verified IK. Machine-tool records (rapid, linear, arc, cycle with a tool axis) are not built yet.
- **Layer 3 (kinematics):** `lithocam.studio.kinematics` provides URDF forward kinematics, the Jacobian and damped least-squares IK. Tests check them against the published UR10e DH parameters and finite differences. `lithocam.robot` (Tesseract) adds collision checks.
- **Layer 4 (writers):**
  - `urscript`: `movej`, and `movel` with joint targets, which moves linearly and ends in the verified configuration.
  - `krl`: `PTP {AXIS}`, `$TOOL`, `$VEL`.
  - `rapid`: `MoveAbsJ`, `tooldata`, `speeddata`.
  - `cl`: neutral text that round-trips.
- **Layer 5 (verification), gate (a):** `post.joint_targets` reads the joint targets back from every dialect. The studio compares them with the records after each post.
- **Not yet:**
  - the machine-definition file (layer 2);
  - G-code dialects;
  - Cartesian robot targets with configuration flags (KUKA S/T, ABB confdata);
  - program splitting by line limit;
  - simulator gates (b) in URSim, OfficeLite and RobotStudio.

## Limits of this reading

- ISO 3592, 4343, 6983 and 14649 were read only through catalogue pages. No clause-level claims are made.
- No official KUKA or ABB manual page was reached. KRL and RAPID details rest on Beckhoff and FZI pages.
- The official FreeCAD wiki blocked the automated fetch. FreeCAD content comes from a mirror of the developer wiki.
- Paper content (Lee and She 1997; the 2023 *Machines* paper) is from abstracts and search snippets, not full text.
- No numbers on fault frequency were found; none are given.
