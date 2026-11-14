# Operations: coverage and roadmap

The list covers the standard 2 / 2.5 / 3 / 5-axis operation families, plus the stone-specific operations. It serves as a checklist; each row says what exists and what does not. Statuses were checked against the studio palette (`studio/catalog.py`) and the generators (`studio/cam.py`, `studio/ops.py`, `studio/program.py`).

Status key:

- **done**: the studio generates the path (a palette step) and a test covers it. The "lithocam" column names the step or module.
- **partial**: the core or an example exists, but the studio has no step for it, or the step is only part of the operation.
- **planned**: not started. The "Builds on" column says which existing modules it will reuse.

## 2-axis and 2.5-axis

| Operation | lithocam | Status | Builds on |
|---|---|---|---|
| 2D profiling (2D contouring) | `kerf_path_convex` for convex parts; fibre waterline contours (`examples/pocket_2p5d.py`); the `profile` step is in the palette without a generator | partial | General polygon offsets (Clipper2, BSL-1.0) and the kerf theorems |
| Face milling | step `face` (zig-zag at stock top minus depth; plunge, ramp or helix entry) | done | Lead-in and lead-out arcs |
| Drilling and tapping (hole machining) | steps `drill`, `core_drill`, `tap` (`studio/ops.py`); holes found in the part or listed | done | Spot drilling, counterbore and countersink cycles |
| Pocketing / 2.5D pocketing | z-level roughing (`rough_plane`) plus raster plus contour finish, verified in the Z-map (`examples/pocket_2p5d.py`); the `pocket` step has no generator | partial | A pocket step: region offsets and the entry moves |
| Slotting | kerf slots and grooves found (`features`); saw-disc and router routes | partial | Slot toolpaths: disc cut and router zig-zag |
| Chamfering / 2.5D chamfer machining | step `chamfer`: V-bit or chamfer bit along the top outline of the part or given edges, at a chamfer width | done | Chamfer of non-top edges and of curved edges with a normal-following axis |
| Engraving | step `engrave`: V-bit along polylines, depth from the groove width, the tip follows the top surface | done | A single-stroke font for lettering (none ships; letters are given as paths) |
| 2.5D wall / cover / flat land machining | step `flat_land`: flat or bull end mill on the horizontal faces, grouped by height | done | Wall finishing (side milling) |
| Adaptive / trochoidal clearing (constant engagement) | - | planned | Medial-axis and engagement-angle papers in the CAM review (K1, K1063, K1064, K1073, K4697) |
| Plunge roughing | - | planned | Drill-like plunges on a grid with the drop-cutter floor |
| Boring, back boring, spot facing, counterbore and countersink | step `bore` (helical boring); the others are not generated | partial | Hole cycles |
| Thread milling (helical) | - | planned | Helical interpolation around the hole axis |
| Peck drilling, chip-break drilling, dwell | steps `drill` and `core_drill` (peck, chip-break retract, dwell) | done | - |
| Entry and linking moves: helical, ramp, plunge, lead-in, lead-out, retract, rapid links | `entry` on `face`, `rough_plane` and `flat_land`: plunge, ramp (angle) or helix (radius, pitch); straight rapid links and retracts in every step | partial | Lead-in and lead-out arcs; the entry on the other milling steps; collision-checked links |

## 3-axis

| Operation | lithocam | Status | Builds on |
|---|---|---|---|
| 3D contouring (drop-cutter along curves) | `adaptive_line` (any segment), drop-cutter for 4 cutter kinds, checked against OpenCAMLib; the `engrave` step follows the top surface along polylines | partial | Curves and polylines as input to a general contouring step |
| Plane finishing (raster) | step `raster` (scallop-verified stepover, boundary); Z-map shows cusps peak at h | done | - |
| Plane roughing | step `rough_plane` (z-levels, tip = max(level, drop-cutter + allowance), entry choice) | done | - |
| Waterline roughing / finishing | step `waterline` (constant-Z loops, climb or conventional, boundary) | done | - |
| Optimized plane / complex (waterline + plane) | step `steep_shallow` (waterline on steep ground, raster on shallow ground, overlap band) | done | - |
| Scallop (constant scallop) | step `adaptive_raster` (cross-feed from the local curvature, scallop limit); the bound itself is `max_stepover` (Lean) | done | Passes that offset along the surface |
| 3D helical, morph | - | planned | - |
| High-speed finishing | - | planned | Smoothed and trochoidal links (K1, K1063, K1064, K1073 in the CAM review) |
| Deep cavity milling | - | planned | Tool holder and shank collision against the stock (the tool spec has holder dimensions) |
| Rest machining | step `rest`: the stock of the last stock simulation before the step, else a Z-map of the earlier milling steps, else the previous tool radius (`docs/OPERATIONS_HOLES_REST.md`) | done | - |
| Pencil (corner cleanup) | step `pencil` (ball-end mill, concave creases) | done | - |
| Side milling | - | planned | Waterline with a flat or bull cutter |
| Lettering and engraving | step `engrave`; text is refused until a single-stroke font exists | partial | A single-stroke font |
| Drive roughing and finishing; combined (waterline + drive) | - | planned | Guide-curve-driven passes |
| Spiral, radial, flowline (UV), project-curve finishing | - | planned | Pass generators feeding `adaptive_line` |
| Steep and shallow split (slope-limited) | step `steep_shallow` | done | - |
| 3D curve milling (tool on a 3D curve) | `adaptive_line` (straight segments) | partial | Polyline and spline input |
| Gouge and collision check of every op (tool, holder, stock) | drop-cutter (no gouge by construction), Z-map and Manifold simulation, SDF stock run, Tesseract collision in the studio | partial | Holder and shank checks in the tool spec |

## 5-axis (and robot 6-axis)

| Operation | lithocam | Status | Builds on |
|---|---|---|---|
| 3+2 indexed (multi-face drilling and milling, prismatic blocks on 4-5 sides) | the `setup` step sets the tool tilt and the turntable angle, and 3-axis steps run in that frame; the `indexed` palette entry has no generator | partial | A test of a tilted setup; setups per face |
| 5D contouring / flank (swarf) of ruled faces | `lithocam.flank`: frames, stepped roughing, Manifold simulation; KR120 path with turntable (`examples/flank_5axis.py`); the `flank` step has no generator | partial | A studio step; gouge bound (hull approximation, measured) |
| 5D surfacing (point milling with tilt) | step `surfacing5` (ball end mill, lead angle, tilt limit, axis-rate limit, planner picks the spin) | done | Collision-aware axis choice |
| 5D by meshes | â€“ | planned | Drop-cutter along a tilted tool axis |
| Undercut machining | â€“ | planned | 5D surfacing + tri-dexel or OpenVDB stock (`docs/STOCK_SIMULATION.md`) |
| Impeller / blade / mold sculpting | - | planned (not a stone priority) | 5D surfacing |
| 4D contouring / rotary roughing and finishing | turntable as a sampled axis; the `rotary4` step is planned | planned | Rotary axis in the cell (turntable joint) |
| 4-axis wrapping (pattern wrapped on a cylinder) | - | planned | Rotary axis + 2D ops in (angle, z) |
| Swarf with tilt, lead and lag control; tool-axis smoothing | flank frames fix the axis on the ruling; `surfacing5` smooths its axis | partial | Axis-control strategies for swarf (robot-flank papers K345, K2397) |
| Multi-axis drilling (holes at any angle) | - | planned | Hole cycles along the hole axis + robot IK |
| Machine/robot kinematics, limits and singularities | verified IK, joint limits, wrap variants, track and turntable | done | Singularity measure along the path |
| Robot 6D (UR10e, KR120) | verified IK, track and turntable, collision with the official meshes, URScript and KRL posts | done | URSim run (blocked on Docker), KUKA and ABB simulation |
| Turntable as a coordinated external axis (E1) | `Station.turntable` with `driven: "e1"`; the table angle is a sample channel, posted and read back (`tests/test_turntable_e1.py`) | done | - |

## Stone-specific (beyond the milling set)

| Operation | lithocam | Status |
|---|---|---|
| Diamond wire, planar and ruled (as in wire EDM 2D/4D) | step `wire_ruled` (ruled faces, rolls, table window); `ruled` labels, `wiresim`, `meshsim`, robot bow saws, Lean wire-tube theorem. The planar wire cut has no studio step | done (ruled); partial (planar) |
| Disc saw (disc roughing, disc cutting 2D/6D) | steps `disc_cut` (kerf-compensated profile), `disc_comb` (comb roughing) and `disc_ruled` (cuts along the rulings of ruled faces) in the studio, from `lithocam.discsaw`; blade-intrusion checks. Tested; not yet visually validated | done |
| Jet cutting 2D/5D (waterjet) | contour-cut route for through cutouts | planned |
| Lathe (stone turning: balusters). Lathe operation set: OD/ID roughing and finishing, facing, hole machining, part-off, OD/ID/face grooving, OD/ID/profile threading | turned-profile route from features | planned |
| Polishing and grinding (stone surface finish, constant force) | step `polish` (`docs/POLISHING.md`): constant-contact pad paths with the tool axis on the surface normal, grit steps with a tool change per grit; force and pressure are simulation attributes only. Tested; not yet visually validated; no process constants built in; robot force control is hardware-gated and dormant | done (paths); force control not built |
| Splitting and guillotine block cutting (quarry) | wire-saw guillotine sequencing exists in Frahan | planned (bridge) |
| Feature-based machining | quiddity + stone rule table (`features`) | done (recognition and routing) |

Out of scope for lithocam: cladding, welding, knife cutting.

## Shared by every operation

| Capability | lithocam | Status | Builds on |
|---|---|---|---|
| Tool library (kinds, dimensions, holder, shank, feeds) | 38 kinds and 63 stone and wood tools from the survey (`docs/TOOLING.md`); CSV files; drop-cutter mapping for 22 of them (flat, ball, bull, cone and the general line-and-arc profile: tapered ball W8, flame burr S13); bound to Python; library selection in the studio | done | Drop-cutter for the remaining envelopes (side-profile wheels and bits, undercut, step drills) |
| Stock models | Z-map (3-axis), SDF stock run in the studio (`studio/process.py`), Manifold meshes (wire and flank) | partial | OpenVDB level set for undercuts (`docs/STOCK_SIMULATION.md`) |
| Simulation and verification (gouge, leftover, scallop) | Z-map deviation maps; Manifold volume deltas; SDF stock run; Lean bounds for scallop and wire tube | done | Per-operation reports |
| Rest-material detection | the `rest` step: stock run, Z-map or previous tool (see above). The stock is compared at the tool centre, so a fillet beside the path is added from the previous tool | done | - |
| Feed-rate control (engagement, chip load, stone-specific feeds) | feed limits per step from the stock run (`lithocam.materials`); no sourced constants for most materials | partial | Engagement from the stock model |
| Toolpath smoothing and arc fitting | `cam.simplify` (tolerance-bounded point thinning); no arc fitting | partial | Blend and fit after CL generation |
| Post-processors | URScript, KRL, RAPID (ABB), G-code for CNC machine files (`post.write_files`, read-back checked), neutral CL text | done | APT/CL file |
| Machine and robot simulation with collision | Tesseract cells with official visual meshes and collision meshes; URSim | partial | URSim run (Docker), ABB IRB cell |
| Setup sheets and operation reports | `studio/sheet.py`: a Markdown sheet per program (tool, kind, feed, rpm, path lengths, time, removed volume with a stock run); studio toolbar action, `Session.setup_sheet`, `python -m lithocam.studio.api sheet` | done | Tool-change and table-turn times |
| Probing and in-process inspection | - | planned (hardware-gated) | Scan-to-part comparison with the stock model |

## Build order (proposed)

1. **Tool library**: done for the tool kinds with a drop-cutter model.
2. **Operations as studio steps.** Done: face, plane roughing, raster, waterline, steep and shallow, adaptive raster, rest, pencil, flat lands, chamfer, engrave, hole cycles, 5D surfacing, disc saw, wire, polish. Open: profile, pocket, slot, adaptive clearing.
3. **Deep-cavity holder collision**; entry moves on the remaining steps.
4. **3+2 setups** with a test; OpenVDB stock for undercuts.
5. **Waterjet toolpaths**; stone lathe.
