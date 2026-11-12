# Operations: coverage and roadmap

The list combines the standard 2 / 2.5 / 3 / 5-axis operation families with the operation names that SprutCAM's public manual uses. SprutCAM names come from the [SprutCAM 15 operations matrix](https://kb.sprutcam.com/SC15manual/en/10389.html) and the SprutCAM 16 [3-axis](https://kb.sprutcam.com/SC16manual/en/10116.html) and [configuration](https://kb.sprutcam.com/SC16manual/en/261.html) pages. No SprutCAM code or documentation text is used; names only, as a checklist.

Status key:

- **done**: implemented and tested; the evidence is linked or named.
- **partial**: the core exists, but there is no complete operation or interface.
- **planned**: not started. The "Builds on" column says which existing modules it will reuse.

## 2-axis and 2.5-axis

| Operation (SprutCAM name) | lithocam | Status | Builds on |
|---|---|---|---|
| 2D profiling (2D contouring) | `kerf_path_convex` for convex parts; fibre waterline contours (`examples/pocket_2p5d.py`) | partial | General polygon offsets (Clipper2, BSL-1.0) and the kerf theorems |
| Face milling | zig-zag at a fixed z with a flat cutter (`zigzag`) | partial | A dedicated op: stock top, stepover from the cutter diameter, lead-in and lead-out |
| Drilling and tapping (hole machining) | holes found by quiddity, routed to drill or core drill (`lithocam.features`) | partial | Drill, peck and tap cycles as CL plus robot/CNC export |
| Pocketing / 2.5D pocketing | z-level roughing plus raster plus contour finish, verified in the Z-map (`examples/pocket_2p5d.py`) | done (example) | Move into the core as an operation |
| Slotting | kerf slots and grooves found (`features`); saw-disc and router routes | partial | Slot toolpaths: disc cut and router zig-zag |
| Chamfering / 2.5D chamfer machining | chamfers found and routed to the profile wheel | partial | Chamfer path with a cone cutter (the cone drop-cutter exists) |
| Engraving | – | planned | Curve-following drop-cutter with a cone (V-bit) cutter (`adaptive_line` on curves) |
| 2.5D wall / cover / flat land machining | – | planned | Horizontal-area detection on the mesh plus a flat cutter |
| Adaptive / trochoidal clearing (constant engagement) | – | planned | Medial-axis and engagement-angle papers in the CAM review (K1, K1063, K1064, K1073, K4697) |
| Plunge roughing | – | planned | Drill-like plunges on a grid with the drop-cutter floor |
| Boring, back boring, spot facing, counterbore and countersink | holes with cbore and csink found by quiddity | planned | Hole cycles |
| Thread milling (helical) | – | planned | Helical interpolation around the hole axis |
| Peck drilling, chip-break drilling, dwell | – | planned | Hole cycles (drill CL + export) |
| Entry and linking moves: helical, ramp, plunge, lead-in, lead-out, retract, rapid links | – | planned | Shared by every operation; collision-checked links |

## 3-axis

| Operation | lithocam | Status | Builds on |
|---|---|---|---|
| 3D contouring (drop-cutter along curves) | `adaptive_line` (any segment), drop-cutter for 4 cutter kinds, checked against OpenCAMLib | partial | Curves and polylines as input |
| Plane finishing (raster) | `zigzag` with scallop-verified stepover; Z-map shows cusps peak at h | done | – |
| Plane roughing | z-levels with tip = max(level, drop-cutter + allowance) (pocket example) | done (example) | Core op with a stock model |
| Waterline roughing / finishing | fibre crossings of the drop-cutter height (pocket example) | partial | M4: push-cutter and fibres in C++ (references from OpenCAMLib's literature) |
| Optimized plane / complex (waterline + plane) | – | planned | Waterline + plane split by slope angle |
| Scallop (constant scallop) | the scallop bound exists (`max_stepover`, Lean) | planned | Offsetting passes on the surface |
| 3D helical, morph | – | planned | – |
| High-speed finishing | – | planned | Smoothed and trochoidal links (K1, K1063, K1064, K1073 in the CAM review) |
| Deep cavity milling | – | planned | Tool holder and shank collision against the stock (the tool spec has holder dimensions) |
| Rest machining: corners cleanup, pencil | – | planned | Z-map leftover map to regions to a smaller cutter |
| Side milling | – | planned | Waterline with a flat or bull cutter |
| Lettering and engraving | – | planned | As engraving |
| Drive roughing and finishing; combined (waterline + drive) | – | planned | Guide-curve-driven passes |
| Spiral, radial, flowline (UV), project-curve finishing | – | planned | Pass generators feeding `adaptive_line` |
| Steep and shallow split (slope-limited) | – | planned | Slope from drop-cutter normals; waterline for steep, raster for shallow |
| 3D curve milling (tool on a 3D curve) | `adaptive_line` (straight segments) | partial | Polyline and spline input |
| Gouge and collision check of every op (tool, holder, stock) | drop-cutter (no gouge by construction), Z-map and Manifold simulation | partial | Holder and shank checks in the tool spec |

## 5-axis (and robot 6-axis)

| Operation | lithocam | Status | Builds on |
|---|---|---|---|
| 3+2 indexed (multi-face drilling and milling, prismatic blocks on 4-5 sides) | turntable sampling (`Positioners`, `facing_yaw`); 3-axis ops per setup | partial | Setups per face; 3-axis op per setup |
| 5D contouring / flank (swarf) of ruled faces | `lithocam.flank`: frames, stepped roughing, Manifold simulation; KR120 path with turntable (`examples/flank_5axis.py`) | done (example) | Gouge bound (hull approximation, measured) |
| 5D surfacing (point milling with tilt) | – | planned | Tool-axis control (lead/lag, tilt) + collision |
| 5D by meshes | – | planned | Drop-cutter along a tilted tool axis |
| Undercut machining | – | planned | 5D surfacing + tri-dexel or OpenVDB stock (`docs/STOCK_SIMULATION.md`) |
| Impeller / blade / mold sculpting | – | planned (not a stone priority) | 5D surfacing |
| 4D contouring / rotary roughing and finishing | turntable as a sampled axis | planned | Rotary axis in the cell (turntable joint) |
| 4-axis wrapping (pattern wrapped on a cylinder) | – | planned | Rotary axis + 2D ops in (angle, z) |
| Swarf with tilt, lead and lag control; tool-axis smoothing | flank frames fix the axis on the ruling | partial | Axis-control strategies and smoothing (robot-flank papers K345, K2397) |
| Multi-axis drilling (holes at any angle) | – | planned | Hole cycles along the hole axis + robot IK |
| Machine/robot kinematics, limits and singularities | verified IK, joint limits, wrap variants, track and turntable | done | Singularity measure along the path |
| Robot 6D (UR10e, KR120) | verified IK, track and turntable, collision with the official meshes, URScript and KRL | done | URSim run (blocked on Docker), KUKA and ABB simulation |

## Stone-specific (beyond SprutCAM's milling set)

| Operation | lithocam | Status |
|---|---|---|
| Diamond wire, planar and ruled (SprutCAM: wire EDM 2D/4D) | `ruled` labels, `wiresim` (exact planar), `meshsim` (ruled), robot bow saws, Lean wire-tube theorem | done |
| Disc saw (SprutCAM: disc roughing, disc cutting 2D/6D) | saw-disc routes from features, kerf theorems | planned (toolpaths) |
| Jet cutting 2D/5D (waterjet) | contour-cut route for through cutouts | planned |
| Lathe (stone turning: balusters). SprutCAM lathe set: OD/ID roughing and finishing, facing, hole machining, part-off, OD/ID/face grooving, OD/ID/profile threading | turned-profile route from features | planned |
| Polishing and grinding (stone surface finish, constant force) | – | planned (robot force control is hardware-gated) |
| Splitting and guillotine block cutting (quarry) | wire-saw guillotine sequencing exists in Frahan | planned (bridge) |
| Feature-based machining (SprutCAM: FBM) | quiddity + stone rule table (`features`) | done (recognition and routing) |

Out of scope for lithocam: cladding, welding, knife cutting.

## Shared by every operation

| Capability | lithocam | Status | Builds on |
|---|---|---|---|
| Tool library (kinds, dimensions, holder, shank, feeds) | the four drop-cutter kinds (flat, ball, bull, cone) | partial | Library in the C++ core with bindings; selection in the Qt GUI |
| Stock models | Z-map (3-axis), Manifold meshes (wire and flank) | partial | OpenVDB level set for undercuts (`docs/STOCK_SIMULATION.md`) |
| Simulation and verification (gouge, leftover, scallop) | Z-map deviation maps; Manifold volume deltas; Lean bounds for scallop and wire tube | done | Per-operation reports |
| Rest-material detection | – | planned | Z-map leftover above tolerance |
| Feed-rate control (engagement, chip load, stone-specific feeds) | – | planned | Engagement from the stock model |
| Toolpath smoothing and arc fitting | – | planned | Blend and fit after CL generation |
| Post-processors | URScript, KRL | partial | G-code (ISO), RAPID for ABB, APT/CL file |
| Machine and robot simulation with collision | Tesseract cells with official visual meshes and collision meshes; URSim | partial | URSim run (Docker), ABB IRB cell |
| Setup sheets and operation reports | – | planned | Per-op tool, time and volume summary |
| Probing and in-process inspection | – | planned (hardware-gated) | Scan-to-part comparison with the stock model |

## Build order (proposed)

1. **Tool library**: tool kinds with dimensions, holder and drop-cutter mapping in the C++ core and bindings; a selection interface in the Qt GUI.
2. **Operations as core objects.** Each operation takes a part, stock, tool and parameters and produces CL passes, which feed the simulation, the robot cell and the exporters. The ops: face, profile, pocket, plane roughing and finishing, waterline (M4), drill cycles, chamfer, engrave.
3. **Rest machining** from the Z-map leftover; deep-cavity holder collision.
4. **3+2 setups** on the turntable; 5D surfacing with tool-axis control; OpenVDB stock for undercuts.
5. **Disc-saw and waterjet toolpaths**; stone lathe.
