# ROBOmove import (Quarra Stone Q-LAB)

Credit: Quarra Stone published ROBOmove backups of their KUKA KR 480 MT and KR 500 cells in [Q-LAB](https://github.com/brnmsmith/Q-LAB). The owner of lithocam has approval to use them with credit.

The Q-LAB repository has no licence file. lithocam therefore reads those files from a local clone at run time and does not redistribute them. No file or file content is in this repository. The clone is found through `LITHOCAM_QLAB`, else `external/Q-LAB` in the folder that holds the lithocam checkout. Tests that need it skip when it is absent.

Code: `python/src/lithocam/robomove.py`. Tests: `python/tests/test_robomove.py`.

## What is in the clone

| Folder | Content |
|---|---|
| `KR480MT-D`, `KR480MT-M` | `EE_`, `RFCONF_` and (D only) `T_` files of the KR 480 cell `KR480_QUARRASTONE_01`. There is no tool table in `KR480MT-M`, and no meshes or cell files in either. |
| `KR500/CellKR500_QUARRASTONE_00` and `_Good` | A full ROBOmove cell: `T_`, `EE_`, `RFCONF_`, `TP_`, `L_`, `CM_` ini files, the `.dwp` scene, two cell `.dll` plug-ins, link meshes (`.iv`), table and rack meshes, a spindle mesh and `tcp_spindle.txt`, textures, `.3ds` and `.ase` basic objects, a wall `.stl`. |

## Readers

| Format | Reader | Result |
|---|---|---|
| `T_*.ini` | `read_tools` | `RoboTool` list |
| `EE_*.ini` | `read_end_effectors` | `EndEffector` list |
| `RFCONF_*.ini` | `read_rfconf` | `TableFrame` per table |
| `TP_*.ini` | `read_tp` | ranges per group |
| `L_*.ini` | `read_lang` | KRL program fragments and the APT word templates |
| `CM_*.ini` | `read_cm` | collision models |
| `tcp_spindle.txt` | `read_tcp_txt` | key and value |
| `.iv` (Open Inventor 1.0 ASCII) | `read_iv` | triangle meshes in metres, with colour |
| `.stl`, `.ase`, `.3ds` | `read_stl`, `read_ase`, `read_3ds` | triangle meshes |
| `.tga`, `.bmp` | `read_image_info` | size and bit depth (pixels not decoded; one `.bmp` has an OS/2 core header) |
| `.dwp` | `read_dwp` | partial: referenced mesh files and object names. The stored poses are not decoded. |
| `.dll` | `read_dll_info` | partial: PE header and strings. The code is not read. |

`inventory()` lists every file with its status. Every file is parsed except the `.dwp` and `.dll` files, which are partial.

The ini files look like XML but use numbers as tag names, so they are read with a small tolerant tag parser. A table's declared `<SZ>` is not trusted: `T_KR480_QUARRASTONE_01.ini` in `KR480MT-D` declares 20 tools and holds 21.

## The tool VALUES vector

Each tool has ten numbers. No file names them. The roles below come from the tool names and from patterns across all four tool tables.

| Field | Role used | Evidence | Status |
|---|---|---|---|
| `TOOL_TYPE` | 0 milling, 1 round or tapered nose, 2 disc | every `DISK` tool is type 2; every `BALL` tool is type 1; flat and granite cutters are type 0 | evidence |
| `v0` | cutter diameter (milling) | equals the number in the name (`85mm`, `50mm`, `20mm BALL`, `23mm GRANITE`; `6mm FLAT GRANITE` is 6.5) | evidence |
| `v0` on discs | arbor or hub diameter | 60 on every disc | inferred |
| `v1` | holder or flange diameter | 180 on every disc; on milling tools larger than `v0` in the older tables | inferred |
| `v2` on discs | disc diameter | 642, 632, 830, 700 for `640mm`, `632mm`, `840mm`, `700mm DISK`; 639 for the "642 mm disc" | evidence |
| `v2` on milling tools | unlabelled | equals `v0` or `v1` in the newer tables; 200 or 252 in the older | unlabelled |
| `v3`, `v4`, `v5` | unlabelled | pattern (5, 80, 5) or zeros | unlabelled |
| `v6` on milling tools | tool length from the end-effector TCP | the name carries it in `50/370` (370); the end effector TCP is the spindle nose and the length is measured from it | evidence |
| `v6` on discs | unlabelled | 115 to 124 | unlabelled |
| `v7` on discs | thickness, used as kerf | 10.7 or 8 on large discs, plausible for a segmented blade | inferred |
| `v7` on milling tools | unlabelled | 20, 10.7 or 0 | unlabelled |
| `v8` | unlabelled | 0, 5, 10 or 16 | unlabelled |
| `v9` | unlabelled | 0 in every tool of every file | unlabelled |
| `TCP_TYPE` | unlabelled | 1 mostly on ball tools | unlabelled |
| `SPINDLE_ASSOC` | none | -1 in every tool | unused |

The end-effector TCP is `x y z a b c` plus one flag. In every file only `b` (pitch) is non-zero (-90, or -90.02 in `KR480MT-M`), so the choice of rotation order does not matter. With that pitch the cutter axis is the flange x axis: the spindle is side-mounted. `lithocam.studio.spindles.Spindle.tcp_flange` represents this. The flag is unlabelled.

## Mapping to lithocam tools

`to_lithocam_tools(tools, prefix, source)` returns the lithocam tools and one `ToolMapping` per input with the unused values and the judgement calls.

| ROBOmove | lithocam |
|---|---|
| name has DISK, DISC, BLADE or SAW, or `TOOL_TYPE` 2 | `DiscSaw`: `diameter` = v2, `bore_diameter` = v0, `holder_diameter` = v1, `kerf` = v7 |
| name has BALL | `BallEndMill`: `diameter` = v0, `corner_radius` = v0 / 2 |
| `TOOL_TYPE` 1 with no shape in the name | `BallEndMill`, noted |
| name has TORIC, CONIC or TAPER | `FlatEndMill` of the same diameter (corner radius and taper angle are not in the file; lithocam rejects the bull-nose and tapered kinds without them; a flat model does not gouge) |
| anything else | `FlatEndMill` |
| milling tools | `stickout` = v6, `holder_diameter` = v1 when larger than v0 |
| name has PCD or CARBIDE | `material` |
| zero-size `NONE` entry | skipped |

`id` is the prefix plus the ROBOmove `ID`. The `ID` repeats within a table, so a repeat gets a suffix (`.2`). `source` is "Quarra Stone Q-LAB (ROBOmove backup)".

### Fields not mapped

- `v3`, `v4`, `v5`, `v8`, `v9` and `v7` on milling tools: unlabelled.
- `v2` on milling tools, `v6` on discs: unlabelled.
- `TCP_TYPE` when 1, `SPINDLE_ASSOC` when not -1: no lithocam field and no known meaning.
- Corner radius of toric tools and taper of conic tools: not in the file.
- Flutes, flute length, material of most tools, cutting data: not in the file.

### Stale records found

- Name and `v0` disagree for `1mm CARBIDE` (10), `2mm PCD` (5) and `2mm CARBIDE` (50).
- `6mm CARBIDE` and `14mm Ball` carry disc-like `v1`, `v2` and `v7`.
- `630mm BLADE` is type 0 but has a disc diameter in `v2`; it is mapped as a disc.
- The KR500 `_Good` table has two tools named `50/370`, one milling and one disc.

## Cells

`cells/make_cells.py` now writes `kr480_r3330_mt` and `kr500_r2830_mt`. Dimensions are from KUKA's public KR 500 FORTEC operating instructions (working-envelope figures). Each number and its source is in the comment above `KR_FORTEC`. Both are registered in `studio/program.py` with the KRL post, and a test checks that the FK reach at the stretched pose matches the datasheet reach within 2 percent.

The datasheet dimensions are for the FORTEC arms. The MT variants are not drawn in that manual, so the same arm dimensions are used. The sign of the 55 mm wrist offset is not visible in the drawing and is taken from the Q-LAB KR500 link meshes.

Findings from the Q-LAB cell:

- The ROBOmove KR500 scene (`.dwp`, scanned for numbers only) uses a 1.525 m wrist link, the R3330 arm, not the 1.025 m of the R2830. The folder name says KR500.
- The cell's `TP_` joint ranges (A2 -125 to 65, A3 -64 to 220, A5 +-130) differ from the KUKA datasheet ranges the cells use (A2 -130 to 20, A3 -100 to 144, A5 +-120). They are ROBOmove tool-path limits, not necessarily the robot's.
- The `KR480MT-D` and `KR480MT-M` folders differ in the spindle TCP by under 2 mm and in the rotating-table frame by several hundred mm.

## Spindle presets

`spindles.register_qlab()` adds `qlab_kr480mt_d`, `qlab_kr480mt_m` and `qlab_kr500` from each cell's spindle end effector, with the source credit. Body size, collet and speeds are not stated in the files and stay unset. The KRL words of the cell program (`SpindleSet`, `LoadTool`, `Coolant`) are noted in `control`.

## Not parsed

- `.dwp`: poses and the scene tree are binary and undocumented here; names and file references only.
- `.dll`: native cell plug-ins; header and strings only.
- Texture pixels (`.tga`, `.bmp`) are not decoded.
- The link meshes are in the ROBOmove home pose. They are read but not yet posed onto the URDF links, so the studio still draws the primitive link shapes.
