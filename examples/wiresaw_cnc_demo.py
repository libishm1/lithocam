# Copyright 2026 The lithocam authors. SPDX-License-Identifier: Apache-2.0
"""G-code and DXF for CNC diamond wire saws: a slab profile (xy), an indexed column and a turned baluster (xyc), and
the faces of a twisted column as ruled surfaces (xyuv).

Usage: python examples/wiresaw_cnc_demo.py OUTDIR
Machine definitions are the placeholders in examples/wiresaw_machines; replace them with the real machine values.
Writes OUTDIR/<program>_<n>.nc and OUTDIR/<program>.dxf and prints a summary. Needs numpy only.
"""
import dataclasses
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[0] / "python" / "src"))
from lithocam import wiresaw_cnc as wc            # noqa: E402
from lithocam.wirepath import load_ruled_json     # noqa: E402

MACHINES = HERE / "wiresaw_machines"
TWISTED = HERE.parent / "python" / "tests" / "data" / "ruled_json" / "twisted_column.json"


def slab_profile(x0=300.0, y0=300.0, w=800.0, h=500.0, r=50.0) -> wc.Profile:
    S = wc.Seg
    return wc.Profile((x0 + r, y0), [
        S((x0 + w - r, y0)), S((x0 + w, y0 + r), (x0 + w - r, y0 + r), True), S((x0 + w, y0 + h - r)),
        S((x0 + w - r, y0 + h), (x0 + w - r, y0 + h - r), True), S((x0 + r, y0 + h)),
        S((x0, y0 + h - r), (x0 + r, y0 + h - r), True), S((x0, y0 + r)), S((x0 + r, y0), (x0 + r, y0 + r), True)],
        closed=True)


def column_side_profile(axis_x=1000.0) -> wc.Profile:
    """Tapered column with entasis, seen from one side: half width from the table axis, height up."""
    return wc.Profile.from_bulge([(axis_x + 160, 300), (axis_x + 150, 700), (axis_x + 125, 1100)], [0.02, 0.015])


def baluster_profile() -> wc.Profile:
    """Turned profile (s, z): radius from the table axis and height, bottom to top."""
    pts = [(70, 300), (70, 340), (45, 400), (45, 470), (68, 560), (68, 640), (40, 720), (40, 800), (55, 860), (30, 900)]
    return wc.Profile.from_bulge(pts, [0, 0.25, 0, 0.3, 0, -0.3, 0, 0.2, 0])


def report(label, files, plan, outdir: Path, dxf: str, stem: str):
    for k, text in enumerate(files, start=1):
        (outdir / f"{stem}_{k}.nc").write_text(text, encoding="utf-8")
    (outdir / f"{stem}.dxf").write_text(dxf, encoding="utf-8")
    s = plan.summary()
    lines = sum(len(t.splitlines()) for t in files)
    print(f"{label:34s} layout {s['layout']:5s} kerf {s['kerf_mode']:4s} files {len(files)} lines {lines:5d} "
          f"cut length {s['path_length_mm']:9.1f} mm  est. cut time {s['cut_minutes']:7.1f} min")


def run(job: wc.WireJob, machine: wc.MachineDef, stem: str, outdir: Path, label: str):
    plan = wc.plan_job(job, machine)
    report(label, wc.gcode_files(plan), plan, outdir, wc.write_dxf(plan), stem)


def main(outdir: Path):
    outdir.mkdir(parents=True, exist_ok=True)
    xy = wc.MachineDef.load(MACHINES / "generic_xy.json")
    xyc = wc.MachineDef.load(MACHINES / "generic_xyc.json")
    xyuv = wc.MachineDef.load(MACHINES / "generic_xyuv.json")
    print("machine definitions are placeholders; feeds below are placeholders too\n")

    # 1. slab outline, xy: kerf offset in the exporter, and the same cut with G42 on the controller
    job = wc.WireJob("slab outline", [wc.Setup2D(slab_profile(), name="outline")], feed=100.0)
    run(job, xy, "slab_xy_cam", outdir, "slab outline (xy, CAM offset)")
    run(job, dataclasses.replace(xy, kerf_mode="g41", d_register=1), "slab_xy_g41", outdir, "slab outline (xy, G41/G42)")

    # 2. indexed 4-sided column: the same side profile at 0, 90, 180 and 270 degrees
    side = column_side_profile()
    job = wc.WireJob("column indexed", [wc.Setup2D(side, table_deg=a, waste="right", name=f"side{a}")
                                       for a in (0, 90, 180, 270)], feed=60.0)
    run(job, xyc, "column_indexed_xyc", outdir, "column, 4 sides indexed (xyc)")

    # 3. turned baluster: table turns 1 degree per mm of profile while the wire follows the profile
    job = wc.WireJob("baluster turned", [wc.CoordSetup(baluster_profile(), deg_per_mm=1.0, waste="+s", max_step_deg=5.0,
                                                       name="baluster")], feed=50.0)
    run(job, xyc, "baluster_turned_xyc", outdir, "baluster, coordinated turning (xyc)")

    # 4. twisted column side faces as ruled surfaces. Faces 0 and 2 have rulings along part y, faces 1 and 3 along part
    #    x, so the part is turned by 90 degrees between the two pairs (a re-clamp): four programs.
    rj = load_ruled_json(str(TWISTED))
    frames = {0: ("z", "y", "-x"), 2: ("z", "y", "-x"), 1: ("z", "x", "y"), 3: ("z", "x", "y")}
    for i in sorted(frames):
        st = wc.ruled_face_setup(rj.faces[i], xyuv, frames[i], offset=(200.0, 0.0, 1000.0), name=f"face{i}", feed=60.0)
        run(wc.WireJob(f"twisted column face {i}", [st]), xyuv, f"twisted_face{i}_xyuv", outdir,
            f"twisted column face {i} (xyuv)")
    print(f"\nwritten to {outdir}")


if __name__ == "__main__":
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    main(Path(sys.argv[1]))
