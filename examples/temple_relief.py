# Copyright 2026 The lithocam authors. SPDX-License-Identifier: Apache-2.0
"""A Thanjavur temple pillar relief milled from a granite slab: the square four-petal padma (lotus) panel.

    python examples/temple_relief.py [--simulate] [--stock] [--quick] [--robots kr120,ur10e] [--stride N]
    python examples/temple_relief.py --build MESH.stl      (rebuild examples/data/padma_relief.npz)

The part is examples/data/padma_relief.npz, a 512 x 512 height field (uint16) of the panel: relief reconstructed by
Libish Murugesan with Depth Anything 3 from a photograph, Thanjavur (source repo
github.com/libishm1/Depth_Anything_3_Motifs). It ships under Apache-2.0 as the owner's own data. The full
reconstruction mesh stays with the owner; --build documents how the height field was made from it.

The height field is rescaled to a 600 x 600 mm panel with the relief normalised to 25 mm deep. The reconstruction's
own z scale is not a measured depth (monocular depth is relative), so 25 mm is a design choice, not a survey. The
part is a closed slab: the relief on a 35 mm base, 60 mm thick (lithocam.studio.cam.heightfield_tris).

Writes the studio stations examples/studio/temple_relief_<robot>.lcs.json (program "Relief"): face the sawn top, rough
in z-levels with a flat diamond finger bit, steep / shallow semi-finish with an 8 mm ball, 5-axis surfacing with a
6 mm ball (lead 15 deg). Feeds and spindle speeds on every cutting step are example values, set explicitly.
--simulate expands each program (numpy IK) and prints the summary per robot; --stock also cuts the SDF stock along
the expansion (lithocam.studio.process.simulate_stock) and reports the removed volume. Timings are printed.

Open a station:  python -m lithocam.studio examples/studio/temple_relief_kr120.lcs.json
"""
import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "python" / "src"))

NPZ = HERE / "data" / "padma_relief.npz"
TOOLS = HERE / "data" / "temple_relief_tools.csv"
STUDIO = HERE / "studio"

CREDIT = ("relief reconstructed by Libish Murugesan with Depth Anything 3 from a photograph, Thanjavur; "
          "source repo github.com/libishm1/Depth_Anything_3_Motifs")
WIDTH_MM, RELIEF_MM, BASE_MM, PX = 600.0, 25.0, 35.0, 512


# ---------------------------------------------------------------- height field from the reconstruction mesh

def _plane(x, y, z, keep=60.0, iters=6):
    """z = a x + b y + c fitted to the lower `keep` percent of residuals, repeated: the ground under a relief (the
    raised carving is the upper tail and drops out)."""
    A = np.c_[x, y, np.ones(len(x))]
    w = np.ones(len(z), bool)
    for _ in range(iters):
        c, *_ = np.linalg.lstsq(A[w], z[w], rcond=None)
        r = z - A @ c
        w = r <= np.percentile(r[w], keep)
    return c


def _rot_to_z(n):
    """Smallest rotation taking the unit vector n to +z (Rodrigues)."""
    n = np.asarray(n, float) / np.linalg.norm(n)
    v, c = np.cross(n, [0.0, 0.0, 1.0]), float(n[2])
    K = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + K + K @ K / (1 + c)


def raster_max(T, x0, y0, cell, nx, ny, chunk=400000):
    """Highest z per grid cell over the triangles T (n, 3, 3): each triangle is sampled on a barycentric lattice
    with spacing at most half a cell, so every cell a triangle covers gets a sample. Cells no triangle covers stay
    -inf. Vectorised per lattice size, in chunks."""
    H = np.full(ny * nx, -np.inf)
    e = np.max(np.linalg.norm(T[:, [1, 2, 0], :2] - T[:, :, :2], axis=2), axis=1)
    level = np.clip(np.ceil(e / (0.5 * cell)), 1, 16).astype(int)
    for n in np.unique(level):
        u, v = np.meshgrid(np.arange(n + 1), np.arange(n + 1))
        m = (u + v) <= n
        W = np.c_[n - u[m] - v[m], u[m], v[m]] / n                 # (k, 3) barycentric weights
        idx = np.nonzero(level == n)[0]
        for s in range(0, len(idx), max(1, chunk // len(W))):
            t = T[idx[s:s + max(1, chunk // len(W))]]
            P = np.einsum("kj,tjd->tkd", W, t).reshape(-1, 3)
            i = np.floor((P[:, 0] - x0) / cell).astype(np.int64)
            j = np.floor((P[:, 1] - y0) / cell).astype(np.int64)
            ok = (i >= 0) & (i < nx) & (j >= 0) & (j < ny)
            np.maximum.at(H, j[ok] * nx + i[ok], P[ok, 2])
    return H.reshape(ny, nx)


def fill_nearest(H):
    """Replace -inf cells by the mean of their filled 8-neighbours, ring by ring (nearest valid value)."""
    H = H.copy()
    bad = ~np.isfinite(H)
    while bad.any():
        P = np.pad(np.where(bad, 0.0, H), 1)
        C = np.pad((~bad).astype(float), 1)
        s = np.zeros_like(H)
        c = np.zeros_like(H)
        ny, nx = H.shape
        for dj in (-1, 0, 1):
            for di in (-1, 0, 1):
                if dj or di:
                    s += P[1 + dj:1 + dj + ny, 1 + di:1 + di + nx]
                    c += C[1 + dj:1 + dj + ny, 1 + di:1 + di + nx]
        new = bad & (c > 0)
        if not new.any():
            raise ValueError("height field has no valid cells to fill from")
        H[new] = s[new] / c[new]
        bad &= ~new
    return H


# The panel square's frame edges in the levelled raster of motifs/thanjavur_meshes/1.stl, as fractions of the
# rotated mesh's xy box (x0, x1, y0, y1). Picked by eye on the levelled 518 x 602 raster (columns 57..470,
# rows 134..547): an automatic threshold mixes the panel's sunken field with the pillar face. Checked by overlaying
# the box on the raster.
PANEL_BOX = (57 / 517, 470 / 517, 134 / 601, 547 / 601)


def build_heightfield(stl, px=PX, width_mm=WIDTH_MM, relief_mm=RELIEF_MM, base_mm=BASE_MM, inset=0.02,
                      box=PANEL_BOX, out=NPZ, preview=None):
    """The shipped height field from the owner's reconstruction mesh:
      1. read the STL; fit the plane of the pillar face around the panel (robust, on a 600-cell raster) and rotate
         the mesh so that plane is horizontal;
      2. take the panel square `box` (fractions of the rotated xy box), the largest centred square inside it, inset
         by `inset` of its side;
      3. raster the top surface over the square at 2 * px cells (highest z per cell over the triangles), fill holes
         from the nearest valid cells, level it on the panel's rim (a robust plane through the band 6 % wide inside
         the square edge) and average 2 x 2 down to px x px;
      4. normalise the relief to 0..relief_mm (0.5 and 99.9 percentiles, clipped) and quantise to uint16.
    Row 0 is the bottom of the panel as photographed (y up)."""
    from lithocam.viz import read_stl
    t0 = time.perf_counter()
    T = np.asarray(read_stl(str(stl)), float).reshape(-1, 3, 3)
    V = T.reshape(-1, 3)
    lo, hi = V[:, :2].min(0), V[:, :2].max(0)
    n0 = 600
    cell0 = float((hi - lo).max()) / (n0 - 1)
    nx0, ny0 = [int(v) for v in np.ceil((hi - lo) / cell0) + 1]
    C0 = fill_nearest(raster_max(T, lo[0], lo[1], cell0, nx0, ny0))
    yy, xx = np.mgrid[0:ny0, 0:nx0]
    a, b, _ = _plane(xx.ravel() * cell0, yy.ravel() * cell0, C0.ravel())
    R = _rot_to_z([-a, -b, 1.0])
    T = (T.reshape(-1, 3) @ R.T).reshape(-1, 3, 3)
    V = T.reshape(-1, 3)
    lo, hi = V[:, :2].min(0), V[:, :2].max(0)
    bx0, bx1 = lo[0] + np.array(box[:2]) * (hi[0] - lo[0])
    by0, by1 = lo[1] + np.array(box[2:]) * (hi[1] - lo[1])
    side = min(bx1 - bx0, by1 - by0) * (1 - 2 * inset)
    cx, cy = 0.5 * (bx0 + bx1), 0.5 * (by0 + by1)
    n = 2 * px
    cell = side / n
    x0, y0 = cx - side / 2, cy - side / 2
    keep = ((T[:, :, 0].max(1) >= x0 - cell) & (T[:, :, 0].min(1) <= x0 + side + cell) &
            (T[:, :, 1].max(1) >= y0 - cell) & (T[:, :, 1].min(1) <= y0 + side + cell))
    raw = raster_max(T[keep], x0, y0, cell, n, n)
    holes = float(np.mean(~np.isfinite(raw)))
    F = fill_nearest(raw)
    yy, xx = np.mgrid[0:n, 0:n]
    rim = np.minimum(np.minimum(xx, n - 1 - xx), np.minimum(yy, n - 1 - yy)) < 0.06 * n
    pa, pb, pc = _plane(xx[rim] * cell, yy[rim] * cell, F[rim])
    L = F - (pa * xx * cell + pb * yy * cell + pc)
    L = L.reshape(px, 2, px, 2).mean(axis=(1, 3))
    zlo, zhi = np.percentile(L, [0.5, 99.9])
    h = np.clip((L - zlo) / (zhi - zlo), 0.0, 1.0)
    q = np.round(h * 65535).astype(np.uint16)
    tilt = float(np.degrees(np.arccos(1 / np.linalg.norm([a, b, 1.0]))))
    meta = dict(width_mm=float(width_mm), height_mm=float(width_mm), relief_mm=float(relief_mm),
                z_scale_mm=float(relief_mm) / 65535.0, base_mm=float(base_mm),
                source=("Depth Anything 3 reconstruction mesh {} ({} triangles); pillar plane tilt {:.2f} deg removed; "
                        "panel square {:.4f} units (mesh units) inset {:.1%}; raster {}^2 max-z per cell, {:.2%} "
                        "holes filled from the nearest cells; levelled on the rim band; 2 x 2 mean to {}^2; "
                        "relief normalised to 0..{:g} mm (0.5 / 99.9 percentiles), not a measured depth").format(
                            Path(stl).parent.name + "/" + Path(stl).name, len(V) // 3, tilt, side, inset, n, holes,
                            px, relief_mm),
                credit=CREDIT, license="Apache-2.0")
    Path(out).parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(str(out), heights=q, **{k: np.asarray(v) for k, v in meta.items()})
    if preview:
        np.save(str(preview), (h[::-1] * 255).astype(np.uint8))
    print("height field {} x {} from {} ({} triangles): tilt {:.2f} deg, holes {:.2%}, {:.1f} s -> {} ({:.0f} kB)"
          .format(px, px, Path(stl).name, len(V) // 3, tilt, holes, time.perf_counter() - t0, out,
                  Path(out).stat().st_size / 1024))
    return q, meta


# ---------------------------------------------------------------- tools and stations

# S1 is the stone library's finger bit (lc.stone_tool_library(), copied verbatim). The library has no ball-end mills
# and no flat bit whose shank fits the UR10e's ER20 collet (13 mm), so R12, B8 and B6 are example sizes, like the
# bunny example's. A station's tool library replaces the built-in ones, hence S1 is repeated here.
TOOLS_CSV = """# temple relief tools: S1 from the lithocam stone library; R12, B8, B6 example sizes (not from a datasheet)
id,name,kind,material,bond,grit,source,diameter,flute_length,shank_diameter,stickout
S1,"Diamond finger bit 3/4 in x 1-1/2 in, 50/60",flat_end_mill,diamond,,50/60,docs/TOOLING.md S1,19.05,38.1,0,0
R12,diamond finger bit D12 roughing (example size),flat_end_mill,diamond,sintered,,example,12,30,12,45
B8,diamond ball nose D8 semi-finishing (example size),ball_end_mill,diamond,electroplated,,example,8,20,8,40
B6,diamond ball nose D6 finishing (example size),ball_end_mill,diamond,electroplated,,example,6,15,6,35
"""

# per robot: work frame origin (m, robot base; the panel's corner x = y = 0, z = 0 on the slab bottom), home joints
# (deg), the flat tool (face + rough) and its stepover. Placed so the whole panel is reachable with the tool down
# and with the 5-axis tilts. KR120: with the panel at z = 0 the 5-axis pass flips the wrist (A5 through 0) and 444
# of 9139 points failed. Raised on a table the wrist stays bent; at [1.25, -0.3, 0.5] four 5-axis feed moves still
# turned A4 / A6 by 11-16 deg (wrist close to A5 = 0 near x = 0..50, y = 400 mm); at [1.1, -0.3, 0.7] none turn a
# joint more than 10 deg. UR10e: [0.4, -0.3, 0.0] had the same clean result (checked 2026-11-14 on a 20 mm stepover
# check path, stride 8).
CELLS = {"kr120": dict(work=[1.10, -0.30, 0.7], home=[0.0, -90.0, 90.0, 0.0, 28.6, 0.0], flat="S1",
                       rough_stepover=9.0, flat_feed=(1200.0, 6000.0), rough_feed=(1000.0, 6000.0)),
         "ur10e": dict(work=[0.40, -0.30, 0.0], home=[0.0, -90.0, 90.0, -90.0, -90.0, 0.0], flat="R12",
                       rough_stepover=6.0, flat_feed=(800.0, 8000.0), rough_feed=(600.0, 8000.0))}

STOCK_TOP = 3.0   # mm of sawn stock above the relief's highest point, removed by the face step


def station(robot, stock_top=STOCK_TOP):
    """Studio station dict for one robot: program "Relief"."""
    c = CELLS[robot]
    flat = c["flat"]
    top = BASE_MM + RELIEF_MM + stock_top
    steps = [
        {"kind": "comment", "params": {"text": "Padma relief, Thanjavur (Depth Anything 3 reconstruction by Libish "
                                               "Murugesan), 600 x 600 mm, relief 25 mm on a 35 mm base"}, "label": ""},
        {"kind": "comment", "params": {"text": "Feeds and spindle speeds on every cutting step are EXAMPLE values, "
                                               "not cutting data: set them for your tools, stone and robot"},
         "label": "Example feeds"},
        {"kind": "set_tool", "params": {"tool": flat}, "label": ""},
        {"kind": "movej", "params": {"joints": c["home"], "speed": 50.0}, "label": "Home"},
        {"kind": "setup", "params": {"stock_margin": 0.0, "yaw": 0.0, "tilt": 0.0}, "label": "Setup top"},
        {"kind": "face", "params": {"tool": flat, "depth": stock_top, "stepover": 0.0, "feed": c["flat_feed"][0],
                                    "rpm": c["flat_feed"][1], "clearance": 10.0},
         "label": "Face the sawn top {} (example feed)".format(flat)},
        {"kind": "rough_plane", "params": {"tool": flat, "stepdown": 4.0, "stepover": c["rough_stepover"],
                                           "allowance": 1.0, "feed": c["rough_feed"][0], "rpm": c["rough_feed"][1],
                                           "clearance": 10.0},
         "label": "Rough {} z-levels, 1 mm allowance (example feed)".format(flat)},
        {"kind": "steep_shallow", "params": {"tool": "B8", "threshold_deg": 40.0, "overlap": 1.0, "stepover": 2.0,
                                             "allowance": 0.2, "feed": 1500.0, "rpm": 10000.0, "clearance": 10.0},
         "label": "Semi-finish B8 steep / shallow, 0.2 mm allowance (example feed)"},
        {"kind": "surfacing5", "params": {"tool": "B6", "stepover": 1.5, "lead": 15.0, "max_tilt": 30.0,
                                          "tolerance": 0.02, "feed": 1200.0, "rpm": 12000.0, "clearance": 10.0},
         "label": "Finish B6 5-axis, lead 15 deg (example feed)"},
        {"kind": "movej", "params": {"joints": c["home"], "speed": 50.0}, "label": "Home"},
    ]
    return {"robot": robot, "work_xyz": c["work"], "work_rpy": [0.0, 0.0, 0.0],
            "part_path": "../data/padma_relief.npz",
            "stock": {"xmin": 0.0, "xmax": WIDTH_MM, "ymin": 0.0, "ymax": WIDTH_MM, "zmin": 0.0, "zmax": top},
            "tool_library": "../data/temple_relief_tools.csv", "tool_length": 0.0, "spindle": None,
            "material": "granite", "robot_factor": 1.0, "ruled_path": "", "turntable": None,
            "programs": [{"name": "Relief", "steps": steps}]}


def station_json(st):
    """The station as text in the layout of the other examples/studio files: one key per line, one step per line."""
    head = [' {}: {}'.format(json.dumps(k), json.dumps(v)) for k, v in st.items() if k != "programs"]
    progs = []
    for p in st["programs"]:
        steps = ",\n".join("    " + json.dumps(s) for s in p["steps"])
        progs.append('  {{\n   "name": {},\n   "steps": [\n{}\n   ]\n  }}'.format(json.dumps(p["name"]), steps))
    return "{\n" + ",\n".join(head) + ',\n "programs": [\n' + ",\n".join(progs) + "\n ]\n}\n"


def write_stations(robots, out=STUDIO):
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    TOOLS.write_text(TOOLS_CSV, encoding="utf-8")
    paths = []
    for r in robots:
        p = out / "temple_relief_{}.lcs.json".format(r)
        text = station_json(station(r))
        assert json.loads(text) == station(r)
        p.write_text(text, encoding="utf-8")
        paths.append(p)
    return paths


def describe_part(stride=1):
    from lithocam.studio import cam
    H, meta = cam.load_heightfield(NPZ)
    t0 = time.perf_counter()
    tris = cam.load_part(NPZ, stride=stride)
    lo, hi = tris.reshape(-1, 3).min(0), tris.reshape(-1, 3).max(0)
    print("part {}: {} x {} grid, {:.0f} x {:.0f} x {:.1f} mm, relief {:.2f} mm, {} triangles (stride {}, {:.2f} s)"
          .format(NPZ.name, H.shape[1], H.shape[0], *(hi - lo), H.max() - H.min(), len(tris), stride,
                  time.perf_counter() - t0))
    print("credit:", meta.get("credit", ""))


# --quick: the plan python/tests/test_heightfield.py checks. Same work frame, tools, steps and tool axes, on every
# 8th row and column of the height field with these stepovers (mm), so a robot expands in about two minutes. The
# full stations have some 300 000 robot points and take longer than 10 minutes per robot with numpy IK.
QUICK_STRIDE = 8
QUICK_STEPOVER = {"rough_plane": 40.0, "steep_shallow": 20.0, "surfacing5": 20.0}


def simulate(robots, stride=1, stock=False, cell_mm=2.0, quick=False):
    from lithocam.studio import api, process
    for r in robots:
        path = STUDIO / "temple_relief_{}.lcs.json".format(r)
        t0 = time.perf_counter()
        s = api.Session.open(path)
        if quick:
            stride = max(stride, QUICK_STRIDE)
            for name in s.programs():
                p = s.program(name)
                for i, st in enumerate(p.list()):
                    if st["kind"] in QUICK_STEPOVER:
                        p.set(i, stepover=QUICK_STEPOVER[st["kind"]])
        if stride != 1:
            s.load_part(s.station.part_path, keep_stock=True, stride=stride)
        t_load = time.perf_counter() - t0
        for name in s.programs():
            t1 = time.perf_counter()
            res = s.simulate(name)
            t_sim = time.perf_counter() - t1
            print("{} {}: {} (load {:.1f} s, expand {:.1f} s)".format(r, name, res.summary(), t_load, t_sim))
            for m in res.messages:
                print("   ", m)
            if stock:
                t2 = time.perf_counter()
                run = process.simulate_stock(s.station, s.robot, res.expansion, s._tool_map(), cell_mm=cell_mm,
                                             snapshots=2)
                print("{} {}: stock removed {:.2f} dm3 of {:.2f} dm3 ({} cuts, {:g} mm grid, {:.1f} s)".format(
                    r, name, run.total_removed / 1e6, run.initial_volume / 1e6, len(run.steps), cell_mm,
                    time.perf_counter() - t2))
                for m in run.messages:
                    print("   ", m)


def main():
    ap = argparse.ArgumentParser(description="Thanjavur padma relief: face, rough, semi-finish, 5-axis finish.")
    ap.add_argument("--build", metavar="MESH", help="rebuild examples/data/padma_relief.npz from the owner's mesh "
                                                   "(motifs/thanjavur_meshes/1.stl), then write the stations")
    ap.add_argument("--robots", default="kr120,ur10e", help="comma-separated: kr120, ur10e")
    ap.add_argument("--simulate", action="store_true", help="expand each station's program and print summaries")
    ap.add_argument("--stock", action="store_true", help="also cut the SDF stock and report the removed volume")
    ap.add_argument("--stride", type=int, default=1, help="plan on every N-th row and column of the height field")
    ap.add_argument("--cell", type=float, default=2.0, help="stock simulation grid, mm")
    ap.add_argument("--quick", action="store_true", help="simulate the coarse check plan (stride 8, wide stepovers; "
                                                         "about two minutes per robot) instead of the full stations")
    a = ap.parse_args()
    robots = [r.strip() for r in a.robots.split(",") if r.strip()]
    bad = [r for r in robots if r not in CELLS]
    if bad:
        ap.error("unknown robot(s): {}".format(", ".join(bad)))
    if a.build:
        build_heightfield(a.build)
    for p in write_stations(robots):
        print("wrote", p)
    describe_part(max(a.stride, QUICK_STRIDE) if a.quick else a.stride)
    if a.simulate or a.stock:
        simulate(robots, a.stride, a.stock, a.cell, a.quick)


if __name__ == "__main__":
    main()
