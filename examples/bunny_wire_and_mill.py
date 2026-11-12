# Copyright 2026 The lithocam authors. SPDX-License-Identifier: Apache-2.0
"""Stanford bunny in stone: diamond-wire cuts take a blank down to the bunny's convex hull, then 3+2 indexed milling
(turntable plus a tilted spindle) roughs with a flat cutter and finishes with two ball cutters.

    python examples/bunny_wire_and_mill.py PATH/bun_zipper.ply OUT_DIR [--height 200] [--run]

The bunny is not part of this repository. Download it from the Stanford 3D Scanning Repository
(http://graphics.stanford.edu/data/3Dscanrep/) and pass the path of bunny/reconstruction/bun_zipper.ply.

Writes to OUT_DIR:
  bunny.stl            the scaled bunny (z up, mm, sitting on z = 0)
  bunny_cuts.json      the planar wire cuts as ruled faces (the studio's wire step plans them)
  tools.csv            the cutters and the wire of this example (example sizes; see the names)
  bunny_ur10e.lcs.json, bunny_kr120.lcs.json   studio stations: program WireBlank, then Mill
  wire_stock.stl       the stock after the wire cuts (Manifold)
With --run it also expands both programs on both robots (Tesseract when installed) and writes the posted programs.

Open a station:  python -m lithocam.studio OUT_DIR/bunny_ur10e.lcs.json
"""
import argparse
import json
import math
import struct
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "python" / "src"))


def read_ply(path):
    """Vertices and triangles of an ASCII PLY (the Stanford scans)."""
    lines = Path(path).read_text().splitlines()
    nv = nf = 0
    k = 0
    while lines[k].strip() != "end_header":
        w = lines[k].split()
        if w[:2] == ["element", "vertex"]:
            nv = int(w[2])
        if w[:2] == ["element", "face"]:
            nf = int(w[2])
        k += 1
    k += 1
    V = np.array([[float(s) for s in lines[k + i].split()[:3]] for i in range(nv)])
    F = []
    for i in range(nf):
        w = [int(s) for s in lines[k + nv + i].split()]
        F += [(w[1], w[1 + j], w[2 + j]) for j in range(1, w[0] - 1)]
    return V, np.array(F)


def write_stl(path, tris):
    tris = np.asarray(tris, dtype=np.float32)
    n = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    n /= np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-12)
    with open(path, "wb") as f:
        f.write(b"lithocam".ljust(80, b" "))
        f.write(struct.pack("<I", len(tris)))
        for nn, t in zip(n, tris):
            f.write(struct.pack("<12fH", *nn, *t.reshape(-1), 0))


def cut_planes(hull_pts, blank_lo, blank_hi, clearance):
    """Planes tangent to the convex hull (plus clearance) that remove material from the blank: 8 around, 8 tilted
    upwards at 45 degrees, and the top. Each: (outward normal, offset)."""
    corners = np.array([[x, y, z] for x in (blank_lo[0], blank_hi[0]) for y in (blank_lo[1], blank_hi[1])
                        for z in (blank_lo[2], blank_hi[2])])
    normals = []
    for k in range(8):
        a = math.radians(45 * k)
        normals.append([math.cos(a), math.sin(a), 0.0])
    for k in range(8):
        a = math.radians(45 * k + 22.5)
        normals.append([math.cos(a) / math.sqrt(2), math.sin(a) / math.sqrt(2), 1 / math.sqrt(2)])
    normals.append([0.0, 0.0, 1.0])
    out = []
    for n in np.array(normals):
        h = float(np.max(hull_pts @ n)) + clearance
        if np.max(corners @ n) > h + 1.0:            # the plane cuts the blank
            out.append((n, h))
    return out


def plane_rulings(n, h, lo, hi, spacing, overhang):
    """Rulings of a planar wire cut across the blank box: the wire runs along the most horizontal direction of the
    plane, rulings step across the plane; endpoints overhang the blank's section."""
    d = np.cross(n, [0.0, 0.0, 1.0])
    if np.linalg.norm(d) < 1e-6:
        d = np.array([1.0, 0.0, 0.0])
    d /= np.linalg.norm(d)
    s = np.cross(n, d)
    # section of the box by the plane: intersect the 12 box edges
    c = np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])
    edges = [(0, 1), (2, 3), (4, 5), (6, 7), (0, 2), (1, 3), (4, 6), (5, 7), (0, 4), (1, 5), (2, 6), (3, 7)]
    pts = []
    for a, b in edges:
        fa, fb = c[a] @ n - h, c[b] @ n - h
        if fa * fb < 0:
            t = fa / (fa - fb)
            pts.append(c[a] + t * (c[b] - c[a]))
    if len(pts) < 3:
        return []
    pts = np.array(pts)
    o = n * h
    us, vs = (pts - o) @ d, (pts - o) @ s
    out = []
    for v in np.arange(vs.min() + spacing / 2, vs.max(), spacing):
        p0 = o + d * (us.min() - overhang) + s * v
        p1 = o + d * (us.max() + overhang) + s * v
        out.append({"p0": p0.tolist(), "p1": p1.tolist(), "normal": n.tolist()})
    return out


def silhouette(pts2d_tris, close_r, clearance):
    """Outline of a mesh projected onto a plane, as a manifold3d CrossSection: the union of the projected
    triangles (Clipper2), closed by close_r (notches narrower than 2 close_r are filled: a wire of radius below
    close_r can follow every concave corner left) and offset outwards by clearance."""
    import manifold3d as mf
    polys = []
    for t in pts2d_tris:
        a = (t[1, 0] - t[0, 0]) * (t[2, 1] - t[0, 1]) - (t[1, 1] - t[0, 1]) * (t[2, 0] - t[0, 0])
        if abs(a) > 1e-9:
            polys.append((t if a > 0 else t[::-1]).tolist())
    cs = mf.CrossSection(polys, mf.FillRule.NonZero)
    cs = cs.offset(close_r, mf.JoinType.Round, 2.0, 64).offset(-close_r, mf.JoinType.Round, 2.0, 64)
    return cs.offset(clearance, mf.JoinType.Round, 2.0, 64).simplify(0.05)


def open_profile(cs, v_floor, spacing):
    """The outline's largest contour without the part below v_floor (the table side), resampled every `spacing`
    mm: points (u, v) and outward unit normals, from one end on the table, over the top, to the other end."""
    loop = max(cs.to_polygons(), key=lambda p: abs(_area(np.asarray(p))))
    P = np.asarray(loop, dtype=float)
    if _area(P) < 0:
        P = P[::-1]                                       # counter-clockwise: outward normal = (dy, -dx)
    low = P[:, 1] < v_floor
    if not low.any():
        raise ValueError("the outline never reaches the table side")
    k = int(np.argmax(low))                               # rotate so the run above the floor is contiguous
    while low[k]:
        k = (k + 1) % len(P)
    P, low = np.roll(P, -k, axis=0), np.roll(low, -k)
    P = P[:int(np.argmax(low))]
    seg = np.linalg.norm(np.diff(P, axis=0), axis=1)
    s = np.r_[0.0, np.cumsum(seg)]
    t = np.linspace(0, s[-1], max(3, int(s[-1] / spacing) + 1))
    Q = np.c_[np.interp(t, s, P[:, 0]), np.interp(t, s, P[:, 1])]
    d = np.gradient(Q, axis=0)
    N = np.c_[d[:, 1], -d[:, 0]]
    return Q, N / np.linalg.norm(N, axis=1, keepdims=True)


def _area(P):
    return 0.5 * float(np.sum(P[:, 0] * np.roll(P[:, 1], -1) - np.roll(P[:, 0], -1) * P[:, 1]))


def profile_face(index, Q, N, view, lo, hi, overhang, lead, spacing, yaw_deg):
    """Ruled face of a profile cut: one wire line per outline point, perpendicular to the view plane, with
    straight lead-in and lead-out rulings from outside the blank. view 'side': outline in (x, z), wire along y;
    'front': outline in (y, z), wire along x."""
    def ruling(u, v, nu, nv):
        if view == "side":
            return {"p0": [u, lo[1] - overhang, v], "p1": [u, hi[1] + overhang, v], "normal": [nu, 0.0, nv]}
        return {"p0": [lo[0] - overhang, u, v], "p1": [hi[0] + overhang, u, v], "normal": [0.0, nu, nv]}
    rs = []
    for k in range(lead, 0, -1):                          # come in along the first point's normal
        u, v = Q[0] + N[0] * spacing * k
        rs.append(ruling(u, v, *N[0]))
    rs += [ruling(u, v, nu, nv) for (u, v), (nu, nv) in zip(Q, N)]
    for k in range(1, lead + 1):                          # leave along the last point's normal
        u, v = Q[-1] + N[-1] * spacing * k
        rs.append(ruling(u, v, *N[-1]))
    return {"index": index, "label": "wire", "ordered": True, "yaw_deg": yaw_deg, "rulings": rs}


def extrude_outline(cs, view, length):
    """The outline extruded through the part (a prism along the wire direction), as a Manifold."""
    m = cs.extrude(length).translate((0.0, 0.0, -length / 2))      # (u, v, w), w along the wire
    if view == "side":   # (u, v, w) -> (x, y, z) = (u, -w, v)
        M = [[1, 0, 0, 0], [0, 0, -1, 0], [0, 1, 0, 0]]
    else:                # (u, v, w) -> (x, y, z) = (w, u, v)
        M = [[0, 0, 1, 0], [1, 0, 0, 0], [0, 1, 0, 0]]
    return m.transform(M)


TOOLS_CSV = """# bunny example tools: example sizes (not from a datasheet); cutting data left unset
id,name,kind,material,bond,diameter,flute_length,shank_diameter,stickout,source
WIRE7,diamond wire 7.2 mm beads (example),diamond_wire,diamond,sintered,7.2,0,0,0,docs/TOOLING.md S18 (bead size)
R12,diamond finger bit D12 roughing (example size),flat_end_mill,diamond,sintered,12,30,12,45,example
B8,diamond ball nose D8 finishing (example size),ball_end_mill,diamond,electroplated,8,20,8,40,example
B4,diamond ball nose D4 detail (example size),ball_end_mill,diamond,electroplated,4,12,6,32,example
P100,resin polishing pads 50-3000 grit (example size),polishing_pad,diamond,resin,100,0,0,0,docs/TOOLING.md S8
"""


def station(robot, work, home, stock):
    setups = [(0, 0), (0, 40), (90, 40), (180, 40), (270, 40)]
    mill = [{"kind": "comment", "params": {"text": "3+2 indexed milling: rough from the top, finish from 5 setups"},
             "label": ""},
            {"kind": "movej", "params": {"joints": home, "speed": 50.0}, "label": "Home"},
            {"kind": "setup", "params": {"stock_margin": 0.0, "yaw": 0.0, "tilt": 0.0}, "label": "Setup top"},
            {"kind": "rough_plane", "params": {"tool": "R12", "stepdown": 8.0, "stepover": 7.0, "allowance": 1.5,
                                               "clearance": 15.0}, "label": "Rough R12"}]
    for yaw, tilt in setups:
        if tilt:
            mill.append({"kind": "setup", "params": {"stock_margin": 0.0, "yaw": float(yaw), "tilt": float(tilt)},
                         "label": "Setup table {} deg, tilt {} deg".format(yaw, tilt)})
            mill.append({"kind": "rough_plane", "params": {"tool": "R12", "stepdown": 8.0, "stepover": 7.0,
                                                           "allowance": 1.5, "clearance": 15.0},
                         "label": "Rough R12 (table {})".format(yaw)})
    for yaw, tilt in setups:
        mill.append({"kind": "setup", "params": {"stock_margin": 0.0, "yaw": float(yaw), "tilt": float(tilt)},
                     "label": "Setup table {} deg, tilt {} deg".format(yaw, tilt)})
        mill.append({"kind": "raster", "params": {"tool": "B8", "stepover": 2.5, "allowance": 0.3, "clearance": 15.0},
                     "label": "Semi-finish B8 (table {}, tilt {})".format(yaw, tilt)})
    mill.append({"kind": "setup", "params": {"stock_margin": 0.0, "yaw": 0.0, "tilt": 0.0}, "label": "Setup top"})
    mill.append({"kind": "raster", "params": {"tool": "B4", "stepover": 1.2, "clearance": 15.0},
                 "label": "Finish B4 (top)"})
    mill.append({"kind": "rest", "params": {"tool": "B4"}, "label": "Rest machining B4 (planned)"})
    mill.append({"kind": "polish", "params": {"tool": "P100"}, "label": "Polish 50-3000 grit (planned)"})
    mill.append({"kind": "movej", "params": {"joints": home, "speed": 50.0}, "label": "Home"})
    wire = [{"kind": "comment", "params": {"text": "Wire the blank down to the bunny's convex hull"}, "label": ""},
            {"kind": "set_tool", "params": {"tool": "WIRE7"}, "label": ""},
            {"kind": "movej", "params": {"joints": home, "speed": 50.0}, "label": "Home"},
            {"kind": "setup", "params": {"stock_margin": 0.0, "yaw": 0.0, "tilt": 0.0}, "label": "Turntable setup"},
            {"kind": "wire_ruled", "params": {"faces": "all", "max_step_deg": 30.0}, "label": "Wire cuts"},
            {"kind": "movej", "params": {"joints": home, "speed": 50.0}, "label": "Home"}]
    return {"robot": robot, "work_xyz": work, "work_rpy": [0.0, 0.0, 0.0], "part_path": "bunny.stl", "stock": stock,
            "tool_library": "tools.csv", "tool_length": 0.0, "spindle": None, "ruled_path": "bunny_cuts.json",
            "turntable": {"diameter": 0.6, "height": 0.04}, "material": "marble", "robot_factor": 1.0,
            "programs": [{"name": "WireBlank", "steps": wire}, {"name": "Mill", "steps": mill}]}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("ply")
    ap.add_argument("out")
    ap.add_argument("--height", type=float, default=200.0, help="bunny height, mm")
    ap.add_argument("--run", action="store_true", help="expand and post both programs on both robots")
    ap.add_argument("--cuts", choices=["twoview", "hull"], default="twoview",
                    help="twoview: side and front silhouette profiles (default); hull: planes on the convex hull")
    a = ap.parse_args()
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)

    V, F = read_ply(a.ply)
    V = V[:, [0, 2, 1]] * np.array([1.0, -1.0, 1.0])     # y-up scan -> z up
    V *= a.height / (V[:, 2].max() - V[:, 2].min())
    V -= [(V[:, 0].max() + V[:, 0].min()) / 2, (V[:, 1].max() + V[:, 1].min()) / 2, V[:, 2].min()]
    tris = V[F]
    write_stl(out / "bunny.stl", tris)
    lo, hi = V.min(0), V.max(0)
    margin = 12.0
    blank_lo, blank_hi = lo - [margin, margin, 0.0], hi + [margin, margin, margin]
    print("bunny {:.0f} x {:.0f} x {:.0f} mm, {} triangles; blank {:.0f} x {:.0f} x {:.0f} mm".format(
        *(hi - lo), len(tris), *(blank_hi - blank_lo)))

    import manifold3d as mf
    hull = mf.Manifold.hull_points(V.tolist())
    hull_pts = np.asarray(hull.to_mesh().vert_properties)[:, :3]
    blank = mf.Manifold.cube(tuple(blank_hi - blank_lo)).translate(tuple(blank_lo))
    cells = {"ur10e": ([0.55, -0.25, 0.0], [0.0, -90.0, 90.0, -90.0, -90.0, 0.0]),
             "kr120": ([1.45, -0.45, 0.0], [0.0, -90.0, 90.0, 0.0, 28.6, 0.0])}
    stock = blank
    if a.cuts == "hull":
        planes = cut_planes(hull_pts, blank_lo, blank_hi, clearance=3.0)
        faces = []
        for k, (n, h) in enumerate(planes):
            rs = plane_rulings(n, h, blank_lo, blank_hi, spacing=18.0, overhang=15.0)
            if len(rs) >= 2:
                faces.append({"index": k, "label": "wire", "rulings": rs})
        for name in cells:
            (out / "bunny_cuts_{}.json".format(name)).write_text(json.dumps(
                {"faces": faces, "rules": {"source": "planar cuts tangent to the convex hull + 3 mm"}, "scale": 1.0},
                indent=1))
        big = 2000.0
        for n, h in planes:   # keep the side of each plane that holds the bunny
            half = mf.Manifold.cube((big, big, big)).translate((-big / 2, -big / 2, -big))   # z <= 0
            half = _rotate_to(half, n)                                                         # n . x <= 0
            stock = stock ^ half.translate(tuple((n * h).tolist()))                            # n . x <= h
    else:   # two-view: side and front silhouettes, each an open profile from the table over the top
        rw = 3.6                                           # wire radius (7.2 mm beads)
        side = silhouette(tris[:, :, [0, 2]], close_r=rw + 2.0, clearance=3.0)
        front = silhouette(tris[:, :, [1, 2]], close_r=rw + 2.0, clearance=3.0)
        Qs, Ns = open_profile(side, v_floor=2.0, spacing=6.0)
        Qf, Nf = open_profile(front, v_floor=2.0, spacing=6.0)
        for name, (work, _) in cells.items():
            phi = math.degrees(math.atan2(-work[1], -work[0]))   # direction from the part to the robot base
            faces = [profile_face(0, Qs, Ns, "side", blank_lo, blank_hi, 15.0, 4, 6.0, phi),
                     profile_face(1, Qf, Nf, "front", blank_lo, blank_hi, 15.0, 4, 6.0, phi - 90.0)]
            (out / "bunny_cuts_{}.json".format(name)).write_text(json.dumps(
                {"faces": faces, "rules": {"source": "two-view silhouette profiles, closed by {:.1f} mm, +3 mm "
                                                     "clearance".format(rw + 2.0)}, "scale": 1.0}, indent=1))
        span = float(np.max(blank_hi - blank_lo)) * 3
        stock = stock ^ extrude_outline(side, "side", span) ^ extrude_outline(front, "front", span)
        faces = [None, None]
        print("side profile {} points, front profile {} points (6 mm spacing)".format(len(Qs), len(Qf)))
    sv = stock.to_mesh()
    write_stl(out / "wire_stock.stl", np.asarray(sv.vert_properties)[:, :3][np.asarray(sv.tri_verts)])
    print("{} cuts: blank {:.2f} dm3 -> after wire {:.2f} dm3 (convex hull {:.2f} dm3)".format(
        a.cuts, blank.volume() / 1e6, stock.volume() / 1e6, hull.volume() / 1e6))
    (out / "tools.csv").write_text(TOOLS_CSV)
    s_lo, s_hi = np.asarray(sv.vert_properties)[:, :3].min(0), np.asarray(sv.vert_properties)[:, :3].max(0)
    stock_box = {"xmin": s_lo[0], "xmax": s_hi[0], "ymin": s_lo[1], "ymax": s_hi[1], "zmin": 0.0, "zmax": s_hi[2]}
    stock_box = {k: float(v) for k, v in stock_box.items()}
    for name, (work, home) in cells.items():
        st = station(name, work, home, stock_box)
        st["ruled_path"] = "bunny_cuts_{}.json".format(name)
        (out / "bunny_{}.lcs.json".format(name)).write_text(json.dumps(st, indent=1))
    print("wrote", ", ".join(sorted(p.name for p in out.iterdir())))
    if a.run:
        run(out)


def _rotate_to(m, n):
    """Rotate a Manifold so its +z axis points along n (rotation as Euler XYZ degrees via a matrix)."""
    z = np.array([0.0, 0.0, 1.0])
    n = np.asarray(n, float) / np.linalg.norm(n)
    v = np.cross(z, n)
    c = float(z @ n)
    if np.linalg.norm(v) < 1e-12:
        R = np.eye(3) if c > 0 else np.diag([1.0, -1.0, -1.0])
    else:
        K = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
        R = np.eye(3) + K + K @ K * (1 / (1 + c))
    M = np.zeros((3, 4))
    M[:, :3] = R
    return m.transform(M.tolist())


def run(out):
    from lithocam import post
    from lithocam.studio import cam, planner, program
    from lithocam.studio.core import lc
    for name in ("ur10e", "kr120"):
        st = program.Station.load(out / "bunny_{}.lcs.json".format(name))
        tools = {t.id: t for t in lc.read_tool_library(st.tool_library).tools}
        tris = cam.load_stl(st.part_path)
        part = (tris, cam.trimesh(tris))
        for prog in st.programs:
            robot = program.load_robot(name)
            be = planner.make_backend(st, program.CELLS)
            ex = program.expand(st, robot, prog, tools, part, backend=be)
            dialect = program.ROBOTS[name][2]
            text = post.write(ex.records, dialect, **program.post_options(name, dialect))
            fn = out / "{}_{}{}".format(prog.name, name, post.EXTENSIONS[dialect])
            fn.write_text(text)
            print("{} {}: {} samples, {} not converged or colliding; posted {} lines to {}".format(
                name, prog.name, len(ex.joints()), ex.unreachable, text.count("\n"), fn.name))
            for m in ex.messages:
                print("   ", m)


if __name__ == "__main__":
    main()
