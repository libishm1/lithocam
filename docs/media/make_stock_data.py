# Copyright 2026 The lithocam authors. SPDX-License-Identifier: Apache-2.0
"""Stock-removal meshes for the docs GIFs (run with a Python that has OCP + manifold3d), after make_robot_data.py:

    python docs/media/make_stock_data.py <data dir>

Wire sweeps: for every robot frame (robot_<name>_column.json), the stock is the blank (the twisted column's
bounding box, i.e. after the planar top and bottom cuts) minus every wire position swept so far, in the robot's own
order: the convex hull of the wire cylinders at consecutive positions (lithocam.meshsim). Each face gets a lead-in
and a lead-out one ruling step beyond its ends; when a face is finished its waste piece falls away (the piece that
holds the part is kept). Writes stock_<name>_column.json: one triangle mesh (mm, part frame) per frame.

Flank roughing: the stock after each roughing layer and the finishing pass, face by face, from the frames of
examples/flank_5axis.py (flank_frames.json). Writes flank_stock_layers.json.
"""
import json
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "python" / "src"))
import manifold3d as mf  # noqa: E402
from lithocam import meshsim, ruled  # noqa: E402
from lithocam.mesh import triangles_from_shape  # noqa: E402

DATA = Path(sys.argv[1])
STEP = ROOT / "python" / "tests" / "data" / "step_ruled" / "twisted_column.step"
WIRE_R = 3.5

shape = ruled.read_step(str(STEP))
target = meshsim.manifold_from_triangles(triangles_from_shape(shape, 0.5))
lo, hi = np.asarray(target.bounding_box()[:3]), np.asarray(target.bounding_box()[3:])
blank = mf.Manifold.cube(tuple(hi - lo)).translate(tuple(lo))


def tris_of(m):
    v, t = meshsim.to_arrays(m)
    return np.round(v[t], 2).tolist()


def wire_line(T_world_m, half_m, place, yaw):
    """Wire end points in the part frame (mm) from a tool0 pose (world, metres) and the table yaw."""
    T = np.asarray(T_world_m)
    x, z = T[:3, 0], T[:3, 2]
    return T, x, z


for name in ("ur10e", "kr120"):
    path = DATA / "robot_{}_column.json".format(name)
    if not path.exists():
        continue
    r = json.loads(path.read_text())
    place = np.asarray(r["placement"])
    bow = 0.25 if name == "ur10e" else 0.6
    stock = blank
    out = []
    faces = []
    for fr in r["frames"]:
        if not faces or faces[-1][0] != fr["face"]:
            faces.append((fr["face"], []))
        faces[-1][1].append(fr)
    for face, frs in faces:
        yaw = frs[0]["yaw"]
        c, s = math.cos(-yaw), math.sin(-yaw)
        Rinv = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])
        lines = []
        for fr in frs:
            T = np.asarray(fr["T"])
            mid = T[:3, 3] + T[:3, 2] * bow                         # wire midpoint (world, m)
            d = T[:3, 0]
            m_part = Rinv @ ((mid - place) * 1e3)                   # part frame, mm
            d_part = Rinv @ d
            lines.append((m_part, d_part))
        # lead-in / lead-out one step beyond the ends
        if len(lines) > 1:
            lines = [(2 * lines[0][0] - lines[1][0], lines[0][1])] + lines + \
                    [(2 * lines[-1][0] - lines[-2][0], lines[-1][1])]
        SUB = 6                                                    # interpolated wire positions per robot step

        def cyl(m, d):                                             # wire just past the blank on both ends
            d = d / np.linalg.norm(d)
            ea = meshsim._exit(m, -d, lo, hi) + 5.0
            eb = meshsim._exit(m, d, lo, hi) + 5.0
            return meshsim.wire_cylinder(m - d * ea, m + d * eb, WIRE_R, 24)

        def step_hulls(a, b):
            seq = [((1 - t) * a[0] + t * b[0], (1 - t) * a[1] + t * b[1]) for t in np.linspace(0, 1, SUB + 1)]
            cs = [cyl(m, d) for m, d in seq]
            return [mf.Manifold.batch_hull([cs[i], cs[i + 1]]) for i in range(SUB)]

        removed = []
        for k, fr in enumerate(frs):
            removed += step_hulls(lines[k], lines[k + 1])          # robot frame k ends at line k + 1
            if k == len(frs) - 1:                                  # finish the face: lead-out, waste falls away
                removed += step_hulls(lines[-2], lines[-1])
            cut = stock - mf.Manifold.batch_boolean(removed, mf.OpType.Add)
            if k == len(frs) - 1:
                cut = max(cut.decompose(), key=lambda p: (p ^ target).volume())
            out.append(tris_of(cut))
        stock = cut
    (DATA / "stock_{}_column.json".format(name)).write_text(json.dumps(out))
    print(name, "frames", len(out), "final volume {:.0f} (part {:.0f})".format(stock.volume(), target.volume()))

# flank roughing layers
fpath = DATA.parent / "flank_frames.json"
alt = Path("D:/code_ws/outputs/2026-11-12/lithocam_examples/flank/flank_frames.json")
fpath = fpath if fpath.exists() else alt
if fpath.exists():
    from lithocam import flank
    frames = json.loads(fpath.read_text())
    stock = blank
    snaps = [tris_of(stock)]
    for face in sorted({f["face"] for f in frames}):
        bands = {}
        for f in frames:
            if f["face"] == face:
                bands.setdefault(f["band"], []).append(flank.FlankFrame(np.asarray(f["tip"]), np.asarray(f["axis"]),
                                                                        f["band"], 0))
        # group bands into layers of equal radial offset: bands are numbered layer by layer
        per_layer = max(1, len(bands) // 8)
        keys = sorted(bands)
        for i in range(0, len(keys), per_layer):
            fr = [x for b in keys[i:i + per_layer] for x in bands[b]]
            stock, removed, gouge = flank.simulate(stock, target, fr, 10.0, 120.0, segments=24)
            snaps.append(tris_of(stock))
        print("flank face", face, "volume {:.0f}".format(stock.volume()))
    (DATA / "flank_stock_layers.json").write_text(json.dumps(snaps))
    print("flank snapshots", len(snaps))
