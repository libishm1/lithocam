# Copyright 2026 The lithocam authors. SPDX-License-Identifier: Apache-2.0
"""Example: 5-axis flank milling of the twisted column's ruled sides, simulated and planned for the KR120.

Needs two environments (the geometry one has OCP + manifold3d, the robot one has tesseract-robotics):

    python examples/flank_5axis.py geometry <out dir>      # CL frames + material-removal simulation
    python examples/flank_5axis.py robot <out dir>         # KR120 joint path (verified IK) + link transforms

Tool: cylindrical cutter R10, flute 120 mm, axis on each ruling offset by R; sides milled in bands of 120 mm along
the 300 mm rulings, zig-zag across the face. Stock: the column's bounding box after the top and bottom faces.
"""
import json
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "python" / "src"))
MODE, OUT = sys.argv[1], Path(sys.argv[2])
OUT.mkdir(parents=True, exist_ok=True)
STEP = ROOT / "python" / "tests" / "data" / "step_ruled" / "twisted_column.step"
R_TOOL, FLUTE = 10.0, 120.0

if MODE == "geometry":
    import manifold3d as mf
    from lithocam import flank, meshsim, ruled
    from lithocam.mesh import triangles_from_shape

    shape = ruled.read_step(str(STEP))
    rep = ruled.analyse_shape(shape, ruled.Rules(n_rulings=48, tool_radius=R_TOOL))
    target = meshsim.manifold_from_triangles(triangles_from_shape(shape, deflection=0.5))
    lo, hi = np.asarray(target.bounding_box()[:3]), np.asarray(target.bounding_box()[3:])
    stock = mf.Manifold.cube(tuple(hi - lo)).translate(tuple(lo))          # squared blank (wire-cut box)
    v0 = stock.volume()
    frames_out, stats = [], []
    t0 = time.time()
    for f in rep.faces:
        if f.surface == "plane" or not f.rulings:
            continue
        depth = flank.stock_depth(f.rulings, lo, hi)
        step = 0.8 * 2 * R_TOOL                                           # radial depth per roughing layer
        layers = [k * step for k in range(int(np.ceil(depth / step)), 0, -1)] + [0.0]
        fr = flank.flank_frames(f.rulings, R_TOOL, FLUTE, layers=layers)
        stock, removed, gouge = flank.simulate(stock, target, fr, R_TOOL, FLUTE)
        stats.append({"face": f.index, "label": f.label, "stock_depth": round(depth, 1), "layers": layers,
                      "frames": len(fr), "removed": removed, "gouge": gouge})
        frames_out += [{"face": f.index, "tip": x.tip.tolist(), "axis": x.axis.tolist(), "band": x.band,
                        "finish": x.band >= 0 and False} for x in fr]
    res = {"tool": "flat cylindrical R10, flute 120", "bbox": [lo.tolist(), hi.tolist()],
           "stock_volume": v0, "target_volume": target.volume(),
           "final_volume": stock.volume(), "left_over": (stock - target).volume(),
           "gouge": (target - stock).volume(), "faces": stats, "seconds": round(time.time() - t0, 1)}
    meshsim.write_stl(stock, str(OUT / "flank_result.stl"))
    (OUT / "flank_frames.json").write_text(json.dumps(frames_out))
    (OUT / "flank_geometry.json").write_text(json.dumps(res, indent=1))
    print(json.dumps({k: v for k, v in res.items() if k != "faces"}, indent=1))
    for s in stats:
        print(s)

elif MODE == "robot":
    import math
    from types import SimpleNamespace as NS

    from lithocam import robot

    frames = json.loads((OUT / "flank_frames.json").read_text())
    geo = json.loads((OUT / "flank_geometry.json").read_text())
    lo, hi = np.asarray(geo["bbox"][0]) * 1e-3, np.asarray(geo["bbox"][1]) * 1e-3
    place, spindle = (1.5, -0.4, 0.0), 0.35
    links = ["base_link", "link_1", "link_2", "link_3", "link_4", "link_5", "link_6"]
    anim, summary, seed = [], [], None
    for face in sorted({f["face"] for f in frames}):
        fr = [f for f in frames if f["face"] == face]
        # turn the face towards the robot: mean outward direction = from the axis line to the tool tips
        tips = np.array([f["tip"] for f in fr])
        n = tips.mean(axis=0)
        n[2] = 0.0
        yaw = math.atan2(-place[1], -place[0]) - math.atan2(n[1], n[0])
        c, s = math.cos(yaw), math.sin(yaw)
        Rz = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])
        cell = robot.Cell("kr120")
        centre = (lo + hi) / 2
        cell.add_box_obstacle("blank", tuple(hi - lo), tuple(Rz @ centre + np.asarray(place)), yaw)
        cell.add_box_obstacle("table", (0.8, 0.8, 0.04), (place[0], place[1], -0.021))
        jp = robot.axis_path(cell, [Rz @ np.asarray(f["tip"]) for f in fr], [Rz @ np.asarray(f["axis"]) for f in fr],
                             spindle, place, seed=seed)
        hits = sum(cell.in_collision(q) for q in jp.joints[::10])
        summary.append({"face": face, "table_deg": round(math.degrees(yaw) % 360, 1), "frames": len(fr),
                        "solved": len(jp.joints), "missing": len(jp.missing), "colliding_sampled": hits,
                        "max_joint_step_deg": round(math.degrees(jp.max_step), 1)})
        if face == 0:
            for i, (q, T) in enumerate(zip(jp.joints, jp.targets)):
                if i % 33:
                    continue
                cell.env.setState(cell.joint_names, np.asarray(q, float))
                st = cell.env.getState()
                anim.append({"tf": {k: np.asarray(st.link_transforms[k].matrix()).tolist() for k in links},
                             "tool0": T[:3, 3].tolist(), "tip": (T[:3, 3] + T[:3, 2] * spindle).tolist(),
                             "axis": (-T[:3, 2]).tolist(), "yaw": yaw, "face": face})
        if jp.joints:
            seed = jp.joints[-1]
        print(json.dumps(summary[-1]), flush=True)
    (OUT / "flank_robot.json").write_text(json.dumps({"placement": place, "spindle": spindle, "summary": summary,
                                                      "anim": anim}))
