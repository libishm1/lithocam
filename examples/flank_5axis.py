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
        fr = flank.flank_frames(f.rulings, R_TOOL, FLUTE)
        stock, removed, gouge = flank.simulate(stock, target, fr, R_TOOL, FLUTE)
        stats.append({"face": f.index, "label": f.label, "frames": len(fr), "removed": removed, "gouge": gouge})
        frames_out += [{"face": f.index, "tip": x.tip.tolist(), "axis": x.axis.tolist(), "band": x.band} for x in fr]
    res = {"tool": "flat cylindrical R10, flute 120", "stock_volume": v0, "target_volume": target.volume(),
           "final_volume": stock.volume(), "left_over": (stock - target).volume(),
           "gouge": (target - stock).volume(), "faces": stats, "seconds": round(time.time() - t0, 1)}
    meshsim.write_stl(stock, str(OUT / "flank_result.stl"))
    (OUT / "flank_frames.json").write_text(json.dumps(frames_out))
    (OUT / "flank_geometry.json").write_text(json.dumps(res, indent=1))
    print(json.dumps({k: v for k, v in res.items() if k != "faces"}, indent=1))
    for s in stats:
        print(s)

elif MODE == "robot":
    from lithocam import robot

    frames = json.loads((OUT / "flank_frames.json").read_text())
    cell = robot.Cell("kr120")
    place, spindle = (1.5, -0.4, 0.0), 0.35
    links = ["base_link", "link_1", "link_2", "link_3", "link_4", "link_5", "link_6"]
    jp = robot.axis_path(cell, [f["tip"] for f in frames], [f["axis"] for f in frames], spindle, place)
    anim = []
    for i, (q, T) in enumerate(zip(jp.joints, jp.targets)):
        if i % 3:
            continue
        cell.env.setState(cell.joint_names, np.asarray(q, float))
        st = cell.env.getState()
        anim.append({"tf": {n: np.asarray(st.link_transforms[n].matrix()).tolist() for n in links},
                     "tool0": T[:3, 3].tolist(), "tip": (T[:3, 3] + T[:3, 2] * spindle).tolist(),
                     "face": frames[i]["face"]})
    res = {"frames": len(frames), "solved": len(jp.joints), "missing": len(jp.missing),
           "max_joint_step_deg": round(float(np.degrees(jp.max_step)), 2), "placement": place, "spindle": spindle}
    (OUT / "flank_robot.json").write_text(json.dumps({**res, "anim": anim}))
    print(json.dumps(res, indent=1))
