# Copyright 2026 The lithocam authors. SPDX-License-Identifier: Apache-2.0
"""Robot data for the docs GIFs (run with a Python that has tesseract-robotics):

    python docs/media/make_robot_data.py <out dir>

Collision-aware wire sweeps over the twisted column's four ruled sides for the UR10e and the KR120: the part (its
STL) and a table are obstacles in the Tesseract scene, every candidate configuration is checked against them with
the robots' collision meshes, and a turntable turns each face towards the robot first. Records every link's 4x4
transform, the table angle and the wire ends per configuration, plus a collision summary.
"""
import json
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "python" / "src"))
from lithocam import robot  # noqa: E402

OUT = Path(sys.argv[1])
OUT.mkdir(parents=True, exist_ok=True)
DATA = ROOT / "python" / "tests" / "data"
rep = robot.load_ruled_json(str(DATA / "ruled_json" / "twisted_column.json"))
CELLS = {
    "ur10e": dict(place=(0.5, -0.3, 0.0), bow=0.25, half=0.22,
                  links=["base_link", "shoulder_link", "upper_arm_link", "forearm_link", "wrist_1_link",
                         "wrist_2_link", "wrist_3_link"]),
    "kr120": dict(place=(1.4, -0.5, 0.0), bow=0.6, half=0.3,
                  links=["base_link", "link_1", "link_2", "link_3", "link_4", "link_5", "link_6"]),
}
for name, c in CELLS.items():
    frames, summary = [], []
    seed = None
    for f in rep.faces:
        if f.label != "wire" or len(f.rulings) < 9:
            continue
        yaw = robot.facing_yaw(f, c["place"])
        cell = robot.Cell(name)
        cell.add_mesh_obstacle("part", str(DATA / "stl" / "twisted_column.stl"), 1e-3, c["place"], yaw)
        cell.add_box_obstacle("table", (0.8, 0.8, 0.04), (c["place"][0], c["place"][1], -0.021))
        jp = robot.wire_face_path(cell, robot.turned_face(f, yaw), 0.0035, c["place"], bow_depth=c["bow"],
                                  roll_deg=tuple(range(-90, 91, 10)), seed=seed)
        hits = sum(cell.in_collision(q) for q in jp.joints)
        summary.append({"face": f.index, "table_deg": round(math.degrees(yaw) % 360, 1), "rulings": len(f.rulings),
                        "solved": len(jp.joints), "colliding": hits,
                        "max_step_deg": round(math.degrees(jp.max_step), 1)})
        for q, T in zip(jp.joints, jp.targets):
            cell.env.setState(cell.joint_names, np.asarray(q, dtype=np.float64))
            st = cell.env.getState()
            tf = {n: np.asarray(st.link_transforms[n].matrix()).tolist() for n in c["links"]}
            x, z = T[:3, 0], T[:3, 2]
            mid = T[:3, 3] + z * c["bow"]
            frames.append({"face": f.index, "yaw": yaw, "tf": tf, "tool0": T[:3, 3].tolist(),
                           "wire": [(mid - x * c["half"]).tolist(), (mid + x * c["half"]).tolist()]})
        if jp.joints:
            seed = jp.joints[-1]
    (OUT / "robot_{}_column.json".format(name)).write_text(json.dumps({"placement": c["place"], "frames": frames,
                                                                       "summary": summary}))
    print(name, json.dumps(summary))
