# Copyright 2026 The lithocam authors. SPDX-License-Identifier: Apache-2.0
"""Robot data for the docs GIF (run with a Python that has tesseract-robotics):

    python docs/media/make_robot_data.py <out dir>

Plans the UR10e wire sweep over the twisted column's four ruled sides (lithocam.robot.wire_face_path) and records,
per configuration, the link origins (Tesseract scene state) and the wire's end points.
"""
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "python" / "src"))
from lithocam import robot  # noqa: E402

OUT = Path(sys.argv[1])
OUT.mkdir(parents=True, exist_ok=True)
rep = robot.load_ruled_json(str(ROOT / "python" / "tests" / "data" / "ruled_json" / "twisted_column.json"))
cell = robot.Cell("ur10e")
place, bow, wr = (0.5, -0.3, 0.0), 0.25, 0.0035
links = ["base_link", "shoulder_link", "upper_arm_link", "forearm_link", "wrist_1_link", "wrist_2_link",
         "wrist_3_link", "tool0"]
frames = []
seed = None
for f in rep.faces:
    if f.label != "wire" or len(f.rulings) < 9:
        continue
    jp = robot.wire_face_path(cell, f, wr, place, bow_depth=bow, seed=seed)
    for q, T in zip(jp.joints, jp.targets):
        cell.env.setState(cell.joint_names, np.asarray(q, dtype=np.float64))
        st = cell.env.getState()
        pts = [np.asarray(st.link_transforms[n].translation()).flatten().tolist() for n in links]
        x, z = T[:3, 0], T[:3, 2]
        mid = T[:3, 3] + z * bow
        half = 0.22
        frames.append({"face": f.index, "links": pts, "wire": [(mid - x * half).tolist(), (mid + x * half).tolist()],
                       "tool0": T[:3, 3].tolist()})
    if jp.joints:
        seed = jp.joints[-1]
(OUT / "robot_ur10e_column.json").write_text(json.dumps({"placement": place, "frames": frames}))
print("frames", len(frames))
