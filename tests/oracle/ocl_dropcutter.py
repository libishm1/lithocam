# Copyright 2026 The lithocam authors. SPDX-License-Identifier: Apache-2.0
"""OpenCAMLib drop-cutter on the same inputs as tools/dropcutter_cli (test oracle only; needs `opencamlib`).

    python ocl_dropcutter.py tris.txt points.txt ball|flat radius
Prints one tip height per point, or nan when OpenCAMLib reports no contact (it returns the start z then).
"""
import sys

import opencamlib as ocl

tris_path, pts_path, kind, radius = sys.argv[1], sys.argv[2], sys.argv[3], float(sys.argv[4])
surf = ocl.STLSurf()
zmin = 1e9
with open(tris_path) as f:
    for line in f:
        v = [float(x) for x in line.split()]
        if len(v) != 9:
            continue
        surf.addTriangle(ocl.Triangle(ocl.Point(*v[0:3]), ocl.Point(*v[3:6]), ocl.Point(*v[6:9])))
        zmin = min(zmin, v[2], v[5], v[8])
cutter = ocl.BallCutter(2 * radius, 10 * radius) if kind == "ball" else ocl.CylCutter(2 * radius, 10 * radius)
bdc = ocl.BatchDropCutter()
bdc.setSTL(surf)
bdc.setCutter(cutter)
start = zmin - 1000.0
with open(pts_path) as f:
    for line in f:
        x, y = (float(s) for s in line.split())
        bdc.appendPoint(ocl.CLPoint(x, y, start))
bdc.run()
for cl in bdc.getCLPoints():
    print("nan" if abs(cl.z - start) < 1e-9 else "{:.12f}".format(cl.z))
