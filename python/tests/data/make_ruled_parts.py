# Copyright 2026 The lithocam authors. SPDX-License-Identifier: Apache-2.0
"""Ruled and non-ruled test parts for lithocam.ruled (exact B-rep STEP via build123d / OpenCascade).

    python -B tests/data/make_ruled_parts.py

twisted_column  a 300 x 300 square at z=0 and the same square turned 30 degrees at z=900, joined by a ruled loft:
                four warped (hyperbolic-paraboloid-like) ruled sides, each a B-spline of degree 1 across the height
cone_frustum    a cone from D400 at z=0 to D200 at z=500
freeform_loft   a smooth (not ruled) loft through three different sections; its side is a negative control
"""
from pathlib import Path

from build123d import Axis, Circle, Plane, Pos, Rectangle, Solid, export_step, loft
from build123d import BuildSketch

OUT = Path(__file__).resolve().parent / "step_ruled"
OUT.mkdir(exist_ok=True)


def section(z, w, h, angle=0.0, circle=None):
    with BuildSketch(Plane.XY.offset(z).rotated((0, 0, angle))) as s:
        if circle:
            Circle(circle)
        else:
            Rectangle(w, h)
    return s.sketch


parts = {
    "twisted_column": loft([section(0, 300, 300), section(900, 300, 300, 30)], ruled=True),
    "cone_frustum": Solid.make_cone(200, 100, 500),
    "freeform_loft": loft([section(0, 400, 300), section(300, 0, 0, circle=170), section(600, 300, 300, 20)],
                          ruled=False),
}
for name, p in parts.items():
    export_step(p, str(OUT / (name + ".step")))
    print(name, "faces", len(p.faces()), "volume {:.0f}".format(p.volume),
          "types", sorted({f.geom_type if isinstance(f.geom_type, str) else f.geom_type.name for f in p.faces()}))
