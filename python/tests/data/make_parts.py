"""Five typical dimension-stone parts as exact B-rep STEP files (OpenCascade via build123d), for a quiddity feature test.

    D:/tools/quiddity_env/Scripts/python.exe -B make_parts.py

Units mm. EXPECTED is the ground truth the recognition is scored against.
"""
import json
from pathlib import Path

from build123d import (Align, Axis, Box, BuildSketch, Cylinder, Plane, Polygon, Pos, RectangleRounded, export_step, extrude,
                       fillet, revolve)

HERE = Path(__file__).resolve().parent
OUT = HERE / "step"
OUT.mkdir(exist_ok=True)
MIN = (Align.MIN, Align.MIN, Align.MIN)
EXPECTED = {
    "cladding_panel": "plate 600x400x30; 4 blind holes D10 depth 20 from the back (a 2x2 hole pattern); "
                      "2 edge kerf slots 3 wide, 15 deep, 100 long, one on each long edge",
    "window_sill": "1200 long extruded profile; front top chamfer 5x5; drip groove 10x10 underneath, 20 from the front; "
                   "rear top rebate (step) 20 deep x 15 high",
    "worktop": "plate 1500x600x20; through sink cutout 500x400 with R25 corners; through tap hole D35",
    "stair_tread": "block 1000x300x40; R10 fillet on the front top edge (nosing); 3 stopped grooves 5 wide x 3 deep x 900 long "
                   "(a groove or slot pattern)",
    "baluster": "turned body, height 380: base D120 x 40, cone to D80, taper D80 to D70, bead D100, cap D110",
}


def sill():
    with BuildSketch(Plane.YZ) as s:                       # local (u, v) = global (y, z)
        Polygon((0, 0), (20, 0), (20, 10), (30, 10), (30, 0), (200, 0), (200, 35), (180, 35), (180, 50), (5, 50), (0, 45),
                align=None)
    return extrude(s.sketch, amount=1200)


def cladding():
    p = Box(600, 400, 30, align=MIN)
    for x in (100, 500):
        for y in (80, 320):
            p -= Pos(x, y, 0) * Cylinder(5, 20, align=(Align.CENTER, Align.CENTER, Align.MIN))
    p -= Pos(250, 385, 13.5) * Box(100, 15.01, 3, align=MIN)
    p -= Pos(250, -0.01, 13.5) * Box(100, 15.01, 3, align=MIN)
    return p


def worktop():
    w = Box(1500, 600, 20, align=MIN)
    with BuildSketch(Plane.XY.offset(-1)) as s:
        RectangleRounded(500, 400, 25)
    w -= Pos(750, 300, 0) * extrude(s.sketch, amount=22)
    w -= Pos(750, 540, -1) * Cylinder(17.5, 22, align=(Align.CENTER, Align.CENTER, Align.MIN))
    return w


def tread():
    t = Box(1000, 300, 40, align=MIN)
    edge = [e for e in t.edges().filter_by(Axis.X) if abs(e.center().Z - 40) < 1e-6 and abs(e.center().Y) < 1e-6]
    t = fillet(edge, radius=10)
    for y in (20, 35, 50):
        t -= Pos(50, y, 37) * Box(900, 5, 3.01, align=MIN)
    return t


def baluster():
    pts = [(0, 0), (60, 0), (60, 40), (40, 50), (35, 300), (50, 330), (50, 360), (55, 360), (55, 380), (0, 380)]
    with BuildSketch(Plane.XZ) as s:                       # local (u, v) = global (x, z)
        Polygon(*pts, align=None)
    return revolve(s.sketch, axis=Axis.Z)


if __name__ == "__main__":
    parts = {"cladding_panel": cladding(), "window_sill": sill(), "worktop": worktop(), "stair_tread": tread(), "baluster": baluster()}
    for name, p in parts.items():
        export_step(p, str(OUT / f"{name}.step"))
        bb = p.bounding_box()
        print(f"{name}: faces {len(p.faces())}, volume {p.volume:.0f} mm3, size {bb.size.X:.0f} x {bb.size.Y:.0f} x {bb.size.Z:.0f}")
    (HERE / "expected.json").write_text(json.dumps(EXPECTED, indent=1), encoding="utf-8")
