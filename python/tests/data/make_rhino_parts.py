# Copyright 2026 The lithocam authors. SPDX-License-Identifier: Apache-2.0
"""Model the five test parts natively in Rhino 8 and write them with Rhino's own STEP writer (tests/data/step_rhino/).

Run inside Rhino 8 (CPython 3): RunPythonScript on this file. The parts match make_parts.py (build123d) in size,
position and volume, so recognition of the two sets can be compared operation by operation.

The writer is RhinoCommon's Rhino.FileIO.FileStp with default options. Calling doc.ExportSelected instead opens the
STEP options dialog, which blocks an unattended run.

Two worktops are written:
  worktop.step                 the sink cutter extruded segment by segment (like ExtrudeCrv with SplitAtTangents=Yes)
  worktop_one_face_cutter.step the cutter extruded as one surface: Rhino writes its wall as one B-spline face and
                               quiddity 0.4.9 reports it as a freeform surface (a known limit, kept as a test)
"""
import math
import os

import Rhino
import Rhino.Geometry as G
import scriptcontext as sc

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "step_rhino")
TOL = 0.001
ATOL = math.radians(1.0)


def box(x0, y0, z0, x1, y1, z1):
    return G.Box(G.BoundingBox(G.Point3d(x0, y0, z0), G.Point3d(x1, y1, z1))).ToBrep()


def cyl(x, y, z0, r, h):
    return G.Cylinder(G.Circle(G.Plane(G.Point3d(x, y, z0), G.Vector3d.ZAxis), r), h).ToBrep(True, True)


def solid(b):
    if b.SolidOrientation == G.BrepSolidOrientation.Inward:
        b.Flip()
    return b


def diff(a, cutters):
    res = G.Brep.CreateBooleanDifference([a], cutters, TOL)
    if not res or len(res) != 1:
        raise RuntimeError("boolean difference returned {}".format(None if res is None else len(res)))
    b = res[0]
    b.MergeCoplanarFaces(TOL)
    return b


def extrude_closed(points, vec):
    crv = G.PolylineCurve([G.Point3d(*p) for p in points] + [G.Point3d(*points[0])])
    b = G.Brep.CreateFromSurface(G.Surface.CreateExtrusion(crv, vec)).CapPlanarHoles(TOL)
    b.Faces.SplitKinkyFaces(ATOL, True)
    return solid(b)


def cladding_panel():
    cut = [cyl(x, y, -1, 5, 21) for x in (100, 500) for y in (80, 320)]
    cut += [box(250, 385, 13.5, 350, 401, 16.5), box(250, -1, 13.5, 350, 15, 16.5)]
    return diff(box(0, 0, 0, 600, 400, 30), cut)


def window_sill():
    prof = [(0, 0, 0), (0, 20, 0), (0, 20, 10), (0, 30, 10), (0, 30, 0), (0, 200, 0), (0, 200, 35), (0, 180, 35),
            (0, 180, 50), (0, 5, 50), (0, 0, 45)]
    return extrude_closed(prof, G.Vector3d(1200, 0, 0))


def worktop(per_segment):
    rect = G.PolylineCurve([G.Point3d(500, 100, -1), G.Point3d(1000, 100, -1), G.Point3d(1000, 500, -1),
                            G.Point3d(500, 500, -1), G.Point3d(500, 100, -1)])
    rr = G.Curve.CreateFilletCornersCurve(rect, 25, TOL, ATOL)
    if per_segment:
        walls = [G.Brep.CreateFromSurface(G.Surface.CreateExtrusion(s, G.Vector3d(0, 0, 22)))
                 for s in rr.DuplicateSegments()]
        sink = G.Brep.JoinBreps(walls, TOL)[0].CapPlanarHoles(TOL)
    else:
        sink = G.Brep.CreateFromSurface(G.Surface.CreateExtrusion(rr, G.Vector3d(0, 0, 22))).CapPlanarHoles(TOL)
    return diff(box(0, 0, 0, 1500, 600, 20), [solid(sink), cyl(750, 540, -1, 17.5, 22)])


def stair_tread():
    t = box(0, 0, 0, 1000, 300, 40)
    ei = [e.EdgeIndex for e in t.Edges if abs(e.PointAtStart.Y) < 1e-6 and abs(e.PointAtEnd.Y) < 1e-6
          and abs(e.PointAtStart.Z - 40) < 1e-6 and abs(e.PointAtEnd.Z - 40) < 1e-6]
    t = G.Brep.CreateFilletEdges(t, ei, [10.0], [10.0], G.BlendType.Fillet, G.RailType.RollingBall, TOL)[0]
    return diff(t, [box(50, y, 37, 950, y + 5, 41) for y in (20, 35, 50)])


def baluster():
    pts = [(0, 0), (60, 0), (60, 40), (40, 50), (35, 300), (50, 330), (50, 360), (55, 360), (55, 380), (0, 380)]
    crv = G.PolylineCurve([G.Point3d(x, 0, z) for x, z in pts])
    rev = G.RevSurface.Create(crv, G.Line(G.Point3d(0, 0, 0), G.Point3d(0, 0, 1)))
    b = G.Brep.CreateFromRevSurface(rev, False, False)
    b.Faces.SplitKinkyFaces(ATOL, True)
    return solid(b)


def main():
    doc = sc.doc
    for o in list(doc.Objects):
        doc.Objects.Delete(o.Id, True)
    doc.AdjustModelUnitSystem(Rhino.UnitSystem.Millimeters, False)
    os.makedirs(OUT, exist_ok=True)
    parts = {"cladding_panel": cladding_panel(), "window_sill": window_sill(), "worktop": worktop(True),
             "worktop_one_face_cutter": worktop(False), "stair_tread": stair_tread(), "baluster": baluster()}
    for name, b in parts.items():
        oid = doc.Objects.AddBrep(b)
        ok = Rhino.FileIO.FileStp.Write(os.path.join(OUT, name + ".step"), doc, Rhino.FileIO.FileStpWriteOptions())
        print("{}: solid {} faces {} volume {:.0f} written {}".format(name, b.IsSolid, b.Faces.Count, b.GetVolume(), ok))
        doc.Objects.Delete(oid, True)
    doc.Modified = False


main()
