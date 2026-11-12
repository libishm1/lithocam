# Copyright 2026 The lithocam authors. SPDX-License-Identifier: Apache-2.0
"""Rhino 8: recognise stone operations on solids and mark them in the model.

Run with RunPythonScript (Rhino 8, CPython 3.9). For each selected closed Brep (or every closed Brep when nothing is
selected) it:
  1. writes the Brep to a temporary STEP file with Rhino's STEP writer (RhinoCommon FileStp, no dialog);
  2. runs lithocam.features.recognise, which canonicalises Rhino's straight edges and calls quiddity;
  3. adds one text dot per operation on layer lithocam::<route>, coloured by route, and prints a table.

Setup, once:
  - quiddity executable: pip install quiddity into any Python 3.11+, then set the environment variable
    LITHOCAM_QUIDDITY to its quiddity(.exe), or put it on PATH;
  - lithocam on Rhino's path: pip install lithocam into Rhino's Python, or set LITHOCAM_PYTHON_SRC to the
    repository's python/src folder. By default this script looks for ../python/src next to itself.

Known limits (quiddity 0.4.9 on Rhino STEP): Rhino writes cones, general revolves and one-surface extrusions of
tangent-continuous curves as B-spline faces, which quiddity reports as freeform surfaces. Extrude cutters segment by
segment (ExtrudeCrv with SplitAtTangents=Yes) to keep their walls planar and cylindrical.
"""
import os
import sys
import tempfile

import Rhino
import scriptcontext as sc
import System

SRC = os.environ.get("LITHOCAM_PYTHON_SRC") or os.path.join(os.path.dirname(os.path.abspath(__file__)), "..",
                                                             "python", "src")
if os.path.isdir(SRC) and SRC not in sys.path:
    sys.path.insert(0, SRC)

from lithocam.features import recognise  # noqa: E402

COLOURS = {"drill": (220, 50, 50), "saw_disc": (40, 110, 220), "profile_wheel": (30, 160, 90),
           "contour_cut": (200, 120, 20), "router": (140, 60, 190), "lathe": (20, 150, 160),
           "unmapped": (120, 120, 120)}


def layer(doc, route):
    name = "lithocam::" + route
    idx = doc.Layers.FindByFullPath(name, -1)
    if idx >= 0:
        return idx
    parent = doc.Layers.FindByFullPath("lithocam", -1)
    if parent < 0:
        parent = doc.Layers.Add("lithocam", System.Drawing.Color.Black)
    lay = Rhino.DocObjects.Layer()
    lay.Name = route
    lay.ParentLayerId = doc.Layers[parent].Id
    lay.Color = System.Drawing.Color.FromArgb(*COLOURS.get(route, (0, 0, 0)))
    return doc.Layers.Add(lay)


def step_of(brep, path):
    """Write one Brep alone with Rhino's STEP writer, via a headless document in the same units."""
    tmp = Rhino.RhinoDoc.CreateHeadless(None)
    try:
        tmp.AdjustModelUnitSystem(sc.doc.ModelUnitSystem, False)
        tmp.Objects.AddBrep(brep)
        return Rhino.FileIO.FileStp.Write(path, tmp, Rhino.FileIO.FileStpWriteOptions())
    finally:
        tmp.Dispose()


def run(doc=None, objs=None):
    doc = doc or sc.doc
    if objs is None:
        objs = [o for o in doc.Objects.GetSelectedObjects(False, False)] or list(doc.Objects)
    breps = [o for o in objs if isinstance(o.Geometry, Rhino.Geometry.Brep) and o.Geometry.IsSolid]
    results = []
    for o in breps:
        fd, path = tempfile.mkstemp(suffix=".step", prefix="lithocam_rhino_")
        os.close(fd)
        try:
            if not step_of(o.Geometry, path):
                print("STEP export failed for", o.Id)
                continue
            fs = recognise(path)
        finally:
            os.remove(path)
        name = o.Attributes.Name or str(o.Id)[:8]
        print("== {}: {} operations, {} faces, {} unexplained, {} edges canonicalised".format(
            name, len(fs.ops), fs.face_count, len(fs.unexplained_faces), fs.canonicalised_edges))
        for op in fs.ops:
            print("   {:<16} {:<14} ({:.1f}, {:.1f}, {:.1f})".format(op.kind, op.route, *op.at))
            if any(v != v for v in op.at):          # NaN: freeform records carry no position
                continue
            attr = Rhino.DocObjects.ObjectAttributes()
            attr.LayerIndex = layer(doc, op.route)
            attr.Name = "lithocam {} {}".format(op.kind, ",".join(op.source))
            doc.Objects.AddTextDot(Rhino.Geometry.TextDot(op.kind, Rhino.Geometry.Point3d(*op.at)), attr)
        for p in fs.patterns:
            print("   pattern {} x{}{}".format(p["kind"], p["count"],
                                               " pitch {}".format(p["pitch"]) if "pitch" in p else ""))
        results.append((o.Id, fs))
    doc.Views.Redraw()
    return results


if __name__ == "__main__":
    run()
