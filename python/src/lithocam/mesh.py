# Copyright 2026 The lithocam authors. SPDX-License-Identifier: Apache-2.0
"""Tessellate a STEP B-rep into triangles for the milling kernel (needs OCP).

    tris = triangles_from_step("part.step", deflection=0.05)    # list of ((x,y,z),(x,y,z),(x,y,z))

`deflection` is the largest distance between the triangles and the exact surface (model units). Planar faces are
exact; curved faces deviate by at most that much, which adds to any toolpath tolerance downstream.
"""
from __future__ import annotations

from typing import List, Tuple

from OCP.BRep import BRep_Tool
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.TopAbs import TopAbs_FACE, TopAbs_REVERSED
from OCP.TopExp import TopExp_Explorer
from OCP.TopLoc import TopLoc_Location
from OCP.TopoDS import TopoDS

from .ruled import read_step

Pt = Tuple[float, float, float]


def triangles_from_shape(shape, deflection: float = 0.05, angular: float = 0.2) -> List[Tuple[Pt, Pt, Pt]]:
    BRepMesh_IncrementalMesh(shape, deflection, False, angular, True)
    out = []
    exp = TopExp_Explorer(shape, TopAbs_FACE)
    while exp.More():
        face = TopoDS.Face(exp.Current())
        loc = TopLoc_Location()
        tri = BRep_Tool.Triangulation_s(face, loc) if hasattr(BRep_Tool, "Triangulation_s") else \
            BRep_Tool.Triangulation(face, loc)
        if tri is not None:
            trsf = loc.Transformation()
            pts = []
            for i in range(1, tri.NbNodes() + 1):
                p = tri.Node(i).Transformed(trsf)
                pts.append((p.X(), p.Y(), p.Z()))
            rev = face.Orientation() == TopAbs_REVERSED
            for i in range(1, tri.NbTriangles() + 1):
                a, b, c = tri.Triangle(i).Get()
                t = (pts[a - 1], pts[c - 1], pts[b - 1]) if rev else (pts[a - 1], pts[b - 1], pts[c - 1])
                out.append(t)
        exp.Next()
    return out


def triangles_from_step(path: str, deflection: float = 0.05, angular: float = 0.2) -> List[Tuple[Pt, Pt, Pt]]:
    return triangles_from_shape(read_step(path), deflection, angular)


def write_stl_ascii(tris, path: str, name: str = "lithocam") -> None:
    """Plain ASCII STL (normals left zero; readers recompute them)."""
    with open(path, "w", encoding="ascii") as f:
        f.write("solid {}\n".format(name))
        for a, b, c in tris:
            f.write("facet normal 0 0 0\n outer loop\n")
            for p in (a, b, c):
                f.write("  vertex {:.9g} {:.9g} {:.9g}\n".format(*p))
            f.write(" endloop\nendfacet\n")
        f.write("endsolid {}\n".format(name))
