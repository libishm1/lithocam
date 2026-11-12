# Copyright 2026 The lithocam authors. SPDX-License-Identifier: Apache-2.0
"""Diamond-wire cutting simulation on exact solids (OCP): yield, kerf loss, and what is left for milling.

Scope of this first version: **planar supporting cuts**, the way a stone block is squared and a slab is cut to
outline. A wire face qualifies when the whole target lies on its inner side (the face plane supports the part).
Each cut, in order:
  - kerf = stock  ∩  slab between the face plane and the plane offset by the wire diameter on the waste side
           (the wire's centre runs at half the diameter; lean/Lithocam/WireTube.lean proves it leaves the part);
  - stock = stock  ∩  inner half-space of the face plane, minus the kerf (the waste piece falls away).
After all cuts:  blank = stock;  to_mill = blank - target;  gouge = target - blank (must be ~0).

Not covered yet: ruled non-planar cuts, and internal contours (a threaded wire around a cutout); those faces are
reported in `skipped` with the reason.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional, Tuple

from OCP.BRepAlgoAPI import BRepAlgoAPI_Common, BRepAlgoAPI_Cut
from OCP.BRepBndLib import BRepBndLib
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
from OCP.BRepGProp import BRepGProp
from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox, BRepPrimAPI_MakeHalfSpace
from OCP.Bnd import Bnd_Box
from OCP.GProp import GProp_GProps
from OCP.gp import gp_Ax3, gp_Dir, gp_Pln, gp_Pnt

from . import ruled as R
from .mesh import triangles_from_shape


def _static(cls, name):
    return getattr(cls, name + "_s", None) or getattr(cls, name)


def volume(shape) -> float:
    p = GProp_GProps()
    _static(BRepGProp, "VolumeProperties")(shape, p)
    return p.Mass()


def _bbox(shape) -> Tuple[float, float, float, float, float, float]:
    b = Bnd_Box()
    _static(BRepBndLib, "Add")(shape, b)
    lo, hi = b.CornerMin(), b.CornerMax()
    return lo.X(), lo.Y(), lo.Z(), hi.X(), hi.Y(), hi.Z()


def _halfspace(point, normal, big: float, keep_side_point):
    """Solid half-space bounded by the plane (point, normal), on the side of keep_side_point."""
    pln = gp_Pln(gp_Ax3(gp_Pnt(*point), gp_Dir(*normal)))
    face = BRepBuilderAPI_MakeFace(pln, -big, big, -big, big).Face()
    return BRepPrimAPI_MakeHalfSpace(face, gp_Pnt(*keep_side_point)).Solid()


@dataclass
class Cut:
    face: int
    normal: Tuple[float, float, float]
    kerf_volume: float
    waste_volume: float


@dataclass
class WireSimReport:
    stock_volume: float
    target_volume: float
    blank_volume: float
    to_mill_volume: float
    gouge_volume: float
    cuts: List[Cut] = field(default_factory=list)
    skipped: List[Tuple[int, str]] = field(default_factory=list)

    @property
    def kerf_volume(self) -> float:
        return sum(c.kerf_volume for c in self.cuts)

    @property
    def waste_volume(self) -> float:
        return sum(c.waste_volume for c in self.cuts)

    @property
    def balance(self) -> float:
        """stock - (blank + kerf + waste): zero up to boolean round-off."""
        return self.stock_volume - (self.blank_volume + self.kerf_volume + self.waste_volume)


def simulate(target, rules: Optional[R.Rules] = None, margin: float = 20.0, stock=None) -> WireSimReport:
    rules = rules or R.Rules()
    rep = R.analyse_shape(target, rules)
    x0, y0, z0, x1, y1, z1 = _bbox(target)
    if stock is None:
        stock = BRepPrimAPI_MakeBox(gp_Pnt(x0 - margin, y0 - margin, z0 - margin),
                                    gp_Pnt(x1 + margin, y1 + margin, z1 + margin)).Shape()
    big = 10 * rep.scale + 10 * margin
    verts = [p for t in triangles_from_shape(target, deflection=1e-3 * rep.scale) for p in t]
    tol = 1e-6 * rep.scale
    v_stock = volume(stock)
    out = WireSimReport(v_stock, volume(target), 0, 0, 0)
    faces = R.faces_of(target)
    done_planes = []
    for f in rep.faces:
        if f.label != "wire":
            continue
        if f.surface != "plane":
            out.skipped.append((f.index, "ruled non-planar cut: not simulated yet"))
            continue
        r0 = f.rulings[0]
        n = r0.normal                                   # outward
        p = r0.p0
        side = [sum((v[k] - p[k]) * n[k] for k in range(3)) for v in verts]
        if max(side) > tol:
            out.skipped.append((f.index, "plane cuts through the part: internal or concave contour"))
            continue
        key = (round(n[0], 9), round(n[1], 9), round(n[2], 9), round(sum(p[k] * n[k] for k in range(3)), 6))
        if key in done_planes:
            continue                                    # coplanar faces share one cut
        done_planes.append(key)
        inside_pt = tuple(p[k] - n[k] * 1.0 for k in range(3))
        d = rules.wire_diameter
        keep = _halfspace(p, n, big, inside_pt)
        p_off = tuple(p[k] + n[k] * d for k in range(3))
        beyond = _halfspace(p_off, n, big, tuple(p_off[k] + n[k] for k in range(3)))
        slab_and_keep = BRepAlgoAPI_Cut(stock, beyond).Shape()          # stock up to the far kerf plane
        kerf = BRepAlgoAPI_Cut(slab_and_keep, keep).Shape()            # the slab the wire turns to slurry
        new_stock = BRepAlgoAPI_Common(stock, keep).Shape()
        vk = volume(kerf)
        v_before, v_after = volume(stock), volume(new_stock)
        out.cuts.append(Cut(f.index, tuple(n), vk, max(0.0, v_before - v_after - vk)))
        stock = new_stock
    out.blank_volume = volume(stock)
    out.to_mill_volume = volume(BRepAlgoAPI_Cut(stock, target).Shape())
    out.gouge_volume = volume(BRepAlgoAPI_Cut(target, stock).Shape())
    out._blank = stock                                  # for display
    return out


def simulate_step(path: str, rules: Optional[R.Rules] = None, margin: float = 20.0) -> WireSimReport:
    return simulate(R.read_step(path), rules, margin)
