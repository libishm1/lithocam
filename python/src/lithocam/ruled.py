# Copyright 2026 The lithocam authors. SPDX-License-Identifier: Apache-2.0
"""Ruled-surface extraction from a B-rep, and a process label per face: wire saw, flank milling or point milling.

A ruled surface is swept by a straight line. That line is where a diamond wire or the side of a flank-milling tool
can lie. For each face this module:

1. finds whether the face is ruled, and along which direction:
   - exactly, from the surface type: plane (any direction), cylinder, cone, linear extrusion, or a B-spline or
     Bezier face of degree 1 with two poles in one direction;
   - otherwise numerically, from the straightness of its iso-curves (`ruled = "approx"` within `Rules.ruled_tol`);
2. samples rulings across the face and clips each one to the face's trim;
3. checks access along each ruling with exact line-solid intersection:
   - the ruling is offset outward by the wire radius (or the tool radius) and extended past the part;
   - a side is free when the extended line meets no material on that side;
4. labels the face:
   - `wire` when every ruling is free on both sides (the wire passes through only waste);
   - `flank` when every ruling is free on at least one side at the tool radius (the shank leaves from that side);
   - `point` otherwise, with the reason.

The labels are geometric reachability for a straight wire or tool. They are not a process plan: a `wire` face
inside a closed contour also needs a threading hole, and the robot and bow clearance are checked later in the
motion planner.

Needs OpenCascade through OCP (`pip install cadquery-ocp`, or build123d which brings it).

    python -m lithocam.ruled part.step [--json out.json] [--wire-diameter 7] [--tool-radius 5]
"""
from __future__ import annotations

import json
import math
from dataclasses import asdict, dataclass, field
from typing import List, Optional, Sequence, Tuple

from OCP.Bnd import Bnd_Box
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.BRepBndLib import BRepBndLib
from OCP.BRepClass import BRepClass_FaceClassifier
from OCP.BRepClass3d import BRepClass3d_SolidClassifier
from OCP.BRepGProp import BRepGProp_Face
from OCP.BRepTools import BRepTools
from OCP.GeomAbs import GeomAbs_SurfaceType as ST
from OCP.gp import gp_Dir, gp_Lin, gp_Pnt, gp_Pnt2d, gp_Vec
from OCP.IntCurvesFace import IntCurvesFace_ShapeIntersector
from OCP.STEPControl import STEPControl_Reader
from OCP.TopAbs import TopAbs_FACE, TopAbs_IN, TopAbs_OUT
from OCP.TopExp import TopExp_Explorer
from OCP.TopoDS import TopoDS

Vec3 = Tuple[float, float, float]
RANK = {"wire": 2, "flank": 1, "point": 0}


def _static(cls, name):
    """OCP builds differ: static methods are exposed as `Name_s` or `Name`."""
    return getattr(cls, name + "_s", None) or getattr(cls, name)


@dataclass
class Rules:
    wire_diameter: float = 7.0        # diamond wire (beaded) diameter; the wire centre runs at half of it
    tool_radius: float = 5.0          # flank-milling tool radius
    wire_length_max: Optional[float] = None   # longest ruling a wire bow spans; None = no limit
    ruled_tol: float = 0.01           # max deviation of an iso-curve from its chord to count as ruled (model units)
    n_rulings: int = 9                # rulings sampled across a face for the access test
    n_samples: int = 48               # trim-classification samples along one ruling before refinement
    plane_angles: int = 8             # in-plane ruling directions tried on a planar face (0 to 180 degrees)
    trim_step: float = 1.0            # on planes, sample the trim at least this often (model units): finds gaps
                                      # in a face (openings, slots) at least this wide


@dataclass
class Ruling:
    p0: Vec3
    p1: Vec3
    length: float
    normal: Vec3                      # outward unit normal at the ruling's midpoint
    wire_free: Tuple[bool, bool]      # side of p0, side of p1: the wire's centre line is clear
    tool_free: Tuple[bool, bool]      # the same at the tool radius
    wire_room: Tuple[bool, bool] = (False, False)   # the wire's full width is clear (sides and top as well)
    tool_room: Tuple[bool, bool] = (False, False)


@dataclass
class FaceResult:
    index: int
    surface: str
    ruled: str                        # exact | approx | no
    deviation: float                  # iso-curve deviation from straight (0 for exact types)
    family: Optional[str]             # u | v | plane:<degrees> | None
    label: str                        # wire | flank | point
    reason: str
    max_ruling: float = 0.0
    rulings: List[Ruling] = field(default_factory=list)


@dataclass
class RuledReport:
    faces: List[FaceResult]
    rules: Rules
    scale: float

    def counts(self):
        out = {}
        for f in self.faces:
            out[f.label] = out.get(f.label, 0) + 1
        return out

    def to_json(self) -> str:
        return json.dumps(asdict(self), indent=1)


# ---------------------------------------------------------------- reading

def read_step(path: str):
    r = STEPControl_Reader()
    if int(r.ReadFile(str(path))) != 1:          # IFSelect_RetDone
        raise RuntimeError("cannot read STEP file: {}".format(path))
    r.TransferRoots()
    return r.OneShape()


def faces_of(shape):
    out = []
    exp = TopExp_Explorer(shape, TopAbs_FACE)
    while exp.More():
        out.append(TopoDS.Face(exp.Current()))
        exp.Next()
    return out


def _scale(shape) -> float:
    box = Bnd_Box()
    _static(BRepBndLib, "Add")(shape, box)
    return box.CornerMin().Distance(box.CornerMax())


# ---------------------------------------------------------------- ruled detection

def _pt(p) -> Vec3:
    return (p.X(), p.Y(), p.Z())


def _iso_deviation(a, family: str, uv) -> float:
    """Max distance of sampled iso-curves from their chords. family 'v' = lines of constant u."""
    umin, umax, vmin, vmax = uv
    worst = 0.0
    for i in range(7):
        s = (i + 0.5) / 7
        pts = []
        for j in range(17):
            t = j / 16
            u, v = (umin + s * (umax - umin), vmin + t * (vmax - vmin)) if family == "v" else \
                (umin + t * (umax - umin), vmin + s * (vmax - vmin))
            pts.append(_pt(a.Value(u, v)))
        p, q = pts[0], pts[-1]
        d = [q[k] - p[k] for k in range(3)]
        dl = math.sqrt(sum(x * x for x in d))
        for x in pts[1:-1]:
            w = [x[k] - p[k] for k in range(3)]
            if dl < 1e-12:
                dist = math.sqrt(sum(c * c for c in w))
            else:
                c = [w[1] * d[2] - w[2] * d[1], w[2] * d[0] - w[0] * d[2], w[0] * d[1] - w[1] * d[0]]
                dist = math.sqrt(sum(cc * cc for cc in c)) / dl
            worst = max(worst, dist)
    return worst


def ruled_families(a, uv, rules: Rules, scale: float):
    """Returns (ruled, deviation, families) where families lists candidate ruling directions."""
    st = a.GetType()
    if st == ST.GeomAbs_Plane:
        return "exact", 0.0, ["plane:{:g}".format(180.0 * k / rules.plane_angles) for k in range(rules.plane_angles)]
    if st in (ST.GeomAbs_Cylinder, ST.GeomAbs_Cone, ST.GeomAbs_SurfaceOfExtrusion):
        return "exact", 0.0, ["v"]
    fams = []
    if st in (ST.GeomAbs_BSplineSurface, ST.GeomAbs_BezierSurface):
        if a.VDegree() == 1 and a.NbVPoles() == 2:
            fams.append("v")
        if a.UDegree() == 1 and a.NbUPoles() == 2:
            fams.append("u")
        if fams:
            return "exact", 0.0, fams
    devs = {f: _iso_deviation(a, f, uv) for f in ("u", "v")}
    ok = sorted((d, f) for f, d in devs.items() if d <= rules.ruled_tol)
    if ok:
        return ("exact" if ok[0][0] <= 1e-9 * scale else "approx"), ok[0][0], [f for _, f in ok]
    return "no", min(devs.values()), []


# ---------------------------------------------------------------- rulings and trimming

def _clip(face, uv0, uv1, n: int, tol: float, step: float = 0.0) -> List[Tuple[float, float]]:
    """Parameter intervals in [0, 1] of the uv segment uv0 -> uv1 that lie on the face (inside its trim).

    Samples at least every `step` (uv units) so that gaps in the trim wider than `step` are found.
    """
    def inside(t):
        u = uv0[0] + t * (uv1[0] - uv0[0])
        v = uv0[1] + t * (uv1[1] - uv0[1])
        return BRepClass_FaceClassifier(face, gp_Pnt2d(u, v), tol).State() != TopAbs_OUT

    def edge(a, b, a_in):              # bisect the boundary between a (state a_in) and b (the other state)
        for _ in range(30):
            m = 0.5 * (a + b)
            if inside(m) == a_in:
                a = m
            else:
                b = m
        return 0.5 * (a + b)

    n = max(n, int(math.ceil(math.dist(uv0, uv1) / step)) + 1) if step else n
    ts = [i / (n - 1) for i in range(n)]
    st = [inside(t) for t in ts]
    out, start = [], (0.0 if st[0] else None)
    for i in range(1, n):
        if st[i] and not st[i - 1]:
            start = edge(ts[i - 1], ts[i], False)
        elif not st[i] and st[i - 1] and start is not None:
            out.append((start, edge(ts[i - 1], ts[i], True)))
            start = None
    if start is not None:
        out.append((start, 1.0))
    return [iv for iv in out if iv[1] - iv[0] > 1e-9]


def _box_clip(p, q, uv):
    """Liang-Barsky clip of the uv segment p -> q to the box (umin, umax, vmin, vmax)."""
    umin, umax, vmin, vmax = uv
    t0, t1 = 0.0, 1.0
    dx, dy = q[0] - p[0], q[1] - p[1]
    for pk, qk in ((-dx, p[0] - umin), (dx, umax - p[0]), (-dy, p[1] - vmin), (dy, vmax - p[1])):
        if abs(pk) < 1e-15:
            if qk < 0:
                return None
            continue
        r = qk / pk
        if pk < 0:
            t0 = max(t0, r)
        else:
            t1 = min(t1, r)
        if t0 > t1:
            return None
    return ((p[0] + t0 * dx, p[1] + t0 * dy), (p[0] + t1 * dx, p[1] + t1 * dy))


def _param_lines(a, family: str, uv, n: int):
    """n parallel uv segments of one ruling family spanning the face's uv box."""
    umin, umax, vmin, vmax = uv
    lines = []
    if family == "v":
        for i in range(n):
            u = umin + (i + 0.5) / n * (umax - umin)
            lines.append(((u, vmin), (u, vmax)))
    elif family == "u":
        for i in range(n):
            v = vmin + (i + 0.5) / n * (vmax - vmin)
            lines.append(((umin, v), (umax, v)))
    else:                               # plane: u and v are lengths along the plane's axes
        th = math.radians(float(family.split(":")[1]))
        d = (math.cos(th), math.sin(th))
        nrm = (-d[1], d[0])
        cu, cv = 0.5 * (umin + umax), 0.5 * (vmin + vmax)
        corners = [(umin, vmin), (umin, vmax), (umax, vmin), (umax, vmax)]
        offs = [(c[0] - cu) * nrm[0] + (c[1] - cv) * nrm[1] for c in corners]
        half = 0.5 * math.dist((umin, vmin), (umax, vmax)) + 1e-9
        for i in range(n):
            o = min(offs) + (i + 0.5) / n * (max(offs) - min(offs))
            c = (cu + o * nrm[0], cv + o * nrm[1])
            seg = _box_clip((c[0] - half * d[0], c[1] - half * d[1]), (c[0] + half * d[0], c[1] + half * d[1]), uv)
            if seg:
                lines.append(seg)
    return lines


class _Access:
    """Exact access tests along an offset ruling line against the whole solid."""

    def __init__(self, shape, scale: float):
        self.shape = shape
        self.scale = scale
        self.eps = 1e-6 * scale
        self.inter = IntCurvesFace_ShapeIntersector()
        self.inter.Load(shape, 1e-7 * scale)

    def free_sides(self, p0: Vec3, p1: Vec3, n: Vec3, r: float) -> Tuple[bool, bool]:
        """Free sides for a wire or tool of radius r lying on the ruling.

        The tool's centre line and three lines on its surface (both sides and the top, as seen from the face) must
        all be clear, so a slot narrower than the tool blocks it.
        """
        L = math.dist(p0, p1)
        if L < self.eps:
            return (False, False)
        d = [(p1[k] - p0[k]) / L for k in range(3)]
        b = [d[1] * n[2] - d[2] * n[1], d[2] * n[0] - d[0] * n[2], d[0] * n[1] - d[1] * n[0]]
        s = r * (1 - 1e-3)
        res = []
        for o in ([0.0, 0.0, 0.0], [s * x for x in b], [-s * x for x in b], [s * x for x in n]):
            res.append(self._line_free([p0[k] + n[k] * (r + self.eps) + o[k] for k in range(3)],
                                       [p1[k] + n[k] * (r + self.eps) + o[k] for k in range(3)]))
        centre = res[0]
        room = (all(x[0] for x in res), all(x[1] for x in res))
        return centre, room

    def _line_free(self, a, b) -> Tuple[bool, bool]:
        d = [b[k] - a[k] for k in range(3)]
        L = math.sqrt(sum(x * x for x in d))
        mid = gp_Pnt(*[(a[k] + b[k]) / 2 for k in range(3)])
        cls = BRepClass3d_SolidClassifier(self.shape, mid, 1e-7 * self.scale)
        if cls.State() == TopAbs_IN:   # the wire or tool does not fit where it would have to run
            return (False, False)
        self.inter.Perform(gp_Lin(gp_Pnt(*a), gp_Dir(*d)), -2 * self.scale, L + 2 * self.scale)
        f0 = f1 = True
        for i in range(1, self.inter.NbPnt() + 1):
            t = self.inter.WParameter(i)
            if t <= L / 2:
                f0 = False
            else:
                f1 = False
        return (f0, f1)


def _normal(face, u: float, v: float) -> Vec3:
    p, vec = gp_Pnt(), gp_Vec()
    BRepGProp_Face(face).Normal(u, v, p, vec)
    m = vec.Magnitude()
    return (vec.X() / m, vec.Y() / m, vec.Z() / m) if m > 0 else (0.0, 0.0, 0.0)


def _rulings(face, a, family: str, uv, rules: Rules, acc: _Access) -> List[Ruling]:
    out = []
    tol = 1e-7 * acc.scale
    for uv0, uv1 in _param_lines(a, family, uv, rules.n_rulings):
        for t0, t1 in _clip(face, uv0, uv1, rules.n_samples, tol, rules.trim_step if family.startswith("plane") else 0.0):
            q0 = (uv0[0] + t0 * (uv1[0] - uv0[0]), uv0[1] + t0 * (uv1[1] - uv0[1]))
            q1 = (uv0[0] + t1 * (uv1[0] - uv0[0]), uv0[1] + t1 * (uv1[1] - uv0[1]))
            p0, p1 = _pt(a.Value(*q0)), _pt(a.Value(*q1))
            ln = math.dist(p0, p1)
            if ln < 1e-4 * acc.scale:
                continue
            n = _normal(face, 0.5 * (q0[0] + q1[0]), 0.5 * (q0[1] + q1[1]))
            wc, wr = acc.free_sides(p0, p1, n, rules.wire_diameter / 2)
            tc, tr = acc.free_sides(p0, p1, n, rules.tool_radius)
            out.append(Ruling(p0=p0, p1=p1, length=ln, normal=n, wire_free=wc, tool_free=tc,
                              wire_room=wr, tool_room=tr))
    return out


def _label(rulings: List[Ruling], rules: Rules) -> Tuple[str, str]:
    if not rulings:
        return "point", "no ruling long enough on the face"
    # Every ruling's centre line must be clear. At least one ruling must also have room for the full wire or
    # tool width: rulings next to a concave corner are allowed to be tight, because the wire or tool leaves its
    # own radius in that corner (as on a real machine).
    longest = max(r.length for r in rulings)
    flank_ok = all(any(r.tool_free) for r in rulings) and any(any(r.tool_room) for r in rulings)
    if all(all(r.wire_free) for r in rulings) and any(all(r.wire_room) for r in rulings):
        if rules.wire_length_max is not None and longest > rules.wire_length_max:
            return ("flank" if flank_ok else "point"), \
                "rulings longer than the wire bow ({:.1f} > {:.1f})".format(longest, rules.wire_length_max)
        return "wire", "every ruling is free on both sides; corners keep the wire radius"
    if flank_ok:
        return "flank", "every ruling is free on one side at the tool radius"
    blocked = sum(1 for r in rulings if not any(r.tool_free))
    return "point", "{} of {} rulings blocked on both sides".format(blocked, len(rulings))


# ---------------------------------------------------------------- driver

_NAMES = {ST.GeomAbs_Plane: "plane", ST.GeomAbs_Cylinder: "cylinder", ST.GeomAbs_Cone: "cone",
          ST.GeomAbs_Sphere: "sphere", ST.GeomAbs_Torus: "torus", ST.GeomAbs_BezierSurface: "bezier",
          ST.GeomAbs_BSplineSurface: "bspline", ST.GeomAbs_SurfaceOfRevolution: "revolution",
          ST.GeomAbs_SurfaceOfExtrusion: "extrusion", ST.GeomAbs_OffsetSurface: "offset",
          ST.GeomAbs_OtherSurface: "other"}


def analyse_shape(shape, rules: Optional[Rules] = None) -> RuledReport:
    rules = rules or Rules()
    scale = _scale(shape)
    acc = _Access(shape, scale)
    results = []
    for i, face in enumerate(faces_of(shape)):
        a = BRepAdaptor_Surface(face, True)
        uv = tuple(_static(BRepTools, "UVBounds")(face))
        ruled, dev, fams = ruled_families(a, uv, rules, scale)
        name = _NAMES.get(a.GetType(), "other")
        if not fams:
            results.append(FaceResult(i, name, ruled, dev, None, "point",
                                      "not ruled (iso-curve deviation {:.4g})".format(dev)))
            continue
        best = None
        for fam in fams:
            rs = _rulings(face, a, fam, uv, rules, acc)
            label, reason = _label(rs, rules)
            longest = max((r.length for r in rs), default=0.0)
            key = (RANK[label], bool(rs), -longest)
            if best is None or key > best[0]:
                best = (key, fam, label, reason, rs, longest)
        _, fam, label, reason, rs, longest = best
        results.append(FaceResult(i, name, ruled, dev, fam, label, reason, longest, rs))
    return RuledReport(results, rules, scale)


def analyse_step(path: str, rules: Optional[Rules] = None) -> RuledReport:
    return analyse_shape(read_step(path), rules)


def main(argv: Optional[Sequence[str]] = None) -> int:
    import argparse  # noqa: PLC0415

    ap = argparse.ArgumentParser(prog="python -m lithocam.ruled", description=__doc__.split("\n\n")[0])
    ap.add_argument("step")
    ap.add_argument("--json")
    ap.add_argument("--wire-diameter", type=float, default=Rules.wire_diameter)
    ap.add_argument("--tool-radius", type=float, default=Rules.tool_radius)
    ap.add_argument("--wire-length-max", type=float, default=None)
    a = ap.parse_args(argv)
    rep = analyse_step(a.step, Rules(wire_diameter=a.wire_diameter, tool_radius=a.tool_radius,
                                     wire_length_max=a.wire_length_max))
    for f in rep.faces:
        print("{:>3} {:<10} {:<6} {:<10} {:<6} max {:>8.1f}  {}".format(
            f.index, f.surface, f.ruled, f.family or "-", f.label, f.max_ruling, f.reason))
    print("counts", rep.counts())
    if a.json:
        with open(a.json, "w", encoding="utf-8") as fh:
            fh.write(rep.to_json())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
