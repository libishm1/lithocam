# Copyright 2026 The lithocam authors. SPDX-License-Identifier: Apache-2.0
"""G-code and DXF export for CNC diamond wire saws, one generic exporter driven by a machine definition (numpy only).

Three axis layouts are covered. All lengths are millimetres, angles are degrees.

``xy``   Trolley (horizontal, carries the wire frame across the stone) and height (the wire frame up and down).
         The wire is a straight horizontal line, so the cut is a plane profile in the trolley-height plane.
``xyc``  ``xy`` plus a table that turns the stone about the vertical axis. Two modes:
         indexed: the table sits at a fixed angle per setup and a plane profile is cut (a 4-sided column is four
         setups at 0, 90, 180 and 270 degrees);
         coordinated: the table turns while the trolley and height move (turned shapes such as balusters).
``xyuv`` Two wire-end carriages, each with its own height (and, when the machine has them, its own trolley). The wire is
         the straight line between the two ends, so a ruled surface is a sequence of wire positions.

Machine definition
------------------
``MachineDef`` holds everything that differs between machines: which letter drives which logical axis (``trolley``,
``height``, ``table``, ``height2``, optional ``trolley2``), travel limits, the table unwinding rule, feed mode, arcs,
kerf handling, M-codes, header and footer, line numbers, file split, decimals, corner handling and lead lengths. It
loads and saves as JSON. No default values are taken from a real machine; the examples under
``examples/wiresaw_machines`` hold placeholders.

Kinematics of coordinated turning (xyc)
---------------------------------------
The wire plane is the vertical plane that holds the trolley and height axes. It contains the table axis, which stands
vertical at trolley position ``table_axis_trolley`` (call it x0). A turned profile is given as (s, z): s is the radius
from the table axis and z the height. The wire is a straight horizontal line in the wire plane, at trolley position
x = x0 + s and height z. The stone turns by theta. A straight wire at distance s from the axis touches the cylinder of
radius s along one line, so over one turn the wire removes everything beyond radius s. With s following the profile,
the envelope is the surface of revolution r(z) = s(z). The wire has radius rw, so the wire centre follows the profile
offset by rw on the waste side (larger s for an outside turning), in the (s, z) plane normal to the profile. The
motion is X = x0 + s, Z = z, C = theta, all interpolated linearly. The table angle follows the path length along the
profile: theta = theta0 + deg_per_mm * (length from the path start), with a cap on the angle per block. A step in the
angle larger than a few degrees makes the envelope a polygon. The wire tilt and its bow are not modelled.

Kerf handling (machine setting ``kerf_mode``)
---------------------------------------------
``cam``  The exporter offsets the profile by the wire radius to the waste side (polygon and arc offset, round or mitre
         joins) and writes the wire-centre path. It refuses profiles whose offset crosses itself or comes closer to
         the profile than the wire radius.
``g41``  The nominal profile is written with G41 (waste on the left of travel) or G42 (waste on the right) and a D word
         on the lead-in block, and G40 on the lead-out block. The D register must hold the wire radius on the
         controller. The lead-in must be at least one wire radius long. Only plane (xy, xyc indexed) profiles can use
         it; coordinated turning and ruled surfaces use ``cam``.

Lead-in is a straight line tangent to the start of the path. Lead-out leaves along the end tangent of an open profile;
a closed profile retraces its lead-in. The exporter checks that the wire-centre lead lines stay at least one wire
radius from the profile. There is no stock model, so rapids between setups are not checked against the stone.

Ruled surfaces (xyuv)
---------------------
``ruled_face_setup`` converts one face of the lithocam ruled-face JSON (``lithocam.wirepath.load_ruled_json``) into a
``RuledSetup``. A frame such as ``("z", "y", "-x")`` says which part axis is the trolley, the depth (along the wire)
and the height direction of the machine. Each ruling is offset by the wire radius along the outward face normal,
intersected with the two carriage planes (at fixed depth positions) and written as (x0, h0, x1, h1). Rulings must be
monotone in the trolley position, the wire ends must stay inside the height travel, and if the machine has no second
trolley axis the wire must stand in a plane of constant trolley position.
"""
from __future__ import annotations

import json
import math
import re
from dataclasses import asdict, dataclass, field, fields
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple, Union

import numpy as np

Vec2 = Tuple[float, float]
_TWO_PI = 2.0 * math.pi
LOGICAL = ("trolley", "height", "trolley2", "height2", "table")
_RESERVED = set("GMFNPDIJKO")


class WireSawError(ValueError):
    """Base class of every refusal from this module."""


class KerfError(WireSawError):
    """The offset path crosses itself or comes closer to the profile than the wire radius."""


class TravelLimitError(WireSawError):
    """A planned position is outside the machine travel."""


class FeedError(WireSawError):
    """A feed is missing or not positive."""


class ProgramError(WireSawError):
    """A written program breaks a structural rule (wire start before the first cut, wire stop at the end)."""


# =====================================================================================================================
# 2D geometry: profile of lines and arcs, offset
# =====================================================================================================================
@dataclass(frozen=True)
class Seg:
    """One profile segment ending at `end`. A line when `center` is None, else an arc about `center` (absolute)."""
    end: Vec2
    center: Optional[Vec2] = None
    ccw: bool = True


def _v(p) -> np.ndarray:
    return np.asarray(p, dtype=np.float64)


def _unit(v: np.ndarray) -> np.ndarray:
    n = float(np.hypot(v[0], v[1]))
    if n < 1e-15:
        raise WireSawError("zero-length segment")
    return v / n


def _cross2(a, b) -> float:
    return float(a[0] * b[1] - a[1] * b[0])


def _ang(v: np.ndarray) -> float:
    return math.atan2(v[1], v[0])


def _sweep(p0, p1, c, ccw: bool) -> float:
    a = _ang(_v(p1) - _v(c)) - _ang(_v(p0) - _v(c))
    if not ccw:
        a = -a
    return a % _TWO_PI


@dataclass
class Profile:
    """A path of lines and arcs in the wire plane (x = trolley, y = height), open or closed.

    Arcs must be shorter than a full turn. A closed profile ends where it starts.
    """
    start: Vec2
    segs: List[Seg]
    closed: bool = False

    def __post_init__(self):
        if not self.segs:
            raise WireSawError("a profile needs at least one segment")
        p = _v(self.start)
        for i, s in enumerate(self.segs):
            q = _v(s.end)
            if s.center is not None:
                if np.hypot(*(q - p)) < 1e-9:
                    raise WireSawError(f"segment {i}: an arc needs distinct end points; split a full circle in two")
                if abs(np.hypot(*(p - _v(s.center))) - np.hypot(*(q - _v(s.center)))) > 1e-6 * max(1.0, np.hypot(*(p - _v(s.center)))):
                    raise WireSawError(f"segment {i}: arc centre is not equidistant from its end points")
            elif np.hypot(*(q - p)) < 1e-9:
                raise WireSawError(f"segment {i}: zero-length line")
            p = q
        if self.closed and np.hypot(*(p - _v(self.start))) > 1e-6:
            raise WireSawError("a closed profile must end at its start point")

    # ---- constructors
    @staticmethod
    def from_points(pts: Sequence[Vec2], closed: bool = False) -> "Profile":
        pts = [tuple(map(float, p)) for p in pts]
        if closed and np.hypot(*(_v(pts[-1]) - _v(pts[0]))) > 1e-9:
            pts = pts + [pts[0]]
        elif closed:
            pts = pts[:-1] + [pts[0]]
        return Profile(pts[0], [Seg(p) for p in pts[1:]], closed)

    @staticmethod
    def from_bulge(pts: Sequence[Vec2], bulges: Sequence[float], closed: bool = False) -> "Profile":
        """Polyline with bulges (DXF convention). bulges[i] belongs to the segment from pts[i] to the next point
        (the last one only when closed). bulge = tan(included angle / 4); positive is counter-clockwise."""
        pts = [tuple(map(float, p)) for p in pts]
        n = len(pts)
        nseg = n if closed else n - 1
        if len(bulges) < nseg:
            bulges = list(bulges) + [0.0] * (nseg - len(bulges))
        segs = []
        for i in range(nseg):
            p, q, b = _v(pts[i]), _v(pts[(i + 1) % n]), float(bulges[i])
            if abs(b) < 1e-12:
                segs.append(Seg(tuple(q)))
                continue
            chord = float(np.hypot(*(q - p)))
            sag = abs(b) * chord / 2.0
            rad = (chord * chord / 4.0 + sag * sag) / (2.0 * sag)
            left = np.array([-(q - p)[1], (q - p)[0]]) / chord
            ctr = (p + q) / 2.0 + math.copysign(1.0, b) * left * (rad - sag)
            segs.append(Seg(tuple(q), tuple(ctr), b > 0))
        return Profile(pts[0], segs, closed)

    def rotated_start(self, i: int) -> "Profile":
        """Closed profile that starts at the start of segment i (to move the lead-in to a convex vertex)."""
        if not self.closed:
            raise WireSawError("rotated_start needs a closed profile")
        n = len(self.segs)
        starts = self.starts()
        segs = [self.segs[(i + k) % n] for k in range(n)]
        return Profile(starts[i % n], segs, True)

    # ---- queries
    def starts(self) -> List[Vec2]:
        out, p = [], self.start
        for s in self.segs:
            out.append(p)
            p = s.end
        return out

    def end(self) -> Vec2:
        return self.segs[-1].end

    def seg_length(self, i: int) -> float:
        p, s = _v(self.starts()[i]), self.segs[i]
        if s.center is None:
            return float(np.hypot(*(_v(s.end) - p)))
        return float(np.hypot(*(p - _v(s.center))) * _sweep(p, s.end, s.center, s.ccw))

    def length(self) -> float:
        return sum(self.seg_length(i) for i in range(len(self.segs)))

    def flatten(self, tol: float = 0.01, step: Optional[float] = None) -> np.ndarray:
        """Points along the path (start included). Arcs deviate from their chords by at most `tol`; every piece is
        at most `step` long when given."""
        pts = [_v(self.start)]
        for p, s in zip(self.starts(), self.segs):
            pts.extend(_flatten_seg(p, s, tol, step))
        return np.array(pts)

    def signed_area(self) -> float:
        pts = self.flatten(0.001)
        x, y = pts[:, 0], pts[:, 1]
        return 0.5 * float(np.sum(x[:-1] * y[1:] - x[1:] * y[:-1]))


def _arc_pieces(r: float, sweep: float, tol: float) -> int:
    if r <= tol:
        return 1
    da = 2.0 * math.acos(max(-1.0, min(1.0, 1.0 - tol / r)))
    return max(1, int(math.ceil(sweep / max(da, 1e-6))))


def _flatten_seg(p, s: Seg, tol: float, step: Optional[float] = None) -> List[np.ndarray]:
    """Points after the start p along segment s."""
    p, q = _v(p), _v(s.end)
    if s.center is None:
        n = 1 if step is None else max(1, int(math.ceil(float(np.hypot(*(q - p))) / step)))
        return [p + (q - p) * (k / n) for k in range(1, n + 1)]
    c = _v(s.center)
    r = float(np.hypot(*(p - c)))
    sw = _sweep(p, q, c, s.ccw)
    n = _arc_pieces(r, sw, tol)
    if step is not None:
        n = max(n, int(math.ceil(r * sw / step)))
    a0 = _ang(p - c)
    sgn = 1.0 if s.ccw else -1.0
    out = [c + r * np.array([math.cos(a0 + sgn * sw * k / n), math.sin(a0 + sgn * sw * k / n)]) for k in range(1, n)]
    out.append(q)
    return out


def _tan_at(p, s: Seg, at_end: bool) -> np.ndarray:
    p, q = _v(p), _v(s.end)
    if s.center is None:
        return _unit(q - p)
    c = _v(s.center)
    v = (q if at_end else p) - c
    t = np.array([-v[1], v[0]]) if s.ccw else np.array([v[1], -v[0]])
    return _unit(t)


class _El:
    """Mutable segment used while offsetting."""
    __slots__ = ("p0", "p1", "c", "ccw")

    def __init__(self, p0, p1, c=None, ccw=True):
        self.p0, self.p1 = _v(p0).copy(), _v(p1).copy()
        self.c = None if c is None else _v(c).copy()
        self.ccw = ccw

    def copy(self):
        return _El(self.p0, self.p1, self.c, self.ccw)

    @property
    def r(self) -> float:
        return float(np.hypot(*(self.p0 - self.c)))

    def tan(self, at_end: bool) -> np.ndarray:
        if self.c is None:
            return _unit(self.p1 - self.p0)
        v = (self.p1 if at_end else self.p0) - self.c
        return _unit(np.array([-v[1], v[0]]) if self.ccw else np.array([v[1], -v[0]]))

    def sweep(self) -> float:
        return _sweep(self.p0, self.p1, self.c, self.ccw)


def _offset_el(e: _El, d: float, side: int) -> _El:
    if e.c is None:
        t = e.tan(False)
        n = np.array([-t[1], t[0]]) * side * d
        return _El(e.p0 + n, e.p1 + n)
    r2 = e.r - side * d if e.ccw else e.r + side * d
    if r2 <= 1e-9:
        raise KerfError(f"an arc of radius {e.r:.3f} on the waste side collapses under an offset of {d:.3f}: "
                        "the profile has a concave arc tighter than the wire radius")
    k = r2 / e.r
    return _El(e.c + (e.p0 - e.c) * k, e.c + (e.p1 - e.c) * k, e.c, e.ccw)


def _ix_ll(a: _El, b: _El) -> List[np.ndarray]:
    ta, tb = a.tan(False), b.tan(False)
    den = _cross2(ta, tb)
    if abs(den) < 1e-12:
        return []
    s = _cross2(b.p0 - a.p0, tb) / den
    return [a.p0 + s * ta]


def _ix_lc(l: _El, c: _El) -> List[np.ndarray]:
    t = l.tan(False)
    f = l.p0 - c.c
    r = c.r
    b = float(np.dot(f, t))
    disc = b * b - (float(np.dot(f, f)) - r * r)
    if disc < 0:
        return []
    sq = math.sqrt(disc)
    return [l.p0 + (-b - sq) * t, l.p0 + (-b + sq) * t]


def _ix_cc(a: _El, b: _El) -> List[np.ndarray]:
    d = float(np.hypot(*(b.c - a.c)))
    ra, rb = a.r, b.r
    if d < 1e-12 or d > ra + rb or d < abs(ra - rb):
        return []
    x = (d * d + ra * ra - rb * rb) / (2 * d)
    h = math.sqrt(max(ra * ra - x * x, 0.0))
    u = (b.c - a.c) / d
    m = a.c + u * x
    n = np.array([-u[1], u[0]])
    return [m + n * h, m - n * h]


def _ix(a: _El, b: _El) -> List[np.ndarray]:
    if a.c is None and b.c is None:
        return _ix_ll(a, b)
    if a.c is None:
        return _ix_lc(a, b)
    if b.c is None:
        return _ix_lc(b, a)
    return _ix_cc(a, b)


def _on_el(e: _El, p: np.ndarray, tol: float) -> bool:
    if e.c is None:
        s = float(np.dot(p - e.p0, e.tan(False)))
        return -tol <= s <= float(np.hypot(*(e.p1 - e.p0))) + tol
    a = _sweep(e.p0, p, e.c, e.ccw)
    sw = e.sweep()
    te = tol / e.r
    return a <= sw + te or a >= _TWO_PI - te


def _slack(d: float) -> float:
    return 0.01 + 2e-3 * d


def offset_profile(profile: Profile, d: float, side: int, join: str = "round", mitre_limit: float = 2.0) -> Profile:
    """Profile offset by distance d to its left (side=+1) or right (side=-1) in the direction of travel.

    At gaps (outside corners) a round join (arc about the corner) or a mitre is inserted; a mitre longer than
    `mitre_limit` offsets falls back to a round join. At overlaps (inside corners) the offset segments are trimmed at
    their intersection. Raises KerfError when segments vanish or no intersection exists, and when the result crosses
    itself or comes closer to the profile than d (the wire would cut the part).
    """
    if d <= 0:
        raise WireSawError("offset distance must be positive")
    if side not in (1, -1):
        raise WireSawError("side must be +1 (left) or -1 (right)")
    starts = profile.starts()
    orig = [_El(p, s.end, s.center, s.ccw) for p, s in zip(starts, profile.segs)]
    n = len(orig)
    base = [_offset_el(e, d, side) for e in orig]
    els = [e.copy() for e in base]
    joins: Dict[int, List[_El]] = {}
    rng = range(n) if profile.closed else range(n - 1)
    for k in rng:
        k2 = (k + 1) % n
        v = orig[k].p1
        tin, tout = orig[k].tan(True), orig[k2].tan(False)
        cr, dt = _cross2(tin, tout), float(np.dot(tin, tout))
        if abs(cr) < 1e-7:
            if dt <= 0:
                raise KerfError(f"cusp at the end of segment {k}: the path reverses on itself")
            mid = (base[k].p1 + base[k2].p0) / 2.0
            els[k].p1, els[k2].p0 = mid.copy(), mid.copy()
        elif cr * side < 0:                                   # gap: outside corner
            a, b = base[k].p1, base[k2].p0
            if join == "mitre":
                den = _cross2(tin, tout)
                s = _cross2(b - a, tout) / den
                m = a + s * tin
                if float(np.hypot(*(m - v))) <= mitre_limit * d and s >= 0:
                    joins[k] = [_El(a, m), _El(m, b)]
                    continue
            joins[k] = [_El(a, b, v, cr > 0)]
        else:                                                 # overlap: inside corner
            cand = [p for p in _ix(base[k], base[k2]) if _on_el(base[k], p, 1e-6) and _on_el(base[k2], p, 1e-6)]
            if not cand:
                raise KerfError(f"offset segments {k} and {k2} do not meet: the profile is too small or too tight "
                                f"for a wire of radius {d:.3f}")
            p = min(cand, key=lambda q: float(np.hypot(*(q - v))))
            els[k].p1, els[k2].p0 = p.copy(), p.copy()
    for k, e in enumerate(els):                               # trimmed segments must keep their direction and sweep
        if e.c is None:
            if float(np.dot(e.p1 - e.p0, base[k].tan(False))) <= 1e-9:
                raise KerfError(f"segment {k} vanishes under an offset of {d:.3f}: it is shorter than the wire allows")
        else:
            sw = e.sweep()
            if sw <= 1e-9 or sw > base[k].sweep() + 1e-9:
                raise KerfError(f"arc segment {k} vanishes under an offset of {d:.3f}")
    chain: List[_El] = []
    for k in range(n):
        chain.append(els[k])
        chain.extend(joins.get(k, []))
    segs = [Seg((float(e.p1[0]), float(e.p1[1])), None if e.c is None else (float(e.c[0]), float(e.c[1])), e.ccw)
            for e in chain]
    out = Profile((float(chain[0].p0[0]), float(chain[0].p0[1])), segs, profile.closed)
    _check_offset(out, profile, d)
    return out


def _seg_dist(points: np.ndarray, poly: np.ndarray) -> np.ndarray:
    """Distance of each point to a polyline."""
    a, b = poly[:-1], poly[1:]
    ab = b - a
    den = np.maximum(np.sum(ab * ab, axis=1), 1e-18)
    out = np.empty(len(points))
    for i in range(0, len(points), 256):
        p = points[i:i + 256]
        ap = p[:, None, :] - a[None]
        t = np.clip(np.sum(ap * ab[None], axis=2) / den[None], 0.0, 1.0)
        proj = a[None] + t[..., None] * ab[None]
        out[i:i + 256] = np.min(np.hypot(*(p[:, None, :] - proj).transpose(2, 0, 1)), axis=1)
    return out


def _self_intersects(pts: np.ndarray, closed: bool) -> bool:
    m = len(pts) - 1
    for i in range(m - 1):
        a, b = pts[i], pts[i + 1]
        j0 = i + 2
        j1 = m - 1 if (closed and i == 0) else m
        if j0 >= j1:
            continue
        c, d = pts[j0:j1], pts[j0 + 1:j1 + 1]
        ab = b - a
        d1 = ab[0] * (c[:, 1] - a[1]) - ab[1] * (c[:, 0] - a[0])
        d2 = ab[0] * (d[:, 1] - a[1]) - ab[1] * (d[:, 0] - a[0])
        cd = d - c
        d3 = cd[:, 0] * (a[1] - c[:, 1]) - cd[:, 1] * (a[0] - c[:, 0])
        d4 = cd[:, 0] * (b[1] - c[:, 1]) - cd[:, 1] * (b[0] - c[:, 0])
        if np.any((d1 * d2 < -1e-12) & (d3 * d4 < -1e-12)):
            return True
    return False


def _check_offset(path: Profile, nominal: Profile, d: float) -> None:
    pts = path.flatten(0.01)
    if _self_intersects(pts, path.closed):
        raise KerfError("the offset path crosses itself: the profile has detail narrower than the wire diameter "
                        f"(wire radius {d:.3f}); simplify the profile or use a thinner wire")
    _check_clearance(path.flatten(0.01, step=max(d / 2.0, 0.5)), nominal, d, "the offset path")


def _check_clearance(points: np.ndarray, nominal: Profile, d: float, what: str) -> None:
    dist = _seg_dist(points, nominal.flatten(0.005))
    if float(dist.min()) < d - _slack(d):
        raise KerfError(f"{what} comes within {float(dist.min()):.3f} mm of the profile, closer than the wire radius "
                        f"{d:.3f} mm: the wire would cut the part")


# =====================================================================================================================
# machine definition
# =====================================================================================================================
@dataclass
class MachineDef:
    """Everything the exporter needs to know about one machine. Values in the examples are placeholders."""
    name: str = "generic"
    layout: str = "xy"                                        # "xy" | "xyc" | "xyuv"
    axes: Dict[str, str] = field(default_factory=lambda: {"trolley": "X", "height": "Y"})
    limits: Dict[str, Tuple[float, float]] = field(default_factory=dict)   # logical axis -> (lo, hi); absent = free
    table_unwind: str = "continuous"                          # "continuous" | "wrap360" | "rewind"
    table_axis_trolley: float = 0.0                           # trolley position of the table axis (coordinated turning)
    feed_mode: str = "G94"                                    # "G94" mm/min, "G93" inverse time
    rotary_feed_deg_min: Optional[float] = None               # table speed for G93 and for the time estimate
    arc_plane: Optional[str] = None                           # "G17" | "G18" | "G19", or None to flatten arcs
    arc_first: str = "trolley"                                # first axis of the arc plane (logical)
    arc_second: str = "height"
    arc_words: Dict[str, str] = field(default_factory=lambda: {"trolley": "I", "height": "J"})   # centre offset words
    chord_tol: float = 0.01                                   # chord tolerance when arcs are flattened
    kerf_mode: str = "cam"                                    # "cam" | "g41"
    d_register: int = 1
    wire_diameter: float = 8.0                                # placeholder
    offset_join: str = "round"                                # "round" | "mitre"
    mitre_limit: float = 2.0
    m_wire_start: int = 3
    m_wire_stop: int = 5
    m_coolant_on: Optional[int] = 8
    m_coolant_off: Optional[int] = 9
    m_program_end: Optional[int] = 30
    header: List[str] = field(default_factory=list)
    footer: List[str] = field(default_factory=list)
    line_numbers: int = 0                                     # step between N words, 0 = none
    max_lines: Optional[int] = None                           # split into files of at most this many lines
    decimals: int = 3
    corner_dwell_s: float = 0.0                               # G4 P at corners, 0 = off
    dwell_in_ms: bool = False                                 # P in milliseconds instead of seconds
    corner_angle_deg: float = 20.0                            # turn that counts as a corner
    corner_slowdown_factor: float = 1.0                       # feed factor near corners, 1 = off
    corner_slowdown_length: float = 10.0                      # mm either side of a corner
    lead_in: float = 50.0
    lead_out: float = 50.0
    carriage_spacing: Optional[float] = None                  # distance between the two wire-end carriages (xyuv)
    max_wire_tilt_deg: Optional[float] = None                 # largest wire tilt from horizontal (xyuv)
    notes: str = ""

    # ---- JSON
    def to_dict(self) -> dict:
        return asdict(self)

    @staticmethod
    def from_dict(d: dict) -> "MachineDef":
        known = {f.name for f in fields(MachineDef)}
        bad = set(d) - known
        if bad:
            raise WireSawError(f"unknown machine definition field(s): {sorted(bad)}")
        d = dict(d)
        if "limits" in d:
            d["limits"] = {k: (float(v[0]), float(v[1])) for k, v in d["limits"].items()}
        m = MachineDef(**d)
        m.validate()
        return m

    def save(self, path) -> None:
        Path(path).write_text(json.dumps(self.to_dict(), indent=2) + "\n", encoding="utf-8")

    @staticmethod
    def load(path) -> "MachineDef":
        return MachineDef.from_dict(json.loads(Path(path).read_text(encoding="utf-8")))

    # ---- helpers
    @property
    def wire_radius(self) -> float:
        return self.wire_diameter / 2.0

    def has(self, axis: str) -> bool:
        return axis in self.axes

    def validate(self) -> None:
        if self.layout not in ("xy", "xyc", "xyuv"):
            raise WireSawError(f"layout must be xy, xyc or xyuv, not {self.layout!r}")
        need = {"xy": ("trolley", "height"), "xyc": ("trolley", "height", "table"),
                "xyuv": ("trolley", "height", "height2")}[self.layout]
        for a in need:
            if a not in self.axes:
                raise WireSawError(f"layout {self.layout} needs the logical axis {a!r} in the axis map")
        for a, letter in self.axes.items():
            if a not in LOGICAL:
                raise WireSawError(f"unknown logical axis {a!r}; use {LOGICAL}")
            if not (isinstance(letter, str) and len(letter) == 1 and letter.isalpha() and letter.isupper()):
                raise WireSawError(f"axis letter for {a} must be one capital letter, got {letter!r}")
            if letter in _RESERVED:
                raise WireSawError(f"axis letter {letter} for {a} clashes with a program word ({''.join(sorted(_RESERVED))})")
        if len(set(self.axes.values())) != len(self.axes):
            raise WireSawError("two logical axes share a letter")
        for a in self.limits:
            if a not in self.axes:
                raise WireSawError(f"limit for {a!r}, which is not an axis of this machine")
        if self.table_unwind not in ("continuous", "wrap360", "rewind"):
            raise WireSawError("table_unwind must be continuous, wrap360 or rewind")
        if self.feed_mode not in ("G94", "G93"):
            raise WireSawError("feed_mode must be G94 or G93")
        if self.kerf_mode not in ("cam", "g41"):
            raise WireSawError("kerf_mode must be cam or g41")
        if self.offset_join not in ("round", "mitre"):
            raise WireSawError("offset_join must be round or mitre")
        if self.wire_diameter <= 0:
            raise WireSawError("wire_diameter must be positive")
        if self.arc_plane is not None:
            if self.arc_plane not in ("G17", "G18", "G19"):
                raise WireSawError("arc_plane must be G17, G18, G19 or null")
            if {self.arc_first, self.arc_second} != {"trolley", "height"}:
                raise WireSawError("arc_first and arc_second must be trolley and height in some order")
            for a in ("trolley", "height"):
                if self.arc_words.get(a) not in ("I", "J", "K"):
                    raise WireSawError(f"arc_words needs I, J or K for {a}")
        if self.decimals < 0 or self.decimals > 8:
            raise WireSawError("decimals must be 0 to 8")
        if self.max_lines is not None and self.max_lines < 20:
            raise WireSawError("max_lines must be at least 20")
        if self.line_numbers < 0:
            raise WireSawError("line_numbers must be >= 0")
        if self.corner_slowdown_factor <= 0 or self.corner_slowdown_factor > 1:
            raise WireSawError("corner_slowdown_factor must be in (0, 1]")
        if self.lead_in < 0 or self.lead_out < 0:
            raise WireSawError("lead lengths must be >= 0")

    def letter(self, axis: str) -> str:
        return self.axes[axis]

    def axis_order(self) -> List[str]:
        return [a for a in LOGICAL if a in self.axes]


# =====================================================================================================================
# job: setups
# =====================================================================================================================
@dataclass
class Setup2D:
    """A plane profile in the wire plane (x = trolley, y = height). On xyc, `table_deg` is the fixed table angle.

    `waste` is "outside" or "inside" (closed profiles: the part is on the other side) or "left" / "right" of the
    direction of travel.
    """
    profile: Profile
    table_deg: float = 0.0
    waste: str = "outside"
    name: str = ""
    feed: Optional[float] = None
    lead_in: Optional[float] = None
    lead_out: Optional[float] = None


@dataclass
class CoordSetup:
    """Coordinated turning on xyc. `profile` is in (s, z): radius from the table axis and height.

    `waste` is "+s" (outside turning: material beyond the profile is waste) or "-s". The table turns `deg_per_mm`
    degrees per mm of path length, from `theta0`, and never more than `max_step_deg` in one block.
    """
    profile: Profile
    deg_per_mm: float
    waste: str = "+s"
    theta0: float = 0.0
    max_step_deg: float = 5.0
    name: str = ""
    feed: Optional[float] = None
    lead_in: Optional[float] = None
    lead_out: Optional[float] = None


@dataclass
class RuledSetup:
    """Wire positions for a ruled surface on xyuv: rows (x0, h0, x1, h1), the wire-centre positions already offset by
    the wire radius (see `ruled_face_setup`). x0, h0 belong to carriage 0 and x1, h1 to carriage 1."""
    samples: np.ndarray
    name: str = ""
    feed: Optional[float] = None
    lead_in: Optional[float] = None
    lead_out: Optional[float] = None

    def __post_init__(self):
        self.samples = np.asarray(self.samples, dtype=np.float64).reshape(-1, 4)


Setup = Union[Setup2D, CoordSetup, RuledSetup]


@dataclass
class WireJob:
    name: str = "job"
    setups: List[Setup] = field(default_factory=list)
    feed: Optional[float] = None          # mm/min across the stone; a setup can override. There is no default.


# =====================================================================================================================
# plan: a neutral sequence of moves and events
# =====================================================================================================================
@dataclass
class Move:
    kind: str                                   # "rapid" | "line" | "arc"
    pos: Dict[str, float]                       # logical axis -> absolute end value (only the axes this block moves)
    start: Dict[str, float]
    center: Optional[Vec2] = None               # arcs: centre in (trolley, height)
    ccw: bool = True
    feed: Optional[float] = None                # mm/min of the cut; None for rapids
    comp: Optional[str] = None                  # "G41", "G42" or "G40" on this block
    setup: int = 0
    role: str = "cut"                           # "position", "lead_in", "cut", "lead_out"
    length: float = 0.0                         # mm in the wire plane (longest carriage path for xyuv)
    angle: float = 0.0                          # table degrees moved


@dataclass
class Event:
    name: str                                   # wire_start wire_stop coolant_on coolant_off dwell comment program_end
    value: float = 0.0
    text: str = ""
    setup: int = 0


@dataclass
class PathRec:
    layer: str
    kind: str                                   # "wire_centre" | "programmed"
    profile: Profile
    setup: int = 0


@dataclass
class Plan:
    machine: MachineDef
    name: str
    items: List[Union[Move, Event]] = field(default_factory=list)
    paths: List[PathRec] = field(default_factory=list)

    def moves(self) -> List[Move]:
        return [i for i in self.items if isinstance(i, Move)]

    def cut_moves(self) -> List[Move]:
        return [m for m in self.moves() if m.kind != "rapid"]

    def modal_positions(self) -> List[Dict[str, float]]:
        """Full axis positions after every move (axes not moved yet are absent)."""
        cur: Dict[str, float] = {}
        out = []
        for m in self.moves():
            cur = {**cur, **m.pos}
            out.append(dict(cur))
        return out

    def path_length(self) -> float:
        return sum(m.length for m in self.cut_moves())

    def cut_minutes(self) -> float:
        mc = self.machine
        t = 0.0
        for m in self.cut_moves():
            t += _move_minutes(m, mc)
        for it in self.items:
            if isinstance(it, Event) and it.name == "dwell":
                t += it.value / 60.0
        return t

    def summary(self) -> dict:
        return {"moves": len(self.moves()), "cut_moves": len(self.cut_moves()),
                "path_length_mm": self.path_length(), "cut_minutes": self.cut_minutes(),
                "kerf_mode": self.machine.kerf_mode, "layout": self.machine.layout}


def _move_minutes(m: Move, mc: MachineDef) -> float:
    t = 0.0
    if m.feed and m.feed > 0:
        t = m.length / m.feed
    if m.angle > 0 and mc.rotary_feed_deg_min:
        t = max(t, m.angle / mc.rotary_feed_deg_min)
    return t


def _pos(mc: MachineDef, x: float, y: float, table: Optional[float] = None) -> Dict[str, float]:
    p = {"trolley": x, "height": y}
    if mc.layout == "xyc":
        p["table"] = 0.0 if table is None else table
    if mc.layout == "xyuv":
        p["height2"] = y
        if mc.has("trolley2"):
            p["trolley2"] = x
    return p


def _lengths(mc: MachineDef, kind: str, start: Dict[str, float], pos: Dict[str, float], center, ccw) -> Tuple[float, float]:
    ang = abs(pos["table"] - start["table"]) if "table" in pos and "table" in start else 0.0
    if "trolley" not in pos or "height" not in pos:
        return 0.0, ang
    s = np.array([start.get("trolley", pos["trolley"]), start.get("height", pos["height"])])
    e = np.array([pos["trolley"], pos["height"]])
    if kind == "arc":
        c = _v(center)
        return float(np.hypot(*(s - c)) * _sweep(s, e, c, ccw)), ang
    if mc.layout == "xyuv" and "height2" in pos:
        t0 = start.get("trolley2", start.get("trolley", pos["trolley"]))
        t1 = pos.get("trolley2", pos["trolley"])
        l0 = float(np.hypot(*(e - s)))
        l1 = float(np.hypot(t1 - t0, pos["height2"] - start.get("height2", pos["height2"])))
        return max(l0, l1), ang
    return float(np.hypot(*(e - s))), ang


class _Builder:
    def __init__(self, mc: MachineDef, name: str):
        self.mc = mc
        self.plan = Plan(mc, name)
        self.cur: Dict[str, float] = {}
        self.wire_on = False
        self.table_ref: Optional[float] = None

    def event(self, name: str, value: float = 0.0, text: str = "", setup: int = 0):
        self.plan.items.append(Event(name, value, text, setup))

    def move(self, kind, pos, feed=None, comp=None, setup=0, role="cut", center=None, ccw=True):
        start = {a: self.cur.get(a, v) for a, v in pos.items()}
        ln, ang = (0.0, 0.0)
        if kind != "rapid":
            ln, ang = _lengths(self.mc, kind, {**self.cur, **start}, pos, center, ccw)
        self.plan.items.append(Move(kind, dict(pos), start, center, ccw, feed, comp, setup, role, ln, ang))
        self.cur.update(pos)

    def wire(self, on: bool, setup: int):
        if on and not self.wire_on:
            self.event("wire_start", setup=setup)
        if not on and self.wire_on:
            self.event("wire_stop", setup=setup)
        self.wire_on = on

    def table_target(self, deg: float) -> float:
        """Written table angle equivalent to `deg`, by the unwinding rule and limits."""
        mc = self.mc
        if mc.table_unwind == "wrap360":
            return deg % 360.0
        ref = self.cur.get("table", 0.0)
        lim = mc.limits.get("table")
        best = None
        for k in range(-40, 41):
            a = deg % 360.0 + 360.0 * k
            if lim and not (lim[0] - 1e-9 <= a <= lim[1] + 1e-9):
                continue
            if best is None or abs(a - ref) < abs(best - ref) - 1e-9:
                best = a
        if best is None:
            raise TravelLimitError(f"table angle {deg:.3f} has no equivalent inside the table limits {lim}")
        return best


def _side_of(profile: Profile, waste: str) -> int:
    if waste == "left":
        return 1
    if waste == "right":
        return -1
    if waste in ("outside", "inside"):
        if not profile.closed:
            raise WireSawError("'outside' and 'inside' need a closed profile; use 'left' or 'right' for an open one")
        ccw = profile.signed_area() > 0
        out_side = -1 if ccw else 1                    # waste outside: right of a ccw loop
        return out_side if waste == "outside" else -out_side
    raise WireSawError(f"waste must be outside, inside, left or right, not {waste!r}")


def _feed_of(s, job: WireJob, idx: int) -> float:
    f = s.feed if s.feed is not None else job.feed
    if f is None or not (f > 0):
        raise FeedError(f"setup {idx} ({getattr(s, 'name', '') or 'unnamed'}): feed is missing or not positive; "
                        "this exporter has no default feed, set WireJob.feed or the setup feed in mm/min")
    return float(f)


def _extend(profile: Profile, lead_in: float, lead_out: float) -> Tuple[np.ndarray, Optional[np.ndarray]]:
    """(lead-in start point or None, lead-out end point or None) of the tangent leads."""
    s0 = _v(profile.start)
    t0 = _tan_at(profile.start, profile.segs[0], False)
    starts = profile.starts()
    tn = _tan_at(starts[-1], profile.segs[-1], True)
    li = s0 - t0 * lead_in if lead_in > 0 else None
    lo = None
    if lead_out > 0:
        if profile.closed:
            lo = li                                   # a closed profile leaves through its lead-in slot
        else:
            lo = _v(profile.end()) + tn * lead_out
    return li, lo


def _lead_clear(profile: Profile, nominal: Profile, r: float, li, lo, which: str) -> None:
    for pt, base_pt in ((li, profile.start), (lo, profile.end())):
        if pt is None:
            continue
        a, b = _v(base_pt), _v(pt)
        n = max(20, int(float(np.hypot(*(b - a))) / max(r / 2.0, 0.5)))
        pts = np.array([a + (b - a) * (k / n) for k in range(n + 1)])
        _check_clearance(pts, nominal, r, f"the {which} lead line")


def _corner_flags(profile: Profile, mc: MachineDef) -> List[Tuple[bool, bool]]:
    n = len(profile.segs)
    flags = [[False, False] for _ in range(n)]
    if mc.corner_dwell_s <= 0 and mc.corner_slowdown_factor >= 1.0:
        return [tuple(f) for f in flags]
    st = profile.starts()
    for k in range(n - 1):
        tin = _tan_at(st[k], profile.segs[k], True)
        tout = _tan_at(st[k + 1], profile.segs[k + 1], False)
        turn = abs(math.degrees(math.atan2(_cross2(tin, tout), float(np.dot(tin, tout)))))
        if turn > mc.corner_angle_deg:
            flags[k][1] = True
            flags[k + 1][0] = True
    return [tuple(f) for f in flags]


def _sub_seg(p, s: Seg, a: float, b: float) -> Tuple[np.ndarray, Seg]:
    """Portion of a segment between arc-length a and b: (start point, segment)."""
    p, q = _v(p), _v(s.end)
    if s.center is None:
        L = float(np.hypot(*(q - p)))
        u = (q - p) / L
        return p + u * a, Seg(tuple(p + u * b))
    c = _v(s.center)
    r = float(np.hypot(*(p - c)))
    a0 = _ang(p - c)
    sg = 1.0 if s.ccw else -1.0
    pa = c + r * np.array([math.cos(a0 + sg * a / r), math.sin(a0 + sg * a / r)])
    pb = c + r * np.array([math.cos(a0 + sg * b / r), math.sin(a0 + sg * b / r)])
    return pa, Seg((float(pb[0]), float(pb[1])), s.center, s.ccw)


def _emit_profile_moves(bd: _Builder, prof: Profile, feed: float, setup: int, table: Optional[float]) -> None:
    mc = bd.mc
    flags = _corner_flags(prof, mc)
    slow_len = mc.corner_slowdown_length
    fac = mc.corner_slowdown_factor
    st = prof.starts()
    for i, s in enumerate(prof.segs):
        L = prof.seg_length(i)
        cs, ce = flags[i]
        cuts = [0.0, L]
        if fac < 1.0:
            if cs and slow_len < L:
                cuts.append(slow_len)
            if ce and slow_len < L:
                cuts.append(L - slow_len)
        cuts = sorted(set(round(c, 9) for c in cuts))
        for a, b in zip(cuts[:-1], cuts[1:]):
            if b - a < 1e-9:
                continue
            mid = 0.5 * (a + b)
            f = feed * (fac if fac < 1.0 and ((cs and mid < slow_len) or (ce and L - mid < slow_len)) else 1.0)
            p0, sub = _sub_seg(st[i], s, a, b)
            if sub.center is not None and mc.arc_plane is None:
                for pt in _flatten_seg(p0, sub, mc.chord_tol):
                    bd.move("line", _pos(mc, float(pt[0]), float(pt[1]), table), feed=f, setup=setup)
            elif sub.center is not None:
                bd.move("arc", _pos(mc, sub.end[0], sub.end[1], table), feed=f, setup=setup,
                        center=sub.center, ccw=sub.ccw)
            else:
                bd.move("line", _pos(mc, sub.end[0], sub.end[1], table), feed=f, setup=setup)
        if ce and mc.corner_dwell_s > 0:
            bd.event("dwell", mc.corner_dwell_s, setup=setup)


def _path_with_leads(prof: Profile, li, lo) -> Profile:
    segs: List[Seg] = []
    start = prof.start
    if li is not None:
        start = (float(li[0]), float(li[1]))
        segs.append(Seg(prof.start))
    segs.extend(prof.segs)
    if lo is not None:
        segs.append(Seg((float(lo[0]), float(lo[1]))))
    return Profile(start, segs, False)


def _plan_profile(bd: _Builder, s: Setup2D, idx: int, feed: float) -> None:
    mc = bd.mc
    if mc.layout == "xy" and abs(s.table_deg) > 1e-12:
        raise WireSawError(f"setup {idx}: table_deg given for a machine without a table axis (layout xy)")
    r = mc.wire_radius
    side = _side_of(s.profile, s.waste)
    centre = offset_profile(s.profile, r, side, mc.offset_join, mc.mitre_limit)   # always checked
    comp = None
    prog = centre
    if mc.kerf_mode == "g41":
        comp = "G41" if side > 0 else "G42"
        prog = s.profile
    lin = mc.lead_in if s.lead_in is None else s.lead_in
    lout = mc.lead_out if s.lead_out is None else s.lead_out
    if comp and lin < r - 1e-9:
        raise WireSawError(f"setup {idx}: lead-in of {lin:g} mm is shorter than the wire radius {r:g} mm and cannot "
                           "activate G41/G42; raise lead_in")
    if comp and lout < r - 1e-9:
        raise WireSawError(f"setup {idx}: lead-out of {lout:g} mm is shorter than the wire radius {r:g} mm and cannot "
                           "cancel G41/G42; raise lead_out")
    cli, clo = _extend(centre, lin, lout)
    _lead_clear(centre, s.profile, r, cli, clo, "lead")
    li, lo = _extend(prog, lin, lout)
    # table and positioning
    table = None
    if mc.layout == "xyc":
        table = bd.table_target(s.table_deg)
        if abs(table - bd.cur.get("table", float("nan"))) > 1e-9 or "table" not in bd.cur:
            bd.wire(False, idx)
            bd.move("rapid", {"table": table}, setup=idx, role="position")
    first = li if li is not None else _v(prog.start)
    bd.wire(False, idx)
    bd.move("rapid", _pos(mc, float(first[0]), float(first[1]), table), setup=idx, role="position")
    bd.wire(True, idx)
    if li is not None:
        bd.move("line", _pos(mc, prog.start[0], prog.start[1], table), feed=feed, comp=comp, setup=idx, role="lead_in")
    _emit_profile_moves(bd, prog, feed, idx, table)
    if lo is not None:
        bd.move("line", _pos(mc, float(lo[0]), float(lo[1]), table), feed=feed, comp="G40" if comp else None,
                setup=idx, role="lead_out")
    bd.wire(False, idx)
    label = _layer_name(idx, s.name)
    bd.plan.paths.append(PathRec(label, "wire_centre", _path_with_leads(centre, cli, clo), idx))
    if comp:
        bd.plan.paths.append(PathRec(label + "_PROG", "programmed", _path_with_leads(prog, li, lo), idx))


def _layer_name(idx: int, name: str) -> str:
    base = re.sub(r"[^A-Za-z0-9_]", "_", name).upper() if name else ""
    return f"S{idx + 1}" + (f"_{base}" if base else "")


def _plan_coord(bd: _Builder, s: CoordSetup, idx: int, feed: float) -> None:
    mc = bd.mc
    if mc.layout != "xyc":
        raise WireSawError(f"setup {idx}: coordinated turning needs layout xyc")
    if mc.kerf_mode == "g41":
        raise WireSawError(f"setup {idx}: coordinated turning needs kerf_mode cam; G41/G42 is only used for plane "
                           "profiles")
    if s.deg_per_mm == 0:
        raise WireSawError(f"setup {idx}: deg_per_mm must not be 0")
    if s.waste not in ("+s", "-s"):
        raise WireSawError("waste must be '+s' or '-s' for turned profiles")
    r = mc.wire_radius
    dz = s.profile.end()[1] - s.profile.start[1]
    if abs(dz) < 1e-9:
        raise WireSawError(f"setup {idx}: a turned profile must change height from start to end")
    side = (-1 if dz > 0 else 1) * (1 if s.waste == "+s" else -1)         # travelling up: +s is on the right
    centre = offset_profile(s.profile, r, side, mc.offset_join, mc.mitre_limit)
    lin = mc.lead_in if s.lead_in is None else s.lead_in
    lout = mc.lead_out if s.lead_out is None else s.lead_out
    if s.profile.closed:
        raise WireSawError("a turned profile must be open")
    li, lo = _extend(centre, lin, lout)
    _lead_clear(centre, s.profile, r, li, lo, "lead-in")
    full = _path_with_leads(centre, li, lo)
    pts = full.flatten(mc.chord_tol)
    seglen = np.hypot(*np.diff(pts, axis=0).T)
    # table angle per point; split moves so that no block turns the table more than max_step_deg
    th = s.theta0 + s.deg_per_mm * np.concatenate([[0.0], np.cumsum(seglen)])
    P: List[Tuple[float, float, float]] = [(pts[0, 0], pts[0, 1], th[0])]
    for k in range(1, len(pts)):
        n = max(1, int(math.ceil(abs(th[k] - th[k - 1]) / s.max_step_deg - 1e-12)))
        for j in range(1, n + 1):
            f = j / n
            P.append((pts[k - 1, 0] + (pts[k, 0] - pts[k - 1, 0]) * f, pts[k - 1, 1] + (pts[k, 1] - pts[k - 1, 1]) * f,
                      th[k - 1] + (th[k] - th[k - 1]) * f))
    if mc.table_unwind == "wrap360":
        lo_a, hi_a = min(p[2] for p in P), max(p[2] for p in P)
        if hi_a - lo_a >= 360.0 - 1e-9 or math.floor(lo_a / 360.0) != math.floor(hi_a / 360.0):
            raise WireSawError(f"setup {idx}: the table turns {hi_a - lo_a:.1f} degrees or crosses a multiple of 360 "
                               "inside a cut; set table_unwind to continuous or rewind for turning")
    if mc.table_unwind != "wrap360":
        shift = bd.table_target(P[0][2]) - P[0][2]
    else:
        shift = -360.0 * math.floor(min(p[2] for p in P) / 360.0)
    x0 = mc.table_axis_trolley
    bd.wire(False, idx)
    bd.move("rapid", {"table": P[0][2] + shift}, setup=idx, role="position")
    bd.move("rapid", _pos(mc, x0 + P[0][0], P[0][1], P[0][2] + shift), setup=idx, role="position")
    bd.wire(True, idx)
    for k, (x, z, t) in enumerate(P[1:], start=1):
        role = "lead_in" if k == 1 and li is not None else "cut"
        bd.move("line", _pos(mc, x0 + x, z, t + shift), feed=feed, setup=idx, role=role)
    bd.wire(False, idx)
    pr = Profile.from_points([(x0 + x, z) for x, z, _ in P])
    bd.plan.paths.append(PathRec(_layer_name(idx, s.name), "wire_centre", pr, idx))


def _plan_ruled(bd: _Builder, s: RuledSetup, idx: int, feed: float) -> None:
    mc = bd.mc
    if mc.layout != "xyuv":
        raise WireSawError(f"setup {idx}: ruled surfaces need layout xyuv")
    if mc.kerf_mode == "g41":
        raise WireSawError(f"setup {idx}: ruled surfaces need kerf_mode cam (the offset is applied to every ruling)")
    S = s.samples
    if len(S) < 2:
        raise WireSawError(f"setup {idx}: a ruled path needs at least two rulings")
    if not mc.has("trolley2"):
        if float(np.max(np.abs(S[:, 0] - S[:, 2]))) > 0.05:
            raise WireSawError(f"setup {idx}: the wire is not in a plane of constant trolley position (ends differ by "
                               f"{float(np.max(np.abs(S[:, 0] - S[:, 2]))):.3f} mm) and the machine has no second "
                               "trolley axis; re-clamp the part or use a machine definition with trolley2")
    d0, d1 = np.diff(S[:, 0]), np.diff(S[:, 2])
    sgn = np.sign(np.sum(d0) + np.sum(d1))
    if sgn == 0:
        raise WireSawError(f"setup {idx}: the rulings do not advance along the trolley axis")
    if np.any(d0 * sgn < -1e-9) or np.any(d1 * sgn < -1e-9) or np.any((np.abs(d0) <= 1e-9) & (np.abs(d1) <= 1e-9)):
        bad = int(np.argmax((d0 * sgn < -1e-9) | (d1 * sgn < -1e-9) | ((np.abs(d0) <= 1e-9) & (np.abs(d1) <= 1e-9))))
        raise WireSawError(f"setup {idx}: rulings are not monotone in trolley position (between rulings {bad} and "
                           f"{bad + 1}); a wire cannot go back along the trolley")
    if mc.max_wire_tilt_deg is not None:
        if mc.carriage_spacing is None:
            raise WireSawError("max_wire_tilt_deg needs carriage_spacing in the machine definition")
        tilt = np.degrees(np.arctan2(np.abs(S[:, 3] - S[:, 1]), mc.carriage_spacing))
        if float(tilt.max()) > mc.max_wire_tilt_deg + 1e-9:
            raise TravelLimitError(f"setup {idx}: wire tilt {float(tilt.max()):.2f} degrees exceeds the machine limit "
                                   f"{mc.max_wire_tilt_deg:g}")
    lin = mc.lead_in if s.lead_in is None else s.lead_in
    lout = mc.lead_out if s.lead_out is None else s.lead_out
    rows = [tuple(r) for r in S]
    if lin > 0:
        rows.insert(0, (S[0, 0] - sgn * lin, S[0, 1], S[0, 2] - sgn * lin, S[0, 3]))
    if lout > 0:
        rows.append((S[-1, 0] + sgn * lout, S[-1, 1], S[-1, 2] + sgn * lout, S[-1, 3]))

    def pos(r):
        p = {"trolley": float(r[0]), "height": float(r[1]), "height2": float(r[3])}
        if mc.has("trolley2"):
            p["trolley2"] = float(r[2])
        return p

    bd.wire(False, idx)
    bd.move("rapid", pos(rows[0]), setup=idx, role="position")
    bd.wire(True, idx)
    for k, r in enumerate(rows[1:], start=1):
        role = "lead_in" if (k == 1 and lin > 0) else ("lead_out" if (k == len(rows) - 1 and lout > 0) else "cut")
        bd.move("line", pos(r), feed=feed, setup=idx, role=role)
    bd.wire(False, idx)
    lay = _layer_name(idx, s.name)
    a = np.array(rows)
    bd.plan.paths.append(PathRec(lay + "_END0", "wire_centre", Profile.from_points(list(zip(a[:, 0], a[:, 1]))), idx))
    bd.plan.paths.append(PathRec(lay + "_END1", "wire_centre", Profile.from_points(list(zip(a[:, 2], a[:, 3]))), idx))


def check_limits(plan: Plan) -> None:
    mc = plan.machine
    for m in plan.moves():
        for ax, v in m.pos.items():
            lim = mc.limits.get(ax)
            if lim and not (lim[0] - 1e-9 <= v <= lim[1] + 1e-9):
                raise TravelLimitError(f"axis {ax} ({mc.letter(ax)}) position {v:.3f} is outside the travel "
                                       f"[{lim[0]:g}, {lim[1]:g}] (setup {m.setup + 1}, {m.role})")


def plan_job(job: WireJob, machine: MachineDef) -> Plan:
    """Plan a job for a machine: moves and events, kerf, leads, limits. Raises WireSawError subclasses on any refusal."""
    machine.validate()
    if not job.setups:
        raise WireSawError("the job has no setups")
    bd = _Builder(machine, job.name)
    if machine.m_coolant_on is not None:
        bd.event("coolant_on")
    for idx, s in enumerate(job.setups):
        feed = _feed_of(s, job, idx)
        if isinstance(s, Setup2D):
            if machine.layout == "xyuv" and abs(s.table_deg) > 1e-12:
                raise WireSawError("table_deg on an xyuv machine")
            _plan_profile(bd, s, idx, feed)
        elif isinstance(s, CoordSetup):
            _plan_coord(bd, s, idx, feed)
        elif isinstance(s, RuledSetup):
            _plan_ruled(bd, s, idx, feed)
        else:
            raise WireSawError(f"setup {idx}: unknown setup type {type(s).__name__}")
    if machine.layout == "xyc" and machine.table_unwind == "rewind" and "table" in bd.cur:
        t = bd.cur["table"]
        if abs(t - (t % 360.0)) > 1e-9:
            bd.move("rapid", {"table": t % 360.0}, setup=len(job.setups) - 1, role="position")
    if machine.m_coolant_off is not None:
        bd.event("coolant_off")
    if machine.m_program_end is not None:
        bd.event("program_end")
    check_limits(bd.plan)
    return bd.plan


# =====================================================================================================================
# G-code writer
# =====================================================================================================================
def _num(v: float, dec: int) -> str:
    s = f"{v:.{dec}f}"
    if s.startswith("-") and float(s) == 0.0:
        s = s[1:]
    return s


def _fnum(v: float, dec: int) -> str:
    s = f"{v:.{dec}f}"
    return s.rstrip("0").rstrip(".") if "." in s else s


def _arc_g3(mc: MachineDef, ccw: bool) -> bool:
    return ccw if (mc.arc_first, mc.arc_second) == ("trolley", "height") else not ccw


@dataclass
class _Item:
    lines: List[str]
    end: Dict[str, float]
    wire_on: bool
    comp_on: bool


def _comment(text: str) -> str:
    return "(" + text.replace("(", "[").replace(")", "]") + ")"


def _feed_word(m: Move, mc: MachineDef) -> Optional[str]:
    if mc.feed_mode == "G94":
        return "F" + _fnum(m.feed, 3)
    t = _move_minutes(m, mc)
    if t <= 0:
        if m.length <= 1e-12 and m.angle > 0 and not mc.rotary_feed_deg_min:
            raise FeedError("a coordinated or table-only cutting block needs rotary_feed_deg_min with feed_mode G93")
        return None
    if m.angle > 0 and not mc.rotary_feed_deg_min:
        raise FeedError("G93 with a moving table needs rotary_feed_deg_min in the machine definition")
    return "F" + _fnum(1.0 / t, 5)


def _move_line(m: Move, mc: MachineDef, last: Dict[str, float]) -> str:
    dec = mc.decimals
    g = {"rapid": "G0", "line": "G1"}.get(m.kind) or ("G3" if _arc_g3(mc, m.ccw) else "G2")
    words = [g]
    if m.comp:
        words.append(m.comp)
        if m.comp != "G40":
            words.append(f"D{mc.d_register}")
    plane_axes = {"trolley", "height"} if m.kind == "arc" else set()
    for ax in mc.axis_order():
        if ax in m.pos and (m.kind != "arc" or ax in plane_axes or abs(m.pos[ax] - last.get(ax, float("nan"))) > 1e-12
                            or ax not in last):
            words.append(mc.letter(ax) + _num(m.pos[ax], dec))
    if m.kind == "arc":
        s = (last.get("trolley", m.pos["trolley"]), last.get("height", m.pos["height"]))
        s = (round(s[0], dec), round(s[1], dec))     # offsets are measured from the written (rounded) start
        off = {"trolley": m.center[0] - s[0], "height": m.center[1] - s[1]}
        for ax in (mc.arc_first, mc.arc_second):
            words.append(mc.arc_words[ax] + _num(off[ax], dec))
    if m.kind != "rapid":
        fw = _feed_word(m, mc)
        if fw:
            words.append(fw)
    return " ".join(words)


def _build_items(plan: Plan) -> List[_Item]:
    mc = plan.machine
    items: List[_Item] = []
    last: Dict[str, float] = {}
    wire_on = False
    comp_on = False
    for it in plan.items:
        lines: List[str] = []
        if isinstance(it, Move):
            lines.append(_move_line(it, mc, {a: round(v, mc.decimals) for a, v in last.items()}))
            last.update(it.pos)
            if it.comp in ("G41", "G42"):
                comp_on = True
            elif it.comp == "G40":
                comp_on = False
        else:
            if it.name == "wire_start":
                lines.append(f"M{mc.m_wire_start}"); wire_on = True
            elif it.name == "wire_stop":
                lines.append(f"M{mc.m_wire_stop}"); wire_on = False
            elif it.name == "coolant_on":
                lines.append(f"M{mc.m_coolant_on}")
            elif it.name == "coolant_off":
                lines.append(f"M{mc.m_coolant_off}")
            elif it.name == "program_end":
                lines.append(f"M{mc.m_program_end}")
            elif it.name == "dwell":
                p = it.value * 1000.0 if mc.dwell_in_ms else it.value
                lines.append("G4 P" + _fnum(p, 3))
            elif it.name == "comment":
                lines.append(_comment(it.text))
        items.append(_Item(lines, dict(last), wire_on, comp_on))
    return items


def _modal_line(mc: MachineDef) -> str:
    w = ["G21", "G90", mc.feed_mode]
    if mc.arc_plane:
        w.append(mc.arc_plane)
    return " ".join(w)


def _header(mc: MachineDef, name: str, k: int, n: int) -> List[str]:
    h = [_comment(f"{name} file {k}/{n}"),
         _comment(f"machine {mc.name} layout {mc.layout} kerf {mc.kerf_mode} wire diameter {_fnum(mc.wire_diameter, 3)}")]
    if mc.kerf_mode == "g41":
        h.append(_comment(f"D{mc.d_register} must hold the wire radius {_fnum(mc.wire_radius, 4)}"))
    h.extend(mc.header)
    h.append(_modal_line(mc))
    return h


def _continuation(mc: MachineDef, prev: _Item) -> List[str]:
    out = []
    if mc.m_coolant_on is not None:
        out.append(f"M{mc.m_coolant_on}")
    if prev.wire_on:
        out.append(f"M{mc.m_wire_start}")
    known = [a for a in mc.axis_order() if a in prev.end]
    out.append(" ".join(["G0"] + [mc.letter(a) + _num(prev.end[a], mc.decimals) for a in known]))
    return out


def _footer(mc: MachineDef, wire_on: bool, final: bool) -> List[str]:
    out = []
    if not final:
        if wire_on:
            out.append(f"M{mc.m_wire_stop}")
        if mc.m_coolant_off is not None:
            out.append(f"M{mc.m_coolant_off}")
        if mc.m_program_end is not None:
            out.append(f"M{mc.m_program_end}")
    out.extend(mc.footer)
    return out


def _numbered(lines: List[str], step: int) -> List[str]:
    if step <= 0:
        return lines
    out, n = [], step
    for ln in lines:
        if ln.startswith("("):
            out.append(ln)
        else:
            out.append(f"N{n} {ln}")
            n += step
    return out


def gcode_files(plan: Plan) -> List[str]:
    """Texts of the program files. One file unless `max_lines` forces a split. A split never happens while G41/G42 is
    active; a continuation file starts with a rapid to the last position of the previous file."""
    mc = plan.machine
    items = _build_items(plan)
    parts: List[List[_Item]] = []
    if mc.max_lines is None:
        parts = [items]
    else:
        worst_head = len(_header(mc, plan.name, 1, 1)) + 3
        worst_foot = len(_footer(mc, True, False))
        cap = mc.max_lines - worst_head - worst_foot
        if cap < 4:
            raise WireSawError("max_lines is too small for the header and footer of this machine definition")
        cur: List[_Item] = []
        used = 0
        for it in items:
            if cur and used + len(it.lines) > cap:
                cut = -1
                for j in range(len(cur) - 1, -1, -1):
                    if not cur[j].comp_on:
                        cut = j
                        break
                if cut < 0:
                    raise WireSawError("cannot split the program inside a G41/G42 span within max_lines; raise "
                                       "max_lines or use kerf_mode cam")
                parts.append(cur[:cut + 1])
                cur = cur[cut + 1:]
                used = sum(len(x.lines) for x in cur)
            cur.append(it)
            used += len(it.lines)
        if cur:
            parts.append(cur)
    texts = []
    for k, part in enumerate(parts, start=1):
        lines = _header(mc, plan.name, k, len(parts))
        if k > 1:
            lines.extend(_continuation(mc, parts[k - 2][-1]))
        for it in part:
            lines.extend(it.lines)
        lines.extend(_footer(mc, part[-1].wire_on, k == len(parts)))
        if mc.max_lines is not None and len(lines) > mc.max_lines:
            raise WireSawError(f"file {k} has {len(lines)} lines, more than max_lines {mc.max_lines}")
        text = "\n".join(_numbered(lines, mc.line_numbers)) + "\n"
        check_program(text, mc, f"file {k}")
        texts.append(text)
    return texts


def write_gcode(job: WireJob, machine: MachineDef) -> List[str]:
    """ISO G-code of a job, one text per file (see `MachineDef.max_lines`)."""
    return gcode_files(plan_job(job, machine))


# =====================================================================================================================
# G-code parser (read-back) and program checks
# =====================================================================================================================
@dataclass
class Block:
    index: int
    n: Optional[int]
    g: List[float]
    m: List[float]
    words: Dict[str, float]
    text: str


_WORD = re.compile(r"([A-Za-z])\s*([-+]?(?:\d+\.?\d*|\.\d+))")


def parse_gcode(text: str) -> List[Block]:
    """Blocks of a program. Comments in () and after ; are removed. `words` holds every word except G, M and N."""
    out = []
    for i, raw in enumerate(text.splitlines()):
        t = re.sub(r"\([^)]*\)", "", raw).split(";")[0].strip()
        if not t:
            continue
        g, m, w, n = [], [], {}, None
        for let, val in _WORD.findall(t):
            let = let.upper()
            v = float(val)
            if let == "G":
                g.append(v)
            elif let == "M":
                m.append(v)
            elif let == "N":
                n = int(v)
            else:
                w[let] = v
        out.append(Block(i, n, g, m, w, t))
    return out


@dataclass
class ReadBack:
    kind: str                       # "rapid" | "line" | "arc"
    pos: Dict[str, float]           # modal absolute position, logical axes
    center: Optional[Vec2] = None
    ccw: bool = True
    block: int = 0


def gcode_positions(text: str, machine: MachineDef) -> List[ReadBack]:
    """Modal axis positions after every motion block (G0, G1, G2, G3), in logical axes."""
    inv = {v: k for k, v in machine.axes.items()}
    cur: Dict[str, float] = {}
    mode = None
    out = []
    for b in parse_gcode(text):
        for g in b.g:
            if g in (0, 1, 2, 3):
                mode = int(g)
        axes = {inv[l]: v for l, v in b.words.items() if l in inv}
        if not axes or mode is None:
            continue
        start = dict(cur)
        cur.update(axes)
        center = None
        ccw = True
        if mode in (2, 3):
            off = {}
            for ax in ("trolley", "height"):
                off[ax] = b.words.get(machine.arc_words[ax], 0.0)
            center = (start.get("trolley", 0.0) + off["trolley"], start.get("height", 0.0) + off["height"])
            g3 = mode == 3
            ccw = g3 if (machine.arc_first, machine.arc_second) == ("trolley", "height") else not g3
        out.append(ReadBack({0: "rapid", 1: "line"}.get(mode, "arc"), dict(cur), center, ccw, b.index))
    return out


def check_program(text: str, machine: MachineDef, label: str = "program") -> None:
    """Wire start before the first cutting block, wire stop after the last, every cut with a positive feed."""
    blocks = parse_gcode(text)
    inv = {v: k for k, v in machine.axes.items()}
    first_cut = last_cut = None
    mode = None
    for i, b in enumerate(blocks):
        for g in b.g:
            if g in (0, 1, 2, 3):
                mode = int(g)
        if any(l in inv for l in b.words) and mode in (1, 2, 3):
            first_cut = i if first_cut is None else first_cut
            last_cut = i
            if machine.feed_mode == "G94" and not b.words.get("F", 0) > 0:
                raise FeedError(f"{label}: cutting block without a positive F: {b.text}")
    if first_cut is None:
        return
    starts = [i for i, b in enumerate(blocks) if float(machine.m_wire_start) in b.m]
    stops = [i for i, b in enumerate(blocks) if float(machine.m_wire_stop) in b.m]
    if not starts or min(starts) > first_cut:
        raise ProgramError(f"{label}: no wire start (M{machine.m_wire_start}) before the first cutting block")
    if not stops or max(stops) < last_cut:
        raise ProgramError(f"{label}: no wire stop (M{machine.m_wire_stop}) after the last cutting block")


# =====================================================================================================================
# DXF writer
# =====================================================================================================================
def _dxf_pairs(pairs: List[Tuple[int, object]]) -> str:
    return "".join(f"{c}\n{v}\n" for c, v in pairs)


def _dxf_entities(rec: PathRec, lw: bool) -> List[Tuple[int, object]]:
    pr = rec.profile
    out: List[Tuple[int, object]] = []
    if lw:
        pts = [pr.start] + [s.end for s in pr.segs]
        out += [(0, "LWPOLYLINE"), (8, rec.layer), (100, "AcDbEntity"), (100, "AcDbPolyline"), (90, len(pts)),
                (70, 1 if pr.closed else 0)]
        for i, p in enumerate(pts):
            out += [(10, repr(float(p[0]))), (20, repr(float(p[1])))]
            if i < len(pr.segs) and pr.segs[i].center is not None:
                s = pr.segs[i]
                sw = _sweep(p, s.end, s.center, s.ccw)
                out.append((42, repr(math.copysign(math.tan(sw / 4.0), 1.0 if s.ccw else -1.0))))
        return out
    for p, s in zip(pr.starts(), pr.segs):
        if s.center is None:
            out += [(0, "LINE"), (8, rec.layer), (10, repr(float(p[0]))), (20, repr(float(p[1]))), (30, "0.0"),
                    (11, repr(float(s.end[0]))), (21, repr(float(s.end[1]))), (31, "0.0")]
        else:
            c = _v(s.center)
            a_s, a_e = _ang(_v(p) - c), _ang(_v(s.end) - c)
            if not s.ccw:
                a_s, a_e = a_e, a_s                       # DXF arcs always run counter-clockwise
            out += [(0, "ARC"), (8, rec.layer), (10, repr(float(c[0]))), (20, repr(float(c[1]))), (30, "0.0"),
                    (40, repr(float(np.hypot(*(_v(p) - c))))), (50, repr(math.degrees(a_s) % 360.0)),
                    (51, repr(math.degrees(a_e) % 360.0))]
    return out


def write_dxf(job: Union[WireJob, Plan], machine: Optional[MachineDef] = None, lwpolyline: bool = False) -> str:
    """DXF of the wire-centre path in the wire plane (x = trolley, y = height), one layer per setup. With G41/G42 the
    programmed (nominal) path is added on a layer ending in _PROG. xyuv writes the two carriages on layers _END0 and
    _END1. The default is R12 (LINE and ARC); `lwpolyline=True` writes LWPOLYLINE with bulges (R13 or later)."""
    plan = job if isinstance(job, Plan) else plan_job(job, machine)
    layers = []
    for rec in plan.paths:
        if rec.layer not in layers:
            layers.append(rec.layer)
    ver = "AC1015" if lwpolyline else "AC1009"
    pairs: List[Tuple[int, object]] = [(0, "SECTION"), (2, "HEADER"), (9, "$ACADVER"), (1, ver), (9, "$INSUNITS"),
                                       (70, 4), (0, "ENDSEC"), (0, "SECTION"), (2, "TABLES"), (0, "TABLE"),
                                       (2, "LAYER"), (70, len(layers))]
    for i, l in enumerate(layers):
        pairs += [(0, "LAYER"), (2, l), (70, 0), (62, 1 + (i % 6)), (6, "CONTINUOUS")]
    pairs += [(0, "ENDTAB"), (0, "ENDSEC"), (0, "SECTION"), (2, "ENTITIES")]
    for rec in plan.paths:
        pairs += _dxf_entities(rec, lwpolyline)
    pairs += [(0, "ENDSEC"), (0, "EOF")]
    return _dxf_pairs(pairs)


# =====================================================================================================================
# ruled faces to xyuv setups
# =====================================================================================================================
def _frame_matrix(frame: Sequence[str]) -> np.ndarray:
    idx = {"x": 0, "y": 1, "z": 2}
    R = np.zeros((3, 3))
    for i, f in enumerate(frame):
        f = f.strip().lower()
        sgn = -1.0 if f.startswith("-") else 1.0
        R[i, idx[f.lstrip("+-")]] = sgn
    if abs(np.linalg.det(R) - 1.0) > 1e-9:
        raise WireSawError(f"frame {tuple(frame)} is not a proper rotation (determinant {np.linalg.det(R):+.0f}); a "
                           "mirrored part would be cut")
    return R


def ruled_face_setup(face, machine: MachineDef, frame: Sequence[str] = ("x", "y", "z"),
                     carriage_depths: Optional[Tuple[float, float]] = None, offset: Sequence[float] = (0, 0, 0),
                     max_depth_angle_deg: float = 60.0, x_tol: float = 0.05, name: str = "",
                     feed: Optional[float] = None) -> RuledSetup:
    """RuledSetup for one face from `lithocam.wirepath.load_ruled_json` (`face.rulings` with p0, p1, normal).

    frame: part axes that become the machine (trolley, depth, height) directions, e.g. ("z", "y", "-x"); it must be a
    rotation. offset: added to the machine coordinates after the rotation. carriage_depths: depth positions of the two
    carriages; default is the mid depth of the face plus or minus half `machine.carriage_spacing`.
    Each ruling is moved by the wire radius along its outward normal, then cut by the two carriage planes.
    """
    if machine.layout != "xyuv":
        raise WireSawError("ruled_face_setup needs an xyuv machine")
    if machine.kerf_mode != "cam":
        raise WireSawError("ruled faces need kerf_mode cam: the wire radius offset is applied to every ruling")
    R = _frame_matrix(frame)
    off = np.asarray(offset, dtype=np.float64)
    rs = face.rulings
    q0 = np.array([R @ _v3(r.p0) + off for r in rs])
    q1 = np.array([R @ _v3(r.p1) + off for r in rs])
    nm = np.array([R @ _v3(r.normal) for r in rs])
    q0 = q0 + nm * machine.wire_radius
    q1 = q1 + nm * machine.wire_radius
    if carriage_depths is None:
        if machine.carriage_spacing is None:
            raise WireSawError("give carriage_depths or set carriage_spacing in the machine definition")
        mid = float(np.mean(np.concatenate([q0[:, 1], q1[:, 1]])))
        carriage_depths = (mid - machine.carriage_spacing / 2.0, mid + machine.carriage_spacing / 2.0)
    rows = []
    cosmin = math.cos(math.radians(max_depth_angle_deg))
    for i in range(len(rs)):
        dv = q1[i] - q0[i]
        L = float(np.linalg.norm(dv))
        if abs(dv[1]) < cosmin * L:
            raise WireSawError(f"ruling {i} runs {math.degrees(math.acos(min(1.0, abs(dv[1]) / L))):.0f} degrees from "
                               f"the depth axis (limit {max_depth_angle_deg:g}); choose another frame or turn the part")
        ends = []
        for d in carriage_depths:
            t = (d - q0[i, 1]) / dv[1]
            p = q0[i] + t * dv
            ends.append((float(p[0]), float(p[2])))
        if not machine.has("trolley2") and abs(ends[0][0] - ends[1][0]) > x_tol:
            raise WireSawError(f"ruling {i}: the wire leaves its trolley plane by {abs(ends[0][0] - ends[1][0]):.3f} mm "
                               "and the machine has no second trolley axis")
        rows.append((ends[0][0], ends[0][1], ends[1][0], ends[1][1]))
    arr = np.array(rows)
    order = np.argsort(arr[:, 0] + arr[:, 2], kind="stable")
    return RuledSetup(arr[order], name or f"face{getattr(face, 'index', '')}", feed)


def _v3(p) -> np.ndarray:
    return np.asarray(p, dtype=np.float64)
