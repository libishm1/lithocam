# Copyright 2026 The lithocam authors. SPDX-License-Identifier: Apache-2.0
"""Circular (disc) saw toolpaths for stone slabs with kerf compensation, numpy only.

Frame: the slab plane is xy, z is up, the slab top is at z = ``top`` and the slab is ``thickness`` thick (so its
underside is at z = top - thickness). All lengths are millimetres, angles are degrees at the API.

Geometry facts used here
------------------------
Run-out. A blade of radius R whose lowest point is a depth d below the top surface (d measured along the blade
plane) has its centre R - d above the top surface (along the blade plane). The rim meets the top surface where
x^2 + (R - d)^2 = R^2, so the groove is longer at the top surface than at its lowest point by

    e(d) = sqrt(2 R d - d^2)

at each end, measured from the lowest point. At in-plane depth h below the top surface the rim reaches only
e(d - h) from the lowest point. The groove depth falls away along the arc between the lowest point and
e(d) beyond it.

Kerf compensation. For an edge a -> b of a counter-clockwise outline the part lies to the left of travel and the
waste to the right. The outward horizontal normal is n = (t_y, -t_x). With bevel angle beta (0 = vertical,
positive = the cut face leans outward going down, negative = undercut) the blade plane contains the edge line at the
top surface and is tilted about it. Its outward unit normal is N = cos(beta) n + sin(beta) ez and the in-plane
downward direction is g = -cos(beta) ez + sin(beta) n. The part-side face of the blade lies on that plane, so the
blade mid-plane is offset by kerf/2 along N. The blade centre for a lowest point at distance u along the edge is

    c(u) = a + u t - (R - d) g + (kerf / 2) N

Corner policy. For a lowest-point position u the whole blade body (a plate of thickness kerf on the waste side of the
face, with top chord e(d) either side of u) must stay out of the part. Let U be the farthest u where the plan
footprint of that plate (including its kerf width) stays clear of the part, found by clipping the part polygon against
the kerf strip (for bevelled edges at several depths, using the part section at each depth). Then the lowest point
may travel to at most U - e at the end of the edge. At a convex corner nothing blocks the extension, so the path
runs past the corner by the requested overcut e(d) + clearance (the overcut lies in waste). At a reflex corner the
part blocks the extension at the corner itself (further back where the corner turns sharply, because the plate has
thickness), so the lowest point stops e(d) short and an uncut wedge of length

    e(d) - e(d - h_b)       (h_b = depth at the slab underside, or d for a blind cut)

remains at the bottom of the groove; for a blind cut that is e(d). The wedge is reported for a follow-up tool.
The strict minimum for full depth at a convex corner is a lowest point exactly on the corner; the e(d) overcut is
the conservative rule asked for in the brief, and it lies in waste.

Depth limit. The flange (radius r_f) must stay above the slab top, so the lowest point may be at most
R - r_f - clearance / cos(beta) deep. A through cut needs d >= thickness / cos(beta) plus a small break-through
allowance. Flange thickness is neglected.

Cut positions are the BLADE CENTRE (not the lowest point), at the mid-plane of the kerf. The lowest point is
centre + R * g_hat.
"""
from __future__ import annotations

import json
import math
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Dict, List, NamedTuple, Optional, Sequence, Tuple

import numpy as np

_EZ = np.array([0.0, 0.0, 1.0])
_EPS = 1e-10          # strip shrink in the clip, mm; the resulting error is below 1e-6 mm for turns above 0.006 deg
_SIN_MIN = 1e-4       # collinear vertex threshold


# ----------------------------------------------------------------------------------------------------------------
# basic formulas
# ----------------------------------------------------------------------------------------------------------------
def runout(radius: float, d: float) -> float:
    """e(d) = sqrt(2 R d - d^2): half-length of the groove at the top surface beyond the lowest point."""
    d = min(max(float(d), 0.0), 2.0 * radius)
    return math.sqrt(max(2.0 * radius * d - d * d, 0.0))


def max_cut_depth(blade_radius: float, flange_radius: float, clearance: float = 1.0, bevel_deg: float = 0.0) -> float:
    """Deepest lowest-point depth (along the blade) that keeps the flange `clearance` above the slab top."""
    return blade_radius - flange_radius - clearance / math.cos(math.radians(bevel_deg))


@dataclass
class Cut:
    """One straight saw cut. `start` and `end` are the BLADE CENTRE (3D, kerf mid-plane); lowest point = centre + R g.

    `normal` is the spindle axis (unit, points to the waste side for profile cuts). `direction` is the cut direction.
    `depth` is the lowest-point depth below the top surface measured along the blade plane. `bevel` is in degrees.
    kind is 'profile' or 'comb'. For profile cuts `edge` is the outline edge index; for comb cuts it is the line index.
    overcut_mm and uncut_mm are the sums over both ends; the per-end values are in the *_start / *_end fields.
    lead_in / lead_out are blade-centre points with the lowest point above the slab top (move in-plane to start/end).
    """
    edge: int
    pass_index: int
    start: Tuple[float, float, float]
    end: Tuple[float, float, float]
    normal: Tuple[float, float, float]
    direction: Tuple[float, float, float]
    depth: float
    bevel: float
    kind: str
    overcut_mm: float
    uncut_mm: float
    radius: float = 0.0
    kerf: float = 0.0
    top: float = 0.0
    lead_in: Tuple[float, ...] = ()
    lead_out: Tuple[float, ...] = ()
    overcut_start: float = 0.0
    overcut_end: float = 0.0
    uncut_start: float = 0.0
    uncut_end: float = 0.0
    through: bool = False


def disc_pose(cut: Cut, s: float) -> np.ndarray:
    """4x4 tool pose at fraction s along the cut (0 = start, 1 = end): tool z = spindle axis = blade normal,
    tool x = cut direction, tool y = z x x, origin at the blade centre (the saw head TCP)."""
    x = np.asarray(cut.direction, float)
    z = np.asarray(cut.normal, float)
    y = np.cross(z, x)
    T = np.eye(4)
    T[:3, 0], T[:3, 1], T[:3, 2] = x, y, z
    a, b = np.asarray(cut.start, float), np.asarray(cut.end, float)
    T[:3, 3] = a + float(s) * (b - a)
    return T


def cuts_to_json(cuts: Sequence[Cut], path=None) -> str:
    """JSON text of the cuts (also written to `path` when given)."""
    text = json.dumps({"format": "lithocam.discsaw.v1", "cuts": [asdict(c) for c in cuts]}, indent=1)
    if path is not None:
        Path(path).write_text(text, encoding="utf-8")
    return text


def load(src) -> List[Cut]:
    """Cuts from JSON text or a file path (inverse of cuts_to_json)."""
    s = str(src)
    text = Path(src).read_text(encoding="utf-8") if not s.lstrip().startswith("{") else s
    d = json.loads(text)
    out = []
    for r in d["cuts"]:
        for k in ("start", "end", "normal", "direction", "lead_in", "lead_out"):
            r[k] = tuple(r[k])
        out.append(Cut(**r))
    return out


# ----------------------------------------------------------------------------------------------------------------
# outline helpers
# ----------------------------------------------------------------------------------------------------------------
def _check_outline(poly) -> np.ndarray:
    P = np.asarray(poly, dtype=float)
    if P.ndim != 2 or P.shape[1] != 2 or len(P) < 3:
        raise ValueError("outline must be an (n, 2) array with n >= 3")
    x, y = P[:, 0], P[:, 1]
    if 0.5 * float(np.sum(x * np.roll(y, -1) - np.roll(x, -1) * y)) <= 0.0:
        raise ValueError("outline must be counter-clockwise with non-zero area")
    E = np.roll(P, -1, axis=0) - P
    L = np.hypot(E[:, 0], E[:, 1])
    if np.any(L <= 1e-12):
        raise ValueError("outline has a zero-length edge")
    T = E / L[:, None]
    cr = T[:, 0] * np.roll(T, -1, axis=0)[:, 1] - T[:, 1] * np.roll(T, -1, axis=0)[:, 0]
    dt = np.sum(T * np.roll(T, -1, axis=0), axis=1)
    bad = np.nonzero((np.abs(cr) < _SIN_MIN) | (dt < -1.0 + 1e-9))[0]
    if len(bad):
        raise ValueError("outline has a collinear or spike vertex at index {} (merge or remove it)".format(
            int((bad[0] + 1) % len(P))))
    return P


def corner_types(outline) -> List[str]:
    """'convex' or 'reflex' per vertex k (the corner between edge k-1 and edge k) of a CCW outline."""
    P = _check_outline(outline)
    T = np.roll(P, -1, axis=0) - P
    T = T / np.hypot(T[:, 0], T[:, 1])[:, None]
    Tp = np.roll(T, 1, axis=0)
    cr = Tp[:, 0] * T[:, 1] - Tp[:, 1] * T[:, 0]
    return ["convex" if c > 0 else "reflex" for c in cr]


def _edge_frames(P):
    E = np.roll(P, -1, axis=0) - P
    L = np.hypot(E[:, 0], E[:, 1])
    T = E / L[:, None]
    Nn = np.stack([T[:, 1], -T[:, 0]], axis=1)
    return L, T, Nn


def _section_vertices(P, nrm, tan_b, zeta):
    """Vertices (m, n, 2) of the part section at vertical depths zeta (m,): edge k offset outward by zeta*tan(beta_k)."""
    zeta = np.atleast_1d(np.asarray(zeta, float))
    prev = np.roll(nrm, 1, axis=0)
    c = np.sum(nrm * P, axis=1)
    cp, tp = np.roll(c, 1), np.roll(tan_b, 1)
    r1 = cp[None, :] + zeta[:, None] * tp[None, :]
    r2 = c[None, :] + zeta[:, None] * tan_b[None, :]
    det = prev[:, 0] * nrm[:, 1] - prev[:, 1] * nrm[:, 0]
    x = (r1 * nrm[None, :, 1] - prev[None, :, 1] * r2) / det[None, :]
    y = (prev[None, :, 0] * r2 - nrm[None, :, 0] * r1) / det[None, :]
    return np.stack([x, y], axis=-1)


def _clip(pts, axis, val, keep_ge):
    out = []
    n = len(pts)
    for i in range(n):
        p, q = pts[i], pts[(i + 1) % n]
        pin = p[axis] >= val if keep_ge else p[axis] <= val
        qin = q[axis] >= val if keep_ge else q[axis] <= val
        if pin:
            out.append(p)
        if pin != qin:
            tt = (val - p[axis]) / (q[axis] - p[axis])
            out.append((p[0] + tt * (q[0] - p[0]), p[1] + tt * (q[1] - p[1])))
    return out


def _free_reach(V, a, t, n, L, s0, s1):
    """(U_fwd, U_back): nearest u (along the edge, from a) at which the part enters the strip s0 < w < s1 beyond the
    edge midpoint (forward) and before it (back). w is the distance along n from the edge line."""
    d = V - a
    pts = list(zip((d @ t).tolist(), (d @ n).tolist()))
    pts = _clip(pts, 1, s0 + _EPS, True)
    if pts:
        pts = _clip(pts, 1, s1 - _EPS, False)
    uf, ub = math.inf, -math.inf
    if pts:
        f = _clip(pts, 0, 0.5 * L, True)
        if f:
            uf = min(p[0] for p in f)
        b = _clip(pts, 0, 0.5 * L, False)
        if b:
            ub = max(p[0] for p in b)
    return uf, ub


def _depths(d_final: float, stepdown: float) -> List[float]:
    if stepdown <= 0:
        raise ValueError("stepdown must be positive")
    n = max(1, int(math.ceil(d_final / stepdown - 1e-9)))
    return [d_final * (k + 1) / n for k in range(n)]


def _limits(blade_diameter, flange_diameter, clearance, bevel_deg_max, depth, thickness, through_extra, what):
    R, rf = 0.5 * blade_diameter, 0.5 * flange_diameter
    if R <= 0 or rf < 0 or rf >= R:
        raise ValueError("need blade_diameter > flange_diameter >= 0")
    return R, rf


# ----------------------------------------------------------------------------------------------------------------
# profile cuts
# ----------------------------------------------------------------------------------------------------------------
def profile_cuts(outline, *, top: float, thickness: float, blade_diameter: float, kerf: float,
                 flange_diameter: float, stepdown: float, depth: Optional[float] = None,
                 bevel_deg=0.0, clearance: float = 1.0, corner_clearance: float = 2.0,
                 lead_clearance: float = 2.0, through_extra: float = 1.0,
                 report: Optional[dict] = None) -> List[Cut]:
    """Kerf-compensated saw cuts around a closed CCW part outline, one set of stepdown passes per edge.

    depth None means a through cut: the lowest point goes thickness/cos(bevel) + through_extra along the blade.
    Otherwise `depth` is the final lowest-point depth along the blade (blind cut; may exceed the slab thickness).
    bevel_deg is a scalar or one value per edge. A pass that cannot be placed (the blade run-out does not fit between
    the obstructions at the two ends of the edge, or the part blocks the kerf strip) is left out and listed in
    report['skipped'] as (edge, pass, reason); the edge is then wholly uncut and needs a follow-up tool.
    Raises ValueError when the slab cannot be cut to the requested depth with this blade and flange.
    """
    P = _check_outline(outline)
    n = len(P)
    if kerf <= 0 or thickness <= 0:
        raise ValueError("kerf and thickness must be positive")
    R, rf = _limits(blade_diameter, flange_diameter, clearance, 0, depth, thickness, through_extra, "profile")
    betas = np.broadcast_to(np.asarray(bevel_deg, float), (n,)).copy()
    if np.any(np.abs(betas) > 75.0):
        raise ValueError("bevel angles above 75 degrees are not supported")
    brad = np.radians(betas)
    cb, sb, tb = np.cos(brad), np.sin(brad), np.tan(brad)
    L, T, Nn = _edge_frames(P)
    skipped = []
    out: List[Cut] = []
    cache: Dict[int, Tuple[float, float]] = {}
    flat = not np.any(betas != 0.0)

    for i in range(n):
        a, b_ = P[i], P[(i + 1) % n]
        t2, n2 = T[i], Nn[i]
        t3 = np.array([t2[0], t2[1], 0.0])
        n3 = np.array([n2[0], n2[1], 0.0])
        N3 = cb[i] * n3 + sb[i] * _EZ
        up = cb[i] * _EZ - sb[i] * n3
        t_in = thickness / cb[i]
        limit = max_cut_depth(R, rf, clearance, betas[i])
        through = depth is None
        d_final = (t_in + through_extra) if through else float(depth)
        if d_final <= 0:
            raise ValueError("depth must be positive")
        if d_final > limit + 1e-9:
            raise ValueError(
                "edge {}: {} needs a lowest-point depth of {:.2f} mm but blade {:.0f} mm with flange {:.0f} mm and "
                "{:.1f} mm clearance reaches at most {:.2f} mm{}".format(
                    i, "a through cut of a {:.1f} mm slab".format(thickness) if through else "the cut",
                    d_final, blade_diameter, flange_diameter, clearance, limit,
                    "; the slab cannot be cut through" if through else ""))
        base = np.array([a[0], a[1], top])
        for k, d in enumerate(_depths(d_final, stepdown)):
            h_b = min(d, t_in)
            zeta_b = h_b * cb[i]
            e0 = runout(R, d)
            if flat:
                if i not in cache:
                    cache[i] = _free_reach(P, a, t2, n2, L[i], 0.0, kerf)
                cap_f, cap_b = cache[i][0] - e0, cache[i][1] + e0
            else:
                zs = np.linspace(0.0, zeta_b, 7)
                V = _section_vertices(P, Nn, tb, zs)
                cap_f, cap_b = math.inf, -math.inf
                for z, Vz in zip(zs, V):
                    uf, ub = _free_reach(Vz, a, t2, n2, L[i], z * tb[i], z * tb[i] + kerf / cb[i])
                    # the far kerf face of an undercut blade (sin(beta) < 0) sits lower, so its rim reaches further
                    ej = runout(R, d - (z + min(0.0, kerf * sb[i])) / cb[i])
                    cap_f, cap_b = min(cap_f, uf - ej), max(cap_b, ub + ej)
            u_f = min(L[i] + e0 + corner_clearance, cap_f)
            u_b = max(-(e0 + corner_clearance), cap_b)
            if u_f < u_b - 1e-9:
                skipped.append((i, k, "run-out {:.1f} mm does not fit between the end obstructions of the edge".format(e0)))
                continue
            e_bot = runout(R, d - h_b)
            oc_s, oc_e = max(0.0, -u_b), max(0.0, u_f - L[i])
            un_s, un_e = max(0.0, u_b - e_bot), max(0.0, (L[i] - u_f) - e_bot)

            def centre(u, dd=d):
                return base + u * t3 + (R - dd) * up + 0.5 * kerf * N3

            lift = (d + lead_clearance) * up
            out.append(Cut(
                edge=i, pass_index=k, start=tuple(centre(u_b)), end=tuple(centre(u_f)),
                normal=tuple(N3), direction=tuple(t3), depth=float(d), bevel=float(betas[i]), kind="profile",
                overcut_mm=oc_s + oc_e, uncut_mm=un_s + un_e, radius=R, kerf=float(kerf), top=float(top),
                lead_in=tuple(centre(u_b) + lift), lead_out=tuple(centre(u_f) + lift),
                overcut_start=oc_s, overcut_end=oc_e, uncut_start=un_s, uncut_end=un_e, through=through))
    if report is not None:
        report["skipped"] = skipped
    return out


def uncut_wedges(cuts: Sequence[Cut], outline) -> List[dict]:
    """Uncut wedges for follow-up tools: for each profile edge end, the deepest pass's uncut length and depth."""
    P = np.asarray(outline, float)
    n = len(P)
    best: Dict[Tuple[int, str], Cut] = {}
    for c in cuts:
        if c.kind != "profile":
            continue
        for end, val in (("start", c.uncut_start), ("end", c.uncut_end)):
            key = (c.edge, end)
            if key not in best or c.depth > best[key].depth:
                best[key] = c
    out = []
    for (edge, end), c in sorted(best.items()):
        length = c.uncut_start if end == "start" else c.uncut_end
        if length > 1e-9:
            v = P[edge] if end == "start" else P[(edge + 1) % n]
            out.append({"edge": edge, "end": end, "vertex": (float(v[0]), float(v[1])), "length_mm": float(length),
                        "depth_mm": float(c.depth)})
    return out


# ----------------------------------------------------------------------------------------------------------------
# saw roughing (combing)
# ----------------------------------------------------------------------------------------------------------------
def _inside_intervals(P, y):
    xs = []
    A, B = P, np.roll(P, -1, axis=0)
    for (x0, y0), (x1, y1) in zip(A.tolist(), B.tolist()):
        if (y0 <= y < y1) or (y1 <= y < y0):
            xs.append(x0 + (y - y0) * (x1 - x0) / (y1 - y0))
    xs.sort()
    return [(xs[j], xs[j + 1]) for j in range(0, len(xs) - 1, 2)]


def _intersect(I, J):
    out = []
    for a0, a1 in I:
        for b0, b1 in J:
            lo, hi = max(a0, b0), min(a1, b1)
            if hi > lo:
                out.append((lo, hi))
    return out


def comb_cuts(region, *, top: float, thickness: float, blade_diameter: float, kerf: float,
              flange_diameter: float, pitch: float, stepdown: float, depth: Optional[float] = None,
              angle_deg: float = 0.0, cross_angle_deg: Optional[float] = None, clearance: float = 1.0,
              lead_clearance: float = 2.0, zigzag: bool = True, report: Optional[dict] = None) -> List[Cut]:
    """Roughing cuts ('combing'): parallel slots at `pitch` across the polygon `region`, then optionally a second set
    at `cross_angle_deg`. Each slot's kerf plate (width kerf, with the top chord e(d) at both ends) stays inside the
    region, so the slot walls are kerf/2 or more from the region boundary and the lowest point stops e(d) short of
    the wall at each end (the wedge that remains is reported as uncut_mm). The first slot centre line is 1e-6 mm
    further in than kerf/2 from the region's lowest extent. A slot that is too short for the run-out at some pass is
    left out and listed in report['skipped'] as (line, pass, reason). Cut.edge is the slot index, counted on across
    the cross set.
    """
    P = np.asarray(region, float)
    if P.ndim != 2 or P.shape[1] != 2 or len(P) < 3:
        raise ValueError("region must be an (n, 2) array with n >= 3")
    if pitch <= 0 or kerf <= 0 or thickness <= 0:
        raise ValueError("pitch, kerf and thickness must be positive")
    R, rf = _limits(blade_diameter, flange_diameter, clearance, 0, depth, thickness, 0, "comb")
    through = depth is None
    d_final = (thickness + 1.0) if through else float(depth)
    limit = max_cut_depth(R, rf, clearance)
    if d_final > limit + 1e-9:
        raise ValueError("comb cut needs a depth of {:.2f} mm but blade {:.0f} mm with flange {:.0f} mm reaches at most "
                         "{:.2f} mm{}".format(d_final, blade_diameter, flange_diameter, limit,
                                              "; the slab cannot be cut through" if through else ""))
    depths = _depths(d_final, stepdown)
    skipped = []
    out: List[Cut] = []
    idx = 0
    h = 0.5 * kerf - 1e-9
    angles = [angle_deg] + ([cross_angle_deg] if cross_angle_deg is not None else [])
    for ang in angles:
        th = math.radians(ang)
        ct, st = math.cos(th), math.sin(th)
        Rm = np.array([[ct, st], [-st, ct]])           # world -> local (cut direction along +x)
        Q = P @ Rm.T
        y0 = float(Q[:, 1].min()) + 0.5 * kerf + 1e-6
        y1 = float(Q[:, 1].max()) - 0.5 * kerf
        ys = []
        y = y0
        while y <= y1 + 1e-12:
            ys.append(y)
            y += pitch
        t3 = np.array([ct, st, 0.0])
        for j, yc in enumerate(ys):
            iv = _intersect(_intersect(_inside_intervals(Q, yc - h), _inside_intervals(Q, yc)),
                            _inside_intervals(Q, yc + h))
            blocked = [float(q[0]) for q in Q if abs(q[1] - yc) < h]
            segs = []
            for lo, hi in iv:
                cuts_at = sorted(u for u in blocked if lo + 1e-9 < u < hi - 1e-9)
                edges = [lo] + [v for u in cuts_at for v in (u, u)] + [hi]
                segs += [(edges[m], edges[m + 1]) for m in range(0, len(edges), 2)]
            for (lo, hi) in segs:
                rev = zigzag and (idx % 2 == 1)
                for k, d in enumerate(depths):
                    e = runout(R, d)
                    ua, ub = lo + e, hi - e
                    if ub < ua - 1e-9:
                        skipped.append((idx, k, "slot {:.1f} mm long is shorter than the {:.1f} mm run-out chord".format(
                            hi - lo, 2 * e)))
                        continue
                    e_bot = runout(R, d - min(d, thickness))
                    un = max(0.0, e - e_bot)

                    def centre(u, d=d, yc=yc):
                        xy = Rm.T @ np.array([u, yc])
                        return np.array([xy[0], xy[1], top + R - d])

                    p0, p1 = (ub, ua) if rev else (ua, ub)
                    tt = -t3 if rev else t3
                    Nn3 = np.array([tt[1], -tt[0], 0.0])
                    lift = np.array([0.0, 0.0, d + lead_clearance])
                    out.append(Cut(
                        edge=idx, pass_index=k, start=tuple(centre(p0)), end=tuple(centre(p1)), normal=tuple(Nn3),
                        direction=tuple(tt), depth=float(d), bevel=0.0, kind="comb", overcut_mm=0.0, uncut_mm=2 * un,
                        radius=R, kerf=float(kerf), top=float(top), lead_in=tuple(centre(p0) + lift),
                        lead_out=tuple(centre(p1) + lift), uncut_start=un, uncut_end=un, through=through))
                idx += 1
    if report is not None:
        report["skipped"] = skipped
    return out


# ----------------------------------------------------------------------------------------------------------------
# verification
# ----------------------------------------------------------------------------------------------------------------
class IntrusionReport(NamedTuple):
    max_intrusion: float            # mm, <= 0 (within tolerance) when no blade point enters the part
    face_distance: Dict[int, float]  # profile edge -> min signed distance of the part-side face to the part (>= 0 ok)
    face_plane_error: Dict[int, float]  # profile edge -> max distance of the edge's end points from the face plane


def _sd_polygon(V, pts):
    """Signed distance (negative inside) from pts (m,2) to polygons V (n,2) or (m,n,2)."""
    if V.ndim == 2:
        V = np.broadcast_to(V, (len(pts),) + V.shape)
    A, B = V, np.roll(V, -1, axis=1)
    AB = B - A
    AP = pts[:, None, :] - A
    den = np.maximum(np.sum(AB * AB, axis=2), 1e-300)
    tt = np.clip(np.sum(AP * AB, axis=2) / den, 0.0, 1.0)
    dist = np.linalg.norm(AP - tt[..., None] * AB, axis=2).min(axis=1)
    py = pts[:, None, 1]
    ay, by = A[..., 1], B[..., 1]
    cond = (ay > py) != (by > py)
    with np.errstate(divide="ignore", invalid="ignore"):
        xint = (B[..., 0] - A[..., 0]) * (py - ay) / (by - ay) + A[..., 0]
    inside = (np.sum(cond & (pts[:, None, 0] < xint), axis=1) % 2) == 1
    return np.where(inside, -dist, dist)


def blade_intrudes(cuts: Sequence[Cut], outline, t: float, samples: int = 9, *, solid: str = "inside",
                   bevels=None, n_angles: int = 48, radii=(1.0, 0.7, 0.4)) -> IntrusionReport:
    """Sample the blade disc volume along every cut (and its lead-in and lead-out) and measure how far it enters the part.

    The disc is sampled on both kerf faces (centre plane -/+ kerf/2), on the rim and at inner radii, plus the exact
    points where the rim crosses the slab top and underside. Only samples inside the slab (z from top - t to top) count.
    solid='inside': the part is the prism (or bevelled solid) under `outline` (profile cuts).
    solid='outside': the part is the slab outside `outline` (comb cuts inside a region).
    Returns IntrusionReport(max_intrusion, face_distance, face_plane_error).
    """
    P = np.asarray(outline, float)
    n = len(P)
    _, _, Nn = _edge_frames(P)
    beta = np.zeros(n)
    if bevels is not None:
        beta = np.broadcast_to(np.asarray(bevels, float), (n,)).copy()
    else:
        for c in cuts:
            if c.kind == "profile":
                beta[c.edge] = c.bevel
    tb = np.tan(np.radians(beta))
    flat = not np.any(beta != 0.0)
    sign = 1.0 if solid == "inside" else -1.0
    ang = np.linspace(0.0, 2 * math.pi, n_angles, endpoint=False)
    worst = -math.inf
    face_d: Dict[int, float] = {}
    plane_err: Dict[int, float] = {}
    for c in cuts:
        T = np.asarray(c.direction, float)
        N = np.asarray(c.normal, float)
        up = np.cross(N, T)
        R, top = c.radius, c.top
        s0, s1 = np.asarray(c.start, float), np.asarray(c.end, float)
        centres = [s0 + s * (s1 - s0) for s in np.linspace(0.0, 1.0, max(2, samples))]
        li, lo_ = np.asarray(c.lead_in, float), np.asarray(c.lead_out, float)
        if li.size:
            centres += [li, 0.5 * (li + s0), lo_, 0.5 * (lo_ + s1)]
        if c.kind == "profile":
            a3 = np.array([P[c.edge][0], P[c.edge][1], top])
            b3 = np.array([P[(c.edge + 1) % n][0], P[(c.edge + 1) % n][1], top])
            fp = s0 - 0.5 * c.kerf * N
            plane_err[c.edge] = max(plane_err.get(c.edge, 0.0), abs(float((a3 - fp) @ N)), abs(float((b3 - fp) @ N)))
        pts_all, w_all = [], []
        for ctr in centres:
            cb = float(up[2]) if abs(up[2]) > 1e-9 else 1e-9
            for w, is_face in ((-0.5 * c.kerf, True), (0.5 * c.kerf, False)):
                cw = ctr + w * N
                yt, yb = (top - cw[2]) / cb, (top - t - cw[2]) / cb
                xu = [np.concatenate([r * R * np.cos(ang) for r in radii])]
                yh = [np.concatenate([r * R * np.sin(ang) for r in radii])]
                for yy in (yt, yb):
                    if abs(yy) < R:
                        ch = math.sqrt(R * R - yy * yy)
                        xs = np.linspace(-ch, ch, 9)
                        xu.append(xs)
                        yh.append(np.full_like(xs, yy))
                xu, yh = np.concatenate(xu), np.concatenate(yh)
                pts_all.append((cw[None, :] + xu[:, None] * T[None, :] + yh[:, None] * up[None, :], is_face))
        for pts, is_face in pts_all:
            z = pts[:, 2]
            keep = (z <= top + 1e-9) & (z >= top - t - 1e-9)
            if not np.any(keep):
                continue
            pk = pts[keep]
            if flat:
                sd = _sd_polygon(P, pk[:, :2])
            else:
                zeta = np.clip(top - pk[:, 2], 0.0, None)
                sd = _sd_polygon(_section_vertices(P, Nn, tb, zeta), pk[:, :2])
            part_sd = sign * sd
            worst = max(worst, float(np.max(-part_sd)))
            if is_face and c.kind == "profile":
                face_d[c.edge] = min(face_d.get(c.edge, math.inf), float(np.min(part_sd)))
    return IntrusionReport(worst, face_d, plane_err)


# ----------------------------------------------------------------------------------------------------------------
# ruled faces: blade posture and depth rules along rulings
# ----------------------------------------------------------------------------------------------------------------
# A ruling is a straight line on the face. A flat blade can follow it only if the blade plane contains the line, so
# the only free posture parameter is the tilt tau of the blade plane about the ruling. Where the face twists along the
# ruling its tangent plane turns about the ruling while the blade plane stays flat. Per ruling, with unit direction d
# and the face normal n(w) at the fraction w along it (from the neighbour rulings of the face, see _twist):
#
#   n(w) ~ d x ( da + w d(b - a) )         da, d(b - a): central differences of the ruling start and of b - a
#   psi(w) = angle about d from n(1/2) to n(w)      (the twist, radians)
#   S = max psi - min psi                            (the twist spread of the ruling)
#
# The groove is cut from the ruling down into the stock in the blade plane (direction g, in the plane, perpendicular to
# d, pointing down). To first order the face leaves the blade plane at depth h by
#
#   e(h, w) = h * tan(tau' - psi'(w))     (e > 0: the face lies on the waste side, the plane is in the part)
#
# (tau' = s tau, psi' = s psi; s = +-1 is the sign that makes g point down). A tolerance `tol` of left-over stock
# (e >= -tol) and `intrusion_tol` into the part (e <= intrusion_tol, 0 = none) at the full depth D give
#
#   psi'_max - atan(tol / D)  <=  tau'  <=  psi'_min + atan(intrusion_tol / D)           (tilt range)
#
# which is empty when S > atan(tol / D) + atan(intrusion_tol / D). The largest depth that keeps it non-empty is the
# curvature depth. For intrusion_tol = 0 it is tol / tan(S); for intrusion_tol = tol it is tol / tan(S / 2). A plane
# (S = 0) has no curvature limit. The depth is also limited by the flange (max_cut_depth) and the blade must stay
# within MAX_BEVEL_DEG of the vertical. The recommended tilt is the middle of the tilt range (for a plane, exactly the tangent plane).
#
# The first order rule ignores the bending of the face across the groove. Every depth is therefore verified on the
# face itself (the ruled sheet between neighbour rulings) and shortened until no sheet point in the groove lies more
# than intrusion_tol in the part or tol outside the blade plane. Stock above a ruling (h < 0) is taken as already
# removed: use the rulings from the top edge of the face downwards.
MAX_BEVEL_DEG = 75.0     # the same limit profile_cuts applies to bevel angles


def _rget(r, k, default=None):
    if isinstance(r, dict):
        return r.get(k, default)
    return getattr(r, k, default)


def _face_rulings(face):
    rs = face["rulings"] if isinstance(face, dict) else getattr(face, "rulings", face)
    rs = list(rs)
    if not rs:
        raise ValueError("the face has no rulings")
    return rs


class _Prep(NamedTuple):
    order: np.ndarray        # sorted position -> index in the face's ruling list
    A: np.ndarray            # (n, 3) ruling starts, consistently oriented, sorted along the sweep
    B: np.ndarray            # (n, 3) ruling ends
    free: list               # (start free, end free) of the oriented ruling
    nrm: np.ndarray          # (n, 3) outward normals at the ruling mid points


def _prepare(face) -> _Prep:
    rs = _face_rulings(face)
    P0 = np.array([_rget(r, "p0") for r in rs], float)
    P1 = np.array([_rget(r, "p1") for r in rs], float)
    NR = np.array([_rget(r, "normal") for r in rs], float)
    FR = [tuple(bool(x) for x in (_rget(r, "tool_free", (True, True)) or (True, True))) for r in rs]
    n = len(rs)
    order = np.arange(n)
    if n > 1:
        mids = 0.5 * (P0 + P1)
        c = mids - mids.mean(axis=0)
        sweep = np.linalg.svd(c, full_matrices=False)[2][0]
        order = np.argsort(c @ sweep, kind="stable")
    A, B, NR = P0[order].copy(), P1[order].copy(), NR[order]
    free = [FR[i] for i in order]
    for i in range(1, n):
        if (B[i] - A[i]) @ (B[i - 1] - A[i - 1]) < 0:
            A[i], B[i] = B[i].copy(), A[i].copy()
            free[i] = free[i][::-1]
    return _Prep(order, A, B, free, NR)


def _rot(v, d, ang):
    """Rotate v (perpendicular to unit d) about d by ang (right hand)."""
    return v * math.cos(ang) + np.cross(d, v) * math.sin(ang)


class _Frame(NamedTuple):
    d: np.ndarray            # unit ruling direction (oriented)
    n_ref: np.ndarray        # reference normal (tangent plane at the middle of the ruling)
    g0: np.ndarray           # in-plane down direction at tau = 0
    s: float                 # +1 / -1, T = s d is the travel direction
    L: float
    w: np.ndarray            # fractions along the ruling
    psi: np.ndarray          # twist angle psi'(w) = s psi(w), radians
    known: bool              # False when the face has one ruling (twist unknown, taken as zero)


def _frame(prep: _Prep, i: int, nw: int = 33) -> _Frame:
    A, B = prep.A, prep.B
    n = len(A)
    ab = B[i] - A[i]
    L = float(np.linalg.norm(ab))
    d = ab / L
    n_rec = prep.nrm[i] - d * (prep.nrm[i] @ d)
    n_rec /= np.linalg.norm(n_rec)
    lo, hi = max(i - 1, 0), min(i + 1, n - 1)
    w = np.linspace(0.0, 1.0, nw)
    known = hi > lo
    nn = None
    if known:
        dA = A[hi] - A[lo]
        dE = (B[hi] - A[hi]) - (B[lo] - A[lo])
        V = dA[None, :] + w[:, None] * dE[None, :]
        nn = np.cross(d, V)
        nrm = np.linalg.norm(nn, axis=1)
        if nrm.min() < 1e-9 * max(nrm.max(), 1e-300) or nrm.max() < 1e-12:
            known, nn = False, None
        else:
            nn = nn / nrm[:, None]
            nn[nn @ n_rec < 0] *= -1.0
    n_ref = n_rec if nn is None else nn[nw // 2]
    n_ref = n_ref - d * (n_ref @ d)
    n_ref /= np.linalg.norm(n_ref)
    g0 = np.cross(d, n_ref)
    s = 1.0 if g0[2] <= 0 else -1.0                 # g points down: g . (-z) >= 0
    g0 = s * g0
    if nn is None:
        psi = np.zeros(nw)
    else:
        psi = s * np.arctan2(np.cross(n_ref[None, :], nn) @ d, nn @ n_ref)
    return _Frame(d, n_ref, g0, s, L, w, psi, known)


def _bevel_deg(fr: _Frame, tau_p: float) -> float:
    """Angle of the in-plane down direction from the vertical, signed by the z component of the blade normal."""
    tau = fr.s * tau_p
    g = _rot(fr.g0, fr.d, tau)
    N = _rot(fr.n_ref, fr.d, tau)
    a = math.degrees(math.acos(max(-1.0, min(1.0, -g[2]))))
    return a if N[2] >= 0 else -a


def _curvature_depth(spread: float, tol: float, intrusion_tol: float) -> float:
    """Largest depth D with atan(tol/D) + atan(intrusion_tol/D) >= spread (inf when spread = 0)."""
    if spread <= 1e-12:
        return math.inf

    def F(D):
        return math.atan(tol / D) + math.atan(intrusion_tol / D)
    if F(1e9) >= spread:
        return math.inf
    lo, hi = 1e-9, 1e9
    if F(lo) < spread:
        return 0.0
    for _ in range(200):
        mid = math.sqrt(lo * hi)
        if F(mid) >= spread:
            lo = mid
        else:
            hi = mid
    return lo


def _tilt_range(fr: _Frame, D: float, tol: float, intrusion_tol: float, max_bevel_deg: float):
    """(tau'_lo, tau'_hi, tau'_recommended, reason) at depth D, within the bevel limit; reason '' when non-empty."""
    lo = float(fr.psi.max()) - math.atan(tol / D)
    hi = float(fr.psi.min()) + math.atan(intrusion_tol / D)
    if lo > hi + 1e-12:
        return lo, hi, None, ("twist {:.3f} deg exceeds what a flat blade can follow at depth {:.1f} mm "
                              "(curvature limit)".format(math.degrees(fr.psi.max() - fr.psi.min()), D))
    ts = np.linspace(lo, max(hi, lo), 721)
    ok = np.array([abs(_bevel_deg(fr, t)) <= max_bevel_deg for t in ts])
    if not ok.any():
        return lo, hi, None, "the blade would lean more than {:.0f} degrees from the vertical (ruling too steep, or face too flat, " "to cut from above)".format(max_bevel_deg)
    mid = 0.5 * (lo + hi)
    if float(fr.psi.max() - fr.psi.min()) < 1e-9:      # no twist (a plane): the exact tilt, not the middle
        mid = float(fr.psi.min())
    k = np.nonzero(ok)[0]
    best = k[np.argmin(np.abs(ts[k] - mid))]
    return float(ts[k[0]]), float(ts[k[-1]]), float(ts[best]), ""


def _ruling_geometry(prep: _Prep, i: int, fr: _Frame, tau_p: float, R: float, d: float, kerf: float,
                     corner_clearance: float = 2.0):
    """Frame, travel limits and end data of one pass on ruling i at lowest-point depth d."""
    tau = fr.s * tau_p
    N = _rot(fr.n_ref, fr.d, tau)
    g = _rot(fr.g0, fr.d, tau)
    T = fr.s * fr.d
    O = prep.A[i] if fr.s > 0 else prep.B[i]
    free = prep.free[i] if fr.s > 0 else prep.free[i][::-1]
    e0 = runout(R, d)
    u_b = -(e0 + corner_clearance) if free[0] else e0
    u_f = fr.L + e0 + corner_clearance if free[1] else fr.L - e0
    return dict(N=N, g=g, T=T, O=O, e0=e0, u_b=u_b, u_f=u_f, free=free)


def _sheet_points(prep: _Prep, h_res: float, nw: int = 41, max_ns: int = 400) -> np.ndarray:
    """Points (m, 3) of the ruled sheet between neighbouring rulings (bilinear in the two end points). The spacing
    across the rulings is at most h_res (mm), so the groove of a pass of depth about 10 h_res holds many points."""
    A, B = prep.A, prep.B
    w = np.linspace(0.0, 1.0, nw)[:, None]
    pts = []
    if len(A) == 1:
        pts.append(A[0] + w * (B[0] - A[0]))
    for i in range(len(A) - 1):
        r0 = A[i][None, :] + w * (B[i] - A[i])[None, :]
        r1 = A[i + 1][None, :] + w * (B[i + 1] - A[i + 1])[None, :]
        gap = float(np.max(np.linalg.norm(r1 - r0, axis=1)))
        ns = int(min(max_ns, max(2, math.ceil(gap / max(h_res, 1e-6)) + 1)))
        for s in np.linspace(0.0, 1.0, ns):
            pts.append((1.0 - s) * r0 + s * r1)
    return np.vstack(pts)

def _extremes(X, geo, R, d):
    """(max e, min e) of the face points inside the groove of one pass: depth 0 <= h <= d along the blade plane and
    within the rim reach of the swept lowest point."""
    rel = X - geo["O"][None, :]
    u, h, e = rel @ geo["T"], rel @ geo["g"], rel @ geo["N"]
    dd = np.clip(d - h, 0.0, 2.0 * R)
    reach = np.sqrt(np.maximum(2.0 * R * dd - dd * dd, 0.0))
    m = (h >= -1e-9) & (h <= d) & (u >= geo["u_b"] - reach) & (u <= geo["u_f"] + reach)
    if not m.any():
        return 0.0, 0.0
    return float(e[m].max()), float(e[m].min())


@dataclass
class RulingFeasibility:
    """Blade rules for one ruling. Angles in degrees, depths in mm along the blade (lowest point below the ruling).

    index          ruling index in the face's list
    cuttable       True when a pass to `depth` exists (depth None: to max_depth, which must exceed zero)
    reason         why not, else ''
    twist_deg      spread S of the face's tangent plane about the ruling
    tilt_min/max   allowed tilt of the blade plane about the ruling at the pass depth, from the plane that is
                   tangent at the middle of the ruling (positive about the oriented ruling start -> end)
    tilt_deg       recommended tilt (middle of the range)
    bevel_deg      the blade's lean from the vertical at the recommended tilt (signed by the blade normal's z)
    curvature_depth, flange_depth   the two depth limits (first order, mm)
    verified_depth the deepest pass the sheet check accepts (<= both limits); max_depth = verified_depth
    """
    index: int
    length: float
    cuttable: bool
    reason: str
    twist_deg: float
    tilt_min_deg: float
    tilt_max_deg: float
    tilt_deg: float
    bevel_deg: float
    curvature_depth: float
    flange_depth: float
    max_depth: float
    depth: float
    twist_known: bool = True


@dataclass
class FaceFeasibility:
    rulings: List[RulingFeasibility]
    n_cuttable: int
    max_depth: float            # smallest max_depth over the cuttable rulings (0 when none)
    min_twist_deg: float
    max_twist_deg: float
    recommended_tilt_deg: float  # median tilt of the cuttable rulings
    all_cuttable: bool

    def summary(self) -> str:
        return ("{} of {} rulings cuttable, depth to {:.1f} mm, twist {:.3f} to {:.3f} deg, tilt {:.2f} deg".format(
            self.n_cuttable, len(self.rulings), self.max_depth, self.min_twist_deg, self.max_twist_deg,
            self.recommended_tilt_deg))


def _analyse(prep, sheet, i, *, R, rf, kerf, tol, intrusion_tol, clearance, depth, max_bevel_deg, corner_clearance):
    fr = _frame(prep, i)
    S = float(fr.psi.max() - fr.psi.min())
    Dc = _curvature_depth(S, tol, intrusion_tol)
    Df0 = max_cut_depth(R, rf, clearance, 0.0)
    D = float(depth) if depth is not None else min(Df0, Dc)
    base = dict(index=int(prep.order[i]), length=fr.L, twist_deg=math.degrees(S), curvature_depth=Dc,
                twist_known=fr.known, depth=D)
    fail = dict(cuttable=False, tilt_min_deg=math.nan, tilt_max_deg=math.nan, tilt_deg=math.nan,
                bevel_deg=math.nan, flange_depth=Df0, max_depth=0.0)
    if D <= 0 or not math.isfinite(D):
        return RulingFeasibility(reason="no positive depth (curvature limit {:.3g} mm)".format(Dc), **fail, **base)
    lo, hi, rec, why = _tilt_range(fr, D, tol, intrusion_tol, max_bevel_deg)
    if rec is None:
        return RulingFeasibility(reason=why, **fail, **base)
    bev = _bevel_deg(fr, rec)
    Df = max_cut_depth(R, rf, clearance, bev)
    # the sheet check: shorten the depth until the groove has no sheet point beyond the tolerances
    Dv = min(D, Df, Dc if math.isfinite(Dc) else D)
    if sheet is None:
        sheet = _sheet_points(prep, max(Dv, 1e-3) / 10.0)
    ok = False
    for _ in range(80):
        if Dv <= 1e-6:
            break
        geo = _ruling_geometry(prep, i, fr, rec, R, Dv, kerf, corner_clearance)
        emax, emin = _extremes(sheet, geo, R, Dv)
        if emax <= intrusion_tol + 1e-7 and -emin <= tol + 1e-7:
            ok = True
            break
        Dv *= 0.97
    ok_depth = Dv if ok else 0.0
    cut = ok and (depth is None or ok_depth >= D - 1e-9)
    why = ""
    if not cut:
        why = ("depth {:.1f} mm exceeds the flange limit {:.1f} mm".format(D, Df) if depth is not None and Df < D - 1e-9
               else "depth {:.1f} mm exceeds the curvature limit {:.1f} mm".format(D, Dc)
               if depth is not None and Dc < D - 1e-9 else "the face check shortens the depth to {:.1f} mm".format(
                   ok_depth) if depth is not None else "no safe depth")
    sgn = fr.s
    t_lo, t_hi = (lo, hi) if sgn > 0 else (-hi, -lo)
    return RulingFeasibility(index=base["index"], length=fr.L, cuttable=bool(cut), reason=why,
                             twist_deg=base["twist_deg"], tilt_min_deg=math.degrees(t_lo),
                             tilt_max_deg=math.degrees(t_hi), tilt_deg=math.degrees(sgn * rec), bevel_deg=bev,
                             curvature_depth=Dc, flange_depth=Df, max_depth=ok_depth, depth=D,
                             twist_known=fr.known)


def _setup(blade_diameter, flange_diameter, kerf, tol, intrusion_tol):
    R, rf = 0.5 * blade_diameter, 0.5 * flange_diameter
    if R <= 0 or rf < 0 or rf >= R:
        raise ValueError("need blade_diameter > flange_diameter >= 0")
    if tol <= 0 or intrusion_tol < 0 or kerf < 0:
        raise ValueError("tol must be positive, intrusion_tol and kerf not negative")
    return R, rf


def ruled_disc_feasibility(face, *, blade_diameter: float, flange_diameter: float, tol: float,
                           intrusion_tol: float = 0.0, kerf: float = 0.0, clearance: float = 1.0,
                           depth: Optional[float] = None, max_bevel_deg: float = MAX_BEVEL_DEG,
                           corner_clearance: float = 2.0) -> FaceFeasibility:
    """Per ruling and per face: can a disc cut along it, how deep, and at what tilt.

    face is a face of lithocam.wirepath.load_ruled_json (or a dict / list of rulings with p0, p1, normal and
    tool_free). tol is the left-over stock allowed between the flat blade and the face at the full depth (mm),
    intrusion_tol the allowed intrusion into the part (0 = none). depth None asks for the deepest safe pass of every
    ruling (its max_depth); a number asks for that depth and marks the rulings that cannot reach it."""
    R, rf = _setup(blade_diameter, flange_diameter, kerf, tol, intrusion_tol)
    prep = _prepare(face)
    sheet = None
    rows = [_analyse(prep, sheet, i, R=R, rf=rf, kerf=kerf, tol=tol, intrusion_tol=intrusion_tol,
                     clearance=clearance, depth=depth, max_bevel_deg=max_bevel_deg,
                     corner_clearance=corner_clearance) for i in range(len(prep.A))]
    rows.sort(key=lambda r: r.index)
    good = [r for r in rows if r.cuttable]
    tw = [r.twist_deg for r in rows]
    return FaceFeasibility(rows, len(good), min((r.max_depth for r in good), default=0.0), min(tw), max(tw),
                           float(np.median([r.tilt_deg for r in good])) if good else math.nan,
                           len(good) == len(rows))


def ruled_disc_cuts(face, *, blade_diameter: float, kerf: float, flange_diameter: float, stepdown: float, tol: float,
                    intrusion_tol: float = 0.0, depth: Optional[float] = None, clearance: float = 1.0,
                    corner_clearance: float = 2.0, lead_clearance: float = 2.0,
                    max_bevel_deg: float = MAX_BEVEL_DEG, report: Optional[dict] = None) -> List[Cut]:
    """Disc passes along the cuttable rulings of a ruled (or planar) face: kind 'ruling', one set of stepdown passes
    per ruling at the recommended tilt, `edge` = ruling index in the face's list.

    The blade plane contains the ruling, its normal points to the waste side and tool z = blade normal as for
    profile cuts, so disc_pose, cuts_to_json and blade_intrudes conventions carry over. Ends marked tool_free get
    the run-out overcut, a blocked end stops e(d) short (uncut wedge, as at a reflex corner). depth None cuts each
    ruling to its own max_depth. Rulings that cannot be cut are left out and listed in report['skipped'] as
    (ruling index, reason)."""
    R, rf = _setup(blade_diameter, flange_diameter, kerf, tol, intrusion_tol)
    if kerf <= 0:
        raise ValueError("kerf must be positive")
    prep = _prepare(face)
    sheet = None
    skipped, out = [], []
    for i in range(len(prep.A)):
        rf_ = _analyse(prep, sheet, i, R=R, rf=rf, kerf=kerf, tol=tol, intrusion_tol=intrusion_tol,
                       clearance=clearance, depth=depth, max_bevel_deg=max_bevel_deg,
                       corner_clearance=corner_clearance)
        if not rf_.cuttable:
            skipped.append((rf_.index, rf_.reason))
            continue
        fr = _frame(prep, i)
        tau_p = math.radians(fr.s * rf_.tilt_deg)
        d_final = rf_.max_depth if depth is None else float(depth)
        for k, d in enumerate(_depths(d_final, stepdown)):
            geo = _ruling_geometry(prep, i, fr, tau_p, R, d, kerf, corner_clearance)
            u_b, u_f = geo["u_b"], geo["u_f"]
            if u_f < u_b - 1e-9:
                skipped.append((rf_.index, "run-out {:.1f} mm does not fit between the ruling ends".format(
                    geo["e0"])))
                continue
            N, g, T, O = geo["N"], geo["g"], geo["T"], geo["O"]
            up = -g

            def centre(u, dd=d):
                return O + u * T + (R - dd) * up + 0.5 * kerf * N

            oc_s, oc_e = max(0.0, -u_b), max(0.0, u_f - fr.L)
            un_s, un_e = max(0.0, u_b), max(0.0, fr.L - u_f)
            lift = (d + lead_clearance) * up
            bev = _bevel_deg(fr, tau_p)
            out.append(Cut(
                edge=rf_.index, pass_index=k, start=tuple(centre(u_b)), end=tuple(centre(u_f)), normal=tuple(N),
                direction=tuple(T), depth=float(d), bevel=float(bev), kind="ruling", overcut_mm=oc_s + oc_e,
                uncut_mm=un_s + un_e, radius=R, kerf=float(kerf), top=float(O[2]),
                lead_in=tuple(centre(u_b) + lift), lead_out=tuple(centre(u_f) + lift), overcut_start=oc_s,
                overcut_end=oc_e, uncut_start=un_s, uncut_end=un_e, through=False))
    if report is not None:
        report["skipped"] = skipped
        report["feasibility"] = None
    return out


class RulingIntrusion(NamedTuple):
    max_intrusion: float            # mm, > 0 means a face point lies inside the blade side of the plane (the part is cut)
    max_excess: float               # mm of stock left between the flat blade and the face
    per_cut: Dict[tuple, tuple]     # (ruling, pass) -> (intrusion, excess)


def ruled_blade_intrudes(cuts: Sequence[Cut], face) -> RulingIntrusion:
    """Check generated ruling passes against the face itself: the ruled sheet between neighbouring rulings.

    For every cut, the sheet points inside its groove (depth 0..d along the blade plane, within the rim reach of
    the swept lowest point) are measured against the part-side face of the blade. max_intrusion is the largest
    distance a point lies on the blade (waste) side, so <= intrusion_tol means the blade does not cut the part;
    max_excess is the largest left-over stock (reported, limited by tol)."""
    prep = _prepare(face)
    depths = [c.depth for c in cuts if c.kind == "ruling"]
    sheet = _sheet_points(prep, (min(depths) if depths else 1.0) / 10.0)
    inv ={int(o): k for k, o in enumerate(prep.order)}
    worst_i, worst_x = -math.inf, 0.0
    per = {}
    for c in cuts:
        if c.kind != "ruling":
            continue
        i = inv[c.edge]
        T, N = np.asarray(c.direction, float), np.asarray(c.normal, float)
        g = np.cross(T, N)
        d_ = prep.B[i] - prep.A[i]
        O = prep.A[i] if T @ d_ > 0 else prep.B[i]
        s0, s1 = np.asarray(c.start, float), np.asarray(c.end, float)
        # ruling line point under the blade centre: centre - (R - d)(-g)... = centre + (R - d) g - kerf/2 N
        base = s0 + (c.radius - c.depth) * g - 0.5 * c.kerf * N
        u_b = float((base - O) @ T)
        u_f = u_b + float((s1 - s0) @ T)
        geo = dict(O=O, T=T, N=N, g=g, u_b=u_b, u_f=u_f)
        emax, emin = _extremes(sheet, geo, c.radius, c.depth)
        per[(c.edge, c.pass_index)] = (emax, -emin)
        worst_i = max(worst_i, emax)
        worst_x = max(worst_x, -emin)
    return RulingIntrusion(worst_i if per else 0.0, worst_x, per)


def load_face(path, index: int):
    """Face `index` of a `python -m lithocam.ruled ... --json` file (needs no OCP: reads the JSON)."""
    from .wirepath import load_ruled_json
    for f in load_ruled_json(str(path)).faces:
        if f.index == index:
            return f
    raise ValueError("no face {} in {}".format(index, path))
