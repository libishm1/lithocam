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
