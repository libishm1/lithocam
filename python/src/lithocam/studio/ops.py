# Copyright 2026 The lithocam authors. SPDX-License-Identifier: Apache-2.0
"""More toolpath generators for the studio: hole cycles (drill, core_drill, tap, bore), rest machining and pencil
finishing, and curvature-adaptive raster finishing.

Every generator follows the cam.py contract: f(tris, mesh, tool, stock, **params) -> list of moves (kind, point mm
in the setup frame[, tool axis]) with kind "rapid", "plunge" or "feed". Units are millimetres. Feeds, pecks, dwell
times and spindle speeds are parameters of the operation, never defaults invented here.

New move kinds (the 4th element and later carry the data; the 3rd is the tool axis, always +z here):
  ("dwell", point, axis, seconds)             stay at point for `seconds`.
  ("tap_in", point, axis, feed, rpm)          feed to point at `feed` mm/min, spindle forward at `rpm`.
  ("tap_out", point, axis, feed, rpm)         feed to point at `feed` mm/min with the spindle REVERSED at `rpm`.
docs/OPERATIONS_HOLES_REST.md says how program.py and post.py treat them.
"""
from __future__ import annotations

import math
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from . import cam
from .core import lc

Move = tuple
_Z = np.array([0.0, 0.0, 1.0])
_GAP = 1.0   # mm: re-entry clearance above the bottom of the previous peck (geometry, not a process value)


def parse_points(text):
    """Hole points from text 'x,y; x,y,z_top; ...' (mm). Empty text, None or a list: returned as is (empty = detect)."""
    if text is None:
        return None
    if not isinstance(text, str):
        return text if len(text) else None
    pts = []
    for s in text.replace("\n", ";").split(";"):
        if not s.strip():
            continue
        v = [float(x) for x in s.split(",")]
        if len(v) not in (2, 3):
            raise ValueError("a hole point is 'x,y' or 'x,y,z_top' in mm (got '{}')".format(s.strip()))
        pts.append(tuple(v))
    return pts or None


def parse_range(text):
    """A diameter range from text 'min,max' (mm); empty text or None gives None (any diameter)."""
    if text is None:
        return None
    if not isinstance(text, str):
        return tuple(text) if len(text) else None
    if not text.strip():
        return None
    v = [float(x) for x in text.replace(";", ",").split(",") if x.strip()]
    if len(v) != 2 or v[0] > v[1]:
        raise ValueError("a diameter range is 'min,max' in mm (got '{}')".format(text.strip()))
    return (v[0], v[1])


# ------------------------------------------------------------------------------------------------------------------
# Hole detection

def _unit_normals(tris: np.ndarray) -> np.ndarray:
    n = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    ln = np.linalg.norm(n, axis=1, keepdims=True)
    return n / np.where(ln > 0, ln, 1.0)


def detect_holes(tris: np.ndarray, min_diameter: float = 0.0, max_diameter: float = math.inf,
                 tol: float = 0.01) -> List[Dict[str, float]]:
    """Vertical cylindrical holes in a triangle mesh (normals out of the material).

    The rim of a hole is the set of edges shared by an upward-facing triangle (normal z above 0.999) and a vertical
    wall triangle (|nz| below 0.02) whose normal points at the hole axis. Rim edges are chained into closed loops;
    a loop is a hole when a least-squares circle (Kasa fit) in xy has every vertex within `tol` mm and the loop has
    at least 8 vertices. The bottom is the lowest vertex on the same circle (radial distance within `tol`).
    Returns dicts x, y, z_top, diameter (of the fitted circle), z_bottom, depth, through (bottom at the part's
    lowest z), sorted by (y, x)."""
    tris = np.asarray(tris, dtype=float).reshape(-1, 3, 3)
    n = _unit_normals(tris)
    top = n[:, 2] > 0.999
    wall = np.abs(n[:, 2]) < 0.02

    def key(p):
        return tuple(np.round(p, 6))

    wall_edges: Dict[tuple, List[int]] = {}
    for t in np.nonzero(wall)[0]:
        for a, b in ((0, 1), (1, 2), (2, 0)):
            wall_edges.setdefault(frozenset((key(tris[t, a]), key(tris[t, b]))), []).append(t)
    adj: Dict[tuple, set] = {}
    pos: Dict[tuple, np.ndarray] = {}
    rim_walls: Dict[tuple, List[int]] = {}
    for t in np.nonzero(top)[0]:
        for a, b in ((0, 1), (1, 2), (2, 0)):
            ka, kb = key(tris[t, a]), key(tris[t, b])
            e = frozenset((ka, kb))
            if e in wall_edges:
                adj.setdefault(ka, set()).add(kb)
                adj.setdefault(kb, set()).add(ka)
                pos[ka], pos[kb] = tris[t, a], tris[t, b]
                for w in wall_edges[e]:
                    rim_walls.setdefault(ka, []).append(w)
    seen, holes = set(), []
    zmin = float(tris[:, :, 2].min())
    allv = tris.reshape(-1, 3)
    for s in adj:
        if s in seen:
            continue
        comp, stack = [], [s]
        seen.add(s)
        while stack:
            k = stack.pop()
            comp.append(k)
            for q in adj[k]:
                if q not in seen:
                    seen.add(q)
                    stack.append(q)
        if len(comp) < 8 or any(len(adj[k]) != 2 for k in comp):
            continue
        P = np.array([pos[k] for k in comp])
        A = np.column_stack([2 * P[:, 0], 2 * P[:, 1], np.ones(len(P))])
        sol = np.linalg.lstsq(A, P[:, 0] ** 2 + P[:, 1] ** 2, rcond=None)[0]
        cx, cy = float(sol[0]), float(sol[1])
        r = math.sqrt(max(sol[2] + cx * cx + cy * cy, 0.0))
        if r <= 0 or float(np.abs(np.hypot(P[:, 0] - cx, P[:, 1] - cy) - r).max()) > tol:
            continue
        if np.ptp(P[:, 2]) > tol:
            continue
        if not (min_diameter <= 2 * r <= max_diameter):
            continue
        w = sorted({t for k in comp for t in rim_walls.get(k, [])})
        cen = tris[w].mean(axis=1)
        inward = np.einsum("ij,ij->i", n[w][:, :2], np.array([cx, cy]) - cen[:, :2])
        if not np.all(inward > 0):          # a boss, not a hole
            continue
        ztop = float(P[:, 2].mean())
        if not np.all(cen[:, 2] < ztop):    # the wall must lie below the rim (the floor of a hole is not a hole)
            continue
        rad = np.hypot(allv[:, 0] - cx, allv[:, 1] - cy)
        zb = float(allv[(np.abs(rad - r) <= tol) & (allv[:, 2] <= ztop + tol), 2].min())
        holes.append({"x": cx, "y": cy, "z_top": ztop, "diameter": 2 * r, "z_bottom": zb, "depth": ztop - zb,
                      "through": bool(zb <= zmin + tol)})
    return sorted(holes, key=lambda h: (round(h["y"], 6), round(h["x"], 6)))


def _targets(tris, stock, points, depth, diameter_range, tol=0.01):
    """Normalise hole targets to dicts x, y, z_top, depth. points None: detect in the part."""
    top = float(stock["zmax"])
    out = []
    if points is None:
        lo, hi = diameter_range if diameter_range else (0.0, math.inf)
        for h in detect_holes(tris, lo, hi, tol):
            out.append({"x": h["x"], "y": h["y"], "z_top": h["z_top"], "diameter": h["diameter"],
                        "depth": float(depth) if depth else h["depth"]})
        return out
    for p in points:
        if isinstance(p, dict):
            d = {"x": float(p["x"]), "y": float(p["y"]), "z_top": float(p.get("z_top", top)),
                 "diameter": float(p.get("diameter", 0.0)), "depth": float(p.get("depth", depth or 0.0))}
        else:
            q = list(p)
            d = {"x": float(q[0]), "y": float(q[1]), "z_top": float(q[2]) if len(q) > 2 else top,
                 "diameter": 0.0, "depth": float(depth or 0.0)}
        out.append(d)
    for d in out:
        if d["depth"] <= 0:
            raise ValueError("hole depth must be > 0 (give depth, or detect holes from the part)")
    return out


def _peck_depths(depth: float, peck: float) -> List[float]:
    """Cumulative depths of the pecks: equal steps, none above `peck`, the last exactly `depth`. peck <= 0: one."""
    if peck <= 0 or peck >= depth:
        return [depth]
    n = int(math.ceil(depth / peck - 1e-9))
    d = [depth * k / n for k in range(1, n)]
    return d + [depth]


def _helix(cx, cy, r, z0, z1, turns, tol) -> np.ndarray:
    """Dense helix points from (cx + r, cy, z0) down to z1, chord sag at most tol, going counter-clockwise."""
    r = max(r, 1e-9)
    step = 2 * math.acos(max(1 - tol / r, -1.0)) if tol < r else math.pi / 2
    step = min(max(step, 1e-3), math.pi / 6)
    n = max(int(math.ceil(2 * math.pi * turns / step)), 8)
    a = np.linspace(0.0, 2 * math.pi * turns, n + 1)
    return np.column_stack([cx + r * np.cos(a), cy + r * np.sin(a), np.linspace(z0, z1, n + 1)])


def _heights(stock, targets, retract_height, clearance):
    top = max([float(stock["zmax"])] + [t["z_top"] for t in targets])
    return top + clearance


def _rz(stock, ztop, retract_height):
    """R plane: above the hole's top and above the stock top, so a retract always clears the stock."""
    return max(float(ztop), float(stock["zmax"])) + float(retract_height)


def _order(targets):
    """Nearest-next order from the first target."""
    todo, out = list(targets), []
    pos = np.zeros(2)
    while todo:
        k = min(range(len(todo)), key=lambda i: (todo[i]["x"] - pos[0]) ** 2 + (todo[i]["y"] - pos[1]) ** 2)
        t = todo.pop(k)
        out.append(t)
        pos = np.array([t["x"], t["y"]])
    return out


def _peck_cycle(t, stock, clear_z, peck, mode, chip_retract, retract_height, dwell, orbit_radius=0.0,
                orbit_turns=1.0, tol=0.01) -> List[Move]:
    x, y, ztop, depth = t["x"], t["y"], t["z_top"], t["depth"]
    rz = _rz(stock, ztop, retract_height)
    depths = _peck_depths(depth, peck)
    mv: List[Move] = [("rapid", np.array([x, y, clear_z]), _Z)]
    if rz < clear_z - 1e-9:
        mv.append(("rapid", np.array([x, y, rz]), _Z))
    for k, d in enumerate(depths):
        zb = ztop - d if k < len(depths) - 1 else ztop - depth      # the last bottom is exactly ztop - depth
        if k == 0 and orbit_radius > 0:
            hx = _helix(x, y, orbit_radius, ztop, zb, orbit_turns, tol)
            mv.append(("rapid", np.array([hx[0, 0], hx[0, 1], rz]), _Z))
            mv.append(("plunge", hx[0], _Z))
            mv += [("feed", p, _Z) for p in hx[1:]]
            mv.append(("feed", np.array([x, y, zb]), _Z))
        else:
            mv.append(("plunge", np.array([x, y, zb]), _Z))
        if k < len(depths) - 1:
            if mode == "chip":
                mv.append(("rapid", np.array([x, y, min(zb + chip_retract, rz)]), _Z))
            else:                                         # full retract: out of the hole, then back down
                mv.append(("rapid", np.array([x, y, rz]), _Z))
                mv.append(("rapid", np.array([x, y, min(zb + _GAP, rz)]), _Z))
    if dwell > 0:
        mv.append(("dwell", np.array([x, y, ztop - depth]), _Z, float(dwell)))
    mv.append(("rapid", np.array([x, y, rz]), _Z))
    mv.append(("rapid", np.array([x, y, clear_z]), _Z))
    return mv


def drill(tris, mesh, tool, stock, points=None, depth=0.0, peck=0.0, peck_mode="full", chip_retract=0.0,
          retract_height=cam.ENTRY_HEIGHT, dwell=0.0, clearance=20.0, diameter_range=None, **_) -> List[Move]:
    """Drill holes. Hole targets: `points` (list of (x, y), (x, y, z_top) or dicts x, y, z_top, depth), or, when
    None, the vertical cylindrical holes detected in the part (detect_holes), optionally limited to
    diameter_range = (min, max) mm.

    depth (mm): below each hole's top; with detected holes, 0 means the detected hole depth. peck (mm): the largest
    advance per peck, 0 = one plunge. peck_mode "full": retract to the R plane (above the hole and the stock top by
    retract_height) between pecks, re-enter `_GAP` above the last bottom. "chip": retract only chip_retract (mm,
    required > 0) for chip breaking. dwell (s): at the bottom, 0 = none. The tip reaches exactly z_top - depth."""
    if peck_mode not in ("full", "chip"):
        raise ValueError("peck_mode must be 'full' or 'chip'")
    if peck_mode == "chip" and peck > 0 and chip_retract <= 0:
        raise ValueError("chip peck mode needs chip_retract > 0")
    tg = _order(_targets(tris, stock, points, depth, diameter_range))
    clear_z = _heights(stock, tg, retract_height, clearance)
    mv: List[Move] = []
    for t in tg:
        mv += _peck_cycle(t, stock, clear_z, peck, peck_mode, chip_retract, retract_height, dwell)
    return mv


def core_drill(tris, mesh, tool, stock, points=None, depth=0.0, peck=0.0, retract_height=cam.ENTRY_HEIGHT,
               dwell=0.0, orbit_radius=0.0, orbit_turns=1.0, tolerance=0.01, clearance=20.0, diameter_range=None,
               **_) -> List[Move]:
    """Stone core drilling. Like drill, but peck > 0 is required and every peck ends in a clearing retract to the R
    plane (slurry and cuttings leave the kerf). orbit_radius (mm, 0 = off): the first peck is entered on a helix of
    that radius about the hole axis over orbit_turns turns, then the bit returns to the axis, so the crown does not
    walk on the stone surface. The cut is the annulus of the bit; its diameter is the tool's."""
    if peck <= 0:
        raise ValueError("core drilling needs peck > 0 (a clearing retract per peck)")
    tg = _order(_targets(tris, stock, points, depth, diameter_range))
    clear_z = _heights(stock, tg, retract_height, clearance)
    mv: List[Move] = []
    for t in tg:
        mv += _peck_cycle(t, stock, clear_z, peck, "full", 0.0, retract_height, dwell, orbit_radius, orbit_turns,
                          tolerance)
    return mv


def tap_feed(pitch: float, rpm: float) -> float:
    """Rigid tapping feed, mm/min = pitch (mm per revolution) x spindle speed (rev/min)."""
    return float(pitch) * float(rpm)


def tap(tris, mesh, tool, stock, points=None, depth=0.0, pitch=0.0, rpm=0.0, retract_height=cam.ENTRY_HEIGHT,
        dwell=0.0, clearance=20.0, diameter_range=None, **_) -> List[Move]:
    """Tapping: ("tap_in") from the R plane to the bottom, optional dwell, ("tap_out") back to the R plane with the
    spindle reversed. Both feed at pitch x rpm (mm/min), carried in the move (4th element) with the rpm (5th)."""
    if pitch <= 0 or rpm <= 0:
        raise ValueError("tap needs pitch (mm per revolution) > 0 and rpm > 0")
    f = tap_feed(pitch, rpm)
    tg = _order(_targets(tris, stock, points, depth, diameter_range))
    clear_z = _heights(stock, tg, retract_height, clearance)
    mv: List[Move] = []
    for t in tg:
        x, y, ztop = t["x"], t["y"], t["z_top"]
        rz = _rz(stock, ztop, retract_height)
        mv.append(("rapid", np.array([x, y, clear_z]), _Z))
        if rz < clear_z - 1e-9:
            mv.append(("rapid", np.array([x, y, rz]), _Z))
        mv.append(("tap_in", np.array([x, y, ztop - t["depth"]]), _Z, f, float(rpm)))
        if dwell > 0:
            mv.append(("dwell", np.array([x, y, ztop - t["depth"]]), _Z, float(dwell)))
        mv.append(("tap_out", np.array([x, y, rz]), _Z, f, float(rpm)))
        mv.append(("rapid", np.array([x, y, clear_z]), _Z))
    return mv


def bore(tris, mesh, tool, stock, points=None, diameter=0.0, depth=0.0, pitch=1.0, tolerance=0.01,
         retract_height=cam.ENTRY_HEIGHT, clearance=20.0, diameter_range=None, **_) -> List[Move]:
    """Helical bore: the tool centre follows a helix of radius (diameter - tool diameter) / 2 about the hole axis,
    descending `pitch` mm per turn (counter-clockwise seen from +z, climb for an M3 spindle on the hole wall), then
    one full circle at the bottom, then a move to the axis and out. The helix is dense points with chord sag at
    most `tolerance`. diameter: the hole diameter (0 = the detected diameter). The point list is as in drill."""
    d_tool = float(tool.diameter)
    tg = _order(_targets(tris, stock, points, depth, diameter_range))
    clear_z = _heights(stock, tg, retract_height, clearance)
    mv: List[Move] = []
    for t in tg:
        D = float(diameter) if diameter > 0 else t["diameter"]
        if D <= d_tool:
            raise ValueError("bore diameter {} must exceed the tool diameter {}".format(D, d_tool))
        if pitch <= 0:
            raise ValueError("bore pitch must be > 0")
        x, y, ztop, dep = t["x"], t["y"], t["z_top"], t["depth"]
        r = (D - d_tool) / 2
        rz = _rz(stock, ztop, retract_height)
        zb = ztop - dep
        hx = _helix(x, y, r, ztop, zb, dep / pitch, tolerance)
        hx[-1, 2] = zb
        fl = _helix(x, y, r, zb, zb, 1.0, tolerance)
        mv += [("rapid", np.array([hx[0, 0], hx[0, 1], clear_z]), _Z)]
        if rz < clear_z - 1e-9:
            mv.append(("rapid", np.array([hx[0, 0], hx[0, 1], rz]), _Z))
        mv.append(("plunge", hx[0], _Z))
        mv += [("feed", p, _Z) for p in hx[1:]]
        mv += [("feed", p, _Z) for p in fl[1:]]
        mv.append(("feed", np.array([x, y, zb]), _Z))
        mv.append(("rapid", np.array([x, y, rz]), _Z))
        mv.append(("rapid", np.array([x, y, clear_z]), _Z))
    return mv


# ------------------------------------------------------------------------------------------------------------------
# Rest machining

def _resized(cutter, radius: float):
    """The cutter with another radius (a bull nose keeps its corner-to-radius ratio)."""
    c = lc.Cutter()
    c.kind = cutter.kind
    c.radius = float(radius)
    c.corner = cutter.corner * radius / cutter.radius if cutter.kind == lc.CutterKind.Bull else cutter.corner
    c.half_angle = cutter.half_angle
    return c


def _field_on(fld, cutter, allowance) -> np.ndarray:
    """h of another cutter on fld's grid (allowance as in cam._Field)."""
    g = cam._offset_cutter(cutter, float(allowance))
    H = np.full(fld.H.shape, -math.inf)
    for j, y in enumerate(fld.ys):
        for i, x in enumerate(fld.xs):
            z = lc.drop_cutter(g, fld.mesh, float(x), float(y))
            if math.isfinite(z):
                H[j, i] = z + float(allowance)
    return H


def _stock_top(stock, xs, ys) -> np.ndarray:
    """Top surface height of a leftover stock at the grid (xs, ys): -inf where the column has no material. stock is
    an SdfStock, or a tuple (xs, ys, Z) of a height field (mm) sampled by nearest cell."""
    X, Y = np.meshgrid(xs, ys)
    if isinstance(stock, tuple):
        gx, gy, Z = (np.asarray(a, dtype=float) for a in stock)
        i = np.clip(np.rint((X - gx[0]) / (gx[1] - gx[0])).astype(int), 0, len(gx) - 1)
        j = np.clip(np.rint((Y - gy[0]) / (gy[1] - gy[0])).astype(int), 0, len(gy) - 1)
        return Z[j, i]
    s, o, c = stock.sdf, stock.origin, stock.cell
    i = np.clip(np.rint((X - o[0]) / c).astype(int), 0, s.shape[0] - 1)
    j = np.clip(np.rint((Y - o[1]) / c).astype(int), 0, s.shape[1] - 1)
    col = s[i, j, :]                                    # (ny, nx, nz)
    occ = col <= 0
    has = occ.any(axis=2)
    nz = col.shape[2]
    k = nz - 1 - np.argmax(occ[:, :, ::-1], axis=2)
    k0 = np.minimum(k, nz - 2)
    a = np.take_along_axis(col, k0[..., None], 2)[..., 0]
    b = np.take_along_axis(col, (k0 + 1)[..., None], 2)[..., 0]
    t = np.where(b > a, -a / np.where(b > a, b - a, 1.0), 0.0)
    z = o[2] + c * (k0 + np.clip(t, 0.0, 1.0))
    return np.where(has, z, -math.inf)


def rest_region(tris, mesh, tool, previous_tool_radius=None, previous_tool=None, leftover=None, threshold=0.05,
                allowance=0.0, cell=0.0, also_previous=False):
    """(field, mask): the xy region the tool must still cut. field is the tool's cam._Field (grid xs, ys, h, H);
    mask is a boolean grid on it.

    From the previous tool: the same tool kind at previous_tool_radius (mm), or previous_tool (an lc.Tool). The tool
    reaches lower than the previous one where h_small < h_large - threshold (h is the drop-cutter tip height, so the
    difference is the material the larger tool could not reach). From `leftover`: an SdfStock, or (xs, ys, Z)
    height-field tuple of the stock after the earlier operations: the region is where its top lies above
    h_small + threshold. leftover wins when both are given, unless also_previous is set: then the region is the
    union of both. The stock is compared at the tool centre only, so material beside the centre path (the fillet a
    larger round tool leaves in an inside corner) shows only through the previous tool; also_previous adds it.
    Samples where the tool is off the part (h = -inf) are never in the region."""
    fld = cam._Field(tris, mesh, tool, allowance, cell)
    Hs = fld.H
    mask = np.zeros(Hs.shape, bool)
    if leftover is not None:
        top = _stock_top(leftover, fld.xs, fld.ys)
        with np.errstate(invalid="ignore"):
            mask = np.isfinite(Hs) & np.isfinite(top) & (top - Hs > threshold)
    if leftover is None or (also_previous and (previous_tool is not None or previous_tool_radius)):
        base = lc.cutter_of(tool)
        if previous_tool is not None:
            big = lc.cutter_of(previous_tool)
        elif previous_tool_radius:
            big = _resized(base, float(previous_tool_radius))
        else:
            raise ValueError("rest needs previous_tool_radius, previous_tool or leftover")
        Hl = _field_on(fld, big, allowance)
        with np.errstate(invalid="ignore"):
            mask = mask | (np.isfinite(Hs) & np.isfinite(Hl) & (Hl - Hs > threshold))
        fld.ceiling = Hl if leftover is None else np.fmax(top, Hl)
    elif leftover is not None:
        fld.ceiling = top
    mask &= Hs >= fld.floor - cam._EPS
    return fld, mask


SKIM_GAP = 2.0      # mm above the highest stone a skim link can meet (above the previous tool's scallop)


def _max_filter(A: np.ndarray, r: int) -> np.ndarray:
    """Max over a (2r+1)^2 window (edges padded with -inf); -inf stays -inf only where the whole window is."""
    out = A.copy()
    for dj in range(-r, r + 1):
        for di in range(-r, r + 1):
            if dj == 0 and di == 0:
                continue
            S = np.full_like(A, -math.inf)
            ys = slice(max(dj, 0), A.shape[0] + min(dj, 0))
            yd = slice(max(-dj, 0), A.shape[0] + min(-dj, 0))
            xs = slice(max(di, 0), A.shape[1] + min(di, 0))
            xd = slice(max(-di, 0), A.shape[1] + min(-di, 0))
            S[yd, xd] = A[ys, xs]
            out = np.fmax(out, S)
    return out


def _skim_fn(fld, tool, clear_z: float):
    """skim(p0, p1) for cam._link: the highest stone along the straight link (the surface the previous tool left, or
    the leftover stock top: no stone above it can remain), grown by this tool's radius, plus SKIM_GAP. None where
    the link crosses a cell with no known ceiling (off the part), or no ceiling is known at all."""
    C = getattr(fld, "ceiling", None)
    if C is None:
        return None
    G = _max_filter(np.where(np.isfinite(C), C, -math.inf), int(math.ceil(tool.diameter / 2 / fld.cell)) + 1)
    x0, y0, c = float(fld.xs[0]), float(fld.ys[0]), float(fld.cell)

    def skim(p0, p1):
        n = max(2, int(math.ceil(math.hypot(p1[0] - p0[0], p1[1] - p0[1]) / c)) + 1)
        t = np.linspace(0.0, 1.0, n)
        i = np.clip(np.rint((p0[0] + (p1[0] - p0[0]) * t - x0) / c).astype(int), 0, G.shape[1] - 1)
        j = np.clip(np.rint((p0[1] + (p1[1] - p0[1]) * t - y0) / c).astype(int), 0, G.shape[0] - 1)
        g = G[j, i]
        if not np.all(np.isfinite(g)):
            return None
        z = float(g.max()) + SKIM_GAP
        return z if z < clear_z else None
    return skim


def _order_runs(runs: List[np.ndarray]) -> List[np.ndarray]:
    """Greedy nearest-neighbour order of the runs (each keeps its direction): from the end of one run to the
    closest start of a run not yet cut."""
    if len(runs) < 3:
        return runs
    S = np.array([r[0, :2] for r in runs])
    left = np.ones(len(runs), bool)
    out, k = [], 0
    for _ in range(len(runs)):
        left[k] = False
        out.append(runs[k])
        if not left.any():
            break
        d = np.hypot(*(S - runs[k][-1, :2]).T)
        d[~left] = np.inf
        k = int(np.argmin(d))
    return out


def rest(tris, mesh, tool, stock, previous_tool_radius=None, previous_tool=None, leftover=None, threshold=0.05,
         mode="raster", stepover=1.0, stepdown=1.0, overlap=0.5, allowance=0.0, tolerance=0.01, clearance=20.0,
         direction="climb", cell=0.0, also_previous=False, low_links=True, **_) -> List[Move]:
    """Rest machining: finishing paths of `tool` only where the previous tool left more than `threshold` mm (see
    rest_region). mode "raster": zig-zag passes along x at `stepover` mm, clipped to the region grown by `overlap`
    mm. mode "waterline": constant-z loops `stepdown` apart, clipped the same way. Empty region: no moves.
    low_links: order the runs nearest first and link them SKIM_GAP above the highest stone along the link (see
    _skim_fn) instead of at the clearance plane; off the part, or with no known ceiling, the clearance plane."""
    fld, mask = rest_region(tris, mesh, tool, previous_tool_radius, previous_tool, leftover, threshold, allowance,
                            cell, also_previous)
    mask = cam._dilate(mask, int(math.ceil(max(overlap, 0.0) / fld.cell)))
    if not mask.any():
        return []
    if mode == "waterline":
        runs = cam._waterline_runs(fld, stepdown, tolerance, direction, mask)
    elif mode == "raster":
        R = tool.diameter / 2
        passes = lc.zigzag(fld.cutter, mesh, *cam._part_rect(fld.lo, fld.hi, R), max(stepover, 0.05),
                           cam._params(tolerance, fld.lo[2], fld.cutter.radius))
        runs = []
        for ps in passes:
            if not len(ps):
                continue
            P = np.array([[c.x, c.y, c.z + fld.a] for c in ps])
            contact = np.array([bool(c.contact) for c in ps])
            runs += cam._runs(P, contact & fld.lookup(mask, P[:, 0], P[:, 1]))
    else:
        raise ValueError("rest mode must be 'raster' or 'waterline'")
    clear_z = stock["zmax"] + clearance
    if not low_links:
        return cam._link(runs, clear_z)
    # the region is scattered: link the runs nearest first and skim between them just above the stone the previous
    # tool left, instead of climbing to the clearance plane after every short run
    return cam._link(_order_runs([r for r in runs if len(r)]), clear_z, skim=_skim_fn(fld, tool, clear_z))


# ------------------------------------------------------------------------------------------------------------------
# Pencil: the concave creases a ball touches at two or more points

def _chain(P: np.ndarray, link: float, min_pts: int) -> List[np.ndarray]:
    """Greedy nearest-neighbour chains of points; a gap above `link` ends a chain."""
    n = len(P)
    if n == 0:
        return []
    cells: Dict[tuple, List[int]] = {}
    for k, p in enumerate(P):
        cells.setdefault((int(p[0] // link), int(p[1] // link)), []).append(k)
    used = np.zeros(n, bool)

    def near(p, prev_dir):
        best, bk = None, -1
        cx, cy = int(p[0] // link), int(p[1] // link)
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for k in cells.get((cx + dx, cy + dy), ()):
                    if used[k]:
                        continue
                    d = P[k, :2] - p[:2]
                    L = float(np.hypot(*d))
                    if L > link:
                        continue
                    if prev_dir is not None and L > 0 and float(d @ prev_dir) / L < -0.3:
                        continue
                    if best is None or L < best:
                        best, bk = L, k
        return bk

    def extend(start):
        out, prev_dir, cur = [], None, start
        while True:
            k = near(P[cur], prev_dir)
            if k < 0:
                return out
            used[k] = True
            d = P[k, :2] - P[cur, :2]
            L = float(np.hypot(*d))
            prev_dir = d / L if L > 0 else prev_dir
            out.append(k)
            cur = k

    chains = []
    for s in range(n):
        if used[s]:
            continue
        used[s] = True
        fwd = extend(s)
        bwd = extend(s)
        idx = bwd[::-1] + [s] + fwd
        if len(idx) >= min_pts:
            chains.append(P[idx])
    return chains


def _order_chains(chains: List[np.ndarray]) -> List[np.ndarray]:
    """Nearest-endpoint order; a chain may be reversed to start at its nearer end."""
    todo, out, pos = list(chains), [], np.zeros(2)
    while todo:
        best = None
        for n, c in enumerate(todo):
            for rev in (False, True):
                e = c[-1] if rev else c[0]
                d = float(np.hypot(*(e[:2] - pos)))
                if best is None or d < best[0]:
                    best = (d, n, rev)
        _, n, rev = best
        c = todo.pop(n)
        c = c[::-1] if rev else c
        out.append(c)
        pos = c[-1, :2]
    return out


_DIRS = ((1, 0), (0, 1), (1, 1), (1, -1))


def pencil_paths(tris, mesh, tool, min_angle_deg=15.0, cell=0.0, min_length=0.0, allowance=0.0, tolerance=0.01,
                 refine=16) -> List[np.ndarray]:
    """Tip polylines (n, 3) along concave creases for a ball tool.

    Bi-tangency: where the ball touches the part at two or more points its centre lies on a kink of the offset
    surface z = h + R (h: the drop-cutter tip height). A concave crease is a valley kink: the slope angle
    atan(dh/dl) increases by at least `min_angle_deg` across a grid sample along some direction l (four
    directions are tried). A vertical wall on a floor is a step of h: its lower side is the valley. Candidates are
    thinned by non-maximum suppression along the best direction, each is located to cell/refine by sampling h
    across the crease (the sample with the largest slope-angle increase), joined by nearest-neighbour chains, and
    chains shorter than min_length (0 = 3 cells) are dropped. Every point is a drop-cutter sample, so it never
    gouges; `allowance` lifts it. Convex edges and smooth surfaces are not creases."""
    if tool.kind != lc.ToolKind.BallEndMill:
        raise ValueError("pencil needs a ball-end mill (got {})".format(lc.tool_kind_name(tool.kind)))
    fld = cam._Field(tris, mesh, tool, 0.0, cell)
    H, c = fld.H, fld.cell
    ny, nx = H.shape
    cut = lc.cutter_of(tool)
    thr = math.radians(min_angle_deg)
    M = np.full((ny, nx), -np.inf)
    D = np.zeros((ny, nx), int)
    for k, (di, dj) in enumerate(_DIRS):
        L = c * math.hypot(di, dj)
        m = np.full((ny, nx), -np.inf)
        # interior samples p with p - d and p + d on the grid
        js = slice(1, ny - 1) if dj else slice(0, ny)
        is_ = slice(1, nx - 1) if di else slice(0, nx)
        jj, ii = np.mgrid[js, is_]
        h0 = H[jj, ii]
        hf = H[jj + dj, ii + di]
        hb = H[jj - dj, ii - di]
        ok = np.isfinite(h0) & np.isfinite(hf) & np.isfinite(hb)
        with np.errstate(invalid="ignore"):
            ang = np.arctan((hf - h0) / L) - np.arctan((h0 - hb) / L)
        m[jj, ii] = np.where(ok, ang, -np.inf)
        better = m > M
        M[better] = m[better]
        D[better] = k
    cand = np.argwhere(M > thr)
    pts = []
    e = c / max(int(refine), 2)
    for j, i in cand:
        di, dj = _DIRS[D[j, i]]
        jn, in_ = j + dj, i + di
        jp, ip = j - dj, i - di
        if not (0 <= jn < ny and 0 <= in_ < nx and 0 <= jp < ny and 0 <= ip < nx):
            continue
        if M[jn, in_] > M[j, i] or M[jp, ip] > M[j, i]:
            continue
        u = np.array([di, dj], float)
        u /= np.linalg.norm(u)
        t = np.arange(-c, c + e / 2, e)
        x0, y0 = fld.xs[i], fld.ys[j]
        h = np.array([lc.drop_cutter(cut, mesh, x0 + s * u[0], y0 + s * u[1]) for s in t])
        if not np.all(np.isfinite(h)):
            continue
        ang = np.arctan(np.diff(h) / e)                    # slope angle of each sub-interval
        inc = ang[1:] - ang[:-1]                           # increase at sample 1 .. n-2
        k = int(np.argmax(inc))
        if inc[k] < 0.5 * thr:
            continue
        s = t[k + 1]
        pts.append((x0 + s * u[0], y0 + s * u[1], h[k + 1]))
    if not pts:
        return []
    P = np.array(pts)
    key = np.round(P[:, :2] / (0.3 * c)).astype(int)       # merge repeats of the same crease sample
    _, first = np.unique(key, axis=0, return_index=True)
    P = P[np.sort(first)]
    chains = _chain(P, 2.5 * c, 3)
    keep = []
    lmin = min_length if min_length > 0 else 3 * c
    for ch in chains:
        if float(np.linalg.norm(np.diff(ch[:, :2], axis=0), axis=1).sum()) < lmin:
            continue
        if len(ch) > 6 and float(np.hypot(*(ch[0, :2] - ch[-1, :2]))) <= 2.5 * c:
            ch = np.vstack([ch, ch[:1]])                   # a closed crease
        ch = ch.copy()
        ch[:, 2] += allowance
        keep.append(ch)
    return _order_chains(keep)


def pencil(tris, mesh, tool, stock, min_angle_deg=15.0, cell=0.0, min_length=0.0, allowance=0.0, tolerance=0.01,
           clearance=20.0, **_) -> List[Move]:
    """Pencil finishing: the ball tool traces every concave crease once (pencil_paths), then simplifies the feed
    runs to `tolerance`."""
    runs = pencil_paths(tris, mesh, tool, min_angle_deg, cell, min_length, allowance, tolerance)
    return cam.simplify(cam._link(runs, stock["zmax"] + clearance), tolerance)


# ------------------------------------------------------------------------------------------------------------------
# Curvature-adaptive stepover

def local_stepover(R, scallop: float, kappa):
    """Largest stepover s (mm, measured along the surface between the two contact points, in the section normal to
    the passes) that keeps the scallop at or below `scallop` for a ball of radius R on a surface of normal curvature
    kappa (1/mm, positive convex, negative concave, 0 flat) across the passes. kappa may be an array.

    Source: the ball-end scallop on a curved surface (Vickers and Quan, 1989, whose effective radius
    1/R_eff = 1/R + kappa gives h ~ s^2 / (8 R_eff) for small s). The exact two-circle form used here: in the
    section, the surface is a circle of signed radius rho = 1 / kappa about O; the ball centres of two adjacent
    passes lie at distance rho + R from O (rho < 0: concave, distance |rho| - R) at angles +-phi; the crest of the
    scallop is on the bisector at distance rho + h from O and at distance R from both centres, so
    1 - cos(phi) = h (2R - h) / (2 (rho + R) (rho + h)) and s = 2 |rho| phi. For kappa = 0 this is the flat
    formula s = 2 sqrt(h (2R - h)). Concave: smaller scallop than flat for the same s; convex: larger. s never
    exceeds the ball diameter. A concave radius not above R cannot be entered: the result is the cap."""
    R, h = float(R), float(scallop)
    if not 0 < h < R:
        raise ValueError("scallop must satisfy 0 < scallop < ball radius")
    k = np.asarray(kappa, dtype=float)
    flat = 2.0 * math.sqrt(h * (2 * R - h))
    out = np.full(k.shape, flat)
    big = np.abs(k) > 1e-9
    rho = np.where(big, 1.0 / np.where(big, k, 1.0), 1.0)
    a, b = rho + R, rho + h
    ok = big & (np.abs(a * b) > 1e-12)
    q = np.where(ok, h * (2 * R - h) / (4 * np.where(ok, a * b, 1.0)), -1.0)
    valid = ok & (q > 0) & (q < 1)
    phi = 2 * np.arcsin(np.sqrt(np.where(valid, q, 0.0)))
    s = np.where(valid, 2 * np.abs(rho) * phi, 2 * R)
    out = np.where(big, np.minimum(s, 2 * R), out)
    return out if k.ndim else float(out)


def _stepover_field(fld, scallop: float, window: float, min_stepover: float):
    """xy stepover (cross-feed, passes along x) on fld's grid, and the mask of cells that count (inside the part's
    xy box, h finite). Slope and curvature come from the offset surface z = h by central differences over `window`.

    Within max(R, window) of the box edge the offset surface is affected by the part's rim (a contact point can lie
    up to R from the ball centre), so the stepover is computed only inside that margin and carried outwards
    unchanged (nearest computed cell). A part smaller than twice the margin gets the flat formula."""
    H, c, R = fld.H, fld.cell, fld.R
    ny, nx = H.shape
    flat = 2.0 * math.sqrt(scallop * (2 * R - scallop))
    margin = max(R, window) + c
    xi = np.nonzero((fld.xs >= fld.lo[0] + margin) & (fld.xs <= fld.hi[0] - margin))[0]
    yi = np.nonzero((fld.ys >= fld.lo[1] + margin) & (fld.ys <= fld.hi[1] - margin))[0]
    X, Y = np.meshgrid(fld.xs, fld.ys)
    box = (X >= fld.lo[0]) & (X <= fld.hi[0]) & (Y >= fld.lo[1]) & (Y <= fld.hi[1]) & np.isfinite(H)
    if len(xi) == 0 or len(yi) == 0:
        return np.maximum(np.full(H.shape, flat), min_stepover), box
    m = max(int(round(window / c)), 1)
    Hn = np.where(np.isfinite(H), H, np.nan)
    padH = np.pad(Hn, m, constant_values=np.nan)

    def sh(dj, di):
        return padH[m + dj * m:m + dj * m + ny, m + di * m:m + di * m + nx]

    w = m * c
    fx = (sh(0, 1) - sh(0, -1)) / (2 * w)
    fy = (sh(1, 0) - sh(-1, 0)) / (2 * w)
    fxx = (sh(0, 1) - 2 * Hn + sh(0, -1)) / w ** 2
    fyy = (sh(1, 0) - 2 * Hn + sh(-1, 0)) / w ** 2
    fxy = (sh(1, 1) - sh(1, -1) - sh(-1, 1) + sh(-1, -1)) / (4 * w ** 2)
    W = np.sqrt(1 + fx ** 2 + fy ** 2)
    vx, vy = -fx * fy, 1 + fx ** 2                               # tangent normal to the pass, in xy
    vHv = fxx * vx ** 2 + 2 * fxy * vx * vy + fyy * vy ** 2
    gv = fx * vx + fy * vy
    kc = -vHv / (W * (vx ** 2 + vy ** 2 + gv ** 2))              # curvature of the offset surface, convex > 0
    with np.errstate(invalid="ignore", divide="ignore"):
        kp = kc / (1 - R * kc)                                   # of the part: offset surfaces share the normal
        kp = np.where(np.isfinite(kp), kp, 0.0)
        s_surf = local_stepover(R, scallop, kp)
        # the passes are ball-centre curves: centre spacing = contact spacing x (1 + R kappa) in the section
        s = s_surf * np.maximum(1 + R * kp, 0.05) * np.sqrt(1 + fx ** 2) / W
    s = np.where(np.isfinite(s), s, flat)
    jj = np.clip(np.arange(ny), yi[0], yi[-1])
    ii = np.clip(np.arange(nx), xi[0], xi[-1])
    s = s[np.ix_(jj, ii)]                                        # carry the interior values out to the edge
    return np.maximum(s, min_stepover), box

def adaptive_raster_paths(tris, mesh, tool, scallop, adaptive=True, stepover=0.0, window=0.0, min_stepover=0.0,
                          allowance=0.0, tolerance=0.01, cell=0.0):
    """(passes, info): tip passes (n, 3) along x for a ball tool, alternating direction, with the cross-feed spacing
    chosen so the scallop stays at or below `scallop`. See local_stepover for the formula.

    adaptive True: marching across the part from y = ymin - R, the next pass is placed at the smallest local xy
    stepover found anywhere in the strip it spans (strip refined twice). adaptive False: the constant spacing equal
    to the smallest local stepover over the whole part, the same worst scallop. stepover > 0 overrides with that
    constant spacing (for comparison, e.g. the flat formula). window (mm, 0 = R): the span of the central
    differences for slope and curvature. min_stepover (mm, 0 = R / 20): floor on the spacing, so a vertical wall
    does not demand infinitely many passes (steep walls belong to waterline / steep_shallow).
    info: ys (pass positions), s_min, s_max (local xy stepover over the part), n_passes."""
    if tool.kind != lc.ToolKind.BallEndMill:
        raise ValueError("adaptive stepover needs a ball-end mill (got {})".format(lc.tool_kind_name(tool.kind)))
    if scallop <= 0:
        raise ValueError("scallop (mm) must be > 0")
    fld = cam._Field(tris, mesh, tool, allowance, cell)
    R = fld.R
    S, inside = _stepover_field(fld, scallop, window if window > 0 else R, min_stepover if min_stepover > 0 else R / 20)
    flat = 2.0 * math.sqrt(scallop * (2 * R - scallop))
    ys_g = fld.ys
    Sc = np.where(inside, S, flat)
    s_min, s_max = float(Sc[inside].min()) if inside.any() else flat, float(Sc[inside].max()) if inside.any() else flat

    def strip_min(y0, y1):
        j0 = max(int(np.searchsorted(ys_g, y0, side="left")) - 1, 0)
        j1 = min(int(np.searchsorted(ys_g, y1, side="right")), len(ys_g) - 1)
        return float(Sc[j0:j1 + 1].min())

    y_lo, y_hi = fld.lo[1] - R + cam._EDGE_INSET, fld.hi[1] + R - cam._EDGE_INSET
    ys = [y_lo]
    while ys[-1] < y_hi - 1e-9:
        y = ys[-1]
        if stepover > 0:
            s = stepover
        elif not adaptive:
            s = s_min
        else:
            s = strip_min(y, y + flat)
            for _ in range(2):
                s = strip_min(y, y + s)
        ys.append(min(y + s, y_hi) if y + s < y_hi else y_hi)
    cut = fld.cutter
    x0, x1 = cam._part_rect(fld.lo, fld.hi, R)[:2]
    prm = cam._params(tolerance, fld.lo[2], R)
    passes = []
    for n, y in enumerate(ys):
        a, b = (x0, x1) if n % 2 == 0 else (x1, x0)
        pts = lc.adaptive_line(cut, mesh, a, y, b, y, prm)
        P = np.array([[p.x, p.y, (p.z if p.contact else fld.lo[2]) + fld.a] for p in pts])
        if len(P):
            passes.append(P)
    return passes, {"ys": np.array(ys), "s_min": s_min, "s_max": s_max, "n_passes": len(passes)}


def adaptive_raster(tris, mesh, tool, stock, scallop=0.0, adaptive=True, stepover=0.0, window=0.0,
                    min_stepover=0.0, allowance=0.0, tolerance=0.01, clearance=20.0, cell=0.0, **_) -> List[Move]:
    """Raster finishing along x with a ball tool, the cross-feed spacing set from the local curvature so the
    scallop is at most `scallop` mm (required). See adaptive_raster_paths and local_stepover."""
    if scallop <= 0:
        raise ValueError("adaptive_raster needs scallop (mm) > 0")
    passes, _info = adaptive_raster_paths(tris, mesh, tool, scallop, adaptive, stepover, window, min_stepover,
                                          allowance, tolerance, cell)
    return cam._link(passes, stock["zmax"] + clearance)


# ------------------------------------------------------------------------------------------------------------------
# Leftover from a Z-map of earlier milling steps (rest machining without a stock simulation)

def cl_passes(moves: Sequence[Move]):
    """Cutting passes (lists of lc.CLPoint) of a move list: the feed and plunge moves, split at every other move."""
    passes, cur = [], []
    for m in moves:
        if m[0] in ("feed", "plunge"):
            c = lc.CLPoint()
            c.x, c.y, c.z, c.contact = float(m[1][0]), float(m[1][1]), float(m[1][2]), True
            cur.append(c)
        elif cur:
            passes.append(cur)
            cur = []
    if cur:
        passes.append(cur)
    return passes


def zmap_leftover(cuts, stock, cell: float = 1.0):
    """The stock top after earlier milling: (xs, ys, Z) with Z in mm (rows y, columns x), the form `rest` takes as
    `leftover`. cuts: [(tool, moves), ...] in program order, vertical-axis paths of tools with a drop-cutter model.
    The Z-map starts as the stock box (a dict xmin .. zmax) cut along every feed and plunge move at `cell` mm."""
    if cell <= 0:
        raise ValueError("leftover_cell must be > 0")
    zm = lc.make_stock(stock["xmin"], stock["xmax"], stock["ymin"], stock["ymax"], float(cell), stock["zmax"])
    for tool, moves in cuts:
        passes = cl_passes(moves)
        if passes:
            lc.cut_passes(zm, lc.cutter_of(tool), passes)
    xs = zm.x0 + zm.cell * np.arange(zm.nx)
    ys = zm.y0 + zm.cell * np.arange(zm.ny)
    return xs, ys, np.asarray(zm.z, dtype=float).reshape(zm.ny, zm.nx)


# ------------------------------------------------------------------------------------------------------------------
# Engraving and chamfering with a V-bit (cone cutter)

_RAY = None


def _ray_cutter():
    """A flat cutter of 1 micrometre radius: its drop-cutter height is the part's height straight under the tip."""
    global _RAY
    if _RAY is None:
        c = lc.Cutter()
        c.kind = lc.CutterKind.Flat
        c.radius = 1e-3
        _RAY = c
    return _RAY


def surface_z(mesh, x: float, y: float) -> float:
    """Height of the part straight under (x, y), or -inf where the part is not."""
    return float(lc.drop_cutter(_ray_cutter(), mesh, float(x), float(y)))


def parse_paths(text) -> List[np.ndarray]:
    """Polylines (n, 2) from text 'x,y; x,y; ... | x,y; ...' (mm, polylines separated by '|'; repeat the first point
    to close one). None or empty text gives []. A list of point lists passes through."""
    if text is None:
        return []
    if not isinstance(text, str):
        return [np.asarray(p, dtype=float).reshape(-1, 2) for p in text] if len(text) else []
    out = []
    for chunk in text.replace("\n", "|").split("|"):
        pts = []
        for s in chunk.split(";"):
            if not s.strip():
                continue
            v = [float(x) for x in s.split(",")]
            if len(v) != 2:
                raise ValueError("a path point is 'x,y' in mm (got '{}')".format(s.strip()))
            pts.append(v)
        if not pts:
            continue
        if len(pts) < 2:
            raise ValueError("a path needs at least 2 points (got 1: '{}')".format(chunk.strip()))
        out.append(np.asarray(pts, dtype=float))
    return out


def _densify(P: np.ndarray, step: float) -> np.ndarray:
    """Points along the polyline P (n, 2), at most `step` apart, keeping every vertex."""
    out = [P[0]]
    for a, b in zip(P[:-1], P[1:]):
        L = float(np.hypot(*(b - a)))
        k = max(int(math.ceil(L / step - 1e-9)), 1)
        out += [a + (b - a) * (i / k) for i in range(1, k + 1)]
    return np.asarray(out)


def _cone(tool, what: str):
    """(half angle rad, tip flat radius mm, tool radius mm) of a V-bit or chamfer bit."""
    ang = float(getattr(tool, "tip_angle", 0.0) or 0.0)
    if ang <= 0 or ang >= 180:
        raise ValueError("{} needs a V-bit or chamfer bit with a tip angle in (0, 180) degrees (tool {} has {:g})"
                         .format(what, tool.id, ang))
    return math.radians(ang / 2), float(tool.tip_diameter or 0.0) / 2, float(tool.diameter) / 2


def engrave_depth(tool, width: float) -> float:
    """Depth (mm) at which a V-bit cuts a groove `width` mm wide at the surface: the cone flank gives
    width = tip_diameter + 2 depth tan(half angle)."""
    half, t, R = _cone(tool, "engraving by width")
    if width > 2 * R + 1e-9:
        raise ValueError("width {:g} mm exceeds the tool diameter {:g} mm".format(width, 2 * R))
    if width <= 2 * t:
        raise ValueError("width {:g} mm is not above the tip flat {:g} mm".format(width, 2 * t))
    return (width - 2 * t) / (2 * math.tan(half))


def _surface_runs(P2: np.ndarray, mesh, dz: float, step: float, tol: float, top=None) -> List[np.ndarray]:
    """Tip runs (n, 3) over the xy polyline P2: tip z = the part height under the point + dz, runs split where the
    part is absent. `top(x, y)` overrides the part height."""
    D = _densify(P2, step)
    f = top or (lambda x, y: surface_z(mesh, x, y))
    zs = np.array([f(x, y) for x, y in D])
    return cam._runs(np.column_stack([D, zs + dz]), np.isfinite(zs))


def engrave(tris, mesh, tool, stock, paths="", width=0.0, depth=0.0, text="", height=0.0, step=0.5, tolerance=0.01,
            clearance=20.0, **_) -> List[Move]:
    """V-bit engraving along polylines. paths: 'x,y; x,y | x,y; ...' (mm). The tip follows the part's top surface
    (the height straight under each sample, drop-cutter of a pointed cutter) at `depth` below it. width > 0 sets
    the depth from the groove width at the surface (engrave_depth; it needs a tool with a tip angle); otherwise
    `depth` (mm) is used with any tool. step: the largest spacing of the samples (mm).

    text and height: lettering needs a single-stroke font, and lithocam ships none, so a non-empty text is refused:
    give the letter strokes as paths."""
    if text and str(text).strip():
        raise ValueError("lettering needs a single-stroke font and lithocam ships none: give the strokes as 'paths'")
    pl = parse_paths(paths)
    if not pl:
        raise ValueError("engrave needs paths 'x,y; x,y | x,y; ...' (mm)")
    d = engrave_depth(tool, float(width)) if width and width > 0 else float(depth)
    if d <= 0:
        raise ValueError("engrave needs width (> 0) or depth (> 0)")
    runs: List[np.ndarray] = []
    for P in pl:
        runs += _surface_runs(P, mesh, -d, max(float(step), 0.05), tolerance)
    return cam.simplify(cam._link(runs, stock["zmax"] + clearance), tolerance)


def top_outline(tris: np.ndarray, max_slope_deg: float = 1.0, tol: float = 0.05) -> List[np.ndarray]:
    """Closed outlines (n, 3), first point repeated, of the part's highest flat top: the boundary of the upward
    faces (normal within max_slope_deg of +z) whose mean height is within `tol` of the highest such face. Each loop
    runs with the material on its right (clockwise round the part, counter-clockwise round a hole), the climb
    direction for a clockwise spindle."""
    tris = np.asarray(tris, dtype=float).reshape(-1, 3, 3)
    n = _unit_normals(tris)
    flat = n[:, 2] >= math.cos(math.radians(max_slope_deg))
    if not flat.any():
        raise ValueError("the part has no upward face within {:g} degrees of +z".format(max_slope_deg))
    zc = tris[:, :, 2].mean(axis=1)
    ztop = float(zc[flat].max())
    sel = np.nonzero(flat & (zc >= ztop - tol))[0]

    def key(p):
        return tuple(np.round(p, 6))
    count: Dict[frozenset, int] = {}
    directed: Dict[tuple, List[tuple]] = {}
    pos: Dict[tuple, np.ndarray] = {}
    for t in sel:
        for a, b in ((0, 1), (1, 2), (2, 0)):
            ka, kb = key(tris[t, a]), key(tris[t, b])
            pos[ka], pos[kb] = tris[t, a], tris[t, b]
            e = frozenset((ka, kb))
            count[e] = count.get(e, 0) + 1
    for t in sel:
        for a, b in ((0, 1), (1, 2), (2, 0)):
            ka, kb = key(tris[t, a]), key(tris[t, b])
            if count[frozenset((ka, kb))] == 1:
                directed.setdefault(kb, []).append(ka)       # reversed: material on the right
    loops, used = [], set()
    for s in list(directed):
        for first in list(directed[s]):
            if (s, first) in used:
                continue
            loop, cur, nxt = [s], s, first
            used.add((s, first))
            while True:
                loop.append(nxt)
                cur = nxt
                if cur == s:
                    break
                cand = [k for k in directed.get(cur, []) if (cur, k) not in used]
                if not cand:
                    break
                used.add((cur, cand[0]))
                nxt = cand[0]
            if loop[-1] == s and len(loop) >= 4:
                loops.append(np.array([pos[k] for k in loop]))
    if not loops:
        raise ValueError("the highest flat top has no closed outline")
    return loops


def chamfer(tris, mesh, tool, stock, width=0.0, paths="", max_slope_deg=1.0, step=0.5, tolerance=0.01,
            clearance=20.0, **_) -> List[Move]:
    """Chamfer with a V-bit or chamfer bit. `width`: the chamfer's horizontal width on the top face (mm, required).
    The tool axis runs along the edge; the cone flank passes through the top edge's inner line `width` in from the
    edge, so the tip sits (width - tip flat radius) / tan(half angle) below the top, on the wall plane. With a flat
    tip a ledge of the tip radius stays on the wall. paths: polylines 'x,y; x,y | ...' (mm) on the edge; empty means
    the outline of the part's highest flat top (top_outline, max_slope_deg). Given polylines take the top height
    from the part under each point (the highest of the point and four 0.01 mm neighbours, so an edge point reads the
    top face, not the wall). The width must not exceed the tool radius. Corners: the tool follows the vertex, so an
    inside miter of the two chamfers is left rounded."""
    half, t, R = _cone(tool, "chamfer")
    w = float(width)
    if w <= 0:
        raise ValueError("chamfer needs width (mm) > 0")
    if w > R + 1e-9:
        raise ValueError("chamfer width {:g} mm exceeds the tool radius {:g} mm".format(w, R))
    if w <= t:
        raise ValueError("chamfer width {:g} mm is not above the tip flat radius {:g} mm".format(w, t))
    c = (w - t) / math.tan(half)
    step = max(float(step), 0.05)
    runs: List[np.ndarray] = []
    pl = parse_paths(paths)
    if pl:
        def top(x, y):
            return max(surface_z(mesh, x + dx, y + dy) for dx, dy in ((0, 0), (0.01, 0), (-0.01, 0), (0, 0.01),
                                                                      (0, -0.01)))
        for P in pl:
            runs += _surface_runs(P, mesh, -c, step, tolerance, top)
    else:
        for L in top_outline(tris, max_slope_deg):
            D = _densify(L[:, :2], step)
            runs.append(np.column_stack([D, np.full(len(D), float(L[0, 2]) - c)]))
    return cam.simplify(cam._link(runs, stock["zmax"] + clearance), tolerance)


# ------------------------------------------------------------------------------------------------------------------
# Flat lands: 2.5D finishing of horizontal areas

def flat_lands(tris: np.ndarray, max_slope_deg: float = 1.0, tol: float = 0.01):
    """[(z, triangle indices)] of the horizontal faces, highest first. A face is horizontal when its normal is within
    max_slope_deg of +z; faces are grouped by their highest vertex, a new group starting where the heights differ by
    more than max(tol, 1e-3) mm; z is the group's highest vertex, so a tool resting there never gouges."""
    tris = np.asarray(tris, dtype=float).reshape(-1, 3, 3)
    n = _unit_normals(tris)
    idx = np.nonzero(n[:, 2] >= math.cos(math.radians(max_slope_deg)))[0]
    if not len(idx):
        return []
    zt = tris[idx, :, 2].max(axis=1)
    order = np.argsort(-zt)
    idx, zt = idx[order], zt[order]
    gap = max(float(tol), 1e-3)
    groups, start = [], 0
    for k in range(1, len(idx) + 1):
        if k == len(idx) or zt[k - 1] - zt[k] > gap:
            groups.append((float(zt[start:k].max()), idx[start:k]))
            start = k
    return groups


def _land_mask(tris, tri_ids, xs, ys) -> np.ndarray:
    """Grid mask (len(ys), len(xs)) of the sample points that lie inside any of the faces (xy projection)."""
    mask = np.zeros((len(ys), len(xs)), bool)
    for t in tri_ids:
        A, B, C = tris[t, 0, :2], tris[t, 1, :2], tris[t, 2, :2]
        i0, i1 = np.searchsorted(xs, min(A[0], B[0], C[0]) - 1e-9), np.searchsorted(xs, max(A[0], B[0], C[0]) + 1e-9)
        j0, j1 = np.searchsorted(ys, min(A[1], B[1], C[1]) - 1e-9), np.searchsorted(ys, max(A[1], B[1], C[1]) + 1e-9)
        if i1 <= i0 or j1 <= j0:
            continue
        X, Y = np.meshgrid(xs[i0:i1], ys[j0:j1])
        den = (B[1] - C[1]) * (A[0] - C[0]) + (C[0] - B[0]) * (A[1] - C[1])
        if abs(den) < 1e-14:
            continue
        a = ((B[1] - C[1]) * (X - C[0]) + (C[0] - B[0]) * (Y - C[1])) / den
        b = ((C[1] - A[1]) * (X - C[0]) + (A[0] - C[0]) * (Y - C[1])) / den
        inside = (a >= -1e-9) & (b >= -1e-9) & (1 - a - b >= -1e-9)
        mask[j0:j1, i0:i1] |= inside
    return mask


def flat_land(tris, mesh, tool, stock, max_slope_deg=1.0, stepover=0.0, allowance=0.0, cell=0.0, tolerance=0.01,
              clearance=20.0, entry="plunge", ramp_angle=3.0, helix_radius=0.0, helix_pitch=1.0,
              **_) -> List[Move]:
    """2.5D finishing of the horizontal areas of the part with a flat or bull end mill. Faces within max_slope_deg
    of +z are grouped by height (flat_lands); at each height, highest first, a zig-zag along x at `stepover` mm
    (0 = 0.7 x the tool diameter) rides at that height wherever the tool centre is over the faces (grown by one grid
    cell) and the drop-cutter height there is that height within max(tolerance, 0.02) mm. That keeps the tool a
    radius off any wall that rises above the land (the inset) and lets it overhang an open edge, and it never
    gouges. allowance lifts every pass. cell: grid of the face mask, mm (0 = min(0.5, tool radius / 5))."""
    if tool.kind not in (lc.ToolKind.FlatEndMill, lc.ToolKind.BullNoseMill):
        raise ValueError("flat_land needs a flat or bull end mill (got {})".format(lc.tool_kind_name(tool.kind)))
    R = float(tool.diameter) / 2
    so = float(stepover) if stepover and stepover > 0 else 0.7 * float(tool.diameter)
    cell = float(cell) if cell and cell > 0 else min(0.5, max(R / 5.0, 0.02))
    eps = max(float(tolerance), 0.02)
    cutter = lc.cutter_of(tool)
    tris = np.asarray(tris, dtype=float).reshape(-1, 3, 3)
    runs: List[np.ndarray] = []
    entries: List[tuple] = []
    for z, ids in flat_lands(tris, max_slope_deg, tolerance):
        sub = tris[ids]
        lo, hi = sub.reshape(-1, 3)[:, :2].min(axis=0), sub.reshape(-1, 3)[:, :2].max(axis=0)
        xs = np.arange(lo[0] - cell, hi[0] + 1.5 * cell, cell)
        ys = np.arange(lo[1] - cell, hi[1] + 1.5 * cell, cell)
        land = cam._dilate(_land_mask(tris, ids, xs, ys), 1)
        y_lines = list(np.arange(lo[1], hi[1], so)) + [hi[1]]
        sample = cell / 2
        xl = np.arange(xs[0], xs[-1] + sample / 2, sample)
        for n, y in enumerate(y_lines):
            j = int(np.clip(np.rint((y - ys[0]) / cell), 0, len(ys) - 1))
            ii = np.clip(np.rint((xl - xs[0]) / cell).astype(int), 0, len(xs) - 1)
            on = np.array([bool(land[j, i]) and abs(lc.drop_cutter(cutter, mesh, float(x), float(y)) - z) <= eps
                           for i, x in zip(ii, xl)])
            P = np.column_stack([xl, np.full(len(xl), y), np.full(len(xl), z + float(allowance))])
            for r in cam._runs(P, on):
                runs.append(r if n % 2 == 0 else r[::-1])
                entries.append((z, (xs[0], xs[-1], ys[0], ys[-1])))
    if not runs:
        return []
    top = float(stock["zmax"]) + clearance
    if entry == "plunge":
        return cam._link(runs, top)
    moves: List[Move] = []
    for r, (z, rect) in zip(runs, entries):
        ent = cam.make_entry(entry, tool, ramp_angle, helix_radius, helix_pitch, rect,
                             (lambda x, y, zz=z + float(allowance): zz), tolerance)
        moves += cam._link([r], top, ent)
    return moves


GENERATORS = {"drill": drill, "core_drill": core_drill, "tap": tap, "bore": bore, "rest": rest, "pencil": pencil,
              "adaptive_raster": adaptive_raster, "engrave": engrave, "chamfer": chamfer, "flat_land": flat_land}

cam.GENERATORS.update(GENERATORS)   # registered with the studio's generators (cam.py imports this module last)
