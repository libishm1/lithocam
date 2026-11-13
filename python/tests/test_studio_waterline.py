# Copyright 2026 The lithocam authors. SPDX-License-Identifier: Apache-2.0
"""Waterline and steep / shallow finishing (lithocam.studio.cam): contour accuracy, no gouge, closed loops, milling
direction, coverage of the two families, and the Z-map leftover against a scallop bound."""
import math

import numpy as np
import pytest

try:
    from lithocam.studio.core import lc  # noqa: F401  (the compiled core, built from bindings/)
except ImportError:
    pytest.skip("_lithocam not built (bindings/); set LITHOCAM_CORE", allow_module_level=True)

from lithocam.studio import cam, catalog  # noqa: E402

TOL = 0.01


def dome(n=24, size=60.0, h=12.0):
    """Same part as test_studio_program.dome: a faceted paraboloid cap on a flat square plate (z = 0)."""
    xs = np.linspace(-size / 2, size / 2, n + 1)
    Z = lambda x, y: h * max(0.0, 1 - (x * x + y * y) / (size / 2) ** 2)   # noqa: E731
    tris = []
    for i in range(n):
        for j in range(n):
            a = (xs[i], xs[j], Z(xs[i], xs[j]))
            b = (xs[i + 1], xs[j], Z(xs[i + 1], xs[j]))
            c = (xs[i + 1], xs[j + 1], Z(xs[i + 1], xs[j + 1]))
            d = (xs[i], xs[j + 1], Z(xs[i], xs[j + 1]))
            tris += [(a, b, c), (a, c, d)]
    return np.array(tris, dtype=float)


def box(lx=40.0, ly=30.0, lz=15.0):
    """Closed box with vertical walls, base at z = 0."""
    v = np.array([[x, y, z] for z in (0, lz) for y in (0, ly) for x in (0, lx)], dtype=float)
    quads = [(0, 2, 3, 1), (4, 5, 7, 6), (0, 1, 5, 4), (2, 6, 7, 3), (0, 4, 6, 2), (1, 3, 7, 5)]
    return np.array([t for a, b, c, d in quads for t in ((v[a], v[b], v[c]), (v[a], v[c], v[d]))])


@pytest.fixture(scope="module")
def tools():
    lib = lc.wood_tool_library()
    ball = [t for t in lib.tools if t.kind == lc.ToolKind.BallEndMill][0]
    flat = [t for t in lib.tools if t.kind == lc.ToolKind.FlatEndMill and t.diameter < 7][0]
    return ball, flat


@pytest.fixture(scope="module")
def parts():
    out = {}
    for name, t in (("dome", dome()), ("box", box())):
        out[name] = (t, cam.trimesh(t), cam.stock_box(t))
    return out


def runs_of(moves):
    """Feed runs: each from its plunge point through the following feeds."""
    runs, cur = [], None
    for k, p, *_ in moves:
        if k == "plunge":
            cur = [p]
        elif k == "feed":
            cur.append(p)
        elif cur is not None:
            runs.append(np.array(cur))
            cur = None
    if cur is not None:
        runs.append(np.array(cur))
    return runs


def ring_max(c, mesh, x, y, r, n=16):
    return max(lc.drop_cutter(c, mesh, x + r * math.cos(a), y + r * math.sin(a))
               for a in np.linspace(0, 2 * math.pi, n, endpoint=False))


def ring_min(c, mesh, x, y, r, n=32):
    return min(lc.drop_cutter(c, mesh, x + r * math.cos(a), y + r * math.sin(a))
               for a in np.linspace(0, 2 * math.pi, n, endpoint=False))


@pytest.mark.parametrize("name,which", [("dome", 0), ("box", 0), ("box", 1)])
def test_waterline_on_contour_no_gouge_closed(parts, tools, name, which):
    tris, mesh, stock = parts[name]
    tool = tools[which]
    c = lc.cutter_of(tool)
    moves = cam.waterline(tris, mesh, tool, stock, stepdown=2.0, tolerance=TOL)
    runs = runs_of(moves)
    assert runs
    zgap = []
    for r in runs:
        assert np.allclose(r[0], r[-1]), "loop not closed"
        assert np.ptp(r[:, 2]) < 1e-5, "a loop is not at constant z"
        for p in r:
            h = lc.drop_cutter(c, mesh, p[0], p[1])
            assert p[2] >= h - 1e-9, "gouge at {}".format(p)
            # within TOL (xy) of the contour: the material (h >= z) is reached within TOL
            assert ring_max(c, mesh, p[0], p[1], TOL) >= p[2] - 1e-9, "point not on the contour: {}".format(p)
            if math.isfinite(h):
                zgap.append(p[2] - h)
        # the straight moves between points cut into the part by at most TOL: the midpoint is within TOL of the
        # no-gouge height vertically, or within TOL horizontally of a position that does not gouge (a wall)
        for x, y, z in (r[1:] + r[:-1]) / 2:
            if lc.drop_cutter(c, mesh, x, y) <= z + TOL + 1e-9:
                continue
            assert ring_min(c, mesh, x, y, TOL) <= z + 1e-6, "chord gouges at {}".format((x, y, z))
    if name == "dome":   # h is continuous on the dome: every point is within TOL of the level in z too
        assert max(zgap) <= TOL + 1e-9


def signed_area(r):
    x, y = r[:, 0], r[:, 1]
    return 0.5 * float(np.sum(x[:-1] * y[1:] - x[1:] * y[:-1]))


def test_waterline_direction(parts, tools):
    """Climb with a clockwise spindle keeps the material on the right: clockwise around an island."""
    tris, mesh, stock = parts["dome"]
    climb = runs_of(cam.waterline(tris, mesh, tools[0], stock, stepdown=3.0, direction="climb"))
    conv = runs_of(cam.waterline(tris, mesh, tools[0], stock, stepdown=3.0, direction="conventional"))
    assert len(climb) == len(conv) > 0
    assert all(signed_area(r) < 0 for r in climb)
    assert all(signed_area(r) > 0 for r in conv)


def nearest_dist(samples, runs, cellsize):
    """Distance from each sample (x, y) to the nearest point of the polylines (densified to 0.1 mm)."""
    pts = []
    for r in runs:
        for a, b in zip(r[:-1], r[1:]):
            n = max(1, int(math.ceil(np.linalg.norm(b[:2] - a[:2]) / 0.1)))
            t = np.linspace(0, 1, n, endpoint=False)[:, None]
            pts.append(a[:2] + t * (b[:2] - a[:2]))
        pts.append(r[-1:, :2])
    P = np.vstack(pts)
    key = np.floor(P / cellsize).astype(int)
    buckets = {}
    for k, p in zip(map(tuple, key), P):
        buckets.setdefault(k, []).append(p)
    buckets = {k: np.array(v) for k, v in buckets.items()}
    out = np.full(len(samples), np.inf)
    for n, s in enumerate(samples):
        i, j = int(math.floor(s[0] / cellsize)), int(math.floor(s[1] / cellsize))
        for di in (-1, 0, 1):
            for dj in (-1, 0, 1):
                b = buckets.get((i + di, j + dj))
                if b is not None:
                    out[n] = min(out[n], float(np.min(np.linalg.norm(b - s, axis=1))))
    return out


@pytest.mark.parametrize("name", ["dome", "box"])
def test_steep_shallow_covers_part(parts, tools, name):
    """Every CL sample where the ball touches the part at or above its floor is within the stepover of a path of
    its family: waterline on steep ground, raster on shallow ground (samples within 5 deg of the threshold may be
    covered by either, the overlap band)."""
    tris, mesh, stock = parts[name]
    ball = tools[0]
    c = lc.cutter_of(ball)
    so, th = 1.0, 30.0
    wl, ra = cam.steep_shallow_paths(tris, mesh, ball, stock, threshold_deg=th, overlap=1.0, stepover=so)
    assert wl and ra
    lo, hi = tris.reshape(-1, 3).min(0), tris.reshape(-1, 3).max(0)
    R, d = c.radius, 0.05
    samples, slope = [], []
    for x in np.arange(lo[0] - R, hi[0] + R, 0.5):
        for y in np.arange(lo[1] - R, hi[1] + R, 0.5):
            h = [lc.drop_cutter(c, mesh, x + dx, y + dy) for dx, dy in ((0, 0), (d, 0), (-d, 0), (0, d), (0, -d))]
            if not all(map(math.isfinite, h)) or min(h) < lo[2] - 1e-9:
                continue                                    # off the part, or rolling off its outer edge
            g = math.hypot((h[1] - h[2]) / (2 * d), (h[3] - h[4]) / (2 * d))
            samples.append((x, y))
            slope.append(math.degrees(math.atan(g)))
    samples, slope = np.array(samples), np.array(slope)
    dw, dr = nearest_dist(samples, wl, so), nearest_dist(samples, ra, so)
    steep, shallow = slope >= th + 5, slope <= th - 5
    assert steep.any() and shallow.any()
    assert np.all(dw[steep] <= so), "steep sample far from the waterline: {:.3f}".format(dw[steep].max())
    assert np.all(dr[shallow] <= so), "shallow sample far from the raster: {:.3f}".format(dr[shallow].max())
    assert np.all(np.minimum(dw, dr) <= so)


def ball_leftover_bound(R, s, alpha, x_span=3.0):
    """Largest vertical leftover of a ball (radius R) whose centres sit on the drop-cutter offset of a 2D concave
    V (floor z = 0 meeting a slope at angle alpha), sampled every s horizontally, over all phases of the samples.
    On the planes far from the crease this is the scallop of spacing s / cos(slope), measured vertically; at the
    crease it adds the ball's unreachable fillet and the O(s) cost of sampling across the kink of the offset."""
    ta = math.tan(alpha)
    xs = np.linspace(-x_span * R, x_span * R, 4001)
    target = np.where(xs < 0, 0.0, xs * ta)
    centre = lambda x: max(R, x * ta + R / math.cos(alpha))   # noqa: E731  (ball touching floor or slope)
    worst = 0.0
    for ph in np.linspace(0, s, 40, endpoint=False):
        env = np.full_like(xs, np.inf)
        for cx in np.arange(-(x_span + 2) * R + ph, (x_span + 2) * R, s):
            dd = xs - cx
            m = np.abs(dd) <= R
            env[m] = np.minimum(env[m], centre(cx) - np.sqrt(R * R - dd[m] ** 2))
        worst = max(worst, float(np.max((env - target)[np.abs(xs) < (x_span - 1) * R])))
    return worst


def test_steep_shallow_zmap_leftover_on_dome(parts, tools):
    """Z-map after waterline + steep / shallow on the dome: the leftover is below the 2D ball bound for the
    dome's steepest facet against its flat plate, plus the path tolerance; the gouge is within the path tolerance
    of the raster's adaptive drop-cutter lines."""
    tris, mesh, stock = parts["dome"]
    ball = tools[0]
    so, th = 1.0, 30.0
    ss = cam.steep_shallow(tris, mesh, ball, stock, threshold_deg=th, overlap=1.0, stepover=so, tolerance=TOL)
    wl = cam.waterline(tris, mesh, ball, stock, stepdown=so * math.tan(math.radians(th)), tolerance=TOL)
    n = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    alpha = math.acos(float(np.min(np.abs(n[:, 2]) / np.linalg.norm(n, axis=1))))
    bound = ball_leftover_bound(ball.diameter / 2, so, alpha) + TOL
    s1, _ = cam.zmap_check(tris, mesh, ball, ss, stock, cell=0.5)
    s2, _ = cam.zmap_check(tris, mesh, ball, wl + ss, stock, cell=0.5)
    for s in (s1, s2):
        assert s.max_dev <= bound, (s.max_dev, bound)
        assert s.min_dev >= -2 * TOL, s.min_dev
    assert s2.max_dev <= s1.max_dev + 1e-9


def test_catalog_and_generators(parts, tools):
    assert catalog.BY_ID["waterline"].status == "done"
    ss = catalog.BY_ID["steep_shallow"]
    assert (ss.category, ss.glyph, ss.status) == ("3-axis", "SS", "done")
    keys = {k for k, *_ in ss.params}
    assert {"threshold_deg", "overlap", "stepover", "stepdown", "allowance", "feed", "rpm", "clearance",
            "tolerance"} <= keys
    assert {"stepdown", "allowance", "tolerance", "clearance", "direction"} <= {k for k, *_ in
                                                                               catalog.BY_ID["waterline"].params}
    tris, mesh, stock = parts["dome"]
    for sid in ("waterline", "steep_shallow"):
        assert sid in cam.GENERATORS
        p = catalog.default_params(catalog.BY_ID[sid])   # the program passes every catalogue parameter
        moves = cam.GENERATORS[sid](tris, mesh, tools[0], stock, **p)
        assert any(k == "feed" for k, *_ in moves)
        assert cam.simplify(moves, p["tolerance"])


def test_allowance_keeps_ball_clear(parts, tools):
    """With stock to leave a, the ball stays a clear of the part: exact for a ball (offset sphere)."""
    tris, mesh, stock = parts["dome"]
    ball = tools[0]
    a = 0.5
    grown = lc.Cutter()
    grown.kind, grown.radius = lc.CutterKind.Ball, ball.diameter / 2 + a
    for r in runs_of(cam.waterline(tris, mesh, ball, stock, stepdown=3.0, allowance=a)):
        for p in r:
            assert p[2] >= lc.drop_cutter(grown, mesh, p[0], p[1]) + a - 1e-9
