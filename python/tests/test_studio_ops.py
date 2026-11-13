# Copyright 2026 The lithocam authors. SPDX-License-Identifier: Apache-2.0
"""Hole cycles, rest machining, pencil and curvature-adaptive raster (lithocam.studio.ops)."""
import math

import numpy as np
import pytest

try:
    from lithocam.studio.core import lc  # noqa: F401  (the compiled core, built from bindings/)
except ImportError:
    pytest.skip("_lithocam not built (bindings/); set LITHOCAM_CORE", allow_module_level=True)

mf = pytest.importorskip("manifold3d")

from lithocam.studio import cam, ops  # noqa: E402

TOL = 0.01


def _tris(m):
    me = m.to_mesh()
    V, F = np.asarray(me.vert_properties)[:, :3], np.asarray(me.tri_verts)
    return V[F].astype(float)


def plate_with_holes():
    """60 x 40 x 10 plate: a through hole d 6 at (15, 20) and a blind hole d 8, 6 deep, at (45, 20)."""
    p = mf.Manifold.cube([60, 40, 10])
    p = p - mf.Manifold.cylinder(12, 3, 3, 48).translate([15, 20, -1])
    p = p - mf.Manifold.cylinder(7, 4, 4, 48).translate([45, 20, 4])
    return _tris(p)


def pocket():
    """50 x 50 x 20 block with a 30 x 30 pocket 10 deep (floor at z = 10): inside corners at the floor."""
    return _tris(mf.Manifold.cube([50, 50, 20]) - mf.Manifold.cube([30, 30, 11]).translate([10, 10, 10]))


def dome(n=24, size=60.0, h=12.0):
    xs = np.linspace(-size / 2, size / 2, n + 1)
    Z = lambda x, y: h * max(0.0, 1 - (x * x + y * y) / (size / 2) ** 2)   # noqa: E731
    tris = []
    for i in range(n):
        for j in range(n):
            a, b = (xs[i], xs[j], Z(xs[i], xs[j])), (xs[i + 1], xs[j], Z(xs[i + 1], xs[j]))
            c, d = (xs[i + 1], xs[j + 1], Z(xs[i + 1], xs[j + 1])), (xs[i], xs[j + 1], Z(xs[i], xs[j + 1]))
            tris += [(a, b, c), (a, c, d)]
    return np.array(tris, dtype=float)


def surface(fn, half=20.0, n=80, base=30.0):
    """Closed slab of a smooth relief z = fn(x, y) over [-half, half]^2 (0.5 mm grid, chord sag well below 1 um)."""
    xs = np.linspace(-half, half, n + 1)
    X, Y = np.meshgrid(xs, xs)
    Z = fn(X, Y)
    H = Z - Z.min()
    t = cam.heightfield_tris(H, 2 * half, 2 * half, base)
    t[:, :, 0] -= half
    t[:, :, 1] -= half
    return t


RHO = 24.0
HALF = 14.0          # patch half-width; the scallop is measured over |x|, |y| < HALF - 4 (the rim band is not planned)
SHAPES = {
    "dome": lambda x, y: np.sqrt(RHO ** 2 - x ** 2 - y ** 2),
    "bowl": lambda x, y: -np.sqrt(RHO ** 2 - x ** 2 - y ** 2),
    "cylinder": lambda x, y: np.sqrt(RHO ** 2 - y ** 2),      # axis along x: curved across the x passes
    "ramp": lambda x, y: 6.0 * np.tanh(y / 5.0) + 0 * x,      # flat, then 50 degrees, then flat again
}


@pytest.fixture(scope="module")
def lib():
    L = lc.wood_tool_library()
    ball = [t for t in L.tools if t.kind == lc.ToolKind.BallEndMill][0]
    drill = [t for t in L.tools if t.kind == lc.ToolKind.FlatEndMill][0]
    return ball, drill


def moves_of(moves, *kinds):
    return [m for m in moves if m[0] in kinds]


# ------------------------------------------------------------------------------------------------------ holes

def test_detect_holes_finds_circles_diameter_depth_and_through():
    h = ops.detect_holes(plate_with_holes())
    assert len(h) == 2
    a, b = sorted(h, key=lambda d: d["x"])
    assert (a["x"], a["y"], a["diameter"], a["z_top"]) == pytest.approx((15, 20, 6, 10), abs=1e-6)
    assert a["through"] and a["depth"] == pytest.approx(10, abs=1e-6)
    assert (b["x"], b["y"], b["diameter"]) == pytest.approx((45, 20, 8), abs=1e-6)
    assert not b["through"] and b["depth"] == pytest.approx(6, abs=1e-6)


def test_detect_holes_ignores_the_outer_boundary_a_boss_and_diameter_filter():
    assert ops.detect_holes(_tris(mf.Manifold.cube([40, 40, 10]))) == []
    boss = mf.Manifold.cube([40, 40, 5]) + mf.Manifold.cylinder(5, 4, 4, 48).translate([20, 20, 5])
    assert ops.detect_holes(_tris(boss)) == []
    assert len(ops.detect_holes(plate_with_holes(), min_diameter=7)) == 1
    assert len(ops.detect_holes(plate_with_holes(), max_diameter=7)) == 1


def run_cycle(gen, lib, **kw):
    t = plate_with_holes()
    ball, drill = lib
    stock = cam.stock_box(t)
    return gen(t, cam.trimesh(t), drill, stock, **kw), stock


def test_drill_reaches_depth_exactly_and_pecks_never_exceed_the_increment(lib):
    mv, stock = run_cycle(ops.drill, lib, points=[(15, 20, 10.0)], depth=10.0, peck=3.0)
    bottoms = [m[1][2] for m in mv if m[0] == "plunge"]
    assert bottoms[-1] == 10.0 - 10.0 and min(bottoms) == 0.0           # exactly z_top - depth
    d = np.diff([10.0] + [10.0 - b for b in bottoms][:0] + [10.0 - b for b in bottoms])
    adv = np.diff(np.r_[0.0, 10.0 - np.array(bottoms)])
    assert (adv > 0).all() and adv.max() <= 3.0 + 1e-12 and len(bottoms) == 4
    assert d.size


def test_drill_full_retract_clears_the_stock_top_between_pecks(lib):
    mv, stock = run_cycle(ops.drill, lib, points=[(15, 20, 10.0)], depth=10.0, peck=3.0, retract_height=2.0)
    ups = [m[1][2] for a, m, b in zip(mv, mv[1:], mv[2:]) if m[0] == "rapid" and b[0] == "rapid"
           and m[1][2] > a[1][2] and a[0] == "plunge"]
    assert len(ups) == 4 and all(      # three between the four pecks, one after the last
        z >= stock["zmax"] + 2.0 - 1e-9 for z in ups)


def test_drill_chip_break_retracts_only_the_chip_distance_and_needs_it(lib):
    with pytest.raises(ValueError):
        run_cycle(ops.drill, lib, points=[(15, 20, 10.0)], depth=10.0, peck=3.0, peck_mode="chip")
    mv, _ = run_cycle(ops.drill, lib, points=[(15, 20, 10.0)], depth=10.0, peck=3.0, peck_mode="chip", chip_retract=0.5)
    plunges = [i for i, m in enumerate(mv) if m[0] == "plunge"]
    for i in plunges[:-1]:
        assert mv[i + 1][0] == "rapid" and mv[i + 1][1][2] - mv[i][1][2] == pytest.approx(0.5)


def test_drill_dwell_and_detected_holes_use_each_holes_own_depth(lib):
    mv, _ = run_cycle(ops.drill, lib, depth=0.0, peck=0.0, dwell=1.5)       # detect: depth from the part
    dw = moves_of(mv, "dwell")
    assert sorted(m[1][2] for m in dw) == [pytest.approx(0.0), pytest.approx(4.0)]
    assert all(m[3] == 1.5 and len(m) == 4 for m in dw)
    xy = {(round(m[1][0], 3), round(m[1][1], 3)) for m in moves_of(mv, "plunge")}
    assert xy == {(15.0, 20.0), (45.0, 20.0)}


def test_core_drill_clears_each_peck_and_enters_on_an_orbit(lib):
    with pytest.raises(ValueError):
        run_cycle(ops.core_drill, lib, points=[(15, 20, 10.0)], depth=8.0, peck=0.0)
    mv, stock = run_cycle(ops.core_drill, lib, points=[(15, 20, 10.0)], depth=8.0, peck=2.5, orbit_radius=1.0,
                          orbit_turns=2.0)
    z = np.array([m[1][2] for m in mv])
    assert z.min() == pytest.approx(10.0 - 8.0, abs=1e-12)
    helix = [m[1] for m in mv if m[0] == "feed"]
    r = [math.hypot(p[0] - 15, p[1] - 20) for p in helix[:-1]]
    assert max(r) == pytest.approx(1.0, abs=1e-9)
    assert min(p[2] for p in helix) == pytest.approx(10.0 - 8.0 / math.ceil(8.0 / 2.5), abs=1e-9)
    clears = [m[1][2] for a, m in zip(mv, mv[1:]) if m[0] == "rapid" and a[0] in ("plunge", "feed")
              and m[1][2] > a[1][2]]
    assert len(clears) >= 3 and all(c >= stock["zmax"] + 2.0 - 1e-9 for c in clears)


def test_tap_feed_equals_pitch_times_rpm_reverses_out_and_returns_to_the_r_plane(lib):
    mv, stock = run_cycle(ops.tap, lib, points=[(15, 20, 10.0)], depth=8.0, pitch=1.25, rpm=400.0, dwell=0.2)
    tin, tout = moves_of(mv, "tap_in")[0], moves_of(mv, "tap_out")[0]
    assert tin[3] == pytest.approx(1.25 * 400.0) == tout[3] == ops.tap_feed(1.25, 400.0)
    assert tin[4] == tout[4] == 400.0
    assert tin[1][2] == pytest.approx(2.0) and tout[1][2] >= stock["zmax"]
    kinds = [m[0] for m in mv]
    assert kinds.index("tap_in") < kinds.index("dwell") < kinds.index("tap_out")
    with pytest.raises(ValueError):
        run_cycle(ops.tap, lib, points=[(15, 20, 10.0)], depth=8.0, pitch=1.25)


def test_bore_helix_is_dense_on_the_circle_descends_and_ends_on_the_floor(lib):
    t = plate_with_holes()
    tool = [x for x in lc.wood_tool_library().tools if x.kind == lc.ToolKind.FlatEndMill and x.diameter < 7][0]
    mv = ops.bore(t, cam.trimesh(t), tool, cam.stock_box(t), diameter=8.0, depth=6.0, pitch=2.0,
                  points=[(45, 20, 10.0)], tolerance=TOL)
    feeds = np.array([m[1] for m in mv if m[0] in ("plunge", "feed")])
    r = (8.0 - tool.diameter) / 2
    ring = feeds[:-1]
    rad = np.hypot(ring[:, 0] - 45, ring[:, 1] - 20)
    assert np.allclose(rad, r, atol=1e-9)
    assert ring[:, 2].min() == pytest.approx(4.0) and np.all(np.diff(ring[:, 2]) <= 1e-12)
    seg = np.linalg.norm(np.diff(ring[:, :2], axis=0), axis=1).max()
    assert r - math.sqrt(max(r * r - (seg / 2) ** 2, 0.0)) <= TOL + 1e-9        # chord sag
    with pytest.raises(ValueError):
        ops.bore(t, cam.trimesh(t), tool, cam.stock_box(t), diameter=6.0, depth=6.0, points=[(45, 20, 10.0)])


# ----------------------------------------------------------------------------------------- pencil and rest

def ring_dist(P, lo, hi):
    """Distance in xy from points to the square ring with corners lo..hi."""
    P = np.asarray(P)[:, :2]
    d = np.minimum.reduce([np.abs(P[:, 0] - lo), np.abs(P[:, 0] - hi), np.abs(P[:, 1] - lo), np.abs(P[:, 1] - hi)])
    inside = (P >= lo - 1e-9).all(1) & (P <= hi + 1e-9).all(1)
    return np.where(inside, d, np.inf)


def test_pencil_follows_the_inside_corner_of_a_pocket(lib):
    ball, _ = lib
    t = pocket()
    R = ball.diameter / 2
    runs = ops.pencil_paths(t, cam.trimesh(t), ball)
    P = np.vstack(runs)
    lo, hi = 10 + R, 40 - R                      # ball centre one radius from each wall, tip on the floor
    floor = P[np.abs(P[:, 2] - 10.0) < 1e-6]
    assert len(floor) > 100
    assert ring_dist(floor, lo, hi).max() < 0.05
    # coverage: every sample on the floor ring has a pencil point nearby
    s = np.linspace(lo + 1, hi - 1, 60)
    ring = np.vstack([np.c_[s, np.full_like(s, lo)], np.c_[s, np.full_like(s, hi)],
                      np.c_[np.full_like(s, lo), s], np.c_[np.full_like(s, hi), s]])
    gap = np.array([np.min(np.hypot(*(floor[:, :2] - q).T)) for q in ring])
    assert gap.max() < 0.6
    # the rest of the path climbs the vertical inside corners: the ball touches two walls (or two rim edges)
    # there, so the tip is above the floor and the ball centre is one radius from the surface
    for p in P[::max(len(P) // 30, 1)]:
        d = min_distance(t, np.array([p[0], p[1], p[2] + R]))
        assert abs(d - R) < 0.05


DOME_RHO, DOME_A = 20.0, 15.0     # sphere radius and the radius of its foot on the plate


def dome_plate():
    """A spherical cap (radius 20, foot radius 15, foot angle 48.6 degrees) on a flat plate at z = 0; the crease is the
    foot circle. 0.33 mm grid."""
    c = math.sqrt(DOME_RHO ** 2 - DOME_A ** 2)
    fn = lambda x, y: np.maximum(np.sqrt(np.maximum(DOME_RHO ** 2 - x ** 2 - y ** 2, 0.0)) - c, 0.0)   # noqa: E731
    return surface(fn, half=22.0, n=132, base=10.0)


def test_pencil_follows_the_foot_of_a_dome_on_a_plate_within_a_tolerance(lib):
    ball, _ = lib
    R = ball.diameter / 2
    t = dome_plate()
    mesh = cam.trimesh(t)
    runs = ops.pencil_paths(t, mesh, ball)
    P = np.vstack(runs)
    assert len(P) > 100
    # analytic crease: ball centre at height R and at distance rho + R from the sphere's centre (0, 0, -c)
    c = math.sqrt(DOME_RHO ** 2 - DOME_A ** 2)
    r_exact = math.sqrt((DOME_RHO + R) ** 2 - (R + c) ** 2)
    base = 10.0   # surface() puts the plate top at base + (z - zmin): z = 0 maps to 10
    r = np.hypot(P[:, 0], P[:, 1])
    assert np.abs(r - r_exact).max() < 0.15
    assert np.abs(P[:, 2] - base).max() < 0.02
    # coverage: the whole circle, not an arc
    ang = np.sort(np.arctan2(P[:, 1], P[:, 0]))
    assert np.diff(np.r_[ang, ang[0] + 2 * math.pi]).max() < math.radians(5)
    cut = lc.cutter_of(ball)
    assert all(lc.drop_cutter(cut, mesh, p[0], p[1]) <= p[2] + 1e-9 for p in P[::7])


def min_distance(tris, p):
    """Distance from point p to a triangle soup (Ericson closest point, vectorised)."""
    a, b, c = tris[:, 0], tris[:, 1], tris[:, 2]
    ab, ac, ap = b - a, c - a, p - a
    d1, d2 = (ab * ap).sum(1), (ac * ap).sum(1)
    bp = p - b
    d3, d4 = (ab * bp).sum(1), (ac * bp).sum(1)
    cp = p - c
    d5, d6 = (ab * cp).sum(1), (ac * cp).sum(1)
    n = np.cross(ab, ac)
    nn = (n * n).sum(1)
    best = np.inf
    # plane projection with barycentric clamp via sampling of the three edges and the interior projection
    t = (ap * n).sum(1) / np.where(nn > 0, nn, 1)
    q = p - t[:, None] * n
    v0, v1, v2 = ab, ac, q - a
    dd00, dd01, dd11 = (v0 * v0).sum(1), (v0 * v1).sum(1), (v1 * v1).sum(1)
    dd20, dd21 = (v2 * v0).sum(1), (v2 * v1).sum(1)
    den = dd00 * dd11 - dd01 ** 2
    u = (dd11 * dd20 - dd01 * dd21) / np.where(den != 0, den, 1)
    w = (dd00 * dd21 - dd01 * dd20) / np.where(den != 0, den, 1)
    inside = (u >= 0) & (w >= 0) & (u + w <= 1)
    if inside.any():
        best = np.abs(t[inside] * np.sqrt(nn[inside])).min()
    for s0, s1 in ((a, b), (b, c), (c, a)):
        e = s1 - s0
        tt = np.clip(((p - s0) * e).sum(1) / np.where((e * e).sum(1) > 0, (e * e).sum(1), 1), 0, 1)
        best = min(best, np.linalg.norm(p - (s0 + tt[:, None] * e), axis=1).min())
    return float(best)


def test_pencil_refuses_a_flat_tool_and_finds_nothing_on_a_smooth_dome(lib):
    ball, flat = lib
    t = surface(SHAPES["dome"], half=HALF, n=100)
    with pytest.raises(ValueError):
        ops.pencil_paths(t, cam.trimesh(t), flat)
    assert ops.pencil_paths(t, cam.trimesh(t), ball) == []


def test_rest_region_is_empty_for_the_same_tool_and_covers_the_crease_for_a_larger_one(lib):
    ball, _ = lib
    t = pocket()
    mesh = cam.trimesh(t)
    R = ball.diameter / 2
    _, same = ops.rest_region(t, mesh, ball, previous_tool_radius=R)
    assert not same.any()
    _, bigger = ops.rest_region(t, mesh, ball, previous_tool_radius=2 * R, threshold=0.05)
    assert bigger.any()
    # the small tool's pencil points (the crease) lie in the region
    fld, mask = ops.rest_region(t, mesh, ball, previous_tool_radius=2 * R, threshold=0.05)
    P = np.vstack(ops.pencil_paths(t, mesh, ball))
    assert fld.lookup(mask, P[:, 0], P[:, 1]).all()
    # and the region is a band at the walls, not the floor's middle
    assert not fld.lookup(mask, np.array([25.0]), np.array([25.0]))[0]


def test_rest_paths_stay_in_the_region_and_use_the_small_tool_depth(lib):
    ball, _ = lib
    t = pocket()
    mesh = cam.trimesh(t)
    R = ball.diameter / 2
    stock = cam.stock_box(t)
    assert ops.rest(t, mesh, ball, stock, previous_tool_radius=R) == []
    for mode in ("raster", "waterline"):
        mv = ops.rest(t, mesh, ball, stock, previous_tool_radius=2 * R, threshold=0.05, mode=mode, overlap=0.0)
        P = np.array([m[1] for m in mv if m[0] in ("plunge", "feed")])
        assert len(P) > 10
        fld, mask = ops.rest_region(t, mesh, ball, previous_tool_radius=2 * R, threshold=0.05)
        grown = cam._dilate(mask, 2)
        assert fld.lookup(grown, P[:, 0], P[:, 1]).all()
        c = lc.cutter_of(ball)
        assert all(p[2] >= lc.drop_cutter(c, mesh, p[0], p[1]) - 1e-6 for p in P)


def test_rest_from_a_leftover_height_field_and_an_sdf_stock(lib):
    from lithocam.sdfstock import SdfStock
    ball, _ = lib
    t = pocket()
    mesh = cam.trimesh(t)
    fld0 = cam._Field(t, mesh, ball)
    xs, ys = fld0.xs, fld0.ys
    # a leftover that is the part's own CL height: nothing above h_small + threshold
    left = (xs, ys, np.where(np.isfinite(fld0.H), fld0.H, -np.inf))
    assert not ops.rest_region(t, mesh, ball, leftover=left, threshold=0.05)[1].any()
    # 1 mm of stock left over the whole floor and corner band: region equals where h is finite and on the part
    left2 = (xs, ys, np.where(np.isfinite(fld0.H), fld0.H + 1.0, -np.inf))
    assert ops.rest_region(t, mesh, ball, leftover=left2, threshold=0.05)[1].any()
    sdf = SdfStock.from_box([0, 0, 0], [50, 50, 20], 1.0)
    top = ops._stock_top(sdf, np.array([25.0]), np.array([25.0]))[0]
    assert top == pytest.approx(20.0, abs=1.0)


# ----------------------------------------------------------------------------------- adaptive stepover

H_TARGET = 0.05
R_BALL = 3.175


def test_local_stepover_formula_limits_and_ordering():
    flat = ops.local_stepover(R_BALL, H_TARGET, 0.0)
    assert flat == pytest.approx(2 * math.sqrt(H_TARGET * (2 * R_BALL - H_TARGET)))
    assert ops.local_stepover(R_BALL, H_TARGET, 1e-12) == pytest.approx(flat, rel=1e-6)
    # at equal contact spacing a convex surface scallops more, a concave one less
    assert ops.local_stepover(R_BALL, H_TARGET, 1 / 50) < flat < ops.local_stepover(R_BALL, H_TARGET, -1 / 50)
    # small-s expansion h = s^2 / 8 (1/R + kappa), the effective radius form
    for k in (1 / 100, -1 / 100):
        s = ops.local_stepover(R_BALL, H_TARGET, k)
        assert s == pytest.approx(math.sqrt(8 * H_TARGET / (1 / R_BALL + k)), rel=0.02)
    # exact two-circle check: contacts at +-phi on a circle of radius |rho|, centres at |rho| +- R
    for k in (1 / 30, -1 / 30):
        rho = abs(1 / k)
        s = ops.local_stepover(R_BALL, H_TARGET, k)
        phi = s / (2 * rho)
        rc = rho + R_BALL if k > 0 else rho - R_BALL
        cx, cy = rc * math.sin(phi), rc * math.cos(phi)
        crest = cy - math.sqrt(R_BALL ** 2 - cx ** 2) if k > 0 else cy + math.sqrt(R_BALL ** 2 - cx ** 2)
        assert abs(crest - rho) == pytest.approx(H_TARGET, rel=1e-6)


_CACHE = {}


def run_adaptive(name, lib, label):
    """(normal scallop, passes) of one raster: label adaptive | constant (global minimum) | flat (flat formula)."""
    key = (name, label)
    if key in _CACHE:
        return _CACHE[key]
    ball, _ = lib
    t = surface(SHAPES[name], half=HALF, n=int(2 * HALF / 0.15))
    mesh, stock = cam.trimesh(t), cam.stock_box(t)
    kw = {"adaptive": {}, "constant": {"adaptive": False},
          "flat": {"stepover": 2 * math.sqrt(H_TARGET * (2 * R_BALL - H_TARGET))}}[label]
    mv = ops.adaptive_raster(t, mesh, ball, stock, scallop=H_TARGET, **kw)
    _, zm = cam.zmap_check(t, mesh, ball, mv, stock, cell=0.1)
    tg = lc.target_heights(mesh, zm)
    Z = np.array(zm.z).reshape(zm.ny, zm.nx)
    G = np.array(tg.z).reshape(zm.ny, zm.nx)
    gy, gx = np.gradient(G, zm.cell)
    xs = zm.x0 + zm.cell * (np.arange(zm.nx) + 0.5)
    ys = zm.y0 + zm.cell * (np.arange(zm.ny) + 0.5)
    X, Y = np.meshgrid(xs, ys)
    inner = (np.abs(X) < HALF - 4) & (np.abs(Y) < HALF - 4)
    normal = ((Z - G) / np.sqrt(1 + gx ** 2 + gy ** 2))[inner]   # leftover along the surface normal
    _CACHE[key] = (float(normal.max()), sum(1 for m in mv if m[0] == "plunge"))
    return _CACHE[key]


@pytest.mark.parametrize("name", ["dome", "bowl", "cylinder", "ramp"])
def test_adaptive_scallop_is_at_most_h_within_the_sampling_tolerance(lib, name):
    s, _ = run_adaptive(name, lib, "adaptive")
    assert s <= 1.1 * H_TARGET, (name, s)
    assert s >= 0.6 * H_TARGET, (name, s)       # and not wastefully tight


def test_adaptive_uses_fewer_passes_than_the_constant_stepover_of_equal_worst_scallop(lib):
    s_a, n_a = run_adaptive("ramp", lib, "adaptive")
    s_c, n_c = run_adaptive("ramp", lib, "constant")
    assert s_c <= 1.1 * H_TARGET
    assert n_a <= 0.85 * n_c, (n_a, n_c)
    assert abs(s_a - s_c) < 0.2 * H_TARGET


def test_flat_formula_stepover_violates_h_on_a_concave_bowl_and_on_a_ramp(lib):
    for name in ("bowl", "ramp"):
        s_f, _ = run_adaptive(name, lib, "flat")
        s_a, _ = run_adaptive(name, lib, "adaptive")
        assert s_f > 1.3 * H_TARGET, (name, s_f)
        assert s_a <= 1.1 * H_TARGET


def test_adaptive_passes_are_closer_on_steeper_ground(lib):
    ball, _ = lib
    t = surface(SHAPES["ramp"], half=HALF, n=int(2 * HALF / 0.15))
    _, info = ops.adaptive_raster_paths(t, cam.trimesh(t), ball, H_TARGET)
    ys = info["ys"]
    d = np.diff(ys)
    mid = d[(ys[:-1] > -2) & (ys[1:] < 2)].mean()
    far = d[(ys[:-1] > 8) & (ys[1:] < 12)].mean()
    assert mid < 0.8 * far
    assert info["n_passes"] == len(ys) and info["s_min"] < info["s_max"]


def test_adaptive_raster_requires_a_scallop_and_a_ball(lib):
    ball, flat = lib
    t = surface(SHAPES["dome"], half=HALF, n=40)
    mesh, stock = cam.trimesh(t), cam.stock_box(t)
    with pytest.raises(ValueError):
        ops.adaptive_raster(t, mesh, ball, stock)
    with pytest.raises(ValueError):
        ops.adaptive_raster(t, mesh, flat, stock, scallop=H_TARGET)