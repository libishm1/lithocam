# Copyright 2026 The lithocam authors. SPDX-License-Identifier: Apache-2.0
"""Height-field parts (lithocam.studio.cam.heightfield_tris / load_part): the slab is closed and oriented, its volume
is the analytic slab volume, the shipped padma relief loads at its stated size, and the temple-relief KR120 station
simulates without errors or unreachable points."""
import json
from pathlib import Path

import numpy as np
import pytest

try:
    from lithocam.studio.core import lc  # noqa: F401  (the compiled core, built from bindings/)
except ImportError:
    pytest.skip("_lithocam not built (bindings/); set LITHOCAM_CORE", allow_module_level=True)

from lithocam.studio import api, cam  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]
NPZ = ROOT / "examples" / "data" / "padma_relief.npz"
KR120 = ROOT / "examples" / "studio" / "temple_relief_kr120.lcs.json"


def bumpy(ny=33, nx=41):
    """A smooth non-negative test relief, mm (rows = y, columns = x)."""
    y, x = np.mgrid[0:ny, 0:nx]
    return 6.0 + 5.0 * np.sin(x / 5.0) * np.cos(y / 7.0) + 0.1 * x


def edges_check(tris):
    """(every undirected edge in exactly two faces, every directed edge exactly once)."""
    V = tris.reshape(-1, 3)
    _, vid = np.unique(np.round(V, 9), axis=0, return_inverse=True)
    F = vid.reshape(-1, 3)
    directed = np.concatenate([F[:, [0, 1]], F[:, [1, 2]], F[:, [2, 0]]])
    _, dcount = np.unique(directed, axis=0, return_counts=True)
    _, ucount = np.unique(np.sort(directed, axis=1), axis=0, return_counts=True)
    return bool(np.all(ucount == 2)), bool(np.all(dcount == 1))


def volume(tris):
    return float(np.einsum("ij,ij->i", tris[:, 0], np.cross(tris[:, 1], tris[:, 2])).sum() / 6.0)


def slab_volume(H, width, depth, base):
    """Base box plus the trapezoid-rule integral of the relief over the rectangle."""
    ny, nx = H.shape
    dx, dy = width / (nx - 1), depth / (ny - 1)
    integral = np.trapezoid(np.trapezoid(H, dx=dx, axis=1), dx=dy) if hasattr(np, "trapezoid") else \
        np.trapz(np.trapz(H, dx=dx, axis=1), dx=dy)
    return width * depth * base + integral


@pytest.mark.parametrize("stride", [1, 3])
def test_heightfield_slab_closed_with_analytic_volume(stride):
    H = bumpy()
    width, depth, base = 120.0, 90.0, 35.0
    tris = cam.heightfield_tris(H, width, depth, base, stride=stride)
    closed, oriented = edges_check(tris)
    assert closed and oriented
    v = volume(tris)
    assert v > 0
    assert abs(v - slab_volume(H, width, depth, base)) / v < 0.005
    lo, hi = tris.reshape(-1, 3).min(0), tris.reshape(-1, 3).max(0)
    assert np.allclose(lo, [0, 0, 0]) and np.allclose(hi[:2], [width, depth])


def test_heightfield_flat_is_exact_box_and_square_pixels():
    H = np.full((5, 9), 2.5)
    tris = cam.heightfield_tris(H, 80.0, base_mm=10.0)            # depth from square pixels: 80 * 4 / 8 = 40
    assert np.isclose(volume(tris), 80.0 * 40.0 * 12.5)
    assert np.allclose(tris.reshape(-1, 3).max(0), [80.0, 40.0, 12.5])
    up = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    assert np.all(np.linalg.norm(up, axis=1) > 0)                 # no degenerate faces


def test_heightfield_rejects_bad_input():
    with pytest.raises(ValueError):
        cam.heightfield_tris(np.zeros(5), 10.0)
    with pytest.raises(ValueError):
        cam.heightfield_tris(np.full((3, 3), np.nan), 10.0)
    with pytest.raises(ValueError):
        cam.heightfield_tris(np.zeros((3, 3)), 10.0, base_mm=0.0)


def test_padma_npz_loads_at_stated_size():
    assert NPZ.stat().st_size < 1024 * 1024
    with np.load(NPZ, allow_pickle=False) as z:
        assert z["heights"].dtype == np.uint16 and max(z["heights"].shape) == 512
        assert "Libish Murugesan" in str(z["credit"]) and "Depth Anything 3" in str(z["credit"])
        assert str(z["license"]) == "Apache-2.0"
    H, meta = cam.load_heightfield(NPZ)
    assert meta["width_mm"] == 600.0 and meta["relief_mm"] == 25.0
    assert abs(H.max() - H.min() - 25.0) < 1e-3                    # normalised relief depth
    tris = cam.load_part(NPZ)
    lo, hi = tris.reshape(-1, 3).min(0), tris.reshape(-1, 3).max(0)
    assert np.allclose(lo, [0, 0, 0]) and np.allclose(hi, [600.0, 600.0, 60.0], atol=1e-3)
    assert edges_check(cam.load_part(NPZ, stride=8)) == (True, True)


def test_session_loads_npz_and_refuses_other_types(tmp_path):
    s = api.Session("kr120")
    s.load_part(NPZ, stock_margin=0.0, stride=4)
    st = s.station.stock
    assert (st["xmax"] - st["xmin"], st["ymax"] - st["ymin"], st["zmax"] - st["zmin"]) == pytest.approx(
        (600.0, 600.0, 60.0), abs=1e-6)
    bad = tmp_path / "part.obj"
    bad.write_text("")
    with pytest.raises(api.ApiError):
        s.load_part(bad)


def test_kr120_station_simulates_clean():
    """The shipped KR120 station: same work frame, stock, tools, steps and tool axes, planned on every 8th row and
    column of the height field with the stepovers widened so the test runs in about a minute (the full plan has some
    300 000 robot points; examples/temple_relief.py --simulate runs it). Every step still sweeps the whole panel."""
    st = json.loads(KR120.read_text(encoding="utf-8"))
    assert st["part_path"] == "../data/padma_relief.npz"
    s = api.Session.open(KR120)
    assert s.part is not None and s.station.stock["zmax"] == pytest.approx(63.0)
    s.load_part(s.station.part_path, keep_stock=True, stride=8)
    p = s.program("Relief")
    cutting = ("face", "rough_plane", "steep_shallow", "surfacing5")
    wide = {"rough_plane": 40.0, "steep_shallow": 20.0, "surfacing5": 20.0}
    kinds = [x["kind"] for x in p.list()]
    assert [k for k in kinds if k in cutting] == list(cutting)
    for i, k in enumerate(kinds):
        if k in cutting:
            prm = p.list()[i]["params"]
            assert prm["feed"] > 0 and prm["rpm"] > 0                 # explicit example feeds on every cut
        if k in wide:
            p.set(i, stepover=wide[k])
    r = s.simulate("Relief")
    assert r.errors == [], r.errors
    assert r.unreachable == 0, [m for m in r.messages if "unreachable" in m]
    assert r.ok
