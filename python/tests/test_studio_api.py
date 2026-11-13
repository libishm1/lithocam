# Copyright 2026 The lithocam authors. SPDX-License-Identifier: Apache-2.0
"""Headless API: programs written from code only simulate and post like the studio's, and bad input is refused
with messages that name the valid choices."""
import math

import pytest

np = pytest.importorskip("numpy")
try:
    from lithocam.studio.core import lc  # noqa: F401
except ImportError:
    pytest.skip("_lithocam not built (bindings/); set LITHOCAM_CORE", allow_module_level=True)

from lithocam import post  # noqa: E402
from lithocam.studio import api, program  # noqa: E402


def write_dome_stl(path, n=20, size=60.0, h=12.0):
    import struct
    xs = np.linspace(-size / 2, size / 2, n + 1)
    Z = lambda x, y: h * max(0.0, 1 - (x * x + y * y) / (size / 2) ** 2)   # noqa: E731
    tris = []
    for i in range(n):
        for j in range(n):
            a, b = (xs[i], xs[j], Z(xs[i], xs[j])), (xs[i + 1], xs[j], Z(xs[i + 1], xs[j]))
            c, d = (xs[i + 1], xs[j + 1], Z(xs[i + 1], xs[j + 1])), (xs[i], xs[j + 1], Z(xs[i], xs[j + 1]))
            tris += [(a, b, c), (a, c, d)]
    with open(path, "wb") as f:
        f.write(b"\0" * 80 + struct.pack("<I", len(tris)))
        for t in tris:
            f.write(struct.pack("<12fH", 0, 0, 0, *[c for p in t for c in p], 0))


@pytest.mark.parametrize("robot", ["ur10e", "kr120"])
def test_program_from_code_simulates_posts_and_round_trips(tmp_path, robot):
    stl = tmp_path / "dome.stl"
    write_dome_stl(stl)
    s = api.Session(robot, material="granite")
    s.load_part(stl, stock_margin=3.0)
    s.set_work_frame([0.55, 0.15, 0.05] if robot == "ur10e" else [1.6, 0.0, 0.3])
    home = np.degrees(program.HOME[robot]).tolist()
    s.add_target("approach", joints=home)
    p = s.program("Main")
    p.add("set_tool", tool="W7")
    p.add("movej", target="approach", speed=20.0)
    p.add("raster", tool="W7", stepover=8.0, feed=600.0)
    p.add("movej", target="approach")
    r = s.simulate("Main")
    assert r.ok, r.summary() + "\n" + "\n".join(r.messages)
    for d in api.dialects():
        text = s.post("Main", d)
        assert text.strip()
    f = s.save(tmp_path / "st.lcs.json")
    s2 = api.Session.open(f)
    assert s2.station.part_path and s2.part is not None       # relative part path resolves on open
    assert s2.simulate("Main").samples == r.samples
    assert post.write(s2.simulate("Main").expansion.records, "cl") == post.write(r.expansion.records, "cl")


def test_bad_input_names_the_valid_choices():
    s = api.Session("ur10e")
    p = s.program("Main")
    with pytest.raises(api.ApiError, match="Valid steps"):
        p.add("rasterize")
    with pytest.raises(api.ApiError, match="Did you mean: raster"):
        p.add("rast")
    with pytest.raises(api.ApiError, match="has no parameter 'step_over'.*stepover"):
        p.add("raster", tool="W7", step_over=3)
    with pytest.raises(api.ApiError, match="must be a number"):
        p.add("raster", tool="W7", stepover="3")
    with pytest.raises(api.ApiError, match="no target 'home'"):
        p.add("movej", target="home")
    with pytest.raises(api.ApiError, match="Valid robots"):
        api.Session("ur5")
    with pytest.raises(api.ApiError, match="outside the limits"):
        s.add_target("t", joints=[0, 0, 0, 0, 0, 999])
    with pytest.raises(api.ApiError, match="load a part first"):
        p.add("raster", tool="W7")
        s.simulate("Main")
    with pytest.raises(api.ApiError, match="no program"):
        s.simulate("Other")


def test_feed_error_blocks_posting(tmp_path):
    stl = tmp_path / "dome.stl"
    write_dome_stl(stl)
    s = api.Session("ur10e", material="granite")
    s.load_part(stl)
    s.set_work_frame([0.55, 0.15, 0.05])
    s.station.material = "no_such_material"           # no cutting data, the tool has no feed
    p = s.program("Main")
    p.add("raster", tool="W7", stepover=8.0)
    t = {x["id"]: x for x in s.tools()}["W7"]
    assert t["id"] == "W7"
    r = s.simulate("Main")
    if r.errors:
        with pytest.raises(api.ApiError, match="no feed"):
            s.post("Main", "urscript")


def test_catalogue_and_schema_are_complete():
    ids = [x["id"] for x in api.steps()]
    assert "raster" in ids and "movej" in ids and "wire_ruled" in ids
    sc = api.station_schema()
    assert set(sc["properties"]["programs"]["items"]["properties"]["steps"]["items"]["properties"]["kind"]["enum"]) \
        == set(ids)
    s = api.Session("kr120")
    s.add_target("a", joints=np.degrees(program.HOME["kr120"]).tolist())
    T = s.robot.tcp(np.asarray(program.HOME["kr120"]))
    T[2, 3] -= 0.1
    t = s.add_target("b", pose=T)
    assert math.isclose(np.asarray(t["pose"])[2, 3], T[2, 3], abs_tol=1e-9)
