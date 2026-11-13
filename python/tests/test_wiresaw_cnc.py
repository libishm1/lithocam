# Copyright 2026 The lithocam authors. SPDX-License-Identifier: Apache-2.0
"""CNC wire saw exporter: read-back of every position, kerf modes, arcs, file split, limits, DXF, refusals."""
import dataclasses
import math
import re
from pathlib import Path

import numpy as np
import pytest

from lithocam import wiresaw_cnc as wc
from lithocam.wirepath import load_ruled_json

ROOT = Path(__file__).resolve().parents[2]
DATA = Path(__file__).resolve().parent / "data"
R = 4.0                                                    # wire radius of the test machines


def mach(layout="xy", **kw) -> wc.MachineDef:
    axes = {"xy": {"trolley": "X", "height": "Y"},
            "xyc": {"trolley": "X", "height": "Z", "table": "C"},
            "xyuv": {"trolley": "X", "height": "Y", "trolley2": "U", "height2": "V"}}[layout]
    base = dict(name="t_" + layout, layout=layout, axes=axes, wire_diameter=2 * R, lead_in=60.0, lead_out=60.0,
                arc_plane="G17" if layout == "xy" else None, decimals=4)
    if layout == "xyc":
        base.update(arc_plane="G18", arc_first="height", arc_second="trolley",
                    arc_words={"trolley": "I", "height": "K"}, table_axis_trolley=500.0, rotary_feed_deg_min=600.0)
    if layout == "xyuv":
        base.update(carriage_spacing=1200.0)
    base.update(kw)
    m = wc.MachineDef(**base)
    m.validate()
    return m


def rounded_rect(w=800.0, h=500.0, r=50.0) -> wc.Profile:
    S = wc.Seg
    return wc.Profile((r, 0.0), [
        S((w - r, 0.0)), S((w, r), (w - r, r), True), S((w, h - r)), S((w - r, h), (w - r, h - r), True),
        S((r, h)), S((0.0, h - r), (r, h - r), True), S((0.0, r)), S((r, 0.0), (r, r), True)], closed=True)


def in_poly(pt, poly) -> bool:
    x, y = pt
    inside = False
    for (x1, y1), (x2, y2) in zip(poly[:-1], poly[1:]):
        if (y1 > y) != (y2 > y) and x < x1 + (y - y1) / (y2 - y1) * (x2 - x1):
            inside = not inside
    return inside


def plan_equals_readback(plan: wc.Plan, text: str, mc: wc.MachineDef):
    rb = wc.gcode_positions(text, mc)
    mp = plan.modal_positions()
    assert len(rb) == len(mp), (len(rb), len(mp))
    tol = 0.5 * 10 ** (-mc.decimals) + 1e-9
    for k, (r, p) in enumerate(zip(rb, mp)):
        assert set(r.pos) == set(p), (k, r.pos, p)
        for ax in p:
            assert abs(r.pos[ax] - p[ax]) <= tol, (k, ax, r.pos[ax], p[ax])
    for r, m in zip(rb, plan.moves()):
        if m.kind == "arc":
            assert r.kind == "arc" and r.ccw == m.ccw
            assert abs(r.center[0] - m.center[0]) <= 2 * tol and abs(r.center[1] - m.center[1]) <= 2 * tol
        else:
            assert r.kind == m.kind


# ---------------------------------------------------------------------------------------------------- read-back
def test_readback_xy_with_arcs():
    mc = mach("xy", line_numbers=10)
    job = wc.WireJob("slab", [wc.Setup2D(rounded_rect(), name="outline")], feed=120.0)
    pl = wc.plan_job(job, mc)
    texts = wc.gcode_files(pl)
    assert len(texts) == 1
    assert any(m.kind == "arc" for m in pl.moves())
    assert re.search(r"^N10 ", texts[0].splitlines()[3] + "\n" + texts[0], re.M)
    plan_equals_readback(pl, texts[0], mc)


def test_readback_xyc_indexed_and_coordinated():
    mc = mach("xyc", feed_mode="G93")
    prof = wc.Profile.from_points([(300.0, 0.0), (280.0, 400.0), (270.0, 800.0)])
    setups = [wc.Setup2D(prof, table_deg=a, waste="right", name=f"side{a}") for a in (0, 90, 180, 270)]
    turned = wc.CoordSetup(wc.Profile.from_bulge([(60, 0), (60, 40), (45, 120), (45, 200), (60, 260)],
                                                 [0, 0.3, 0, -0.2]), deg_per_mm=1.0, waste="+s", name="baluster")
    job = wc.WireJob("column", setups + [turned], feed=90.0)
    pl = wc.plan_job(job, mc)
    texts = wc.write_gcode(job, mc)
    plan_equals_readback(pl, texts[0], mc)
    tab = [m.pos["table"] for m in pl.moves() if "table" in m.pos]
    assert {0.0, 90.0, 180.0, 270.0} <= set(round(t, 6) for t in tab)
    assert all("F" in b.words for b in wc.parse_gcode(texts[0]) if b.g and b.g[-1] in (1, 2, 3))   # inverse time


def test_readback_xyuv_four_words():
    mc = mach("xyuv", feed_mode="G93")
    x = np.linspace(100, 900, 9)
    S = np.column_stack([x, 800 + 0.2 * x, x + 20, 900 + 0.1 * x])
    job = wc.WireJob("ruled", [wc.RuledSetup(S, name="f")], feed=60.0)
    pl = wc.plan_job(job, mc)
    text = wc.write_gcode(job, mc)[0]
    plan_equals_readback(pl, text, mc)
    assert re.search(r"G1 X[\d.-]+ Y[\d.-]+ U[\d.-]+ V[\d.-]+ F", text)


# ---------------------------------------------------------------------------------------------------- kerf
def path_of(plan: wc.Plan, which="wire_centre"):
    rec = [p for p in plan.paths if p.kind == which][0]
    return rec.profile


def test_cam_offset_distance_and_side():
    mc = mach("xy")
    L = wc.Profile.from_points([(0, 0), (600, 0), (600, 300), (300, 300), (300, 500), (0, 500)], closed=True)
    for prof in (rounded_rect(), L):
        pl = wc.plan_job(wc.WireJob("k", [wc.Setup2D(prof)], feed=100.0), mc)
        full = path_of(pl)
        nom = prof.flatten(0.002)
        pts = full.flatten(0.002, step=2.0)
        d = wc._seg_dist(pts, nom)
        inner = pts[1:-1]
        # the middle of the path (without the lead lines) is at the wire radius from the profile
        n_lead = int(60.0 / 2.0) + 2
        mid = d[n_lead:-n_lead]
        assert np.all(np.abs(mid - R) < 0.02), (mid.min(), mid.max())
        assert d.min() >= R - 0.02
        for p in pts[::7]:
            assert not in_poly(p, nom)                      # waste side = outside the part
    # inside waste (a pocket): path inside the profile
    pl = wc.plan_job(wc.WireJob("k", [wc.Setup2D(L, waste="inside", lead_in=0, lead_out=0)], feed=100.0), mc)
    pts = path_of(pl).flatten(0.002, step=5.0)
    nom = L.flatten(0.002)
    assert all(in_poly(p, nom) for p in pts[::5])


def test_g41_before_first_cut_and_g40_after():
    mc = mach("xy", kerf_mode="g41", d_register=7)
    open_p = wc.Profile.from_points([(0, 0), (300, 0), (300, 200), (500, 200)])
    for waste, code in (("left", "G41"), ("right", "G42")):
        job = wc.WireJob("g", [wc.Setup2D(open_p, waste=waste)], feed=80.0)
        text = wc.write_gcode(job, mc)[0]
        lines = [l for l in text.splitlines() if not l.startswith("(")]
        g1 = [i for i, l in enumerate(lines) if l.startswith("G1") or " G1 " in l]
        a = [i for i, l in enumerate(lines) if f"{code} D7" in l]
        z = [i for i, l in enumerate(lines) if "G40" in l]
        assert len(a) == 1 and len(z) == 1
        assert a[0] == g1[0] and z[0] == g1[-1]               # on the lead-in and the lead-out block
        assert lines[a[0]].startswith("G1 " + code)
        # the programmed path is the nominal profile
        rb = wc.gcode_positions(text, mc)
        assert any(abs(r.pos["trolley"] - 300.0) < 1e-6 and abs(r.pos["height"] - 200.0) < 1e-6 for r in rb)
    short = dataclasses.replace(mc, lead_in=2.0)
    with pytest.raises(wc.WireSawError, match="lead-in"):
        wc.write_gcode(wc.WireJob("g", [wc.Setup2D(open_p, waste="left")], feed=80.0), short)


def test_g41_unsupported_for_turning_and_ruled():
    mc = mach("xyc", kerf_mode="g41")
    turned = wc.CoordSetup(wc.Profile.from_points([(50, 0), (50, 100)]), deg_per_mm=1.0)
    with pytest.raises(wc.WireSawError, match="cam"):
        wc.plan_job(wc.WireJob("t", [turned], feed=50.0), mc)


def test_self_intersecting_offset_refused():
    slot = wc.Profile.from_points([(0, 0), (100, 0), (100, 50), (52.5, 50), (52.5, 20), (47.5, 20), (47.5, 50), (0, 50)],
                                  closed=True)
    with pytest.raises(wc.KerfError):
        wc.plan_job(wc.WireJob("s", [wc.Setup2D(slot)], feed=50.0), mach("xy"))
    # the same slot is fine with a wire thinner than the slot
    wc.plan_job(wc.WireJob("s", [wc.Setup2D(slot)], feed=50.0), mach("xy", wire_diameter=3.0, lead_in=0, lead_out=0))


def test_lead_in_must_not_cut_the_part():
    # start at a reflex vertex of an L: the backward tangent of the first segment runs into the part
    L = wc.Profile.from_points([(300, 300), (300, 500), (0, 500), (0, 0), (600, 0), (600, 300)], closed=True)
    with pytest.raises(wc.KerfError, match="lead"):
        wc.plan_job(wc.WireJob("l", [wc.Setup2D(L)], feed=50.0), mach("xy"))


def test_turned_profile_envelope_and_table():
    mc = mach("xyc", table_axis_trolley=500.0)
    cyl = wc.Profile.from_points([(50, 0), (50, 300)])
    pl = wc.plan_job(wc.WireJob("t", [wc.CoordSetup(cyl, deg_per_mm=1.0, max_step_deg=5.0)], feed=50.0), mc)
    cuts = pl.cut_moves()
    assert all(abs(m.pos["trolley"] - (500.0 + 50.0 + R)) < 1e-9 for m in cuts)   # wire centre at s + wire radius
    steps = np.diff([0.0] + [m.pos["table"] for m in cuts])
    assert steps.max() <= 5.0 + 1e-9
    assert abs(cuts[-1].pos["table"] - cuts[0].start["table"] - 1.0 * (300 + 120)) < 1e-6
    # crossing 360 degrees inside a cut is refused on a wrapping table
    with pytest.raises(wc.WireSawError, match="360"):
        wc.plan_job(wc.WireJob("t", [wc.CoordSetup(cyl, deg_per_mm=1.0)], feed=50.0), mach("xyc", table_unwind="wrap360"))


# ---------------------------------------------------------------------------------------------------- arcs, split
def test_arcs_vs_flattened_agree():
    tol = 0.01
    ma = mach("xy", decimals=6)
    mf = mach("xy", decimals=6, arc_plane=None, chord_tol=tol)
    job = wc.WireJob("a", [wc.Setup2D(rounded_rect())], feed=100.0)
    pa, pf = wc.plan_job(job, ma), wc.plan_job(job, mf)
    assert any(m.kind == "arc" for m in pa.moves()) and all(m.kind != "arc" for m in pf.moves())
    exact = path_of(pa).flatten(0.0005, step=1.0)
    pts = np.array([[m.pos["trolley"], m.pos["height"]] for m in pf.cut_moves()])
    assert wc._seg_dist(pts, exact).max() < 1e-3                       # flattened vertices lie on the arcs
    mid = (pts[:-1] + pts[1:]) / 2
    assert wc._seg_dist(mid, exact).max() <= tol + 1e-6                # chords deviate by at most the tolerance
    la = sum(m.length for m in pa.cut_moves())
    lf = sum(m.length for m in pf.cut_moves())
    assert abs(la - lf) / la < 1e-3


def test_file_split_keeps_continuity():
    mc = mach("xy", arc_plane=None, chord_tol=0.001, max_lines=60, line_numbers=5)
    job = wc.WireJob("split", [wc.Setup2D(rounded_rect(), name="a"), wc.Setup2D(rounded_rect(300, 200, 40), name="b")],
                     feed=100.0)
    pl = wc.plan_job(job, mc)
    texts = wc.gcode_files(pl)
    assert len(texts) >= 3
    assert all(len(t.splitlines()) <= 60 for t in texts)
    per = [wc.gcode_positions(t, mc) for t in texts]
    for k in range(1, len(per)):
        assert per[k][0].pos == per[k - 1][-1].pos and per[k][0].kind == "rapid"      # starts where the last ended
    joined = per[0] + [r for p in per[1:] for r in p[1:]]
    mp = pl.modal_positions()
    assert len(joined) == len(mp)
    for r, p in zip(joined, mp):
        assert all(abs(r.pos[a] - p[a]) <= 0.5e-4 + 1e-9 for a in p)
    for t in texts:                                                                  # each file is a full program
        wc.check_program(t, mc)
        assert t.rstrip().splitlines()[-1].endswith("M30")
    # inside a G41 span the program cannot be cut
    g41 = mach("xy", kerf_mode="g41", arc_plane=None, chord_tol=0.001, max_lines=40)
    with pytest.raises(wc.WireSawError, match="G41"):
        wc.write_gcode(wc.WireJob("g", [wc.Setup2D(rounded_rect())], feed=50.0), g41)


# ---------------------------------------------------------------------------------------------------- checks
def test_travel_limit_violation_raises():
    mc = mach("xy", limits={"trolley": (0.0, 500.0), "height": (0.0, 1000.0)})
    with pytest.raises(wc.TravelLimitError, match="trolley"):
        wc.write_gcode(wc.WireJob("t", [wc.Setup2D(rounded_rect())], feed=50.0), mc)
    ok = mach("xy", limits={"trolley": (-200.0, 1200.0), "height": (-200.0, 1000.0)})
    wc.write_gcode(wc.WireJob("t", [wc.Setup2D(rounded_rect())], feed=50.0), ok)
    mu = mach("xyuv", limits={"height2": (0.0, 850.0)})
    S = np.array([[100, 800, 100, 800], [200, 800, 200, 900.0]])
    with pytest.raises(wc.TravelLimitError, match="height2"):
        wc.write_gcode(wc.WireJob("t", [wc.RuledSetup(S)], feed=50.0), mu)


def test_table_unwinding_rules():
    prof = wc.Profile.from_points([(300.0, 0.0), (300.0, 100.0)])
    mk = lambda a: wc.WireJob("u", [wc.Setup2D(prof, table_deg=x, waste="right") for x in a], feed=50.0)
    tabs = lambda pl: [m.pos["table"] for m in pl.moves() if "table" in m.pos]
    # wrapping table: values in [0, 360)
    t = tabs(wc.plan_job(mk([-90.0, 450.0]), mach("xyc", table_unwind="wrap360")))
    assert set(round(x, 6) for x in t) == {270.0, 90.0}
    # continuous with limits: the equivalent angle nearest the current one inside the limits
    t = tabs(wc.plan_job(mk([0.0, 270.0, 0.0]), mach("xyc", limits={"table": (-180.0, 180.0)})))
    assert set(round(x, 6) for x in t) == {0.0, -90.0}
    with pytest.raises(wc.TravelLimitError):
        wc.plan_job(mk([0.0]), mach("xyc", limits={"table": (10.0, 20.0)}))
    # rewind: continuous during the job, rapid back into 0..360 at the end
    pl = wc.plan_job(mk([0.0, 120.0, 240.0, 0.0]), mach("xyc", table_unwind="rewind", limits={"table": (-720.0, 720.0)}))
    t = tabs(pl)
    assert 360.0 in [round(x, 6) for x in t]                        # went on to 360 instead of back to 0
    assert t[-1] == 0.0 and pl.moves()[-1].kind == "rapid"          # unwound after the last cut


def test_feed_missing_raises():
    prof = wc.Profile.from_points([(0, 0), (100, 0)])
    mc = mach("xy")
    with pytest.raises(wc.FeedError, match="feed"):
        wc.write_gcode(wc.WireJob("f", [wc.Setup2D(prof, waste="left")]), mc)
    with pytest.raises(wc.FeedError):
        wc.write_gcode(wc.WireJob("f", [wc.Setup2D(prof, waste="left")], feed=0.0), mc)
    text = wc.write_gcode(wc.WireJob("f", [wc.Setup2D(prof, waste="left", feed=33.0)]), mc)[0]   # per-setup feed is enough
    assert "F33" in text


def test_program_structure_checks():
    mc = mach("xy")
    prof = wc.Profile.from_points([(0, 0), (100, 0)])
    text = wc.write_gcode(wc.WireJob("p", [wc.Setup2D(prof, waste="left")], feed=40.0), mc)[0]
    wc.check_program(text, mc)
    lines = text.splitlines()
    first = next(i for i, l in enumerate(lines) if l.startswith("G1"))
    no_start = "\n".join(l for l in lines if l != "M3")
    with pytest.raises(wc.ProgramError, match="wire start"):
        wc.check_program(no_start, mc)
    no_stop = "\n".join(l for l in lines if l != "M5")
    with pytest.raises(wc.ProgramError, match="wire stop"):
        wc.check_program(no_stop, mc)
    order = lines[:]
    order.insert(first + 1, order.pop(order.index("M3")))                 # start after the first cut
    with pytest.raises(wc.ProgramError):
        wc.check_program("\n".join(order), mc)


def test_corner_dwell_and_slowdown():
    mc = mach("xy", kerf_mode="g41", arc_plane=None, corner_dwell_s=1.5, corner_slowdown_factor=0.5,
              corner_slowdown_length=10.0)
    prof = wc.Profile.from_points([(0, 0), (200, 0), (200, 100), (0, 100)])
    text = wc.write_gcode(wc.WireJob("c", [wc.Setup2D(prof, waste="left")], feed=100.0), mc)[0]
    assert text.count("G4 P1.5") == 2                                      # two corners
    feeds = [b.words["F"] for b in wc.parse_gcode(text) if b.g and b.g[-1] == 1]
    assert 50.0 in feeds and 100.0 in feeds
    ms = dataclasses.replace(mc, dwell_in_ms=True)
    assert "G4 P1500" in wc.write_gcode(wc.WireJob("c", [wc.Setup2D(prof, waste="left")], feed=100.0), ms)[0]


# ---------------------------------------------------------------------------------------------------- xyuv
def test_ruled_monotone_and_trolley_plane():
    mc = mach("xyuv")
    S = np.array([[100, 800, 100, 800], [300, 800, 300, 800], [200, 800, 200, 800.0]])
    with pytest.raises(wc.WireSawError, match="monotone"):
        wc.plan_job(wc.WireJob("m", [wc.RuledSetup(S)], feed=50.0), mc)
    no_u = mach("xyuv", axes={"trolley": "X", "height": "Y", "height2": "V"})
    tilted = np.array([[100, 800, 110, 800], [300, 800, 310, 800.0]])
    with pytest.raises(wc.WireSawError, match="trolley"):
        wc.plan_job(wc.WireJob("m", [wc.RuledSetup(tilted)], feed=50.0), no_u)
    wc.plan_job(wc.WireJob("m", [wc.RuledSetup(tilted)], feed=50.0), mc)  # independent second trolley: fine
    steep = mach("xyuv", max_wire_tilt_deg=10.0)
    with pytest.raises(wc.TravelLimitError, match="tilt"):
        wc.plan_job(wc.WireJob("m", [wc.RuledSetup(np.array([[100, 800, 100, 1100], [300, 800, 300, 1100.0]]))],
                               feed=50.0), steep)


def test_twisted_column_ruled_faces():
    rj = load_ruled_json(str(DATA / "ruled_json" / "twisted_column.json"))
    mc = mach("xyuv", limits={"trolley": (0, 4000), "height": (0, 2500), "trolley2": (0, 4000), "height2": (0, 2500)})
    frames = {0: ("z", "y", "-x"), 2: ("z", "y", "-x"), 1: ("z", "x", "y"), 3: ("z", "x", "y")}
    for i, fr in frames.items():
        face = rj.faces[i]
        st = wc.ruled_face_setup(face, mc, fr, offset=(200.0, 0.0, 1000.0))
        job = wc.WireJob("tw", [st], feed=60.0)
        pl = wc.plan_job(job, mc)
        text = wc.write_gcode(job, mc)[0]
        plan_equals_readback(pl, text, mc)
        # the written wire is the original ruling pushed by the wire radius along the outward normal
        Rm = wc._frame_matrix(fr)
        order = np.argsort([(Rm @ np.array(r.p0))[0] for r in face.rulings], kind="stable")
        d0, d1 = None, None
        mid = float(np.mean([(Rm @ (np.array(p) + R * np.array(r.normal)))[1]
                             for r in face.rulings for p in (r.p0, r.p1)]))
        d0, d1 = mid - 600.0, mid + 600.0
        for row, ri in zip(st.samples, np.argsort(
                [(Rm @ (np.array(r.p0) + np.array(r.p1)) / 2)[0] for r in face.rulings], kind="stable")):
            r = face.rulings[ri]
            w0, w1 = np.array([row[0], d0, row[1]]), np.array([row[2], d1, row[3]])
            u = (w1 - w0) / np.linalg.norm(w1 - w0)
            pm = Rm @ ((np.array(r.p0) + np.array(r.p1)) / 2) + np.array([200.0, 0.0, 1000.0])
            assert abs(np.linalg.norm(np.cross(pm - w0, u)) - R) < 1e-6


def test_ruled_frame_must_be_a_rotation_and_ruling_angle():
    rj = load_ruled_json(str(DATA / "ruled_json" / "twisted_column.json"))
    mc = mach("xyuv")
    with pytest.raises(wc.WireSawError, match="rotation"):
        wc.ruled_face_setup(rj.faces[0], mc, ("z", "y", "x"))
    with pytest.raises(wc.WireSawError, match="depth axis"):
        wc.ruled_face_setup(rj.faces[1], mc, ("z", "y", "-x"))             # rulings run along x there
    with pytest.raises(wc.WireSawError, match="kerf_mode cam"):
        wc.ruled_face_setup(rj.faces[0], mach("xyuv", kerf_mode="g41"), ("z", "y", "-x"))


# ---------------------------------------------------------------------------------------------------- DXF
def read_dxf(text: str):
    toks = text.splitlines()
    pairs = [(int(toks[i]), toks[i + 1].strip()) for i in range(0, len(toks) - 1, 2)]
    ents, cur, in_ent = [], None, False
    for c, v in pairs:
        if c == 0 and v == "SECTION":
            continue
        if c == 2 and v == "ENTITIES":
            in_ent = True
            continue
        if c == 0 and v in ("ENDSEC", "EOF") and in_ent:
            in_ent = False if v == "ENDSEC" else in_ent
            continue
        if not in_ent:
            continue
        if c == 0:
            cur = {"type": v, "codes": []}
            ents.append(cur)
        else:
            cur["codes"].append((c, v))
    return ents


def dxf_points(ents, layer):
    pts = []
    for e in ents:
        g = {}
        for c, v in e["codes"]:
            g.setdefault(c, []).append(v)
        if g[8][0] != layer:
            continue
        if e["type"] == "LINE":
            pts += [(float(g[10][0]), float(g[20][0])), (float(g[11][0]), float(g[21][0]))]
        elif e["type"] == "ARC":
            c = np.array([float(g[10][0]), float(g[20][0])])
            r = float(g[40][0])
            a0, a1 = math.radians(float(g[50][0])), math.radians(float(g[51][0]))
            sw = (a1 - a0) % (2 * math.pi)
            n = max(2, int(r * sw / 1.0))
            pts += [tuple(c + r * np.array([math.cos(a0 + sw * k / n), math.sin(a0 + sw * k / n)])) for k in range(n + 1)]
        elif e["type"] == "LWPOLYLINE":
            xs, ys, bs = [float(v) for v in g[10]], [float(v) for v in g[20]], [float(v) for v in g.get(42, [])]
            # bulges are listed after their vertex; rebuild with from_bulge
            codes = [(c, float(v)) for c, v in e["codes"] if c in (10, 20, 42)]
            vx, bl = [], []
            for c, v in codes:
                if c == 10:
                    vx.append([v, None]); bl.append(0.0)
                elif c == 20:
                    vx[-1][1] = v
                else:
                    bl[-1] = v
            pr = wc.Profile.from_bulge([tuple(p) for p in vx], bl)
            pts += [tuple(p) for p in pr.flatten(0.001, step=1.0)]
    return np.array(pts)


@pytest.mark.parametrize("lw", [False, True])
def test_dxf_parses_back_to_the_planned_path(lw):
    mc = mach("xy")
    job = wc.WireJob("d", [wc.Setup2D(rounded_rect(), name="Outline 1")], feed=100.0)
    pl = wc.plan_job(job, mc)
    dxf = wc.write_dxf(job, mc, lwpolyline=lw)
    assert "AC1015" in dxf if lw else "AC1009" in dxf
    ents = read_dxf(dxf)
    types = {e["type"] for e in ents}
    assert types == ({"LWPOLYLINE"} if lw else {"LINE", "ARC"})
    layer = pl.paths[0].layer
    assert layer == "S1_OUTLINE_1"
    pts = dxf_points(ents, layer)
    plan_path = path_of(pl).flatten(0.0005, step=1.0)
    assert wc._seg_dist(pts, plan_path).max() < 2e-3                       # every DXF point is on the planned path
    # the end points of the path are in the file
    first, last = path_of(pl).start, path_of(pl).end()
    for p in (first, last):
        assert np.min(np.hypot(*(pts - np.array(p)).T)) < 1e-6


def test_dxf_layers_per_setup_and_g41_programmed_layer():
    mc = mach("xyc", kerf_mode="cam")
    prof = wc.Profile.from_points([(300.0, 0.0), (300.0, 100.0), (280.0, 200.0)])
    job = wc.WireJob("d", [wc.Setup2D(prof, table_deg=a, waste="right", name=f"s{a}") for a in (0, 90)], feed=50.0)
    ents = read_dxf(wc.write_dxf(job, mc))
    assert {dict(e["codes"])[8] for e in ents} == {"S1_S0", "S2_S90"}
    g = mach("xy", kerf_mode="g41")
    ents = read_dxf(wc.write_dxf(wc.WireJob("d", [wc.Setup2D(prof, waste="left", name="a")], feed=50.0), g))
    assert {dict(e["codes"])[8] for e in ents} == {"S1_A", "S1_A_PROG"}


# ---------------------------------------------------------------------------------------------------- definitions
def test_machine_json_round_trip_and_examples(tmp_path):
    mc = mach("xyc", limits={"table": (-360.0, 360.0)}, header=["(h)"], max_lines=500)
    p = tmp_path / "m.json"
    mc.save(p)
    assert wc.MachineDef.load(p) == mc
    for name in ("generic_xy", "generic_xyc", "generic_xyuv"):
        m = wc.MachineDef.load(ROOT / "examples" / "wiresaw_machines" / f"{name}.json")
        assert m.name == name
    with pytest.raises(wc.WireSawError):
        wc.MachineDef.from_dict({"layout": "xyz"})
    with pytest.raises(wc.WireSawError, match="unknown"):
        wc.MachineDef.from_dict({"nope": 1})
    with pytest.raises(wc.WireSawError, match="share a letter"):
        mach("xy", axes={"trolley": "X", "height": "X"})
    with pytest.raises(wc.WireSawError, match="clashes"):
        mach("xy", axes={"trolley": "X", "height": "D"})
