"""Tests for lithocam.interaction with synthetic values."""
import numpy as np
import pytest

from lithocam import interaction as ix
from lithocam.fea import VoxelFEA

E, NU = 1000.0, 0.25


def test_milling_forces_chain_and_directions():
    r = ix.milling_forces(
        removed_volume=2.0, step_time=4.0, specific_energy=10.0, diameter=2.0, rpm=60.0,
        feed_dir=(1, 0, -0.5), tool_axis=(0, 0, 1), radial_ratio=0.5, axial_ratio=0.25,
    )
    assert r.mrr == pytest.approx(0.5)
    assert r.power == pytest.approx(5.0)
    assert r.cutting_speed == pytest.approx(np.pi * 2.0)
    assert r.Ft == pytest.approx(5.0 / (2 * np.pi))
    assert r.Fr == pytest.approx(0.5 * r.Ft) and r.Fa == pytest.approx(0.25 * r.Ft)
    assert np.allclose(r.t_hat, (-1, 0, 0))  # opposite the feed, perpendicular to the axis
    assert abs(r.t_hat @ r.a_hat) < 1e-12 and abs(r.r_hat @ r.t_hat) < 1e-12
    assert abs(r.r_hat @ r.a_hat) < 1e-12
    assert np.linalg.norm(r.force_on_tool) == pytest.approx(r.Ft * np.sqrt(1 + 0.25 + 0.0625))
    with pytest.raises(ValueError):
        ix.milling_forces(1, 1, 1, 1, 1, (0, 0, 1), (0, 0, 1), 0.1, 0.1)


def test_stepped_cantilever_reduces_to_uniform():
    d, L, F = 2.0, 30.0, 1.5
    I = np.pi * d**4 / 64
    exact = F * L**3 / (3 * E * I)
    got = ix.stepped_cantilever_tip_deflection(F, E, d, 10.0, 1.0, 20.0, d)
    assert got == pytest.approx(exact, rel=1e-12)


def _cylinder(nx, nz_layers):
    """Voxel cylinders about the z axis. nz_layers: list of (diameter_cells, layers)."""
    n = max(d for d, _ in nz_layers)
    c = n / 2.0
    ii, jj = np.meshgrid(np.arange(n) + 0.5, np.arange(n) + 0.5, indexing="ij")
    rr = np.hypot(ii - c, jj - c)
    layers = []
    for d, k in nz_layers:
        disk = rr <= d / 2.0
        layers += [disk] * k
    return np.stack(layers, axis=2)


def test_tool_deflection_matches_fea_voxel_cylinder():
    Ds, Ls, factor, Dn, Lf = 12, 14, 0.5, 12, 14  # cells; flute equivalent diameter 6
    occ = _cylinder(Ds, [(Ds, Ls), (int(factor * Dn), Lf)])
    h = 1.0
    m = VoxelFEA(occ, (0, 0, 0), h, E, NU)
    fixed = m.empty_fixed()
    m.fix_where(fixed, lambda x, y, z: z < 1e-9)
    loads = m.empty_loads()
    F = 1.0
    tipz = Ls + Lf
    top = m.used_node_mask() & (m.node_coords()[..., 2] > tipz - 1e-9)
    loads[top, 0] = F / top.sum()
    r = m.solve(loads, fixed, tol=1e-9, max_iter=20000)
    assert r.converged
    fea = r.displacement[top, 0].mean()
    # compare with the closed form on the nominal diameters
    closed = ix.stepped_cantilever_tip_deflection(F, E, Ds, Ls, factor, Lf, Dn)
    assert abs(fea - closed) / closed < 0.15


def test_robot_compliance_two_link_planar():
    k1, k2, l1, l2 = 2.0, 4.0, 1.0, 1.0
    # q = (0, pi/2): link 1 along x, link 2 along y. Both axes along z.
    pts = np.array([[0, 0, 0], [l1, 0, 0]], float)
    tcp = np.array([l1, l2, 0.0])
    J = ix.geometric_jacobian_revolute(np.array([[0, 0, 1.0], [0, 0, 1.0]]), pts, tcp)
    assert np.allclose(J[:2], [[-1, -1], [1, 0]])
    C = ix.cartesian_compliance(J, [k1, k2])
    hand = np.array([[1 / k1 + 1 / k2, -1 / k1], [-1 / k1, 1 / k1]])
    assert np.allclose(C[:2, :2], hand)
    assert np.allclose(C, C.T)
    w = np.zeros(6)
    w[:2] = (1.0, 0.0)
    d = ix.tcp_deflection(J, [k1, k2], w)
    assert np.allclose(d[:2], hand[:, 0])


def test_wire_forces_and_bow_uniform_case():
    w = ix.wire_forces(specific_energy=8.0, kerf=0.5, contact_length=2.0, feed_speed=0.25,
                       wire_speed=10.0, normal_ratio=0.4)
    assert w.power == pytest.approx(8 * 0.5 * 2 * 0.25)
    assert w.Ft == pytest.approx(w.power / 10.0)
    assert w.Fn == pytest.approx(0.4 * w.Ft) and w.q == pytest.approx(w.Fn / 2.0)
    L, T = 2.0, 50.0
    b = ix.wire_bow(w.Fn, L, L, T)
    assert b.max_deflection == pytest.approx(w.q * L**2 / (8 * T), rel=1e-12)
    assert b.lag_angle == pytest.approx(np.arctan(w.Fn / (2 * T)))
    # partial contact: profile peak equals the closed form and is smaller than uniform over span
    Fn, Lc = 3.0, 0.5
    bp = ix.wire_bow(Fn, L, Lc, T)
    x, y = ix.wire_bow_profile(Fn, L, Lc, T, n=2001)
    assert y.max() == pytest.approx(bp.max_deflection, rel=1e-6)
    assert y[0] == 0 and y[-1] == pytest.approx(0, abs=1e-12)
    assert bp.max_deflection > ix.wire_bow(Fn, L, L, T).max_deflection


def test_wire_contact_length_from_samples():
    occ = np.zeros((10, 10, 10), bool)
    occ[2:7, :, :] = True  # stock occupies x in [2, 7) for cell 1
    x = np.linspace(-1, 11, 241)
    pts = np.stack([x, np.full_like(x, 5.0), np.full_like(x, 5.0)], axis=1)
    lc = ix.wire_contact_length(pts, occ, (0, 0, 0), 1.0)
    assert lc == pytest.approx(5.0, abs=0.1)
    assert not ix.occupancy_lookup([[20, 0, 0]], occ, (0, 0, 0), 1.0)[0]


def _fin_part():
    base_h, fin_h = 4, 12
    occ = np.zeros((20, 12, base_h + fin_h), bool)
    occ[:, :, :base_h] = True
    occ[6:14, 5:7, base_h:] = True  # ear-like fin: 8 wide, 2 thick, 12 tall
    return occ, base_h, fin_h


def test_part_check_fin_peaks_at_root():
    occ, base_h, fin_h = _fin_part()
    top = base_h + fin_h
    pc = ix.part_check(
        occ, (0, 0, 0), 1.0, E, NU, z_fix=0.0,
        force=(0, 1.0, 0), contact_point=(10, 7, top), tensile_strength=1.0e6,
        load_radius=3.0,
    )
    assert pc.result.converged
    i, j, k = pc.peak_voxel
    assert base_h - 1 <= k <= base_h + 2  # at the fin root
    assert 6 <= i < 14 and 5 <= j < 7
    assert pc.max_principal > 0 and pc.max_von_mises > 0
    assert pc.safety_factor == pytest.approx(1.0e6 / pc.max_principal)
    assert pc.flagged_count == 0
    weak = ix.part_check(
        occ, (0, 0, 0), 1.0, E, NU, 0.0, (0, 1.0, 0), (10, 7, top),
        tensile_strength=0.9 * pc.max_principal, load_radius=3.0,
    )
    assert weak.flagged_count > 0 and weak.safety_factor == pytest.approx(0.9)
    # bending stress falls with height: the 90 % band sits near the root, not the tip
    ks = np.nonzero(weak.flagged)[2]
    assert ks.max() <= base_h + 4
