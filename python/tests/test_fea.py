"""Validation of lithocam.fea against closed forms. Synthetic material values."""
import numpy as np
import pytest

from lithocam.fea import VoxelFEA, element_stiffness

E, NU = 1000.0, 0.25  # synthetic, consistent units


def _box(nx, ny, nz):
    return np.ones((nx, ny, nz), dtype=bool)


def test_axial_bar_tension():
    nx, ny, nz, h = 12, 2, 2, 0.5
    m = VoxelFEA(_box(nx, ny, nz), (0, 0, 0), h, E, NU)
    fixed = m.empty_fixed()
    m.fix_where(fixed, lambda x, y, z: x < 1e-9, dofs=(0,))
    m.fix_where(fixed, lambda x, y, z: y < 1e-9, dofs=(1,))
    m.fix_where(fixed, lambda x, y, z: z < 1e-9, dofs=(2,))
    L, A = nx * h, (ny * h) * (nz * h)
    P = 3.0
    X = m.node_coords()
    tip = (X[..., 0] > L - 1e-9) & m.used_node_mask()
    loads = m.empty_loads()
    # consistent nodal weights for a uniform traction on a 2x2 face grid: 1,2,1 per axis
    w1 = np.array([1.0, 2.0, 1.0])
    w = np.einsum("j,k->jk", w1, w1) / 16.0
    loads[-1, :, :, 0] = P * w
    r = m.solve(loads, fixed, tol=1e-12)
    assert r.converged
    ux = r.displacement[-1, :, :, 0].mean()
    exact = P * L / (E * A)
    assert abs(ux - exact) / exact < 0.03
    assert tip.sum() == 9


def _cantilever(nz_el, L=10.0, d=1.0):
    h = d / nz_el
    nx = int(round(L / h))
    m = VoxelFEA(_box(nx, nz_el, nz_el), (0, 0, 0), h, E, NU)
    fixed = m.empty_fixed()
    m.fix_where(fixed, lambda x, y, z: x < 1e-9)
    loads = m.empty_loads()
    P = 1.0
    w1 = np.ones(nz_el + 1)
    w1[0] = w1[-1] = 0.5
    w1 /= w1.sum()
    loads[-1, :, :, 2] = -P * np.einsum("j,k->jk", w1, w1)
    r = m.solve(loads, fixed, tol=1e-10, max_iter=20000)
    assert r.converged
    tip = r.displacement[-1, :, :, 2].mean()
    I = d * d**3 / 12.0
    A = d * d
    G = E / (2 * (1 + NU))
    kappa = 5.0 / 6.0  # Timoshenko shear coefficient for a rectangle
    exact = P * L**3 / (3 * E * I) + P * L / (kappa * G * A)
    return abs(tip), exact


def test_cantilever_tip_deflection_and_refinement():
    errs = []
    for n in (2, 4, 6):
        tip, exact = _cantilever(n)
        errs.append(abs(tip - exact) / exact)
    assert errs[1] < 0.10
    assert errs[2] < 0.10
    assert errs[2] < errs[0]


def test_patch_uniform_strain_exact():
    n = 4
    m = VoxelFEA(_box(n, n, n), (0.0, 0.0, 0.0), 0.3, E, NU)
    A = np.array([[1e-3, 2e-3, -1e-3], [2e-3, -3e-4, 5e-4], [-1e-3, 5e-4, 7e-4]])
    b = np.array([0.01, -0.02, 0.03])
    X = m.node_coords()
    ufield = X @ A.T + b
    fixed = m.empty_fixed()
    bnd = np.zeros(m.nshape, bool)
    bnd[0] = bnd[-1] = True
    bnd[:, 0] = bnd[:, -1] = True
    bnd[:, :, 0] = bnd[:, :, -1] = True
    m.fix_mask(fixed, bnd)
    r = m.solve(m.empty_loads(), fixed, fixed_values=ufield, tol=1e-13, max_iter=5000)
    assert r.converged
    assert np.max(np.abs(r.displacement - ufield)) < 1e-9
    eps = 0.5 * (A + A.T)
    tr = np.trace(eps)
    lam = E * NU / ((1 + NU) * (1 - 2 * NU))
    mu = E / (2 * (1 + NU))
    sig = 2 * mu * eps + lam * tr * np.eye(3)
    voigt = np.array([sig[0, 0], sig[1, 1], sig[2, 2], sig[1, 2], sig[0, 2], sig[0, 1]])
    assert np.max(np.abs(r.stress - voigt)) < 1e-9 * np.abs(voigt).max() + 1e-9


def test_element_stiffness_rigid_modes():
    Ke = element_stiffness(0.7, E, NU)
    w = np.linalg.eigvalsh(Ke)
    assert np.sum(np.abs(w) < 1e-8 * w.max()) == 6


def test_symmetric_loading_symmetric_displacement():
    nx, ny, nz = 8, 4, 6
    m = VoxelFEA(_box(nx, ny, nz), (0, 0, 0), 1.0, E, NU)
    fixed = m.empty_fixed()
    m.fix_where(fixed, lambda x, y, z: z < 1e-9)
    loads = m.empty_loads()
    m.add_point_load(loads, (nx / 2, ny / 2, nz), (0, 0, -1.0))
    r = m.solve(loads, fixed, tol=1e-12)
    u = r.displacement
    assert r.converged
    sc = np.abs(u).max()
    assert np.abs(u[..., 2] - u[::-1, :, :, 2]).max() < 1e-8 * sc
    assert np.abs(u[..., 2] - u[:, ::-1, :, 2]).max() < 1e-8 * sc
    assert np.abs(u[..., 0] + u[::-1, :, :, 0][...]).max() < 1e-8 * sc
    assert np.abs(u[..., 1] + u[:, ::-1, :, 1]).max() < 1e-8 * sc


def test_colors_and_grid_helpers():
    occ = np.zeros((3, 3, 3), bool)
    occ[:2] = True
    m = VoxelFEA(occ, (0, 0, 0), 1.0, E, NU)
    fixed = m.empty_fixed()
    m.fix_where(fixed, lambda x, y, z: x < 1e-9)
    loads = m.empty_loads()
    m.add_sphere_surface_load(loads, (2, 1.5, 1.5), 1.0, (0, 0, -1.0))
    r = m.solve(loads, fixed)
    g = m.element_field_to_grid(r.von_mises)
    assert np.isnan(g[2]).all() and not np.isnan(g[:2]).any()
    c = m.voxel_colors(r.von_mises)
    assert c.shape == (3, 3, 3, 4) and c[2, 0, 0, 3] == 0 and c[0, 0, 0, 3] == 255
