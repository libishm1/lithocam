"""Simplified linear-elastic voxel finite elements.

Eight-node hexahedral elements on the occupied voxels of a uniform grid.
Isotropic E and nu are supplied by the caller. All elements share one
24x24 stiffness matrix. The operator is matrix-free and the solver is a
Jacobi-preconditioned conjugate gradient.

Units are the caller's, but they must be consistent (for example N, mm, MPa).
No physical constants live in this module.

Limits: linear elastic, isotropic, small strain, static. Voxel staircase
surfaces give stress concentrations that a smooth part does not have.
Reference: voxel finite elements as used in topology optimisation, e.g.
Andreassen et al., Struct. Multidisc. Optim. 43 (2011), the 88-line code.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

# Natural coordinates of the 8 nodes (xi, eta, zeta) in {-1, +1}.
_SIGNS = np.array(
    [
        [-1, -1, -1],
        [1, -1, -1],
        [1, 1, -1],
        [-1, 1, -1],
        [-1, -1, 1],
        [1, -1, 1],
        [1, 1, 1],
        [-1, 1, 1],
    ],
    dtype=float,
)
_OFFSETS = ((_SIGNS + 1) / 2).astype(np.int64)  # (di, dj, dk) per node


def _cell3(cell):
    c = np.asarray(cell, dtype=float)
    if c.ndim == 0:
        c = np.full(3, float(c))
    if c.shape != (3,) or np.any(c <= 0):
        raise ValueError("cell must be a positive scalar or 3 positive values")
    return c


def constitutive_matrix(E, nu):
    """6x6 isotropic matrix, Voigt order xx, yy, zz, yz, zx, xy (engineering shear)."""
    if E <= 0 or not (-1.0 < nu < 0.5):
        raise ValueError("need E > 0 and -1 < nu < 0.5")
    lam = E * nu / ((1 + nu) * (1 - 2 * nu))
    mu = E / (2 * (1 + nu))
    D = np.zeros((6, 6))
    D[:3, :3] = lam
    D[0, 0] = D[1, 1] = D[2, 2] = lam + 2 * mu
    D[3, 3] = D[4, 4] = D[5, 5] = mu
    return D


def _bmat(dN):
    """Strain-displacement matrix from (m,3) physical shape-function gradients."""
    m = dN.shape[0]
    B = np.zeros((6, 3 * m))
    for n in range(m):
        dx, dy, dz = dN[n]
        B[0, 3 * n] = dx
        B[1, 3 * n + 1] = dy
        B[2, 3 * n + 2] = dz
        B[3, 3 * n + 1] = dz
        B[3, 3 * n + 2] = dy
        B[4, 3 * n] = dz
        B[4, 3 * n + 2] = dx
        B[5, 3 * n] = dy
        B[5, 3 * n + 1] = dx
    return B


def _grads(xi, eta, zeta, h):
    """Physical gradients of the 8 trilinear shape functions at a natural point."""
    s = _SIGNS
    t = np.array([xi, eta, zeta])
    f = 1 + s * t  # (8,3)
    dN = np.empty((8, 3))
    dN[:, 0] = 0.125 * s[:, 0] * f[:, 1] * f[:, 2]
    dN[:, 1] = 0.125 * s[:, 1] * f[:, 0] * f[:, 2]
    dN[:, 2] = 0.125 * s[:, 2] * f[:, 0] * f[:, 1]
    return dN * (2.0 / h)


def element_stiffness(cell, E, nu, incompatible=True):
    """24x24 stiffness of one brick, 2x2x2 Gauss.

    With incompatible=True the Wilson incompatible-mode enrichment (9 modes,
    condensed out) is added. It removes the shear locking of plain trilinear
    bricks in bending. Bricks are parallelepipeds, so the enrichment passes
    the constant-strain patch test.
    """
    h = _cell3(cell)
    D = constitutive_matrix(E, nu)
    g = 1 / np.sqrt(3.0)
    detj = h.prod() / 8.0
    Kuu = np.zeros((24, 24))
    Kua = np.zeros((24, 9))
    Kaa = np.zeros((9, 9))
    for xi in (-g, g):
        for eta in (-g, g):
            for ze in (-g, g):
                B = _bmat(_grads(xi, eta, ze, h))
                Kuu += B.T @ D @ B * detj
                if incompatible:
                    # modes M_k = 1 - t_k^2, derivative only along axis k
                    t = (xi, eta, ze)
                    dM = np.zeros((3, 3))
                    for k in range(3):
                        dM[k, k] = -2 * t[k] * 2.0 / h[k]
                    G = _bmat(dM)
                    Kua += B.T @ D @ G * detj
                    Kaa += G.T @ D @ G * detj
    if incompatible:
        Kuu = Kuu - Kua @ np.linalg.solve(Kaa, Kua.T)
    return 0.5 * (Kuu + Kuu.T)


@dataclass
class FEAResult:
    displacement: np.ndarray  # (nx+1, ny+1, nz+1, 3), zero on unused nodes
    von_mises: np.ndarray  # (n_el,)
    max_principal: np.ndarray  # (n_el,)
    stress: np.ndarray  # (n_el, 6) Voigt xx,yy,zz,yz,zx,xy
    iterations: int
    residual: float  # relative residual norm
    converged: bool


class VoxelFEA:
    """Matrix-free voxel FEA on an occupancy grid.

    occupancy: bool (nx, ny, nz). origin: (3,) position of the grid corner.
    cell: edge length (scalar or 3). E, nu: caller supplied.
    """

    def __init__(self, occupancy, origin, cell, E, nu, incompatible=True):
        occ = np.asarray(occupancy, dtype=bool)
        if occ.ndim != 3 or not occ.any():
            raise ValueError("occupancy must be a non-empty 3D bool grid")
        self.occ = occ
        self.origin = np.asarray(origin, dtype=float).reshape(3)
        self.h = _cell3(cell)
        self.E, self.nu = float(E), float(nu)
        self.shape = occ.shape
        self.nshape = tuple(s + 1 for s in occ.shape)
        self.D = constitutive_matrix(self.E, self.nu)
        self.Ke = element_stiffness(self.h, self.E, self.nu, incompatible)
        self.el_ijk = np.argwhere(occ)  # (n_el,3)
        self.n_el = len(self.el_ijk)
        nn = self.nshape
        ijk = self.el_ijk[:, None, :] + _OFFSETS[None, :, :]  # (n_el,8,3)
        gid = (ijk[..., 0] * nn[1] + ijk[..., 1]) * nn[2] + ijk[..., 2]
        used = np.unique(gid)
        lookup = np.full(nn[0] * nn[1] * nn[2], -1, dtype=np.int64)
        lookup[used] = np.arange(len(used))
        self.used_gid = used
        self.n_node = len(used)
        conn = lookup[gid]  # (n_el,8)
        self.edofs = (conn[:, :, None] * 3 + np.arange(3)).reshape(self.n_el, 24)
        self.ndof = 3 * self.n_node
        self._lookup = lookup
        self._diag = None
        # centre B for stress recovery
        self._DBc = self.D @ _bmat(_grads(0.0, 0.0, 0.0, self.h))

    # ----- geometry helpers -------------------------------------------
    def node_coords(self):
        """(nx+1, ny+1, nz+1, 3) node positions."""
        ax = [self.origin[d] + self.h[d] * np.arange(self.nshape[d]) for d in range(3)]
        return np.stack(np.meshgrid(*ax, indexing="ij"), axis=-1)

    def used_node_mask(self):
        m = np.zeros(self.nshape[0] * self.nshape[1] * self.nshape[2], dtype=bool)
        m[self.used_gid] = True
        return m.reshape(self.nshape)

    def surface_node_mask(self):
        """Used nodes that touch at least one empty voxel or the grid exterior."""
        pad = np.pad(self.occ, 1, constant_values=False)
        nx, ny, nz = self.shape
        full = np.ones(self.nshape, dtype=bool)
        for di, dj, dk in _OFFSETS:
            # the voxel with this node at corner (di,dj,dk) has index node - (di,dj,dk)
            sl = pad[
                1 - di : 1 - di + nx + 1,
                1 - dj : 1 - dj + ny + 1,
                1 - dk : 1 - dk + nz + 1,
            ]
            full &= sl
        return self.used_node_mask() & ~full

    def nearest_node(self, point):
        """Grid index (i,j,k) of the used node nearest to point."""
        X = self.node_coords()
        d2 = ((X - np.asarray(point, float)) ** 2).sum(-1)
        d2 = np.where(self.used_node_mask(), d2, np.inf)
        return np.unravel_index(np.argmin(d2), d2.shape)

    # ----- boundary conditions ----------------------------------------
    def empty_fixed(self):
        return np.zeros(self.nshape + (3,), dtype=bool)

    def empty_loads(self):
        return np.zeros(self.nshape + (3,))

    def fix_where(self, fixed, predicate, dofs=(0, 1, 2)):
        """Fix dofs on used nodes where predicate(x, y, z) is True. Returns fixed."""
        X = self.node_coords()
        m = np.asarray(predicate(X[..., 0], X[..., 1], X[..., 2]), dtype=bool)
        m &= self.used_node_mask()
        for d in dofs:
            fixed[..., d] |= m
        return fixed

    def fix_mask(self, fixed, mask, dofs=(0, 1, 2)):
        m = np.asarray(mask, dtype=bool) & self.used_node_mask()
        for d in dofs:
            fixed[..., d] |= m
        return fixed

    def add_point_load(self, loads, point, force):
        """Add a force (3,) at the used node nearest point."""
        loads[self.nearest_node(point)] += np.asarray(force, float)
        return loads

    def add_sphere_surface_load(self, loads, center, radius, force):
        """Spread force (3,) equally over surface nodes within radius of center."""
        X = self.node_coords()
        d2 = ((X - np.asarray(center, float)) ** 2).sum(-1)
        m = self.surface_node_mask() & (d2 <= radius**2)
        n = int(m.sum())
        if n == 0:
            raise ValueError("no surface node inside the sphere")
        loads[m] += np.asarray(force, float) / n
        return loads

    # ----- operator ----------------------------------------------------
    def apply_K(self, u):
        ue = u[self.edofs]
        fe = ue @ self.Ke
        return np.bincount(self.edofs.ravel(), weights=fe.ravel(), minlength=self.ndof)

    def diagonal(self):
        if self._diag is None:
            d = np.tile(np.diag(self.Ke), (self.n_el, 1))
            self._diag = np.bincount(self.edofs.ravel(), weights=d.ravel(), minlength=self.ndof)
        return self._diag

    def _pack(self, grid):
        """(nx+1,ny+1,nz+1,3) -> (ndof,)"""
        flat = grid.reshape(-1, 3)[self.used_gid]
        return flat.reshape(-1)

    def _unpack(self, vec):
        out = np.zeros((self.nshape[0] * self.nshape[1] * self.nshape[2], 3))
        out[self.used_gid] = vec.reshape(-1, 3)
        return out.reshape(self.nshape + (3,))

    # ----- solve -------------------------------------------------------
    def solve(self, loads, fixed, fixed_values=None, tol=1e-8, max_iter=5000):
        """Solve K u = f with Dirichlet dofs. Returns FEAResult.

        loads, fixed: grids of shape (nx+1, ny+1, nz+1, 3).
        fixed_values: optional prescribed displacement grid, same shape.
        tol: relative residual target. max_iter: iteration cap.
        """
        f = self._pack(np.asarray(loads, float))
        fix = self._pack(np.asarray(fixed, bool)).astype(bool)
        if not fix.any():
            raise ValueError("no fixed dofs: the model would float")
        free = ~fix
        up = np.zeros(self.ndof)
        if fixed_values is not None:
            up = self._pack(np.asarray(fixed_values, float)) * fix
        b = np.where(free, f - self.apply_K(up) if up.any() else f, 0.0)
        dg = self.diagonal()
        minv = np.where(free, 1.0 / dg, 0.0)
        bn = np.linalg.norm(b)
        x = np.zeros(self.ndof)
        it, res, ok = 0, 0.0, True
        if bn > 0:
            r = b.copy()
            z = minv * r
            p = z.copy()
            rz = r @ z
            ok = False
            for it in range(1, max_iter + 1):
                Ap = np.where(free, self.apply_K(np.where(free, p, 0.0)), 0.0)
                alpha = rz / (p @ Ap)
                x += alpha * p
                r -= alpha * Ap
                res = np.linalg.norm(r) / bn
                if res < tol:
                    ok = True
                    break
                z = minv * r
                rz_new = r @ z
                p = z + (rz_new / rz) * p
                rz = rz_new
        u = x + up
        ue = u[self.edofs]
        sig = ue @ self._DBc.T  # (n_el,6)
        vm = self.von_mises(sig)
        pr = self.max_principal_of(sig)
        return FEAResult(self._unpack(u), vm, pr, sig, it, float(res), ok)

    @staticmethod
    def von_mises(sig):
        sx, sy, sz, syz, szx, sxy = sig.T
        return np.sqrt(
            0.5 * ((sx - sy) ** 2 + (sy - sz) ** 2 + (sz - sx) ** 2)
            + 3 * (syz**2 + szx**2 + sxy**2)
        )

    @staticmethod
    def max_principal_of(sig):
        sx, sy, sz, syz, szx, sxy = sig.T
        T = np.empty((len(sig), 3, 3))
        T[:, 0, 0], T[:, 1, 1], T[:, 2, 2] = sx, sy, sz
        T[:, 1, 2] = T[:, 2, 1] = syz
        T[:, 0, 2] = T[:, 2, 0] = szx
        T[:, 0, 1] = T[:, 1, 0] = sxy
        return np.linalg.eigvalsh(T)[:, -1]

    # ----- results -----------------------------------------------------
    def element_field_to_grid(self, values, fill=np.nan):
        """Element values (n_el,) -> (nx, ny, nz) grid, `fill` on empty voxels."""
        out = np.full(self.shape, fill, dtype=float)
        e = self.el_ijk
        out[e[:, 0], e[:, 1], e[:, 2]] = values
        return out

    def voxel_colors(self, values, vmin=None, vmax=None):
        """Colour per voxel, (nx, ny, nz, 4) uint8 RGBA, blue to red ramp.

        Empty voxels are fully transparent. vmin/vmax default to the data range.
        """
        v = np.asarray(values, float)
        lo = float(v.min()) if vmin is None else vmin
        hi = float(v.max()) if vmax is None else vmax
        t = np.clip((v - lo) / (hi - lo), 0, 1) if hi > lo else np.zeros_like(v)
        rgb = np.stack([t, 1 - np.abs(2 * t - 1), 1 - t], axis=1)
        out = np.zeros(self.shape + (4,), dtype=np.uint8)
        e = self.el_ijk
        out[e[:, 0], e[:, 1], e[:, 2], :3] = np.round(rgb * 255).astype(np.uint8)
        out[e[:, 0], e[:, 1], e[:, 2], 3] = 255
        return out
