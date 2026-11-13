# Tool-to-stock interaction and voxel FEA

Two modules: `lithocam.fea` (voxel finite elements) and `lithocam.interaction` (force, deflection and part-strength models). Both take plain arrays. Every material and process constant is a parameter. No default physical value is stored in code.

Tests: `python/tests/test_fea.py`, `python/tests/test_interaction.py`. All use synthetic values.

## 1. Voxel FEA (`fea.py`)

Eight-node bricks on the occupied voxels of a uniform grid. Isotropic E and nu. One 24x24 stiffness matrix serves every element.

- Element stiffness: Ke = sum over 2x2x2 Gauss points of B^T D B det(J). Voigt order xx, yy, zz, yz, zx, xy.
- Enrichment: Wilson incompatible modes (9 modes, condensed out), on by default. Plain trilinear bricks lock in bending. Bricks are parallelepipeds, so the modes keep the constant-strain patch test.
- Operator: `apply_K(u)` gathers `u[edofs]` as (n_el, 24), multiplies by Ke, and scatters with `np.bincount`. No global matrix.
- Solver: conjugate gradient with a Jacobi (diagonal) preconditioner. Fixed dofs are masked. Result carries `iterations`, relative `residual`, `converged`.
- Boundary conditions: grids of shape (nx+1, ny+1, nz+1, 3). `fix_where(fixed, predicate, dofs)`, `fix_mask`, `add_point_load` (nearest used node), `add_sphere_surface_load` (equal share over surface nodes inside a sphere). Prescribed displacements via `fixed_values`.
- Results: nodal displacement grid, per-element stress at the centre (`stress`), `von_mises`, `max_principal`. `element_field_to_grid(values)` gives a voxel grid (NaN where empty). `voxel_colors(values, vmin, vmax)` gives RGBA uint8 per voxel on a blue to red ramp, transparent where empty.

Signature: `VoxelFEA(occupancy, origin, cell, E, nu, incompatible=True)`, then `solve(loads, fixed, fixed_values=None, tol=1e-8, max_iter=5000) -> FEAResult`.

### Validation (measured)

| Test | Result |
|---|---|
| Axial bar, 12x2x2 bricks, exact PL/(EA) | relative error 6e-15 (consistent face loads, roller supports; the test limit is 3%) |
| Cantilever, L/d = 10, 4 bricks through depth, vs Euler-Bernoulli plus Timoshenko shear | error -0.66% |
| Same, refinement 1, 2, 4, 6, 8 bricks through depth | -1.10%, -0.94%, -0.66%, -0.58%, -0.54% |
| Same, 4 bricks, plain trilinear (no enrichment) | -3.7% |
| Patch test, linear displacement on the boundary | interior nodes and stresses exact to 1e-9 |
| Symmetric loading | symmetric and antisymmetric components equal to 1e-8 of the peak |
| Element stiffness | exactly 6 zero eigenvalues (rigid modes) |

The cantilever reference is delta = P L^3 / (3 E I) + P L / (kappa G A), with kappa = 5/6 for a rectangle (Timoshenko beam theory). The shear term is 0.8% of the total at L/d = 10. The clamped end constrains the Poisson contraction, which is a small additional stiffening.

### Cost

Measured on this machine, tolerance 1e-6, cantilever bar with a tip load (a slender, hard case for Jacobi CG).

| Voxels | Grid | Build | Solve | Iterations |
|---|---|---|---|---|
| 51 200 | 50x32x32 | 0.04 s | 5.2 s | 279 |
| 50 000 | 100x20x25 | 0.05 s | 8.3 s | 446 |

Iterations grow with slenderness. A thin fin on a base converges in fewer steps.

## 2. Interaction models (`interaction.py`)

### 2.1 Milling forces

`milling_forces(removed_volume, step_time, specific_energy, diameter, rpm, feed_dir, tool_axis, radial_ratio, axial_ratio, side=1)`

- MRR = V / dt
- P = u * MRR (u is the specific cutting energy, energy per volume)
- v_c = pi D rpm / 60
- Ft = P / v_c
- Fr = radial_ratio * Ft, Fa = axial_ratio * Ft

Direction simplification: one resultant per step. Tangential force on the tool is opposite the component of the feed that is perpendicular to the tool axis. Radial is perpendicular to that component and to the axis, `r_hat = -side * (axis x feed_perp)`, and the caller picks `side` so it points away from the stock. Axial is along the axis (axis points from tip to holder). A pure plunge raises `ValueError`. There is no tooth-by-tooth variation and no engagement angle. The method follows the mechanistic specific-energy cutting models in the machining literature.

### 2.2 Tool deflection

`stepped_cantilever_tip_deflection(force, E, shank_diameter, shank_length, flute_diameter_factor, flute_length, flute_diameter)`

Holder clamped. Shank of diameter Ds over Ls. Flute of equivalent diameter De = factor * D over Lf. L = Ls + Lf. With I = pi d^4 / 64, superposition of the two segments under a tip load gives

delta = F / (3E) * [ (L^3 - Lf^3) / I1 + Lf^3 / I2 ]

For equal diameters this is F L^3 / (3 E I). Euler-Bernoulli: no shear, rigid holder. The factor for the flute is the caller's choice. `stepped_cantilever_stiffness` returns 1 / delta per unit force.

Validation against `fea.py` on a voxel cylinder (shank 12 cells wide, 14 long; flute 6 wide, 14 long): FEA 0.02119, closed form 0.02067, error +2.5% (limit 15%). The voxel circle has a slightly different area and inertia than the ideal circle. The FEA also includes shear.

### 2.3 Robot compliance

`geometric_jacobian_revolute(axes, points, tcp)`, `cartesian_compliance(J, joint_stiffness)`, `tcp_deflection(J, joint_stiffness, wrench)`.

C = J Kq^-1 J^T, with Kq = diag(joint stiffness). Deflection = C @ wrench (force then moment, matching the Jacobian rows). Test: two planar links, q = (0, pi/2), unit links, k1 = 2, k2 = 4. Hand result C = [[1/k1 + 1/k2, -1/k1], [-1/k1, 1/k1]]. The code matches exactly (to rounding). Link flexibility, gearbox backlash and gravity sag are outside the model.

### 2.4 Diamond wire

`wire_forces(specific_energy, kerf, contact_length, feed_speed, wire_speed, normal_ratio)`:
P = u * kerf * Lc * v_feed, Ft = P / v_wire, Fn = ratio * Ft, q = Fn / Lc.

Contact length: pass it, or compute it with `wire_contact_length(points, occupancy, origin, cell)`. Each polyline segment counts fully when the cell at its midpoint is occupied (`occupancy_lookup`, nearest cell). The error is about one cell per entry or exit.

`wire_bow(normal_force, span, contact_length, tension)`: taut string, pinned ends, constant tension T, load q over a length Lc centred in the span. Small-deflection assumption: slopes are small, T does not change with the bow, no bending stiffness, no wire sag, no wire speed effect on the shape. From T y'' = -q:

- reactions R = Fn / 2
- midspan deflection = Fn (2L - Lc) / (8 T)
- support slope R / T = Fn / (2T), lag angle = its arctangent

For Lc = L this equals q L^2 / (8 T). The test asserts that equality to 1e-12, and that the sampled profile from `wire_bow_profile` peaks at the closed-form value (1e-6).

### 2.5 Part check

`part_check(occupancy, origin, cell, E, nu, z_fix, force, contact_point, tensile_strength, load_radius=None, distributed=None)`

Clamps every node with z <= z_fix. Applies the force at the nearest node, or over surface nodes within `load_radius`. A point load has an unbounded stress in the continuum, so give a radius of a few cells to get a stable peak. Returns max von Mises, max principal stress, safety factor = tensile strength / max principal, a boolean grid of elements above the strength, the peak voxel, and the FEA result.

Test: a base 20x12x4 with a fin 8 wide, 2 thick, 12 tall and a lateral tip load. The peak principal element is at voxel (13, 5, 4), the first fin layer above the base, at the fin edge. The elements above 90% of the peak lie in z layer 4 only.

## 3. Limits

- Linear elastic, isotropic, small strain, static. Stone is anisotropic along bedding and rift. The model ignores that.
- No dynamics, no chatter, no tool wear, no thermal load.
- No fracture mechanics. Stone is brittle and its tensile strength scatters strongly. The safety factor is a screening index against a caller strength. Use a conservative strength from tests, not a mean.
- Voxel staircase surfaces create artificial stress concentrations at corners. Peak stress depends on cell size. Compare peaks between runs at one cell size.
- Stress is read at the element centre. Corner peaks are not resolved.
- Force models give a mean resultant. Real forces vary with the tooth pass and the engagement.
- The robot model is joint compliance only.

## 4. Sources

- Euler-Bernoulli and Timoshenko beam theory (cantilever deflection and shear correction).
- Mechanistic specific-energy cutting models (force from power over cutting speed).
- Voxel finite elements in topology optimisation, e.g. Andreassen et al., "Efficient topology optimization in MATLAB using 88 lines of code", Struct. Multidisc. Optim. 43, 2011.
- Wilson et al. (1973) incompatible displacement modes for bending of bricks and quadrilaterals.
