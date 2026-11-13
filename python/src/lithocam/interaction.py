"""Simplified tool-to-stock interaction models.

Every input is caller supplied. No physical constant is defined here.
Units must be consistent across a call. The comments give the SI units that
make the formulas dimensionally correct: m, s, N, W, Pa, J/m^3 for specific
energy, rad for angles, N/m for stiffness, N*m/rad for joint stiffness.

Limits: static, linear, quasi-steady. No chatter, no tool wear, no thermal
effects, no fracture mechanics.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

# ---------------------------------------------------------------- milling


@dataclass
class MillingForces:
    mrr: float  # material removal rate, volume / s
    power: float  # cutting power, W
    cutting_speed: float  # v_c, m/s
    Ft: float  # tangential force magnitude, N
    Fr: float  # radial force magnitude, N
    Fa: float  # axial force magnitude, N
    force_on_tool: np.ndarray  # (3,) N, in the frame of the input vectors
    t_hat: np.ndarray
    r_hat: np.ndarray
    a_hat: np.ndarray


def _unit(v, name):
    v = np.asarray(v, float).reshape(3)
    n = np.linalg.norm(v)
    if n == 0:
        raise ValueError(f"{name} has zero length")
    return v / n


def milling_forces(
    removed_volume,
    step_time,
    specific_energy,
    diameter,
    rpm,
    feed_dir,
    tool_axis,
    radial_ratio,
    axial_ratio,
    side=1.0,
):
    """Mechanistic specific-energy milling force for one step.

    removed_volume [m^3], step_time [s], specific_energy u [J/m^3],
    diameter D [m], rpm [1/min]. radial_ratio = Fr/Ft, axial_ratio = Fa/Ft.
    feed_dir: direction of tool motion relative to the stock.
    tool_axis: unit direction pointing from the tip toward the holder.
    side: +1 or -1, which side of the feed the stock is on (see below).

    MRR = V / dt. P = u * MRR. v_c = pi D rpm / 60. Ft = P / v_c.

    Simplification: the force is a single resultant per step. Its tangential
    part acts on the tool opposite the feed component that is perpendicular to
    the tool axis. The radial part is perpendicular to both that feed
    component and the axis: r_hat = -side * (axis x feed_perp). The caller
    sets side so that r_hat points away from the stock (flip the sign if the
    stock is on the other side of the feed). The axial part is along the axis (pulling out of
    the stock, toward the holder, when axial_ratio > 0). Tooth-by-tooth
    variation and the engagement angle are ignored. A pure plunge (feed
    parallel to the axis) has no tangential direction and raises ValueError.
    """
    if step_time <= 0 or diameter <= 0 or rpm <= 0 or removed_volume < 0:
        raise ValueError("step_time, diameter, rpm must be > 0 and removed_volume >= 0")
    a = _unit(tool_axis, "tool_axis")
    f = _unit(feed_dir, "feed_dir")
    fp = f - (f @ a) * a
    if np.linalg.norm(fp) < 1e-9:
        raise ValueError("feed is parallel to the tool axis: tangential direction undefined")
    fp /= np.linalg.norm(fp)
    t_hat = -fp
    r_hat = -float(np.sign(side)) * np.cross(a, fp)
    mrr = removed_volume / step_time
    power = specific_energy * mrr
    vc = np.pi * diameter * rpm / 60.0
    Ft = power / vc
    Fr, Fa = radial_ratio * Ft, axial_ratio * Ft
    F = Ft * t_hat + Fr * r_hat + Fa * a
    return MillingForces(mrr, power, vc, Ft, Fr, Fa, F, t_hat, r_hat, a)


# ----------------------------------------------------------- tool deflection


def stepped_cantilever_tip_deflection(
    force, E, shank_diameter, shank_length, flute_diameter_factor, flute_length, flute_diameter
):
    """Tip deflection of a stepped round cantilever under a tip force.

    Holder end clamped at x = 0. Shank segment [0, Ls] has diameter Ds.
    Flute segment [Ls, Ls + Lf] has equivalent diameter De = factor * D,
    with D = flute_diameter (the nominal cutter diameter).
    Euler-Bernoulli, no shear, rigid clamp.

    Superposition of the two segments with a unit load at the tip, with
    L = Ls + Lf:
      delta = F/(3E) * [ (L^3 - Lf^3)/I1 + Lf^3/I2 ]
    I = pi d^4 / 64. Returns the deflection (same unit as length, for the
    same force unit and E).
    """
    I1 = np.pi * shank_diameter**4 / 64.0
    I2 = np.pi * (flute_diameter_factor * flute_diameter) ** 4 / 64.0
    L = shank_length + flute_length
    return force / (3.0 * E) * ((L**3 - flute_length**3) / I1 + flute_length**3 / I2)


def stepped_cantilever_stiffness(
    E, shank_diameter, shank_length, flute_diameter_factor, flute_length, flute_diameter
):
    """Tip stiffness = 1 / deflection per unit force."""
    return 1.0 / stepped_cantilever_tip_deflection(
        1.0, E, shank_diameter, shank_length, flute_diameter_factor, flute_length, flute_diameter
    )


# --------------------------------------------------------- robot compliance


def geometric_jacobian_revolute(joint_axes, joint_points, tcp):
    """6xn geometric Jacobian for revolute joints.

    joint_axes (n,3) unit axes in the base frame, joint_points (n,3) a point on
    each axis, tcp (3,). Column i = [axis_i x (tcp - p_i); axis_i]. Linear rows
    first, angular rows last.
    """
    ax = np.asarray(joint_axes, float)
    p = np.asarray(joint_points, float)
    tcp = np.asarray(tcp, float)
    J = np.zeros((6, len(ax)))
    for i in range(len(ax)):
        J[:3, i] = np.cross(ax[i], tcp - p[i])
        J[3:, i] = ax[i]
    return J


def cartesian_compliance(J, joint_stiffness):
    """C = J Kq^-1 J^T. J: (6,n) or (m,n). joint_stiffness: (n,) N*m/rad."""
    J = np.asarray(J, float)
    k = np.asarray(joint_stiffness, float)
    if np.any(k <= 0) or k.shape != (J.shape[1],):
        raise ValueError("joint_stiffness must be n positive values")
    return (J / k) @ J.T


def tcp_deflection(J, joint_stiffness, wrench):
    """TCP displacement and rotation = C @ wrench (force N, moment N*m).

    Valid for small loads about one posture. Link flexibility, gearbox
    hysteresis and gravity sag are outside the model.
    """
    return cartesian_compliance(J, joint_stiffness) @ np.asarray(wrench, float)


# ------------------------------------------------------------ diamond wire


def occupancy_lookup(points, occupancy, origin, cell):
    """Nearest-cell occupancy at points (m,3). Outside the grid returns False."""
    occ = np.asarray(occupancy, bool)
    p = np.atleast_2d(np.asarray(points, float))
    idx = np.floor((p - np.asarray(origin, float)) / cell).astype(np.int64)
    inside = np.all((idx >= 0) & (idx < np.array(occ.shape)), axis=1)
    out = np.zeros(len(p), bool)
    ii = idx[inside]
    out[inside] = occ[ii[:, 0], ii[:, 1], ii[:, 2]]
    return out


def wire_contact_length(points, occupancy, origin, cell):
    """Wire length inside the stock from an ordered polyline of sample points.

    Each segment counts fully if its midpoint cell is occupied (nearest-cell
    lookup). The error is about one cell per entry or exit.
    """
    p = np.asarray(points, float)
    mid = 0.5 * (p[:-1] + p[1:])
    seg = np.linalg.norm(p[1:] - p[:-1], axis=1)
    return float(seg[occupancy_lookup(mid, occupancy, origin, cell)].sum())


@dataclass
class WireForces:
    power: float
    Ft: float
    Fn: float
    q: float  # normal load per length over the contact, N/m


def wire_forces(specific_energy, kerf, contact_length, feed_speed, wire_speed, normal_ratio):
    """Diamond wire cutting forces.

    P = u * kerf * Lc * v_feed (u in J/m^3, kerf and Lc in m, v_feed in m/s).
    Ft = P / v_wire. Fn = normal_ratio * Ft. q = Fn / Lc.
    """
    if wire_speed <= 0 or contact_length <= 0:
        raise ValueError("wire_speed and contact_length must be > 0")
    P = specific_energy * kerf * contact_length * feed_speed
    Ft = P / wire_speed
    Fn = normal_ratio * Ft
    return WireForces(P, Ft, Fn, Fn / contact_length)


@dataclass
class WireBow:
    max_deflection: float  # at midspan, m
    end_slope: float  # tan of the angle at the supports
    lag_angle: float  # atan(end_slope), rad: angle of the wire at the span ends


def wire_bow(normal_force, span, contact_length, tension):
    """Bow of a taut wire under a uniform normal load.

    Span L between pinned supports, tension T held constant, load q = Fn/Lc
    over a length Lc centred in the span. Small-deflection assumption: slopes
    are small, T does not change with the bow, and the wire is a perfect
    string (no bending stiffness, no sag, no wire speed effects).
    Solving T y'' = -q(x):
      reactions R = Fn / 2 on each side
      midspan deflection = Fn (2L - Lc) / (8 T)
    For Lc = L this is q L^2 / (8 T). The slope at each support is R / T and
    the lag angle is its arctangent.
    """
    if not (0 < contact_length <= span) or tension <= 0:
        raise ValueError("need 0 < contact_length <= span and tension > 0")
    ymax = normal_force * (2 * span - contact_length) / (8.0 * tension)
    slope = normal_force / (2.0 * tension)
    return WireBow(ymax, slope, float(np.arctan(slope)))


def wire_bow_profile(normal_force, span, contact_length, tension, n=201):
    """Deflection y(x) at n points along the span (same assumptions as wire_bow)."""
    x = np.linspace(0, span, n)
    q = normal_force / contact_length
    a = 0.5 * (span - contact_length)
    R = normal_force / 2.0
    y = np.where(
        x <= a,
        R * x / tension,
        np.where(
            x <= a + contact_length,
            R * x / tension - q * (x - a) ** 2 / (2 * tension),
            R * (span - x) / tension,
        ),
    )
    return x, y


# -------------------------------------------------------------- part check


@dataclass
class PartCheck:
    max_von_mises: float
    max_principal: float
    safety_factor: float  # tensile_strength / max tensile principal; inf if none
    flagged: np.ndarray  # (nx,ny,nz) bool, principal stress > strength
    flagged_count: int
    peak_voxel: tuple  # (i,j,k) of the max principal stress
    result: object  # FEAResult
    model: object  # VoxelFEA


def part_check(
    occupancy,
    origin,
    cell,
    E,
    nu,
    z_fix,
    force,
    contact_point,
    tensile_strength,
    load_radius=None,
    distributed=None,
    tol=1e-8,
    max_iter=5000,
):
    """Static strength check of the remaining part.

    The part is clamped on every node with z <= z_fix (the base on the table).
    force (3,) acts at contact_point: on the nearest node, or spread over the
    surface nodes within load_radius if given (a point load has an unbounded
    stress in a continuum, so pass a radius of about a few cells for a
    meaningful peak). distributed: optional dict {center, radius, force} for a
    second load spread over surface nodes in a sphere.
    Safety factor = tensile_strength / max principal stress. Stone is brittle
    and its strength scatters, so treat the factor as a screening index and
    not as a guarantee.
    """
    from .fea import VoxelFEA

    m = VoxelFEA(occupancy, origin, cell, E, nu)
    fixed = m.empty_fixed()
    m.fix_where(fixed, lambda x, y, zz: zz <= z_fix + 1e-12)
    loads = m.empty_loads()
    if load_radius is None:
        m.add_point_load(loads, contact_point, force)
    else:
        m.add_sphere_surface_load(loads, contact_point, load_radius, force)
    if distributed is not None:
        m.add_sphere_surface_load(
            loads, distributed["center"], distributed["radius"], distributed["force"]
        )
    r = m.solve(loads, fixed, tol=tol, max_iter=max_iter)
    pr = r.max_principal
    mx = float(pr.max())
    sf = float(tensile_strength / mx) if mx > 0 else float("inf")
    flag = m.element_field_to_grid((pr > tensile_strength).astype(float), fill=0.0) > 0
    pk = tuple(int(v) for v in m.el_ijk[int(np.argmax(pr))])
    return PartCheck(float(r.von_mises.max()), mx, sf, flag, int(flag.sum()), pk, r, m)
