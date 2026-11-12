/-
Copyright 2026 The lithocam authors. SPDX-License-Identifier: Apache-2.0

A diamond wire against a planar face, in any real inner product space.
Mirrored by python/src/lithocam/ruled.py (rulings offset by the wire radius) and robot.wire_target.

Setting: the part lies in `{p | ⟪p - a, n⟫ ≥ 0}` (unit inward normal `n`, face point `a`). The wire's centre line runs
through `a - r • n` with a direction `d` parallel to the face (`⟪d, n⟫ = 0`). The wire is every point within `r` of
that line (a tube of radius `r`).
-/
import Mathlib

open scoped InnerProductSpace

namespace Lithocam

variable {E : Type*} [NormedAddCommGroup E] [InnerProductSpace ℝ E]

private lemma inner_centre (a n d : E) (r t : ℝ) (hn : ‖n‖ = 1) (hd : ⟪d, n⟫_ℝ = 0) :
    ⟪(a - r • n + t • d) - a, n⟫_ℝ = -r := by
  have hnn : ⟪n, n⟫_ℝ = 1 := by rw [real_inner_self_eq_norm_sq, hn]; norm_num
  rw [show (a - r • n + t • d) - a = -(r • n) + t • d by abel, inner_add_left, inner_neg_left,
    real_inner_smul_left, real_inner_smul_left, hnn, hd]
  ring

/-- No gouge: every point of the wire (within `r` of its centre line) is on the waste side of the face plane. -/
theorem wire_tube_outside_part {a n d p : E} {r t : ℝ} (hn : ‖n‖ = 1) (hd : ⟪d, n⟫_ℝ = 0)
    (hp : ‖p - (a - r • n + t • d)‖ ≤ r) : ⟪p - a, n⟫_ℝ ≤ 0 := by
  set q := a - r • n + t • d
  have hsplit : ⟪p - a, n⟫_ℝ = ⟪p - q, n⟫_ℝ + ⟪q - a, n⟫_ℝ := by
    rw [← inner_add_left]; congr 1; abel
  have hq : ⟪q - a, n⟫_ℝ = -r := inner_centre a n d r t hn hd
  have hcs : ⟪p - q, n⟫_ℝ ≤ ‖p - q‖ * ‖n‖ := real_inner_le_norm _ _
  rw [hn, mul_one] at hcs
  linarith

/-- No extra waste: the wire touches the face; the face point under the centre point is exactly `r` away. -/
theorem wire_tube_touches_face {a n d : E} {r t : ℝ} (hn : ‖n‖ = 1) (hr : 0 ≤ r) :
    ‖(a + t • d) - (a - r • n + t • d)‖ = r := by
  rw [show (a + t • d) - (a - r • n + t • d) = r • n by abel, norm_smul, hn, mul_one, Real.norm_eq_abs,
    abs_of_nonneg hr]

end Lithocam
