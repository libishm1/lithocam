/-
Copyright 2026 The lithocam authors. SPDX-License-Identifier: Apache-2.0

Kerf offset of a wire or blade along a straight part edge, in any real inner product space.
Mirrored by tests/test_kerf.cpp, which extends the check to convex parts with circumscribed corner arcs.

Setting: the part lies in the half-space `{p | ⟪p - a, n⟫ ≥ 0}` with unit inward normal `n` and boundary point `a`.
The wire centre runs on the line through `a - (k/2) • n`. The kerf is every point within `k/2` of that line,
measured along `n`.
-/
import Mathlib

open scoped InnerProductSpace   -- provides ⟪x, y⟫_ℝ (RealInnerProductSpace gives the unsubscripted form)

namespace Lithocam

variable {E : Type*} [NormedAddCommGroup E] [InnerProductSpace ℝ E]

private lemma inner_shift (a n p : E) (k : ℝ) (hn : ‖n‖ = 1) :
    ⟪p - (a - (k / 2) • n), n⟫_ℝ = ⟪p - a, n⟫_ℝ + k / 2 := by
  have hnn : ⟪n, n⟫_ℝ = 1 := by rw [real_inner_self_eq_norm_sq, hn]; norm_num
  rw [show p - (a - (k / 2) • n) = (p - a) + (k / 2) • n by abel, inner_add_left, real_inner_smul_left, hnn,
    mul_one]

/-- No part loss: every point of the kerf lies on the waste side of (or on) the part boundary. -/
theorem kerf_strip_outside_part {a n p : E} {k : ℝ} (hn : ‖n‖ = 1)
    (hp : |⟪p - (a - (k / 2) • n), n⟫_ℝ| ≤ k / 2) : ⟪p - a, n⟫_ℝ ≤ 0 := by
  rw [inner_shift a n p k hn] at hp
  linarith [(abs_le.mp hp).2]

/-- No extra waste: the kerf reaches the part boundary (the boundary point `a` is in the kerf). -/
theorem kerf_touches_boundary {a n : E} {k : ℝ} (hn : ‖n‖ = 1) (hk : 0 ≤ k) :
    |⟪a - (a - (k / 2) • n), n⟫_ℝ| ≤ k / 2 := by
  rw [inner_shift a n a k hn, sub_self, inner_zero_left, zero_add, abs_of_nonneg (by linarith)]

end Lithocam
