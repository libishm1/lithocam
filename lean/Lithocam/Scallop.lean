/-
Copyright 2026 The lithocam authors. SPDX-License-Identifier: Apache-2.0

Scallop height of a ball-end tool and the stepover bound. Mirrored by tests/test_scallop.cpp.
-/
import Mathlib

namespace Lithocam

/-- Scallop (cusp) height between two parallel passes of a ball-end tool of radius `R` at stepover `s`
on a flat surface. Meaningful for `0 ≤ s ≤ 2R`. -/
noncomputable def scallop (R s : ℝ) : ℝ := R - Real.sqrt (R ^ 2 - s ^ 2 / 4)

/-- Largest stepover with scallop at most `h`. Meaningful for `0 ≤ h ≤ R`. -/
noncomputable def maxStepover (R h : ℝ) : ℝ := 2 * Real.sqrt (h * (2 * R - h))

/-- Safety: a stepover within the bound leaves a scallop no higher than `h`. -/
theorem scallop_le_of_stepover_le {R h s : ℝ} (hh0 : 0 ≤ h) (hhR : h ≤ R) (hs0 : 0 ≤ s)
    (hs : s ≤ maxStepover R h) : scallop R s ≤ h := by
  unfold scallop
  unfold maxStepover at hs
  have hq : 0 ≤ h * (2 * R - h) := mul_nonneg hh0 (by linarith)
  have ht := Real.sqrt_nonneg (h * (2 * R - h))
  have hsq := Real.sq_sqrt hq
  have h1 : s / 2 ≤ Real.sqrt (h * (2 * R - h)) := by linarith
  have h2 : s ^ 2 / 4 ≤ h * (2 * R - h) := by
    nlinarith [mul_self_le_mul_self (by linarith : (0:ℝ) ≤ s / 2) h1]
  have key : R - h ≤ Real.sqrt (R ^ 2 - s ^ 2 / 4) := by
    calc R - h = Real.sqrt ((R - h) ^ 2) := (Real.sqrt_sq (by linarith)).symm
      _ ≤ Real.sqrt (R ^ 2 - s ^ 2 / 4) := Real.sqrt_le_sqrt (by nlinarith)
  linarith

/-- Exactness: at the bound the scallop equals `h`, so the bound is not conservative. -/
theorem scallop_maxStepover {R h : ℝ} (hh0 : 0 ≤ h) (hhR : h ≤ R) :
    scallop R (maxStepover R h) = h := by
  unfold scallop maxStepover
  have hq : 0 ≤ h * (2 * R - h) := mul_nonneg hh0 (by linarith)
  have hsq := Real.sq_sqrt hq
  have e : R ^ 2 - (2 * Real.sqrt (h * (2 * R - h))) ^ 2 / 4 = (R - h) ^ 2 := by
    rw [mul_pow, hsq]; ring
  rw [e, Real.sqrt_sq (by linarith)]
  ring

/-- Monotonicity: a larger stepover never gives a smaller scallop. -/
theorem scallop_mono {R s₁ s₂ : ℝ} (h₀ : 0 ≤ s₁) (h₁₂ : s₁ ≤ s₂) : scallop R s₁ ≤ scallop R s₂ := by
  unfold scallop
  have : R ^ 2 - s₂ ^ 2 / 4 ≤ R ^ 2 - s₁ ^ 2 / 4 := by nlinarith
  linarith [Real.sqrt_le_sqrt this]

/-- The cancellation-free form used by the C++ implementation equals the definition. -/
theorem scallop_eq_stable {R s : ℝ} (hR : 0 < R) (hs : s ^ 2 / 4 ≤ R ^ 2) :
    scallop R s = (s ^ 2 / 4) / (R + Real.sqrt (R ^ 2 - s ^ 2 / 4)) := by
  unfold scallop
  have hq : 0 ≤ R ^ 2 - s ^ 2 / 4 := by linarith
  have hsq := Real.sq_sqrt hq
  have hd : 0 < R + Real.sqrt (R ^ 2 - s ^ 2 / 4) := by positivity
  rw [eq_div_iff hd.ne']
  nlinarith [hsq]

end Lithocam
