# Contract — `dyx3_rpp::guidance`

Source: `rpp_controller_node.py` — `_project_onto_path` 4097, `_project_onto_segment` 3610,
`_get_lookahead_point` 4180, `_segment_lookahead_point` 3397, `_corner_clip` 3579, `_path_curvature_at` 3718,
`_walk_path_samples` 3971, `_max_preview_curvature` 4043, `_pivot_intercept_heading` 3669.
Spec §7.4 row `guidance`. Status: source-verified; see `rpp_overview.md` for evidence level.

## 1. Projection (cross-track, progress)

* **Smooth** — `_project_onto_path`: windowed segment search `[hint−2, hint+4)` (full scan when the hint is
  invalid, the window is < 3 wide, or after a jump). Per segment: clamped parameter `t`, foot point,
  distance; keep the minimum. **`signed_xtrack = copysign(d, cross_z)`, + = right.** Hint persists
  per tick; a window with only zero-length segments invalidates it.
  *Known defect C9*: no monotonic-progress window — on a closed shape with coincident start/end the foot can
  snap back across the seam. Closed runs seed the hint at 0 (`_hint_valid = closed`) as a partial mitigation.
* **Segment** — `_project_onto_segment` on the *current* segment only. Returns `(t, foot, signed_e,
  dist_to_end_along)`. **`signed_e = cross_z / seg_len`** (the 2-D cross product annihilates the
  along-track component, so it is the exact perpendicular distance even when `t` is clamped).
  Rationale (field, 2026-07-30 bags `…151006`, `…152342`): the old `copysign(|pos−foot|, cross_z)` reported
  the *along-track gap to the vertex* as cross-track for one cycle at every segment handover
  (−4.88/−4.44/+4.85/+4.65 cm reported vs −0.34/−0.15/+0.44/+0.12 cm true). `t`, `foot` and
  `dist_to_end_along` stay clamped (the state machine and lookahead need a point ON the segment).
* **Progress**: `_path_travel_m = max(_path_travel_m, cum_s[i] + t·seg_len)` — monotonic; local spinning
  never counts as travel.

## 2. Lookahead distance (both profiles)

```
v_for_ld = max(min_linear_vel, last_speed_cmd if > 0 else 0.5·max_v)
v_for_ld = 0.7·v_for_ld + 0.3·max_v          # low-pass, prevents a 1-step limit cycle
L_d_raw  = lookahead_time·v_for_ld + xtrack_lookahead_gain·|xtrack|
L_d      = clamp(L_d_raw, min_lookahead_dist, max_lookahead_dist)
```
`L_d_raw` is published (`/rpp/debug[8]`) so saturation is visible.

### Smooth-profile curvature cap (P5.1)
Pure pursuit cuts inside an arc by `e ≈ L²κ/8` (no speed term — why halving speed changed nothing while the
lookahead sweep changed the error 3×). With `smooth_max_arc_cut_m > 0` and `κ_path > 1e-6`:
`L_cap = sqrt(8·e_target/κ_path)`, `L_d = max(min(L_d, L_cap), smooth_min_arc_ld_m)`. **The cap REPLACES the
legacy floor `smooth_curvature_ld_coeff/κ`** (they are irreconcilable: floor ∝ 1/κ, cap ∝ 1/√κ). Field anchor:
κ = 0.43, e = 0.005 ⇒ L ≤ 0.305 m; measured inside-cut +1.15 → −0.40 cm, marking RMS 2.31 → 1.34 cm (3/3 runs).
**Smooth only, by design**: segment mode keeps the long lookahead (a short lookahead there is the
2026-07-29 instability regime — lookahead truncated to 0.12–0.21 m gave bimodal 1.4/8 cm).
`κ_path = _path_curvature_at(seg_idx, curvature_baseline_m)`: Menger curvature with a **half-baseline walked on
arc length** (not index); adjacent-vertex curvature on 4–10 cm spacing was 3× wrong (kappa_max 1.251 vs true
0.42 on the 2026-07-30 curve). Stable from ~0.10 m baseline up.

## 3. Lookahead point

### Smooth — `_get_lookahead_point`
Walk arc length `L_d` from the foot; off the end returns the final waypoint with `hit_end = True`.
If the lookahead lands on the rover (`l_actual < 1e-6`) retry with `min_lookahead_dist`; if still degenerate →
`IDLE` zero (guards against stop-start motion on curved paths).

### Segment — `_segment_lookahead_point(seg, foot, L_d, max_junction_deg, extend_past_end, extend_past_corner)`
Walk forward from the foot, **crossing a vertex only while the junction turn ≤
`segment_lookahead_cross_collinear_deg`**. Rules (each is a field fix):
1. *Collinear vertices are not corners* — flag-boundary / must-hit anchors on a straight run must not clip the
   lookahead (2026-07-30 bag `…154238`: lookahead 0.286→0.187→0.054 m then 0.560; steering gain ∝ 1/L² ⇒
   hundredfold spike, −2.2 → +5.2 cm overshoot, line painted in two pieces). `max_junction_deg ≤ 0` restores
   the pre-fix geometry (A/B arm).
2. **D15 — path end is not a corner**: extend the aim point past the final vertex along the final bearing
   (`segment_endpoint_lookahead_extend`, default **True**). Without it the aim pins to the endpoint and the actual
   lookahead decays to zero on arrival (bags `stg_8644443e` 12:53/12:54: L 0.452 → 0.062 m; ±17.8° yaw-setpoint
   swing during a straight hop's terminal braking; repeatable arrival walk −3.61/−2.62/−1.41/−1.35/+0.09 cm).
   Raising `min_lookahead_dist` cannot fix it (0.35→0.45 made it worse).
3. **Real corner** — never cross into the next leg. `extend_past_corner` (`segment_corner_lookahead_extend`,
   default **False**) extends along the **incoming** tangent by exactly `remaining`; guarded by `seg_rem ≤ 1e-9`
   (rover already at/past the vertex ⇒ pin to the vertex). Was "clean but inert" in the field.

## 4. Steering geometry

Body frame from NED yaw: `x_body = dn·cos(yaw) + de·sin(yaw)`, `y_body = −dn·sin(yaw) + de·cos(yaw)`
(y_body > 0 = right, FRD). `l_actual = hypot(x_body, y_body)`; `θe = atan2(y_body, x_body)`;
smooth `κ = 2·y_body/l_actual²`. Segment mode steers by the **bearing** to the lookahead point.

## 5. Pivot intercept (D1)

Pivot target = bearing from the *current* position to a point `pivot_intercept_dist_m` beyond the rover's own
projection on the leg (clamped to the leg end), so the release-heading gate nulls the lateral offset the corner
stop left instead of preserving it. Falls back to the leg direction when disabled
(`pivot_to_intercept_enabled`), `d_int ≤ 0`, degenerate geometry, or `d_to_int < max(0.05, 0.25·d_int)`.
Exactly on the leg ⇒ identical to the plain leg heading.

## 6. Evidence needed
Fixtures: recorded `(pose, path)` ticks from the 2026-07-30/08-04 bags above for projection, handover
cross-track and lookahead length (extract `/rpp/debug`, `/rpp/segment_debug`, `/rpp/conditioned_path`).
Prototype pins: `test_segment_lookahead_collinear.py` (not carried), `test_endpoint_lookahead.py`,
`test_curvature_lookahead_cap.py`, `test_segment_projection_handover.py` (carried).
