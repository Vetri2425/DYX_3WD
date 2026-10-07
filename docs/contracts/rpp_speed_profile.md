# Contract — `dyx3_rpp::speed_profile`

Source: `rpp_controller_node.py` — `_alignment_accel_scale` 1601, `_update_kappa_hard_latch` 1642,
`_apply_smooth_speed_slew` 1658, speed sections of `_control_segment_profile` (4548–4705) and
`_control_loop_impl` (5324–5470), `_stop_latch_filter` 4221, `_corner_brake_velocity` 5684, `_align_speed_ok` 5712.
Pure functions: `_alignment_accel_scale`, `_update_kappa_hard_latch`, `_apply_smooth_speed_slew`.

## 1. Effective ceiling and derived distances

`max_v = min(max_linear_vel, mission_speed)` (hardware ceiling vs per-job operator knob).
`approach_d = max(approach_velocity_scaling_dist, max_v²/(2·max_linear_decel) + 0.10)` — the configured value is a **floor**.

## 2. Smooth profile speed law (P4.1, P1.1)

* `kappa_speed` = worst preview curvature: `_max_preview_curvature` samples three points per preview
  `k = 1..N` at `(k−½)L_d, kL_d, (k+½)L_d` (first sample ≥ 0.05 m), Menger curvature each; stops when two samples
  both ran off the path end. `N = max(preview_curvature_n, ceil(preview_curvature_distance_m / L_d))`; `N ≤ 1`
  ⇒ `|κ|` at the lookahead. Path-intrinsic (independent of pose).
* Lateral-acceleration limit: `v_lat = sqrt(a_lat_max / kappa_speed)`;
  `speed = clamp(min(max_v, v_lat), regulated_linear_scaling_min_speed, max_v)`; `kappa_speed ≈ 0 ⇒ max_v`.
* Approach scaling (smooth, open run): if `dist_to_goal < approach_d` **and** `path_travel ≥ approach_d`:
  `speed = min(speed, max(min_approach_linear_velocity, speed·dist/approach_d))` → state `APPROACH`.
  **Closed run** (circle): scale on `run_length − path_travel`, not Euclidean distance (field bag 20260613_200921:
  Euclidean distance ≈ 0 near the seam crept at ~3 cm/s for 118 s and never traced the loop).

## 3. Segment profile speed law

Start `speed = max_v`. Then, in order:
1. **Pre-corner slowdown** — non-final segment, `segment_slowdown_dist > 0`, `dist_to_corner < slowdown`, corner angle
   ≥ `segment_corner_threshold_deg`: `speed = max(segment_min_corner_speed, max_v·dist/slowdown)`.
2. **Run-end approach** — `approach_ref = dist_to_corner` on the final segment, else ∞; if
   `endpoint_approach_run_remaining` then `approach_ref = min(approach_ref, remaining_along_run)` (monotonic).
   If `approach_ref < approach_d`: `speed = min(speed, max(segment_endpoint_approach_speed, max_v·approach_ref/approach_d))`.
   *History (2026-08-01 bag `stg_9ecf2985`)*: measured against the final **segment** only, a fused 0.1 m run-out clipped
   the 0.9 m ramp to 0.1 m — entered at 0.79 m/s and stopped 54.6 cm past the goal. Fix 3: if the active run's tail is an
   unpainted run-out, `0 < speed < transit_runout_min_speed_m_s` and still outside the relaxed tolerance, command
   `transit_runout_min_speed_m_s` (sub-dead-band crawls are unactuatable: PX4 `RO_SPEED_TH` ≈ 0.1 m/s; 54 s spent closing 8 cm).
3. **Accel up** — only when `speed > last_speed_cmd`: `speed = min(speed, last + max_linear_accel·accel_scale·dt)`.
   `accel_scale = min(heading_scale, curvature_scale)` — each linear between its *full* and *none* limits
   (`accel_gate_heading_full/none_deg`, `accel_gate_curv_full/none`); both must settle before full acceleration;
   never impedes deceleration. Segment mode passes curvature 0.
4. **P4 floor** — `if speed < p4_zero_vel_threshold and speed_before_accel < p4_zero_vel_threshold and last_speed_cmd > 0: speed = 0`.
   Applies only when the *intended* target is below the floor: during normal ramp-up the accel-limited value can be below the
   floor for a few cycles, and zeroing it creates a permanent `0 → delta → 0` deadlock. Invariant: `p4_zero_vel_threshold <
   min_approach_linear_velocity`, else the rover hard-zeros throughout the approach (checked at boot, warning only).
5. **Stop latch** (default off, `stop_latch_enabled`) — applied **after** the accel ramp. Enforces `cmd ∈ {0} ∪ [min_actuatable, vmax]`:
   unlatched & `0<speed<th`: within capture distance ⇒ latch + hard 0, else command `th`; latched: relaunch at `≥th` if the stop is
   `> release` away, one actuated nudge if short of capture and measured stationary, else hold 0. Reference distance = corner
   (when PRE_CORNER active) or run end on the final segment — never a collinear pass-through vertex.

## 4. Smooth slew (Patch 2) — `_apply_smooth_speed_slew`

Returns `(speed, mode)`: `speed_raw > last` ⇒ accel-limited, mode 0. Else approach-active **or** `speed_raw < p4_floor` ⇒ immediate,
mode 3. Else hard-κ latched ⇒ immediate, mode 2. Else `speed_cmd_decel_m_s2 > 0` ⇒ `max(raw, last − decel·dt)`, mode 1; else immediate, mode 2.
Hysteresis latch `_update_kappa_hard_latch`: enter at `|κ| ≥ kappa_hard_enter`, leave at `≤ kappa_hard_exit`; if `enter ≤ exit` ⇒ stateless
`|κ| ≥ enter`. Reset on mission/run/idle/abort/stop. Purpose: flicker in `v_lat_limit` stepped 0.45↔0.89 on sweeps.

## 5. Time base

`dt` measured from the node clock, clamped `[0, 0.1]`; first tick after an install uses `1/CONTROL_HZ`. The C++ port must use the
same clock domain (`use_sim_time`-aware) and record loop jitter (RppStatus).

## 6. Defects / open
`D3` terminal deceleration begins ~0.4 m before the paint end when `approach_velocity_scaling_dist` (0.9 m floor) exceeds the aft
extension (wiggle −2…−3.7 cm at 3.76–3.80 m while spraying). `D7` 0.8 m/s untested.
The prototype's own tests `test_speed_cmd_slew.py`, `test_terminal_approach_run_remaining.py` pin parts of this; **6 of the latter fail on
the `demo-ready` defaults** (see `rpp_legacy_evidence.md`) — the expected values assume the older retuned set.
