# Contract — `dyx3_rpp::stop_pivot_fsm`  (the crown jewel and the minefield)

Source: `rpp_controller_node.py` — `_corner_stop_satisfied` 5730, `_corner_brake_velocity` 5684, `_align_speed_ok` 5712,
`_reset_corner_pivot_state` 5721, `_pivot_timeout_budget` 5782, `_pivot_timed_out` 5801, `_run_alignment_hold` 3210,
`_hold_before_run_advance` 3004, `_hold_at_completion` 3070, `_segment_endpoint_precise_stop_tick` 2826,
`_precise_stop_ready` 2749, `_point_hold_tick` 2580, `_point_handshake_ready` 2521, `_stop_latch_filter` 4221,
the corner block of `_control_segment_profile` 4372–4546, `_apply_run` 2374 / `_advance_run` 2469,
`_endpoint_capture_recovered` 6007. Spec §7.4: *"ports as an explicit state machine with named states and logged transitions;
the current implicit flag-soup is what made those bugs invisible."*

## 1. Why the flags must become states

The prototype's stop/pivot logic is ~10 booleans/timestamps (`_corner_stop_entered`, `_corner_stop_settle_since`,
`_corner_stop_complete`, `_pivot_started`, `_pivot_timeout_warned`, `_pivot_turn_angle_rad`, `_align_settle_since`,
`_run_align_pending`, `_run_boundary_stop_pending`, `_completion_stop_pending`, `_segment_endpoint_stop_active`,
`_stop_latched`, `_entry_spray_hold`, point-hold keys …) reset in `_reset_corner_pivot_state()` / `_apply_run()`.
Closed bugs that landed here: dead-band limbo, double-stop / dead-entry / terminal-crawl, run-boundary miss-and-runaway,
endpoint overshoot (1.08 m run-past, bag 2026-07-10_20-07 Line_2m), square multi-run stall, the corner-extension patch that
was clean but inert, triangle-apex-2 un-pivoted entry, 54 s terminal crawl, 50 s hunting at a paint start (D4).

## 2. Proposed named states (derived from the flag combinations; **not** in the prototype)

| State | Entry condition (prototype flags) | Output |
|---|---|---|
| `TRACKING` | no hold flags set | steering from guidance |
| `BRAKE` (CORNER_STOP, stop not yet confirmed) | run-boundary/corner/endpoint/point hold active, `_corner_stop_complete = false`, `_corner_stop_satisfied() = false` | I1 brake (±body axis, capped), yaw-rate 0 |
| `STOP_CONFIRMED` | `_corner_stop_satisfied()` just became true | → `PIVOT` (or advance) |
| `PIVOT` (CORNER_ALIGN) | `_corner_stop_complete = true`, heading outside release band | pivot command, spray forced off |
| `RELEASE_SETTLE` | `heading_ok` (inside release band) | brake to settle: `heading_ok ∧ yaw_rate_ok ∧ speed_ok` for `segment_align_settle_s` |
| `RUN_ADVANCE` | settle satisfied (or collinear junction) | `_advance_run` / `_segment_idx += 1`, reset |
| `ENDPOINT_PRECISE` | final run, residual ≤ trigger, `segment_precise_endpoint_stop_enabled` | decel profile / creep, bounded by `segment_endpoint_precise_max_s` |
| `COMPLETION_HOLD` | `_completion_stop_pending` | brake to confirmed stop → `DONE` |
| `POINT_HOLD` | near an un-dwelled must-hit point (`point_hold_enabled`) | brake → dwell `point_hold_s` (or handshake) → release |
| `DONE` | `_path_done` | zero, published every tick |

Every transition must be logged with a reason (spec §7.4) and every state's output must end at zero on a failure.

## 3. Behaviour (normative)

### 3.1 Stop confirmation (`_corner_stop_satisfied`) — invariant I3
First call latches `_corner_stop_entered`. Fresh velocity (< 0.3 s old): require `|v| < segment_stop_speed_threshold` **and**
`|ω| < segment_stop_yaw_rate_threshold`, continuously for `segment_stop_dwell_s` (any violation resets the dwell; `dwell ≤ 0` ⇒
immediate). Stale velocity: after `_CORNER_STOP_MAX_HOLD_S = 2.0 s` return True (log warn); before that False. **Never** time out on fresh
data above the threshold.

### 3.2 Hard-corner execution (segment profile), at the corner within `segment_corner_acceptance_radius`
1. `path_corner_deg < segment_corner_threshold_deg` ⇒ advance immediately (collinear junction keeps momentum; only a real corner zeroes `_last_speed_cmd`).
2. else: `BRAKE` until stop confirmed (I1/I3) → `_corner_stop_complete = true`.
3. `PIVOT`: target heading = `_pivot_intercept_heading` toward the next leg; command per `rpp_motion_output.md`; speed memory `= corner_speed`
   where `corner_speed = max(0.05, segment_min_corner_speed)`.
4. Release gate: `|heading_err| ≤ min(release_tol, segment_pivot_release_max_deg)`, where `release_tol = segment_heading_tolerance_deg`, widened to
   `segment_timeout_heading_tolerance_deg` after the pivot watchdog fires; **hard cap** `segment_pivot_release_max_deg` (never launch grossly mis-headed).
   Plus `|ω| < segment_stop_yaw_rate_threshold` and measured speed `< segment_align_speed_threshold`; with **stale velocity** these two are
   replaced by "timed out" (bounded fallback — never deadlock on a missing sample). Hold the gate for `segment_align_settle_s`.
5. Once heading is inside the release band: **stop driving the pivot vector and brake** (continuing to command `corner_speed` keeps speed above the
   release threshold until the watchdog).
6. Watchdog: `budget = max(segment_pivot_spinup_margin_s + angle/segment_nominal_pivot_rate_rad_s, segment_turn_timeout_s)`, clamped to
   `segment_pivot_timeout_max_s` (> 0). Angle captured on the first call; one warning when it fires.

### 3.3 Run boundary (`_hold_before_run_advance`)
No next run ⇒ False. Next turn < threshold ⇒ `_advance_run()` immediately. Else latch `_run_boundary_stop_pending`, brake until
`_corner_stop_satisfied()`, then `_advance_run(pre_stopped=True)` which carries `_corner_stop_complete = true` into the new run so
`_run_alignment_hold` pivots directly instead of repeating CORNER_STOP.

### 3.4 `_apply_run` — per-run reset (the list the C++ must reproduce)
`_run_align_pending` (+ `_run_align_turn_rad`) set if the heading step into this run ≥ threshold, **or** (run 0 only) when
`entry_prealign_enabled` or the run starts on a MARK (then forced regardless of the parameter, turn budget π); `_entry_spray_hold = true`;
corner/pivot state reset; `_path`, `_path_s`, `_spray_flags`, must-hit rank map, profile, `_segment_idx = 0`,
`_segment_state`, `_path_done = false`, `_completion_stop_pending = false`, endpoint-stop flags, `_path_travel_m = 0`, kappa latch reset,
`_run_tail_transit_m` recomputed, `_closest_seg_hint = 0`, `_hint_valid = run.closed`.

### 3.5 Completion (`_hold_at_completion`) — D3
Latch `_completion_stop_pending`; brake (I1) until `_corner_stop_satisfied()`, then `_path_done = true`, `DONE`. **Checked before the goal test every tick**
so a coast back above `xy_goal_tolerance` cannot re-enter tracking. Origin (bag 2026-07-10_20-07 Line_2m): PX4 velocity-OFFBOARD coasts on a bare zero setpoint; the
old code published zero at `dist ≤ tol`, the rover drifted 1.08 m past, the goal check flipped false and it accelerated away.

### 3.6 Endpoint precise stop (default **on** in demo-ready)
Final run only. Residual along the final segment tangent (+ ahead, − overshot), cross-track at the endpoint, radial distance.
`trigger = v²/(2·decel) + along_tol + trigger_margin`. Negative residual engages immediately. Done when `|residual| ≤ along_tol ∧ |cross| ≤ cross_tol ∧ stopped`
→ `_hold_at_completion`; timeout `segment_endpoint_precise_max_s` accepts the best position **only when stopped**; past the timeout while still moving, the C++ **brakes**
(body-axis brake) until the shared stop confirmation holds and then finishes (XR-RPP-001, BEHAVIOUR CHANGE: the prototype kept creeping). Lateral miss outside
`segment_endpoint_max_correction_m` ⇒ brake and warn (no aggressive diagonal chase). Creep speed `segment_endpoint_creep_speed` when stopped but off the mark;
else `feedforward_brake_speed(profile_dist, decel, max(speed, creep))`. Direction: along the segment (±), or at the endpoint when a small lateral correction is needed.
Pure helpers (`precise_stop.py`): trigger `max(floor, v²/2a)`, `v = min(√(2ad), cap)`, `along_track_residual`, bang-bang `servo_speed`, `reached`.

### 3.7 Point hold (default off) and handshake (default off)
Per must-hit point: trigger radius `point_hold_acceptance_m` (or the feed-forward `v²/2a` when `point_precise_stop_enabled`), brake → confirmed stop
(precise mode also needs `|along residual| ≤ point_arrival_tolerance_m`; servo mode creeps, timeout `precise_stop_max_s` accepts best position) → dwell
`point_hold_s` **or** handshake release (`/spray/point_done` for this rank within the session; backstop `point_hold_max_s` — never wedge; manual mode additionally
waits for `/point/advance` with matching index after the proof, optional `manual_wait_timeout_s`). Each point is held once per mission (`_point_hold_done_keys`).

## 4. Required explicit-contract change (rewrite, not port)
The prototype pivots by commanding a small velocity **vector** at the exit heading (clamped ≤ 75° off the nose) because the old firmware derived heading from the
vector and froze below 1 cm/s. With `MotionSetpoint` the pivot is `MODE_PIVOT` (speed 0, explicit yaw rate); brakes are signed `speed_body_x`; the 75° clamp, the
`max(0.05, …)` pivot speed and the forward-cone logic become unnecessary but their *intent* (never turn the long way; never reverse into a 180° singularity) must be
preserved by the pivot-rate law. **GATE 1** must prove PIVOT/reverse on the bench; **GATE 4** re-validates every threshold above.

## 5. Evidence
Prototype tests carried: `test_corner_pivot.py`, `test_corner_stop_brake.py`, `test_corner_absorb_pivot.py`, `test_segment_stop.py`,
`test_completion_stop.py`, `test_segment_endpoint_precise_stop.py` (1 fails — see evidence note), `test_pivot_intercept.py`, `test_point_hold_rpp.py`,
`test_point_handshake_rpp.py`, `test_precise_stop*.py`, `test_entry_prealign.py`. **Missing for the C++**: replay of the *failure* missions (R6) — LOCAL ACTION:
extract the bags named above and replay them through the legacy node as the oracle trace.
