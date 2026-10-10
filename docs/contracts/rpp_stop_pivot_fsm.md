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
**C++ deviation (XR-RPP-011, BEHAVIOUR CHANGE):** the prototype counts the 2.0 s from the hold entry, so after 2 s of braking on a fresh
velocity a single stale tick confirmed the stop on a frozen measurement. The C++ counts it from the first stale tick of the hold (a fresh
sample restarts the count). `gate4_equivalence_test` applies exactly that rule to the ancestor's STOP sequences and counts the ticks it
changes (2199 of 10093); `orchestrator_equivalence_test` pins one documented window (`seg_square_nohold_vel` ticks 85-86, the stop is
confirmed one tick later).

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
7. **Exported timeout (interfaces 0.17.0, not in the prototype).** `CornerOutput.pivot_timed_out` is true only on a `Pivot` output with the
   watchdog expired, i.e. the heading is outside even the widened release band; false on `Brake`, `SettleBrake` and `Advance`. The core
   carries it from both pivot sites (the corner pivot in the segment profile and the run-entry alignment pivot, the only two callers of
   `CornerFsm::step`) into `TickOutput.pivot_timed_out`, and the node into `RppStatus.pivot_timed_out` (only while `STATE_PIVOTING`).
   What the FSM does on a timeout is **unchanged**: the release band widens to `segment_timeout_heading_tolerance_deg` (capped by
   `segment_pivot_release_max_deg`) and the pivot continues. RPP does not stop on it: **`dyx3_mission` pauses the mission**
   (`MissionState.REASON_RPP_PIVOT_TIMEOUT`); the pause resets the corner state (`pause()`), so a resume re-confirms the stop and starts a
   fresh watchdog. Origin: 2026-10-10, 25.8 s in PIVOT with an 86° heading error, every gate green, nothing reported.
   Tests: `rpp_modules_test` `CornerFsm.PivotTimedOutIsReportedOnlyWhilePivotingPastTheWatchdog`, `rpp_core_test`
   `ACornerPivotThatNeverTurnsReportsTheTimeoutUntilReleased`, `AStartAtRunNBeginsWithTheEntryAlignmentOfRunN`.

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

**Start at run N (`MissionState.start_run_index`, interfaces 0.17.0, not in the prototype).** `RppCore::start_at_run(N)` after
`install_mission` is `_apply_run(N, pre_stopped=True)`: the state section 3.3 leaves behind when run N is reached after a confirmed stop at a
hard run boundary (alignment pending if the boundary turn is at least the threshold, `_corner_stop_complete` carried, so the first tick
pivots directly). DERIVED — NOT FROM V1 SPEC: the entry pose of a resumed run is unknown, so `_run_align_turn_rad` is π (the run-0 worst
case) instead of the boundary turn; with the boundary budget a correct 163° alignment (about 7 s at the default pivot rate) would be
reported as a pivot timeout after 5.0 s. An index past the last run is refused (the node: STOP + ERROR). Open: a resumed run whose boundary
turn is below the threshold gets no alignment at all, exactly as at a collinear boundary, although the rover may stand at any heading.

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

**BEHAVIOUR CHANGE, not in the prototype — dead band and brake hold (2026-10-10).** Evidence: mission 0001 run 3 (recorder bag),
final-run endpoint: 14 forward→reverse command reversals in 8.6 s, commanded speed alternating +0.07 / −0.085 m/s while `dist_to_goal` was 0.000–0.010 m,
measured speed up to 0.125 m/s, finished only by the `segment_endpoint_precise_max_s` (8 s) timeout brake, 7 mm from the point. Mechanism: finishing needs
`|residual| ≤ along_tol ∧ |cross| ≤ cross_tol ∧ stopped`, but while not yet `stopped` the prototype law commanded `min(√(2·decel·|residual|), cap)` toward
residual = 0 for **any** non-zero residual (0.084 m/s at 1 cm, 0.046 m/s at 3 mm) with a sign flip at the end plane, and `cap = max(speed, creep)` rose with the
rocking. A vehicle with speed-loop / drivetrain latency (0.1–0.3 s) overshoots the plane, the sign flips, and "stopped" (speed < `segment_stop_speed_threshold`
for `segment_stop_dwell_s`) is never reached. The C++ therefore differs from the prototype as follows (no new parameter):
1. **Dead band = the arrival band.** When `|residual| ≤ along_tol ∧ |cross| ≤ cross_tol` and the stop is not yet confirmed, the command is the **brake**
   (`CmdKind::Brake`, body-axis velocity-reversal brake, heading held — the same path as the timeout brake), not a creep. The stop confirmation can then be
   reached and the endpoint finishes on the first tick that is `stopped`, as before.
2. **Brake-hold hysteresis.** Once braking inside the band the brake is held while `|residual| ≤ along_tol + endpoint_capture_past_m ∧ |cross| ≤ cross_tol`
   (a few millimetres of coast must not abandon it). Beyond that band (a real overshoot) the latch is released and the creep law gives the reverse correction.
   The latch is also released when the rover is `stopped` outside the finish geometry, so the `stopped ∧ radial > band → creep` nudge is unchanged.
   *DERIVED — NOT FROM V1 SPEC:* `endpoint_capture_past_m` (the prototype's "capture past" allowance, default 0.02 m) is reused as the hold band; no new number.
   The latch is `RppCore::endpoint_brake_hold_`, reset with `segment_endpoint_stop_active_` (mission install, per-run reset, pause/resume) and on engage / finish.
3. **Feed-forward to the plane (the prototype law, kept).** Outside the band the creep law is the prototype's
   `feedforward_brake_speed(max(0, profile_dist), decel, cap)`, evaluated to the end plane (radial distance for the lateral correction).
   A band-edge variant (`max(0, profile_dist − along_tol)`, briefly in `fb74658`) was taken back after review (2026-10-10): by
   construction it aims short of the point, and the goal is 1 cm.

   **Stop-position distribution** (`rpp_core_test` `TheEndpointStopPositionDistributionOnAFirstOrderVehicle`; closed loop on a
   first-order vehicle, 50 Hz, approach from 0.6 m at 0.45 m/s, default parameters; residual at rest, + short of the plane, − past it;
   reversals = forward/reverse sign changes of the command above 2 cm/s; complete = time to the completion latch). Both variants keep the
   in-band brake and the brake hold; they differ only in the feed-forward outside the band.

   | lag (time constant + command latency) | band edge: residual | reversals | complete | **plane (ships)**: residual | reversals | complete |
   |---|---|---|---|---|---|---|
   | 0.1 s | +16.4 mm | 1 | 3.66 s | +13.6 mm | 1 | 3.64 s |
   | 0.2 s | +6.8 mm | 1 | 3.44 s | +1.0 mm | 1 | 3.44 s |
   | 0.3 s | −6.6 mm | 1 | 3.42 s | −15.0 mm | 1 | 3.44 s |
   | 0.1 s + 0.1 s | +7.6 mm | 2 | 3.56 s | −0.1 mm | 2 | 3.58 s |
   | 0.2 s + 0.1 s | −5.8 mm | 1 | 3.28 s | −16.7 mm | 1 | 3.32 s |
   | 0.3 s + 0.1 s | −10.7 mm | 2 | 4.06 s | −4.7 mm | 2 | 4.32 s |
   | mean / max abs / rms | +1.3 / 16.4 / 9.7 mm | | | −3.6 / 16.7 / 10.9 mm | | |

   Asserted for the shipped law, every lag: complete within 5 s, at most 2 reversals, |residual| ≤ `along_tol` (0.02 m); the mean residual
   over the sweep within 1 cm of the plane. Reading: in this model neither variant is biased short; the ±1.7 cm spread comes from the
   in-band brake and the coast behind the lag, not from where the feed-forward reaches zero. The 1 cm goal is therefore not met by either
   law on a 0.1–0.3 s drivetrain; what decides it is the brake/coast at the band, to be measured on the rover (Gate 4 / field ladder).

Unchanged: the trigger, negative-residual engagement, the feed-forward outside the band, the `stopped ∧ radial > band → creep` nudge, the lateral-miss brake
(`radial > segment_endpoint_max_correction_m`), the XR-RPP-001 timeout brake (kept as the backstop), `hold_at_completion`, the debug rows and `cross_track_right`.
The orchestrator equivalence test lists the affected prototype ticks as documented deviations: every mismatching tick is a C++ in-band brake tick (247 values on
98 ticks, in scenarios `seg_line_fast_tail` 107–145, `seg_line_tail` 134–149, `seg_overshoot` 6–18, `seg_precise_offline` 213–229, `seg_precise_params` 99–103,
`seg_runout_precise` 114–126; `auto_mixed` no longer deviates), plus the 10 of XR-RPP-011: 257 in total (525 with the band-edge feed-forward).
Tests: `rpp_core_test.cpp` (`TheEndpointDeadBand*`, `TheEndpointBrakeIsHeld*`, `AnEndpointStoppedOffTheMark*`, `TheEndpointFeedForwardOutsideTheBandIsEvaluatedToThePlane`,
`TheEndpointStopPositionDistributionOnAFirstOrderVehicle`). Field re-validation of the 2 cm band at the rover's real stop behaviour is still owed (Gate 4 / field ladder).

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
