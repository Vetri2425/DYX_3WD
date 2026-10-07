# Contract — `dyx3_rpp::terminal`

Source: `rpp_controller_node.py` — `_is_closed_run` 3153, `_run_min_travel` 3172, `_path_progress_at` 3196,
`_update_path_progress` 3206, `_measure_tail_transit_m` 5914, `_run_tail_is_transit` 5937, `_run_remaining_along` 5952,
`_goal_tol_effective` 5976, `_endpoint_capture_recovered` 6007, goal checks in `_control_loop_impl` 5141–5200.

## 1. Run-level quantities

* `closed` run: `len ≥ 3`, length ≥ `close_loop_min_len_m`, `|first − last| ≤ close_loop_threshold_m`. Euclidean distance-to-goal is ≈ 0 from the start, so
  completion needs the circumference.
* `_run_min_travel`: **closed** ⇒ `closed_loop_min_travel_frac · length`; **open** ⇒ `min(min_goal_travel_m, 0.5·length)` (the cap keeps a short hop from
  deadlocking). A run may not declare DONE / advance before `path_travel ≥ min_travel` (prevents instant-DONE when the rover starts at the goal, e.g. a closed square).
* `_run_remaining_along = max(0, length − path_travel)`, **monotonic** (immune to the closed-seam and out-and-back traps; saturates at 0 on overshoot). Returns *None* when the
  progress cache is unusable (no run, zero length, `cum_s` out of step) — callers must then fall back to their previous per-segment measure; a broken cache must never remove
  braking that existed.
* Tail transit: `_measure_tail_transit_m` = along-path length from the last painted point to the run end, or 0 if the run does not end unpainted **or paints nowhere**
  (a transit/approach leg is not a run-out). History: treating "ends unpainted" as run-out gave the approach leg to the mission start the relaxed 10 cm tolerance and it stopped
  up to 10 cm SHORT of paint start — arrival degraded 1.4 cm → 7.4–8.5 cm (three b48df83 runs).

## 2. Goal tolerance (`_goal_tol_effective`)
`goal_tol` (`xy_goal_tolerance`, 0.02 m) unless the run has an unpainted tail: then `max(goal_tol, min(transit_runout_goal_tolerance_m, 0.5·tail))`.
Capped at **half the tail** so it self-scales: a tolerance equal to the 0.1 m standard run-out meant the run-out could never be entered — DONE latched at the mark end and the
rover parked on the wet paint end (measured +0.0/+0.7/+1.1 cm from paint end, 9–10 cm short of goal). **D4**: the tolerance is global; 0.01 deadlocks run boundaries (50 s hunting at a
paint start, bag 200257) — a per-leg tolerance is a known wish, not implemented.

## 3. Endpoint capture recovery (`_endpoint_capture_recovered`, default **on**)
A second, bounded arrival test for "drove THROUGH the run end without entering the ball". All required: (1) `remaining_along ≤ goal_tol_eff` and not None;
(2) the rover is past the end plane by more than `endpoint_capture_past_m` along the final segment tangent; (3) perpendicular residual ≤ `endpoint_capture_max_miss_m`
(logged error and **refuse** otherwise — never accept a wide miss). Euclidean `dist_to_goal` is the wrong quantity once past the plane (it grows with the along-track term and would revoke
the recovery exactly as the rover runs away).

## 4. Goal-time choreography
At goal: `run_idx + 1 < len(runs)` ⇒ `_hold_before_run_advance` (stop first if a hard corner), else `_hold_at_completion` (brake to confirmed stop → `DONE`). See `rpp_stop_pivot_fsm.md`.
The C++ must keep `_completion_stop_pending` checked **before** the goal test.

## 5. Known defects
`C7` obsolete (endpoint architecture rebuilt: paint end 0.1–1.7 cm, rest deliberately in the run-out). `D2` approach-stop scatter 2–4 cm. `D3` see `rpp_speed_profile.md`.
`C9` closed-loop projection snap-back.
