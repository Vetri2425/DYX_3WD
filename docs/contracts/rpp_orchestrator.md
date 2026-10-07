# dyx3_rpp orchestrator (`RppCore`) — contract

**Status:** FIRST SLICE, 2026-10-07 (cloud session). **Spec:** V1 section 7.4. Read with `rpp_overview.md` (the tick
order in the prototype) and the per-module contracts. Source: `include/dyx3_rpp/rpp_core.hpp`, `src/rpp_core.cpp`.

`RppCore::tick()` is the 50 Hz control decision: gates in front of the controller, projection, the goal test, the
smooth-profile speed/steering law and the segment-profile tracking law, and the state each tick leaves behind. It is pure C++
(no ROS, no allocation in `tick()`), reads parameters from `ParamSet` each tick (a LIVE change takes effect on the next tick),
and takes the clock as an argument (`int64` ns, so replay and tests are exact).

## 1. What this slice is, and is not

**Ported and proven tick by tick** against the verbatim prototype: `_control_loop` (measured `dt`, clamp 0.1 s),
`_control_loop_impl` up to and including the goal test, the whole smooth-profile tracking (steps 1 to 8), the segment-profile
tracking while a run is followed (lookahead, corner slowdown, run-end approach, accel ramp, P4 floor, stop latch, forward-cone
clamp), `_run_alignment_hold` / `_point_hold_tick` / `_segment_endpoint_precise_stop_tick` *entry conditions*, the tangent
corner advance (below the corner threshold), single-point runs, `_apply_run` / `_install_mission` state reset, the RTK gate
with recovery hold, pose staleness with the extrapolation horizon, velocity-based pose extrapolation with the latency bias,
the EKF jump guard with reset compensation, the spray heading gates (`_gate_spray`), `_publish_zero` semantics and the
`/rpp/debug` and `/rpp/segment_debug` rows.

**Not ported — handed off, fail to zero.** When the carried code would enter one of the stop/pivot machines below, the C++
publishes **zero** (`v = 0`, `yaw_rate = 0`, state IDLE, commanded-speed memory cleared) and reports which machine in
`TickOutput::handoff`. It never guesses a stop. These are exactly the pieces `stop_pivot_fsm` (explicit state machine, proven
against the prototype's primitives) is meant to run; wiring it in is the next slice:

| `Handoff` | The prototype enters | Condition mirrored here |
|---|---|---|
| `RunAlignment` | `_run_alignment_hold` pivot before a run | a pending alignment on a non-degenerate first leg |
| `PointHold` | `_point_hold_tick` | `point_hold_enabled` (any value of the feature is unported) |
| `RunBoundaryHold` | `_hold_before_run_advance` | end of a non-final run, or the latch |
| `CompletionHold` | `_hold_at_completion` | end of the final run, or the latch |
| `EndpointPreciseStop` | `_segment_endpoint_precise_stop_tick` engaged | final run, travel >= min, residual inside braking distance (default ON in the prototype) |
| `CornerStopPivot` | the hard-corner stop-and-pivot inside `_control_segment_profile` | not final segment, inside `segment_corner_acceptance_radius`, corner >= `segment_corner_threshold_deg` |

Also not ported: progress publication (`progress_publish_enabled`), the point handshake (`/point/done`, `/point/advance`),
precise point stop, entry pre-align pivot itself, `RppStatus` and the node. **Until the handoffs are replaced, a C++ run stops at
the first corner, the first run boundary and the end of the mission: this slice is not drivable on its own.** The precision
path on a rover remains `dyx3_rpp_legacy`.

## 2. Tick order (as implemented; prototype line references in `rpp_overview.md`)

1. `dt = clamp(now - last_tick, 0, 0.1)`; first tick after a mission install uses `1/50`.
2. No pose yet: IDLE zero. Pose age `> pose_max_age_s + (extrapolation ? imu_max_extrap_age_s : 0)`: STALE zero.
3. Extrapolate the pose by `v * (pose_age + max(0, pose_latency_bias_s))` when extrapolation is on and the velocity is younger
   than the horizon.
4. RTK gate (`require_rtk_fix`): fresh sample, `fix_type >= 6` and in {5, 6} (the prototype hard-codes the minimum 6, so
   RTK_FLOAT is rejected), accuracy known (`rtk_require_accuracy`) and `<= rtk_max_hrms_m`, then `rtk_recover_hold_s` of continuous
   acceptance. Refusal: RTK_WAIT zero with the reason in `TickOutput::rtk_reason`.
5. No path: IDLE zero. Path done: DONE zero.
6. Jump guard: jump `> max(ekf_jump_threshold_m, max(max_v, v_measured) * max(worst recent pose gap, 1/50) + 0.03)`. With
   `ekf_reset_compensation` and jump `<= ekf_reset_max_absorb_m` the jump is absorbed into the tracking offset; otherwise JUMP_SKIP zero
   and the projection hint is invalidated. JUMP_SKIP does not clear the commanded-speed memory.
7. Pending run alignment, point hold, run-boundary latch, completion latch, endpoint precise stop, goal test: hand off (section 1).
8. Smooth: projection with the windowed hint, lookahead distance / arc cap / point, steering, preview curvature, lateral
   acceleration law, approach scaling, hard-kappa latch and slew, P4 floor, feed-forward yaw rate, lateral correction, forward cone.
   Segment: segment skipping, projection onto the segment, tangent corner advance, lookahead, corner slowdown, run-end approach
   (along-run remaining distance), accel ramp, P4 floor, stop latch, forward cone.

## 3. DERIVED — NOT FROM V1 SPEC

* A handed-off tick publishes zero and **clears** `last_speed_cmd`. The prototype would have run the machine. This is the fail-to-zero
  choice for an unported path, not a port.
* `PointHold` hands off whenever `point_hold_enabled` is set, even on ticks where the prototype's overlay would not have acted:
  refusing to drive with an unported feature enabled is safer than driving without it.
* The C++ parameter table is stricter than the prototype where the prototype used `0` as "off": `curvature_baseline_m` and
  `max_yaw_rate_body` must be `> 0` here (the prototype allowed `0`). Nothing in the evidence uses `0`.
* Test hooks `mark_alignment_done()` and `test_set_last_speed_cmd()` exist only for the equivalence harness (a rover that is already
  moving; a run whose entry pivot is not under test). Production code must not call them.
* The RTK gate's minimum fix type is the prototype's hard-coded 6. `dyx3_motion_guard` has `rtk_min_fix_type` (default 6): the two
  agree today. **Open (human):** whether RTK_FLOAT should ever be accepted for marking; do not change one without the other.

## 4. Proof and its limits

`tools/gate4/gen_orchestrator_vectors.py` drives the **real carried `RPPControllerNode`** (its `__init__`, parameters, `_path_cb`,
`_control_loop`) in the Humble container with an injected clock, captured publishers and plain message objects (float32 fields
would otherwise round the controller's doubles). A closed-loop kinematic rover model consumes the controller's own command and produces
poses, velocities and GPS with noise, blackouts (pose / GPS / velocity), GPS float / poor accuracy / unknown accuracy, and an EKF jump.
All inputs and outputs are recorded; the C++ replays the **same inputs** and compares every tick: the velocity vector, yaw rate, the
16 debug values, the segment-debug row and its publish count, the handoff, and (the first 40 ticks of every episode, then every
fifth) 21 state fields.

`test/fixtures/gate4_orchestrator_vectors.txt`, 4.5 MB: **81 episodes, 8 338 ticks, 0 mismatches**; states reached: STALE, IDLE,
TRACKING, APPROACH, RTK_WAIT, JUMP_SKIP (DONE is not: nothing reaches `path_done` before a handoff); handoffs reached: all six
(`RunAlignment` x1, `CornerStopPivot` x2, `EndpointPreciseStop` x4, `RunBoundaryHold` x2, `CompletionHold` x11).

Mutation check (each applied to `rpp_core.cpp`, rebuilt, test must fail; not committed): caught — speed memory, jump absorb,
jump threshold velocity term, recovery hold, spray entry hold, stop-latch capture, along-run remaining approach, forward-cone angle,
curvature used for the accel gate, latency bias, extrapolation horizon, velocity-age horizon, pose-gap window, hint invalidation on
JUMP_SKIP, JUMP_SKIP memory exemption, GPS staleness bound, lateral-correction sign, preview-curvature lookahead and count,
approach distance, corner slowdown distance, accel gate, yaw-command freeze, hard-kappa latch input, run-out minimum speed. **Survived
(inputs do not reach them):** the 0.3 s velocity-freshness constant (only the jump threshold uses it); the `l_min` retry when the
lookahead lands on the rover (unreachable by a rover that moves along its path); the `min_travel` gate for travel in
[0.5, 1) x min (needs a path that returns to its own start); the stop-latch reference during a corner slowdown (the corner pivot
takes over before the latch distance). They are listed, not claimed.

**Not proven here:** the rover model is a crude stand-in, not recorded motion — the same test on real bag poses is a LOCAL ACTION
(needs the bags); loop timing and jitter; DDS; anything the handoffs stand for. libm differences: expected values are stored with 13
digits and compared at 1e-9 relative.

## 5. Open questions for the human

* RTK_FLOAT acceptance (above).
* Whether the stop/pivot machines should keep the prototype's behaviour bit for bit (shadow-run oracle) or take the explicit FSM's
  cleaner semantics where the two differ (`rpp_stop_pivot_fsm.md` lists them).
* `progress_publish_enabled` / point handshake: port, drop, or move to the mission layer.
