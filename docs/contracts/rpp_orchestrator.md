# dyx3_rpp orchestrator (`RppCore`) — contract

**Status:** orchestrator complete except the point hold, 2026-10-07 (cloud session). **Spec:** V1 section 7.4. Read with `rpp_overview.md` (the tick
order in the prototype) and the per-module contracts. Source: `include/dyx3_rpp/rpp_core.hpp`, `src/rpp_core.cpp`.

`RppCore::tick()` is the 50 Hz control decision: gates in front of the controller, projection, the goal test, the
smooth-profile speed/steering law and the segment-profile tracking law, and the state each tick leaves behind. It is pure C++
(no ROS, no allocation in `tick()`), reads parameters from `ParamSet` each tick (a LIVE change takes effect on the next tick),
and takes the clock as an argument (`int64` ns, so replay and tests are exact).

## 1. What is ported, what is not

**Ported and proven tick by tick** against the verbatim prototype: `_control_loop` (measured `dt`, clamp 0.1 s),
`_control_loop_impl` end to end, the whole smooth-profile tracking (steps 1 to 8), the segment-profile tracking (lookahead,
corner slowdown, run-end approach, accel ramp, P4 floor, stop latch, forward-cone clamp), and **all of the stop/pivot machinery**,
run through the explicit `stop_pivot_fsm` classes: the hard-corner stop-and-pivot with its release gates and watchdog, the
tangent-corner advance, the run-boundary hold (and the carried stop into the next run's entry pivot), the run-entry alignment
pivot, the endpoint precise stop (default ON in the prototype), the completion hold and `DONE`; also single-point runs,
`_apply_run` / `_install_mission` state reset, the RTK gate with recovery hold, pose staleness with the extrapolation horizon,
velocity-based pose extrapolation with the latency bias, the EKF jump guard with reset compensation, `_publish_zero` semantics,
and the `/rpp/debug` and `/rpp/segment_debug` rows. Phase C2 removes the legacy heading spray verdict from RPP; the planner mark diagnostic has no valve authority.

**One stop confirmation, shared.** The prototype keeps a single stop-confirmation state (`_corner_stop_entered` /
`_corner_stop_settle_since`) used by the corner flow, the run-boundary hold, the completion hold and the endpoint precise stop;
a precise stop that ends in the completion hold reuses the dwell it has already served. The C++ keeps that behaviour on purpose:
one `StopConfirm` owned by `RppCore`, referenced by `CornerFsm` and both `StopHold`s, reset exactly where the prototype calls
`_reset_corner_pivot_state()`. Two independent confirmations would add a second dwell at every completion.

**Not ported — the one remaining handoff.** `point_hold_enabled` (the per-point dwell overlay, default OFF in the prototype):
a tick with it enabled publishes **zero** (`v = 0`, `yaw_rate = 0`, commanded-speed memory cleared) and reports
`Handoff::PointHold`. Not ported at all: the point handshake (`/point/done`, `/point/advance`) and precise point stop. The ROS node, `RppStatus`, and coarse tracking progress evidence exist; point-level progress remains unported.

**Two command encodings.** The prototype speaks a NED velocity vector; the firmware-aware pivot and brake are vectors too. `TickOutput`
carries that vector (`v_n`, `v_e`, `yaw_rate`) so the tick can be compared exactly, and also the decision in the controller's own
terms (`cmd`: STOP / TRACK / BRAKE / PIVOT, `brake_speed` signed along the nose, `pivot_heading_err`) which is what the node maps to
`MotionSetpoint` modes. The two are derived from the same code path in the same tick.

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
7. Pending run alignment (pivot toward the first leg; releases into tracking in the same tick), then, in order: point hold
   (handoff), the run-boundary latch, the completion latch, the endpoint precise stop (segment profile, final run), the goal test
   (end of a non-final run: boundary hold; end of the final run: completion hold).
8. Smooth: projection with the windowed hint, lookahead distance / arc cap / point, steering, preview curvature, lateral
   acceleration law, approach scaling, hard-kappa latch and slew, P4 floor, feed-forward yaw rate, lateral correction, forward cone.
   Segment: segment skipping, projection onto the segment, the hard corner within `segment_corner_acceptance_radius` (the
   `CornerFsm`: BRAKE, PIVOT, RELEASE_SETTLE, ADVANCE; a tangent junction advances at once and keeps its momentum), lookahead, corner
   slowdown, run-end approach (along-run remaining distance), accel ramp, P4 floor, stop latch, forward cone.

## 3. DERIVED — NOT FROM V1 SPEC

* A handed-off tick (point hold only) publishes zero and **clears** `last_speed_cmd`. The prototype would have run the overlay. This is the
  fail-to-zero choice for an unported feature, not a port. `PointHold` hands off whenever `point_hold_enabled` is set, even on ticks where
  the prototype's overlay would not have acted: refusing to drive with an unported feature enabled is safer than driving without it.
* The C++ parameter table is stricter than the prototype where the prototype used `0` as "off": `curvature_baseline_m` and
  `max_yaw_rate_body` must be `> 0` here. Nothing in the evidence uses `0`.
* Test hooks `mark_alignment_done()` and `test_set_last_speed_cmd()` exist only for the equivalence harness (a rover that is already
  moving; a run whose entry pivot is not under test). Production code must not call them.
* The RTK gate's minimum fix type is the prototype's hard-coded 6. `dyx3_motion_guard` has `rtk_min_fix_type` (default 6): the two
  agree today. **Open (human):** whether RTK_FLOAT should ever be accepted for marking; do not change one without the other.
* `StopPivotParams::corner_speed` is carried for the FSM's own output but the core derives the pivot speed itself
  (`max(0.05, segment_min_corner_speed)`), as the prototype does; changing the FSM field alone changes nothing.
* The pivot and brake still go out as the prototype's velocity vectors (small vector at the exit heading, clamped 75 degrees off the nose;
  body-axis brake). The `MotionSetpoint` rewrite (explicit `MODE_PIVOT`, signed brake) is `motion_output`'s and is selected from `cmd`;
  whether the pivot rate law is acceptable on the new firmware is GATE 1 (`rpp_motion_output.md` section 4).

## 4. Proof and its limits

`tools/gate4/gen_orchestrator_vectors.py` drives the **real carried `RPPControllerNode`** (its `__init__`, parameters, `_path_cb`,
`_control_loop`) in the Humble container with an injected clock, captured publishers and plain message objects (float32 fields
would otherwise round the controller's doubles). A closed-loop kinematic rover model consumes the controller's own command and produces
poses, velocities and GPS with noise, blackouts (pose / GPS / velocity), GPS float / poor accuracy / unknown accuracy, and an EKF jump.
All inputs and outputs are recorded; the C++ replays the **same inputs** and compares every tick: the velocity vector, yaw rate, the
16 debug values, the segment-debug row and its publish count, the handoff, and (the first 40 ticks of every episode, then every
fifth) 26 state fields. Episodes run to `DONE` (or the tick budget).

`test/fixtures/gate4_orchestrator_vectors_{1,2,3}.txt` (each under the repository's 5 MB limit):
**101 episodes, 13 905 ticks, 0 mismatches** outside the documented deviations, which the test pins by scenario, tick window and
count: `seg_square_nohold_vel` ticks 85-86 (10 values), XR-RPP-011 (`rpp_stop_pivot_fsm.md` section 3.1). States reached: STALE, IDLE, TRACKING, APPROACH, DONE, RTK_WAIT, JUMP_SKIP; segment states:
TRACK, PRE_CORNER, CORNER_ALIGN, DONE, CORNER_STOP; commands STOP / TRACK / BRAKE / PIVOT (about 740 brake and 830 pivot ticks).
Episodes include a full square (four hard corners), entry alignments (large, small, stale velocity, smooth profile, the
`entry_prealign_enabled` pivot), two runs with a hard boundary, the endpoint precise stop with several parameter sets, and the
completion hold.

Mutation check (each applied to `rpp_core.cpp`, rebuilt, test must fail; not committed): about 45 mutations, **caught** — speed memory,
jump absorb, jump threshold velocity term, recovery hold, stop-latch capture, along-run remaining approach,
forward-cone angle, curvature used for the accel gate, latency bias, extrapolation horizon, velocity-age horizon, pose-gap window,
hint invalidation on JUMP_SKIP, JUMP_SKIP memory exemption, GPS staleness bound, lateral-correction sign, preview-curvature lookahead
and count, approach distance, corner slowdown distance, accel gate, yaw-command freeze, hard-kappa latch input, run-out minimum speed,
the carried stop into the entry pivot, the entry pivot's watchdog angle, the shared stop dwell at completion, the precise-stop trigger
margin, creep rule, profile distance, the brake sign, the pivot release cap, the pivot speed floor at an entry pivot.
**Phase F closure (2026-10-08):** Four additional episodes were generated by the carried-node workflow
(`tools/gate4/gen_orchestrator_vectors.py --check` reproduces the committed fixtures byte for byte). Each
passes against the unmodified C++ implementation (baseline). Under a temporary mutation of `rpp_core.cpp`
(never committed; replay with `DYX3_ORCH_SCENARIO=<name>`), **three** of them fail as stated below and are
mutation-closed; **one (`fault_vel_blackout_jump`) is baseline coverage only** because its claimed mutation
was not reproduced on independent replay.

| Episode | Reference-backed baseline | Temporary mutation and first failure |
|---|---|---|
| `seg_boundary_55deg_sharp` | 2 conditioned runs, 330 ticks, 0 mismatches | Add 15° to the run-alignment threshold (45° → 60°) at `rpp_core.cpp` `segment_corner_threshold_deg` use in the run-alignment decision (`:115`, `:461`): first mismatch tick 123 on independent replay (113 as first reported). The `path_conditioner.cpp` threshold is not killed. The prior `seg_boundary_55deg` forced the segment profile, which makes the carried conditioner keep the whole path in one run. The new episode uses the supported `auto` profile and zero corner smoothing to retain the 54.96° run boundary. |
| `seg_start_beyond_end` | 100 ticks, 0 mismatches; reference commands reverse correction | Ignore the negative residual when choosing endpoint correction direction: tick 6, expected −0.10 m/s, got +0.10 m/s. |
| `seg_corner_direct_release` | 199 ticks, 0 mismatches; `segment_align_settle_s=0` with permissive stop/alignment gates | Omit the settled-corner speed-memory reset: tick 50, expected zero velocity/speed memory, got 0.12 m/s. |
| `fault_vel_blackout_jump` | 110 ticks, 0 mismatches (baseline coverage only) | **NOT CLOSED.** The reported 0.3 s → 0.5 s velocity-freshness mutation was not reproduced on independent replay: changing `vel_is_fresh` (0.05–100 s), the jump-threshold use at `rpp_core.cpp:768`, and the IMU-extrapolation age at `:791` all give 0 mismatches, i.e. the velocity-freshness constant is still a survivor. |

**Other survivors (the inputs do not reach them):**
* the 0.3 s velocity-freshness constant in `vel_is_fresh` (re-opened: no episode reaches it, see `fault_vel_blackout_jump`);
* the `l_min` retry when the lookahead lands on the rover (unreachable by a rover that moves along its path);
* the `min_travel` gate for travel in [0.5, 1) x min (needs a path that returns to its own start);
* the stop-latch reference during a corner slowdown (the corner pivot takes over before the latch distance);
* `StopPivotParams::corner_speed` (a dead field, see section 3).
They are listed, not claimed.

**Not proven here:** the rover model is a crude stand-in, not recorded motion; the same test on real bag poses is a LOCAL ACTION (needs
the bags). Loop timing and jitter; DDS; the firmware's reaction to the pivot vector and the brake vector. libm differences: expected
values are stored with 13 digits and compared at 1e-9 relative. A mission that starts in the middle of a multi-vertex segment run (the
episodes that start part-way along a run) is not a case the prototype supports either: the segment index only advances at a vertex.

## 5. Open questions for the human

* RTK_FLOAT acceptance (above).
* The explicit FSM reproduced the prototype's behaviour on every tick of the fixture; where the contract proposed cleaner semantics
  (`rpp_stop_pivot_fsm.md`) they were NOT adopted, because the equivalence is the Gate 7 oracle. Decide before changing any.
* `progress_publish_enabled` / point handshake: port, drop, or move to the mission layer.

## Input hardening (RPP-002 / RPP-004 / GEO-002, production, not in the prototype)
`on_pose` / `on_velocity` drop a sample with any non-finite field (the previous one ages out). A pose age that is negative or non-finite is
STALE (STOP). Extrapolation uses the velocity only when it is finite and its age is in `[0, imu_max_extrap_age_s)`. A projection with
`valid == false` (geometry may return it for a degenerate window) publishes zero (IDLE) and drops the hint. Behaviour-neutral for valid
input: the equivalence vectors are unchanged.
