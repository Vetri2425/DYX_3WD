# dyx3_rpp node (`RppNode`) — contract

**Status:** 2026-10-07 (cloud session); 2026-10-10 interfaces 0.17.0 (start run, pivot timeout). **Spec:** V1 section 7.4. The decision lives in `RppCore` (`rpp_orchestrator.md`); this node is the ROS wiring only.
**Authority:** none beyond the path-following decision. It publishes no safety verdict; `dyx3_motion_guard` is the last authority before PX4.

## 1. Interfaces

| Direction | Topic | Type | QoS | Notes |
|---|---|---|---|---|
| in | `/dyx3/vehicle_state` | VehicleState | R1 | pose is fed only while `position_valid` and `attitude_valid` and north, east and heading are finite; velocity while `velocity_valid` and both components and the yaw rate are finite (RPP-002; the core repeats the check). An invalid sample is **not** fed: the pose ages out and the core stops (STALE) |
| in | `/dyx3/rtk_status` | RtkStatus | R1 | `fix_type` and `horizontal_accuracy_m` (0 = unknown, passed as unknown) |
| in | `/dyx3/mission/state` | MissionState | R1 | `path_artifact_sha256` + `mission_id` select the path; `state == RUNNING` is the only state in which the core ticks; `start_run_index` (0.17.0) the run the core begins at (section 2, "Start run") |
| out | `/dyx3/rpp/motion_setpoint` | MotionSetpoint | R1 | every tick (per pose sample, or watchdog; section 2), always `valid`; STOP unless running. `source_pose_sample_stamp` (IF-003, 0.14.0) = `VehicleState.px4_sample_stamp` of the newest pose fed into the core, on every command including STOP; zero until a valid pose arrived (latency evidence, never gating) |
| out | `/dyx3/rpp/status` | RppStatus | R1 | every tick; `mission_id` is the acknowledgement `dyx3_mission` waits for; `run_index` the run the core is on (the start run from the load on); `pivot_timed_out` (0.17.0) section 2, "Pivot timeout" |

## 2. Behaviour

* **When the core ticks (C2, `event_driven`).** Default: a `VehicleState` that carries a **new** `px4_sample_stamp` (non-zero and different
  from the last one that ticked) is fed exactly as before and then ticks the core **in the same callback**, at the same clock instant, so the
  command leaves with the pose that caused it (pose age ≈ 0 at the tick instead of up to one 20 ms timer period). A repeated or unstamped
  sample is fed but does not tick. The `tick_hz` timer becomes a watchdog and is reset by every sample tick: it ticks only after **two
  periods** without a sample tick (a sample late by less than one period, PX4 cadence jitter, is not silence and must not cause a second tick
  for one sample), then once per period while samples stay away. Silence is therefore still ticked at `tick_hz`, so the stale-pose STOP lands
  on the same `pose_max_age_s` deadline as in timer mode (tested: 0.52 s at 50 Hz in both modes). A sample arriving right after a watchdog
  tick still ticks (the pose must not wait). The core, its inputs and its tick semantics are unchanged: the same calls in the same order with
  the same times give the same outputs (gate4 / orchestrator equivalence drive the core directly). `tick_dt` is the time since the previous
  tick, as before. `loop_jitter_*` / `loop_overrun_count` then measure the tick interval, i.e. the sample cadence, against `1/tick_hz`.
  RPP and the other callbacks share the node's single default callback group on the single-threaded executor (`main.cpp`): no lock is needed.
  `event_driven=false`: the timer ticks at `tick_hz` whatever arrives (the behaviour before 0.14.0).
  DERIVED — NOT FROM V1 SPEC: the 1.5-period / 0.5-period watchdog thresholds (structural fractions of `1/tick_hz`, not tuning values).

* **Load by id.** On a `MissionState` in LOADING / READY / RUNNING / PAUSED with a non-empty hash, the node reads `<artifact_dir>/<sha>.dyx3path`
  with the same reader `dyx3_mission` uses (the hash is verified), turns the points into runs with `condition_path` (the proven port of `_path_cb`),
  and installs those exact runs in `RppCore`. It writes an immutable `DYX3COND 1` artifact from the installed runs and publishes its content hash in
  `RppStatus.conditioned_execution_sha256`. Spray uses that artifact for projection. A **new mission id** loads again even for the same source file.
  A missing or corrupt file, or a path that conditions to nothing, is `STATE_ERROR` (STOP) and is retried once a second. Filesystem
  calls use the `error_code` overloads and any exception inside the load is a failed load (XR-RPP-010); `main` sends its bounded STOP
  burst even when spinning ended with an exception.
* **Start run (`MissionState.start_run_index`, 0.17.0).** A resumed execution (`StartMission.resume`, `dyx3_mission`) names the first
  run not yet completed. The node passes the `start_run_index` of the message that triggers the load to the load: after conditioning,
  an index that is not a run of the conditioned path (`>= runs`) is **refused**: STOP, `STATE_ERROR`, one error line, no conditioned
  artifact, and **no retry** (the same artifact conditions to the same runs; the conditioning parameters are IDLE_ONLY while a mission is
  loaded), until a new mission id loads. A valid index is installed with `RppCore::install_mission` then `RppCore::start_at_run(index)`,
  which is `apply_run(index, pre_stopped = true)`: run N begins in exactly the state the run-boundary hold leaves behind when run N is
  reached after the stop at a hard run boundary (entry alignment pending when the boundary turn is at least
  `segment_corner_threshold_deg`, stop already confirmed, so the first RUNNING tick is the alignment pivot of run N; the per-run reset
  of run N). One difference: the entry pose of a resumed run is unknown, so the alignment watchdog budgets the worst case (pi), as run 0
  does, instead of the boundary turn (`rpp_stop_pivot_fsm.md` section 3.4). Index 0 is the plain start. The index is **latched at the
  load**; the message that carries the RUNNING transition may still set it (`start_at_run` again) as long as no RUNNING tick of this load
  has run; any other change while loaded is ignored with one warning. `RppStatus.run_index` reports the start run from the load on
  (LOADED included), so the first RUNNING status already shows it, and no command is ever computed for a run before it.
  DERIVED — NOT FROM V1 SPEC: the pre-stopped start (the mission arms and engages before RUNNING, so the rover stands still), the
  worst-case budget, the no-retry refusal and the RUNNING-transition window.
* **Pivot timeout (`RppStatus.pivot_timed_out`, 0.17.0).** True on a tick on which the core pivots (`STATE_PIVOTING`; the corner pivot
  or the run-entry alignment pivot) with the pivot watchdog expired and the heading still outside the (widened) release band; false on
  every other tick and whenever the core is not ticked (LOADED, paused, ERROR, COMPLETE). Origin: 2026-10-10, 25.8 s in PIVOT with an 86°
  heading error, every gate green, and no signal. The core keeps pivoting and keeps the band widening (`rpp_stop_pivot_fsm.md` section
  3.2); RPP decides nothing more: `dyx3_mission` pauses the mission on it (`REASON_RPP_PIVOT_TIMEOUT`). The rising edge is logged once.
* **Not RUNNING** (loaded and waiting, or paused): STOP every tick, `STATE_LOADED`. On the RUNNING to not-RUNNING edge the core forgets its motion
  memory (`pause()`: speed memory, hard-curvature latch, stop confirmation; and, XR-RPP-008, the jump-guard position, the tick period,
  the projection hint of an open run, the precise-stop engagement and its timer, the stop latch: a resume is a fresh start of the same run, so a
  coast while paused is neither a JumpSkip nor an EKF offset) so a resume ramps from rest. A mission is never resumed by RPP on its own: that is `dyx3_mission`'s FSM.
* **RUNNING:** one `RppCore::tick` per control tick, mapped by `command_from_tick` (`rpp_command.hpp`):
  STOP -> `MODE_STOP`; TRACK -> `TRACK_RATE` (smooth run: speed = |v|, rate = feed-forward + feedback) or `TRACK_HEADING` (segment run: speed = |v|,
  heading = the bearing of the core's velocity vector whenever it is non-zero, the core's frozen heading only for an exactly zero vector, so a stop never
  snaps to North and the slow first ticks after a pivot never carry the previous leg's heading (XR-RPP-007)); BRAKE -> signed speed along the nose with the nose held (reverse is a
  negative speed, never a spot turn; permission is the guard's); PIVOT -> `MODE_PIVOT` with `omega = clamp(1.5 * err, +/- max_yaw_rate_body)`; CREEP -> `MODE_CREEP` along the nose, or `TRACK_HEADING` toward the precise stop's correction vector when it is more than 2° off the nose axis
  (forward ahead of the beam, reverse behind it; XR-RPP-001, `rpp_motion_output.md` section 2).
  Any non-finite value becomes the canonical STOP.
  A running tick on which the core publishes nothing (`velocity_published == false`: a run handover that needs no alignment) repeats the previous
  running tick's command and state **once** instead of the default STOP (XR-RPP-002); a second consecutive silent tick, or one with no previous
  running command, is STOP.
* **State reporting** (`RppStatus.state`): TRACKING only while actually tracking (tick state TRACKING or APPROACH) — `dyx3_spray` reads TRACKING as the
  "mission has started" evidence and PIVOTING as the pivot gate, so STOPPING (brake, corner stop, a gate refusal), CREEPING (precise stop) and COMPLETE must never be
  reported as TRACKING; LOADED while waiting; ERROR when the artifact is bad or an unported feature is enabled; COMPLETE when the core finished.
  `cross_track_right_m` is right-positive everywhere (`frames.md`), including the endpoint precise stop, whose legacy debug value is left-positive
  and is negated for the status only (XR-RPP-005; control uses its magnitude). Also reported: the prototype's tick/segment codes, the RTK refusal reason, conditioned artifact hash, `spray_request` (legacy diagnostic only; not actuator
  authority), the loop jitter and overrun count (`LoopTimer`).
* **Unported feature** (`point_hold_enabled`): STOP, `STATE_ERROR`, `handoff = 1`. It refuses to drive rather than drive without the overlay.

## 3. Parameters

All 117 RPP parameters are ROS parameters with their `ParamSet` class: a change is validated (range, enumerations, cross-parameter relations) and applied
atomically in the set-callback, recorded in the journal, and **refused** (never deferred) when it is RESTART at runtime or IDLE_ONLY while a mission is loaded or active
(`MissionState` LOADING, READY, RUNNING or PAUSED, or a path loaded or failing to load). RPP-006: the core reads many IDLE_ONLY values every tick
(`require_rtk_fix`, `pose_max_age_s`, `rtk_*`, `ekf_*`, ...), so a change accepted while PAUSED would switch a safety gate on resume; the
conditioning subset would silently wait for the next load. Change them with no mission (IDLE, or after COMPLETED / ABORTED / ERROR).
Startup values (launch file) go through `init_many`: any class, same validation, fail loud (the constructor throws) — never a silent fallback to a default.
XR-RPP-009 sanity upper bounds (DERIVED, defined in `tools/gen_param_tables.py`): `pose_max_age_s`, `rtk_fix_timeout_s` and
`curvature_baseline_m` at most 2.0; `preview_curvature_n` and `corner_smooth_arc_pts` at most 64 (the core also caps the preview count at
64); every Int must fit an `int`.
Node-level: `tick_hz` (50, in [20, 100]; DERIVED from the prototype's `CONTROL_HZ`), `artifact_dir` (`/var/lib/dyx3/missions`),
`event_driven` (true, RESTART: tick per pose sample with `tick_hz` as the watchdog, section 2; false = free-running `tick_hz` timer).

## 4. Real-time discipline

`main` calls `mlockall` (best effort: a container without the capability continues and says so). The core allocates nothing in `tick()` (`rpp_core_test` TickNeverAllocates counts heap allocations through a corner pivot, the precise stop
and completion; the corner-FSM transition log is a fixed ring of string literals, XR-RPP-009); the node's publishes
use the middleware's normal path (small fixed-size messages, no strings on the hot path); logging happens only on a state transition. SCHED_FIFO 80 on CPU 4,
shared with motion_guard, comes from the launch prefix in `dyx3_bringup/launch/control_graph.launch.py` (DERIVED, 2026-10-09). The main loop blocks in
`spin_once(5 ms)`: it must never poll, because it shares a FIFO core with the guard. **Not done:** the measurement of any of this (timing is not provable off-target).

## 5. Proof

`rpp_node_test` (in-process, private DDS domain, injected monotonic clock, a kinematic stand-in vehicle): 32 cases — startup validation; load by id and
acknowledgement; a missing artifact; a **whole mission driven to COMPLETE** (the line is marked where the planner says, stops on the final point within 6 cm);
stale pose; RTK drop with the reason; pause and resume from rest; entry pivot; parameter classes; the unported feature; every emitted mode contract-conforming; a final approach 3 cm to the side of the endpoint completes within 10 s
with at most 2 speed reversals (XR-RPP-001); on an L-shaped mission the first heading after the corner pivot is the exit leg (XR-RPP-007).
Command-level cases (XR-RPP-006) drive the stand-in from the published `MotionSetpoint` alone, which the equivalence suites (core output)
cannot see: an L-shaped mission (corner pivot sign, heading per leg, at most 5 cm off the path, COMPLETE at the end); an endpoint 5 cm to the
side (every creep moves toward it, ends within 2 cm); a pause with a 0.2 m coast and a resume (STOP while paused, ramp from rest, COMPLETE).
A tangent run handover (smooth TRANSIT arc into a segment MARK line) crossed at speed publishes no STOP before COMPLETE (XR-RPP-002).
0.17.0: `start_run_index = 2` on a three-run path: LOADED and the first RUNNING status show `run_index 2`, the first command is the
alignment pivot of run 2, run 2 is driven to COMPLETE and no status ever shows an earlier run; a start run carried by the RUNNING
transition is taken, a later change is ignored; an index past the last run is STOP + ERROR (also after the retry interval) and a new
mission id loads again; a stationary pivot publishes `pivot_timed_out` only after the budget, still PIVOTING, and drops it once released.
`rpp_core_test` / `rpp_modules_test` pin the same at core and FSM level (`ACornerPivotThatNeverTurns*`, `AStartAtRunN*`,
`AnInvalidStartRun*`, `CornerFsm.PivotTimedOut*`).
The stand-in can limit its acceleration (`accel_limit`) so a body-axis brake decelerates through zero as a vehicle does. `rpp_core_test` pins core behaviour the prototype did not have (the precise-stop timeout brake).
The stand-in vehicle does exactly what the last command asks: it proves the wiring and the state machine, not the controller on a rover.

## 6. Open questions for the human

* `TRACK_HEADING` for segment runs vs `TRACK_RATE` with `segment_yaw_rate_gain * theta_e` (DERIVED choice, GATE 4).
* `spray_request` is diagnostic/deprecated and never wired into actuator authority. The exact RPP heading error, validity, run index, and progress are telemetry consumed by `dyx3_spray`; the spray controller owns MARK/TRANSIT, heading cut/entry hold, safety, and final valve decision.
* SCHED_FIFO priority / CPU affinity; `ROS_DOMAIN_ID`; `artifact_dir` provisioning (the backend writes it, the services read it).
