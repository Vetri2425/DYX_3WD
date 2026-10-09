# dyx3_rpp node (`RppNode`) — contract

**Status:** 2026-10-07 (cloud session). **Spec:** V1 section 7.4. The decision lives in `RppCore` (`rpp_orchestrator.md`); this node is the ROS wiring only.
**Authority:** none beyond the path-following decision. It publishes no safety verdict; `dyx3_motion_guard` is the last authority before PX4.

## 1. Interfaces

| Direction | Topic | Type | QoS | Notes |
|---|---|---|---|---|
| in | `/dyx3/vehicle_state` | VehicleState | R1 | pose is fed only while `position_valid` and `attitude_valid`; velocity while `velocity_valid`. An invalid sample is **not** fed: the pose ages out and the core stops (STALE) |
| in | `/dyx3/rtk_status` | RtkStatus | R1 | `fix_type` and `horizontal_accuracy_m` (0 = unknown, passed as unknown) |
| in | `/dyx3/mission/state` | MissionState | R1 | `path_artifact_sha256` + `mission_id` select the path; `state == RUNNING` is the only state in which the core ticks |
| out | `/dyx3/rpp/motion_setpoint` | MotionSetpoint | R1 | every tick, always `valid`; STOP unless running |
| out | `/dyx3/rpp/status` | RppStatus | R1 | every tick; `mission_id` is the acknowledgement `dyx3_mission` waits for |

## 2. Behaviour

* **Load by id.** On a `MissionState` in LOADING / READY / RUNNING / PAUSED with a non-empty hash, the node reads `<artifact_dir>/<sha>.dyx3path`
  with the same reader `dyx3_mission` uses (the hash is verified), turns the points into runs with `condition_path` (the proven port of `_path_cb`),
  and installs those exact runs in `RppCore`. It writes an immutable `DYX3COND 1` artifact from the installed runs and publishes its content hash in
  `RppStatus.conditioned_execution_sha256`. Spray uses that artifact for projection. A **new mission id** loads again even for the same source file.
  A missing or corrupt file, or a path that conditions to nothing, is `STATE_ERROR` (STOP) and is retried once a second.
* **Not RUNNING** (loaded and waiting, or paused): STOP every tick, `STATE_LOADED`. On the RUNNING to not-RUNNING edge the core forgets its motion
  memory (`pause()`: speed memory, hard-curvature latch, stop confirmation) so a resume ramps from rest. A mission is never resumed by RPP on its own: that is `dyx3_mission`'s FSM.
* **RUNNING:** one `RppCore::tick` per control tick, mapped by `command_from_tick` (`rpp_command.hpp`):
  STOP -> `MODE_STOP`; TRACK -> `TRACK_RATE` (smooth run: speed = |v|, rate = feed-forward + feedback) or `TRACK_HEADING` (segment run: speed = |v|,
  heading = the bearing of the core's velocity vector whenever it is non-zero, the core's frozen heading only for an exactly zero vector, so a stop never
  snaps to North and the slow first ticks after a pivot never carry the previous leg's heading (XR-RPP-007)); BRAKE -> signed speed along the nose with the nose held (reverse is a
  negative speed, never a spot turn; permission is the guard's); PIVOT -> `MODE_PIVOT` with `omega = clamp(1.5 * err, +/- max_yaw_rate_body)`; CREEP -> `MODE_CREEP` along the nose, or `TRACK_HEADING` toward the precise stop's correction vector when it is more than 2° off the nose axis
  (forward ahead of the beam, reverse behind it; XR-RPP-001, `rpp_motion_output.md` section 2).
  Any non-finite value becomes the canonical STOP.
* **State reporting** (`RppStatus.state`): TRACKING only while actually tracking (tick state TRACKING or APPROACH) — `dyx3_spray` reads TRACKING as the
  "mission has started" evidence and PIVOTING as the pivot gate, so STOPPING (brake, corner stop, a gate refusal), CREEPING (precise stop) and COMPLETE must never be
  reported as TRACKING; LOADED while waiting; ERROR when the artifact is bad or an unported feature is enabled; COMPLETE when the core finished.
  Also reported: the prototype's tick/segment codes, the RTK refusal reason, conditioned artifact hash, `spray_request` (legacy diagnostic only; not actuator
  authority), the loop jitter and overrun count (`LoopTimer`).
* **Unported feature** (`point_hold_enabled`): STOP, `STATE_ERROR`, `handoff = 1`. It refuses to drive rather than drive without the overlay.

## 3. Parameters

All 117 RPP parameters are ROS parameters with their `ParamSet` class: a change is validated (range, enumerations, cross-parameter relations) and applied
atomically in the set-callback, recorded in the journal, and **refused** (never deferred) when it is RESTART at runtime or IDLE_ONLY while a mission runs.
Startup values (launch file) go through `init_many`: any class, same validation, fail loud (the constructor throws) — never a silent fallback to a default.
Node-level: `tick_hz` (50, in [20, 100]; DERIVED from the prototype's `CONTROL_HZ`), `artifact_dir` (`/var/lib/dyx3/missions`).

## 4. Real-time discipline

`main` calls `mlockall` (best effort: a container without the capability continues and says so). The core allocates nothing in `tick()`; the node's publishes
use the middleware's normal path (small fixed-size messages, no strings on the hot path); logging happens only on a state transition. SCHED_FIFO 80 on CPU 4,
shared with motion_guard, comes from the launch prefix in `dyx3_bringup/launch/control_graph.launch.py` (DERIVED, 2026-10-09). The main loop blocks in
`spin_once(5 ms)`: it must never poll, because it shares a FIFO core with the guard. **Not done:** the measurement of any of this (timing is not provable off-target).

## 5. Proof

`rpp_node_test` (in-process, private DDS domain, injected monotonic clock, a kinematic stand-in vehicle): 13 cases — startup validation; load by id and
acknowledgement; a missing artifact; a **whole mission driven to COMPLETE** (the line is marked where the planner says, stops on the final point within 6 cm);
stale pose; RTK drop with the reason; pause and resume from rest; entry pivot; parameter classes; the unported feature; every emitted mode contract-conforming; a final approach 3 cm to the side of the endpoint completes within 10 s
with at most 2 speed reversals (XR-RPP-001); on an L-shaped mission the first heading after the corner pivot is the exit leg (XR-RPP-007).
The stand-in can limit its acceleration (`accel_limit`) so a body-axis brake decelerates through zero as a vehicle does. `rpp_core_test` pins core behaviour the prototype did not have (the precise-stop timeout brake).
The stand-in vehicle does exactly what the last command asks: it proves the wiring and the state machine, not the controller on a rover.

## 6. Open questions for the human

* `TRACK_HEADING` for segment runs vs `TRACK_RATE` with `segment_yaw_rate_gain * theta_e` (DERIVED choice, GATE 4).
* `spray_request` is diagnostic/deprecated and never wired into actuator authority. The exact RPP heading error, validity, run index, and progress are telemetry consumed by `dyx3_spray`; the spray controller owns MARK/TRANSIT, heading cut/entry hold, safety, and final valve decision.
* SCHED_FIFO priority / CPU affinity; `ROS_DOMAIN_ID`; `artifact_dir` provisioning (the backend writes it, the services read it).
