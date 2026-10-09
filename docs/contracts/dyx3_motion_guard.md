# dyx3_motion_guard — contract

**Status:** draft for review, written before the implementation (CLAUDE.md §6: SAFETY-CRITICAL).
**Spec:** V1 §7.6, §4.3.1 (operator link), §5.2; Phase plan 6.
**Authority:** the last software authority before PX4. It validates, limits, gates and fails to
zero. **It never invents a correction**: if RPP is wrong the guard stops the rover, it does not
steer. It is the single owner of every safety gate (E-stop, RTK, arming, heading, estimator,
operator link, PX4 link, mission). `dyx3_mission` consumes `SafetyGateStatus` and never
re-implements a gate. A backend or tablet E-stop is a *request* to this node.

## 1. Interfaces

| Direction | Topic / service | Type | Notes |
|---|---|---|---|
| in | `/dyx3/rpp/motion_setpoint` | MotionSetpoint | the only motion source. (The quarantined legacy node's `output_topic` is set to this name in config.) |
| in | `/dyx3/mission/state` | MissionState | mission gate |
| in | `/dyx3/vehicle_state` | VehicleState | arming, nav state, estimate validity |
| in | `/dyx3/estimator_health` | EstimatorHealth | heading and estimator gates |
| in | `/dyx3/rtk_status` | RtkStatus | RTK gate |
| in | `/dyx3/operator_link` | OperatorLinkStatus | tablet heartbeat (gateway owns the timeout) |
| in | `/dyx3/px4_link/status` | Px4LinkStatus | link gate |
| service | `/dyx3/motion_guard/set_emergency_stop` | SetEmergencyStop | latch / clear |
| out | `/dyx3/motion_guard/command` | MotionSetpoint | consumed only by `dyx3_px4_link` |
| out | `/dyx3/motion_guard/status` | MotionSetpointStatus | per decision: input seq, reason, applied command |
| out | `/dyx3/safety_gate` | SafetyGateStatus | fixed 10 Hz, independent of RPP |
| out | `/dyx3/emergency_stop_state` | EmergencyStopState | fixed 10 Hz |

## 2. Fixed-rate decision loop

A timer at `publish_rate_hz` (default 50, DERIVED = the prototype's 50 Hz control rate) runs the
decision below **whether or not RPP is publishing** and always publishes a command. A guard
that only reacted to incoming messages could not report the loss of RPP. The output command is
always valid and contract-conforming: either the (limited) RPP command or the canonical STOP
(`mode=STOP`, `speed=0`, `yaw=NaN`, `rate=0`, `valid=true`). The output `seq` is the guard's
own, monotonic over the process lifetime.

## 3. Decision order

For the freshest RPP command, the first failing check wins and yields STOP with its reason:

| # | Check | Reason |
|---|---|---|
| 1 | message valid: `valid`, known mode, finite values, per-mode NaN pattern of `MotionSetpoint.msg` (STOP is always acceptable) | `INVALID_MESSAGE` (1) |
| 2 | fresh: arrival within `command_max_age_s` on the guard's steady clock (message stamps are never trusted for freshness: replay and clock skew) | `STALE` (2) |
| 3 | sequence: strictly increasing; a decrease or the first command ever starts a new publisher session and `session_accept_count` strictly increasing commands must follow before motion is accepted; an equal seq never refreshes freshness | `SEQUENCE` (3) |
| — | a clean STOP command is forwarded as STOP here, whatever the gates say, `accepted=true`, reason `OK` | |
| 4 | E-stop latched | `ESTOP` (5) |
| 5 | PX4 link: session alive, handshake ok, no stale topic, status fresh | `PX4_LINK_UNHEALTHY` (8) |
| 6 | operator link alive and fresh | `OPERATOR_LINK_LOST` (10) |
| 7 | arming: `arming_state == ARMED`, `nav_state == OFFBOARD`, no PX4 failsafe, vehicle state fresh | `ARMING_GATE` (11) |
| 8 | RTK: fix RTK_FLOAT/RTK_FIXED at or above `rtk_min_fix_type`, corrections fresh, horizontal accuracy known and within `rtk_max_hrms_m`, status fresh | `RTK_GATE` (6) |
| 9 | heading: estimator health fresh and `flags_valid`, GNSS yaw fusion intended and not faulted, yaw not rejected, `VehicleState.attitude_valid` | `HEADING_UNHEALTHY` (9) |
| 10 | estimator: position and velocity valid, no inertial dead reckoning, no horizontal position/velocity rejection | `ESTIMATOR_UNHEALTHY` (12) |
| 11 | mission: `MissionState` fresh and `RUNNING` | `MISSION_GATE` (4) |
| 12 | hard envelopes (section 5): clamp, never refuse | `LIMIT_CLAMPED` (7) with `accepted=true`, `clamped=true` |

`SafetyGateStatus` is the aggregate of checks 4–10 (everything except the mission gate and the
command's own validity) at a fixed 10 Hz with the same priority order; `ok=false` is
authoritative and default. A gate input that was never received or is older than its limit is
**failing**, never "assumed fine". A gate failing on a non-STOP command always zeroes the output at
the same tick: there is no ramp-down, because fail-to-zero is immediate (a ramp would be a
guard-invented motion).

Recovery has no latch other than the E-stop and the sequence rule: when every check passes
again the next valid RPP command is forwarded. A *mission* must not silently resume after a
safety pause: that is `dyx3_mission`'s FSM (never auto-resume), not this node's.

## 4. Emergency stop

`SetEmergencyStop(asserted, source)`: sources `tablet`, `backend`, `ble`, `physical` are accepted
(`REASON_INVALID_SOURCE` otherwise). Asserting latches immediately; the next decision tick outputs
STOP. Clearing needs an explicit `asserted=false` from a valid source. The latch survives
everything except a process restart. DERIVED — NOT FROM V1 SPEC: boot state is *clear* (the
physical E-stop is hardware, and a not-RUNNING mission already gates motion); a human may require
boot-asserted. `EmergencyStopState` is published at 10 Hz; a consumer treats its absence as asserted.

## 5. Limits

Applied after every gate passed, to the forwarded command, never to STOP:

| Hard envelope | Parameter | Default | Source |
|---|---|---|---|
| forward speed | `max_forward_speed_mps` | 1.0 | prototype `max_linear_vel` default. Field runs 0.35; re-validate at GATE 4 |
| reverse speed | `max_reverse_speed_mps` | **0.10** | **DERIVED — human decision 2026-10-08 (review H6 / fix plan B1).** RPP's active brake is capped at 0.08 m/s and terminal creep is 0.10 m/s. Initial bench value; re-validate at GATE 1. |
| absolute yaw rate | `max_yaw_rate_radps` | 0.45 | prototype `max_yaw_rate_body` default. Re-validate at GATE 4 |

**B2 / review H5 authority decision (human, 2026-10-08): RPP is the sole normal motion-profile
owner.** Motion Guard does not ramp acceleration/deceleration, jerk-limit speed, or limit yaw-rate
change. If an RPP speed/yaw-rate is inside the three hard envelopes above, the guard forwards it
unchanged. A hard-envelope violation is clamped and reported `LIMIT_CLAMPED`; any safety-gate
failure is still an immediate canonical STOP.

The previous accel/decel/jerk/yaw-accel shaper is retained only in the pure-C++ `Limits` test harness
behind `profile_shaping_test_mode`. That flag defaults false, is not a ROS parameter, and the
production node does not declare the four shaping values as ROS parameters. It exists only to keep
the old limiter mechanics directly testable; it is not a second production controller.

## 6. Parameters

| Name | Default | Class | Source |
|---|---|---|---|
| `publish_rate_hz` | 50 | RESTART | DERIVED (prototype 50 Hz) |
| `command_max_age_s` | 0.2 | RESTART | DERIVED from prototype `input_max_age_s`; re-validate GATE 4 |
| `session_accept_count` | 3 | RESTART | DERIVED — Phase plan leaves the count to this phase |
| `vehicle_state_max_age_s`, `rtk_status_max_age_s` | 0.5 | RESTART | prototype `pose_max_age_s` / `rtk_fix_timeout_s` |
| `estimator_health_max_age_s`, `operator_link_max_age_s`, `px4_link_max_age_s`, `mission_state_max_age_s` | 0.5 | RESTART | DERIVED, same convention |
| `rtk_min_fix_type` | 6 | RESTART | prototype `min_fix_type` |
| `rtk_max_hrms_m` | 0.10 | RESTART | prototype `rtk_max_hrms_m` |
| `require_gnss_yaw_fusion` | true | RESTART | DERIVED: CLAUDE.md §3, dual-antenna heading is first-class |
| hard envelopes | section 5 | RESTART (LIVE target) | spec §9 lists speed limits as LIVE |

**This version reads every parameter once at start** and rejects every runtime parameter update
with an explicit startup-only reason. Invalid startup values throw; there is no fallback to a
default. No MotionGuard parameter is currently LIVE or IDLE_ONLY. A validated runtime path that
only **tightens** a hard envelope while a mission is RUNNING remains a follow-up. RPP owns
accel/decel/jerk profile parameters.
A restart mid-mission in OFFBOARD aborts the run (CLAUDE.md section 7), which is why it must come.
`yaw_test_ratio_max` and the other ratio limits are **not implemented**: `estimator_status` is not on
DDS at the flashed firmware (`EstimatorHealth.test_ratios_valid` is always false), and no numeric
limit has a source.

## 7. Real-time discipline

`mlockall`, no heap allocation and no logging syscall on the decision path (the decision is a pure
function on stack structs; a reason change is recorded from the node after the publish with
rate-limited logging). QoS declared per topic: commands and gates reliable depth 1, status
reliable depth 10.

Shutdown: rclcpp's own signal handler is disabled (as in `rpp_node`). SIGINT/SIGTERM raise a flag,
the spin loop exits, then a bounded burst (5) of canonical STOP goes out on
`/dyx3/motion_guard/command` while the context is still up. SIGPIPE is ignored. The stop-on-exit that
matters most is in `dyx3_px4_link`, the last hop; this burst only helps if `px4_link` outlives the guard.

## 8. Not proven off-target

Loop jitter, behaviour under DDS reordering, the real tablet heartbeat path, the interaction with the
firmware's own offboard-loss handling. Off-target proof: every row of the decision table, the
sequence rule, the limiter, and a randomised property test that no non-STOP command ever leaves the
guard while any gate fails.
