# Topics, services and QoS of the control graph

**Status:** extracted from the code on 2026-10-07 (cloud session), not from the spec. Where this table and a node's source disagree, the source is right
and this file is stale: regenerate it. Types are `dyx3_interfaces/msg/*` unless a `px4_msgs` name is given. Everything runs in a private DDS domain
(`ROS_DOMAIN_ID` has no default: the launchers refuse to start without it; the number is an OPEN item). **Only `dyx3_px4_link` touches `/fmu/**`.**

QoS shorthand: `R1` = reliable, keep-last 1 (state that is only useful when it is the newest); `R10` / `R16` / `R32` / `R64` = reliable with that depth (commands, chunks,
events that must not drop); `R100` = point results.

## Control chain (the motion path, in order)

| Topic | Type | Publisher | Subscribers | QoS | Rate |
|---|---|---|---|---|---|
| `/dyx3/vehicle_state` | VehicleState | px4_link | rpp, motion_guard, spray, mission, gateway | R1 | once per new FCU local-position sample (about 50 Hz), published in the sample's callback (`event_driven`, default); while the local position is stale, a 50 Hz republish of the cache with the validity flags cleared. With `event_driven=false`: a 20 ms gate republishes the newest cached sample (the same sample can repeat). `px4_sample_stamp` identifies the sample (IF-005) |
| `/dyx3/estimator_health` | EstimatorHealth | px4_link | motion_guard, gateway | R1 | 10 Hz, newest cached `estimator_status_flags` sample (about 1 Hz from the FCU) |
| `/dyx3/rtk_status` | RtkStatus | gnss_rtk | rpp, motion_guard, spray, gateway | R1 | 5 Hz |
| `/dyx3/mission/state` | MissionState | mission | rpp, spray, motion_guard, recorder, gateway | R1 | 10 Hz (and on change) |
| `/dyx3/rpp/motion_setpoint` | MotionSetpoint | **rpp** | motion_guard | R1 | once per new pose sample (about 50 Hz), 50 Hz watchdog in silence (`event_driven`); always, STOP when not running |
| `/dyx3/rpp/status` | RppStatus | **rpp** | mission, spray, gateway | R1 | 50 Hz |
| `/dyx3/motion_guard/command` | MotionSetpoint | motion_guard | px4_link | R1 | once per RPP command, 50 Hz watchdog when RPP is silent (`event_driven`); always valid |
| `/dyx3/motion_guard/status` | MotionSetpointStatus | motion_guard | gateway | R1 | per decision |
| `/dyx3/safety_gate` | SafetyGateStatus | motion_guard | mission, gateway | R1 | 10 Hz |
| `/dyx3/emergency_stop_state` | EmergencyStopState | motion_guard | spray, gateway | R1 | 10 Hz (absence == asserted) |
| `/dyx3/operator_link` | OperatorLinkStatus | system_gateway | motion_guard | R1 | gateway-owned heartbeat verdict |
| `/dyx3/px4_link/status` | Px4LinkStatus | px4_link | motion_guard, recorder, gateway | R1 | |

## Spray

| Topic | Type | Publisher | Subscribers | QoS |
|---|---|---|---|---|
| `/dyx3/spray/actuator_command` | SprayActuatorCommand | spray, spray_watchdog | px4_link | R10 / R16 |
| `/dyx3/spray/actuator_ack` | SprayActuatorAck | px4_link | spray, spray_watchdog | R10 / R16 |
| `/dyx3/spray/lease` | SprayLease | spray | spray_watchdog | R1 |
| `/dyx3/spray/watchdog_status` | SprayWatchdogStatus | spray_watchdog | spray | R1 |
| `/dyx3/spray/state` | SprayState | spray | (backend/recorder via the gateway) | R10 |
| `/dyx3/spray/status` | SprayStatus | spray | gateway | R10 |

## GNSS corrections, recording

| Topic | Type | Publisher | Subscribers | QoS |
|---|---|---|---|---|
| `/dyx3/gnss_report` | GnssReport | px4_link | gnss_rtk, gateway | R5 |
| `/dyx3/rtcm` | RtcmData | gnss_rtk | px4_link | R32 |
| `/dyx3/ntrip_status` | NtripStatus | gnss_rtk | gateway | R1 |
| `/dyx3/ulog_chunk` | UlogChunk | px4_link | recorder | R64 |
| `/dyx3/recorder/status` | RecorderStatus | recorder | gateway | R1 |
| `/dyx3/mission/point_result` | PointResult | mission | gateway | R100 |

## `/fmu/**` (dyx3_px4_link only)

In: `/fmu/out/{timesync_status, vehicle_local_position_v1, vehicle_status_v1, vehicle_attitude, estimator_status_flags, vehicle_gps_position, vehicle_command_ack,
message_format_response, ulog_stream}`. Out: `/fmu/in/{offboard_control_mode, trajectory_setpoint, rover_speed_setpoint, rover_attitude_setpoint, rover_rate_setpoint,
vehicle_command, gps_inject_data, ulog_stream_ack, message_format_request}`. QoS follows PX4's uXRCE-DDS (best effort for sensors out, reliable in).
`estimator_status` (the test ratios) is NOT on DDS at the flashed firmware: `EstimatorHealth.test_ratios_valid` is always false (firmware `dds_topics` change needed).

## Services and actions

| Name | Type | Server | Clients |
|---|---|---|---|
| `/dyx3/mission/start` `.../pause` `.../resume` `.../abort` `.../skip_point` | StartMission, PauseMission, ResumeMission, AbortMission, SkipPoint | mission | gateway |
| `/dyx3/mission/execute` (action) | ExecuteMission | mission | (canonical interface; `start` is an admission-only wrapper) |
| `/dyx3/motion_guard/set_emergency_stop` | SetEmergencyStop | motion_guard | gateway |
| `/dyx3/px4_link/arm`, `/dyx3/px4_link/set_offboard` | ArmDisarm, SetOffboard | px4_link | gateway |
| `/dyx3/spray/set_manual` | SetSprayManual | spray | gateway |

## Rules this table encodes

* One motion source: nothing but `dyx3_rpp` publishes `/dyx3/rpp/motion_setpoint`, and nothing but `dyx3_motion_guard` publishes `/dyx3/motion_guard/command`.
* No safety verdict is published by the backend or the tablet; their E-stop is a service request to the guard (via the gateway).
* RPP and spray load the SAME content-addressed artifact the mission names in `MissionState.path_artifact_sha256`.
* Open: the DDS scoping (loopback vs the FCU Ethernet interface), the recorded topic list (raw `/fmu` is excluded), and per-topic latency budgets (timing is not provable off-target).
