# Interface changelog

## 0.1.0 — 2026-10-07

- Initial frozen `dyx3_interfaces` surface: canonical motion command/status, vehicle and
  subsystem status messages, mission services, and `ExecuteMission` action.
- Motion commands are NED/FRD with signed body-X speed. See
  `docs/contracts/frames.md` for the frozen frame, sign, unit, and clock contract.

## 0.2.0 — 2026-10-07 (Phase 1 review fixes; nothing consumed 0.1.0 yet)

- **BREAKING** `RtkStatus.fix_type` constants now mirror `px4_msgs/SensorGps.fix_type`
  exactly: `FIX_3D` 2→3; added `FIX_2D=2`, `FIX_RTCM_CODE_DIFFERENTIAL=4`,
  `FIX_EXTRAPOLATED=8`. `FIX_UNKNOWN=0`, `FIX_NONE=1`, `FIX_RTK_FLOAT=5`,
  `FIX_RTK_FIXED=6` unchanged. Migration: none — no package consumed 0.1.0 values.
- `MotionSetpointStatus`: added `REASON_HEADING_UNHEALTHY=9`,
  `REASON_OPERATOR_LINK_LOST=10`, `REASON_ARMING_GATE=11`,
  `REASON_ESTIMATOR_UNHEALTHY=12`. Existing values 0–8 unchanged.
- New `EstimatorHealth.msg` (GNSS-yaw rejection/fault, yaw/position/velocity test
  ratios, dead reckoning) for `dyx3_motion_guard`. Default = no data = unhealthy.
- **BREAKING** `AbortMission`: request gains `REASON_UNSPECIFIED=0`; response
  `RESULT_OK/RESULT_NOT_ACTIVE`/`result_code` renamed `REASON_OK/REASON_NOT_ACTIVE`/
  `reason_code` (consistent with every other service). Wire layout unchanged
  (one `uint8`), source-level rename only.
- Documented ownership: `ExecuteMission` is the canonical mission-execution
  interface; `StartMission` is an admission-only wrapper over the same
  `dyx3_mission` code path. No wire change.

## 0.3.0 — 2026-10-07 (Phase 3/6 needs)

- `MissionState`: appended `string path_artifact_sha256` (content hash of the loaded artifact;
  dyx3_rpp loads the same file by this id). Appended field — existing fields unchanged.
- New `SafetyGateStatus` (guard-owned aggregate of every gate except the mission gate; default
  `ok = false`), `OperatorLinkStatus` (gateway-owned tablet heartbeat state; default `alive = false`),
  `EmergencyStopState` (guard-owned latched E-stop; absence of a fresh message == asserted).

## 0.4.0 — 2026-10-07 (Phase 7 needs)

- New `Px4LinkStatus` (session / handshake / staleness / fail-to-zero evidence; default = unhealthy),
  `GnssReport` (raw FCU GNSS pass-through; only px4_link may read /fmu), `RtcmData` (gnss_rtk →
  px4_link RTCM chunks), `UlogChunk` (px4_link → recorder ULog chunks).
- New services `ArmDisarm` and `SetOffboard` (owner dyx3_px4_link). Defaults are the safe request
  (disarm / disable). No existing definition changed.

## 0.5.0 — 2026-10-07 (Phase 8 needs)

- `GnssReport`: appended `latitude_deg`, `longitude_deg`, `altitude_msl_m`, `hdop` (the NTRIP GGA back-feed needs a
  rover position and the receiver's own satellite/HDOP figures instead of the prototype's placeholders).
  Appended fields — existing fields unchanged; consumers of 0.4.0 are unaffected.
- New `NtripStatus` (link state, correction age and rate, frame/byte/CRC counters, reconnects, FIX transitions) for
  the backend and the recorder. Default = no corrections.

## 0.6.0 — 2026-10-07 (Phase 9 needs)

- New `SprayActuatorCommand` / `SprayActuatorAck` (dyx3_spray and the spray watchdog ask `dyx3_px4_link`, the only
  package that touches /fmu, to drive the valve via DO_SET_ACTUATOR / DO_SET_SERVO and report the FCU's ack),
  `SprayLease` (typed replacement for the prototype's JSON lease), `SprayWatchdogStatus`, `SprayStatus` (rich status for the
  recorder/backend), and the `SetSprayManual` service. No existing definition changed.


## 0.7.0 — 2026-10-07 (timesync evidence)

- `Px4LinkStatus`: appended `timesync_valid`, `timesync_offset_us`, `timesync_round_trip_us` (the FCU timesync estimate, for the recorder's
  start/end log, `dyx3-health` and the gateway's telemetry). Appended fields — existing fields unchanged. Values only: no gate consumes them yet
  (no numeric convergence criterion exists; F-tasks A1.4).

## 0.8.0 — 2026-10-07 (RPP node)

- `RppStatus`: new `STATE_LOADED=7` (the verified path is loaded and RPP waits for the mission to run; this is the acknowledgement
  `dyx3_mission` waits for) and appended `tick_state`, `segment_state` (the prototype's diagnostic codes), `spray_request` (the
  heading-gated MARK request; the valve stays owned by `dyx3_spray`), `handoff` and `rtk_reason`. Appended fields and one new constant —
  existing fields and constants unchanged; consumers of 0.7.0 are unaffected.
