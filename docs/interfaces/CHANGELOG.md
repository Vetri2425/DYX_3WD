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
