# Contract — `dyx3_rpp` overview and control-tick order

**Status:** extracted 2026-10-07 from `PX4_DXP` `build/demo-ready` @ `fc6436b`, `src/rpp_controller_node.py`
(6 285 lines, 119 parameters). Read with the per-module contracts: `rpp_path_conditioner.md`,
`rpp_guidance.md`, `rpp_speed_profile.md`, `rpp_stop_pivot_fsm.md`, `rpp_terminal.md`,
`rpp_spray_gate.md`, `rpp_motion_output.md`. Spec §7.4 decomposition; spec §10 contract-first.

**Evidence level:** source-verified. The prototype's own pytest suite was run against the carried
copy (`dyx3_rpp_legacy/test/dxp_verbatim`): **183 passed, 7 failed — identical in the untouched
PX4_DXP checkout** (list in `rpp_legacy_evidence.md`). No field bag was available in this environment.

## 1. What the controller is

A 50 Hz (`CONTROL_HZ = 50`) path follower that, per tick, reads the latest pose (NED + yaw), measured
velocity and GNSS quality, and commands a **NED velocity vector** (+ a body yaw rate, + the spray flag).
Two tracking profiles share one skeleton:

* **segment** — straight legs and hard corners. Steering = bearing to a lookahead point; corners are
  *stop → pivot → release*; the legacy yaw rate is `segment_yaw_rate_gain · θe` (P on heading error).
* **smooth** — continuous curvature. Pure-pursuit curvature `κ = 2·y_body/L²`; legacy yaw rate
  `κ·speed (+ yaw_rate_feedback_gain·θe)`; speed regulated by the worst preview curvature.

## 2. One tick, in order (`_control_loop` → `_control_loop_impl`, lines 4753–5550)

| # | Step | Failure / outcome |
|---|---|---|
| 0 | `_drain_pending_mission()` (atomic install of a staged mission) | — |
| 1 | `dt = clamp(now − last_tick, 0, 0.1)`; first tick after install uses `1/50` | clamp prevents one huge accel step; floor 0 prevents a negative dt lowering the ceiling |
| 2 | derive `max_v = min(max_linear_vel, mission_speed)`; `approach_d = max(approach_velocity_scaling_dist, v²/(2·max_linear_decel)+0.10)`; `jump_thr = max(ekf_jump_threshold_m, max(max_v, v_meas)·max(worst_pose_gap, 1/50)+0.03)` | derived values are **floors**, never below the configured parameter |
| 3 | no pose → `IDLE` zero | |
| 4 | pose older than `pose_max_age_s` (+`imu_max_extrap_age_s` if extrapolating) → `STALE` zero | **fail to zero** |
| 5 | optional velocity extrapolation `pos += v·(pose_age + pose_latency_bias_s)` | default off |
| 6 | RTK gate (`require_rtk_fix`): immediate drop, slow recovery (`rtk_recover_hold_s`) → `RTK_WAIT` zero | **fail to zero** |
| 7 | no path → `IDLE`; `_path_done` → `DONE` zero | |
| 8 | jump guard: `|Δpos| > jump_thr` → either absorb into `_ekf_reset_offset` (`ekf_reset_compensation`, ≤ `ekf_reset_max_absorb_m`) or `JUMP_SKIP` zero + hint reset | one-tick skip |
| 9 | `_run_alignment_hold` (pivot toward the new run's heading) | returns early while holding |
| 10 | smooth profile: `_project_onto_path` | |
| 11 | point-hold overlay `_point_hold_tick` (default off) | |
| 12 | `_run_boundary_stop_pending` → `_hold_before_run_advance` | |
| 13 | `_completion_stop_pending` → `_hold_at_completion` **before** the goal test | a coast back above tolerance must not re-enter tracking |
| 14 | segment final-run precise-stop hook | |
| 15 | goal reached (`dist ≤ goal_tol_eff` or `endpoint_capture_recovered`) after `min_travel` → advance run (stop first if a hard corner) or `_hold_at_completion` | |
| 16 | `segment` → `_control_segment_profile`; else smooth steps 1–8 below | |
| smooth | project → adaptive `L_d` → curvature cap → lookahead point → `κ`, `θe` → preview-κ speed → approach scaling → accel/decel slew → P4 floor → FF yaw rate → NED vector → forward-cone clamp → publish | |

## 3. Invariants (named in the prototype; the C++ must preserve them)

* **I1** — a brake command is *exactly* ± along the body axis (`_corner_brake_velocity`), never an off-axis
  recenter vector. Capped at `segment_brake_velocity_cap_m_s`; zero when velocity is stale, braking is
  disabled, the rover is already below `segment_stop_speed_threshold`, or motion is lateral-dominant
  (`|v_fwd| < 0.5·speed`).
* **I3** — "stopped" means *measured* speed < `segment_stop_speed_threshold` **and** |yaw-rate| <
  `segment_stop_yaw_rate_threshold` continuously for `segment_stop_dwell_s`. A **fresh** velocity above the
  threshold NEVER times out into a pivot; only a **stale** velocity does (after `_CORNER_STOP_MAX_HOLD_S`
  = 2.0 s). This was a field bug (pivot started from the wrong point at ~0.14 m/s).
* **Forward cone** — commanded bearing is clamped to ±75° (`_CORNER_MAX_BEARING_OFFSET_RAD`) of the nose
  (`_clamp_velocity_to_forward_cone`) so the *old firmware's* reverse detection never flipped a turn
  (BUG-T3). Under the explicit contract this clamp is a property of the legacy, not of the new interface.
* **Every failure publishes zero**: IDLE / STALE / RTK_WAIT / JUMP_SKIP / DONE all call `_publish_zero`
  (velocity 0, yaw-rate 0, spray off, `_last_speed_cmd = 0` except JUMP_SKIP, kappa latch reset).

## 4. Inputs and frames

Pose: ENU `PoseStamped` (converted at the use-site: north = `pos.y`, east = `pos.x`,
`yaw_ned = π/2 − yaw_enu`, wrapped). Velocity: ENU `TwistStamped` (`v_n = lin.y`, `v_e = lin.x`,
`yaw_rate_ned = −ang.z`). GNSS: `GPSRAW.fix_type` + `h_acc` in **millimetres** (0 ⇒ unknown). Path:
`nav_msgs/Path` in `local_ned` (x = North, y = East, z = bitfield). **Production removes every ENU
conversion** — the conversions above are what `docs/contracts/frames.md` forbids on the new path;
that is exactly why the 119 parameters must be re-validated (GATE 4).

Cross-track sign: `cross_z = dx·(pos_e − foot_e) − dy·(pos_n − foot_n)`, **positive = rover to the RIGHT of
the directed path** (matches `frames.md`).

## 5. Outputs

`/rpp/velocity_ned` (Vector3Stamped, NED), `/rpp/yaw_rate_body` (Float32, NED clockwise-positive),
`/spray/active` (Bool), `/rpp/debug` (Float32MultiArray, append-only 55 slots), `/rpp/segment_debug` (10),
`/rpp/conditioned_path`, optional `/rpp/progress`, `/rpp/milestone`. Production maps these to
`MotionSetpoint`, `RppStatus`, `SprayState` inputs — see `rpp_motion_output.md`.

## 6. State codes (diagnostic ABI, `/rpp/debug[7]`)

`STALE=-1 IDLE=0 TRACKING=1 APPROACH=2 DONE=3 RTK_WAIT=4 JUMP_SKIP=5`.
`SegmentStateCode`: `INACTIVE=0 TRACK_SEGMENT=1 PRE_CORNER_SLOWDOWN=2 CORNER_ALIGN=3 DONE=4 CORNER_STOP=5`.
The server treats STALE/RTK_WAIT/JUMP_SKIP as no-drive equivalents.

## 7. Known open items carried

`C5` segment↔smooth seam, `C9` closed-loop projection snap-back (global-closest search with no monotonic
window in `_project_onto_path`), `D1` pivot walk 0.6–4.8 cm, `D2` approach-stop scatter, `D3` terminal
deceleration inside the paint, `D4` global `xy_goal_tolerance`, `D7` 0.8 m/s rung untested, `A3` EKF reset
(now principled via `VehicleState.xy_reset_counter` — the heuristic jump guard can be replaced).
