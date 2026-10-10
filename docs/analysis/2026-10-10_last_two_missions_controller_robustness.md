# Controller robustness — the last two missions of 2026-10-10 (0007 and 0001)

**Data:** `last2_missions_2026-10-10.tar.gz` (owner upload): recorder runs (`rosbag2` sqlite3+zstd,
`manifest.json`, `summary.json`, `ulog/stream.ulg`) for mission **0007** (stack `172b047`, 13:46 UTC)
and mission **0001** (stack `7f9651d`, 14:25 UTC), plus the PX4 SD logs
`2026-10-10_13-46-47Z_mission_0007.ulg` (63.8 s) and `2026-10-10_14-25-02Z_mission_0001.ulg` (87.9 s).
Both missions: `1_Aug.csv`, three sides of a square, four runs (`run_index` 0–3), `mission_speed`
0.6 m/s, `segment_command_mode = heading`, RTK fixed throughout, EKF healthy throughout.
**Method:** the bags were decoded with a CDR reader driven by the `dyx3_interfaces` definitions of
each run's stack version; the ULogs with `pyulog`. Every number below is computed from those files,
nothing is taken from the HANDOFF. Scripts are not committed (scratch); the computation is simple
enough to repeat from `tools/`.
**Question asked:** is the controller *robust* — not only accurate on a good day — and what does the
evidence say about how it fails.

---

## 1. Verdict

| | Mission 0007 (`172b047`) | Mission 0001 (`7f9651d`) |
|---|---|---|
| Completed | yes, `COMPLETED` by RPP | yes (disarm + MANUAL release confirmed in the ULog) |
| Straight-line tracking (RPP cross-track, mid-leg at 0.55–0.60 m/s) | p50 0.1–0.9 cm | p50 0.2–0.9 cm |
| Corner-entry error (first 0.5 m after a pivot) | 0.2–1.7 cm, signed | **1.0–3.2 cm**, signed, one sign per leg |
| Braking-tail error (last 1 m before a corner, 0.2 m/s) | 1.5–3.9 cm | 1.0–2.3 cm |
| Corner point capture error (`PointResult.error_m`) | 0.4 / 1.7 / 3.6 cm | 0.1 / 2.7 / 1.9 / 1.8 cm |
| Final endpoint | 0.4 cm (CREEP 0.3 s, then STOPPING 7.6 s) | 0.7 cm, **CREEP ended by its 8 s timeout** while rocking ±8 cm/s |
| Pivot settle hunting (PIVOT↔STOPPING cycles per corner) | 1–3 | 1–4 |
| Chain latency `pose_to_write_age` (PX4 sample → write to PX4) | p50 3.1–7.5 ms, p95 10–11 ms, max 25 ms | **p50 3.2–3.5 ms, p95 4.2–4.5 ms, max 14.5 ms** |
| RPP cadence (setpoint dt) | p99 25.4 ms, max 28.7 ms | p99 21.5 ms, max 29.6 ms |
| RPP overruns (lifetime) | 32 | 1 |
| Guard refusals / clamps | 0 / 0 | 0 / 0 |
| px4_link faults | overrun only (8–11) | overrun only (1–2); 0 gap events, 0 resets |
| Drivetrain | 1 RoboClaw error in 64 s | **2 822 RoboClaw errors in the first 26 s; the rover did not move for 25.8 s while commanded to pivot; nothing in the stack reacted** |

**Verdict.** On straights the controller is accurate and the timing chain is healthy — the
`7f9651d` stack is measurably better than `172b047` (latency p95 4.4 vs 10.4 ms, 1 vs 32 overruns).
The robustness problems are at the **corners, the endpoints and the drivetrain**, and they are of
three kinds: (A) a drivetrain fault is invisible to the stack and leaves the rover stalled or, worse,
moving late; (B) the pivot fine-alignment cannot be executed by the drivetrain, so every corner
starts 1–3 cm off and the FSM hunts; (C) the endpoint precise stop rocks and finishes on a timeout.
None of these is a code-quality defect; they are controller/plant interactions, and the first is a
safety-relevant gap.

---

## 1b. Tracking in detail — commanded vs achieved, oscillation, scatter, swing

Computed over the steady part of every TRACK leg (commanded speed ≥ 0.5 m/s; the legs are only
~2.5 m, so 1.3–2.3 s each), from the 50 Hz bag topics (`RppStatus`, `MotionSetpoint`,
`VehicleState`).

| Quantity (steady straight, 0.55–0.60 m/s) | Mission 0007 (4 legs) | Mission 0001 (4 legs) |
|---|---|---|
| Cross-track mean per leg (signed) | +1.0 / −0.0 / −0.8 / −0.7 cm | −0.9 / +1.5 / +0.8 / +0.1 cm |
| Cross-track std / peak-to-peak | 0.25–0.49 / 1.2–1.9 cm | 0.27–0.77 / 0.9–2.5 cm |
| Cross-track zero crossings per leg | 1–5 | 1–3 |
| Heading error mean per leg (signed) | −0.28 / −0.24 / +0.35 / +0.66° | +0.34 / +0.07 / −0.64 / −0.70° |
| Heading error std / p2p | 0.13–0.30° / 0.4–1.3° | 0.19–0.36° / 0.9–1.6° |
| Heading zero crossings per leg (dither period) | 3–12 (≈0.3–0.9 s) | 3–13 (≈0.3–0.5 s) |
| Achieved heading lag behind `yaw_setpoint` | 0–420 ms (corr 0.58–0.94) | 0–200 ms (corr 0.54–0.95) |
| `yaw_setpoint` step per 20 ms tick (command smoothness) | std 0.0004–0.0006 rad, max 0.0027 rad (0.15°) | std 0.0004–0.0007 rad, max 0.0022 rad |
| Speed: command − measured | mean +0.5…+1.9 cm/s, std 1.3–2.8 cm/s | mean +0.3…+1.3 cm/s, std 1.7–3.3 cm/s |
| Measured yaw rate on the straight | std 0.016–0.039 rad/s, p2p 0.08–0.23 rad/s, 12–20 zero crossings per leg | std 0.025–0.040 rad/s, p2p 0.12–0.24 rad/s, 8–20 zero crossings |
| corr(cross-track, heading error) | +0.63 / −0.50 / −0.53 / −0.66 | +0.70 / −0.45 / +0.84 / +0.73 |

**Reading.**
- **No limit cycle on the straights.** Cross-track crosses zero 1–5 times per leg and its
  "dominant period" equals the leg duration: the error is a slow first-order correction of the
  corner-entry offset (1–3 cm, Finding B) plus a small per-leg bias, not an oscillation. The
  heading dithers at ±0.3–0.5° with a 0.3–0.9 s period around a **per-leg bias of 0.3–0.7°** that
  changes sign from leg to leg (so the corr sign flips). A steady heading bias with the
  cross-track still converging is what a heading-mode tracker with a lookahead does: the yaw
  setpoint is offset to pull the cross-track in, and the achieved heading follows it with 0–0.4 s
  lag. There is no left-right swing beyond that dither; the signed means above are the "swing".
- **RPP's commands are smooth** (yaw setpoint moves ≤ 0.15° per tick, speed command std ≈
  measured std). **The fast content is in the measured yaw rate**: ±0.1 rad/s, 12–20 zero crossings
  in ~2 s ≈ 5 Hz. RPP cannot produce it (its command changes 0.03°/tick); it is the PX4 yaw-rate
  loop and the RoboClaw drivetrain (`RO_YAW_RATE_P/I`, wheel-speed quantisation, the deadband
  patches B1 #7/#8). It is visible in the 0.3 cm cross-track scatter and in the ±0.5° heading
  dither; it is not what limits accuracy today — the corner entry and the braking tail are.
- **Commanded vs achieved speed** tracks within 1–2 cm/s mean, with ~8 % overshoot at the
  plateau (PX4 `RO_MAX_THR_SPEED`); the ramp to 0.6 m/s is PX4's `RO_ACCEL_LIM` 0.5 m/s².
- **What accuracy on a straight is actually limited by:** (1) the entry offset from the pivot
  (Finding B), (2) the braking tail before each corner (1.0–3.9 cm signed drift in the last metre
  at 0.2 m/s, where the heading-mode tracker barely corrects at low speed), (3) a 0.3–0.7° heading
  bias per leg. Items (1) and (2) are controller structure; (3) is partly drivetrain asymmetry.

## 1c. Rates — what the controller actually runs at, and what it is fed

Instantaneous rates from consecutive message times in the bags (all runs, both missions).

| Signal | Mean over a run | Instantaneous min / p50 / max | Worst gap |
|---|---|---|---|
| PX4 EKF2 `vehicle_local_position` sample stamps (`VehicleState.px4_sample_stamp`) | **50.0 Hz** | 33.3 / 50.0 / 100 Hz — PX4 alternates 30 ms and 10 ms steps (EKF2 output on the 10 ms IMU grid) | 30.0 ms |
| `/dyx3/vehicle_state` at RPP (receipt) | 50.0 Hz | `7f9651d`: 42–45 / 50.0 / 54–57 Hz; `172b047`: 36–39 / 50 / 68–72 Hz | 22–30 ms |
| `/dyx3/rpp/motion_setpoint` (RPP tick output) | **50.0 Hz** | `7f9651d`: 33.7–44 / 50.0 / 53–57 Hz; `172b047`: 36–39 / 50 / 68–105 Hz (double ticks) | 29.6 ms (one tick) |
| `/dyx3/motion_guard/command` (to px4_link) | 50.0 Hz | as RPP ± 1 Hz | 27.8 ms |
| px4_link → PX4 (`offboard_control_mode` + the setpoint set) | 100 Hz timer plus one write per guard command (design) | **not measurable from these logs**: the PX4 logger decimates `offboard_control_mode` / `rover_*_setpoint` to 10 Hz and `/fmu/**` is not bagged | — |
| `/dyx3/px4_link/status` | 9.5 Hz | 9.0 / 9.3 / 10.1 | 111 ms |

**Control rate:** RPP ticks once per new PX4 sample, event-driven, so the control loop *is* PX4's
50 Hz local-position cadence; the guard decides on each RPP command; px4_link writes on each guard
command and keeps a 100 Hz heartbeat between them. On `7f9651d` the cadence is tight (p1–p99
46–54 Hz, 1 overrun in 88 s); `172b047` showed double ticks (p99 68 Hz) and 32 overruns. The
33/100 Hz p1/p99 at the PX4 end is PX4's own output jitter, not transport.

**Pose input to RPP:** `/dyx3/vehicle_state`, built by px4_link from PX4 EKF2's
`/fmu/out/vehicle_local_position_v1` (NED `x, y, vx, vy, heading, xy_reset_counter`, validity flags),
`vehicle_attitude` (quaternion; the yaw rate is derived on the Jetson from consecutive attitude
samples, 50 ms low-pass) and `vehicle_status`. It is the **EKF2 fused estimate** (RTK position +
dual-antenna heading + wheel encoders + IMU, lever arms inside EKF2), not the raw GNSS and not
the antenna. RPP ages it from receipt (0.5 s limit), not from the sample stamp; EKF resets are the
mission's business (pause), RPP still carries the prototype's jump heuristic.

**To see what PX4 receives at 100 Hz:** raise the SD logger rate for `offboard_control_mode`, or
add `/fmu/in/offboard_control_mode` to the recorder's topic list for timing runs (it is excluded
by design today). One of the two is needed before the timing contract can be closed on evidence.

## 2. Finding A — a dead RoboClaw link is invisible to the whole stack (mission 0001)

> Owner note 2026-10-10: the RoboClaw had been powered off by hand and nobody noticed — the
> outage itself was not a fault of the firmware or the rover. What stands is only that the stack
> could not see it (every gate green for 26 s under a motion command); the firmware hypothesis
> below is withdrawn.

**What happened.** `vehicle_status` shows OFFBOARD at t = 0.72 s. RPP goes `STOPPING → PIVOT` at
0.73 s with a −1.503 rad (86°) heading error and commands −0.45 rad/s. The PX4 log then shows, for
the next 25.8 s:

- `[roboclaw] ACK timeout` ×1402, `Error reading encoders` ×706, `Select timeout 0` ×703 — about 110
  failed UART transactions per second, in every 5 s bin from 0 to 25 s, then **none** for the rest of
  the mission;
- `wheel_encoders.wheel_speed` NaN, `rover_rate_status.measured_yaw_rate` 0.000, local position
  constant at (0.98, −1.57), `actuator_motors.control[0]` mean 0.215 (PX4 *was* commanding the
  motors);
- at ~24–26 s the link recovers by itself, the wheels spin (+1.0 / −1.0 rad/s), the pivot executes
  (−0.37 rad/s measured) and the mission proceeds normally. The `estimator_aid_src_wheel_encoder`
  reports `fused = 1.00` for the whole log, i.e. the aid source kept fusing while the driver was
  failing to read the encoders.

**What the stack did.** RPP stayed in `PIVOT` for 25.8 s. Its pivot watchdog (`segment_turn_timeout_s`
5 s, cap `segment_pivot_timeout_max_s` 9 s) *did* fire, but the only effect of a timeout is to widen
the release band from 2° to 3° (`stop_pivot_fsm.cpp:120-124`, `release_max` 3°); with an 86° error
the FSM simply keeps pivoting, with no fault, no state change and no reason code (`RppStatus` has
no field for it; `tick_state` stayed 1). The guard saw a fresh pose, a fresh link, RTK fixed, an
armed OFFBOARD vehicle — every gate OK. PX4 raised no failsafe. The mission stayed `RUNNING`. The
operator saw a rover doing nothing for 26 s, then starting to turn on its own.

**Why it matters.** Two failure modes, both real: (1) *stall without a reason* — a mission that hangs
in PIVOT indefinitely with a green dashboard; (2) *late motion* — a vehicle that has been "doing
nothing" for half a minute starts moving when the link recovers, which is exactly when someone walks
up to it. The same mechanism would hide a RoboClaw failure mid-line (the rover coasts to a stop at
`RO_DECEL_LIM` while RPP keeps commanding 0.6 m/s until the endpoint timeout).

**Root cause (hypothesis, outside this repo).** The signature — every transaction failing with
`Select timeout 0` at ~110 Hz, then spontaneous recovery — matches F-tasks B1 #6 (`129cfc40ae`,
"raw mode + `select()` `fd_set` reuse bug": `FD_ZERO`/`FD_SET` once, `select()` in the loop; after one
timeout the set is cleared and every later `select()` returns 0 immediately). Whether that patch is in
`8279fa4be3`, and why the link recovered, must be checked in the firmware tree. HANDOFF had already
seen "[roboclaw] ACK timeout (3 seen)" on 2026-10-10 (open item 8) — this log shows it is not rare.

**What the companion must do regardless of the firmware fix** (proposals, owner decision):
1. **An actuator-plausibility gate in `dyx3_motion_guard`** (the guard "never invents a correction",
   but it may refuse): commanded |yaw rate| ≥ `segment_nominal_pivot_rate_rad_s` (0.4) or speed ≥ a
   fraction of `mission_speed`, with measured yaw rate and speed ≈ 0 (`VehicleState.yaw_rate_radps`,
   velocity) for longer than the pivot spin-up margin (`segment_pivot_spinup_margin_s` 1.0 s) → STOP
   with a new reason (`REASON_ACTUATOR_STALL`, interfaces bump) and mission PAUSED. All thresholds
   are existing parameters; no new number.
2. **RPP must surface a pivot timeout**: `RppStatus` gains `pivot_timed_out` (or `handoff`-style
   code) and the mission pauses on it with `REASON_RPP_ERROR`-class detail, instead of pivoting
   forever.
3. PX4-side: expose RoboClaw health (`wheel_encoders` validity or a driver status) and gate arming on
   it; this is firmware (F-tasks B1 #5–#8 are exactly the RoboClaw patches).

## 3. Finding B — the fine pivot cannot be executed, so every corner starts 1–3 cm off

**Evidence.** At every corner the FSM reaches a residual heading error of −0.034 to −0.054 rad
(2.0–3.1°) and then hunts: `PIVOT → STOPPING → PIVOT → STOPPING …` 1–4 times with `yawrate_cmd`
−0.05 to −0.08 rad/s, measured yaw rate ≈ 0, wheel speeds ≈ 0 (mission 0001 run 1: four cycles
between 4.19 s and 5.03 s, heading error pinned at −0.052 rad throughout). The command is the P-law
`segment_yaw_rate_gain` 1.5 × error (1.5 × 0.052 = 0.078 rad/s): below what the RoboClaw/PX4 rate
loop actually produces (full-rate pivots track at 0.82–0.99 of the command; small ones at 0).
Release happens when the error sits inside the 3° band after the timeout (`segment_heading_tolerance_deg`
2.0, `segment_timeout_heading_tolerance_deg` 3.0, `segment_pivot_release_max_deg` 3.0), so the leg
starts with ~3° of heading error. The signed cross-track in the first 0.5 m is then +0.9 / +1.5 /
−2.4 / −2.8 cm (mission 0001) and −0.2 / −1.1 / +1.2 / +1.6 cm (0007), converging below 0.5 cm by
0.5–1.0 m. That is the largest single contributor to the p95 numbers.

**Cost.** 1–2.6 s per corner of hunting, and a 1–3 cm cross-track error over the first ~0.7 m of
every painted leg — a visible defect at the corner of every square.

**Candidates (tuning/design, owner decision; no number invented here):**
- a floor on the pivot rate at the drivetrain's minimum effective rate (measure it: the smallest
  commanded yaw rate that produces motion — from these logs it is between 0.08 and 0.37 rad/s), with
  a short pulse-and-measure strategy for the last degrees instead of a proportional crawl;
- or tighten the release band only if the floor makes it reachable (today a 2° band is not reachable
  by a 0.08 rad/s command);
- the prototype's "pivot to intercept" (`pivot_to_intercept_enabled` 1, `pivot_intercept_dist_m`
  0.35) is on: check whether it is what leaves the 2–3 cm lateral offset after alignment, since the
  pivot aims at an intercept point rather than the leg.
- PX4 side: the rate loop's response to small setpoints (`RO_YAW_RATE_P/I`, RoboClaw deadband patches
  B1 #7/#8) decides whether a floor is needed at all.

## 4. Finding C — the endpoint precise stop rocks and finishes on its timeout (mission 0001 run 3)

**Evidence.** `CREEP` entered at `dist_to_goal` 0.58 m, reached 0.004 m within 1.5 s, then for 6.5 s
commanded speeds alternating between +0.07 and −0.085 m/s with `dist_to_goal` oscillating 0.000–0.007
m; the state ended at exactly 8.0 s (the precise-stop timeout) with 0.7 cm remaining. Cross-track
during CREEP was 0.2 cm p50 (the lateral fix `5965d90` works). Mission 0007 (older stack) did not
show this: CREEP lasted 0.3 s and STOPPING 7.6 s, final error 0.4 cm.

**Mechanism (exact, from `RppCore::precise_stop_tick`).** Finishing needs `|residual| ≤
segment_endpoint_arrival_tolerance_m (0.02)`, `|cross| ≤ 0.02` **and** "stopped" (measured speed
< `segment_stop_speed_threshold` 0.02 m/s and yaw rate < 0.05 rad/s for `segment_stop_dwell_s`
0.3 s). But while not stopped the law commands `speed = min(√(2·decel·|residual|), max(speed,
creep))` with the **sign of the residual**, for any non-zero residual: 0.084 m/s at 1 cm, 0.046 at
3 mm, 0 only exactly on the plane. The vehicle (speed loop + drivetrain, 0.1–0.3 s) overshoots,
the sign flips, the command reverses — 14 reversals in 8.6 s, measured speed up to 12.5 cm/s with
0–1 cm to go — and "stopped" is never satisfied until the 8 s timeout's brake (XR-RPP-001) ends
it. The two stacks run identical RPP code here (only telemetry fields changed between `172b047`
and `7f9651d`); mission 0007 happened to satisfy the gate on its first pass.

**Fix (this branch, `dyx3_rpp`, BEHAVIOUR CHANGE, only existing parameters):** (1) inside the
arrival band the command is a brake (zero speed, heading held), not a creep, so the stop can be
confirmed; (2) the brake is held while `|residual| ≤ along_tol + endpoint_capture_past_m` (0.02,
the prototype's capture-past allowance reused as hysteresis) so a few millimetres of coast do not
re-arm the creep; (3) the feed-forward speed outside the band is evaluated to the band edge
(`|residual| − along_tol`) instead of to the plane, so it reaches zero where the brake takes over.
The 8 s timeout stays as the backstop. Tests: tick-level reproduction of the old command inside
the band, and a closed-loop first-order vehicle model that must complete within 4 s with ≤ 2 sign
changes. **Field re-validation at the next endpoint is required** (this is a control-law change).

## 5. What is healthy (measured, do not re-litigate)

- **Timing chain (closes review M1 / X-003 / X-006 doubt):** PX4 sample → `/fmu/out` → px4_link →
  `/dyx3/vehicle_state` → rpp → motion_guard → px4_link → `/fmu/in` = **3.2–3.5 ms p50, 4.2–4.5 ms
  p95, 14.5 ms max** on `7f9651d`, with px4_link under normal scheduling. The 10 ms p95 on `172b047`
  came from a busier Jetson (32 RPP overruns), not from the design. PX4's local-position cadence is
  20 ms with 30 ms gaps at p99 (PX4 side). The event-driven chain works as designed; giving px4_link
  RT priority is still right for the tail (P4), but the single-container composition (PC-2) is not
  needed for latency.
- **Fail-safe chain:** never triggered (0 guard refusals, 0 gap events, 0 session resets, no
  estimator rejections, fix 6 throughout, hrms ≤ 2.3 cm, corrections age ≤ 0.37 s).
- **Speed loop (PX4):** |setpoint − measured| mean 1.2–2.4 cm/s; 8 % overshoot at the 0.6 m/s
  plateau (0.65 measured) — `RO_MAX_THR_SPEED` tuning, not a defect.
- **MANUAL release (`fe1770b`, open item 6) proven live:** mission 0001 ULog shows `DO_SET_MODE
  (1,1)` at 87.20 s, ACK accepted at 87.24 s, `nav_state` 14 → 0 at 87.25 s after the disarm at
  86.90 s. Close the item.
- **ULog over DDS is not usable:** every run summary reports ~1 000 gaps per 40 s and "no header";
  the SD log is the evidence source. `ulog_streaming_enabled` is turned off by default in this
  branch.
- The 0.36–0.43 s gap in *all* ULog topics at t ≈ 0.9 s of each log is the SD logger starting; not a
  transport event.

## 6. How to fix each issue (root cause → fix → where)

| Issue (measured) | Root cause | Fix | Where / status |
|---|---|---|---|
| Endpoint rocking (14 reversals, timeout finish) | creep law commands `√(2·a·|residual|)` with the residual's sign inside the 2 cm band; "stopped" never satisfied | brake inside the band; hold with `endpoint_capture_past_m` hysteresis; feed-forward to the band edge | `dyx3_rpp`, **done on this branch** (`fb74658`); field re-validate at the next endpoint |
| Corner entry 1–3 cm, pivot hunting 1–4 cycles | fine pivot `1.5 × 2–3° = 0.05–0.08 rad/s` is below the drivetrain's minimum effective rate; FSM dithers at the 2°/3° release band | measure the minimum rate that moves the rover (0.08–0.37 rad/s from these logs); command `max(gain·err, floor)` with a short pulse for the last degrees; only then tighten the band — or fix the deadband in PX4 so small rates execute | GATE 4 tuning (owner value); PX4 `RO_YAW_RATE_I`, RoboClaw deadband patches B1 #7/#8 |
| Braking-tail drift 1–4 cm in the last metre | at 0.2 m/s the heading-mode tracker's lateral correction is weak | the `heading` vs `rate` A/B (`segment_command_mode`, review §8.1) — RPP's own yaw-rate law with κ·v feed-forward; or a speed-scheduled lateral-gain floor at approach speed | GATE 4 A/B, no default change without the number |
| Per-leg heading bias 0.3–0.7° | steady yaw offset: PX4 attitude P-loop without integral action on the rate error, plus drivetrain asymmetry | PX4 `RO_YAW_RATE_I`; per-wheel RoboClaw calibration (`RBCLW_*`), checked on a constant-speed straight; RPP already compensates through the lookahead | PX4 tuning (PC-5) |
| 5 Hz yaw-rate dither ±0.1 rad/s | PX4 rate loop / wheel-speed quantisation at 50 Hz encoder reads | lower `RO_YAW_RATE_P` or filter the measured rate (`RO_YAW_RATE_TH`, encoder-speed LPF); verify on `rover_rate_status` | PX4 tuning; harmless today |
| Speed overshoot +8 % at the plateau | `RO_MAX_THR_SPEED` feed-forward scale | set from the measured 0.96 m/s full-command speed | PX4 param, already proposed |
| "Commanded but not moving" invisible (any cause) | no actuator-plausibility check in the chain | guard gate: commanded rate ≥ nominal pivot rate (or speed ≥ a fraction of `mission_speed`) with measured ≈ 0 for longer than `segment_pivot_spinup_margin_s` → STOP + reason; RPP surfaces its pivot timeout; the mission pauses on it | owner decision (new reason code → interfaces bump) |
| PX4-side receive rate not observable | logger decimation; `/fmu/in` not bagged | full-rate `offboard_control_mode` in the SD profile, or bag `/fmu/in/offboard_control_mode` for timing runs | recorder topic list, small |

## 7. Actions, ranked

1. **Endpoint stop:** the dead-band/brake-hold fix is on this branch (§4); re-validate at the next
   endpoint (expect ≤ 2 reversals, completion within ~2 s of entering the band, no timeout finish).
2. **Corner alignment (GATE 4 tuning):** measure the minimum effective pivot rate, decide floor vs
   band (§3). Until then, expect 1–3 cm at every corner entry and 1–2.6 s of hunting per corner.
3. **Braking tail:** the last metre before a corner drifts 1–4 cm at 0.2 m/s (§1b); the tracker's
   lateral correction at low speed is the next structural item after the corners.
4. **Companion (owner decision, interfaces bump):** actuator-plausibility gate in the guard and a
   pivot timeout surfaced by RPP (§2) — the stack must be able to tell "commanded but not moving"
   whatever the cause (a powered-off RoboClaw included).
5. **Evidence:** run the circle/arc step as the `heading` vs `rate` A/B (review §8.1) — these two
   missions prove the straight-line regime only; and the 5 Hz yaw-rate dither (§1b) is the PX4
   rate loop's to tune, not RPP's.
