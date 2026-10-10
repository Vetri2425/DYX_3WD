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

## 2. Finding A — a dead RoboClaw link is invisible to the whole stack (mission 0001)

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

**Mechanism.** The capture needs "stopped" (speed < `segment_stop_speed_threshold` 0.02 m/s) and the
along-track residual within tolerance at the same time; the controller keeps commanding ±0.05–0.085
m/s (`endpoint_approach_speed` class, reverse allowed to −0.10) for a residual of a few millimetres,
so the vehicle never counts as stopped. The outcome (7 mm) is acceptable, the path to it (6.5 s of
rocking at the endpoint, finish by timeout) is not robust: on a slope or with a sticky drivetrain the
rocking can grow, and the spray OFF at the end of a MARK run happens during it.

**Candidates:** a dead-band on the along-track residual below which the command is zero and the stop
is confirmed on the measured speed only (the register's XR-RPP-001 recommendation: "finish when
stopped and |residual| ≤ along_tol"); the residual tolerance exists (`point_capture_radius_m` is 0.10
at mission level; RPP's own along-track tolerance is the parameter to use). Owner decision at GATE 4.

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

## 6. Actions, ranked

1. **Firmware:** confirm whether B1 #6 (`select()` fd_set reuse) is in `8279fa4be3`; read the RoboClaw
   driver's recovery path; treat `[roboclaw] ACK timeout` as a pre-arm blocker on the rover until
   understood (open item 8 is now a blocker, not a watch).
2. **Companion (owner decision, interfaces bump):** actuator-plausibility gate in the guard; pivot
   timeout surfaced by RPP and acted on by the mission (§2).
3. **Corner alignment (GATE 4 tuning):** measure the minimum effective pivot rate, decide floor vs
   band (§3). Until then, expect 1–3 cm at every corner entry.
4. **Endpoint (GATE 4):** along-track dead-band in the precise stop (§4).
5. **Evidence:** run the circle/arc step as the `heading` vs `rate` A/B (review §8.1) — these two
   missions prove the straight-line regime only.
