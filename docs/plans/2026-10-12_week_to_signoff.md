# Week plan to sign-off — Mon 2026-10-12 → Sat 2026-10-17

Owner schedule (decided 2026-10-11):

| Day | Date | Goal |
|---|---|---|
| **Monday** | 2026-10-12 | **Last day for RPP**: 100 Hz control path, the review items, yaw and speed fine tuning, the RPP field ladder closed |
| **Tuesday** | 2026-10-13 | **Spray and flow control** |
| **Wednesday** | 2026-10-14 | **Full controller mission: the first production demo** |
| Thursday | 2026-10-15 | Fixes from the demo, soak, app items |
| Friday | 2026-10-16 | Customer readiness: signing, protection, parameter freeze, clean reinstall, docs |
| **Saturday** | 2026-10-17 | **Project sign-off** |

Starting point: DYX_3WD `master` `0157792` (CI green, release published, **not installed**); rover 01 runs
`rover-7f9651d48f`; firmware `8279fa4be3`; app `Trajectory` `0d8225c`. The numbered open items (1–28) are in
`docs/agents/HANDOFF.md`, "Open items from 2026-10-10"; review IDs (PXL-, MS-, PC-, X-) are in
`docs/reviews/production/open_items.md`.

Rules for the week: rover motion only with the owner present; every PX4 `param set` after a disarmed read; one
variable per test; every run recorded under `3WD_PROD/Recorder/<DD-MM-YYYY>/<NN_session>/{Bags,Logs}`.

---

## Monday 2026-10-12 — last day for RPP

### M0. Before any motion (bench, wheels up)

- [ ] Owner decision, then fix: **an armed, idle rover after a `dyx3-services` restart is never disarmed**. Proposal:
      px4_link disarms on start when PX4 is armed, no mission owns it and px4_link did not arm it.
- [ ] `fcu_param_dump_enabled=false` by default until a bench read shows `/fmu/out` rates and `pose_to_write_age`
      unchanged during the read (item 27).
- [ ] Upgrade rover 01 to the `0157792` release, rover idle. `dyx3-health --deep` OK; px4_link handshake OK; every
      `/fmu` topic visible; nothing on the LAN (Fast DDS profile, localhost-only); px4_link runs FIFO 70 (item 27).
- [ ] Unit-split fault injection: kill `dyx3-services` while armed → guard STOP, heartbeat kept, the new disarm rule;
      kill `dyx3-control` → STOP burst, PX4 disarms; the E-stop latch survives a services restart; XRCE agent loss,
      Ethernet cable pull, Jetson power loss, RTK and heading loss: measure the actual wheel stop for each.
- [ ] Stall gate: one pivot from standstill and one straight start must not trip `REASON_ACTUATOR_STALL`.
- [ ] Confirm the MANUAL release live: "leaving OFFBOARD: MANUAL requested" → "arm ok", RC on (item 6).

### M1. HIGH PRIORITY — 100 Hz control, input and streaming; a more reliable path

Today the control loop is PX4's **50 Hz** local-position cadence (measured, analysis 2026-10-10 §1c). EKF2 already
predicts at 100 Hz (`EKF2_PREDICT_US` 10000); the 50 Hz ceiling is the firmware's `dds_topics.yaml`
(`vehicle_local_position` `rate_limit: 50`).

Input (firmware, `dyx-3wd-production`; CI build, archive in `PX4-Firmware/3WD/`, flash only on the owner's word):
- [ ] `vehicle_local_position` 50 → **100 Hz**; `vehicle_attitude` 100 Hz; enable `vehicle_angular_velocity` 100 Hz
      (native yaw rate instead of differencing attitude); `estimator_status_flags` 5 → 50 Hz; add
      `estimator_aid_src_gnss_yaw` (PC-2b). `vehicle_status` stays event-rate.
- [ ] Firmware pin bump in the installer (PC-9) and px4_msgs rebuild only if `msg/` changes (it should not).

Control (companion):
- [ ] RPP ticks once per sample, so 100 Hz input gives a 100 Hz tick: prove the step fits the 10 ms budget under
      load (M4: RPP `step()` duration p99 with the recorder and backend busy).
- [ ] Guard and px4_link at 100 Hz: staleness limits re-derived from the **measured** 10 ms period (2–3 periods), never
      guessed.
- [ ] `segment_command_mode` `heading` vs `rate`: the yaw-rate path is the reason for the DDS migration; decide it by
      the A/B in M3.

Streaming and measurement:
- [ ] Measure the write into PX4 (item 24): bag `/fmu/in/offboard_control_mode` for one timing run or raise the SD
      logger rate; record `pose_to_write_age` p50 / p99 at 100 Hz.
- [ ] Recorder bags the control topics at full rate (check disk and `mission_bag_metrics.py` at 100 Hz).
- [ ] Tablet telemetry stays at **10 Hz (20 Hz at most)**: the app's JS thread was the cause of the link drops;
      100 Hz to the tablet buys nothing and costs the link.

Reliable path:
- [ ] XRCE-DDS over Ethernet at 100 Hz: per-topic gap count and worst gap for 30 min; zero session resets.
- [ ] Agent restart recovery (STEP1-R1, 15.8 s against 5 s) and agent `SIGSTOP` with OFFBOARD active (M2: does the
      10 ms writer `max_blocking_time` hold?).
- [ ] QoS end to end for `/fmu/in` setpoints (what px4_link declares vs what the agent offers).
- [ ] 60-restart stress of `dyx3-control` with the new units (cloud-phase item 22).

### M2. The review items (Fable's report and the branch review)

Bench checks of what has never run on the rover (item 27): persisted progress + resume + re-engage (pause → kill
services → restart → resume; deliberate OFFBOARD loss → resume; the record survives a reboot); interfaces 0.17.0 in
the app (item 28: `resume=true`, show `resumed_run_index`, the pivot-timeout and stall reasons).

Should-fix items awaiting the owner's decision (each: decide, then fix or record "accepted risk"):
- [ ] armed vehicle with no owner after a services restart (M0);
- [ ] no entry alignment on a resumed boundary below 45°;
- [ ] run index not tied to the conditioning;
- [ ] a fresh start erases the saved progress;
- [ ] FCU parameter read default (M0);
- [ ] stall thresholds copied from live RPP parameters at start-up;
- [ ] `dyx3-param save` pins every default (and ignores a directory-sync failure): verify save → restart → value.

Smaller review items (Codex customer-readiness review 2026-10-11): pin the RPP equivalence deviations per scenario
and window, not the aggregate 373; the mission progress writer advances memory before the durable write and drops a
failed batch — define what the operator sees and write first.

Other open review items: telemetry seq gaps on the tablet (G1 shipped; show drops); RTK `rtk_state` rover event
(item 13); recorder parameter dump overlap (item 27).

### M3. Fine tuning: yaw and speed control, then close the RPP ladder

- [ ] Fit the drivetrain response from the bags (commanded vs measured speed and yaw rate): first-order τ and dead
      time. The endpoint stop is 0.3 mm on the rate-limited plant but 2.2–17.5 mm on the first-order lag model
      (item 25): the fitted τ decides which is real.
- [ ] Speed loop (PX4 `RO_SPEED_P/I`, `RO_ACCEL_LIM` 0.5, `RO_DECEL_LIM` 2.0, `RO_JERK_LIM` 4): Offboard steps
      0.2 → 0.6 → 0.8 m/s; overshoot and settling recorded.
- [ ] Yaw loop (PX4 `RO_YAW_RATE_P/I`, `RO_YAW_P`): rate steps and pivots; then the `heading` vs `rate` A/B on the
      circle with everything else fixed.
- [ ] RPP ladder (owner order): square ✅ → **circle / arc** (A/B above) → **multi-shape** → **speed steps** 0.6 → 0.8
      (→ 1.0) m/s with curvature slowdown → **pivots, extensions, mark/transit split** → **endpoint ≤ 1 cm,
      ≤ 2 reversals, no timeout finish**.
- [ ] Gate-table numbers from `tools/analysis/mission_bag_metrics.py` for every accepted run.
- [ ] End of day: **RPP frozen** — the accepted values in `/etc/dyx3/rpp.yaml`, the FCU set saved with `dyx3-param`,
      both committed under `config/`.

---

## Tuesday 2026-10-13 — spray and flow control

- [ ] Independent valve close when every Jetson path is lost (X-012 / C3): spray watchdog and PWM disarm proven on
      the bench.
- [ ] Spray boundary defect: `projection_direction_gate_deg` 0 ships enabled (S3) — fix before paint.
- [ ] Spray timing: valve open/close latency measured; lead/lag compensation from the measured latency.
- [ ] Flow by speed: flow follows `RppStatus` speed (speed regulation from Monday); minimum speed for paint.
- [ ] The five unported spray features (architecture audit §8): port or record "not in V1".
- [ ] Paint trials: line start/end, dashed lines, corners, mark/transit split; measure paint loss and overspray.
- [ ] End of day: spray parameters frozen in `/etc/dyx3/spray.yaml`, committed.

## Wednesday 2026-10-14 — full controller mission, the first production demo

- [ ] Tablet → plan → upload → start: a multi-shape mission with spray at production speed, RPP and spray frozen.
- [ ] Operator stops: tablet E-stop assert/clear, RC kill, pause/resume, resume after a services restart.
- [ ] Every run recorded; gate table from `mission_bag_metrics.py` (cross-track p50/p95, endpoint, coverage, paint
      loss); demo report in `docs/field/`.

## Thursday 2026-10-15 — fixes from the demo, soak, app

- [ ] Fix what the demo found (one commit per finding, CI green).
- [ ] Soak ≥ 4 h (M5): link gaps, overruns, memory, temperatures.
- [ ] App: `resume=true` (item 28), aligned entry (17), Fields/import freeze (18), RTK profile contract (19), spray
      REST polls → events (20), dead code (21); release APK.
- [ ] Owner decisions: which app branch becomes `main` (3); old tablet .106 (4).

## Friday 2026-10-16 — customer readiness

- [ ] Release signing (minisign) and branch protection + 2FA (PC-6; decide how it fits direct-to-master).
- [ ] FCU parameter baseline frozen and bench-checked (PC-5), committed as the production profile.
- [ ] Clean reinstall on the Jetson: wipe `/opt/dyx3`, install the signed release, < 15 min, zero compiler runs.
- [ ] Prototype A/B options removed from `dyx3_rpp` (PC-3) once Monday's values are frozen; `dyx3_rpp_legacy` at
      GATE 7: keep as the oracle or delete.
- [ ] Docs: README, CLAUDE.md, contracts and the open-item register match the signed release.
- [ ] Owner decisions: stop policy (1), NTRIP over HTTP (2), gate3/4 generators (5), field network (12).

## Saturday 2026-10-17 — sign-off

- [ ] Acceptance checklist: the gate-table numbers from Wednesday's demo and the soak, every CRITICAL/HIGH item
      closed or accepted by the owner in writing, the signed release on rover 01.
- [ ] Hand-over: release, firmware, app APK, parameter profiles, this plan with every box ticked or explained.
