# Production review — open items

Part-by-part production-readiness review of the 3WD stack. Each part is reviewed externally (ChatGPT, CodeRabbit
style, one prompt per part), then **every finding is verified by Claude against the code** before it is
recorded here. Findings are not accepted on the reviewer's word: severities are re-rated against the real
producers, defaults and call paths.

Goal the review is measured against:
- ultra-low, deterministic latency from pose to setpoint;
- high-rate pose input and high-rate RPP mission control;
- no hang, stall or silent degradation; fail to STOP on any fault;
- about 1 cm cross-track accuracy on straights and arcs.

Review baseline: DYX_3WD `master` `252778e` (firmware `8279fa4be3`).

## Severity

| Severity | Meaning | Must be fixed before |
|---|---|---|
| **CRITICAL** | Unsafe or uncommanded motion or paint; hang or stall with no STOP; wrong sign or frame | any autonomous field run |
| **HIGH** | Breaks the latency, rate or determinism goal; wrong recovery; misbehaves in plausible field conditions | production |
| **MEDIUM** | Degraded performance, robustness or observability; contract and code disagree | V1 sign-off |
| **LOW** | Hardening, clarity, minor efficiency, documentation | when convenient |

## Verification status

| Status | Meaning |
|---|---|
| **ACCEPTED** | Confirmed in the code at the stated severity |
| **ACCEPTED ↓ / ↑** | Confirmed, but re-rated (the reviewer's severity is shown struck through) |
| **DOUBT** | Plausible, not proven: needs a measurement or a decision before it is rated |
| **REJECTED** | Does not hold against the code; the reason is recorded |
| **FIXED** | Closed by a commit (SHA recorded) |

## Part tracker

| # | Part | Reviewed | Verified | Open (C/H/M/L) | Status |
|---|---|---|---|---|---|
| 1 | `dyx3_rpp` | 2026-10-09 | 2026-10-09 | 0 / 1 / 4 / 3 | open |
| 2 | `dyx3_motion_guard` | 2026-10-09 (+CR) | 2026-10-09 | 0 / 0 / 3 / 6 | open |
| 3 | `dyx3_px4_link` | 2026-10-09 (+CR) | 2026-10-09 | **1** / 4 / 6 / 3 | open |
| 4 | `dyx3_interfaces` (+ px4_msgs pin) | 2026-10-09 | 2026-10-09 | 0 / 0 / 1 / 6 | open |
| 5 | `dyx3_mission` | 2026-10-09 | 2026-10-09 | 0 / 0 / 3 / 4 | open |
| 6 | `dyx3_gnss_rtk` | 2026-10-09 | 2026-10-09 | 0 / 0 / 1 / 3 | open |
| 7 | `dyx3_spray` | 2026-10-09 | 2026-10-09 | 0 / 2 / 2 / 1 | open |
| 8 | `dyx3_geometry` | 2026-10-09 | 2026-10-09 | 0 / 0 / 1 / 5 | open |
| 9 | `dyx3_bringup` + systemd (RT, CPU, restart) | 2026-10-09 | 2026-10-09 | 0 / 0 / 3 / 2 | open |
| 10 | `dyx3_system_gateway` | 2026-10-09 (+CR) | 2026-10-09 | 0 / 2 / 2 / 8 | open |
| 11 | `dyx3_recorder` | 2026-10-09 (×2) | 2026-10-09 | 0 / 5 / 10 / 9 | open |
| 12 | backend | 2026-10-09 (+CR) | 2026-10-09 | 0 / 3 / 4 / 6 | open |
| 13 | installer / deployment | 2026-10-09 | 2026-10-09 | 0 / 5 / 13 / 7 | open |
| 14 | tablet app (`Three_Wheel_v2` `App-Polish`) | — | — | — | deferred (not this session) |
| 15 | PX4 firmware rover path (`dyx-3wd-production`) | — | — | — | deferred (not this session) |
| 16 | `dyx3_rpp_legacy` | not reviewed: reference only, deleted at GATE 7 | | | |

---

## 0. Pending since the cloud phases (owner-deferred, 2026-10-08)

The owner deferred these until `claude/cloud-phases` was finished and merged. That is now done (master `ad58e72`
and later). Status was checked against the repo and firmware on 2026-10-09. They are tracked here because
several are the root causes the part reviews keep hitting.

| ID | Item | Status | Links |
|---|---|---|---|
| PC-1a | Verdict on the 7 prototype tests that failed in the cloud run (`test_terminal_approach_run_remaining`, `test_segment_endpoint_precise_stop`, …): real defects or a version mismatch? | open | `docs/contracts/rpp_legacy_evidence.md` |
| PC-1b | Review the DERIVED edits the cloud session made inside `dyx3_rpp_legacy` (signed reverse, as-flown pivot thresholds 40°/2°, closed-loop rate mode). The oracle is meant to be verbatim | open | GATE 4 / GATE 7 |
| PC-1c | LOCAL ACTION: extract the bag fixtures on the Mac (legacy decode, geometry bag replay: `geometry_bag_replay_test` currently exits 77 = SKIPPED). This closes **GATE 3 on the field corpus** | open | HANDOFF P4, P12 |
| PC-2 | **Timing proposal**, agreed 2026-10-08 but never written or built:<br>- RPP input `/fmu/out/vehicle_odometry` at 100 Hz, `local_position` only for resets and the reference;<br>- control tick 100 Hz, triggered by odometry arrival plus a watchdog;<br>- rpp + motion_guard + px4_link in **one component container, intra-process**;<br>- setpoints + OffboardControlMode at 100 Hz, BEST_EFFORT KEEP_LAST(1); `vehicle_command` RELIABLE;<br>- path once per mission, RELIABLE + TRANSIENT_LOCAL; status topics at 100 Hz to the bag;<br>- SCHED_FIFO + pinned CPU + mlockall.<br>The code today: 50 Hz free-running timers, separate processes, reliable depth-1 setpoints | **open: root of X-003, RPP-008, MG-002, X-006** | `docs/architecture/proposals/` (to write) |
| PC-2b | Firmware `dds_topics.yaml`:<br>- `estimator_status_flags` 5 → 50 Hz;<br>- add `estimator_aid_src_gnss_yaw` (test ratios for the guard);<br>- `vehicle_odometry` / `vehicle_local_position` at 100 Hz;<br>- **add `vehicle_angular_velocity`** (RPP-009).<br>At `8279fa4be3` all are unchanged; angular velocity is commented out at line 56 | **open: owner decision (V1 firmware frozen)** | RPP-009, X-007 |
| PC-2c | `COM_OF_LOSS_T` 0.5 s (offboard-loss timeout). **Not in the parameter baseline** (PX4 default applies) | open | X-008, upstream #27514 |
| PC-3 | Prod cleanup: remove the unused prototype A/B arms from `dyx3_rpp` (parameters + code), mark them "NOT PORTED — evidence" in `docs/tuning/parameter_registry.md`, and leave the oracle alone.<br>- DROP: `use_imu_extrapolation`, `segment_corner_lookahead_extend`, `stop_latch_enabled`, `ekf_reset_compensation` (replaced by `xy_reset_counter`, X-002), `progress_publish_enabled`. All 5 are still in `rpp_param_table.inc`.<br>- **Owner to decide:** `point_hold_enabled`, `point_precise_stop_enabled`, `point_handshake_enabled`, `endpoint_approach_run_remaining` (point/dot marking? long straights?) | open | `docs/migration/ab_arm_audit.md`; RPP-003/004 shrink if extrapolation is dropped |
| PC-4 | RTK transport. Superseded by the owner's decision of 2026-10-09: **no automatic failover**. USB_DIRECT is done (task 1). Still to build and prove: LoRa + USB, LoRa + DDS | partly done | `docs/plans/2026-10-08_production_rtk_plan.md` |
| PC-5 | **FCU parameter baseline**, checked at the bench and not inherited from the prototype:<br>- `EKF2_GPS_P_NOISE` 0.015 (**baseline still 0.05**, flagged unsafe);<br>- `EKF2_GPS_V_NOISE` 0.05 (baseline 0.2);<br>- `RO_YAW_RATE_TH` 0.5 (baseline 0.4);<br>- re-measure `EKF2_IMU_POS_*`, the antenna positions and `GPS_YAW_OFFSET`;<br>- `RBCLW_QPPS_MAX`, `RO_MAX_THR_SPEED`;<br>- one decel value shared by all stop logic; decide whether zero-speed stops bypass `RO_DECEL_LIM` (0.3 m/s²);<br>- never send heading-error feedback as yaw rate (κ·v feed-forward only) | **open: before the 4-point Mission test** | `config/px4/3wd_6x_carry_from_proto.params` |
| PC-6 | Prebuilt release artifacts. The proposal was accepted. Still undecided before any customer delivery: minisign signing, branch protection + 2FA (conflicts with direct-to-master) | partly done | `docs/architecture/proposals/2026-10-08_prebuilt-release-artifacts.md` |
| PC-7a | Health check: PX4 `UXRCE_DDS_DOM_ID` must equal `ROS_DOMAIN_ID` (42). Nothing enforces it | open | installer / `dyx3-health` |
| PC-7b | Verify the `ROS_LOCALHOST_ONLY` discovery between the nodes and the XRCE agent (`dyx3-platform` does not read `ros.env`) | open | |
| PC-7c | One installer test is not self-contained ("dry-run mentions useradd") | open | `installer/tests` |
| PC-8 | The prototype bag manifests pair `as_run_config.rpp_params` names with the wrong values. The recorder must write correct name/value pairs | **Pairing PASS; precision FAIL** (doubles stored with 6 decimals, REC-020); coverage gap REC-002 | `dyx3_recorder` |
| PC-9 | `installer/pins/firmware.pin` still pins `27a7ac9284`, but the rover runs `8279fa4be3`. `msg/`, `srv/` and `dds_topics.yaml` are identical between the two, so px4_msgs on the rover is correct. Bump the pin to `8279fa4be3` for traceability; a px4_msgs rebuild follows at the next upgrade | open (hygiene); **blocked by INS-003**: install via a fresh install, or after the INS-002 re-exec fix | |

Done: tcpdump in the installer (PC-7); mavlink-router template server mode; WENC timer fix in `8279fa4be3`.

---

## 1. `dyx3_rpp`

Reviewer verdict: REQUEST CHANGES (2 CRITICAL, 3 HIGH, 3 MEDIUM). After verification: **0 CRITICAL, 1 HIGH,
4 MEDIUM, 3 LOW**. The reviewer's CRITICALs do not hold against the current producer and mission flow, but the
verification found a HIGH the reviewer missed (RPP-009).

Facts used to re-rate (all at `252778e`):
- `/dyx3/vehicle_state` has one producer, `dyx3_px4_link`. It sets `position_valid` only for finite x/y,
  `velocity_valid` only for finite vx/vy, and `attitude_valid` only when the heading is finite
  (`dyx3_px4_link/src/vehicle_state_assembler.cpp:11-12,32`).
- RPP's clock is `steady_clock` (`rpp_node.cpp:33,47`): receive and tick times come from one monotonic clock.
- `use_imu_extrapolation` defaults to **false** (`rpp_param_table.inc:102`): extrapolation is off in production.
- A mission is loaded while it is LOADING/READY; RPP publishes STOP until RUNNING (`rpp_node.cpp:184-205,335-339`).

| ID | Severity | Status | Area | Where | Item |
|---|---|---|---|---|---|
| RPP-009 | **HIGH** | ACCEPTED (new, found in verification) | Fail-safe / accuracy | `dyx3_px4_link/src/px4_link_node.cpp:838-860`; `rpp_core.cpp:171-176,436`; `stop_pivot_fsm.cpp:51,131` | `VehicleState.yaw_rate_radps` is never filled, so RPP always sees a yaw rate of 0 |
| RPP-001 | MEDIUM (~~CRITICAL~~) | ACCEPTED ↓ | Stall / RT | `rpp_node.cpp:200-202,217-312,321-322` | Mission load, conditioning, hashing and disk I/O run on the FIFO-80 control thread |
| RPP-003 | MEDIUM (~~HIGH~~) | ACCEPTED ↓ | Latency | `rpp_node.cpp:57-72`; `rpp_core.cpp:157-169,782` | Pose age is measured from RPP receipt, not the PX4 sample time |
| RPP-005 | MEDIUM (~~HIGH~~) | DOUBT (measure) | RT | `control_graph.launch.py:34`; `main.cpp:22-36` | RPP and `motion_guard` share CPU 4 at equal FIFO 80 |
| RPP-008 | MEDIUM | DOUBT (measure) | Latency | `rpp_node.cpp:125-127` | Free-running tick, not synchronised to pose arrival |
| RPP-002 | LOW (~~CRITICAL~~) | ACCEPTED ↓ | Hardening | `rpp_node.cpp:57-72` | No `isfinite` check at the RPP boundary |
| RPP-004 | LOW (~~HIGH~~) | ACCEPTED ↓ | Hardening | `rpp_core.cpp:782-797` | Negative or non-finite age is not rejected before extrapolation |
| RPP-006 | LOW (~~MEDIUM~~) | ACCEPTED ↓ | Params | `rpp_node.cpp:116`; `docs/contracts/rpp_node.md:40` | IDLE_ONLY conditioning parameters are accepted while READY/PAUSED but take effect only at the next load |
| RPP-007 | — | REJECTED | — | `rpp_node.cpp:317-343` | "Tick overrun has no fail-safe" |

### RPP-009 — HIGH — yaw rate is never populated; stop and pivot confirmation ignore rotation

- `VehicleState.yaw_rate_radps` exists (`dyx3_interfaces/msg/VehicleState.msg:18`), but `px4_link` never sets it.
  It does not subscribe to an angular-rate topic, so the field is always 0.
- RPP feeds it into `yaw_rate_ned_` (`rpp_node.cpp:70`; `rpp_core.cpp:174`). It reaches `StopTelemetry.yaw_rate`
  (`rpp_core.cpp:436`).
- `StopConfirm::satisfied` (`stop_pivot_fsm.cpp:51`) and the pivot settle test (`stop_pivot_fsm.cpp:131`) require
  `|yaw_rate| < segment_stop_yaw_rate_threshold` (0.05 rad/s). With 0 always passed in, that check is always
  true. A stop or pivot can be confirmed while the rover is still rotating, which means corner overshoot and a
  mis-aligned start of the next line.
- The prototype had a real yaw rate from MAVROS. The legacy shim maps this same field
  (`dyx3_rpp_legacy/input_shim.py:75`), so the equivalence suites, which inject yaw rate directly, cannot catch it.
- **Fix: owner decision needed.** `/fmu/out/vehicle_angular_velocity` is commented out in
  `src/modules/uxrce_dds_client/dds_topics.yaml:56` at firmware `8279fa4be3`, and `VehicleLocalPosition` has no
  yaw-rate field.
  - (a) **Firmware:** enable `vehicle_angular_velocity` in `dds_topics.yaml`, with a rate limit of about 50 Hz.
    This is the clean source: a real gyro rate. It is a firmware change, and V1 is frozen.
  - (b) **Jetson only:** `px4_link` derives the yaw rate from consecutive `vehicle_attitude` quaternions,
    using PX4 `timestamp_sample` deltas, with wrap handling and a short low-pass filter.
    - No firmware change, but it adds noise and lag.
  - Either way: state and test the sign (NED, clockwise positive). Add a node test: rotating at 0.2 rad/s must
    not confirm a stop.
- Belongs to the `px4_link` part; tracked here because RPP is where it bites.

### RPP-001 — MEDIUM — blocking mission load on the control thread
- Confirmed: `load_mission()` runs inside the mission-state callback and the `step()` retry, on the
  single-threaded executor of a FIFO-80 process. It runs `load_artifact`, `condition_path`, SHA-256, serialise,
  `create_directories`, write, rename.
- Re-rated because the load happens only while the mission is LOADING/READY, or while `load_failed_` is set.
  In those states RPP outputs STOP, and a new mission cannot start until the previous one is terminal.
  `px4_link` still fails to zero on a stale guard command (`command_max_age_s` 0.2).
- What remains real:
  - RPP is on the same CPU as `motion_guard` at the **same FIFO priority**. A long CPU-bound conditioning run
    therefore delays the guard's ticks too (see RPP-005).
  - A failing load is retried every 1 s inside the control tick.
- **Fix:** prepare the mission off the RT thread (worker thread or the mission node). Install it into the core
  with a constant-time handoff. Measure `condition_path` time on the Jetson for the largest real mission first.

### RPP-003 — MEDIUM — receipt time instead of sample time
- Confirmed: `pose_recv_ns_` is the RPP callback time. `px4_link` also stamps freshness on receipt and
  republishes on a 20 ms gate. `px4_sample_stamp` is carried in `VehicleState` but unused.
- Re-rated: with extrapolation off by default, pose age only feeds the 0.5 s staleness test, so control
  accuracy is not affected today. It becomes HIGH if latency compensation is turned on for the 1 cm goal.
- **Fix (when compensation is pursued):**
  - carry the sample age end to end;
  - validate the clock domain (PX4 timesync offset);
  - keep the receipt-time watchdog separate.

### RPP-005 — MEDIUM — DOUBT — shared FIFO CPU
- RPP and `motion_guard` are both `taskset -c 4 chrt -f 80`. Under equal-priority FIFO, one does not preempt
  the other until it blocks.
- The tick is short and `spin_once(5 ms)` blocks, so this matters mainly together with RPP-001.
- **Measure** on the Jetson with `cyclictest`, `perf sched` or `ros2_tracing`: guard and RPP wake-up latency,
  missed periods, RT throttling, nominal and during a mission load.

### RPP-008 — MEDIUM — DOUBT — free-running tick
- Confirmed that the tick is not tied to pose arrival.
- The whole chain has four unsynchronised timer hops: `px4_link` state gate 20 ms → RPP 50 Hz → guard 50 Hz →
  `px4_link` writer 100 Hz. That is about 35 ms typical and 70 ms worst of added age by phase analysis alone.
- The fix is a chain-level design decision: phase-lock, event-driven or 100 Hz. Decide it after measuring
  p50/p99 pose-to-PX4 latency. Tracked together with MG and PXL items X-003.

### RPP-002 / RPP-004 — LOW — boundary hardening
- The current producer cannot deliver non-finite pose, velocity or heading with the valid flags set. The
  steady clock cannot run backwards. Extrapolation is off.
- Still worth a one-line `isfinite` guard at the RPP boundary and `pose_age_s < 0` → STOP, so the package does
  not depend on its producer's discipline.
- Behaviour-neutral for valid input; re-run the equivalence suites.

### RPP-006 — LOW — IDLE_ONLY while READY/PAUSED
- The code matches the contract ("refused … IDLE_ONLY while a mission runs").
- Conditioning parameters changed while READY or PAUSED are stored but apply only at the next load. The
  reported `conditioned_execution_sha256` stays truthful for what is running.
- **Fix:** document "takes effect at the next mission load", or refuse the conditioning subset while a
  mission is loaded.

### RPP-007 — REJECTED
- A late tick still computes with the current time and re-checks pose age against `pose_max_age_s`, so a stale
  pose stops.
- A late tick with a fresh pose is a valid tick. Downstream, the guard (`command_max_age_s` 0.2) and
  `px4_link` enforce command age.
- The overrun counter is observability, which is correct. A separate control-gap fault would duplicate
  existing protection.

---

## 2. `dyx3_motion_guard`

Reviewer verdict: REQUEST CHANGES (0 CRITICAL, 2 HIGH, 3 MEDIUM, 1 LOW). After verification: **0 CRITICAL,
0 HIGH, 3 MEDIUM, 4 LOW**.
- The reviewer's two HIGHs do not hold as defects. MG-001 is a timeout rationale at 2.5–25× the producer
  periods; MG-002 is the same measurement item as RPP-005.
- Verification found a MEDIUM the reviewer missed (MG-007).

Confirmed good, recorded so later reviews do not re-open them:
- **STOP is immediate, never ramped.** `fail()` resets the limiter state (`fail_to_zero.cpp:39-46`). Accel and
  jerk shaping is not in the production node; RPP owns the motion profile (`motion_guard_node.cpp:193-194`).
- **A never-heard input fails.** Every gate input keeps its failing defaults until first seen
  (`motion_guard_node.cpp:213-219`).
- **A clean STOP is always forwarded** whatever the gates say (`fail_to_zero.cpp:54-59`).
- **E-stop asserts immediately.** The service callback runs `step()` at once (`motion_guard_node.cpp:140`).
- **E-stop aborts the mission**, and a mission must be started again by the operator. The mission node aborts on
  `SafetyGateStatus` reason ESTOP (`dyx3_mission/src/mission_node.cpp:233-243`), and that gate status is computed
  from the gate inputs, independently of the command mode (`motion_guard_node.cpp:266-273`).

Producer periods used to re-rate MG-001:
| Input | Rate | Source |
|---|---|---|
| `vehicle_state` | 50 Hz | `px4_link` |
| `estimator_health` | 10 Hz | `px4_link` |
| `px4_link/status` | 10 Hz | `px4_link_node.cpp:883` |
| `operator_link` | 10 Hz | gateway |
| `mission/state` | 10 Hz | mission |
| `rtk_status` | 5 Hz | `rtk_node.cpp:112` |

| ID | Severity | Status | Area | Where | Item |
|---|---|---|---|---|---|
| MG-007 | **MEDIUM** | ACCEPTED (new, found in verification) | E-stop / spray | `motion_guard_node.cpp:266-279,140` | E-stop and safety-gate state are published on a 100 ms cadence, not on change |
| MG-003 | MEDIUM | ACCEPTED | Shutdown | `main.cpp:9-15` | No final STOP on SIGTERM (default rclcpp signal handling) |
| MG-002 | MEDIUM (~~HIGH~~) | DOUBT (measure, with RPP-005) | RT | `motion_guard_node.cpp:144-146` | Timer-only output with no measured deadline on the shared FIFO CPU |
| MG-001 | LOW (~~HIGH~~) | ACCEPTED ↓ | Gates | `motion_guard_node.cpp:173-178` | 0.5 s freshness for every status input: rationale not documented per input |
| MG-004 | LOW (~~MEDIUM~~) | ACCEPTED ↓ (policy) | E-stop | `estop_gate.cpp:5-10`; `docs/contracts/dyx3_motion_guard.md:70-73` | Any valid source can clear an E-stop asserted by another |
| MG-005 | LOW (~~MEDIUM~~) | ACCEPTED ↓ | Fail-safe | `fail_to_zero.cpp:5-24` | Sequence tracking cannot tell a second publisher from a restart |
| MG-006 | LOW | ACCEPTED | RT | `motion_guard_node.cpp:260-263` | `RCLCPP_WARN` on reason change inside the control callback |

### MG-007 — MEDIUM — E-stop state reaches the mission and spray up to 100 ms late, and a short pulse can be missed
- The command STOP is immediate. But `/dyx3/safety_gate` and `/dyx3/emergency_stop_state` are published only
  when 100 ms have passed (`motion_guard_node.cpp:266`). The `step()` called from the E-stop service obeys the
  same gate.
- Consequences:
  - the mission abort (via `safety_gate`) and the spray node's E-stop input (`spray_node.cpp:100-102`) react up
    to 100 ms late, which is about 10 cm of paint at 1 m/s if the spray lease has not already closed the valve;
  - an assert-then-clear within one 100 ms window is never seen by the mission. The mission is then not aborted,
    and motion can continue when the clear arrives.
- **Fix:** publish the gate and E-stop state immediately when the E-stop latch changes (in the service callback),
  then keep the 10 Hz cadence. Add a test: assert and clear 30 ms apart → the mission is ABORTED.

### MG-003 — MEDIUM — no final STOP on SIGTERM
- Confirmed: `rclcpp::init` with default signal handling, `rclcpp::spin`, no STOP path. RPP has one
  (`dyx3_rpp/src/main.cpp`).
- Impact is limited. `control_graph.launch.py:65` shuts the whole graph down when any node exits, so on a
  graph stop every node receives SIGINT together. RPP's STOP burst then lands on a guard that is already
  stopping. A guard-side STOP only helps if `px4_link` outlives it.
- **The stop-on-exit that matters is in `px4_link`, the last hop** → X-010.
- Still add the guard STOP, mirroring `rpp_node`'s `main.cpp` (cheap, symmetric).

### MG-002 — MEDIUM — DOUBT — no measured deadline
- Same root as RPP-005 (shared CPU 4, FIFO 80).
- Downstream, `px4_link` zeroes at 0.2 s command age.
- Measure together with RPP-005: guard wake-up jitter, `step()` duration, missed periods; nominal and during a
  mission load.

### MG-001 — LOW — 0.5 s status freshness
- Re-rated. 0.5 s is 2.5 periods for `rtk_status` (5 Hz), 5 periods for the 10 Hz inputs and 25 for
  `vehicle_state`.
- The physical state does not change because a status publisher went silent.
- A dead in-graph producer (mission, `px4_link`, gateway) takes the whole graph down (`on_exit=Shutdown`).
  `operator_link` has its own loss timeout in the gateway.
- The reviewer's scenario (RTK publisher silent, rover keeps moving for 0.5 s on a still-valid fix) is not a
  hazard.
- **Fix:** document the per-input rationale. Optionally tighten `vehicle_state_max_age_s` (50 Hz producer)
  together with RPP's `pose_max_age_s`.

### MG-004 — LOW — cross-source E-stop clear (policy)
- The code matches the contract ("clearing needs an explicit asserted=false from a valid source").
- Clearing does not resume motion: the mission was aborted and must be started again (see above; MG-007 covers
  the short-pulse gap).
- The only client is the system gateway (`gateway_node.cpp:48`), so the source string is not authentication.
- **Owner decision:** keep "any source clears", or "only the asserting source, or the physical one, clears".

### MG-005 — LOW — second publisher
- Production has one publisher. Strictly interleaved streams keep resetting the session below `accept_count`
  (`fail_to_zero.hpp:21`) and fail closed.
- The realistic risk is a hand-launched `dyx3_rpp_legacy`, whose `output_topic` is configured to the same name
  (`docs/contracts/dyx3_motion_guard.md:15`).
- **Fix:** a `dyx3-health` check for exactly one publisher on `/dyx3/rpp/motion_setpoint`.

### MG-006 — LOW — log on reason change
- Fires only on change, after the command has been published. Keep it; consider throttling it if reasons
  flap.

---

## 3. `dyx3_px4_link`

Reviewer verdict: REQUEST CHANGES (1 CRITICAL, 3 HIGH, 3 MEDIUM, 1 LOW). After verification: **0 CRITICAL,
3 HIGH, 1 MEDIUM, 3 LOW, 1 rejected**.
- The CRITICAL (PXL-001) is real as an **unmeasured acceptance gate**, not as a code defect: PX4 owns the stop
  after companion death. It is re-rated HIGH and merged with X-008 / PC-2c.
- Verification confirmed that PXL-004 sits literally **inside** the 100 Hz writer tick.
- Verification found one more parameter item the reviewer missed (X-011).

Facts used (code at `8236c65`, firmware `8279fa4be3`, `config/px4/3wd_6x_carry_from_proto.params`):
- **Offboard loss.** `COM_OBL_RC_ACT` is 7 (Disarm), set in the baseline. `COM_OF_LOSS_T` is **not set**, so the
  firmware default of 1.0 s applies; the prototype had 30 s, which is good not to have carried over.
  `RoverDifferential` resets and stops on disarm.
- **What the heartbeat does.** On `link_ok` = false it is withdrawn, because no trustworthy zero can be published
  (`offboard_heartbeat.cpp:21-28`). A stale `/fmu/out` topic keeps the heartbeat and sends STOP (gate), which is
  correct.
- **`main.cpp:15-17`:** plain `rclcpp::spin`; no STOP on SIGTERM (= X-010).
- **One tick order** (`px4_link_node.cpp:721-780`): monitor → handshake requests → `service_spray_transactions`
  (may call `dispatch_next_spray_transaction` → `reserve()`) → gate → **setpoint publish** → status.

| ID | Severity | Status | Area | Where | Item |
|---|---|---|---|---|---|
| PXL-002 | **HIGH** | ACCEPTED | Stop | `px4_link_node.cpp:290-296`; `offboard_heartbeat.cpp:5-20` | `SetOffboard(false)` stops the setpoint stream at once, with no STOP first: PX4 keeps the last setpoint until offboard loss (1.0 s), then disarms |
| PXL-004 | **HIGH** | ACCEPTED (mechanism; magnitude to measure) | Stall | `spray_ack_tokens.cpp:59-92`; `px4_link_node.cpp:658-685,759,772` | Two `fsync`s (file + directory) per new spray transaction, inside the 100 Hz writer tick, before the setpoint publish |
| PXL-001 | **HIGH** (~~CRITICAL~~) | ACCEPTED ↓ (acceptance gate; merged with X-008, PC-2c) | Stop | `main.cpp:15-17`; firmware `commander_params.c` | No measured stop bound after process, agent, Ethernet or Jetson loss: PX4 offboard-loss (1.0 s default) + disarm is the only path |
| PXL-003 | MEDIUM (~~HIGH~~) | DOUBT (measure, with X-006) | Stall | `px4_link_node.cpp:721-780`; `main.cpp:16` | One non-RT executor serves the writer plus ULog, RTCM, spray, handshake and services |
| PXL-005 | LOW (~~MEDIUM~~) | DOUBT (bench) | Correctness | `px4_link_node.cpp:347`; `offboard_heartbeat.cpp:32-44` | 0.5 s prestream before the OFFBOARD request; a rejection goes to terminal `Failed` with no retry |
| PXL-006 | LOW (~~MEDIUM~~) | ACCEPTED ↓ | State | `vehicle_state_assembler.cpp:13-27` | z, vz, deltas and the reference lat/lon/alt are copied without `isfinite` |
| PXL-008 | LOW | ACCEPTED | Latency | `px4_link_node.cpp:762-766` | `std::to_string` + string compare every tick (small-string, no heap; trivial) |
| PXL-007 | — | REJECTED (duplicate) | Tests | — | "Unit tests do not prove physical stop time": this is the X-008 / PXL-001 measurement, not a separate defect |

### PXL-002 — HIGH — disabling offboard drops the stream without a STOP
- `SetOffboard(false)` calls `offboard_->enable(false)` and replies OK at once (`px4_link_node.cpp:290-296`).
- On the next tick `step()` returns `Disabled` with `publish_heartbeat` = false (`offboard_heartbeat.cpp:17-20`).
  The very next writer tick publishes nothing: no STOP, no mode change.
- PX4 keeps applying the last rover setpoint until `COM_OF_LOSS_T` (1.0 s), then disarms (`COM_OBL_RC_ACT` 7).
  At 1 m/s that is up to about 1 m on the last command.
- The service reply means "Jetson stopped sending", not "rover stopped".
- **Fix:**
  1. On disable, publish the STOP set for a short window (e.g. 0.2–0.3 s, ≥ 20 ticks) before withdrawing the
     heartbeat.
  2. Then send an explicit mode change (Hold) or disarm, per the owner's policy.
  3. Reply only when `vehicle_status` confirms, or say "requested" in the result.
- **Test:** disable while commanding 0.5 m/s → zero speed from the next tick; PX4 leaves OFFBOARD within the
  window.

### PXL-004 — HIGH — `fsync` inside the 100 Hz writer
- Each new spray transaction (`ack_token == 0`) calls `SprayAckTokens::reserve()` → `persist()`. That opens a temp
  file, then write + `fsync` + rename + `fsync` of the directory (`spray_ack_tokens.cpp:59-78`).
- It runs from `dispatch_next_spray_transaction()` (`px4_link_node.cpp:665`), which is called from
  `service_spray_transactions()` **inside `step()` before the setpoint publish** (lines 759 → 772).
- Valve commands happen while moving, at every line start and end. eMMC/NVMe `fsync` is typically a few ms, but
  it is unbounded under I/O load.
- A persist failure sets `failed_` permanently. Spray is then refused until restart: fail-safe, but it stops
  marking.
- **Fix:**
  - Reserve identities in **blocks** (e.g. persist `next + 64` once, hand out 64 from memory). That keeps the
    never-reuse guarantee across power loss and leaves one `fsync` per 64 transactions.
  - Or refill the block from a worker thread.
  - Never delete the `fsync`.
- **Test:** inject a 50 ms `persist` delay → the writer's maximum inter-tick gap must stay under 15 ms.

### PXL-001 — HIGH — no measured stop bound after companion loss (acceptance gate)
- Confirmed: nothing on the Jetson can stop the rover once `px4_link`, the agent, Ethernet or the Jetson is gone.
  The bound is PX4's: `COM_OF_LOSS_T` (default **1.0 s**, not in the baseline) → `COM_OBL_RC_ACT` 7 (disarm) →
  `RoverDifferential` stop.
- Upstream #27514 reports about 900 ms of continued setpoint application after the external process dies.
- `main.cpp` sends no STOP on SIGTERM either (X-010). A systemd stop while moving falls into the same 1 s path.
- **Fix:**
  - set `COM_OF_LOSS_T` explicitly (PC-2c; candidate 0.5 s, it must exceed the worst writer gap);
  - add the SIGTERM STOP burst (X-010);
  - HIL-measure every row of the stop matrix: SIGKILL, SIGTERM, agent kill, cable pull, Jetson power-off; wheels
    up first, then on the ground at mission speed. Record ULog nav state, arming and wheel speed, plus the
    stopping distance.

### PXL-003 — MEDIUM — DOUBT — shared non-RT executor
- Confirmed: one `SingleThreadedExecutor` under normal scheduling (X-006) runs the writer timer and every callback.
- Publishes are reliable with KEEP_LAST depth 1. Fast DDS does not block on a full history for KEEP_LAST, so a
  multi-ms `publish()` stall is plausible only from the transport and is **unproven**.
- Measure first: inter-tick gap p50/p99/max with ULog streaming, RTCM over DDS and spray active; agent kill;
  saturated Ethernet.
- The structural fix is PC-2 (one RT container, writer on its own callback group/thread).

### PXL-005 / PXL-006 / PXL-008 — LOW
- **PXL-005.** A rejected OFFBOARD request is a benign refusal (no motion). Confirm the prestream PX4 needs on a
  cold boot at the bench. Add one retry, or make `Failed` visible to the operator.
- **PXL-006.** The horizontal pose, velocity and heading are already finite-gated. Gate
  `global_reference_valid` on finite lat/lon/alt for the gateway and map consumers.
- **PXL-008.** Compare the enum, not a string; trivial.

---

## 4. `dyx3_mission`

Reviewer verdict: REQUEST CHANGES (0 CRITICAL, 2 HIGH, 4 MEDIUM, 1 LOW). After verification: **0 CRITICAL,
0 HIGH, 3 MEDIUM, 4 LOW**.
- Both HIGHs are real behaviours, but they are product-semantics and recovery decisions, not safety defects.
- No path into RUNNING bypasses the gate; this is confirmed below.

Confirmed good:
- **The only two ways into RUNNING both check the gate.** They are READY → RUNNING via `rpp_ack(gate_ok)`
  (`mission_node.cpp:268-273`) and PAUSED → RUNNING via `resume(gate_ok)`.
- **A recovered gate never resumes a mission.** `evaluate_gate` ignores PAUSED.
- **The RPP acknowledgement is filtered by `mission_id`** (`:253`). RPP and mission live in one launch graph, so
  they restart together; a stale acknowledgement from a previous instance cannot arrive.
- **An RPP artifact-load failure is not a silent wait.** RPP publishes `STATE_ERROR` while `load_failed_`
  (`rpp_node.cpp:330-333`), and the mission goes to ERROR (`:261-267`).
- **Mission IDs restart at each boot, but the recorder does not collide.** It names run directories by time +
  `mission_id` + `unique_run_path` (`recorder_node.cpp:205`). The backend does not key on `mission_id`.
- **RPP and spray load the path through the same C++ reader** (`dyx3_mission::load_artifact`; spray via
  `spray_node.cpp:90`). So the C++ consumers cannot disagree about whether an artifact is valid.

| ID | Severity | Status | Area | Where | Item |
|---|---|---|---|---|---|
| MS-001 | MEDIUM (~~HIGH~~) | ACCEPTED ↓ (owner decision) | Points | `point_journal.cpp:47-88`; `docs/contracts/dyx3_mission.md:57-61` | `PointResult COMPLETED` means "came within 0.10 m", not "marked" |
| MS-002 | MEDIUM (~~HIGH~~) | ACCEPTED ↓ (owner decision) | Restart | `mission_node.cpp:379-383`; `mission_fsm.cpp` | Point progress lives in memory only; after a graph restart the mission is IDLE and progress is lost |
| MS-003 | MEDIUM | ACCEPTED | Fault | `mission_node.cpp:252-300` | No RPP-status freshness check while RUNNING |
| MS-006 | LOW (~~MEDIUM~~) | ACCEPTED ↓ | Fault | `mission_node.cpp:36,293-299` | `rpp_ack_timeout_s` = 0 disables the READY timeout |
| MS-004 | LOW (~~MEDIUM~~) | ACCEPTED ↓ | Stall | `path_artifact.cpp:122-127`; `mission_node.cpp:385-387` | Artifact read and SHA-256 with no size limit, inside the Start service callback |
| MS-005 | LOW (~~MEDIUM~~) | ACCEPTED ↓ | Artifact | `path_artifact.cpp:88-116` | The C++ reader accepts non-canonical metadata and numbers that the Python decoder would refuse |
| MS-007 | LOW | ACCEPTED | Points | `mission_node.cpp:283-288` | The journal uses `position_valid` but not the sample age |

### MS-001 — MEDIUM — "COMPLETED" is geometric (owner decision)
- The code matches the contract: a must-hit vertex is COMPLETED when the rover enters 0.10 m and then leaves.
  The result carries the closest-approach distance.
- The result does not say the paint valve was open there. It is published to the tablet via the gateway
  (`gateway_node.cpp:197-199`) as `last_point_result`, so an operator can read it as "marked".
- **Owner decision:**
  - (a) keep it geometric and show it as "reached (x cm)" in the app;
  - (b) add spray evidence (valve ON across the vertex) before COMPLETED.
- The 0.10 m radius is not the 1 cm quality check. Decide where marking quality is judged: recorder or
  post-run report.

### MS-002 — MEDIUM — progress lost on restart (owner decision)
- Confirmed. The journal and FSM are in memory; a restart comes back IDLE. There is no automatic motion (X-009
  is satisfied at code level: IDLE needs a new Start).
- What is lost is which points were done, so the operator restarts the whole path or re-plans by hand.
- **Owner decision:** resume-from-point after a crash (persist the journal and execution id atomically; a
  continuation mission starts at the first unresolved point, with explicit operator confirmation), or
  whole-mission restart only.
- The recorder bag still holds the truth for reconciliation.

### MS-003 — MEDIUM — RPP status silence while RUNNING
- Confirmed: no receipt time is kept for `RppStatus`.
- If RPP hangs without dying (a dead RPP takes the graph down), the guard's 0.2 s command age stops the rover,
  but the mission keeps showing RUNNING with no progress.
- **Fix:** keep the RPP status receipt time. If it is older than about 0.5 s while RUNNING, PAUSE with a
  distinct reason. Test: silence RPP status → PAUSED, and no auto-resume.

### MS-006 / MS-004 / MS-005 / MS-007 — LOW
- **MS-006.** The READY wait can only become endless if RPP is alive but never publishes, since a load failure
  already ends in ERROR. Set a finite default (the largest real mission's conditioning time + margin, measured
  with RPP-001) and a distinct reason code.
- **MS-004.** The mission node is not RT, loading happens before RUNNING, and a stall fails closed (the guard
  needs a fresh RUNNING). Add a size cap at the backend upload limit.
- **MS-005.** The SHA covers the bytes and all C++ readers agree. Only align with the Python decoder (shared
  test vectors).
- **MS-007.** Telemetry only. Gate on sample age once X-001 (sample time end to end) exists.

---

## 5. `dyx3_spray`

Reviewer verdict: REQUEST CHANGES (1 CRITICAL, 2 HIGH, 2 MEDIUM). After verification: **0 CRITICAL, 2 HIGH,
2 MEDIUM, 1 LOW**.
- The CRITICAL (manual spray) is a documented, operator-only bench feature. It is re-rated MEDIUM as a policy
  decision.
- The overriding open question is unchanged and moves to X-012: does the valve close physically when every
  Jetson path is lost?

Confirmed good:
- **A safety refusal closes the valve at once.** It goes straight to OFF and bypasses debounce, because the FSM
  reads the safety verdict directly (`docs/contracts/dyx3_spray.md:146-151`). Debounce applies only to
  geometric boundary edges.
- **Manual ON cannot open during an E-stop or when disarmed.** `safety_allows_on` checks E-stop, armed and the
  watchdog before the manual exemption (`spray_controller.cpp:135-141`).
- **Manual ON needs the operator token.** It is reachable only through backend `POST /spray/manual` with the
  operator role (`routes.py:179-181`) → gateway → `/dyx3/spray/set_manual`. It has a hard 10 s expiry.
- **px4_link gives the watchdog's OFF priority** over queued controller commands and refuses the replay of an
  older ON epoch (`px4_link_node.cpp:532-620`).
- **Correction to the review prompt:** the valve delays are **not** 0. The defaults are `solenoid_open_delay_s`
  0.18 and `solenoid_close_delay_s` 0.05, plus `on_overspray_margin_m` 0.02 (`spray_param_table.inc:21-24`).
  They are prototype values, not measured on this valve. The nozzle offsets are 0.

| ID | Severity | Status | Area | Where | Item |
|---|---|---|---|---|---|
| SP-002 | **HIGH** | ACCEPTED | Boundary | `spray_controller.cpp:318-328`; contract `dyx3_spray.md:94-97` | 3-tick debounce delays every boundary edge by up to 60 ms, and the lead maths does not compensate |
| SP-003 | **HIGH** | DOUBT (bench measurement; = production-readiness #7) | Boundary | `spray_param_table.inc:21-24` | Valve open/close delays and the nozzle offset are prototype values, not measured |
| SP-001 | MEDIUM (~~CRITICAL~~) | ACCEPTED ↓ (owner decision) | Manual | `spray_controller.cpp:140-141,203-223` | Manual ON overrides the mission, path and RTK gates, including **during a RUNNING autonomous mission** |
| SP-004 | MEDIUM | ACCEPTED | Close | `spray_controller.cpp:236-238`; `spray_gates.cpp:136-144` | `min_spray_speed_mps` is declared but unused; STOPPING/CREEPING may keep the valve ON while the rover is almost stopped on a MARK leg |
| SP-005 | LOW (~~MEDIUM~~) | ACCEPTED ↓ | Stall | `spray_node.cpp:271-337` | Conditioned-artifact load runs on the spray executor |

### SP-002 — HIGH — debounce latency is not in the boundary lead
- Confirmed. `debounce_samples` 3 at 50 Hz: an edge needs 3 identical ticks, so it is delayed 40–60 ms with
  phase. That is about 2 cm at 0.35 m/s and up to **6 cm at 1 m/s**, on OPEN and CLOSE.
- The contract already measured about 2 cm late at 0.35 m/s and left acceptance to GATE 5.
- This breaks the ±1–2 cm goal at any production speed above about 0.35 m/s.
- **Fix (behaviour-preserving for safety OFF):** add the debounce delay `(debounce_samples − 1) / tick_hz × v` to
  both the ON and OFF leads in the projection, keeping the debounce for noise rejection. Or lower the debounce
  once the pose noise is measured.
- **Test:** the boundary edge error at 0.35 / 0.5 / 1.0 m/s in `spray_core_test` must be ≤ 1 cm in simulation.

### SP-003 — HIGH — DOUBT — valve timing not measured
- The lead compensation is implemented and uses prototype delays (0.18 / 0.05 s). A 30 ms error is 3 cm at
  1 m/s.
- **Bench, same task as production-readiness #7:**
  - ON/OFF electrical command → paint edge latency (p50/p95) at the working pressure;
  - nozzle forward and lateral offset relative to the navigation reference point;
  - then set the parameters.

### SP-001 — MEDIUM — manual spray authority (owner decision)
- Re-rated. It is a deliberate bench feature in the contract ("Manual (bench) spray is exempt … armed + watchdog
  suffice"). It is operator-only, E-stop and disarm still close it, and it expires after 10 s.
- The real gap: `manual_active_` returns before the ownership check, so a manual ON **during a RUNNING mission**
  overrides the boundary geometry for up to 10 s.
- **Owner decision:** refuse manual while a mission is LOADING/READY/RUNNING/PAUSED; optionally only when the
  rover is stationary or in a maintenance mode. An RC-driven manual marking mode would be a separate, explicit
  feature.

### SP-004 — MEDIUM — near-stationary marking
- Confirmed: no general minimum-speed OFF. The contract removed that gate on purpose for endpoint creep.
- STOPPING into a corner on a MARK leg can therefore paint a blob as the speed reaches 0. `terminal_off_speed_mps`
  covers only the terminal.
- **Fix:** OFF when the speed is below `min_spray_speed_mps` unless in terminal creep. Test: stationary on MARK
  → OFF; terminal creep → no gap; corner STOPPING → OFF at the boundary.

### SP-005 — LOW
- Loads happen on a mission or path change, normally before RUNNING; the node is not RT; a refusal fails OFF.
- Same pattern as RPP-001 and MS-004. Fold into one "prepare the mission off the control thread" change.

Not counted: the lease has no sequence or replay check (`safety_lease.cpp:40-57`). There is one publisher, and a
single stale lease cannot keep the watchdog alive. Hardening only.

---

## 6. `dyx3_gnss_rtk`

Reviewer verdict: acceptance pending (0 CRITICAL, 1 HIGH, 3 MEDIUM); the reviewer marked it a partial review.
After verification: **0 CRITICAL, 0 HIGH, 1 MEDIUM, 3 LOW**.
- The reviewer's two status concerns cannot let the guard pass a bad fix: the guard checks the receiver's own
  fix and rejects an accuracy of 0.

Confirmed good:
- **The guard's RTK gate is strict** (`dyx3_motion_guard/include/.../rtk_gate.hpp:23-30`). It requires all of:
  - a fresh `rtk_status` and `corrections_fresh`;
  - fix 5 or 6, and at least `rtk_min_fix_type` 6;
  - an accuracy that is finite and **> 0**, so the unknown sentinel 0 fails;
  - hrms ≤ 0.10 m.
  Spray also gates on `corrections_fresh` (`spray_gates.cpp:86-88`). A stale or missing GNSS report leaves
  `fix_type` 0, which fails.
- **The fix comes from the receiver's own report** (`/dyx3/gnss_report` from PX4's driver) and is never derived
  from correction packets.
- **The only serial write to the UM982 is validated RTCM frames** (`transport_sink.cpp:81-104`, with
  partial-write handling). No configuration command was found. The LoRa port is read-only. The NTRIP writes go
  to the caster (request + GGA), not to the receiver.
- **One injector:** `injection_authority` selects one sink, and generation numbers reject callbacks from an old
  source.
- **RTCM integrity:** CRC-24Q, partial-frame buffering and resynchronisation (`rtcm_parser.cpp:43-85`).
- **The control socket is limited:** Unix socket 0660, 64 KiB request cap, its own thread
  (`control_socket.cpp:45-63`).
- **A bad config starts the service STOPPED/ERROR** with the socket up (`rtk_node.cpp:79-103`).

| ID | Severity | Status | Area | Where | Item |
|---|---|---|---|---|---|
| RTK-001 | MEDIUM (~~HIGH~~) | ACCEPTED ↓ | Stall | `ntrip_client.cpp:270-291,386-395`; `rtk_node.cpp:231-244` | `getaddrinfo()` has no deadline; a config change joins the NTRIP worker, so a hung resolver hangs the control socket |
| RTK-002 | LOW (~~MEDIUM~~) | ACCEPTED ↓ | Status | `rtk_node.cpp:376-395`; `correction_health.cpp:13-15` | `correction_age_s` is the age since source receipt, not the receiver's correction age |
| RTK-003 | LOW (~~MEDIUM~~) | ACCEPTED ↓ | Status | `rtk_node.cpp:384-388` | Accuracy 0 = unknown; the guard and RPP already reject it |
| RTK-004 | LOW (~~MEDIUM~~) | DOUBT (measure) | Stall | `rtk_node.cpp:112-117,372-379` | The 200 ms status timer also polls the USB readback |

### RTK-001 — MEDIUM — DNS with no deadline
- Confirmed. `getaddrinfo` sits outside `connect_timeout_s`. It runs on the NTRIP worker thread, not the ROS
  executor, so status publishing continues and corrections are already gone if DNS is broken.
- The real stall: `SET_CONFIG` / STOP on the control-socket thread → `apply_config` → `authority_->stop()` → join
  the worker. That blocks until glibc's resolver gives up, about 5 s × attempts per `resolv.conf`.
- The tablet's RTK control hangs meanwhile. A service stop can wait for the systemd default TimeoutStopSec of
  90 s, which is not set in `dyx3-rtk.service`.
- **Fix:** resolve with a deadline (`getaddrinfo_a` with a timeout, or a resolver thread whose result is
  abandoned on stop without holding the client); set `TimeoutStopSec`.
- **Test:** a black-holed DNS + SET_CONFIG must complete in ≤ connect_timeout + 1 s.

### RTK-002 / RTK-003 / RTK-004 — LOW
- **RTK-002.** The safety decision rests on the receiver's fix and accuracy; `correction_age_s` is diagnostics.
  Label it "source age" in the app. Add the receiver correction age (the UM982 differential age from GGA
  readback; the control status already has `receiver_correction_age_*`) when the interface is next bumped.
- **RTK-003.** Consumers already treat 0 as unknown (guard `rtk_gate.hpp:28`; RPP `rpp_node.cpp:75-80`). Only
  the app display needs "—" instead of 0.000 m.
- **RTK-004.** The readback is non-blocking. Measure the status inter-publish gap during USB unplug/replug and
  SET_CONFIG; the guard allows 0.5 s, which is 2.5 periods.

The reviewer's cross-part notes: "GNSS report freshness" is X-001. "Guard interpretation of unknown accuracy" is
closed by the gate above. "DDS RTCM chunk acceptance in PX4" is still to be checked when the PX4_DDS transport is
used; the rover uses USB_DIRECT today.

---

## 7. `dyx3_geometry`

Reviewer verdict: REQUEST CHANGES pending verification (0 CRITICAL, 1 HIGH, 4 MEDIUM, 1 LOW). After
verification: **0 CRITICAL, 0 HIGH, 1 MEDIUM, 5 LOW**.
- The reviewer's HIGH depends on a 30 m curvature baseline; the real default is **0.15 m**.
- No wrong NED heading or cross-track sign was found. The CMake setup disables FP contraction and has no
  `-ffast-math`.
- At 5 km, double precision is about 1e-12 m, so site scale is not a precision risk.

Facts used:
- `curvature_baseline_m` defaults to 0.15, LIVE, bound `[0, HUGE_VAL)` (`rpp_param_table.inc:50`).
- `path_resample_spacing_m` defaults to 0.08 (`:51`).
- Per tick, RPP calls `curvature_at` (`rpp_core.cpp:942`) and `project_onto_path` (`:864`, smooth profile).
- `line_intersection` and `resample` run only at path conditioning (`path_conditioner.cpp:315,513`).

| ID | Severity | Status | Area | Where | Item |
|---|---|---|---|---|---|
| GEO-004 | MEDIUM | DOUBT (test) | RT / Hint | `project_onto_path.cpp:28-35`; `rpp_core.cpp:864-865` | A valid hint on the last 1–2 segments widens to a **full-path scan every tick**; ties go to the lowest index |
| GEO-001 | LOW (~~HIGH~~) | ACCEPTED ↓ | RT | `curvature.cpp:30-44`; `rpp_param_table.inc:50` | Baseline walk is linear in `baseline / spacing`: about 2 steps at the defaults, but the LIVE parameter has no upper bound |
| GEO-002 | LOW (~~MEDIUM~~) | ACCEPTED ↓ | Correctness | `project_onto_path.cpp:43-78`; header `:21,29` | An all-degenerate window returns `valid=true` with cross-track 0 (only the hint is invalidated); RPP never reads `.valid` |
| GEO-003 | LOW (~~MEDIUM~~) | ACCEPTED ↓ | Numeric | `line_intersection.cpp:13-17` | Absolute determinant threshold 1e-9 m² (load time only; matches Python) |
| GEO-005 | LOW (~~MEDIUM~~) | ACCEPTED ↓ | API | `resample.cpp:21` | `spacing <= 0` lets NaN through to a float → long conversion |
| GEO-006 | LOW | ACCEPTED | API | `point.hpp:14-23` | `PathView` lifetime and bounds are the caller's discipline |

### GEO-004 — MEDIUM — DOUBT — full scan at the end of every run
- Confirmed mechanism. For `hint.seg ≥ n − 3`, the window `[seg − 2, min(n − 1, seg + 4))` is narrower than 3, so
  it widens to `[0, n − 1]`. During the final 1–2 segments of every smooth run, each tick scans the whole run.
  - At 0.08 m spacing, a 1 km run is about 12,500 segments, roughly 50–100 µs per tick on the Orin.
  - That is within 10 ms, but it is pure waste and grows with the run length.
- **Correctness question:** on a **closed run** (end = start, which the conditioner detects at
  `path_conditioner.cpp:461,526`), the full scan can tie between the last segment and segment 0, and the strict
  `<` picks segment 0. `update_path_progress` could then regress near the finish. The Python ancestor behaves the
  same, so GATE 3 cannot catch it.
- **Test:**
  - a closed square run driven to the end → progress is monotonic and COMPLETE is reached;
  - a 12,500-point run at the end → count the segments scanned.
- **Fix:** clamp a fixed-width window inside `[0, n − 1]` instead of widening to a full scan; full scans only for
  an invalid hint. This changes GATE 3 output at run ends only.

### GEO-001 / GEO-002 / GEO-003 / GEO-005 / GEO-006 — LOW
- **GEO-001.** At 0.15 m / 0.08 m the walk is about 2 segments each way. Give `curvature_baseline_m` a sane upper
  bound (e.g. 2 m) in the parameter table, or precompute cumulative lengths if the bound must stay large.
- **GEO-002.** The behaviour matches its documented contract ("valid false only for an empty path"), and
  conditioning removes duplicates and resamples at 0.08 m, so a 6-segment all-degenerate window cannot occur on
  an installed run. Hardening: return `valid=false` when nothing was scanned, and make RPP stop on
  `!valid`.
- **GEO-003.** The determinant is about 1e-6 even for 1 cm segments at 1°, so it is far above 1e-9 for real
  corners. A scale-aware threshold changes GATE 3; leave it unless a real path fails.
- **GEO-005.** The parameter comes from validated ROS parameters. Add `!std::isfinite(spacing)` and an output-size
  cap anyway (one line).
- **GEO-006.** Document the lifetime rule in the header.

---

## 9. `dyx3_bringup` + systemd

Reviewer verdict: REQUEST CHANGES (0 CRITICAL, 1 HIGH, 3 MEDIUM, 1 LOW new; plus verdicts on the known items).
After verification: **0 CRITICAL, 0 HIGH, 3 MEDIUM, 2 LOW**.

Corrections and confirmations:
- **Correction to the review prompt:** an XRCE agent or mavlink-router crash does **not** restart the control
  graph. `start-platform.sh:29-47` supervises each child in its own loop, restarting after 2 s, and the platform
  unit stays active.
  - Only an explicit `systemctl restart/stop dyx3-platform` propagates to `dyx3-ros` through `Requires=`.
  - During an agent restart (≥ 2 s + session recovery), the setpoints to PX4 stop, and PX4 offboard loss applies
    (X-008).
- **Confirmed: RPP and the guard run every thread at FIFO 80 on CPU 4**, including the DDS receive and event
  threads, because the `taskset` + `chrt` prefix is process-wide. `px4_link` and the agent are SCHED_OTHER and
  not pinned. No CPU isolation, IRQ affinity or RT bandwidth setting is owned by the repository.
- **No node YAML exists anywhere in the repository.** `config/{mission,motion_guard,rpp,spray,recorder,rtk}/`
  hold only `.gitkeep`, and the installer installs none. Every rover runs every node on its built-in, versioned
  defaults. This re-rates BR-001.

Verdicts on known items (recorded; not new):
| Item | Verdict |
|---|---|
| RPP-005 / MG-002 | Structure confirmed (shared core, equal FIFO, DDS threads inherit FIFO 80); severity awaits the Jetson measurement |
| X-006 / PXL-003 | Confirmed configuration; timing unmeasured |
| X-005 | Open: `After=` is activation order, not agent or PX4 readiness |
| X-010 / MG-003 | Confirmed: concurrent shutdown; `px4_link` has no final STOP. Highest-priority stop fix |
| PC-2 | Not implemented |
| PC-7a / PC-7b | Not enforced / bench test needed |
| X-009 | **Satisfied at code level** (the mission FSM starts IDLE; RUNNING needs a new Start + RPP ack); fault-injection test still absent |

| ID | Severity | Status | Area | Where | Item |
|---|---|---|---|---|---|
| BR-001 | MEDIUM (~~HIGH~~) | ACCEPTED ↓ (owner decision) | Config | `control_graph.launch.py:41-50,63`; `config/*/.gitkeep` | A missing `<stem>.yaml` silently means built-in defaults, and **no YAML exists at all**, so the code defaults are the production configuration |
| BR-002 | MEDIUM | ACCEPTED | Restart | `dyx3-ros.service:7-13` | No progress watchdog: a live but stalled node keeps the unit "active" forever |
| BR-004 | MEDIUM | ACCEPTED | Tests | `test_control_graph_launch.py:10-40` | Launch tests check topology only; no failure-propagation, restart or stop-while-moving tests |
| BR-003 | LOW (~~MEDIUM~~) | ACCEPTED ↓ | Observability | `start-platform.sh:29-47` | An agent crash loop leaves `dyx3-platform` "active"; child restarts are not counted |
| BR-005 | LOW | ACCEPTED | Security | `deployment/systemd/*.service` | Every unit shares the user `dyx3` and writes to all of `/var/lib/dyx3`, `/var/log/dyx3`, `/run/dyx3` |

### BR-001 — MEDIUM — the configuration policy (owner decision)
- Today the defaults compiled into each node, generated from `docs/tuning/parameter_registry.md`, **are** the
  configuration. That is reproducible (rover == repo), so a missing file is the normal state, not a lost
  approval.
- The risk appears as soon as per-rover YAML is introduced, e.g. per-rover limits or a measured valve delay
  (SP-003), nozzle offset or antenna geometry: then a missing or wrong-key file silently falls back.
- **Owner decision:**
  - (a) keep "defaults = config" for V1 and document it;
  - (b) introduce per-rover YAML: installed by the installer, `DYX3_REQUIRE_CONFIG=1` in `dyx3-ros.service`, and
    a post-start check of effective parameters against the manifest. (b) is needed once SP-003 / PC-5 produce
    per-rover values.

### BR-002 — MEDIUM — no progress watchdog
- Confirmed. `on_exit=Shutdown` catches exits, not stalls. A `SIGSTOP`ped or hung `px4_link` leaves the unit
  active. PX4 offboard loss stops the rover; the Jetson does not recover or report it.
- **Fix:** an application heartbeat (the writer, guard and RPP progress) → `sd_notify WATCHDOG=1` from a small
  supervisor; `Type=notify` + `WatchdogSec` set from measurements.
- **Test:** SIGSTOP each control process → declared unhealthy within the deadline → the graph restarts → the
  mission is IDLE.

### BR-004 — MEDIUM — no runtime fault-injection tests
- Bench-only suite on the Jetson (wheels up): kill each of the 6 nodes; agent restart keeps the graph; platform
  restart propagation; stop while moving; missing or invalid YAML (once BR-001 (b)); a stalled process.

### BR-003 / BR-005 — LOW
- **BR-003.** Safety is unaffected: the session loss shows in `px4_link/status`, so the guard STOPs and the
  tablet sees "PX4 link unhealthy". Log and count child restarts, and expose the agent's health in
  `dyx3-health`.
- **BR-005.** Least-privilege hardening after an inventory of write paths. No motion impact.

---

## 10. `dyx3_system_gateway`

Reviewer verdict: REQUEST CHANGES (2 CRITICAL, 3 HIGH, 3 MEDIUM). After verification: **0 CRITICAL, 2 HIGH,
1 MEDIUM, 5 LOW**.
- The "rogue local socket client" findings need code already running on the rover as `dyx3`. At that point the
  backend is already compromised, so they are defence-in-depth, not CRITICAL.
- The SIGPIPE finding is confirmed, and it is worse than "the gateway exits".

Confirmed good:
- **Validation is strict:** the JSON parser has depth, duplicate-key, number and line-size limits, and every
  command is validated (`command_validator.cpp`).
- **The IPC thread hands commands to the ROS timer** through a mutex inbox.
- **E-stop and heartbeat are processed first in each batch** (`gateway_node.cpp:426-433`).
- **Service calls are asynchronous with a 2 s deadline.**
- **The operator link uses a steady clock;** `alive` requires a client AND a heartbeat younger than 2.0 s.
- A hung backend holding the socket open ends in a STOP after about 2.1 s.

| ID | Severity | Status | Area | Where | Item |
|---|---|---|---|---|---|
| GW-002 | **HIGH** (~~CRITICAL~~) | ACCEPTED | Lifecycle | `ipc_server.cpp:169-175`; `main.cpp` | `write()` to a closed peer raises **SIGPIPE** (no `MSG_NOSIGNAL`, no `SIG_IGN`) → the gateway dies → `on_exit=Shutdown` **takes down the whole control graph** |
| GW-004 | **HIGH** | ACCEPTED (owner decision) | Link | `gateway_node.cpp:254-258`; `docs/contracts/backend.md` §3 | Operator-loss budget ≈ 1.5 + 2.0 + 0.1 + 0.02 = **3.62 s** before the guard STOP: 1.27 m at 0.35 m/s, 3.6 m at 1 m/s. Not approved |
| GW-001 | MEDIUM (~~CRITICAL~~) | ACCEPTED ↓ | Auth | `ipc_server.cpp:138`; `operator_link.hpp:15-25`; every unit `User=dyx3` | Any process running as `dyx3` can connect, heartbeat and send commands; there is no `SO_PEERCRED`, and all services share one UID |
| GW-003 | LOW (~~HIGH~~) | DOUBT ↓ | Command | `gateway_node.cpp:311-365` | Abort could overtake an in-flight Start, because they go through separate services |
| GW-005 | LOW (~~HIGH~~) | ACCEPTED ↓ | Command | `ipc_server.cpp:136-149`; `gateway_node.cpp:277-289` | No reserved slot for E-stop at `max_clients` 4; priority commands bypass the 256 inbox cap |
| GW-006 | LOW (~~MEDIUM~~) | ACCEPTED ↓ | Command | `command_validator.cpp:76-94` | The request id is correlation only; a manual retry of `skip_point` after a timeout can skip twice |
| GW-007 | LOW (~~MEDIUM~~) | ACCEPTED ↓ | Stall | `gateway_node.cpp:319-335,435-443` | Timed-out rclcpp requests are never `remove_pending_request`ed |
| GW-008 | LOW (~~MEDIUM~~) | ACCEPTED ↓ | JSON | `json.cpp:63-74` | Raw invalid UTF-8 is accepted in strings |

### GW-002 — HIGH — SIGPIPE kills the control graph
- Confirmed. There is no `MSG_NOSIGNAL`, `SIG_IGN` or `sigaction` in the gateway (only `dyx3_gnss_rtk` ignores
  SIGPIPE). `write()` runs before the `POLLHUP` check (`ipc_server.cpp:165-175`).
- **Trigger:** any backend restart (crash, `Restart=always`, upgrade) while telemetry is queued, at 5 Hz, i.e.
  usually. The gateway dies, the launch `on_exit=Shutdown` stops all 6 nodes, and PX4 goes into offboard loss
  → disarm.
- **A backend restart can therefore abort a running mission and stop the rover through the failsafe, not under
  control.**
- **Fix (two lines):** `send(..., MSG_NOSIGNAL)` in the IPC writer, and `std::signal(SIGPIPE, SIG_IGN)` in
  `main.cpp` (also worth adding to `px4_link` and the others).
- **Test:** a subprocess with default SIGPIPE, pending output, the peer closes → the gateway survives and the
  client is removed. Plus a bench test: `systemctl restart dyx3-backend` during a run → `dyx3-ros` is not
  restarted.

### GW-004 — HIGH — the operator-loss budget (owner decision)
- The arithmetic is confirmed and the contract already marks it OPEN. The 0.5 s relay does not add to the worst
  case; the last relay can land just before the 1.5 s expiry.
- **Owner decision:** the approved loss-to-STOP time at the production speed.
  - Example: tablet 1.0 s + gateway 1.0 s ≈ 2.1 s → 0.7 m at 0.35 m/s.
  - Validate against real field Wi-Fi (false stops) before lowering.
  - Decide together with BE-001 (which tablet's heartbeat counts).
- Note: the gateway stamps a heartbeat when the ROS timer drains it, not when the IPC thread receives it. The
  difference is at most one 10 ms timer under normal load; include it in the measurement.

### GW-001 — MEDIUM — the socket trust boundary
- The socket is 0660 owned by `dyx3`, and every service runs as `dyx3`. Only root or a `dyx3` process can
  connect.
- The realistic risk is a debug tool or a second backend instance heartbeating, not an outside attacker.
- **Fix:** a dedicated `dyx3-backend` UID + `SO_PEERCRED` check, and associate the heartbeat with that
  connection. This goes together with BR-005 (per-service identities).

### LOW items
- **GW-003.** Both requests go through one participant to one single-threaded mission node, so a reorder needs
  transport skew between two local services. If it happens, the rover shows RUNNING and the operator aborts
  again. A mission command epoch would close it fully.
- **GW-005.** Reserve a slot for the backend (follows GW-001) and coalesce heartbeats.
- **GW-006.** The backend does not auto-retry. Add an operation id only if `skip_point` double-skips are seen.
- **GW-007 / GW-008.** Focused hardening with regression tests.

---

## 11. `dyx3_recorder`

Reviewer verdict: REQUEST CHANGES (1 CRITICAL, 7 HIGH, 9 MEDIUM). After verification + the Opus cross-check: **0 CRITICAL, 5 HIGH,
10 MEDIUM, 9 LOW**.
- The recorder never affects motion directly. Its HIGHs are about **evidence**: runs that cannot prove what ran,
  with which tuning, from the first second.
- **PC-8 verdict: PASS.** Name/value pairs come from each returned `rclcpp::Parameter` (`recorder_node.cpp:61-64`),
  so the prototype's positional mismatch cannot recur. Coverage is a separate problem (REC-002).

Confirmed in code:
- **Parameters are collected before the bag starts.** `collector_(param_nodes_, param_timeout_s_)` runs at
  `recorder_node.cpp:245`, then `bag_.start(...)` at `:284`; plus rosbag2 discovery.
- **Each run's `.ulg` file starts mid-stream.** `UlogCapture::open` creates a new file per run (`ulog_capture.cpp:7-15`),
  while `px4_link` sends LOGGING_START once at link-up (`px4_link_node.cpp:716`).
- **The running firmware is never read.** `installer/lib/release.sh:272` writes `firmware_running: "unavailable…"`,
  and the rover is flashed by hand. Today the pin says `27a7ac9284` while the FCU runs `8279fa4be3` (PC-9), so
  every run's `versions.json` is already wrong about the firmware.
- No secrets were found in the topics or parameter snapshots; the NTRIP configuration is outside ROS parameters.

| ID | Severity | Status | Area | Where | Item |
|---|---|---|---|---|---|
| REC-001 | **HIGH** (~~CRITICAL~~) | ACCEPTED ↓ | Disk | `recorder_node.cpp:155,216-219` | `min_free_bytes` defaults to 0 and only warns; no retention. Runs share `/var/lib/dyx3` with the missions, the RTK state and the spray-ACK ledger |
| REC-002 | **HIGH** | ACCEPTED | Completeness | `recorder_node.cpp:147-149` | `param_nodes` omits **`rpp`** (119 tuning parameters) and `system_gateway` |
| REC-004 | **HIGH** | ACCEPTED | Completeness | `recorder_node.cpp:204-284` | The bag starts only after RUNNING + up to 6 × 2 s of parameter RPCs + discovery: the first seconds of motion and spray are lost |
| REC-005 | **HIGH** | ACCEPTED | Completeness | `ulog_capture.cpp:7-15`; `px4_link_node.cpp:716` | The per-run `.ulg` starts mid-stream with no ULog header, so it is probably unreadable |
| REC-006 | **HIGH** | ACCEPTED | Completeness | `recorder_node.cpp:224-234`; `release.sh:272` | Runs record the **expected** firmware SHA, not the running one; overlay hash and px4_msgs source missing |
| REC-003 | MEDIUM (~~HIGH~~) | ACCEPTED ↓ | Completeness | `recorder_node.cpp:241-254,338-341` | Parameters snapshotted at start and end only; LIVE changes mid-run are not journaled |
| REC-008 | MEDIUM (~~HIGH~~) | ACCEPTED ↓ | Lifecycle | `run_lifecycle.cpp:28-55` | A run never closes if MissionState stops for good (a restarted graph publishes IDLE and closes it) |
| REC-009 | MEDIUM | ACCEPTED | Lifecycle | `recorder_node.cpp:70-114` | A recorder restart leaves the interrupted run directory unmarked |
| REC-012 | MEDIUM | ACCEPTED | Process | `recorder_node.cpp:353-365` | A dead `ros2 bag` child is reported but not restarted |
| REC-013 | MEDIUM | ACCEPTED | Process | `recorder_node.cpp:311-312,372-388` | `bag_.running()` (waitpid) is polled from status while `stop_run` stops the child: a race on a 3-thread executor |
| REC-016 | MEDIUM | ACCEPTED | Completeness | `recorder_node.cpp:123-146` | No raw PX4 timing topic; `conditioned_execution_sha256` is not in the manifest (ties to X-001 / PC-2) |
| REC-007 | LOW (~~HIGH~~) | ACCEPTED ↓ | Config | `recorder_node.cpp:120` | `config_dir` default `/etc/dyx3/config`, but the launch reads `/etc/dyx3`. No node YAML exists yet (BR-001), so nothing is lost today |
| REC-010 | LOW (~~MEDIUM~~) | ACCEPTED ↓ | Durability | `run_manifest.cpp:164-175` | Atomic rename without `fsync` of the file or directory |
| REC-011 | LOW (~~MEDIUM~~) | ACCEPTED ↓ | Durability | `recorder_node.cpp:220-259,335-345` | Some `write_file_atomic` results are ignored |
| REC-014 | LOW (~~MEDIUM~~) | ACCEPTED ↓ | Process | `recorder_node.cpp:311-318` | A SIGTERM/SIGKILL escalation does not clear `bag_healthy_throughout` |
| REC-015 | LOW (~~MEDIUM~~) | DOUBT ↓ | Load | `ulog_capture.cpp:41-42` | `fwrite` + `fflush` per ULog chunk; KEEP_LAST QoS does not block the publisher, so this does not back-pressure `px4_link` |
| REC-017 | LOW (~~MEDIUM~~) | ACCEPTED ↓ | Lifecycle | `recorder_node.cpp:204-215` | A failed directory creation leaves the lifecycle "recording" |

### HIGH fixes (one recorder change set)
- **REC-001.** Required `min_free_bytes` (e.g. 5 GB), enforced **during** recording (stop the bag, mark the run
  degraded, never touch the shared state). Retention keeps the newest N days or GB, never the active or
  un-offloaded runs. Best: put `/var/lib/dyx3/runs` on its own partition or quota (owner decision).
  - Note: a full disk is fail-safe for motion (the spray-ACK ledger refuses, the RTK config cannot be saved), but
    it stops work.
- **REC-002.** Add `rpp`, `system_gateway` and `recorder`. Check the list against the launch graph in a test.
- **REC-004.** Pre-roll: start the bag at **READY** (the rover is still stopped), collect the parameters
  asynchronously, and mark the RUNNING time in the summary.
- **REC-005.** Either keep one continuous boot-level `.ulg` and record each run's byte/time range, or restart PX4
  logging per run so the header is included. **Owner decision** on the ULog strategy. The SD-card log on the FCU
  remains the backup.
- **REC-006.** Read the running firmware identity from PX4 (e.g. the `ver` / git hash via MAVLink
  AUTOPILOT_VERSION or a DDS topic) into `versions.json`, and flag a mismatch with `firmware_expected_sha`. Bump
  the pin (PC-9) meanwhile.

### Second review (Opus, same commit): cross-check and additions
Opus agreed with REC-001/002/004/005/006 and the race (REC-013), and found **four confirmed defects the first
review missed**. Verified against the code and the upstream ROS 2 Humble sources (rclcpp `humble`, rosbag2
`humble`):

| ID | Severity | Status | Area | Where | Item |
|---|---|---|---|---|---|
| REC-018 | MEDIUM | ACCEPTED | Lifecycle | `recorder_node.cpp:61`; `main.cpp:34-39` | `SyncParametersClient::list_parameters` **throws** on a timeout (rclcpp humble `parameter_client.cpp:544`), and the collector has no `try`. A node that is discovered but not answering, e.g. a stalled `px4_link` (BR-002), **crashes the recorder** at run start (half-written directory) or at run stop (no `summary.json`). From the destructor it can call `std::terminate` |
| REC-019 | MEDIUM | ACCEPTED | Completeness | `recorder_node.cpp:62-65` | `get_parameters` returns an empty vector on a timeout, so the node is recorded `reachable: true, params: {}`: a silent gap |
| REC-020 | MEDIUM | ACCEPTED | Completeness | `recorder_node.cpp:63` | `value_to_string()` formats doubles with `std::to_string`, i.e. **6 fixed decimals** (rclcpp humble `parameter_value.cpp:93`): `1e-7` → `"0.000000"`. **PC-8 amended:** the pairing passes, the value precision fails. Use `%.17g` for doubles and double arrays |
| REC-021 | MEDIUM | ACCEPTED | Durability | `recorder_node.cpp:150-151` | The rosbag2 SQLite defaults (`synchronous=OFF`, journal in memory; `metadata.yaml` only on close) mean a power cut or the SIGKILL escalation can leave a corrupt `.db3`. Add `--storage-preset-profile resilient` (exists in Humble: `sqlite_storage.cpp:136-174`) or move to MCAP (owner decision) |
| REC-022 | LOW | ACCEPTED | Config | `deployment/scripts/start-recorder.sh:9` | The recorder starts with no parameter file, so `vehicle_id`/`operator` are always "unknown" and the config snapshot is always empty (same root as REC-007 / BR-001) |
| REC-023 | LOW | DOUBT (measure) | Load | `dyx3-recorder.service` | No `Nice` / `IOSchedulingClass` / `CPUAffinity`. Measure the recorder's effect on the PXL-004 `fsync` (same ext4 journal) |
| REC-024 | LOW | ACCEPTED | Process | `dyx3-recorder.service`; `start-recorder.sh:9` | `KillMode=control-group` sends SIGTERM to the bag child at the same time as the recorder; the main PID is the `ros2 run` Python wrapper. Exec the node binary and use `KillMode=mixed` |

Also from Opus, folded into existing items:
- the recorder can write `/var/lib/dyx3/rtk` (the credential store), and the ULog `fopen` is not CLOEXEC, so the
  bag child inherits it → BR-005;
- no operator warning for low disk or a recorder in ERROR (the gateway forwards `free_bytes`) → part of REC-001.

Corrections to the Opus review: `/dyx3/vehicle_state` is 50 Hz, not 100 Hz (the 20 ms gate in `px4_link`), so its
bag estimate is about 25 MB/h high (about 1.2–2.3 GB per field day). RPP has 119 parameters per the registry.

---

## 12. backend

Reviewer verdict: REQUEST CHANGES (1 CRITICAL, 3 HIGH, 6 MEDIUM). After verification: **0 CRITICAL, 1 HIGH,
3 MEDIUM, 6 LOW**.
- No route lets an unauthenticated client or a viewer start, resume, arm or enable motion; this is confirmed.
- Every backend stall found ends in a STOP: the gateway keeps its own 2.0 s clock. These are availability
  issues, not unsafe motion.
- The real issue is that the heartbeat is shared across tablets (BE-001).

Confirmed good:
- **Socket.IO refuses missing or unknown tokens on connect** (`realtime/hub.py:30-36`). Heartbeat needs the
  operator role (`:48-53`). E-stop: any authenticated client may assert, only an operator may clear (`:55-63`),
  and a delivery failure is reported with `delivered`.
- **The relay forwards only while the tablet heartbeat is ≤ 1.5 s old** (`realtime/relay.py:46-58`).
  - The gateway runs its own 2.0 s timeout, so a stalled backend event loop causes a **STOP**, not a stuck-alive
    link.
  - Worst case 1.5 + 2.0 = **3.5 s** (≈ 1.2 m at 0.35 m/s), documented as OPEN in `docs/contracts/backend.md`
    section 3.
- **A double Start is refused while a mission is active**, by the mission FSM (busy).
- **No path traversal:** artifact ids must be 64 lowercase hex; run ids are restricted (`storage/runs.py:9-13`).
- **App-plan uploads are stream-limited** to 20 MiB before reading (`routes.py:189-200`).

| ID | Severity | Status | Area | Where | Item |
|---|---|---|---|---|---|
| BE-001 | **HIGH** (~~CRITICAL~~) | ACCEPTED ↓ (owner decision) | Link | `realtime/relay.py:33-37`; `hub.py:38-53` | One global heartbeat timestamp: **any** operator tablet keeps the operator link alive for a mission started by another |
| BE-002 | **HIGH** | ACCEPTED (raised by CR-3: parsed **before** auth, merged into XR-BE-001) | Input | `routes.py:211-224`; `parse_routes.py:18-25` | Multipart DXF: the 20 MiB check happens after Starlette has spooled the whole upload |
| BE-003 | **HIGH** | ACCEPTED (raised by CR-3: parsed **before** auth, merged into XR-BE-001) | Input | `routes.py:312-314,364-379` | RTK JSON write routes take unbounded `dict` bodies |
| BE-004 | MEDIUM (~~HIGH~~) | ACCEPTED ↓ | Input | `mission/service.py:74-109`; `routes.py:224` | DXF planning runs in a worker thread with no runtime or output budget; CPU-bound Python shares the GIL with the event loop |
| BE-005 | LOW (~~MEDIUM~~) | ACCEPTED ↓ | Robustness | `routes.py:238-267` | Artifact read, hash and decode on the event loop for `/missions/{sha}`, `/path`, `/start` |
| BE-006 | LOW (~~MEDIUM~~) | ACCEPTED ↓ | Link | `gateway/client.py:170-179` | `drain()` sits outside the 3 s request timeout |
| BE-007 | LOW (~~MEDIUM~~) | ACCEPTED ↓ | Auth | `auth/tokens.py:45-73`; `hub.py:27-36` | Token revocation needs a backend restart (as documented) |
| BE-008 | LOW (~~MEDIUM~~) | ACCEPTED ↓ | Auth | `routes.py:136-139`; `docs/contracts/backend.md:20` | The contract says heartbeat is "any authenticated", the code says operator: the code is the safer one, so fix the contract |
| BE-009 | LOW (~~MEDIUM~~) | ACCEPTED ↓ | Robustness | `mission/path_artifact.py:245-268` | No retention or capacity check for stored mission artifacts |
| BE-010 | LOW (~~MEDIUM~~) | DOUBT (measure) | Link | `routes.py:202` | `json.loads` of up to 20 MiB on the event loop |

### BE-001 — HIGH — the shared heartbeat (owner decision)
- Confirmed. `OperatorLinkRelay` keeps a single `_last`, and any operator session's heartbeat refreshes it.
  `on_disconnect` clears it only when **no** operator session remains.
- Today both tablets use the **same** operator token (`tablet-1`), so even a per-token lease would not separate
  them.
- Scenario: tablet A starts a mission and then loses Wi-Fi. Tablet B, open in someone's pocket with the app
  still heartbeating, keeps the rover driving with nobody watching A's screen.
- Re-rated HIGH, not CRITICAL: a second operator tablet is a human holding an E-stop. But "is someone actually
  supervising" is not proven.
- **Owner decision:**
  - (a) single control lease: the tablet that starts or resumes owns the heartbeat, a transfer is explicit, and
    other tablets can E-stop only;
  - (b) accept "any operator tablet supervises" and document it.
  - With (a), give each tablet its own token (`tablet-2`) or a per-install device id.
- **Test:** two operator sessions, A owns, B heartbeats, A disconnects → the relay stops.

### BE-002 / BE-003 / BE-004 — MEDIUM — resource admission (one patch)
- All three need an operator token. Bearer authentication is a FastAPI dependency, resolved before the body
  parameters; verify with a test.
- A misbehaving client could still fill the disk or memory, or hold the planner, during a run.
- **CPU-bound planning in a thread holds the GIL** in 5 ms slices. That is not a stall, but it slows the relay
  and could cause a false operator-link STOP if an operator uploads the next mission while the rover is
  marking.
- **Fix (one change):**
  - an ASGI body-size limiter (Content-Length + streamed count) on `/missions`, `/path/parse-dxf` and the RTK
    writes;
  - planning in a separate **process** with a wall-clock and point-count budget, one at a time.
- **Test:** an oversized upload gives 413 before spooling; a pathological DXF is terminated at the budget while
  heartbeats keep flowing.

### LOW items
- **BE-005 / BE-010.** Move the artifact load and the large `json.loads` to `anyio.to_thread` (one-liners). The
  impact is a false STOP at worst.
- **BE-006.** Wrap `drain()` and the reply wait in one `asyncio.timeout`. A hung gateway already ends in a guard
  STOP (its operator-link publication goes stale).
- **BE-007.** The documented procedure is "delete the entry, restart `dyx3-backend`". Live revocation can come
  with QR pairing.
- **BE-008.** Update the contract to "operator".
- **BE-009.** Retention policy together with the recorder (REC review) and the RTK state on `/var/lib/dyx3`.

---

## `dyx3_interfaces` + px4_msgs pin (tracker #4)

Reviewer verdict: changes required (0 CRITICAL, 1 HIGH, 5 MEDIUM, 1 LOW). After verification: **0 CRITICAL,
0 HIGH, 1 MEDIUM, 6 LOW**.
- No interface default can cause unsafe motion or paint.
- **The px4_msgs pin is safe:** the pin `27a7ac9284` and the flashed `8279fa4be3` have identical `msg/`, `srv/` and
  `dds_topics.yaml`. `px4_link` hash-checks 16 message formats and fails closed on a mismatch. PC-9 stays a
  traceability clean-up.

Confirmed good:
- **The default `MotionSetpoint` is safe** (`valid=false`, `mode=STOP`).
- **The NaN rules are documented.**
- **Every service has a defined 0 = OK result**, and ExecuteMission sets its terminal result explicitly.
- **The installer builds px4_msgs from the firmware's own `msg/` + `srv/`** (both plain and versioned).
- Float32 local positions are about 0.5 mm resolution at 5 km, which is not a precision risk at site scale.

| ID | Severity | Status | Area | Where | Item |
|---|---|---|---|---|---|
| IF-003 | MEDIUM | ACCEPTED | Time | `MotionSetpoint.msg:5-15`; `rpp_node.cpp:382-390`; `motion_guard_node.cpp:232-240` | The source-pose sample time is not carried through RPP → guard → px4_link, so pose-to-command age cannot be measured from the data (the latency acceptance gates need it; ties to X-001 / X-003 / PC-2) |
| IF-001 | LOW (~~HIGH~~) | ACCEPTED ↓ | Hot-path | `RppStatus.msg:46-48`; `rpp_node.cpp:398` | A 64-char `string` SHA is copied into `RppStatus` on every 50 Hz tick (one heap allocation; a non-loaned `publish` serialises anyway). Fold into PC-2's bounded, loanable hot-path types (`uint8[32]` digest) |
| IF-002 | LOW (~~MEDIUM~~) | ACCEPTED ↓ | Time | `EstimatorHealth.msg:10-11`; `px4_link_node.cpp` (only `VehicleState` sets `px4_sample_stamp`, line 836) | `EstimatorHealth.px4_sample_stamp` is never filled, so it is always 0 |
| IF-004 | LOW (~~MEDIUM~~) | ACCEPTED ↓ | Safety | `VehicleState.msg:7-18`; `vehicle_state_assembler.cpp:11-12` | `position_valid` / `velocity_valid` reflect XY only; `down_m` / `velocity_down_mps` have no validity flag (no control consumer uses them on a 2D rover) |
| IF-005 | LOW (~~MEDIUM~~) | ACCEPTED ↓ | Doc | `docs/contracts/topics.md:14` | The contract says `/dyx3/vehicle_state` is published "on each FCU sample"; it is a 20 ms timer republishing the latest cached sample |
| IF-006 | LOW (~~MEDIUM~~) | ACCEPTED ↓ | Tests | `interface_abi_test.cpp` | `conditioned_execution_sha256`, `heading_evidence_valid` and `path_travel_m` (0.9 / 0.10 additions) are not pinned; there is no schema fingerprint test |
| IF-007 | LOW | ACCEPTED | Doc | `SetEmergencyStop.srv:1-2` | The accepted E-stop sources (`tablet`, `backend`, `ble`, `physical`) are not documented in the `.srv` |

### Grouping
- **One breaking interface release, 0.14.0, with PC-2:**
  - IF-003: `source_pose_sample_stamp` in `MotionSetpoint`, preserved by the guard;
  - IF-001: a fixed `uint8[32]` digest;
  - IF-004: vertical validity flags;
  - plus RPP-009's yaw-rate source.
  Build and migrate all consumers together.
- **Non-breaking now:** IF-002 (fill the stamp from `EstimatorStatusFlags.timestamp_sample`); IF-005, IF-006 and
  IF-007 (docs and tests).

---

## 13. installer / deployment

Reviewer verdict: REQUEST CHANGES (1 CRITICAL, 6 HIGH, 11 MEDIUM, 7 LOW). After verification: **0 CRITICAL,
5 HIGH, 13 MEDIUM, 7 LOW**.
- Three of the HIGHs are field-proven on rover 01: INS-002 (old installer), INS-004 (killed ssh upgrade) and
  INS-006 (two CH340s).
- No secret was found in the repository or in a world-readable file.

Confirmed in code at `109ec39`:
- No arming or mission check anywhere in `installer/`.
- No `trap` / `systemd-run` / `setsid`.
- `upgrade.sh:8-15` sources every `lib/*.sh` from the **running** release.
- `flock` on fd 9 inherited by children (`common.sh:113-116`).
- `usb_serial.sh:47` dies unless exactly one CH340.
- `ros.env.tmpl:6` `#ROS_DOMAIN_ID=` commented out.
- `network_install.sh:199-204` deletes the hotspot profile on invalid input.
- No `sync` in `release.sh`.

Confirmed good:
- Releases keyed by full SHA, with a `.complete` marker.
- An **atomic** symlink switch (`ln -sfn` temp + `mv -T`).
- Digest-checked artifacts.
- XRCE agent and mavlink-router pinned by commit and verified.
- Secrets 0640 `root:dyx3`, the NetworkManager keyfile 0600, and the PSK never in argv or logs.

| ID | Severity | Status | Area | Where | Item |
|---|---|---|---|---|---|
| INS-001 | **HIGH** (~~CRITICAL~~) | ACCEPTED ↓ | Interlock | `release.sh:201,218,315,325` | Install, upgrade and rollback restart `dyx3-platform` + `dyx3-ros` with **no armed or mission check**. During a mission → offboard loss → disarm (same outcome as GW-002) |
| INS-002 | **HIGH** | ACCEPTED (field-proven) | Origin | `upgrade.sh:8-15` | The upgrade runs the **running** release's installer and pins; only units, templates and the manifest come from the target |
| INS-004 | **HIGH** | ACCEPTED (field-proven) | Atomicity | `release.sh:194-203` | No detach or signal handling; the hotspot is re-activated right after the switch (it drops the ssh session that runs the upgrade); no boot-time finish or revert |
| INS-006 | **HIGH** | ACCEPTED (field-proven) | Health | `usb_serial.sh:47`; `release.sh:203,316` | A fault already present before the upgrade (a LoRa CH340, an unplugged UM982) fails **every upgrade and every rollback**; no health baseline before the switch |
| INS-007 | **HIGH** | ACCEPTED | Fresh | `ros.env.tmpl:6`; `start-backend.sh:13`; `release.sh:221-225` | A fresh install cannot pass its own health gate: `ROS_DOMAIN_ID` is commented out, the backend binds the hotspot address, the hotspot SSID is empty. It then reverts with no actionable message |
| INS-003 | MEDIUM | ACCEPTED | Origin | `release.sh:163,178`; `artifacts.sh:86-90`; `dyx3-env.sh:18-19` | **A firmware-pin change cannot be installed by `dyx3-upgrade`:** px4_msgs is built for the old pin while the target's launchers want the new one. **Blocks PC-9 by upgrade** (same fix as INS-002) |
| INS-005 | MEDIUM (~~HIGH~~) | ACCEPTED ↓ | Health | `health_check.sh:41-42,80` | One `is-active` sample right after restart, and the gateway check is a socket-file test: a crash-looping release can pass. `dyx3-rollback` still works |
| INS-008 | MEDIUM (~~HIGH~~) | DOUBT (bench) | Atomicity | `release.sh:192-194` | No `sync` before `.complete` and the switch: a power cut right after an upgrade can leave `current` → files not yet written |
| INS-009 | MEDIUM | ACCEPTED (field-proven) | Atomicity | `common.sh:113-116` | Lock fd 9 is inherited: `nmcli` held `install.lock` after the killed upgrade. Fix: `"$@" 9>&-` |
| INS-010 | MEDIUM | ACCEPTED | Network | `network_install.sh:199-204,215-266` | Any `hotspot.env` validation failure **deletes** the working access point (the operator's only link without a router) |
| INS-011 | MEDIUM | ACCEPTED | Atomicity | `systemd_install.sh:19-37` | Unit files are never removed; a rollback keeps the newer release's units |
| INS-012 | MEDIUM | ACCEPTED | Supply | `artifacts.sh:146-153`; `release.sh:174-177` | A digest mismatch in `auto` mode silently falls back to a source build (tampered looks like missing) |
| INS-013 | MEDIUM | ACCEPTED | Supply | `artifacts.sh:79-106` | The artifact's `rclcpp` version is recorded but never compared with the rover's |
| INS-014 | MEDIUM | DOUBT | Atomicity | `artifacts.sh:156-158` | The px4_msgs archive contains its own `.complete`; an interrupted extraction can look complete |
| INS-015 | MEDIUM | ACCEPTED | Fresh | `dependencies.sh:36-74`; `usb_serial.sh:124-131` | A fresh install compiles the XRCE agent, mavlink-router and the ch341 module and needs the internet (the goal says zero compiles, < 15 min); the module is unsigned |
| INS-016 | MEDIUM | ACCEPTED | Atomicity | `release.sh:209-219,320-326` | Revert and restore are never health-checked; a failed gate discards a good source build |
| INS-017 | MEDIUM | DOUBT (rover) | Network | `network_install.sh:412-434` | No-auto-updates does not cover snapd, and the L4T kernel packages are not held (a kernel move breaks ch341 → INS-006) |
| INS-018 | MEDIUM | ACCEPTED | Tests | `installer/tests/run_tests.sh` | No interruption, origin, interlock, two-CH340, archive-escape or fresh-install tests |
| INS-019…025 | LOW | ACCEPTED | various | see the review | `FCU_KEEP_DHCP` not persisted; tar member check is a string prefix; README disagrees (hotspot on 192.168.3.x); `dyx3-health` without root; shims written in place; unpinned pip / ROS key from a branch / CI `--clobber`; the hotspot/FCU overlap check is a string match |

### Fix order (installer change set)
1. **INS-001:** `require_rover_idle` before every switch or restart; unknown state = refuse; bench override.
   Needs X-016.
2. **INS-002 + INS-003:** re-exec the **target's** installer after the fetch, with an installer API version.
   The first upgrade carrying it still runs the old one. **Do the PC-9 pin bump after this lands**, or by a
   fresh install.
3. **INS-004 + INS-009:** self-detach with `systemd-run`; `9>&-`; skip the hotspot re-activation when the keyfile
   is unchanged and move it after health; mark `upgrade_in_progress` for a boot-time finish or revert.
4. **INS-006 + INS-016:** a health baseline before the switch (an already-failing check cannot blame the new
   release); a health check after the revert.
5. **INS-007:** `ROS_DOMAIN_ID=42` in the template (decided), and refuse early with a list of missing per-rover
   inputs.

---

## Cross-review (independent Opus agents, 2026-10-09)

Each hot-path part was re-read in full by an independent agent. The agent checked every recorded finding and
looked for anything missed. Every new item below was verified by me against the code before being recorded.

### CR-2 — `dyx3_motion_guard` + `dyx3_px4_link`

Verdicts: the agent **agreed with all 19 recorded verdicts** (MG-001…007, PXL-001…008, IF-002/004/005, X-010).
One addition to PXL-001: `COM_FAIL_ACT_T` does not delay a Disarm (`framework.cpp:490-493`), and disarm zeroes
the motors at once (`RoverDifferential.cpp:90-93`). So the companion-loss bound is `COM_OF_LOSS_T` (1.0 s) + one
commander cycle.

**Qualifiers to earlier "confirmed good" lines:**
- MG: "STOP is immediate, never ramped" is true **on the Jetson only**. PX4 ramps it (XR-GPX-002).
- SP: "px4_link gives the watchdog's OFF priority" applies over **queued** requests only, **not over the in-flight
  one** (XR-GPX-001).

Time to the first STOP on the wire:
| Fault | First STOP |
|---|---|
| RPP silent | ≤ 0.23 s |
| Guard dead or starved | ≤ 0.21 s |
| E-stop | ≤ 10 ms |
| A status topic silent | 0.52 s |
| `px4_link` dead / hung / SIGTERM, or `SetOffboard(false)` | **no STOP sent**; PX4 holds the last setpoint 1.0 s, then disarms |

New findings (verified by me against the code and the firmware):
| ID | Severity | Status | Where | Item |
|---|---|---|---|---|
| XR-GPX-001 | **CRITICAL** | ACCEPTED (code path; needs one lost ACK) | `px4_link_node.cpp:26,544-566,659,703` | **Spray OFF can wait up to 5 s behind an in-flight ON.** While a transaction is in flight, `dispatch_next_spray_transaction` returns. The in-flight request clears only on its ACK or after `kSprayTransactionTimeoutS = 5.0`. The watchdog's OFF has priority over the queue only. FCU out-topics are best-effort, so **one lost ACK** after an ON means a line-end OFF, a watchdog OFF or an E-stop-driven OFF reaches PX4 up to 5 s late: up to 5 m of paint at 1 m/s, or a puddle at an E-stop. Fix: an OFF pre-empts the in-flight ON (fail its ACK, dispatch at once); in-flight timeout about 0.3 s; keep republishing the in-flight request. Test: ON dispatched, no ACK, watchdog OFF → command 187 OFF on the wire within 50 ms |
| XR-GPX-002 | **HIGH** | ACCEPTED (fw + params); distance to measure | fw `src/lib/rover_control/RoverControl.cpp:126-134` (+ `throttleControl` :59-67); `3wd_6x_carry_from_proto.params` `RO_DECEL_LIM 0.3` | **Every Jetson STOP, the E-stop included, is ramped by PX4 at 0.3 m/s²:** from 1.0 m/s that is ≈ 3.3 s / 1.7 m, from 0.35 m/s ≈ 1.2 s / 0.2 m. The E-stop never disarms. A crash (1.0 s, then an immediate disarm stop) can stop faster than a software E-stop. Fix: E-stop also disarms, or a "hard stop" that bypasses the slew; set `RO_DECEL_LIM` from a measured braking value. **Owner decision; part of PC-5** |
| XR-GPX-003 | MEDIUM | DOUBT (firmware; deferred to the firmware review) | fw `DifferentialSpeedControl.cpp:113-117`; `dds_topics.h.em` | PX4 age-checks only `offboard_control_mode`. If one `/fmu/in` setpoint reader breaks while the heartbeat flows, PX4 keeps the last setpoint with no time limit |
| XR-GPX-004 | MEDIUM | ACCEPTED | `px4_link_node.cpp:182-183` vs fw `uxrce_dds_client/utilities.hpp:80,137` | **ULog streaming never works:** the FCU writers are BEST_EFFORT and `px4_link` subscribes RELIABLE, so they never match and no chunk is ever received. The test passes because its fake FCU publishes RELIABLE. (Makes REC-005 moot until fixed; the SD-card log is the only ULog today) |
| XR-GPX-005 | MEDIUM | ACCEPTED | `px4_link_node.cpp:556-574,626-637` | Every ACK of a reasserted spray command counts as "unmatched" and logs an **unthrottled WARN** on the writer executor (2 Hz idle, 20 Hz in watchdog bursts) |
| XR-GPX-006 | MEDIUM | DOUBT (bench) | `px4_link_node.cpp:388-393` (`system_clock`) | A backward wall-clock step of more than 1 s (first NTP sync mid-run, site LAN coming up) makes `offboard_control_mode` look old → offboard loss → disarm mid-line. Fix: slew-only time sync while armed; a "clock synced" mission precondition |
| XR-GPX-007 | MEDIUM | ACCEPTED | `px4_link_node.cpp:772`; `offboard_heartbeat.cpp:21-26` | The heartbeat carries the guard's command outside the Active state (the contract says an explicit STOP). On a link loss Active → Prestream, and OFFBOARD is **re-requested automatically** when the link returns (the contract says never silently). Mitigated by the guard's nav_state gate and the mission PAUSE |
| XR-GPX-008 | LOW | ACCEPTED | `limits.cpp:51-69`; `RO_YAW_RATE_LIM 30` | The guard's 0.45 rad/s yaw envelope does not apply in TRACK_HEADING (NaN rate); PX4 allows 0.52 rad/s |
| XR-GPX-009 | LOW | ACCEPTED | `motion_guard_node_test.cpp:284-313` | Tests do not prove E-stop immediacy (they run 0.3 s first) or the 0.5 s freshness boundary |

### CR-3 — `dyx3_system_gateway` + backend relay

Verdicts on recorded findings:
- **BE-002 and BE-003 are raised to HIGH, CONFIRMED.**
  - FastAPI reads and parses the request body (`fastapi/routing.py:422-425`) **before** `solve_dependencies`
    (`:473`), so the bearer token is checked only **after** the whole body is buffered.
  - Every multipart and JSON-body route is reachable **without a token**: `/estop`, `/vehicle/*`,
    `/spray/manual`, the RTK writes, `/missions`, `/path/parse-dxf`.
  - The earlier assumption that "auth comes first" was wrong.
- **GW-002 stays HIGH, but the trigger is narrower.** A peer that closes with unread data gives
  ECONNRESET → POLLERR, which is caught at `ipc_server.cpp:165` before the write.
  - SIGPIPE needs the peer's receive queue empty AND the gateway's `out` buffer non-empty.
  - Deterministic trigger: a line rejected on the IPC thread (`gateway_node.cpp:268-275`, also "busy" `:286`)
    followed by an immediate close.
  - Otherwise a microsecond window after each telemetry append.
  - The fix is unchanged.
- **GW-001: two more effects.** `alive` counts *any* client, and the link state is not reset when the backend
  disconnects. With a second client connected, a backend crash becomes about 2.1 s to STOP instead of about
  0.12 s.
- **BE-001:** the agent rates it CRITICAL by the rubric. The recorded HIGH stands **only if the owner accepts
  option (b)** ("any operator tablet supervises"); otherwise CRITICAL.
- **GW-004:** 3.63 s is the no-lag case; with backend event-loop lag it reaches about 5.6 s (XR-BE-002).
- Confirmed: a hung but connected backend **cannot** keep `alive` (heartbeats come only from the relay task).
  A backend crash with the backend as the only client → STOP in about 0.12 s.
- All other GW and BE findings: agreed.

New findings:
| ID | Severity | Status | Where | Item |
|---|---|---|---|---|
| XR-BE-001 | **HIGH** | ACCEPTED (verified) | `routes.py:143,170-181,313,365,372`; FastAPI `routing.py:422-473` | **Pre-auth unbounded body parsing on every body route.** An unauthenticated LAN host can POST a GB to `/api/estop`: memory growth, OOM risk, an event-loop stall → false STOP and delayed real E-stops. Fix: pure-ASGI middleware that checks the bearer token and the size cap **before** reading the body (merges BE-002 and BE-003) |
| XR-BE-002 | MEDIUM | DOUBT (test) | `relay.py:36-37`; `hub.py:52` | A heartbeat is stamped when it is processed, not when it was received. After a loop stall, queued heartbeats replay as fresh and extend the link by up to the stall time |
| XR-BE-003 | MEDIUM | DOUBT (test) | `relay.py:60-63,71`; `routes.py:125-126` | The relay task catches only `GatewayError`. Any other exception ends it silently (a permanent STOP, fail-safe) while `/health` still shows `tablet_alive` |
| XR-BE-004 | MEDIUM | DOUBT (rover) | `dyx3-backend.service` (`ProtectSystem=strict`, no `PrivateTmp` / `TMPDIR`) | Uploads above 1 MiB spool to `/tmp`, which may be read-only under strict, so DXF upload would fail in production. Fix: `PrivateTmp=yes` or `TMPDIR=/var/lib/dyx3/tmp` |
| XR-GW-001 | MEDIUM | ACCEPTED | `gateway_node.cpp:244`; `hub.py`, `relay.py` | No audit log of E-stop assert/clear (who), link alive transitions, client drops or service timeouts |
| XR-GW-002 | LOW | ACCEPTED | `client.py:122,139` | At `max_clients`, the reconnect resets its backoff on connect, so accept-then-close flaps at 5 Hz |
| XR-GW-003 | LOW | ACCEPTED | `ipc_server.cpp:32,78` | Unconditional `unlink` of the socket path: a second gateway (a manual launch) steals or deletes the live socket |
| XR-GW-004 | LOW | ACCEPTED | `gateway_node_test.cpp:32,238-242`; `test_api.py:24-35` | Tests pass for the wrong reason: `estop_answers` unused (the timeout path is untested); the auth tests send no body, so they cannot catch XR-BE-001 |

---

## Cross-part items (raised by the part reviews; owned by later parts)

| ID | Owner part | Item | Status |
|---|---|---|---|
| X-001 | `px4_link` | Freshness stamped on receipt; `timestamp_sample` not used for age | open, review with PXL |
| X-002 | `px4_link` / RPP | `xy_reset_counter` and `delta_xy` are published but RPP uses its own jump heuristic | open, review with PXL |
| X-003 | chain | Four unsynchronised timer hops pose → PX4 (about 35 ms typical / 70 ms worst) | open, measure first |
| X-004 | `motion_guard` | Numeric input policy and response timing of the guard | in MG review |
| X-005 | bringup / systemd | Shared FIFO CPU placement, start-up and shutdown ordering | open |
| X-006 | `px4_link` / bringup | `px4_link`, the final 100 Hz writer to PX4, runs under **normal scheduling**, unlike RPP and the guard (`control_graph.launch.py:33-34`) | DOUBT, measure |
| X-007 | `px4_link` | Root cause of RPP-009: no angular-rate subscription | ACCEPTED, HIGH |
| X-008 | `px4_link` / firmware | Measure the real stop time for three separate events: the guard sends STOP; the guard dies while `px4_link` lives (0.2 s gate); the whole graph dies (PX4 offboard loss: `COM_OF_LOSS_T` is not in the baseline, `COM_OBL_RC_ACT` 7, upstream #27514) | open, measure (from MG review) |
| X-009 | mission / bringup | Prove that a systemd graph restart or an E-stop clear can never resume a mission without the operator | open, test (from MG review) |
| X-010 | `px4_link` | On a graph stop every node gets SIGINT together, so only the last hop can guarantee a final STOP: `px4_link` must publish STOP and stop the offboard heartbeat cleanly before exiting | ACCEPTED: confirmed `main.cpp:15-17`; part of PXL-001 |
| X-011 | PX4 parameter baseline | `COM_RCL_EXCEPT` = 4 (bit 2 = Offboard, firmware `commander_params.c:633-645`): **RC loss triggers no failsafe in OFFBOARD**. Carried from the prototype. The RC kill switch still works while the RC link is alive. Owner decision: keep it (the tablet link and the guard govern autonomy) or clear bit 2 so RC loss stops autonomous runs. Test it with production-readiness #5 | open, owner decision (found in PXL verification) |
| X-012 | PX4 firmware / hardware / spray | **Physical valve close when every Jetson path is lost.** The controller and the watchdog both reach the valve only through `px4_link` → DDS → PX4. Prove on hardware what the valve output does on disarm, on offboard loss (1.0 s → disarm) and on loss of actuator commands; measure it with the valve driver. Owner decision (open since 2026-10-07): (a) a secondary UART path, (b) a PX4 companion-loss failsafe that disarms, plus a disarmed-output level that is valve-closed | **open: blocks fail-closed sign-off** |
| X-013 | installer / health | `installer/lib/health_check.sh` (deep graph check, about line 166) lists `/dyx3_mission /motion_guard /px4_link /spray /system_gateway`, **without `/rpp`**. Missing nodes and absent `/fmu` topics are WARN, not FAIL | open, installer review (from BR review) |
| X-014 | architecture / network | `docs/architecture/...V1.md` §4.3 says "no router, no site LAN", but the rover now runs a site LAN (`network.env`, 192.168.3.0/24) next to the hotspot by owner decision of 2026-10-09. Amend §4.3 and the operator-link reasoning (§4.3.1) | open, doc (from BR review) |
| X-015 | backend / app / network | On the site LAN, tablet Bearer tokens travel over **plain HTTP**, and the UDP beacon cannot prove rover identity, so a LAN attacker can sniff tokens or spoof a rover. Owner decision: TLS (self-signed, pinned per rover at pairing, fits QR pairing), a VPN, or the hotspot only for production | open, owner decision (from BE review) |
| X-016 | installer / gateway / mission | The installer needs an authoritative, root-readable "rover idle" state (disarmed AND mission not RUNNING; unknown ≠ idle) to refuse install, upgrade or rollback mid-run (INS-001) | open (from INS review) |
| X-017 | installer / gnss_rtk / USB | CH340 provisioning counts adapters instead of selecting by `ID_PATH`, and re-enumeration de-authorises every CH340 (`usb_serial.sh:315-326`): this blocks the LoRa radio next to the UM982 (field history; owner: LoRa after the app is solid) | open (from INS review) |
