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
| 2 | `dyx3_motion_guard` | 2026-10-09 | 2026-10-09 | 0 / 0 / 3 / 4 | open |
| 3 | `dyx3_px4_link` | 2026-10-09 | 2026-10-09 | 0 / 3 / 1 / 3 | open |
| 4 | `dyx3_interfaces` (+ px4_msgs pin) | — | — | — | prompt issued |
| 5 | `dyx3_mission` | — | — | — | |
| 6 | `dyx3_gnss_rtk` | — | — | — | |
| 7 | `dyx3_spray` | — | — | — | |
| 8 | `dyx3_geometry` | — | — | — | |
| 9 | `dyx3_bringup` + systemd (RT, CPU, restart) | — | — | — | |
| 10 | `dyx3_system_gateway` | — | — | — | |
| 11 | `dyx3_recorder` | — | — | — | |
| 12 | backend | — | — | — | |
| 13 | installer / deployment | — | — | — | |
| 14 | tablet app (`Three_Wheel_v2` `App-Polish`) | — | — | — | |
| 15 | PX4 firmware rover path (`dyx-3wd-production`) | — | — | — | |
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
| PC-8 | The prototype bag manifests pair `as_run_config.rpp_params` names with the wrong values. The recorder must write correct name/value pairs | open: check in the recorder review | `dyx3_recorder` |
| PC-9 | `installer/pins/firmware.pin` still pins `27a7ac9284`, but the rover runs `8279fa4be3`. `msg/`, `srv/` and `dds_topics.yaml` are identical between the two, so px4_msgs on the rover is correct. Bump the pin to `8279fa4be3` for traceability; a px4_msgs rebuild follows at the next upgrade | open (hygiene) | |

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
