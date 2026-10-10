# Production review — transport, timing, event handling, backend ↔ tablet (2026-10-10)

**Scope:** what blocks *reliable* autonomous operation, judged on the code at `master` `750784c`
(firmware `8279fa4be3`, rover 01 on `rover-7f9651d48f`). Areas: PX4 ↔ Jetson DDS transport and QoS,
how commands reach PX4, the node-to-node control chain and its timing, the mission state machine,
the gateway ↔ backend IPC, the backend's async execution and the Socket.IO path to the tablet, and
the supporting services (RTK, spray, recorder, bringup).
**Method:** four read-only code traces (one per area, facts with `file:line`), every finding below
re-read in the source by the reviewer, cross-checked against the field evidence in
`docs/agents/HANDOFF.md` (2026-10-10 entries) and against the existing register
`docs/reviews/production/open_items.md` so that known items are not re-reported as new.
**Not done:** no build, no test, no bag was opened (bags are not in Git). Timing claims below are
reasoning from the code plus the numbers the field entries recorded; each one names the
measurement that would settle it.
**Severity scale:** the register's (CRITICAL unsafe motion / HIGH breaks the latency-rate-recovery
goal / MEDIUM degraded or contract-vs-code / LOW hardening).

---

## 0. Verdict in five lines

1. **The motion safety chain is sound.** Every hop (RPP → guard → px4_link → PX4) fails to an
   explicit zero on its own steady clock, PX4 disarms 0.5 s after the stream stops (measured
   0.69 s, 9 mm roll), E-stop is prioritised end to end, and the DDS handshake fails loud. Nothing
   found here commands uncommanded motion.
2. **The chain is event-driven end to end** (hardening C1/C4): a PX4 sample triggers RPP, the guard
   and the write to PX4 in callbacks, not timers. The register's X-003 ("four unsynchronised timer
   hops, 35/70 ms") is structurally addressed; one bag measurement closes it (§6 M1).
3. **What blocks reliable *operation* is recovery, not safety:** PX4's arm/mode refusals are
   invisible (every refusal is a 2–4 s timeout with no reason), any node exit restarts the whole
   graph and disarms, and a paused mission has no re-engage path and no persisted progress. A
   single hiccup mid-line means a visible defect in the paint and a manual restart.
4. **The transport is correct but not reproducible or pinned:** rover 01 is loopback-only by a hand
   edit the template does not carry, nothing pins the RMW or a DDS profile, nothing enforces the
   domain match, and px4_link — the final 100 Hz hop — is the only non-RT process in an RT chain.
5. **Backend ↔ tablet is in good shape:** bounded, asynchronous, prioritised, stress-tested. The
   remaining weakness is the tablet's own JS thread, which is why the owner's stop-policy decision
   (physical E-stop vs RC kill) matters more than any backend change.

---

## 1. Production blockers (ranked)

### P1 — HIGH — PX4 refusals are invisible: no ack path for arm and mode commands
*Known in part (HANDOFF open item 7, register PXL / X-008). Escalated here because it is the first
thing an operator hits.*

- `px4_link` publishes `VEHICLE_CMD_COMPONENT_ARM_DISARM` / `DO_SET_MODE` and confirms only from
  `vehicle_status` (`px4_link_node.cpp:949-963`). `vehicle_command_ack` is consumed only for spray
  (`:712-744`); `ArmDisarm.srv` defines `REASON_REJECTED_BY_FCU` and nothing sets it (grep: 0).
- Consequence: a PX4 "Preflight Fail" or an arm denial becomes `REASON_TIMEOUT` after 2.0 s; the
  mission reports `ARM_TIMEOUT` (11) after 4 s; the tablet shows a timeout. Field evidence: "PX4
  refused every arm after a run" (HANDOFF 2026-10-10 evening) took a field session to diagnose.
- The MANUAL release (`fe1770b`) is bounded (0.5 s retry, 1.5 s) and still unverified live (item 6).
- **Fix (small, in `px4_link`):** match `VehicleCommandAck` for commands 400/176 on
  `(command, target_system, target_component)` — the same echo mechanism the spray path already
  proves — and answer the pending service at once with `REASON_REJECTED_BY_FCU` + the PX4
  `result`. Add `pre_flight_checks_pass`, `failsafe` and `arming_state` to the mission's pre-arm
  gate reason so Start is refused *before* the arm is attempted (item 7).

### P2 — HIGH — No recovery from a transient: progress in memory, no re-engage, whole-graph restart
*MS-002 and the launch policy are registered as owner decisions; the combined effect is new.*

- `control_graph.launch.py:65`: any of the six nodes exiting shuts the whole launch down
  (`DERIVED`, lines 9-10) → px4_link's 0.3 s STOP burst → PX4 offboard loss 0.5 s → disarm.
  A crash in the gateway or spray (non-safety code) therefore ends the run exactly like a crash in
  the guard. GW-002 (SIGPIPE) was this path in the field before it was fixed.
- `mission_node`: point progress lives in memory only (MS-002); after a graph restart the mission
  is IDLE.
- From PAUSED the only exits are `resume` (requires the full gate, i.e. `nav_state == 14` and
  armed) or `abort`. px4_link never re-requests OFFBOARD (`offboard_heartbeat.cpp`, state
  `Lost`), and the mission never re-runs ARMING→ENGAGING from PAUSED. So after any PX4-side
  OFFBOARD or arm loss the operator's only verb is abort → new start from the first point, with the
  rover repositioned by hand.
- **Why it matters here:** the painted line is the product. A mid-line stop followed by a restart
  from point 0 is a visible defect, not an inconvenience.
- **Fix (owner decision, then two medium changes):**
  1. Persist the point journal per `(mission_id, run_index)` and accept `start` with
     `resume_from_point` (or re-plan from the current pose in the app, since the tablet is the
     trajectory author).
  2. Add `PAUSED → ARMING → ENGAGING → RUNNING` as an explicit re-engage path (the sequencer already
     has both steps), gated exactly like a start.
  3. Split `dyx3-ros` into a control unit (px4_link, guard, rpp) and a services unit (mission,
     spray, gateway). The guard already fails to zero when `mission/state` goes stale (0.5 s), so a
     services crash becomes a PAUSED-like stop instead of a disarm. Keep `on_exit=Shutdown` inside
     each unit.

### P3 — HIGH — The DDS transport is right on rover 01 by hand, not by the repo
*Partly known (HANDOFF:979 item 1, PC-7a/7b). The localhost half is still open; the rest is new.*

- `ros.env.tmpl:11` ships `#DYX3_ROS_LOCALHOST_ONLY=1` commented out ("OPEN"); `dyx3-env.sh:42`
  exports nothing by default. Rover 01 has `ROS_LOCALHOST_ONLY=1` by hand (HANDOFF:979). A fresh
  rover's graph discovers over `eth0` and the hotspot `wlan0`: a tablet-side laptop or the 4WD
  rover on domain 42 joins the graph (architecture §4.4 names this failure explicitly). Violates
  the "every hardware fix persists" rule (CLAUDE.md §4).
- No file in `deployment/` or `installer/` sets `RMW_IMPLEMENTATION`, `FASTRTPS_DEFAULT_PROFILES_FILE`
  or any XML profile. The RMW and the transport mix (SHM + UDP) are whatever the Humble image
  defaults to. Architecture §8: "A `fastdds_no_shm.xml` exists today for a reason. Re-derive that
  reason; neither inherit nor drop the file blindly." It was dropped blindly.
- Nothing reads PX4 `UXRCE_DDS_DOM_ID` or `UXRCE_DDS_PTCFG`; `health_check.sh:358-392` infers the
  match from `session_alive` and only WARNs (PC-7a).
- `dyx3-ros` `Requires=dyx3-platform` orders the *script*, not a listening agent; no
  `ExecStartPre`/wait. Tolerated today because px4_link stays in `HandshakePending` → STOP, so this
  is availability only.
- **Fix:** `DYX3_ROS_LOCALHOST_ONLY=1` as the template default; pin `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`
  and ship a profiles XML under `deployment/network/` (see P4 for what goes in it); make the
  domain/PTCFG check a FAIL that reads the baseline `config/px4/*.params`; health FAIL (not WARN)
  on "session dead while the agent listens".

### P4 — HIGH (measure first) — px4_link is the only non-RT hop, its writers can block, and its executor is shared
*X-006, PXL-003 and PC-2 are registered as DOUBT. This adds the mechanism and the test.*

- Scheduling: `RT_CONTROL_PACKAGES = {motion_guard, rpp}` (`control_graph.launch.py:33-34`);
  px4_link runs `SCHED_OTHER` on any CPU while RPP and the guard are FIFO 80 on CPU 4. The chain's
  deadline is set by its weakest hop, and that hop shares the Jetson with the backend (Python), the
  recorder's zstd bag compression, the agent and `mavlink-router`.
- QoS: all `/fmu/in` writers are `reliable, KEEP_LAST(1)` (`px4_link_node.cpp:94-115`), required
  because PX4's uXRCE readers are reliable (contract §1). With Fast DDS a reliable writer whose
  reader stops acknowledging can block `publish()` up to the writer's `max_blocking_time`
  (default 100 ms) per call — the vendor documents this for KEEP_ALL and says only "discard the
  oldest" for KEEP_LAST, so **treat it as unproven until the bench test below runs**. Five
  publishes per cycle → up to 0.5 s per writer cycle in the single executor thread that also runs
  the staleness monitor, the status publisher and the service replies. Safety is unaffected (PX4
  disarms on its own), observability and recovery are.
- The one agent-loss test on record (`pkill MicroXRCEAgent`, HANDOFF 2026-10-10 11:30) ran
  **disarmed with no OFFBOARD session**, i.e. with the writer not writing. The blocking path was
  never exercised.
- Executor load: one `SingleThreadedExecutor` serves the 100 Hz writer, RTCM (6 Hz), spray
  transactions, the handshake, both services and **ULog streaming** (`ulog_streaming_enabled`
  default true). ULog over DDS only started working when XR-GPX-004 fixed its QoS; its load on this
  executor has not been measured since.
- **Fix:** `chrt -f 70` (below the guard) + a CPU for px4_link in the launch prefix; a profiles XML
  with a small `max_blocking_time` (≤ one period) for the `/fmu/in` writers; `ulog_streaming_enabled`
  false in production until measured (the SD log is pulled anyway via `bench_tools/ulog_pull.py`).
  Then M1 and M2 in §6 decide whether PC-2's single-process composition is still needed.

### P5 — HIGH (policy, owner) — The only software stop reaches the rover through the tablet's JS thread
*Open item 1 for the owner; restated with the code evidence.*

- The E-stop path is well built: Socket.IO `estop` → `gw.request` → Unix socket → gateway inbox
  (E-stop exempt from the 256 cap, sorted first) → `set_emergency_stop` → the guard latches and
  publishes STOP inside the service callback (`motion_guard_node.cpp:144-163`). Bounded at every
  hop, 1 s gateway timeout, honest verdict returned.
- But the sender is a React app whose JS thread blocked > 5 s on 2026-10-10 (HANDOFF "Tablet link
  drops"), and the operator link is no longer a motion gate by owner decision. The RC kill is the
  only stop that does not go through the tablet. BLE is explicitly not an E-stop path (§4.3.1).
- **Decision needed before customer runs:** a physical E-stop on the vehicle (hard-wired, not
  through the Jetson), or the RC kill as the designated stop with an RC-on precondition in the
  pre-arm gate (today `COM_RCL_EXCEPT 7` makes RC loss invisible in OFFBOARD, so a dead RC is not
  detected either).

---

## 2. HIGH / MEDIUM findings by area

### 2.1 PX4 ↔ Jetson transport (`dyx3_px4_link`, agent, environment)

| # | Sev | Finding | Where | Status |
|---|---|---|---|---|
| T1 | MEDIUM | **Wall clock in the PX4-bound timestamps and in the mission's freshness.** px4_link stamps every `/fmu/in` message with `system_clock` (`:467-473`, by contract §7). The mission node runs *all* its timeouts and freshness (RPP status 0.5 s, gate 0.5 s, arm/offboard deadlines, READY timeout) on `get_clock()->now()` = system time (`mission_node.hpp:81`, `mission_node.cpp:352-356`), unlike RPP/guard/px4_link which use `steady_clock`. The repo persists no time-sync policy (no chrony/timesyncd config anywhere); the Orin Nano has no battery RTC, so the first NTP sync over the LTE dongle is a step. A forward step > 0.5 s while RUNNING can pause the mission (`RPP_STALE`) in a ≤ 100 ms window; the px4_link side (XR-GPX-006) is worse: PX4 sees `offboard_control_mode` old → offboard loss → disarm. | `mission_node.cpp:352`, `px4_link_node.cpp:467`, XR-GPX-006 | mission part **new** |
| T2 | MEDIUM | **Timesync warm-up gate not built** (F-tasks A1.4, contract "Timesync evidence"). Values are recorded, nothing gates. Control impact is small because the firmware converts stamps itself; the evidence (bag ↔ ulog correlation) is what suffers. | contract §"Timesync evidence" | known, no criterion |
| T3 | MEDIUM | **Recovery after agent restart is 15.8 s vs 5 s target** (STEP1-R1): platform restart delay 2 s, PX4 client reconnect ~8 s (firmware), handshake ≤ 5 s (`handshake_timeout_s` 5, retry 1). The handshake part is ours. | `px4_link_node.cpp:413`, HANDOFF 11:30 | known |
| T4 | LOW | Seq-reset semantics differ between hops: the guard needs 3 strictly increasing commands after a reset (`session_accept_count`), px4_link discards one and STOPs one tick. Both safe; document the combined worst case (3 + 1 ticks ≈ 80 ms of STOP after an RPP restart). | `fail_to_zero.cpp:13-17`, `rover_setpoint_writer.cpp:76-84` | new, doc |
| T5 | LOW | `/dyx3/gnss_report` is `QoS(5)` with no explicit reliability (defaults reliable) — inconsistent with the "declare QoS per topic" rule, harmless. | `px4_link_node.cpp:251` | new |

**What is right and should not be re-litigated:** per-topic staleness on measured periods with
`steady_clock` (covers upstream #27388); explicit STOP, never silence, while the link is healthy;
heartbeat withdrawn only when no trustworthy zero exists; handshake re-armed on session reset;
0.3 s STOP burst on SIGTERM; `SIGPIPE` ignored; `mlockall`; no allocation in `publish_setpoint_set`.

### 2.2 Control chain timing (`rpp`, `motion_guard`, `mission`)

| # | Sev | Finding | Where | Status |
|---|---|---|---|---|
| C1 | — | **Event-driven chain confirmed.** VehicleState is published in the local-position callback; RPP ticks in its VehicleState callback (`rpp_node.cpp:58-94`); the guard decides in its command callback (`motion_guard_node.cpp:92-97`); px4_link writes in its command callback (`:276-279`). Each hop's timer is a watchdog only (1.5-period thresholds). Hop timeouts: RPP pose 0.5 s → guard command 0.2 s → px4_link command 0.2 s → PX4 `COM_OF_LOSS_T` 0.5 s; px4_link's own `stale_local_position_s` 0.2 s bounds the first. | — | positive |
| C2 | MEDIUM | **Mission load on the FIFO-80 thread, retried at 1 Hz forever on failure** (file I/O, hashing, `ostringstream`, `RCLCPP_*`, `std::vector` growth in `load_mission_impl`, called from `step()` at `rpp_node.cpp:363-364`). A persistent artifact error hammers disk from the control thread while STOP is published. | `rpp_node.cpp:256-354` | RPP-001 known; retry loop new |
| C3 | MEDIUM | **Guard applies no acceleration/jerk limits in production** (`profile_shaping_test_mode` false; accel/decel/jerk are not ROS parameters, `limits.hpp:22-25`, node comment 216-217). Architecture §7.6 lists them. PX4's `RO_ACCEL_LIM/RO_DECEL_LIM` slew instead. Documented deviation; record it in the guard contract so a future RPP bug (a step command) is understood to reach PX4 unshaped. | `motion_guard_node.cpp:216` | new, doc |
| C4 | LOW | RPP loop jitter is the interval between `step()` calls, not the duration of `step()`; in event-driven mode it measures PX4's sample cadence, not RPP's cost (`diagnostics.hpp`). Open item 9 ("max 20 ms, 4 overruns") is therefore mostly DDS/PX4 cadence. Add a `step()` duration max to `RppStatus` to separate the two. | `rpp_node.cpp:361`, `diagnostics.hpp:12-17` | new |
| C5 | LOW | `RppStatus.conditioned_execution_sha256` copies a `std::string` per status publish (IF-001). | `rpp_node.cpp:477` | known |
| C6 | LOW | The mission node's parameter callback is IDLE-only for the nine doubles and refuses in terminal states too (COMPLETED/ABORTED/ERROR); fine, but contract says "disarmed, no mission". | `mission_node.cpp:275-279` | new, doc |

**Right, keep:** services never block or lock (mission uses `std::async` for load/place and
`async_send_request` for px4_link; E-stop decides inline); IDLE_ONLY/RESTART refused with reasons;
single-threaded executors make lock-free parameter reads sound (RPP should state that reliance in
source, it does not).

### 2.3 Gateway ↔ backend ↔ tablet (`dyx3_system_gateway`, `backend`)

| # | Sev | Finding | Where | Status |
|---|---|---|---|---|
| G1 | MEDIUM | **Telemetry frames carry no sequence or stamp** at the top level (gateway `{"v":1,"type":"telemetry","snapshot":…}`, `gateway_node.cpp:830-834`; Socket.IO adds only `age_s`). `rover_event` has `seq`, `gateway_seq`, `t_mono_s`, `t_wall_ms`. The tablet cannot detect a reordered or stale telemetry frame except by `age_s` (backend receipt age, not source age). Add `seq` and `t_mono_s` to the telemetry frame. | `gateway_node.cpp:830`, `hub.py:99` | new |
| G2 | LOW | One `asyncio.ensure_future` per telemetry frame, unbounded (`client.py:182-186`). Benign because python-engineio queues per socket and the 170 s stress test showed flat memory; a 5 s event-loop stall would still fan out 50 queued frames at once. Coalesce to "newest only" before emit. | `client.py:185` | new |
| G3 | LOW | `rtk_serial_ports` does synchronous `Path.iterdir()/exists()` on the event loop (`routes.py:496-506`). Trivial today. | `routes.py:496` | new |
| G4 | LOW | RTK config routes are read-modify-write of the whole config with an *optional* client revision (`routes.py:430-448`); two tablets can clobber each other. Open item 19 rewrites this surface anyway. | `routes.py:430` | new |
| G5 | — | **Right:** newline-JSON IPC with 64 KiB line cap, non-blocking writes with `MSG_NOSIGNAL`, 1 MiB per-client buffer then disconnect (slow consumer), inbox 256 with E-stop exempt and sorted first, pending 64, all service calls async with per-command deadlines; backend: pure-ASGI auth + size cap before any body byte, one-process uvicorn, plan compile in a spawned process with a 60 s budget, artifact `mkstemp+fsync+replace`, 202 only after the mission node accepted, honest 503/504 (`delivered` false/null), bounded 256-event queue with drop counting, event replay on connect, Socket.IO ping 5/20 s pinned. No blocker found. | — | positive |

### 2.4 Supporting services (`gnss_rtk`, `spray`, `recorder`, bringup)

| # | Sev | Finding | Where | Status |
|---|---|---|---|---|
| S1 | HIGH | **An RTK profile change during a run can stop the rover.** `apply_config` runs on the control-socket thread holding `lifecycle_m_` and calls `NtripClient::stop()`, which **joins** the worker; the worker may be inside `getaddrinfo()` with no timeout (`ntrip_client.cpp:393`). The 200 ms status timer needs `lifecycle_m_` → `RtkStatus` stops → 0.5 s later the guard's `RtkGate` fails → STOP mid-line. The app exposes RTK profile management (open item 19) and LTE DNS stalls are ordinary. Registered as RTK-001 MEDIUM; escalated because the trigger is an operator action. **Fix:** refuse `SET_CONFIG` while the mission is active (the mission state is on the bus), and resolve DNS outside the lock with a deadline. | `rtk_node.cpp:232`, `ntrip_client.cpp:291,393` | RTK-001 ↑ |
| S2 | MEDIUM | NTRIP stream liveness is bytes, not valid RTCM (`ntrip_client.cpp:571-573`); a caster that sends keep-alives but no corrections never reconnects. `corrections_fresh` (10 s on CRC-valid frames) catches it at the guard, so the rover stops correctly but never self-heals. Reconnect on "no valid frame for N s" too. | `ntrip_client.cpp:559-573` | new |
| S3 | HIGH (spray milestone) | **The known boundary-continuity defect ships enabled.** `projection_direction_gate_deg` default 0.0 (disabled); `spray_defect_test.cpp:95` proves the station teleports > 0.5 m and a TRANSIT leg marks with the shipped default, and `:122` proves 30–120° gates fix it. Every valve/nozzle default is the prototype's, none marked measured (`spray_params.hpp:7`). Decide the gate value and measure the valve before paint trials (H12, SP-003). Not a driving blocker. | `spray_param_table.inc:37`, `spray_defect_test.cpp` | known-deferred; restated |
| S4 | MEDIUM | `health_check.sh` covers no RTK state, recorder status or watchdog status; the deep DDS and graph checks only WARN; `release.sh` gates on the non-deep run. A rover with a dead watchdog or an NTRIP worker in `Error` passes an upgrade health gate. | `health_check.sh:279-293,358-392,400` | new |
| S5 | LOW | Recorder `stop_run` blocks its mission callback up to ~14 s (bag stop) + 9 nodes × 3 RPC × 2 s parameter collection, under `run_mu_`. Own process, own executor; affects only how late the run closes. | `recorder_node.cpp:600,671` | new |
| S6 | LOW | `versions.json` `firmware_running` is hardcoded "unavailable"; `params_fcu.json` is "unavailable (OPEN)". Provenance of a run is the *expected* firmware (REC-006). | `recorder_node.cpp:512-517` | known |
| S7 | LOW | Three unbounded containers in long-running processes: `UlogCapture::gaps_`, `summary_.notes`, spray `ParamSet::journal_`. Growth is per event, not per tick; cap them anyway. | see trace | new |

**Right, keep:** RTK never writes anything but validated RTCM to the receiver (hard rule 2026-10-07
holds); USB sink is non-blocking with 0.2 s deadline and 2 s reopen, never exits; the spray
watchdog is a separate unit that fails closed from its constructor and bursts OFF at 20 Hz on every
cause change; the recorder is a sibling unit with 2 GiB floor, 20 GiB retention and ULog gap
accounting; all threads are joined; no wall-clock timeouts in these three packages.

---

## 3. Transport architecture as implemented (reference)

```
PX4 (uxrce_dds_client, best-effort writers, reliable readers, UXRCE_DDS_DOM_ID 42, PTCFG 1 localhost)
  │ Ethernet 10.41.10.2 → 10.41.10.1:8888 UDP
  ▼
MicroXRCEAgent udp4 -p 8888   (dyx3-platform, supervised, 2 s restart; no DDS env of its own)
  │ Fast DDS, domain 42, loopback (rover 01: ROS_LOCALHOST_ONLY=1 by hand — see P3)
  ▼
px4_link  SCHED_OTHER, 1 thread (SingleThreadedExecutor, spin_once 5 ms), mlockall
  in : /fmu/out/{timesync 1 Hz, local_position 50 Hz, attitude 100 Hz, status 2.5 Hz, est_flags 1 Hz, gps 5 Hz, battery, cmd_ack, msg_format, ulog}  SensorDataQoS (BE, KL5)
  out: /fmu/in/{offboard_control_mode, trajectory_setpoint(NaN), rover_speed, rover_attitude, rover_rate}  RELIABLE KL1, 5 msgs/cycle, 100 Hz timer + on every guard command
       /fmu/in/{vehicle_command KL10, gps_inject_data KL10, ulog_stream_ack KL16, message_format_request KL10}  RELIABLE
  gate order each cycle: handshake → session(timesync fresh 3 s) → topic stale mask → seq reset → have cmd → cmd age 0.2 s → cmd contract → mode map; failure = explicit STOP {0, NaN, 0}
  offboard session: Disabled → Prestream 0.5 s (STOP) → Requested (DO_SET_MODE 176/6) → Active (nav_state 14, ≤ 2 s) → Lost (never re-requested); disable = 0.3 s STOP then MANUAL 176/1 ×3
  timestamps: system_clock µs (firmware converts); freshness: steady_clock
  │ /dyx3/vehicle_state RELIABLE KL1, per PX4 sample (≈50 Hz), republished at 50 Hz with flags cleared when stale
  ▼
rpp   FIFO 80 CPU 4, 1 thread, mlockall; ticks per new px4_sample_stamp, watchdog 50 Hz; pose_max_age 0.5 s; STOP unless RUNNING
  │ /dyx3/rpp/motion_setpoint RELIABLE KL1
  ▼
motion_guard  FIFO 80 CPU 4, 1 thread, mlockall; decides per command, watchdog 50 Hz; inputs 0.5 s (steady), cmd 0.2 s; gates estop → px4_link → arming/nav 14 → RTK(6, 0.10 m) → heading → estimator → mission RUNNING; limits 1.0 / −0.10 m/s, 0.45 rad/s (no accel/jerk)
  │ /dyx3/motion_guard/command RELIABLE KL1, always valid, explicit STOP on any failure
  ▼
px4_link (above)
```

Side paths: `gnss_rtk` (own unit) → `/dyx3/rtcm` RELIABLE KL32 ≤ 300 B → `gps_inject_data`, or
USB_DIRECT to the UM982 (production); `spray` → `/dyx3/spray/actuator_command` → `vehicle_command`
187, 0.3 s transaction, OFF pre-empts ON; `spray_watchdog` (own unit) OFF at 20 Hz / 2 Hz;
`recorder` (own unit) bags `/dyx3/**` (no `/fmu/out`), `/dyx3/ulog_chunk` RELIABLE KL64.

Mission: `IDLE → LOADING (async load) → PLACING (async place) → ARMING (arm ≤ 4 s) → ENGAGING
(set_offboard ≤ 5 s) → READY (RPP ack ≤ 30 s) → RUNNING → COMPLETED | PAUSED (gate lost, RPP
stale 0.5 s, EKF reset, operator) | ABORTED (operator, E-stop → disarm) | ERROR`; terminal release
= `set_offboard(false)` then `disarm`, one at a time, no retry. Mission timeouts on the ROS (wall)
clock — see T1.

## 4. Backend ↔ tablet as implemented (reference)

```
gateway (1 thread ROS + 1 IPC thread, poll 200 ms)
  telemetry: every step (10 ms timer or wake) if ≥ 1/telemetry_hz (10 Hz) and a client exists → {"v":1,"type":"telemetry","snapshot":{14 sources: {age_s,fresh,data} | null, gateway:{…}}}
  events: fcu_link / estop / mission_state / operator_link / stale-transitions, coalesced 10 ms, seq + t_mono_s + t_wall_ms, replayed on connect
  commands: newline JSON {v,id,cmd,args} → inbox 256 (E-stop exempt, first) → async service, per-command deadline (start 2 s, estop 1 s, arm 4 s, offboard 5 s) → reply {id, ok, code, reason, data}
  │ /run/dyx3/gateway.sock, 64 KiB line cap, 1 MiB out buffer then drop, max_clients 4
  ▼
backend  uvicorn 0.54 one process, FastAPI 0.143 + python-socketio 5.17 (asgi), Python 3.10
  gateway client: asyncio streams, reconnect 0.2→3 s, request_timeout 6 s, events → Queue(256) drop-oldest, telemetry → ensure_future(emit) per frame
  hub: rover_event {kind, seq, gateway_seq, t_mono_s, t_wall_ms, coalesced, replay, data} broadcast; telemetry {snapshot, age_s} broadcast; replay of latest events to a new sid
  REST: pure-ASGI bearer + Content-Length/stream cap (20 MiB plan, 64 KiB else) before body read; plan compiled in a spawned process (60 s budget, one at a time); start = read+hash artifact off-loop → gateway → 202 {execution{mission_id, request_id, duplicate}}
  Socket.IO: ping 5 s / timeout 20 s; events in: heartbeat (operator, display only), estop (assert any role / clear operator) → gateway 6 s; CORS []; bind 10.42.0.1:8000 (env)
```

The operator heartbeat is relayed (0.5 s) and published as `/dyx3/operator_link` for display and
the recorder; nothing gates on it (owner decision 2026-10-10, verified in `mission_gate.cpp:10-12`).

---

## 5. Register cross-reference

**New in this review:** P2 (combined effect), P3 (localhost template, RMW/profile unpinned,
agent-readiness), P4 (blocking mechanism, test gap, ULog load), T1 (mission on wall clock), T4, T5,
C2 (retry loop), C3, C4, C6, G1–G4, S1 (escalation), S2, S4, S5, S7.

**Known and still open, confirmed in code:** PXL arm-ack gap (open item 7 → P1), MS-002, X-006,
PXL-003, PC-2 (now only the composition and QoS parts), RTK-001, XR-GPX-006, A1.4 timesync gate,
STEP1-R1, REC-006, H12/SP-003, PC-7a/7b, X-015, BE-001 (UX only now).

**Candidates to close or demote after one measurement each:**
- X-003 (four timer hops) → closed by M1 if `pose_to_write_age` p99 is within a few ms.
- GW-004 (operator-loss budget) → moot: the operator link is not a gate.
- BE-001 (shared heartbeat keeps the link alive) → UX/display only now; keep as LOW.
- PC-2 → re-scope to: px4_link RT priority (P4), QoS profile (P3/P4), `vehicle_odometry` 100 Hz
  (firmware, owner). The "one component container" part is no longer needed for latency if M1 passes.

**Register items this review confirms FIXED in the tree** (all `hardening/2026-10-10` SHAs are
ancestors of `750784c`): XR-GPX-001, PXL-002, X-010/PXL-001 (burst), GW-002, GW-001, GW-005,
XR-GPX-004, XR-GPX-007, RPP-009/X-007, XR-RPP-001/002/005, XR-BE-001/002/003, REC-001/002/004/005,
INS-001/002/004/006/007, X-013.

---

## 6. Measurements that settle the DOUBTs (bench, in order)

| # | Measurement | Settles | How |
|---|---|---|---|
| M1 | `Px4LinkStatus.pose_to_write_age_s` p50/p99/max over the three 2026-10-10 mission bags | X-003, X-006, PC-2 scope | `ros2 bag` export of `/dyx3/px4_link/status`; quote next to `loop_jitter_max_us` |
| M2 | **Agent loss with OFFBOARD Active**, wheels up: `kill -STOP` the agent (not `pkill`) for 3 s, then `-CONT`; then `pkill`. Record px4_link status rate, fault code timeline, time to PX4 disarm, recovery time | P4 blocking, STEP1-R1 | `bench_tools` link soak script + `journalctl -u dyx3-ros` |
| M3 | Companion-loss stop distance at 0.6 m/s (step 5 did 0.2 m/s: 0.69 s to disarm, 9 mm) | PXL-001 / A1.1 gate table | `offboard_sign_test.py --cut` with wheels down, `wheel_encoders` in the ulog |
| M4 | RPP `step()` duration max under full load (recorder + backend + ULog streaming on) after adding the field (C4) | open item 9 | `RppStatus` in the bag |
| M5 | Multi-hour soak with per-topic rates | F-tasks A1.3 (#27388) | existing `link_soak_*.csv` tooling, ≥ 4 h |
| M6 | Wall-clock step ±2 s while OFFBOARD Active, wheels up (`date -s`), with and without a mission RUNNING | T1, XR-GPX-006 | watch `nav_state`, `MissionState.reason_code`, px4_link `fault` |
| M7 | `SET_CONFIG` on the RTK socket with DNS black-holed (`iptables` drop UDP 53) while OFFBOARD Active | S1 | `RtkStatus` gap, guard reason 6, time to STOP |

---

## 7. Fix order (desk, smallest first; each with a test)

1. **P1** arm/mode ack matching + `REASON_REJECTED_BY_FCU`; pre-arm gate includes
   `pre_flight_checks_pass` (px4_link + mission; ~1 day; fake-FCU test sends DENIED).
2. **P3** template `DYX3_ROS_LOCALHOST_ONLY=1`; `RMW_IMPLEMENTATION` + profiles XML shipped;
   domain/PTCFG check reads the baseline and FAILs (deployment/installer; installer tests).
3. **P4** px4_link `chrt -f 70` + CPU in the launch prefix; `ulog_streaming_enabled=false` default;
   writer `max_blocking_time` in the profile (launch + px4_link param; M1/M2 after).
4. **T1** mission node → `steady_clock` for every age and deadline (keep ROS time for stamps only).
5. **S1** refuse RTK `SET_CONFIG` while the mission is active; DNS with a deadline outside the lock.
6. **G1** `seq` + `t_mono_s` on the telemetry frame (gateway + hub + one app change).
7. **P2** owner decision: persisted journal + `resume_from_point`, re-engage from PAUSED, control /
   services unit split (interfaces bump; the largest item here).
8. **S3/S4/S2** before the spray milestone: direction gate value, valve measurement, health coverage,
   NTRIP valid-frame liveness.

---

## 8. Is the architecture complete? Is this the most robust approach? What should be replaced?

Source: a deliverable-by-deliverable audit of architecture V1 §4–§14 against the tree at `2c09587`
and the gate records in HANDOFF (133 rows; evidence per row is in the audit, summarised here).

### 8.1 Completeness — the honest status

| DONE | PARTIAL | MISSING | SUPERSEDED (owner decision) | GATE-OPEN |
|---|---|---|---|---|
| 48 | 43 | 11 | 8 | 23 |

Non-negotiables (§14, 19 bullets): 12 present, **5 violated**, 2 superseded.

**Answer: no — the code is complete for a straight-line / square mission; the programme is not
accepted against any of its own numbers, and "only hardening is open" is not true.** Four
categories remain open, and only the first is hardening:

1. **Hardening** (§1–§2 of this review, and the register): P1–P5, T1, S1, S3, S4.
2. **Acceptance gates — all of the programme's numeric gates are open.** GATE 1 (OffboardControlMode
   flag combination: reverse, zero-speed pivot, CREEP and the no-republish ulog check never
   recorded), GATE 3 (geometry equivalence proven on the Git corpus only; `geometry_bag_replay_test`
   exits 77 = skipped), GATE 4 (120 parameters never re-validated in NED; no shape RMS on the new
   stack), GATE 7 (no shadow run; `dyx3_rpp_legacy` still in tree). The §10 gate table has **no
   entry measured**: full-mission RMS + p95 + max, arc/lshape/square/U-turn RMS, pivot wobble
   ≤ 0.50 cm, net walk ≤ 0.83 cm, coverage, spray loss. The only field number is a 3-side square at
   1.1 cm p50 / 3.9 cm p95, which is a different metric. Stage 0.1 (quantify the arc payoff from
   the old bags) and 0.3 (close the baseline questions) were never done. **R0 — one variable per
   stage — was violated outright:** firmware, 6X, Ethernet, DDS, C++ RPP and the hotspot first ran
   together on 2026-10-10 with no per-stage number.
3. **Unported behaviour** (the port is not finished): RPP point hold / point handshake / progress
   (tick publishes zero and reports `STATE_ERROR` if enabled), `run_sequencer` as a module, the
   `stop_pivot_fsm` transition ring (built, never drained or exported), RPP's own `xy_reset_counter`
   use (still the carried jump heuristic), 5 of the spray features (dash, point, heartbeat,
   fallback), the F5 battery (reverse, pivot, CREEP, stop distance at speed).
4. **Missing infrastructure the spec requires:** FCU parameter read path (`params_fcu.json` is
   always "unavailable" — a run cannot prove which PX4 parameters it ran with, the exact failure
   §7.9 was written to end); parameter profiles `production/precision/development.yaml` and
   `dyx3-param get|set|save` (a live tweak is never persisted); per-node `/etc/dyx3/*.yaml`
   shipped (today the C++ defaults *are* the production configuration, BR-001); parameter-class
   enforcement in px4_link, gnss_rtk, recorder and gateway; the RT executor split (§8: control on
   its own callback group, telemetry/params on a lower-priority executor — every node is one
   `SingleThreadedExecutor`); a pinned DDS profile; BLE and mDNS (replaced by the UDP beacon);
   `dyx3-version` without the firmware hash of the running FCU.

**The single most important open item is not in any of the four lists above.** The migration's
central claim is "publish the yaw rate directly and the arc floor (err ≈ ω/`RO_YAW_P`, 1.46 cm)
disappears". The code for it exists (`TRACK_RATE`, `rover_rate_setpoint`), but
`segment_command_mode` defaults to `"heading"` (`rpp_param_table.inc:119`; registry row 252,
human decision 2026-10-08, "explicit GATE 4 A/B selector; default stays behaviour-compatible until
rover measurements choose otherwise"). So today's rover drives arcs through PX4's attitude
controller — the same P-only heading loop as before, on a new transport. **The circle/arc step of
the field ladder must be run as that A/B (`heading` vs `rate`, same shape, same speed, same day)**
or it will measure the old floor and the programme's go/no-go stays unanswered. F-tasks and the
ladder entry do not say this; it should be written into the ladder now.

Two documents contradict the state of the rover and should be corrected: `README.md` "Nothing here
has run on a rover" and CLAUDE.md "174 = 120 + 54 parameters" (generated tables are 117 + 49,
proposal 2026-10-08).

### 8.2 Is this the most robust and proven way? — choice by choice

| Choice | Verdict | Reasoning |
|---|---|---|
| **uXRCE-DDS + direct rover setpoints** instead of MAVROS velocity offboard | **Keep — right, proven on rover 01** | The only v1.17 path that carries yaw rate to the rover controller; 1.1 cm p50 on the square. The upstream risks (#27388 silent stop, #27514 stale setpoint, #27860 no retry) are all mitigated Jetson-side today. The *proven* long-term fix for #27514/#27860 is in the autopilot (F-tasks C5/C6, candidate patches) — not carried. Carry them; they are small and remove two reasons the companion must be perfect. |
| **Explicit-zero chain + PX4 `COM_OF_LOSS_T` 0.5 s → disarm** as the safety model | **Keep — proven (0.69 s, 9 mm at 0.2 m/s)** | Standard offboard practice. Weakness: disarm is the *only* reaction to companion loss; at 0.85 m/s the roll-out after disarm is unmeasured (M3). A firmware setpoint-freshness timeout (C6) would stop in-mode instead of disarming. |
| **Event-driven callbacks across three processes** (px4_link → rpp → guard → px4_link) | **Keep; measure** | Simpler and more fault-isolated than the proposed single component container; latency should be four loopback hops. Keep the separate processes; give px4_link RT priority (P4). Decide after M1. |
| **Reliable KEEP_LAST(1) setpoints to the agent** | **Keep; bound it** | Forced by PX4's reliable readers; best-effort would not match. Pin `max_blocking_time` in a profile (P3/P4) so a stuck agent cannot stall the writer thread. |
| **Fast DDS as RMW (unpinned)** | **Keep Fast DDS; pin it** | The agent *is* Fast DDS; staying on one vendor avoids cross-vendor interop surprises. Switching to Cyclone now would be a new variable with no number behind it. Pin `rmw_fastrtps_cpp` + a profile. |
| **One `dyx3-ros` launch, any exit = whole graph down** | **Replace** | The least robust choice in the stack: non-safety code (gateway, spray, mission) can disarm the rover mid-line. Split into a control unit and a services unit; the guard already fails to zero when `mission/state` goes stale. |
| **Mission progress in memory, no re-engage** | **Replace** | A transient becomes a restart from point 0 with manual repositioning. Persist the journal; add `resume_from_point` and a PAUSED → ARMING → ENGAGING path. |
| **Arm/mode confirmation by `vehicle_status` timeout only** | **Replace** | PX4 says *why* it refuses, in `vehicle_command_ack`; the spray path already matches acks. Every refusal today is an opaque timeout. |
| **RTK over USB to the UM982 (option C)** | **Keep — the most robust of the three** | Removes PX4 and the firmware from the correction path; proven outdoors (fix 6). Gap: no rollback transport (option B not built) and the NTRIP liveness is bytes-based (S2). Accept the no-rollback risk; fix S1/S2. |
| **RTK worker in-process with the status publisher** | **Fix, not replace** | One lock across a joinable DNS-blocked thread (S1). Small change. |
| **Backend: FastAPI + Socket.IO, no `rclpy`, Unix-socket gateway, E-stop as a prioritised request** | **Keep — well built** | Bounded and asynchronous end to end, stress-tested. The weak link is the tablet's JS thread and plaintext HTTP on the LAN (X-015). Nothing in the backend needs replacing for robustness. |
| **Tablet as the single trajectory author, backend admits only** | **Keep (owner decision)** | Removes the second geometry builder. Consequence to accept: GATE 3's bag corpus and the vector generators died with the Python path engine — freeze the vectors under `tools/` (open item 5). |
| **Spray through `VehicleCommand` 187 over DDS with an fsync'd identity ledger** | **Keep for now; measure before trusting** | Unusually heavy machinery for a valve, but it buys the one property that matters: PX4 disarm closes the valve (X-012). Never run with paint; valve/nozzle numbers are prototype values; the boundary defect ships enabled (S3). Measure (SP-003) before deciding whether a Jetson-side GPIO path with a hardware fail-closed is simpler. |
| **Legacy Python oracle + equivalence vectors as the port method** | **Keep — but finish it** | Proven method (13 905 ticks, 0 mismatches on synthetic scenarios). It is unfinished until it runs on recorded bags (PC-1c) and on the rover (I2), and the oracle is deleted (GATE 7). |
| **Forked EKF2 wheel-encoder fusion on v1.17** | **Keep; verify in the field** | Necessary (not upstream), GATE 2 replay passed. `EKF2_WENC_CTRL 1` is live before `EKF2_IMU_POS_*` was re-measured on the 6X mount and before any field pivot-wobble number — R1 is open. |
| **Guard without acceleration/jerk limits (PX4 slews)** | **Keep; document** | Reasonable: one owner of the profile (RPP) and one slew (PX4). Record it in the guard contract (C3). |
| **No physical E-stop; RC kill + tablet E-stop** | **Decide (owner)** | The tablet is not a deterministic path; RC loss is invisible in OFFBOARD by configuration. A hard-wired E-stop is the proven answer on marking machines. |

### 8.3 What to replace — consolidated and ranked

1. Whole-graph restart policy → control / services unit split (P2).
2. In-memory progress + no re-engage → persisted journal, `resume_from_point`, re-engage (P2).
3. Timeout-only arm/mode confirmation → ack-matched with PX4's reason (P1).
4. Hand-edited, unpinned DDS environment → installer-shipped env + Fast DDS profile (P3, P4).
5. Mission node wall clock → steady clock (T1).
6. px4_link `SCHED_OTHER` → FIFO below the guard (P4).
7. `segment_command_mode` default → decided by the arc A/B, not left at `heading` by inertia (8.1).
8. Companion-only mitigation of #27514/#27860 → carry firmware C6/C5 (8.2).
9. Spray valve numbers and boundary gate → measured values before paint (S3).

Everything else in the stack is the right approach and should not be re-litigated; it should be
**finished** (gates, unported features, missing infrastructure) and **measured** (§6).

Agent: claude
