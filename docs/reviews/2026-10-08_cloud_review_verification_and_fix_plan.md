# Verification of the "Final Cloud Code Review" and fix plan

**Reviewed report:** `DYX_3WD_Final_Cloud_Code_Review.md` (HEAD `dcb5812`). **Verified against:** the same HEAD, by reading the code named in each finding on 2026-10-08.
Nothing here is a code change yet. Items that need a human decision, a bench or the Jetson are marked **[HUMAN]**, **[BENCH]**, **[JETSON]**.

## 1. Verdict on the verdict

The report's overall conclusion stands: the RPP port is not the weak point, and the branch is **not field-ready** until the spray authority, command-ACK and valve fail-safe items are closed.
Of the 32 findings: **22 are real and confirmed in the code** (C1, C2, C3, H1, H2, H3, H4, H5, H6, H8, H9, H10, H11, H14, M1, M2, M3, M5, M6, L1, L3, L4); of these, **5 need a correction** to severity, probability or the proposed fix (C2, H1, H3, H10, L1). **8 are known open decisions or planned work, not new defects** (H7, H12, H13, H15, M7, M8, M9, M10). **M4** is a schema note for the production RTK redesign. **L2** I did not verify.

| ID | Verdict | Note |
|---|---|---|
| C1 | **Confirmed** | `mission_running_` is stored (`spray_controller.hpp:82,140`) and never read; `tracking_seen_` is set on the first TRACKING and only cleared on path load (`spray_controller.cpp:24,44`); `auto_safety_status` has no mission or RPP-freshness term (`spray_gates.cpp:127-138`). Pause or abort mid-mark leaves the valve ON. |
| C2 | **Confirmed, severity High** | ACKs match on `command` id only, FIFO (`px4_link_node.cpp:497-512`); the ACK has no source or seq. Overlapping requests with the same MAV_CMD can be mis-attributed after a timeout/late ACK. PX4 normally acks in order, so a swap needs a drop or a late ACK, which is why I rate it High rather than Critical. Serialising is still the right fix because the worst outcome is a false "OFF proven". |
| C3 | **Confirmed, not a code defect** | Already recorded as OPEN item 1 in HANDOFF. Needs a firmware parameter or a hardware path and a bench. **[HUMAN][BENCH]** |
| H1 | **Confirmed, probability low** | `on_ack` sets `off_confirmed_` and nothing clears it on a mapping change (`watchdog_core.cpp:20-27`). Needs a runtime mapping change; treat as Medium. |
| H2 | **Confirmed** | Spray never reads `corrections_fresh`; only the guard does (`motion_guard_node.cpp:107`). |
| H3 | **Confirmed, the proposed fix is wrong as written** | The cancel callback does not finalise the goal (`mission_node.cpp:127-132`). But inside that callback the goal is not yet "canceling", so `finish_goal_if_terminal()` there would call `abort()`, not `canceled()`. Finalise from the timer after the cancel is accepted. |
| H4 | **Confirmed, and my own RPP triggers it** | A bad artifact makes RPP publish `STATE_ERROR` with the mission id; the mission treats `!= IDLE` as the acknowledgement (`mission_node.cpp:231`). |
| H5 | **Confirmed** | The guard applies accel, decel, jerk and yaw-accel limits (`limits.cpp`, `motion_guard_node.cpp:182-185`). Defaults equal RPP's, so the practical reshaping is small today; the authority overlap is the problem. |
| H6 | **Confirmed, behaviour-affecting** | `max_reverse_speed_mps = 0.0` (`motion_guard_node.cpp:180`) turns every RPP brake and overshoot-correction (negative `speed_body_x`) into zero. The I1 active brake therefore does nothing on the real stack. This is the most important line in the report. |
| H7 | **Design decision, not a defect** | I flagged it as DERIVED/open. Settle at GATE 4 as the report says. |
| H8 | **Confirmed** | Already flagged open in `rpp_node.md`. |
| H9 | **Confirmed** | Spray builds its model from the raw artifact; RPP conditions it (the conditioner fuses a short unpainted lead into the mark and rounds corners). Flagged DERIVED/open in the spray contract. |
| H10 | **Confirmed, severity Medium, fix needs care** | No TLS (`ntrip_client.cpp`). "TLS required by default" would break most plain-NTRIP casters (port 2101). Make TLS supported and selectable per profile, warn on plaintext with credentials, never silently downgrade a profile that asked for TLS. **[HUMAN]** for the default. |
| H11 | **Confirmed** | `chunks_forwarded_` increments before publish and before px4_link accepts (`rtk_node.cpp:137`). |
| H12, H13 | **Known open, not new** | Both are recorded OPEN decisions with a replay tool (H12) and a quantified delay (H13). They need field data. |
| H14 | **Confirmed** | `release.sh:191` dies with the failed release still `current`. |
| H15 | **Accurate, planned work** | Not a defect in what was built. |
| M1 | **Confirmed** | `on_parameters` validates and returns success without updating the cached members or the timer (`mission_node.cpp:152-186`). |
| M2 | **Confirmed** | The guard's contract states "read once at start (target LIVE)"; the registry says LIVE. Reject runtime change until implemented. |
| M3 | **Confirmed** | `last_step_s_` is set at the top of `step()` (`px4_link_node.cpp:528`) so `(now - last_step_s_) < 1.0` at line 698 is always true after the first overrun. |
| M4 | **Design note** | Fold into the production RTK schema (H15). |
| M5 | **Confirmed** | One `send()` for the GGA on a nonblocking socket (`ntrip_client.cpp:371`); a partial write is counted as a failure after the bytes are on the wire. |
| M6 | **Confirmed** | `.complete` is touched in `build_release` (`release.sh:96`), before static verification (`:169`); its own comment at `:64` claims otherwise. |
| M7, M8 | **Known open** | M8 (pivot gain 1.5) is bench-only. M7 is a decision. |
| M9 | **Known open** | Listed as mutation survivors in `rpp_orchestrator.md`. |
| M10 | **Intentional** | Recorded in the manifest and HANDOFF. |
| L1 | **Likely confirmed** | `stop()` exchanges the fd out of `active_fd_` and only shuts it down (`ntrip_client.cpp:202`); the worker's later `close_fd()` then finds -1. Prove with an fd-count test before fixing. |
| L3, L4 | **Confirmed** | `start-ros.sh:2` omits rpp; HANDOFF and CLAUDE.md still say 174 = 120 + 54 (HANDOFF line 290 already records 173). CLAUDE.md is human-owned: propose, do not edit. |

### Two things the review did not say
* **H6 and H5 interact with my brake design:** even with a bounded reverse limit, the guard's decel limit (0.5 m/s²) and the `RO_*` firmware reverse behaviour decide whether a 5–8 cm/s brake vector actually stops the rover. Bench it (GATE 1), do not assume.
* **`spray_request` (H8) is true on the fused unpainted lead** (the node test shows it from the first metre). Wiring it as an AND-gate before fixing H9 would still paint wrongly. Do H9 first, then H8.

## 2. Fix plan

Phases are ordered by risk to paint and to the rover. Each item lists the change, the test that must exist, and the done-criterion. Items marked CODE can be done in the cloud and proven off-target; the rest cannot.

### Phase A — stop paint when the mission stops (CODE, do first)
**A1 · C1 spray mission and RPP ownership** — `dyx3_spray`
* Add to `GateInputs`: `mission_running` and `rpp_fresh` (new `rpp_timeout_s`, DERIVED; no source, default 0.5 s, IDLE_ONLY) and a current `rpp_marking_ok` (latest `RppStatus.state` is TRACKING, or APPROACH-equivalent; not ERROR, COMPLETE, LOADED, STOPPING).
* Replace the permanent `tracking_seen_` latch with: latched only while the same mission id is RUNNING; cleared on any non-RUNNING mission state and on path load.
* `auto_safety_status` fails with reasons `"mission not running"`, `"rpp stale"`, `"rpp not marking"`; the existing OFF path then closes the valve and revokes the lease.
* Decision to take: STOPPING during a corner stop must still allow the last 2 cm of the leg (the existing comment says so); keep TRACKING/STOPPING-with-spray-on allowed, but not LOADED/ERROR/COMPLETE/IDLE. **[HUMAN]** confirm.
* Tests (node test, fake FCU): valve ON then PauseMission, AbortMission, mission COMPLETED/ERROR, RppStatus ERROR, RppStatus silent for `rpp_timeout_s`, RPP process kill; each asserts an OFF command and an OFF ack. Also a regression test that a normal mark is not interrupted.
* Done: all six cases green, and the existing 11 spray node tests unchanged.

**A2 · H4 READY acknowledgement** — `dyx3_mission`
* Acknowledge only on `LOADED`, `TRACKING`, `STOPPING`, `PIVOTING`, `CREEPING`; evaluate `ERROR` and `COMPLETE` first.
* Tests: ERROR in READY -> mission ERROR (not RUNNING); COMPLETE in READY -> not an ack.

**A3 · H3 action cancel** — `dyx3_mission`
* Cancel callback sets `cancel_pending_`; `on_timer` calls `finish_goal_if_terminal()` once the goal reports canceling. Test: cancel -> goal state CANCELED, `RESULT_ABORTED`.

### Phase B — motion authority and the brake (CODE + BENCH)
**B1 · H6 bounded reverse** — set `max_reverse_speed_mps` to a small bounded value for brake and overshoot correction (candidate 0.10 m/s, no source: **[HUMAN]**), covered by a guard test that a −0.08 brake passes unchanged and −0.5 is clamped. **[BENCH]** reverse behaviour (GATE 1).
**B2 · H5 guard authority** — decide, then change: either remove accel/decel/jerk/yaw-accel from the normal path and keep hard envelopes only (the report's recommendation, and the architecture's "RPP owns the profile"), or keep them as a documented second-line limit set well above RPP's. **[HUMAN]**. If removed: keep the fields for a safety-test mode, update `dyx3_motion_guard.md` section 5, and keep the property test "no non-STOP command leaves while a gate fails".
**B3 · H7 steering authority** — add a parameter `segment_command_mode` (`heading` | `rate`, IDLE_ONLY, default `heading` until measured); implement `rate` as `yaw_rate = segment_yaw_rate_gain * theta_e` clamped, so GATE 4 can A/B on the rover. No default change without the bench. **[BENCH]**
**B4 · M8 pivot gain** — bench at 30/60/90/135/180 degrees; record requested vs measured rate, overshoot and settling; then freeze the gain.

### Phase C — one geometry, one spray verdict (CODE, after A)
**C1 · H9 shared execution geometry** — publish the conditioned run(s) as the single source: RPP writes an immutable conditioned-path artifact (content-addressed like the DYX3PATH, with a `source_sha256` and the conditioner parameters in its meta), announced on a latched topic or in `RppStatus`; `dyx3_spray` builds its `PathModel` from it. Alternative (cheaper, weaker): have spray call `condition_path` itself with the same parameters; rejected because it duplicates the conditioner and its parameter set. Needs a contract note and an interface bump. Test: for every archived mission, spray boundaries computed on the conditioned geometry vs the raw artifact (report the largest station difference).
**C2 · H8 `spray_request`** — after C1, decide **[HUMAN]**: AND it into the spray decision (planner MARK AND RPP heading verdict AND gates), or move the heading gate into spray and delete it from RPP. Either way remove the unused output. Do not ship both.
**C3 · H12 gate default** — run `spray_projection_replay --gates 0,45,60,90` on 5–10 field artifacts plus the bag traces **[HUMAN][BENCH]**, then set the default and add a test that the shipped default is not 0 if the data says so.
**C4 · H13 debounce** — add the measured debounce latency (`debounce_ticks / tick_hz * speed`) to the lead calculation, or debounce an earlier signal; test that ON/OFF edge error stays within the 1 cm budget at 0.35 m/s on the synthetic out-and-back. Decision on method **[HUMAN]**.

### Phase D — FCU command/ACK integrity (CODE)
**D1 · C2 serialised spray transactions** — `dyx3_px4_link`: one spray `VehicleCommand` in flight; later requests queue (bounded) with newest-wins per source; watchdog OFF preempts: it purges pending ON and goes next. ACK is attributed to the single in-flight request; a late ACK after timeout is discarded and counted. Tests per the review (controller ON + watchdog OFF, reversed and delayed ACK, reassert flood, timeout then late ACK, OFF/OFF overlap) plus: a stale ACK never proves watchdog OFF.
**D2 · H1** — `WatchdogCore::on_lease`: detect a mapping change (backend, index, value, servo instance, PWM); clear `off_confirmed_`, drop the in-flight proof, send OFF on the new mapping. Test: prove OFF on A, change to B, `off_authority_ready` false until B's ACK.
**D3 · H2** — spray consumes `corrections_fresh` in its RTK snapshot and gate (same predicate as the guard, shared helper so the two cannot drift). Test: fix RTK_FIXED + corrections stale -> spray refuses.

### Phase E — RTK hardening (CODE) 
**E1 · H11 + M4** — separate counters (source bytes, valid frames, frames handed to px4_link, frames px4_link accepted) and report them in `NtripStatus`; px4_link reports accepted/dropped back (small status field); `chunks_forwarded` renamed or removed. Interface bump.
**E2 · M5 + L1** — send-all with `poll` and a deadline for the GGA; fix the fd ownership in `stop()`. Tests: loopback with a tiny send buffer (partial write); fd-count before/after 50 start/stop cycles.
**E3 · H10 TLS** — optional TLS (OpenSSL) selected per NTRIP profile, with certificate verification on by default when TLS is on, an explicit `allow_plaintext` setting, and a warning when credentials go over plaintext. **[HUMAN]** default policy; **[JETSON]** casters.
**E4 · H15 production RTK architecture** — separate plan (source NTRIP|LoRa, transport USB_DIRECT|PX4_DDS, control socket, persistent config, single injection authority). It needs the `plan.md` the review cites; I do not have it in this repository. Do not start before it is in the repo.

### Phase F — small correctness and hygiene (CODE)
* **M1** mission parameters: apply atomically (timer re-created, cached values updated) or reject at runtime; test that a new `state_publish_hz` changes the publish period.
* **M2** guard: reject runtime change of LIVE-registered limits until implemented; registry note.
* **M3** px4_link: separate `last_overrun_s_`; test that the fault clears one second after the last overrun.
* **H14** installer: on first-install health failure stop the new services, clear `current`, mark the release failed; staged-root test. **M6**: write `.complete` after verification; test that a release failing static verification has no `.complete`.
* **L3** fix the `start-ros.sh` comment. **L4** propose to the human that CLAUDE.md §3b and HANDOFF line 60 use 173 = 119 + 54 (CLAUDE.md is human-owned: a proposal in `docs/architecture/proposals/`, not an edit).
* **M9** add orchestrator vectors for the four surviving mutants (turn 50–60 degrees at a run boundary with real runs; start beyond the end plane; a corner released straight into tracking with `align_settle_s = 0` and loose gates; a 0.32 s velocity blackout with a jump). Done when each mutation is caught.

### Phase G — not code: hardware, firmware, field
* **C3** valve physical closure: with the valve ON, kill `dyx3_px4_link`, kill MicroXRCEAgent, unplug Ethernet, remove Jetson power; measure the output. Options on the table: PX4 companion-loss failsafe param, the secondary UART, a hardware relay. **[HUMAN][BENCH]**. Until proven, document that the valve has no independent close path and do not field-run with paint.
* GATE 1 PX4 bench (pivot, brake, reverse, rate tracking), Jetson timing and jitter, real-bag replay of RPP and spray, field validation. Promote services in the installer manifest only after these (M10).
* M7 point hold / handshake / progress: decide port, drop or move to mission ownership. **[HUMAN]**

## 3. Order of work and size

| Order | Items | Rough size | Blocks |
|---|---|---|---|
| 1 | A1, A2, A3 | 1 session | any paint trial |
| 2 | D1, D2, D3 | 1 session | any valve proof |
| 3 | B1, B2, B3 (code part) | 1 session + decisions | GATE 1 bench |
| 4 | C1, then C2 | 1–2 sessions + decision | spray accuracy claim |
| 5 | F (all) and E1, E2 | 1 session | none |
| 6 | E3 | 1 session after policy | secure NTRIP |
| 7 | C3, C4, B4, G | field and bench | production |
| 8 | E4 | separate plan | production RTK |

## 4. Decisions I need from you before coding
1. A1: confirm spray may stay ON through STOPPING (corner stop) but never through LOADED / ERROR / COMPLETE / IDLE or a non-RUNNING mission, and `rpp_timeout_s` 0.5 s.
2. B1: the bounded reverse speed for brake (candidate 0.10 m/s).
3. B2: remove the guard's accel/decel/jerk from the normal path, or keep them as a second line.
4. C2: AND-gate `spray_request` into spray, or move the heading gate into spray.
5. E3: plaintext allowed by default per profile, or TLS required.
6. Where `plan.md` (production RTK) lives, so E4 can start.
