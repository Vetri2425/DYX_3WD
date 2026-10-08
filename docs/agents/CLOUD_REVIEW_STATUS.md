# Cloud Review — final status record

**Read this before implementing any fix.** It is the permanent answer to "was this already fixed?".
Source of the findings: `docs/reviews/2026-10-08_cloud_review_verification_and_fix_plan.md` (32 findings,
frozen decisions in its section 5). Hardware and field items are tracked here but are *not* code defects.

## 1. Integration state

| | |
|---|---|
| Branch / tested commit | `claude/cloud-phases` @ `9549679` (documentation commit on top of this one follows) |
| GitHub Actions run | `37766642047` — **success, all 9 jobs** (installer, repo hygiene, legacy quarantine, clang-format, tools, interface freeze, geometry native, **colcon build + test (ROS 2 Humble)**, backend + path engine) |
| ROS result in that run | 12 packages built; **440 tests, 0 errors, 0 failures, 2 skipped** (previous green baseline, run `37755170359`: 432 tests, 2 skipped) |
| Same result locally | `./tools/dev/ros2_humble.sh build-test` — see `docs/agents/LOCAL_ROS2_BUILD_ENV.md` |
| Not on master | Nothing here is merged to `master`. |

**What green CI does and does not prove.** It proves the software builds on arm64 and the off-target tests
pass. It does **not** prove: valve closure, PX4/DDS behaviour, timing and jitter, RTK, paint quality, or
accuracy. Those are sections 4–5 below.

## 2. How to read the status column

| Status | Meaning for a future agent |
|---|---|
| **FIXED** | Implemented, tested off-target, in CI. Do **not** re-fix; if you think it regressed, that is a *new regression* — reproduce it first. |
| **FIXED – BENCH PENDING** | Software done; a number or behaviour still needs the rover. Do not change the value without bench data. |
| **DEFERRED DESIGN** | Known, deliberately not done. Needs a human decision. Do not resolve silently. |
| **OPEN – HARDWARE/FIELD** | Cannot be closed in software. |
| **NOT VERIFIED** | Never checked. |

Blocks columns: **DRY** = basic dry-run (no paint) testing, **PAINT** = paint-enabled testing,
**PROD** = production acceptance. "—" = does not block.

## 3. Completed software fixes

| ID | Sev. | Status | Evidence (commit) | Verification | Remaining limitation | Contract | Next action | DRY | PAINT | PROD |
|---|---|---|---|---|---|---|---|---|---|---|
| C1 | Critical | **FIXED** | `780bb29` (A1) | spray core/controller tests: pause, abort, complete, error, RPP error/stale → OFF with ack in one tick; normal mark not interrupted; CI | `rpp_timeout_s` 0.5 s is DERIVED — re-validate from Jetson jitter | `dyx3_spray.md` | Jetson jitter check | — | cleared in software (see C3) | open until C3 |
| C2 | High | **FIXED** | `4847d53`, `ffe623e`, `aa68194`, `463ff77`, `6b8b4b1` (D1) | px4_link node + ACK-token tests: serialized in-flight, newest-wins, nonreused ACK identities, late ACK discarded, watchdog OFF supersedes every queued non-watchdog command, stale ACK cannot prove OFF | Real FCU ACK ordering/latency unmeasured | `dyx3_px4_link.md` §14 | Bench: ACK behaviour on the real FCU | — | software cleared | needs bench |
| H1 | Medium | **FIXED** | `3ca04dc` (D2) | watchdog tests: OFF proof bound to actuator mapping; mapping change clears proof | Runtime mapping changes not exercised on hardware | `dyx3_spray.md`, `dyx3_px4_link.md` | — | — | — | — |
| H2 | High | **FIXED** | `63ec3d3` (D3). *HANDOFF cites `03763c6`: that is the pre-rebase SHA, not in branch history.* | spray gate test: RTK fixed + corrections stale → refuse | Correction-staleness threshold is shared with the guard; not field-tuned | `dyx3_spray.md` | — | — | — | — |
| H3 | High | **FIXED** | `780bb29` (A3) | mission test: cancel → CANCELED, RESULT_ABORTED | — | `dyx3_mission.md` | — | — | — | — |
| H4 | High | **FIXED** | `780bb29` (A2) | mission test: RPP ERROR in READY → mission ERROR, not RUNNING | — | `dyx3_mission.md` | — | — | — | — |
| H5 | High | **FIXED** | `a3d54c9` (B2) | guard tests: RPP is sole speed-profile owner; guard applies hard envelopes only; "no motion leaves while a gate fails" property test kept | Guard shaping retained only as test-only legacy profile | `dyx3_motion_guard.md` §5 | — | — | — | — |
| H6 | High | **FIXED – BENCH PENDING** | `94d5548` (B1) | guard test: −0.08 passes, −0.5 clamped to −0.10 | 0.10 m/s is an *initial* bound (not field-tuned); whether a 5–8 cm/s brake vector stops the rover depends on firmware `RO_*` and the guard decel — GATE 1 | `dyx3_motion_guard.md` | GATE 1 bench: reverse/brake | — | bench | bench |
| H8 | Medium | **FIXED** (ownership moved) | `f59031e` (C2) | spray owns heading cut / entry hold / valve gating; `spray_request` is deprecated diagnostic (still present in `RppStatus`, `rpp_node.cpp`) | Removal of the deprecated output not done; `projection_direction_gate_deg` default still 0.0 (see H12) | `dyx3_spray.md`, `rpp_node.md` | Remove `spray_request` in a later interface bump | — | — | — |
| H9 | High | **FIXED** | `bd26b23` (C1), test fixture fixes `aefe7cf`, `7bef9de` | spray builds boundaries from the shared conditioned geometry; archived-fixture comparison recorded in HANDOFF (square_2x2 Δ 2e-14 m; mission_straight_5m Δ 0.10 m) | Real-bag spray replay not run | `dyx3_spray.md`, `rpp_path_conditioner.md` | Real-bag replay (LOCAL) | — | — | needs replay |
| H10 | Medium | **FIXED** | `caa8ceb` (E3) | per-profile `security = PLAINTEXT\|TLS`, verified TLS, no silent downgrade, plaintext+credentials warning; unit/loopback tests | Real casters not tested; no Jetson TLS test | `dyx3_gnss_rtk.md` | Test against real caster | — | — | needs real caster |
| H11 | High | **FIXED** | `7d9cd59` (E1) | separate RTCM delivery-stage counters; px4_link accepted/dropped reported back | — | `dyx3_gnss_rtk.md`, `dyx3_px4_link.md` | — | — | — | — |
| H14 | High | **FIXED** | `f6834a4` (F1) | installer staged-root: failed first install stops services and clears `current`; failed upgrade restores previous release; 80/80 | Not run on the Jetson (apt/systemd) | `installer/README.md` | Jetson install rehearsal | — | — | needs Jetson |
| M1 | Medium | **FIXED** | `e4b9712` (F2); terminal-state tests `c3dc01b` | mission node tests: atomic batch, timer period changes, invalid batch changes nothing, active/paused/terminal rejected | IDLE_ONLY applies only in IDLE; COMPLETED/ABORTED/ERROR have no route back to IDLE, so reconfiguring needs a node restart (documented, not a new transition) | `dyx3_mission.md` | Human: decide whether a terminal→IDLE reset is wanted | — | — | — |
| M2 | Medium | **FIXED** | `c7f3c6f` (F3) | guard node test: all 15 startup-read parameters reject runtime change; hard limits unchanged | Also rejects `use_sim_time` at runtime (harmless) | `dyx3_motion_guard.md`, `parameter_registry.md` | — | — | — | — |
| M3 | Low | **FIXED** | `c09b073` (F4), `4b0e669`, `7795e83` | node test: warning clears 1 s after last *actual* overrun, lifetime counter kept; invalid/backward clock publishes explicit STOP (full control set), baseline untouched | Timing itself unmeasured on target | `dyx3_px4_link.md` §11 | — | — | — | — |
| M4 | Design note | Folded into M5/E1 counters; schema goes with E4 | `7d9cd59` | — | Production RTK schema not built | `dyx3_gnss_rtk.md` | See E4 below | — | — | — |
| M5 | Medium | **FIXED** | `bdf7598` (E2) | loopback test with tiny send buffer (partial write) | — | `dyx3_gnss_rtk.md` | — | — | — | — |
| M6 | Medium | **FIXED** | `f6834a4` (F1) | staged-root: release failing static verification has no `.complete` | as H14 | `installer/README.md` | — | — | — | — |
| L1 | Low | **FIXED** | `bdf7598` (E2) | fd-count test over start/stop cycles | — | `dyx3_gnss_rtk.md` | — | — | — | — |
| L3 | Low | **FIXED** | `adfd97d` (F5) | launcher comment matches the 6-node launch graph | — | — | — | — | — | — |
| L4 | Low | **PROPOSED** (human-owned file) | `adfd97d` proposal `docs/architecture/proposals/2026-10-08_claude-parameter-count-clarification.md` | counts verified by `tools/gen_param_tables.py --check` (117 RPP / 49 spray) | `CLAUDE.md` still says 174 = 120 + 54 | — | Human applies the proposal in `docs/architecture/proposals/2026-10-08_claude-status-correction.md` | — | — | — |
| M9 | Medium | **PARTIALLY CLOSED** | `47796ad`, `9549679` (F6) | independent replay: 3 of 4 new scenarios kill their mutation; `fault_vel_blackout_jump` does **not** (baseline coverage only) | The 0.3 s velocity-freshness constant is a survivor again; other listed survivors unchanged | `rpp_orchestrator.md` | Find the exact mutation or a scenario that reaches `vel_is_fresh` | — | — | before Gate 7 |

## 4. Deferred design decisions

| ID | Sev. | Status | Evidence | Next action | DRY | PAINT | PROD |
|---|---|---|---|---|---|---|---|
| H7 | Medium | **DEFERRED – selector built** (`24b74dd`, B3) | `segment_command_mode` = `heading` (default) \| `rate`, IDLE_ONLY | A/B on the rover at GATE 4; no default change without bench data | — | — | decide |
| H12 | High | **DEFERRED – field data** | `projection_direction_gate_deg` default 0.0 (disabled). Tool exists: `spray_projection_replay --gates 0,45,60,90` (`dcb5812`) | Run on 5–10 field artifacts, set default, add test | — | **decide before paint trials** | decide |
| H13 | High | **DEFERRED – field data** | Debounce latency (`debounce_samples` × tick) is carried in the boundary budget; not yet added to the lead calculation (plan item C4) | Human picks method; test edge error ≤ 1 cm at 0.35 m/s | — | boundary accuracy | decide |
| H15 | — | **PLANNED** | Plan committed: `docs/plans/2026-10-08_production_rtk_plan.md` (`2df62b8`). **E4 is its own project, not a Cloud Review code defect.** | Start E4 separately | — | — | needed |
| M7 | — | **DEFERRED** | point hold / handshake / progress ownership undecided | Human: port, drop or move | — | — | decide |
| M10 | — | **INTENTIONAL** | Services not promoted in the manifest until bench/Jetson gates pass | Promote after bench | — | — | — |

## 5. Hardware / firmware validation (cannot be closed in software)

| ID | Sev. | Status | What is missing | DRY | PAINT | PROD |
|---|---|---|---|---|---|---|
| **C3** | Critical | **OPEN – HARDWARE** | Valve has **no independent close path**: every OFF rides the DDS link. Test: valve ON, then kill `dyx3_px4_link`, kill MicroXRCEAgent, unplug Ethernet, remove Jetson power; measure the output. Options undecided: PX4 companion-loss failsafe parameter, secondary UART, hardware relay (HANDOFF "OPEN ITEMS" 1). | — | **BLOCKS — do not field-run with paint** | blocks |
| — | | **OPEN – BENCH** | GATE 1: pivot, brake, reverse, rate tracking against real PX4; M8 pivot-gain sweep (30/60/90/135/180°); upstream #27514 stale-setpoint ~900 ms, #27497 differential in Mission Mode, #27388 `uxrce_dds_client` silent stop | before field | before paint | blocks |
| M8 | Medium | **OPEN – BENCH** | pivot gain 1.5 (bench only) | — | — | blocks |
| — | | **OPEN – JETSON** | timing and jitter, systemd, installer rehearsal, udev/network | — | — | blocks |

## 6. Field calibration and accuracy validation

All **OPEN**, none are software defects: real-bag replay of RPP and spray; valve timing (`solenoid_*_delay_s`),
nozzle offset, paint quality; RTK fix quality and correction latency; cross-track numbers vs the baseline in
`CLAUDE.md` §2 (arc 1.46 / lshape 0.90 / square 0.87 / U-turn 1.06 cm @ 0.35 m/s). Block PAINT and PROD.
**L2** was never verified by anyone (**NOT VERIFIED**, Low).

## 7. Remaining regression coverage

- `fault_vel_blackout_jump` mutation (M9 above) and the other documented mutation survivors in `rpp_orchestrator.md`.
- Node tests for F2–F4 now run in CI (they could not run on the Mac before `LOCAL_ROS2_BUILD_ENV.md`).
- Skipped in CI (2): the bag cross-track fixture test (no bag fixture) and the pinned message-hash set test.
- No test exercises real DDS timing, FCU ACKs, or valve hardware.

## 8. Rules for the next agent

1. Check this table and the named contract **before** writing a fix. FIXED = reproduce a failing test first.
2. An item in sections 4–6 is not yours to resolve silently — it needs a human decision or hardware.
3. A new defect that is not in this table is a *new finding*: add it here with its evidence.
4. Never report a hardware behaviour as verified from CI.
