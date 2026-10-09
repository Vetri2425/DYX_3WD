# HANDOFF

Append a section per work session. Newest last.

---

## 2026-09-05 — Claude — repository skeleton (Milestone 1)

**Changed**
- Created `DYX_3WD` workspace, mirroring the `DYX_4WD` build flow.
- `docs/architecture/DYX_3WD_Production_Stack_Architecture_V1.md` — the specification.
- `docs/Firmware/F-tasks.md` — upstream issue audit + 39-commit carry-over audit.
- `.github/workflows/ci.yml` — 6 jobs (see below).
- 12 ROS package skeletons, backend, 5 systemd units, installer skeleton, `CLAUDE.md`.

**Deliberate divergences from DYX_4WD** — do not "fix" these to match:
1. **Five services, not four.** `dyx3-rtk` is a sibling of the backend, not a child. In
   `PX4_DXP` the NTRIP client was a child process of `rover-server`, so any `server/**` deploy
   silently dropped the rover to FLOAT with no warning in the app.
2. **`dyx3_rpp_legacy` exists** — a quarantined `ament_python` package. 4WD has no equivalent.
   CI job `legacy_quarantine` enforces the four quarantine terms.
3. **No `dyx3_trajectory` package.** The path engine stays Python in `backend/path_engine/`
   (spec 7.2). 4WD has a C++ `dyx_trajectory`.
4. **`dyx3_geometry` is native-testable** (`-DDYX3_NATIVE_TESTS=ON`), with its own CI job.
5. **`dyx3_motion_guard` / `dyx3_px4_link`** replace 4WD's `dyx_motion_control` /
   `dyx_px4_gateway` — named for responsibility, not message conversion.

**Ran**
- `python3 -c "yaml.safe_load(...)"` on `ci.yml` — parses, 6 jobs.
- Quarantine + hygiene logic by hand before committing. **This caught a real bug in my own
  CI**: `legacy_quarantine` grepped the production manifest for `dyx3_rpp_legacy` and matched
  the comment that documents why it is excluded — a correct manifest would have failed the
  build. Fixed by stripping comments (`sed 's/#.*//'`) before the grep.
- `clang-format -i` across 120 stub sources after CI run 1 failed on formatting: Google style
  collapses empty namespace bodies to `namespace X {}  // namespace X`.

**CI status: GREEN, 6/6** — run `33915731980` on `2c7aedd`.
- ✅ colcon build + test (all 12 packages, incl. rosidl and the ament_python package)
- ✅ dyx3_geometry native tests (no ROS installation present)
- ✅ clang-format · ✅ backend ruff+pytest · ✅ legacy_quarantine · ✅ repo hygiene

Green CI proves the packages **configure and build**. It proves nothing about behaviour —
there is none yet.

**Not run**
- `colcon build` locally — no ROS 2 on this Mac.
- Anything on hardware. **Nothing in this repo has run on a rover.**

**DERIVED — NOT FROM V1 SPEC**
- Package module file lists (e.g. `dyx3_rpp`'s ten modules) are read off the architecture's
  decomposition table but the exact file split is a judgement call.
- systemd hardening stanzas are ported from `PX4_DXP`'s `px4-dxp.service`, including the
  `SCHED_FIFO 80` / `CPUAffinity=4` block on `dyx3-ros`. Those numbers came from a CubeOrange+
  Jetson tuning session and **must be re-derived** for the production hardware.
- `/opt/dyx3/current/bin/dyx3-*` ExecStart paths assume the filesystem layout in spec 12.

**Next agent should do first**
1. Push and get CI green. It has never run.
2. Milestone 2 — freeze `dyx3_interfaces`, starting with `MotionSetpoint`.
3. Classify the 173 legacy parameters (119 RPP + 54 spray) as LIVE / IDLE_ONLY / RESTART.

**Open questions for the human**
- `F-tasks.md` Part E, questions 1–6 — in particular whether to reproduce upstream #27497
  (rover differential won't turn in Mission Mode on v1.17) before designing the F4 gate.
- Commit attribution: `CLAUDE.md` in the firmware repos says no Claude attribution; the current
  session config requires a `Co-Authored-By` trailer. Which wins?

---

## 2026-09-05 (later) — Claude — CI green, status captured

**Changed**
- `2c7aedd` — clang-format across 120 stub sources. CI now 6/6 green.
- `CLAUDE.md` §3b — current status, decided-and-not-to-be-re-litigated list, known risks.
- Firmware repo `CLAUDE.md` — current status, F1 order, upstream blockers.

**Repositories now standing**

| Repo | State |
|---|---|
| `Vetri2425/DYX_3WD` | public, `master`, CI 6/6 green |
| `Vetri2425/PX4-Autopilot-3WD-Prod` | public, `dyx-3wd-production`, v1.17.0 pinned, build green |
| `3WD_PROD/PX4-Firmware/3WD/` (4WD: `Way_to_Mark/PX4-Firmware/4WD/`) | per-vehicle artifact archive |

**Notable result:** the firmware artifact records `git_identity = f3de5d1` — our commit, not
the base hash. The old fork's `cp`-overlay CI made every build record `54f0455f` regardless of
content, which is why `ver_sw` could never identify a flash. Building from a real committed
tree fixes it structurally.

**DERIVED — NOT FROM V1 SPEC**
- Nothing new this session beyond the Milestone 1 markers above.

**Next agent should do first**
1. **Stage 0.1** — quantify the arc-tracking payoff analytically from existing bags. Days of
   desk work that either validates the programme or reshapes it. Cheapest possible test of the
   central hypothesis; do it before building more.
2. Milestone 2 — freeze `dyx3_interfaces`, starting with `MotionSetpoint`.
3. F1.1 — RoboClaw drivetrain patches as semantic diffs against v1.16.2 ancestors.

**Open questions for the human**
- `F-tasks.md` Part E, questions 1–6. Most consequential: reproduce upstream #27497 before
  designing the F4 gate, or design around it?
- ~~Pin `clang-format` in CI?~~ **CLOSED 2026-09-05** — pinned to `20.1.8` via the PyPI
  wheel in both repos, version echoed into the CI log, pin recorded in `.clang-format`.
  Verified beforehand that 20.1.8 and 23.1.0 both report 0 violations on both trees, so the
  pin is not masking a disagreement.
- ~~Commit attribution?~~ **CLOSED 2026-09-05, human decision: no AI attribution.** The rule
  is now explicit in all four repos' `CLAUDE.md` and overrides any tool or session default.

  ⚠ **Ten already-pushed commits still carry a `Co-Authored-By` trailer** (4 here, 1 in
  DYX_4WD, 4 in the 3WD firmware repo, 1 in the 4WD firmware repo). They were deliberately
  **left as-is**, for two reasons:
  1. Cleaning them means rewriting and force-pushing public shared branches, which every
     one of these repos explicitly forbids.
  2. Firmware commit `f3de5d1ccd` is load-bearing: its SHA is embedded in the archived
     artifact directory name, in `build_info.txt`, and **inside the built binary itself**
     (`git_identity = f3de5d1`). Rewriting it would break the artifact-to-commit provenance
     chain — the exact property that makes a flash identifiable and that the `cp`-overlay
     approach never had. A trailer is cosmetic; that chain is not.

## 2026-10-07 — Claude — Track F1 firmware rebase complete (code), GATE 2 passed

**Branch:** `claude/f17-b2-decisions` (this repo, docs only). Firmware commits are in
`Vetri2425/PX4-Autopilot-3WD-Prod` on `dyx-3wd-production`, each pushed alone, CI green,
artifact archived in `3WD_PROD/PX4-Firmware/3WD/<sha>-<slug>/`.

| Roadmap item | Firmware commit | Gate evidence |
|---|---|---|
| CI target → `px4_fmu-v6x_rover` | `1416913c85` | CI |
| F1.1 RoboClaw QPPS / raw UART / select / 255 / deadbands | `14e3fb88b2` | CI |
| F1.2 logger (`wheel_encoders`, all multi-EKF aid-source instances) | `385034b7ef`, `809b3561c0` | CI |
| F1.3 WENC fusion + lever arm (`fuseBodyFrameVelocity` extracted from EV) | `b5189bd734` | GATE 2: neutrality (state bit-identical) + compatibility vs v1.16.2 (pivot wobble 1.86 vs 1.96 cm; lever-arm failure reproduced on both) |
| F1.4A RoboClaw encoder timestamp before UART | `07bcfdc601` | CI (replay cannot exercise timing) |
| F1.4B `EKF2_GPS_YAW_N/_G` | `1cc5c364b8` | GATE 2: reproduces 08-05 failure (16.8/24.4 % vs recorded 17.5/24.3 %), floor fixes it, 0 reverse actions |
| F1.5 RTCM over DDS (`/fmu/in/gps_inject_data`, PublicationMulti) | `07741c2f26` | CI + generator check |
| C5 DDS reconnect after agent restart (upstream #26848 backport) | `1bc34ef933` | CI |
| F1.8/C3 ULog streaming over DDS | `9f07777a32` | CI + generator check |
| F1.7 explicit-setpoint gate in `DifferentialOffboardMode` | `b19901b004` | CI (see contract) |

**Findings that correct earlier docs**
- F-tasks C1 is already satisfied on v6x: `rover.px4board` is a variant of `default.px4board`,
  which has `CONFIG_MODULES_UXRCE_DDS_CLIENT=y`; Ethernet/netman fallback IP is 10.41.10.2.
- A1.2 / #27497 is not a firmware bug (reporter: a motor not flagged Reverse).
- #27860 (C5 original concern) is serial-baud specific; startup retry (#23723) is in v1.17.0.
  The real gap was reconnect after agent restart (#26848), now backported.
- C6 is not needed under path A: OFFBOARD already has `COM_OF_LOSS_T` freshness; the remaining
  hole (live heartbeat, stale setpoints) is a companion obligation.

**What I ran / could not run:** CI builds of every commit (pinned container); local SITL
replay builds and 900+ replays for GATE 2 (verification only, never archived). Not run: anything
on hardware — no flash, no bench, no DDS agent, no RTK.

**DERIVED — NOT FROM V1 SPEC**
- GATE 2 needed a v1.16.2→v1.17 `SensorGps` ULog converter (2 additive fields, byte-level,
  round-trip-verified) — without it v1.17 replay silently drops all GNSS.
- Replay of these logs is not native EKF2 replay (`SDLOG_PROFILE=1`, no `ekf2_timestamps`) and
  is run-to-run nondeterministic on every binary, including pre-F1.3; it is ~3× pessimistic on
  absolute pivot wobble vs the recorded field estimate. Relative comparisons only.

**Half-finished / next**
1. Bench (spec F2/F4 + GATE 1): set `RBCLW_QPPS_MAX`, `RO_MAX_THR_SPEED`; motor/encoder signs;
   128/128 zero; dual-antenna heading vs physical heading; keep `EKF2_WENC_CTRL=0` until
   `EKF2_IMU_POS_*` is re-measured on the 6X mount.
2. GATE 1: STOP / TRACK_HEADING / TRACK_RATE / PIVOT / CREEP / reverse + companion kill →
   closes B2 rows 15/16 (`docs/contracts/F1.7_B2_firmware_decisions.md`).
3. Field: RTK over DDS (`gps status` + FIX transition), ULog stream completeness, DDS agent
   restart (C5).

**Still open for fine tracking** (`PX4_DXP/docs/FIRMWARE_PENDING_PATCHES.md`, each confirmed
present in v1.17 source): A1 RoboClaw serial never resyncs after a stray byte (no `tcflush`);
A2 encoder reads every mixer cycle; **A9 `fuseBodyFrameVelocity()` refreshes the global
velocity-fusion timers — while WENC fuses, GNSS loss is masked and the GSF yaw rescue cannot
fire; fix before enabling WENC**; A6 zero-side-slip constraint stays on during pivots; GPS-driver
submodule C2 (NMEA restart cycle), C3 (config spam), F7 (variance-as-σ). C1/C4/F5 are resolved
by dropping the always-landed land-detector patch (verify F5 in the first bench log).
Parameter-only (bench): `EKF2_GPS_P_NOISE` 0.015, `EKF2_GPS_V_NOISE` 0.05, `RO_YAW_RATE_TH` 0.5
after F1.4B, re-measured `EKF2_IMU_POS_*` / GNSS antenna positions, `GPS_YAW_OFFSET`.

**Open questions for the human**
- `COM_OF_LOSS_T` and `COM_OBL_RC_ACT` values (left OPEN in the contract).
- Spray valve output on the 6X (deferred; parameter only).
- The WENC +2.3 cm GNSS-position innovation seen in replay on both v1.16.2 and v1.17: calibrate
  `EKF2_WENC_RAD` on the new vehicle before enabling WENC.

### 2026-10-07 (later) — Claude — GNSS ownership rule, heading-recovery investigation, fine-tracking batch

**Human decisions:** PX4 must not auto-configure the GNSS receiver (contract
`docs/contracts/GNSS_receiver_configuration.md`; current firmware still violates it via
`request_unicore_messages()`). A9 held: neutral in replay but its mechanism was never observed,
and `_time_last_hor_vel_fuse` also drives dead-reckoning classification — prove on the bench first.

**Firmware since the F1 handoff:** A1 RoboClaw RX resync `420768d812`, A2 speed-only encoder reads
(`wheel_angle` = NaN) `4eb9465e3c` — both CI green, archived; `4eb9465e3c` is the current flash
candidate. A9 patch preserved (worktree + `PX4-Firmware/3WD/_held_patches/`).

**Heading-recovery investigation (replay, +90°/+20° GNSS-heading fault, WENC off):** not a defect
introduced by any of our patches — identical in field-flown v1.16.2, stock v1.17 and current.
EKF2 resets only on a *continuous* ~8 s rejection run. If the fault is mostly rejected (log_311) the
EKF resets cleanly and recovers 10.8 s after the fault ends; if it is mostly accepted (log_285,
creeping rover) post-fault rejections are intermittent, the reset never fires, and heading stays
tens of degrees wrong for ~190 s with no flag. Mitigation belongs in the companion
(`dyx3_motion_guard`): monitor `reject_yaw` and GNSS-yaw innovation/test ratio, pause on unhealthy.
Firmware option for later (design decision): reset on rejection *ratio*. Add a heading-disturbance
field test (e.g. shade one antenna) with recovery time logged. Correction: an earlier report of
"stuck ≥ 30 s on both logs" was a measurement error (window maximum included pre-reset time).

## 2026-10-07 (evening) — Claude — 3WD_PROD move, Jetson bring-up, first flashes, DDS link proven

**Branch:** `claude/proposal-ethernet-dds-only` (this repo, docs only). Firmware commits in
`Vetri2425/PX4-Autopilot-3WD-Prod` on `dyx-3wd-production`.

**Roles from here (human decision):** ChatGPT writes code. Claude reviews, deploys and installs
(Claude has human-equivalent authority in this repo, incl. `installer/`/`deployment/` and merging
its own PRs after green CI). Flow: code on the Mac → `claude|codex/<topic>` branch → PR → merge →
Jetson pulls and builds → services restart. Never hand-edit on the Jetson.

**Layout:** production repos moved `~/Vetri/Way_to_Mark/` → `~/Vetri/3WD_PROD/`
(`DYX_3WD`, `PX4-Autopilot-3WD-Prod`, `PX4-Firmware/3WD/`). Way_to_Mark is 4WD-only.
⚠ Commit `196e3db` (path update) was pushed **directly to master** — a §5 violation; not repeated.

**Hardware (Holybro Pixhawk Jetson Baseboard, Orin Nano 8 GB, Ubuntu 22.04.5, L4T R36.5, NVMe):**
hostname `dyx-3wd`, user `flash`, SSH key from the Mac (`ssh dyx-3wd`, finds the Jetson by MAC),
passwordless sudo until 2026-12-07 (systemd timer removes it). JetPack/CUDA deliberately not
installed — no GPU workload in the spec. Bring-up gotchas are in
`docs/architecture/proposals/2026-10-07_ethernet-dds-only-companion-link.md` (recovery switch,
POWER1 powers the Ethernet switch, netman `/fs/mtd_net` survives flashing).

**Firmware flashed (first time on hardware):**

| Commit | What | CI | Archive | Flashed |
|---|---|---|---|---|
| `f4f99058c0` | SYS_AUTOSTART 50000; DDS over Ethernet defaults; static 10.41.10.2 (DHCPC removed); CI fetches tags → reports v1.17.0 not 0.0.0 (tag `v1.17.0` pushed to origin) | 37626647252 ✅ | ✅ sha256 f4c9c0db… | ✅ |
| `27a7ac9284` | `MAV_2_CONFIG 0` — Jetson↔PX4 is Ethernet + uXRCE-DDS only | 37628456182 ✅ | ✅ sha256 56e8134d… | ✅ **current** |

Board network switched once via `net.cfg` + `netman update` (it had a stored 192.168.0.3
fallback). The firmware agent's Slack summary says "nothing flashed" and "27a7ac9284 has no
archive" — both superseded by this table.

**Verified on hardware (2026-10-07):** `ver` = 1.17.0 release, NuttX 12.12.0; PX4 eth0
10.41.10.2 ↔ Jetson 10.41.10.1 ping 0.2 ms; MicroXRCEAgent v2.4.3 on :8888 → session from
10.41.10.2, `uxrce_dds_client` connected, timesync converged; `ros2 topic list` shows 68 `/fmu`
topics (28 out / 40 in) incl. `rover_{speed,attitude,rate}_setpoint`, `gps_inject_data`,
`ulog_stream`. MAVLink now only on TELEM1 (57600) + USB.
**Not verified:** topic payloads (px4_msgs build unfinished), RTK, any motion, agent across reboot.

**Jetson software state — NOT production yet:** installed by ad-hoc scripts in `~flash`
(`~/dyx3_deps_stage{1,2,3}.sh`), which must be replaced by `installer/`:
- stage 1 ✅ ROS 2 Humble ros-base, ros-dev-tools, rosdep, ament_cmake_gtest, clang-format (no `apt upgrade`)
- stage 2 partial: MicroXRCEAgent v2.4.3 → `/usr/local` ✅; `~/px4_ws` px4_msgs (release/1.17
  skeleton + firmware `msg/`+`srv/` @ f4f99058c0, identical to 27a7ac9284 — the firmware changes
  `EstimatorAidSource3d.msg`, so stock px4_msgs would silently mismatch) was building when the
  Jetson became unreachable (SSH banner timeout → likely memory pressure at `-j3`); backend venv
  and DYX_3WD colcon build not reached
- stage 3 queued: mavlink-router → would enable upstream `mavlink-router.service` with
  `/etc/mavlink-router/main.conf` (PX4 UDP server 10.41.10.1:14550, QGC TCP 5760) — must be
  folded into `dyx3-platform` instead
- the XRCE agent was started by hand (nohup) — does not survive reboot

**Next (in order)**
1. Installer (`installer/lib/*`) reproducing the above into the §12 layout: `dyx3` user,
   `/opt/dyx3/{releases,current,bin}`, `/etc/dyx3`, `/var/lib/dyx3`; pinned XRCE agent v2.4.3,
   pinned mavlink-router, px4_msgs built from a firmware-SHA pin with `MAKEFLAGS=-j1/-j2`;
   NM profile for the FCU Ethernet; systemd units. Then `dyx3-upgrade/rollback/health/version`.
2. `dyx3-platform` = XRCE agent + mavlink-router under systemd. Enable only implemented services
   (the other four units are still `exit 1` stubs).
3. Run the installer on the Jetson, prove it, delete the `~flash` ad-hoc files.
4. Spec work: Milestone 2 (`MotionSetpoint`), 174-parameter classification, F3 reconnect gate.

**DERIVED — NOT FROM V1 SPEC**
- `dyx3-platform` (not a separate `dyx3-px4-link.service`) supervises the agent — see proposal.
- Bench network: FCU Ethernet profile keeps DHCP alongside 10.41.10.1/24 while on a site router;
  production is static-only with external RJ45 ports empty.
- Hostname `dyx-3wd`; §4.3 names `rover-3wd.local`. Open.

**Open questions for the human:** hostname (`dyx-3wd` vs `rover-3wd`); whether the recorder's
loss of FCU parameter snapshots (MAV_2 off) is acceptable for provenance or needs a USB/param
dump step; the uncommitted EKF2 edits in the firmware working tree (other session) — owner?

## 2026-10-07 — Codex — Phase 1 stopped before interface edits

**Inspection completed**
- Read the V1 architecture, `CLAUDE.md`, Phase plan, current handoff, accepted Ethernet/DDS
  proposal, all current contracts, the `dyx3_interfaces` package manifest/CMake/message, and
  the relevant `PX4_DXP` source, tests, README, and PX4 message definitions.
- Confirmed that `ros2_ws/src/dyx3_interfaces/CMakeLists.txt` and
  `ros2_ws/src/dyx3_interfaces/msg/MotionSetpoint.msg` were already uncommitted when this
  session began. They were not modified by this session.

**Resolved source/spec count mismatch — human decision 2026-10-07**
- Freeze the registry at **173 = 119 RPP + 54 spray**. The old 120/174 figure is a broad-grep
  artefact: its extra match was a comment at `PX4_DXP/src/rpp_controller_node.py:679`.
- The source method is the count of executable `declare_parameter(` calls, excluding comments;
  there are 119 unique RPP names and the count has been stable since prototype commit
  `42d8d4b` (2026-08-20). See
  `docs/architecture/proposals/2026-10-07_parameter-count-173.md`.

**Not run at the stop point**
- No build or test yet: the stop condition was reached before implementation.
- `colcon` remains unavailable on this Mac.

**Git state**
- No commit had been created when the initial count question stopped the work.

## 2026-10-07 — Codex — Phase 1 complete, pending Claude review

**Changed**
- Froze `dyx3_interfaces` at package version `0.1.0`: nine messages, six mission/emergency
  services, and `ExecuteMission`; added an ABI/default gtest.
- Corrected `MotionSetpoint`'s contract comments: ROS-time stamp, NED yaw/yaw-rate, signed
  body-X speed, invalid-is-STOP semantics, and publisher-session sequence semantics.
- Added the single frame/sign/unit/clock contract at `docs/contracts/frames.md`.
- Added the 173-row registry at `docs/tuning/parameter_registry.md`; all classes are
  `TBD — human` because the prototype evidence does not assign Phase 9 mutability classes.
- Added the accepted 173-count proposal, updated the mutable Phase plan, and added the
  interface changelog plus CI version/changelog enforcement.

**Ran**
- `git diff --check` — passed.
- Basic ROS IDL field-syntax validation over all new `.msg`, `.srv`, and `.action` files —
  passed.
- Registry audit: 173 unique documented names equals 119 executable RPP declarations plus
  54 executable spray declarations — passed.
- `clang-format==20.1.8 --dry-run --Werror` over all C++ sources — passed.
- Python YAML parse of `.github/workflows/ci.yml` — passed; confirms `interface_freeze` exists.

**Not run**
- `colcon build/test`: **colcon NOT RUN**. This Mac has no colcon and Docker is unavailable,
  so the required Humble container command cannot run.
- No hardware, DDS, or rover test was attempted.

**DERIVED — NOT FROM V1 SPEC**
- The concrete fields and ABI reason/state enum values in the interfaces are the minimum
  typed surface needed by the named Phase 1 contracts. Their zero values are deliberately
  safe states; all motion uses the separate invalid-is-STOP rule.

**Next**
- Claude should review the unpushed local Phase 1 commit. Do not start Phase 2 without human
  approval.

---

## 2026-10-07 — Claude (cloud, branch `claude/cloud-phases`) — P1.1, P11a, P12

Cloud session, code only. Branch `claude/cloud-phases` (not master). **No attribution trailers** per CLAUDE.md §5.
Running log; later phases append below. Toolchain used: `ros:humble-ros-base` from `public.ecr.aws` (Docker Hub rate-limited),
`mavros_msgs`+`geographic_msgs` built from source for message definitions only, `px4_msgs` built **by the installer's own
`build_px4_msgs`** from firmware `27a7ac9284` (4 min at `-j4` in the container; the code path is verified, the Jetson `-j1` timing is not).

### P1.1 — Phase 1 review fixes (`dyx3_interfaces` 0.1.0 → 0.2.0)
Done: `RtkStatus.fix_type` mirrors `SensorGps` (`FIX_3D` 2→3; added 2D/RTCM_CODE_DIFFERENTIAL/EXTRAPOLATED); `MotionSetpointStatus`
reasons 9–12 (heading unhealthy, operator link lost, arming gate, estimator unhealthy); new `EstimatorHealth.msg`; `AbortMission` has a
defined 0 and `REASON_`-named result; `ExecuteMission` documented as canonical (`StartMission` = admission wrapper, same `dyx3_mission`
code path); parameter registry has a PROPOSED class + rationale for all 173 (77 LIVE / 88 IDLE_ONLY / 6 RESTART / 2 TBD).
**Finding (needs human):** `/fmu/out/estimator_status` and `estimator_aid_src_*` are **not in `dds_topics.yaml` @27a7ac9284**; only
`estimator_status_flags` is. So test ratios (yaw/pos/vel) are unavailable over DDS until a firmware change; `EstimatorHealth.test_ratios_valid`
stays false and the guard must use the boolean flags (`reject_yaw`, `cs_gnss_yaw_fault`, `cs_inertial_dead_reckoning`, `reject_hor_pos/vel`).
Ran: colcon build+test (8 interface tests pass). DERIVED: new reason numbers; EstimatorHealth field set.

### P11a — minimal deployment slice
`installer/` (install.sh, upgrade.sh, verify.sh, lib/*, pins/*, tests) + `deployment/scripts/start-platform.sh` + `dyx3-platform.service`.
See `installer/README.md`. Ran: shellcheck clean; `installer/tests/run_tests.sh` 41/41 on a staged root with fake colcon/ss/ping (build refusal,
atomic switch, auto-revert on failed health, config preservation, prune, unknown ref, supervisor restart/SIGTERM); the real `build_px4_msgs`
end-to-end in the Humble container (found + fixed a real bug: the sparse firmware checkout was inside the colcon workspace).
**NOT run**: anything on a Jetson — apt, ROS apt repo, pinned XRCE agent + mavlink-router builds, NetworkManager, systemd, real DDS.
DERIVED: mavlink-router pin = tag `v4` (`42529d5`) — HANDOFF named no version; XRCE agent installs to `/usr/local` (as the proven bring-up); `dyx3-health` is
minimal (platform only); FCU Ethernet profile has `FCU_KEEP_DHCP=1` for the bench; config templates are `.tmpl` because the hygiene gate forbids tracked `*.env`.

### P12 — legacy port-in + contracts
`dyx3_rpp_legacy`: `rpp_controller_node.py` + 5 helpers + `path_publisher_node.py` copied **byte-for-byte** from PX4_DXP `build/demo-ready` @ `fc6436b`
(sha256-pinned, `test/VERBATIM.sha256`), 26 of the prototype's own tests carried verbatim. Only new code: `output_stage.py` (pure, decodes NED vector + yaw rate →
`MotionSetpoint`, incl. reverse and pivot hysteresis), `legacy_node.py` (subclass overriding exactly `_publish_velocity/_publish_yaw_rate`),
`input_shim.py` (VehicleState/RtkStatus → MAVROS-named topics). `docs/contracts/rpp_*.md` (8 docs).
Ran: 29 package tests + prototype suite: **183 pass / 7 fail, identical in the untouched PX4_DXP checkout** (`rpp_legacy_evidence.md`).
**Findings (need human):**
1. **Default divergence** — `docs/tuning/default_divergence.md`: `build/demo-ready` (registry source) vs older `Upgrade_speed` retuned set (`min_lookahead 0.35`, `a_lat_max 0.3`, `mission_speed 0.35`…). Which seeds production?
2. **Spot-turn thresholds conflict inside the prototype** (docstring 30°/5°, comment 10°/5°; as-flown FCU params 40°/2°). Used 40°/2° (DERIVED).
3. **Pivot-rate law is unspecified** (old firmware owned it); legacy uses `1.5·err`, clamp 0.45 (DERIVED, GATE 1).
4. `output_mode` default `heading` (field-proven information flow); `rate` is the precision mode — flip after GATE 1. The legacy yaw rate is already closed-loop (pure-pursuit κv / 1.5·θe).
5. `/path` (nav_msgs/Path, z bitfield) and point-handshake topics have no adapter yet — needs Phase 3/4 path artifact.
NOT run: the legacy node under rclpy with live topics; `input_shim`.

### LOCAL ACTIONS NEEDED (Mac, bags at `~/Vetri/3WD_Proto/PX4_DXP/bags`, `PX4_Logs`)
1. Legacy decode evidence: `python3 tools/extract_legacy_decode_fixture.py <bag_dir> -o ros2_ws/src/dyx3_rpp_legacy/test/fixtures/legacy_decode_<name>.json` for a square bag (e.g. `square_20260611_*`), an arc bag and an extension line bag; then `pytest ros2_ws/src/dyx3_rpp_legacy/test/test_fixture_decode.py` (currently SKIPPED, not passing).
2. On the Jetson: `installer/install.sh --production` and `dyx3-upgrade <ref>`; confirm `-j1` px4_msgs build time/memory; `systemctl status dyx3-platform`; `dyx3-health --deep`.
3. Re-run the carried prototype tests: `DYX3_RUN_DXP_TESTS=1 pytest ros2_ws/src/dyx3_rpp_legacy/test/dxp_verbatim` and decide what the 7 failures mean.


### P4 — path engine carried, `DYX3PATH 1` artifact, `dyx3_geometry`
Path engine copied into `backend/src/dyx3_backend/path_engine/` (only the import prefix changed; `ORIGIN.sha256` + provenance test). 466 of its 509 tests
pass; 43 (old `PathManager`) skip with a reason. Artifact (`docs/contracts/path_artifact.md`): ASCII, LF, id = sha256 of the bytes, strict decoder, atomic store.
`dyx3_geometry`: 13 no-ROS functions; `test/gate3_equivalence_test.cpp` compares 119 147 values against the verbatim PX4_DXP ancestors (max 1.1e-13 / 2 ULP),
unit tests 855 checks clean under ASan/UBSan. **GATE 3 is demonstrated on the Git corpus only — NOT closed:** the bag-derived poses and field missions are a LOCAL ACTION
(`tools/extract_geometry_bag_fixture.py`, then `geometry_bag_replay_test`, currently exit 77 = SKIPPED). Fixture must be generated under **Python 3.10** (3.12 `sum()` is compensated).

### P3 — `dyx3_mission`
Contract first (`docs/contracts/dyx3_mission.md`): 8 states, transition table, never auto-resume, E-stop aborts, READY waits for the RPP ack.
Pure-C++ FSM, SHA-256, artifact reader, point journal; thin node (action + 5 services). 5 ctest targets pass in the Humble container.
DERIVED: `rpp_ack_timeout_s` default 0 (disabled) and RPP-staleness have no numeric source. **Open (human):** E-stop *aborts* vs *pauses*.

### P7 — `dyx3_px4_link` (`dyx3_interfaces` 0.4.0)
Contract first: `docs/contracts/dyx3_px4_link.md`. Pure core (no ROS, no px4_msgs): `message_hash`, `rover_setpoint_writer` (mapper + `CommandGate`), `dds_session`
(per-topic staleness, #27388), `msg_version_handshake`, `offboard_heartbeat`, `vehicle_state_assembler`; one node that touches `/fmu/**`.
Facts found in the **firmware source** that change the design (not in the spec):
1. The handshake request must carry the **base** topic name (`/fmu/out/vehicle_status`, no `_v1`): the firmware matches the uORB name (`uxrce_dds_client.cpp:448-455`).
2. The firmware converts timestamps in both directions inside its serialisers: the link stamps with the **Jetson system clock** and never applies `estimated_offset`.
3. `/fmu/in` publishers are **reliable** (the XRCE agent's reader; a best-effort writer would not match) — unverifiable off-target, check with `ros2 topic info -v` at GATE 1.
Proof: the C++ hash equals the firmware's own Python `get_message_hash` (run unmodified behind a parsing shim, `tools/px4_msg_hash/`) for **all 235 messages** of the pinned set.
Fault-injection tests with a fake FCU: no setpoints before handshake, mismatch is loud and refuses everything, session loss re-arms the handshake, one silent topic
forces STOP while the session stays up, guard silence -> explicit zero with the heartbeat kept, sequence reset, RTCM/ULog paths.
CI now builds `px4_msgs` from the pinned firmware (cached) before `colcon build`.
DERIVED: all `stale_*_s` limits and `command_max_age_s` 0.2 (from prototype `input_max_age_s`) — re-validate at GATE 4; "STOP with heartbeat kept" reading of the F1.7 obligation;
`LOGGING_START param1=0` (firmware logger source not in the sparse checkout).
NOT run: any DDS to a real FCU, real timesync, jitter, the kill-process stop distance (A1.1 / F5).

### P6 — `dyx3_motion_guard` (SAFETY-CRITICAL)
Contract first (`docs/contracts/dyx3_motion_guard.md`). Pure core: `motion_types`, `limits`, `estop_gate`, `rtk_gate`, `mission_gate`, `fail_to_zero`,
`freshness_watchdog`; fixed-rate (50 Hz) node that always publishes (a valid command or the canonical STOP) so the loss of RPP is itself reported.
Ordered 12-row decision table; every row has a test, plus a randomised property test: **no non-STOP command ever leaves the guard while any gate fails**.
24 core + 8 node tests (fault injection, private DDS domain, injected clock).
DERIVED: limits are PROTOTYPE defaults (forward 1.0, yaw 0.45, accel 0.20, decel 0.50) — re-validate GATE 4; **reverse limit 0 (no source)**; jerk / yaw-accel limits
off (no source); `command_max_age_s` 0.2; `session_accept_count` 3; E-stop boots CLEAR (physical E-stop is hardware); parameters are read once at start (the LIVE
class is the target, the runtime-change callback is NOT built).
Open (human): E-stop abort vs pause; reverse limit; ratio limits need `estimator_status` on DDS (firmware `dds_topics` change).

### P5 — `dyx3_rpp` (PARTIAL — modules, not the controller)
Built: `rpp_params` (119 parameters generated from the registry, class enforcement, journal), `guidance`, `speed_profile`, `stop_pivot_fsm` (explicit `CornerFsm`,
`StopConfirm`, `PivotWatchdog`, `StopHold`; times in int64 ns), `terminal`, `motion_output`. `gate4_equivalence_test` compares ~24k values bit-exactly with the
verbatim Python (`tools/gate4/gen_rpp_vectors.py`); a 1 % error in the slew law is caught.
**NOT built (listed in `docs/contracts/dyx3_rpp.md`): orchestrator tick, path conditioner, `rpp_node`, run sequencer, `spray_gate`, `RppStatus` jitter, point-hold /
handshake / precise-stop features.** Nothing drives a rover yet. DERIVED: `kPivotRateGain` 1.5 (the old firmware owned the pivot-rate law).

### P8 — `dyx3_gnss_rtk`
Contract first. `rtcm_parser` (CRC-24Q, resync one byte after the preamble, buffer cap), `rtcm_transport` (MAVLink `GPS_RTCM_DATA` flag layout, <=300 B chunks),
`correction_health`, `gga_provider`, `ntrip_client` (pure protocol + threaded poll() socket client), `rtk_node`. Nothing here configures the UM982; no serial access.
**Credentials come from the environment only** (`DYX3_NTRIP_*`, `EnvironmentFile`), never argv / logs / Git.
Proof: 22 RTCM tests, loopback fake caster (8), node tests (6), `ntrip_equivalence_test` vs the prototype (300 streams, 400 CRC, 600 GGA; the fixture records the source sha256).
DERIVED: `correction_fresh_s` 10 (the prototype's stream-liveness bound), `gnss_report_max_age_s` 1.0, `rate_window_s` 10, `max_buffer_bytes` 8192.
Interfaces 0.5.0: `GnssReport` gained lat/lon/alt/hdop; new `NtripStatus`. NOT run: a real caster, the LTE link, the GPS driver's reassembly, an RTK FIX transition.

### P9 — `dyx3_spray` (actuator + boundary semantics) — interfaces 0.6.0
Contract first: `docs/contracts/dyx3_spray.md`. Three authority layers: controller (`spray_node`, decides WHERE), `dyx3_px4_link` (the only package touching `/fmu`;
DO_SET_ACTUATOR 187 / DO_SET_SERVO 183, immediate `result 255` ack when the link is not proven), and the **independent watchdog** (`spray_watchdog`, its own executable;
OFF on absent / denied / malformed / stale lease; startup fail-closed; proves it can close the valve before the controller may open it).
Pure cores: `spray_fsm` (7 states, `spraying` only in ON_CONFIRMED, acks matched by `cmd_seq`), `safety_lease`, `flow_model`, `boundary_projection` (windowed projection
with the direction gate, MARK lead, terminal shutoff, cross-track hysteresis), `spray_gates`, `spray_controller`, `watchdog_core`.
Proof: `spray_equivalence_test` — 17 001 FSM events, 600 lease + 893 monitor, 1 831 flow, 2 500 RTK-quality, 15 365 gate steps, **12 900 decisions** vs vectors from the
VERBATIM prototype (`tools/gate4/gen_spray_vectors.py`, sources pinned in `tools/gate4/spray_dxp/VERBATIM.sha256`); decisions exact, derived distances to 1e-12 (CPython
`hypot` != libm); six mutations each caught. 24 core + 8 gate + 4 defect + 11 node tests (fake FCU link, controller death closed by the watchdog, link refusal never latches ON,
E-stop, manual override, runtime parameter classes). `spray_params`: 45 of 54 registry rows (the 9 excluded rows belong to unported features, with reasons in the generator).
**KNOWN-OPEN DEFECT (spec 7.8) — NOT FIXED.** `projection_direction_gate_deg` default stays **0.0 = disabled** (a spray-boundary decision for a human with field evidence).
`test/spray_defect_test.cpp` reproduces the mechanism on a synthetic out-and-back (the gate-off case ASSERTS the teleport; the gate-on case is continuous). Synthetic, not the bags.
NOT ported (listed in the contract): dash and point modes, the `/spray/active` heartbeat, RPP-progress boundary source, legacy fallback, MAVROS service path.
DERIVED (all in the contract): E-stop first in the gate order and consumed fail-closed; vehicle state trusted only while fresh; RTK gate evaluated every tick (the prototype only when earlier
gates passed, which let a fix drop skip the recovery hold); tracking evidence = `RppStatus.TRACKING` only; watchdog burst keyed on the OFF cause (the prototype re-armed the 20 Hz burst every
tick while stale); node tick 50 Hz; path geometry = the planned artifact polyline, **not** RPP's conditioned path (open: re-derive boundaries when `path_conditioner` exists).
Observed, carried, in the boundary budget: **debounce (3 ticks) delays every valve edge ~2.1 cm at 0.35 m/s** and the lead maths does not compensate.
NOT run: valve timing (`solenoid_*_delay_s`), nozzle offset (1.6-6.6 cm, defaults 0.0), the real FCU ack path, paint quality, DDS timing.
Open (human): FCU / hardware valve fail-safe if `dyx3_px4_link` dies while the valve is ON (nothing in this repo can close it); default of `projection_direction_gate_deg`; debounce vs lead.

### LOCAL ACTIONS NEEDED (P6-P9)
1. Bags: replay `stg_d8a4f2ad` / `stg_46ba8830` through `spray_defect_test`'s drive model (the real defect); GATE 3/4 bag replay for geometry and RPP modules.
2. Regenerate / verify the vectors (need the path engine and rclpy, Humble container, **Python 3.10** for GATE 3): `tools/gate4/gen_spray_vectors.py --check`, `gen_rpp_vectors.py --check`, `gen_ntrip_vectors.py --check`.
3. Jetson: node tests are timing-sensitive; run `colcon test` on the target. `ros2 topic info -v /fmu/in/...` must show RELIABLE (px4_link). Confirm `mavlink-router` pin and `COM_OF_LOSS_T`.
4. (done in interfaces 0.8.1) `Px4LinkStatus.handshake_ok` comment corrected.

### OPEN ITEMS — NOT DECIDED (human, recorded 2026-10-07; do not resolve silently)
1. **Valve close when the DDS path fails.** Every OFF path in this repo (controller, independent watchdog, px4_link refusal) rides the same DDS link, so if DDS or `dyx3_px4_link` dies while the valve is ON, nothing here can close it.
   Human ideas, undecided: (a) use the currently unused secondary UART link to disarm the rover and close the valve; (b) a PX4 companion-loss failsafe parameter that disarms. Neither is verified:
   check in the firmware source/bench which parameter applies, and that the valve output goes to its closed (disarmed) level on disarm. Not designed yet.
2. **`projection_direction_gate_deg`** stays 0.0 (disabled). No decision.
3. **Decision input for 2 (tool built, missions still a LOCAL ACTION):** `spray_projection_replay --gates 0,45,60,90 <artifacts>` now exists (see the spray contract section 6; first table on the 4 archived missions: gate off leaves ~1.0 m of hole on one pitch and 1.3 m of spurious paint on the other, 45/60 degrees remove almost all of it, 90 is worse). Original wording: replay 5-10 real missions (bag-derived, with real MARK/TRANSIT boundaries; the Git corpus is almost all MARK) through the projection with the gate off and on, compare station continuity and valve edges, then decide.
   A replay harness is to be added; the missions are a LOCAL ACTION.

### P10 — `dyx3_recorder`, `dyx3_system_gateway`
**Recorder** (`docs/contracts/dyx3_recorder.md`): a run directory opens on `MissionState` RUNNING and closes on a terminal state: `manifest.json`, `versions.json` (copy of `/etc/dyx3/versions.json`),
`params_ros.json` (start+end), `params_fcu.json`, `config_snapshot/` (secrets excluded by name), supervised `ros2 bag record` child (SIGINT -> SIGTERM -> SIGKILL), ULog reassembled from `/dyx3/ulog_chunk`
with a gap list, `summary.json` with `provenance_complete`. It never gates a mission. 13 core + 7 node tests (fake bag child, injected clock).
**NOT proven:** `ros2 bag record` itself (rosbag2 is not installed in CI), disk-full, real ULog bytes. **`params_fcu.json` is always "unavailable"**: no FCU parameter read path exists in this stack (OPEN, human).
DERIVED/OPEN: the recorded topic list (raw `/fmu/out/**` and `/dyx3/rtcm` excluded), `min_free_bytes` 0 = no check, `bag_stall_s` off (no source for either), one run per (mission_id, run_index).
**Gateway** (`docs/contracts/dyx3_system_gateway.md`): strict JSON parser, NDJSON over a Unix socket (`/run/dyx3/gateway.sock`, 0660), command validator (syntax only, no policy), nine service forwards with the downstream verdict
returned verbatim, E-stop processed before everything else in a batch and never reported accepted when undelivered (`service_unavailable` / `timeout`), per-source aged telemetry snapshot, and the **operator-link heartbeat** (R13).
10 core + 7 node tests over a real socket. A Python client (the backend's) was also run against the real C++ node (`tools/gateway_smoke.py`).
DERIVED/OPEN: **`operator_link_timeout_s` 2.0 (no source; at 0.35 m/s = 70 cm)**, `telemetry_hz` 5, `max_clients` 4, `service_timeout_s` 2.0.

### P2 — backend
`docs/contracts/backend.md`. FastAPI + python-socketio, never imports rclpy. Hashed bearer tokens with viewer/operator roles, fail closed (**auth model DERIVED/OPEN**); an asyncio gateway client that never queues a command
for later and reports `delivered` true/false/unknown; upload -> the carried path engine -> content-addressed `DYX3PATH 1` artifact (size/extension/engine-error handling, the client filename is never a path); runs view; the tablet-heartbeat
relay (`heartbeat_relay_s` 0.5, `tablet_heartbeat_timeout_s` 1.5: DERIVED/OPEN; worst-case tablet-drop-to-guard delay = 1.5 + 2.0 s); Socket.IO hub (anyone may assert E-stop, only an operator clears). 40 new tests, 536 pass.
**Not built (OPEN):** NTRIP profile management (the credentials file is `root:dyx3 0640`; how the backend changes it is undecided), settings/storage beyond missions and runs, report generation, retention.
**Not ported, questions for the human:** the prototype's arbiter, joystick/manual-drive gateway and emergency-stop plumbing. Manual driving is a motion source and needs its own contract; none exists.
`cors_allowed_origins=[]` (same origin) until the tablet app's origin is decided. NOT run: uvicorn under systemd, the real tablet.

### P11 (remainder) — deployment
Launchers for ros / rtk / recorder / backend; **`dyx3-spray-watchdog.service` as its own unit** (not tied to dyx3-ros); `dyx3_bringup/control_graph.launch.py` (mission, guard, px4_link, spray, gateway; any node exit shuts the graph down so systemd
restarts it: DERIVED); `deployment/scripts/dyx3-env.sh` refuses to start without `ROS_DOMAIN_ID` (no number in the spec: set it in `/etc/dyx3/ros.env`) or the px4_msgs overlay of the pinned firmware; `dyx3-rollback` (verifies the target first, restores on
an unhealthy result, twice = undo), `dyx3-version` (the firmware identity printed is the PIN; the running FCU's is "unavailable"), `/etc/dyx3/versions.json` for the recorder, backend venv built with the release (failure = warning), health extras.
Installer tests 69/69 on a staged root. **`[enabled_services]` is still `dyx3-platform` only on purpose**: nothing but platform has ever run on a rover, and an enabled service that fails health makes `dyx3-upgrade` revert.
OPEN: per-node RT priority/affinity (the unit applies FIFO 80 / CPU 4 to the whole tree), DDS scoping (loopback-only is stricter than the eth0 whitelist and survives an unplugged FCU cable), the backend port 8000 (DERIVED).
NOT run: anything on a Jetson (systemd, apt, pip, NetworkManager, real DDS).

### Timesync (interfaces 0.7.0)
`Px4LinkStatus.timesync_{valid,offset_us,round_trip_us}`; the recorder logs them at run start and end (a sample older than 1 s is recorded as not valid). **The mission precondition F-tasks A1.4 asks for is NOT built**: no source gives a numeric convergence
criterion (the issue only says ~4 ms is expected vs ~40 ms right after boot = 1.4 cm). OPEN (human): the criterion and who gates on it.

### P0 — Stage 0 tooling
`tools/analysis/{timebase,arc_floor}.py`, synthetic-data tests only (see `docs/analysis/stage0.md`). 0.1 needs the (omega, heading error) series extracted from the real arc bags (LOCAL ACTION) and a human's lateral-geometry choice to turn a heading floor into centimetres;
0.2 needs real bags and the rewiring of the prototype's own analysis tools (not in this repo); 0.3 is not code.

### P5 (remainder) — the RPP orchestrator and node (`dyx3_rpp`, interfaces 0.8.0)
Contracts: `docs/contracts/rpp_orchestrator.md`, `rpp_node.md`, `rpp_path_conditioner.md`, `topics.md` (new: every topic/service/QoS extracted from the code).
**Path conditioner** (`condition_path`): the whole of `_path_cb`'s run building; 2 091 cases incl. 182 end-to-end calls of the carried `_path_cb`; five mutations caught.
**Orchestrator** (`RppCore::tick`, pure C++, no allocation in `tick`): the gates in front of the controller, goal test, smooth + segment tracking, and the **whole stop/pivot flow through the explicit
`stop_pivot_fsm`** (corner pivot, run-boundary hold, entry alignment, endpoint precise stop, completion hold) with ONE shared stop confirmation exactly as the prototype shares it. Only the point hold
is not ported (the tick publishes zero and the node reports `STATE_ERROR`, `handoff = 1`). **Proof: tick by tick against the REAL carried node** (`tools/gate4/gen_orchestrator_vectors.py`, Humble container; injected clock,
captured publishers, plain messages so float32 never rounds a double; a closed-loop kinematic rover model with pose/GPS/velocity blackouts, GPS float / poor / unknown accuracy, an EKF jump):
**97 episodes, 13 166 ticks, 0 mismatches**, episodes run to DONE; ~45 mutations applied to the C++, survivors listed in the contract (inputs do not reach them).
**Node** (`RppNode`): loads the artifact BY ID (the file the mission and spray load), conditions it, ticks only while the mission is RUNNING (STOP every tick otherwise; `STATE_LOADED` is the acknowledgement
the mission waits for), maps the decision to `MotionSetpoint` (`rpp_command`), publishes `RppStatus` incl. loop jitter, all 119 parameters as ROS parameters with their class (refused, never deferred).
11 in-process cases incl. **a whole mission driven to COMPLETE** by a stand-in vehicle (marks where the planner says, stops on the final point within 6 cm).
**Interfaces 0.8.0**: `RppStatus.STATE_LOADED` + `tick_state`, `segment_state`, `spray_request`, `handoff`, `rtk_reason` (appended). `rpp_node` is now in the control graph (`dyx3_bringup`).
DERIVED (all in the contracts): TRACK_HEADING for segment runs / TRACK_RATE for smooth runs; pivot rate `clamp(1.5 * err)`; `tick_hz` 50; `LoopTimer` overrun = period > 1.5 x target; the unported feature refuses to drive;
`curvature_baseline_m` and `max_yaw_rate_body` must be > 0 (the prototype allowed 0 = off); `run_sequencer` / `spray_gate` stubs removed (their logic is inside `RppCore`).
NOT run: anything on PX4 or a rover; loop timing / jitter; the firmware's reaction to the pivot and brake commands; the controller on recorded bag poses (the rover model is a stand-in).
Open (human): RTK_FLOAT acceptance (the prototype hard-codes minimum fix 6; the guard's `rtk_min_fix_type` is 6 too: change both or neither); whether `spray_request` should veto the valve (it is a heading verdict on
the CONDITIONED run, which fuses a short unpainted lead into the mark, so it can be true where the planner's flag is false); SCHED_FIFO priority / CPU affinity; the point hold / handshake / progress publication (port, drop,
or move to the mission layer); survivors of the mutation check that need scenarios I could not construct (run-boundary turns 45-62 degrees, precise-stop reverse after an overshoot, a corner released straight into tracking).
LOCAL ACTION: run the orchestrator test on real bag poses; `tools/gate4/gen_orchestrator_vectors.py --check` (Humble container); bench the pivot and brake commands (GATE 1).

## 2026-10-08 — Claude (cloud, branch `ccr-9531e512-2ndub7`) — review fix plan Phase A (A1, A2, A3) + frozen decisions

Plan: `docs/reviews/2026-10-08_cloud_review_verification_and_fix_plan.md`. **Section 5 of that file records the six human decisions frozen today** (A1 spray ownership, B1 reverse cap 0.10 m/s, B2 RPP sole profile owner, C2 heading gate moves into spray, E3 explicit `PLAINTEXT | TLS` per NTRIP profile, E4 plan location). Read it before B–G.
Docs: the production RTK plan is now in the repo, verbatim, at `docs/plans/2026-10-08_production_rtk_plan.md` (E4 input; header records the E3 security decision).

### A1 · C1 spray mission/RPP ownership — `dyx3_spray`
* New `ownership_status()` gate (pure, `spray_gates`), second after E-stop: mission RUNNING, `RppStatus` fresh (`rpp_timeout_s`), same `mission_id`, state TRACKING/STOPPING/PIVOTING/CREEPING. Reasons `mission not running`, `rpp stale`, `rpp mission mismatch`, `rpp not marking`. `GateInputs.ownership` defaults fail-closed.
* `tracking_seen_` is no longer a permanent latch: set only by TRACKING of the RUNNING mission, cleared on non-RUNNING, mission id change, path load. `note_rpp(state, mission_id, t)` and `set_mission(running, id)` replace the bool API.
* `safety_allows_on` (reassert path) also requires ownership; manual bench spray stays exempt.
* New parameter `rpp_timeout_s` 0.5 IDLE_ONLY: a "Production additions" section in `parameter_registry.md` (not part of the 173), generator expect 45 -> 46, tables regenerated.
* **DERIVED — NOT FROM V1 SPEC:** CREEPING is an allowed state (endpoint creep is still on the leg; geometry decides MARK). Equivalence tests set `ownership = ok` because the prototype has no such gate.
* Tests: gate unit test (every state, boundary age 0.5 vs 0.51, mismatch, defaults fail closed); 4 controller tests (pause/abort/complete/error -> OFF in one tick with ack; RPP ERROR/COMPLETE/LOADED/IDLE; RPP silent 0.48 s still ON, 0.52 s OFF; STOPPING keeps the mark and a resume needs fresh TRACKING; another mission's TRACKING is not evidence); 1 node test over 6 cases (PauseMission, AbortMission, COMPLETED, mission ERROR, RppStatus ERROR, RPP killed) each asserting OFF command, OFF ack, lease revoked.

### A2 · H4 READY acknowledgement, A3 · H3 action cancel — `dyx3_mission`
* READY is acknowledged only by LOADED/TRACKING/STOPPING/PIVOTING/CREEPING; ERROR is evaluated first (READY -> ERROR); COMPLETE in READY is ignored.
* Cancel still aborts in the callback; `on_timer` finalises the goal once it is CANCELING (result CANCELED, `RESULT_ABORTED`).
* Tests: the cancel test now also asserts the action result; new `RppErrorInReadyIsAnErrorNotAnAcknowledgement`, `RppCompleteInReadyIsNotAnAcknowledgement`.

### What I ran / could not run
* RAN (no ROS in this container): `dyx3_spray_core` built natively with g++ `-Wall -Wextra -Wpedantic -Werror` against googletest 1.14; `spray_core_test` 28/28, `spray_gates_test` 9/9, `spray_defect_test` 6/6, `spray_equivalence_test` 1/1 (all vector files). `tools/gen_param_tables.py --check` ok. clang-format 18 applied to changed files.
* NOT RUN: `spray_node_test`, `mission_node_test` and the mission node build (need ROS 2 / rclcpp_action) — **CI must prove them**; check the CI run of this push. Nothing ran on a Jetson or a rover.

### Next (waiting for the human's go — limited credit)
Phase B (B1 reverse cap 0.10, B2 remove guard accel/decel/jerk from the normal path, B3 code part), then D (D1 serialised spray ACKs: High, mandatory before field use), C, E, F per the plan. Not started.

## B1 — motion guard reverse cap 0.10 m/s (cloud session, branch `claude/cloud-phases`)

Applied the supplied `B1_reverse_guard_cap_0p10.patch` on 780bb29: `max_reverse_speed_mps` default 0.0 -> 0.10
(`limits.hpp`, `motion_guard_node.cpp`), the limits test now covers -0.08 passing and -0.5 clamped to -0.10, contract table updated.
B2 (accel/decel/jerk shaping) and B3 are NOT implemented. The patch needed `git apply --recount` (its test-hunk line counts were off; content unchanged).

Ran: `guard_core_test` compiled standalone with g++ -Wall -Wextra -Wpedantic against gtest 1.14 — 24/24 pass.
Could not run: `motion_guard_node_test` and the colcon workspace (no ROS 2 in this container); CI is the check for those.
DERIVED: 0.10 m/s is an initial bench value, not field-tuned; re-validate at GATE 1. The older HANDOFF line "reverse limit 0 (no source)" is superseded.
Note: with the accel/decel shaper still active (B2), a -0.08 brake from standstill is ramped by max_accel 0.20 m/s2, so the envelope is no longer the limiting factor.

## 2026-10-08 — Phase C1 implementation (local validation)

RPP now writes immutable content-addressed `DYX3COND 1` artifacts from the exact in-memory conditioned runs it installs, including source artifact SHA256, conditioner config, run profile/boundaries, points, spray flags, and must-hit flags. `RppStatus.conditioned_execution_sha256` (interfaces 0.9.0) advertises the current mission's artifact. Spray loads only that artifact and verifies both its content hash and source SHA against `MissionState`; its `PathModel` preserves run boundaries and cannot bridge runs. Failure to load or verify leaves the model cleared; there is no raw-path fallback.

Archived fixture comparison with shipped conditioner defaults: `square_2x2` max nearest same-kind boundary station delta raw vs conditioned = `2.13e-14 m`; `mission_straight_5m` = `0.10 m` (one matching boundary each). This is planned-artifact geometry, not replay/field evidence.

Checks available at the C1 commit boundary: parameter table generator OK (RPP 120, spray 46); backend tests 529 passed, 44 skipped; native conditioned-artifact / spray projection harness passed; mission artifact source compiled with `-Werror`; spray projection source compiled with `-Werror`; clang-format dry-run and `git diff --check` clean. `colcon`, ROS 2, and GoogleTest are unavailable on this Mac, so RPP/spray node tests, C++ gtest suites, full `colcon build/test`, and formal RPP conditioner/orchestrator equivalence suites were not run. C2 began after the available local C1 checks passed.

## C2 — spray sole heading and valve verdict owner (local validation)

Removed the RPP heading cut and entry hold from `RppCore`. `RppStatus.spray_request` remains only as deprecated planner diagnostic and is ignored by all spray actuator logic. RPP publishes `heading_error_rad`, `heading_evidence_valid`, `run_index`, and its exact `path_travel_m`; `dyx3_spray` now applies fresh-heading validation, the existing 30 degree cut through the immediate OFF gate, and entry-hold release from the same heading/progress evidence. MARK/TRANSIT still comes from C1's conditioned artifact. The existing STOPPING allowance, production PIVOT gate, pause/abort/RPP ownership protections, E-stop, RTK, watchdog, and actuator FSM remain active.

Moved `spray_heading_cut_deg`, `spray_entry_max_heading_deg`, and `spray_entry_release_travel_m` ownership from RPP to spray; defaults and mutability remain unchanged. Generated parameter counts are RPP 117 / spray 49. `segment_command_mode` remains `heading`. Bumped `dyx3_interfaces` to 0.10.0 for the appended heading evidence validity and path progress fields; `spray_request` was documented as deprecated.

Checks run: generator check OK (117 / 49); backend 529 passed, 44 skipped; native spray heading harness 5 assertions passed (MARK healthy, heading cut, invalid heading evidence, TRANSIT); RPP/spray core changed sources compile with `-Werror`; clang-format dry-run and `git diff --check` clean. Archived C1 station comparison remains `square_2x2` 2.13e-14 m / `mission_straight_5m` 0.10 m. Not run: RPP/spray C++ gtests (including node, gates, core, conditioner/orchestrator equivalence), Motion Guard gtests, or colcon build/test; this Mac has no ROS 2, `colcon`, or GoogleTest.

## 2026-10-08 — Phase C CI regression fix and D1 ACK integrity

Phase C test-only integration fixes are committed and pushed in `aefe7cf` and `7bef9de`.
`RppNode.PublishesFreshHeadingAndProgressEvidenceOnlyForActiveTracking` now follows RPP commands
and waits boundedly for active tracking; `SprayEquivalence.AllVectors` supplies healthy heading
evidence to isolate its legacy gate vectors, while dedicated tests continue to check fail-closed
heading evidence. Phase C CI run `37740472931` passed all jobs, including `colcon build` and
`colcon test` (386 tests, 0 failures, 2 skipped).

Inspected the exact production firmware tree at
`27a7ac92845317b0276776242c504215809b2a0f`. `VehicleCommand.source_component` and
`VehicleCommandAck.target_component` are both `uint16`; `Commander::answer_command` copies the
former into the latter (firmware `src/modules/commander/Commander.cpp:2674-2679`). Command 187
uses that helper for its ACCEPTED response (`:1495-1497`); command 183 reaches the same helper for
the UNSUPPORTED response (`:1544-1548`). This verifies source-component ACK correlation for both
IDs without modifying firmware.

D1 is implemented locally in `dyx3_px4_link`: only one spray `VehicleCommand` is sent at a time;
the queue is bounded, newest-wins per source, and watchdog OFF purges queued ON and takes priority.
Each dispatched request gets a source-component transaction token from 2 through 999 (1000+ is
reserved for PX4 mode executors). The next token is durably advanced before publish in
`/var/lib/dyx3/state/px4_link_spray_ack_next`; tokens never wrap. Missing/corrupt/unwritable state
or exhaustion refuses the request, including watchdog OFF, with a failed ACK. Preserve this state
across companion restarts and upgrades; the installer seeds it only for a first install. A missing
file on an installed rover stays fail closed. Reset it to 2 only after PX4 has also reset and prior
ACKs are gone. Final ACKs require command plus matching target-component;
timeouts and queue/link failures publish false acknowledgements, and late/unmatched ACKs are
discarded and counted. Added regression coverage for token matching, serialized/reassert-flood
behavior, watchdog OFF priority, OFF/OFF overlap, and a late ACK after timeout. Updated the PX4
link and spray contracts and clarified result 255 in the ack message comment; no wire fields
changed and no interface version bump is required.

Checks run for D1: `clang-format --dry-run --Werror` and `git diff --check` pass. Not run: D1
`px4_link_node_test`, other ROS gtests, or colcon build/test; ROS 2 and `colcon` are unavailable
on this Mac. D1 has not been pushed and has no CI result yet.

### D1 token-lifetime audit and hardening — local

At exact PX4 SHA `27a7ac92845317b0276776242c504215809b2a0f`, IDs >=1000 are mode-executor identities
(`Commander::getSourceFromCommand`, `Commander::handleCommandsFromModeExecutors`); MAVLink's
`MavlinkCommandSender` suppresses normal command forwarding for those IDs and `mavlink_main.cpp`
suppresses ACK forwarding for targets >=1000. The actuator command itself ignores source identity,
but using that range would change component ownership. `source_system` is copied to ACK target
system and is used by `Mavlink::handleAndGetCurrentCommandAck` for target routing, so varying it is
not a safe independent ID axis. Follow-up commit `ffe623e` uses each component ID 2..999 once,
advances the durable high-water mark before publishing, continues after process restart, and
refuses on exhaustion or missing/corrupt/unwritable state. Fresh install seeds the state; an
installed rover with a missing ledger stays refused. Four allocator tests cover the range boundary,
oldest-token non-reuse, process restart, and corrupt state. A px4-link node test verifies exhausted
identity cannot produce a successful watchdog OFF ACK. Native standalone allocator smoke assertions
passed; node/gtest/colcon execution is unavailable locally. Operational limit: only 998 spray
transactions per ledger lifetime; after exhaustion, reinitialize only after PX4 and companion are
reset together and old ACKs are gone.

### D2 — watchdog OFF proof bound to mapping — commit `3ca04dc`

Watchdog proof identity is `(backend, actuator_set_index, off_value, servo_instance, off_pwm_us)`.
An identical update keeps proof; a changed mapping clears it immediately, cancels the matching
in-flight proof sequence, withholds ON authority, and requires an OFF ACK for the new mapping.
Stale A ACKs cannot satisfy B. Added core tests for A-to-B transition, map change during A OFF,
each mapping field, identical update, and shutdown fail-closed behavior. Standalone watchdog
mapping smoke test, `-Werror` source compilation, clang-format dry-run, and `git diff --check` pass;
ROS/gtest is unavailable locally.

### D3 — require fresh RTK corrections in spray — commit `03763c6`

`RtkSnapshot` now carries `RtkStatus.corrections_fresh`; the spray `RtkGate` fails immediately and
resets its recovery timer when false, even if the RTK fix gate parameter is disabled. Existing
configured FIXED/FLOAT minimum and horizontal-accuracy rules and RTK status timeout remain intact.
RPP/mission/geometry/heading checks remain separate subsequent gates. Motion Guard already consumes
`corrections_fresh` in its pure RTK predicate, but sharing that predicate would couple spray's
configurable accuracy/recovery behavior to Motion Guard's fixed rule; both now require the same
correction-freshness evidence without changing either policy. Added gate and controller tests plus a
spray-node test for FIXED + good accuracy + stale corrections. Existing vector equivalence feeds
fresh corrections explicitly so it continues to compare the prior gate dimensions; dedicated tests
cover stale and missing correction evidence. Changed core sources compile with `-Werror`; an eight-
assertion standalone RTK-gate smoke test passed; parameter check passes (RPP 117 / spray 49);
backend suite passes 529 with 44 skipped; clang-format dry-run and full branch `git diff --check`
pass. ROS 2, colcon, and GoogleTest are unavailable, so package gtests and full colcon build/test
were not run.

### D1 logical-request reassert correction — local follow-up

At branch HEAD `63ec3d3`, inspection confirmed every spray publish was enqueued/dispatched as a
new request, so the spray node's same-sequence 2 Hz ON reassert could consume another durable
`source_component` token per dispatch. The px4-link correction keys an exact request by producer
source, sequence, ON/OFF, backend, actuator/servo mapping, physical value/PWM, and translated PX4
command/parameters. Exact duplicates are coalesced while pending/in flight; after an ACCEPTED PX4
ACK, the same request is suppressed until that source changes its request. A distinct request or a
retry after an unconfirmed failure gets a new never-reused token. Companion restart clears only the
in-memory duplicate cache; the durable high-water mark continues. Therefore 10,000 repeated ON
heartbeats after the original accepted request consume zero additional tokens (one token total for
that original request).

`source_system` expansion remains excluded. Rechecked exact production PX4 SHA
`27a7ac92845317b0276776242c504215809b2a0f`: Commander echoes source system/component into ACK
target system/component; the MAVLink ACK router uses target system for routing. Component IDs
1000+ retain mode-executor semantics. There is no companion-visible verified reset generation, so
the 2..999 IDs remain finite and fail closed at exhaustion; only a coordinated PX4 + companion
reset with the old ACK stream gone permits operator reinitialization.

Remaining capacity risk: `WatchdogCore` increments its sequence for each OFF retry. Those retries
are distinct proof transactions and therefore still consume one token each; a prolonged watchdog
retry condition can exhaust the 998-token pool. The link does not reuse a prior OFF token because
the older ACK must not be treated as proof of a newer sequence. This needs a separate design
decision if prolonged watchdog retry availability must be guaranteed.

Added px4-link node tests for 10,000 acknowledged same-request reasserts, new ON→OFF identity and
late-ACK rejection, actuator mapping change, and repeated watchdog OFF suppression. Local
`clang-format --dry-run --Werror`, `git diff --check`, and generated parameter check pass. Backend
suite passes 529 with 44 skipped. ROS 2 and `colcon` are not installed, so the px4-link node gtests,
remaining D1/D2/D3 ROS gtests, and colcon build/test were not run. The exact firmware tree was at
the required SHA but contains pre-existing modifications; it was read-only during this work.

### Phase D — nonreused pair identities and physical reasserts (2026-10-08)

This section supersedes the D1 capacity/reassert statements above. The pinned firmware full-tree
command audit is recorded in `docs/contracts/dyx3_px4_link.md` §14. The physical 183/187 execution
paths do not use `source_system`; command 187's actuator-set consumer reads only command and
parameters, while 183 is unsupported. Spray ACKs now match the nonreused ordered pair
`(source_system, source_component)` and command ID. Systems 1..255 and components 2..999 yield
254,490 logical identities per controlled PX4/companion correlation epoch. The durable allocator
migrates the old numeric high-water mark without reusing a pair. It is never automatically reset;
a companion restart or upgrade preserves it. Reset requires a controlled joint PX4/companion reset
with the previous DDS/uXRCE ACK stream gone.

Controller ON and watchdog OFF physical reasserts continue to reach PX4 while retaining their
logical pair. Watchdog uses one sequence through a continuous OFF-required period; a fresh OFF
period after ON became allowed, or a changed physical mapping, gets a new sequence. ACKs for
identical retransmissions may prove that same epoch; old pairs cannot prove newer epochs. The
allocator and unmatched-ACK count are exposed on `Px4LinkStatus` (interface 0.11.0). There is no
ACK-completed token recycling. The remaining capacity risk is exhaustion of 254,490 distinct
logical epochs before a proven joint reset; exhaustion fails closed.
Cloud Review E1 local: interface 0.12.0 separates NTRIP source bytes, CRC-valid frames, RTK publication handoff, and px4_link accepted/refused chunk counters. The latter stop at `GpsInjectData` publication; receiver health remains separate. Phase D CI 37749513456 is red on the spray ACK test; do not push E1 until Phase D is resolved.
Cloud Review E2 local: NTRIP GGA send-all is nonblocking, cancellation-aware, and bounded by the existing stream timeout. The worker owns final socket close; stop only wakes and shuts down the current fd under the ownership mutex, then joins. No socket integer is used without the lifetime lock for shutdown/close.
Cloud Review E3 local: existing `ntrip.env` installations require explicit `DYX3_NTRIP_SECURITY=PLAINTEXT|TLS` migration. TLS verifies system or configured CA trust and host identity, with no downgrade; plaintext credentials trigger a fixed warning and status bit. Interface 0.13.0 appends security status fields. The full E4 profile/backend architecture remains unimplemented.

### Cloud Review Phase F implementation authorization (2026-10-08)

The human explicitly assigned Phase F implementation, local tests, and local commits to Codex.
Claude retains the independent safety-critical review and push decision. Codex must not push.
F1 makes `.complete` follow successful build and static verification; runtime health failure
marks the candidate `.failed` and removes its completion marker. A failed first install stops
and disables its enabled services and clears `current`; a failed upgrade restores units,
shims, version provenance, and services for the prior release. Staged-root suite: 80/80 on
macOS with GNU Bash/coreutils/findutils; no Jetson validation claimed.

F2 applies the four mission IDLE_ONLY values in one validated callback batch and replaces the
state timer when its rate changes. Requests outside IDLE or with invalid values are refused.
The production entry point uses `rclcpp::spin` (single-threaded); Humble has no post-set
parameter callback. Mission node tests cover atomic batches, timer frequency, effective gate
freshness, invalid values, and active-mission rejection, but ROS 2/colcon is unavailable on
this Mac, so these tests require Claude's independent CI review.

F3 rejects every MotionGuard runtime parameter update because the node reads all 15 declared
values at construction. The hard speed and yaw envelopes retain their current values. The
contract and production-additions registry now classify these values as RESTART; future LIVE
limits require an atomic validated runtime path. A node regression test covers refusal,
unchanged hard limits, ordinary pass-through, and RTK gate failure to STOP. ROS node tests
remain unavailable locally.

F4 records `last_overrun_s_` separately from `last_step_s_`, so the one-second diagnostic
warning expires after the last actual late tick while the lifetime overrun counter remains.
Nonfinite/backward injected times are ignored. A fake-FCU node test covers normal ticks,
recent and repeated overruns, recovery, counter retention, and invalid time. ROS/colcon remains
unavailable locally; the existing Phase D ACK tests are unchanged and await CI execution.

F5 corrected only the ROS launcher comment after checking the launch graph's six nodes.
The legacy 173-count is corrected in this handoff's old action list; the registry now
distinguishes that legacy total from the generated current 117 RPP / 49 spray tables and
two production-only additions. A proposal requests the corresponding human-owned
`CLAUDE.md` corrections; that file and the frozen architecture were not edited.

F6 remains open. The archived `seg_boundary_55deg` fixture has one conditioned run, so it
cannot exercise a run boundary; `seg_overshoot` does not cross the end plane; and the
existing loose-corner and blackout/jump episodes do not reach the two remaining mutation
sites. The generator's approved carried-node workflow failed at `rclpy.parameter` on this
Mac (`python3 tools/gate4/gen_orchestrator_vectors.py --check`); ROS 2 Humble, colcon,
and a container runtime are absent. The existing pure C++ orchestrator equivalence test
was built natively with GoogleTest and passed all 97 archived scenarios / 13,166 ticks
with zero mismatches. No new expected vectors or mutation-failure claims were fabricated.
The four scenarios require reference generation and baseline/mutant verification in the
Humble environment before F6 can be committed.

### Local ROS 2 Humble environment + Phase F verification (2026-10-08, claude)

Added a persistent local environment: Colima profile `dyx3-ros2` (aarch64, vz), image
`dyx3-ros2-humble:local` from `ros:humble-ros-base`, firmware-pinned px4_msgs overlay built by the repo's
own `build_px4_msgs` into a persistent volume. **ROS 2 IS available on the Mac via
`./tools/dev/ros2_humble.sh build-test`** — see `docs/agents/LOCAL_ROS2_BUILD_ENV.md`. Do not report ROS
as unavailable. `CLAUDE.md` §9 got a one-paragraph pointer (human-authorized); `AGENTS.md` created.

Verified on Phase F HEAD `8ed46be` + F4 follow-up: 12 packages built; `colcon test-result`: **437 tests,
0 errors, 0 failures, 2 skipped** (CI baseline before Phase F: 432). The F2 (mission), F3 (MotionGuard)
and F4 (px4_link) node tests ran and passed, as did spray/watchdog, gnss_rtk, RPP, interfaces. Repeated
after a Colima stop/start with no reinstall (overlay "already built", incremental build 1.5 s).
One compiler warning (`dyx3_system_gateway/src/json.cpp:252` missing-field-initializers), pre-existing.

F4 follow-up (separate commit): the Phase F early return in `Px4LinkNode::step()` on a non-finite or
backward clock now publishes an explicit `stop_setpoint()` instead of omitting the tick; node test extended
to assert one STOP per bad tick. Review findings F2 "stricter than active" (state != IDLE) and the
`use_sim_time` rejection in MotionGuard remain documented, unchanged.

**F6 remains OPEN** and is an acceptance item: the carried-reference generator
(`tools/gate4/gen_orchestrator_vectors.py`) has not been retried in this environment; the four scenarios
still have no reference outputs or baseline-pass / mutation-fail evidence. Not pushed.

## 2026-10-08 — Codex — Phase F closure (isolated branch)

Started at `8ed46bea153aafa1fb6832c4b6fa635ec0771eb1` in the main repository. All
corrections are in `/Users/dyx_a1/Vetri/3WD_PROD/DYX_3WD_phase_f_closure` on
`codex/phase-f-closure`; no push, merge or cherry-pick was performed. The two original
untracked patch files in the main repository were left untouched.

* F4 `f937e1a`: an invalid injected link clock previously returned before publishing the
  explicit setpoint. It now publishes the full STOP control set with the independent system
  clock timestamp, without advancing the valid-time overrun baseline. The fake-FCU test covers
  NaN, infinity, backward time and normal forwarding recovery. The overrun recovery, watchdog
  priority and ACK correlation tests passed in the same ROS node suite.
* F2 `88b642a`: the FSM has no terminal-to-IDLE event; new start goes directly to LOADING.
  IDLE_ONLY therefore remains strict in PAUSED and all terminal states. The contract describes
  the operator route: stop/disarm, restart the mission node to IDLE, configure, then start a new
  mission. Node tests cover atomic rejection in PAUSED/COMPLETED/ABORTED/ERROR and acceptance
  after restart. The existing timer-frequency and atomicity tests passed.
* F6: the carried Python controller generated four additional reference episodes. In the
  isolated equivalence replay, each baseline had zero mismatches; each corresponding temporary
  RPP mutation failed; production source was restored and each baseline passed again. The
  55-degree case requires the supported `auto` profile to create two conditioned runs: forced
  `segment` intentionally keeps a single run. Exact mutations/ticks are in
  `docs/contracts/rpp_orchestrator.md`. The full equivalence corpus is now 101 scenarios,
  13,905 ticks, zero mismatches. No production RPP code was changed.

**Verification on this worktree:** ROS Humble arm64 build 12/12 packages; full colcon test
440 tests, 0 errors/failures, 2 skipped; installer staged-root 80 passed; backend pytest
529 passed, 44 skipped; carried-reference generator `--check`; ShellCheck on Phase F installer
and launcher scripts; clang-format on changed C++ files; parameter generator check (117 RPP,
49 spray); `git diff --check`. ROS used the firmware-pinned `px4_msgs` overlay and an isolated
`dyx3-codex-phase-f-ws` volume; it did not build Claude's working tree.

**DERIVED — NOT FROM V1 SPEC:** restarting the mission node while the vehicle is stopped and
disarmed is the operational return to IDLE for runtime reconfiguration; no reset event was
invented. The four synthetic RPP episode inputs are selected to reach the recorded branch,
while all expected outputs come from the carried controller.

**Not run:** hardware/Jetson timing, PX4 transport, physical valve closure, real-bag replay,
or field RTK validation. The full ROS suite skipped the bag cross-track fixture test (no bag
fixture) and the whole pinned message-hash set test. Other orchestrator mutation survivors
listed in the contract remain
open outside the four Phase F scenarios. Claude should independently review and integrate the
local commits, then decide on the final push.

### Phase F integration review (2026-10-08, claude) — NOT PUSHED

Integrated codex 88b642a (F2 terminal-state tests/docs) and c1f9f63 (F6) by cherry-pick; codex f937e1a
(F4) applied as a test+contract-only commit because 4b0e669 already has the identical production line.
Integrated HEAD: 12/12 packages, 440 tests, 0 failures, 2 skipped (local Humble env, firmware overlay).

F6 independent replay (scratch copy, repo untouched; `DYX3_ORCH_SCENARIO=<name>`; generator `--check` is
byte-identical so fixtures do come from the carried controller; original episodes unchanged):
- seg_start_beyond_end: baseline 0 mismatches; "always +1 sign" mutant fails (rpp_core.cpp ~707/714) — REPRODUCED.
- seg_corner_direct_release: omit `zero_speed_memory` reset (rpp_core.cpp:1159) fails at tick 49 — REPRODUCED.
- seg_boundary_55deg_sharp: +15 deg on the run-alignment threshold at rpp_core.cpp:115 (and :461) fails
  (first mismatch tick 123 here vs 113 in the contract; the path_conditioner.cpp:484 threshold is NOT killed).
- fault_vel_blackout_jump: NOT REPRODUCED. 0.3->0.5 (also 0.05, 0.31, 0.32, 100 s) in `vel_is_fresh`
  (rpp_core.cpp:189), removing the freshness check on `v_meas` (:768), and the IMU-extrapolation
  age (:791) all give 0 mismatches. Contract claim "fails at tick 28" needs the exact mutation from codex.
Until that is resolved the contract row for fault_vel_blackout_jump should not be treated as closed.

### Phase F integrated, pushed and CI green; Cloud Review status record (2026-10-08, claude)

Phase F is on `claude/cloud-phases` (push `6b8b4b1..9549679`). **GitHub Actions run `37766642047`, commit
`9549679`: all 9 jobs succeeded**, including ROS 2 Humble `colcon build + test`: 12 packages, **440 tests,
0 errors, 0 failures, 2 skipped**. Supersedes the "NOT PUSHED" headings of the two entries above (kept as
history). F4: one production implementation (`4b0e669`) plus Codex's stronger test (`7795e83`). F2 and F6 from
Codex (`c3dc01b`, `47796ad`). F6 result: **three mutation-closed scenarios, one baseline-only**
(`fault_vel_blackout_jump` not reproduced; the velocity-freshness constant is a survivor again).

Correction: the D3 entry above cites `03763c6`; that is a pre-rebase SHA no longer in branch history — the
commit is `63ec3d3`.

**Read `docs/agents/CLOUD_REVIEW_STATUS.md` before implementing any fix.** It lists every Cloud Review finding with
commit, verification, limits, contract and what it blocks. Distinguish: already fixed, known but deferred,
unverified hardware behaviour, newly discovered regression. Open and blocking paint: **C3** (no independent
valve-close path), H12 gate default, H13 debounce lead, GATE 1 bench, Jetson timing, field validation.
Local ROS 2: `./tools/dev/ros2_humble.sh build-test`. A proposal for the human-owned `CLAUDE.md` status text is in
`docs/architecture/proposals/2026-10-08_claude-status-correction.md`. Documentation-only change; no code touched.

---

## 2026-10-08 — Claude — first rover bring-up: FCU params, firmware fix, installer, deploy, RTCM path

Deploy branch `claude/cloud-phases`; this session's commits: `0ab0402`, `a186f54`, `f7f5a07`, `eed1946`,
`42c065b`, `4bb213a`, `3ecd5c5`, `19d3566`, `2edc844`, `84518cd` (+ this docs commit). Firmware repo: `9ab2ad3162`.
**Not merged to `master`.** Hardware: Holybro Pixhawk Jetson Baseboard (Pixhawk 6X + Orin Nano), office LAN.

**FCU (Pixhawk 6X)**
- Prototype Cube Orange+ params (`config/px4/proto_ref_01.params`, PX4 v1.16.2) carried to the 6X:
  `config/px4/3wd_6x_carry_from_proto.params` (only values differing from the prod defaults; no calibration;
  excluded prototype hacks CBRK_SUPPLY_CHK, COM_ARM_SDCARD 0, COM_OF_LOSS_T/COM_OBC_LOSS_T 30, UAVCAN, UXRCE_DDS_CFG 0).
  Backup of the 6X before: `config/px4/6x_before_carry.params`. Ports as on the prototype: UM982 TELEM1 230400
  (MAV_0_CONFIG 0), RoboClaw GPS2 115200, spray PWM_AUX_FUNC1 301 (FMU PWM OUT 1).
- Human inputs: 8S LiFePO4 (BAT1_N_CELLS 8, V_EMPTY 2.75, V_CHARGED 3.40), 24 Ah; IMU now 55 mm **behind** the
  antenna → EKF2_IMU_POS_X -0.055 (antenna X 0, Z -0.4 unchanged); track 0.47, wheel radius 0.1498, yaw offset 180.
- Live params added today: UXRCE_DDS_DOM_ID 42, **UXRCE_DDS_PTCFG 1**, MAV_2_CONFIG 1000 (QGC over Ethernet).
- **Firmware `9ab2ad3162` fix(roboclaw)**: Run() called the mixer before the UART was open → select() on fd 0 with an
  uninitialised timeout → wq:hp_default hung forever (PWM out, battery dead). Reproduced on an unconnected port.
  Fixed: fd -1, 11 ms timeout at construction, UART before mixer, own serial work queue, 1 s handshake retry,
  no err(), 115200 fallback. CI-built, archived, flashed; RoboClaw connects, wheel_encoders publish.

**Jetson / installer** (first real run of the installer)
- Measured: on-Jetson install 40 min 31 s (px4_msgs 16 min 49 s at -j2, stack 22 min 48 s, packages one at a time).
  → **Prebuilt artifacts** (`42c065b`, proposal ACCEPTED: master publishes, SHA-256 now, keep 20): CI job
  `rover_artifacts` + `rover_publish`, installer `lib/artifacts.sh`. Upgrade now **20–22 s**, zero compilers;
  CI and rover px4_msgs message-set hash identical (`8ae57326…`), same rclcpp build.
- Bugs fixed: bench FCU profile dropped the default route (`0ab0402`); router UDP Server on 10.41.10.1 never sees
  PX4's broadcast (`a186f54`, see OPEN below); launchers lacked `bin/dyx3-env.sh` + rcl could not create its log dir
  under the root-owned HOME (`4bb213a`); px4_link stale limits below the measured 1.010 s timesync /
  estimator-flags period → session flap every second (`3ecd5c5`, `2edc844`); health check ran before restarted
  services were up and probed the wrong backend address (`84518cd`).
- All six services verified, then enabled in the manifest (`19d3566`). **Power-cycle test passed**: all six up,
  0 restarts, DDS 68 topics / odometry 100 Hz, 0 session flaps, NTRIP reconnects once DNS is up, backend 200.

**RTCM path (indoors)**: Emlid caster (profile "office", credentials only in `/etc/dyx3/ntrip.env`) → gnss_rtk →
`/dyx3/rtcm` 6.36 msg/s → px4_link → `/fmu/in/gps_inject_data` 6.35 msg/s → PX4 "rate RTCM injection 5.77 Hz",
rtcm_crc_failed False. fix_type 0 / 3 sats = indoors. Earlier the base was offline (mountpoint absent from the
sourcetable, caster sends 0 bytes) — our client detects that as a stream timeout and reconnects.

**Rover-local state (not in git, deliberate)**: `/etc/dyx3/ros.env` (domain 42, localhost-only),
`/etc/dyx3/ntrip.env`, `/etc/dyx3/backend.env` **bench bind 0.0.0.0** (no hotspot 10.42.0.1 yet),
`/etc/dyx3/mavlink-router.conf` **hand-set to `Mode=Server Address=0.0.0.0 Port=14550`** (UNVERIFIED, see OPEN).
Legacy removed: old `mavlink-router.service` disabled, NM `eth-baseboard` autoconnect off. Still to delete:
`~flash/dyx3_deps_stage*.sh`, `~/px4_ws`, `~/dyx3_venv` (ad-hoc 2026-10-07 setup).

**OPEN — for the next session**
1. **PX4 Ethernet TX stall (safety-relevant).** Twice PX4 answered ping but sent nothing it originates (no DDS, no
   MAVLink, no ARP; MAVLink still counted 2 KB/s "tx"), recovered only by an FCU reboot. Both times right after the
   XRCE agent restarted (dyx3-platform restart / repeated restarts). A ping *from* PX4 never left the board. The first
   time 5 `netinit` monitor threads existed (one per MAVLink-shell NSH session) and USB was unplugged mid-shell; the
   second time neither applied. Hypothesis: STM32H7 Ethernet TX path wedges on the reconnect burst (upstream
   #26160-like). Next: with USB attached, restart the agent in a loop while sniffing; repeat with MAV_2_CONFIG 0;
   inspect NuttX net/driver state when stalled. Fix in firmware and/or add a PX4-side TX watchdog.
2. **mavlink-router mode is unresolved.** Client mode (template `a186f54`) works until the router restarts: PX4 keeps
   its old ephemeral partner port → QGC dead until FCU reboot. Server on 0.0.0.0:14550 (rover, hand-set) should
   survive restarts via PX4's broadcast but was not verified (the TX stall hit during the test). Verify, then fix the
   template; TCP 5760 stays the GCS path.
3. Calibration (gyro, simple accel `PREFLIGHT_CALIBRATION` param5=4, level horizon) — rover is too big for 6-side.
4. Motion test (wheels up), outdoor RTK fix, then the pending list (merge → master, timing proposal, A/B cleanup,
   RTK USB-primary/DDS-fallback, hotspot profile + restore backend bind).
5. Installer test "dry-run mentions: useradd" is not hermetic (fails where the dyx3 user exists).

**DERIVED — NOT FROM V1 SPEC**: stale limits 3.0 s for timesync/estimator flags (three measured periods);
health settle 30 s; artifact layout/naming; bench backend bind 0.0.0.0; DDS localhost-only via PTCFG 1.
**Not run**: motion, calibration, spray valve, outdoor RTK, timing under load, fresh-rover install from artifacts
(this rover reused its px4_msgs; the download path is covered by staged tests only).

---

## 2026-10-08 — Codex — production RTK source, transport and backend control

Branch `codex/rtk-production` starts at the deployed `fbc7165` tree. No rover deploy, push or merge was performed.
The existing NTRIP → PX4_DDS path remains available. The worker now supports NTRIP or LoRa as source and USB_DIRECT
or PX4_DDS as the single selected transport, with no automatic failover. The source/transport switch closes the old
sink before activating the new one and rejects frames from earlier source generations. USB writes accept only complete
CRC-valid RTCM3 frames; receiver GGA is observed without sending receiver configuration commands.

The first runtime config is stored under `/var/lib/dyx3/rtk/` (`dyx3:dyx3 0700`, config file `0600`) using a
same-directory temporary file, file fsync, rename and directory fsync. An existing read-only
`/etc/dyx3/ntrip.env` (`root:dyx3 0640`) is imported once when no runtime config exists, retaining PX4_DDS on
the deployed rover. A fresh install selects NTRIP + USB_DIRECT, but leaves unknown serial paths, bauds and timing
unset. The service's `ReadWritePaths` is unchanged. The backend offers authenticated RTK REST controls through the
worker's versioned local socket; passwords are write-only. The backend shares Unix user `dyx3` with the worker, so
filesystem permissions alone do not isolate those secrets (see `docs/contracts/dyx3_rtk.md`).

Off-target verification: the RTK package tests, full ROS 2 container build/test, backend pytest, staged installer
tests, Ruff, ShellCheck and formatting checks passed in this session. Serial tests cover pty unplug/replug,
single-sink authority and interrupted config save. The interface extension is proposed in
`docs/architecture/proposals/2026-10-09_rtk_interfaces.md`; no `dyx3_interfaces` or `dyx3_px4_link` files changed.

**Bench answers still needed before using USB or LoRa:** which UM982 COM is connected to USB versus PX4 TELEM1
(must be different COMs), USB baud, whether that COM emits GGA, and the LoRa radio model and baud. Off-target tests
do not establish receiver acceptance, RTK FIX, correction latency or field accuracy.

---

## 2026-10-09 — Claude — release prep for the bench, and hand fixes the repo does not carry yet

**Deploy branch `claude/cloud-phases`:** Codex RTK commits, plus `f18d008` (an invalid RTK config or seed starts
STOPPED with reason CONFIG_INVALID instead of crash-looping) and `cb8ea42` (`StateDirectory=dyx3/rtk`). An upgrade
runs the *installed* release's installer scripts but the *new* unit files. Without that unit setting, upgrading
from `84518cd` would leave `/var/lib/dyx3/rtk` missing, and NTRIP → DDS would stop.
**Firmware `dyx-3wd-production` @ 4393fb07e1:**
- NuttX TX-ring guard; NuttX now comes from the fork `Vetri2425/NuttX`;
- XRCE fd closed once;
- GPS partial RTCM writes completed and counted.
CI is green and the build is archived. Not flashed. Bench runbook: `3WD_PROD/BENCH_2026-10-09.md`.

**New rule (CLAUDE.md §4): every hardware fix persists in the repo.** These rover-local hand edits from 2026-10-08
are NOT yet reproducible on a fresh rover. Each one is a bug until the repo carries it:
1. `/etc/dyx3/ros.env` has `ROS_DOMAIN_ID=42` and `ROS_LOCALHOST_ONLY=1` by hand. The template leaves
   `ROS_DOMAIN_ID` commented out, so a fresh rover gets domain 0, while PX4 `UXRCE_DDS_DOM_ID` is 42 and `/fmu` is
   invisible. Fix: the template default, plus a health check that the domain matches the PX4 parameter baseline.
2. `/etc/dyx3/backend.env` has `DYX3_BACKEND_HOST=0.0.0.0` (bench, no hotspot). Fix: the hotspot profile
   (Codex task C), then back to `10.42.0.1`.
3. `/etc/dyx3/mavlink-router.conf` was hand-edited: first to client mode, then server `0.0.0.0:14550` (unverified).
   Fix: bench re-test, then the template in `deployment/network/mavlink-router.conf.tmpl`.
4. PX4 parameters (`UXRCE_DDS_DOM_ID 42`, `UXRCE_DDS_PTCFG 1`, `MAV_2_CONFIG 1000`, battery, `EKF2_IMU_POS_X`) are
   recorded in `config/px4/3wd_6x_carry_from_proto.params`, but nothing applies them. Needs a documented or
   scripted apply step for the next rover.
5. NTRIP credentials in `/etc/dyx3/ntrip.env`: correctly kept off Git. The creation procedure must be in
   installer/README (template exists; the step does not).

---

## 2026-10-09 (night) — Claude — session close: what is ready for the bench, and what is open

**Single branches (human rule).** One authoritative branch per repo. Agent work merges only after Claude's review.
Branches are never deleted unless the human asks.

| Repo | Authoritative branch | State |
|---|---|---|
| DYX_3WD | `master` = `ad58e72` | Fast-forwarded from `claude/cloud-phases` `a1f7cd4`, plus merge of `claude/px4link-flaky-tests` (`545c05d`). CI publishes `rover-<sha>` from `master` (and `claude/cloud-phases`) |
| PX4-Autopilot-3WD-Prod | `dyx-3wd-production` = `8279fa4be3` | **V1 final candidate.** No firmware work after the bench, unless the bench finds a defect |
| NuttX (new fork `Vetri2425/NuttX`) | `dyx-3wd-production` = `e462af8eb3` | PX4's pinned `fb2fadf6` + one backport commit |
| Three_Wheel_v2 (app) | `main` | Nothing merged yet. Agy branches: `agy/prod-contract-plan` → `agy/debug-client` → `agy/prod-transport` |

**Firmware since `9ab2ad3162`.** Each commit was pushed alone, CI is green, and every build is archived in
`3WD_PROD/PX4-Firmware/3WD/`:
- `58154ff8f1`: NuttX STM32H7 RX replies never transmit into a full TX ring. Backport of Apache NuttX
  `be559984e549` / PR #19930, the suspected cause of the 2026-10-08 Ethernet stall. +24 B.
- `f991ebb98a`: the XRCE client closes its UDP fd once on deinit; the transport owns it.
- `4393fb07e1`: GPS RTCM injection:
  - runs every UART pass;
  - partial or EAGAIN writes are finished from their offset, never restarted;
  - a fragment pending more than 500 ms (DERIVED) is dropped and counted;
  - new `gps status` counters.
- `8279fa4be3`: wheel-encoder fusion no longer refreshes `_time_last_hor_vel_fuse` / `_ver_`. Before this, it
  blocked the EKF-GSF yaw emergency reset (`gps_control.cpp` `isTimedOut(_time_last_hor_vel_fuse)`) and hid inertial
  dead reckoning while GNSS velocity was rejected. The rover runs `EKF2_WENC_CTRL 1`. sha256 `21288e6d…`,
  FLASH 1,789,852 B (+600 B vs `9ab2ad3162`), AXI unchanged, `msg/` unchanged.
- Verification:
  - CI builds of all four;
  - local `px4_sitl_default` compile;
  - `make tests TESTFILTER=EKF`: 24/26 pass. `EKF_accelerometer` and `EKF_flow` fail **identically without the
    WENC fix**, so the failures pre-exist; WENC is compiled only on `fmu-v6x/rover`, so the SITL tests do not reach it.
- **Not done:** the GATE-2 replay of the WENC change; the pty RTCM byte test. **The bench decides acceptance.**
- The `.gitmodules` NuttX URL now points to the fork. The main firmware checkout
  `~/Vetri/3WD_PROD/PX4-Autopilot-3WD-Prod` still holds the original uncommitted WENC edit (identical to
  `8279fa4be3`). Its local `dyx-3wd-production` is 4 behind. All work was done in the worktree
  `~/Vetri/3WD_PROD/PX4-3WD-ethfix`, branch `fw/eth-stall-rtcm`.

**Companion, reviewed and merged into `master` this session:**
- Codex RTK (`07d541c`, `5dae379`, `0720549`): LoRa source; USB sink that accepts only validated RTCM3 frames; single
  transport authority with no failover; persistent config and secrets in `/var/lib/dyx3/rtk/` (0700/0600); control
  socket; REST.
- `f18d008`: an invalid config or seed starts STOPPED / ERROR `CONFIG_INVALID` with the control socket up, instead
  of crash-looping.
- `cb8ea42`: `StateDirectory=dyx3/rtk` in the unit, because an upgrade runs the OLD release's installer scripts with
  the NEW unit files.
- `545c05d`: px4_link spray-watchdog tests wait for delivery. CI failed once on a commit that did not touch
  px4_link. Not reproduced locally even on one loaded core, so the timing assumption was removed rather than a
  race fixed. No assertion was weakened.
- `a1f7cd4`: CLAUDE.md §4 rule "every hardware fix persists in the repo" and the list of five rover hand edits
  (previous HANDOFF entry).

**Companion, done by Codex but NOT yet reviewed or merged:** `codex/rover-app-gaps`, rebased on `master`.
- Reviewed by Claude:
  - `945979d` `POST /api/missions/plan`: rover contract `docs/contracts/app_planned_mission.md` v1.1. Rules
    R1 one spray state per run, R2 strict mark/travel alternation, R3 contiguous runs within 1 mm, R4 shared-point
    encoding (spray bit from the earlier run, must-hit ORed). The path engine is never imported; RPP and mission
    code are unchanged. A C++ boundary test runs on a backend-generated fixture.
  - `99554ea`: parse-only `POST /api/path/parse-dxf`. The CSV parse endpoints have no UI caller and were not added.
  - `f3d4c8e`: optional isolated hotspot at 10.42.0.1, configured from `/etc/dyx3/hotspot.env`.
  - `b129b6a`: tablet token command:
    `sudo -u dyx3 /opt/dyx3/current/venv/bin/python -m dyx3_backend.auth.tokens create --file /var/lib/dyx3/state/auth.json --name tablet-1 --role operator`,
    then restart `dyx3-backend`.
- Not yet reviewed: `5955163` Socket.IO ping 5 s / 5 s (DERIVED), `0135dee` Wi-Fi country IN, power save off,
  non-DFS band/channel, `0edbc44` body-size limit on `/missions/plan`.
- Codex reports: backend pytest 564 passed / 43 skipped, installer 97/0, ROS 459 / 0 failures.
- **To do:** review E–G, run the full checks, merge into `master`, CI release.

**App (Agy):**
- `docs/PROD_CONTRACT_UPGRADE_PLAN.md`: 77 app interactions mapped; 13 rover gaps found. GAP-04 and the parse
  endpoint are now built by Codex.
- `agy/debug-client`: production client layer (prod REST/Socket.IO, staleness 1.0 s / 2.5 s, heartbeat, E-stop
  confirm-to-clear, mission builder, discovery).
- `agy/prod-transport`: single transport, honest disconnect, 401 stops retry, AppState; heartbeat fixed-rate with
  a 350 ms cap (the old version computed the next delay before awaiting, so one slow request gave a 1.7 s gap,
  past the rover's 1.5 s limit); builder R1–R3 and 422 mapping. One uncommitted file.
- **Signing gap:** release builds use the DEBUG key, and `android/` is git-ignored (also in the 4WD app, whose
  signing lives only in an untracked build.gradle). Required: a committed Expo config plugin that injects release
  signing from `DYX_RELEASE_*` in `~/.gradle/gradle.properties` (the existing DYX key).
- Owner direction: one product; the engineering panel shows only in debug builds or behind a protected toggle.
- **Tablet safety for 2026-10-09:** the 00:37 APK is for watching only. Drive with RC and QGC (TCP 5760) until the
  reviewed, signed build lands.

**Wi-Fi.** The baseboard has the dev-kit Realtek RTL8822CE (2.4 and 5 GHz). On the 4WD it stalled at 5–8 m.
Likely causes: Wi-Fi power save, regulatory domain "00", antenna placement, DFS/channel. 15 m should be easily
possible. Plan: 5 GHz on channel 36 or 149 (non-DFS), country IN, power save off, external antennas outside the
enclosure. Fallback: a MT7612U or MT7921AU USB adapter.

**Notes for the next session:**
- Codex and Claude shared one DYX_3WD clone once tonight, and a Claude docs commit landed on Codex's branch (moved
  to the deploy branch by fast-forward, nothing lost). Claude now edits in its own worktree
  (scratchpad `dyx3-master`) while an agent works in `~/Vetri/3WD_PROD/DYX_3WD`.
- Upgrade target: `master` `ad58e72` once its CI release `rover-ad58e72…` is published
  (otherwise `cb8ea42`, the same code minus docs and the test fix).

## 2026-10-09 — Codex — rover backend app gaps (local branch only)

Branch `codex/rover-app-gaps` was reset to `origin/claude/cloud-phases` at `a1f7cd4` as instructed.
Nothing was pushed or deployed. Commits are one concern each: app-planned mission ingest,
parse-only DXF import, optional hotspot, tablet token documentation, and the rover-side
contract/handoff documentation. The app and prototype repositories were read only.

**Mission ingest:** `POST /api/missions/plan` requires operator auth and accepts the app v1
payload with the rover v1.1 R1–R4 rules. `DYX3PATH 1` and all `dyx3_rpp` / `dyx3_mission`
source remain unchanged. The flag bits agree: bit 0 spray, bit 1 must-hit. RPP builds its split
flags from `z & 1`, so must-hit cannot create a boundary. The backend rejects mixed spray,
same-type neighbours and noncontiguous runs; it omits each later run's first point and ORs
must-hit into the prior shared endpoint. The Python mirror and a C++ gtest loaded a fixture
generated by the backend encoder and verified reconstruction. Accepted payloads add no transit
legs or geometry. RPP's existing conditioner still runs at mission install.

**Import:** the app branch's UI calls `/api/path/parse-dxf` in the template/rover-side DXF
flows. `parse-point-csv` and `parse-point-gps-csv` exist as unused `pathApi.ts` helpers and have
no UI callers, so no CSV HTTP route was added. DXF parsing uses a temporary file, returns the
prototype's entity-summary shape, and writes nothing to the missions directory. The existing
path-engine CSV parser is covered directly by a parse test.

**Network:** optional `dyx3-hotspot` profile uses NetworkManager shared IPv4 at
`10.42.0.1/24`. Blank `/etc/dyx3/hotspot.env` (`root:dyx3 0640`) disables it; a configured
profile is generated only when a Wi-Fi device is found. Keyfile mode is `0600`; PSK is absent
from command arguments and logs. A pre-up firewall hook blocks forwarding between Wi-Fi and
FCU Ethernet in both directions. The bench backend's `0.0.0.0` env override was untouched.
**Hardware remains unverified:** AP capability, tablet DHCP, firewall operation and the live
`curl http://10.42.0.1:8000/api/ping` check need the rover bench.

**Tablet token:** existing `python -m dyx3_backend.auth.tokens` CLI was verified with a
temporary auth store: operator role, hash-only `auth.json`, mode `0600`, token printed once.
The exact production command and backend restart are in `installer/README.md`; no shim was needed.

**Off-target verification:** Ruff passed; backend pytest **564 passed, 43 skipped** (carried
path-engine tests requiring prototype orchestration); ShellCheck passed; staged installer
tests in the ROS container **97 passed, 0 failed** (prebuilt-artifact subtests skipped because
that container's tar lacks zstd); `ros2_humble.sh build-test` passed twice, final run **12
packages, 459 tests, 0 errors, 0 failures, 2 skipped**. The C++ cross-check was included in
that final run. No on-rover or tablet test was performed.

**DERIVED — NOT FROM V1 SPEC:** `RUN_TOO_SHORT` is the new 422 code for the required two-point
minimum; the hotspot selects the first Wi-Fi device reported by NetworkManager, restricts
unquoted SSID/PSK values to a safe ASCII subset, and uses a pre-up firewall hook to keep
NetworkManager's shared forwarding off the FCU link. Verify those choices on actual Wi-Fi
hardware before the field demo. The R1–R4 rules themselves were the human decision, not
derived here.

## 2026-10-09 — Codex — app-gap follow-up E–G (local branch only)

Rebased `codex/rover-app-gaps` onto `origin/master` at `ad58e72`, then onto the later
documentation commit `67a7ea6`; E–G remain local.
Nothing was pushed or deployed. Socket.IO ping interval and timeout now read
`DYX3_SIO_PING_INTERVAL_S` and `DYX3_SIO_PING_TIMEOUT_S`, defaulting to 5 seconds each;
the separate 1.5-second tablet heartbeat remains the safety timeout. App mission-plan
requests reject more than `upload_max_bytes` before JSON parsing, with 413 `too_large`
for both Content-Length and streamed requests.

The optional hotspot installer now applies `DYX3_WIFI_COUNTRY` at every boot through
`dyx3-wifi-regdom.service` (template default `IN`), disables Wi-Fi power save in the
profile, and installs a driver modprobe option only when the detected RTL8822CE module's
`modinfo` confirms it. The profile pins band, non-DFS channel, and WPA2-PSK. Explicit
20/40 MHz AP width needs NetworkManager 1.50; older versions use 20 MHz auto, and a
requested 40 MHz profile is refused. The health check reports `iw reg get` and Wi-Fi
power save. The README has the two-band, 15-minute range test.

**Verification after rebase:** Ruff and ShellCheck passed; backend pytest **569 passed,
43 skipped**; staged installer tests in the ROS container **106 passed, 0 failed**
(prebuilt-artifact subtests skipped because that container's tar lacks zstd); ROS
`build-test` **12 packages, 459 tests, 0 errors, 0 failures, 2 skipped**.

**DERIVED — re-validate in field Wi-Fi:** both 5-second Socket.IO ping values and the
NetworkManager 20 MHz auto behavior on versions before 1.50. The Wi-Fi hardware/AP,
regulatory state, driver option effect, tablet DHCP, and 5/10/15/25 m range remain
unverified on a rover. No code is half-finished; the rover bench check is the open
field task.

---

## 2026-10-09 (02:00) — Claude — final pre-bench state

- `codex/rover-app-gaps` E–G reviewed:
  - E: Socket.IO ping 5/5 s via settings, rejecting values ≤ 0 and non-finite values;
  - F: hotspot radio settings, with the country from a systemd unit, driver power save only from `modinfo`-listed
    options, DFS refused, 40 MHz only on NetworkManager ≥ 1.50, dormant until `hotspot.env` is filled in;
  - G: the `/missions/plan` body bounded by Content-Length and while streaming.
  Fast-forwarded into **`master` @ `8af2595` = the deploy target** (release `rover-8af2595…`). The Codex branch is kept.
- Firmware V1 candidate `8279fa4be3` is unchanged. **No more code changes to the rover stack or firmware before the
  bench.** Later `master` commits are docs only.
- App: Agy is still working on `agy/prod-transport` (uncommitted `App.tsx`, `ModernHomeUI.tsx`,
  `ModernSettingsPage.tsx`). Nothing is merged into the app's `main`, and release signing is not implemented. The
  00:37 APK is DEBUG-signed: watch only.
- The next session starts from `~/Vetri/3WD_PROD/HANDOFF_PROMPT_2026-10-09.md`, which includes the app review
  checklist and the ready signing task for Agy.

---

## 2026-10-09 (08:30) — Claude — operator app merged and signed

Three_Wheel_v2 `main` was fast-forwarded to **`dbb2ba1`** (`agy/prod-transport`), after Claude's review:
- single AppTransportService (socket, REST, heartbeat, secure storage);
- fixed-rate 500 ms heartbeat with a 350 ms timeout;
- honest disconnect, and a 401 stops retrying;
- AppState handling;
- one telemetry store with staleness;
- dual-path E-stop;
- mission builder obeying R1–R3, with 422 mapping;
- spray locked out;
- the prototype port only behind an explicit option;
- release signing from the committed Expo plugin `plugins/withAndroidReleaseSigning.js`.

Checks: tsc passes; vitest 984/985. The one failure (`roadMarkingCsvPath` "never closes into a polygon ring") is
identical on `Runtime_Path`, so it pre-exists.

Signed APK: `~/Vetri/3WD_PROD/App-Releases/dbb2ba1-agy-prod-transport/app-release.apk`. Certificate is the DYX
release key (`5eaacb96…`); APK sha256 `730c6111…`. Tablets must uninstall the old debug-signed build first.
No `agy/*` branches were deleted.

---

## 2026-10-09 (08:55) — Claude — bench started; handed to Codex

- **08:35:** flashed firmware `8279fa4be3` over USB (verify OK). `ver git`: PX4 `8279fa4be3`, NuttX `e462af8eb3`.
  DDS connected; `gps status` shows the new RTCM counters, 6.16 Hz, 0 dropped.
- **08:42:** `dyx3-upgrade 8af2595` in 31 s, health OK.
  - The agent restart caused **no stall** (first restart on the new firmware).
  - RTK worker `INJECTING`, NTRIP → PX4_DDS, imported from ntrip.env, 0 CRC failures. 81 transport failures only
    during the startup handshake; flat afterwards.
  - `/var/lib/dyx3/rtk` created by StateDirectory.
- **Skipped:** the stall reproduction on the old firmware (the owner chose to flash first).
- **Hand fix to persist:** `tcpdump` was installed on the Jetson for the stall procedure. It must be added to the
  installer packages.
- Live session handed to Codex: `docs/agents/handoff_2026-10-09_codex_bench.md`. The bench runbook is
  `docs/bench/2026-10-09_bench_runbook.md`; the helpers are `tools/bench/nsh.py` (PX4 MAVLink shell over USB) and
  `tools/bench/rtkstat.py` (RTK worker status). The next-session brief is
  `docs/agents/handoff_2026-10-09_next_session.md`.

---

## 2026-10-09 (09:35) — Claude — local branch cleanup; every repo on its authoritative branch

At the owner's request, all essential local work was merged first. Then merged or superseded local branches were
removed, **locally only**; nothing was deleted on GitHub.

| Repo | Checked out (local = GitHub) | Local branches kept |
|---|---|---|
| DYX_3WD | `master` @ `5226a16` | `codex/phase-a-stall-validation` (Codex is running the bench in this clone) |
| PX4-Autopilot-3WD-Prod | `dyx-3wd-production` @ `8279fa4be3` (NuttX `e462af8eb3` from `Vetri2425/NuttX`) | `dyx-4wd-production` (the 4WD product line) |
| Three_Wheel_v2 | `main` @ `dbb2ba1` | none |

- **Merged:** two docs commits that existed only on local `master`, now on `master` as `25a642c` and `5226a16`:
  - `docs/migration/ab_arm_audit.md` (prototype A/B arms audit; input for the production cleanup);
  - `docs/architecture/proposals/2026-10-08_rtk_correction_upgrade_plan.md` (early RTK plan, superseded by
    `docs/plans/2026-10-08_production_rtk_plan.md`).
- **Superseded, not merged:** `codex/phase-f-closure`.
  - Its px4_link F4 fix is already on `master` in equivalent form: explicit STOP on an invalid clock in
    `Px4LinkNode::step`.
  - Its F6 mutation tests are `47796ad`; F2 was already in.
  - Backup of all local-only branches: `~/Vetri/3WD_PROD/Patches/dyx3_local_branches_backup_2026-10-09.bundle`.
- **Firmware checkout:** the uncommitted WENC edit was byte-identical to `8279fa4be3` and was discarded, then the
  branch fast-forwarded. The worktree `PX4-3WD-ethfix` and the branches `fw/eth-stall-rtcm`,
  `codex/eth-ring-guard-rtcm` and `codex/fw-eth-rtcm-v2` were removed; their content is in `dyx-3wd-production`.
- **App:** `agy/debug-client`, `agy/prod-contract-plan` and `agy/prod-transport` were removed locally; all are
  contained in `main`. The remote `agy/prod-transport` is kept.
- **Codex:** the DYX_3WD clone is now on `master`. **Run `git switch codex/phase-a-stall-validation` before
  committing.** Its untracked `tools/bench/phaseA_executor.py` is untouched.
- **Not touched:** the `.kilo/worktrees/tourmaline-basketball` worktree, and old-session GATE-2 scratch worktrees
  registered in the firmware repo.

Live on the rover (unchanged): firmware `8279fa4be3`, stack `8af2595` (code identical to `master`; later commits
are docs only), app `dbb2ba1`.

## 2026-10-09 (09:43 IST) — Codex — Phase A stopped on RTK transport recovery

Bench-tool work is on topic branch `codex/phase-a-stall-validation-run2`, based on `master` at `5226a16`.
No commit or push was made. Added `tools/bench/phaseA_executor.py`; the workspace-level
`../bench_tools/phaseA_executor.py` is now a compatibility entry point to that tracked executor. It records
subprocess status/stderr/duration, retries empty USB NSH reads, distinguishes stall from DDS and observability
failures, brackets DDS recovery time with USB poll observations, and writes durable JSON/CSV iteration records.
RTK recovery now waits for `INJECTING`, then compares delivered/failure counters at 15-second intervals over a
30-second window. This window was derived from the bench observation below. The local self-check passed, and the
read-only preflight passed for firmware, stack, config, disarmed state, DDS/timesync, capture growth and disk.

Owner reconfirmed the physical stop, disarmed PX4, wheels-up or isolated drive power, disabled spray, RC kill
switch, USB access, capture and disk. No PX4 reboot, firmware flash, parameter change, Phase B test or reboot test
was performed.

**Firmware/stack:** PX4 `8279fa4be33d5fc26c3b895c7e4a0a8660fcfff1` (`8279fa4be3`), NuttX
`e462af8eb32142e415b184aabe44c655eab4b735`; installed stack
`8af2595e1cc919de89b39fea0fb2aec6e26eb934` (`8af2595`). `MAV_2_CONFIG=1000`.

**Executor calibration, excluded from the fresh batch:** one restart was first evaluated with a 10-second RTK
window. Transport failures rose 980 → 1036 → 1066 while the worker moved from `INJECTING` through
`WAIT_TRANSPORT` back to `INJECTING`. A subsequent read-only 44-second observation showed failures flat at 1066,
delivered frames increasing 18,882 → 19,136 and PX4 injection at 6.00–6.16 Hz. This showed the 10-second window
was too short and led to the revised recovery window; this restart is not counted.

**Fresh batch:** stopped at 2/30 under the stop-on-failure rule.

- Iteration 1 PASS: DDS and timesync recovered; 388 PX4-originated non-ICMP frames in the 5-second window.
  Reconnect interval was 0–4,069.9 ms (the lower bound is the restart trigger because recovery occurred before the
  first successful USB poll). RTK entered `INJECTING`; over its final 15-second stable comparison, failures held
  at 1,157 and delivered frames increased by 92. PX4 RTCM injection was 6.0 Hz, with zero dropped bytes, short
  writes, EAGAINs or write errors. Commander cycle count increased; PX4 remained disarmed with the same firmware.
- Iteration 2 RTK_NOT_RECOVERED: DDS and timesync recovered; 386 PX4-originated non-ICMP frames in 5 seconds.
  RTK remained `WAIT_TRANSPORT` for the 30-second recovery window; delivered frames remained 20,003 while failures
  rose 1,210 → 1,414. A read-only post-stop snapshot at 09:42:22 IST still showed `WAIT_TRANSPORT`, delivered
  20,003, failures 1,534, PX4 RTCM injection 0 Hz and zero dropped bytes/write errors. PX4-originated traffic
  continued (1,572 non-ICMP frames in 5 seconds); `dyx3-platform`, `dyx3-rtk` and `dyx3-backend` were active.
  No further restart was issued.

Persistent PCAP (private, not in Git):
`/home/flash/bench_2026-10-09/phaseA_mav2cfg_1000/fcu_all_phaseA.pcap`.
Fresh batch JSON/CSV and per-iteration diagnostics are in
`~/Vetri/3WD_PROD/bench_tools/phaseA_results_fresh_20261009_093938/`; the earlier four-iteration evidence remains
untouched in `phaseA_results/`. The initial short-window experiment and its read-only follow-up are in
`phaseA_results_fresh_20261009_093622/`.

**Disposition:** no PX4 Ethernet TX stall was observed; the RTK transport did not recover on iteration 2. Phase A
did not reach 30/30 and firmware `8279fa4be3` is **not accepted**. Verdict: inconclusive pending investigation of
the Jetson RTK PX4_DDS transport recovery. Do not proceed to Phase B until that issue is resolved and the owner
authorizes it.

## 2026-10-09 (09:55 IST) — Codex — USB_DIRECT migration stopped at hardware identity gate

The owner reconfirmed the safety conditions, including PX4 disarmed, drive isolated or wheels raised, spray
disabled, RC/kill switch and independent USB NSH available, secure configuration backup available, and the
physical stop engaged. No live RTK setting was changed and no service/platform restart was issued.

Read-only Jetson discovery at `dyx-3wd` (`192.168.1.32`) found one USB serial bridge:
`1a86:7523 QinHeng Electronics CH340 serial converter`, product string `USB Serial`, with no USB serial number.
It was attached to `usbfs`; no `/dev/ttyACM*`, `/dev/ttyUSB*`, or `/dev/serial/by-id/` entry existed, and no
serial node was available to the `dyx3` service. The repository and local manufacturer-document search did not
establish which UM982 COM this bridge represents, that it is distinct from the TELEM1 COM, or that the exposed
interface is configured to accept RTCM. The official UM982 documentation establishes three UARTs and RTCM 3.x
support in general, but that does not identify this rover's USB bridge/COM mapping. The architecture notes also
warn that the supplied C-RTK 2HP USB/COM2/UART2 wiring can starve PX4; the physical port mapping must be proven.

The production GET_STATUS snapshot showed NTRIP connected, 26,163 CRC-valid frames and zero CRC failures;
the selected transport remained `PX4_DDS`, `WAIT_TRANSPORT`, with 20,003 frames delivered historically,
6,160 transport failures, and stale last-delivery age (~834 s). Receiver readback was unavailable, fix type 0,
and correction age invalid. This does not prove receiver acceptance. The existing RTK config revision 1 was
copied to `/var/lib/dyx3/rtk/pre-usb-migration-20261009.json` on the Jetson with owner `dyx3:dyx3`, mode
`0600`; credentials were not printed or copied into the repository. No configuration field was changed.

**Blocker / next action:** do not select a port or baud and do not transmit RTCM until the physical USB/UM982
COM mapping and separation from TELEM1 are established from the correct carrier/receiver wiring documentation
or a documented hardware inspection. The Jetson's current enumeration has no usable tty. After that evidence is
available, continue with interface settings, API configuration, correction acceptance and DDS-exclusion proof.
Ethernet restart stress is deferred because the USB_DIRECT acceptance prerequisite is unmet. Sanitized discovery
evidence is in `../bench_tools/usb_migration_20261009/`; prior Phase A evidence and executor changes remain intact.

## 2026-10-09 (10:20 IST) — Codex — CH340/CH341 production remediation prepared, not deployed

The owner approved implementation of a narrowly scoped BRLTTY exclusion and investigation of a driver for the
current Jetson kernel, while directing that no transport change happen yet. The same owner explicitly required a
proposal before driver deployment and prohibited kernel replacement, firmware/PX4 parameter changes, UM982
configuration, push/merge, and release deployment without approval. No live Jetson file, package, driver binding,
service, or RTK setting was changed in this session.

**Root cause evidence:** Jetson has one QinHeng `1a86:7523` USB Serial bridge, no USB serial number, and it appears
under `usbfs`; no tty node or `/dev/serial/by-id` path exists. `brltty-udev.service` is active and PID 438 has its
file descriptor open on `/dev/bus/usb/001/004`. The installed `85-brltty.rules` contains the generic
`ENV{PRODUCT}=="1a86/7523/*"` claim. The running kernel is `5.15.185-tegra` (L4T 36.5.0); kernel config has
`CONFIG_USB_SERIAL=m` and `CONFIG_USB_SERIAL_CH341` unset, `modinfo ch341` reports missing, and `usbserial.ko`
exists. The installed header package is `nvidia-l4t-kernel-headers 5.15.185-tegra-36.5.0-20260115194252`.
The running kernel config reports `CONFIG_MODULE_SIG=y`, with no `CONFIG_MODULE_SIG_FORCE` setting.

**Driver provenance/compatibility:** the packaged source is NVIDIA's public L4T R36.5.0 `ch341.c`, with the
source and archive SHA-256 hashes recorded in `installer/drivers/ch341-dyx3-1.0.0/PROVENANCE`. The source was
compiled as an out-of-tree module on the rover against `/lib/modules/5.15.185-tegra/build`; the compile-only probe
compiled successfully and reported `vermagic=5.15.185-tegra SMP preempt mod_unload modversions aarch64`, an alias for
`usb:v1A86p7523...`, and dependency `usbserial`. It was not installed or loaded. DKMS is currently absent from the
rover; the production installer adds it as an APT dependency. The production route packages this versioned source
via DKMS, with `AUTOINSTALL=yes` and `BUILD_EXCLUSIVE_KERNEL` restricted to the exact validated kernel. The
installer can add only the exact headers version for this installed kernel after simulation confirms no kernel
package changes; unavailable headers or a different kernel fail with a specific message. No arbitrary `.ko`
download or kernel replacement is used.

**Repository integration prepared:** `install.sh` provisions before its first release switch. Upgrade installs
the provisioning unit first in enabled-service order, so old and new release managers run it immediately after
switching and before restarting the six rover services. The helper provisions DKMS and, when needed, exact
matching NVIDIA headers only after APT simulation proves no kernel package change. It requires exactly one
adapter, discovers each rover's own `ID_PATH`, shadows (never edits) BRLTTY's package file, refuses unmanaged
overrides, stages DKMS source idempotently, checks module vermagic and live binding, verifies `/dev/ttyUSB*`
group `dialout`, and records `/etc/dyx3/ch341-adapter.env`.
`dyx3-usb-serial-check.service` provisions and verifies the binding on activation and every boot.
The unit is in the production manifest so an older deployed release manager will start it after
switching to the new release, bootstrapping the first upgrade without a one-off SSH repair. An optional, explicit
`DYX3_UM982_USB_ID_PATH` records `/etc/dyx3/usb-receiver.env` only after the physical mapping is confirmed and a
matching by-path node exists. No identity is inferred for the UM982 and no RTK config is changed. A focused
uninstaller removes only installer-managed udev/DKMS/identity artifacts and requires reboot to release an
in-use module. Docs updated: installer README, bench runbook, and cloud review status.

`bash -n` and ShellCheck pass for the changed scripts. `installer/tests/run_tests.sh` passes **133/133** when run
with the installed GNU coreutils/findutils paths, matching the repo's documented macOS test setup. The test runner
covers adapter absence/ambiguity, unsupported kernel, BRLTTY path scoping and unmanaged override preservation,
DKMS source/provenance staging, idempotence, missing module/binding, boot check presence, and waiting for explicit
physical receiver confirmation. Tests do not establish runtime driver binding or tty permissions on hardware.

**Status / gate:** the owner approved driver deployment subject to diff review, topic-branch commit, owner-approved
master merge, successful CI/release artifacts, fresh physical safety checks, and successful pre-deployment
validation. No release deployment or hardware change has happened. The pre-existing Phase A executor remains
untracked and outside this commit. The original USB_DIRECT migration is a separate later phase. Even after driver
acceptance, stop before
USB_DIRECT until documentation/physical inspection proves the CH340-to-UM982 COM mapping, separation from PX4
TELEM1, supported baud, and RTCM input capability. USB_DIRECT migration and the 30-cycle Ethernet stress batch
remain outstanding.

## 2026-10-09 (continuation) — Codex — CH341 remediation committed; release and deployment gates pending

Continue from topic branch `codex/phase-a-stall-validation-run2`, HEAD
`7360199522b6069267cf250f2efa9adc8063940d` (`Add production CH341 USB provisioning`). The production remediation
is committed. Preserve the existing Phase A evidence and executor work; `tools/bench/phaseA_executor.py` is still
an untracked, pre-existing bench change and is outside this commit. The handoff itself has accumulated local
working-tree edits; review and retain them when committing any future documentation change.

**No production deployment has happened.** There has been no push, merge, CI run, release publication, upgrade,
BRLTTY rule installation, DKMS installation, module load/bind, reboot, RTK configuration change, USB_DIRECT
selection, RTCM transmission, or Ethernet stress run. Firmware, PX4 parameters, the six rover services, and the
live RTK config remain unchanged. The compile-only module probe and all rover discovery were read-only with
respect to installed system state.

Local validation for the committed installer change: `bash -n` and ShellCheck passed; the installer suite passed
**133/133** using the documented GNU coreutils/findutils paths. On-rover source/header compatibility evidence and
the module vermagic are recorded in the preceding section. The packaged DKMS source is version `ch341-dyx3 1.0.0`;
it is not installed on the rover. Do not treat compile-only evidence as live binding or deployment acceptance.

**Next steps for Claude:**

1. Inspect `git status`, `git show --stat --oneline HEAD`, and the complete committed diff. Keep the unrelated
   untracked bench executor and all pre-existing handoff history intact. Do not amend or rewrite the remediation
   commit.
2. Do not push or merge without the owner’s approval. The owner’s release gate requires an owner-approved merge
   to `master`, all required CI checks passing, and a published `rover-<full-sha>` GitHub Release with verified
   artifacts. Record the merge SHA, CI run, release tag, and artifact evidence.
3. After that gate, use the standard production upgrade and explicitly require prebuilt application artifacts;
   do not rebuild ROS 2 on Jetson. The documented command is
   `sudo DYX3_ARTIFACTS=prebuilt /opt/dyx3/bin/dyx3-upgrade <full-sha>`. Kernel-specific DKMS provisioning is
   separate from ARM64 application artifacts and must build against the exact supported Jetson kernel/headers.
4. Immediately before live deployment, reconfirm disarmed state, isolated drive power, disabled spray power, and
   diagnostic/rollback access. Apply only the reviewed scoped BRLTTY exclusion and compatible CH341 driver. Follow
   the documented safe-shutdown/reboot procedure, then verify binding for `1a86:7523`, BRLTTY exclusion, tty and
   stable identity, `dyx3` access, boot-time and installer checks, idempotent rerun, all six services, and PX4
   Ethernet DDS. Preserve diagnostics and use the versioned uninstaller/rollback if any check fails.
5. Stop after the CH341/BRLTTY deployment report. Do not select `USB_DIRECT` or send RTCM until a separate
   documented/physical verification establishes the CH340-to-UM982 COM mapping, separation from PX4 TELEM1,
   baud, and RTCM input capability. USB migration and Ethernet restart testing require a later owner approval.

Current working-tree status at handoff update: `docs/agents/HANDOFF.md` is modified and
`tools/bench/phaseA_executor.py` is untracked. Neither is part of commit `7360199522b6069267cf250f2efa9adc8063940d`.

---

## 2026-10-09 (11:50) — Claude — Task 1 DONE: RTK over USB to the UM982 (USB_DIRECT)

Owner priority: USB RTK → Ethernet no-stall + QGC → calibration/tuning/mission → backend/frontend → RPP.

**Release:** master `0eafcd5`, live on the rover; health OK, all 6 services plus `dyx3-usb-serial-check`.
Merged Codex `7360199` (CH341 packaging), plus Claude fixes:
- `d8716af`: the RTK worker accepts `/dev/serial/by-path`. The CH340 has no USB serial number.
- `f2d1888`: provisioning no longer aborts when no identity file exists yet. The unguarded `sed` exited 2
  under pipefail.
- `0eafcd5`:
  - the module is built directly with kbuild, **no DKMS**: `apt install dkms` pulled gcc-12 and upgraded 11
    system libraries, which the installer's guard refused;
  - the restart loop no longer aborts before health/rollback. A failed unit restart had left `current`
    switched with old processes running;
- plus this commit: BRLTTY is masked and the adapter re-enumerated (below), and `tcpdump` is in the base packages.

**Receiver identity, proven from the receiver itself:**
- Method: passive listen at every baud, then only the read-only queries `CONFIG` and `UNILOGLIST`. Nothing on the
  receiver was changed.
- USB (CH340 `1a86:7523`, `/dev/serial/by-path/platform-3610000.usb-usb-0:2.1:1.0-port0`) = **UM982 COM3**:
  GNGSV and GPVTG are logged only on COM3, and the USB stream carries them.
- COM1 (to PX4 TELEM1) is separate.
- COM1, COM2 and COM3 are all 230400 baud.
- COM3 outputs GGA at 5 Hz, so readback is available.

**Hand steps on the rover, now in the installer (this commit):**
- `systemctl mask --now brltty-udev.service brltty.service`. The BRLTTY daemon kept the CH340 claimed via usbfs
  even after the scoped udev exclusion.
- Re-authorize the device (`authorized` 0 → 1) so `ch341` binds.
- Note: on this rover the units were masked by hand before the marker file existed, so
  `/etc/dyx3/brltty-masked-by-dyx3` is absent. The uninstaller will not unmask them unless the marker is added.

**RTK config rev 2 (in `/var/lib/dyx3/rtk/config.json`):** `NTRIP → USB_DIRECT`, baud 230400,
`write_timeout_s` 0.2, `reopen_delay_s` 2.0 (both DERIVED).

**Evidence (indoors):**
- Worker `INJECTING`; delivered = valid frames (361 after ~1 min), **0 failures**.
- `/dyx3/rtcm` has no traffic, so there is no DDS/PX4 injection and no double injection.
- Receiver readback: GGA quality **1 → 2 (DGPS)**, receiver correction age **1.2 s** (valid), 7 satellites,
  HDOP 2.8. PX4 also shows fix type 4.
- Still to do outdoors: RTK FLOAT/FIXED.

**Found for task 2 (not fixed here):**
- RTK PX4_DDS transport never recovered after an agent restart: stuck in WAIT_TRANSPORT since 09:41.
- A `mission…` process at RT priority burns 94 % of one core; Jetson load average ~50.
- `dyx3-platform` takes 90 s to stop on restart (systemd stop timeout).
- A Codex trace prompt for the first two was handed to the owner.

---

## 2026-10-09 — Codex T2 trace before proposed fixes (`codex/t2-dds-rtk-and-rt-cpu`)

Started from `origin/master` `0eafcd5` in a separate worktree. No rover mutation is authorized in this task.

**RTK trace:** `rtk_node.cpp:208-215` requires a received fresh `Px4LinkStatus` with both
`session_alive` and `handshake_ok`, plus a matched `/dyx3/rtcm` subscriber. `DdsSink::deliver()`
calls `open()` again on every fresh frame (`transport_sink.cpp:200-224`), so its inactive flag
is not a permanent latch. `InjectionAuthority::deliver()` retains the selected DDS sink and
does not fall back to USB (`injection_authority.cpp:29-32`). `px4_link_node.cpp:737-752` re-arms
the handshake after a session reset, and `:882-923` continues publishing status at 10 Hz.
The 09:41 bench observations record fresh valid frames and rising delivery failures, but only
summary RTK status, so they do not identify which readiness input remained false. The rover
accepted TCP/22, but initial SSH reads intermittently stalled; live topic/status evidence is
still being collected. The proposed fix must preserve single-sink generation authority and
report readiness inputs explicitly.

**CPU trace:** `dyx3_mission` runs `rclcpp::spin` (`mission/src/main.cpp:8-17`) and its normal
10 Hz timer (`mission/src/mission_node.cpp:150-151,290-301`); no busy loop is visible in its
code. `dyx3-ros.service:27-30` applies FIFO 80 and CPU 4 to the entire ROS launch tree, so
mission inherits real-time priority despite `mission/src/main.cpp:1-2` declaring it non-RT.
The launch graph (`dyx3_bringup/launch/control_graph.launch.py`) contains all six ROS nodes.
The code evidence points to inherited scheduling as the definite priority defect; the exact
source of the observed mission CPU usage and polkit/journal load still needs live process and
journal evidence. The architecture §8 grants RT discipline to RPP and motion_guard, not mission.

**Read-only rover evidence collected after the initial trace (about 11:40 IST):**
`/proc/loadavg` read `65.98 63.03 61.24`. `ps` showed PID 14265 `mission_node` as
`FF`/RTPRIO 80 at 87.0% CPU, PID 14291 `px4_link_node` as `FF`/80 at 0.0%, and
PID 14304 `rpp_node` as `FF`/80 at 0.0%. Per-thread `ps -L` isolated the 87.0% to
mission TID 14275, not its main executor TID 14265. `GET_STATUS` through the RTK
read-only control socket returned `WAIT_TRANSPORT`, 20,003 delivered, 44,358 failures,
`dds_session_alive=false`, `dds_handshake_ok=false`, both PX4 RTCM counters null, and
transport delivery age 7,200 s. Null PX4 counters in `status_json()` mean the status
sample is stale, so the definite failed readiness condition is **fresh link status**.
The matched `/dyx3/rtcm` subscription could not be established from this snapshot:
`ros2 topic echo --once /dyx3/px4_link/status` and `ros2 topic info -v /dyx3/rtcm`
timed out under the load. `journalctl` over 10 minutes counted 137,795 and 73,587
`brltty` log entries from two PIDs; a sample was repeated `Ignored Byte` messages.
That separate journal load is consistent with the CH341/BRLTTY remediation already
committed in this repo but not yet deployed, per the preceding handoff sections.
No rover process, service, or configuration was changed.

**Local fix and verification:** `b6c4799` adds the exact DDS block reason, status age,
and RTCM subscriber count to RTK `GET_STATUS`, plus a deterministic DdsSink/authority
test that covers stale status, handshake pending, subscription disappearance/rematch,
recovery on the next valid frame, no replay, and no USB fallback. `455d847` removes
FIFO 80/CPU 4 from the entire `dyx3-ros.service` launch tree and applies the existing
allocation only to RPP and motion_guard at launch. A launch-plan test checks that
mission, px4_link, spray and gateway remain non-RT; a mission integration test checks
10 Hz idle callbacks and less than 0.5 CPU second over 1.5 wall seconds. The mission
source contains no busy loop; the high-CPU task was a secondary thread, so the code
fix removes its inherited RT scheduling rather than changing mission callbacks.

`./tools/dev/ros2_humble.sh test-pkg dyx3_gnss_rtk dyx3_mission dyx3_bringup`
passed (462 tests reported by `test-result`, 0 failures, 2 pre-existing skips).
`./tools/dev/ros2_humble.sh build-test` passed for all 12 packages, same result:
462 tests, 0 errors/failures, 2 skips. The full build had existing CMake deprecation
and gateway missing-initializer warnings. `clang-format` was applied to changed C++
and `git diff --check` passed. No timing, scheduler, or DDS recovery claim has been
verified on the rover because no deployment/restart was authorized.

**Claude rover-side actions after review/merge:** deploy this branch only under the
owner's bench safety gate, then verify `mission_node` is normal scheduling, only RPP
and motion_guard are FIFO 80 on CPU 4, `px4_link` publishes fresh status and its
handshake completes after an XRCE restart, `/dyx3/rtcm` matches, RTK resumes
`INJECTING` with increasing deliveries and no replay, and PX4 `gps status` injection
returns. Measure the 100 Hz control chain and ensure RPP/guard middleware threads do
not spin under FIFO. Apply/verify the already committed CH341/BRLTTY remediation as
a separate reviewed rover action; its log flood remains live in this trace.
The previous 53% `polkitd` snapshot was transient (later `ps` showed 0.6% and
`journalctl _COMM=polkitd --since -10min` had no entries); its cause is not proven.

**Open limitation:** the local tests prove retry semantics and idle scheduling policy,
not the exact Fast DDS thread behavior or a successful rover recovery. The RTK status
was stale; `/dyx3/rtcm` match could not be observed while the graph was starved.
The field stress test and PX4 RTCM rate are required before accepting this fix.

---

## 2026-10-09 — Claude — review and merge of Codex T2 (`codex/t2-dds-rtk-and-rt-cpu`)

Merged the three Codex commits onto `0f0eb4f`, plus one Claude fix commit. Nothing on the rover was touched.

**Reviewed, off-target (Humble container):**
- The launch prefix `taskset -c 4 chrt -f 80` runs as an unprivileged user with RLIMIT_RTPRIO=99. Every thread of
  rpp_node and motion_guard_node is FF/80, affinity 4. Mission, px4_link, spray and gateway are TS on all cores.
- With RLIMIT_RTPRIO=0, chrt fails with EPERM. The node exits and the graph shuts down, so the failure is loud.
- RTK `block_reason` uses the same order as the DdsSink readiness predicate. It adds no secret.
  Injection semantics are unchanged.

**Fixed (Claude commit):**
- `rpp_node` polled with `spin_some` in a tight loop (14 % of a core, 14 k wakeups/s). It now uses `spin_once(5 ms)`
  (0.75 %). This matters because rpp shares the FIFO core with the guard.
- `LimitMEMLOCK=infinity`: mlockall now succeeds.
- `TimeoutStopSec=15`: the graph stopped in 0.2 s off-target. The rover paid 90 s.
- spray_node, spray_watchdog and recorder still poll the same way, at about 14 % of a core each. They are not
  real-time, so this costs CPU but does not cause starvation. Fix them separately.

**Still unexplained:** what was spinning in mission TID 14275 at 87 %. Off-target, mission idles at 0.4 % of a core.
If it still spins at normal priority after deploy, take a per-thread look (`top -H -p`, `/proc/<pid>/task/*/comm`).

**Rover checks after deploy:**
- `ps -eLo pid,tid,cls,rtprio,psr,pcpu,comm`: only rpp_node and motion_guard threads FF 80, PSR 4; every other
  node TS. `systemctl show dyx3-ros -p LimitRTPRIO,LimitMEMLOCK,TimeoutStopUSec`.
- No "mlockall failed" in the journal.
- `px4_link` `loop_overrun_count` stays flat at normal priority. If it rises, give px4_link its own core
  (for example `taskset -c 5 chrt -f 70`). Never put it on CPU 4.
- RTK reaches `block_reason: READY` and then `INJECTING`.
- `systemctl restart dyx3-ros` finishes in well under 15 s.

## 2026-10-09 (12:10) — Claude — new rovers select the UM982 USB receiver automatically (task 1 persistence)

Branch `claude/usb-auto-select`. It makes the hand step "enter the USB receiver path and baud" unnecessary on the
next rover. This rover's existing RTK config rev 2 is not touched: an existing `config.json` always wins.

**How it works:**
- **Installer (`installer/lib/usb_serial.sh`), CH341 provisioning:**
  - for the 3WD hardware profile only (UM982 on USB port `platform-3610000.usb-usb-0:2.1`, CH340, UM982 COM3,
    230400 baud);
  - a passive, read-only 3 s read must see at least 5 checksum-valid NMEA sentences including GGA;
  - if it does, it writes `/etc/dyx3/usb-receiver.env` with `DYX3_USB_RECEIVER_ID_PATH`, `_DEVICE` (by-path link)
    and `_BAUD`.
  - A port outside the profile, a silent port or a busy port records nothing; the operator then enters the
    receiver path and baud as before.
  - Nothing is ever sent to the receiver.
- **`dyx3-rtk.service`:** `EnvironmentFile=-/etc/dyx3/usb-receiver.env`; starts after `dyx3-usb-serial-check`.
- **Worker (`rtk_config.cpp` `initial_from_environment`):**
  - Only when no runtime config exists, it uses the recorded identity, so the start is `NTRIP + USB_DIRECT`.
    `write_timeout_s` 0.2 and `reopen_delay_s` 2.0 are DERIVED.
  - It also overrides the legacy-seed DDS default.
  - An invalid recorded path or baud fails closed (`CONFIG_INVALID`, STOPPED).
- **`tools/dev/ros2_humble.sh`:** takes `DYX3_WS_VOLUME`, so parallel worktrees don't share a colcon volume.
- **Contract:** `docs/contracts/dyx3_rtk.md` (persistent configuration section).

**Verified (container):**
- colcon build-test: 460 tests, 0 failures. New `RtkConfigStore.FreshRoverUsesInstallerRecordedUsbReceiver`.
- Installer suite: 131 passed, 0 failed. New: profile records by-path and baud; silent or busy records nothing;
  RTK unit reads the env file and is ordered after provisioning.

**Not verified yet:** a fresh install on real hardware (the next rover, or after deleting `config.json` on a bench
rover, which the owner must approve).

## 2026-10-09 (12:40) — Claude — QGC over TELEM2 serial tried and reverted: Jetson UART defect on L4T 36.5

Goal: match PX4's official baseboard layout (Ethernet = DDS only, `MAV_2_CONFIG 0`) by moving QGC/MAVLink to
TELEM2 ↔ Jetson `/dev/ttyTHS1`.

**What was done:**
- On the FCU: `MAV_1_CONFIG 102`, `SER_TEL2_BAUD 921600`, `MAV_1_MODE 0`, then reboot.
  PX4 ran MAVLink on `/dev/ttyS4` @ 921600 as intended.

**Result: blocked by the Jetson side.**
- `ttyTHS1` (3100000.serial) receives the right byte rate (~2.2 kB/s, matching the PX4 TX rate), but about 96 % of
  the bytes read as `0x00`. No usable MAVLink frames; the same with `stty` + `cat`.
- This matches a known NVIDIA issue: the DMA UART RX path is broken on JetPack 6.2.2 / L4T 36.5 (our release:
  R36 REVISION 5.0). NVIDIA's suggestion (adding `iommus` to the `uarta` DT node, or forcing PIO) is untested and
  changes the boot device tree.
  See forums.developer.nvidia.com threads 363837 and 364699.
- The Pixhawk USB is not wired to the Jetson on this baseboard, so it is not an alternative.

**Reverted:** `MAV_1_CONFIG 0`, `SER_TEL2_BAUD` reset to default, FCU rebooted. After the reboot:
`uxrce_dds_client` Running, connected; disarmed. Nothing to persist; the FCU is exactly as before.

**Decision:** keep MAVLink + DDS together on Ethernet (`MAV_2_CONFIG 1000`). The task 2 stall test (30 restarts at
1000, 30 at 0) decides whether that is safe. Revisit TELEM2 only after an NVIDIA UART fix (newer L4T, or a tested
DT overlay).

Side note: both FCU reboots today reconnected DDS without a stall (2 informal reboots; not the stress test).

## 2026-10-09 (12:50) — Claude — Task 2: stall stress test PASSED (60/60); task 2 fixes deployed

**Deployed:** release `5f41038` (Codex T2 + review fixes) at 12:26, then `cacc1ba` at 12:49 (this branch: task C
and the spray/recorder poll fix). Firmware `8279fa4be3` unchanged. Health OK both times.

**Stall stress, rover disarmed, 30× `systemctl restart dyx3-platform` per phase, 3 s between restarts:**

| Phase | Checks after each restart | Result |
|---|---|---|
| `MAV_2_CONFIG 1000` (MAVLink + DDS on Ethernet) | FCU ping; QGC TCP 5760 heartbeat through the router; PX4 `uxrce_dds_client` Running, connected | **30/30 PASS**. Agent journal: 30 starts, 30 new "session established". Agent start → session 2.9–5.0 s or 12.0 s (client retry cadence). QGC heartbeat back in 1.0–2.3 s every time. |
| `MAV_2_CONFIG 0` (DDS only), checked over the USB console | FCU ping; a NEW "session established" in the agent journal after the restart; PX4 Running, connected | **30/30 PASS**. New session 3.4–6.1 s after restart. |

- Zero stalls. The FCU Ethernet TX path never stopped: ping and the session resumed every time.
- One iteration showed `restart=15.8s`. The journal shows the stop took 7 ms; the delay was ssh latency from the
  Mac, not the rover.
- `MAV_2_CONFIG` restored to 1000 and the FCU rebooted. QGC on TCP 5760 and DDS were connected afterwards.
- The test scripts live in the Claude scratchpad only; the procedure is the one in the bench runbook.
- **Still to do for task 2:** 3 FCU power cycles and 3 Jetson reboots (these also prove ch341, RTK USB and
  mavlink-router reboot persistence). Then persist the mavlink-router server-mode template: the hand-edited
  `0.0.0.0:14550` Server endpoint is now proven by the 30 QGC reconnects.

**Scheduling after the deploy (`ps -eLo cls,rtprio,psr`):**
- Only `rpp_node` (11 threads) and `motion_guard` (12 threads) are FF 80 on CPU 4. Everything else is TS.
- `dyx3-ros` has LimitRTPRIO 99, LimitMEMLOCK infinity, TimeoutStopSec 15 s. No "mlockall failed".
- px4_link: status 9.5 Hz, `loop_overrun_count` 0. RTK USB_DIRECT INJECTING, 0 failures; px4_link
  `rtcm_chunks_*` 0, so there is no DDS double path.

**Poll fix (`cacc1ba`), on the rover:** spray_node 46 → 6 %, spray_watchdog 37 → 2.8 %, recorder 37 → 1.2 % of a
core. The Jetson is 94 % idle, load average 1.8 (about 50 this morning).

**Task C proven on hardware:** the 12:49 upgrade ran CH341 provisioning while the RTK service was restarting. It
recorded `/etc/dyx3/usb-receiver.env` = by-path `platform-3610000.usb-usb-0:2.1:1.0-port0` @ 230400 ("NMEA/GGA
verified passively"). That is identical to the hand-entered config. The existing RTK config rev 2 was left
untouched, as designed.

## 2026-10-09 (13:10) — Claude — Task 2 DONE: 3 full power cycles pass; mavlink-router template persisted

**3 full power cycles** (FCU + Jetson together, owner at the rover, USB unplugged, rover disarmed):

| Cycle | Jetson booted | Agent → DDS session | Health | QGC 5760 + PX4 DDS | RTK |
|---|---|---|---|---|---|
| 1 | 13:00:34 | 13:00:47 → 13:00:51 | OK | heartbeat 20.4 s, DDS 26.6 s after power loss | USB INJECTING, 0 failures |
| 2 | 13:02:37 | 13:02:50 → 13:02:55 | OK | Running, connected | USB INJECTING, 0 failures |
| 3 | 13:04:35 | 13:04:48 → 13:04:53 | OK | Running, connected; disarmed | USB INJECTING, 0 failures |

- After every boot: BRLTTY units inactive (masked), the `ch341` by-path node present, and only RPP and
  motion_guard at FF 80 on CPU 4.
- This also proves reboot persistence for task 1 (ch341 + RTK USB) and for the router server mode.

**Persisted:** `deployment/network/mavlink-router.conf.tmpl` is now the proven server mode
(`Mode=Server Address=0.0.0.0 Port=14550`, TCP 5760 for QGC). The comment explains why client mode and a
10.41.10.1 bind fail. A new installer test pins the template.
- This rover's `/etc/dyx3/mavlink-router.conf` (created once, never overwritten) already has the same endpoints;
  only its comment text is older. No rover change is needed.

**Task 2 is complete.** Next is task 3 (calibration, RoboClaw motion, PX4 mode basics). Before it:
- check `MAV_2_MODE`/`MAV_2_RATE` (low Ethernet stream rates);
- note upstream #27497: a differential rover doesn't turn in Mission mode on v1.17;
- note the PX4 rover docs (v1.17): the rover modules are "experimental"; ROS 2 control should prefer the PX4
  ROS 2 Interface (rover setpoint types) over raw Offboard.

## 2026-10-09 (13:45) — Claude — Jetson tablet hotspot live (192.168.3.100, 5 GHz ch 149)

Owner plan:
- The tablet and ssh use the Jetson hotspot. Office Ethernet is not the field path.
- A USB internet dongle is the uplink; it is not plugged in yet.
- Rover addresses: 192.168.3.100 = first 3WD; the next rover gets .101.

**Repo (`installer/lib/network_install.sh`, template, README, 7 new installer tests; 141/0):**
- `DYX3_HOTSPOT_ADDRESS` is the per-rover CIDR. The FCU subnet is refused; blank means 10.42.0.1/24.
- `DYX3_HOTSPOT_IFACE`: blank picks the first non-USB Wi-Fi, so a USB dongle is never made the AP.
- The installer turns the Wi-Fi radio on, and warns when a client Wi-Fi profile is bound to the AP device.
- **The 5 GHz default channel is 149.** The Jetson's vendor `rtl8822ce` driver is self-managed for regulatory
  and locked to its world plan (`phy#0 (self-managed) country 00`, chplan 0x7F). These were all tried on the rover
  and none changes it: `iw reg set IN`, the `rtw_country_code=IN` module parameter (loaded, ignored), and
  writing `IN` to the proc `country_code`. Under that plan only 5745 MHz (ch 149) is not no-IR, so ch 36 failed
  with "Failed to start AP functionality". 2.4 GHz ch 1–11 also works.

**Rover-local (not in the repo):**
- `/etc/dyx3/hotspot.env`: SSID `DYX_3WD`, PSK set by the owner (not recorded here),
  `DYX3_HOTSPOT_ADDRESS=192.168.3.100/24`, `DYX3_HOTSPOT_IFACE=wlP1p1s0`, band `a`.
  - Pre-edit copy: `/etc/dyx3/hotspot.env.bak-20261009` (empty credentials).
- NetworkManager had the Wi-Fi radio off (`WirelessEnabled=false`); the installer step turned it on.
- The phone uplink profile "Vetri's Moto" was bound to `wlP1p1s0` with autoconnect on. It is now
  `autoconnect=no`. Rebind it, or a dongle profile, to the dongle interface when the dongle arrives.
- The hotspot step was run from this branch's script (`/home/flash/hs/`); the next release upgrade re-applies it
  from the repo.
- A 2.4 GHz switch was run by mistake at 13:33 (the tool reported it as rejected, but it had already executed)
  and was reverted to 5 GHz at 13:37 on the owner's instruction.

**Verified:**
- After a Jetson reboot: `dyx3-hotspot` comes up by itself in AP mode, ch 149 / 20 MHz, at 192.168.3.100/24;
  dnsmasq DHCP 192.168.3.109–254; power save off.
- FORWARD DROP between `wlP1p1s0` and `enP8p1s0` in both directions. The tablet has no internet through the
  rover by design until the dongle uplink exists.
- Tablet (Tab S10 Lite) got 192.168.3.199.
- **Antennas were not connected at first:** −85 dBm and 19 % loss at 5 m, both RF paths ~10–16 %, MCS0.
  The owner fitted both antennas: −50 dBm near the rover, −57 dBm at 5 m indoors, 1/32 loss, VHT 2SS.
  The ping RTT spikes (≤ 868 ms) at strong signal are the idle tablet's power save, not the radio.
- `iw` reports txpower 7 dBm, but the driver's real target is 16–17 dBm per path (`target_tx_power`).

**Still to do:**
- A 15-min stability test with the app streaming.
- An outdoor line-of-sight walk at 5/10/15/25 m (logger: `/home/flash/wifi_range_log.sh <tablet-ip>`, run as
  `systemd-run --unit=dyx3-bench-wifirange`).
- Set `DYX3_BACKEND_HOST=192.168.3.100` in `/etc/dyx3/backend.env` once the app works over the hotspot.
- Dongle uplink.
- Drop the office DHCP on `enP8p1s0` once Wi-Fi is the access path.
- Fit the antennas outside any metal on every rover.

## 2026-10-09 (13:55) — Claude — hotspot client-internet cap; FCU profile static; ready for the dongle

- **Client-internet cap, ported from the 4WD prototype**
  (`~/Vetri/4WD_Proto/rover_ws/scripts/network/99-dyx-hotspot-qos`, measured 2026-09-28):
  - The installer writes `/etc/NetworkManager/dispatcher.d/99-dyx3-hotspot-qos`.
  - It sets the Wi-Fi txqueuelen to 100, and adds a mangle FORWARD `xt_hashlimit` police on forwarded hotspot
    traffic. Only forwarded traffic is capped; the backend, ssh and NTRIP are not.
  - Owner decision (option B): **2 MiB/s down, 1 MiB/s up**. Set per rover with `DYX3_HOTSPOT_*_LIMIT/BURST`
    and `DYX3_HOTSPOT_TXQUEUELEN` in `hotspot.env`.
  - Why a cap at all: the tegra 5.15 kernel has no qdisc beyond `pfifo_fast` (no fq_codel, cake, HTB or TBF;
    `/proc/config.gz`). The office Raspberry Pi hotspot (192.168.1.100, kernel 6.8 raspi) runs **fq_codel**,
    which is why it gives full 20+ Mbps without app stalls.
  - **Planned real fix:** build `sch_fq_codel` for the tegra kernel the same way as ch341 (kbuild against the
    exact headers, installer-provisioned), then drop the cap.
- The FCU and isolation hooks now call `iptables -w`: a concurrent NM dispatcher run hit "xtables lock" once.
- **Rover `dyx3-fcu` profile set to static** (`ipv4.method manual`, 10.41.10.1/24, never-default) with
  `nmcli connection modify` only, so it takes effect at the next boot.
  - The rover was installed with the bench flag `FCU_KEEP_DHCP=1` (DHCP + static). With the office cable removed,
    a DHCP timeout with IPv6 disabled could fail the whole connection and drop the FCU address.
  - The repo default is already static, so no repo change is needed.
- **USB 4G dongle:** the 4WD prototype used the same kind of dongle with no setup (plug and play). This Jetson has
  `cdc_ether`, `rndis_host`, `cdc_ncm`, `option`, usb-modeswitch and ModemManager. `qmi_wwan`, `cdc_mbim` and
  `huawei_cdc_ncm` are missing, which matters only if the dongle is a QMI/MBIM modem.
- Installer suite 143/0.

## 2026-10-09 (14:00) — Claude — running on hotspot + 4G dongle (no Ethernet); automatic OS updates off

**Field access path proven** (office cable removed, dongle plugged, power-cycled):
- The 4G dongle is plug and play: Qualcomm `05c6:90b4` as `usb2` (`cdc_ether`/RNDIS), 192.168.8.199/24 by DHCP
  through NM's automatic "Wired connection 1". It is the only default route; HTTP 200 in 0.4 s, ping ~34 ms.
- ssh `flash@192.168.3.100` over the hotspot works. Mac ↔ rover ping 3.0 ms average, 0/30 loss.
- QGC on TCP `192.168.3.100:5760`: ATTITUDE 15 Hz, GPS 5 Hz (same as over the office cable).
- Backend `/api/ping` 200 in 10 ms. Health OK. DDS connected. FCU static 10.41.10.1.
- RTK: NTRIP over 4G → USB_DIRECT INJECTING, 0 failures.
- The Mac's internet runs through the rover NAT; the cap dropped 0 packets.

**Fix: automatic OS updates disabled** (`install_no_auto_updates`, runs on every install and upgrade):
- After boot, PackageKit, fwupd and update-notifier pulled ~20 MB over 4G in 4 minutes, with apt-daily-upgrade
  14 minutes away. QGC felt slow.
- Now: `/etc/apt/apt.conf.d/99dyx3-no-auto-updates` (periodic all 0), and these units masked: apt-daily(-upgrade),
  update-notifier-download/motd, fwupd-refresh, packagekit. After that the 4G rx is 1.7 KB/s (NTRIP only).
- Manual `apt` still works. Installer tests 145/0.

GPS indoors after the power cycle: fix 0, 4 satellites (expected). Next: outdoor RTK/heading and a line-of-sight
Wi-Fi range walk.

## 2026-10-09 (15:05) — Claude — task 3: rover crept backwards on arm, and steering was mirrored; both fixed (RC)

**Symptoms (owner, wheels up):** on arm in Manual the rover drove backwards. Steering right turned left.

**Root cause (read-only on the FCU, then log replay):**
- **Creep:** `manual_control_setpoint.throttle` was −0.128 with the sticks at rest.
  - Throttle is ch3 (`RC_MAP_THROTTLE 3`, `RC3_REV −1`). Its raw rest value is 1560, but `RC3_TRIM` was 1999
    (QGC throttle calibration stores the trim at one end). `rc_update.cpp` then auto-centres the trim to
    (min+max)/2 = 1499.
  - `DifferentialManualMode` passes throttle to the motors with **no deadzone**, so −0.128 became reverse
    drive; the RoboClaw driver's 3 % deadband cannot hide it.
- **Mirrored steering:** log `log7_2026-10-09_09-26-16.ulg` showed the owner's LEFT stick arriving as roll +1.
  - PX4, the mixer and the RoboClaw are consistent: Motor 1 = right (`control[0]`), and the encoders follow the
    commands. So only the input sign was wrong, and auto modes were never affected.
- The live mapping (throttle 3 / roll 1 / pitch 2 / yaw 4) differed from the repo baseline (throttle 2 / roll 3 /
  pitch 1). A QGC radio calibration probably rewrote it to the drone layout. The live layout is what the owner
  drives with.

**Fix (on the FCU, saved):** `RC3_TRIM 1560`, `RC1_REV −1`.

**Verified** (`log8_2026-10-09_09-29-38.ulg`, armed, wheels up):
- LEFT stick → roll −1 → right +1 / left −1 → encoders +7.0 / −7.0 rad/s (left turn). RIGHT → the mirror image.
- At rest after release: motors ≤ 0.017 (below the deadband), encoders 0.00 rad/s; throttle −0.005 to −0.007.
- Logs are in `~/Vetri/3WD_PROD/PX4-Firmware/3WD/logs_2026-10-09/`.

**Persisted:**
- `config/px4/3wd_6x_carry_from_proto.params`: `RC_MAP_*` 3/1/2/4, `RC1_REV −1`, `RC3_REV −1`.
- New `config/px4/3wd_rover01_rc_calibration.params`: this transmitter's RC1–4 min/trim/max/rev, plus the rule
  that after any QGC radio calibration you set `RC3_TRIM` to the measured ch3 rest and re-apply the mapping.

**4WD vs 3WD RoboClaw/encoder comparison:**

| | 4WD proto (v1.16.2, `1fe1789089`) | 3WD prod (v1.17, `8279fa4be3`) |
|---|---|---|
| Control mode | `RBCLW_VEL_CTRL` open/closed loop | always closed loop |
| Zero handling | <1 % → stop | 3 % deadband + <1 % → stop |
| Stop in Hold (AUTO_LOITER) | explicit | none |
| Fault reporting | `esc_status`, comm-failure flag | none |
| Encoder reads | speed + counters at `RBCLW_ENC_HZ` 20 | speed only, every cycle |
| EKF2 wheel fusion | yes | yes (`EKF2_WENC_CTRL 1`, `RAD 0.1498`, `LAT_N 0.1`) |
| `COUNTS_REV` / `QPPS_MAX` | 160000 / 171000 | 148000 / 172000 |

Port candidates (firmware is V1-frozen; needs the owner's OK): Hold-mode stop, `esc_status`/fault reporting,
open-loop fallback.

## 2026-10-09 (16:20) — Claude — production-readiness open list (owner-requested)

What still blocks production grade, by risk. Proven so far:
- 60/60 agent restarts with zero Ethernet stalls, plus 3 full power cycles.
- RT starvation fixed (Jetson load ~50 → 1.8). QGC recovers in 1–2 s.
- RTK over USB, 0 failures, reboot-persistent.
- Hotspot + 4G dongle without Ethernet. No OS auto-updates.
- Arm creep and mirrored steering fixed.

**Safety blockers (before any unattended or autonomous driving):**
1. **RoboClaw serial timeout is disabled** (`<timeout>0</timeout>` in `config/vehicle/roboclaw/Roboclaw_09-10-2026.cfg`,
   read from the prod RoboClaw today). If PX4 stops sending (FCU crash, cable cut), the RoboClaw keeps the last
   speed command forever.
   - Fix: set ~0.5 s in Motion Studio, then Write Settings.
   - Verify wheels-up: unplug the RoboClaw serial lead while it drives; the wheels must stop.
2. **RoboClaw battery window:** min main 5.9 V (factory default: no over-discharge cut), max 27.8 V.
   - PX4 is set for 8S LiFePO4 (3.4 V full / 2.75 V empty per cell ≈ 27.2 / 22 V). A full pack (28.8–29.2 V) or
     braking can trip overvoltage and stop the drive mid-run.
   - Set min ≈ 22 V and max ≈ 30 V after confirming the battery type and the RoboClaw model rating.
3. **`RBCLW_QPPS_MAX` 172000 is above the RoboClaw's tuned QPPS 165500** (both motors, today's autotune), so the speed
   loop saturates at full stick. Set 149000 (90 %) on the FCU and in `config/px4/3wd_6x_carry_from_proto.params`.
4. **Upstream #27514** (stale setpoint ~900 ms after the external process dies): the px4_link fail-to-zero (0.2 s) is
   not hardware-proven. Kill `dyx3-ros` while driving wheels-up and measure the time to stop.
5. **RC loss, tablet-link loss and app E-stop:** none tested on this rover (wheels-up first).
6. **No RoboClaw fault visibility:** the 3WD driver publishes no `esc_status`. The 4WD has it; porting it is a firmware
   change (V1 frozen), so it needs the owner's decision.
7. **Spray valve close test** not done (not needed for the drive-only demo).

**Autonomy blockers (Mission / Offboard / RPP):**
8. **UM982 dual-antenna heading** reports `UNIHEADINGA INSUFFICIENT_OBS`, so there is no yaw and the EKF has no global
   position. Prove it outdoors: antenna 2 cabling and sky view.
9. **RTK FIXED** not yet observed (indoor DGPS only).
10. **Upstream #27497:** a differential rover does not turn in Mission mode on v1.17. Verify; a workaround may be
    needed.
11. PX4 rover modules are "experimental", and raw Offboard is not guaranteed valid for rovers. The px4_link Offboard
    path needs wheels-up, then ground validation.

**Stall / hang / crash, not fully closed:**
12. **Long soak:** hours with motion + DDS + RTK + QGC + app together. Only the restart stress and power cycles are
    done.
13. **Upstream #27388** (uXRCE client silently stops publishing): px4_link detects staleness and fails to zero, but
    there is no automatic recovery. The soak shows whether it happens.
14. **Hotspot:** no 15-min app-stream test and no outdoor 15–25 m walk yet. The tegra kernel has no fq_codel, so the
    2/1 MiB/s cap stands in; the real fix is building `sch_fq_codel` (ch341-style kbuild).
15. The mission_node 87 % CPU root cause was never found (gone since the RT fix). Watch it in the soak.

**Production hygiene (rover == repo):**
16. The rover runs release `cacc1ba`; the hotspot, QoS, no-auto-update and RC changes were applied by hand.
    Upgrade to the latest master release.
17. No scripted apply/verify of the PX4 parameter baseline on a new FCU.
18. `/etc/dyx3/backend.env` still binds `0.0.0.0`; set `192.168.3.100`.
19. A fresh-rover install from scratch has never run end to end; USB auto-select is proven only on upgrade.
20. Persistence debt: ROS domain health check, NTRIP credential procedure doc, BRLTTY marker missing on this rover.
21. The app (`Three_Wheel_v2` `dbb2ba1`) has never connected over the hotspot: host config, tablet token, end-to-end
    test.
22. Commit `config/vehicle/roboclaw/Roboclaw_09-10-2026.cfg` (untracked in the main clone) after items 1–2.

**Suggested order:** RoboClaw settings 1–3, then the rover upgrade 16, then wheels-up safety stops 4–5, then outdoors
8–10 and 14, then the soak 12.

## 2026-10-09 (16:50) — Claude — app auth on the 3WD; QR pairing planned

- **Production auth:** one static operator Bearer token per tablet, SHA-256 hashed in
  `/var/lib/dyx3/state/auth.json`. No expiry; survives reboots and upgrades. Revoke a tablet by deleting its entry
  and restarting `dyx3-backend`.
  - `tablet-1` (operator) was created on rover 01 today. The plaintext was handed over via `/home/flash/tablet-1.token`
    (0600), which the owner deletes after pasting. Never logged.
  - Owner decision: keep tokens rather than the prototype password login.
- **The app build on the tablet comes from `Three_Wheel_v2` `App-Polish`, not `main`.**
  - `App-Polish` already sends `Authorization: Bearer` for NTRIP profiles (`main` sent nothing, which gave 401), and
    has no dead prototype logout/password buttons.
  - The two branches have diverged: the same safety code exists under different SHAs on each. Reconcile them so
    that `main` is authoritative again, with the owner's OK, then rebuild the signed APK.
- **Future (after Saturday): QR pairing.** The rover shows a QR code for a new tablet token (a CLI or local page);
  the app scans it. No copy-paste; still one token per tablet (`tablet-1`, `tablet-2`, …).

## 2026-10-09 (17:40) — Claude — site router + hotspot together; app discovery beacon (backend side)

**Field network on rover 01 now:**
- A site router (`192.168.3.1`) is cabled into the rover Ethernet (the baseboard switch shared with the FCU).
  - The Jetson has `192.168.3.150/24` on `enP8p1s0` next to `10.41.10.1/24`; the default route goes via the
    router at metric 200.
  - Per-rover file `/etc/dyx3/network.env` (new); applied by `install_fcu_network`.
- The **hotspot moved to `192.168.2.100/24`** (`hotspot.env`), so it never shares the router's subnet and both
  run at once. The installer now refuses a hotspot on the site-LAN subnet.
- ssh, backend (`/api/ping` 200, Socket.IO 200), FCU ping, DDS and internet were verified over the router.
  - The Jetson's own range was weak, which is why the owner moved to the router.
- RTK after the switch: the caster accepts the login but our mountpoint is **not in the caster source table**,
  and the stream sends no RTCM. **The base station is offline** (external): check its power, internet and NTRIP
  output. The rover recovers by itself.

**Discovery (owner requirement: no fixed IP; one token across networks; auto-connect like the 4WD app):**
- `/api/ping` now returns `rover_id` (stable: `DYX3_ROVER_ID` or sha256 of the machine-id) and `rover_name`.
- The backend beacons UDP 5003 every 1 s on each non-FCU network, each with that network's IP (contract 1a).
  Ported from the 4WD UDP beacon.
- App side (Three_Wheel_v2): beacon listener, a per-`rover_id` token, auto-connect, no hardcoded IPs. In progress.

Installer 148/0 (new: site LAN address + gateway, FCU-subnet refusal, hotspot/LAN clash refusal). Backend pytest
574 passed, ruff clean.

## 2026-10-09 (18:00) — Claude — session state: network, app discovery, RoboClaw; what is next

**Rover 01 now (release `cacc1ba` + hand-applied, repo-matching config):**
- **Network:**
  - Site router "DYX_4WD_PROTOTYPE" (`192.168.3.1`, the 4WD prototype's router) is cabled into the rover
    Ethernet.
  - Jetson `192.168.3.150/24` on `enP8p1s0` (next to FCU `10.41.10.1`), default route via the router at metric
    200 (`/etc/dyx3/network.env`).
  - Hotspot `DYX_3WD` at `192.168.2.100/24`, 5 GHz ch 149, still enabled as a backup path (`hotspot.env`).
  - The 4G dongle is not present at the moment.
  - QGC: TCP `192.168.3.150:5760`. ssh: `flash@192.168.3.150` (or `192.168.2.100` on the hotspot).
- **RoboClaw:** `config/vehicle/roboclaw/Roboclaw_01_DYX_3WD.cfg` (serial timeout 0.5 s, battery 21.4–28.4 V,
  QPPS 168000). FCU `RBCLW_QPPS_MAX` 151200 (saved).
  - This closes production-readiness items 1–3 (16:20 list).
  - Still to check: a fully charged pack must stay below 28.4 V.
- **RC:** `RC3_TRIM 1560`, `RC1_REV -1`, mapping throttle 3 / roll 1 / pitch 2 / yaw 4; wheels-up verified
  (log8).
- **Tablet token:** `tablet-1` (operator) exists in `auth.json`. Delete the handover file
  `/home/flash/tablet-1.token` once the app has it.
- **RTK:** the base station for our mountpoint is **offline** (not in the caster source table, no RTCM). External;
  the rover recovers by itself once it streams again.

**App (`yasarbaiiiii-blip/Three_Wheel_v2`, branch `claude/app-token-memory` from `App-Polish` `d693060`):**
- `2e75125`: token remembered (per rover); Disconnect keeps it; "Forget saved token". Also fixed the auth-runtime
  effect that dropped the Bearer token after connect (NTRIP profiles got 401).
- `6de4066`:
  - UDP beacon discovery on 5003 (`src/services/roverBeacon.ts`, react-native-udp 4.1.7 as in the 4WD app);
  - tokens keyed by `rover_id`, so one token works on the router and the hotspot;
  - auto-connect to the last rover with a saved token; a manual Disconnect pauses it;
  - **all hardcoded IPs removed**, and the HTTP fallback sweeps only the tablet's own subnet (it used to sweep 8
    fixed subnets, ~2000 probes every 5 s).
- Checks: tsc clean; vitest 1020/1021 (the only failure, roadMarkingCsvPath "never closes into a polygon ring",
  also fails on App-Polish).
- Signed APKs (DYX release key, cert 5eaacb96…):
  - `App-Releases/2e75125-app-polish-token-memory/`;
  - the `6de4066` build is installed on tablet `R5GYA14C7CY` (adb). Tablet `R5GL1016QFB` has the `2e75125`
    build.
  - Note: a second app, `com.dyxgcs.mobile` (older DYX-GCS), is also on the tablets.
- **Not merged** into `App-Polish`/`main`, by owner rule, until verified on the tablet. Afterwards reconcile
  `App-Polish` vs `main` (the same safety code under different SHAs) so a single authoritative branch remains.

**Backend (DYX_3WD master):**
- `f3bdf8a`: `/api/ping` returns `rover_id`/`rover_name`; per-network discovery beacon (contract 1a).
- `cc3c82a`: `network.env` site LAN, and the hotspot may not share the LAN subnet.
- `bd49374`: RoboClaw cfg + `RBCLW_QPPS_MAX`.
- CI was running at 18:00.

**Next, in order:**
1. When CI is green, upgrade rover 01 to the latest master release (this starts the beacon). Then check the app
   lists the rover by name at `192.168.3.150`, paste the token once, and check auto-connect on both the router
   and the hotspot.
2. Merge the app branch after the tablet test; reconcile `App-Polish`/`main`; archive the APK with its
   build_info.
3. Production-readiness list (16:20), remaining items: #4 (fail-to-zero after `dyx3-ros` dies), #5 (RC, link
   and E-stop stops), #6 (RoboClaw fault visibility, needs a firmware decision), #8–10 (outdoors: UM982 heading,
   RTK FIXED, Mission-mode turn bug), #12 (soak), #14 (fq_codel), #17–21.
4. Future: QR pairing for tablet tokens.

## 2026-10-09 (18:25) — Claude — rover on bd49374 (beacon live); app auto-connect verified; upgrade lessons

**Rover 01 is on release `bd49374`.** Health OK, 7/7 services.
- Discovery beacon live on both networks: router `192.168.3.150`, hotspot `192.168.2.100`; never on the FCU link.
- `/api/ping` → `rover_id dyx3-916baa908b`, `rover_name dyx-3wd` (a nicer name via `DYX3_ROVER_NAME` in
  backend.env is optional).

**Upgrade lessons (do this every time):**
1. **Run upgrades detached:**
   `sudo systemd-run --unit=dyx3-upgrade-<sha> --setenv=DYX3_ARTIFACTS=prebuilt /opt/dyx3/bin/dyx3-upgrade <sha>`,
   then follow `journalctl -u dyx3-upgrade-<sha>`.
   - An upgrade piped through an ssh session dies on SIGPIPE at its next log line when the session ends, possibly
     mid-activation.
   - Today a killed run left `current` switched but un-activated, and its `nmcli` child held
     `/run/dyx3/install.lock`. Recovery: wait for the lock holder to exit, then re-activate with `DYX3_FORCE=1`.
2. **The upgrade runs the *old* release's installer.**
   - From `cacc1ba`, that rewrote the hotspot to 10.42.0.1 / ch 36 (dead). The forced re-activation with the new
     release fixed it.
   - From `bd49374` onward the installer knows `DYX3_HOTSPOT_ADDRESS`, ch 149 and `network.env`, so this does not
     recur.
3. **`dyx3-usb-serial-check` fails closed with two CH340s.** The owner had plugged in a **LoRa radio** (CH340, USB
   port 2.3, 115200 baud, valid RTCM 1005/1074/1084/1094/1124/1230 from a base).
   - That failed health and auto-rolled back the first attempt (`cc3c82a`). The LoRa is unplugged and parked by
     the owner.
   - TODO: the installer should select the UM982 by its profile port (2.1) and leave other CH340s (LoRa) alone.
     The test stub must return a per-device ID_PATH.

**App:**
- Tablet `R5GYA14C7CY` had a **debug-signed** dev build (no beacon code), which is why discovery was slow, and
  DYX-signed updates were rejected (`INSTALL_FAILED_UPDATE_INCOMPATIBLE`). Uninstalled with the owner's OK.
- **The "release crashes" report was the missing Mapbox token:** `.env` is gitignored, and a build without it
  crashes with `MapboxConfigurationException` when the map mounts after connect.
  - New `plugins/withRequiredMapboxToken.js` makes prebuild fail without a `pk.` token.
  - Build worktrees need `cp ../Three_Wheel_v2/.env .`.
- Verified 18:16 on `R5GYA14C7CY` (router):
  - beacon heard 2 s after launch;
  - **auto-connect** with the saved token (by rover_id);
  - connected about 3 s from launch;
  - map OK, no crash.
- APK: `App-Releases/4891a3d-app-polish-beacon-autoconnect/`.
- Branch `claude/app-token-memory` (`2e75125`, `6de4066`, `4891a3d`) merged into `App-Polish` on the owner's
  instruction.

**App cleanup list:**
- After connect, App-Polish still calls the prototype routes `/api/healthz`, `/api/telemetry/latest` and
  `/api/mission/loaded-path` (404 on the production backend). Move them to production routes.
- Mission start/stop still guesses production vs prototype from the URL port (see the 16:50 entry).
- Reconcile `App-Polish` and `main` into one authoritative branch.
- Update tablet `R5GL1016QFB` (it has `2e75125`).
- Hands-on owner test pending: Disconnect/Connect, relaunch, and the hotspot path.

## 2026-10-09 (18:35) — Claude — motor rewiring: M1 is now the left wheel; fixed in the RoboClaw output mapping

- The owner rewired the drive: **RoboClaw M1 = LEFT wheel, M2 = RIGHT wheel** (each motor moved with its
  encoder). Steering then came out mirrored.
- **Fix:** `RBCLW_FUNC1 102`, `RBCLW_FUNC2 101` (FCU saved, plus `config/px4/3wd_6x_carry_from_proto.params`).
  - `RC1_REV` stays −1. A left/right swap must be fixed in the output mapping; the stick sign would fix only
    manual, and Mission/Offboard/RPP would turn the wrong way.
  - EKF2 uses only the mean of `wheel_encoders.wheel_speed[0..1]` (EKF2.cpp:2243), so the swapped right/left
    labels in that topic are harmless.
- **Verified:**
  - Before: log23/24 (12:49 UTC) showed consistent motor/encoder pairs, so a channel-level move.
  - After: log28 (13:00 UTC), stick left → PX4 right motor +0.17…+0.48 → M2 encoder +1.0…+2.9 rad/s; left motor
    −0.19…−0.49 → M1 −1.1…−3.0 rad/s.
  - The owner confirmed physically: it turns correctly.
- `config/vehicle/roboclaw/README.md` documents the current wiring and the rule.
- Log download over the router: request as a GCS component (source_component 190). Component 0 gets no LOG_ENTRY
  reply through mavlink-router.

## 2026-10-09 (18:40) — Claude — END OF DAY: rover 01 state and the single open list (start here next session)

**Rover 01 (all of this is in the repo or the rover-local /etc/dyx3 files the repo documents):**

| Area | State |
|---|---|
| Firmware | `8279fa4be3` (V1 final). Ethernet stall fixed: 60/60 restarts + 3 power cycles |
| Stack | release `bd49374`, health OK, 7/7 services. Upgrade **only detached** (`systemd-run`, see 18:25) |
| Network | router "DYX_4WD_PROTOTYPE" `192.168.3.1` → Jetson `192.168.3.150` (`network.env`); hotspot `DYX_3WD` `192.168.2.100` ch 149 (`hotspot.env`); FCU `10.41.10.1`; no 4G dongle; office cable removed |
| Access | `ssh -o ProxyCommand=none -o HostKeyAlias=dyx-3wd flash@192.168.3.150`; QGC TCP `192.168.3.150:5760` |
| Discovery | UDP beacon 5003 on both networks; `rover_id dyx3-916baa908b`, name `dyx-3wd` |
| Drive | RoboClaw M1 = LEFT, M2 = RIGHT → `RBCLW_FUNC1 102`, `RBCLW_FUNC2 101`; `RBCLW_QPPS_MAX 151200`; RoboClaw serial timeout 0.5 s, battery 21.4–28.4 V (`config/vehicle/roboclaw/`) |
| RC | `RC3_TRIM 1560`, `RC1_REV −1`, mapping throttle 3 / roll 1 / pitch 2 / yaw 4 (`config/px4/3wd_rover01_rc_calibration.params`) |
| RTK | USB_DIRECT to UM982 (COM3, CH340 port 2.1). NTRIP base **offline** (mountpoint not in the source table). LoRa radio found on CH340 port 2.3 @115200 with valid RTCM; unplugged and parked |
| Auth | operator token `tablet-1`; delete `/home/flash/tablet-1.token` once both tablets have it |

**App (`Three_Wheel_v2`):**
- `App-Polish` = `4891a3d`: token remembered per rover_id, beacon discovery, auto-connect, no hardcoded IPs, the
  Mapbox build guard.
- Signed APK in `App-Releases/4891a3d-app-polish-beacon-autoconnect/`. Verified on tablet `R5GYA14C7CY`: rover
  found in about 2 s, auto-connect, map OK.
- The owner's local `Three_Wheel_v2` clone has an **uncommitted** `plugins/withAndroidMapboxToken.js` plus an
  `app.json` entry. It writes the native `mapbox_access_token` resource, which is better than my guard. Pending
  decision: adopt it, drop `withRequiredMapboxToken`, rebuild.

**Open list, in suggested order:**
1. Owner hands-on app test: Disconnect/Connect, relaunch, hotspot path. Update tablet `R5GL1016QFB`.
2. Mapbox plugin decision (above), then rebuild, verify, push App-Polish.
3. App cleanup:
   - prototype routes `/api/healthz`, `/api/telemetry/latest`, `/api/mission/loaded-path` (404);
   - mission start/stop port guessing;
   - reconcile `App-Polish` and `main` into one authoritative branch.
4. Production-readiness (16:20 list), remaining:
   - #4 fail-to-zero when `dyx3-ros` dies (#27514);
   - #5 RC loss, link loss and app E-stop stops (wheels-up);
   - #6 RoboClaw fault visibility (needs a firmware decision);
   - #8–10 outdoors: UM982 dual-antenna heading, RTK FIXED, Mission-mode turn (#27497);
   - #12 long soak;
   - #14 build `sch_fq_codel` for tegra, then drop the hotspot cap;
   - #17 scripted PX4 parameter apply/verify;
   - #19 fresh-install end-to-end;
   - #20 persistence debt;
   - check that a full pack stays below 28.4 V.
5. Installer: select the UM982 by port 2.1 and allow a LoRa CH340 next to it (test stub with a per-device
   ID_PATH). Then **LoRa correction verification** (owner: after the app is solid).
6. Base station: bring the NTRIP mountpoint back online (external).
7. Future: QR pairing for tablet tokens; nicer `DYX3_ROVER_NAME`.

## 2026-10-09 (18:50) — Claude — app: owner's Mapbox plugin adopted (App-Polish 4096af9); tablet R5GL1016QFB updated

- **The owner's `plugins/withAndroidMapboxToken.js` is adopted** (Three_Wheel_v2 `App-Polish` `4096af9`; my
  env-only guard `withRequiredMapboxToken` and its test are deleted).
  - It fails prebuild without a `pk.` token **and** writes the native `mapbox_access_token` string resource, so
    MapView has the token even before JS `setAccessToken()` runs.
  - New test `src/plugins/__tests__/withAndroidMapboxToken.test.ts` runs the strings mod: refusal, write,
    replace-not-duplicate. Plugin tests 5/5; tsc clean.
  - The owner's uncommitted change was stashed, fast-forwarded, moved into the commit, and the stash dropped.
    Their local clone is at `4096af9` and clean.
- **APK** `App-Releases/4096af9-app-polish-mapbox-native-token/`: built from a clean worktree at
  `origin/App-Polish` with `.env`; DYX key `5eaacb96…`; the `mapbox_access_token` resource is in the APK.
- **Tablet `R5GL1016QFB`** updated to `4096af9` (from `2e75125`, data kept). On the router:
  - beacon listener up, rover found in 1.2 s;
  - the owner connected once (no saved token for this rover on that tablet yet);
  - map OK, 0 crashes.
  - From now on it auto-connects.
- **Tablet `R5GYA14C7CY`** still has `4891a3d` (works, auto-connect verified); update it to `4096af9` when next
  plugged in.
- The open list is unchanged otherwise (see "END OF DAY" 18:40); items 1–2 are now partly done: Mapbox decision
  done; the owner's hands-on test and the hotspot path are still pending.

## 2026-10-09 (19:00) — Claude — FIRST THING TOMORROW: prove the M1/M2 output mapping without the stick

Open safety question:
- Both of today's states (M1 = right with `FUNC1 101`; after the rewiring, M1 = left with `FUNC1 102`) were judged
  through manual driving with `RC1_REV −1`.
- `RC1_REV −1` came from logs 6/7 (the owner's stick-left read as roll +1); that direction was never physically
  confirmed.
- If it was wrong, the stick sign masks a mirrored output mapping: manual looks right, while Mission, Offboard and
  RPP turn the wrong way. Wheels-up logs cannot tell the two apart.

**Test (wheels up, spray off, disarmed; check the `actuator_test` help for the exact syntax first):**
1. `ver git`, `param show RBCLW_FUNC*`, `param show RC1_REV`.
2. Drive function 102 (left command → `RBCLW_FUNC1` → M1) at about +0.1 for 2 s: the **LEFT wheel must turn
   forward**.
3. Drive function 101 (right command → M2): the **RIGHT wheel must turn forward**.
4. During each, run `listener wheel_encoders`: `[0]` = M1, `[1]` = M2 signs.
- **Pass:** close the "residual sign dependency" item in `docs/Firmware/F-tasks.md`.
- **Fail:** set `RBCLW_FUNC1 101`, `RBCLW_FUNC2 102`, `RC1_REV +1`, then re-test the outputs and the stick.
  **No Mission/Offboard until it passes.**
- Better still, once wheels-down: in Manual, stick left must give a negative gyro yaw rate (left turn) in the log.

Also from the owner's note:
- `installer/pins/firmware.pin` still says `27a7ac9284`; the FCU runs `8279fa4be3`. The msgs are identical, but the
  pin selects the `px4_msgs` build, so change it deliberately (CI + upgrade).
- Stale `4393fb07e1` line in `docs/bench/2026-10-09_bench_runbook.md`.

## 2026-10-09 (19:10) — Claude — CORRECTION: motor mapping is mirrored; tomorrow's plan (owner)

**Correction to the 18:35 and 18:40 entries:**
- Owner, from Motion Studio: **RoboClaw M1 = RIGHT motor, M2 = LEFT motor.**
- The FCU has `RBCLW_FUNC1 102` / `RBCLW_FUNC2 101`, which sends the left command to the right wheel: **mirrored**.
- Manual only looks correct because `RC1_REV −1` inverts the stick too (two inversions cancel). Mission, Offboard
  and RPP have no stick in the path, so they would steer the wrong way and can spin on a heading correction.
- Today's sequence explains it: first the stick sign was "fixed", then the mapping was swapped after a reported
  rewiring. Each step only looked right in manual.
- `config/vehicle/roboclaw/README.md` is marked "under correction". The repo baseline still holds the rover's
  current (wrong) values until the test below proves the right ones.
- **No Mission/Offboard/RPP until step 1 passes.**

**Tomorrow, in this order (owner):**
1. **Motor output mapping and stick (wheels up, spray off, disarmed):**
   - a. Set `RBCLW_FUNC1 101`, `RBCLW_FUNC2 102`.
     - `actuator_test` on function 101 only (about 0.1 for 2 s): the **RIGHT wheel turns forward**.
     - Function 102: the **LEFT wheel forward**.
     - `listener wheel_encoders` signs agree (`[0]` = M1 = right).
     - Check the `actuator_test` help for the exact syntax first.
   - b. Live stick read: right stick fully RIGHT ⇒ `manual_control_setpoint.roll` **> 0**; set `RC1_REV` so that
     holds (expected `+1`). Left stick forward ⇒ throttle > 0.
   - c. Manual, wheels up, then wheels down: stick right turns right, stick left turns left, forward is forward.
     Wheels down, the log must show gyro yaw > 0 for a right turn.
   - d. Persist `RBCLW_FUNC*`, `RC1_REV` (baseline + `3wd_rover01_rc_calibration.params`), fix the README, and record
     the logs here.
2. **RTCM injection via USB to the UM982 (outdoors):** worker INJECTING, receiver GGA quality and correction age,
   PX4 fix type → RTK FLOAT/FIXED.
   - The NTRIP base for our mountpoint was **offline** today (not in the caster source table). Bring it back, or
     test with a working mountpoint. LoRa stays parked.
   - Also check the UM982 dual-antenna heading (UNIHEADINGA must leave `INSUFFICIENT_OBS`), so the EKF gets yaw and
     a global position.
3. **PX4 Mission mode, 4-point mission** (outdoors, open sky):
   - preconditions: step 1 passed; heading valid; EKF global position; RC kill switch tested;
   - watch upstream #27497 (a differential rover does not turn in Mission mode on v1.17);
   - log the run; check the turns, tracking and the stop at the end.
4. **Only if motor output, steering, heading and mission are all correct:** start the frontend and backend contract
   / app integration (Three_Wheel_v2 `App-Polish` `4096af9`; the cleanup list is in the 18:25 and 18:40 entries).

---

## 2026-10-09 — Codex — PX4 Mission differential rover #27497 investigation

**Scope and result:** Investigated firmware `dyx-3wd-production` at `8279fa4be3` and companion
`master` at `1bea449`. No firmware or parameter fix is validated. `make px4_fmu-v6x_rover`
passed (FLASH 1,825,036 B, 92.83%). The only differential SITL airframe is
`50000_gz_rover_differential`; the Gazebo submodule is uninitialized and `gz` is absent on
both macOS and the available PX4 container. SIH has only `10045_sihsim_rover_ackermann`.
No square mission, before/after metrics, or SITL ULogs exist. The six local 2026-10-09
ULogs in `PX4-Firmware/3WD/logs_2026-10-09/` only show `nav_state=0`, with no
`mission_result` sequence, and cannot establish Mission behavior.
`make px4_sitl_test unit-RoverControl` built the rover library test and
`./build/px4_sitl_test/unit-RoverControl --gtest_brief=1` passed **2/2** tests.

**Findings:** Upstream [#27497](https://github.com/PX4/PX4-Autopilot/issues/27497) was closed
2026-10-07 as unable to reproduce. The reporter corrected a reversed motor configuration,
but later described unresolved Mission pauses. No fixing PR is linked to the issue. In our
source, `mission.cpp:201-288` fills `position_setpoint_triplet`, `navigator_main.cpp:900-901`
publishes it, `DifferentialAutoMode.cpp:49-91` converts it to a rover waypoint, and
`DifferentialPosControl.cpp:59-108` selects driving or spot turning. Navigator acceptance
uses `NAV_ACC_RAD` (`mission_block.cpp:308-374`); `MIS_YAW_ERR` is only considered for
rotary-wing yaw (`mission_block.cpp:382-405`). The 0.05 m radius is a plausible source of
waypoint hunting under position noise, but that has not been measured. The rate output
depends on `RD_WHEEL_TRACK / RO_MAX_THR_SPEED` (`RoverControl.cpp:195-215`); the baseline
has 0.47 m and 1.28 m/s, so upstream's missing feedforward suggestion does not directly
apply. `RO_YAW_RATE_TH` also suppresses small commanded rates (`DifferentialRateControl.cpp:77-83`),
but there is no evidence that its 0.4 deg/s setting causes 90-degree turn failures.

**Upstream diff since v1.17.0:** `3ebb2923a1` / PR #28476 changes rover work scheduling
for SIH lockstep; `b07a720cfd` / PR #28840 explicitly zeros motors after disarm;
`6cf8d80bdd` / PR #26452 guards speed planning when decel/jerk limits are disabled;
`06dc5d0b5f` / PR #27894 corrects parameter descriptions. The remaining changes in
the rover control path are `78c5eafa85` / PR #28469 (parking mode),
`afa4170acb` (offboard timestamp), `2a2a44550a` (parameter format migration),
`95d6a3c171` / PR #27211 (parameter docs), and `ce3e62841f` / PR #26476
(module infrastructure). Navigator has broad later work, including
`07bac1389c` / PR #27571 (fixed-wing waypoint switch distance), but no #27497-linked
Mission differential fix. Our fork's rover diff from v1.17.0 is only `b19901b004`
(OFFBOARD velocity setpoint handling), which is outside the Mission path.

**Tomorrow's exact field configuration and gate:** Apply the existing rover navigation
baseline values unchanged: `NAV_ACC_RAD=0.05`, `MIS_YAW_ERR=5`,
`RD_TRANS_DRV_TRN=0.7`, `RD_TRANS_TRN_DRV=0.0349066`, `RD_WHEEL_TRACK=0.47`,
`RO_MAX_THR_SPEED=1.28`, `RO_YAW_RATE_I=0.01`, `RO_YAW_RATE_TH=0.4`; do not flash an
unvalidated Mission patch. Independently correct the known hardware mapping to
`RBCLW_FUNC1=101`, `RBCLW_FUNC2=102` and verify motor direction with wheels raised,
as specified in the preceding handoff entry. Confirm valid GNSS yaw, EKF global position,
and RC kill. Then run a guarded four-point ~10 m square in Mission mode, return to the
first point, and save a ULog. Record each waypoint's acceptance time, cross-track maximum,
corner yaw error, spot-turn/driving transitions and any back-and-forth behavior. Only after
the baseline run, compare `NAV_ACC_RAD=0.3` and `0.5` as temporary field trials if the owner
chooses; restore the baseline after each run until a value is validated. Re-validate any
accepted value in the field before changing the production parameter file.

**DERIVED — NOT FROM V1 SPEC:** `NAV_ACC_RAD=0.3` and `0.5` are diagnostic comparison
values from the task, not recommended production tuning. The simulator limitation prevents
the requested parameter and firmware A/B; controller behavior remains unproven.

## 2026-10-09 (late) — Claude — production code review complete; controlled task list for 2026-10-10

**Production review (rover stack):**
- The register is `docs/reviews/production/open_items.md`.
- 13 parts were reviewed:
  - by ChatGPT, prompt per part, CodeRabbit style;
  - by Opus for recorder and installer;
  - every finding then **verified against the code**;
  - then an **independent Opus cross-review of the hot path** (CR-1 rpp + geometry, CR-2 guard + px4_link,
    CR-3 gateway + backend relay).
- Totals: **1 CRITICAL, 22 HIGH, 59 MEDIUM, 67 LOW**, 3 rejected.
- Firmware rover path and tablet app: deferred to a later session.
- No code was changed today; the review is documentation only.

**Review facts that change how we operate the rover now (until fixed):**
- **In PX4 Mission mode the companion guard is NOT in the loop.** The app E-stop goes to the guard, which only
  gates OFFBOARD setpoints. During tomorrow's Mission test the stops are the **RC kill switch** and an RC/QGC
  mode change (Hold).
- Every STOP, including an E-stop, is ramped by PX4 at `RO_DECEL_LIM` 0.3 m/s² (XR-GPX-002): about 1.7 m from
  1 m/s, about 0.2 m from 0.35 m/s.
- `COM_OF_LOSS_T` is not in the baseline (default 1.0 s). On companion loss, PX4 holds the last setpoint 1.0 s,
  then disarms (`COM_OBL_RC_ACT` 7).
- `COM_RCL_EXCEPT` = 4: **RC loss triggers no failsafe in OFFBOARD** (X-011).
- **Do not while armed or in a run:**
  - `dyx3-upgrade` / `dyx3-rollback` (INS-001: no idle check);
  - restart `dyx3-backend` (GW-002: SIGPIPE can kill the whole control graph);
  - `SetOffboard(false)` (PXL-002: drops the stream with no STOP).
- **Spray stays OFF on the rover until XR-GPX-001 is fixed:** a spray OFF can wait up to 5 s behind an
  un-ACKed ON.

### Controlled task list — 2026-10-10 (owner's order; each step has a gate, do not skip ahead)

**T0 — Pre-flight (bench, 15 min)**
- Checks:
  - `dyx3-health --deep` OK;
  - FCU `ver git` = `8279fa4be3`;
  - DDS connected, timesync converged;
  - QGC on TCP `192.168.3.150:5760`;
  - battery ≤ 28.4 V;
  - spray hardware disconnected or OFF.
- Gate: all green. Abort: any FAIL → fix first.

**T1 — Motor output mapping and stick (wheels up, disarmed, spray off).** Detail in the 19:00 and 19:10 entries.
- Set `RBCLW_FUNC1 101`, `RBCLW_FUNC2 102`.
- `actuator_test`: function 101 → RIGHT wheel forward; function 102 → LEFT wheel forward.
  `listener wheel_encoders` signs agree.
- Stick right ⇒ `manual_control_setpoint.roll` > 0 → set `RC1_REV` (expected +1). Throttle forward > 0.
- Manual, wheels up, then down: right is right, forward is forward; the log shows gyro yaw > 0 on a right turn.
- **Gate:** all pass, logs saved. **Persist:** baseline params, `3wd_rover01_rc_calibration.params`, RoboClaw
  README, commit + push.
- **Abort:** any mirrored result → stop; nothing autonomous today.

**T2 — Safety parameters (owner decides values first; persist to the baseline file the same day)**
- `COM_OF_LOSS_T`: proposed **0.5** (PC-2c).
- `COM_RCL_EXCEPT`: owner choice (X-011). Recommended **0** for autonomy, so RC loss stops OFFBOARD and Mission.
- `RO_DECEL_LIM`: keep 0.3 for today; **measure** the braking distance in T4 (XR-GPX-002).
- PC-5 estimator noise: `EKF2_GPS_P_NOISE` 0.015, `EKF2_GPS_V_NOISE` 0.05, `RO_YAW_RATE_TH` 0.5 are the earlier
  recommendations. Change only with the owner's OK; log before and after.
- **RC kill switch test** (wheels up, armed): kill stops the outputs at once. **Gate:** works. **Abort:** fails →
  no ground motion.

**T3 — RTCM over USB to the UM982 + dual-antenna heading (outdoors, open sky)**
- RTK worker INJECTING; receiver GGA quality and correction age; PX4 fix type → RTK FLOAT / FIXED.
  - The NTRIP base was offline: bring it up or use a working mountpoint.
  - LoRa stays parked.
- UNIHEADINGA leaves `INSUFFICIENT_OBS`; EKF yaw valid; global position valid.
- **Never configure the UM982** (read-only CONFIG / UNILOGLIST only).
- **Gate:** RTK FIXED (or FLOAT, noted) + valid heading + EKF global position. **Abort:** no heading → no Mission.

**T4 — PX4 Mission mode, 4-point square (about 10 m sides), from QGC**
- Preconditions: T1–T3 passed; RC kill tested; an observer at the RC; open area; low speed (`RO_SPEED_LIM` as
  baseline); spray off.
- Keep `NAV_ACC_RAD` 0.05 (baseline). If it drives back and forth near a waypoint (#27497, F-tasks A1.2), stop
  (Hold) and trace from the ULog: do not change parameters in the field without a log.
- Record per corner: reached y/n, time, overshoot, heading at exit. Record the end stop and the stopping distance
  after Hold (for `RO_DECEL_LIM`).
- **Gate:** all 4 points reached and turns correct. **Abort:** a wrong turn direction or a spin → RC kill, back
  to T1.

**T5 — Only if T1–T4 pass: app integration start**
- Three_Wheel_v2 `App-Polish` `4096af9`.
- Cleanup list: the 18:25 and 18:40 entries.
- Contract: `docs/contracts/backend.md`.
- Pending owner decisions before production use: BE-001 (one controlling tablet vs any), GW-004 (tablet-loss
  budget), X-015 (plain HTTP on the site LAN).

**Desk track (Claude; code fixes, no rover needed; deploy only on request)**, in order:
1. **XR-GPX-001:** spray OFF pre-empts the in-flight ON; timeout about 0.3 s.
2. **GW-002:** `MSG_NOSIGNAL` + `SIGPIPE` ignored.
3. **X-010 / PXL-002:** `px4_link` STOP on exit and before offboard disable.
4. **XR-BE-001:** auth and size check before body parsing.
5. **INS-001 / 002 / 004 / 006 / 009:** installer interlock, re-exec, detach, baseline health, lock fd.
6. **XR-RPP-001 + XR-RPP-006:** precise-stop lateral fix; tests at command level.

Each fix: tests + `ros2_humble.sh build-test`, HANDOFF entry, commit `Agent: Claude`.

**Housekeeping:**
- delete `/home/flash/tablet-1.token` once both tablets have the token;
- remove the stale `4393fb07e1` line from `docs/bench/2026-10-09_bench_runbook.md`;
- the PC-9 pin bump is **blocked by INS-003** (do it via a fresh install or after the re-exec fix).
