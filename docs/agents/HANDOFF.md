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
3. Classify all 174 parameters (120 RPP + 54 spray) as LIVE / IDLE_ONLY / RESTART.

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
