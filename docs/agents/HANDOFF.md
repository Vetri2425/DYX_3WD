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
| `Way_to_Mark/PX4-Firmware/{3WD,4WD}/` | per-vehicle artifact archive |

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
