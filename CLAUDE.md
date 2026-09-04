# CLAUDE.md — DYX 3WD Production Stack

This file governs how AI agents work in this repository. Read it fully before making any
change. It applies to **all** agents, not only Claude.

---

## 1. Read order (every session, before any edit)

1. `docs/architecture/DYX_3WD_Production_Stack_Architecture_V1.md` — the specification.
2. `docs/Firmware/F-tasks.md` — firmware patch backlog, upstream blockers, the 39-commit
   carry-over audit. Read before touching anything firmware-adjacent.
3. This file — how to work in the repo.
4. `docs/agents/HANDOFF.md` — what the previous agent did and what is in flight.
5. The `README.md` of any package you are about to touch — it states that package's authority.

---

## 2. What this repository is

The clean production rewrite of the DYX 3WD **precision ground-marking rover** stack.

The rover paints lines on prepared surfaces — roads, aprons, car parks. **The painted line is
the product, so accuracy is not a quality attribute, it is the deliverable.**

Current baseline is **sub-2 cm, not sub-cm**: arc 1.46 / lshape 0.90 / square 0.87 / U-turn
1.06 cm @0.35 m/s; full-mission truth 1.69–1.91 cm; square is the only real sub-1 cm baseline;
circle never passed. Judge every change by whether it moves those numbers.

`PX4_DXP` is **read-only evidence**. Nothing is edited there. Behaviour is ported only after it
is verified against source and field bags, contract-tested, and re-implemented cleanly.
**No blind Python-to-C++ translation.**

Stack: tablet app → Python backend → C++ system gateway → C++ ROS 2 control graph → PX4 over
uXRCE-DDS. No MAVROS in the control path.

---

## 3. How this vehicle differs from DYX_4WD

Do not assume the 4WD rules transfer. Four differences are load-bearing:

| | 3WD | 4WD |
|---|---|---|
| **Wheel encoders** | **Primary aid.** `EKF2_WENC_CTRL=1`; the lever-arm fix is field-verified at 1.52 → 0.50 cm pivot wobble. | Explicitly deferred. |
| **Heading** | Dual-antenna UM982 GNSS heading is a first-class sensor with its own failure modes. | IMU + RTK position. |
| **Marking actuator** | Spray valve with **geometric boundary semantics** — where it opens is part of the accuracy spec. | None comparable. |
| **Path engine** | ~8 000 lines of CAD/DXF/CRS geometry. **Stays Python, in the backend** (spec 7.2). | C++ `dyx_trajectory`. |

Terrain is why the encoder decision differs: prepared surfaces here, loose soil there.
`dyx3_` is the package prefix. Never share interfaces with the 4WD stack without an explicit
human decision.

---

## 3b. Current status — 2026-09-05

**Milestone 1 complete. Skeleton only: no control logic, no firmware patches.**

| | |
|---|---|
| This repo | `Vetri2425/DYX_3WD` — public, branch `master`, **CI green (6/6)** |
| Firmware repo | `Vetri2425/PX4-Autopilot-3WD-Prod` — public, branch `dyx-3wd-production`, base PX4 **v1.17.0 == `d6f12ad1c4`**, first build green, pristine + CI only |
| Artifact archive | `Way_to_Mark/PX4-Firmware/3WD/<short-sha>-<slug>/` |
| Hardware | still CubeOrange+ / MAVROS in the field. **Nothing here has run on a rover.** |

**Green CI proves:** all 12 packages configure and build (including `dyx3_interfaces`
with rosidl and the lone `ament_python` package), `dyx3_geometry` builds and tests with
no ROS installation, the quarantine and hygiene gates work.
**Green CI does not prove:** any behaviour. There is none yet.

### What exists

- The V1 specification and `docs/Firmware/F-tasks.md`.
- 12 package skeletons with module file stubs — **empty namespaces, no targets**.
- Backend with a `/api/ping` endpoint and one test. Path engine directory is empty.
- 5 systemd units, installer skeleton, production manifest. **All non-functional stubs.**

### What is decided and must not be silently re-litigated

- Base pin `v1.17.0`, identical to the 4WD repo. Any divergence between vehicles must be
  our patches, never the base.
- Firmware patches are **real commits**, never `cp`-overlays. The new build already proves
  this works: the `.px4` records `git_identity = f3de5d1`, our commit — old fork builds all
  recorded base hash `54f0455f` regardless of content, which is why `ver_sw` could never
  identify a build.
- Transport and command interface migrate **together**. DDS while still publishing an XY
  velocity vector keeps the structural tracking floor and buys nothing.
- Gates are **acceptance gates, not start gates**: build anything at any time; accept
  nothing without its number.

### Immediate next steps

1. **Milestone 2** — freeze `dyx3_interfaces`, starting with `MotionSetpoint`.
2. Classify all 174 parameters (120 RPP + 54 spray) as LIVE / IDLE_ONLY / RESTART.
3. **F1.1** in `docs/Firmware/F-tasks.md` — RoboClaw drivetrain patches as semantic diffs
   against their v1.16.2 ancestors.
4. Stage 0.1 — quantify the arc-tracking payoff analytically from existing bags **before**
   committing further. It is days of desk work that validates or reshapes the programme.

### Known risks carried into this repo

- **Upstream #27514** (`risk:safety-critical`): PX4 applies a stale setpoint for ~900 ms
  after an external process dies — 31 cm at 0.35 m/s. Our fail-to-zero is not optional.
- **Upstream #27497**: rover differential does not turn in Mission Mode on v1.17 stable.
  Blocks the F4 gate as written.
- **Upstream #27388**: `uxrce_dds_client` silently stops publishing; only an FC reboot
  recovers it. Detect per-topic staleness, not just session liveness.
- `clang-format` is **unpinned** in CI. Local 23.1.0 and Ubuntu's apt version agree today;
  a runner image bump can fail the job on untouched code. Pin it before real C++ lands.

---

## 4. Rules that apply to every agent

**Never modify `docs/architecture/DYX_3WD_Production_Stack_Architecture_V1.md`** without an
explicit human decision. Propose changes in `docs/architecture/proposals/`.

**Flag ambiguity, do not resolve it silently.** Mark judgement calls inline:

```
// DERIVED — NOT FROM V1 SPEC: <what you assumed and why>
```

**Do not invent tuning values.** Gains, thresholds, timeouts and limits come from the
specification, from field evidence, or from a human. If none exists, leave the parameter
commented out with a note. A plausible-looking number in a production config is worse than an
absent one. The frozen corner-stop defaults are in spec section 9 — carry them verbatim, then
**re-validate**, because removing the ENU↔NED conversion silently re-interprets all 120 of them.

**Do not claim a build or test passed unless you ran it.** State what you ran, what you could
not run, and why. Inferred success is a failure of the report.

**One decision, one owner.** Duplicated authority is the primary failure mode this rewrite
exists to fix.

**Fixtures come from recorded evidence, never hand-written expectations.** Three bugs shipped in
`PX4_DXP` because a test's "truth" mirrored the bug. This is why `dyx3_geometry` has no ROS
dependency — it must be testable against real bag data in seconds.

**No backup files.** No `.bak`, `.backup`, `.before_*`, `_old`, `_v2`. CI rejects these.

**Never commit:** bags (`*.db3`, `*.mcap`, `*.ulg`), logs, `build/`, `install/`, `log/`,
NTRIP passwords, WiFi PSKs, SIM/APN credentials, machine tokens, runtime-generated missions.
**This repository is public.**

---

## 5. Branches and commits

```
claude/<topic>    codex/<topic>    agy/<topic>
```

Conventional Commits with a spec trailer:

```
feat(rpp): add cross-track error module

<body>

Agent: claude
Spec: Section 7.4
```

Never push directly to the default branch. Never force-push a shared branch.

---

## 6. Path ownership and review

| Path | May author | Review |
|---|---|---|
| `ros2_ws/src/dyx3_rpp/` | Claude, Codex | **Claude — SAFETY-CRITICAL** |
| `ros2_ws/src/dyx3_motion_guard/` | Claude | **Claude — SAFETY-CRITICAL** |
| `ros2_ws/src/dyx3_px4_link/` | Claude | **Claude — SAFETY-CRITICAL** |
| `ros2_ws/src/dyx3_spray/` | Claude, Codex | **Claude — actuator + boundary semantics** |
| `ros2_ws/src/dyx3_interfaces/` | Claude | **Claude — frozen after Milestone 2** |
| `ros2_ws/src/dyx3_geometry/` | Claude, Codex | Claude |
| `ros2_ws/src/dyx3_mission/`, `dyx3_gnss_rtk/` | Claude, Codex | Claude |
| `ros2_ws/src/dyx3_rpp_legacy/` | Claude | **Claude — see §8** |
| `ros2_ws/src/dyx3_recorder/`, `dyx3_bringup/`, `dyx3_system_gateway/` | Codex, Agy | any |
| `backend/` (incl. `path_engine/`) | Codex | Claude for `path_engine/` |
| `installer/`, `deployment/` | Agy, Codex | human |
| `config/` | Claude | **human — field-affecting** |
| `docs/` | any | any |

`config/` changes reach the rover. Treat them as hardware changes.

---

## 7. Code conventions

**Keep `rclcpp` out of algorithm modules.** In `dyx3_rpp`, files like `guidance.cpp`,
`speed_profile.cpp`, `stop_pivot_fsm.cpp`, `terminal.cpp` are pure C++ on plain structs, no ROS
includes. Only `rpp_node.cpp` touches ROS. `dyx3_geometry` has no ROS dependency at all.

**Every tunable is classified** `LIVE`, `IDLE_ONLY` or `RESTART` in its descriptor, validated in
the callback, and recorded on change. No unvalidated value reaches a control loop. Tuning by
restart is not merely inconvenient here — a restart mid-mission in OFFBOARD **aborts the run**.

**Fail to zero.** Every failure path ends at `speed = 0`, `yaw_rate = 0`, `mode = STOP`, with a
recorded reason code. Never fail to "hold last command" — see `docs/Firmware/F-tasks.md` A1.1,
where PX4 itself holds a stale setpoint for ~900 ms.

**Real-time discipline** in `dyx3_rpp` and `dyx3_motion_guard`: `mlockall`, no heap allocation
in the control loop, no logging syscalls on the hot path, DDS QoS declared per topic.

C++17. `clang-format` per the repo file. Tests use `ament_cmake_gtest`.

---

## 8. `dyx3_rpp_legacy` — the quarantine

The single permitted Python package in the control graph. It is the `PX4_DXP` controller carried
in verbatim with only its output stage rewired, and it exists for two reasons: it delivers the
precision win months before the C++ port finishes, and it is the **shadow-run oracle** the C++
modules are validated against per tick.

Quarantine terms, enforced by the `legacy_quarantine` CI job:

- no other `ament_python` package may exist in `ros2_ws/src`
- it must not appear in `installer/manifests/production.manifest`
- no package may declare a dependency on it
- **it is deleted when Gate 7 passes** — a tracked deliverable, not a cleanup task

---

## 9. Build and test

```bash
# ROS workspace
cd ros2_ws && colcon build --symlink-install && colcon test && colcon test-result --verbose

# Pure geometry, no ROS at all
cmake -S ros2_ws/src/dyx3_geometry -B build/geom_native -DDYX3_NATIVE_TESTS=ON
cmake --build build/geom_native && ctest --test-dir build/geom_native

# Backend + path engine
pip install -e "backend[dev,path-engine]" && ruff check backend/src backend/tests && pytest backend/tests
```

**Authoritative:** CI on `ubuntu-24.04-arm`, matching the Jetson's architecture.

**Not testable off-target, ever:** timing and latency figures, DDS transport to PX4, systemd
behaviour, the installer, udev, network configuration, and anything requiring RTK fix. Do not
report these as verified from a laptop.

---

## 10. Hardware and firmware

The rover has **no router**. The Jetson is the access point (hotspot + BLE), with a USB LTE
dongle for WAN/NTRIP and a direct Ethernet cable to the FCU. Addressing is fixed in the
architecture document — do not invent IP addresses.

⚠ **A tablet dropout is not a PX4 datalink loss.** The Jetson still talks to PX4, so
`NAV_DLL_ACT` never fires. The Jetson owns that failsafe itself.

Firmware lives in `Vetri2425/PX4-Autopilot-3WD-Prod` (base v1.17.0 == `d6f12ad1c4`). Never edit
firmware from this repository. Never `cp`-overlay firmware files — patches are real commits.

---

## 11. Handoff

Before ending a session, append to `docs/agents/HANDOFF.md`: what you changed and the branch,
what you ran and what you could not run, every `DERIVED — NOT FROM V1 SPEC` decision, what is
half-finished, and open questions for the human. The next agent may be a different model with no
memory of your reasoning.

---

## 12. Never

- Modify the frozen V1 architecture document without a human decision
- Touch `PX4_DXP/`, `Vetri/PX4-Autopilot/`, or any firmware tree from this repo
- Add MAVROS to the control path
- Put safety authority in the backend or the tablet
- Ship `dyx3_rpp_legacy` to a production rover
- Report an unrun build as passing
- Commit a credential — this repository is public
