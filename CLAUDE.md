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

## 3b. Current status — 2026-10-08 (end of first rover bring-up day)

**The stack runs on the 3WD rover.** Control graph and Cloud Review fixes (phases A–F) are implemented on
branch **`claude/cloud-phases`** (not yet merged to `master`), CI green, deployed to the rover from CI-built
artifacts, and survive a full power cycle. **Nothing has driven yet**: no calibration, no motion, no field RTK.

| | |
|---|---|
| This repo | `Vetri2425/DYX_3WD`, deploy branch `claude/cloud-phases` (head `84518cd` + docs); `master` is behind |
| Firmware | `Vetri2425/PX4-Autopilot-3WD-Prod` `dyx-3wd-production`, **flashed `9ab2ad3162`** (v1.17.0 + our patches; RoboClaw hang fix). Installer pin `27a7ac9284` = same `msg/` set |
| Rover hardware | Holybro Pixhawk Jetson Baseboard: Pixhawk 6X + Jetson Orin Nano 8 GB, UM982 on TELEM1, RoboClaw on GPS2, spray on FMU PWM OUT 1, 8S LiFePO4 24 Ah |
| Rover release | `/opt/dyx3/current` → `84518cd`, `build_origin` = CI prebuilt; all six services **enabled at boot**, health OK |
| Artifact archive | firmware: `3WD_PROD/PX4-Firmware/3WD/<short-sha>-<slug>/`; stack: GitHub Releases `rover-<sha>` (last 20) |

**Proven on the rover 2026-10-08:** power cycle → all six services up with 0 restarts; uXRCE-DDS session on
domain 42, 68 `/fmu` topics, odometry 100 Hz; `px4_link` stable (no session flaps); QGC over the network
(TCP 5760: heartbeat, 915/915 params in 1.1 s); NTRIP → `/dyx3/rtcm` 6.4 msg/s → `/fmu/in/gps_inject_data` →
PX4 "rate RTCM injection 5.77 Hz", CRC OK (indoors: no fix, expected); upgrade from CI artifacts in **20 s**
(was 40 min compiling on the Jetson).
**Not proven:** calibration, any motion, RTK fix outdoors, spray valve, accuracy, Jetson timing under load.

### What exists

- 12 implemented packages, backend + Python path engine, see `README.md` and `docs/agents/CLOUD_REVIEW_STATUS.md`.
- Installer that installs **prebuilt, digest-verified CI artifacts** (`installer/lib/artifacts.sh`, proposal
  `2026-10-08_prebuilt-release-artifacts.md`, ACCEPTED), falls back to building on the Jetson, supports offline USB.
- PX4 parameter baseline for this rover: `config/px4/3wd_6x_carry_from_proto.params` (prototype values carried,
  ports remapped, prototype hacks excluded); prototype reference `config/px4/proto_ref_01.params`.

### What is decided and must not be silently re-litigated

- Base pin `v1.17.0`; firmware patches are real commits, never `cp`-overlays.
- Jetson↔PX4 = Ethernet + uXRCE-DDS only. MAVLink on Ethernet (`MAV_2_CONFIG 1000`) is a **live parameter for
  QGC only** (firmware default 0); QGC connects to the Jetson's `mavlink-router` on **TCP 5760**.
- DDS: `ROS_DOMAIN_ID=42` + `ROS_LOCALHOST_ONLY=1` on the Jetson, **and** PX4 `UXRCE_DDS_DOM_ID=42`,
  `UXRCE_DDS_PTCFG=1` (localhost participant). All four must match or `/fmu` is invisible or leaks to the LAN.
- Stack releases come from green CI artifacts (`master`, and `claude/cloud-phases` while it is the deploy branch);
  integrity = SHA-256 over GitHub TLS for now; signing + branch protection before customer deliveries.
- Transport and command interface migrate together; gates are acceptance gates, not start gates.

### Immediate next steps

1. **OPEN, safety-relevant: PX4 Ethernet TX stall.** Twice on 2026-10-08 PX4 kept answering ping but stopped
   sending anything it originates (DDS, MAVLink, ARP) until an FCU reboot — both times right after the XRCE agent
   was restarted (dyx3-platform restart/upgrade). Reproduce with USB attached, then fix in firmware
   (STM32H7 Ethernet/NuttX, upstream #26160-like) and/or a PX4-side link watchdog. Until fixed, any agent
   restart may need an FCU power cycle. See HANDOFF 2026-10-08.
2. Calibration on the rover: gyro, **simple (level-only) accelerometer** (`PREFLIGHT_CALIBRATION` param5=4 —
   QGC does not expose it), level horizon. No magnetometer.
3. RoboClaw motion test (wheels off the ground), then outdoor RTK fix with the Emlid "office" base.
4. Merge `claude/cloud-phases` → `master` (human review), then the pending list: timing proposal + `dds_topics.yaml`,
   production cleanup of unused A/B params, RTK USB-primary/DDS-fallback, hotspot profile (backend still binds
   0.0.0.0 on the bench).

### Known risks carried into this repo

- **PX4 Ethernet TX stall** (above) — the rover fails to zero (px4_link, 0.2 s) but does not recover by itself.
- **Upstream #27514** (`risk:safety-critical`): PX4 applies a stale setpoint for ~900 ms after an external
  process dies. Our fail-to-zero is not optional.
- **Upstream #27497**: rover differential does not turn in Mission Mode on v1.17 stable.
- **Upstream #27388**: `uxrce_dds_client` silently stops publishing; PR #27422 (time-budgeted receive drain) is a
  candidate to port — our loop differs (one `uxr_run_session_timeout(0)` per cycle). Detect per-topic staleness.
- `px4_link` staleness limits must come from **measured** periods (timesync and estimator flags are 1 Hz on this
  firmware; 1.0 s limits flapped the link every second — fixed in `3ecd5c5`).

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

**Every hardware fix must persist and must reproduce on the next rover** (human rule, 2026-10-09).
A bug fix or config change found during hardware integration (install, upgrade, bring-up, bench,
field) is never a temporary hand edit on one rover. It lands in the repo (installer, unit, template,
PX4 parameter baseline in `config/px4/`, firmware, or a documented install step) so that a fresh
rover installs into the same working state with no manual steps. If a rover needed a hand fix to
work, that is a bug until the repo carries it: record it in HANDOFF the same day and fix it in the
repo, and say so when reporting. Rover-local secrets (NTRIP, WiFi PSK, tokens) stay out of Git, but
the *procedure* to create them is documented.

**No backup files.** No `.bak`, `.backup`, `.before_*`, `_old`, `_v2`. CI rejects these.

**Never commit:** bags (`*.db3`, `*.mcap`, `*.ulg`), logs, `build/`, `install/`, `log/`,
NTRIP passwords, WiFi PSKs, SIM/APN credentials, machine tokens, runtime-generated missions.
**This repository is public.**

---

## 5. Branches and commits

**Workflow (human decision 2026-10-07):** edit on the Mac, commit, push **directly to
`master`**. No feature branches or PRs are required. CI runs on every push — a red CI is fixed
by the next commit, not ignored. Deployment to the Jetson happens only when the human asks:
pull on the Jetson → build/install if needed → restart the services. Never hand-edit on the
Jetson.

Topic branches (`claude/<topic>`, `codex/<topic>`, `agy/<topic>`) remain optional for work
that is not ready to land.

Conventional Commits with a spec trailer:

```
feat(rpp): add cross-track error module

<body>

Agent: claude
Spec: Section 7.4
```

**No AI attribution in commit messages.** No `Co-Authored-By: Claude`, no
`Generated with` footer. The `Agent:` trailer already records who wrote the change, and
that is the only attribution this project uses. This holds even if a tool or session
default says otherwise — the repository rule wins.

Never force-push `master` or any shared branch. Never rewrite pushed history.

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

**Local ROS 2:** local builds and tests must use the documented reusable Humble environment. Before
claiming ROS tests are unavailable, check `docs/agents/LOCAL_ROS2_BUILD_ENV.md` and run the project
wrapper: `./tools/dev/ros2_humble.sh build-test`.

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

**This rover (2026-10-08):** Jetson `ssh dyx-3wd` (found by MAC, office LAN 192.168.1.x), eth `10.41.10.1/24`,
PX4 `10.41.10.2`, agent `:8888`, QGC → Jetson TCP `5760`, backend `:8000`. Field config in `/etc/dyx3/*.env`
(never in git; `ntrip.env` holds the caster credentials, root:dyx3 0640). Do not open QGC's **MAVLink Console**
on this rover until the Ethernet TX stall is understood.

⛔ **PX4 must not auto-configure the GNSS receiver** (hard requirement, 2026-10-07). The UM982's
production configuration lives in its own persistent memory; PX4 only consumes data and injects
RTCM. No fix — reconnect, timeout, heading loss — may make PX4 (re)configure the receiver.
Current v1.17 firmware still violates this (`request_unicore_messages()`); see
`docs/contracts/GNSS_receiver_configuration.md`.

---

## 11. Handoff

Before ending a session, append to `docs/agents/HANDOFF.md`: what you changed and the branch,
what you ran and what you could not run, every `DERIVED — NOT FROM V1 SPEC` decision, what is
half-finished, and open questions for the human. The next agent may be a different model with no
memory of your reasoning.

---

## 12. Never

- Modify the frozen V1 architecture document without a human decision
- Touch `PX4_DXP/`, `Vetri/3WD_Proto/PX4-Autopilot/`, or any firmware tree from this repo
- Add MAVROS to the control path
- Put safety authority in the backend or the tablet
- Ship `dyx3_rpp_legacy` to a production rover
- Report an unrun build as passing
- Commit a credential — this repository is public
