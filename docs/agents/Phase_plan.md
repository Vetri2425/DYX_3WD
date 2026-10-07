# DYX 3WD Companion Stack — Phase Map

Repository:

`~/Vetri/3WD_PROD/DYX_3WD`

This prompt defines the production companion-stack work phases.

Do NOT try to implement the whole stack at once.

When I give you a phase number, work ONLY inside that phase unless a dependency must be inspected for context.

Do not silently modify another phase.

Before coding, always read:

1. `docs/architecture/DYX_3WD_Production_Stack_Architecture_V1.md`
2. `CLAUDE.md`
3. `docs/agents/HANDOFF.md`
4. README of packages involved in the selected phase

Firmware is separate:

`Vetri2425/PX4-Autopilot-3WD-Prod`

Current production firmware:

`27a7ac92845317b0276776242c504215809b2a0f`

Do not modify firmware from this repository.

`px4_msgs` on the Jetson must be generated from this firmware SHA's `msg/` + `srv/`
(the firmware modifies `EstimatorAidSource3d.msg`; stock `px4_msgs` silently mismatches).

---

# Phase 0 — Stage 0: evidence and toolchain

Paths:

`tools/analysis/`, `tools/replay/`

Purpose (spec §11, Stage 0 — "days of desk work that protect months"):

- 0.1 quantify the arc-tracking payoff from the existing arc bags (`err ≈ ω/RO_YAW_P`)
- 0.2 **re-base the analysis toolchain to the DDS timebase** (§4.6) — every gate in both
  tracks is unevaluable without it
- 0.3 close the open baseline (`RO_YAW_RATE_TH` A/B, `RO_YAW_RATE_P` backfire, the three
  unexplained failures, the spray boundary gap)

No vehicle needed.

---

# Phase 1 — Interfaces / Contracts

Package:

`ros2_ws/src/dyx3_interfaces`

Purpose:

Freeze the canonical ROS 2 interfaces that every other subsystem uses.

Includes:

- `MotionSetpoint`
- `MotionSetpointStatus`
- `VehicleState`
- `RppStatus`
- `MissionState`
- `PointResult`
- `RtkStatus`
- `SprayState`
- `RecorderStatus`
- mission services
- `ExecuteMission.action`

Rules:

- interfaces freeze early
- avoid future ABI breaks
- document units and frames
- safe zero/default values
- no business logic here
- **frames and signs stated for every motion field:** yaw / yaw-rate in **NED** (0 = North,
  positive = clockwise, rad and rad/s — PX4 `RoverAttitudeSetpoint`/`RoverRateSetpoint` are NED);
  `speed_body_x` **signed** (negative = reverse, per the F1.7 contract row 16)
- every timestamp names its clock (node/ROS time, so replay with `use_sim_time` works)
- freeze is enforced: a gtest pins ABI constants and default values; CI interface-compatibility
  check (spec §7.1: field change ⇒ version bump + migration note)

Also in this phase (spec S1 deliverable):

- **classify all 174 parameters** (120 RPP + 54 spray) as `LIVE` / `IDLE_ONLY` / `RESTART`,
  each with one owner. Required before Phase 5 and Phase 9.

Current work:

`MotionSetpoint` is implemented locally (branch `vetri/m2-motion-setpoint`) and is
**under Claude review — not approved**. Open findings before freeze: yaw/yaw-rate frame and
sign (HIGH), signed `speed_body_x` / reverse (HIGH), default message must read as STOP+invalid,
`seq` reset on publisher restart, `stamp` clock, ABI gtest, stale commented CMake block.

Preferred model:
**Terra High** for interface design.
Luna for mechanical rosidl wiring/tests.

---

# Phase 2 — Backend / API

Path:

`backend/`

Purpose:

Operator/backend plane.

Includes:

- FastAPI
- Socket.IO
- tablet-facing API
- mission upload
- configuration API
- path-engine invocation
- mission/result storage
- backend ↔ `dyx3_system_gateway` boundary

Important:

Backend owns NO direct motion authority.

No `rclpy` in backend.

Backend failure must not directly create unsafe motion.

Preferred model:
**Luna** for normal API work.
**Terra High** for architecture/state-management changes.

---

# Phase 3 — Mission

Package:

`ros2_ws/src/dyx3_mission`

Purpose:

Mission and point execution state machine.

Includes:

- mission lifecycle
- point sequencing
- pause/resume
- abort
- skip
- run state
- point state
- completion/error handling

Mission decides WHAT should be executed.

It must not directly publish `/fmu/**`.

Preferred model:
**Terra High**

---

# Phase 4 — Trajectory / Geometry

Paths:

`backend/src/dyx3_backend/path_engine/`

and

`ros2_ws/src/dyx3_geometry`

Purpose:

Generate and represent the geometric path the rover must follow.

Backend path engine remains Python by architecture decision.

Includes:

- DXF/CAD ingestion
- CRS/geodesic conversion
- line/arc construction
- densification
- path ordering
- geometric primitives
- projections
- along-track / cross-track geometry
- trajectory/path artifacts

`dyx3_geometry`:

- pure C++
- NO ROS dependency
- shared mathematical geometry needed by control code

Requirements:

- recorded evidence for fixtures
- numerical equivalence where Python and C++ overlap
- no unnecessary rewrite of the Python CAD/path engine

Preferred model:
**Terra High** for algorithms.
**Luna** for repetitive tests/data plumbing.

---

# Phase 5 — Motion / RPP

Package:

`ros2_ws/src/dyx3_rpp`

Purpose:

Production path-following controller.

Main modules include:

- path conditioner
- guidance
- speed profile
- stop/pivot FSM
- terminal approach
- spray gate interaction

Input:

trajectory/path + vehicle state

Output:

canonical `MotionSetpoint`

Modes:

- STOP
- TRACK_HEADING
- TRACK_RATE
- PIVOT
- CREEP

RPP owns path-level rotational intent.

It does NOT publish directly to PX4.

Requirements:

- pure C++ algorithm modules
- `rclcpp` only at node boundary
- deterministic control loop
- no heap/logging syscall in hot path
- preserve precision behavior from verified field evidence

Preferred model:
**Terra High only** for core control logic.

---

# Phase 6 — Motion Guard / Safety

Package:

`ros2_ws/src/dyx3_motion_guard`

Purpose:

Final safety authority before motion reaches PX4.

Input:

`MotionSetpoint`

Output:

validated/clamped motion command

Responsibilities:

- mode validation
- finite/NaN contract validation
- sequence checks
- timestamp/freshness checks
- command limits
- state gating
- safety leases
- command timeout
- fail-to-zero
- reason/status reporting
- `seq` decrease = new publisher session; require fresh commands before accepting motion
- GNSS-heading health: monitor EKF2 `reject_yaw` and GNSS-yaw innovation test ratio; pause on
  unhealthy (HANDOFF 2026-10-07: a mostly-accepted heading fault can leave yaw wrong ~190 s
  with no flag)
- operator-link loss (§4.3.1): tablet heartbeat timeout from `dyx3_system_gateway` → controlled
  stop. PX4 cannot see it.

Non-negotiable:

Every invalid/fault path becomes:

`speed = 0`
`yaw_rate = 0`
`mode = STOP`

Never hold stale motion.

Preferred model:
**Terra High only**

---

# Phase 7 — PX4 Link / DDS

Package:

`ros2_ws/src/dyx3_px4_link`

Purpose:

The ONLY ROS package permitted to touch `/fmu/**`.

Production transport:

Jetson `10.41.10.1`
↔ Ethernet ↔
PX4 `10.41.10.2`

uXRCE-DDS agent:

UDP `8888`

Responsibilities:

- DDS/application lifecycle
- PX4 topic interface
- `px4_msgs` compatibility
- OFFBOARD heartbeat
- VehicleCommand
- arm/disarm
- mode control
- rover speed setpoint
- rover attitude/rate setpoints
- PX4 state → canonical `VehicleState`
- link freshness
- reconnect handling
- >0.5 s command-gap protection
- fail-to-zero
- **per-topic staleness**, not just session liveness (upstream #27388: the DDS client can
  silently stop publishing; only an FC reboot recovers)
- `px4_msgs` handshake against the firmware SHA — **fail loud** on mismatch (§4.6)
- F3 gate: survives FCU reboot, cable pull, agent restart and Jetson reboot unattended

No MAVROS.

No UART companion transport.

Preferred model:
**Terra High only**

---

# Phase 8 — GNSS / RTK

Package:

`ros2_ws/src/dyx3_gnss_rtk`

Purpose:

RTK correction delivery and correction-health monitoring.

Includes:

- NTRIP
- RTCM transport
- `/fmu/in/gps_inject_data`
- correction freshness
- correction-rate monitoring
- RTK status
- FIX transition monitoring

Hard rule:

PX4/companion must NOT configure the UM982 receiver.

Receiver configuration belongs to UM982 persistent memory.

Preferred model:
**Terra High** for transport/state logic.
**Luna** for diagnostics/tools.

---

# Phase 9 — Spray

Package:

`ros2_ws/src/dyx3_spray`

Purpose:

Real-time marking actuator control.

Includes:

- spray FSM
- geometric spray boundaries
- safety lease
- watchdog
- manual override
- RTK gate
- nozzle/flow model
- 54 spray parameters

Spray position is part of the precision product.

Existing known defect around projection/boundary continuity must be reproduced with evidence before fixing.

Preferred model:
**Terra High only**

---

# Phase 10 — Recorder / System Gateway

Packages:

`ros2_ws/src/dyx3_recorder`

`ros2_ws/src/dyx3_system_gateway`

Purpose:

Observability and backend boundary.

Recorder:

- ROS bags
- run manifests
- provenance
- firmware SHA
- stack SHA
- px4_msgs SHA
- parameter/config snapshots where available — FCU parameters are **not** readable while
  `MAV_2_CONFIG=0` (proposal 2026-10-07); record that explicitly
- ULog stream reassembly (`/fmu/out/ulog_stream` + ack) into the run directory (§5.4.3)
- result/report metadata

System Gateway:

- single ROS ↔ backend boundary
- converts internal ROS state/actions into backend-facing API
- backend never talks directly to motion/PX4 nodes

Preferred model:
**Luna** for most work.
**Terra High** for lifecycle/error-state design.

---

# Phase 11 — Bringup / Deployment / Installer

Paths:

`ros2_ws/src/dyx3_bringup`

`deployment/`

`installer/`

Purpose:

Turn the codebase into a reproducible production rover installation.

Includes:

- launch
- systemd
- `dyx3-platform`
- XRCE Agent process
- production directories
- permissions
- Ethernet configuration
- health checks
- install
- upgrade
- rollback
- version reporting

Target filesystem:

`/opt/dyx3/`
`/etc/dyx3/`
`/var/lib/dyx3/`
`/var/log/dyx3/`
`/run/dyx3/`

Operator commands:

- `dyx3-install`
- `dyx3-upgrade`
- `dyx3-rollback`
- `dyx3-health`
- `dyx3-version`

No manual production Jetson edits.

Preferred model:
**Terra High** for upgrade/rollback/systemd design.
**Luna** for scripts/tests/docs.

---

# Phase 12 — Legacy port-in / contract extraction (spec S2)

Paths:

`ros2_ws/src/dyx3_rpp_legacy` (quarantined, CLAUDE.md §8)

`docs/contracts/`

Purpose:

- carry the `PX4_DXP` controller in verbatim with only its output stage rewired to
  `MotionSetpoint` — the shadow-run oracle the C++ is validated against per tick
- write `docs/contracts/` from the Python source and the field bags; the contracts are the
  specification the C++ is built against
- unblocks Phases 4 (GATE 3 numeric equivalence), 5, 6 and 9

Quarantine: never in the production manifest, nothing depends on it, deleted at Gate 7.

Preferred model:
**Terra High**

---

# Development order

Use this order unless I explicitly override it:

`0 Stage 0` (desk work, parallel to everything)
→ `1 Interfaces` (+ parameter classification)
→ `11a Deployment — minimal slice`: dependencies, `dyx3` user + dirs, `dyx3-platform`
  (XRCE agent + mavlink-router), px4_msgs from the firmware pin, `dyx3-upgrade`.
  Needed early: the Mac → Git → Jetson → build → restart rule cannot run without it.
→ `12 Legacy port-in / contracts`
→ `4 Trajectory/Geometry`
→ `3 Mission`
→ `7 PX4 Link`
→ `6 Motion Guard` (built before the RPP it protects)
→ `5 Motion/RPP`
→ `8 GNSS/RTK`
→ `9 Spray`
→ `10 Recorder/Gateway`
→ `2 Backend`
→ `11 Deployment — remainder` (rollback, health, version, hotspot, LTE)

Some phases may proceed in parallel when independent.

Do not interpret this as permission to implement multiple phases in one task.

---

# Coding rule

When I say:

`Work on Phase N`

you must:

1. inspect that phase
2. identify the next incomplete logical item
3. implement only that item
4. keep the diff narrow
5. run available tests
6. report changed files and tests
7. stop for Claude review before merging

Branches: `codex/<topic>` for ChatGPT/Codex work, `claude/<topic>` for Claude
(CLAUDE.md §5). Never push to `master`.

Do not automatically start the next phase.

---

# Review workflow

Coder:
GPT-6 Luna or GPT-6 Terra High (ChatGPT).

Reviewer, deployer, installer:
Claude.

Claude reviews the actual local diff/commit.

One review pass unless it finds a real BLOCKER/HIGH defect.

Merge: Claude has human-equivalent authority (human decision 2026-10-07) and merges after
review passes and CI is green. The human can override any merge.

Deploy: after merge, Claude runs `dyx3-upgrade` on the Jetson (once Phase 11a exists).

---

# Non-negotiables

- fail to zero
- never hold stale motion
- production accuracy is the deliverable
- no MAVROS
- no companion UART
- Ethernet + DDS for PX4↔Jetson
- only `dyx3_px4_link` touches `/fmu/**`
- backend has no motion authority
- PX4 never configures UM982
- no invented tuning values
- recorded evidence for behavioral fixtures
- one logical change per commit
- no direct push to master
- never claim an unrun test passed