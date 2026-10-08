# DYX_3WD — Production Stack

Clean production rewrite of the DYX 3WD **precision ground-marking rover** software stack.

| | |
|---|---|
| **Vehicle** | 3-wheel differential marking rover, Pixhawk 6X + Jetson Orin |
| **Specification** | [`docs/architecture/DYX_3WD_Production_Stack_Architecture_V1.md`](docs/architecture/DYX_3WD_Production_Stack_Architecture_V1.md) |
| **Firmware backlog** | [`docs/Firmware/F-tasks.md`](docs/Firmware/F-tasks.md) |
| **Firmware repo** | [`Vetri2425/PX4-Autopilot-3WD-Prod`](https://github.com/Vetri2425/PX4-Autopilot-3WD-Prod) — PX4 v1.17.0 (`d6f12ad1c4`) |
| **Evidence source** | `PX4_DXP` — **read-only** |
| **Agent rules** | [`CLAUDE.md`](CLAUDE.md) |

## Layout

```
ros2_ws/src/     12 packages — 11 C++ (ament_cmake) + 1 quarantined Python oracle
backend/         FastAPI + Socket.IO + the CAD/CRS path engine. No rclpy.
config/          runtime profiles, grouped by owning package
deployment/      systemd units, network, udev
installer/       install / upgrade / verify / rollback
firmware/        overlay-to-rebase migration notes, param baselines
tools/           analysis, replay, field, migration
docs/            architecture, contracts, interfaces, safety, validation
```

## Packages

| Package | Authority |
|---|---|
| `dyx3_interfaces` | Shared msg/srv/action. `MotionSetpoint` is the single canonical command. |
| `dyx3_geometry` | Pure C++, **no ROS dependency**. Testable on a laptop in seconds. |
| `dyx3_mission` | Which target and segment is active. |
| `dyx3_rpp` | How the rover follows the active path. |
| `dyx3_rpp_legacy` | ⚠ Quarantined Python shadow oracle. **Deleted at Gate 7.** |
| `dyx3_motion_guard` | Is the command safe, fresh, valid, in limits? Fails to zero. |
| `dyx3_px4_link` | The only package permitted to touch `/fmu/**`. |
| `dyx3_gnss_rtk` | NTRIP, RTCM, correction health. Its own service, never a child of the backend. |
| `dyx3_spray` | Marking actuator with geometric boundary semantics. |
| `dyx3_recorder` | Field evidence. A run without provenance is not evidence. |
| `dyx3_system_gateway` | The single ROS ↔ backend boundary. |
| `dyx3_bringup` | The only production launch authority. |

## Build

```bash
cd ros2_ws && colcon build --symlink-install && colcon test
cmake -S ros2_ws/src/dyx3_geometry -B build/geom_native -DDYX3_NATIVE_TESTS=ON && ctest --test-dir build/geom_native
pip install -e "backend[dev,path-engine]" && pytest backend/tests
```

CI on `ubuntu-24.04-arm` is authoritative — it matches the Jetson's architecture.

## Status

Software for the full control graph is implemented and tested off-target. **Nothing here has run on a rover**
(hardware is still CubeOrange+/MAVROS in the field).

| | |
|---|---|
| Branch | `claude/cloud-phases` (integrates the Cloud Review fixes, phases A–F); not merged to `master` |
| CI | run `37766642047` on `9549679`: all 9 jobs green; ROS 2 Humble `colcon`: 12 packages, 440 tests, 0 failures, 2 skipped |
| Cloud Review | **[`docs/agents/CLOUD_REVIEW_STATUS.md`](docs/agents/CLOUD_REVIEW_STATUS.md)** — per-finding status, evidence and blockers |
| Local ROS 2 | `./tools/dev/ros2_humble.sh build-test` — [`docs/agents/LOCAL_ROS2_BUILD_ENV.md`](docs/agents/LOCAL_ROS2_BUILD_ENV.md) |

**Not paint-ready.** C3 (the valve has no independent close path if the DDS link dies) is open and needs a
bench; GATE 1 PX4 bench, Jetson timing, real-bag replay, RTK and field accuracy validation are all open.
Green CI proves the software builds and the off-target tests pass, not hardware behaviour.
The production RTK architecture (E4) is a separate plan: `docs/plans/2026-10-08_production_rtk_plan.md`.

Accuracy baseline is sub-2 cm, not sub-cm (see `CLAUDE.md` §2).
