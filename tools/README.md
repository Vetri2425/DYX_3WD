# tools

Desk-side tools: no ROS needed unless a tool says so. Field data (bags, ULogs) is never in Git. Run these tools on the Mac or
the Jetson against `3WD_PROD/Bags/<date>/`. Tests for these tools are in `tests/`. CI runs them (`.github/workflows/ci.yml`, job
`tools_tests`, Python 3.10, `pytest numpy rosbags`).

| Path | What it is |
|---|---|
| `analysis/mission_bag_metrics.py` | **Gate and field-ladder numbers from a recorder run or mission**: rates, chain latency, tracking (steady / corner entry / braking tail), pivot hunting, endpoint, mission/guard/RTK/estimator histograms, RPP-own full-run cross-track. Decodes `dyx3_interfaces` CDR from the `.msg` definitions of the recording stack (no ROS). Usage and metric definitions: `docs/analysis/README.md`. |
| `analysis/arc_floor.py`, `analysis/timebase.py` | Stage 0 tools (`docs/analysis/stage0.md`). |
| `analysis/param_divergence.py` | Compare `declare_parameter()` defaults between two PX4_DXP source trees. |
| `extract_geometry_bag_fixture.py`, `extract_legacy_decode_fixture.py` | Fixture extractors from PX4_DXP rosbag2 files (`rosbags`). |
| `gen_param_tables.py` | Generate the C++ parameter descriptor tables from `docs/tuning/parameter_registry.md` (`--check` in CI). |
| `gen_path_artifact_fixtures.py` | Verify the committed C++ path-artifact fixtures against the Python reader. |
| `gateway_smoke.py` | Smoke test of a running `dyx3_system_gateway` through the backend client. |
| `gate3/`, `gate4/` | GATE 3 / GATE 4 vector generators from the verbatim PX4_DXP code. |
| `px4_msg_hash/` | Test vectors from the firmware's own message-hash code (see its README). |
| `bench/` | Bench tools for the rover (NSH, Offboard sign test, RTK status, UM982 USB probe). |
| `dev/` | The reusable local ROS 2 Humble build environment (`docs/agents/LOCAL_ROS2_BUILD_ENV.md`). |
| `tests/` | pytest suite for the tools above. |

```bash
python3 -m pytest tools/tests -q
python3 tools/analysis/mission_bag_metrics.py 3WD_PROD/Bags/<date>/<run>
```
