"""Regenerate the C++ cross-check fixture from a fixed app-plan payload."""

from pathlib import Path

from dyx3_backend.mission.app_plan import compile_plan

PAYLOAD = {
    "client": "Three_Wheel_v2", "client_version": "1.0.0", "name": "run_boundary_cross_check",
    "frame": "ekf_local_ned", "runs": [
        {"type": "travel", "points": [[0, 0, 2], [1, 0, 0]]},
        {"type": "mark", "points": [[1, 0, 3], [2, 0, 3], [2, 1, 3]]},
        {"type": "travel", "points": [[2, 1, 2], [3, 1, 0]]},
    ],
}
FIXTURE = Path(__file__).resolve().parents[2] / "ros2_ws/src/dyx3_rpp/test/fixtures/app_plan_boundary.dyx3path"


if __name__ == "__main__":
    FIXTURE.write_bytes(compile_plan(PAYLOAD))
