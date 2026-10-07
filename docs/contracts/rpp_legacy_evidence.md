# Evidence note — the prototype's own tests on `build/demo-ready`

Run 2026-10-07 in `ros:humble-ros-base` (rclpy + `mavros_msgs` built from source for message definitions only):

| Run | Result |
|---|---|
| prototype tests that exercise the carried modules, **untouched PX4_DXP checkout** (`src/`) | **183 passed, 7 failed** |
| same tests against the byte-identical copy in `ros2_ws/src/dyx3_rpp_legacy/test/dxp_verbatim` | **183 passed, 7 failed** (identical set) |

Failing (identical in both):
`test_segment_endpoint_precise_stop.py::test_timeout_backstop_hands_to_completion_when_stopped`,
`test_terminal_approach_run_remaining.py::{test_brakes_while_still_on_the_mark_segment[0.8|0.6|0.4|0.25], test_long_final_segment_is_unchanged, test_off_switch_restores_the_previous_behaviour}`.

Interpretation (unverified): the terminal-approach tests assume the older retuned defaults (`a_lat_max 0.3`, `mission_speed 0.35`, `endpoint_approach_run_remaining True`); `demo-ready` carries
`a_lat_max 0.04`, `mission_speed 1.0`, `endpoint_approach_run_remaining False` (see `docs/tuning/default_divergence.md`). So the prototype's own suite is **red on the branch whose defaults the registry
copies**. Not carried (need spray modules): `test_point_handshake.py`, `test_point_handshake_spray.py`, `test_spray_pivot_gate.py`, `test_spray_rpp_boundary.py`.

Re-run: `DYX3_RUN_DXP_TESTS=1 pytest ros2_ws/src/dyx3_rpp_legacy/test/dxp_verbatim` with ROS + `mavros_msgs` sourced.
