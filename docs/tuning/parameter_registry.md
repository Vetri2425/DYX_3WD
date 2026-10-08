# Parameter registry

**Status:** Phase 1 source inventory + **PROPOSED** mutability classes (2026-10-07). The
classes below are a proposal for human review; they are the Claude-cloud author's reading of
spec §9, not an approved decision. Where the evidence genuinely does not settle a class the
row says `TBD — human`.

## Count method

This registry contains **173** parameters: **119 RPP + 54 spray**. The count is the number of
executable `declare_parameter(` calls in the read-only prototype; comment-only occurrences are
excluded. In particular, the comment at `PX4_DXP/src/rpp_controller_node.py:679` is not a
parameter. The historical 120/174 discrepancy is recorded in
`docs/architecture/proposals/2026-10-07_parameter-count-173.md`.

Current defaults are copied verbatim from the prototype. They are evidence, not approval for
the NED production control path, and must be re-validated at **GATE 4** (spec §9 / §11): the
prototype tuned all of them in the ENU frame, and removing the ENU↔NED conversion silently
re-interprets them.

## Classes (spec §9)

| Class | Changeable | Rule used here |
|---|---|---|
| `LIVE` | while driving | gains, lookahead, speed / yaw-rate limits, terminal and pivot thresholds |
| `IDLE_ONLY` | disarmed, no mission | mode/feature toggles, geometry, calibration, anything that moves a spray boundary, safety gates and freshness bounds, path-conditioning (applied once at mission install) |
| `RESTART` | never at runtime | identifiers, backend/transport selection, supervised-process configuration |

An `IDLE_ONLY` change during a mission is rejected with a reason, never deferred (spec §9).

### Principles behind the proposal

1. **Safety gates may be tightened but not relaxed while moving** — all RTK / accuracy /
   freshness bounds are `IDLE_ONLY`. (The authoritative runtime copy lives in
   `dyx3_motion_guard`; the RPP/spray copies exist only because the prototype duplicated them.)
2. **Spray boundary semantics are part of the accuracy spec** — everything that moves where the
   valve opens or closes, or what paint it applies, is `IDLE_ONLY`. Moving a handful of these
   (e.g. the cross-track gate thresholds) to `LIVE` for field tuning is a one-word change if the
   human wants it; the conservative reading was chosen.
3. **Path-conditioning parameters** act when the mission is installed; changing them later
   leaves already-installed runs inconsistent, so they are `IDLE_ONLY`.
4. **Feature toggles** that enable/disable an FSM branch are `IDLE_ONLY` (spec §9 "mode defaults").
5. Each class must be enforced in the node's parameter callback and recorded on change.

Owner: the package in the `Owner` column is the single owner (CLAUDE.md "one decision, one owner").

| Name | Type | Current default | Source line | PROPOSED class | Rationale | Owner package |
|---|---|---|---|---|---|---|
| `max_linear_vel` | float | `1.0` | PX4_DXP/src/rpp_controller_node.py:285 | **LIVE** | speed limit; spec §9 lists speed limits as LIVE | dyx3_rpp |
| `min_linear_vel` | float | `0.15` | PX4_DXP/src/rpp_controller_node.py:286 | **LIVE** | speed limit; spec §9 lists speed limits as LIVE | dyx3_rpp |
| `min_lookahead_dist` | float | `0.52` | PX4_DXP/src/rpp_controller_node.py:299 | **LIVE** | guidance gain/lookahead; spec §9 lists lookahead and gains as LIVE | dyx3_rpp |
| `max_lookahead_dist` | float | `1.0` | PX4_DXP/src/rpp_controller_node.py:300 | **LIVE** | guidance gain/lookahead; spec §9 lists lookahead and gains as LIVE | dyx3_rpp |
| `lookahead_time` | float | `1.6` | PX4_DXP/src/rpp_controller_node.py:303 | **LIVE** | guidance gain/lookahead; spec §9 lists lookahead and gains as LIVE | dyx3_rpp |
| `a_lat_max` | float | `0.04` | PX4_DXP/src/rpp_controller_node.py:311 | **LIVE** | guidance gain/lookahead; spec §9 lists lookahead and gains as LIVE | dyx3_rpp |
| `regulated_linear_scaling_min_speed` | float | `0.3` | PX4_DXP/src/rpp_controller_node.py:312 | **LIVE** | guidance gain/lookahead; spec §9 lists lookahead and gains as LIVE | dyx3_rpp |
| `xy_goal_tolerance` | float | `0.02` | PX4_DXP/src/rpp_controller_node.py:315 | **LIVE** | terminal/capture threshold; spec §9 lists terminal thresholds as LIVE | dyx3_rpp |
| `min_goal_travel_m` | float | `0.5` | PX4_DXP/src/rpp_controller_node.py:319 | **IDLE_ONLY** | path conditioning is applied once at mission install; a mid-run change would leave the installed runs inconsistent with the thresholds | dyx3_rpp |
| `close_loop_threshold_m` | float | `0.15` | PX4_DXP/src/rpp_controller_node.py:331 | **IDLE_ONLY** | path conditioning is applied once at mission install; a mid-run change would leave the installed runs inconsistent with the thresholds | dyx3_rpp |
| `close_loop_min_len_m` | float | `1.0` | PX4_DXP/src/rpp_controller_node.py:332 | **IDLE_ONLY** | path conditioning is applied once at mission install; a mid-run change would leave the installed runs inconsistent with the thresholds | dyx3_rpp |
| `closed_loop_min_travel_frac` | float | `0.9` | PX4_DXP/src/rpp_controller_node.py:333 | **LIVE** | terminal/capture threshold; spec §9 lists terminal thresholds as LIVE | dyx3_rpp |
| `approach_velocity_scaling_dist` | float | `0.6` | PX4_DXP/src/rpp_controller_node.py:337 | **LIVE** | terminal/capture threshold; spec §9 lists terminal thresholds as LIVE | dyx3_rpp |
| `min_approach_linear_velocity` | float | `0.1` | PX4_DXP/src/rpp_controller_node.py:338 | **LIVE** | terminal/capture threshold; spec §9 lists terminal thresholds as LIVE | dyx3_rpp |
| `endpoint_approach_run_remaining` | bool | `False` | PX4_DXP/src/rpp_controller_node.py:386 | **IDLE_ONLY** | feature/mode toggle for an FSM branch; spec §9 "mode defaults" are IDLE_ONLY — flipping a branch mid-run changes the state machine under it | dyx3_rpp |
| `transit_merge_max_len_m` | float | `2.0` | PX4_DXP/src/rpp_controller_node.py:387 | **IDLE_ONLY** | path conditioning is applied once at mission install; a mid-run change would leave the installed runs inconsistent with the thresholds | dyx3_rpp |
| `transit_runout_goal_tolerance_m` | float | `0.10` | PX4_DXP/src/rpp_controller_node.py:388 | **LIVE** | terminal/capture threshold; spec §9 lists terminal thresholds as LIVE | dyx3_rpp |
| `transit_runout_min_speed_m_s` | float | `0.10` | PX4_DXP/src/rpp_controller_node.py:389 | **LIVE** | terminal/capture threshold; spec §9 lists terminal thresholds as LIVE | dyx3_rpp |
| `endpoint_capture_recover_enabled` | bool | `True` | PX4_DXP/src/rpp_controller_node.py:432 | **IDLE_ONLY** | feature/mode toggle for an FSM branch; spec §9 "mode defaults" are IDLE_ONLY — flipping a branch mid-run changes the state machine under it | dyx3_rpp |
| `endpoint_capture_past_m` | float | `0.02` | PX4_DXP/src/rpp_controller_node.py:433 | **LIVE** | terminal/capture threshold; spec §9 lists terminal thresholds as LIVE | dyx3_rpp |
| `endpoint_capture_max_miss_m` | float | `0.10` | PX4_DXP/src/rpp_controller_node.py:434 | **LIVE** | terminal/capture threshold; spec §9 lists terminal thresholds as LIVE | dyx3_rpp |
| `segment_precise_endpoint_stop_enabled` | bool | `True` | PX4_DXP/src/rpp_controller_node.py:442 | **IDLE_ONLY** | feature/mode toggle for an FSM branch; spec §9 "mode defaults" are IDLE_ONLY — flipping a branch mid-run changes the state machine under it | dyx3_rpp |
| `segment_endpoint_arrival_tolerance_m` | float | `0.02` | PX4_DXP/src/rpp_controller_node.py:443 | **LIVE** | terminal/capture threshold; spec §9 lists terminal thresholds as LIVE | dyx3_rpp |
| `segment_endpoint_cross_tolerance_m` | float | `0.02` | PX4_DXP/src/rpp_controller_node.py:444 | **LIVE** | terminal/capture threshold; spec §9 lists terminal thresholds as LIVE | dyx3_rpp |
| `segment_endpoint_max_correction_m` | float | `0.15` | PX4_DXP/src/rpp_controller_node.py:445 | **LIVE** | terminal/capture threshold; spec §9 lists terminal thresholds as LIVE | dyx3_rpp |
| `segment_endpoint_precise_decel_m_s2` | float | `0.35` | PX4_DXP/src/rpp_controller_node.py:446 | **LIVE** | terminal/capture threshold; spec §9 lists terminal thresholds as LIVE | dyx3_rpp |
| `segment_endpoint_trigger_margin_m` | float | `0.10` | PX4_DXP/src/rpp_controller_node.py:447 | **LIVE** | terminal/capture threshold; spec §9 lists terminal thresholds as LIVE | dyx3_rpp |
| `segment_endpoint_creep_speed` | float | `0.10` | PX4_DXP/src/rpp_controller_node.py:448 | **LIVE** | terminal/capture threshold; spec §9 lists terminal thresholds as LIVE | dyx3_rpp |
| `segment_endpoint_precise_max_s` | float | `8.0` | PX4_DXP/src/rpp_controller_node.py:449 | **LIVE** | terminal/capture threshold; spec §9 lists terminal thresholds as LIVE | dyx3_rpp |
| `p4_zero_vel_threshold` | float | `0.02` | PX4_DXP/src/rpp_controller_node.py:450 | **LIVE** | terminal/capture threshold; spec §9 lists terminal thresholds as LIVE | dyx3_rpp |
| `pose_max_age_s` | float | `0.5` | PX4_DXP/src/rpp_controller_node.py:453 | **IDLE_ONLY** | safety freshness/gate bound; relaxing it while moving must not be possible (guard owns the authoritative copy) | dyx3_rpp |
| `path_frame_id` | string | `"local_ned"` | PX4_DXP/src/rpp_controller_node.py:454 | **RESTART** | frame identifier; fixed by the NED contract (docs/contracts/frames.md), not a runtime choice | dyx3_rpp |
| `ekf_jump_threshold_m` | float | `0.05` | PX4_DXP/src/rpp_controller_node.py:461 | **IDLE_ONLY** | safety freshness/gate bound; relaxing it while moving must not be possible (guard owns the authoritative copy) | dyx3_rpp |
| `ekf_reset_compensation` | bool | `False` | PX4_DXP/src/rpp_controller_node.py:476 | **IDLE_ONLY** | feature/mode toggle for an FSM branch; spec §9 "mode defaults" are IDLE_ONLY — flipping a branch mid-run changes the state machine under it | dyx3_rpp |
| `ekf_reset_max_absorb_m` | float | `0.30` | PX4_DXP/src/rpp_controller_node.py:481 | **IDLE_ONLY** | safety freshness/gate bound; relaxing it while moving must not be possible (guard owns the authoritative copy) | dyx3_rpp |
| `require_rtk_fix` | bool | `True` | PX4_DXP/src/rpp_controller_node.py:486 | **IDLE_ONLY** | safety freshness/gate bound; relaxing it while moving must not be possible (guard owns the authoritative copy) | dyx3_rpp |
| `rtk_fix_timeout_s` | float | `0.5` | PX4_DXP/src/rpp_controller_node.py:487 | **IDLE_ONLY** | safety freshness/gate bound; relaxing it while moving must not be possible (guard owns the authoritative copy) | dyx3_rpp |
| `rtk_require_accuracy` | bool | `True` | PX4_DXP/src/rpp_controller_node.py:488 | **IDLE_ONLY** | safety freshness/gate bound; relaxing it while moving must not be possible (guard owns the authoritative copy) | dyx3_rpp |
| `rtk_max_hrms_m` | float | `0.10` | PX4_DXP/src/rpp_controller_node.py:489 | **IDLE_ONLY** | safety freshness/gate bound; relaxing it while moving must not be possible (guard owns the authoritative copy) | dyx3_rpp |
| `rtk_recover_hold_s` | float | `1.0` | PX4_DXP/src/rpp_controller_node.py:490 | **IDLE_ONLY** | safety freshness/gate bound; relaxing it while moving must not be possible (guard owns the authoritative copy) | dyx3_rpp |
| `preview_curvature_n` | int | `4` | PX4_DXP/src/rpp_controller_node.py:498 | **LIVE** | guidance gain/lookahead; spec §9 lists lookahead and gains as LIVE | dyx3_rpp |
| `xtrack_lookahead_gain` | float | `0.05` | PX4_DXP/src/rpp_controller_node.py:504 | **LIVE** | guidance gain/lookahead; spec §9 lists lookahead and gains as LIVE | dyx3_rpp |
| `smooth_lateral_gain` | float | `1.5` | PX4_DXP/src/rpp_controller_node.py:519 | **LIVE** | guidance gain/lookahead; spec §9 lists lookahead and gains as LIVE | dyx3_rpp |
| `smooth_lateral_max_deg` | float | `8.0` | PX4_DXP/src/rpp_controller_node.py:520 | **LIVE** | guidance gain/lookahead; spec §9 lists lookahead and gains as LIVE | dyx3_rpp |
| `smooth_curvature_ld_coeff` | float | `0.20` | PX4_DXP/src/rpp_controller_node.py:528 | **LIVE** | guidance gain/lookahead; spec §9 lists lookahead and gains as LIVE | dyx3_rpp |
| `smooth_max_arc_cut_m` | float | `0.005` | PX4_DXP/src/rpp_controller_node.py:549 | **LIVE** | guidance gain/lookahead; spec §9 lists lookahead and gains as LIVE | dyx3_rpp |
| `smooth_min_arc_ld_m` | float | `0.40` | PX4_DXP/src/rpp_controller_node.py:563 | **LIVE** | guidance gain/lookahead; spec §9 lists lookahead and gains as LIVE | dyx3_rpp |
| `curvature_baseline_m` | float | `0.15` | PX4_DXP/src/rpp_controller_node.py:569 | **LIVE** | guidance gain/lookahead; spec §9 lists lookahead and gains as LIVE | dyx3_rpp |
| `path_resample_spacing_m` | float | `0.08` | PX4_DXP/src/rpp_controller_node.py:582 | **IDLE_ONLY** | path conditioning is applied once at mission install; a mid-run change would leave the installed runs inconsistent with the thresholds | dyx3_rpp |
| `corner_smooth_radius_m` | float | `0.5` | PX4_DXP/src/rpp_controller_node.py:583 | **IDLE_ONLY** | path conditioning is applied once at mission install; a mid-run change would leave the installed runs inconsistent with the thresholds | dyx3_rpp |
| `corner_smooth_arc_pts` | int | `6` | PX4_DXP/src/rpp_controller_node.py:584 | **IDLE_ONLY** | path conditioning is applied once at mission install; a mid-run change would leave the installed runs inconsistent with the thresholds | dyx3_rpp |
| `tracking_profile` | string | `"auto"` | PX4_DXP/src/rpp_controller_node.py:591 | **IDLE_ONLY** | feature/mode toggle for an FSM branch; spec §9 "mode defaults" are IDLE_ONLY — flipping a branch mid-run changes the state machine under it | dyx3_rpp |
| `segment_corner_threshold_deg` | float | `45.0` | PX4_DXP/src/rpp_controller_node.py:592 | **IDLE_ONLY** | path conditioning is applied once at mission install; a mid-run change would leave the installed runs inconsistent with the thresholds | dyx3_rpp |
| `segment_lookahead_cross_collinear_deg` | float | `5.0` | PX4_DXP/src/rpp_controller_node.py:605 | **LIVE** | guidance gain/lookahead; spec §9 lists lookahead and gains as LIVE | dyx3_rpp |
| `segment_endpoint_lookahead_extend` | bool | `True` | PX4_DXP/src/rpp_controller_node.py:629 | **IDLE_ONLY** | feature/mode toggle for an FSM branch; spec §9 "mode defaults" are IDLE_ONLY — flipping a branch mid-run changes the state machine under it | dyx3_rpp |
| `segment_corner_lookahead_extend` | bool | `False` | PX4_DXP/src/rpp_controller_node.py:648 | **IDLE_ONLY** | feature/mode toggle for an FSM branch; spec §9 "mode defaults" are IDLE_ONLY — flipping a branch mid-run changes the state machine under it | dyx3_rpp |
| `segment_simplify_max_offset_m` | float | `0.01` | PX4_DXP/src/rpp_controller_node.py:656 | **IDLE_ONLY** | path conditioning is applied once at mission install; a mid-run change would leave the installed runs inconsistent with the thresholds | dyx3_rpp |
| `entry_prealign_enabled` | bool | `True` | PX4_DXP/src/rpp_controller_node.py:686 | **IDLE_ONLY** | feature/mode toggle for an FSM branch; spec §9 "mode defaults" are IDLE_ONLY — flipping a branch mid-run changes the state machine under it | dyx3_rpp |
| `segment_slowdown_dist` | float | `0.50` | PX4_DXP/src/rpp_controller_node.py:687 | **LIVE** | terminal/capture threshold; spec §9 lists terminal thresholds as LIVE | dyx3_rpp |
| `segment_min_corner_speed` | float | `0.12` | PX4_DXP/src/rpp_controller_node.py:695 | **LIVE** | terminal/capture threshold; spec §9 lists terminal thresholds as LIVE | dyx3_rpp |
| `segment_endpoint_approach_speed` | float | `0.03` | PX4_DXP/src/rpp_controller_node.py:706 | **LIVE** | terminal/capture threshold; spec §9 lists terminal thresholds as LIVE | dyx3_rpp |
| `stop_latch_enabled` | bool | `False` | PX4_DXP/src/rpp_controller_node.py:721 | **IDLE_ONLY** | feature/mode toggle for an FSM branch; spec §9 "mode defaults" are IDLE_ONLY — flipping a branch mid-run changes the state machine under it | dyx3_rpp |
| `stop_latch_min_actuatable_m_s` | float | `0.10` | PX4_DXP/src/rpp_controller_node.py:722 | **LIVE** | terminal/capture threshold; spec §9 lists terminal thresholds as LIVE | dyx3_rpp |
| `stop_latch_capture_dist_m` | float | `0.02` | PX4_DXP/src/rpp_controller_node.py:723 | **LIVE** | terminal/capture threshold; spec §9 lists terminal thresholds as LIVE | dyx3_rpp |
| `stop_latch_release_dist_m` | float | `0.20` | PX4_DXP/src/rpp_controller_node.py:724 | **LIVE** | terminal/capture threshold; spec §9 lists terminal thresholds as LIVE | dyx3_rpp |
| `segment_corner_acceptance_radius` | float | `0.05` | PX4_DXP/src/rpp_controller_node.py:725 | **LIVE** | terminal/capture threshold; spec §9 lists terminal thresholds as LIVE | dyx3_rpp |
| `segment_heading_tolerance_deg` | float | `2.0` | PX4_DXP/src/rpp_controller_node.py:753 | **LIVE** | terminal/capture threshold; spec §9 lists terminal thresholds as LIVE | dyx3_rpp |
| `segment_yaw_rate_gain` | float | `1.5` | PX4_DXP/src/rpp_controller_node.py:754 | **LIVE** | guidance gain/lookahead; spec §9 lists lookahead and gains as LIVE | dyx3_rpp |
| `spray_entry_max_heading_deg` | float | `5.0` | PX4_DXP/src/rpp_controller_node.py:770 | **IDLE_ONLY** | spray-entry gate: changes WHERE the spray valve may open, which is part of the accuracy spec; never mid-mission | dyx3_rpp |
| `spray_entry_release_travel_m` | float | `0.6` | PX4_DXP/src/rpp_controller_node.py:771 | **IDLE_ONLY** | spray-entry gate: changes WHERE the spray valve may open, which is part of the accuracy spec; never mid-mission | dyx3_rpp |
| `spray_heading_cut_deg` | float | `30.0` | PX4_DXP/src/rpp_controller_node.py:772 | **IDLE_ONLY** | spray-entry gate: changes WHERE the spray valve may open, which is part of the accuracy spec; never mid-mission | dyx3_rpp |
| `segment_stop_speed_threshold` | float | `0.02` | PX4_DXP/src/rpp_controller_node.py:778 | **LIVE** | terminal/capture threshold; spec §9 lists terminal thresholds as LIVE | dyx3_rpp |
| `segment_stop_yaw_rate_threshold` | float | `0.05` | PX4_DXP/src/rpp_controller_node.py:779 | **LIVE** | terminal/capture threshold; spec §9 lists terminal thresholds as LIVE | dyx3_rpp |
| `segment_stop_dwell_s` | float | `0.30` | PX4_DXP/src/rpp_controller_node.py:780 | **LIVE** | terminal/capture threshold; spec §9 lists terminal thresholds as LIVE | dyx3_rpp |
| `point_hold_enabled` | bool | `False` | PX4_DXP/src/rpp_controller_node.py:788 | **IDLE_ONLY** | feature/mode toggle for an FSM branch; spec §9 "mode defaults" are IDLE_ONLY — flipping a branch mid-run changes the state machine under it | dyx3_rpp |
| `point_hold_s` | float | `2.0` | PX4_DXP/src/rpp_controller_node.py:789 | **LIVE** | terminal/capture threshold; spec §9 lists terminal thresholds as LIVE | dyx3_rpp |
| `point_hold_acceptance_m` | float | `0.10` | PX4_DXP/src/rpp_controller_node.py:790 | **LIVE** | terminal/capture threshold; spec §9 lists terminal thresholds as LIVE | dyx3_rpp |
| `progress_publish_enabled` | bool | `False` | PX4_DXP/src/rpp_controller_node.py:796 | **LIVE** | diagnostics toggle only; no effect on motion | dyx3_rpp |
| `progress_approach_dist_m` | float | `0.30` | PX4_DXP/src/rpp_controller_node.py:797 | **LIVE** | terminal/capture threshold; spec §9 lists terminal thresholds as LIVE | dyx3_rpp |
| `point_precise_stop_enabled` | bool | `False` | PX4_DXP/src/rpp_controller_node.py:799 | **IDLE_ONLY** | feature/mode toggle for an FSM branch; spec §9 "mode defaults" are IDLE_ONLY — flipping a branch mid-run changes the state machine under it | dyx3_rpp |
| `point_arrival_tolerance_m` | float | `0.02` | PX4_DXP/src/rpp_controller_node.py:800 | **LIVE** | terminal/capture threshold; spec §9 lists terminal thresholds as LIVE | dyx3_rpp |
| `precise_stop_mode` | string | `"feedforward"` | PX4_DXP/src/rpp_controller_node.py:801 | **IDLE_ONLY** | feature/mode toggle for an FSM branch; spec §9 "mode defaults" are IDLE_ONLY — flipping a branch mid-run changes the state machine under it | dyx3_rpp |
| `precise_stop_decel_m_s2` | float | `0.30` | PX4_DXP/src/rpp_controller_node.py:802 | **LIVE** | terminal/capture threshold; spec §9 lists terminal thresholds as LIVE | dyx3_rpp |
| `precise_stop_creep_speed` | float | `0.05` | PX4_DXP/src/rpp_controller_node.py:803 | **LIVE** | terminal/capture threshold; spec §9 lists terminal thresholds as LIVE | dyx3_rpp |
| `precise_stop_max_s` | float | `8.0` | PX4_DXP/src/rpp_controller_node.py:804 | **LIVE** | terminal/capture threshold; spec §9 lists terminal thresholds as LIVE | dyx3_rpp |
| `point_handshake_enabled` | bool | `False` | PX4_DXP/src/rpp_controller_node.py:813 | **IDLE_ONLY** | feature/mode toggle for an FSM branch; spec §9 "mode defaults" are IDLE_ONLY — flipping a branch mid-run changes the state machine under it | dyx3_rpp |
| `point_execution_mode` | string | `"auto"` | PX4_DXP/src/rpp_controller_node.py:814 | **IDLE_ONLY** | feature/mode toggle for an FSM branch; spec §9 "mode defaults" are IDLE_ONLY — flipping a branch mid-run changes the state machine under it | dyx3_rpp |
| `manual_wait_timeout_s` | float | `0.0` | PX4_DXP/src/rpp_controller_node.py:815 | **LIVE** | terminal/capture threshold; spec §9 lists terminal thresholds as LIVE | dyx3_rpp |
| `point_hold_max_s` | float | `10.0` | PX4_DXP/src/rpp_controller_node.py:816 | **LIVE** | terminal/capture threshold; spec §9 lists terminal thresholds as LIVE | dyx3_rpp |
| `segment_brake_velocity_cap_m_s` | float | `0.08` | PX4_DXP/src/rpp_controller_node.py:823 | **LIVE** | pivot/stop-pivot tuning; spec §9 lists pivot tuning as LIVE | dyx3_rpp |
| `segment_align_settle_s` | float | `0.20` | PX4_DXP/src/rpp_controller_node.py:830 | **LIVE** | pivot/stop-pivot tuning; spec §9 lists pivot tuning as LIVE | dyx3_rpp |
| `segment_align_speed_threshold` | float | `0.02` | PX4_DXP/src/rpp_controller_node.py:831 | **LIVE** | pivot/stop-pivot tuning; spec §9 lists pivot tuning as LIVE | dyx3_rpp |
| `segment_turn_timeout_s` | float | `5.0` | PX4_DXP/src/rpp_controller_node.py:835 | **LIVE** | pivot/stop-pivot tuning; spec §9 lists pivot tuning as LIVE | dyx3_rpp |
| `segment_timeout_heading_tolerance_deg` | float | `3.0` | PX4_DXP/src/rpp_controller_node.py:841 | **LIVE** | pivot/stop-pivot tuning; spec §9 lists pivot tuning as LIVE | dyx3_rpp |
| `segment_pivot_spinup_margin_s` | float | `1.0` | PX4_DXP/src/rpp_controller_node.py:849 | **LIVE** | pivot/stop-pivot tuning; spec §9 lists pivot tuning as LIVE | dyx3_rpp |
| `segment_nominal_pivot_rate_rad_s` | float | `0.40` | PX4_DXP/src/rpp_controller_node.py:850 | **LIVE** | pivot/stop-pivot tuning; spec §9 lists pivot tuning as LIVE | dyx3_rpp |
| `segment_pivot_timeout_max_s` | float | `9.0` | PX4_DXP/src/rpp_controller_node.py:851 | **LIVE** | pivot/stop-pivot tuning; spec §9 lists pivot tuning as LIVE | dyx3_rpp |
| `segment_pivot_release_max_deg` | float | `3.0` | PX4_DXP/src/rpp_controller_node.py:860 | **LIVE** | pivot/stop-pivot tuning; spec §9 lists pivot tuning as LIVE | dyx3_rpp |
| `pivot_to_intercept_enabled` | bool | `True` | PX4_DXP/src/rpp_controller_node.py:885 | **IDLE_ONLY** | feature/mode toggle for an FSM branch; spec §9 "mode defaults" are IDLE_ONLY — flipping a branch mid-run changes the state machine under it | dyx3_rpp |
| `pivot_intercept_dist_m` | float | `0.35` | PX4_DXP/src/rpp_controller_node.py:886 | **LIVE** | pivot/stop-pivot tuning; spec §9 lists pivot tuning as LIVE | dyx3_rpp |
| `connector_absorb_m` | float | `0.20` | PX4_DXP/src/rpp_controller_node.py:894 | **IDLE_ONLY** | path conditioning is applied once at mission install; a mid-run change would leave the installed runs inconsistent with the thresholds | dyx3_rpp |
| `connector_min_corner_deg` | float | `20.0` | PX4_DXP/src/rpp_controller_node.py:895 | **IDLE_ONLY** | path conditioning is applied once at mission install; a mid-run change would leave the installed runs inconsistent with the thresholds | dyx3_rpp |
| `use_imu_extrapolation` | bool | `False` | PX4_DXP/src/rpp_controller_node.py:912 | **IDLE_ONLY** | feature/mode toggle for an FSM branch; spec §9 "mode defaults" are IDLE_ONLY — flipping a branch mid-run changes the state machine under it | dyx3_rpp |
| `imu_max_extrap_age_s` | float | `0.10` | PX4_DXP/src/rpp_controller_node.py:915 | **IDLE_ONLY** | safety freshness/gate bound; relaxing it while moving must not be possible (guard owns the authoritative copy) | dyx3_rpp |
| `pose_latency_bias_s` | float | `0.0` | PX4_DXP/src/rpp_controller_node.py:926 | **IDLE_ONLY** | timing compensation changes the pose the controller sees; calibrate at rest, not mid-run | dyx3_rpp |
| `use_feedforward_yaw_rate` | bool | `True` | PX4_DXP/src/rpp_controller_node.py:933 | **IDLE_ONLY** | feature/mode toggle for an FSM branch; spec §9 "mode defaults" are IDLE_ONLY — flipping a branch mid-run changes the state machine under it | dyx3_rpp |
| `yaw_rate_feedback_gain` | float | `0.0` | PX4_DXP/src/rpp_controller_node.py:936 | **LIVE** | guidance gain/lookahead; spec §9 lists lookahead and gains as LIVE | dyx3_rpp |
| `max_yaw_rate_body` | float | `0.45` | PX4_DXP/src/rpp_controller_node.py:943 | **LIVE** | speed limit; spec §9 lists speed limits as LIVE | dyx3_rpp |
| `max_linear_accel` | float | `0.20` | PX4_DXP/src/rpp_controller_node.py:950 | **LIVE** | speed-profile accel/decel/curvature gate; a limit/gain, spec §9 LIVE | dyx3_rpp |
| `mission_speed` | float | `1.0` | PX4_DXP/src/rpp_controller_node.py:976 | **LIVE** | speed limit; spec §9 lists speed limits as LIVE | dyx3_rpp |
| `max_linear_decel` | float | `0.5` | PX4_DXP/src/rpp_controller_node.py:983 | **LIVE** | speed-profile accel/decel/curvature gate; a limit/gain, spec §9 LIVE | dyx3_rpp |
| `accel_gate_heading_full_deg` | float | `2.0` | PX4_DXP/src/rpp_controller_node.py:988 | **LIVE** | speed-profile accel/decel/curvature gate; a limit/gain, spec §9 LIVE | dyx3_rpp |
| `accel_gate_heading_none_deg` | float | `5.0` | PX4_DXP/src/rpp_controller_node.py:989 | **LIVE** | speed-profile accel/decel/curvature gate; a limit/gain, spec §9 LIVE | dyx3_rpp |
| `accel_gate_curv_full` | float | `0.05` | PX4_DXP/src/rpp_controller_node.py:990 | **LIVE** | speed-profile accel/decel/curvature gate; a limit/gain, spec §9 LIVE | dyx3_rpp |
| `accel_gate_curv_none` | float | `0.15` | PX4_DXP/src/rpp_controller_node.py:991 | **LIVE** | speed-profile accel/decel/curvature gate; a limit/gain, spec §9 LIVE | dyx3_rpp |
| `speed_cmd_decel_m_s2` | float | `0.30` | PX4_DXP/src/rpp_controller_node.py:996 | **LIVE** | speed-profile accel/decel/curvature gate; a limit/gain, spec §9 LIVE | dyx3_rpp |
| `kappa_hard_enter` | float | `0.25` | PX4_DXP/src/rpp_controller_node.py:997 | **LIVE** | speed-profile accel/decel/curvature gate; a limit/gain, spec §9 LIVE | dyx3_rpp |
| `kappa_hard_exit` | float | `0.15` | PX4_DXP/src/rpp_controller_node.py:998 | **LIVE** | speed-profile accel/decel/curvature gate; a limit/gain, spec §9 LIVE | dyx3_rpp |
| `preview_curvature_distance_m` | float | `0.0` | PX4_DXP/src/rpp_controller_node.py:1001 | **LIVE** | guidance gain/lookahead; spec §9 lists lookahead and gains as LIVE | dyx3_rpp |
| `actuator_set_index` | int | `1` | PX4_DXP/src/spray_controller_node.py:819 | **IDLE_ONLY** | actuator electrical/hardware mapping; changing mid-line is a visible paint defect | dyx3_spray |
| `on_value` | float | `1.0` | PX4_DXP/src/spray_controller_node.py:825 | **IDLE_ONLY** | actuator electrical/hardware mapping; changing mid-line is a visible paint defect | dyx3_spray |
| `off_value` | float | `-1.0` | PX4_DXP/src/spray_controller_node.py:826 | **IDLE_ONLY** | actuator electrical/hardware mapping; changing mid-line is a visible paint defect | dyx3_spray |
| `flow_modulation_enabled` | bool | `False` | PX4_DXP/src/spray_controller_node.py:833 | **IDLE_ONLY** | flow model value; a mid-line change is a visible paint defect | dyx3_spray |
| `min_flow_value` | float | `0.2` | PX4_DXP/src/spray_controller_node.py:834 | **IDLE_ONLY** | flow model value; a mid-line change is a visible paint defect | dyx3_spray |
| `rated_marking_speed_mps` | float | `0.35` | PX4_DXP/src/spray_controller_node.py:835 | **IDLE_ONLY** | flow model value; a mid-line change is a visible paint defect | dyx3_spray |
| `max_flow_slew_per_s` | float | `2.0` | PX4_DXP/src/spray_controller_node.py:836 | **IDLE_ONLY** | flow model value; a mid-line change is a visible paint defect | dyx3_spray |
| `point_dwell_flow_value` | float | `1.0` | PX4_DXP/src/spray_controller_node.py:839 | **IDLE_ONLY** | flow model value; a mid-line change is a visible paint defect | dyx3_spray |
| `debounce_samples` | int | `3` | PX4_DXP/src/spray_controller_node.py:840 | **IDLE_ONLY** | safety lease / debounce / freshness timing; relaxing it while a run is active must not be possible | dyx3_spray |
| `reassert_hz` | float | `2.0` | PX4_DXP/src/spray_controller_node.py:841 | **IDLE_ONLY** | safety lease / debounce / freshness timing; relaxing it while a run is active must not be possible | dyx3_spray |
| `require_offboard` | bool | `True` | PX4_DXP/src/spray_controller_node.py:842 | **IDLE_ONLY** | safety lease / debounce / freshness timing; relaxing it while a run is active must not be possible | dyx3_spray |
| `active_timeout_s` | float | `0.5` | PX4_DXP/src/spray_controller_node.py:843 | **IDLE_ONLY** | safety lease / debounce / freshness timing; relaxing it while a run is active must not be possible | dyx3_spray |
| `manual_override_timeout_s` | float | `10.0` | PX4_DXP/src/spray_controller_node.py:844 | **IDLE_ONLY** | safety lease / debounce / freshness timing; relaxing it while a run is active must not be possible | dyx3_spray |
| `command_service` | string | `"/mavros/cmd/command"` | PX4_DXP/src/spray_controller_node.py:845 | **RESTART** | MAVROS service name; obsolete under DDS (DERIVED: expected to be removed/replaced by the px4_link actuator path — human to confirm) | dyx3_spray |
| `spray_watchdog_required` | bool | `True` | PX4_DXP/src/spray_controller_node.py:850 | **RESTART** | independent safety-watchdog process configuration; read once at start so the watchdog cannot be re-tuned by the node it supervises | dyx3_spray |
| `spray_watchdog_timeout_s` | float | `1.0` | PX4_DXP/src/spray_controller_node.py:851 | **RESTART** | independent safety-watchdog process configuration; read once at start so the watchdog cannot be re-tuned by the node it supervises | dyx3_spray |
| `use_distance_aware_spray` | bool | `True` | PX4_DXP/src/spray_controller_node.py:852 | **IDLE_ONLY** | nozzle geometry / valve latency calibration: sets the geometric open/close boundary, which is the accuracy spec | dyx3_spray |
| `nozzle_forward_offset_m` | float | `0.0` | PX4_DXP/src/spray_controller_node.py:853 | **IDLE_ONLY** | nozzle geometry / valve latency calibration: sets the geometric open/close boundary, which is the accuracy spec | dyx3_spray |
| `nozzle_lateral_offset_m` | float | `0.0` | PX4_DXP/src/spray_controller_node.py:854 | **IDLE_ONLY** | nozzle geometry / valve latency calibration: sets the geometric open/close boundary, which is the accuracy spec | dyx3_spray |
| `solenoid_open_delay_s` | float | `0.18` | PX4_DXP/src/spray_controller_node.py:857 | **IDLE_ONLY** | nozzle geometry / valve latency calibration: sets the geometric open/close boundary, which is the accuracy spec | dyx3_spray |
| `solenoid_close_delay_s` | float | `0.05` | PX4_DXP/src/spray_controller_node.py:858 | **IDLE_ONLY** | nozzle geometry / valve latency calibration: sets the geometric open/close boundary, which is the accuracy spec | dyx3_spray |
| `anticipatory_margin_m` | float | `0.02` | PX4_DXP/src/spray_controller_node.py:861 | **IDLE_ONLY** | nozzle geometry / valve latency calibration: sets the geometric open/close boundary, which is the accuracy spec | dyx3_spray |
| `on_overspray_margin_m` | float | `0.02` | PX4_DXP/src/spray_controller_node.py:862 | **IDLE_ONLY** | nozzle geometry / valve latency calibration: sets the geometric open/close boundary, which is the accuracy spec | dyx3_spray |
| `off_overspray_margin_m` | float | `0.0` | PX4_DXP/src/spray_controller_node.py:863 | **IDLE_ONLY** | nozzle geometry / valve latency calibration: sets the geometric open/close boundary, which is the accuracy spec | dyx3_spray |
| `terminal_off_epsilon_m` | float | `0.05` | PX4_DXP/src/spray_controller_node.py:872 | **IDLE_ONLY** | nozzle geometry / valve latency calibration: sets the geometric open/close boundary, which is the accuracy spec | dyx3_spray |
| `terminal_off_speed_mps` | float | `0.05` | PX4_DXP/src/spray_controller_node.py:873 | **IDLE_ONLY** | nozzle geometry / valve latency calibration: sets the geometric open/close boundary, which is the accuracy spec | dyx3_spray |
| `min_spray_speed_mps` | float | `0.05` | PX4_DXP/src/spray_controller_node.py:895 | **IDLE_ONLY** | nozzle geometry / valve latency calibration: sets the geometric open/close boundary, which is the accuracy spec | dyx3_spray |
| `spray_off_during_pivot` | bool | `True` | PX4_DXP/src/spray_controller_node.py:901 | **IDLE_ONLY** | nozzle geometry / valve latency calibration: sets the geometric open/close boundary, which is the accuracy spec | dyx3_spray |
| `segment_state_timeout_s` | float | `1.0` | PX4_DXP/src/spray_controller_node.py:902 | **IDLE_ONLY** | safety lease / debounce / freshness timing; relaxing it while a run is active must not be possible | dyx3_spray |
| `consume_rpp_progress` | bool | `False` | PX4_DXP/src/spray_controller_node.py:909 | **IDLE_ONLY** | cross-track/projection gating of the spray; defines boundary semantics (and the open projection-continuity defect) — never mid-run | dyx3_spray |
| `progress_timeout_s` | float | `0.3` | PX4_DXP/src/spray_controller_node.py:910 | **IDLE_ONLY** | safety lease / debounce / freshness timing; relaxing it while a run is active must not be possible | dyx3_spray |
| `max_xtrack_error_m` | float | `0.05` | PX4_DXP/src/spray_controller_node.py:930 | **IDLE_ONLY** | cross-track/projection gating of the spray; defines boundary semantics (and the open projection-continuity defect) — never mid-run | dyx3_spray |
| `xtrack_trip_error_m` | float | `0.08` | PX4_DXP/src/spray_controller_node.py:937 | **IDLE_ONLY** | cross-track/projection gating of the spray; defines boundary semantics (and the open projection-continuity defect) — never mid-run | dyx3_spray |
| `xtrack_gate_min_off_s` | float | `0.2` | PX4_DXP/src/spray_controller_node.py:941 | **IDLE_ONLY** | cross-track/projection gating of the spray; defines boundary semantics (and the open projection-continuity defect) — never mid-run | dyx3_spray |
| `projection_window_back_m` | float | `0.5` | PX4_DXP/src/spray_controller_node.py:953 | **IDLE_ONLY** | cross-track/projection gating of the spray; defines boundary semantics (and the open projection-continuity defect) — never mid-run | dyx3_spray |
| `projection_window_fwd_m` | float | `2.0` | PX4_DXP/src/spray_controller_node.py:954 | **IDLE_ONLY** | cross-track/projection gating of the spray; defines boundary semantics (and the open projection-continuity defect) — never mid-run | dyx3_spray |
| `projection_reacquire_dist_m` | float | `1.0` | PX4_DXP/src/spray_controller_node.py:958 | **IDLE_ONLY** | cross-track/projection gating of the spray; defines boundary semantics (and the open projection-continuity defect) — never mid-run | dyx3_spray |
| `projection_direction_gate_deg` | float | `0.0` | PX4_DXP/src/spray_controller_node.py:972 | **IDLE_ONLY** | cross-track/projection gating of the spray; defines boundary semantics (and the open projection-continuity defect) — never mid-run | dyx3_spray |
| `pose_timeout_s` | float | `0.5` | PX4_DXP/src/spray_controller_node.py:973 | **IDLE_ONLY** | safety lease / debounce / freshness timing; relaxing it while a run is active must not be possible | dyx3_spray |
| `velocity_timeout_s` | float | `0.5` | PX4_DXP/src/spray_controller_node.py:974 | **IDLE_ONLY** | safety lease / debounce / freshness timing; relaxing it while a run is active must not be possible | dyx3_spray |
| `spray_require_rtk_fix` | bool | `True` | PX4_DXP/src/spray_controller_node.py:980 | **IDLE_ONLY** | spray RTK/accuracy safety gate; relaxing mid-run must not be possible | dyx3_spray |
| `spray_min_fix_type` | int | `6` | PX4_DXP/src/spray_controller_node.py:983 | **IDLE_ONLY** | spray RTK/accuracy safety gate; relaxing mid-run must not be possible | dyx3_spray |
| `gps_fix_timeout_s` | float | `0.5` | PX4_DXP/src/spray_controller_node.py:986 | **IDLE_ONLY** | safety lease / debounce / freshness timing; relaxing it while a run is active must not be possible | dyx3_spray |
| `gps_recover_hold_s` | float | `1.0` | PX4_DXP/src/spray_controller_node.py:989 | **IDLE_ONLY** | safety lease / debounce / freshness timing; relaxing it while a run is active must not be possible | dyx3_spray |
| `spray_require_accuracy` | bool | `True` | PX4_DXP/src/spray_controller_node.py:998 | **IDLE_ONLY** | spray RTK/accuracy safety gate; relaxing mid-run must not be possible | dyx3_spray |
| `spray_max_hrms_m` | float | `0.10` | PX4_DXP/src/spray_controller_node.py:1002 | **IDLE_ONLY** | spray RTK/accuracy safety gate; relaxing mid-run must not be possible | dyx3_spray |
| `point_arrival_max_speed_mps` | float | `0.05` | PX4_DXP/src/spray_controller_node.py:1007 | **IDLE_ONLY** | spray RTK/accuracy safety gate; relaxing mid-run must not be possible | dyx3_spray |
| `point_arrival_timeout_s` | float | `60.0` | PX4_DXP/src/spray_controller_node.py:1010 | **IDLE_ONLY** | safety lease / debounce / freshness timing; relaxing it while a run is active must not be possible | dyx3_spray |
| `allow_legacy_spray_active_fallback` | bool | `True` | PX4_DXP/src/spray_controller_node.py:1011 | **TBD — human** | legacy-compat fallback: whether it should exist at all in production is a human call | dyx3_spray |
| `actuator_backend` | string | `"mavlink_actuator"` | PX4_DXP/src/spray_controller_node.py:1014 | **RESTART** | actuator backend/transport selection; fixed at start (DERIVED: MAVROS-era value must be re-mapped to the DDS path) | dyx3_spray |
| `servo_instance` | int | `1` | PX4_DXP/src/spray_controller_node.py:1017 | **RESTART** | actuator backend/transport selection; fixed at start (DERIVED: MAVROS-era value must be re-mapped to the DDS path) | dyx3_spray |
| `off_pwm_us` | int | `0` | PX4_DXP/src/spray_controller_node.py:1018 | **IDLE_ONLY** | actuator electrical/hardware mapping; changing mid-line is a visible paint defect | dyx3_spray |
| `on_pwm_us` | int | `1800` | PX4_DXP/src/spray_controller_node.py:1019 | **IDLE_ONLY** | actuator electrical/hardware mapping; changing mid-line is a visible paint defect | dyx3_spray |
| `spray_enabled` | bool | `True` | PX4_DXP/src/spray_controller_node.py:1025 | **TBD — human** | master enable: turning it OFF must always work (but via the stop path, not a parameter); turning it ON mid-run is not safe — human to confirm the semantics | dyx3_spray |

## Production additions (not in the prototype, not part of the 173)

Parameters the production stack adds. They are not counted above. Each has a recorded decision.

| name | type | default | source | class | rationale | owner |
|---|---|---|---|---|---|---|
| `rpp_timeout_s` | float | `0.5` | DERIVED — human decision 2026-10-08 (review C1 / fix plan A1): matches the stack's 0.5 s freshness convention, 25 missed ticks at 50 Hz; re-validate from Jetson jitter data | **IDLE_ONLY** | spray refuses (valve OFF) when `RppStatus` is older than this; relaxing it while a run is active must not be possible | dyx3_spray |
| `segment_command_mode` | string | `"heading"` | DERIVED — human decision 2026-10-08 (review H7 / fix plan B3): explicit GATE 4 A/B selector; default stays behavior-compatible until rover measurements choose otherwise | **IDLE_ONLY** | `heading` keeps segment `TRACK_HEADING`; `rate` publishes RPP's segment yaw-rate law (`segment_yaw_rate_gain * theta_e`, clamped by `max_yaw_rate_body`). Never switch steering authority mid-run | dyx3_rpp |
