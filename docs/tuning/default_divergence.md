# Default divergence between PX4_DXP lineages

**Status:** evidence note for GATE 4, 2026-10-07. **Not a decision.** The human must say which
lineage seeds production defaults.

`docs/tuning/parameter_registry.md` carries the defaults of **`build/demo-ready` @ `fc6436b`**
(2026-10-06) verbatim, as instructed. That tree has 119 RPP + 54 spray parameters.

Evidence about the lineages (checked 2026-10-07 by fetching each ref, AST-extracting the
executable `declare_parameter` calls):

- `baseline_master` @ `6d5ba62` (2026-08-20): 115 RPP + 51 spray. **Every shared RPP default is
  identical to `build/demo-ready`**; the only spray default that differs is `gps_fix_timeout_s`
  (0.5 vs 2.0).
- `Upgrade_speed` @ `67150be` (2026-08-04, older): 91 RPP + 51 spray. Its defaults are the
  **retuned set** that `PX4_DXP/docs/BUG_CLOSURE_CHECKLIST.md` (D6) says `cfbf5f3` promoted for
  the 0.35 m/s field work (`min_lookahead_dist 0.35`, `lookahead_time 1.0`, `a_lat_max 0.3`,
  `mission_speed` 0.35 here / 0.5 in the checklist text). The two newer trees do **not** carry
  those values.

So the field-validated 0.35 m/s accuracy baseline in the V1 spec plausibly corresponds to the
**older** retuned defaults, while the newer trees (including the one the registry is taken from)
have reverted to the earlier 0.52 / 1.6 / 1.0 m/s set. Two readings, neither verifiable from this
repository:

1. The newer values are intentional (a demonstration profile or a later retune).
2. The retune was lost on a merge, and the newer trees silently run a different operating point
   from the one the accuracy numbers describe.

Until resolved, the C++ and legacy ports must not treat the registry defaults as the precision
baseline. Re-validation at GATE 4 must name the lineage it validates against.

Table below: `build/demo-ready@fc6436b` vs `Upgrade_speed@67150be`, generated with
`tools/analysis/param_divergence.py` (AST; executable `declare_parameter` calls only):

### RPP

`build/demo-ready@fc6436b`: 119 parameters; `Upgrade_speed@67150be`: 91 parameters.

**Default differs (13)**

| Parameter | `build/demo-ready@fc6436b` | `Upgrade_speed@67150be` |
|---|---|---|
| `a_lat_max` | `0.04` | `0.3` |
| `approach_velocity_scaling_dist` | `0.6` | `0.9` |
| `endpoint_approach_run_remaining` | `False` | `True` |
| `lookahead_time` | `1.6` | `1.0` |
| `max_linear_accel` | `0.2` | `0.35` |
| `max_linear_vel` | `1.0` | `0.8` |
| `min_lookahead_dist` | `0.52` | `0.35` |
| `mission_speed` | `1.0` | `0.35` |
| `segment_heading_tolerance_deg` | `2.0` | `3.0` |
| `segment_min_corner_speed` | `0.12` | `0.08` |
| `segment_pivot_release_max_deg` | `3.0` | `5.0` |
| `segment_timeout_heading_tolerance_deg` | `3.0` | `4.0` |
| `use_imu_extrapolation` | `False` | `True` |

**Only in `build/demo-ready@fc6436b` (28):** `accel_gate_curv_full`, `accel_gate_curv_none`, `accel_gate_heading_full_deg`, `accel_gate_heading_none_deg`, `endpoint_capture_max_miss_m`, `endpoint_capture_past_m`, `endpoint_capture_recover_enabled`, `kappa_hard_enter`, `kappa_hard_exit`, `preview_curvature_distance_m`, `rtk_fix_timeout_s`, `rtk_max_hrms_m`, `rtk_recover_hold_s`, `rtk_require_accuracy`, `segment_corner_lookahead_extend`, `segment_endpoint_arrival_tolerance_m`, `segment_endpoint_creep_speed`, `segment_endpoint_cross_tolerance_m`, `segment_endpoint_max_correction_m`, `segment_endpoint_precise_decel_m_s2`, `segment_endpoint_precise_max_s`, `segment_endpoint_trigger_margin_m`, `segment_precise_endpoint_stop_enabled`, `speed_cmd_decel_m_s2`, `stop_latch_capture_dist_m`, `stop_latch_enabled`, `stop_latch_min_actuatable_m_s`, `stop_latch_release_dist_m`
**Only in `Upgrade_speed@67150be` (0):** —

### Spray

`build/demo-ready@fc6436b`: 54 parameters; `Upgrade_speed@67150be`: 50 parameters.

**Default differs (2)**

| Parameter | `build/demo-ready@fc6436b` | `Upgrade_speed@67150be` |
|---|---|---|
| `gps_fix_timeout_s` | `0.5` | `2.0` |
| `solenoid_open_delay_s` | `0.18` | `0.1` |

**Only in `build/demo-ready@fc6436b` (4):** `projection_direction_gate_deg`, `spray_require_accuracy`, `spray_watchdog_required`, `spray_watchdog_timeout_s`
**Only in `Upgrade_speed@67150be` (0):** —

