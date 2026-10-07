# Estimator health consumed by dyx3_motion_guard. Published by dyx3_px4_link,
# which derives it from /fmu/out/estimator_status_flags (always available) and,
# when the firmware exposes it, /fmu/out/estimator_status (test ratios).
#
# Fail-safe defaults: every field is zero/false, and flags_valid=false means
# "no estimator data received" which the guard MUST treat as UNHEALTHY.
# Why this exists: a mostly-accepted GNSS-heading fault leaves EKF2 yaw wrong for
# ~190 s with no flag (HANDOFF 2026-10-07), so the guard must watch rejection and
# test ratios itself.
builtin_interfaces/Time stamp
builtin_interfaces/Time px4_sample_stamp

# True once an estimator_status_flags sample has been received and is fresh.
bool flags_valid

# True when estimator_status (test ratios) has been received and is fresh. The
# current firmware does NOT publish estimator_status over DDS (dds_topics.yaml
# @27a7ac9284 carries only estimator_status_flags). While false, the three
# *_test_ratio fields are 0 and carry no information; the guard must then rely on
# the boolean flags only. See docs/agents/HANDOFF.md (open question).
bool test_ratios_valid

# From estimator_status_flags.
bool gnss_yaw_fusion_intended     # cs_gnss_yaw
bool gnss_yaw_fault               # cs_gnss_yaw_fault: heading declared faulty, no longer fused
bool reject_yaw                   # reject_yaw: yaw observation currently rejected
bool reject_hor_pos               # reject_hor_pos
bool reject_hor_vel               # reject_hor_vel
bool inertial_dead_reckoning      # cs_inertial_dead_reckoning: no horizontal-velocity constraint
uint32 innovation_fault_status_changes

# From estimator_status. Low-pass filtered ratio of the innovation to its gate:
# 1.0 = at the gate. Dimensionless, non-negative.
float32 yaw_test_ratio            # hdg_test_ratio
float32 position_test_ratio       # pos_test_ratio (horizontal)
float32 velocity_test_ratio       # vel_test_ratio
