# Aggregated safety gates, published by dyx3_motion_guard at a FIXED rate independent of whether a
# MotionSetpoint is arriving. The guard is the single owner of gate logic; dyx3_mission consumes
# this to refuse a start and to auto-pause a running mission — it never re-implements a gate.
#
# Gates included: E-stop, RTK, arming/nav state, heading health, estimator health, operator link,
# PX4 link. NOT included: the mission gate itself (the guard's own check that a mission is RUNNING).
#
# Fail-safe default: ok = false. `ok == false` is authoritative; reason_code carries a
# MotionSetpointStatus.REASON_* value that is meaningful only while ok == false (the FIRST failing
# gate in the guard's documented priority order). A default-constructed message therefore means
# "no data: not safe".
builtin_interfaces/Time stamp
bool ok
uint8 reason_code

# --- appended in interfaces 0.15.0 (mission contract v2) ---
# Pre-arm verdict: every gate above EXCEPT "armed" and "nav_state == OFFBOARD", plus
# VehicleState.global_reference_valid. dyx3_mission arms the vehicle only while this is true (it
# never re-implements a gate). Same priority order as `ok`; the arming check keeps only "vehicle
# state fresh and no PX4 failsafe". Fail-safe default: false. pre_arm_reason_code carries a
# MotionSetpointStatus.REASON_* value meaningful only while pre_arm_ok == false (the first failing
# pre-arm gate; REASON_GLOBAL_REFERENCE_INVALID when only the EKF global reference is missing).
bool pre_arm_ok
uint8 pre_arm_reason_code
