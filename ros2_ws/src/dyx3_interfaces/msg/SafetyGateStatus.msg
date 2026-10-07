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
