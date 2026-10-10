# Single canonical motion command from RPP to dyx3_motion_guard.
# Nothing downstream of RPP may invent motion; nothing upstream may bypass it.
# See docs/contracts/frames.md for the frozen frame, sign, unit, and clock rules.

# Command-generation time from the publisher's ROS/node clock. It must honour
# use_sim_time so rosbag replay preserves temporal semantics.
builtin_interfaces/Time stamp

# V1 Spec §7.1 requires seq.
# DERIVED — NOT FROM V1 SPEC:
# uint64 and monotonic-per-publisher-lifetime semantics are chosen so the
# receiver can detect stale or duplicate commands without practical wraparound.
# Starts at 0 after each publisher restart; a decrease denotes a new publisher
# session. The Phase 6 motion guard decides its acceptance count.
uint64 seq

# V1 Spec §5.2 control modes.
# DERIVED — NOT FROM V1 SPEC:
# These numeric assignments are fixed interface ABI values.
uint8 MODE_STOP=0
uint8 MODE_TRACK_HEADING=1
uint8 MODE_TRACK_RATE=2
uint8 MODE_PIVOT=3
uint8 MODE_CREEP=4

uint8 mode

# Signed longitudinal velocity in the FRD body X axis, metres/second. Positive
# is forward; negative is reverse. Reverse permission is enforced by the guard,
# never by this message. Must be 0 in MODE_STOP and MODE_PIVOT.
float32 speed_body_x

# Rotation-command contract:
#   MODE_STOP:
#     speed_body_x = 0
#     yaw_setpoint = NaN
#     yaw_rate_setpoint = 0
#
#   MODE_TRACK_HEADING:
#     yaw_setpoint = finite
#     yaw_rate_setpoint = NaN
#
#   MODE_TRACK_RATE:
#     yaw_setpoint = NaN
#     yaw_rate_setpoint = finite
#
#   MODE_PIVOT:
#     speed_body_x = 0
#     yaw_setpoint = NaN
#     yaw_rate_setpoint = finite
#
#   MODE_CREEP:
#     yaw_setpoint = NaN
#     yaw_rate_setpoint = finite
#
# DERIVED — NOT FROM V1 SPEC:
# NaN explicitly marks the inactive rotational representation so stale values
# cannot silently acquire control authority.

# Heading setpoint in radians, NED: 0 = North, positive clockwise, wrapped
# [-pi, pi]. Required finite only for MODE_TRACK_HEADING.
float32 yaw_setpoint

# Yaw-rate setpoint in rad/s, NED/FRD positive clockwise viewed from above.
# Required finite only for MODE_TRACK_RATE, MODE_PIVOT, and MODE_CREEP.
float32 yaw_rate_setpoint

# Default message is MODE_STOP plus valid=false. A consumer must treat
# valid=false as STOP regardless of the other field contents and fail to zero.
bool valid

# --- appended in interfaces 0.14.0 (IF-003) ---
# PX4 timestamp_sample of the pose this command was computed from. Same time base as
# VehicleState.px4_sample_stamp: the Jetson system clock (PX4 timestamps on /fmu/out arrive
# already converted by the uXRCE-DDS client; dyx3_px4_link applies no offset). Zero = no pose.
#   dyx3_rpp: VehicleState.px4_sample_stamp of the newest pose fed into its core before this
#     command was computed, on every command including STOP; zero until a valid pose arrived.
#   dyx3_motion_guard: preserved unchanged on every command it forwards from RPP (accepted,
#     clamped, or a clean STOP). On its own canonical STOP (any refusal, no command, shutdown) it
#     carries the px4_sample_stamp of the newest VehicleState the guard received (zero when that
#     message reported no fresh local position), zero if it never received one.
# Latency evidence only (dyx3_px4_link pose-to-write age). Never used for freshness or gating.
builtin_interfaces/Time source_pose_sample_stamp
