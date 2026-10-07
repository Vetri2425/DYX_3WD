# Result emitted by dyx3_motion_guard for each received MotionSetpoint.
# All fields default to the ROS IDL safe zero values: numeric 0, bool false.
builtin_interfaces/Time stamp
uint64 input_seq

# Frozen reason-code ABI. Non-OK outcomes represent a fail-to-zero or clamp.
uint8 REASON_OK=0
uint8 REASON_INVALID_MESSAGE=1
uint8 REASON_STALE=2
uint8 REASON_SEQUENCE=3
uint8 REASON_MISSION_GATE=4
uint8 REASON_ESTOP=5
uint8 REASON_RTK_GATE=6
uint8 REASON_LIMIT_CLAMPED=7
uint8 REASON_PX4_LINK_UNHEALTHY=8
uint8 reason_code

bool accepted
bool clamped

# Applied command; default mode 0 is the safe STOP command. Units and frames
# are exactly MotionSetpoint's docs/contracts/frames.md contract.
uint8 mode
float32 speed_body_x
float32 yaw_setpoint
float32 yaw_rate_setpoint
bool valid
