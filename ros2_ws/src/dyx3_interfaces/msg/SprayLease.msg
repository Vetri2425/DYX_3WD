# Short-lived permission for the spray valve to stay energised, published by dyx3_spray while its full gate is valid.
# The independent dyx3_spray_watchdog sends OFF whenever the lease is absent, denied, malformed or stale, so the valve
# closes even if the controller dies. Also carries the OFF mapping the watchdog must use. Fail-safe default: no ON.
builtin_interfaces/Time stamp
bool allow_on
uint32 command_seq
uint8 BACKEND_ACTUATOR=0
uint8 BACKEND_SERVO_PWM=1
uint8 backend
# 1..6
uint8 actuator_set_index
# Normalized -1..1: the actuator value meaning OFF.
float32 off_value
# 1..16
uint8 servo_instance
# 0..2200
uint16 off_pwm_us
