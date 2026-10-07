# Request to drive the spray actuator, from dyx3_spray (controller) or dyx3_spray_watchdog. dyx3_px4_link is the
# only package that touches /fmu, so it translates this into VEHICLE_CMD_DO_SET_ACTUATOR (187) or
# VEHICLE_CMD_DO_SET_SERVO (183) and answers with SprayActuatorAck. Fail-safe default: OFF on the controller.
builtin_interfaces/Time stamp
# Monotonic per source. The sender's FSM only accepts the ack whose seq matches its latest command.
uint32 seq
uint8 SOURCE_CONTROLLER=0
uint8 SOURCE_WATCHDOG=1
uint8 source
uint8 BACKEND_ACTUATOR=0
uint8 BACKEND_SERVO_PWM=1
uint8 backend
bool on
# BACKEND_ACTUATOR: DO_SET_ACTUATOR with `value` (normalized, -1..1) in slot `actuator_set_index` (1..6), the other
# five slots NaN. BACKEND_SERVO_PWM: DO_SET_SERVO with instance `servo_instance` and `pwm_us` (0..2200).
uint8 actuator_set_index
float32 value
uint8 servo_instance
uint16 pwm_us
