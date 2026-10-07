builtin_interfaces/Time stamp
# All fields default to zero/false; STATE_OFF is safe. commanded_flow_normalized
# is in the actuator backend's documented normalized range.
uint8 STATE_OFF=0
uint8 STATE_ON=1
uint8 STATE_FAULT=2
uint8 state
bool enabled
bool watchdog_healthy
float32 commanded_flow_normalized
