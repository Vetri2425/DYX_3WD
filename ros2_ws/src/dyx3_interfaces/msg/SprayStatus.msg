# Rich spray status for the recorder and the backend (SprayState stays the coarse actuator view). Fail-safe defaults.
builtin_interfaces/Time stamp
uint8 FSM_OFF_UNCONFIRMED=0
uint8 FSM_OFF_CONFIRMED=1
uint8 FSM_ON_PENDING=2
uint8 FSM_ON_CONFIRMED=3
uint8 FSM_OFF_PENDING=4
uint8 FSM_RECOVERY=5
uint8 FSM_DISABLED=6
uint8 fsm_state
# True only in FSM_ON_CONFIRMED (invariant 1: no optimistic ON).
bool spraying
bool desired
bool geometry_desired
bool safety_ok
# Short reason of the first failing spray gate; empty when safety_ok.
string safety_reason
bool manual_active
bool projection_valid
# Arc length of the nozzle projection and cross-track distance (metres); meaningful only if projection_valid.
float32 projection_s_m
float32 xtrack_error_m
bool xtrack_tripped
# Distance to the next MARK boundary the lead anticipates; NaN when there is none.
float32 distance_to_boundary_m
uint8 EVENT_NONE=0
uint8 EVENT_ON_EARLY=1
uint8 EVENT_OFF_EARLY=2
uint8 EVENT_TERMINAL_OFF=3
uint8 event
float32 commanded_flow
uint32 cmd_seq
