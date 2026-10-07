builtin_interfaces/Time stamp

# All fields default to zero; STATE_IDLE and REASON_NONE are safe. Mission,
# run, and point indices are non-negative identifiers.

uint8 STATE_IDLE=0
uint8 STATE_LOADING=1
uint8 STATE_READY=2
uint8 STATE_RUNNING=3
uint8 STATE_PAUSED=4
uint8 STATE_COMPLETED=5
uint8 STATE_ABORTED=6
uint8 STATE_ERROR=7
uint8 state

uint32 mission_id
uint32 run_index
uint32 point_index
uint8 REASON_NONE=0
uint8 REASON_OPERATOR=1
uint8 REASON_SAFETY=2
uint8 REASON_RTK=3
uint8 REASON_PATH_ERROR=4
uint8 REASON_INTERNAL_ERROR=5
uint8 reason_code
