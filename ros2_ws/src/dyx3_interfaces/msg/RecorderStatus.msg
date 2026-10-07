builtin_interfaces/Time stamp
# All fields default to zero/false; STATE_IDLE is safe. Byte fields are
# non-negative counts.
uint8 STATE_IDLE=0
uint8 STATE_RECORDING=1
uint8 STATE_FINALIZING=2
uint8 STATE_ERROR=3
uint8 state
bool bag_healthy
uint64 bytes_written
uint64 free_bytes
