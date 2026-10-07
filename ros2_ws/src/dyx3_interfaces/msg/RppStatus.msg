builtin_interfaces/Time stamp

# All fields default to zero; STATE_IDLE is safe. Cross-track is metres RIGHT
# positive, heading is NED radians, speeds are m/s, rates are rad/s, and loop
# timing values are microseconds. Indices/counts are non-negative.

uint8 STATE_IDLE=0
uint8 STATE_TRACKING=1
uint8 STATE_STOPPING=2
uint8 STATE_PIVOTING=3
uint8 STATE_CREEPING=4
uint8 STATE_COMPLETE=5
uint8 STATE_ERROR=6
uint8 state

uint32 mission_id
uint32 run_index
float32 cross_track_right_m
float32 heading_error_rad
float32 commanded_speed_mps
float32 commanded_yaw_rate_radps

# Measured control-loop health. Jitter is actual-period minus target-period.
float32 loop_jitter_us
float32 loop_jitter_max_us
uint64 loop_overrun_count
