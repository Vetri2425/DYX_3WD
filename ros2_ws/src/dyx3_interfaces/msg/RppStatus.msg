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
# A verified path is loaded for this mission and RPP is waiting for it to run (the acknowledgement
# dyx3_mission waits for in READY). Commands are STOP.
uint8 STATE_LOADED=7
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

# --- appended in interfaces 0.8.0 (existing fields unchanged) ---
# The prototype's own diagnostic codes, so the recorder and the shadow-run comparison see what RPP saw.
# tick_state: -1 STALE, 0 IDLE, 1 TRACKING, 2 APPROACH, 3 DONE, 4 RTK_WAIT, 5 JUMP_SKIP.
int8 tick_state
# segment_state: 0 INACTIVE, 1 TRACK_SEGMENT, 2 PRE_CORNER_SLOWDOWN, 3 CORNER_ALIGN, 4 DONE, 5 CORNER_STOP.
uint8 segment_state
# RPP's request to open the valve: the MARK flag of the tracked segment of the CONDITIONED run, AFTER the heading
# gates (cut at spray_heading_cut_deg; entry hold until the pivot residual is inside spray_entry_max_heading_deg).
# The conditioned run fuses a short unpainted lead into the mark (so the rover does not stop at every mark start),
# hence this can be true where the planner's flag is false: it is a heading verdict for the tracked leg, NOT the
# valve boundary. A request, never an actuation: the valve is owned by dyx3_spray (lease, watchdog, boundary
# projection on the artifact).
bool spray_request
# Features this RPP does not run: 0 none, 1 point hold (point_hold_enabled). Non-zero with state ERROR.
uint8 handoff
# Why RPP refused to drive on the RTK gate: 0 ok, 1 unavailable, 2 stale, 3 fix below minimum,
# 4 not a rover fix, 5 accuracy unknown, 6 accuracy too large, 7 recovering (hold after a drop).
uint8 rtk_reason
