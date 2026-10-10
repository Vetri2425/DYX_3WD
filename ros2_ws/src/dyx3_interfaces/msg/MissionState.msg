builtin_interfaces/Time stamp

# All fields default to zero; STATE_IDLE and REASON_NONE are safe. Mission,
# run, and point indices are non-negative identifiers.
# dyx3_mission publishes this on EVERY transition (immediately) and at state_publish_hz.
# Lifecycle (docs/contracts/dyx3_mission.md): IDLE -> LOADING -> PLACING -> ARMING -> ENGAGING ->
# READY -> RUNNING -> COMPLETED / ABORTED / ERROR; RUNNING <-> PAUSED.

uint8 STATE_IDLE=0
uint8 STATE_LOADING=1
uint8 STATE_READY=2
uint8 STATE_RUNNING=3
uint8 STATE_PAUSED=4
uint8 STATE_COMPLETED=5
uint8 STATE_ABORTED=6
uint8 STATE_ERROR=7
# 0.15.0: the trajectory is being placed in the EKF local frame (execution artifact written).
uint8 STATE_PLACING=8
# 0.15.0: /dyx3/px4_link/arm (true) requested, waiting for the confirmation.
uint8 STATE_ARMING=9
# 0.15.0: /dyx3/px4_link/set_offboard (true) requested, waiting for the confirmation.
uint8 STATE_ENGAGING=10
uint8 state

uint32 mission_id
uint32 run_index
uint32 point_index
uint8 REASON_NONE=0
uint8 REASON_OPERATOR=1
# A guard gate failed (gate_reason_code names it); RTK has its own code.
uint8 REASON_SAFETY=2
uint8 REASON_RTK=3
# The artifact is missing, malformed or fails its hash.
uint8 REASON_PATH_ERROR=4
uint8 REASON_INTERNAL_ERROR=5
# --- appended in 0.15.0 (existing values unchanged) ---
# VehicleState.xy_reset_counter (or the EKF global reference) changed after placement.
uint8 REASON_EKF_RESET=6
# PLACING: no fresh VehicleState, or no valid EKF global reference to place an anchored path.
uint8 REASON_EKF_REFERENCE_INVALID=7
# PLACING: the placed trajectory (or its anchor) lies outside placement_max_distance_m of the
# EKF origin.
uint8 REASON_PLACEMENT_OUT_OF_BOUNDS=8
# PLACING: the artifact meta has no "frame" key (an old or planner-made artifact), or an
# inconsistent one ("local_ned" without an anchor, "ekf_local_ned" with one, an unknown value).
# EKF-local is never assumed.
uint8 REASON_NO_PLACEMENT_FRAME=9
uint8 REASON_ARM_REFUSED=10
uint8 REASON_ARM_TIMEOUT=11
uint8 REASON_OFFBOARD_REFUSED=12
uint8 REASON_OFFBOARD_TIMEOUT=13
# READY: dyx3_rpp did not acknowledge the execution artifact within rpp_ack_timeout_s.
uint8 REASON_RPP_ACK_TIMEOUT=14
# Emergency stop asserted (the mission aborts and the vehicle is disarmed).
uint8 REASON_ESTOP=15
# dyx3_rpp reported STATE_ERROR for this mission.
uint8 REASON_RPP_ERROR=16
# RUNNING and no RppStatus of this mission within rpp_status_max_age_s: automatic pause.
uint8 REASON_RPP_STALE=17
# 0.17.0: RPP reported pivot_timed_out while RUNNING (commanded to pivot, heading not reached): PAUSED.
uint8 REASON_RPP_PIVOT_TIMEOUT=18
uint8 reason_code

# Content hash of the EXECUTION artifact (docs/contracts/path_artifact.md): lowercase hex
# sha256, empty when nothing is placed. dyx3_rpp loads the SAME file by this id, so mission
# and RPP can never disagree about which path is being driven. Since 0.15.0 this is the placed
# (EKF local frame) artifact; it equals source_artifact_sha256 for an "ekf_local_ned" artifact.
# Set from ARMING on; cleared on ERROR (the execution is unloaded).
string path_artifact_sha256

# --- appended in interfaces 0.15.0 (mission contract v2) ---
# The artifact the operator started (StartMission.path_artifact_sha256); kept until the next start.
string source_artifact_sha256
# StartMission.request_id of this execution (empty when none was given).
string request_id
# Human-readable cause of the current state / reason (e.g. "arm refused by px4_link (reason 1)").
string reason_detail
# With REASON_SAFETY / REASON_RTK: the guard gate that failed (MotionSetpointStatus.REASON_*). 0 otherwise.
uint8 gate_reason_code
# The step the lifecycle is waiting on.
uint8 WAIT_NONE=0
uint8 WAIT_ARTIFACT=1
uint8 WAIT_PLACEMENT=2
uint8 WAIT_ARM=3
uint8 WAIT_OFFBOARD=4
uint8 WAIT_RPP_ACK=5
uint8 WAIT_OPERATOR=6
uint8 WAIT_OFFBOARD_RELEASE=7
uint8 WAIT_DISARM=8
uint8 waiting_on
# When the current state was entered (dyx3_mission ROS clock).
builtin_interfaces/Time state_entered

# --- appended in interfaces 0.17.0 ---
# The run RPP starts (or resumed) this execution from. 0 for a fresh start. A start with StartMission.resume=true
# restores the persisted progress of the same path artifact and sets this to the first run not yet completed;
# dyx3_rpp installs the artifact and begins at this run instead of run 0.
uint32 start_run_index
