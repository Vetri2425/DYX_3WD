# Health of the PX4 link, published by dyx3_px4_link (the only package that touches the /fmu namespace).
# Fail-safe defaults: everything false / zero means "link not healthy".
builtin_interfaces/Time stamp

# True while /fmu/out/timesync_status is fresh: the uXRCE-DDS session to the FCU is alive.
bool session_alive
# True once EVERY used topic's message hash reported by the firmware equals the hash computed from
# the installed px4_msgs definitions (architecture 4.6). False until proven. A mismatch does not clear
# by itself: the handshake is re-run when the uXRCE-DDS session resets (an FCU reboot), and a hash that
# still differs stays false. Fix px4_msgs and restart; never accept a mismatch.
bool handshake_ok
# True while the offboard heartbeat (OffboardControlMode) is being published.
bool offboard_heartbeat_active
# True while the link is forcing STOP because the guard's command is stale/invalid (fail to zero).
bool failing_to_zero

# First fault (priority order) or FAULT_NONE.
uint8 FAULT_NONE=0
uint8 FAULT_NO_SESSION=1
uint8 FAULT_HANDSHAKE_MISMATCH=2
uint8 FAULT_TOPIC_STALE=3
uint8 FAULT_COMMAND_STALE=4
uint8 FAULT_LOOP_OVERRUN=5
uint8 FAULT_HANDSHAKE_PENDING=6
uint8 fault

# Bit i set = monitored topic i is stale. Bit assignment (frozen, see docs/contracts/dyx3_px4_link.md):
# 0 timesync_status, 1 vehicle_local_position, 2 vehicle_status, 3 vehicle_attitude,
# 4 estimator_status_flags, 5 vehicle_gps_position.
uint32 stale_topics_mask
# Largest age (seconds) among monitored topics, on the link's ROS clock; 0 when nothing seen yet.
float32 worst_topic_age_s
# Age of the newest command received from the guard (seconds); large when none ever arrived.
float32 command_age_s
# Cycles in which the publish loop ran later than 1.5 periods (jitter/overrun evidence).
uint64 loop_overrun_count
# Times the command stream had a gap above command_max_age_s (each forced a STOP).
uint64 command_gap_events
# Times the session was lost and re-established (agent restart, FCU reboot, cable pull).
uint32 session_resets

# uXRCE-DDS timesync (architecture 4.6, F-tasks A1.4: the offset can be ~40 ms for 5-10 minutes after boot = 1.4 cm at 0.35 m/s).
# VALUES ONLY: no convergence criterion has a numeric source yet, so nothing gates on these (OPEN, docs/contracts/dyx3_px4_link.md).
# True while /fmu/out/timesync_status is fresh AND at least one sample has been received; the two fields below are meaningless otherwise.
bool timesync_valid
# TimesyncStatus.estimated_offset (smoothed offset companion<->PX4, microseconds).
int64 timesync_offset_us
# TimesyncStatus.round_trip_time (microseconds) of the newest sample.
uint32 timesync_round_trip_us

# Spray ACK correlation-pair allocator, durable across companion restarts.
uint32 spray_identities_used
uint32 spray_identities_remaining
# Also true when allocator state is unavailable/corrupt; no new proof can be sent.
bool spray_identities_exhausted
uint64 spray_unmatched_ack_count
