# Canonical PX4 state fan-out. All vectors are local NED; q is FRD body -> NED.
# All fields default to the ROS IDL safe zero values: numeric 0, bool false.
# Validity flags must be true before consumers use their associated measurement.
builtin_interfaces/Time stamp
builtin_interfaces/Time px4_sample_stamp

bool position_valid
bool velocity_valid
bool attitude_valid
float32 north_m
float32 east_m
float32 down_m
float32 velocity_north_mps
float32 velocity_east_mps
float32 velocity_down_mps
float32[4] q_frd_to_ned
float32 heading_rad
float32 yaw_rate_radps

# PX4 vehicle_local_position reset/reference fields. Latitude/longitude are
# WGS84 degrees; reference altitude is metres AMSL.
uint8 xy_reset_counter
float32 delta_north_m
float32 delta_east_m
bool global_reference_valid
float64 reference_latitude_deg
float64 reference_longitude_deg
float32 reference_altitude_m_amsl

# PX4 vehicle_status fields; numeric values are PX4 ABI values. Zero means
# unspecified/not-active and is safe until a fresh valid state is received.
uint8 arming_state
uint8 nav_state
bool failsafe
bool preflight_checks_pass

# --- appended in interfaces 0.14.0 (IF-004) ---
# PX4 vehicle_local_position z_valid / v_z_valid for down_m / velocity_down_mps: true only while
# the local-position sample is fresh, the PX4 flag is set and the value is finite. Ground marking
# does not consume the vertical estimate; these flags exist so no consumer ever has to assume it.
bool vertical_position_valid
bool vertical_velocity_valid

# --- appended in interfaces 0.16.0 ---
# PX4 battery_status (1 Hz), for display only: no gate reads it. battery_valid is true only while a
# sample is fresh (3 s) and PX4 reports the battery connected; otherwise the values are NaN.
bool battery_valid
float32 battery_voltage_v
float32 battery_current_a
float32 battery_remaining        # 0..1
