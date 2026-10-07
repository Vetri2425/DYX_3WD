# Raw GNSS state from the FCU (/fmu/out/vehicle_gps_position = px4_msgs/SensorGps), republished
# without interpretation by dyx3_px4_link. dyx3_gnss_rtk fuses it with NTRIP freshness into RtkStatus;
# only px4_link may read the /fmu namespace, hence this pass-through.
# Fail-safe default: fix_type 0 (unknown), valid = false.
builtin_interfaces/Time stamp
bool valid
# SensorGps.fix_type values (see RtkStatus.msg).
uint8 fix_type
float32 horizontal_accuracy_m      # SensorGps.eph; 0 is a SENTINEL meaning "unknown" on some boots (A14)
float32 vertical_accuracy_m        # SensorGps.epv
uint8 satellites_used
# Dual-antenna heading as reported by the receiver (rad, NED, 0 = North, clockwise); NaN if absent.
float32 heading_rad
float32 heading_accuracy_rad
# Position as reported by the receiver (WGS84), for the NTRIP GGA back-feed (VRS casters withhold RTCM until
# they hear a rover position). Valid only while `valid` is true; NaN when the receiver reports none.
float64 latitude_deg
float64 longitude_deg
float32 altitude_msl_m
# Horizontal dilution of precision (SensorGps.hdop); 0 when unknown.
float32 hdop
