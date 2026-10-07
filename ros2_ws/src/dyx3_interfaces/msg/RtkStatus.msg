builtin_interfaces/Time stamp

# fix_type mirrors px4_msgs/SensorGps.fix_type EXACTLY (PX4 ABI values), so the
# value can be passed through from /fmu/out/vehicle_gps_position without a
# translation table. 0 is also "unknown / no data" and is the safe default.
# Source: msg/SensorGps.msg @ firmware 27a7ac9284.
uint8 FIX_UNKNOWN=0
uint8 FIX_NONE=1
uint8 FIX_2D=2
uint8 FIX_3D=3
uint8 FIX_RTCM_CODE_DIFFERENTIAL=4
uint8 FIX_RTK_FLOAT=5
uint8 FIX_RTK_FIXED=6
uint8 FIX_EXTRAPOLATED=8
uint8 fix_type

# All other fields default to zero/false (unknown, not fresh). correction_age_s
# is seconds, horizontal_accuracy_m is metres, satellites_used is a count.
bool corrections_fresh
float32 correction_age_s
float32 horizontal_accuracy_m
uint8 satellites_used
