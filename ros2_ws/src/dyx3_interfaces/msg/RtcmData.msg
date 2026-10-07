# One RTCM chunk for the GNSS receiver. dyx3_gnss_rtk fragments RTCM frames to <= 300 bytes and sets
# the fragment flags exactly as the MAVLink GPS_RTCM_DATA sender would; dyx3_px4_link forwards it to
# /fmu/in/gps_inject_data without interpretation (architecture 5.4.1). Validation is `gps status` on
# the FCU and a FIX transition, never the presence of the topic.
builtin_interfaces/Time stamp
# GpsInjectData.flags (LSB: 1 = fragmented; bits 1-2 fragment id; bits 3-7 sequence id).
uint8 flags
uint8[] data
