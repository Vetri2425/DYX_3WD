# One chunk of the FCU's ULog stream (/fmu/out/ulog_stream), republished by dyx3_px4_link after it has
# acknowledged the chunk to the FCU. dyx3_recorder reassembles it into the run directory.
# Gaps are detectable from msg_sequence (uint16, wraps).
builtin_interfaces/Time stamp
uint16 msg_sequence
uint8 first_message_offset
uint8 flags
uint8[] data
