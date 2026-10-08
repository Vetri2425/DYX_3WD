# Outcome of a SprayActuatorCommand, from dyx3_px4_link. A command refused, superseded while queued,
# or timed out is acked with success=false and result 255, so the sender's FSM takes its failure path.
builtin_interfaces/Time stamp
uint32 seq
uint8 source
bool success
# px4_msgs VehicleCommandAck.result (0 accepted ... 6 cancelled), or 255 when not completed by the link.
uint8 RESULT_LINK_REFUSED=255
uint8 result
