# Outcome of a SprayActuatorCommand, from dyx3_px4_link. A command that cannot be sent (link unhealthy, handshake not
# proven) is acked with success=false at once, so the sender's FSM takes its failure path instead of waiting.
builtin_interfaces/Time stamp
uint32 seq
uint8 source
bool success
# px4_msgs VehicleCommandAck.result (0 accepted ... 6 cancelled), or 255 when the link refused to send.
uint8 RESULT_LINK_REFUSED=255
uint8 result
