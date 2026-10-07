# Tablet/operator heartbeat state, published by dyx3_system_gateway (architecture 4.3.1). PX4 cannot
# see a dropped tablet, so the Jetson owns this failsafe: dyx3_motion_guard stops on alive == false.
#
# Fail-safe default: alive = false (a node that never heard the operator reports "not alive").
builtin_interfaces/Time stamp
bool alive
# Seconds since the last operator heartbeat, on the gateway's ROS clock; 0 when never heard.
float32 age_s
