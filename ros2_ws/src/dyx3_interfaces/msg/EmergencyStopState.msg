# Latched emergency-stop state, published by dyx3_motion_guard (the single authority; a backend
# E-stop is only a REQUEST via the SetEmergencyStop service).
#
# Fail-safe default: asserted = true would be unusual for an ROS IDL default, so the contract is
# explicit instead: consumers MUST treat the ABSENCE of a fresh message as "asserted".
builtin_interfaces/Time stamp
bool asserted
# Free text naming who asserted it ("tablet", "backend", "ble", "physical"); empty when clear.
string source
