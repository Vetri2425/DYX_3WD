# Heartbeat of the independent spray watchdog. dyx3_spray refuses ON unless this is fresh and off_authority_ready.
# Fail-safe default: nothing ready.
builtin_interfaces/Time stamp
# True while a fresh, valid lease explicitly allows ON.
bool allow_on
bool watchdog_alive
# True once an OFF command has been accepted by the FCU path since the last failure (the watchdog PROVED it can close).
bool off_authority_ready
bool off_inflight
# Why OFF is currently being enforced ("no controller lease", "controller lease stale (0.412s > 0.350s)", ...); empty when ON is allowed.
string off_reason
# Seconds since the last lease; large when none ever arrived.
float32 lease_age_s
uint8 backend
