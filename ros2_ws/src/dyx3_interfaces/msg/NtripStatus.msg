# State of the NTRIP correction link, published by dyx3_gnss_rtk (architecture 5.3: losing RTK is a first-class
# vehicle state, not a log line). Fail-safe defaults: everything false / zero = "no corrections".
builtin_interfaces/Time stamp
uint8 STATE_STARTING=0
uint8 STATE_CONNECTING=1
uint8 STATE_STREAMING=2
uint8 STATE_RECONNECTING=3
uint8 STATE_ERROR=4
uint8 STATE_STOPPED=5
uint8 state
bool connected
# True only while RTCM frames are arriving fresh (see RtkStatus.corrections_fresh for the gate value).
bool streaming
# Seconds since the last complete, CRC-valid RTCM frame; large when none ever arrived.
float32 correction_age_s
# RTCM frames per second over the monitor window (0 until the window has data).
float32 correction_rate_hz
uint64 frames_total
uint64 bytes_total
# Frames discarded for a bad CRC-24Q, and bytes skipped while hunting for a preamble.
uint64 crc_failures
uint64 resync_bytes
uint32 reconnect_count
# Seconds the current connection has been established; 0 when not connected.
float32 connected_for_s
# Short reason of the last failure ("caster rejected: HTTP/1.1 401", "timeout"); never contains credentials.
string last_error
# Raw NTRIP response-body bytes received, including invalid RTCM and partial frames.
uint64 source_bytes_received
# CRC-valid complete RTCM frames parsed from the source.
uint64 valid_rtcm_frames
# RTCM chunks successfully published toward px4_link. This does not prove link acceptance.
uint64 chunks_handed_off
# GGA back-feeds sent to the caster.
uint64 gga_sent
# FIX transitions seen (RtkStatus.fix_type changes) and the current fix, for the recorder's evidence trail.
uint32 fix_transitions
uint8 fix_type

# Explicit NTRIP configuration (0 only for an unconfigured/invalid legacy environment).
uint8 SECURITY_UNSPECIFIED=0
uint8 SECURITY_PLAINTEXT=1
uint8 SECURITY_TLS=2
uint8 security
# True only after a TLS handshake with verified certificate and matching host identity.
bool tls_verified
# Sticky since the last successful TLS handshake; inspect last_error for the failure class.
bool tls_verification_failed
# True while a plaintext profile carries credentials. Safe to expose to status/UI.
bool plaintext_credentials_warning
