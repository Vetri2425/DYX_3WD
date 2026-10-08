# dyx3_gnss_rtk — contract

**Status:** draft for review, written before the implementation. **Spec:** V1 §5.3, §5.4.1, §7 (package list), Phase plan 8.
**Evidence:** `PX4_DXP/ntrip_rtcm_node.py` (595 lines) and `ntrip_protocol.py`, read-only. Spec §2: *"NTRIP / RTCM
protocol handling — works. Do not spend risk budget here."* This package is therefore a faithful port of the working
protocol behaviour into its own service, plus the structure the prototype lacked.

## 1. Hard rules

* **Nothing here configures the UM982.** The receiver's production configuration lives in its own persistent memory
  (CLAUDE.md §10, `docs/contracts/GNSS_receiver_configuration.md`). This package forwards RTCM bytes and *observes*; it
  has no code path that writes anything but RTCM to the receiver, and no serial/UART access at all (the corrections
  travel Jetson → `dyx3_px4_link` → `/fmu/in/gps_inject_data` → the GPS driver).
* **Its own service, never a child of the backend** (the defect §5.3 names: a `server/**` deploy silently dropped the
  rover to FLOAT). It autostarts and reconnects forever; losing corrections is a published, recorded state.
* **No credentials in argv, logs, status or Git.** The prototype read the password from stdin; here the host, port,
  mountpoint, user and password come from the environment (`EnvironmentFile=/etc/dyx3/ntrip.env`, `root:dyx3 0640`),
  and the password is never formatted into any string except the single Authorization header.
* Only `dyx3_px4_link` touches `/fmu/**`. This package publishes `RtcmData` and never talks to the FCU.

## 2. Interfaces

| Direction | Topic | Type | Notes |
|---|---|---|---|
| out | `/dyx3/rtcm` | RtcmData | one message per chunk, <= 300 bytes, flags set as the MAVLink `GPS_RTCM_DATA` sender would |
| out | `/dyx3/rtk_status` | RtkStatus | 5 Hz; consumed by `dyx3_motion_guard` (RTK gate) and spray |
| out | `/dyx3/ntrip_status` | NtripStatus | 1 Hz; link state, age, rate, counters, FIX transitions |
| in | `/dyx3/gnss_report` | GnssReport | raw FCU GNSS (`px4_link`): fix type, accuracy, satellites, HDOP, position |

`NtripStatus.source_bytes_received` counts raw caster response-body bytes, even when RTCM is invalid. `valid_rtcm_frames` counts CRC-valid parser output. `frames_total` and `bytes_total` retain the valid-frame and valid-frame-byte meanings. `chunks_handed_off` increments only after `/dyx3/rtcm` publication returns; it does not mean the PX4 link accepted a chunk. `Px4LinkStatus.rtcm_chunks_accepted` and `rtcm_chunks_dropped` are separately owned by px4_link. None of these counters prove receiver consumption or RTK FIX.

## 3. RTCM framing and chunking

* **Frame extraction** (carried verbatim in behaviour): scan for the `0xD3` preamble; 10-bit length from the 2-byte
  field (reserved high bits are tolerated: some casters set them); a frame is `3 + len + 3` bytes; a CRC-24Q mismatch
  discards the frame and **resumes the scan one byte after that preamble** (not after the claimed frame); an incomplete
  frame stays buffered for the next read. CRC-24Q polynomial `0x1864CFB`.
* **Hardening vs the prototype (DERIVED):** the unconsumed buffer is capped (`max_buffer_bytes`, a stream that never
  yields a valid frame must not grow without bound; on overflow the oldest bytes are dropped and counted in
  `resync_bytes`). Counters for CRC failures and resynchronised bytes exist so a degrading link is visible.
* **Chunking:** a frame of at most 300 bytes is one chunk with `flags = (seq << 3)`; a longer frame (RTCM3 allows 1029 bytes)
  is split into at most 4 chunks of at most 300 bytes with `flags = 1 | (fragment_id << 1) | (seq << 3)`; `seq` increments
  once per frame modulo 32. This follows the MAVLink `GPS_RTCM_DATA` flags layout that the GPS driver's reassembly expects
  (spec §5.4.1): LSB = fragmented, bits 1–2 fragment id, bits 3–7 sequence id. **Not provable off-target:** the
  driver-side reassembly. Validation is `gps status` on the FCU and a FIX transition, never the presence of the topic.

## 4. NTRIP client (carried behaviour)

* Request: `GET /<mountpoint> HTTP/1.0`, `Host`, `Ntrip-Version: Ntrip/2.0`, `User-Agent: NTRIP ROS2/1.0`, HTTP Basic
  `Authorization`. Response: accept **only** an `ICY` or `HTTP/x.y` status line whose status code is exactly `200`; a
  header ends at a blank line, or (ICY) after its status line; more than 2048 bytes without a header is "bad response".
  Bytes after the header are RTCM and are kept.
* Connect/handshake timeout 10 s; stream read timeout 10 s (`_STREAM_SOCKET_TIMEOUT_S`): a silent stream is a dead one
  and triggers a reconnect. Reconnect backoff `min(5 * 2^attempt, 60)` s, reset on a successful handshake, interruptible
  on shutdown. A half-open stream is repaired by closing the socket after 3 consecutive failed GGA back-feeds.
* **GGA back-feed** (VRS casters withhold RTCM until they hear a position): sent immediately after the handshake and then
  every 10 s, only if the position is usable (finite, in range, not older than 5 s — `gga_position_is_usable`). The
  prototype used placeholders for satellites (8) and HDOP (1.0) and a quality derived from covariance; here the receiver's
  own `satellites_used`, `hdop` and `fix_type` are used (quality: 1 GPS, 2 DGPS, 4 RTK fixed, 5 RTK float). NMEA checksum is
  the XOR of the bytes between `$` and `*`.
* I/O runs on its own thread; the ROS thread only reads atomics/snapshots. Shutdown closes the socket to unblock `recv`.

## 5. Correction health and `RtkStatus`

* `correction_age_s` = time since the last CRC-valid frame (monotonic clock). `corrections_fresh` = age <=
  `correction_fresh_s`. **DERIVED — NOT FROM V1 SPEC:** default 10.0 s = the prototype's stream-liveness bound
  (`_STREAM_SOCKET_TIMEOUT_S`); no accuracy-derived limit exists in the evidence. Base stations send epochs at about 1 Hz, so
  10 s is tolerant; **re-validate at GATE 4** against recorded RTK_FIXED retention (tightening it is a one-word change).
* `correction_rate_hz`: frames per second over a sliding window (`rate_window_s`, DERIVED 10 s). Published, **not gated**:
  no source exists for a minimum rate.
* `RtkStatus` fail-safe: `fix_type` and accuracy come from the newest `GnssReport`; if it is older than
  `gnss_report_max_age_s` (DERIVED 1.0 s, the same bound `dyx3_px4_link` uses for the GPS topic) the status reports
  `FIX_UNKNOWN`, accuracy 0 and `corrections_fresh = false`. `corrections_fresh` is also false whenever the stream is down.
  `horizontal_accuracy_m == 0` is a *sentinel meaning unknown* (A14); it is passed through unchanged and the guard treats it as failing.
* **FIX transition monitor:** every change of `fix_type` is recorded (time, from, to, correction age at that moment) and
  logged; a drop from RTK_FIXED or RTK_FLOAT while corrections are *fresh* is logged at WARN (it is the signature of corrections
  arriving but being useless: wrong fragmenting, wrong mountpoint, rejected by the receiver). `NtripStatus.fix_transitions` counts them.

## 6. Parameters

| Name | Default | Class | Source |
|---|---|---|---|
| `ntrip_host`, `ntrip_port`, `ntrip_mountpoint`, `ntrip_user` | from environment | RESTART | prototype CLI args; secrets via `EnvironmentFile` |
| `ntrip_password` | environment only, never a ROS parameter | RESTART | CLAUDE.md §4 |
| `connect_timeout_s`, `stream_timeout_s` | 10, 10 | RESTART | prototype constants |
| `gga_interval_s`, `gga_max_fix_age_s` | 10, 5 | RESTART | prototype constants |
| `backoff_base_s`, `backoff_max_s` | 5, 60 | RESTART | prototype formula |
| `correction_fresh_s` | 10 | IDLE_ONLY | DERIVED, see section 5 |
| `gnss_report_max_age_s` | 1.0 | IDLE_ONLY | DERIVED |
| `rate_window_s` | 10 | IDLE_ONLY | DERIVED |
| `max_buffer_bytes` | 8192 | RESTART | DERIVED hardening |

## 7. Acceptance

Off-target (CI): CRC-24Q against known vectors, frame extraction (split frames, garbage, bad CRC resync, reserved bits,
buffer cap), chunk flags for 1 to 4 fragments and sequence wrap, response parsing (ICY/HTTP/200 only), GGA formatting and
checksum, the health/fix monitors, and the NTRIP client end-to-end against a loopback fake caster (auth header, ICY and HTTP,
rejection, silence timeout, reconnect backoff, GGA back-feed). **Not provable off-target:** a real caster, the LTE link, the
GPS driver's reassembly, and any RTK FIX transition on the receiver.
