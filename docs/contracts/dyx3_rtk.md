# Production RTK correction contract (v1)

**Authority:** `dyx3-rtk.service` owns the configured source, parser, single injection
transport, persisted configuration and runtime status. FastAPI is an authenticated client of
its local control socket. `dyx3_px4_link` remains the only writer of `/fmu/**`; in DDS mode the
RTK node publishes the existing `/dyx3/rtcm` chunks and px4_link forwards them unchanged.
No code configures the UM982. This contract implements the human RTK plan and the 2026-10-09
source/transport decisions; it does not establish any untested receiver or field behavior.

## Selection and authority

`source = NTRIP | LORA`, `transport = USB_DIRECT | PX4_DDS`, and
`desired_state = RUNNING | STOPPED`. Every source can use either transport. There is no
automatic source or transport failover. An unavailable configured device or DDS path causes
degraded status and retry of that configured path only.

Only one `TransportSink` is selected. Reconfiguration increments the source generation,
closes the old sink, joins the old source, atomically saves the new config, constructs the new
sink and source, then accepts only frames from the new generation. A failed save reactivates
the configuration that is actually on disk. Source reconnects clear partial RTCM input; no
RTCM queue survives a sink failure, switch or process restart. The USB sink's public write
method accepts `ValidatedFrame`, which checks preamble, complete length and CRC-24Q; serial
commands and arbitrary buffers cannot be passed to it.

## State and events

The worker state enum is `STOPPED`, `STARTING`, `WAIT_SOURCE`, `WAIT_RTCM`,
`WAIT_TRANSPORT`, `INJECTING`, `DEGRADED`, `RECONFIGURING`, `ERROR`.
Transition events contain UTC timestamp, previous and new state, stable reason code,
human-readable reason, and configuration revision. The bounded status history retains the
last 100 transitions. Current reason codes are `SERVICE_STOPPED`, `SERVICE_START`,
`STOP_REQUESTED`, `SOURCE_NOT_CONFIGURED`, `SOURCE_UNAVAILABLE`, `RTCM_STALE`,
`TRANSPORT_UNAVAILABLE`, `RTCM_DELIVERED`, `CONFIG_CHANGE`, and
`CONFIG_RECOVERY_FAILED`. Client code should display the reason text and retain unknown
future reason codes.

The control status separates `source`, `transport`, and `receiver`. Source counters include
received bytes, CRC-valid frames, CRC failures, tolerated nonzero reserved-header bits
(`invalid_headers`), resync bytes, partial timeouts, reconnects, and last message type.
Transport counters include delivered frames/bytes, failures, opens/reopens, and age. For
PX4_DDS, a successful local `/dyx3/rtcm` publish is the delivery stage; the separate
`Px4LinkStatus.rtcm_chunks_accepted/dropped` counters report px4_link's publication verdict.
Neither stage proves receiver consumption or RTK FIX.

The three ages have different meanings:

| Field | Meaning |
|---|---|
| `source_valid_rtcm_age_s` | Monotonic age since the latest CRC-valid frame from the selected source. `null` before any current-source frame. |
| `transport_delivery_age_s` | Monotonic age since the selected sink completed a frame write (USB) or local chunk publication (DDS). `null` before delivery. |
| `receiver_correction_age_s` | Receiver-reported differential age from a checksum-valid USB GGA sentence, only when present and fresh. Otherwise `null` and `receiver_correction_age_valid=false`. DDS mode has no receiver age in the current `GnssReport` interface. |

The existing `/dyx3/rtk_status` and `/dyx3/ntrip_status` ROS messages remain unchanged for
the safety consumers. The RTK status marks corrections fresh only when source and selected
transport delivery are both fresh and the FCU GNSS report is fresh. The richer source,
transport, age and event model is available through the control socket and REST API.

## Persistent configuration and migration

The v1 JSON file is `/var/lib/dyx3/rtk/config.json`, with schema, revision, updated_at,
desired_state, source, transport, `ntrip.active_profile_id`, `ntrip.profiles[]`, `lora`, and
`usb`. Each NTRIP profile contains id, name, host, port, mountpoint, username, write-only
password, explicit `PLAINTEXT|TLS` security, optional CA file, connect/stream/GGA timing and
backoff. LoRa has stable by-id device, baud, read timeout and reopen delay. USB has stable
by-id receiver device, baud, write timeout and reopen delay. An empty device path is allowed
so a new installation can boot and show `WAIT_SOURCE` or `WAIT_TRANSPORT` before hardware
identity, baud and timeouts are entered. Unknown hardware values are stored as zero/unset,
not guessed. No tty enumeration path or device auto-detection is used.

On a fresh installation with no configured NTRIP seed, the first config is
`NTRIP + USB_DIRECT + RUNNING`. The by-id receiver path and bench-confirmed baud must be
entered before USB injection can work. On upgrade from the deployed NTRIP → DDS service,
if no runtime config exists and `/etc/dyx3/ntrip.env` has a caster host, the worker imports
that seed once as profile `legacy` and selects `NTRIP + PX4_DDS + RUNNING`. The runtime file
is authoritative thereafter; the service never writes back to the seed. A missing or invalid
explicit seed security value is an error, never inferred from port 2101.

The installer creates `/var/lib/dyx3/rtk/` as `dyx3:dyx3 0700` if missing and never
overwrites, deletes or rolls back its contents. Runtime files are `0600`. Save writes a
temporary file in the same directory, `fsync`s it, renames it over `config.json`, then
`fsync`s the directory. A failed pre-rename save leaves the old config intact. The unit
retains `ProtectSystem=strict` and its existing `ReadWritePaths`; `/etc/dyx3/ntrip.env`
remains read-only `root:dyx3 0640` seed material.

**Known isolation limit:** the backend and RTK worker both run as `dyx3`, so filesystem
permissions alone do not separate the backend from stored secrets. The control socket and
REST API never return passwords. A dedicated `dyx3-rtk` Unix user is a possible later
hardening step and is outside this change.

## Control and REST

The Unix socket is `/run/dyx3/rtk-control.sock` (`0660`). Protocol v1 accepts one
newline-delimited JSON request per connection and returns one newline-delimited JSON reply.
Requests contain `{"v":1,"cmd":"..."}`. `GET_STATUS` and `GET_CONFIG` return data;
`SET_CONFIG` requires a complete `config` object with the currently observed revision;
`START` and `STOP` persist `desired_state`. Invalid candidates leave running config
unchanged. `GET_CONFIG` omits every password and includes `password_set`; an omitted password
on `SET_CONFIG` retains the stored value for the same profile id. A password is accepted only
on write and never appears in status, events, logs or any reply. Requests and replies are
bounded to 64 KiB.

Authenticated backend routes are `GET /api/rtk/status`, `GET|PUT /api/rtk/config`,
`POST /api/rtk/start|stop`, `GET|PUT /api/rtk/source`,
`GET|PUT /api/rtk/transport`, `GET|POST /api/rtk/profiles`,
`PATCH|DELETE /api/rtk/profiles/{id}`, and `GET /api/rtk/serial-ports`.
Reads require viewer; writes require operator. Worker unavailability returns 503. The
serial list is read from `/dev/serial/by-id` and never chooses a device automatically.
There is exactly one `/api/rtk/status` route in the assembled FastAPI app. WebSocket and
tablet UI changes are separate work.

## Hardware answers still required

Before using USB or LoRa on a rover, bench-confirm which UM982 COM maps to USB versus
PX4 TELEM1 (they must be different COMs), the USB COM baud, whether that COM emits GGA,
and the LoRa radio model and baud. Off-target tests prove framing and routing behavior,
not the hardware link, RTK FIX, latency or accuracy.
