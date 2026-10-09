# dyx3_px4_link — contract

**Status:** draft for review, written before the implementation (CLAUDE.md §6: SAFETY-CRITICAL).
**Spec:** V1 §4.6, §5.4, §7.7; `docs/contracts/F1.7_B2_firmware_decisions.md` (explicit-control
contract); proposal `2026-10-07_ethernet-dds-only-companion-link.md`.
**Authority:** this is the only package that touches `/fmu/**`. Nothing else in the repo may
publish or subscribe there. It owns the DDS session liveness view, the `px4_msgs` handshake, the
offboard heartbeat, setpoint publication and the fan-out of PX4 state into repo-owned messages.
It has no mission, path or spray logic and never decides a motion — it only maps the guard's
command to PX4 setpoints and falls to zero when that command is not trustworthy.

## 1. Pinned message set

`px4_msgs` is built from the pinned firmware (`installer/pins/firmware.pin`, flashed SHA
`27a7ac92845317b0276776242c504215809b2a0f`): `msg/*.msg`, `msg/versioned/*.msg`, `srv/*.srv`.
Stock `px4_msgs` from apt is never used. Versioned messages carry a `_v<MESSAGE_VERSION>` suffix on
the DDS topic when the version is above 0:

| Direction | DDS topic | Type | QoS | Notes |
|---|---|---|---|---|
| in (we publish) | `/fmu/in/offboard_control_mode` | OffboardControlMode (v0) | reliable, depth 1 | heartbeat, every cycle |
| in | `/fmu/in/trajectory_setpoint` | TrajectorySetpoint (v0) | reliable, depth 1 | velocity = NaN, every cycle |
| in | `/fmu/in/rover_speed_setpoint` | RoverSpeedSetpoint | reliable, depth 1 | |
| in | `/fmu/in/rover_attitude_setpoint` | RoverAttitudeSetpoint | reliable, depth 1 | |
| in | `/fmu/in/rover_rate_setpoint` | RoverRateSetpoint | reliable, depth 1 | |
| in | `/fmu/in/vehicle_command` | VehicleCommand (v0) | reliable, depth 10 | arm, mode, logging, serialized spray |
| in | `/fmu/in/gps_inject_data` | GpsInjectData | reliable, depth 10 | RTCM pass-through |
| in | `/fmu/in/ulog_stream_ack` | UlogStreamAck | reliable, depth 16 | |
| in | `/fmu/in/message_format_request` | MessageFormatRequest | reliable, depth 10 | handshake |
| out (we subscribe) | `/fmu/out/timesync_status` | TimesyncStatus | best effort | session liveness |
| out | `/fmu/out/vehicle_local_position_v1` | VehicleLocalPosition (v1) | best effort | |
| out | `/fmu/out/vehicle_status_v1` | VehicleStatus (v1) | best effort | **rate limited to 5 Hz by firmware** |
| out | `/fmu/out/vehicle_attitude` | VehicleAttitude (v0) | best effort | q is FRD to NED |
| out | `/fmu/out/estimator_status_flags` | EstimatorStatusFlags | best effort | only estimator topic on DDS |
| out | `/fmu/out/vehicle_gps_position` | SensorGps | best effort | |
| out | `/fmu/out/vehicle_command_ack` | VehicleCommandAck | best effort | spray ACK matching (section 14) |
| out | `/fmu/out/message_format_response` | MessageFormatResponse | best effort, depth 10 | handshake; a lost response is re-requested |
| out | `/fmu/out/ulog_stream` | UlogStream | best effort, depth 16 | no firmware rate limit |

**QoS of `/fmu/in`:** reliable. DERIVED — NOT FROM V1 SPEC: the XRCE agent's reader is reliable (the
stock PX4 offboard examples publish with the default reliable QoS); a best-effort writer would not
match it. Not provable off-target: confirm with `ros2 topic info -v` at GATE 1. Every `/fmu/out`
subscription is best effort: the firmware's uXRCE-DDS writers are all best effort
(`src/modules/uxrce_dds_client/utilities.hpp:80,137` at the flashed firmware), and a reliable
reader never matches a best-effort writer (XR-GPX-004: `ulog_stream` used to be reliable and no
chunk was ever received). A lost `ulog_stream` chunk shows as a `msg_sequence` gap to the
consumer; the fake FCU in the tests publishes best effort so it cannot hide a mismatch.

`estimator_status` (test ratios) and `estimator_aid_src_*` are **not** exposed over DDS at the
flashed firmware. `EstimatorHealth.test_ratios_valid` is therefore always false until a firmware
`dds_topics.yaml` change; the guard must rely on the boolean flags. (Open question, HANDOFF.)

## 2. Published and served (repo interfaces)

| Name | Type | Rate | Content |
|---|---|---|---|
| `/dyx3/vehicle_state` | VehicleState | 50 Hz | one fan-out of position, velocity, attitude, resets, status. Validity flags false until each source is fresh |
| `/dyx3/estimator_health` | EstimatorHealth | 10 Hz | from `estimator_status_flags` |
| `/dyx3/gnss_report` | GnssReport | on sample | raw `SensorGps`, no interpretation |
| `/dyx3/px4_link/status` | Px4LinkStatus | 10 Hz | see section 6 |
| `/dyx3/ulog_chunk` | UlogChunk | on sample | after the ack was sent |
| subscribes `/dyx3/motion_guard/command` | MotionSetpoint | | the only command source |
| subscribes `/dyx3/rtcm` | RtcmData | | forwarded to `gps_inject_data` |

The RTCM subscription counts each chunk exactly once: `rtcm_chunks_accepted` after `GpsInjectData` publication returns, or `rtcm_chunks_dropped` when session/handshake/topic readiness or chunk size fails. These px4_link-process lifetime counters do not assert DDS delivery, PX4 handling, or receiver receipt.
The RTCM callback checks current per-topic freshness at callback time, rather than relying on the previous 100 Hz status cycle, so a session or topic that went stale between cycles is refused.
| service `/dyx3/px4_link/arm` | ArmDisarm | | arm needs a healthy link; disarm is always allowed |
| service `/dyx3/px4_link/set_offboard` | SetOffboard | | heartbeat first, then the mode command |

## 3. The setpoint stage

The link owns a fixed timer at `publish_rate_hz` (default 100, **never below 100** — validated;
the F1.7 contract says ≥ 100 Hz). Every tick publishes the full explicit-control set. The command
is the newest `MotionSetpoint` from the guard if it is *fresh*: age on the link's clock
≤ `command_max_age_s`, `valid=true`, a legal mode, and the field contract of `MotionSetpoint.msg`
satisfied (NaN pattern per mode, finite values, `speed_body_x` = 0 in STOP and PIVOT). Anything
else becomes STOP (section 4).

| Mode | offboard_control_mode | `trajectory_setpoint.velocity` | `rover_speed_setpoint.speed_body_x` | `rover_attitude_setpoint.yaw_setpoint` | `rover_rate_setpoint.yaw_rate_setpoint` |
|---|---|---|---|---|---|
| STOP | velocity=true, all other flags false | NaN×3 | 0 | NaN | 0 |
| TRACK_HEADING | same | NaN×3 | v | ψ | NaN |
| TRACK_RATE | same | NaN×3 | v | NaN | r |
| PIVOT | same | NaN×3 | 0 | NaN | r |
| CREEP | same | NaN×3 | v | NaN | r |

All five fields of the table are written **every cycle**; NaN is sent, never omitted (a missing
update leaves the firmware's cached finite value in force). In `trajectory_setpoint`, `position`,
`acceleration` and `jerk` are NaN, `yaw` and `yawspeed` are NaN, and `velocity` is NaN — PX4 must
never see a finite velocity from this link. `RoverSpeedSetpoint.speed_body_y` is NaN (not mecanum).
`TRACK_HEADING` publishes `rate = NaN`: the firmware attitude controller produces the rate.

All `timestamp` fields are the companion's **system (wall) clock** in microseconds (section 7).

## 4. Fail to zero (CLAUDE.md §7, F-tasks A1.1)

The link forces STOP (speed 0, yaw NaN, rate 0) in every one of these cases, evaluated each tick,
with no latching beyond the condition itself:

1. No guard command ever received since start.
2. Newest command older than `command_max_age_s` (counts one `command_gap_events` per gap).
3. Newest command `valid=false`, unknown mode, non-finite where finite is required, or breaking
   the per-mode contract.
4. `handshake_ok` false (never publish a setpoint on a format that is not proven).
5. `session_alive` false.
6. A monitored topic stale (section 5) — covers a state fan-out that silently stopped (#27388).
7. A command sequence reset (`seq` decreased): the first command of a new publisher session is
   discarded and one extra tick is STOP; that way a restarted RPP cannot resume an old motion.

Why STOP while keeping the heartbeat, rather than going silent: F1.7 asks the companion to stop
publishing `offboard_control_mode` once RPP output is stale, because a live heartbeat with *stale
setpoints* is the hole `COM_OF_LOSS_T` cannot see. The link never publishes a stale setpoint: it
publishes a **fresh explicit zero**, which is the strongest command available and keeps the
vehicle in OFFBOARD instead of triggering PX4's own loss action (`COM_OBL_RC_ACT`) while the
guard recovers. The heartbeat is withdrawn when the link itself cannot publish a trustworthy
zero: `handshake_ok` false or `session_alive` false. `set_offboard(false)` withdraws it only after
an explicit STOP window (section 9).
DERIVED — NOT FROM V1 SPEC: this interpretation of the F1.7 obligation. Flagged for the human.

`command_max_age_s` is DERIVED from the prototype's `input_max_age_s` 0.2 (re-validate at GATE 4):
the guard publishes its command at the RPP rate (50 Hz), so 0.2 s is ten missed ticks.

**Orderly shutdown (X-010).** rclcpp's signal handler is disabled: SIGINT/SIGTERM only set a flag,
the executor loop (`spin_once`, 5 ms) exits, and, while the context is still up and without
spinning the writer timer again, the explicit STOP set is published at 100 Hz for 0.3 s
(`Px4LinkNode::publish_shutdown_stop`). Only where the heartbeat was running on the last tick; no
heartbeat is ever started at shutdown. Then the process exits and PX4's offboard-loss handling
runs with STOP as the last setpoint. SIGPIPE is ignored.

**Not covered here, on purpose:** if this *process* dies (SIGKILL, crash, hang) or the agent,
Ethernet or Jetson goes, PX4 keeps the last setpoint for up to `COM_OF_LOSS_T` (upstream #27514,
~900 ms measured). That is firmware territory (F5 / A1.1) and the reason `COM_OF_LOSS_T` needs a
human value; the stop bound is an acceptance measurement (PXL-001). This package's obligation is
the in-process part.

## 5. Per-topic staleness (upstream #27388)

`uxrce_dds_client` can silently stop publishing one topic while the session stays up. Each
monitored topic has its own last-receive time on the link's clock (steady). A topic is stale when
its age exceeds its limit. Bit assignment (frozen with `Px4LinkStatus`):

| Bit | Topic | Limit parameter | Default | Source |
|---|---|---|---|---|
| 0 | timesync_status | `stale_timesync_s` | 3.0 | MEASURED on the rover 2026-10-08: period 1.010 s (σ 0.3 ms); 1.0 flapped the session every second → three missed samples |
| 1 | vehicle_local_position | `stale_local_position_s` | 0.2 | MEASURED 50 Hz (`/fmu/out/vehicle_local_position_v1`); 10 missed |
| 2 | vehicle_status | `stale_vehicle_status_s` | 1.0 | MEASURED 1.98 Hz (0.506 s, `/fmu/out/vehicle_status_v1`) while idle, not the 5 Hz rate limit; ~2 missed |
| 3 | vehicle_attitude | `stale_attitude_s` | 0.2 | MEASURED 100 Hz; 20 missed |
| 4 | estimator_status_flags | `stale_estimator_flags_s` | 3.0 | MEASURED 0.99 Hz (1.010 s); 1.0 flapped every second → three missed. Revisit when the firmware rate rises (timing contract) |
| 5 | vehicle_gps_position | `stale_gps_s` | 1.0 | MEASURED 5.0 Hz (UM982); 5 missed |

All limits are DERIVED, none comes from the spec or a bag: they are `IDLE_ONLY`, validated
positive, and must be re-validated at GATE 4 from recorded topic periods
(`tools/` extraction, LOCAL ACTION). A topic that was never received is stale. The session is
alive only while bit 0 is clear. `session_resets` counts transitions alive→dead→alive, and a
reset **re-arms the handshake** (an agent restart or FCU reboot may bring different firmware).

## 6. Handshake (architecture 4.6, F3)

The link refuses to publish setpoints until every used topic's format is proven identical on both
sides. For each topic in the used set the link publishes `MessageFormatRequest`
(`protocol_version=1`) and expects a `MessageFormatResponse` with `success` and a `message_hash`
equal to the hash it computes from the **installed** `px4_msgs` `.msg` definitions.

- Hash: FNV-1a 32-bit (offset `0x811c9dc5`, prime `0x1000193`) over the concatenation of
  `"<type> <name>\n"` for every non-constant field in file order, nested message types expanded
  recursively after the field that names them (`Tools/msg/px_generate_uorb_topic_helper.py`,
  `get_message_fields_str_for_message_hash`). Array suffixes stay in the type (`float32[3]`).
- **The request carries the base topic name.** The firmware matches the last path component
  against the uORB name (`uxrce_dds_client.cpp:448-455`), which has no `_v1` suffix; the
  request uses `/fmu/<in|out>/<uorb_name>` for versioned types too.
- Request timeout `handshake_timeout_s`; retried every `handshake_retry_s` until the response
  arrives. A response with `success=false` or a different hash is `FAULT_HANDSHAKE_MISMATCH`:
  **latched until the session resets or the process restarts**, fail loud (error log, status). A session
  reset (agent restart, FCU reboot or reflash) re-arms the handshake: the other side may now be
  different firmware. (`Px4LinkStatus.handshake_ok` text says "permanent for the process lifetime";
  that is stricter than this and will be corrected at the next interface version bump.)
  Until all responses arrive the fault is `FAULT_HANDSHAKE_PENDING` (not permanent).
- `.msg` definitions come from the installed `px4_msgs` (`share/px4_msgs/msg/*.msg`). A missing
  definition is a mismatch, not a skip.
- The pure hash function is proven against the firmware's own Python implementation, run
  unmodified through a parsing shim over every message of the pinned firmware
  (`tools/px4_msg_hash/`); vectors in `test/fixtures/px4_msg_hash_vectors.txt`.

## 7. Time

The firmware converts timestamps in both directions inside its uXRCE-DDS (de)serialisers
(`Tools/msg/templates/ucdr/msg.h.em:139-169`: outgoing `timestamp + time_offset`, incoming
`min(timestamp - time_offset, hrt_absolute_time())`, offset from the client's timesync against the
agent's system clock). Consequence: **the link stamps every message it publishes with the Jetson
system clock in microseconds and never applies `TimesyncStatus.estimated_offset` itself**;
`TimesyncStatus` is used for liveness only. PX4 timestamps seen on `/fmu/out` are already in the
Jetson system-clock domain.
All freshness decisions use the link's own steady clock and the local arrival time, never message
timestamps, so a clock step on either side cannot make a stale sample look fresh.
`VehicleState.px4_sample_stamp` carries `timestamp_sample` of the local-position sample.

## 8. State fan-out

`VehicleState` is assembled at 50 Hz from the newest sample of each source. Mapping (NED, rad):
`x,y,z` to `north/east/down`; `vx,vy,vz` to `velocity_*`; `q` (FRD to NED) copied as is;
`heading` to `heading_rad`. Validity: `position_valid = xy_valid`, `velocity_valid = v_xy_valid`,
`attitude_valid = heading_good_for_control` and an attitude sample is fresh. Dead reckoning and
estimator faults are *not* folded into these flags; the guard owns those gates through
`EstimatorHealth`. A stale source clears its validity flags in the same tick. `yaw_rate_radps`
is 0 until a `vehicle_angular_velocity` source is added (it is not on DDS today); no validity
flag depends on it. `xy_reset_counter`, `delta_*` and the global reference are copied unchanged.

## 9. Arm and mode

`SetOffboard(enable=true)`: start publishing the heartbeat with explicit STOP, wait
`offboard_prestream_s` (default 0.5, DERIVED, PX4 needs a stream before it accepts OFFBOARD), then
publish `VehicleCommand` `VEHICLE_CMD_DO_SET_MODE` (176) with `param1=1`, `param2=6`
(`PX4_CUSTOM_MAIN_MODE_OFFBOARD`). Accepted only when the link is healthy. Success is declared
when `vehicle_status.nav_state == 14` within `offboard_confirm_timeout_s`, not on the command
being sent. `ArmDisarm`: `VEHICLE_CMD_COMPONENT_ARM_DISARM` (400) with `param1` 1 or 0; arm is
refused unless the link is healthy; confirmation by `vehicle_status.arming_state`
(`ARMED` 2 / `DISARMED` 1). Both set `from_external=true`, `source_system=1`,
`source_component=1`, `target_system=1`, `target_component=1` — DERIVED: stock companion
addressing; no figure exists in the spec.

Arm never starts motion by itself; the heartbeat carries STOP until the guard commands otherwise.

`SetOffboard(enable=false)` (PXL-002): from the next writer tick the heartbeat carries the explicit
STOP set, whatever the guard commands, for `offboard_disable_stop_s` (default 0.3 s, validated
> 0); then the heartbeat is withdrawn and PX4 leaves OFFBOARD through its own offboard-loss
handling (`COM_OF_LOSS_T`, `COM_OBL_RC_ACT`) with a zero as the last setpoint. Dropping the
stream at once would leave PX4 applying the last motion setpoint until the loss timeout. The
window only runs while the link is healthy (otherwise there is no trustworthy zero to send), and a
repeated disable does not restart it. The reply is immediate and unchanged: `accepted=true`,
`REASON_OK` means "the link stopped commanding motion and started the STOP window", **not** "PX4
left OFFBOARD" or "the rover stopped". No mode change or disarm is sent (owner policy, open).
`Px4LinkStatus.offboard_heartbeat_active` is true while the heartbeat is actually published,
including the STOP window.

## 10. ULog and RTCM

- `ulog_stream` chunks with `FLAGS_NEED_ACK` are acknowledged immediately (a `UlogStreamAck`
  with the same `msg_sequence`) *before* republishing as `UlogChunk`, because the FCU blocks its
  stream on the ack. The start command (`VEHICLE_CMD_LOGGING_START` 2510) is sent once per
  session when `ulog_streaming_enabled`.
- `RtcmData` becomes one `GpsInjectData` per chunk (`len = data.size() ≤ 300`, flags copied,
  `device_id = 0`). Oversized data is rejected, counted and never truncated. The link does
  not interpret RTCM and never touches the receiver configuration (hard requirement 2026-10-07).

## 11. Real-time discipline

The publish timer and the callbacks use pre-sized buffers and no heap allocation on the tick path;
logging on the tick is rate-limited and happens outside it. `mlockall` is requested at startup
(failure is logged, not fatal in a container). Loop lateness over 1.5 periods increments
`loop_overrun_count`; the fault is reported but does **not** stop the vehicle by itself — a
late tick is still a fresh setpoint, and the staleness checks above are what bound the harm.
The lifetime counter is never cleared. `FAULT_LOOP_OVERRUN` remains visible for one second from
the most recent actual late tick and then clears if no higher-priority fault exists. Invalid or
backward steady-clock ticks do not advance the loop baseline or health state. They publish an
explicit STOP control set immediately, using the independent system-clock timestamp; the next
valid tick resumes the normal gate and overrun accounting. The same applies to nonfinite or
negative injected timestamps.

## 12. Parameters

| Name | Default | Class | Source |
|---|---|---|---|
| `publish_rate_hz` | 100 | RESTART | F1.7 (≥ 100); validated ≥ 100 |
| `command_max_age_s` | 0.2 | IDLE_ONLY | DERIVED from prototype `input_max_age_s`, re-validate GATE 4 |
| `stale_*_s` (6) | section 5 | IDLE_ONLY | DERIVED, re-validate GATE 4 |
| `handshake_timeout_s`, `handshake_retry_s` | 5.0, 1.0 | IDLE_ONLY | DERIVED |
| `offboard_prestream_s`, `offboard_confirm_timeout_s` | 0.5, 2.0 | IDLE_ONLY | DERIVED |
| `offboard_disable_stop_s` | 0.3 | IDLE_ONLY | DERIVED (PXL-002): ≥ 20 ticks of STOP, well inside `COM_OF_LOSS_T` 1.0 s; validated > 0 |
| `ulog_streaming_enabled` | true | IDLE_ONLY | |
| `spray_transaction_timeout_s` | 0.3 | RESTART | DERIVED (XR-GPX-001): pinned PX4 answers 187/183 at once; bounds how long a queued spray request can wait; validated > 0 |

## 13. Acceptance

Off-target (CI): hash vectors vs the firmware's own function, setpoint mapping per table, every
fail-to-zero case, staleness boundaries, handshake lifecycle (pending, ok, permanent mismatch,
re-arm after reset), a fake-PX4 fault-injection harness. **Not provable off-target:** DDS
transport, the real timesync, jitter on the Jetson, the stop distance after a killed process, the
actual `COM_OF_LOSS_T` behaviour. Those are GATE 4 / F5 on the bench.

## 14. Spray command/ACK transactions (D1 and Phase D)

Spray valve commands are serialized: at most one spray `VehicleCommand` is in flight, with a
bounded queue of 16 total in-flight/queued requests. While queued, the newest request from each
source replaces that source's older queued request; a replaced request receives a failed
`SprayActuatorAck` (`result=255`). A watchdog OFF supersedes every queued request from any other source (ON **and** OFF; each receives a
failed `SprayActuatorAck`, `RESULT_LINK_REFUSED`) and is placed at the front of the queue (commit `6b8b4b1`). If all queue capacity is occupied by watchdog OFF requests, a new request is
refused rather than displacing them. Link loss fails the in-flight request and all queued requests.

**An OFF never waits behind an in-flight ON** (XR-GPX-001). FCU ACKs arrive on a best-effort
topic, so one lost ACK would otherwise hold a line-end, watchdog or E-stop OFF until the ON timed
out. Any OFF (controller or watchdog) that arrives while an ON is in flight fails that ON
(`RESULT_LINK_REFUSED`), goes to the front of the queue and is dispatched in the same callback,
without waiting for a tick. A late ACK of the pre-empted ON cannot confirm the OFF (different
identity). An OFF does not pre-empt an in-flight OFF. An unanswered in-flight request is failed
after `spray_transaction_timeout_s` (default 0.3 s, validated > 0); until then every exact reassert
of it is republished with its identity.

Each dispatched logical proof epoch receives a durable `(VehicleCommand.source_system,
VehicleCommand.source_component)` identity. The allocator uses source systems 1..255 and component
IDs 2..999, in that order, for 254,490 nonreused pairs per PX4/companion correlation epoch.
System 0 is excluded because MAVLink uses it for broadcast/unspecified targeting. Component 1 is
reserved for the companion's ordinary command identity; 1000+ has PX4 mode-executor semantics.
The ordered-pair high-water mark is persisted at
`/var/lib/dyx3/state/px4_link_spray_ack_next` before any corresponding VehicleCommand is
published. It is reserved in blocks of 64 pairs (PXL-004): when the in-memory block is used up, the
mark is advanced by 64 with the same write + `fsync` + rename + directory `fsync`, and the next 64
pairs are handed out from memory. One durable write per 64 transactions instead of two `fsync`s
inside the 100 Hz writer tick per transaction. A restart (including after power loss) resumes at
the persisted mark, so the unused rest of a block is skipped (at most 63 pairs per restart) and no
pair is ever reused. A failed block write fails closed and latches until restart, as before. New writes use `v2 <next-index>`; the old numeric component ledger 2..1000 is read as
the corresponding system-1 high-water mark, preserving every previously allocated pair. The
first-install installer seeds the legacy-compatible value 2 only if the file does not exist.
Process restart and upgrade preserve the ledger. Missing, corrupt, unwritable, or exhausted state
fails closed, including watchdog OFF proof. Diagnostics expose identities used, remaining,
exhausted, and unmatched ACK count.

An exact logical request is keyed by producer source, sequence, ON/OFF intent, complete ROS
mapping, physical value/PWM, and translated PX4 command/parameters (NaNs compare equal).
Every physical reassert of that same request publishes another VehicleCommand with its original
pair. Its timestamp may change; its actuator command and correlation pair do not. An ACKed
reassert is still sent. A retry after a timeout of the same logical request keeps the pair:
a delayed ACK from any identical transmission safely proves that same request. A distinct
sequence, mapping, intent, or physical value creates a new pair. A watchdog OFF period keeps one
sequence across its 20 Hz burst and 2 Hz reasserts; it starts a new sequence when OFF was no
longer required and becomes required again, or when the physical mapping changes. D2 still
withdraws readiness immediately on mapping change.

ACK matching requires command ID **and both** ACK target system and target component. The pinned
firmware (`27a7ac92845317b0276776242c504215809b2a0f`) echoes command source system/component
in `Commander::answer_command`; command 187 gets an immediate ACCEPTED answer there, and 183 an
immediate UNSUPPORTED answer. Each physical transmission can produce a terminal ACK. Extra or
late ACKs for an old pair cannot prove a newer epoch because pairs are never reused. `IN_PROGRESS`
is not terminal. This is idempotent retry semantics: an earlier accepted ACK for the exact same
physical request may prove that request; it cannot prove changed mapping or intent.
Because every reassert draws its own ACK, an ACK whose command and pair match a producer's
confirmed or last dispatched epoch is expected and ignored (XR-GPX-005); only an ACK for a pair no
current epoch holds increments `spray_unmatched_ack_count`, with a warning throttled to one per
5 s.
When watchdog OFF displaces a controller ON, the link retires that specific controller ON
heartbeat: a late duplicate of it cannot reopen the valve after the OFF proof. A genuinely new
controller verdict has a new logical identity.

The exact firmware's actuator-set consumer uses command ID, actuator parameters and set index;
it does not inspect source system or component. Command 183 has no physical handler in the pinned
firmware and Commander reports UNSUPPORTED. Commander filters by *target* system/component; its
source-component special cases concern mode executors and unrelated mode commands. The uXRCE
input publishes the command fields unchanged. MAVLink does not forward commands marked
`from_external`; its ACK router can forward an ACK to a seen MAVLink peer according to the echoed
source system/component, but this does not affect DDS ACK publication or actuator execution.
The source-system range 1..255 therefore does not change the physical 183/187 command semantics.

The audit searched the entire pinned firmware tree for `vehicle_command` subscriptions,
183/187 handlers, source/target field reads, and uXRCE/MAVLink routing. The relevant paths are
`src/modules/uxrce_dds_client/dds_topics.yaml` and `vehicle_command_srv.cpp` (DDS input and ACK
output), `src/modules/commander/Commander.cpp` (target filter, immediate 183/187 answers,
source-to-ACK echo, mode-executor path), `src/lib/mixer_module/functions/FunctionActuatorSet.hpp`
(187 physical value), `src/modules/mavlink/streams/COMMAND_LONG.hpp` and
`mavlink_command_sender.cpp` (external command forwarding exclusion), and
`src/modules/mavlink/mavlink_main.cpp` (MAVLink ACK forwarding). Other subscribers handle other
command IDs; full-tree searches found no second 183/187 actuator handler and no 183/187 branch
whose physical effect depends on `source_system`. This finding is limited to the pinned SHA.

The allocator has no automatic reset. Reinitialization is permitted only as a controlled joint
PX4 and companion reset after the previous PX4 instance and DDS/uXRCE ACK stream are gone.
Neither a companion-only restart nor a software reinstall/upgrade establishes this boundary.
Without proof of that joint boundary, retain the ledger; never recycle or wrap an identity while
PX4 may still be alive. Exhaustion rejects actuator requests and cannot fabricate OFF proof.

The queue and token allocator are owned by `dyx3_px4_link`. Neither spray producer infers FCU completion
from publication; controller and watchdog state advance only from their matching
`SprayActuatorAck` sequence/source.

## Timesync evidence (interfaces 0.7.0)

`Px4LinkStatus` carries `timesync_valid`, `timesync_offset_us` (`estimated_offset`) and `timesync_round_trip_us` of the newest `TimesyncStatus`; zero and
not valid when the session is down or nothing arrived. **Values only.** F-tasks A1.4 (upstream #28519) says the offset can be ~40 ms for 5-10 minutes after boot
(1.4 cm at 0.35 m/s) and asks for "a boot-time warm-up as a mission precondition". **That gate is NOT built:** no numeric convergence criterion exists in any
source (the issue only says ~4 ms is expected), and inventing one would put an arbitrary number in a safety path. **OPEN (human):** the criterion (offset below X
for Y seconds?) and who gates on it (`dyx3_mission` precondition vs `dyx3_motion_guard`). Until then the evidence is recorded: the recorder logs it at run start
and end, and the gateway's telemetry shows it.
