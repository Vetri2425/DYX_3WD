# dyx3_system_gateway — contract

**Status:** draft for review, written before the implementation. **Spec:** V1 §7.10, §4.3.1 (operator-link loss), R13. **Authority:** none over motion or safety.
It is the **single ROS <-> backend boundary**: the backend (Python, no `rclpy`) talks to it over a Unix domain socket; it turns validated requests into ROS service calls and
publishes a canonical telemetry snapshot. A backend or tablet E-stop is a **request** to `dyx3_motion_guard`; the gateway never stops anything itself. The one thing it owns
is the **operator-link heartbeat** (§4.3.1): it publishes `OperatorLinkStatus`, and `dyx3_motion_guard` stops on `alive == false`.

## 1. Transport

* `SOCK_STREAM` Unix socket (`socket_path`, default `/run/dyx3/gateway.sock`, mode 0660, group `dyx3`). One instance per path (XR-GW-003): at start the gateway takes an exclusive `flock` on `<socket_path>.lock` and **refuses to start** while another instance holds it; holding the lock, a stale socket file (crashed run) is replaced. At stop it unlinks the socket only if it is still the inode it bound; the lock file itself stays (harmless, released by the kernel on exit). At most `max_clients` (4, DERIVED) connections.
* **Newline-delimited JSON, UTF-8**, one object per line, every message carries `"v":1`. A line longer than 64 KiB, invalid JSON, invalid UTF-8 inside a string (strict RFC 3629: no overlong forms, surrogates or code points above U+10FFFF; GW-008), a wrong `v`, duplicate keys or trailing garbage
  is answered `{"ok":false,"code":"bad_message"}` (the connection stays open unless the line overflows, which closes it). A client whose outbound buffer exceeds 1 MiB is dropped (slow consumer).
* Everything the gateway writes is valid UTF-8: client bytes are never echoed unless they passed validation, and any invalid byte in a ROS string field is written as `\ufffd`.
* The gateway never trusts the client: every field is validated (section 3) before anything reaches ROS.
* A client that disconnects (also with replies still queued for it) is simply removed: socket writes use `MSG_NOSIGNAL` and the node ignores `SIGPIPE`, so a backend restart can never take the gateway, and with it the control graph, down (GW-002).
* Writes never block the node: a reply, an event or a telemetry push only appends to that client's out buffer (bounded at 1 MiB, see above) and the socket thread writes it non-blocking. A slow or stuck client therefore costs only itself: it cannot stall the ROS executor, another client, a command or an E-stop. Every queue is bounded: the command inbox (section 2), the table of service calls awaiting an answer (64), each client's out buffer, and one pending event per kind.

### Client -> gateway
`{"v":1,"id":<int>,"cmd":"<name>","args":{...}}`  — `id` is echoed in the reply (client-chosen, not interpreted).

### Gateway -> client
* reply: `{"v":1,"id":<id>,"ok":<bool>,"code":"<snake_case>","reason":"<text>","data":{...}}`. Always this typed shape: `data` is an object (`{}` when there is nothing to report).
* telemetry push: `{"v":1,"type":"telemetry","snapshot":{...}}` at `telemetry_hz` (5, DERIVED) to every client.
* event push: `{"v":1,"type":"event","event":"<kind>",...}` to every client, the moment a status changes (section 1.3).

A client ignores a `type` it does not know, so message types can be added without a protocol version change; `v` changes only for an incompatible change.

### 1.3 Events (the status channel)

The gateway pushes a status change as an event the moment it sees it; the 5 Hz telemetry snapshot stays as the complete periodic picture. One event family:

```json
{"v":1,"type":"event","event":"mission_state","seq":42,"t_mono_s":18234.512039871,"t_wall_ms":1791624580734,
 "coalesced":0,"replay":false,
 "data":{"state":3,"mission_id":7,"run_index":0,"point_index":0,"reason_code":0,
         "path_artifact_sha256":"<64 hex>","source_artifact_sha256":"<64 hex>","request_id":"tab-1:9f2c",
         "reason_detail":"","gate_reason_code":0,"waiting_on":4,"state_entered":1791624580.5,
         "fresh":true,"stamp_s":1791624580.733912}}
```

| field | meaning |
|---|---|
| `event` | the kind (table below) |
| `seq` | per gateway process, starts at 1, +1 per pushed event across all kinds: the push order. It restarts at 1 when the gateway restarts (a smaller `seq` than the last one seen = a new gateway) |
| `t_mono_s` | gateway steady clock when the carried value was received |
| `t_wall_ms` | gateway wall clock (Unix ms) when the event was pushed |
| `coalesced` | transitions folded into this event and not pushed on their own (0 = none lost) |
| `replay` | `true` only on the connect replay (below) |
| `data` | the full current value of that kind |

| `event` | source | transition (pushed when it changes) | `data` |
|---|---|---|---|
| `mission_state` | `/dyx3/mission/state` | `state`, `reason_code` or `waiting_on` | the snapshot `mission` fields (below), `fresh`, `stamp_s` (the message stamp) |
| `operator_link` | the gateway's own operator link (section 4) | `alive` | `alive`, `age_s`, `cause`: `heartbeat` \| `timeout` \| `connection_closed` \| `never` |
| `fcu_link` | `/dyx3/px4_link/status` | `session_alive` or `handshake_ok` | `fresh`, `session_alive`, `handshake_ok`, `fault`, `session_resets` |
| `estop` | `/dyx3/emergency_stop_state` | `asserted` or `source` | `fresh`, `asserted`, `source` |

* **Stale is a transition.** All three topic sources publish periodically. When one is older than `snapshot_fresh_s` its kind is pushed once with `data` = `{"fresh":false}`: the state is unknown, never the last value. The next message is a transition again.
* **Coalescing** (`event_coalesce_s`, 10 ms): a transition is pushed at once unless an event of the same kind went out less than 10 ms ago; then it waits until 10 ms after that push (plus at most one 10 ms node tick), and every value arriving meanwhile folds into that one event, which carries the latest value and counts the folded transitions in `coalesced`. At most 100 events per second per kind. The topic sources are subscribed with a depth-10 history, so two transitions that arrive before the node runs are both seen (with depth 1 the first would be overwritten in DDS).
* **On connect** the gateway sends every new client the latest event of each kind, in `seq` order, with `"replay":true` and the original `seq`, before anything else it reads from that client. A client therefore has the current state at once after a (re)connect and never polls. A push can race the connect, so the same event can arrive both live and as a replay: **per kind, the event with the highest `seq` is the current one**; an event with a lower or equal `seq` than the last one of its kind is a duplicate.
* **The `mission_state` fields** (the snapshot's `mission.data` has exactly the same, minus `fresh`/`stamp_s`; mission contract v2, interfaces 0.15.0, `docs/contracts/dyx3_mission.md` sections 2 and 3). Every `MissionState` field is carried, none renamed or reinterpreted:

| field | meaning |
|---|---|
| `state` | 0 IDLE, 1 LOADING, 2 READY, 3 RUNNING, 4 PAUSED, 5 COMPLETED, 6 ABORTED, 7 ERROR, 8 PLACING, 9 ARMING, 10 ENGAGING |
| `mission_id` | the **execution id** (the same as `StartMission`'s reply) |
| `run_index`, `point_index` | progress |
| `reason_code` | 0 NONE, 1 OPERATOR, 2 SAFETY, 3 RTK, 4 PATH_ERROR, 5 INTERNAL_ERROR, 6 EKF_RESET, 7 EKF_REFERENCE_INVALID, 8 PLACEMENT_OUT_OF_BOUNDS, 9 NO_PLACEMENT_FRAME, 10 ARM_REFUSED, 11 ARM_TIMEOUT, 12 OFFBOARD_REFUSED, 13 OFFBOARD_TIMEOUT, 14 RPP_ACK_TIMEOUT, 15 ESTOP, 16 RPP_ERROR, 17 RPP_STALE |
| `gate_reason_code` | with `reason_code` 2 / 3: the guard gate that failed (`MotionSetpointStatus.REASON_*`), else 0 |
| `reason_detail` | human-readable cause (and `release: ...` for a failed release step); `""` when none |
| `path_artifact_sha256` | the **execution** artifact RPP loads (the placed trajectory); `""` when nothing is placed |
| `source_artifact_sha256` | the artifact the operator started; kept until the next start |
| `request_id` | the `start_mission` idempotency key of this execution; `""` when none was given |
| `waiting_on` | the step the lifecycle waits on: 0 NONE, 1 ARTIFACT, 2 PLACEMENT, 3 ARM, 4 OFFBOARD, 5 RPP_ACK, 6 OPERATOR, 7 OFFBOARD_RELEASE, 8 DISARM |
| `state_entered` | when the current state was entered (the mission node's ROS clock, seconds since the epoch, float64) |

  The transition key of `mission_state` is `(state, reason_code, waiting_on)`: a change of the step being waited on is an event even when state and reason stay (a release moving from `OFFBOARD_RELEASE` to `DISARM`, or ending in `NONE`). `reason_detail`, `gate_reason_code` and `state_entered` change only together with one of the three. The gateway does not interpret any value; it passes the numbers through (the names above are for readers).

## 2. Commands

| `cmd` | `args` | ROS target | Notes |
|---|---|---|---|
| `heartbeat` | `{}` | (internal) | the tablet heartbeat relayed by the backend; see section 4 |
| `get_snapshot` | `{}` | (internal) | reply `data` = the telemetry snapshot |
| `start_mission` | `{"path_artifact_sha256": "<64 lowercase hex>", "request_id"?: "<1..64 of [A-Za-z0-9._:-]>"}` | `StartMission` | `request_id` (optional): the client's idempotency key, passed to `StartMission.request_id` and echoed as `data.request_id` in every reply to that command (accepted, rejected, `timeout`, `service_unavailable`, `busy`). The bound 1..64 is the one `dyx3_mission` and the backend enforce, so a longer or malformed id is refused here: `invalid_command`, nothing reaches ROS. **Reply `data`:** `accepted`, `reason_code`, `mission_id` (the execution id), `duplicate` (the `request_id` matched the most recent execution: that execution's `mission_id`, nothing new started) and `gate_reason_code` (with `reason_code` 3 SAFETY_GATE: the guard's first failing pre-arm gate). A mission refusal is `rejected` with these fields verbatim, **except** `reason_code` 4 INVALID_REQUEST (the mission node refused the id; the gateway's own check makes this unreachable unless the two bounds drift apart): that is `invalid_command`, `data` unchanged. Accepting a start does not mean it will run: progress and a failure arrive as `mission_state` events. |
| `abort_mission` | `{"reason": "operator"\|"safety"\|"unspecified"}` | `AbortMission` | |
| `pause_mission` / `resume_mission` / `skip_point` | `{}` | `PauseMission` / `ResumeMission` / `SkipPoint` | |
| `estop` | `{"asserted": bool, "source": "tablet"\|"backend"\|"ble"\|"physical"}` | `SetEmergencyStop` (motion_guard) | **never queued behind other commands, never rate limited, never refused `busy`** |
| `arm` | `{"arm": bool}` | `ArmDisarm` (px4_link) | |
| `offboard` | `{"enable": bool}` | `SetOffboard` (px4_link) | |
| `spray_manual` | `{"on": bool}` | `SetSprayManual` (spray) | |

Commands are queued from the socket thread, which also wakes the node's executor (a guard condition), so a batch is processed at once; the node's 10 ms timer remains as the periodic tick. Within a batch an `estop` is processed strictly first, then the heartbeat,
then the rest in arrival order. Heartbeats in one batch are coalesced: only the most recent one refreshes the operator link, every one is answered `ok`. At most 256 commands
wait between two batches; beyond that every command except `estop` (heartbeats included) is answered `busy` (GW-005). At most 64 service calls await their answer at once; beyond that every command except `estop` is answered `busy`.

Every service call is asynchronous and has its own deadline: a command that waits (an `arm` or `offboard` px4_link is still confirming) never delays another command, an `estop`, an event or the telemetry push.

Unknown `cmd`, unknown `args` keys, missing or wrongly typed fields -> `invalid_command`. The gateway adds **no** policy of its own (no "arm only if ..."): the safety/mission authorities
downstream decide and their `accepted` / `reason_code` are returned verbatim in `data` (`{"accepted":..,"reason_code":..}` plus service-specific fields). If the target service is not
available the reply is `service_unavailable` at once; if it does not answer within its command's timeout (below) the reply is `timeout` and the request is also removed from the ROS client, so a late answer is discarded and never-answered requests do not accumulate (GW-007) — **an E-stop request that could not be delivered is reported as failed, never as accepted.**

**Per-command timeouts** (each must exceed the time the target itself may legitimately take to answer, or a success is reported as `timeout`):

| commands | parameter | default | why |
|---|---|---|---|
| `estop` | `estop_timeout_s` | 1.0 | motion_guard answers inside its service callback, with no downstream wait. A failed E-stop must reach the operator first (so the physical stop is used): it must not exceed any other timeout (checked at start). |
| `start_mission`, `pause_mission`, `resume_mission`, `abort_mission`, `skip_point`, `spray_manual` | `service_timeout_s` | 2.0 | fast accepts: the target decides and answers at once, progress comes as `mission_state` events. `StartMission` is an admission (no file read, no PX4 call; mission contract v2), so 2.0 s is generous; the default is unchanged. |
| `arm` | `arm_timeout_s` | 4.0 | px4_link answers after PX4 confirms the arming state, within its `arm_confirm_timeout_s` (2.0); `vehicle_status` arrives at 2 Hz, so 2 s of margin covers that and DDS. |
| `offboard` | `offboard_timeout_s` | 5.0 | px4_link answers after `offboard_prestream_s` (0.5) + `offboard_confirm_timeout_s` (2.0) + its 1.0 s service margin = 3.5 s; 1.5 s of margin. |

The old single 2.0 s returned `timeout` for an arm or OFFBOARD that px4_link later confirmed. All four are at most 30 s. A client's own request timeout must exceed the largest of them (the backend's `request_timeout_s` >= 6.0 s with these defaults).

## 3. Telemetry snapshot

One JSON object, assembled from the latest message of each source with its receive age. `null` = never received. Every source carries `age_s` (gateway steady clock) and `fresh`
(age <= `snapshot_fresh_s`, default 1.0, DERIVED); a consumer must treat a stale or missing source as unknown, never as the last value. Sources: `vehicle_state`, `estimator_health`,
`rtk_status`, `gnss_report`, `ntrip_status`, `px4_link`, `safety_gate`, `emergency_stop`, `motion_guard`, `rpp`, `mission`, `last_point_result`, `spray`, `recorder`, plus `gateway`
(`operator_alive`, `operator_age_s`, `clients`, `schema`, and `ipc`: the cumulative `dropped_slow` / `overflows` / `rejected_full` socket counters, `event_seq` (the last event pushed) and `dispatch_last_us` / `dispatch_max_us` (command line received -> ROS service request sent), all added in place without a protocol version change). `safety_gate` carries `ok`, `reason_code` and, since interfaces 0.15.0, `pre_arm_ok` (every guard gate except armed and OFFBOARD, plus the EKF global reference) with `pre_arm_reason_code` (the first failing pre-arm gate, meaningful while `pre_arm_ok` is false). Field subsets are chosen for the tablet; the recorder, not the gateway, is the evidence path.
The existing `ntrip_status` subset includes the selected security mode, verified TLS state, verification failure, plaintext credential warning, source bytes, valid frames, and RTK handoff count. The existing `px4_link` subset includes accepted/refused RTCM chunk counts. No credential or Authorization value is serialized. These are observation fields, not an RTK control API.

## 4. Operator link (R13)

`OperatorLinkStatus` is published at 10 Hz regardless of connected clients; every `alive` transition is also pushed at once as an `operator_link` event (section 1.3). `alive == true` iff **the connection that sent the last valid `heartbeat` is still connected AND that heartbeat arrived within `operator_link_timeout_s`**.
The heartbeat is bound to its connection (GW-001): when that connection closes, `alive` drops at the next publish, even if other clients stay connected; another client counts only after it sends its own `heartbeat`.
Never heard, heartbeating client gone, or timeout -> `alive == false`, `age_s` = time since the last heartbeat (0 when never). A dead backend therefore also reads as a dead operator link (fail-safe).
**TODO (GW-001):** the socket does not authenticate its peer. Every service runs as the same `dyx3` uid, so `SO_PEERCRED` cannot tell the backend from any other `dyx3` process; a peer-uid check needs a dedicated backend uid first (BR-005). Until then file mode 0660 / group `dyx3` is the only access control.
The backend must send `heartbeat` only while the *tablet* is heartbeating to it, so a tablet WiFi dropout propagates through.
**`operator_link_timeout_s` default 2.0 is DERIVED — NOT FROM V1 SPEC** (§4.3.1 says "design explicitly in Stage E", no number exists). **OPEN (human):** the value; too short stops the rover on WiFi
jitter, too long drives on after a dropout (at 0.35 m/s, 2 s = 70 cm). Boot behaviour: `alive == false` until the first heartbeat, so the guard blocks motion until a tablet is present.

### Audit log (XR-GW-001)
The node logs at WARN: every operator-link `alive` transition (with the reason: heartbeat timeout or heartbeating connection closed), every E-stop request with its
`source` and client and its outcome (accepted / rejected / unavailable / timeout), every service call that times out or finds its service unavailable, and, checked once a
second, any increase of the dropped-slow, overflow and refused-at-`max_clients` counters. Client count changes are logged at INFO.

## 5. Parameters (RESTART)

`socket_path`, `max_clients` 4, `telemetry_hz` 5, `operator_link_timeout_s` 2.0, `service_timeout_s` 2.0, `estop_timeout_s` 1.0, `arm_timeout_s` 4.0, `offboard_timeout_s` 5.0, `snapshot_fresh_s` 1.0, `operator_link_hz` 10, `event_coalesce_s` 0.01 (in [0, 1]). Invalid values stop the node at start (timeouts finite, > 0, <= 30 s, `estop_timeout_s` not above any other).

## 6. Not in this package

Authentication/authorisation of the tablet (backend), the REST/Socket.IO surface (backend), joystick/manual drive (**not ported, see backend questions**), BLE. The socket's file mode/group is the only
local access control; the backend is the policy owner for who may send what.

## 7. Acceptance

Off-target: JSON parser (strict, duplicate keys, depth, escapes), command validation table, snapshot ageing, operator-link timing, the socket server (framing, oversized line, slow consumer, max clients,
stale socket), and a node test with fake services (reply routing; an E-stop that is accepted, rejected, delivered but never answered (exactly `timeout`) and undeliverable (exactly `service_unavailable`), never reported accepted unless the guard accepted it; heartbeat -> `OperatorLinkStatus`).
Events: ordering, coalescing and replay (core); every kind pushed by the node, stale pushes, the connect replay; per-command deadlines (no `timeout` for an OFFBOARD still inside its 5 s); `request_id` passed, bounded to 64 (65 refused at the gateway, nothing sent to ROS) and echoed (also on `timeout`); the start reply's `mission_id`, `duplicate` and `gate_reason_code`, and mission INVALID_REQUEST typed `invalid_command`; every v2 `MissionState` field in the event and the snapshot, and a `waiting_on` change alone producing an event; `safety_gate.pre_arm_ok` / `pre_arm_reason_code`; an E-stop during a pending 5 s OFFBOARD wait is dispatched and answered at once; a client that never reads is dropped while every event still reaches the other client at once.
**Latency targets** (measured by `gateway_node_test` in a container on the development Mac, executor on its own thread; 20 repeats under full CPU load): MissionState publish -> event line on the socket, p50 < 5 ms (measured 0.4-0.5 ms idle, p95 < 3 ms loaded); command line received -> ROS service request sent, mean < 2 ms (measured 0.03 ms idle, 0.06-0.25 ms loaded).
**Not provable off-target:** socket permissions under systemd, the real tablet path, latency on the Jetson.
