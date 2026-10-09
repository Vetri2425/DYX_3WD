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

### Client -> gateway
`{"v":1,"id":<int>,"cmd":"<name>","args":{...}}`  — `id` is echoed in the reply (client-chosen, not interpreted).

### Gateway -> client
* reply: `{"v":1,"id":<id>,"ok":<bool>,"code":"<snake_case>","reason":"<text>","data":{...}}`
* telemetry push: `{"v":1,"type":"telemetry","snapshot":{...}}` at `telemetry_hz` (5, DERIVED) to every client.

## 2. Commands

| `cmd` | `args` | ROS target | Notes |
|---|---|---|---|
| `heartbeat` | `{}` | (internal) | the tablet heartbeat relayed by the backend; see section 4 |
| `get_snapshot` | `{}` | (internal) | reply `data` = the telemetry snapshot |
| `start_mission` | `{"path_artifact_sha256": "<64 lowercase hex>"}` | `StartMission` | |
| `abort_mission` | `{"reason": "operator"\|"safety"\|"unspecified"}` | `AbortMission` | |
| `pause_mission` / `resume_mission` / `skip_point` | `{}` | `PauseMission` / `ResumeMission` / `SkipPoint` | |
| `estop` | `{"asserted": bool, "source": "tablet"\|"backend"\|"ble"\|"physical"}` | `SetEmergencyStop` (motion_guard) | **never queued behind other commands, never rate limited, never refused `busy`** |
| `arm` | `{"arm": bool}` | `ArmDisarm` (px4_link) | |
| `offboard` | `{"enable": bool}` | `SetOffboard` (px4_link) | |
| `spray_manual` | `{"on": bool}` | `SetSprayManual` (spray) | |

Commands are queued from the socket thread and processed in batches by the node's 10 ms timer. Within a batch an `estop` is processed strictly first, then the heartbeat,
then the rest in arrival order. Heartbeats in one batch are coalesced: only the most recent one refreshes the operator link, every one is answered `ok`. At most 256 commands
wait between two batches; beyond that every command except `estop` (heartbeats included) is answered `busy` (GW-005).

Unknown `cmd`, unknown `args` keys, missing or wrongly typed fields -> `invalid_command`. The gateway adds **no** policy of its own (no "arm only if ..."): the safety/mission authorities
downstream decide and their `accepted` / `reason_code` are returned verbatim in `data` (`{"accepted":..,"reason_code":..}` plus service-specific fields). If the target service is not
available the reply is `service_unavailable` at once; if it does not answer within `service_timeout_s` (2.0, DERIVED) the reply is `timeout` and the request is also removed from the ROS client, so a late answer is discarded and never-answered requests do not accumulate (GW-007) — **an E-stop request that could not be delivered is reported as failed, never as accepted.**

## 3. Telemetry snapshot

One JSON object, assembled from the latest message of each source with its receive age. `null` = never received. Every source carries `age_s` (gateway steady clock) and `fresh`
(age <= `snapshot_fresh_s`, default 1.0, DERIVED); a consumer must treat a stale or missing source as unknown, never as the last value. Sources: `vehicle_state`, `estimator_health`,
`rtk_status`, `gnss_report`, `ntrip_status`, `px4_link`, `safety_gate`, `emergency_stop`, `motion_guard`, `rpp`, `mission`, `last_point_result`, `spray`, `recorder`, plus `gateway`
(`operator_alive`, `operator_age_s`, `clients`, `schema`, and `ipc`: the cumulative `dropped_slow` / `overflows` / `rejected_full` socket counters, added in place without a protocol version change). Field subsets are chosen for the tablet; the recorder, not the gateway, is the evidence path.
The existing `ntrip_status` subset includes the selected security mode, verified TLS state, verification failure, plaintext credential warning, source bytes, valid frames, and RTK handoff count. The existing `px4_link` subset includes accepted/refused RTCM chunk counts. No credential or Authorization value is serialized. These are observation fields, not an RTK control API.

## 4. Operator link (R13)

`OperatorLinkStatus` is published at 10 Hz regardless of connected clients. `alive == true` iff **the connection that sent the last valid `heartbeat` is still connected AND that heartbeat arrived within `operator_link_timeout_s`**.
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

`socket_path`, `max_clients` 4, `telemetry_hz` 5, `operator_link_timeout_s` 2.0, `service_timeout_s` 2.0, `snapshot_fresh_s` 1.0, `operator_link_hz` 10. Invalid values stop the node at start.

## 6. Not in this package

Authentication/authorisation of the tablet (backend), the REST/Socket.IO surface (backend), joystick/manual drive (**not ported, see backend questions**), BLE. The socket's file mode/group is the only
local access control; the backend is the policy owner for who may send what.

## 7. Acceptance

Off-target: JSON parser (strict, duplicate keys, depth, escapes), command validation table, snapshot ageing, operator-link timing, the socket server (framing, oversized line, slow consumer, max clients,
stale socket), and a node test with fake services (reply routing; an E-stop that is accepted, rejected, delivered but never answered (exactly `timeout`) and undeliverable (exactly `service_unavailable`), never reported accepted unless the guard accepted it; heartbeat -> `OperatorLinkStatus`).
**Not provable off-target:** socket permissions under systemd, the real tablet path, latency.
