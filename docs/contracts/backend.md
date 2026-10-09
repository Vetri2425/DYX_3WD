# backend — contract

**Status:** draft for review, written before the implementation. **Spec:** V1 §7.10, §7.2, §4.3.1; CLAUDE.md §3 and §12. **Authority:** none over motion.
The backend (Python, FastAPI + Socket.IO) owns REST, Socket.IO, auth, mission upload/report, telemetry delivery and path/CAD ingestion. It **never imports `rclpy`**,
sends no PX4 command and has no safety logic: every command goes to `dyx3_system_gateway` over its Unix socket (`docs/contracts/dyx3_system_gateway.md`), and a backend
or tablet E-stop is a **request** to `dyx3_motion_guard`. If the gateway is not connected, a command is **not delivered and the caller is told so** (never queued, never retried later).

## 1. HTTP API (all under `/api`, JSON; auth = `Authorization: Bearer <token>` except `/api/ping`)

| Route | Role | Notes |
|---|---|---|
| `GET /ping` | none | liveness, plus `rover_id` and `rover_name` (identity, no secret; see 1a) |
| `GET /health` | viewer | backend, gateway connection, age of the last telemetry, tablet heartbeat, `relay_running` (the relay task is alive), `operator_alive` (relay running, tablet fresh, and a relayed heartbeat acknowledged by the gateway within `tablet_heartbeat_timeout_s`; the gateway's own timer stays authoritative) |
| `POST /missions` (multipart: `file` + optional `origin_n`, `origin_e`, `rotation_deg`, `unit_scale`, `close_loop`, `anchor`) | operator | upload -> path engine -> `DYX3PATH 1` artifact stored by sha256 (idempotent) -> summary |
| `GET /missions`, `GET /missions/{sha}`, `GET /missions/{sha}/path` | viewer | stored artifacts, summary, points |
| `POST /missions/{sha}/start` | operator | gateway `start_mission` |
| `POST /mission/abort` `{reason}` / `pause` / `resume` / `skip_point` | operator | gateway |
| `POST /estop` `{asserted}` | **assert: any authenticated role; clear: operator** | gateway `estop` with `source = "tablet"` |
| `POST /vehicle/arm` `{arm}`, `POST /vehicle/offboard` `{enable}`, `POST /spray/manual` `{on}` | operator | gateway |
| `POST /heartbeat` | any authenticated | the tablet heartbeat (section 3) |
| `GET /telemetry` | viewer | the latest gateway snapshot, with its age |
| `GET /runs`, `GET /runs/{id}` | viewer | the recorder's `summary.json` / `manifest.json` (read-only) |

Error mapping of a gateway verdict: `ok` -> 200; downstream `rejected` -> 409 (body carries the downstream `reason_code`, verbatim); `invalid_command` -> 400; `service_unavailable` -> 503;
`timeout` -> 504; gateway not connected -> 503 with `"delivered": false`. Every error body: `{"ok":false,"code":...,"reason":...,"delivered":bool,"data":...}`.
An upload is rejected (413/415/422) for: size over `upload_max_bytes`, extension not `.dxf`/`.csv`/`.waypoints`, an engine error (message returned), or an artifact the reader would refuse. The uploaded name is never used as a path.

**Admission (before any body byte is read).** Every HTTP request under `/api` except `GET /api/ping` first passes an ASGI layer
(`api/admission.py`; it wraps the FastAPI app only, Socket.IO is not behind it):
- a missing or unknown bearer token -> **401** (`{"detail": ...}`, `WWW-Authenticate: Bearer`) without reading the body, so a malformed body
  from an unauthenticated client is 401, not 422. The role check (403) stays in the route;
- a body over the route's cap -> **413** `{"ok":false,"code":"too_large",...}`, from `Content-Length` before reading, or as soon as the
  streamed byte count passes the cap (chunked bodies). Caps: `POST /missions/plan` = `upload_max_bytes`; the multipart uploads
  `POST /missions` and `POST /path/parse-dxf` = `upload_max_bytes` + 64 KiB multipart envelope (the route still checks the file
  against `upload_max_bytes` exactly); every other route = `json_body_max_bytes` (`DYX3_JSON_BODY_MAX_BYTES`, default 64 KiB, DERIVED);
- a non-numeric `Content-Length` -> 400;
- on the JSON routes (not the uploads), a body nested deeper than 32 levels (DERIVED) -> **400** `{"ok":false,"code":"bad_request",...}`,
  checked on the stream before FastAPI parses it (deep nesting otherwise ends in a parse error or a 500, depending on the Python version).

Artifact reads for `GET /missions/{sha}`, `GET /missions/{sha}/path` (including rendering its JSON) and `POST /missions/{sha}/start`
run in a worker thread, never on the event loop (BE-005).

**Planning budget (BE-004).** DXF planning (`POST /missions`), app-plan parsing and compiling (`POST /missions/plan`) and DXF parsing
(`POST /path/parse-dxf`) run in a separate, freshly spawned process (`mission/planner.py`), never on the event loop and never in a
thread that shares the GIL with the heartbeat relay:
- **one job at a time**: a second planning request while one runs -> **409** `{"ok":false,"code":"busy",...}` (retry later);
- wall-clock budget `plan_timeout_s` (`DYX3_PLAN_TIMEOUT_S`, default 60 s, DERIVED, process start-up included): over budget the
  process is terminated (then killed) -> **422** `plan_budget_exceeded`;
- point budget `plan_max_points` (`DYX3_PLAN_MAX_POINTS`, default 200 000, DERIVED) for a DXF plan -> **422** `points_limit_exceeded`
  (an app plan keeps its own 50 000-point limit);
- a JSON body nested too deeply -> **400** `INVALID_PAYLOAD`; a planning process that dies without a result -> **500** `planner_crashed`.


## 1a. Rover identity and LAN discovery

- **Identity:** `rover_id` is stable across IP changes, network switches, reboots and upgrades:
  `DYX3_ROVER_ID`, else `dyx3-` + the first 10 hex characters of sha256(`/etc/machine-id`).
  `rover_name` is `DYX3_ROVER_NAME`, else the hostname. Both come from `/etc/dyx3/backend.env`.
  The app keys its saved operator token by `rover_id`, so one token works on every network.
- **Beacon:** every `DYX3_BEACON_INTERVAL_S` (default 1.0 s, DERIVED) the backend sends one UDP datagram to
  port `DYX3_BEACON_PORT` (default 5003) at the subnet broadcast address of **each** IPv4 network it is on.
  Each datagram carries the address the rover has on that network:
  `{"type":"dyx3_beacon","v":1,"rover_id":"…","rover_name":"…","ip":"192.168.3.150","port":8000}`.
  - Networks in `DYX3_BEACON_EXCLUDE` (default the FCU link `10.41.10.0/24`) are never beaconed.
  - `DYX3_BEACON_ENABLE=0` turns the beacon off.
  - The beacon carries no secret and controls nothing; a send failure never affects the API.
  - The pattern comes from the 4WD prototype (UDP 5002); 3WD uses its own port and type so the two apps never mix.

## 2. Auth (DERIVED — NOT FROM V1 SPEC; OPEN)

Static bearer tokens with a role, stored **hashed** (sha256) in `auth_file` (default `/var/lib/dyx3/state/auth.json`, `{"version":1,"tokens":[{"name","role","sha256"}]}`); comparison is constant time.
Roles: `viewer` < `operator`. **No token file or an empty one means every protected route is denied** (fail closed). `python -m dyx3_backend.auth.tokens create --name N --role R` prints a new token once and
adds its hash. Socket.IO connects with `auth={"token": ...}`. A user/password model, pairing, token rotation and expiry are **not designed**: OPEN (human).

## 3. Tablet heartbeat -> operator link (R13)

The tablet sends `POST /api/heartbeat` or the Socket.IO event `heartbeat`. The backend relays a gateway `heartbeat` every `heartbeat_relay_s` **only while the tablet heartbeat is younger than `tablet_heartbeat_timeout_s`**,
and the gateway turns that into `OperatorLinkStatus` (its own `operator_link_timeout_s`). The last authenticated Socket.IO connection closing clears the tablet heartbeat at once. A tablet WiFi dropout
therefore stops the relay and `dyx3_motion_guard` stops the rover. The relay task never ends on an error: an unexpected exception in a tick is
logged and that tick relays nothing (fail to STOP), and the next tick runs as usual.
**Stall guard (XR-BE-002).** A tablet heartbeat is stamped when the event loop processes it, so after a loop stall the queued heartbeats would
look fresh. The relay measures the gap between its own ticks; when it exceeds `heartbeat_relay_s` + 0.3 s (DERIVED), heartbeats stamped since
the previous tick are discarded (the stamp held at that tick is kept) and heartbeats stamped within the next `heartbeat_relay_s` are ignored
(the REST/Socket.IO answer is unchanged). A live tablet re-proves itself with its next heartbeat; a slow gateway reply trips the same guard,
in the fail-to-STOP direction. `heartbeat_relay_s` 0.5 and `tablet_heartbeat_timeout_s` 1.5 are **DERIVED — NOT FROM V1 SPEC**; the worst-case delay is
`tablet_heartbeat_timeout_s + gateway operator_link_timeout_s` (3.5 s with the defaults = 1.2 m at 0.35 m/s). **OPEN (human):** the numbers.

## 4. Socket.IO

`/socket.io`, connect requires a valid token. Server -> client: `telemetry` (every gateway push, same body as `GET /telemetry`), `gateway` (`{"connected": bool}` on change).
Client -> server: `heartbeat` (as above), `estop` `{asserted}` (same rules as REST; the ack carries the verdict). Anything else is ignored.

## 5. Not in this package / OPEN

* **RTK profile management** (host/mountpoint/user/password): the independent `dyx3-rtk` worker stores runtime profiles in `/var/lib/dyx3/rtk/config.json` (`dyx3:dyx3 0600`). The backend changes them through the local control socket and never returns passwords. `/etc/dyx3/ntrip.env` stays a read-only, first-boot seed (`root:dyx3 0640`). See `dyx3_rtk.md` for the REST and control contracts.
* **Settings and storage beyond missions/runs**, retention/pruning of artifacts and runs: not designed.
* **Not ported from the prototype** (decisions needed): the arbiter, joystick/manual-drive gateway and the prototype's emergency-stop plumbing. Manual driving is a motion source and needs its own contract; none is implemented.
* BLE, report generation (PDF), multi-tablet arbitration.

## 6. Acceptance

Off-target: token store (hashing, roles, fail-closed), gateway client against a fake socket server (framing, reconnect, timeout, pending requests failed on disconnect, telemetry cache), the REST surface with a fake gateway
(auth matrix, error mapping, estop role rule, delivered:false), upload (size, extension, engine error, idempotent store, summary), the heartbeat relay timing, the Socket.IO hub with a fake emitter.
The Python client was also run once against the REAL C++ gateway node in the Humble container (`tools/gateway_smoke.py`): framing, replies, `service_unavailable` for every service command, `invalid_command` for an unknown one, telemetry push, all as specified.
**Not provable off-target:** the gateway socket's permissions under systemd, the real tablet, WiFi behaviour, uvicorn under systemd.
