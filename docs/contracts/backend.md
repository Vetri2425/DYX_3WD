# backend — contract

**Status:** draft for review, written before the implementation. **Spec:** V1 §7.10, §7.2, §4.3.1; CLAUDE.md §3 and §12. **Authority:** none over motion.
The backend (Python, FastAPI + Socket.IO) owns REST, Socket.IO, auth, mission upload/report, telemetry delivery and path/CAD ingestion. It **never imports `rclpy`**,
sends no PX4 command and has no safety logic: every command goes to `dyx3_system_gateway` over its Unix socket (`docs/contracts/dyx3_system_gateway.md`), and a backend
or tablet E-stop is a **request** to `dyx3_motion_guard`. If the gateway is not connected, a command is **not delivered and the caller is told so** (never queued, never retried later).

## 1. HTTP API (all under `/api`, JSON; auth = `Authorization: Bearer <token>` except `/api/ping`)

| Route | Role | Notes |
|---|---|---|
| `GET /ping` | none | liveness of the backend process only |
| `GET /health` | viewer | backend, gateway connection, age of the last telemetry, tablet heartbeat |
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

## 2. Auth (DERIVED — NOT FROM V1 SPEC; OPEN)

Static bearer tokens with a role, stored **hashed** (sha256) in `auth_file` (default `/var/lib/dyx3/state/auth.json`, `{"version":1,"tokens":[{"name","role","sha256"}]}`); comparison is constant time.
Roles: `viewer` < `operator`. **No token file or an empty one means every protected route is denied** (fail closed). `python -m dyx3_backend.auth.tokens create --name N --role R` prints a new token once and
adds its hash. Socket.IO connects with `auth={"token": ...}`. A user/password model, pairing, token rotation and expiry are **not designed**: OPEN (human).

## 3. Tablet heartbeat -> operator link (R13)

The tablet sends `POST /api/heartbeat` or the Socket.IO event `heartbeat`. The backend relays a gateway `heartbeat` every `heartbeat_relay_s` **only while the tablet heartbeat is younger than `tablet_heartbeat_timeout_s`**,
and the gateway turns that into `OperatorLinkStatus` (its own `operator_link_timeout_s`). The last authenticated Socket.IO connection closing clears the tablet heartbeat at once. A tablet WiFi dropout
therefore stops the relay and `dyx3_motion_guard` stops the rover. `heartbeat_relay_s` 0.5 and `tablet_heartbeat_timeout_s` 1.5 are **DERIVED — NOT FROM V1 SPEC**; the worst-case delay is
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
