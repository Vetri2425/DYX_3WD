# backend — contract

**Status:** draft for review, written before the implementation. **Spec:** V1 §7.10, §7.2, §4.3.1; CLAUDE.md §3 and §12. **Authority:** none over motion.
The backend (Python, FastAPI + Socket.IO) owns REST, Socket.IO, auth, admission and storage of tablet-planned missions, telemetry delivery and the status event stream. It plans no geometry: the tablet app is the single trajectory author (owner decision 2026-10-10), and the only way a trajectory enters is `POST /missions/plan` (section 1b). It **never imports `rclpy`**,
sends no PX4 command and has no safety logic: every command goes to `dyx3_system_gateway` over its Unix socket (`docs/contracts/dyx3_system_gateway.md`), and a backend
or tablet E-stop is a **request** to `dyx3_motion_guard`. If the gateway is not connected, a command is **not delivered and the caller is told so** (never queued, never retried later).

## 1. HTTP API (all under `/api`, JSON; auth = `Authorization: Bearer <token>` except `/api/ping`)

| Route | Role | Notes |
|---|---|---|
| `GET /ping` | none | liveness, plus `rover_id` and `rover_name` (identity, no secret; see 1a) |
| `GET /health` | viewer | backend, gateway connection, age of the last telemetry, tablet heartbeat, `relay_running` (the relay task is alive), `operator_alive` (relay running, tablet fresh, and a relayed heartbeat acknowledged by the gateway within `tablet_heartbeat_timeout_s`; the gateway's own timer stays authoritative), and a diagnostic `mission` block (section 1c) |
| `POST /missions/plan` (JSON) | operator | app-planned mission (section 1b) -> `DYX3PATH 1` artifact stored by sha256 -> **201** `{ok, mission, normalisation}` |
| `GET /missions`, `GET /missions/{sha}`, `GET /missions/{sha}/path` | viewer | stored artifacts, summary, points; `/path` also returns `frame` and `anchor` (section 1b) |
| `POST /missions/{sha}/start` `{request_id?}` | operator | gateway `start_mission`; **202** accepted (section 1c) |
| `POST /mission/abort` `{reason}` / `pause` / `resume` / `skip_point` | operator | gateway |
| `POST /estop` `{asserted}` | **assert: any authenticated role; clear: operator** | gateway `estop` with `source = "tablet"` |
| `POST /vehicle/arm` `{arm}`, `POST /vehicle/offboard` `{enable}`, `POST /spray/manual` `{on}` | operator | gateway |
| `POST /heartbeat` | **operator** | the tablet heartbeat (section 3) |
| `GET /telemetry` | viewer | the latest gateway snapshot, with its age |
| `GET /runs`, `GET /runs/{id}` | viewer | the recorder's `summary.json` / `manifest.json` (read-only) |

Error mapping of a gateway verdict: `ok` -> 200 (`start`: 202, section 1c); downstream `rejected` -> 409 (body carries the downstream `reason_code`, verbatim); `invalid_command` -> 400; `service_unavailable` -> 503;
`timeout` -> 504; gateway not connected -> 503 with `"delivered": false`. Every error body: `{"ok":false,"code":...,"reason":...,"delivered":bool,"data":...}`.
**Gateway reply wait.** `request_timeout_s` (`DYX3_REQUEST_TIMEOUT_S`, default **6.0 s**) is how long the backend waits for a gateway reply. It must exceed the slowest per-command timeout of the gateway
(`offboard_timeout_s` 5.0 s, `dyx3_system_gateway.md` section 2), otherwise the tablet is told `timeout` (504) for an OFFBOARD that the gateway then confirms. Settings refuses a value below 6.0 s (or non-finite)
at start-up, including `DYX3_REQUEST_TIMEOUT_S`. The same value is the RTK control socket's request wait.

A plan is rejected (413/400/422) for: a body over `upload_max_bytes`, invalid JSON, a violated rule of section 1b, or an artifact the reader would refuse.

**Removed (2026-10-10).** The backend has no DXF/CSV/waypoints file-upload planner and no DXF parser: `POST /missions` (multipart upload) and `POST /path/parse-dxf` do not exist (a POST to either is 405 / 404), and neither does the backend's path engine (`path_engine/`, `ezdxf`, `geographiclib`, the `path-engine` install extra). The tablet parses CAD files itself.

**Admission (before any body byte is read).** Every HTTP request under `/api` except `GET /api/ping` first passes an ASGI layer
(`api/admission.py`; it wraps the FastAPI app only, Socket.IO is not behind it):
- a missing or unknown bearer token -> **401** (`{"detail": ...}`, `WWW-Authenticate: Bearer`) without reading the body, so a malformed body
  from an unauthenticated client is 401, not 422. The role check (403) stays in the route;
- a body over the route's cap -> **413** `{"ok":false,"code":"too_large",...}`, from `Content-Length` before reading, or as soon as the
  streamed byte count passes the cap (chunked bodies). Caps: `POST /missions/plan` = `upload_max_bytes` (`DYX3_UPLOAD_MAX_BYTES`, default
  20 MiB, DERIVED, exact); every other route = `json_body_max_bytes` (`DYX3_JSON_BODY_MAX_BYTES`, default 64 KiB, DERIVED);
- a non-numeric `Content-Length` -> 400;
- on the JSON routes (not `POST /missions/plan`, which is checked in the planning process), a body nested deeper than 32 levels (DERIVED) -> **400** `{"ok":false,"code":"bad_request",...}`,
  checked on the stream before FastAPI parses it (deep nesting otherwise ends in a parse error or a 500, depending on the Python version).

Artifact reads for `GET /missions/{sha}`, `GET /missions/{sha}/path` (including rendering its JSON) and `POST /missions/{sha}/start`
run in a worker thread, never on the event loop (BE-005).

**Planning budget (BE-004).** App-plan parsing and compiling (`POST /missions/plan`) runs in a separate, freshly spawned process
(`mission/planner.py`), never on the event loop and never in a thread that shares the GIL with the heartbeat relay:
- **one job at a time**: a second plan request while one runs -> **409** `{"ok":false,"code":"busy",...}` (retry later);
- wall-clock budget `plan_timeout_s` (`DYX3_PLAN_TIMEOUT_S`, default 60 s, DERIVED, process start-up included): over budget the
  process is terminated (then killed) -> **422** `plan_budget_exceeded`;
- point budgets (section 1b): at most 50 000 submitted points and 200 000 stored points after densify -> **422** `POINTS_LIMIT_EXCEEDED`;
- a JSON body nested too deeply -> **400** `INVALID_PAYLOAD`; a planning process that dies without a result -> **500** `planner_crashed`.

## 1b. App-planned missions (`POST /missions/plan`, mission contract v2)

The tablet is the only trajectory author: it parses the file, splits it into runs and sends them. The backend **never plans or re-plans** geometry; it
validates, applies only the two lossless normalisations below, and stores the result. The endpoint's full rules are in
`app_planned_mission.md` (v2); this section summarises them with the REST surface.

**Payload.**
```json
{"client": "Three_Wheel_v2", "client_version": "2.0.0", "name": "Pitch_A",
 "frame": "local_ned", "anchor": {"lat": 48.137154, "lon": 11.576124, "alt": 519.5},
 "origin_ne_m": [0.0, 0.0],
 "runs": [{"type": "travel", "points": [[0.0, 0.0, 2], [12.0, 0.0, 2]]},
          {"type": "mark",   "points": [[12.007, 0.0, 3], [14.0, 0.0, 1], [16.0, 0.0, 3]]}]}
```
- `client`, `client_version` (nonempty strings), `frame`, `runs` are required; `name` (<= 128 characters, default
  `app_planned_mission`), `anchor` and `origin_ne_m` (`[north, east]`, default `[0, 0]`; see the frame rules) are optional.
- A run is `{"type": "mark"|"travel", "points": [[north_m, east_m, flags], ...]}`, at least two points; flags 0..3 (bit 0 spray
  intent, bit 1 must-hit). Coordinates are finite and within +-10 000 m; at most 50 000 submitted points.
- Rules R1-R4 of `app_planned_mission.md` keep their v1 meaning, with R3's tolerance now the 10 mm snap below (spray bit matches the run type, runs alternate, runs are contiguous, the
  shared boundary point keeps the previous run's coordinates and spray bit and ORs both must-hit bits).

**Frame and anchor (safety-critical).**
- `frame: "local_ned"` = north/east metres relative to the geodetic `anchor`, the WGS84 position of the trajectory's local
  origin. **An anchor is required**: without one -> 422 `ANCHOR_REQUIRED`. Points relative to an app GPS origin are never
  driven as if they were EKF-local.
- **The anchor IS the origin:** point `(n, e)` is metres north/east of the anchor. With an anchor, `origin_ne_m` must be
  absent or exactly `[0, 0]` (stored as `[0.0, 0.0]`); anything else -> 422 `ORIGIN_WITH_ANCHOR`. Without an anchor
  (`ekf_local_ned`), `origin_ne_m` is recorded as given.
- `frame: "ekf_local_ned"` = the points are already in the rover's EKF local frame (bench and debug). It takes no anchor
  (an anchor with it -> 422 `INVALID_FRAME`).
- Any other frame -> 422 `INVALID_FRAME`.
- `anchor` = `{lat, lon, alt?}`: WGS84 degrees, finite numbers (not booleans or strings), `|lat| <= 90`, `|lon| <= 180`;
  `alt` optional (ellipsoid height in m, finite, `|alt| <= 10 000`, DERIVED). Unknown keys, a missing `lat`/`lon` or a
  non-object -> 422 `INVALID_ANCHOR`. `anchor: null` counts as no anchor.

**Normalisation (lossless, reported).**
1. **Densify:** a step longer than 5 m (`MAX_STEP_M`) is split into `ceil(length / 5)` equal collinear sub-steps. The
   inserted points lie on the submitted segment, carry the run's spray bit and never the must-hit bit (they are not app
   vertices). The mark/transit lengths are the drawn lengths (unchanged). This replaces v1's `STEP_TOO_LARGE` refusal.
   At most 200 000 points are stored after densify (DERIVED) -> otherwise 422 `POINTS_LIMIT_EXCEEDED`.
2. **Boundary snap:** a run whose first point is within **10 mm** (Euclidean) of the previous run's end is moved exactly onto
   it (v1: 1 mm per axis). Over 10 mm -> 422 `runs_not_contiguous`.

**Artifact.** `DYX3PATH 1` (`path_artifact.md`), `engine app_v1`, header line `frame local_ned` unchanged (the C++ reader
accepts only that). The canonical meta records:

| meta key | value |
|---|---|
| `frame` | `"local_ned"` or `"ekf_local_ned"`, as submitted |
| `anchor` | `{"alt": float\|null, "lat": float, "lon": float}` for `local_ned`; `null` for `ekf_local_ned` |
| `densified_steps` | number of submitted steps that were split |
| `max_boundary_snap_m` | largest boundary snap in m (`0.0` when every boundary was bit-exact) |
| `origin_ne_m` | `[0.0, 0.0]` with an anchor; as submitted for `ekf_local_ned` |
| `total_mark_length_m`, `total_transit_length_m`, `num_waypoints` | drawn lengths in m (unchanged by densify); stored point count |
| `source` | `{type: "app_planned", client, client_version, name, num_runs}` |

The same payload always yields the same bytes and sha256; a different anchor is a different mission.

**Response** `201`:
```json
{"ok": true,
 "mission": {"sha256": "45f5de73589602a4e0edaa26f0ac43a41e4215ed4afe2f027db42dccf3db49db", "engine_id": "app_v1",
             "num_points": 6, "num_spray_points": 2,
             "mark_length_m": 4.0, "transit_length_m": 12.0, "bbox_ne_m": [0.0, 0.0, 16.0, 0.0],
             "source": {"type": "app_planned", "client": "Three_Wheel_v2", "client_version": "2.0.0",
                        "name": "Pitch_A", "num_runs": 2}},
 "normalisation": {"densified_steps": 1, "max_boundary_snap_m": 0.006999999999999673}}
```
For that payload the stored points (and the preview) are `[[0.0,0.0,2],[4.0,0.0,0],[8.0,0.0,0],[12.0,0.0,2],[14.0,0.0,1],[16.0,0.0,3]]`:
the 12 m travel leg became three 4 m sub-steps, the mark run starts exactly at `[12.0, 0.0]` with the travel run's spray bit
and the OR of both must-hit bits (R4).
Errors: `{"ok": false, "code", "reason"}`; 400 `INVALID_PAYLOAD` (invalid JSON, missing or malformed structure), 422 for
`ANCHOR_REQUIRED`, `INVALID_ANCHOR`, `ORIGIN_WITH_ANCHOR`, `INVALID_FRAME`, `INVALID_RUN_TYPE`, `INVALID_FLAGS`, `NON_FINITE_COORDINATE`,
`OUT_OF_BOUNDS`, `EMPTY_MISSION`, `RUN_TOO_SHORT`, `POINTS_LIMIT_EXCEEDED`, `mixed_spray_in_run`, `adjacent_runs_same_type`,
`runs_not_contiguous`; plus the admission and planning-budget answers of section 1.

**Preview** `GET /missions/{sha}/path` returns the stored geometry, bit-exact what is driven before placement:
`{"sha256", "frame", "anchor", "points": [[north_m, east_m, flags], ...]}`, e.g.
`{"sha256": "45f5…", "frame": "local_ned", "anchor": {"alt": 519.5, "lat": 48.137154, "lon": 11.576124}, "points": [...]}`. `frame` and `anchor` come from the meta; an
artifact without them (one stored before frame metadata existed) reads `"frame": "local_ned", "anchor": null`.

## 1c. Mission start and health (mission contract v2)

**Start** `POST /missions/{sha}/start`, optional JSON body `{"request_id": "<id>"}` and/or header `Idempotency-Key: <id>`.
- `<id>`: 1-64 characters of `A-Z a-z 0-9 . _ : -`; if both are given they must be equal. Otherwise 422
  `invalid_request_id` (an unknown body key is FastAPI's 422). Nothing reaches the gateway.
- The artifact must be readable here first (400 `bad_id`, 404 `not_found`).
- Gateway command: `{"cmd": "start_mission", "args": {"path_artifact_sha256": "<sha>", "request_id": "<id>"}}`;
  `request_id` is present only when the client sent one (the backend never invents one). The mission side owns
  idempotency: a duplicate id returns the same execution and starts nothing new. The backend keeps no state for it.
- Accepted -> **202**: an acknowledgement only; the lifecycle (loading, placing, arming, ...) arrives as Socket.IO events,
  never by polling.
  ```json
  {"ok": true, "accepted": true,
   "execution": {"mission_id": 7, "request_id": "tab-1:9f2c"},
   "data": {"accepted": true, "reason_code": 0, "mission_id": 7}}
  ```
  `execution.mission_id` is the gateway reply's `data.mission_id` (null if absent); `execution.request_id` is the id sent
  (null without one); `data` is the gateway reply's `data`, untouched.
- Errors, typed `{"ok": false, "code", "reason", "delivered", "data"}` with the gateway's `data` (including the mission's
  `reason_code`) untouched: `rejected` -> **409**; `invalid_command` -> 400; `service_unavailable` / `busy` -> 503;
  gateway not connected -> **503** `"delivered": false` (not delivered: nothing started); `timeout` or no reply in time ->
  **504** `"delivered": null` (unknown: re-read the mission state before retrying with the same `request_id`).

**Health** `GET /health` includes a diagnostic `mission` block copied from the latest gateway snapshot's `mission` source:
`{"lifecycle_state": str|null, "last_error": any|null, "age_s": float|null, "fresh": bool}`. A field the snapshot does not
carry is null; `fresh` is true only when the snapshot is fresh (`telemetry_stale_s`) and its mission source is `fresh`.
Diagnostic only: nothing depends on polling it.


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

The tablet sends `POST /api/heartbeat` or the Socket.IO event `heartbeat`; **both require the operator role** (a viewer cannot keep the
operator link alive: REST answers 403, the Socket.IO ack is `{"ok": false, "code": "forbidden"}`). The backend relays a gateway `heartbeat`
every `heartbeat_relay_s` **only while the tablet heartbeat is younger than `tablet_heartbeat_timeout_s`**, and the gateway turns that into
`OperatorLinkStatus` (its own `operator_link_timeout_s`). When the last **operator** Socket.IO session closes, the tablet heartbeat is cleared
at once (a viewer session closing changes nothing). A tablet WiFi dropout therefore stops the relay and `dyx3_motion_guard` stops the rover.
`heartbeat_relay_s` 0.5 and `tablet_heartbeat_timeout_s` 1.5 are **DERIVED — NOT FROM V1 SPEC**; the worst-case delay is
`tablet_heartbeat_timeout_s + gateway operator_link_timeout_s` (3.5 s with the defaults = 1.2 m at 0.35 m/s). **OPEN (human):** the numbers.

The relay task never ends on an error: an unexpected exception in a tick is logged and that tick relays nothing (fail to STOP), and the next
tick runs as usual.

**Stall guard (XR-BE-002).** A tablet heartbeat is stamped when the event loop processes it, so after a loop stall the queued heartbeats would
look fresh. The relay measures the gap between its own ticks; when it exceeds `heartbeat_relay_s` + 0.3 s (DERIVED), heartbeats stamped since
the previous tick are discarded (the stamp held at that tick is kept) and heartbeats stamped within the next `heartbeat_relay_s` are ignored
(the REST/Socket.IO answer is unchanged). A live tablet re-proves itself with its next heartbeat; a slow gateway reply trips the same guard,
in the fail-to-STOP direction.

## 4. Socket.IO

`/socket.io`, connect requires a valid token (`auth={"token": ...}`; a bad one is refused with `unauthorized`). `cors_allowed_origins` is empty (same-origin / native clients only).
Transport timing: `sio_ping_interval_s` 5.0 and `sio_ping_timeout_s` 5.0 (DERIVED; session cleanup only, the tablet heartbeat owns the safety timeout).

**Server -> client: two events.**

* `telemetry` `{"snapshot": {...}|null, "age_s": float|null}` on every gateway snapshot push (5 Hz, `telemetry_hz`), to every session. The periodic picture; it carries no
  ordering and no replay. (`GET /telemetry` returns `{"connected", "age_s", "snapshot"}`.)
* `rover_event`: **the single status event.** Every status change reaches the tablet as one `rover_event` the moment it is known, so the tablet never polls. The old `gateway`
  event (`{"connected": bool}`) is gone: its information is the `gateway_link` kind below.

```json
{"kind": "mission_state", "seq": 118, "gateway_seq": 42, "t_mono_s": 18234.512039871, "t_wall_ms": 1791624580734,
 "coalesced": 0, "replay": false,
 "data": {"state": 3, "mission_id": 7, "run_index": 0, "point_index": 0, "reason_code": 0,
          "path_artifact_sha256": "<64 hex>", "fresh": true, "stamp_s": 1791624580.733912}}
```

| field | meaning |
|---|---|
| `kind` | `mission_state`, `operator_link`, `fcu_link`, `estop` (the gateway's status events, `dyx3_system_gateway.md` section 1.3) or `gateway_link` (the backend's own link to the gateway) |
| `seq` | **the backend's** sequence number: starts at 1 per backend process, +1 per `rover_event` recorded, across all kinds. It is the ordering the tablet uses. Unlike `gateway_seq` it does not restart when the gateway restarts, and a replay keeps the `seq` of the event it repeats |
| `gateway_seq` | the gateway's `seq` for that event (restarts at 1 when the gateway restarts); `null` for `gateway_link` |
| `t_mono_s` | gateway steady clock (seconds) when the carried value was received; `null` for `gateway_link`. Comparable only with other `t_mono_s` of the same gateway process |
| `t_wall_ms` | Unix ms: the gateway's wall clock when the event was pushed; for `gateway_link`, the backend's wall clock when the link state changed |
| `coalesced` | transitions the gateway folded into this event (0 = none lost); always 0 for `gateway_link` |
| `replay` | `true` on the copy sent to a session right after it connects, and on a gateway event that is itself the gateway's connect replay; otherwise `false` |
| `data` | the full current value of that kind, never a delta |

| `kind` | `data` |
|---|---|
| `mission_state` | the snapshot `mission` fields (`state`, `mission_id`, `run_index`, `point_index`, `reason_code`, `path_artifact_sha256`, ...), `fresh`, `stamp_s` |
| `operator_link` | `alive`, `age_s`, `cause`: `heartbeat` \| `timeout` \| `connection_closed` \| `never` |
| `fcu_link` | `fresh`, `session_alive`, `handshake_ok`, `fault`, `session_resets` |
| `estop` | `fresh`, `asserted`, `source` |
| `gateway_link` | `{"connected": bool}`: the backend's socket to the gateway. `false` until the first connection and while the gateway is down |

Rules for the tablet:

* **Per kind, the event with the highest `seq` is the current state**; an event whose `seq` is not above the last one of its kind is a duplicate (a push can race the connect replay).
* **Stale is a transition.** A gateway-sourced kind whose source went silent arrives once with `data` = `{"fresh": false}`: the state is unknown, never the last value. When `gateway_link`
  is `connected: false`, **every other kind is unknown** (the backend does not re-emit them as stale); the gateway's replay on reconnect re-establishes each kind, with new `seq`.
* **On connect** a session first receives, with `replay: true`, the latest `rover_event` of every kind, in `seq` order (it includes `gateway_link`, and `mission_state`/`operator_link`/`fcu_link`/`estop`
  for each kind the backend has seen since it started), so it has the current state at once and never polls.
* Gateway events are handed on in the order the gateway sent them, one at a time, from a bounded queue (256). A subscriber cannot block the gateway socket; if the queue overflows, the oldest
  event is dropped and counted (`events_dropped`), because a newer event of the same kind supersedes it.
* The backend's own `seq`, `replay` copies and `gateway_link` are the only things it adds; `data` is the gateway's, untouched.

**Client -> server:** `heartbeat` (operator only, as section 3), `estop` `{asserted}` (same rules as REST; the ack carries the verdict). Anything else is ignored.

## 5. Not in this package / OPEN

* **RTK profile management** (host/mountpoint/user/password): the independent `dyx3-rtk` worker stores runtime profiles in `/var/lib/dyx3/rtk/config.json` (`dyx3:dyx3 0600`). The backend changes them through the local control socket and never returns passwords. `/etc/dyx3/ntrip.env` stays a read-only, first-boot seed (`root:dyx3 0640`). See `dyx3_rtk.md` for the REST and control contracts.
* **Settings and storage beyond missions/runs**, retention/pruning of artifacts and runs: not designed.
* **Not ported from the prototype** (decisions needed): the arbiter, joystick/manual-drive gateway and the prototype's emergency-stop plumbing. Manual driving is a motion source and needs its own contract; none is implemented.
* BLE, report generation (PDF), multi-tablet arbitration.

## 6. Acceptance

Off-target: token store (hashing, roles, fail-closed), gateway client against a fake socket server (framing, reconnect, timeout, pending requests failed on disconnect, telemetry cache), the REST surface with a fake gateway
(auth matrix, error mapping, estop role rule, delivered:false, start 202 / request id / typed start errors, the health mission block),
app-plan admission (densify on the segment, boundary snap and its limit, frame and anchor rules, meta, size, idempotent store, summary; the removed upload and parse routes answer 404/405), the heartbeat relay timing, the Socket.IO hub with a fake emitter (`rover_event` ordering, replay, `gateway_link`).
The Python client was also run once against the REAL C++ gateway node in the Humble container (`tools/gateway_smoke.py`): framing, replies, `service_unavailable` for every service command, `invalid_command` for an unknown one, telemetry push, all as specified.
**Not provable off-target:** the gateway socket's permissions under systemd, the real tablet, WiFi behaviour, uvicorn under systemd.
