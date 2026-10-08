# Rover contract — app-planned mission ingest v1.1

`POST /api/missions/plan` accepts the operator app's v1 payload (`Three_Wheel_v2` contract
`app_planned_mission_v1.md`) under an operator bearer token. The app owns geometry, order,
extensions, collinearity, and must-hit points. The backend validates and writes a deterministic
`DYX3PATH 1` artifact with `engine app_v1`; it never calls `PathEngine`. Success is HTTP 201 with
the same `{"ok":true,"mission":...}` summary as `POST /api/missions`.

## Payload

Required: `client` and `client_version` (nonempty strings), `frame` (`local_ned`), and nonempty
`runs`. Optional: `name` (up to 128 characters, default `app_planned_mission`) and `origin_ne_m`
(`[north,east]`, default `[0,0]`). A run has `type: "mark" | "travel"` and at least two ordered
`[north_m,east_m,flags]` points. Coordinates and origin are finite local NED metres within
±10,000 m. Flags are integers 0..3: bit 0 is spray intent; bit 1 is must-hit. Each step within a
run is at most 5 m; at most 50,000 submitted points are accepted.

## Run encoding rules

RPP constructs `raw_flags` from `z & 1` and `split_runs_by_flag` (see
`ros2_ws/src/dyx3_rpp/src/path_conditioner.cpp:179,477`). Its run boundary is a **spray-state
change**, not a must-hit point. The endpoint enforces these additional rules:

1. **R1 `mixed_spray_in_run`**: every mark point has bit 0 set; every travel point has it clear.
2. **R2 `adjacent_runs_same_type`**: successive runs alternate. Either type may begin or end the mission.
3. **R3 `runs_not_contiguous`**: each next run's first north/east is within 0.001 m per axis of
   the prior run's last point. A gap needs an explicit app-supplied travel run.
4. **R4 encoding**: the first run is written in full. Each later run omits its first point. The
   stored shared point uses the prior run's exact coordinates and spray bit; its must-hit bit is
   the OR of both submitted copies. RPP prepends that stored point to the next run, assigning
   the next run's spray state to its reconstructed boundary point. A submitted boundary within
   0.001 m but not bit-exact is represented by the prior endpoint's coordinates.

The backend does no reordering, densification, corner treatment, or transit insertion. RPP's
existing conditioner may subsequently split at corners or condition geometry during mission
install; this endpoint does not change that code or its settings.

## Errors

Error bodies are `{"ok":false,"code":"...","reason":"..."}`. Invalid JSON, missing required
fields, or malformed structure use HTTP 400 `INVALID_PAYLOAD`. Every semantic rejection below
uses HTTP 422:

| Code | Rejection |
|---|---|
| `INVALID_FRAME` | Frame other than `local_ned` |
| `INVALID_RUN_TYPE` | Run type other than mark/travel |
| `INVALID_FLAGS` | Noninteger flags or outside 0..3 |
| `NON_FINITE_COORDINATE` | Coordinate or origin is not finite numeric data |
| `OUT_OF_BOUNDS` | Coordinate or origin outside ±10,000 m |
| `EMPTY_MISSION` | No runs |
| `RUN_TOO_SHORT` | Fewer than two points in a run |
| `POINTS_LIMIT_EXCEEDED` | More than 50,000 submitted points |
| `STEP_TOO_LARGE` | Within-run step over 5 m |
| `mixed_spray_in_run` | R1 |
| `adjacent_runs_same_type` | R2 |
| `runs_not_contiguous` | R3 |

The artifact stores canonical JSON metadata with origin, mark and transit lengths, waypoint
count, and source `{type:"app_planned",client,client_version,name,num_runs}`. The hash is over
the complete bytes, so the same accepted payload yields the same SHA-256. Existing mission
list, read, path, and start routes use this artifact without special handling.
