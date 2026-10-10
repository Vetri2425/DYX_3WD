# Rover contract — app-planned mission ingest v2

`POST /api/missions/plan` accepts the operator app's trajectory under an operator bearer token. The app owns geometry,
order, run split, extensions, collinearity, must-hit points and the geodetic anchor. The backend validates, applies only
the two lossless normalisations below, and writes a deterministic `DYX3PATH 1` artifact with `engine app_v1`; it never
has no path engine and never plans or re-plans the trajectory: this is the only way a trajectory enters the backend
(owner decision 2026-10-10). Success is HTTP 201 with a `mission` summary plus a `normalisation` report. Implementation: `backend/src/dyx3_backend/mission/app_plan.py`;
the REST surface around it: `backend.md` section 1b.

## Payload

```json
{"client": "Three_Wheel_v2", "client_version": "2.0.0", "name": "Pitch_A",
 "frame": "local_ned", "anchor": {"lat": 48.137154, "lon": 11.576124, "alt": 519.5},
 "runs": [{"type": "travel", "points": [[0.0, 0.0, 2], [12.0, 0.0, 2]]},
          {"type": "mark",   "points": [[12.007, 0.0, 3], [14.0, 0.0, 1], [16.0, 0.0, 3]]}]}
```

Required: `client` and `client_version` (nonempty strings), `frame`, and nonempty `runs`. Optional: `name` (up to 128
characters, default `app_planned_mission`), `anchor`, and `origin_ne_m` (`[north, east]`, default `[0, 0]`; see below).
A run has `type: "mark" | "travel"` and at least two ordered `[north_m, east_m, flags]` points. Coordinates and origin are
finite metres within ±10,000 m. Flags are integers 0..3: bit 0 is spray intent; bit 1 is must-hit. At most 50,000
submitted points are accepted.

## Frame and anchor (safety-critical)

Exactly one rule per frame:

| `frame` | `anchor` | Meaning of a point `(n, e)` | `origin_ne_m` |
|---|---|---|---|
| `local_ned` | **required** | metres north/east of the anchor: the anchor IS the origin | absent or exactly `[0, 0]`, else `ORIGIN_WITH_ANCHOR` |
| `ekf_local_ned` | **forbidden** | already in the rover's EKF local frame (bench and debug) | recorded as given |

- `local_ned` without an anchor (absent or `null`) -> `ANCHOR_REQUIRED`. Points relative to an app GPS origin are never
  driven as if they were EKF-local.
- `ekf_local_ned` with an anchor -> `INVALID_FRAME`. Any other frame -> `INVALID_FRAME`.
- `anchor` = `{lat, lon, alt?}`: the WGS84 position of the trajectory's local origin, in degrees; finite numbers (not
  booleans or strings), `|lat| <= 90`, `|lon| <= 180`; `alt` optional (ellipsoid height in m, finite, `|alt| <= 10,000`,
  DERIVED). A non-object, a missing `lat`/`lon` or an unknown key -> `INVALID_ANCHOR`.

## Normalisation (lossless, reported)

1. **Densify:** a within-run step longer than 5 m is split into `ceil(length / 5)` equal collinear sub-steps. The inserted
   points lie on the submitted segment, carry the run's spray bit and never the must-hit bit (they are not app vertices).
   The recorded mark and transit lengths are the drawn lengths. At most **200,000 points are stored** after densify
   (DERIVED) -> otherwise `POINTS_LIMIT_EXCEEDED`.
2. **Boundary snap:** a run whose first point is within **10 mm** (Euclidean) of the previous run's last point is moved
   exactly onto it. Over 10 mm -> `runs_not_contiguous`; a gap needs an explicit app-supplied travel run.

The 201 response reports both: `"normalisation": {"densified_steps": <steps split>, "max_boundary_snap_m": <largest snap, m>}`
(`0` and `0.0` when the trajectory was stored exactly as submitted).

## Run encoding rules

RPP constructs `raw_flags` from `z & 1` and `split_runs_by_flag` (see
`ros2_ws/src/dyx3_rpp/src/path_conditioner.cpp:179,477`). Its run boundary is a **spray-state
change**, not a must-hit point. The endpoint enforces these additional rules:

1. **R1 `mixed_spray_in_run`**: every mark point has bit 0 set; every travel point has it clear.
2. **R2 `adjacent_runs_same_type`**: successive runs alternate. Either type may begin or end the mission.
3. **R3 `runs_not_contiguous`**: each next run's first point is within 10 mm of the prior run's last point, and is snapped
   onto it (Normalisation 2).
4. **R4 encoding**: the first run is written in full. Each later run omits its first point. The
   stored shared point uses the prior run's exact coordinates and spray bit; its must-hit bit is
   the OR of both submitted copies. RPP prepends that stored point to the next run, assigning
   the next run's spray state to its reconstructed boundary point.

Apart from the two normalisations, the backend does no reordering, corner treatment, or transit insertion. RPP's existing
conditioner may subsequently split at corners or condition geometry during mission install; this endpoint does not change
that code or its settings.

## Errors

Error bodies are `{"ok":false,"code":"...","reason":"..."}`. Invalid JSON, missing required
fields, or malformed structure use HTTP 400 `INVALID_PAYLOAD`. Every semantic rejection below
uses HTTP 422:

| Code | Rejection |
|---|---|
| `INVALID_FRAME` | Frame other than `local_ned` / `ekf_local_ned`, or `ekf_local_ned` with an anchor |
| `ANCHOR_REQUIRED` | `local_ned` without an anchor |
| `INVALID_ANCHOR` | Anchor not `{lat, lon, alt?}` with finite values in range |
| `ORIGIN_WITH_ANCHOR` | `origin_ne_m` other than absent or `[0, 0]` next to an anchor |
| `INVALID_RUN_TYPE` | Run type other than mark/travel |
| `INVALID_FLAGS` | Noninteger flags or outside 0..3 |
| `NON_FINITE_COORDINATE` | Coordinate or origin is not finite numeric data |
| `OUT_OF_BOUNDS` | Coordinate or origin outside ±10,000 m |
| `EMPTY_MISSION` | No runs |
| `RUN_TOO_SHORT` | Fewer than two points in a run |
| `POINTS_LIMIT_EXCEEDED` | More than 50,000 submitted points, or more than 200,000 stored points after densify |
| `mixed_spray_in_run` | R1 |
| `adjacent_runs_same_type` | R2 |
| `runs_not_contiguous` | R3: a run boundary gap over 10 mm |

## Artifact

The artifact header line stays `frame local_ned` (the only frame the C++ reader accepts). The canonical JSON metadata
records `frame` and `anchor` (`{"alt": float|null, "lat": float, "lon": float}`, or `null` for `ekf_local_ned`), the
normalisation (`densified_steps`, `max_boundary_snap_m`), `origin_ne_m` (`[0.0, 0.0]` with an anchor), mark and transit
lengths, waypoint count, and source `{type:"app_planned",client,client_version,name,num_runs}`. The hash is over the
complete bytes, so the same accepted payload yields the same SHA-256, and a different anchor is a different mission.
Existing mission list, read and start routes use this artifact without special handling; the path preview also returns
its `frame` and `anchor`.
