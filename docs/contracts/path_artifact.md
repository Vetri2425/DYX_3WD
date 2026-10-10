# Contract — path artifacts (`DYX3PATH 1`, `DYX3COND 1`)

Implementation: `backend/src/dyx3_backend/mission/path_artifact.py`. Tests: `backend/tests/test_path_artifact.py`.
Spec §7.2: *"a versioned, content-hashed path artifact carried into the run manifest."*

## Identity
`artifact id = sha256(file bytes)` (lowercase hex). The file is stored as `<id>.dyx3path` under `/var/lib/dyx3/missions/`.
`StartMission.path_artifact_sha256`, `ExecuteMission` goals, `PointResult`/`MissionState` provenance and the run manifest all carry this id.
A reader MUST hash the bytes it parsed and refuse a mismatch (`decode(data, expected_sha256=…)`). Content-addressing makes `store()` idempotent and
makes "same mission" a byte-for-byte statement.

## Format (ASCII, LF, final newline)
```
DYX3PATH 1
frame local_ned
engine <engine_id>              # first 16 hex of sha256(path_engine/ORIGIN.sha256): traces to PX4_DXP fc6436b
meta <canonical JSON>           # sorted keys, separators (",",":"), ensure_ascii, no NaN/Infinity
points <N>
<north_m> <east_m> <flags>      # N lines, Python repr(float), flags 0..3
end <N>
```
`flags`: bit0 = spray ON, bit1 = **must-hit** (source CAD/survey vertex, never simplified away). Bit-test, never `> 0.5` (spray-OFF must-hit = `2`).
`north_m`, `east_m`: local NED metres relative to the mission origin recorded in `meta.origin_ne_m`. `DYX3PATH` holds the planned polyline.
For an app-planned mission (`engine app_v1`, `backend.md` section 1b) the header line stays `frame local_ned` and the meta says what
the coordinates are relative to: `meta.frame` = `"local_ned"` (north/east metres from the WGS84 `meta.anchor`
`{"alt": float|null, "lat": float, "lon": float}`) or `"ekf_local_ned"` (already the rover's EKF local frame, `meta.anchor` = `null`).
It also records the admission normalisation: `meta.densified_steps` (steps over 5 m split into collinear sub-steps) and
`meta.max_boundary_snap_m` (largest run-boundary snap, at most 0.010 m). An artifact without `meta.frame` (a DXF upload)
carries no anchor.
At mission install, RPP is the sole owner of `dyx3_rpp::path_conditioner` and writes the resulting immutable `DYX3COND 1` artifact.

## Conditioned execution artifact (`DYX3COND 1`)

Stored as `<sha256>.dyx3cond` in the same artifact directory. The SHA256 is over the complete deterministic file bytes. The file records the
original `DYX3PATH` source SHA256, every geometry-affecting conditioner parameter, and ordered runs. Each run stores its RPP profile and each
conditioned point's NED coordinates, spray flag, and must-hit flag. `RppStatus.conditioned_execution_sha256` names the artifact for its mission.
Spray verifies both the file hash and its `source` against `MissionState.path_artifact_sha256`, then builds the projection model directly from the
conditioned coordinates and flags. Run IDs prevent projection from inventing a segment between runs. Spray never conditions the raw path and has no
raw-geometry fallback. Missing, malformed, stale, wrong-source, or hash-mismatched files clear the path model and keep autonomous spray OFF.

Determinism: canonical decimal values use `max_digits10`, stable field and run ordering, no timestamps, and a final LF. Changing conditioner config
or generated geometry changes the content hash. RPP drives the in-memory runs used to produce the artifact; the artifact stores those same values.

## Determinism rules (tested)
No timestamps, no host paths, no wall-clock timings (`planning_time_s`, `filepath` are stripped), canonical JSON, canonical float text (a reader rejects
a non-canonical form so one mission can only have one byte representation), planning the same file twice yields the same bytes.

## Versioning
`FORMAT_VERSION = 1`. A reader refuses any other version. A change of fields or semantics bumps the version and documents a migration; the artifact id of an old
mission never changes.

## Strict decode rejects
bad magic/version/frame, non-canonical meta, count mismatch, missing/wrong `end` marker, trailing lines, CRLF, no final newline, non-ASCII, non-finite or
non-canonical numbers, flags outside 0..3.

## C++ reader (to be written in `dyx3_mission`, Phase 3)
`strtod` on the two coordinate fields round-trips `repr(float)` exactly; no JSON dependency (the `meta` line is opaque to C++ but covered by the hash).

## What it is not
Not a trajectory (no speeds/time), not conditioned runs, not the `/rpp/conditioned_path`. Not the old `staged_mission.json`.
