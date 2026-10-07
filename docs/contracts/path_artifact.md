# Contract — path artifact (`DYX3PATH 1`)

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
`north_m`, `east_m`: local NED metres relative to the mission origin recorded in `meta.origin_ne_m`. The artifact holds the **planned polyline only**;
`dyx3_rpp::path_conditioner` splits/conditions it at install (one place for geometry decisions).

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
