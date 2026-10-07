# Contract — path engine (carried unchanged in behaviour)

Location: `backend/src/dyx3_backend/path_engine/` (spec §7.2: stays Python, runs once per mission upload, never in the loop).
Provenance: byte-identical to `PX4_DXP` `build/demo-ready` @ `fc6436b` `path_engine/` except `from|import path_engine` → `dyx3_backend.path_engine`
(`ORIGIN.sha256`; enforced by `backend/tests/test_path_engine_provenance.py`). Logger names (`path_engine.*`) and all behaviour are unchanged.

## Evidence
* The prototype's 509 path-engine tests: **509 pass in the untouched checkout; 466 pass + 43 skipped in the port.** The 43 skips need
  `server/path_manager.py` (the old orchestrator, replaced in Phase 2); they are *not run* here, not passing.
* Dependencies actually imported: `ezdxf`, `geographiclib` (+ `hypothesis`, `pytest` for tests). `pyproject.toml` previously listed pyproj/numpy/shapely — corrected.
* Corpus available in Git (inputs): `square_2x2.dxf`, `soccer_pitch_fifa_edited.dxf`, `soccer_field_penalty_area.dxf`, `mission_straight_5m.waypoints`
  (`backend/tests/data/missions/`). The field DXF/CSV missions (e.g. `tes_cross_line*.dxf`, surveyed CSVs) are on the Mac — LOCAL ACTION.

## Behaviour that must not change (bug history → why)
* **Georef north-scale** (`e483d53`, field-proven 2.3255 m vs 2.3255 m WGS84, residual 0.000 mm): do NOT re-fix or revert; PX4's local frame uses a sphere (R = 6371000),
  WGS84 differs +0.5096 % N / −0.1290 % E (open A13: frame metres reported as ground metres) — `2dc7155` must not be reverted, the sphere makes the EKF frame self-consistent.
* **Vertex provenance** (A2/A9, `64c12ff`): `must_hit` survives every merge/dedupe; `_merge_chain` / `decompose_line_chain_to_edges` remap `vertex_indices` (4-LINE square keeps 4/4 corners).
* Sparse-arc G0 joints, the must-hit densify bug, the fitter discarding stakes ≤ 1.5 m (all fixed in the prototype; tests carried).
* Extensions: PRE(OFF) → MARK(ON) → AFT(OFF), aft run-out `max(aft_extension_m, 0.1)`; terminal boundary vertex (A17 is a *spray-node* defect, not the planner's).
* Solenoid latency compensation lives in `spray.py` (planner side) AND `nozzle_*` / `solenoid_*_delay_s` in the spray node — double-compensation risk; owner decision pending (Phase 9).

## Known open (carried, not fixed here)
A13 (frame metres vs ground metres; metric-geometry only), A11 (point-CSV validation swallowed by the survey fallback — **server-side `point_ingest.py`, not carried yet**).

## Known unknowns
`PathManager` sidecar/entity-order behaviour is exercised by the 43 skipped tests; Phase 2 must re-establish equivalent coverage when it builds storage and mission upload.
