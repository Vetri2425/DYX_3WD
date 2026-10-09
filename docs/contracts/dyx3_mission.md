# Contract — `dyx3_mission`

Spec §7.1/§7.4/§4.3.1; Phase plan P3. **Mission decides WHAT is executed; it never computes steering, never publishes `/fmu/**`,
and never re-implements a safety gate.** Authored 2026-10-07 from the spec and the prototype's `mission_runner_node.py`/`mission_progress.py`
semantics (not ported line by line; the prototype's mission-runner touched MAVROS, the server and control arbitration, all gone).

## 1. Single owner of mission start
`ExecuteMission.action` is canonical (goal = start, feedback = progress, cancel = abort); `StartMission.srv` is an admission-only wrapper. **Both call the
same `MissionNode::begin_mission()`**; nothing else may start a mission. A second start while a mission is active is rejected (`REASON_BUSY`).

## 2. States (frozen ABI values, `MissionState.msg`)
`IDLE 0 · LOADING 1 · READY 2 · RUNNING 3 · PAUSED 4 · COMPLETED 5 · ABORTED 6 · ERROR 7`. Reasons: `NONE 0 · OPERATOR 1 · SAFETY 2 · RTK 3 · PATH_ERROR 4 · INTERNAL_ERROR 5`.

| State | Meaning | Motion allowed? |
|---|---|---|
| IDLE | no mission | no |
| LOADING | artifact being read + hash-verified | no |
| READY | artifact verified and published (`MissionState.path_artifact_sha256`); waiting for `dyx3_rpp` to acknowledge it (`RppStatus.mission_id` == this mission) | no |
| RUNNING | RPP acknowledged; **the only state in which the guard's mission gate opens** | yes |
| PAUSED | stopped by operator or by an automatic safety pause; resumable by an explicit Resume only | no |
| COMPLETED / ABORTED / ERROR | terminal; left only by a new accepted start | no |

## 3. Transition table (normative — `mission_fsm.cpp` implements exactly this; `mission_fsm_test.cpp` enumerates every state×event)
Events: `start(gate_ok)` · `artifact_loaded(valid)` · `rpp_ack` · `pause` · `resume(gate_ok)` · `abort(reason)` · `rpp_complete` · `rpp_error` ·
`gate_lost(guard_reason)` · `estop` · `rpp_stale` · `skip_point(has_active_point)`.

| From ↓ / Event → | start | artifact_loaded | rpp_ack | pause | resume | abort | rpp_complete | rpp_error | gate_lost | estop |
|---|---|---|---|---|---|---|---|---|---|---|
| IDLE | →LOADING if gate_ok else **reject SAFETY_GATE** | ✗ | ✗ | reject NOT_RUNNING | reject NOT_PAUSED | reject NOT_ACTIVE | ✗ | ✗ | – | – |
| LOADING | reject BUSY | →READY if valid, else →ERROR(PATH_ERROR) | ✗ | reject NOT_RUNNING | reject NOT_PAUSED | →ABORTED | ✗ | →ERROR(INTERNAL) | →ABORTED(SAFETY/RTK) | →ABORTED(SAFETY) |
| READY | reject BUSY | ✗ | →RUNNING if gate ok else →ABORTED(SAFETY/RTK) | reject NOT_RUNNING | reject NOT_PAUSED | →ABORTED | ✗ | →ERROR(INTERNAL) | →ABORTED(SAFETY/RTK) | →ABORTED(SAFETY) |
| RUNNING | reject BUSY | ✗ | – (ignored) | →PAUSED(OPERATOR) | reject NOT_PAUSED | →ABORTED | →COMPLETED | →ERROR(INTERNAL) | **→PAUSED(SAFETY/RTK)** | →ABORTED(SAFETY) |
| PAUSED | reject BUSY | ✗ | – | reject NOT_RUNNING | →RUNNING if gate_ok else **reject SAFETY_GATE** | →ABORTED | – | →ERROR(INTERNAL) | – (stays) | →ABORTED(SAFETY) |
| COMPLETED / ABORTED / ERROR | →LOADING if gate_ok (new mission id) else reject SAFETY_GATE | ✗ | ✗ | reject NOT_RUNNING | reject NOT_PAUSED | reject NOT_ACTIVE | ✗ | ✗ | – | – |

`rpp_ack_timeout` (READY and no RPP acknowledgement within `rpp_ack_timeout_s`) is **READY →ERROR(INTERNAL_ERROR)**, logged as event `rpp_ack_timeout`; in every other state it is `–`.
`rpp_stale` (no `RppStatus` of this mission within `rpp_status_max_age_s` while RUNNING) is **RUNNING →PAUSED(SAFETY)**, logged as event `rpp_stale`; in every other state it is `–`.
It reuses `REASON_SAFETY` because the reason codes are frozen `.msg` constants; the distinct cause is the logged event/detail.
`resume` additionally requires a fresh `RppStatus` (refused `SAFETY_GATE`), so a resume cannot re-enter RUNNING only to be paused again on the next tick.

`✗` = illegal in that state: the FSM refuses it, logs it, and does not change state (never silent). `–` = no effect. `skip_point` is accepted only in RUNNING/PAUSED with an
active point (it records a `PointResult` SKIPPED; no state change), else `REASON_NO_ACTIVE_POINT` / `REASON_NOT_RUNNING`.

**Never auto-resume.** A gate that recovers does not restart motion; a human issues Resume, which re-checks the gate.
**DERIVED — NOT FROM V1 SPEC:** E-stop aborts (not pauses) so an interrupted mission cannot silently continue (the prototype's `emergency.py` is not ported — human question);
auto-pause on gate loss; READY exists to require an RPP acknowledgement before any motion.

## 4. Every transition is logged
`Transition{seq, from, to, event, reason, detail, stamp_ns}`; the last 256 are retained, each goes to `/rosout`, and the full state is republished as `MissionState` on every
transition and at 10 Hz (DERIVED rate). Illegal events are logged as refused transitions.

### READY acknowledgement and action cancel (review H4/H3, fix plan A2/A3, 2026-10-08)
* Only `RppStatus` LOADED, TRACKING, STOPPING, PIVOTING or CREEPING of this mission acknowledges READY. ERROR is evaluated
  first (READY -> ERROR, `REASON_INTERNAL_ERROR`); COMPLETE in READY is neither an ack nor a completion.
* `ExecuteMission` cancel aborts the mission inside the cancel callback (REASON_OPERATOR); the goal is not CANCELING yet
  there, so the timer finalises it once it is: result code CANCELED, `RESULT_ABORTED`.

## 5. Inputs
`SafetyGateStatus` (guard-owned aggregate, 10 Hz; stale > 0.5 s or never seen ⇒ **not ok**), `RppStatus`, `VehicleState` (position for the point journal).
Mission never reads RTK/estimator/E-stop directly — one owner per gate (the guard).

## 6. Point journal
Must-hit vertices (bit1 of the artifact flags) in path order; **point index = rank among must-hit vertices** (same as the prototype's `_musthit_rank_by_key`). A vertex is *captured* when the rover
comes within `point_capture_radius_m` (default **0.10 m = prototype `point_hold_acceptance_m`**, DERIVED); the result is emitted when the rover leaves the radius or the path ends:
`COMPLETED` with the **closest-approach distance** and NED position; a vertex bypassed because a later vertex was captured first is `FAILED`; operator skip is `SKIPPED`; unresolved at the end ⇒ `FAILED`.
(Journal reports geometry; the *dwell/handshake* logic is RPP/spray's — `point_hold*` parameters are not mission parameters.)

## 7. Path artifact
C++ reader + SHA-256 (`path_artifact.cpp`, `sha256.cpp`); must refuse everything the Python `decode()` refuses (`docs/contracts/path_artifact.md`), and compute the same hash. Coordinates must be in Python `repr(float)` spelling (so `1`, `1.50`, `1e0`, `+1.0`, hex floats, NaN/inf are refused) and `meta` must be a canonical JSON object (valid, sorted unique keys, compact separators, `ensure_ascii` escapes, canonical numbers); both were previously accepted by the C++ reader and refused by Python. The reader is stricter than Python only on control/non-ASCII bytes, an empty engine id, flag spelling and JSON nesting depth (> 64). Cross-checked against the Python decoder on 14k generated and mutated artifacts: the C++ reader never accepted what Python refused. The file must be a regular file of at most `kMaxArtifactBytes` (64 MiB, DERIVED: a planned path is a few MB, the backend caps a mission at 50k points and uploads at 20 MiB); the size is checked before the file is read so the Start service callback cannot be stalled by a huge or special file. A mismatch is
`ERROR(PATH_ERROR)` and the start response is `REASON_INVALID_ARTIFACT`.

## 8. Parameters (classes per spec §9)
| name | default | class | source |
|---|---|---|---|
| `missions_dir` | `/var/lib/dyx3/missions` | RESTART | spec §12 filesystem |
| `state_publish_hz` | 10 | IDLE_ONLY | DERIVED |
| `gate_status_max_age_s` | 0.5 | IDLE_ONLY | prototype freshness convention (`pose_max_age_s`/`rtk_fix_timeout_s` 0.5) — DERIVED |
| `point_capture_radius_m` | 0.10 | IDLE_ONLY | prototype `point_hold_acceptance_m` — DERIVED |
| `rpp_status_max_age_s` | 0.5 | IDLE_ONLY | RPP/guard freshness convention (guard command age 0.2 s, other limits 0.5 s) — DERIVED; finite, > 0 |
| `rpp_ack_timeout_s` | 30 | IDLE_ONLY | **no source** — conservative DERIVED value; must exceed the largest mission's RPP conditioning time (raise it for larger missions). Finite and > 0: 0 ("disabled") is refused, a READY mission can no longer wait forever |

The five IDLE_ONLY values are applied as one validated batch only while the FSM is IDLE.
`state_publish_hz` replaces the wall timer, while the freshness, capture-radius and RPP-ACK
values update their effective caches together. A rejected atomic parameter request leaves both
ROS values and effective values unchanged. The node runs on a single-threaded executor, so
parameter application and timer callbacks do not overlap.
COMPLETED, ABORTED and ERROR remain terminal in this FSM: a new accepted start goes directly
to LOADING, not IDLE. Thus IDLE_ONLY changes are rejected in all three terminal states. To
reconfigure after a mission, the operator must first leave the vehicle safely stopped and
disarmed, then restart the mission node; its initial state is IDLE, where the validated batch
can be applied before a new mission starts. A new StartMission is not a reconfiguration route.

## 9. Open questions
E-stop → ABORTED vs PAUSED; the numeric value of `rpp_ack_timeout_s` (default 30 s, no source; the RPP-status staleness auto-pause is implemented with a DERIVED 0.5 s limit); mission-id persistence across reboot (currently per-boot counter).
