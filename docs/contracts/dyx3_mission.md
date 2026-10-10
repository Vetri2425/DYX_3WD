# Contract — `dyx3_mission`

Spec §7.1/§7.4/§4.3.1; Phase plan P3; **mission contract v2** (`docs/plans/2026-10-10_mission_contract.md` §3,
interfaces 0.15.0). **Mission decides WHAT is executed and drives the vehicle's arm / OFFBOARD lifecycle through
`dyx3_px4_link`; it never computes steering, never publishes `/fmu/**`, and never re-implements a safety gate.**
Authored 2026-10-07 from the spec and the prototype's `mission_runner_node.py`/`mission_progress.py` semantics; rewritten
2026-10-10 for v2; recovery from a transient (persisted progress, resume, re-engage from PAUSED, pivot-timeout pause) added
2026-10-10 for interfaces 0.17.0 (review `docs/reviews/2026-10-10_production_blockers_transport_timing_review.md` P2).

## 1. Single owner of mission start; asynchronous admission
`ExecuteMission.action` is canonical (goal = start, feedback = progress, cancel = abort); `StartMission.srv` is an admission
wrapper. **Both call the same `MissionNode::begin_mission()`**; nothing else may start a mission.

**The start is asynchronous (v2).** `begin_mission()` only admits or refuses: no file I/O, no hashing, no PX4 call. It
returns the execution id (`mission_id`, incremented on every accepted start, per process) in well under 50 ms
(`StartIsAnAdmissionThatReturnsAtOnce`). Loading, placement, arming and OFFBOARD then run as the lifecycle below, driven by
the node's timers and callbacks; the file work runs on a worker thread (`std::async`, polled every 5 ms while it runs), the
PX4 calls are asynchronous clients with explicit timeouts. A missing or corrupt artifact is therefore an *accepted* start
that ends in `ERROR(PATH_ERROR)`.

Admission, in order:

| check | refusal (`StartMission.Response`) |
|---|---|
| `request_id` longer than 64 or outside `[A-Za-z0-9._:-]` | `REASON_INVALID_REQUEST` (4) |
| `request_id` non-empty and equal to the most recent execution's | **accepted, `duplicate = true`, that execution's `mission_id`; nothing starts** |
| artifact id not 64 lowercase hex | `REASON_INVALID_ARTIFACT` (1); no execution is created |
| an execution is active, or the previous one is still releasing OFFBOARD / disarming (or a file job of it still runs) | `REASON_BUSY` (2) |
| the guard's pre-arm gate (`SafetyGateStatus.pre_arm_ok`) not fresh and ok | `REASON_SAFETY_GATE` (3), `gate_reason_code` = the guard's first failing pre-arm gate (`MotionSetpointStatus.REASON_*`; `REASON_STALE` when no fresh `SafetyGateStatus`) |

**Idempotency.** The `request_id` of the most recent execution keeps returning that execution (active or already
terminal), so a client retry after a lost reply never starts a second execution. An action goal carries no `request_id`.
A duplicate also returns that execution's `resumed_run_index`.

**Resume (0.17.0).** `StartMission.resume = true` starts the execution from the persisted progress of the same source
artifact (section 9a) instead of run 0, and answers the run it starts from in `resumed_run_index`. The decision is made
at admission from the node's in-memory copy of the progress records (read at construction, kept current on every write),
so admission still does no file I/O. Without a usable record (none, or one whose execution COMPLETED) the start is a
fresh start and `resumed_run_index = 0`. `resume = false` always starts at run 0 and overwrites the record (from PLACING
on, section 9a). The `ExecuteMission` action goal has no resume field: it is always `resume = false`. A resumed start is
admitted exactly like any other (same refusals, same lifecycle).

## 2. States (`MissionState.msg`; 0..7 frozen, 8..10 appended in 0.15.0)

```
IDLE --start--> LOADING --artifact ok--> PLACING --placed--> ARMING --armed--> ENGAGING --OFFBOARD confirmed-->
READY --RPP ack + full gate--> RUNNING --complete--> COMPLETED
RUNNING <--pause / automatic pause | resume (full gate)--> PAUSED
PAUSED --resume, only arm/OFFBOARD lost--> ARMING --armed--> ENGAGING --OFFBOARD confirmed + full gate--> RUNNING
          (re-engage, section 3a: published as PAUSED until RUNNING; no READY handshake)
any pre-RUNNING step fails --> ERROR(reason) + STOP + release what this execution engaged/armed + execution unloaded
active --abort--> ABORTED;  E-stop asserted in any active state --> ABORTED(ESTOP) + disarm
```

| State | value | Meaning | Vehicle | Motion allowed? |
|---|---|---|---|---|
| IDLE | 0 | no execution since the node started | — | no |
| LOADING | 1 | the source artifact is read and hash-verified (worker thread) | as found | no |
| PLACING | 8 | the trajectory is placed in the EKF frame, the execution artifact written (worker thread) | as found | no |
| ARMING | 9 | `/dyx3/px4_link/arm(true)` sent, waiting for its confirmed answer | arming | no |
| ENGAGING | 10 | `/dyx3/px4_link/set_offboard(true)` sent, waiting for its confirmed answer | armed | no |
| READY | 2 | armed + OFFBOARD; `path_artifact_sha256` published; waiting for `dyx3_rpp` to acknowledge it **and** the full gate | armed, OFFBOARD, STOP | no |
| RUNNING | 3 | **the only state in which the guard's mission gate opens** | armed, OFFBOARD | yes |
| PAUSED | 4 | operator pause or automatic pause; left only by an explicit Resume (which re-engages when the vehicle lost only arm / OFFBOARD, section 3a); also published during that re-engage | armed, OFFBOARD, STOP stream (re-engage: arming / engaging, STOP) | no |
| COMPLETED / ABORTED / ERROR | 5 / 6 / 7 | terminal; left only by a new accepted start (→ LOADING) | released (section 5) | no |

**Reasons** (`MissionState.REASON_*`): `NONE 0 · OPERATOR 1 · SAFETY 2 · RTK 3 · PATH_ERROR 4 · INTERNAL_ERROR 5` and, 0.15.0:
`EKF_RESET 6 · EKF_REFERENCE_INVALID 7 · PLACEMENT_OUT_OF_BOUNDS 8 · NO_PLACEMENT_FRAME 9 · ARM_REFUSED 10 · ARM_TIMEOUT 11 ·
OFFBOARD_REFUSED 12 · OFFBOARD_TIMEOUT 13 · RPP_ACK_TIMEOUT 14 · ESTOP 15 · RPP_ERROR 16 · RPP_STALE 17` and, 0.17.0:
`RPP_PIVOT_TIMEOUT 18` (section 3b).
With `SAFETY` / `RTK`, `gate_reason_code` names the guard gate (`MotionSetpointStatus.REASON_*`), 0 otherwise.

**Other `MissionState` fields (0.15.0):** `path_artifact_sha256` = the **execution** artifact RPP loads (set when PLACING
succeeds, cleared on ERROR); `source_artifact_sha256` = the artifact the operator started (kept until the next start); `dyx3_recorder` writes both to the run's `manifest.json` (`docs/contracts/dyx3_recorder.md`);
`request_id`; `reason_detail` (human-readable cause, plus `; release: …` when a release step failed); `waiting_on`
(`WAIT_NONE 0 · ARTIFACT 1 · PLACEMENT 2 · ARM 3 · OFFBOARD 4 · RPP_ACK 5 · OPERATOR 6 · OFFBOARD_RELEASE 7 · DISARM 8`; a
pending release step takes precedence over the state's own wait; during a re-engage PAUSED carries `ARM` / `OFFBOARD`);
`state_entered` (ROS time of the latest transition, a re-engage's internal ARMING / ENGAGING steps included).
0.17.0: `start_run_index`, the run `dyx3_rpp` starts this execution from (0, or the resumed run, section 9a), set from
LOADING on and kept for the whole execution.

## 3. Transition table (normative — `mission_fsm.cpp` implements exactly this; `mission_fsm_test.cpp` enumerates every state × event)
"Active" = LOADING, PLACING, ARMING, ENGAGING, READY, RUNNING, PAUSED. "Before RUNNING" = LOADING … READY.

| Event | Legal in | Result |
|---|---|---|
| `start(pre_arm_ok)` | IDLE, COMPLETED, ABORTED, ERROR | → LOADING (new `mission_id`) if `pre_arm_ok`, else **refused SAFETY_GATE**; active: **refused BUSY** |
| `artifact_loaded(valid)` | LOADING | → PLACING, or → ERROR(PATH_ERROR) |
| `placed(ok, reason)` | PLACING | → ARMING, or → ERROR(NO_PLACEMENT_FRAME / EKF_REFERENCE_INVALID / PLACEMENT_OUT_OF_BOUNDS / INTERNAL_ERROR) |
| `armed(ok, reason)` | ARMING | → ENGAGING, or → ERROR(ARM_REFUSED / ARM_TIMEOUT) |
| `engaged(ok, reason)` | ENGAGING | → READY, or → ERROR(OFFBOARD_REFUSED / OFFBOARD_TIMEOUT); during a re-engage `ok` → no change (OFFBOARD confirmed, waiting for the full gate) |
| `rpp_ack(full_gate_ok)` | READY | → RUNNING only with the full gate; otherwise **held** (no change, logged) until the gate passes or the ack timeout; RUNNING/PAUSED: ignored |
| `pause` | RUNNING | → PAUSED(OPERATOR); else refused NOT_RUNNING |
| `resume(ok)` | PAUSED | → RUNNING if ok, else **refused SAFETY_GATE**; during a re-engage: accepted, no change (a retry); else refused NOT_PAUSED |
| `reengage` | PAUSED | → ARMING (re-engage, section 3a); during a re-engage: accepted, no change; else refused NOT_PAUSED |
| `reengage_gate(full_gate_ok)` | ENGAGING of a re-engage, OFFBOARD confirmed | → RUNNING with the full gate; otherwise **held** (no change); anything else: illegal |
| `reengage_timeout(guard_reason)` | ENGAGING of a re-engage, OFFBOARD confirmed | → ERROR(SAFETY / RTK) (the full gate never passed); anything else: no effect |
| `abort(reason)` | active | → ABORTED(OPERATOR, or SAFETY when requested); else refused NOT_ACTIVE |
| `rpp_complete` | RUNNING | → COMPLETED; PAUSED and a re-engage's ARMING / ENGAGING: ignored |
| `rpp_error` | READY, RUNNING, PAUSED, a re-engage's ARMING / ENGAGING | → ERROR(RPP_ERROR); LOADING…ENGAGING of a start: ignored (RPP holds nothing of this execution yet) |
| `gate_lost(guard_reason)` | before RUNNING (a re-engage's ARMING / ENGAGING included) | → **ERROR**(SAFETY / RTK) — the *pre-arm* gate is the one evaluated before RUNNING |
| | RUNNING | → **PAUSED**(SAFETY / RTK) — the *full* gate; PAUSED / inactive: no effect |
| `estop` | active | → **ABORTED(ESTOP)** (and the vehicle is disarmed, section 5) |
| `ekf_reset(detail)` | READY, RUNNING, PAUSED | → PAUSED(EKF_RESET) (a PAUSED → PAUSED self-transition replaces the pause reason) |
| | ARMING, ENGAGING (a re-engage's too) | → ERROR(EKF_RESET) |
| `rpp_ack_timeout(gate_ok, guard_reason)` | READY | → ERROR(RPP_ACK_TIMEOUT), or ERROR(SAFETY / RTK) when it was the full gate that never passed |
| `rpp_stale` | RUNNING | → PAUSED(RPP_STALE) |
| `rpp_pivot_timeout` | RUNNING | → PAUSED(RPP_PIVOT_TIMEOUT) (section 3b); elsewhere no effect |
| `skip_point(has_point)` | RUNNING, PAUSED, a re-engage's ARMING / ENGAGING | accepted with an active point (a `PointResult` SKIPPED; no state change), else refused NO_ACTIVE_POINT; else NOT_RUNNING |

An event outside its "legal in" set is refused as `illegal_in_state`, logged, and changes nothing (never silent).

**Never auto-resume.** A gate that recovers, an EKF that settles or an RPP that reports again does not restart motion; a
human issues Resume. Resume requires, in this order of refusal reasons (`ResumeMission.Response`):
the EKF reference unchanged since placement and a fresh `VehicleState` (`REASON_EKF_REFERENCE_CHANGED` 4); the **full** guard
gate, armed + OFFBOARD included (`REASON_NOT_ARMED_OR_OFFBOARD` 3 when the guard's reason is `ARMING_GATE`, else
`REASON_SAFETY_GATE` 2); a fresh `RppStatus` (`REASON_SAFETY_GATE`). **Except (0.17.0):** when the reference is unchanged,
`RppStatus` is fresh, the pre-arm gate is fresh and ok, and the full gate fails only with `ARMING_GATE` (the vehicle is no
longer armed or no longer in OFFBOARD), the resume is **accepted** (`REASON_OK`) and re-engages (section 3a). Every other
refusal is unchanged; in particular `REASON_NOT_ARMED_OR_OFFBOARD` is still answered when the arming gate is the guard's
first failing gate but another pre-arm gate fails too. OFFBOARD or arm lost while paused is never re-engaged
*automatically*: only this operator call does it. An accepted resume (and an accepted re-engage) re-baselines
`xy_reset_counter` (the operator saw the EKF reset and the reference is unchanged).

### 3a. Re-engage from PAUSED (0.17.0)
After a PX4-side OFFBOARD or arm loss the mission pauses (`gate_lost` → PAUSED(SAFETY, `gate_reason_code` ARMING_GATE)).
A resume then runs the start's own PX4 steps again inside the same execution (same `mission_id`, same execution artifact,
same journal):

```
                 resume (only ARMING_GATE fails, pre-arm ok,
                 reference unchanged, RppStatus fresh)
   PAUSED ───────────────────────────────────────────────► ARMING ──armed──► ENGAGING ──OFFBOARD confirmed──┐
     ▲   published: PAUSED, waiting_on ARM                   │    published: PAUSED, waiting_on OFFBOARD   │
     │                                                       │                                             ▼
     │                                     arm refused / timeout ─► ERROR(ARM_REFUSED / ARM_TIMEOUT)   full gate ok?
     │                                     OFFBOARD refused / timeout ─► ERROR(OFFBOARD_REFUSED / _TIMEOUT)  │ yes
     │                                     full gate not ok within offboard_timeout_s ─► ERROR(SAFETY / RTK) ▼
     └────────────── pause / automatic pause ◄──────────────────────────────────────────────────────────── RUNNING
```

* **Steps.** ARMING sends `arm(true)` (px4_link answers at once when PX4 is still armed), ENGAGING sends
  `set_offboard(true)`, each with its existing timeout (`arm_timeout_s`, `offboard_timeout_s`) and its existing failure
  reasons, exactly as for a start; the pre-arm gate must hold at each step's entry and throughout (lost → ERROR(SAFETY /
  RTK)), an E-stop aborts, an EKF reset is ERROR(EKF_RESET). Every ERROR releases (section 5).
* **No READY handshake.** `dyx3_rpp` still holds the execution, loaded and paused. Instead, after OFFBOARD is confirmed the
  FSM waits for the **full** guard gate (the guard's verdict may lag px4_link's confirmation) and enters RUNNING only with it.
  DERIVED — NOT FROM V1 SPEC: that hold is bounded by `offboard_timeout_s` from the confirmation; a full gate that never
  passes is ERROR(SAFETY / RTK) with the guard's reason, as READY's `rpp_ack_timeout` does.
* **What RPP and every other consumer see.** `dyx3_rpp` unloads on ARMING / ENGAGING, so a re-engage's ARMING / ENGAGING is
  **published as `STATE_PAUSED`** (with `waiting_on` ARM / OFFBOARD, `reason_code` NONE and a `reason_detail` saying so) until
  RUNNING is re-entered. RPP keeps streaming STOP; the guard's mission gate stays closed.
* **During the re-engage** a retried resume is accepted without effect, pause is refused (NOT_RUNNING), abort aborts,
  skip-point works as in PAUSED, an RPP ERROR is ERROR(RPP_ERROR), an RPP COMPLETE is ignored.
* **Ownership.** The release after a re-engage undoes everything the execution did: OFFBOARD was requested by the start,
  and an arm confirmed (or in doubt) in any cycle stays owned even when a later re-arm is refused (section 5).

### 3b. Pivot timeout (0.17.0)
`RppStatus.pivot_timed_out == true` for the current `mission_id` while RUNNING (RPP's corner / alignment pivot watchdog
expired with the heading outside the release band) → PAUSED(`REASON_RPP_PIVOT_TIMEOUT` 18), `reason_detail`
"rpp pivot watchdog expired (heading not reached)". It is an automatic pause like RPP_STALE: armed, OFFBOARD, STOP, never an
automatic resume; a later resume goes through section 3a if the vehicle lost arm / OFFBOARD meanwhile. DERIVED — NOT FROM
V1 SPEC: one pause per expiry. RPP keeps the flag up while its watchdog stays expired; after the operator resumed from this
pause, the same expiry does not pause again. Only a new expiry (RPP reported `false` in between) pauses again.

**DERIVED / owner decisions:** E-stop aborts and disarms (owner, 2026-10-10); automatic pause on gate loss; READY requires an
RPP acknowledgement and the full gate before any motion.

## 4. Lifecycle steps (entry actions — `MissionNode::enter()`)
* **LOADING:** `load_artifact(missions_dir, source_sha)` on a worker thread (MS-004 size cap, hash verified against the file
  name). Not valid → ERROR(PATH_ERROR).
* **PLACING:** needs a fresh `VehicleState` (≤ `vehicle_state_max_age_s`; else ERROR(EKF_REFERENCE_INVALID)). Records the EKF
  reference (`reference_latitude_deg/longitude_deg`) and `xy_reset_counter`, then places and stores the execution artifact on
  a worker thread (section 6). The point journal is built over the **execution** (EKF-frame) points.
* **ARMING / ENGAGING:** the pre-arm gate must be fresh and ok **at that instant** (else ERROR(SAFETY/RTK), or ABORTED(ESTOP)
  for an E-stop); then `arm(true)` / `set_offboard(true)` is sent. **Never arm unless every pre-arm gate is OK.**
* **READY:** starts the `rpp_ack_timeout_s` clock. `dyx3_rpp` loads `path_artifact_sha256` (it is active on READY) and reports
  `RppStatus` LOADED (or TRACKING/STOPPING/PIVOTING/CREEPING) for this `mission_id`; that acknowledges READY. ERROR is evaluated
  first (READY → ERROR(RPP_ERROR)); COMPLETE in READY is neither an ack nor a completion.
* **While active:** every `SafetyGateStatus` and the state timer re-evaluate the gate — before RUNNING the pre-arm gate (loss
  → ERROR), in RUNNING the full gate (loss → PAUSED); an E-stop reason in either → ABORTED(ESTOP). RUNNING with no `RppStatus`
  of this mission within `rpp_status_max_age_s` → PAUSED(RPP_STALE); RUNNING and `RppStatus.pivot_timed_out` →
  PAUSED(RPP_PIVOT_TIMEOUT) (section 3b).
* **EKF reset:** from ARMING on, a change of `VehicleState.xy_reset_counter` (or, for an anchored placement, of the EKF global
  reference or its validity) is reported once → `ekf_reset` (section 3).
* **Pre-arm gate:** `SafetyGateStatus.pre_arm_ok` / `pre_arm_reason_code`, published by the guard (`docs/contracts/dyx3_motion_guard.md`):
  every guard gate except "armed" and "OFFBOARD" — E-stop clear, PX4 link healthy, operator link, vehicle state fresh with no
  PX4 failsafe, RTK fixed, heading valid, estimator healthy — plus `VehicleState.global_reference_valid`. The mission node
  never re-implements a gate; stale (> `gate_status_max_age_s`) or never seen ⇒ not ok.

## 5. PX4 calls and terminal actions (`px4_sequencer.cpp`)
All through `dyx3_px4_link` (`/dyx3/px4_link/arm` `ArmDisarm`, `/dyx3/px4_link/set_offboard` `SetOffboard`), which itself
pre-streams STOP, commands and **confirms**. One request in flight at a time, each with its own timeout:

| call | timeout parameter | default | validated | why |
|---|---|---|---|---|
| `arm(true)`, `arm(false)` | `arm_timeout_s` | 4.0 s | > 2.0 s, ≤ 60 s | px4_link confirms the arm within `arm_confirm_timeout_s` 2.0 s |
| `set_offboard(true/false)` | `offboard_timeout_s` | 5.0 s | > 3.5 s, ≤ 60 s | px4_link answers within prestream 0.5 s + confirm 2.0 s (+ 1.0 s margin) |

px4_link's own definitive answer therefore always arrives before the mission's timeout. A mission timeout drops the client
request (`remove_pending_request`); a late answer is ignored.

**What this execution owns.** The sequencer records whether *this execution* armed (confirmed, in flight, timed out, or
refused with `ArmDisarm.REASON_TIMEOUT` = "commanded, not confirmed": all count as armed — on any doubt, disarm) and whether it
requested OFFBOARD. A refused arm (px4_link refused before commanding) owns nothing **of its own**: an execution that
re-engages (section 3a) runs a second arm → OFFBOARD cycle, and what an earlier cycle armed stays owned (a refused re-arm
never clears it). Ownership is reset only by the next execution's `begin_execution`.

**Release, on entry to every terminal state** (best effort, each step with its timeout, each result logged; a failed step is
appended to `reason_detail` as `release: …` and does not stop the next step):

| terminal | release |
|---|---|
| COMPLETED, ABORTED | `set_offboard(false)` if this execution requested OFFBOARD, then `arm(false)` if it armed |
| ERROR | the same: OFFBOARD released only if this execution engaged it, disarm only if it armed (or may have); **the execution is unloaded** (`path_artifact_sha256` cleared, source artifact and journal dropped), so nothing can drive it later |
| ABORTED(ESTOP) | as above, and `arm(false)` **always** (owner decision: E-stop disarms), even if this execution never armed |

A release overtakes an engage step still in flight (that request's answer no longer drives the lifecycle; the disarm is sent
at once). Throughout, STOP is what reaches PX4: the guard fails to zero outside RUNNING, and px4_link's `set_offboard(false)`
holds STOP before withdrawing the heartbeat. A new start is refused BUSY until the release has finished (`waiting_on` is
`OFFBOARD_RELEASE` / `DISARM` meanwhile, `NONE` after). **PAUSED does not release:** armed, OFFBOARD, STOP stream.

## 6. Placement (frame) — SAFETY-CRITICAL (`frame_placement.cpp`)
The artifact meta (canonical JSON) decides:

| meta | placement |
|---|---|
| `"frame":"local_ned"` + `"anchor":{"lat","lon"}` | **anchored**: placed as below; a new content-addressed execution artifact |
| `"frame":"ekf_local_ned"`, anchor null or absent | **passthrough**: already EKF local NED; execution sha = source sha, nothing written |
| no `"frame"` (old / planner artifacts), `local_ned` without anchor, `ekf_local_ned` with one, unknown frame, a placed execution (`"ekf_execution"`) | ERROR(NO_PLACEMENT_FRAME): EKF-local is never assumed |

**Coordinates of an anchored artifact (owner decision 2026-10-10).** Point (n, e) values are **metres in the WGS84 ENU
local tangent plane at the anchor** (anchor at ellipsoid height 0), exactly the tablet's model: a CAD drawing is a flat
plane, and this plane is tangent to the ellipsoid at the anchor. WGS84 `a = 6378137`, `f = 1/298.257223563`,
`e² = f(2 − f)`. A small-distance approximation (`north = Δφ · M(φ0)`, `east = Δλ · N(φ0) cos φ0`) is NOT used: it drifts
18 mm at 1 km east and 36 mm at 1 km × 1 km from the plane at 13° N, over the 1 cm goal.

**One function, one path — `place_point(ekf_ref, anchor, (n, e))`, for every point (the anchor is (0, 0)):**
1. invert the tablet model at the anchor: the plane point `(e, n, u = 0)` rotated from ENU at the anchor into ECEF, plus the
   anchor's ECEF, then ECEF → geodetic (latitude iterated to convergence, height dropped). Exact, no distance limit;
2. project (φ, λ) into the EKF frame with **PX4's own `MapProjection::project`** (`src/lib/geo/geo.cpp` of the pinned
   firmware, ported verbatim in double precision): azimuthal equidistant on the sphere `R = 6 371 000 m`
   (`CONSTANTS_RADIUS_OF_EARTH`) about the live EKF reference `VehicleState.reference_latitude_deg/longitude_deg`:
   `c = acos(sin φr sin φ + cos φr cos φ cos Δλ)`, `k = c / sin c` (1 at c = 0),
   `x = k (cos φr sin φ − sin φr cos φ cos Δλ) R`, `y = k cos φ sin Δλ R`.

This is the function EKF2 uses to turn a GNSS fix into a local position, so a placed point is exactly where the EKF will
report the rover when its GNSS reads that point's latitude/longitude. There is no flag and no "simple translation"
fallback.

**Why not a translation.** EKF metres are sphere metres. At 13° N the WGS84 radii are `M = 6 338 659.94 m` (meridian) and
`N = 6 379 217.59 m` (prime vertical), so 100 m of ground north is `100.510200 m` of EKF (`≈ 100 R/M`, +0.510 %) and 100 m of
ground east is `99.871182 m` (`≈ 100 R/N`, −0.129 %).
Translating the design by the projected anchor would paint it 51.0 cm short north-south and 12.9 cm long east-west per
100 m (`EllipsoidScaleAtThirteenDegreesNorth`).

**Accuracy.** Step 1 is exact for the defined model (round trip with the forward ENU < 0.03 mm at 1 km, 1.5 mm at 5 km). Step 2 is PX4's projection itself; the only
difference from the EKF is that PX4 returns `float`: within 1 km a float32 has a resolution of ≤ 6.1e-5 m, so the EKF's own
positions are quantised to ≤ 0.06 mm. **The placed point matches the EKF's placement of the same latitude/longitude to
< 0.1 mm within 1 km (well under 1 mm per km).** The pipeline does not add the sphere-versus-ellipsoid error, because it uses
the same sphere as the EKF. The azimuthal projection's own second-order terms appear in the EKF frame exactly as the EKF
sees them (the plane's east axis and the projection's agree to 1 µm at 100 m).

**Bounds and failures.** The anchor and every **placed** point must lie within `placement_max_distance_m` (≤ 1000 m) of the
EKF origin (the projection accuracy statement above holds there), else ERROR(PLACEMENT_OUT_OF_BOUNDS); an anchor at a pole
(no east direction) is out of bounds too. No valid EKF global reference → ERROR(EKF_REFERENCE_INVALID).

**Execution artifact.** DYX3PATH 1, canonical (`serialize_artifact`, Python `repr` floats), re-read by the same reader RPP uses
before it is accepted, written as `<missions_dir>/<sha256>.dyx3path` (temporary file + rename, so RPP never sees a partial
file). Meta (sorted keys): `{"execution":{"anchor_ekf_ne_m":[n,e],"ekf_reference":{"lat","lon"},"method":
"wgs84_enu_tangent_plane_to_px4_map_projection","source_sha256":…},"frame":"ekf_execution","source_meta":{…}}`. The same source
and reference give the same bytes and sha. An execution artifact is never accepted as a source (frame `ekf_execution`).

**EKF reset after placement.** The placement records `xy_reset_counter` and the reference. A change while READY, RUNNING
or PAUSED → PAUSED(EKF_RESET) (ARMING/ENGAGING → ERROR(EKF_RESET)), never an automatic resume; a changed reference also
makes Resume refuse (`REASON_EKF_REFERENCE_CHANGED`).

## 7. Every transition is logged
`Transition{seq, from, to, event, reason, detail, stamp_ns}`; the last 256 are retained, each goes to `/rosout`. A
`MissionState` is published **immediately on every transition** (depth 20, reliable: several transitions can follow each
other in one callback; a subscriber that wants them all keeps a depth > 1) and at `state_publish_hz`. Illegal events are
logged as refused transitions. `ExecuteMission` cancel aborts (REASON_OPERATOR) inside the cancel callback; the goal is not
CANCELING yet there, so the timer finalises it once it is (CANCELED, `RESULT_ABORTED`).

## 8. Inputs
`SafetyGateStatus` (guard aggregate, 10 Hz: full gate and pre-arm gate; stale > `gate_status_max_age_s` or never seen ⇒ **not
ok**), `RppStatus` (only this `mission_id`), `VehicleState` (EKF reference, `xy_reset_counter`, `global_reference_valid`,
position for the point journal), and the px4_link service answers. Mission never reads RTK / estimator / E-stop directly —
one owner per gate (the guard); E-stop is recognised as the guard's `REASON_ESTOP` (the guard's highest-priority gate).

**Clocks.** All mission timing is on the node's steady clock (`std::chrono::steady_clock`, injectable in the `MissionNode`
constructor for tests): input freshness (`gate_status_max_age_s`, `rpp_status_max_age_s`, `vehicle_state_max_age_s`, measured
from the receipt time, never from the message stamp), the READY `rpp_ack_timeout_s`, the px4 sequencer deadlines
(`arm_timeout_s`, `offboard_timeout_s`) and the FSM's transition times (`Transition.stamp_ns`). A wall-clock step (the first
NTP sync over the LTE link; the Orin has no battery RTC) therefore cannot make a fresh input look stale or let a deadline
expire early, which is the same rule `dyx3_rpp`, `dyx3_motion_guard` and `dyx3_px4_link` follow. A negative age (a clock that
went back) is not fresh either. Message stamps keep ROS time: `MissionState.stamp`, `MissionState.state_entered` (the ROS time
recorded at the transition) and `PointResult.stamp`; no age or deadline is ever computed from them.

## 9. Point journal
Must-hit vertices (bit1 of the artifact flags) of the **execution** artifact, in path order; **point index = rank among
must-hit vertices**. A vertex is *captured* when the rover comes within `point_capture_radius_m` (default **0.10 m = prototype
`point_hold_acceptance_m`**, DERIVED); the result is emitted when the rover leaves the radius or the path ends: `COMPLETED`
with the **closest-approach distance** and NED position; a vertex bypassed because a later vertex was captured first is
`FAILED`; operator skip is `SKIPPED`; unresolved at the end ⇒ `FAILED`.

## 9a. Persisted progress and resume (0.17.0)
Point progress no longer lives only in memory: after a graph restart, a reboot or an abort, a start with `resume = true`
continues the same path instead of starting at point 0 (a mid-line restart from point 0 is a visible defect).

**File.** `<missions_dir>/progress/<path_artifact_sha256>.json`, keyed by the **source** artifact
(`StartMission.path_artifact_sha256`): an anchored path's execution artifact depends on the EKF reference, which a
transient (a PX4 reboot) may change, while the must-hit ranks are the same in source and execution (placement keeps point
order and flags). One line of JSON, keys in this order (`mission_progress.cpp`):

```
{"schema":1,"path_artifact_sha256":"<64 hex>","mission_id":7,"run_index":2,"point_index":3,
 "completed_points":[0,1,2],"state":"PAUSED","updated_utc":"2026-10-10T12:00:00Z"}
```

| key | meaning |
|---|---|
| `schema` | 1; a reader refuses any other value |
| `mission_id` | the execution that wrote it (per process: evidence, not identity) |
| `run_index` | the **first run not yet completed**: the highest run `RppStatus` reported for this execution while RUNNING / PAUSED / re-engaging with RPP holding the path (RPP drives its runs in order, so the run it reports is in progress and every earlier one is complete); the start run before RPP reports |
| `point_index` | the active must-hit point (`MissionState.point_index`) |
| `completed_points` | the must-hit points whose `PointResult` was issued **while driving** (COMPLETED, SKIPPED, or FAILED by a bypass), ascending, always `0..k-1` (the journal resolves in path order). The points FAILED by the end-of-execution sweep of an ABORTED / ERROR execution are **not** in it: they were never reached |
| `state` | the `MissionState` name of the FSM at the write (a re-engage's ARMING / ENGAGING are written as such) |
| `updated_utc` | system clock, ISO 8601; a record only, never used for an age |

**When.** From the moment the artifact is verified (LOADING → PLACING) until the terminal state of the execution, on every
state change (one record after each batch of entry actions) and on every `PointResult` issued while driving, and when RPP's
run advances. A start whose artifact never verifies writes nothing (no record for a missing file). The terminal state is the
last record of an execution. **COMPLETED marks the record complete** (`state` "COMPLETED"); it is kept as evidence and is never
resumed. ABORTED / ERROR records stay resumable.

**How.** Off the executor thread: the node queues the record (at most one pending write per artifact, the newest wins) and
one `std::async` writer job at a time writes the queue, polled by the 5 ms work timer like the artifact jobs. Each write is
atomic: temporary file `<sha>.json.tmp.<pid>`, `fsync`, `rename`, `fsync` of the directory (the pattern of
`store_execution_artifact`, plus the syncs). A failed write is logged and changes nothing else (the mission is not
stopped). The node destructor writes whatever is still queued. At construction (before the executor runs) the node reads
every valid record of the directory once into memory; invalid or oversized (> `kMaxProgressBytes` 4 MiB, DERIVED) files
and files not named after the id they hold are skipped with a warning; temporary files are ignored. The in-memory copy is
updated whenever a record is queued, so admission never reads a file.

**Resume semantics** (`StartMission.resume = true`, a record for the artifact that is not COMPLETED):
* `resumed_run_index` = `MissionState.start_run_index` = the record's `run_index`, from LOADING on; `dyx3_rpp` installs the
  execution and begins at that run (an index outside the conditioned runs is RPP's ERROR → ERROR(RPP_ERROR)). The run
  restarts at its beginning: RPP owns run boundaries, the mission does not cut a run.
* The journal is pre-loaded with the `completed_points` (`PointJournal::restore_resolved`): it continues at point `k` and the
  restored points are never re-issued, even when the restarted run drives over them again. A record that does not fit the
  execution (more points than its must-hit vertices) is ERROR(INTERNAL_ERROR) at PLACING (it cannot come from this node).
* The new execution writes its own records over the old one from PLACING on, with the restored points first.
* A point of an earlier run that was never resolved (possible only for a capture in progress at a run boundary) is resolved
  as usual by the journal (captured again, or FAILED when bypassed).

Without a usable record the start is a fresh one (`resumed_run_index = 0`). `resume = false` starts at run 0 and the record is
overwritten at PLACING (mission id, run 0, no points).

## 10. Path artifact
C++ reader + SHA-256 (`path_artifact.cpp`, `sha256.cpp`); must refuse everything the Python `decode()` refuses
(`docs/contracts/path_artifact.md`) and compute the same hash; Python `repr(float)` coordinates and canonical JSON meta;
regular file ≤ `kMaxArtifactBytes` (64 MiB), size checked before reading. The C++ writer (`serialize_artifact`) reproduces the
Python writer byte for byte. A source that fails is `ERROR(PATH_ERROR)` (the start itself was accepted, section 1).

## 11. Parameters (classes per spec §9)
| name | default | class | validated | source |
|---|---|---|---|---|
| `missions_dir` | `/var/lib/dyx3/missions` | RESTART | | spec §12 filesystem (RPP's `artifact_dir` must be the same directory) |
| `state_publish_hz` | 10 | IDLE_ONLY | (0, 100] | DERIVED |
| `gate_status_max_age_s` | 0.5 | IDLE_ONLY | > 0 | prototype freshness convention — DERIVED |
| `point_capture_radius_m` | 0.10 | IDLE_ONLY | > 0 | prototype `point_hold_acceptance_m` — DERIVED |
| `rpp_status_max_age_s` | 0.5 | IDLE_ONLY | > 0 | RPP/guard freshness convention — DERIVED |
| `rpp_ack_timeout_s` | 30 | IDLE_ONLY | > 0 | **no source** — conservative DERIVED; must exceed the largest mission's RPP conditioning time |
| `arm_timeout_s` | 4.0 | IDLE_ONLY | > 2.0, ≤ 60 | > px4_link `arm_confirm_timeout_s` (section 5) |
| `offboard_timeout_s` | 5.0 | IDLE_ONLY | > 3.5, ≤ 60 | > px4_link prestream + confirm + margin (section 5) |
| `placement_max_distance_m` | 1000 | IDLE_ONLY | (0, 1000] | the placement accuracy statement (section 6) |
| `vehicle_state_max_age_s` | 0.5 | IDLE_ONLY | > 0 | freshness convention — DERIVED |

Every value is finite. Invalid values are refused at construction (the node does not start) and at runtime. The IDLE_ONLY
values are applied as one validated atomic batch only while the FSM is IDLE; a rejected request changes nothing. Terminal
states are not IDLE: to reconfigure after a mission, leave the vehicle stopped and disarmed and restart the node.

## 12. Open questions
The numeric value of `rpp_ack_timeout_s`; mission-id persistence across reboot (per-process counter, so a `request_id` is
only deduplicated within one process lifetime). 0.17.0: the bound of the re-engage's full-gate hold (`offboard_timeout_s`
reused, section 3a); whether a pivot-timeout pause should latch per expiry (section 3b) or per level once RPP resets its
watchdog on resume; retention of COMPLETED progress records (kept as evidence, one per artifact); whether a resume should
also be offered for a different source artifact covering the same drawing (today a re-planned path is a new artifact and a
fresh start); whether the services/control unit split (review P2 item 3) lands before field use of resume.
