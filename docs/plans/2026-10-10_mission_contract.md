# Mission contract v2: tablet trajectory → rover drives it (2026-10-10)

Owner direction (2026-10-10):
- The **frontend** parses the file (DXF/CSV/template) and **creates the trajectory**.
- The **rover drives exactly that trajectory**: the backend and controller honor it and never re-plan it.
- Reuse the existing endpoints, improve the lifecycle, harden the transport, and make it fast.
- The operator does only: **upload → preview → start → pause/resume → stop/abort**. No arming, no mode switching for
  missions. That matches the prototype `3WD_Proto/PX4_DXP` (`server/offboard_controller.py`), without its weaknesses.

The app (`Three_Wheel_v2`) is **out of scope** for this round. Its integration follows the contract written here.

## 1. Division of responsibility

| Layer | Owns | Must not |
|---|---|---|
| Frontend (tablet) | file parsing, trajectory geometry, run split (mark/travel), must-hit flags, the geodetic anchor | assume the rover's EKF origin |
| Backend (FastAPI) | admission, **lossless** normalisation (§2), content-addressed storage, preview, lifecycle commands → gateway | change geometry beyond §2 |
| Gateway | one IPC hop, no policy, typed results, fast event push | block on long ROS calls |
| `dyx3_mission` | the **mission lifecycle** (§3): gates, frame placement, arm, OFFBOARD, RPP hand-off, terminal handling | let motion happen without every gate |
| `px4_link` | the arm and OFFBOARD mechanics, with confirmation (already exists: prestream, mode command, confirm) | decide policy |
| RPP / guard | tracking / last-hop safety (unchanged) | — |

## 2. Trajectory admission: "honor the frontend"

Endpoint reused: `POST /api/missions/plan` (`backend/src/dyx3_backend/mission/app_plan.py`), version-tolerant.

**Geometry is never re-planned.** The backend may only apply changes that cannot alter the path or that are
explicitly reported:
1. **Lossless densify:** a step longer than `MAX_STEP_M` (5 m) is split into collinear points. The geometry is
   identical; this replaces today's `STEP_TOO_LARGE` rejection (travel legs from the app are 2-point jumps).
   Recorded in meta (`densified_steps`).
2. **Boundary snap:** a run whose first point is within **10 mm** of the previous run's end is snapped to it. The
   10 mm is the app's own join tolerance; today the backend rejects anything over 1 mm. Snaps over 10 mm are still
   rejected. The maximum snap is recorded in meta (`max_boundary_snap_m`).
3. Everything else unchanged: point order, flags, run types, must-hit provenance, and R1–R4.
4. **Geodetic anchor (frame, safety-critical):** the payload gains `anchor: {lat, lon, alt?}`. It is the WGS84
   position of the trajectory's local origin (`origin_ne_m` stays as the local offset).
   - The anchor is stored in the artifact meta.
   - **A payload without an anchor is accepted only with `frame: "ekf_local_ned"`**, i.e. an explicit statement
     that the points are already in the rover's EKF frame (bench and debug use).
   - Points relative to an app GPS origin can never be driven as if they were EKF-local.
5. The response is unchanged in shape: `201 {ok, mission:{sha256, …}}`. It adds `normalisation: {densified_steps,
   max_boundary_snap_m}`, so the frontend can show "accepted as drawn".
6. Preview: `GET /api/missions/{sha}/path` returns the stored geometry (local NE + anchor). Bit-exact: what is
   previewed is what is driven, before placement.

## 3. Mission lifecycle (rover, `dyx3_mission`)

Endpoint reused: `POST /api/missions/{sha}/start` → gateway `start_mission` → `/dyx3/mission/start`.
**Start becomes asynchronous:** the service accepts and returns immediately with the execution id; progress is
pushed as MissionState events. No call holds the gateway's 2 s timeout while files load or PX4 arms.

```
IDLE --start--> LOADING --artifact ok--> PLACING --frame ok--> ARMING --armed--> ENGAGING --OFFBOARD confirmed-->
READY --RPP ack--> RUNNING --complete--> COMPLETED --(STOP, leave OFFBOARD, disarm)--> (IDLE on next start)
any pre-RUNNING step fails --> ERROR(reason) + STOP + disarm-if-we-armed + execution unloaded
RUNNING --pause--> PAUSED (armed, OFFBOARD, STOP stream) --resume(gates ok)--> RUNNING
RUNNING|PAUSED|READY --abort--> ABORTED (STOP, disarm)
E-stop asserted (any state) --> ABORTED (STOP, disarm)   [owner: E-stop disarms]
```

**Gates before arming:** every motion-guard gate except "armed" and "OFFBOARD":
- E-stop clear;
- PX4 link healthy;
- operator link (Start only);
- RTK fixed;
- heading valid (`cs_gnss_yaw`);
- estimator healthy;
- EKF global reference valid.

The guard must publish this separately from its full gate (a new field), so the mission node does not re-implement
the gate.

**Placement (frame):** the anchor is converted to the EKF local NED with the live EKF reference (`VehicleState`
`ref_lat/ref_lon`, `xy_reset_counter`). The result is a **content-addressed execution artifact** (new sha), so RPP
loads it unchanged. Both shas are recorded in MissionState and by the recorder.
- PLACING fails if the EKF reference is invalid, or if the trajectory lies outside the envelope.
- An `xy_reset_counter` change while READY or RUNNING → PAUSED(reason `EKF_RESET`).

**Arming / OFFBOARD:** through the existing `/dyx3/px4_link/arm` and `set_offboard` services. They already
pre-stream, command and **confirm**. The step timeouts are explicit parameters, each longer than px4_link's own
confirmation time.

**Terminal handling:**
- COMPLETED and ABORTED: STOP stream → `set_offboard(false)` → `arm(false)`. Spray is already closed by its gates.
- ERROR before RUNNING: disarm only if this execution armed.

## 4. Transport hardening and speed

| Item | Today | v2 |
|---|---|---|
| Start latency | synchronous: artifact read + SHA inside the gateway's 2 s timeout | accepted in < 50 ms; progress by events |
| MissionState to tablet | 5 Hz gateway snapshot | **pushed on every change** (event), plus the 5 Hz snapshot |
| Gateway timeouts | one 2 s for all commands | per command: arm/OFFBOARD wait longer than px4_link's confirm; start, pause and abort are fast accepts |
| Idempotency | none | `start` carries a client request id; a duplicate returns the same execution and starts nothing new |
| Errors | mixed | typed `{ok:false, code, reason, data}` end to end; mission reason codes preserved to the tablet |
| Health | separate endpoints | `/api/health` includes the mission lifecycle state and the last error |

## 5. Work packages (agents)

| Agent | Scope (files) | Depends on |
|---|---|---|
| **A: Backend admission + API** | `backend/**` (`app_plan.py`, `service.py`, routes, settings), `docs/contracts/backend.md`, `path_artifact.md`. Implements §2, the async start response shape, idempotency key and typed errors (§4 rows 1, 4, 5, 6 backend side) | none |
| **B: Mission lifecycle** | `ros2_ws/src/dyx3_mission/**`, `dyx3_interfaces` (MissionState states/reasons, SafetyGateStatus pre-arm field, StartMission async; one interface release with CHANGELOG), `dyx3_motion_guard` (publish the pre-arm gate), `docs/contracts/dyx3_mission.md`. Implements §3 | none (A and C consume its interface) |
| **C: Gateway transport** | `ros2_ws/src/dyx3_system_gateway/**`, `backend/src/dyx3_backend/gateway/` + `realtime/` (relay of events), `docs/contracts/dyx3_system_gateway.md`. Implements §4: event push of MissionState, per-command timeouts, request ids | B's interface: it codes against the field names agreed in this plan, then rebuilds after B merges |

**Ordering:** A and B in parallel; C in parallel on everything that does not need B's new fields; final integration
after B merges.

**Acceptance (all local first, then one push):**
- full workspace build and tests;
- backend pytest;
- installer tests;
- **bench:** start from the backend API with the rover disarmed → auto arm → OFFBOARD → drives a small straight+arc
  trajectory → complete → disarm.

Not in this round: the app; the operator-loss "observer" policy (needs the RC-kill or physical E-stop decision);
spray.
