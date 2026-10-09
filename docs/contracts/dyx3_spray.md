# dyx3_spray — contract

**Status:** draft for review, written before the implementation (CLAUDE.md §6: actuator + boundary semantics).
**Spec:** V1 §7.8, §3, Phase plan 9. **Evidence:** `PX4_DXP` `build/demo-ready` @ `fc6436b`:
`spray_fsm.py`, `spray_safety_lease.py`, `spray_safety_watchdog_node.py`, `spray_flow_model.py`, `spray_controller_node.py`
(2 792 lines), read-only. The painted line is the product: **where the valve opens is part of the accuracy spec.**

## 1. What is ported and what is not

| Ported (C++, equivalence-tested against the verbatim Python) | Not ported (listed so nobody assumes it) |
|---|---|
| `SpraySafetyStateMachine` (7 states, invariants 1 and 2, ack timeout, RECOVERY backoff) | **Dash mode** (`spray_modes.DashMeter`) and **point mode** (`PointMeter`) — the session-config schema that drives them |
| safety lease validation + freshness monitor; the **independent watchdog** as its own executable | the `/spray/active` heartbeat net (B4) and RPP-authored boundary source (G2); spray owns boundaries from the conditioned artifact |
| speed-proportional flow modulator | the legacy `allow_legacy_spray_active_fallback` / `use_distance_aware_spray` switch (distance-aware is the only path) |
| distance-aware continuous decision: path model, windowed projection **with the direction gate**, MARK-boundary lead, terminal shutoff, cross-track hysteresis | MAVROS (`command_service`): the valve is driven through `dyx3_px4_link` |
| gate stack: armed / OFFBOARD, pose + velocity freshness, RTK gate with recovery hold, tracking-seen (B5), fresh RPP heading evidence and cut, pivot (CORNER_ALIGN) gate, E-stop | |
| manual override with a hard expiry; debounce; ON re-assertion | |

48 of the 57 parameters assigned to spray in the updated registry are carried (`tools/gen_param_tables.py`); the 9 that belong to the unported features
are excluded with a reason in the generator. Two carried parameters do nothing in the prototype either and are kept only for
registry parity: `anticipatory_margin_m` (declared, never read) and `min_spray_speed_mps` (its gate was removed: spraying is a
question of WHERE the nozzle is, slow means thin flow, never off).

## 2. Authority and the three layers

1. **Controller** (`spray_node`) is the sole final spray verdict owner: conditioned MARK/TRANSIT geometry, RPP heading cut and entry hold, safety gates, and valve ON/OFF. It drives the FSM and never talks to the FCU. RPP publishes tracking evidence only; `spray_request` is deprecated diagnostics and is ignored.
2. **`dyx3_px4_link`** is the only package touching `/fmu/**`. It turns `SprayActuatorCommand` into
  `VEHICLE_CMD_DO_SET_ACTUATOR` (187; value in slot `actuator_set_index`, the other five NaN) or `DO_SET_SERVO` (183;
   `pwm_us` clamped to 2200) and maps `/fmu/out/vehicle_command_ack` back into `SprayActuatorAck`. It serializes valve
   transactions and correlates each ACK by command ID plus a unique, nonreused logical-proof
   `(source_system, source_component)` pair echoed into the ACK target fields by the pinned PX4 firmware.
   Exact physical reasserts continue on the wire with the same pair and do not allocate a new identity.
   Queue replacement, link loss, and timeout are acked **false** (`result 255`);
   late or unmatched FCU ACKs are discarded and cannot confirm a newer transaction. See the D1 details in
   `docs/contracts/dyx3_px4_link.md` section 14.
3. **Independent watchdog** (`spray_watchdog`, its own process and unit): sends OFF whenever the lease is absent,
   denied, malformed or stale, and publishes `SprayWatchdogStatus`. It must survive the controller dying. **Open hardware
   question (human):** if `dyx3_px4_link` or the FCU path dies while the valve is ON, nothing in this repository can close
   it; the FCU/hardware fail-safe (actuator timeout, relay default) must cover that and is firmware/hardware territory.

## 3. The state machine (carried verbatim in behaviour)

`OFF_UNCONFIRMED -> OFF_CONFIRMED -> ON_PENDING -> ON_CONFIRMED -> OFF_PENDING -> (RECOVERY | OFF_CONFIRMED | DISABLED)`.
Invariant 1: `spraying` is true only in `ON_CONFIRMED` — there is no optimistic-ON window. Invariant 2: every command
carries a monotonic `cmd_seq`; an ack applies only if its seq matches. ON is refused until a fresh OFF is confirmed
(re-enable always passes through `OFF_UNCONFIRMED`). A safety loss forces OFF on the *edge* and then falls silent once OFF
is confirmed (no 50 Hz flood while the rover sits disarmed). An ack that never arrives is treated as a failure after
`ack_timeout_s` (1.0): a pending ON becomes a fresh OFF, a pending OFF enters RECOVERY with backoff
`min(0.5 * 2^attempt, 5.0)`, reset by a safety-loss edge or `note_event_reset`.

## 4. Lease and watchdog

`SprayLease` (typed message; the prototype's JSON schema carried the same fields) is validated strictly: backend in
{actuator, servo_pwm}; `actuator_set_index` 1..6; `off_value` finite and in [-1, 1]; `servo_instance` 1..16;
`off_pwm_us` 0..2200. The controller publishes it every tick with
`allow_on = desired && safety_ok && enabled && fsm.commanded` (ON_PENDING included so a watchdog OFF cannot race a freshly
dispatched ON). An unknown backend never grants an ON lease. The watchdog's `off_reason(now)`:
no lease -> "no controller lease"; invalidated -> its reason; age > `lease_timeout_s` (0.35) -> stale; `allow_on=false` ->
"denies ON"; else ON is allowed. Receive-time freshness, never the sender's stamp. Watchdog OFF cadence: burst
20 Hz for 1.5 s after the OFF CAUSE changes (no lease / invalidated / stale / denied / shutdown), then 2 Hz. **DERIVED — NOT FROM
V1 SPEC:** the prototype compared the full reason text, and the stale text embeds the lease age, so while a lease stayed stale it
re-armed the 20 Hz burst on every tick forever; keying on the cause removes that (tested); an OFF whose ack takes longer than 1.0 s is retried; startup is fail-closed
before any lease; shutdown sends OFF.

### Watchdog OFF proof follows actuator mapping (D2)

`off_authority_ready` means that the current mapping's OFF command received a successful, matching
`SprayActuatorAck`. Mapping identity is `(backend, actuator_set_index, off_value, servo_instance,
off_pwm_us)`, the complete set of fields carried by the lease that can affect the commanded
destination/value. Lease permission and sequence are not part of physical identity; the lease does
not carry an ON value. An identical mapping update preserves proof. A changed mapping immediately
clears proof, invalidates any old mapping OFF in flight, makes `allow_on` false, and schedules an
OFF with the new mapping. Only the new OFF sequence can restore readiness; late ACKs for the prior
sequence are ignored. The existing px4_link single-inflight serialization and watchdog-OFF queue
priority ensure a queued controller ON cannot overtake the required OFF. An already dispatched
transaction completes or times out first, after which the new mapping OFF is sent.

## 5. Distance-aware decision (continuous mode)

Nozzle position = pose + body-frame offsets (forward, lateral right). The path model carries `cumulative_s` and MARK
boundaries from the flag changes; **a path that ends on MARK gets a synthetic terminal MARK->TRANSIT boundary**. Lead:
`on_lead = v * (open_delay + debounce_delay) + on_margin`, `off_lead = max(0, v * (close_delay + debounce_delay) - off_margin)`
with `debounce_delay = (max(1, debounce_samples) - 1) / tick_hz` (SP-002, see below); the valve opens early before
TRANSIT->MARK and closes early before MARK->TRANSIT. Terminal shutoff: forced OFF within `terminal_off_epsilon_m` of the final
station at speed <= `terminal_off_speed_mps` (the OFF lead is ~1 mm at creep speed, so the geometric boundary is never
crossed). Cross-track gate with hysteresis (trip at the wide level, clear at the tight one, and stay off at least
`xtrack_gate_min_off_s`); it is **load-bearing** and must not be loosened.

**Geometry source (C1).** RPP conditions the source `DYX3PATH` once and writes a content-addressed `DYX3COND 1` artifact. It
contains source SHA256, conditioner config, and exact ordered conditioned runs/points/flags. RPP publishes its SHA256 on
`RppStatus`; spray loads that artifact only when its source SHA matches the mission's `path_artifact_sha256`. The spray model
uses those coordinates, flags, and run boundaries directly. It never conditions independently and never falls back to raw
mission geometry. Missing, malformed, mismatched, or stale geometry leaves the path unloaded and autonomous spray OFF.

**Debounce latency is led (SP-002; DERIVED — NOT FROM V1 SPEC, the prototype did not compensate it).** `debounce_samples` (3)
means the debounced desire follows the raw one only after 3 identical ticks, so a geometric edge reaches the FSM
`(debounce_samples - 1)` ticks after the raw decision flips: 40 ms at 50 Hz, i.e. 1.4 cm at 0.35 m/s and 4 cm at 1 m/s, on the
OPEN and on the CLOSE. That delay is deterministic, so it is added to both valve delays in the lead above, using the node's real
control period (`1 / tick_hz`, not a constant 50 Hz). The debounce itself is kept for noise rejection. What remains is the
sampling of the crossing: the raw decision flips at the first tick past the lead point, so each edge is late by `[0, v / tick_hz)`
(< 0.7 cm at 0.35 m/s, < 1 cm at 0.5 m/s, **< 2 cm at 1 m/s and 50 Hz**; < 1 cm at 1 m/s needs a 100 Hz tick). Measured in
`spray_core_test` (`DebounceIsLedSoValveEdgesLandOnTheBoundaryAtProductionSpeeds`, every sample phase). Valve delays, nozzle
offset, pose latency and the link/FCU path are not in this figure (not provable off-target, SP-003). **Safety OFF never goes
through the debounce or the lead:** the FSM reads the safety verdict directly, so E-stop, disarm, watchdog, lease and ownership
refusals close at once (tested with a 10-sample debounce).

## 6. KNOWN-OPEN DEFECT — projection continuity (spec 7.8)

**Spray boundary gap — OPEN.** On a path that doubles back, the approach leg and the marked leg sit centimetres apart. The
nearest-segment search cannot separate them by distance, so `projection.s` can teleport between the legs and jump over a MARK
boundary: the valve then opens inside the mark. Prototype evidence (bags `stg_d8a4f2ad`, `stg_46ba8830`): the station jumped
4.45 -> 5.05 in one step straddling the boundary at 4.770, the valve opened 29.7 cm inside the mark; 4 of 18 runs lost 23-40 cm
with RTK fixed, safety_ok true and xtrack under 2.2 cm.

State of the code carried here: a spatial window around the previous station (`projection_window_back_m` 0.5,
`projection_window_fwd_m` 2.0, `projection_reacquire_dist_m` 1.0) **and a direction gate**
(`projection_direction_gate_deg`) that rejects segments running against the vehicle heading. The window alone does not fix
it (it is spatial); the direction gate is the only signal that separates coincident legs. **The gate's default is 0.0 = DISABLED**
(the prototype's A/B default), so the defect is open by default. This repository does not change that value: it is spray-boundary
semantics, hence a human decision with field evidence (CLAUDE.md §4).

`test/spray_defect_test.cpp` reproduces it with the verbatim algorithm on a synthetic out-and-back path (legs 2 cm apart, 3 mm
lateral noise): with the gate disabled the reported station teleports across the leg and the MARK flag flips at the wrong place
(the test asserts the defect is present — it documents it); with the gate enabled the station is continuous. **This is a
synthetic reproduction of the mechanism, not the field bags**; replaying `stg_d8a4f2ad` / `stg_46ba8830` is a LOCAL ACTION.
Also open: nozzle offset 1.6-6.6 cm (`nozzle_*_offset_m` are 0.0 defaults).

**Replay over mission geometry (decision input; `spray_projection_replay`).** The executable drives a synthetic rover ALONG the planned path of any
artifact (1 cm right of the line, 3 mm wobble, 7 mm steps, the shipped window) through the projection, gate off and on, and counts station teleports
(> 0.25 m in one sample) and samples whose projected MARK/TRANSIT flag disagrees with the plan (outside a 5 cm band around each boundary):

    spray_projection_replay --gates 0,45,60,90 <artifact.dyx3path | directory>...

Run on the four archived missions in Git (2026-10-07; planned geometry, **not** recorded traces; two of them are huge synthetic pitches, and the corpus is
almost all MARK, so it is a weak sample for the hard cases):

| mission | gate 0 (shipped) | gate 45 / 60 | gate 90 |
|---|---|---|---|
| `mission_straight_5m`, `square_2x2` | clean (the square: 1 teleport at the closed-loop seam, flag right) | clean | clean |
| `soccer_field_penalty_area` (457 m) | 1 teleport, **143 samples (~1.0 m) with a hole in the line** | 0 wrong flags (the seam teleport remains) | 0 wrong |
| `soccer_pitch_fifa_edited` (1207 m) | 9 teleports, **419 wrong samples: 184 spurious MARK (~1.3 m of paint where none is planned), 235 holes** | 4 teleports, 9 wrong samples | 9 teleports, 98 wrong |

Reading (not a decision): the gate removes almost all of the exposure on the two missions that double back; 45 and 60 degrees behave the same, 90 is visibly worse
(too loose to separate legs that run at right angles); a small residual remains at 45-60 on the pitch, and the closed-loop seam (end point = start point) teleports
whatever the gate. The decision and the angle stay with a human, on 5 to 10 missions from the field plus the bag traces. **LOCAL ACTION:** export the artifacts of
the field missions from the backend (`/var/lib/dyx3/missions/*.dyx3path`) and run the command above.

## 7. Gates (first failing wins; the reason string is published)

**Mission / RPP ownership (review C1, fix plan A1, human decision 2026-10-08), second in the order after E-stop.** Autonomous spray
may operate only while `MissionState` is RUNNING, `RppStatus` is fresh (`rpp_timeout_s`, 0.5 s, IDLE_ONLY, DERIVED: the stack's
0.5 s freshness convention = 25 missed ticks at 50 Hz; re-validate from Jetson jitter), reports the same `mission_id`, and is in
TRACKING, STOPPING (corner stop lays the last ~2 cm of the leg), PIVOTING (then the pivot gate below decides) or CREEPING
(DERIVED — NOT FROM V1 SPEC: endpoint creep is still on the leg; the geometry decides whether it is MARK). IDLE, LOADED,
COMPLETE, ERROR and unknown values refuse. Reasons: `mission not running`, `rpp stale`, `rpp mission mismatch`,
`rpp not marking`. Any refusal goes through the normal OFF path: OFF command at once (no debounce: the FSM reads the safety
verdict directly) and the lease's `allow_on` drops. The tracking evidence (B5) is no longer a permanent latch: it is set only by
TRACKING of the RUNNING mission and cleared by any non-RUNNING mission state, a mission id change, or a path load, so a resume
needs fresh TRACKING. Manual (bench) spray is exempt, as before (armed + watchdog suffice).

disarmed; not OFFBOARD (`require_offboard`); path not loaded; pose stale (`pose_timeout_s`); velocity stale
(`velocity_timeout_s`); RTK gate (below); awaiting tracking (B5: no `RppStatus` TRACKING since the path loaded, so a rover parked
on a spray-flagged vertex 0 cannot open the valve); pivoting in place (`RppStatus.state == PIVOTING`, CORNER_ALIGN only — never
CORNER_STOP, which still lays the last 2 cm of the leg; stale pivot state fails open immediately). DERIVED additions: **fresh RPP heading evidence must be valid and the heading cut refuses ON; E-stop asserted or
its state missing/stale (> 0.5 s) -> OFF** (consumers treat absence as asserted; first in the order), and the watchdog heartbeat
must be fresh and `off_authority_ready` (`spray_watchdog_required`). Also DERIVED: the vehicle state (armed / OFFBOARD / pose) is
only trusted while fresh (`pose_timeout_s`), so manual ON cannot ride a dead state stream; the prototype read a latched `/state`.

**Deliberate divergences from the prototype's gate evaluation (DERIVED, safer):**
* The RTK gate (and its recovery timer) is evaluated EVERY tick. The prototype evaluated it only after the earlier gates passed,
  so a fix drop that happened while e.g. the pose was stale never reset the recovery hold, and the gate reopened with no hold.
* Tracking evidence is `RppStatus.state == TRACKING` only. The prototype's segment states 1 and 2 (TRACK_SEGMENT,
  PRE_CORNER_SLOWDOWN) have no one-to-one mapping (our 2 is STOPPING, which is not tracking); fail closed: it can only delay the
  first mark, never advance it. Pivot gate: PIVOTING only (CORNER_ALIGN); STOPPING (CORNER_STOP) never gates.

RTK gate (carried from `rtk_quality.py`): `RtkStatus.corrections_fresh` is mandatory, including when
`spray_require_rtk_fix` is disabled. With the fix gate enabled, only fix types 5 and 6 at or above
`spray_min_fix_type` qualify; unknown accuracy (0) fails closed when `spray_require_accuracy`;
accuracy must be <= `spray_max_hrms_m`; RTK status sample age must be <= `gps_fix_timeout_s`;
**asymmetric hysteresis**: any loss of corrections, fix, required accuracy, or status freshness is
immediate, and re-enable requires `gps_recover_hold_s` of continuous good RTK input. FLOAT remains
accepted only when the configured minimum fix type permits it.

Heading evidence is part of the gate stack after mission/RPP ownership: it must be marked valid, finite, and received within `rpp_timeout_s`; stale or unavailable heading evidence refuses ON. The heading cut is evaluated from the current `RppStatus.heading_error_rad` and forces the normal immediate OFF safety path. Entry hold releases from that same current heading metric or RPP's `path_travel_m` progress evidence. STOPPING remains eligible for the final leg portion; PIVOTING remains blocked by the existing production pivot gate.

## 8. Interfaces

| Direction | Name | Type |
|---|---|---|
| in | `/dyx3/vehicle_state` | VehicleState (pose NED, heading, velocity, arming/nav state) |
| in | `/dyx3/rtk_status` | RtkStatus |
| in | `/dyx3/rpp/status` | RppStatus (fresh state, heading error/validity, run index, progress; `spray_request` ignored) |
| in | `/dyx3/mission/state` | MissionState (`path_artifact_sha256`: the flags come from the same artifact RPP loads) |
| in | `/dyx3/emergency_stop_state` | EmergencyStopState |
| in | `/dyx3/spray/watchdog_status` | SprayWatchdogStatus |
| in | `/dyx3/spray/actuator_ack` | SprayActuatorAck |
| service | `/dyx3/spray/set_manual` | SetSprayManual |
| out | `/dyx3/spray/actuator_command` | SprayActuatorCommand |
| out | `/dyx3/spray/lease` | SprayLease (reliable, depth 1) |
| out | `/dyx3/spray/state`, `/dyx3/spray/status` | SprayState, SprayStatus |

The watchdog subscribes `/dyx3/spray/lease` and `/dyx3/spray/actuator_ack` and publishes `/dyx3/spray/watchdog_status`
and `/dyx3/spray/actuator_command` (source = watchdog).

## 9. Parameters

45 carried parameters + 1 production addition (`rpp_timeout_s`) + 3 C2 heading-verdict parameters moved from RPP. The final verdict parameters are owned by `dyx3_spray` (`docs/tuning/parameter_registry.md`, classes as proposed there; defaults verbatim from the prototype and
**re-validated at GATE 5**). Two are `TBD — human` in the registry and are treated as follows until decided: `spray_enabled`
default true, **IDLE_ONLY**. The actuator value range is validated structurally (`off_value`, `on_value` in [-1, 1]; `min_flow_value`
within [off, on]); no tuning value is invented. Watchdog constants (`lease_timeout_s` 0.35, `off_retry_hz` 2, `off_burst_hz` 20,
`off_burst_duration_s` 1.5, `command_ack_timeout_s` 1.0) are the prototype's, RESTART, in the watchdog's own parameters.

## 9b. Node behaviour (not in the prototype's shape)

Control tick 50 Hz (`tick_hz`, DERIVED = the prototype's 20 ms timer; node-level, RESTART). The lease is published every tick
(reliable, depth 1; the watchdog's timeout is 0.35 s); `SprayStatus`/`SprayState` at 10 Hz and on every FSM or event change. An ON
already commanded is re-published to `dyx3_px4_link` at `reassert_hz` with the same `cmd_seq` (a heartbeat: no FSM transition).
The link sends each exact source/sequence/intent/actuator mapping/physical value reassert to PX4
with the same logical correlation pair. A late duplicate ACK remains a no-op in the FSM. Events and safety-loss edges are handled at the next tick (<= 20 ms), not inside the
subscription callbacks as the prototype did. The signal handler only raises a flag so the shutdown OFF can still be published while
the DDS context is up (rclcpp's own handler would kill the context first); both executables then flush for a bounded time.
Runtime parameter changes go through `ParamSet` (class rules, validation, journal); `use_sim_time` passes through; the two
node-level parameters (`tick_hz`, `artifact_dir`) are RESTART.

## 10. Acceptance

Off-target (all run, see HANDOFF): `spray_equivalence_test` compares the FSM (17 001 scripted events), lease validation (600) and
monitor (893), the flow modulator (1 831), `evaluate_rtk_quality` (2 500), the gate stack with the recovery hold (15 365 steps plus 400
single evaluations, the controller's own methods run unbound) and the continuous decision (12 900 ticks over the archived and
mixed-flag missions, synthetic out-and-backs, reverse/jumpy/noisy drives) against vectors from the VERBATIM prototype
(`tools/gate4/gen_spray_vectors.py`, sources pinned in `tools/gate4/spray_dxp/VERBATIM.sha256`). Every decision is exact; derived
distances agree to 1e-12 relative because CPython's `math.hypot` is not libm's. Mutation-checked (gate threshold, terminal epsilon,
recovery hold, projection tie-break, slew law, cross-track clear level: each is caught). `spray_defect_test` reproduces the open
defect. `spray_node_test` runs the controller and the independent watchdog against a fake FCU link. The vector generator needs the
path engine and rclpy (Humble container), so CI does not regenerate it: **LOCAL ACTION** to re-run `gen_spray_vectors.py --check`.
**Not provable off-target:** valve timing (`solenoid_*_delay_s`), nozzle offset, paint quality, the real FCU ack path.
