# Proposal — split `dyx3-ros` into a control unit and a services unit

**Status:** IMPLEMENTED in the repository (2026-10-10), **not yet run on the rover**. Owner acceptance pending.
**Source:** `docs/reviews/2026-10-10_production_blockers_transport_timing_review.md`, P2 fix part 3.
**Supersedes:** architecture §12, where a single `dyx3-ros.service` runs "the production control graph". The architecture
document itself is not edited (CLAUDE.md §4); this proposal records the change.
`DERIVED — NOT FROM V1 SPEC` throughout: V1 names one graph unit and gives no restart policy for it.

## 1. What changed

| Before | After |
|---|---|
| `dyx3-ros.service` → `start-ros.sh` → `control_graph.launch.py`: mission, motion_guard, px4_link, rpp, spray, system_gateway | `dyx3-control.service` → `start-control.sh` → `control_graph.launch.py`: **px4_link, motion_guard, rpp** |
| | `dyx3-services.service` → `start-services.sh` → `services_graph.launch.py`: **mission, spray, system_gateway** |
| `on_exit=Shutdown` on all six: any node exiting stops all six | `on_exit=Shutdown` **within each unit**: a node exiting stops its own unit only |

- **One node table.** `control_graph.launch.py` keeps `GRAPH` (all six rows) and `UNIT_PACKAGES` (which unit starts which
  package). Each launch file starts only its own unit's rows, so no node can be in both launches or in neither.
  `services_graph.launch.py` loads its rows from that table. The table stays in `control_graph.launch.py` because the
  `dyx3_recorder` REC-002 test parses `GRAPH` from that file.
- **Scheduling unchanged.** rpp and motion_guard `taskset -c 4 chrt -f 80`, px4_link `taskset -c 4 chrt -f 70`, all in
  `dyx3-control`, which keeps `LimitRTPRIO=99` and `LimitMEMLOCK=infinity`. Mission, spray and the gateway run under normal
  scheduling, so `dyx3-services` has neither limit.
- **Unit dependencies.**
  - `dyx3-control`: `After=`/`Requires=dyx3-platform`, as `dyx3-ros` had.
  - `dyx3-services`: `After=`/`Wants=dyx3-control`. Never `Requires=`, `BindsTo=` or `PartOf=`: the control unit must keep
    running when the services unit stops or restarts.
  - `dyx3-recorder`: `After=`/`Wants=dyx3-control`. `dyx3-backend`: `After=`/`Wants=dyx3-services`, whose gateway socket
    it talks to.
  - `dyx3-spray-watchdog`: tied to neither unit.
- **Both units keep** `Restart=always`, `RestartSec=5`, `TimeoutStopSec=15` (with its DERIVED note), `User=dyx3`,
  `EnvironmentFile=-/etc/dyx3/ros.env` and the same hardening.
- **XRCE-agent wait.** The bounded wait in the old `start-ros.sh` is now in `start-control.sh` only, because px4_link is the
  one node that talks to the agent.
- **Installer.**
  - `[enabled_services]` order (= restart order): usb-serial-check, platform, control, services, rtk, spray-watchdog,
    recorder, backend.
  - `install_units` installs only units a release both ships and lists in `[services]`. It stops, disables, `reset-failed`s
    and deletes every other `dyx3-*` unit, along with its `*.wants/` links, even when only a dangling link is left
    (INS-011, now idempotent). The upgrade therefore removes `dyx3-ros.service`, and a rollback across the split removes
    the two new units and brings `dyx3-ros` back.
  - `stop_enabled_services` stops units in reverse order: consumers first.
  - Health gates the gateway check on `dyx3-services`, the px4_link sample on `dyx3-control`, and lists each unit's nodes
    only when that unit is enabled.
  - The idle check treats any running `dyx3-control`, `dyx3-services` or (pre-split) `dyx3-ros` as a running graph. A
    pre-split `dyx3-ros` in a manifest counts as both units, so the health baseline of the running old release and a
    rollback target keep their gateway and graph checks.

## 2. Why: the measured P2 mechanism

`control_graph.launch.py:65` put `on_exit=Shutdown` on every one of the six nodes. When any node exited:

1. `ros2 launch` shut the whole graph down;
2. px4_link sent its 0.3 s STOP burst and exited;
3. PX4's offboard loss (0.5 s) disarmed the rover mid-line.

A crash in non-safety code (the gateway, spray, mission) therefore ended the run exactly like a crash in the guard. GW-002
(gateway SIGPIPE) followed this path in the field before it was fixed. The painted line is the product: a mid-line disarm,
then a restart from point 0, leaves a visible defect.

The guard already fails to zero when `/dyx3/mission/state` goes stale (`mission_state_max_age_s` = 0.5 s,
`REASON_MISSION_GATE`). Once px4_link no longer dies with the services, a services crash becomes a PAUSED-like stop instead
of a disarm.

## 3. Failure matrix (what the rover does)

Expected behaviour from the unit files, the launch files and the node contracts. **None of it has been measured on the
rover yet** (§5).

| What dies | Unit restarted by systemd | Motion | PX4 | Spray | Tablet | Recovery |
|---|---|---|---|---|---|---|
| mission, spray or gateway (one node) | `dyx3-services` only (all three nodes restart) | motion_guard sees `mission/state` stale after 0.5 s → `REASON_MISSION_GATE`, speed 0 / yaw rate 0; rpp keeps running and its output is gated | px4_link keeps the offboard heartbeat with explicit STOP; **OFFBOARD stays on, armed. Nothing disarms.** | spray controller gone; `dyx3-spray-watchdog` keeps requesting OFF through px4_link, which is still up | backend loses the gateway socket: no telemetry, no tablet E-stop until the unit is back. The E-stop latch is in motion_guard (control) and survives. The RC kill always works | after about 5 s + start-up the mission node is **IDLE** (progress was in memory, MS-002) and rpp unloads (STOP). The operator restarts (or, once P2 parts 1–2 land, resumes) from the tablet |
| px4_link, motion_guard or rpp | `dyx3-control` only. `dyx3-services` keeps running (`Wants=` does not propagate a stop) | the chain is gone | px4_link STOP burst (if it can still send), then offboard loss 0.5 s → **disarm**, as before the split | watchdog requests OFF; px4_link is gone, so the request has nowhere to go until it is back, same as before | gateway up: the tablet sees the rover stop and px4_link go stale | mission sees the vehicle leave OFFBOARD / disarm and handles it as today (abort / error path). The operator starts again |
| `dyx3-platform` (XRCE agent supervisor) | `dyx3-platform` and, through `Requires=`, `dyx3-control`; `dyx3-services` is not restarted | as the row above | offboard loss → disarm | as the row above | as the row above | as the row above |
| `dyx3-backend` | backend only | none: the backend is not a motion gate (owner decision 2026-10-10) | unchanged | unchanged | reconnects | none needed |
| `dyx3-recorder`, `dyx3-rtk`, `dyx3-spray-watchdog` | their own unit only | unchanged; RTK loss reaches the guard's RTK gate as before | unchanged | watchdog restart: `RestartSec=2` | unchanged | unchanged |

## 4. New state the operator must know about: armed and idle

After a services restart during a run, PX4 is still **armed in OFFBOARD**, holding STOP, while the new mission node is
IDLE and owns nothing. Before the split this state could not happen, because the disarm always followed. As far as the
code read here shows:

- The mission's release (COMPLETED / ABORTED / ERROR → OFFBOARD off, disarm) only disarms what that mission instance
  armed (`Px4Sequencer::arm_owned()`). A fresh IDLE mission does not disarm a vehicle it did not arm.
- The installer refuses an upgrade or rollback while the gateway reports ARMED, and while `dyx3-control` runs with no
  answering gateway ("unknown"). That is correct.
- A new start from the tablet runs LOADING → PLACING → ARMING → ENGAGING with the vehicle already armed and in OFFBOARD.
  Whether PX4 v1.17 ACKs an arm request for an already-armed vehicle as ACCEPTED, and whether ENGAGING treats "already
  OFFBOARD" as success, is **not verified**.

Open for the owner: should the mission adopt an armed, OFFBOARD vehicle it finds at start-up (disarm it, or offer
re-engage), or is disarming by RC/QGC the procedure? This needs a mission change, which is out of scope here.

## 5. What is not verified

- Nothing in this proposal has run on a rover. systemd ordering, `Wants=` versus `Requires=` propagation, the 0.5 s
  mission-gate stop under a real services crash, PX4 staying armed in OFFBOARD through a services restart, and the backend
  reconnecting to the gateway after the restart all need the bench procedure.
- `docs/bench/fault_injection.md` still describes the single `dyx3-ros` unit and expects every node kill to restart the
  whole graph. It must be rewritten for two units before the bench run: a kill of mission, spray or gateway must restart
  `dyx3-services` only, leave `dyx3-control`'s `NRestarts` unchanged, and keep PX4 armed in OFFBOARD.
- Tested off-target:
  - `installer/tests/run_tests.sh`, staged root with a recorded `systemctl`: installed and enabled units, restart order,
    removal of the stale `dyx3-ros` unit (idempotent, including a dangling link), rollback across the split, health gating.
  - `dyx3_bringup` launch tests: node sets, prefixes, `on_exit` per unit, no node in both launches.

## 6. Follow-ups outside this change

- `docs/bench/fault_injection.md`: two units (above).
- `tools/bench/offboard_sign_test.py`: says "stop dyx3-ros". It must stop `dyx3-control` (the only unit that writes
  `/fmu/in`) and, because mission talks to px4_link, `dyx3-services` too.
- `deployment/network/ros.env.tmpl` line 1: the comment lists `dyx3-ros` among the readers of `ros.env`.
- Architecture §12 diagram: replace `dyx3-ros.service` with the two units when the owner accepts this proposal.
- Review P2 parts 1–2 (persisted journal / `resume_from_point`, the PAUSED → ARMING → ENGAGING re-engage path). They turn
  "operator restarts from point 0" into "operator resumes".
