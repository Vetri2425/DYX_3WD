# F-tasks — Firmware patches, upstream issues, and the v1.16.2 → v1.17.0 carry-over audit

**Scope:** everything that must land in `Vetri2425/PX4-Autopilot-3WD-Prod` before the DYX 3WD
rover flies a production mission on Pixhawk 6X.
**Base:** PX4 `v1.17.0` == `d6f12ad1c4`
**Old firmware:** `Vetri2425/PX4-Autopilot` @ `main`, base v1.16.2, **39 own commits**
**Companion architecture:** `../architecture/DYX_3WD_Production_Stack_Architecture_V1.md`
**Date:** 2026-09-05

---

## 0. How to read this

| Mark | Meaning |
|---|---|
| ✅ **VERIFIED** | Checked against v1.17.0 source or the GitHub issue itself during authoring. |
| ⚠ **INFERRED** | Reasoned from adjacent evidence. Confirm before acting. |
| 🔴 **BLOCKER** | Must be closed or mitigated before the dependent stage can pass. |
| 🟡 **WATCH** | Real risk, not on the critical path yet. |
| ⚪ **N/A** | Investigated and ruled out. Recorded so nobody researches it twice. |

**Accuracy context.** The current rover is **sub-2 cm, not sub-cm**: arc 1.46 / lshape 0.90 /
square 0.87 / U-turn 1.06 cm @0.35 m/s; full-mission truth 1.69–1.91 cm; square is the only
real sub-1 cm baseline; circle never passed. Anything below is judged by whether it moves
those numbers.

---

# PART A — Upstream open issues

Searched `PX4/PX4-Autopilot`, open issues, 2026-09-05, across rover / differential / offboard /
uXRCE-DDS / EKF2 / RTCM / wheel-encoder / timesync.

## A1 · Blockers — must close or mitigate

### 🔴 A1.1 — #27514 · External modes: setpoints applied ~900 ms after the external process dies
`risk:safety-critical`, `scope:offboard`, `scope:commander` · opened 2026-05-29 · **open, 5 comments** ✅

> *"If an external mode stops publishing … PX4's controllers have no setpoint-freshness check or
> timeout. They keep applying the last received setpoint."*

**Why this is the most important issue in this document.** Our entire safety model assumes
fail-to-zero. At 0.35 m/s, 900 ms of stale setpoint is **31 cm of uncommanded travel**; at the
SPD-T1 target of 1.0 m/s it is **90 cm**. A marking rover that keeps painting for 900 ms after
the companion dies is both an accuracy defect and a safety defect.

**Our exposure is worse than the reporter's**, because we are moving *from* MAVROS velocity
offboard *to* direct rover setpoints — the same class of stale-setpoint window applies to
`rover_speed_setpoint` / `rover_rate_setpoint`.

**Mitigation (do not wait for upstream):**
1. `dyx3_motion_guard` enforces freshness on our side of the link and commands explicit zero.
2. `dyx3_px4_link` streams a heartbeat and stops publishing setpoints — not just stops
   *updating* them — the instant RPP goes stale.
3. **F5 must include a measured test:** kill the companion process mid-drive and record the
   distance travelled before motion stops. That number goes in the gate table.
4. Track the upstream discussion; a firmware-side freshness timeout is the real fix and is worth
   contributing to.

### 🔴 A1.2 — #27497 · Rover Differential doesn't turn in Mission Mode (v1.17 Stable)
`vehicle:rover`, `kind:bug` · opened 2026-05-28 · **open, 9 comments** ✅

> *"The rover moves back and forth, trying to reach the first waypoint without success … tries
> to make a turn, fails to do so, and ends up driving back and forth over the same spot."*
> Manual / Stabilized / Position modes drive fine. Reporter used a **custom `rover.px4board`**.

**Directly blocks Track F4** (mission-mode test and basic tune), which is our clean read on
firmware health with no companion code in the path. If F4 reproduces this, F4's gate cannot be
evaluated as written.

**Action:** attempt reproduction early — it is cheap, needs no companion stack, and the
reporter's configuration (custom rover board target, differential) is close to ours. If it
reproduces, either F4's gate changes to Position-mode validation, or we debug it — and a fix
here is a strong upstream contribution.

### 🔴 A1.3 — #27388 · `uxrce_dds_client` stops publishing after random uptime, needs FC reboot
`kind:bug`, `scope:middleware`, `stale` · opened 2026-05-19, updated 2026-08-20 · **open, 6 comments** ✅

Topics silently stop delivering; `ros2 topic list` still shows them, `echo`/`hz` show nothing.
Restarting the agent does not recover it — **only a flight-controller reboot does.**

**This is an existential risk to a DDS-only control path.** It is exactly the failure our
architecture has no answer for: the link looks alive and carries nothing.

**Mitigation:**
- `dyx3_px4_link` must detect **data staleness per topic**, not just session liveness — the
  bug's whole signature is a live session with dead topics.
- Staleness ⇒ `dyx3_motion_guard` fail-to-zero ⇒ mission pause, surfaced to the operator.
- **F3's gate must include a soak test**, hours not minutes, logging per-topic rates. This is
  the single best reason to run F3 as a long-duration bench service before trusting it.
- The `stale` label means it may be auto-closed without a fix. Do not read that as resolved.

### 🔴 A1.4 — #28519 · Slow timesync convergence ⇒ large uXRCE-DDS timestamp offset at boot
`status:needs-triage` · opened **2026-09-03** (two days old) · **open** ✅

> *"reported timestamp is off by approximately 40 ms, roughly 10× the expected offset … it takes
> approximately **5 to 10 minutes** after boot for the offset to converge."*

**Direct accuracy impact.** 40 ms of timestamp error at 0.35 m/s is **1.4 cm of position error**
attributed to the wrong instant — the same order as our entire tracking budget. It also
corrupts every bag↔ulog correlation, which is the measurement apparatus for every gate in both
tracks.

**Mitigation:**
- **A boot-time warm-up is now a mission precondition.** The rover must not accept a mission
  until timesync has converged. `dyx3-health` reports timesync offset; the mission gate consumes it.
- The recorder logs the offset at mission start and end (§7.9 of the architecture).
- This reinforces architecture §4.6: the analysis toolchain re-base is Stage 0 work.

### 🔴 A1.5 — #27860 · uXRCE-DDS client does not retry when the agent is absent at boot
`kind:bug` · opened 2026-07-10 · **open** ✅

Client comes up before agent (serial transport) and **stays disconnected forever**.

**This is our exact boot order.** `dyx3-platform` starts the agent; the FCU may well boot first.
A rover that silently never connects after a power cycle is a field-stopper.

**Mitigation:** `dyx3-platform` must start the agent **before** FCU power-up where possible, and
`dyx3-health` must treat "no DDS session" as a hard red. Consider an FCU-side retry patch — this
is a small, well-scoped upstream contribution.

## A2 · Watch list

| # | Title | Why it matters | Status |
|---|---|---|---|
| 🟡 #27685 | Rover goes into HOLD immediately after starting mission (`vehicle:rover`, `scope:commander`) | Same F4 surface as A1.2; differential rover, manual fine, position/mission fail. May be the same root cause. | open, 2026-07-14 ✅ |
| 🟡 #28108 | Rover failsafes into **Altitude** mode from Position (`vehicle:rover`, `scope:commander`) | Altitude is not a valid rover mode — behaviour undefined. Our failsafe design must not assume a sane fallback. | open, 2026-08-01 ✅ |
| 🟡 #27357 | commander/ModeManagement: stale per-`source_id` `config_control_setpoints` cache | Touches external-mode registration, which our DDS control path depends on. | open ✅ |
| 🟡 #28092 | uXRCE-DDS namespace mismatch SITL vs hardware (`-n` vs `UXRCE_DDS_NS_IDX`) | Bites when SITL/replay validation moves to hardware. Relevant to Stage 0.2 tooling. | open, 2026-08-13 ✅ |
| 🟡 #24482 | No uXRCE-DDS comms Pixhawk 6C ↔ companion via FTDI | Serial-transport reliability on 6-series. Directly relevant to the **F3/F5 TELEM2 serial** phase. | open ✅ |

## A3 · Investigated and ruled out

| # | Title | Verdict |
|---|---|---|
| ⚪ #28507 | Sustained along-track velocity oscillation on long straight mission | Multicopter `FlightTaskAuto` setpoint generation. We drive OFFBOARD, never `FlightTaskAuto`. ✅ |
| ⚪ #27313 | ekf2 `_position_sensor_ref` plumbing unused | Gates only the **external-vision** position bias estimator. We fuse GNSS + WENC, no EV. ✅ |
| ⚪ #28263, #27330, #27110, #27013 | EKF2 altitude / rangefinder / optical-flow / FW altitude | Altitude-domain. A ground rover's height source is irrelevant to planar tracking. ✅ |
| ⚪ #25956 | MavlinkReceiver UDP deadlock on NuttX under high traffic | Would matter if MAVLink carried control. It carries only params/QGC/logs. Keep MAVLink traffic low. ⚠ |
| ⚪ #27152 | GPS not detected 1.15.0 → 1.16.1 | Predates our base; UM982 is detected on the current stack. Re-check after the 6X swap. ⚠ |

> **No open upstream issue was found for:** dual-antenna GNSS heading accuracy, EKF2 wheel-encoder
> fusion, or RTCM injection over DDS. Not because they are healthy — because **PX4 upstream has no
> wheel-encoder aid source and no GNSS-yaw noise parameter at all.** Those are entirely our code,
> so their bugs are ours to find. See Part B.

---

# PART B — The 39 fork commits: carry-over audit

`git log --author='Vetri|DYX' main` on `Vetri2425/PX4-Autopilot` = **39 commits**.
Verdict for each below. **Nothing is carried by `cp`.** Every row that survives is re-anchored by
diffing the fork file against its **v1.16.2 stock ancestor** and re-applying that semantic diff
onto v1.17.0.

## B1 · MUST CARRY — accuracy- or function-critical

| # | Commit | What it does | v1.17 status |
|---|---|---|---|
| 1 | `5b6461692e` | **EKF2 wheel-encoder body-frame velocity fusion** — the whole aid source | ✅ **Not upstream.** v1.17 `aid_sources/` has no `wheel_encoder`. 13 files. |
| 2 | `255453d967` | **IMU lever-arm correction in WENC fusion** — corrects `ω × imu_pos_body` before fusion | ✅ Not upstream. **Field-verified: pivot wobble 1.52 → 0.50 cm median, net walk 2.02 → 0.83 cm, n=44/105, 34 ulogs.** The single largest accuracy patch we own. |
| 3 | `015c67484e` + `a6e9e12e2a` | **`EKF2_GPS_YAW_N` + `EKF2_GPS_YAW_G`** — decouple GNSS-yaw gain from the outlier gate | ✅ Not upstream — v1.17 `params_gnss.yaml` has only `EKF2_GPS_YAW_OFF`. Replay-verified, never flashed. |
| 4 | `1d82e616f8` | **RoboClaw: timestamp encoder read before the UART transactions** | ✅ Not upstream. Closes ~264 ms unbounded jitter into the WENC fusion buffer. Meaningless without #1. |
| 5 | `7a229fe997` | **RoboClaw QPPS velocity drive** — closed-loop on encoders | ✅ **Confirmed still needed.** v1.17 `setMotorSpeed()` still sends `DriveForwardMotor1` via `sendUnsigned7Bit` — open-loop 7-bit. Fork sends `sendSigned32Bit(DriveSpeedMotor1, qpps)`. Upstream diff v1.16.2→v1.17 for this driver is **4 insertions / 10 deletions** — essentially untouched. |
| 6 | `129cfc40ae` | RoboClaw: raw mode + `select()` `fd_set` reuse bug | ✅ v1.17 still has the same `FD_ZERO`/`FD_SET`-once-then-`select()`-in-loop pattern (lines 139–140, 355, 405). Bug intact. |
| 7 | `09bcec636f` | RoboClaw: centre output range 256 → 255, kills armed-zero creep | ⚠ Re-verify against v1.17's mapping. Creep at zero command is a stopping-accuracy defect — directly on our 1 cm stop budget. |
| 8 | `57f6b772fc` / `60218dfe81` | RoboClaw: deadband zero command at high `QPPS_MAX` | ⚠ **Two commits, near-identical titles — reconcile before carrying.** Determine whether one supersedes the other; carry one. |
| 9 | `06309e41a7` | Logger: WENC fusion debug topics on rover | ✅ Prerequisite for verifying #1/#2 in the field. |
| 10 | `9641ea9a99` | Logger: stop silently dropping WENC aid-source topics from a cold boot | ✅ Without it, GATE 2 evidence does not exist. |
| 11 | `53940ead83` | Logger: pin WENC aid-source instance count (was reserving 7 dead slots) | ✅ |
| 12 | `d90864a374` | WENC velocity-fusion **design spec** (docs) | Carry as `docs/` — it is the contract for #1 and the reference for re-anchoring. |

**B1 is the critical path of F1.** Items 1–4 are the accuracy patches; 5–8 are the drivetrain;
9–11 are the instrumentation without which none of it can be verified.

## B2 · RE-EVALUATE — upstream moved, do not re-apply blind

| # | Commit | Issue | What changed in v1.17 |
|---|---|---|---|
| 13 | `ceaf49e697` + `98952babd3` | RoverLandDetector returns `true` unconditionally | ✅ v1.17 `_get_landed_state()` **evolved** — it now has waypoint-distance logic (`distance_to_curr_wp < NAV_ACC_RAD && !next.valid`) and still ends `return !_armed;`. The blunt `return true` may now defeat upstream behaviour we want. **Review, don't paste.** |
| 14 | `d703b202d4` + `ecf1d7b519` | navigator `mission_block`: rover waypoint-acceptance bypass | ✅ v1.17 `mission_block.cpp` **now has `VEHICLE_TYPE_ROVER` handling at line 213**, which v1.16.2 lacked. The patched line (162) still exists. Determine whether upstream's rover branch already covers the case. |
| 15 | `1e2ce81a4a` | `rover_differential` OFFBOARD safety patches P1–P4 | ✅ **The v1.17 rover module is restructured**: `DifferentialSpeedControl`, `DifferentialRateControl`, `DifferentialAttControl`, `DifferentialActControl`, `DifferentialDriveModes/{Auto,Manual,Offboard}`. P1–P4 targeted a structure that no longer exists. **Re-derive the intent, discard the diff.** |
| 16 | `fa6f6bc983` | Bug 7 — reverse OFFBOARD velocity drove backward instead of spot-turning | ⚠ Likely **obsolete by architecture**: the bug is a symptom of inferring rotation from an XY velocity vector. Publishing `rover_rate_setpoint` directly removes the inference. **Confirm the failure cannot recur on the new interface, then drop.** |
| 17 | `58fe9b4698` + `de704a0aa9` | `RD_TANK_MODE` — two-paddle direct motor control | ⚠ Check `DifferentialManualMode` in v1.17 before re-adding. Manual-mode-only; no bearing on tracking accuracy. Low priority. |

**B2 is where a careless migration silently loses behaviour or silently fights upstream.** Each
row needs a decision recorded in `docs/contracts/`, not a merge.

## B3 · DO NOT CARRY

| Commits | Why |
|---|---|
| `24d78a8139`, `9dcb994802`, `a4d8f849b3`, `8cbb51d1fe`, `c7cc5860e6`, `ee0b76bfd5`, `30df715491` | CI plumbing for the **`cp`-overlay** build. That entire approach is abolished (see below). |
| `210fcc22b6`, `f9f7f9c7e8`, `05a2718acd`, `96cb7a58bd` | CubeOrangePlus `rover.px4board` creation and fixes. ✅ v1.17 ships a **stock `boards/px4/fmu-v6x/rover.px4board`** with `CONFIG_DRIVERS_ROBOCLAW=y`. Re-derive for v6x from the stock target; do not port the CubeOrange board file. |
| `62619611d6` | "swap IK signs to match **Sabertooth** wiring" — ⚠ different motor controller. The rover runs RoboClaw. **Confirm no residual sign dependency, then drop.** |
| `bfe914ceeb` + `617cce5a52` | A fix and its own revert. Net zero. Carry neither. |
| `d62f42fed0`, `b4c1307415` | `CLAUDE.md` docs for the old repo. Superseded by the new repo's own. |
| `24d78a8139` (also noted above) | "drop incompatible DifferentialPosControl from v1.16.2 overlay" — ✅ **the constraint is gone**: `RoverSpeedSetpoint.msg`/`RoverSpeedStatus.msg` exist in v1.17. Whether to overlay PosControl is now an open design choice, not a compile constraint. |

## B4 · Summary

| Verdict | Count |
|---|---|
| **B1 — must carry** | 13 commits → ~16 files |
| **B2 — re-evaluate** | 7 commits |
| **B3 — do not carry** | 19 commits |
| **Total** | 39 |

> **Roughly half the old fork does not survive the upgrade** — most of it CI scaffolding for a
> build method we are abolishing, and board files superseded by a stock upstream target. The
> irreplaceable core is **13 commits**, and 4 of them (WENC fusion, lever arm, GNSS-yaw floor,
> RoboClaw timestamp) carry essentially all of the accuracy value.

## B5 · ⛔ The `cp`-overlay pattern does not come with us

The old CI copied 27 fork files onto a clean v1.16.2 checkout **without committing**. Costs
already paid:

- **`ver_sw` can never identify a build.** Every build records base hash `54f0455f`. A flash is
  undetectable from firmware version or FCU parameters — the artifact directory name is the only
  discriminator, and those directories have already been lost from disk once.
- **`git log` / `git blame` lie about every overlaid file.** This produced a confident-but-wrong
  "`DifferentialVelControl` isn't compiled" audit of a module that was flying.
- Re-anchoring is invisible: nothing records *which* v1.16.2 ancestor a file was diffed from.

**In the new repo, every patch is a real commit on `dyx-3wd-production`,** and the build carries
an overlay/rebase hash that `dyx3-version` prints.

---

# PART C — New patches, not in the old fork

| # | Patch | Size | Rationale |
|---|---|---|---|
| C1 | `rover.px4board`: add `CONFIG_MODULES_UXRCE_DDS_CLIENT=y` | **1 line** | ✅ Absent from **every** stock `rover.px4board` (both `cubeorangeplus` and `fmu-v6x`), though present in `default.px4board`. Without it there is no DDS on a rover build. Also enables proving DDS on the **current CubeOrange+** over TELEM2 serial before any hardware change. |
| C2 | `dds_topics.yaml`: add `/fmu/in/gps_inject_data` | **2 lines** | ✅ `GpsInjectData.msg` exists; `gps_inject_data` is **already subscribed by `src/drivers/gps/gps.cpp`** — consumer side needs no change. `dds_topics.yaml` is build-time codegen via `generate_dds_topics.py`. Puts RTK on the data plane. |
| C3 | `dds_topics.yaml`: add `ulog_stream` + `ulog_stream_ack` | **4 lines** | ✅ Both msgs exist. Streams the flight log into the run directory as the run happens — provenance obtained structurally instead of by procedure. **Optional; high value for `dyx3_recorder`.** |
| C4 | Board target for `px4_fmu-v6x_rover` | small | v1.17 ships the stock target; add WENC Kconfig + DDS client + confirm RoboClaw. Replaces the whole CubeOrange board-file lineage. |
| C5 | *(candidate)* uXRCE-DDS client connection retry | medium | Mitigates **A1.5 / #27860** at the source. Small, well-scoped, and a genuine upstream contribution. |
| C6 | *(candidate)* Setpoint freshness timeout in rover controllers | medium | Mitigates **A1.1 / #27514** at the source. Upstream is actively discussing it — align rather than fork. |

### C2 implementation contract — the part that fails silently

`mavlink_receiver.cpp:2603` (`handle_message_gps_rtcm_data`) does nothing but copy `len`, `flags`
and `data` into the uORB message. **All reassembly happens downstream in the GPS driver.** So the
ROS-side NTRIP client must chunk RTCM frames larger than **300 bytes** and set the fragment flag
(LSB of `flags`) exactly as the MAVLink sender would.

> ⚠ A community report describes `gps_inject_data` visible in the PX4 shell while `gps status`
> showed no corrections received. **Validate on `gps status` and a FIX-type transition on the
> vehicle — never on topic presence.**

---

# PART D — F1 work order

```
F1.0  Rebase discipline
      └─ derive each B1 patch as a semantic diff vs its v1.16.2 stock ancestor
         (NOT the fork's post-v1.18 file — EKF2 renamed ekf2_gps_ctrl,
          ekf2_hdg_gate, _gnss_checks.passed() after v1.16.2)

F1.1  Drivetrain            B1 #5,6,7,8   RoboClaw QPPS, raw mode, creep, deadband
F1.2  Instrumentation       B1 #9,10,11   logger topics  ← before the estimator work
F1.3  Estimator             B1 #1,2       WENC fusion + lever arm        ⟵ GATE 2
F1.4  GNSS yaw              B1 #3,4       EKF2_GPS_YAW_N/_G + timestamp  ⟵ GATE 2
F1.5  Transport             C1, C2        DDS client on rover + RTCM
F1.6  Board                 C4            px4_fmu-v6x_rover target
F1.7  Decisions             B2 #13-17     one recorded decision per row → docs/contracts/
F1.8  Optional              C3            ulog streaming
```

**F1.2 before F1.3 is deliberate.** The logger patches are what make GATE 2 evaluable; landing
the estimator work first means flying blind through the highest-risk change in the programme.

## Gates

| Gate | Applies to | Criterion |
|---|---|---|
| **GATE 2** (replay) | F1.3, F1.4 | Decision-identical `estimator_aid_src_*` output at default parameters across the existing corpus. **Not** "it builds". Harness: `px4-firmware-verification` skill + `~/Vetri/f2v2_replay/`. |
| **F2 field** | after flash | Pivot wobble ≤ **0.50 cm** median, net walk ≤ **0.83 cm** — same drift-radius methodology as the 34-ulog baseline. |
| **F3 soak** | bridge service | Multi-hour run, per-topic rate logging, survives FCU reboot / cable pull / agent restart. **Covers A1.3.** |
| **F4 mission** | firmware health | AUTO mission completes with no companion code in the path. **Blocked by A1.2 — attempt reproduction first.** |
| **F5 offboard** | command interface | GATE 1 flag combination; stale-companion stop distance measured (**A1.1**); arc RMS vs the F2 baseline — **the programme go/no-go**. |
| **RTK** | C2 | `gps status` shows corrections received **and** FIX type transitions. Topic presence is not evidence. |
| **Timesync** | A1.4 | Offset converged before a mission is accepted; value recorded at mission start and end. |

---

# PART E — Open questions

1. **B1 #8** — `57f6b772fc` vs `60218dfe81` are near-duplicate deadband commits. Which supersedes which?
2. **B3 `62619611d6`** — the Sabertooth IK sign swap. Confirm no residual sign dependency survived into the RoboClaw path before dropping it.
3. **B2 #15** — P1–P4 targeted a rover module that no longer exists. What was each patch actually *protecting against*? That intent must be re-derived from the field history, not the diff.
4. **A1.2 / #27497** — reproduce before F4, or design F4's gate to avoid it?
5. **C5 / C6** — contribute upstream, or carry as fork patches? Upstream is cleaner long-term but slower than our schedule.
6. **`DifferentialPosControl`** — now compilable on v1.17. Overlay it, or stay on stock? It changes disarm gating, and stock already gates output behind `flag_armed`. **Do not overlay reflexively just because it compiles.**
