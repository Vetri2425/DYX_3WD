# DYX 3WD Production Stack Architecture — V1

**Vehicle:** 3-wheel precision ground-marking rover
**This repository:** `Way_to_Mark/DYX_3WD` — the production stack. Built here, from zero.
**Evidence source:** `PX4_DXP` @ `baseline_master` lineage — **read-only**. Nothing is edited there.
**Firmware source:** `Vetri2425/PX4-Autopilot` @ `main` — ⛔ never `~/px4-rover-build`
**Date:** 2026-09-05 · **Revision:** V1 (supersedes the 2026-09-04 draft written inside PX4_DXP)

---

## 0. How to read this document

This is not a port of the DYX_4WD architecture. That document was read once, for format.
Every technical decision below is derived from the actual `PX4_DXP` system: its source, its
field evidence, its known bugs, and its firmware fork.

| Marker | Meaning |
|---|---|
| **VERIFIED** | Checked against the repositories on disk during authoring, with the command cited. |
| **GATE** | Cannot be settled from a desk. Blocks the dependent work until answered on the bench or in the field. |

Everything else is design intent, and is open to argument.

Two rules govern the whole programme:

> **`PX4_DXP` is evidence, not a template.** Its behaviour is authoritative. Its structure is not.
>
> **Change one variable at a time.** An unattributable regression is worse than no change.

---

## 1. The decision: refactor, or start fresh?

Both, and the split is not a compromise — it follows from where the value actually lives.

**The structure is worthless. The behaviour is the entire asset.**

`PX4_DXP` today: loose Python at the repository root, `old source/` and `baseline/`
directories tracked in Git, `src/` mixing production nodes with 60+ test files, four
hand-rolled systemd units, no ROS workspace, no build system, backend and control logic
sharing a process boundary that does not exist. None of that is worth carrying.

But inside it sits something that cannot be re-derived: **~30 closed field bugs encoded in
the control logic**, 120 parameters tuned across dozens of field days, and a geometry stack
whose failure modes were found the expensive way.

So the answer is:

> **Fresh repository. Fresh structure. Fresh build system. Fresh services.
> Migrated behaviour — never re-invented, never blind-translated.**

### 1.1 What carries, what is rewritten, what is abandoned

| Carry verbatim (behaviour is the asset) | Why |
|---|---|
| RPP control logic | ~30 closed bugs are encoded in it. Enters as `dyx3_rpp_legacy` (§7.5), then is ported module by module against itself. |
| 120 RPP + 54 spray parameters, and the frozen corner-stop defaults | Field-tuned. Re-validated, not re-derived. |
| `path_engine/` — DXF, CRS, arc chains, extensions, segment ordering | ~8 000 lines with a hard bug history. Stays Python (§7.2). |
| Spray FSM, safety lease, independent watchdog, flow model | Field-proven safety structure. |
| NTRIP / RTCM protocol handling | Works. Do not spend risk budget here. |
| Analysis + replay tooling | Re-based for the new timebase (§4.5), not rewritten. |

| Rewritten (structure was the defect) | Why |
|---|---|
| Motion output stage | The whole point of the migration (§5). |
| MAVROS coupling — 16 topics/services | Becomes `dyx3_px4_link`. |
| `server/ros_node.py`, `offboard_controller.py` | Becomes `dyx3_system_gateway` + `dyx3_motion_guard`. Backend loses `rclpy`. |
| `rtk_manager.py` spawning `ntrip_rtcm_node.py` as a child process | A `server/**` deploy silently drops the rover to FLOAT. Becomes an independent service. |
| 4 hand-rolled systemd units + `deploy.sh` | Installer-managed, with verify and rollback. |
| `bag-autorecord.service` | Becomes `dyx3_recorder` with full provenance. |

| Abandoned outright | Why |
|---|---|
| `old source/`, `baseline/`, root-level loose scripts | Git is the backup. |
| ENU↔NED conversion | `px4_msgs` is NED/FRD natively. A whole class of sign bug disappears (§7.5). |
| MAVROS liveness hack (`_state_recv_time` override) | Real DDS session liveness replaces it. |
| `/mavros/param` stale mirror | No mirror exists over DDS. Params come from the service plane, live. |
| `px4_pluginlists_rover.yaml`, `px4_start_service.sh` | MAVROS-specific. |
| `ntrip.service` | Documented dead end — needs RTKLIB `str2str` and `~/ntrip_stream.sh`, neither of which exists. |
| The CI `cp`-overlay firmware pattern | §3.8. Replaced by a real rebase. |

### 1.2 The consequence for sequencing

Because behaviour migrates rather than being re-invented, the legacy Python controller must
**run inside the new workspace**, not be left behind in the old repository. It is the
shadow-run oracle. You cannot measure equivalence against something you cannot execute in
the same graph.

This is what makes the staged plan in §11 possible, and it is why the precision win arrives
long before the C++ rewrite finishes.

---

## 2. What we are changing, ranked by real risk

| # | Upgrade | Risk | Why |
|---|---|---|---|
| 1 | CubeOrange+ → Pixhawk 6X (`px4_fmu-v6x`) | **Low–Medium** | Board port is stock upstream. But it invalidates measured mount geometry (§4.1). |
| 2 | UART/USB → Ethernet | **Low** | 6X has the port; PX4 `netman` is mature. Risk is topology, not the link. |
| 3 | v1.16.2 + overlay → v1.17.0 | **HIGH** | The fork's EKF2 wheel-encoder fusion and GNSS-yaw work do not exist upstream. All of it re-anchors by hand. |
| 4 | MAVROS2 → uXRCE-DDS | **Medium**, **largest payoff** | Removes the documented structural tracking floor. Three services fall off the DDS path and need a plan. |
| 5 | Python → C++ control nodes | **HIGH**, **zero direct precision gain** | 6 285 lines, 120 parameters, the bug-densest FSM in the project. |
| 6 | Production workspace + single installer | **Medium** | Discipline. Cheap now, expensive retrofitted. |

**The dominant risk is none of these individually — it is doing them together.** See R0 in §13.

---

## 3. Verified firmware findings

Checked against the repositories on disk during authoring.

### 3.1 The precision problem this migration actually solves

The current record, from `CLAUDE.md` and the 77-bag shape audit:

| Shape @0.35 m/s | RMS |
|---|---|
| square | 0.87 cm ← the only real sub-1 cm baseline |
| lshape | 0.90 cm |
| U-turn | 1.06 cm |
| **arc** | **1.46 cm** |
| circle | never passed |

Full-mission truth (08-05) was **1.91 → 1.69 cm**. This is a **sub-2 cm rover**, not a
sub-cm rover, and the distribution is not random: squares and L-shapes are straights plus
pivots, where the floor does not bind. Arcs and circles are continuous curvature, where it
does.

`CLAUDE.md`, on the arc case:

> *"Arc (smooth RPP) — structural floor, DEFERRED: velocity OFFBOARD discards
> `trajectory_setpoint.yawspeed`; pure-P → following err `≈ ω/RO_YAW_P`. `RO_YAW_P=1.5`.
> Companion `yaw_rate_feedback_gain` is a NO-OP in velocity mode."*

**The migration attacks precisely the failure that exists.** That is a far stronger
justification than protecting a baseline — and it is also a falsifiable one, which is why
§11 Stage 0 quantifies it from existing bags before a line is written.

### 3.2 v1.17 exposes the rover setpoints over DDS

**VERIFIED** — `git show v1.17.0:src/modules/uxrce_dds_client/dds_topics.yaml`

```text
/fmu/in/rover_position_setpoint     px4_msgs::msg::RoverPositionSetpoint
/fmu/in/rover_speed_setpoint        px4_msgs::msg::RoverSpeedSetpoint
/fmu/in/rover_attitude_setpoint     px4_msgs::msg::RoverAttitudeSetpoint
/fmu/in/rover_rate_setpoint         px4_msgs::msg::RoverRateSetpoint
/fmu/in/rover_throttle_setpoint     px4_msgs::msg::RoverThrottleSetpoint
/fmu/in/rover_steering_setpoint     px4_msgs::msg::RoverSteeringSetpoint
```

Yaw rate stops being discarded. `yaw_rate_feedback_gain` stops being a no-op.

### 3.3 …but the naive path throws it away again

**VERIFIED** — `v1.17.0:.../DifferentialOffboardMode/DifferentialOffboardMode.cpp`

```cpp
} else if (offboard_control_mode.velocity) {
    const Vector2f velocity_ned(trajectory_setpoint.velocity[0], trajectory_setpoint.velocity[1]);
    rover_speed_setpoint.speed_body_x     = velocity_ned.norm();
    rover_attitude_setpoint.yaw_setpoint  = atan2f(velocity_ned(1), velocity_ned(0));
}
```

Migrate transport but keep publishing an XY velocity vector, and PX4 v1.17 does `‖v‖` and
`atan2(vy, vx)` — **today's information loss on a new transport.**

> Transport and command interface must move together. DDS without the rover-setpoint
> interface is cost with no benefit.

**GATE 1 — Bench.** Determine the `OffboardControlMode` flag combination under which
externally-published `rover_speed_setpoint` + `rover_rate_setpoint` reach the control chain
without `DifferentialOffboardMode` republishing over them from a stale `trajectory_setpoint`.
Two candidate paths exist. **Blocks Stage B.**

### 3.4 DDS can be proven on the *current* hardware — cheaply

**VERIFIED** — v1.17 `uxrce_dds_client.cpp` implements `uxrSerialTransport`
(`uxr_init_serial_transport`, `init serial %s @ %d baud`). Serial transport is fully
supported; Ethernet is not required to use DDS.

**VERIFIED** — `CONFIG_MODULES_UXRCE_DDS_CLIENT=y` is present in `default.px4board` for both
`cubeorangeplus` and `fmu-v6x`, but **absent from either `rover.px4board`**.

**VERIFIED** — `boards/cubepilot/cubeorangeplus/rover.px4board` **does not exist upstream in
v1.17.0**. It is fork-only, already on the overlay list, already carrying
`CONFIG_DRIVERS_ROBOCLAW=y` and `CONFIG_EKF2_WHEEL_ENCODER=y`.

> **Enabling uXRCE-DDS on the current CubeOrange+ is one Kconfig line in a file you already
> own.** This is what makes Stage B possible before any hardware change, and it is the
> single most important sequencing fact in this document.

### 3.5 A v1.16.2 overlay blocker dissolves

**VERIFIED** — `RoverSpeedSetpoint.msg` and `RoverSpeedStatus.msg` exist in v1.17.0 and not
in v1.16.2. The CI note —

```text
# NOTE: DifferentialPosControl is intentionally NOT overlaid.
# The fork's PosControl ... requires RoverSpeedSetpoint.msg / RoverSpeedStatus.msg,
# which do not exist in v1.16.2 -> compile failure.
```

— no longer applies. Whether it *should* be overlaid is a separate decision: it changes
disarm gating, and stock v1.16.2 PosControl already gates output behind `flag_armed`. Do not
overlay it reflexively just because it now compiles.

### 3.6 The expensive part: the EKF2 work is not upstream

**VERIFIED** — `v1.17.0 src/modules/ekf2/EKF/aid_sources/` contains `airspeed,
aux_global_position, auxvel, barometer, drag, external_vision, gnss, gravity, magnetometer,
optical_flow, range_finder, sideslip` — **no `wheel_encoder`**.

**VERIFIED** — `v1.17.0:src/modules/ekf2/params_gnss.yaml` has only `EKF2_GPS_YAW_OFF`.
Neither `EKF2_GPS_YAW_N` nor `EKF2_GPS_YAW_G` exists upstream.

Re-anchoring by hand onto v1.17:

```text
msg/EstimatorAidSource3d.msg
src/modules/ekf2/{Kconfig, CMakeLists.txt, EKF2.cpp, EKF2.hpp}
src/modules/ekf2/{params_wheel_encoder.yaml, params_gnss_yaw.yaml}
src/modules/ekf2/EKF/{CMakeLists.txt, common.h, control.cpp, ekf.h,
                      estimator_interface.h, estimator_interface.cpp}
src/modules/ekf2/EKF/aid_sources/wheel_encoder/wheel_encoder_fusion.cpp
src/modules/ekf2/EKF/aid_sources/gnss/gnss_yaw_control.cpp
```

This carries two field-verified wins the marking spec depends on:

- **`255453d967`** — WENC IMU lever-arm correction. Pivot wobble radius **1.52 → 0.50 cm**
  median, net walk **2.02 → 0.83 cm**, n=44 pre / n=105 post pivot windows, 34 ulogs.
- **`1d82e616f8`** — RoboClaw encoder timestamp taken *before* the UART transaction,
  closing ~264 ms of unbounded jitter into the WENC fusion buffer.

Plus built-but-unflashed F2 v2 (`015c67484e`, `a6e9e12e2a`): `EKF2_GPS_YAW_N` + `EKF2_GPS_YAW_G`.

**This is the highest-risk item in the programme.** EKF2 is not a stable API across
1.16 → 1.17: `common.h`, `ekf.h` and `control.cpp` all move. A re-anchor that compiles is
not a re-anchor that works.

**GATE 2 — Replay.** Re-anchored v1.17 EKF2 must be replay-verified against the same log
corpus as the v1.16.2 work before flashing. Harness: the `px4-firmware-verification` skill
and the artifacts at `~/Vetri/f2v2_replay/`. Pass criterion is **decision-identical
`estimator_aid_src_*` output at default parameters** — the neutrality bar F2 v2 already
cleared — not "it builds".

### 3.7 What survives intact

**VERIFIED** — `src/drivers/roboclaw` exists in v1.17.0. `msg/WheelEncoders.msg` exists
upstream. `boards/px4/fmu-v6x/rover.px4board` is a **stock upstream target** carrying
`CONFIG_DRIVERS_ROBOCLAW=y`.

The 6X therefore gets an upstream-maintained rover target, unlike the CubeOrange+ one which
the fork has to supply.

### 3.8 Kill the `cp`-overlay pattern during the re-anchor

The current CI copies 27 fork files onto a clean v1.16.2 checkout without committing.
Consequences already paid:

- **`ver_sw` can never identify a fork build** — every build records base hash `54f0455f`.
  A flash is undetectable from firmware version or FCU params.
- `git log` / `git blame` lie about every overlaid file. This produced a confident-but-wrong
  "`DifferentialVelControl` isn't compiled" audit, of a module that was flying.
- Build directories are the only discriminator, and they have already been lost from disk
  once.

Since 13+ EKF2 files must be re-derived by hand anyway, the marginal cost of doing it
properly is near zero:

> **Rebase the fork onto `v1.17.0` as real commits. Tag it. Build from the fork directly.**
> Recover each patch by diffing the overlaid file against its **v1.16.2 stock ancestor** —
> that diff is the actual semantic change. Re-apply the diff, not the file.

Output: identifiable, bisectable firmware history, and a `dyx3-version` that can finally
tell you what is on the vehicle.

### 3.9 RTCM cannot ride stock DDS

**VERIFIED** — `gps_inject_data` does not appear in v1.17.0's `dds_topics.yaml`. See §5.3.

---

## 4. Hardware and link topology

### 4.1 Hardware

| Item | Current | Target |
|---|---|---|
| FCU | CubeOrangePlus | **Pixhawk 6X** (`px4_fmu-v6x`) |
| Companion | Jetson Orin (`192.168.1.102`) | unchanged |
| FCU ↔ Jetson | `/dev/ttyACM0` @ 921600 | **Ethernet** (via DDS-over-serial in Stage B) |
| GNSS | UM982 dual-antenna, TELEM1 | unchanged |
| Motors | RoboClaw, closed-loop QPPS | unchanged |
| Geometry | differential, `RD_WHEEL_TRACK=0.470` | unchanged |
| Marking | AUX1 valve, `PWM_AUX_FUNC1=301` | same function, new 6X pin map |

> ⚠ **The 6X swap invalidates measured mount geometry.** `EKF2_IMU_POS_X=0.100` was
> *live-verified* on the current mount, and `GPS_YAW_OFFSET=180.0` is a round number that was
> assumed rather than measured (open bug B1). Different IMU set, different vibration
> signature, different mounting. Stage C re-measures all of it — which is exactly why the
> hardware change must not overlap the estimator change.

> **Confirm the third wheel physically.** `RD_WHEEL_TRACK` + 2-channel RoboClaw +
> `rover_differential` imply two driven wheels and one undriven. Its caster/steering
> behaviour affects pivot dynamics, which is where this rover's accuracy lives.

### 4.2 Two planes, one cable

```text
                      Jetson Orin
        ┌────────────────────────────────────┐
        │  Micro XRCE-DDS Agent      :8888   │◄─── DATA PLANE — control + state
        │  MAVLink sidecar          :14540   │◄─── SERVICE PLANE
        │  QGC forward              :14550   │
        └──────────────────┬─────────────────┘
                           │  Ethernet 10.41.10.0/24   (Stage C)
                           │  serial TELEM2 @ 921600   (Stage B)
        ┌──────────────────┴─────────────────┐
        │             Pixhawk 6X             │
        │  uxrce_dds_client                  │
        │  mavlink instance 2                │
        └────────────────────────────────────┘
```

**Data plane — uXRCE-DDS.** Control setpoints, vehicle state, estimator output. Hard
real-time. Nothing else on it.

**Service plane — MAVLink/UDP.** Everything DDS structurally cannot carry:

1. **QGC bridge** — a hard rule. QGC speaks MAVLink; there is no DDS QGC.
2. **FCU parameters** — PX4 params are **not** exposed over uXRCE-DDS. `quick_params.py`
   reads them today via `/mavros/param`. Without a MAVLink link, both the tool and the
   "read the FCU, never the table from memory" discipline die.
3. **RTCM injection** — §5.3.
4. **ULog download.**

> **"No MAVROS in the control path" ≠ "no MAVLink on the vehicle."**
> MAVLink stops being a control transport and becomes a service transport.
> It leaves the loop, not the machine.

### 4.3 Field topology — no router, no site LAN

**Decision (2026-09-05): the rover carries its own network. There is no router and no site
LAN in the production path.**

```text
                    ┌──────────────── Jetson Orin ────────────────┐
   operator tablet  │                                             │
   ── WiFi ────────►│ wlan0  HOTSPOT (AP+DHCP+NAT)  10.42.0.1/24  │
   Mac / QGC / SSH  │ BLE    discovery · pairing · status beacon   │
   ── WiFi ────────►│                                             │
                    │ usb0   LTE dongle ──► NTRIP caster (WAN)    │
                    │                                             │
                    │ eth0   10.41.10.1/24 ──── direct cable ─────┼──► Pixhawk 6X
                    └─────────────────────────────────────────────┘        10.41.10.2/24
```

**This is the right call, and it is simpler than what it replaces.** Four things follow
from it, and one of them is a gap that must be designed rather than inherited.

**1. The FCU link never needed a router.** `10.41.10.0/24` is a **point-to-point cable**
between two static addresses — no switch, no DHCP, no routing. "No router" changes nothing
about §4.4; it only removes the site LAN that used to sit beside it.

**2. Hotspot.** Use NetworkManager `ipv4.method=shared` on `wlan0` — that single setting
provides AP + DHCP + NAT, rather than hand-rolling `hostapd` and `dnsmasq`. SSID and PSK
come from `/etc/dyx3/`. Publish **mDNS/avahi** so the Mac reaches `flash@rover-3wd.local`
without anyone asking what address it got today.

> ⚠ **Do not attempt simultaneous AP + station mode on the same radio.** It is chipset-
> dependent and unreliable. The LTE dongle removes the need entirely: the hotspot serves
> clients, the dongle serves WAN. Keep that separation — it is the reason this design is
> clean. Verify the Orin's WiFi chipset supports sustained AP mode before committing.

**3. WAN via LTE dongle is an accuracy improvement, not just convenience.** RTK correction
delivery stops depending on site WiFi or an operator's phone. `dyx3-platform` owns dongle
bring-up, reconnect and link health, and NTRIP link state becomes a recorded vehicle state.

**4. SSH and QGC both ride the hotspot.** The Mac joins the rover's AP, gets a lease, and
reaches SSH and the QGC UDP forward. Note the consequence: the Mac loses its own internet
while joined, and the tablet app and QGC now share one radio. Acceptable in the field —
worth knowing before a demo.

### 4.3.1 The gap this creates: operator-link loss is not a PX4 failsafe

With no router and the tablet as the only operator link, a WiFi dropout is **not** an FCU
datalink loss. The Jetson is still talking to PX4, so `NAV_DLL_ACT=6` (Disarm) will not
fire. Nothing in PX4 knows the operator is gone.

> **The Jetson must own this failsafe itself.** A tablet heartbeat timeout is a first-class
> vehicle event: `dyx3_system_gateway` detects it, `dyx3_motion_guard` executes a controlled
> stop. This is new behaviour, not a port — design it explicitly in Stage E.

**BLE** is good for discovery, pairing and a status beacon. It is **not** an E-stop path:
its latency and delivery are not deterministic. If a BLE stop button is wanted, it enters
the arbiter as one more *request* alongside the tablet's — never as the sole path, and never
in place of a physical E-stop, which remains the authority.

### 4.4 Addressing

```text
Pixhawk 6X eth0   10.41.10.2/24   PX4 netman default — point-to-point
Jetson eth0       10.41.10.1/24   XRCE agent + MAVLink sidecar
Jetson wlan0      10.42.0.1/24    hotspot: tablet, Mac, QGC
Jetson usb0       (LTE)           WAN — NTRIP only
```

The FCU cable is not bridged or routed to `wlan0`. The tablet never sees DDS traffic.
`ROS_DOMAIN_ID` is pinned and DDS discovery is **scoped to `eth0`** — an unconstrained
default-domain FastDDS that can see the hotspot will find a laptop, or the 4WD rover, and
you will lose a day to it.

### 4.5 PX4 configuration

```text
# Ethernet (Stage C)
/fs/microsd/net.cfg -> DEVICE=eth0 BOOTPROTO=static
                       IPADDR=10.41.10.2 NETMASK=255.255.255.0 ROUTER=10.41.10.1

# Data plane
UXRCE_DDS_CFG    = TELEM2 (Stage B)  ->  Ethernet (Stage C)
UXRCE_DDS_AG_IP  = 10.41.10.1        (int32-encoded)
UXRCE_DDS_PRT    = 8888
UXRCE_DDS_DOM_ID / UXRCE_DDS_KEY     pinned per vehicle

# Service plane
MAV_2_CONFIG     = Ethernet
MAV_2_MODE       = Onboard
MAV_2_REMOTE_PRT = 14540
MAV_2_BROADCAST  = 0
```

### 4.6 Two things that break quietly

**Message-definition hash mismatch.** uXRCE-DDS validates message formats
(`/fmu/in/message_format_request` is in v1.17's subscription list). A `px4_msgs` built from
the wrong branch produces **silent no-data**, not an error.

> `px4_msgs` is pinned to the exact firmware release, its SHA recorded in the run manifest,
> and a startup handshake fails loudly on mismatch. A `dyx3_px4_link` responsibility, not a hope.

**Timebase change.** MAVROS delivered ROS-time-stamped messages. `px4_msgs` carry PX4 `hrt`
microseconds reconciled by the DDS client's own timesync. Every existing analysis tool —
`tools/analyze_bag_ulog.py`, the replay harness, every bag↔ulog correlation — assumes the
old alignment.

> **The analysis toolchain re-base is Stage 0 work, not cleanup.** Without it you cannot
> measure whether the migration regressed accuracy, during the exact change that most needs
> measuring. Every gate in §11 is unevaluable until this is done.

**Serial-transport bandwidth (Stage B only).** DDS over TELEM2 @ 921600 will not carry the
full topic set at full rate. Stage B subscribes a **minimal set** — local position,
attitude, status, and the setpoint publications. Ethernet removes the constraint in Stage C.
This is a known, temporary, and acceptable limitation of the probe.

---

## 5. Control authority

### 5.1 The change

```text
TODAY                                  TARGET
─────                                  ──────
RPP (Python)                           dyx3_rpp
  │ TwistStamped, XY vector, ENU         │ MotionSetpoint {mode, speed, yaw, yaw_rate}
  ▼                                      ▼
MAVROS                                 dyx3_motion_guard   (validate, limit, gate, fail-to-zero)
  ▼                                      ▼
PX4 velocity offboard                  dyx3_px4_link
  ├─ derives bearing from the vector     │ /fmu/in/rover_speed_setpoint
  ├─ DISCARDS commanded yaw rate  ◄──    │ /fmu/in/rover_rate_setpoint  (or attitude)
  └─ pure-P yaw, err ≈ ω / RO_YAW_P      ▼
                                       PX4 v1.17 rover_differential
                                         ├─ DifferentialSpeedControl
                                         ├─ DifferentialRateControl / AttControl
                                         └─ DifferentialActControl → RoboClaw
```

RPP owns the **path-level rotational decision**. PX4 owns the **inner physical loop**.
Neither infers what the other meant.

### 5.2 Control modes

These are not new inventions. They make explicit the modes already latent in
`rpp_controller_node.py`.

| Mode | Speed | Rotation | Formalises |
|---|---|---|---|
| `STOP` | 0 | 0 | `_publish_zero`, stop latch, dwell |
| `TRACK_HEADING` | v | `yaw_setpoint` | straight-segment tracking |
| `TRACK_RATE` | v | `yaw_rate_setpoint` | arc / active correction — **recovers the floor** |
| `PIVOT` | 0 | `yaw_rate_setpoint` | corner stop-pivot, `_corner_pivot_velocity` |
| `CREEP` | v ≤ endpoint speed | `yaw_rate_setpoint` | terminal approach, `endpoint_approach_speed=0.03` |

`CREEP` is separated deliberately. The 0.03 m/s crawl **cannot steer**: it misses the 2 cm
capture ball, the Euclidean `approach_ref` then grows, and the rover accelerates away into an
E-stop. Giving the terminal regime its own mode and its own limits makes that failure
representable instead of emergent.

### 5.3 RTK — the subsystem that must not break

Today's defects, both of which must die in the migration:

- `ntrip_rtcm_node.py` is a **child process of `rover-server`**, so any `server/**` deploy
  silently drops the rover to FLOAT **with no warning in the app**.
- No autostart after a `rover-server` restart.

**VERIFIED:** `gps_inject_data` is not in stock v1.17 `dds_topics.yaml` — but adding it is
far cheaper than first assumed. See §5.4, which **supersedes the earlier "Option B for V1"
recommendation.**

| Option | Path | Verdict |
|---|---|---|
| **A** | add `gps_inject_data` to `dds_topics.yaml` | **Recommended.** A 2-line build-time patch (§5.4.1). Keeps RTK on the data plane. |
| **B** | NTRIP → `GPS_RTCM_DATA` over the MAVLink service plane | Fallback. Field-proven semantics, zero firmware surface. Keep as the Stage-A path and as the rollback. |
| **C** | NTRIP → **UM982 second UART, direct from the Jetson** | **Evaluate seriously.** Removes PX4 from the correction path entirely (§5.4.4). |

Structurally, regardless of transport: RTK is **its own service**, never a child of the
backend; it **autostarts** and self-reconnects; correction age, fix type and link state are
published, recorded, and gated on by both the drive and spray gates. Losing RTK is a
**first-class vehicle state**, not a log line.

---

## 5.4 Can the service plane be eliminated? — patch feasibility research

Researched 2026-09-05 against the v1.17.0 source and community reports. The question was
whether RTCM, parameters, ULog pull and QGC can all move to DDS, letting MAVLink disappear
entirely.

**Answer: RTCM yes, cheaply. ULog probably. Parameters and QGC, no.** The service plane
shrinks substantially but does not vanish.

| Need | DDS path in v1.17? | Cost | Verdict |
|---|---|---|---|
| **RTCM injection** | **Yes** | 2 lines | **Patch it** |
| **ULog capture** | Partially — streaming, not SD pull | Small patch + Jetson reassembler | **Attractive — solves provenance** |
| **Parameter get/set** | **No** | New msgs + module + rebase burden | **Keep MAVLink** |
| **QGC** | **No, structurally** | — | **Keep MAVLink** |

### 5.4.1 RTCM over DDS — a two-line patch

**VERIFIED.** `msg/GpsInjectData.msg` exists in v1.17 (`uint16 len`, `uint8 flags`,
`uint8[300] data`, `ORB_QUEUE_LENGTH=8`). The `gps_inject_data` uORB topic is **already
subscribed by the GPS driver** (`src/drivers/gps/gps.cpp`) — the consumer side needs no
change at all. And `dds_topics.yaml` is consumed at **build time** by
`generate_dds_topics.py` from `uxrce_dds_client/CMakeLists.txt`, so exposing a topic is a
codegen input, not code:

```yaml
subscriptions:
  - topic: /fmu/in/gps_inject_data
    type: px4_msgs::msg::GpsInjectData
```

That is the entire firmware change. It is smaller than the MAVLink alternative's Jetson-side
work, and it puts RTK on the same transport as everything else in the data plane.

**Two implementation details that decide whether it works:**

- **Fragmentation is the caller's job.** The MAVLink path
  (`mavlink_receiver.cpp:2603`, `handle_message_gps_rtcm_data`) does nothing but copy `len`,
  `flags` and `data` straight into the uORB message — **all reassembly happens downstream in
  the GPS driver.** So the ROS-side NTRIP client must chunk RTCM frames larger than 300 bytes
  and set the fragment flag (LSB) exactly as the MAVLink sender would. Get this wrong and
  corrections arrive but are silently useless.
- **A community report describes exactly that failure**: `gps_inject_data` visible in the PX4
  shell while `gps status` showed no corrections received. **Validation is `gps status` on the
  FCU and a FIX-type transition on the vehicle — never the presence of the topic.**

### 5.4.2 Parameters — no path, and it is a known gap

**VERIFIED.** v1.17 has `ParameterSetValueRequest/Response`, `ParameterResetRequest`,
`ParameterSetUsedRequest`. They look promising and are **not usable**:

- consumed only by `src/lib/parameters/parameters_primary.cpp` and `parameters_remote.cpp` —
  internal **multi-MCU parameter sync** plumbing, not an external API
- addressed by `uint16 parameter_index`, **not by name**
- **there is no GET path at all**

Community reports confirm it: there is no documented programmatic way to set PX4 parameters
from ROS 2 over uXRCE-DDS, and people fall back to QGroundControl.

A custom patch would mean new name-addressed request/response messages, a get path, a
module to service them, and a rebase burden on every firmware upgrade — for a capability
MAVLink already provides correctly. **Not worth it. Parameters keep MAVLink.**

This also preserves `quick_params.py` and the "read the FCU, never the table from memory"
discipline, which has already caught four documented-vs-live parameter errors.

### 5.4.3 ULog — the patch worth considering

**VERIFIED.** `msg/UlogStream.msg` and `msg/UlogStreamAck.msg` exist as uORB topics; they
back MAVLink log *streaming*. Exposing them over DDS is another `dds_topics.yaml` addition
plus a Jetson-side reassembler.

This is worth doing for a reason beyond transport tidiness:

> **It streams the flight log into the run directory as the run happens.** No MAVLink FTP, no
> SD-card pull, no post-hoc collection step. `dyx3_recorder` gets the ulog automatically,
> alongside the bag, inside the same run folder — which is exactly the provenance property
> §7.9 demands, obtained structurally instead of by procedure.

Caveat: this is *streaming*, not retrieval of completed logs from the SD card. Keep MAVLink
FTP (or the card) as the recovery path for anything the stream misses.

### 5.4.4 The option that removes the question — UM982 second UART

The UM982 exposes multiple serial ports. If the FCU uses COM1, the **Jetson can drive COM2
for RTCM injection directly.**

That makes RTK correction delivery completely independent of PX4: no `dds_topics.yaml`
patch, no MAVLink message, no autopilot involvement, no fragmentation contract. Correction
delivery stops being coupled to firmware upgrades — permanently.

Cost: one cable, and the Jetson needs the port. Given that RTK is the foundation of the
entire accuracy spec and the current architecture has already dropped the rover to FLOAT
silently through a *software deploy*, decoupling it from the autopilot deserves a serious
look before the 6X is wired.

**Action:** confirm the UM982's spare port and its RTCM input configuration during Stage C
harness design, while the wiring is still on paper.

### 5.4.5 QGC — MAVLink stays

QGC speaks MAVLink. There is no DDS QGroundControl and none is planned. It is also how you
flash, calibrate, and recover a vehicle that will not boot — the exact situations where a
custom tool is least trustworthy.

> **Verdict: the service plane shrinks to parameters, QGC and log recovery — but it stays.**
> "No MAVROS in the control path" remains achievable and is the actual goal. "No MAVLink on
> the vehicle" is not achievable in v1.17, and pursuing it would cost more than it returns.

---

## 6. Workspace layout

Package prefix **`dyx3_`**. Not cosmetic: `dyx_` belongs to the 4WD stack, and a shared
prefix invites a shared interface package. The two vehicles genuinely disagree about what a
rover is — this one has a marking actuator and treats wheel encoders as a primary aid.
Keep them separate. If a small `dyx_core_interfaces` later earns its existence on evidence,
extract it then.

```text
DYX_3WD/
├── ros2_ws/src/
│   ├── dyx3_interfaces/       msg / srv / action only — no logic
│   ├── dyx3_geometry/         C++ library, NO ROS dependency, exhaustively tested
│   ├── dyx3_mission/          mission + run/point FSM
│   ├── dyx3_rpp/              path following — the C++ target
│   ├── dyx3_rpp_legacy/       ⚠ QUARANTINED Python oracle (§7.5) — deleted at Gate 7
│   ├── dyx3_motion_guard/     validity, limits, watchdog, fail-to-zero
│   ├── dyx3_px4_link/         the only package permitted to touch /fmu/**
│   ├── dyx3_gnss_rtk/         NTRIP, RTCM, correction health
│   ├── dyx3_spray/            marking actuator + safety lease + watchdog
│   ├── dyx3_recorder/         bag + manifest + provenance
│   ├── dyx3_system_gateway/   the single ROS ↔ backend boundary
│   └── dyx3_bringup/          launch + config authority
├── backend/src/dyx3_backend/  Python. FastAPI + Socket.IO + path engine. No rclpy.
├── config/                    profiles, grouped by owning package
├── deployment/                systemd, network, udev
├── installer/                 install / upgrade / verify / rollback
├── firmware/                  overlay-to-rebase migration, param baselines, build pinning
├── tools/                     analysis, replay, field, migration
└── docs/{architecture,contracts,interfaces,safety,tuning,validation,migration,hardware}/
```

**Rules.** All production ROS packages are `ament_cmake`, C++17 — with exactly one
time-boxed exception, `dyx3_rpp_legacy`. `dyx3_geometry` has no ROS dependency, so it is
unit- and replay-testable without a ROS installation. No backup files, no bags, no logs, no
secrets tracked. No `rclpy` in the backend; no FastAPI in a ROS node.

---

## 7. Package design

### 7.1 `dyx3_interfaces`

Frozen first — everything is written against it.

```text
msg/  MotionSetpoint, MotionSetpointStatus, VehicleState, RppStatus,
      MissionState, PointResult, RtkStatus, SprayState, RecorderStatus
srv/  StartMission, PauseMission, ResumeMission, AbortMission, SkipPoint, SetEmergencyStop
action/ ExecuteMission
```

`MotionSetpoint` carries `{stamp, seq, mode, speed_body_x, yaw_setpoint, yaw_rate_setpoint,
valid}` and is the single canonical command. Nothing downstream of RPP invents motion;
nothing upstream bypasses it. `MotionSetpointStatus` reports what the guard accepted and why
it clamped — a guard that clamps silently produces unexplainable field runs.

Interface stability is CI-enforced: a field change needs a version bump and a migration note.

### 7.2 Path engine — stays Python, by decision

~8 000 lines: DXF parsing, CRS/geodesic conversion, arc chains, per-line extensions,
segment-order optimisation. It runs **once per mission upload**, never in the loop.

Its bug history is the argument: the georef north-scale bug (explicitly marked *do not
re-fix*), sparse-arc G0 joints, the must-hit densify bug, the fitter discarding stakes
≤1.5 m. Rewriting it in C++ buys latency nobody needs and risks re-opening bugs that cost
field days to close.

- **Geometry primitives shared with the controller** → `dyx3_geometry` (C++), because the
  controller needs them at rate and both sides must agree exactly.
- **CAD / CRS / DXF ingestion and planning** → Python, in the backend, behind a service
  boundary, emitting a **versioned, content-hashed path artifact** carried into the run manifest.

This is a deliberate exception to "C++ everywhere", and it is justified: C++ is required
where determinism and rate matter, not as an identity.

**GATE 3.** Any geometry function existing on both sides must be proven numerically
equivalent against the Python implementation over the archived mission corpus before the
C++ version enters the loop.

### 7.3 `dyx3_geometry`

Pure library, no ROS, no allocation on hot paths:

```text
distance · angle_wrap · heading_delta · segment_heading · perpendicular_distance
project_onto_segment · project_onto_path · line_intersection
path_length · cumulative_lengths · resample · curvature_at · max_preview_curvature
```

Every one has a direct ancestor in `rpp_controller_node.py`, and every one gets a test whose
fixtures come from **recorded bag data**, not hand-written expectations.

> `CLAUDE.md` records that three bugs shipped because a test's "truth" mirrored the bug.
> Fixture-from-evidence is the countermeasure, and it is mandatory here.

Sign conventions are frozen once in `docs/contracts/frames.md` — cross-track positive =
RIGHT — and referenced everywhere. The memory index contains multiple sign-error incidents.

### 7.4 `dyx3_rpp` — decomposing the monolith

**VERIFIED:** `src/rpp_controller_node.py` — 6 285 lines, one class, ~90 methods,
**120 declared parameters**. `_control_loop_impl` spans 652 lines; `_control_segment_profile`
spans 486.

The decomposition is read out of the existing method groupings. It describes structure
already latent in the file, not a design imposed on it.

| C++ module | Absorbs |
|---|---|
| `path_conditioner` | `_simplify_path_for_profile`, `_dp_mark_keep`, `_simplify_with_indices`, `_split_runs_by_flag`, `_runs_collinear`, `_merge_collinear_runs`, `_is_short_transit_run`, `_absorb_short_connectors`, `_split_run_at_corners`, `_resample_path`, `_smooth_corners`, `_build_poses` |
| `run_sequencer` | `_install_mission`, `_apply_run`, `_advance_run`, `_next_run_turn`, `_next_run_requires_alignment`, `_drain_pending_mission` |
| `guidance` | `_segment_lookahead_point`, `_get_lookahead_point`, `_corner_clip`, `_path_curvature_at`, `_walk_path_samples`, `_max_preview_curvature`, `_pivot_intercept_heading` |
| `speed_profile` | `_apply_smooth_speed_slew`, `_alignment_accel_scale`, `_update_kappa_hard_latch`, `_reset_kappa_hard_latch`, `_corner_brake_velocity`, `_align_speed_ok` |
| `stop_pivot_fsm` | `_precise_stop_ready`, `_segment_endpoint_precise_stop_tick`, `_hold_before_run_advance`, `_hold_at_completion`, `_run_alignment_hold`, `_point_hold_tick`, `_point_handshake_ready`, `_corner_pivot_velocity`, `_corner_stop_satisfied`, `_pivot_timeout_budget`, `_pivot_timed_out`, `_stop_latch_filter`, `_endpoint_capture_recovered` |
| `terminal` | `_run_remaining_along`, `_goal_tol_effective`, `_run_min_travel`, `_is_closed_run`, `_measure_tail_transit_m`, `_run_tail_is_transit` |
| `spray_gate` | `_segment_spray_active`, `_publish_spray_active`, `_gate_spray` |
| `motion_output` | `_publish_velocity`, `_publish_yaw_rate`, `_publish_zero`, `_clamp_velocity_to_forward_cone` — **rewritten, not ported** |
| `diagnostics` | `_publish_debug`, `_publish_segment_debug`, `_debug_xtrack`, `_emit_progress`, `_update_path_progress` |
| `rpp_node` | ROS wiring, timing, parameter callbacks. Thin. |

**`stop_pivot_fsm` is the crown jewel and the minefield.** Almost the entire bug history
lands in it: dead-band limbo, double-stop / dead-entry / terminal-crawl, run-boundary
miss-and-runaway, endpoint overshoot, square multi-run stall, the corner-extension patch
that was clean but inert. It ports as an **explicit state machine with named states and
logged transitions** — the current implicit flag-soup is what made those bugs invisible.

**What disappears:** the ENU↔NED conversion (`_enu_pose_to_ned`, `yaw_NED = π/2 − yaw_ENU`,
the `vel.x = v_e, vel.y = v_n, vel.z = −v_d` output flip). `px4_msgs` is NED/FRD natively.
A whole class of sign bug goes with it — **but every one of the 120 parameters was tuned in
the old frame, and deleting the conversion silently re-interprets all of them.** GATE 4.

**What improves by accident:** `/fmu/out/vehicle_local_position` carries `xy_reset_counter`
and `ref_lat/ref_lon`. Today a `px4-dxp` restart shifts the local frame 38 cm and there is a
heuristic `test_ekf_reset_compensation.py`. With the counter exposed, EKF reset handling
becomes **principled**.

### 7.5 `dyx3_rpp_legacy` — the quarantined oracle

This package is why the precision win arrives before the rewrite.

`rpp_controller_node.py` is carried into the new workspace **as-is**, as an `ament_python`
package, with exactly one change: the output stage publishes `MotionSetpoint` instead of
`TwistStamped`. That is a change to `_publish_velocity` / `_publish_yaw_rate` — not a rewrite.

It serves two purposes:

1. **It delivers the precision payoff in Stage B**, on control logic with dozens of field
   days behind it, months before the C++ port completes.
2. **It is the shadow-run oracle.** The C++ modules are validated against it, per tick, in
   the same graph, on the same inputs. You cannot measure equivalence against something you
   cannot execute.

Quarantine terms, written down so it cannot become permanent:

- explicitly waived from the "no Python in the control graph" rule, and the waiver names Gate 7
- never installed by `install.sh --production`
- excluded from the release artifact
- **deleted when Gate 7 passes.** Its removal is a tracked deliverable, not a cleanup task.

### 7.6 `dyx3_motion_guard`

The last software authority before PX4. Named for responsibility, not message conversion.

Freshness watchdogs on every input · finite-value checks · speed, yaw-rate, acceleration and
jerk limits · mission-state, E-stop, RTK and arming gates · sequence monotonicity ·
**fail-to-zero with a recorded reason code**.

It never invents a correction. If RPP is wrong, the guard stops the rover; it does not steer.

### 7.7 `dyx3_px4_link`

The only package permitted to touch `/fmu/**`. DDS session lifecycle and agent liveness ·
`px4_msgs` version handshake, **fail loud** · offboard heartbeat with margin over PX4's
≥2 Hz requirement · arm/disarm/mode via `VehicleCommand` · rover setpoint publication per
§5.2 · state fan-out into one `VehicleState`.

The `>0.5 s` offboard-gap failsafe rule survives unchanged and is enforced **here**, close to
the wire, not in RPP.

### 7.8 `dyx3_spray`

Marking is a real-time actuator subsystem with geometric semantics. Ports the existing FSM,
safety lease, independent watchdog, manual override, RTK gate, flow model and session config
(**54 parameters**).

Carries one known-open defect **explicitly**:

> **Spray boundary gap — OPEN.** 4 of 18 runs lost 23–40 cm; `projection.s` jumps the MARK
> boundary on out-and-back paths; fix `b385712` was **inert**. Nozzle offset 1.6–6.6 cm also open.

Port it as "works, with a known projection-continuity defect, and here is the test that
reproduces it." A migration is the right moment to give a long-open bug a permanent
regression test.

The **independent safety watchdog stays a separate process** — it must survive the spray
controller dying.

### 7.9 `dyx3_recorder`

Field evidence is the project's actual currency.

```text
runs/2026-09-05_141530_mission_0042/
├── rosbag2/  ├── ulog/          # pulled over the service plane
├── manifest.json                # run, vehicle, operator, mission
├── versions.json                # stack SHA, px4_msgs SHA, firmware SHA, overlay hash
├── params_ros.json              # every ROS parameter, start and end
├── params_fcu.json              # every FCU parameter, read live
├── config_snapshot/  └── summary.json
```

> **A run without provenance is not evidence.** The memory index is full of retracted claims
> built on documented parameter values that did not match the vehicle: `EKF2_WENC_NOISE`
> documented 0.1/0.1 vs 0.35/0.35 vs live 0.10/0.10; `RO_YAW_RATE_LIM` documented 30, actual
> 22; `EKF2_WENC_GATE` documented 3, actual 5.0. The recorder exists to end that class of
> error structurally.

Runs as its own service; keeps recording when the backend dies.

### 7.10 `dyx3_system_gateway` and the backend

```text
Backend ──► validate ──► ROS service/action
ROS telemetry ──► canonical snapshot ──► Unix domain socket ──► Backend
```

Backend keeps REST, Socket.IO, auth, mission upload/report, telemetry delivery, settings,
storage, NTRIP profile management, and path/CAD ingestion (§7.2). It loses `rclpy`, ROS
executors, direct PX4 commands, and the RTK client as a child process.

Backend E-stop is a **request**. Authority lives in `dyx3_motion_guard` and PX4.

---

## 8. Real-time design

A genuine C++ payoff. The current `px4-dxp.service` already runs `SCHED_FIFO` 80 pinned to
CPU 4, commented *"Reduces timer jitter from ±20 ms to ±2 ms, enabling 250 Hz control loop."*
Python cannot honour that at 250 Hz with a 6 000-line loop body.

For `dyx3_rpp` and `dyx3_motion_guard`: `mlockall(MCL_CURRENT|MCL_FUTURE)` · **no heap
allocation in the control loop** — pre-sized buffers, no vector growth, no string formatting
on the hot path · no logging syscalls in the loop; diagnostics via a lock-free ring drained
by a non-RT thread · deterministic executor, control on its own callback group and thread,
telemetry and parameters on a lower-priority executor · CPU affinity and RT priority declared
per node in the unit file · DDS QoS declared per topic — setpoints `BEST_EFFORT` +
`KEEP_LAST(1)`, commands and mission transitions `RELIABLE`.

A `fastdds_no_shm.xml` exists today for a reason. Re-derive that reason; neither inherit nor
drop the file blindly.

Loop timing is **measured and recorded** — publish jitter and overrun counts into `RppStatus`
so a field bag can answer "was the loop healthy?" without a rebuild.

---

## 9. Parameter architecture

**VERIFIED:** 120 `declare_parameter` calls in the RPP node, 54 in the spray node.

Today, runtime changes do not survive an `rpp-pipeline` restart — and a restart mid-mission
in OFFBOARD **aborts the run** (2026-08-01, 0.70 m/s; `NAV_RCL_ACT`/`NAV_DLL_ACT` are both
Disarm). Tuning by restart is not merely inconvenient here. It is unsafe.

| Class | Changeable | Examples |
|---|---|---|
| `LIVE` | while driving | lookahead, gains, speed/yaw-rate limits, terminal thresholds, pivot tuning |
| `IDLE_ONLY` | disarmed, no mission | vehicle geometry, antenna offsets, wheel track, mode defaults |
| `RESTART` | never at runtime | PX4 IP, DDS transport, ROS domain, interface, backend port, paths |

Every parameter carries its class in its descriptor. An `IDLE_ONLY` change during a mission
is **rejected with a reason**, never silently deferred.

Every callback validates before accepting: range, finiteness, cross-parameter consistency
(`min_corner_speed ≤ brake_velocity_cap`), vehicle limits, mission-state legality.

Every accepted or rejected change is recorded as
`timestamp · node · parameter · old → new · source · mission_id · accepted · reason`
into the bag and manifest. *"What was active at 14:22:31?"* must be answerable from the
recording alone.

Profiles (`production.yaml`, `precision.yaml`, `development.yaml`) are promoted by **explicit
save**. A live tweak never silently rewrites production config.

The frozen corner-stop defaults from `CLAUDE.md` seed `production.yaml` — carried over
verbatim, then **re-validated**, because §7.4 deletes the frame they were tuned under.

---

## 10. Migration method

No file is translated. Every component passes:

```text
existing behaviour
  → extract contract from source + field bags   →  docs/contracts/<component>.md
  → implement clean C++ module
  → unit test with fixtures taken from recorded data
  → bag replay: new vs old, numeric equivalence
  → shadow run: new computes, dyx3_rpp_legacy drives
  → field A/B against a named baseline
  → accept or revert
```

**Contract-first.** `docs/contracts/` records what a component does, what it may assume, its
known defects, and the evidence behind each claim — written **before** the C++ and reviewed
against the Python.

**Shadow running is not optional for RPP.** §7.5 exists to make it possible.

**Every gate is a number.** "It builds" and "it drove" are not gates. The bar:

| Metric | Bar |
|---|---|
| Full-mission RMS (`tracking.overall`) | ≤ named baseline, quoted with p95 and max |
| Shape RMS @0.35 m/s | arc 1.46 / lshape 0.90 / square 0.87 / U-turn 1.06 cm |
| Pivot wobble radius, median | ≤ 0.50 cm |
| Pivot net walk, median | ≤ 0.83 cm |
| Traversal coverage | 100 % |
| Spray boundary loss | no regression vs the open 23–40 cm defect |

> Quote **full-mission RMS + p95 + max**. Never `geometry.xtrack_vs_planned`.

---

## 11. Sequencing — two parallel tracks, one variable per change

**Revised 2026-09-05.** Firmware leads. Software refactor runs beside it, not behind it.

### 11.0 Gates are acceptance gates, not start gates

The earlier revision read as though nothing could be *written* until the firmware work
passed. That was wrong, and it is corrected here:

> **You may build any component at any time.
> You may not *accept* a stage without its number.**

This keeps the property that matters — R0, attribution — while unblocking everything that
does not actually depend on hardware. A C++ module can be written, unit-tested against
recorded fixtures, and replay-validated long before there is a vehicle to run it on. What it
cannot do is be declared correct without a field number.

Two tracks run concurrently and converge at integration:

```text
TRACK F — Firmware & Bridge          TRACK S — Software Stack
(hardware-gated, leads)              (desk work, starts immediately)

F1  firmware rebase + all patches    S1  workspace, CI, interfaces, param classes
F2  flash production candidate       S2  legacy port-in + contracts extraction
F3  DDS bridge service, always-on    S3  geometry + path conditioner (replay-tested)
F4  mission-mode test + basic tune   S4  guidance / speed / stop-pivot FSM
F5  full offboard test suite         S5  motion guard + px4_link
         └──────────────┬──────────────────────┘
                        ▼
              I1  integration on the vehicle
              I2  shadow run vs dyx3_rpp_legacy
              I3  field A/B → accept or revert
```

Track S never waits on Track F for **construction**. It waits on Track F only for
**acceptance** — and its own gates (GATE 3 geometry equivalence, replay divergence) are
measurable at a desk against archived bags.

---

### Track F — Firmware and bridge

**F1 · Production firmware candidate.** Rebase the fork onto `v1.17.0` as real commits
(§3.8), re-anchoring by semantic diff against the v1.16.2 ancestor — not by copying files.
Everything that must be in the flashed binary goes in here, once:

```text
WENC EKF2 fusion (13 files)          lever-arm fix 255453d967
RoboClaw QPPS + timestamp fix        1d82e616f8
GNSS-yaw innovation floor            EKF2_GPS_YAW_N + EKF2_GPS_YAW_G (F2 v2)
logger topics                        wheel_encoders, aid-source instances
rover module overlays                RoverDifferential, DifferentialVelControl
rover.px4board                       + CONFIG_MODULES_UXRCE_DDS_CLIENT=y   (§3.4)
dds_topics.yaml                      + /fmu/in/gps_inject_data             (§5.4.1)
dds_topics.yaml                      + ulog_stream / ulog_stream_ack       (§5.4.3, optional)
```

- **GATE 2** — replay neutrality against the existing corpus, *before* flashing.
- Build artifact carries an **overlay/rebase hash** so the vehicle can identify itself (§3.8).

**F2 · Flash and estimator verification.** On the **CubeOrange+** first, MAVROS unchanged, so
the estimator change is isolated from the transport and hardware changes.
**Gate:** pivot wobble ≤ 0.50 cm median, net walk ≤ 0.83 cm — same methodology as the 34-ulog
baseline. Validate RTCM-over-DDS with `gps status` and a FIX transition, never topic presence.

**F3 · DDS bridge service.** `dyx3-px4-link.service` — the always-on connection, the DDS-era
successor to `px4-dxp.service`. Session lifecycle, agent supervision, `px4_msgs` handshake,
reconnect, liveness. Runs on TELEM2 serial first (§3.4), Ethernet after the 6X.
**Gate:** survives FCU reboot, cable pull, agent restart, and Jetson reboot without manual intervention.

**F4 · Mission-mode validation and basic tune.** PX4's own AUTO mission mode, driven from QGC,
on the new firmware. This exercises the estimator, the rover controllers and the drivetrain
**without any companion control code in the path** — the cleanest possible read on whether
the firmware itself is healthy.
**Gate:** mission completes; tracking and pivot behaviour consistent with the F2 numbers.

**F5 · Offboard test suite.** The acceptance battery for the command interface, run from a
minimal harness rather than the full stack: speed + heading, speed + yaw-rate, zero-speed
pivot, stop, arm/disarm, offboard entry/exit, `>0.5 s` gap failsafe, watchdog fail-to-zero.
- **GATE 1** — the `OffboardControlMode` flag combination (§3.3), resolved here.
- **Go/no-go gate** — arc RMS via the yaw-rate path vs the F2 baseline on the same shape and
  speed. **This is where the programme's central claim holds or does not.**

---

### Track S — Software stack

**S1 · Workspace.** Repository, `ament_cmake` skeletons, `.clang-format`, `.clang-tidy`, CI
(build + test + lint + interface check + reject bags/secrets/backups), systemd skeletons,
installer skeleton, `dyx3_interfaces` frozen, all 174 parameters classified LIVE /
IDLE_ONLY / RESTART with owners. **Start today. It blocks nothing and is cheap only now.**

**S2 · Legacy port-in and contract extraction.** `dyx3_rpp_legacy` (§7.5) enters the
workspace with its output stage rewired to `MotionSetpoint`. In parallel, `docs/contracts/`
is written from the Python source and the field bags — the contracts are the specification
the C++ is built against, and writing them surfaces the undocumented behaviour early.

**S3–S5 · C++ modules.** `dyx3_geometry` (**GATE 3** — numeric equivalence against Python
over the archived corpus, measurable at a desk) → `path_conditioner` → `guidance` →
`speed_profile` → `stop_pivot_fsm` → `terminal` → `spray_gate`. `dyx3_motion_guard` is built
**before** the thing it protects and fault-injection tested. `dyx3_px4_link` grows from the
F3 minimal bridge into the full package.

Replay validation against archived missions — including specifically the missions that
exposed the historic failures — runs continuously and needs no vehicle.

---

### Integration

**I1** Deploy the C++ graph alongside `dyx3_rpp_legacy` on the vehicle.
**I2** Shadow run — new computes, legacy drives, per-tick divergence measured over full missions.
**I3** Field A/B against the named baseline.
- **GATE 4** — frame conversion gone; all 120 parameters re-validated in NED; shape RMS
  against §10's bar.
- **GATE 7** — full-mission equivalence. On pass, `dyx3_rpp_legacy` is **deleted** (§7.5).

**Stage C (hardware)** — Pixhawk 6X, Ethernet, hotspot service, BLE, LTE dongle — slots in
after F5 and before I3. It is isolated on purpose: it re-measures `EKF2_IMU_POS_X`, re-verifies
`GPS_YAW_OFFSET` (open bug B1) and antenna geometry (open bug B3), and re-maps AUX1. Its gate
is *match the F5 numbers on the new hardware*.

---

### Stage 0 — still first, still cheap

Three things stay ahead of everything, because they cost days and protect months:

**0.1 Quantify the payoff.** The floor is `err ≈ ω/RO_YAW_P`, `RO_YAW_P=1.5`. Compute the
predicted arc improvement from the **existing arc bags**. If it predicts 1.46 → ~0.5 cm the
programme is unambiguously correct; if ~1.2 cm it is still right for maintainability and
upgrade path, but it is sold and sequenced differently. **The cheapest possible test of the
central hypothesis.**

**0.2 Re-base the analysis toolchain** for the DDS timebase (§4.6). Every gate in both tracks
is unevaluable without it.

**0.3 Close the open baseline.** The `RO_YAW_RATE_TH` 1.0→0.5 A/B, the `RO_YAW_RATE_P` 0.17
backfire (still live, still unresolved), three unexplained failures, the spray boundary gap.

> **You cannot A/B against a baseline you do not understand.** Each open question becomes an
> unattributable ghost in the new stack — and F2's gate is a comparison against exactly that
> baseline.

**0.4** F2 v2 no longer needs a separate flash — it folds into F1, since F1 is a production
candidate rather than an experiment.

---

---

### Sizing and critical path

| Item | Effort | Gates | Note |
|---|---|---|---|
| Stage 0 | S–M | — | days of desk work; protects months |
| S1 workspace | M | — | start today, blocks nothing |
| F1 firmware candidate | **L** | GATE 2 | the single biggest firmware cost |
| F2 flash + verify | S | pivot wobble | isolates the estimator |
| F3 DDS bridge service | M | reconnect survival | |
| F4 mission-mode tune | S | firmware health | no companion code in the path |
| **F5 offboard suite** | **S–M** | **GATE 1 + go/no-go** | **smallest step, largest decision** |
| S2 legacy + contracts | M | — | unblocks all of S3–S5 |
| S3–S5 C++ modules | **XL** | GATE 3, replay | the long pole; runs concurrently throughout |
| Stage C hardware | M | match F5 | |
| I1–I3 integration | L | GATE 4, GATE 7 | ends with `dyx3_rpp_legacy` deleted |

The critical path is **F1 → F2 → F3 → F5**, and it is short. The long pole is **S3–S5**,
which runs beside it the whole way. That is the point of the two-track structure: the
decision that could stop the programme arrives early and cheaply, while the work that takes
longest is never idle waiting for it.

---

## 12. Services, filesystem, operator commands

```text
dyx3-platform.service    network, XRCE agent, MAVLink sidecar, devices, dirs, permissions
        │
        ▼
dyx3-ros.service         the production control graph
     ┌──┴────┬─────────────┐
     ▼       ▼             ▼
dyx3-rtk  dyx3-backend  dyx3-recorder
```

`dyx3-rtk` is a **sibling, not a child** of the backend — the structural fix for the
NTRIP-dies-on-deploy defect. RTK and recorder keep running when the backend dies. The control
graph stays safe when anything above it dies.

```text
/opt/dyx3/{current -> releases/x.y.z, releases/, bin/}
/etc/dyx3/{vehicle,rpp,motion_guard,px4,rtk,spray}.yaml + backend.env
/var/lib/dyx3/{missions,runs,bags,reports,state}/
/var/log/dyx3/   /run/dyx3/
```

Secrets (`ntrip.env`, machine tokens) live in `/etc/dyx3` as `root:dyx3 0640`. Never in Git.

```bash
dyx3-install --production   # idempotent; non-zero exit on any failed health check
dyx3-upgrade <release>      # stop → install → verify → switch symlink → start → health
dyx3-rollback               # previous known-good release
dyx3-health                 # platform, graph, DDS, PX4, RTK, spray, backend, recorder, disk
dyx3-version                # stack SHA, firmware SHA, px4_msgs SHA, OVERLAY HASH, profile
dyx3-param get|set|save     # via the real ROS parameter authority
```

`dyx3-version` must print the **firmware overlay/rebase hash**. §3.8 explains why a PX4
version string is not enough — and why Stage A fixes it at the source.

---

## 13. Risk register

| # | Risk | Severity | Mitigation |
|---|---|---|---|
| **R0** | **Six variables change together; a regression is unattributable** | **Critical** | §11 — one variable per stage, numeric gate between each. The single most important control in this document. |
| R1 | EKF2 re-anchor compiles but degrades estimation | Critical | GATE 2 replay; field pivot-drift re-measure; do not touch `EKF2_WENC_CTRL` in the same A/B window |
| R2 | Frame-conversion removal silently re-interprets 120 tuned parameters | Critical | GATE 4; full re-validation; shadow run before authority |
| R3 | Migration lands on the naive `trajectory_setpoint` path, keeping the floor | High | GATE 1 before Stage B; §5.1 is the acceptance criterion, not the transport |
| R4 | `px4_msgs` version drift → silent no-data | High | startup handshake, fail loud; SHA in manifest; CI pin |
| R5 | Timebase change invalidates the toolchain, so no gate is measurable | High | Stage 0.2 — toolchain re-base is prerequisite work, not cleanup |
| R6 | Stop/pivot FSM port re-opens closed bugs | High | explicit named states; replay against the archived *failure* missions specifically |
| R7 | Open baseline questions become ghosts in the new stack | High | Stage 0.3 — close them first |
| R8 | `dyx3_rpp_legacy` becomes permanent | Medium | quarantine terms in §7.5; deletion is a tracked Gate 7 deliverable |
| R9 | RTK regression during transport migration | Medium | Option B (MAVLink) retained as rollback; Option C (§5.4.4) removes the coupling entirely; RTK becomes an independent service |
| R10 | 6X swap invalidates mount geometry mid-estimator-change | Medium | Stage C is isolated and re-measures everything |
| R11 | Two vehicles, one prefix, accidental coupling | Medium | `dyx3_` prefix; no shared interfaces until earned |
| R12 | Serial DDS bandwidth limits F3/F5 | Low | minimal topic set; documented, temporary, resolved by Stage C |
| **R13** | **Operator-link loss is not a PX4 failsafe** — no router means a tablet dropout leaves `NAV_DLL_ACT` unfired while the rover keeps driving | **High** | §4.3.1 — Jetson owns the tablet heartbeat timeout; `dyx3_motion_guard` executes a controlled stop. New behaviour, designed in Stage E, not ported. |
| **R14** | **RTCM-over-DDS fails silently** — corrections publish but never reach the GNSS module (fragmentation contract; community-reported) | Medium | §5.4.1 — validate on `gps status` and a FIX transition, never topic presence; MAVLink path retained as rollback |
| **R15** | Single radio carries tablet, QGC and SSH; AP+STA attempted on one chipset | Medium | §4.3 — LTE dongle serves WAN so station mode is never needed; verify sustained AP support before committing |

---

## 14. Non-negotiables

- **One variable per stage.** A numeric gate between each.
- Transport and command interface migrate **together**.
- RPP owns path-level speed and rotational intent. PX4 owns the inner loop. Neither infers.
- Nothing bypasses `MotionSetpoint`. Nothing downstream of RPP invents motion.
- `dyx3_motion_guard` fails to zero, always, with a recorded reason.
- The backend is never a safety authority and never imports `rclpy`.
- No production Python in the control graph — one time-boxed waiver, with a deletion gate.
- Every tunable has exactly one owner and a declared class.
- Normal tuning never requires a relaunch, because a relaunch mid-mission aborts the run.
- Every run records stack SHA, firmware SHA, overlay hash, `px4_msgs` SHA, and both parameter
  sets. **A run without provenance is not evidence.**
- No file is translated blind. Contract first, replay second, shadow third, field last.
- `PX4_DXP` is **read-only evidence**. Nothing is edited there.
- Firmware sources are `Vetri2425/PX4-Autopilot` @ `main`. ⛔ Never `~/px4-rover-build`.
- Firmware is **rebased, never `cp`-overlaid**. The build must be identifiable from the vehicle.
- MAVLink leaves the control path. It does not leave the vehicle — parameters, QGC and log
  recovery keep it (§5.4).
- The rover carries its own network. No router, no site LAN, no dependence on operator WiFi.
- Operator-link loss is the Jetson's failsafe to own. PX4 cannot see it.
- BLE is discovery and status. It is never the only stop path, and never replaces a physical E-stop.
- RTK health is validated on `gps status` and FIX transitions — never on topic presence.
