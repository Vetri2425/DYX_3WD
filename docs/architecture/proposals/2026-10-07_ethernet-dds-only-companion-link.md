# Proposal — Jetson ↔ PX4 is Ethernet + uXRCE-DDS only

**Status:** accepted by the human, 2026-10-07. Implemented in firmware; spec text not yet amended.
**Amends:** §4.2, §4.4, §4.5, §5.4.2, §5.4.5, §11 (F3), §12, §14.

## Decision

```text
Jetson eth 10.41.10.1/24 ── Ethernet ── PX4 eth0 10.41.10.2/24 (static)

uXRCE-DDS   UXRCE_DDS_CFG 1000, UXRCE_DDS_AG_IP 170461697 (10.41.10.1), UXRCE_DDS_PRT 8888
MAVLink     MAV_2_CONFIG 0 by default — no MAVLink instance on Ethernet

NO: TELEM/UART companion transport, MAVROS, a MAVLink control path
USB: maintenance interface only — QGC, parameters, flashing, console
```

DDS owns the entire PX4 ↔ Jetson application link. MAVLink is not on the cable unless a human
turns it on.

## What changes against V1

| V1 says | Now |
|---|---|
| §4.2 two planes on one cable: DDS data plane + MAVLink service plane (sidecar :14540, QGC forward :14550) | One plane on the cable by default. The service plane exists only when `MAV_2_CONFIG=1000` is set as a **live parameter** (`param set` + `param save` + reboot); it persists until a parameter reset. |
| §4.5 `MAV_2_MODE=Onboard`, `MAV_2_REMOTE_PRT=14540`, `MAV_2_BROADCAST=0` | `MAV_2_CONFIG=0` default. When enabled, board defaults apply (Normal, UDP 14550, broadcast on) and `mavlink-router` in `dyx3-platform` forwards it to QGC (TCP 5760). |
| §5.4.2 "Parameters keep MAVLink" | Parameters go over USB. **Accepted consequence:** while MAV_2 is off the Jetson cannot read FCU parameters (v1.17 has no DDS get path), so `dyx3_recorder` cannot snapshot the FCU parameter set automatically. Re-enabling MAV_2 restores it with no code change. |
| §5.4.5 QGC over the Jetson hotspot | QGC over USB by default; over the hotspot only with MAV_2 enabled. |
| §14 "MAVLink … does not leave the vehicle" | MAVLink stays on the vehicle (USB, TELEM1) but leaves the companion cable. |
| §11 F3 `dyx3-px4-link.service` vs §12 `dyx3-platform` running the XRCE agent | `dyx3-platform` supervises the XRCE agent and `mavlink-router`. `dyx3_px4_link` owns session liveness and the `px4_msgs` handshake. No separate `dyx3-px4-link.service`. |

## Hardware facts found during bring-up (Holybro Pixhawk Jetson Baseboard, Orin Nano 8 GB)

- The "direct cable" of §4.3 is the baseboard's on-board RTL8367S switch, which joins the Jetson,
  the Pixhawk and two external Ethernet ports. **Leave the external ports empty in production**;
  a router plugged in there is on the FCU network.
- The switch is powered from the Pixhawk circuit: no POWER1, no Ethernet link (USB alone is not
  enough).
- PX4 netman keeps its configuration in `/fs/mtd_net`, which survives flashing. Firmware
  defaults apply only when it is empty. A board that ever ran DHCP/fallback firmware must be
  switched once with `/fs/microsd/net.cfg` + `netman update -i eth0`.
- The Jetson recovery switch label is ambiguous; in the wrong position the Jetson enumerates as
  USB `APX` 0955:7523 and never boots Linux.

## Firmware

`Vetri2425/PX4-Autopilot-3WD-Prod`, `dyx-3wd-production`:

- `f4f99058c0` — SYS_AUTOSTART 50000, DDS over Ethernet defaults, static 10.41.10.2 (DHCPC
  removed), CI fetches tags so builds report v1.17.0 instead of 0.0.0.
- `27a7ac9284` — `MAV_2_CONFIG 0`. Flashed and verified 2026-10-07: DDS session established,
  timesync converged, 68 `/fmu` topics on the Jetson.
