# Bench plan — Friday 2026-10-09 (office, rover on stands)

Goal for the day:
1. Rover stack upgraded to the RTK release.
2. PX4↔Jetson Ethernet stall: reproduced on the old firmware, then proven fixed on the new one.
3. All four RTK source × transport combinations tested.
4. Basic tuning in Acro and Mission (wheels up first, then ground).

Saturday field test = **driving only, no paint**, unless the valve-close test passes.

Credentials: NTRIP profile "office" (Emlid, MP23960). Enter it from the app or the API, never in files or chat logs.

---

## 0. Safety preconditions (every step)
- Wheels off the ground (stands) or drive power isolated for steps 2–3. Spray valve and pump physically disabled.
- RC transmitter on, kill switch tested.
- PX4 **USB console** connected (NSH, not MAVLink-over-Ethernet) for all firmware and stall steps.
- Save every terminal transcript. Do not commit captures to the public repo.

## 1. Upgrade the rover stack (≈ 5 min) — first thing
Release: `claude/cloud-phases` @ **cb8ea42** (RTK production + the two Claude fixes). CI publishes GitHub
release `rover-cb8ea42…`. The branch head is now `a1f7cd4`, a docs-only commit (the persistence rule) with the
same code; either sha works once its CI release exists.

**New rule:** any fix made on the rover today must also land in the repo (CLAUDE.md §4). See the HANDOFF
2026-10-09 list of five existing hand edits.

```bash
ssh dyx-3wd
sudo dyx3-upgrade 8af2595      # master (RTK + app gaps + hotspot); prebuilt artifact, ~20–30 s, health OK
                               # fallback: ad58e72 (no app gaps) or cb8ea42 (RTK only)
cat /etc/dyx3/versions.json    # stack sha cb8ea42…
systemctl status dyx3-rtk --no-pager
ls -ld /var/lib/dyx3/rtk       # drwx------ dyx3 dyx3 (created by systemd StateDirectory)
sudo cat /var/lib/dyx3/rtk/config.json | jq '{source,transport,desired_state,revision}'
#   expected after upgrade: NTRIP + PX4_DDS + RUNNING (imported once from ntrip.env)
```

API check: `GET /api/rtk/status` → `worker_state` INJECTING once NTRIP streams.

If it is not healthy, run `sudo dyx3-rollback` and stop to investigate.

Note: the upgrade **restarts the XRCE agent**, which is exactly the stall trigger. Watch for the stall right here
(step 2's symptoms). If PX4 goes silent, that is the first reproduction: capture it (step 2b) before any reboot.

## 2. Ethernet TX stall — reproduce on OLD firmware 9ab2ad3162 (≤ 60 min, time-boxed)
Full procedure: `~/Vetri/3WD_PROD/OPUS or ChatGPT report for Firmware stall/2026-10-09_eth_tx_stall_procedure.md`.

**2a. Set up captures.**
- Terminal A: `sudo tcpdump -i enP8p1s0 -nn -e -s0 -U -w ~/stall_old.pcap ether host de:cd:81:ec:a0:52`
  - Verify the MAC first with `ip neigh`.
- Terminal B: `sudo tcpdump -i enP8p1s0 -nn -e 'ether src de:cd:81:ec:a0:52 and not icmp'`
  - This is the live "is PX4 originating anything?" view. Ping replies do NOT count.
- Terminal C: the restart loop. 30 cycles of `systemctl restart dyx3-platform` with `sleep 3`, as in the procedure doc.

**2b. On a stall** (terminal B goes silent and stays silent after the agent is back):
- Run on the USB NSH **immediately**, then again **75 s later** (to see whether NuttX's 60 s TX timeout recovers it):
  `ifconfig`, `netstat`, `free`, `uxrce_dds_client status`, `work_queue status`, `top -1`, `mavlink status`, `gps status`.
- Record any self-recovery, then reboot the FCU.

**2c. If there is no stall after 30 cycles:**
- Run one more round of 30 with `MAV_2_CONFIG 1000`, if it isn't already at 1000 (QGC traffic on Ethernet).
- Then stop: the "before" evidence is the two stalls from 2026-10-08.

## 3. Flash the NEW firmware and stress it (≈ 45 min)
Candidate (**V1 final**): **`dyx-3wd-production` @ 8279fa4be3** (`v1.17.0-33-g8279fa4be3`), official CI build
[run 37834754885](https://github.com/Vetri2425/PX4-Autopilot-3WD-Prod/actions/runs/37834754885):
`~/Vetri/3WD_PROD/PX4-Firmware/3WD/8279fa4be3-fix-ekf2-wheel-encoder-fusion-does-not-refresh-the-global-velocity-fusion-timers/px4_fmu-v6x_rover.px4`
- sha256 `21288e6d50e5fab6440932b5508d048bbf38db3c01f14d3942f4986dea1846ee`
- FLASH 1,789,852 B (+600 B vs 9ab2ad3162); AXI SRAM unchanged.
- Adds the WENC timer fix (8279fa4be3) on top of 4393fb07e1. With EKF2_WENC_CTRL 1, watch `estimator_status_flags`
  (inertial dead reckoning) and any yaw reset during the GNSS tests. If anything looks wrong, 4393fb07e1 is
  archived as a fallback.

Check the sha256 before flashing: `shasum -a 256 <file>`.

What changed since 9ab2ad3162 (plus **8279fa4be3**: wheel-encoder fusion no longer refreshes the global
velocity-fusion timers, so the EKF-GSF yaw reset and the dead-reckoning detection work again):
- **58154ff8f1:** NuttX STM32H7 TX-ring guard (the stall fix). The NuttX fork is `Vetri2425/NuttX`.
- **f991ebb98a:** XRCE client closes its fd once on reconnect.
- **4393fb07e1:** GPS RTCM partial writes completed and counted; new `gps status` counters.

`msg/` is unchanged, so the Jetson px4_msgs needs no rebuild.

**3a. Flash.**
- QGC → Vehicle Setup → Firmware → Advanced → custom file, over **USB**.
- Parameters are kept. Then check `ver all` shows `4393fb07e1`.

**3b. Smoke check.**
- DDS session up: `/fmu` topics visible on the Jetson, odometry at 100 Hz.
- QGC over TCP 5760 works.
- `gps status` shows the new RTCM counter lines.

**3c. Stress.**
- Same restart loop as step 2: 30 cycles with `MAV_2_CONFIG 1000`, then 30 with `MAV_2_CONFIG 0`. Terminal B must never stay silent.
- Pass = 60 restarts, zero stalls, DDS back every time.
- Then 3 FCU power cycles plus 3 Jetson reboots, with DDS reconnecting each time.

**3d. Rollback if needed.** If anything regresses, reflash `9ab2ad3162` (archived) and report.

**3e. mavlink-router server-mode re-test.** Was inconclusive on 10-08:
- set `Mode=Server Address=0.0.0.0 Port=14550`;
- restart dyx3-platform 3×;
- QGC on TCP 5760 must regain the heartbeat each time.

## 4. RTK — four combinations, 10 min each, 2 min gap (≈ 50 min, outdoors with sky view)

### CH340/CH341 remediation gate (before any USB_DIRECT selection)

The repository installer supports the exact Jetson/L4T kernel `5.15.185-tegra` from L4T 36.5.0.
Fresh setup and `dyx3-upgrade` provision a version-pinned NVIDIA-source DKMS driver, create a
BRLTTY shadow rule limited to the detected adapter `ID_PATH`, verify `/dev/ttyUSB*` and `dialout`,
and enable `dyx3-usb-serial-check.service` for provisioning and boot verification. The unit is in
the production manifest so the existing release manager starts it after the first upgrade switch,
even when that manager came from an older installed release. Unsupported kernels, missing
matching headers, missing/ambiguous adapters, or failed binding stop provisioning. The driver source
is under `installer/drivers/ch341-dyx3-1.0.0/`; rollback is documented in `installer/README.md`.

The current rover has not received this repository-owned remediation yet. Do not report it fixed
until the owner authorizes deployment and the following read-only acceptance evidence is collected:
`lsusb -t` shows `Driver=ch341`; `modinfo -F vermagic ch341` matches `uname -r`; the adapter
interface's sysfs `driver` symlink resolves to `ch341`; a single stable
`/dev/serial/by-path/...` link resolves to the resulting `/dev/ttyUSB*`; `dyx3` can open it
exclusively; and the `dyx3-usb-serial-check.service` passes after reboot and on a second installer
run. Do not send serial bytes at this stage.

Before recording the receiver path, identify the physical CH340-to-UM982 connection and prove it is
a separate COM from PX4 TELEM1. Only after that inspection, set
`DYX3_UM982_USB_ID_PATH=<the detected ID_PATH>` for an installer run. Verify the selected COM's
baud and RTCM input support from the applicable UM982 carrier/interface documentation. The installer
then records `/etc/dyx3/usb-receiver.env`; it does not select USB_DIRECT or write receiver settings.
Never carry a by-path value from one rover to another.

If BRLTTY's generic match is present, the override lives in `/etc/udev/rules.d/85-brltty.rules` and
excludes only the detected physical path. Driver rollback is `sudo bash
/opt/dyx3/current/installer/usb_serial_uninstall.sh` followed by reboot. No kernel replacement,
PX4 firmware change, or receiver configuration is part of this procedure.

**Before slot 2, gather these** (read-only, never configure the UM982):
- `ls -l /dev/serial/by-id/`. The UM982 USB name and the LoRa radio name must contain only `A-Z a-z 0-9 _ . -`.
- UM982 USB baud. Confirm the USB COM is a **different** COM from the one wired to PX4 TELEM1.
- Whether GGA is output on that USB COM: `sudo timeout 5 cat /dev/serial/by-id/<um982>`, read-only, with the RTK service not using it.
- LoRa radio baud. The base must be transmitting RTCM over LoRa.

Switch with the API (operator token). Example:

```bash
curl -s -H "Authorization: Bearer $TOK" localhost:8000/api/rtk/config | jq .   # note "revision"
# edit transport/source + usb/lora blocks, keep revision, PUT it back:
curl -s -X PUT -H "Authorization: Bearer $TOK" -H 'Content-Type: application/json' \
     -d @rtk_config.json localhost:8000/api/rtk/config | jq '.revision,.source,.transport'
```

| Slot | Source → transport | Watch |
|---|---|---|
| 1 | NTRIP → DDS | `gps status` RTCM counters (dropped bytes, short writes, uORB lost), PX4 injection rate vs `/dyx3/rtcm` rate |
| 2 | NTRIP → USB | `/api/rtk/status` transport counters, `receiver.gga_*`, receiver correction age |
| 3 | LoRa → USB | LoRa read errors and reopens, frames valid vs CRC fail |
| 4 | LoRa → DDS | same as slot 1 |

Record for each slot:
- time to RTK FLOAT and to FIXED;
- correction age (source, delivery, receiver);
- frames in vs delivered;
- errors;
- `worker_state` history (`events`).

During each 2-minute gap: `curl …/api/rtk/status > slotN.json` and `gps status` on the NSH.

Also prove there is no dual injection: in USB mode `/dyx3/rtcm` must be silent (`ros2 topic hz /dyx3/rtcm` shows nothing).

## 5. Calibration and tuning (rest of the day)
**Calibration** (USB NSH or QGC):
- gyro;
- simple accelerometer, level-only: `commander calibrate accel quick` or `PREFLIGHT_CALIBRATION` param5=4;
- level horizon. No magnetometer.

**Acro, wheels up, then ground:**
- RoboClaw direction and sign check;
- rate tracking;
- pivot.

**Mission, ground:**
- a short straight and one corner from a pre-staged mission;
- watch the motion_guard, RPP and mission states;
- E-stop and tablet-loss stop test.

**Spray valve close test** (decides whether Saturday can paint): with the valve ON, kill px4_link, kill the agent, unplug Ethernet, cut Jetson power. Measure the output each time.

## 6. End of day
Write HANDOFF entries in DYX_3WD:
- firmware result;
- RTK slot table;
- calibration;
- the go / no-go for Saturday.
