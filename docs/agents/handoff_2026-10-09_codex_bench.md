# Handoff to Codex — continue today's bench session (2026-10-09, from ~08:50 IST)

You are taking over a live bench session on the **DYX 3WD road-marking rover**:
- Pixhawk 6X + Jetson Orin Nano on a Holybro baseboard;
- UM982 GNSS on TELEM1, RoboClaw on GPS2, spray on FMU PWM OUT 1.

The owner is at the rover. Saturday 2026-10-10 is the field demo: driving only, no paint.

## Read first
1. `~/Vetri/3WD_PROD/BENCH_2026-10-09.md`: the runbook. Follow it in order from step 3c.
2. `~/Vetri/3WD_PROD/DYX_3WD/CLAUDE.md` §3b (status) and §4 (rules).
3. `~/Vetri/3WD_PROD/DYX_3WD/docs/agents/HANDOFF.md`: the 2026-10-09 entries.
4. `~/Vetri/3WD_PROD/HANDOFF_PROMPT_2026-10-09.md`: full context, and the open work list (sections B and C).
5. Stall procedure: `~/Vetri/3WD_PROD/OPUS or ChatGPT report for Firmware stall/2026-10-09_eth_tx_stall_procedure.md`.

## Owner rules (binding)
- **Every fix made on the rover must also land in the repo** (installer, unit, template, `config/px4/` params, or a
  documented step) and be recorded in HANDOFF the same day. No temporary hand edits.
- **One authoritative branch per repo:**
  - DYX_3WD `master`; firmware `dyx-3wd-production`; app Three_Wheel_v2 `main`.
  - Work on a topic branch.
  - Merge into the authoritative branch only after the checks pass AND the owner says OK.
  - **Never delete branches.**
- **The firmware is final (V1)**: no firmware changes today unless the bench finds a defect. If it does, stop and
  report to the owner.
- Commits: no AI attribution; trailer `Agent: Codex`; stage files by explicit path; never force-push. Never commit
  secrets (NTRIP, Wi-Fi PSK, tokens).
- **Never configure the UM982.** Read-only observation only; the Jetson writes only CRC-valid RTCM3.
- Rover motion: only with the owner present, the RC kill switch tested, and the wheels off the ground for the first
  tests. Spray stays disabled.

## Current state (verified by Claude at 08:42–08:50)
| Item | State |
|---|---|
| FCU firmware | **`8279fa4be3` flashed at 08:35** (V1 final). `ver git`: PX4 `8279fa4be3`, NuttX `e462af8eb3`. Archive `~/Vetri/3WD_PROD/PX4-Firmware/3WD/8279fa4be3-…` |
| Rover stack | **`/opt/dyx3/current` = `8af2595`** (DYX_3WD master), upgraded at 08:42 in 31 s; health OK, all 6 services active |
| Stall | The upgrade restarted the XRCE agent (1st restart on the new firmware): **no stall**. The 60-restart stress test has NOT been run yet |
| RTK | Worker `INJECTING`, NTRIP → PX4_DDS (imported from ntrip.env, rev 1); 0 CRC failures. 81 transport failures happened only in the startup window, before the px4_link handshake, and stayed flat after. PX4 `gps status`: RTCM 6.16 Hz, 0 dropped, 0 short writes |
| App | Three_Wheel_v2 `main` @ `dbb2ba1`; signed APK `~/Vetri/3WD_PROD/App-Releases/dbb2ba1-agy-prod-transport/app-release.apk`. Uninstall the old debug-signed build first. A tablet token must be created (command below) |
| Stall skipped | The "before" reproduction on old firmware was skipped (owner chose to flash first) |

## Access and tools
- Jetson: `ssh dyx-3wd` (user `flash`, NOPASSWD sudo).
  - IP 192.168.1.32 on the office LAN. If ssh hangs, run `echo 192.168.1.32 > ~/.ssh/.dyx-3wd.lastip`; mDNS returns
    nothing.
  - Or `ssh -o ProxyCommand=none -o HostKeyAlias=dyx-3wd flash@192.168.1.32`.
- PX4 console over USB (`/dev/cu.usbmodem01`, keep QGC closed):
  `python3 -I ~/Vetri/3WD_PROD/bench_tools/nsh.py /dev/cu.usbmodem01 "uxrce_dds_client status" "gps status"`
  (MAVLink SERIAL_CONTROL; any NSH commands as arguments).
- RTK worker status on the Jetson:
  - `scp ~/Vetri/3WD_PROD/bench_tools/rtkstat.py dyx-3wd:/tmp/`
  - then `ssh dyx-3wd 'sudo -u dyx3 python3 /tmp/rtkstat.py'`
- Packet capture on the Jetson (tcpdump is now installed). Run it as a transient unit so it survives ssh:
  `sudo systemd-run --unit=dyx3-bench-capture --property=RuntimeMaxSec=900 /usr/bin/tcpdump -i enP8p1s0 -nn -e -s0 -U -w /home/flash/bench_2026-10-09/<name>.pcap ether host de:cd:81:ec:a0:52`
  - Live view: `sudo tcpdump -i enP8p1s0 -nn -e 'ether src de:cd:81:ec:a0:52 and not icmp'`.
  - PX4 MAC: `de:cd:81:ec:a0:52`, PX4 IP: 10.41.10.2.
- Tablet token (prints once; never log or commit it):
  `sudo -u dyx3 /opt/dyx3/current/venv/bin/python -m dyx3_backend.auth.tokens create --file /var/lib/dyx3/state/auth.json --name tablet-1 --role operator`,
  then `sudo systemctl restart dyx3-backend`.
- Firmware flash (only if the owner asks for a rollback):
  `python3 ~/Vetri/3WD_PROD/PX4-3WD-ethfix/Tools/px_uploader.py --port /dev/cu.usbmodem01 <file.px4>`.
  Fallbacks: `4393fb07e1`, `9ab2ad3162`.

## Today's remaining sequence (runbook §3c onward)
1. **Stall stress test.** Rover disarmed.
   - Capture running.
   - 30× `sudo systemctl restart dyx3-platform` with `sleep 3`, at `MAV_2_CONFIG 1000`.
   - After each restart, PX4 must show `Running, connected` and PX4-originated frames must resume.
   - Then set `MAV_2_CONFIG 0`, reboot the FCU, run 30 more, and restore 1000.
   - Then 3 FCU power cycles and 3 Jetson reboots.
   - **Pass = zero stalls.** On a stall: run the NSH snapshot immediately, again 75 s later, then reboot. See the procedure doc.
2. **mavlink-router server-mode re-test** (runbook 3e): QGC on TCP 5760 must regain the heartbeat after 3 router
   restarts. Then fix `deployment/network/mavlink-router.conf.tmpl` to match (persistence rule).
3. **RTK, 4 combinations × 10 min with 2 min gaps, outdoors** (runbook §4). Switch with the API or
   `SET_CONFIG` (keep `revision`).
   - First collect read-only: the `/dev/serial/by-id/*` names, UM982 USB COM vs TELEM1 COM (must differ), USB baud,
     GGA on USB, LoRa model and baud.
   - Record FLOAT/FIXED times, ages, counters.
   - Prove there is no `/dyx3/rtcm` traffic in USB mode.
4. **Wi-Fi:**
   - `nmcli device`, `iw reg get`, `lsmod | grep -i rtl`;
   - fill in `/etc/dyx3/hotspot.env` (owner picks the SSID/PSK; rover-local only);
   - re-run the installer network step;
   - range test at 5/10/15/25 m on both bands (installer/README.md);
   - then set `backend.env` back to `10.42.0.1` if the hotspot works, and persist it.
5. **App on the tablet:** install the signed APK, connect, telemetry and staleness, E-stop assert/clear, heartbeat
   loss → rover stop. Upload one small app-planned mission, check `GET /api/missions/{sha}`, start / pause / abort.
6. **Calibration, then motion:**
   - gyro; level-only accel (`commander calibrate accel quick` or `PREFLIGHT_CALIBRATION` param5=4); level horizon;
   - Acro wheels-up (direction/sign, rate, pivot);
   - one short mission on the ground;
   - E-stop and tablet-loss stop;
   - valve-close test (decides whether Saturday can paint).
7. **Go / no-go for Saturday**, then write the HANDOFF entry in DYX_3WD.

## Persistence debt to fix in the repo today (topic branch, owner OK to merge)
1. **tcpdump was hand-installed** on the Jetson: add it to the installer's APT package list (`installer/lib/dependencies.sh`).
2. **`ros.env`:** `ROS_DOMAIN_ID=42` and `ROS_LOCALHOST_ONLY=1` must be the template default, plus a health check
   that compares the domain with the PX4 parameter.
3. **`backend.env` 0.0.0.0** → 10.42.0.1 once the hotspot is proven.
4. **mavlink-router mode** from the re-test result.
5. **A documented or scripted step that applies `config/px4/3wd_6x_carry_from_proto.params` to a new FCU.** Add
   today's calibration and parameter changes to that baseline.
6. **The NTRIP credential creation step** in installer/README.

## Report format for the owner
After each step: what you ran, the result, the evidence (numbers or file paths), what failed, and what changed in
the repo. Never claim a pass you did not observe.
