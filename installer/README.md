# installer — Phase 11a minimal deployment slice

**Status:** code only. Tested against a staged root with fake tools (`installer/tests/run_tests.sh`);
**never run on a Jetson.** The local Claude verifies on the rover.

## Commands

| Command | Implemented | What it does |
|---|---|---|
| `installer/install.sh --production [--ref R] [--skip-deps] [--dry-run]` (`dyx3-install`) | 11a | OS check → `dyx3` user → `/opt/dyx3`,`/etc/dyx3`,`/var/lib/dyx3`,`/var/log/dyx3` → tmpfiles for `/run/dyx3` → apt deps → pinned MicroXRCEAgent → pinned mavlink-router → ROS 2 Humble → FCU Ethernet profile → first release (same path as upgrade). Idempotent. |
| `installer/upgrade.sh <git-ref>` (`dyx3-upgrade`) | 11a | fetch → `releases/<sha>` → colcon build (manifest packages) → static verification → `.complete` → atomic `current` symlink → install units → restart enabled services → health. Failed health removes `.complete` and records `.failed`; an upgrade restores the previous release and a first install stops its services and removes `current`. |
| `installer/verify.sh [--deep]` (`dyx3-health`) | 11a (platform only) | release complete, px4_msgs built for the pinned firmware, enabled units active, XRCE agent listening, FCU ping (WARN only), `--deep`: live `/fmu` topics. Phase 11 extends it to the rest of the graph. |
| `installer/rollback.sh` (`dyx3-rollback`) | 11 | switch back to the release recorded at the last switch → verify it first → units, shims, `versions.json` → restart → health; an unhealthy rollback restores the release it started from; running it twice undoes it. Refuses when nothing is recorded or the previous release was pruned. |
| `installer/version.sh` (`dyx3-version`) | 11 | `key=value` lines: stack SHA, previous release, **pinned** firmware SHA, `px4_msgs` message-set sha256, profile. The running FCU's own identity is reported as `unavailable` (no FCU read path yet; architecture 3.8's overlay hash is OPEN). |
| `dyx3-param get\|set\|save` | not built | needs `dyx3_rpp`'s parameter authority (P5) |

## Prebuilt release artifacts (no compiling on the rover)

Green CI on `master` (and `claude/cloud-phases` while it is the deploy branch) builds the release on arm64
(`ros:humble-ros-base`, the Jetson's Ubuntu 22.04 + Humble userland) with the same `build_px4_msgs` /
`build_release`, and publishes GitHub Release `rover-<sha>`: `release-<sha>.tar.zst`,
`px4_msgs-<firmware-sha>.tar.zst`, `artifacts.env` (provenance), `SHA256SUMS`. Last 20 kept.

`dyx3-install` / `dyx3-upgrade` try it first: download → every file checked against `SHA256SUMS` → OS/arch/
ROS/firmware-pin/stack-SHA match → members must stay under their `opt/dyx3/...` prefix → extract → the usual
verify / `.complete` / switch / health. `versions.json` records `build_origin` (the CI run, or "built on this
machine").

| Variable | Default | Meaning |
|---|---|---|
| `DYX3_ARTIFACTS` | `auto` | `auto`: prebuilt, else build here · `prebuilt`: fail without a valid artifact · `source`: always build here |
| `DYX3_ARTIFACT_DIR` | — | take the files from a directory (USB stick, no WAN) instead of downloading |
| `DYX3_COLCON_WORKERS` | `1` | packages built in parallel when building here (CI uses `nproc`) |

Upgrade after the CI run for that commit is green; before that there is no artifact and `auto` builds here.
Integrity today is SHA-256 over GitHub TLS (human decision 2026-10-08); signing is a pre-delivery item
(proposal §7).

## Pins (one place each)

`installer/pins/{microxrce_agent,mavlink_router,firmware}.pin`. Changing `FIRMWARE_SHA` makes the next
upgrade rebuild `px4_msgs` from that SHA's `msg/`+`srv/` (stock px4_msgs mismatches — the firmware
modifies `EstimatorAidSource3d.msg`). The message-set identity (`px4_msgs.sha256`) is recorded next to
the build for the `dyx3_px4_link` handshake and the run manifest.

## Layout it creates

```
/opt/dyx3/releases/<sha>/        git archive + ros2_ws/{build,install} + bin/dyx3-<service> + .complete
/opt/dyx3/current -> releases/<sha>
/opt/dyx3/bin/dyx3-{install,upgrade,health}   shims exec'ing the CURRENT release's scripts
/opt/dyx3/px4_msgs/<firmware-sha>/            shared by all releases; built once, -j1 by default
/etc/dyx3/{platform.env,mavlink-router.conf}  created once, NEVER overwritten by an upgrade
/var/lib/dyx3/state/{repo.git,previous_release}
```

## Build load

`DYX3_BUILD_JOBS` defaults to **1** (the Orin Nano stalled at `-j3` building px4_msgs). Raise it only
after watching memory on the rover. Builds run under `nice -n 10`.

## What the staged tests do not prove

apt, the ROS apt repo, the pinned XRCE agent / mavlink-router builds, NetworkManager, systemd, udev,
real DDS to the FCU, and the supervisor under systemd. See HANDOFF "LOCAL ACTIONS NEEDED".

## CH340 / CH341 USB serial provisioning

The production installer provisions the QinHeng `1a86:7523` bridge through
`installer/lib/usb_serial.sh` during both fresh installation and upgrades. The currently validated
kernel is **Jetson Linux L4T 36.5.0, `5.15.185-tegra`, aarch64**, with the matching NVIDIA
`nvidia-l4t-kernel-headers` tree. The running kernel config has `CONFIG_USB_SERIAL=m` and no
`CONFIG_USB_SERIAL_CH341`; the installer builds the versioned `ch341-dyx3` source in
`installer/drivers/ch341-dyx3-1.0.0/`, copied from the public NVIDIA L4T 36.5.0 source archive.
`PROVENANCE` records archive and source SHA-256 values. The module is built directly (kbuild) against
the exact installed headers with the existing build-essential GCC 11 toolchain and installed to
`/lib/modules/<kernel>/extra/ch341-dyx3/` with a source-hash stamp; it is rebuilt only when missing or
the source changes. **DKMS is not used**: on the rover, installing it pulled gcc-12 and upgraded 11
system libraries, which this installer refuses. If exact matching headers are missing, the installer
simulates installation of that header version and refuses any transaction that upgrades or
replaces the NVIDIA kernel. A new kernel is rejected
with a specific incompatibility message until its matching source, headers, build and live binding
are validated. No kernel package is replaced by this flow.

Provisioning requires exactly one attached `1a86:7523` bridge. It reads that rover's `ID_PATH` from
udev and records it in `/etc/dyx3/ch341-adapter.env`; it never copies a physical-port mapping from
another rover. If the installed BRLTTY vendor rule contains its generic `1a86:7523` match, the
installer shadows (does not edit) it in `/etc/udev/rules.d/85-brltty.rules` and adds an exclusion for
only the detected `ID_PATH`. An existing unmanaged override is preserved and causes a precise stop.
The installer verifies module metadata, kernel vermagic, sysfs binding, `/dev/ttyUSB*`, and `dialout`
ownership. The enabled `dyx3-usb-serial-check.service` provisions and verifies on activation and
every boot. Keeping it in the production manifest also bootstraps the first upgrade from an older
installed release: the existing release manager installs and starts newly manifested units after
the release switch. `dyx3-health` reports a missing provisioned adapter as a failure.

After the change is merged and CI publishes the matching full-SHA rover artifact, require the
prebuilt path during the production upgrade:

```bash
sudo DYX3_ARTIFACTS=prebuilt /opt/dyx3/bin/dyx3-upgrade <full-sha>
```

This prevents a ROS 2/application rebuild on the Jetson. The application artifact is built in CI;
The provisioning step separately compiles the pinned CH341 source against the rover's exact running-kernel headers.

This identifies the USB bridge, not its UM982 COM function. Do not write
`DYX3_UM982_USB_ID_PATH` until the physical USB-to-UM982 wiring has been inspected and the selected
COM is confirmed distinct from PX4 TELEM1. After that confirmation, rerun the installer with
`DYX3_UM982_USB_ID_PATH=<detected-ID_PATH>` in its environment. It records the matching stable
`/dev/serial/by-path/...` path in `/etc/dyx3/usb-receiver.env`; it does not change RTK config or
send bytes to the receiver. Confirm supported baud and RTCM input capability from the correct
receiver/carrier documentation before configuring USB_DIRECT.

Rollback removes only the installer-managed artifacts:

```bash
sudo bash /opt/dyx3/current/installer/usb_serial_uninstall.sh
sudo reboot
```

The script refuses to remove an unmanaged BRLTTY override. Reboot restores the vendor BRLTTY rule
and unloads the ch341 module once it is removed. The uninstaller does not alter the receiver, PX4 firmware,
PX4 parameters, or RTK config. Staged tests cover missing/ambiguous adapter, unsupported kernel,
BRLTTY scoping, unmanaged override preservation, source/provenance staging, missing module, missing
binding, successful health state, receiver-identity gating, and repeat-run idempotence. These checks
do not establish live Jetson behavior; current-rover driver deployment still requires the separate
owner approval and hardware validation recorded in the bench runbook.


## Phase 11 additions

### Tablet access and token

The optional NetworkManager profile `dyx3-hotspot` uses the onboard Wi-Fi device as an access
point at the per-rover `DYX3_HOTSPOT_ADDRESS`.
- Fleet plan (2026-10-09): `192.168.3.100/24` for the first 3WD, `192.168.3.101/24` for the next.
- A blank address falls back to `10.42.0.1/24`. An address on the FCU subnet `10.41.10.0/24` is refused.

The device is `DYX3_HOTSPOT_IFACE`, or, when that is blank, the first Wi-Fi device that is not on USB.
A USB Wi-Fi dongle is the rover's internet uplink (client mode) and is never chosen automatically.
Bind the uplink's client profile to the dongle, for example
`sudo nmcli connection modify "<uplink profile>" connection.interface-name <dongle iface>`.
The installer warns if any Wi-Fi client profile is bound to the access-point device. It also turns
the Wi-Fi radio on, because a radio left off (`WirelessEnabled=false`) keeps the device
"unavailable". Set `DYX3_BACKEND_HOST` in `/etc/dyx3/backend.env` to the same address so the app
reaches the backend over the hotspot. NetworkManager's `shared` IPv4 mode provides tablet DHCP. The profile
has no default route and no bridge. A NetworkManager pre-up hook installs firewall DROP rules
between that Wi-Fi interface and the FCU Ethernet interface in both directions. The Wi-Fi
hardware and AP support still need a rover bench check. NetworkManager's shared mode can
forward to other default routes, such as LTE; the FCU link is explicitly blocked.

`/etc/dyx3/hotspot.env` is created once from an empty template (`root:dyx3`, mode `0640`) and
never overwritten. Set unquoted `DYX3_HOTSPOT_SSID` (1–32 ASCII letters, digits, `.`, `_`, `-`)
and `DYX3_HOTSPOT_PSK` (8–63 characters from the template's supported ASCII punctuation and
letters/digits). Leave either blank to disable the access point. The installer skips profile
creation when no Wi-Fi device is present. It writes the secret only to a root-owned `0600`
NetworkManager keyfile; it never sends the passphrase in a command argument or log.
Set `DYX3_WIFI_COUNTRY` to the operating country's two-letter code (template: `IN`).
The installer applies it at boot through `dyx3-wifi-regdom.service`. Set
`DYX3_WIFI_BAND=a` (5 GHz, default) or `bg` (2.4 GHz), an optional
`DYX3_WIFI_CHANNEL` (default 36 or 6 respectively), and `DYX3_WIFI_WIDTH=20|40`
(default 20). Only non-DFS 5 GHz channels 36–48 and 149–165 are accepted.
NetworkManager before 1.50 uses its safe 20 MHz auto width; 40 MHz is refused
on those versions. The profile uses WPA2-PSK only and disables Wi-Fi power save.
If the installed RTL8822CE driver exposes a known power-save option through
`modinfo`, the installer also writes a matching modprobe option for the next boot.

Bench check after filling the env file and rerunning the installer or upgrading:

```bash
nmcli device
sudo nmcli connection up dyx3-hotspot
# Connect the tablet to the configured SSID, then from the tablet:
curl http://<hotspot address>:8000/api/ping   # e.g. 192.168.3.100
iw reg get
iw dev <if> get power_save
```

For the 15-minute bench range check, test both `a` and `bg` bands. At 5, 10,
15, and 25 m, keep the tablet connected for 15 minutes and record continuous
ping loss/latency to the hotspot address, the app's telemetry age, and RSSI from
`iw dev <if> station dump` on the rover. Record any reconnects and the actual
channel/width; rerun the installer after changing the band in `hotspot.env`.

The bench backend's existing `DYX3_BACKEND_HOST=0.0.0.0` in `/etc/dyx3/backend.env` is left
as configured.

Create a tablet operator token in the backend's production auth store with:

```bash
sudo -u dyx3 /opt/dyx3/current/venv/bin/python -m dyx3_backend.auth.tokens create --file /var/lib/dyx3/state/auth.json --name tablet-1 --role operator
sudo systemctl restart dyx3-backend
```

The first command prints the token once; record it privately for the tablet. The auth store
contains only its hash and has mode `0600`. Run this before a mission because the backend loads
the store at startup and a restart interrupts the tablet connection.

* **Services** (`[services]` in the manifest): `dyx3-platform`, `dyx3-ros` (mission, motion_guard, px4_link, spray, system_gateway via `dyx3_bringup/control_graph.launch.py`),
  `dyx3-rtk`, `dyx3-backend`, `dyx3-recorder`, and **`dyx3-spray-watchdog` as its own unit** (not tied to `dyx3-ros`, so it survives the graph dying).
  `[enabled_services]` lists **all six** since 2026-10-08, when each was verified running on the 3WD rover. An enabled service that fails its environment
  makes the post-upgrade health check revert the whole upgrade, so a new service joins the list only after it has run on the rover.
* **Environment**: `/etc/dyx3/{ros,backend,ntrip}.env` templates are created once and never overwritten (`ntrip.env` is `root:dyx3 0640`). `ROS_DOMAIN_ID` has no default: the launchers refuse to start without it. `dyx3-env.sh` also refuses to start without the px4_msgs overlay built for the pinned firmware.
  Existing `ntrip.env` files must be migrated explicitly to set `DYX3_NTRIP_SECURITY=PLAINTEXT` or `TLS`; no port-based or legacy default is applied. Optional `DYX3_NTRIP_CA_FILE` supplies a private TLS CA PEM. TLS uses system trust paths when that value is absent.
* **`/etc/dyx3/versions.json`** is rewritten on every switch/rollback; the recorder copies it into every run.
* **Backend venv** (`<release>/venv`) is built with the release; a failed `pip install` (no WAN) is a warning, not a failed upgrade.
* **Health**: gateway socket, backend ping (both only for ENABLED services), data-volume report (FAIL only when completely full: no threshold invented), `--deep` lists the graph's nodes (WARN).
* **Real-time allocation** (DERIVED, 2026-10-09): only `rpp` and `motion_guard` run SCHED_FIFO 80 on CPU 4, through a `taskset`/`chrt` launch prefix in `control_graph.launch.py`; the `dyx3-ros` unit no longer sets a policy or affinity for the whole tree. The unit must keep `LimitRTPRIO` >= 80 (it runs as `dyx3` without CAP_SYS_NICE) and `LimitMEMLOCK=infinity` (mlockall).
* **OPEN**: DDS scoping (loopback-only vs an eth0 whitelist), the backend port (8000, DERIVED), and the ROS domain number.

Tested against a staged root (`installer/tests/run_tests.sh`, 80 checks); **never run on a Jetson**.


### Selecting the UM982 USB link (USB_DIRECT) on a new rover

Proven on the 3WD rover 2026-10-09. Never configure the receiver: only the read-only queries `CONFIG` and
`UNILOGLIST` are sent, and only to identify the port.

1. Provisioning (this installer) binds the CH340 to `ch341`. BRLTTY is masked because its daemon keeps the
   adapter claimed through usbfs. Result: `/dev/ttyUSB0`, `/dev/serial/by-path/<...>:1.0-port0`.
2. Identify the COM, with the RTK worker not using the port:
   `sudo python3 tools/bench/um982_usb_probe.py /dev/serial/by-path/<...>` (passive, every baud), then
   `... --query <baud>`. Compare the passive message set with `UNILOGLIST`. The USB COM must not be the COM
   wired to PX4 TELEM1 (COM1 on this rover).
   This rover: USB = **COM3** (GNGSV + GPVTG appear only there), all COMs 230400 baud. COM3 outputs GGA at
   5 Hz, so the worker reads fix quality and correction age back.
3. Select it through the RTK control API (or socket `SET_CONFIG`, keeping `revision`): `transport=USB_DIRECT`,
   `usb.receiver_device` = the by-path link, `usb.baud` = the COM's baud, `write_timeout_s` 0.2 and
   `reopen_delay_s` 2.0 (both DERIVED; re-validate in the field). The config persists in
   `/var/lib/dyx3/rtk/config.json`.
4. Accept when the worker is `INJECTING` with delivered = valid frames and 0 failures, `/dyx3/rtcm` has no
   publisher traffic, and the receiver readback shows GGA quality > 1 with a valid correction age.
   Measured 2026-10-09 indoors: quality 2 (DGPS), correction age 1.2 s, 0 failures.
