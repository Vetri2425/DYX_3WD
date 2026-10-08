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


## Phase 11 additions

* **Services** (`[services]` in the manifest): `dyx3-platform`, `dyx3-ros` (mission, motion_guard, px4_link, spray, system_gateway via `dyx3_bringup/control_graph.launch.py`),
  `dyx3-rtk`, `dyx3-backend`, `dyx3-recorder`, and **`dyx3-spray-watchdog` as its own unit** (not tied to `dyx3-ros`, so it survives the graph dying).
  `[enabled_services]` is still **`dyx3-platform` only**: the others are implemented in code but have never run on a rover, and enabling one that fails its
  environment would make the post-upgrade health check revert the whole upgrade. Move a service into `[enabled_services]` once it has been verified on the rover.
* **Environment**: `/etc/dyx3/{ros,backend,ntrip}.env` templates are created once and never overwritten (`ntrip.env` is `root:dyx3 0640`). `ROS_DOMAIN_ID` has no default: the launchers refuse to start without it. `dyx3-env.sh` also refuses to start without the px4_msgs overlay built for the pinned firmware.
  Existing `ntrip.env` files must be migrated explicitly to set `DYX3_NTRIP_SECURITY=PLAINTEXT` or `TLS`; no port-based or legacy default is applied. Optional `DYX3_NTRIP_CA_FILE` supplies a private TLS CA PEM. TLS uses system trust paths when that value is absent.
* **`/etc/dyx3/versions.json`** is rewritten on every switch/rollback; the recorder copies it into every run.
* **Backend venv** (`<release>/venv`) is built with the release; a failed `pip install` (no WAN) is a warning, not a failed upgrade.
* **Health**: gateway socket, backend ping (both only for ENABLED services), data-volume report (FAIL only when completely full: no threshold invented), `--deep` lists the graph's nodes (WARN).
* **OPEN**: per-node real-time priority/affinity (the `dyx3-ros` unit applies FIFO 80 / CPU 4 to the whole tree), DDS scoping (loopback-only vs an eth0 whitelist), the backend port (8000, DERIVED), and the ROS domain number.

Tested against a staged root (`installer/tests/run_tests.sh`, 80 checks); **never run on a Jetson**.
