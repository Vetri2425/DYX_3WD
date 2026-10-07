# installer — Phase 11a minimal deployment slice

**Status:** code only. Tested against a staged root with fake tools (`installer/tests/run_tests.sh`);
**never run on a Jetson.** The local Claude verifies on the rover.

## Commands

| Command | Implemented | What it does |
|---|---|---|
| `installer/install.sh --production [--ref R] [--skip-deps] [--dry-run]` (`dyx3-install`) | 11a | OS check → `dyx3` user → `/opt/dyx3`,`/etc/dyx3`,`/var/lib/dyx3`,`/var/log/dyx3` → tmpfiles for `/run/dyx3` → apt deps → pinned MicroXRCEAgent → pinned mavlink-router → ROS 2 Humble → FCU Ethernet profile → first release (same path as upgrade). Idempotent. |
| `installer/upgrade.sh <git-ref>` (`dyx3-upgrade`) | 11a | fetch → `releases/<sha>` → colcon build (manifest packages) → **verify before switching** → atomic `current` symlink → install units → restart enabled services → health; **reverts automatically** if post-switch health fails. |
| `installer/verify.sh [--deep]` (`dyx3-health`) | 11a (platform only) | release complete, px4_msgs built for the pinned firmware, enabled units active, XRCE agent listening, FCU ping (WARN only), `--deep`: live `/fmu` topics. Phase 11 extends it to the rest of the graph. |
| `dyx3-rollback`, `dyx3-version`, `dyx3-param` | Phase 11 | not yet |

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
