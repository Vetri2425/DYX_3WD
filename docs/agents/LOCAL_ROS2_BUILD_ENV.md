# Local ROS 2 Humble build/test environment

**ROS 2 is available on the Mac through a container, even though it is not installed natively.**
Do not report ROS tests as unavailable because `colcon` is missing on macOS. Run:

```bash
./tools/dev/ros2_humble.sh build-test      # full workspace build + colcon test + test-result
```

CI on `ubuntu-24.04-arm` remains the authoritative integration gate; this reproduces its
`ros2_build_test` job locally (`.github/workflows/ci.yml`).

## What it is

| | |
|---|---|
| Runtime | Colima (Lima VM, `vz`, native aarch64) profile `dyx3-ros2`, Docker CLI via `brew install colima docker` |
| Image | `dyx3-ros2-humble:local`, built from `ros:humble-ros-base` (Ubuntu 22.04, arm64) + `ros-humble-mavros-msgs`, build tools, rsync (`tools/dev/Dockerfile.ros2-humble`) |
| Docker client | Isolated: `DOCKER_CONFIG=~/.dyx3-docker`, `DOCKER_HOST=unix://~/.colima/dyx3-ros2/docker.sock`. Does not touch other docker contexts or credential helpers. |
| Default VM size | 6 CPU / 12 GB RAM / 80 GB disk (override `DYX3_CPUS`, `DYX3_MEM_GB`, `DYX3_DISK_GB` before the first start) |
| Intel Macs | Would build amd64: not an exact match of the arm64 CI. Say so in any report. |

Persistent state (survives sessions, container exit, `stop`, Mac reboot) lives in Docker volumes:

- `dyx3-px4-msgs` -> `/opt/dyx3/px4_msgs/<firmware-sha>/install` : firmware-pinned px4_msgs overlay.
- `dyx3-ws` -> `/work/repo` : Linux copy of the working tree with `ros2_ws/{build,install,log}` kept between runs.

Logs: `build/local-ros2-logs/{build,test,test-result}-<UTC stamp>.log` (git-ignored).

## Commands

```bash
./tools/dev/ros2_humble.sh setup          # start runtime, build image, build px4_msgs overlay (first time only is slow)
./tools/dev/ros2_humble.sh build          # sync source + colcon build --symlink-install
./tools/dev/ros2_humble.sh test           # colcon test + test-result --verbose (non-zero on any failure)
./tools/dev/ros2_humble.sh build-test     # both
./tools/dev/ros2_humble.sh test-pkg dyx3_mission dyx3_px4_link   # build+test selected packages
./tools/dev/ros2_humble.sh shell          # interactive shell in the sourced workspace
./tools/dev/ros2_humble.sh exec <cmd>     # e.g. exec colcon test --packages-select dyx3_rpp --retest-until-pass 1
./tools/dev/ros2_humble.sh status
./tools/dev/ros2_humble.sh stop           # stop the VM; deletes nothing
```

Every run prints the exact host commit SHA and whether the tree has uncommitted changes. The working
tree (committed, modified and untracked files) is rsynced into the Linux volume, so unpushed work is
what gets tested. Excluded: `.git`, host `build/`, `ros2_ws/{build,install,log}`, root-level `*.patch`.

## px4_msgs overlay

Built by the repository's own `build_px4_msgs` (`installer/lib/ros_install.sh`): px4_msgs skeleton at
`PX4_MSGS_SKELETON_REF`, `msg/` + `srv/` replaced from `FIRMWARE_SHA` in `installer/pins/firmware.pin`.
Never stock px4_msgs. The overlay carries `firmware.sha`, `px4_msgs.sha256` and `.complete`.
A stamp of `firmware.pin` + `ros_install.sh` is kept; if either changes, the overlay is wiped and
rebuilt at the next `build`/`setup`. (A new `FIRMWARE_SHA` also lands in a new directory.)

## Updating / limitations

- Image changes: edit the Dockerfile, then `docker rmi dyx3-ros2-humble:local` (with the env vars the
  script sets) and rerun `setup`. Nothing is pruned automatically.
- Not covered locally: real-time timing, DDS transport to PX4, systemd, installer on target, RTK.
- Only the `ros2_build_test` job is reproduced. Native geometry, formatting, backend and installer
  checks run on the Mac directly (installer tests need GNU coreutils/findutils first on PATH).
- No credentials or machine IDs are stored here.
