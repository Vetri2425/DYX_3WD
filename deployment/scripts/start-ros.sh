#!/usr/bin/env bash
# DYX 3WD — dyx3-ros launcher: the production control graph (mission, motion_guard, px4_link, spray, system_gateway).
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=dyx3-env.sh
. "${HERE}/dyx3-env.sh"
dyx3_env_load || exit 1
exec ros2 launch dyx3_bringup control_graph.launch.py config_dir:="${DYX3_CONFIG_DIR:-/etc/dyx3}"
