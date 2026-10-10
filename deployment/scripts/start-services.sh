#!/usr/bin/env bash
# DYX 3WD — dyx3-services launcher: the non-safety half of the graph (mission, spray, system_gateway) via dyx3_bringup
# services_graph.launch.py, under normal scheduling. No XRCE-agent wait: none of these nodes talks to the agent (px4_link, in
# dyx3-control, does). A crash here restarts this unit only; dyx3-control keeps px4_link, motion_guard and rpp running (STOP).
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=dyx3-env.sh
. "${HERE}/dyx3-env.sh"
dyx3_env_load || exit 1

exec ros2 launch dyx3_bringup services_graph.launch.py config_dir:="${DYX3_CONFIG_DIR:-/etc/dyx3}"
