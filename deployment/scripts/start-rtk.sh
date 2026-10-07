#!/usr/bin/env bash
# DYX 3WD — dyx3-rtk launcher: NTRIP client and RTCM delivery. A SIBLING of the backend, never its child. Credentials come from the environment (EnvironmentFile=/etc/dyx3/ntrip.env), never argv.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=dyx3-env.sh
. "${HERE}/dyx3-env.sh"
dyx3_env_load || exit 1
exec ros2 run dyx3_gnss_rtk rtk_node
