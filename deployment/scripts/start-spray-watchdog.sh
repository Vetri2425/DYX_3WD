#!/usr/bin/env bash
# DYX 3WD — dyx3-spray-watchdog launcher: the independent spray fail-closed watchdog. Its own process and unit so it survives the controller dying.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=dyx3-env.sh
. "${HERE}/dyx3-env.sh"
dyx3_env_load || exit 1
exec ros2 run dyx3_spray spray_watchdog
