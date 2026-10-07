#!/usr/bin/env bash
# DYX 3WD — dyx3-recorder launcher: field evidence recorder. Keeps recording when the backend dies.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=dyx3-env.sh
. "${HERE}/dyx3-env.sh"
dyx3_env_load || exit 1
exec ros2 run dyx3_recorder recorder_node
