#!/usr/bin/env bash
# DYX 3WD — dyx3-recorder launcher: field evidence recorder. Keeps recording when the backend dies.
#
# REC-024: exec the recorder_node BINARY, not `ros2 run`. With `ros2 run` the unit's main PID was the Python wrapper, so
# systemd's SIGTERM did not reach the recorder directly and the stop path (bag finalised on SIGINT, end-of-run parameter calls,
# summary.json) depended on the wrapper forwarding it. Now the main PID is the recorder itself; dyx3-recorder.service uses
# KillMode=mixed so only it receives SIGTERM and the `ros2 bag` child it supervises is left to finalise its database.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=dyx3-env.sh
. "${HERE}/dyx3-env.sh"
dyx3_env_load || exit 1

# Resolve the installed binary from AMENT_PREFIX_PATH (the sourced overlays, highest priority first) without starting Python.
recorder_bin=""
IFS=: read -r -a ament_prefixes <<<"${AMENT_PREFIX_PATH:-}"
for prefix in "${ament_prefixes[@]:-}"; do
  if [ -n "${prefix}" ] && [ -x "${prefix}/lib/dyx3_recorder/recorder_node" ]; then
    recorder_bin="${prefix}/lib/dyx3_recorder/recorder_node"
    break
  fi
done
if [ -z "${recorder_bin}" ]; then
  echo "dyx3-recorder: recorder_node not found under AMENT_PREFIX_PATH (is the release's workspace built?)" >&2
  exit 1
fi

# REC-022: optional per-rover parameters (vehicle_id, operator; template config/recorder/recorder.yaml.example). Never mandatory:
# no file means the built-in defaults. A file that exists but cannot be read is an error, not a silent fall back to "unknown".
params_file="${DYX3_RECORDER_PARAMS:-/etc/dyx3/recorder.yaml}"
args=()
if [ -e "${params_file}" ]; then
  if [ ! -r "${params_file}" ]; then
    echo "dyx3-recorder: ${params_file} exists but is not readable by $(id -un)" >&2
    exit 1
  fi
  args=(--ros-args --params-file "${params_file}")
fi

exec "${recorder_bin}" "${args[@]}"
