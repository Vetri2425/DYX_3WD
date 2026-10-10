#!/usr/bin/env bash
# DYX 3WD — dyx3-control launcher: the control chain (px4_link, motion_guard, rpp) via dyx3_bringup control_graph.launch.py.
# The XRCE-agent wait lives here only: px4_link is the one node that talks to the agent. dyx3-services (start-services.sh) does not wait.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=dyx3-env.sh
. "${HERE}/dyx3-env.sh"
dyx3_env_load || exit 1

# Availability only, never a gate: wait for the XRCE agent to bind its UDP port, so px4_link does not start its handshake into nothing on
# every boot. DERIVED — NOT FROM V1 SPEC: 10 s total, polled every 0.5 s. The platform restart delay is 2 s (DYX3_RESTART_DELAY_S in
# start-platform.sh) and the agent needs well under that to bind. Whatever happens, the control chain starts: px4_link stays in handshake-pending
# STOP until the agent and the FCU session are up. The port is read, not executed, from platform.env, with the same default as start-platform.sh.
agent_wait_s="${DYX3_AGENT_WAIT_S:-10}"
agent_poll_s=0.5
xrce_port="$(sed -n 's/^DYX3_XRCE_PORT=//p' "${DYX3_PLATFORM_ENV:-/etc/dyx3/platform.env}" 2>/dev/null | tail -n1 || true)"
xrce_port="${xrce_port:-${DYX3_XRCE_PORT:-8888}}"

# agent_listening: a UDP listener on the XRCE port. ss output is captured first: `ss | grep -q` under pipefail can report failure
# when grep exits early.
agent_listening() {
  local out
  out="$(ss -H -lun "sport = :${xrce_port}" 2>/dev/null || true)"
  [ -n "${out}" ]
}

if command -v ss >/dev/null 2>&1; then
  waited_polls=0
  max_polls=$((agent_wait_s * 2))
  until agent_listening; do
    if [ "${waited_polls}" -ge "${max_polls}" ]; then break; fi
    sleep "${agent_poll_s}"
    waited_polls=$((waited_polls + 1))
  done
  if agent_listening; then
    echo "dyx3-control: XRCE agent listening on udp/${xrce_port} after $((waited_polls / 2)).$((waited_polls % 2 * 5)) s; starting the control chain"
  else
    echo "dyx3-control: WARN: nothing listens on udp/${xrce_port} after ${agent_wait_s} s; starting the control chain anyway (px4_link holds STOP until the session is up)"
  fi
else
  echo "dyx3-control: ss not available; not waiting for the XRCE agent on udp/${xrce_port}; starting the control chain"
fi

exec ros2 launch dyx3_bringup control_graph.launch.py config_dir:="${DYX3_CONFIG_DIR:-/etc/dyx3}"
