#!/usr/bin/env bash
# dyx3-platform — supervises the Micro XRCE-DDS Agent (data plane) and mavlink-router
# (service plane). Each child is restarted on failure with a fixed delay; the supervisor
# itself is restarted by systemd. On SIGTERM both children are stopped.
#
# This script supervises PROCESSES only. It never configures the FCU, never configures the
# GNSS receiver, and never publishes anything: session liveness and the px4_msgs
# handshake belong to dyx3_px4_link (proposal 2026-10-07).
set -uo pipefail

ENV_FILE="${DYX3_PLATFORM_ENV:-/etc/dyx3/platform.env}"
if [ -r "${ENV_FILE}" ]; then
  # shellcheck disable=SC1090
  . "${ENV_FILE}"
fi
: "${DYX3_XRCE_PORT:=8888}"
: "${DYX3_MAVROUTER_CONF:=/etc/dyx3/mavlink-router.conf}"
: "${DYX3_MAVROUTER_ENABLE:=1}"
: "${DYX3_RESTART_DELAY_S:=2}"
: "${DYX3_AGENT_BIN:=MicroXRCEAgent}"
: "${DYX3_ROUTER_BIN:=mavlink-routerd}"
: "${DYX3_PLATFORM_STATE_DIR:=/run/dyx3}"

log() { printf 'dyx3-platform: %s\n' "$*"; }

pids=()

# BR-003: child restarts are otherwise invisible (the unit stays "active" while a child crash-loops). Each supervisor keeps its
# own count since this unit started, logs every restart with the count and the exit status, and publishes
# ${DYX3_PLATFORM_STATE_DIR}/platform_restarts.<name> (restarts=, last_exit_status=, updated_utc=) with temp file + rename.
# One file per child, not one shared file: the supervisors are separate processes and a shared file would need a lock.
# dyx3-health reads these. A write failure (read-only or missing directory) never affects supervision.
publish_restarts() {
  local name="$1" count="$2" status="$3" f
  f="${DYX3_PLATFORM_STATE_DIR}/platform_restarts.${name}"
  if printf 'restarts=%s\nlast_exit_status=%s\nupdated_utc=%s\n' "${count}" "${status}" "$(date -u +%Y-%m-%dT%H:%M:%SZ)" \
    2>/dev/null >"${f}.tmp.${BASHPID}" && mv -f "${f}.tmp.${BASHPID}" "${f}" 2>/dev/null; then
    return 0
  fi
  rm -f "${f}.tmp.${BASHPID}" 2>/dev/null
  return 0
}

# supervise <name> <cmd...>: run in a loop (as a background subshell) until told to stop.
# The TERM trap lives INSIDE the subshell so the child is stopped and not restarted.
supervise() {
  local name="$1"
  shift
  local child=0 stop=0 restarts=0
  trap 'stop=1; [ "${child}" -ne 0 ] && kill -TERM "${child}" 2>/dev/null' TERM INT
  publish_restarts "${name}" 0 none
  while [ "${stop}" -eq 0 ]; do
    log "${name}: starting"
    "$@" &
    child=$!
    wait "${child}"
    local rc=$?
    # A trapped signal interrupts wait before the child is reaped: reap it now.
    wait "${child}" 2>/dev/null
    [ "${stop}" -eq 1 ] && break
    child=0
    restarts=$((restarts + 1))
    log "${name}: exited rc=${rc}; restart #${restarts} in ${DYX3_RESTART_DELAY_S}s"
    publish_restarts "${name}" "${restarts}" "${rc}"
    sleep "${DYX3_RESTART_DELAY_S}" &
    wait $!
  done
  log "${name}: stopped"
}

shutdown() {
  log "stopping"
  for p in "${pids[@]:-}"; do
    [ -n "${p}" ] && kill -TERM "${p}" 2>/dev/null
  done
  for p in "${pids[@]:-}"; do
    [ -n "${p}" ] && wait "${p}" 2>/dev/null
  done
  exit 0
}
trap shutdown TERM INT

if ! command -v "${DYX3_AGENT_BIN}" >/dev/null 2>&1; then
  log "FATAL: ${DYX3_AGENT_BIN} not found (run dyx3-install)"
  exit 1
fi

supervise xrce-agent "${DYX3_AGENT_BIN}" udp4 -p "${DYX3_XRCE_PORT}" &
pids+=($!)

if [ "${DYX3_MAVROUTER_ENABLE}" = "1" ]; then
  if command -v "${DYX3_ROUTER_BIN}" >/dev/null 2>&1 && [ -r "${DYX3_MAVROUTER_CONF}" ]; then
    supervise mavlink-router "${DYX3_ROUTER_BIN}" -c "${DYX3_MAVROUTER_CONF}" &
    pids+=($!)
  else
    # The data plane must not depend on the service plane: log and carry on.
    log "WARN: mavlink-router unavailable or unconfigured; service plane disabled"
  fi
fi

# Block until stopped. A supervisor only ends on TERM, so reaching the end means shutdown.
wait
