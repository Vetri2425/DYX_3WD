#!/usr/bin/env bash
# DYX 3WD installer — health checks. Source, do not execute.
# shellcheck shell=bash
#
# Output: one PASS / WARN / FAIL line per check. FAIL makes health_run return non-zero.
# WARN is for things that can legitimately be off during an upgrade (FCU powered down).
# Deep checks (DDS topics) need the ROS environment and a live FCU.

# shellcheck source=rover_state.sh
. "${INSTALLER_DIR}/lib/rover_state.sh"

_health_fail=0
_pass() { printf 'PASS  %s\n' "$*"; }
_warn() { printf 'WARN  %s\n' "$*"; }
_fail() {
  printf 'FAIL  %s\n' "$*"
  _health_fail=1
}

# health_release <release-dir>: a built release is internally complete (no services needed).
health_release() {
  local rel="$1" require_complete="${2:-1}"
  if [ "${require_complete}" = 1 ]; then
    [ -f "${rel}/.complete" ] && _pass "release ${rel##*/} marked complete" || _fail "release ${rel##*/} not complete"
  fi
  [ -f "${rel}/ros2_ws/install/setup.bash" ] && _pass "ros2_ws install present" || _fail "ros2_ws/install/setup.bash missing"
  [ -x "${rel}/bin/dyx3-platform" ] && _pass "bin/dyx3-platform executable" || _fail "bin/dyx3-platform missing"
  [ -f "${rel}/bin/dyx3-env.sh" ] && _pass "bin/dyx3-env.sh present" || _fail "bin/dyx3-env.sh missing (no ROS launcher can start)"
  local pm
  pm="$(px4_msgs_dir)"
  if [ -f "${pm}/.complete" ]; then
    _pass "px4_msgs built for firmware ${FIRMWARE_SHA:0:10} (msgs sha256 $(cut -c1-12 "${pm}/px4_msgs.sha256" 2>/dev/null))"
  else
    _fail "px4_msgs for firmware ${FIRMWARE_SHA:0:10} not built"
  fi
}

health_platform() {
  local rel="${1:-${DYX3_CURRENT}}"
  if systemd_available; then
    local name state
    while IFS= read -r name; do
      [ -n "${name}" ] || continue
      state="$(systemctl is-active "${name}.service" 2>/dev/null || true)"
      if [ "${state}" = "active" ]; then _pass "${name}.service active"; else _fail "${name}.service is '${state}'"; fi
    done < <(manifest_section enabled_services "${rel}/installer/manifests/production.manifest")
  else
    _warn "systemd checks skipped"
  fi
  # shellcheck disable=SC1090
  [ -r "${DYX3_ETC}/platform.env" ] && . "${DYX3_ETC}/platform.env"
  local port="${DYX3_XRCE_PORT:-8888}" px4="${DYX3_FCU_PX4_IP:-10.41.10.2}"
  if have ss; then
    if ss -H -lun "sport = :${port}" 2>/dev/null | grep -q .; then
      _pass "XRCE agent listening on udp/${port}"
    else
      _fail "nothing listening on udp/${port} (XRCE agent down)"
    fi
  else
    _warn "ss not available; agent port not checked"
  fi
  if have ping; then
    if ping -c1 -W1 "${px4}" >/dev/null 2>&1; then _pass "FCU ${px4} reachable"; else _warn "FCU ${px4} not reachable (powered? cable?)"; fi
  fi
}

# _enabled <service>: is it in the manifest's [enabled_services]?
_enabled() { manifest_section enabled_services "${2:-${DYX3_CURRENT}/installer/manifests/production.manifest}" 2>/dev/null | grep -qx "$1"; }

# health_extras: the parts beyond the platform. Each check is gated on the service being ENABLED in the manifest, so an
# unverified service never turns the health red; disk is always reported.
# _settle <function> [args...]: poll a check once a second for up to DYX3_HEALTH_SETTLE_S (default 30): health runs
# right after an upgrade restarted the services, and the gateway socket / backend need seconds to come up (rover 2026-10-08).
_settle() {
  local n=0 limit="${DYX3_HEALTH_SETTLE_S:-30}"
  until "$@"; do
    n=$((n + 1))
    [ "${n}" -ge "${limit}" ] && return 1
    sleep 1
  done
}

_gateway_up() { [ -S "${DYX3_RUN}/gateway.sock" ]; }
_backend_up() { curl -fsS --max-time 3 "http://$1:$2/api/ping" >/dev/null 2>&1; }
_rtk_up() {
  [ -S "${DYX3_RUN}/rtk-control.sock" ] || return 1
  python3 - "${DYX3_RUN}/rtk-control.sock" <<'PY' >/dev/null 2>&1
import json
import socket
import sys

with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as sock:
    sock.settimeout(3)
    sock.connect(sys.argv[1])
    sock.sendall(b'{"v":1,"cmd":"GET_STATUS"}\n')
    reply = b''
    while not reply.endswith(b'\n') and len(reply) <= 65536:
        chunk = sock.recv(4096)
        if not chunk:
            break
        reply += chunk
    message = json.loads(reply)
    assert message.get('v') == 1 and message.get('ok') is True
PY
}

# _backend_env <KEY>: one value from /etc/dyx3/backend.env (what the service really binds), never executed.
_backend_env() { sed -n "s/^$1=//p" "${DYX3_ETC}/backend.env" 2>/dev/null | tail -n1; }

health_wifi() {
  if [ -f "${DYX3_ROOT}/etc/NetworkManager/system-connections/dyx3-hotspot.nmconnection" ]; then
    local wifi_iface reg_state power_state
    wifi_iface="$(sed -n 's/^interface-name=//p' "${DYX3_ROOT}/etc/NetworkManager/system-connections/dyx3-hotspot.nmconnection" | head -n1)"
    if have iw; then
      reg_state="$(iw reg get 2>&1 || true)"
      power_state="$(iw dev "${wifi_iface}" get power_save 2>&1 || true)"
      _pass "Wi-Fi regulatory domain (iw reg get): ${reg_state//$'\n'/; }"
      _pass "Wi-Fi ${wifi_iface} power save: ${power_state//$'\n'/; }"
    else
      _warn "iw unavailable; Wi-Fi regulatory domain and power save not checked"
    fi
  fi
}

health_extras() {
  local rel="${1:-${DYX3_CURRENT}}" m="${1:-${DYX3_CURRENT}}/installer/manifests/production.manifest"
  health_wifi
  if declare -F health_usb_serial >/dev/null 2>&1; then health_usb_serial; fi
  if _enabled dyx3-ros "${m}"; then
    if _settle _gateway_up; then _pass "gateway socket present"; else _fail "gateway socket ${DYX3_RUN}/gateway.sock missing (dyx3-ros / system_gateway down?)"; fi
  fi
  if _enabled dyx3-rtk "${m}"; then
    if _settle _rtk_up; then _pass "dyx3-rtk control socket answers GET_STATUS"; else _fail "dyx3-rtk control socket unavailable"; fi
  fi
  if _enabled dyx3-backend "${m}" && have curl; then
    local host port
    host="${DYX3_BACKEND_HOST:-$(_backend_env DYX3_BACKEND_HOST)}"
    host="${host:-10.42.0.1}"
    port="${DYX3_BACKEND_PORT:-$(_backend_env DYX3_BACKEND_PORT)}"
    port="${port:-8000}"
    # a wildcard bind is reached on loopback
    case "${host}" in 0.0.0.0 | "::") host=127.0.0.1 ;; esac
    if _settle _backend_up "${host}" "${port}"; then _pass "backend answers on ${host}:${port}"; else _fail "backend does not answer on ${host}:${port}"; fi
  fi
  if _enabled dyx3-recorder "${m}" && [ ! -d "${DYX3_VAR_LIB}/runs" ]; then _fail "runs directory ${DYX3_VAR_LIB}/runs missing"; fi
  # disk: report; FAIL only when the data volume is completely full (no threshold is invented here).
  if have df; then
    local line avail pct
    line="$(df -Pk "${DYX3_VAR_LIB}" 2>/dev/null | awk 'NR==2 {print $4, $5}')"
    avail="${line% *}"
    pct="${line#* }"
    if [ -n "${line}" ]; then
      if [ "${avail:-0}" -le 0 ]; then _fail "disk ${DYX3_VAR_LIB} is full"; else _pass "disk ${DYX3_VAR_LIB}: $((avail / 1024)) MiB free (${pct} used)"; fi
    fi
  fi
}

# health_graph: deep. The control graph's nodes are visible (WARN, not FAIL: needs ROS and a running graph).
health_graph() {
  local rel="${1:-${DYX3_CURRENT}}" pm nodes
  _enabled dyx3-ros "${rel}/installer/manifests/production.manifest" || return 0
  [ -f "${ROS_SETUP}" ] || {
    _warn "ROS not installed; graph check skipped"
    return 0
  }
  pm="$(px4_msgs_dir)"
  nodes="$(bash -c "set +u; . '${ROS_SETUP}'; . '${pm}/install/setup.bash'; . '${rel}/ros2_ws/install/setup.bash' 2>/dev/null; timeout 15 ros2 node list 2>/dev/null" || true)"
  local n
  for n in /dyx3_mission /motion_guard /px4_link /spray /system_gateway; do
    if printf '%s\n' "${nodes}" | grep -qx "${n}"; then _pass "node ${n} up"; else _warn "node ${n} not visible"; fi
  done
}

# health_dds: live /fmu topics (needs ROS + a running FCU session).
health_dds() {
  local rel="${1:-${DYX3_CURRENT}}" pm n
  pm="$(px4_msgs_dir)"
  if [ ! -f "${ROS_SETUP}" ]; then
    _warn "ROS not installed; DDS check skipped"
    return 0
  fi
  n="$(bash -c "set +u; . '${ROS_SETUP}'; . '${pm}/install/setup.bash'; . '${rel}/ros2_ws/install/setup.bash' 2>/dev/null; timeout 15 ros2 topic list 2>/dev/null | grep -c '^/fmu/'" || true)"
  if [ "${n:-0}" -gt 0 ]; then _pass "DDS: ${n} /fmu topics visible"; else _warn "DDS: no /fmu topics visible (FCU session not up?)"; fi
}

# health_run [--deep] [release-dir]
health_run() {
  local deep=0
  if [ "${1:-}" = "--deep" ]; then
    deep=1
    shift
  fi
  local rel="${1:-${DYX3_CURRENT}}" pins
  # The release's own pin (INS-003): a revert or rollback across a firmware-pin change checks its own px4_msgs.
  pins="$(_pins_dir_of "${rel}")"
  local PINS_DIR="${pins}"
  _health_fail=0
  load_pin firmware
  health_release "$(readlink -f "${rel}")"
  health_platform "${rel}"
  health_extras "${rel}"
  if [ "${deep}" -eq 1 ]; then
    health_dds "${rel}"
    health_graph "${rel}"
  fi
  if [ "${_health_fail}" -eq 0 ]; then
    log "health: OK"
    return 0
  fi
  log "health: FAILED"
  return 1
}
