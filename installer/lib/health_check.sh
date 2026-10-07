#!/usr/bin/env bash
# DYX 3WD installer — health checks. Source, do not execute.
# shellcheck shell=bash
#
# Output: one PASS / WARN / FAIL line per check. FAIL makes health_run return non-zero.
# WARN is for things that can legitimately be off during an upgrade (FCU powered down).
# Deep checks (DDS topics) need the ROS environment and a live FCU.

_health_fail=0
_pass() { printf 'PASS  %s\n' "$*"; }
_warn() { printf 'WARN  %s\n' "$*"; }
_fail() {
  printf 'FAIL  %s\n' "$*"
  _health_fail=1
}

# health_release <release-dir>: a built release is internally complete (no services needed).
health_release() {
  local rel="$1"
  [ -f "${rel}/.complete" ] && _pass "release ${rel##*/} marked complete" || _fail "release ${rel##*/} not complete"
  [ -f "${rel}/ros2_ws/install/setup.bash" ] && _pass "ros2_ws install present" || _fail "ros2_ws/install/setup.bash missing"
  [ -x "${rel}/bin/dyx3-platform" ] && _pass "bin/dyx3-platform executable" || _fail "bin/dyx3-platform missing"
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
  local rel="${1:-${DYX3_CURRENT}}"
  _health_fail=0
  load_pin firmware
  health_release "$(readlink -f "${rel}")"
  health_platform "${rel}"
  [ "${deep}" -eq 1 ] && health_dds "${rel}"
  if [ "${_health_fail}" -eq 0 ]; then
    log "health: OK"
    return 0
  fi
  log "health: FAILED"
  return 1
}
