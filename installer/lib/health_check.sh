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

_svc_prop() { systemctl show -p "$2" --value "$1.service" 2>/dev/null || true; }
_svc_active() { [ "$(_svc_prop "$1" ActiveState)" = active ]; }

# health_services <release-dir>: INS-005. One is-active sample right after a restart let a crash-looping release pass.
# Every enabled unit must reach active (within DYX3_HEALTH_SETTLE_S), then STAY active for DYX3_HEALTH_HOLD_S (default
# 10, sampled every second) with NRestarts unchanged.
health_services() {
  local rel="$1" hold="${DYX3_HEALTH_HOLD_S:-10}" name i
  local -a names=()
  local -A restarts=() gone=()
  mapfile -t names < <(manifest_section enabled_services "${rel}/installer/manifests/production.manifest" | grep .)
  for name in "${names[@]}"; do
    if _settle _svc_active "${name}"; then
      restarts[${name}]="$(_svc_prop "${name}" NRestarts)"
    else
      _fail "${name}.service is '$(_svc_prop "${name}" ActiveState)'"
    fi
  done
  [ "${#restarts[@]}" -gt 0 ] || return 0
  for ((i = 0; i < hold; i++)); do
    sleep 1
    for name in "${!restarts[@]}"; do
      [ -z "${gone[${name}]:-}" ] && ! _svc_active "${name}" && gone[${name}]="$(_svc_prop "${name}" ActiveState)"
    done
  done
  for name in "${names[@]}"; do
    [ -n "${restarts[${name}]+x}" ] || continue
    if [ -n "${gone[${name}]:-}" ]; then
      _fail "${name}.service went '${gone[${name}]}' during the ${hold} s hold"
    elif [ "$(_svc_prop "${name}" NRestarts)" != "${restarts[${name}]}" ]; then
      _fail "${name}.service restarted during the ${hold} s hold (NRestarts ${restarts[${name}]} -> $(_svc_prop "${name}" NRestarts))"
    elif ! _svc_active "${name}"; then
      _fail "${name}.service is '$(_svc_prop "${name}" ActiveState)' after the hold"
    else
      _pass "${name}.service active for ${hold} s with no restart"
    fi
  done
}

# _platform_restart_counts: BR-003. start-platform.sh publishes platform_restarts.<child> (restarts=, last_exit_status=) under
# ${DYX3_RUN}; the unit stays "active" while the XRCE agent or mavlink-router crash-loops, so report the counts. WARN only: a
# restart is not by itself a failed upgrade (the INS-005 hold decides that), but it must be visible.
_platform_restart_counts() {
  local f name n st
  for f in "${DYX3_RUN}"/platform_restarts.*; do
    [ -f "${f}" ] || continue
    name="${f##*/platform_restarts.}"
    case "${name}" in *.tmp.*) continue ;; esac
    n="$(sed -n 's/^restarts=//p' "${f}" | head -n1)"
    st="$(sed -n 's/^last_exit_status=//p' "${f}" | head -n1)"
    case "${n}" in
      '' | *[!0-9]*) _warn "dyx3-platform ${name}: unreadable restart count in ${f}" ;;
      0) _pass "dyx3-platform ${name}: no restarts since the unit started" ;;
      *) _warn "dyx3-platform ${name}: restarted ${n} time(s) since the unit started (last exit status ${st:-unknown}); see journalctl -u dyx3-platform" ;;
    esac
  done
}

health_platform() {
  local rel="${1:-${DYX3_CURRENT}}"
  if systemd_available; then
    health_services "${rel}"
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
  _platform_restart_counts
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

# INS-005: the socket file outlives a dead gateway; ask it for a snapshot instead.
_gateway_up() {
  [ -S "${DYX3_GATEWAY_SOCK}" ] && have python3 &&
    DYX3_GATEWAY_QUERY_TIMEOUT_S="${DYX3_GATEWAY_PING_TIMEOUT_S:-2}" _gateway_query ping >/dev/null 2>&1
}
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
    if _settle _gateway_up; then _pass "gateway answers get_snapshot on ${DYX3_GATEWAY_SOCK}"; else _fail "gateway does not answer on ${DYX3_GATEWAY_SOCK} (dyx3-ros / system_gateway down?)"; fi
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

# ---- deep checks (HEALTH-DDS). The services run as ${DYX3_USER} with /etc/dyx3/ros.env (systemd EnvironmentFile) and
# dyx3_env_load (ROS_DOMAIN_ID; DYX3_ROS_LOCALHOST_ONLY=1 -> ROS_LOCALHOST_ONLY=1). dyx3-health runs as root without that
# file, so its ros2 calls looked at domain 0 and warned "no /fmu topics" and "node /px4_link not visible" on a healthy rover
# (2026-10-10). Every ros2 call below runs as the service user, in the services' environment, with a private ROS_HOME.

# _ros_env_lines: the KEY=VALUE lines of ros.env as systemd's EnvironmentFile reads them (blank and comment lines skipped,
# one pair of surrounding quotes removed). Never executed.
_ros_env_lines() {
  local line k v
  [ -r "${DYX3_ETC}/ros.env" ] || return 0
  while IFS= read -r line || [ -n "${line}" ]; do
    line="${line#"${line%%[![:space:]]*}"}"
    case "${line}" in '' | '#'* | ';'*) continue ;; esac
    k="${line%%=*}"
    [ "${k}" != "${line}" ] && [[ "${k}" =~ ^[A-Za-z_][A-Za-z0-9_]*$ ]] || continue
    v="${line#*=}"
    case "${v}" in \"*\" | \'*\') v="${v:1:${#v}-2}" ;; esac
    printf '%s=%s\n' "${k}" "${v}"
  done <"${DYX3_ETC}/ros.env"
}
_ros_env_value() { _ros_env_lines | sed -n "s/^$1=//p" | tail -n1; }

# _ros_env_desc: "ROS_DOMAIN_ID=<n>, ROS_LOCALHOST_ONLY=<0|1>" as the launchers set them.
_ros_env_desc() {
  local lo=0
  if [ "$(_ros_env_value DYX3_ROS_LOCALHOST_ONLY)" = 1 ] || [ "$(_ros_env_value ROS_LOCALHOST_ONLY)" = 1 ]; then lo=1; fi
  printf 'ROS_DOMAIN_ID=%s, ROS_LOCALHOST_ONLY=%s' "$(_ros_env_value ROS_DOMAIN_ID)" "${lo}"
}

# _ros_cli <release-dir> <timeout-s> <ros2 args...>: one ros2 call as ${DYX3_USER} (sudo -n when run as root), with only
# ros.env and the launcher's own dyx3_env_load (the release's bin/dyx3-env.sh), a throw-away ROS_HOME and ROS_LOG_DIR.
_ros_cli() {
  local rel="$1" t="$2" envsh
  shift 2
  envsh="${rel}/bin/dyx3-env.sh"
  [ -f "${envsh}" ] || envsh="${rel}/deployment/scripts/dyx3-env.sh"
  local -a as_user=() envs=()
  mapfile -t envs < <(_ros_env_lines)
  if [ "$(id -u)" -eq 0 ] && [ "${DYX3_USER}" != root ] && have sudo; then as_user=(sudo -n -u "${DYX3_USER}"); fi
  # shellcheck disable=SC2016  # expanded by the inner shell
  "${as_user[@]}" env -i PATH="${PATH}" "${envs[@]}" DYX3_RELEASE_DIR="${rel}" DYX3_ROS_SETUP="${ROS_SETUP}" \
    DYX3_PX4_MSGS_DIR="${DYX3_PX4_MSGS_DIR}" bash -c '
      t="$1" envsh="$2"
      shift 2
      h="$(mktemp -d /tmp/dyx3-health-ros.XXXXXX)" || exit 1
      export HOME="${h}" ROS_HOME="${h}" ROS_LOG_DIR="${h}/log"
      rc=1
      if . "${envsh}" && dyx3_env_load >/dev/null; then
        timeout "${t}" ros2 "$@"
        rc=$?
      fi
      rm -rf "${h}"
      exit "${rc}"' dyx3-health-ros "${t}" "${envsh}" "$@"
}

# DERIVED — NOT FROM V1 SPEC: discovery spin for the daemon-less ros2 calls (the rover's graph is on loopback; seconds,
# not a tuning value of the vehicle).
DYX3_HEALTH_ROS_SPIN_S="${DYX3_HEALTH_ROS_SPIN_S:-3}"

# health_graph: deep. The control graph's nodes are visible (WARN, not FAIL: needs ROS and a running graph). Secondary to
# the px4_link sample in health_dds. `ros2 node list` under-reports right after a start: up to three asks, no daemon.
health_graph() {
  local rel="${1:-${DYX3_CURRENT}}" nodes="" n i missing
  local -a want=(/dyx3_mission /motion_guard /px4_link /rpp /spray /system_gateway)
  _enabled dyx3-ros "${rel}/installer/manifests/production.manifest" || return 0
  [ -f "${ROS_SETUP}" ] || {
    _warn "ROS not installed; graph check skipped"
    return 0
  }
  for i in 1 2 3; do
    nodes="$(_ros_cli "${rel}" 15 node list --no-daemon --spin-time "${DYX3_HEALTH_ROS_SPIN_S}" 2>/dev/null || true)"
    missing=0
    for n in "${want[@]}"; do printf '%s\n' "${nodes}" | grep -qx "${n}" || missing=1; done
    [ "${missing}" -eq 0 ] && break
  done
  # Every node control_graph.launch.py starts, /rpp included (X-013).
  for n in "${want[@]}"; do
    if printf '%s\n' "${nodes}" | grep -qx "${n}"; then _pass "node ${n} up"; else _warn "node ${n} not visible (ros2 as ${DYX3_USER}, $(_ros_env_desc), ${i} tries)"; fi
  done
}

# _px4_link_sample <release-dir>: one /dyx3/px4_link/status sample, as
# "session_alive=<b> handshake_ok=<b> fault=<n> stale_topics_mask=<n> source=<gateway|ros2>". First from the gateway's
# get_snapshot (the INS-005 query: root-readable, no DDS discovery); when it has no fresh px4_link entry, from one
# `ros2 topic echo --once` in the services' environment. rc 1 when neither has a sample.
_px4_link_sample() {
  local rel="$1" line y sa ho f m
  if [ -S "${DYX3_GATEWAY_SOCK}" ] && have python3 && line="$(_gateway_query px4_link 2>/dev/null)"; then
    printf '%s source=gateway\n' "${line#* }"
    return 0
  fi
  [ -f "${ROS_SETUP}" ] || return 1
  y="$(_ros_cli "${rel}" 15 topic echo --once --no-daemon --qos-reliability reliable --qos-durability volatile \
    /dyx3/px4_link/status dyx3_interfaces/msg/Px4LinkStatus 2>/dev/null || true)"
  sa="$(sed -n 's/^session_alive: //p' <<<"${y}" | head -n1)"
  ho="$(sed -n 's/^handshake_ok: //p' <<<"${y}" | head -n1)"
  f="$(sed -n 's/^fault: //p' <<<"${y}" | head -n1)"
  m="$(sed -n 's/^stale_topics_mask: //p' <<<"${y}" | head -n1)"
  [ -n "${sa}" ] || return 1
  printf 'session_alive=%s handshake_ok=%s fault=%s stale_topics_mask=%s source=ros2\n' "${sa}" "${ho:-?}" "${f:-?}" "${m:-?}"
}

# _kv <key> <line>: the value of key=value in a px4_link sample line.
_kv() {
  local w
  local -a ws
  read -ra ws <<<"$2"
  for w in "${ws[@]}"; do
    if [ "${w%%=*}" = "$1" ]; then
      printf '%s' "${w#*=}"
      return 0
    fi
  done
}

# _px4_fault_name <n> / _px4_stale_names <mask>: Px4LinkStatus.msg constants and the frozen stale-topic bits.
_px4_fault_name() {
  case "$1" in
    0) echo NONE ;; 1) echo NO_SESSION ;; 2) echo HANDSHAKE_MISMATCH ;; 3) echo TOPIC_STALE ;;
    4) echo COMMAND_STALE ;; 5) echo LOOP_OVERRUN ;; 6) echo HANDSHAKE_PENDING ;; *) echo UNKNOWN ;;
  esac
}
_px4_stale_names() {
  local -a bits=(timesync_status vehicle_local_position vehicle_status vehicle_attitude estimator_status_flags vehicle_gps_position)
  local i out=""
  case "$1" in '' | *[!0-9]*) return 0 ;; esac
  for i in "${!bits[@]}"; do [ $(($1 >> i & 1)) -eq 1 ] && out="${out} ${bits[i]}"; done
  printf '%s' "${out# }"
}

# _xrce_port / _xrce_listening: the agent's UDP port (platform.env, read not executed); rc 0 something listens on it,
# 1 nothing does, 2 ss is missing.
_xrce_port() {
  local p
  p="$(sed -n 's/^DYX3_XRCE_PORT=//p' "${DYX3_ETC}/platform.env" 2>/dev/null | tail -n1)"
  printf '%s' "${DYX3_XRCE_PORT:-${p:-8888}}"
}
_xrce_listening() {
  have ss || return 2
  ss -H -lun "sport = :$(_xrce_port)" 2>/dev/null | grep -q .
}

# _px4_baseline_param <release-dir> <NAME>: the value of a PX4 parameter in the repo baseline
# config/px4/3wd_6x_carry_from_proto.params ("vehicle component NAME value type", tab separated), or "?" when the file or the
# parameter is missing. Read at check time, never hardcoded; no FCU parameter is read.
_px4_baseline_param() {
  local v
  v="$(awk -v n="$2" '$1 !~ /^#/ && $3 == n { v = $4 } END { print v }' "$1/config/px4/3wd_6x_carry_from_proto.params" 2>/dev/null)"
  printf '%s' "${v:-?}"
}

# health_dds: deep. Is the FCU session up? Authoritative: one /dyx3/px4_link/status sample (WARN, except below); it replaces
# counting /fmu topics, which said nothing about the link and under-reported right after a start. FAIL when the XRCE agent
# listens but px4_link sees no PX4 session: the DDS environment cannot be told from a domain/participant mismatch or a dead FCU
# link, and a rover in that state cannot be driven. "No sample at all" stays WARN (the graph may simply be down).
# DERIVED — NOT FROM V1 SPEC: the sample is healthy with session_alive, handshake_ok and no stale topic. `fault` is
# reported, not judged: COMMAND_STALE is what px4_link reports while no guard command flows.
# PC-7a: px4_link sees /fmu only when PX4's UXRCE_DDS_DOM_ID equals ROS_DOMAIN_ID and UXRCE_DDS_PTCFG=1 matches the
# localhost-only graph. A live session therefore proves the domains match; a dead one while the agent listens names the
# likely causes. No FCU parameter is read.
health_dds() {
  local rel="${1:-${DYX3_CURRENT}}" s sa ho f m stale detail dom rc=0
  _enabled dyx3-ros "${rel}/installer/manifests/production.manifest" || return 0
  if ! s="$(_px4_link_sample "${rel}")"; then
    _warn "px4_link: no /dyx3/px4_link/status sample from the gateway or from ros2 as ${DYX3_USER} ($(_ros_env_desc)); dyx3-ros down?"
    return 0
  fi
  sa="$(_kv session_alive "${s}")"
  ho="$(_kv handshake_ok "${s}")"
  f="$(_kv fault "${s}")"
  m="$(_kv stale_topics_mask "${s}")"
  stale="$(_px4_stale_names "${m}")"
  detail="session_alive=${sa} handshake_ok=${ho} fault=${f} $(_px4_fault_name "${f}") stale_topics_mask=${m}${stale:+ (${stale})}; via $(_kv source "${s}")"
  dom="$(_ros_env_value ROS_DOMAIN_ID)"
  dom="${dom:-unset}"
  if [ "${sa}" = true ]; then
    if [ "${ho}" = true ] && [ "${m}" = 0 ]; then _pass "px4_link: FCU session alive (${detail})"; else _warn "px4_link: FCU session alive but not healthy (${detail})"; fi
    _pass "DDS domain: px4_link sees PX4 on ROS_DOMAIN_ID ${dom}, so PX4 UXRCE_DDS_DOM_ID matches"
    return 0
  fi
  _warn "px4_link: FCU session down (${detail})"
  _xrce_listening || rc=$?
  case "${rc}" in
    0) _fail "DDS domain: the XRCE agent listens on udp/$(_xrce_port) but px4_link sees no PX4 session on ROS_DOMAIN_ID ${dom}; likely PX4 UXRCE_DDS_DOM_ID != ${dom} or UXRCE_DDS_PTCFG != 1 (localhost-only); else FCU power or the Ethernet cable. Repo baseline expects UXRCE_DDS_DOM_ID=$(_px4_baseline_param "${rel}" UXRCE_DDS_DOM_ID) and UXRCE_DDS_PTCFG=$(_px4_baseline_param "${rel}" UXRCE_DDS_PTCFG); this rover runs $(_ros_env_desc)" ;;
    1) _warn "DDS domain: no PX4 session and nothing listens on udp/$(_xrce_port): the XRCE agent is down (dyx3-platform)" ;;
    *) _warn "DDS domain: no PX4 session on ROS_DOMAIN_ID ${dom}; likely the XRCE agent is down, PX4 UXRCE_DDS_DOM_ID != ${dom}, or UXRCE_DDS_PTCFG != 1 (localhost-only)" ;;
  esac
}

# ---- baseline (INS-006): a check that already fails on the running release (a second CH340, an unplugged receiver)
# must not fail the next release, or every upgrade and every rollback reverts.
# health_capture <release-dir> <file>: health_run, its PASS/WARN/FAIL lines also saved to <file>.
health_capture() {
  local rc=0
  mkdir -p "$(dirname "$2")"
  health_run "$1" | tee "$2" || rc=$?
  return "${rc}"
}

# _health_fail_keys <file>: the FAIL lines, release SHAs masked, sorted.
_health_fail_keys() {
  sed -n 's/^FAIL  //p' "$1" 2>/dev/null | sed -E 's/[0-9a-f]{40}/<sha>/g; s/\b[0-9a-f]{10}\b/<sha>/g' | LC_ALL=C sort -u
}

# health_verdict <baseline-file> <after-file>: 0 when every FAIL in <after-file> already failed in the baseline.
# Reports both kinds.
health_verdict() {
  local old new l
  old="$(LC_ALL=C comm -12 <(_health_fail_keys "$1") <(_health_fail_keys "$2"))"
  new="$(LC_ALL=C comm -13 <(_health_fail_keys "$1") <(_health_fail_keys "$2"))"
  if [ -n "${old}" ]; then
    warn "already failing before the switch (a fault of the rover, not of the release):"
    while IFS= read -r l; do warn "  ${l}"; done <<<"${old}"
  fi
  [ -z "${new}" ] && return 0
  warn "failing only since the switch:"
  while IFS= read -r l; do warn "  ${l}"; done <<<"${new}"
  return 1
}

# health_judge <release-dir> <baseline-file> <after-file>: health of the release now, against the baseline (if any).
# Sets HEALTH_RESULT to OK, "OK apart from failures present before the switch", or FAILED.
health_judge() {
  if health_capture "$1" "$3"; then
    HEALTH_RESULT=OK
    return 0
  fi
  if [ -s "$2" ] && health_verdict "$2" "$3"; then
    HEALTH_RESULT="OK apart from failures present before the switch"
    return 0
  fi
  HEALTH_RESULT=FAILED
  return 1
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
