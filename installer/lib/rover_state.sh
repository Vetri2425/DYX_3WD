#!/usr/bin/env bash
# DYX 3WD installer — what the running control graph says about the rover. Source, do not execute.
# shellcheck shell=bash
#
# The only authoritative, root-readable source is dyx3_system_gateway's Unix socket (newline JSON, protocol v1,
# docs/contracts/dyx3_system_gateway.md). The installer asks it for one snapshot and reads:
#   vehicle_state.data.arming_state   PX4 vehicle_status ABI: 1 = DISARMED, 2 = ARMED, 0 = no FCU status
#   mission.data.state                dyx3_interfaces/MissionState: 3 = RUNNING, 4 = PAUSED
# each with the gateway's own `fresh` flag (snapshot_fresh_s, 1 s).
#
# Rover idle (X-016), in this order:
#   dyx3-ros not running and nothing listens on the socket   -> idle (nothing on the Jetson can move the rover)
#   ARMED or mission RUNNING (fresh or not)                   -> busy: refuse
#   fresh vehicle_state DISARMED and fresh mission not RUNNING -> idle
#   anything else (no answer, no FCU status, stale data)       -> unknown: refuse ("unknown is not idle")
# Override: DYX3_FORCE_UNSAFE=1 (bench only; logged loudly).

DYX3_GATEWAY_SOCK="${DYX3_GATEWAY_SOCK:-${DYX3_RUN}/gateway.sock}"

# _gateway_query <mode>: one get_snapshot round trip. Prints "<verdict> <detail>" on one line.
#   mode=idle  rc 0 idle, 1 busy, 2 unknown, 3 nothing listening (no socket file / connection refused)
#   mode=ping  rc 0 the gateway answered get_snapshot with ok:true, 2 otherwise, 3 nothing listening
_gateway_query() {
  python3 - "${DYX3_GATEWAY_SOCK}" "${DYX3_GATEWAY_QUERY_TIMEOUT_S:-5}" "$1" <<'PY'
import json
import socket
import sys
import time

path, timeout, mode = sys.argv[1], float(sys.argv[2]), sys.argv[3]


def out(verdict, detail, rc):
    print(f"{verdict} {detail}")
    sys.exit(rc)


sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
sock.settimeout(timeout)
try:
    sock.connect(path)
except (FileNotFoundError, ConnectionRefusedError) as e:
    out("absent", f"nothing listens on {path} ({e.strerror})", 3)
except OSError as e:
    out("unknown", f"cannot connect to {path}: {e}", 2)

deadline = time.monotonic() + timeout
reply = None
buf = b""
try:
    sock.sendall(b'{"v":1,"id":1,"cmd":"get_snapshot"}\n')
    while reply is None:
        left = deadline - time.monotonic()
        if left <= 0:
            out("unknown", f"the gateway did not answer get_snapshot within {timeout:g} s", 2)
        sock.settimeout(left)
        chunk = sock.recv(65536)
        if not chunk:
            out("unknown", "the gateway closed the connection without answering", 2)
        buf += chunk
        if len(buf) > (8 << 20):
            out("unknown", "the gateway answer is oversized", 2)
        while b"\n" in buf and reply is None:
            line, buf = buf.split(b"\n", 1)
            try:
                msg = json.loads(line)
            except ValueError:
                continue
            # Telemetry broadcasts carry "type"; the reply carries our id.
            if isinstance(msg, dict) and msg.get("id") == 1 and "type" not in msg:
                reply = msg
except socket.timeout:
    out("unknown", f"the gateway did not answer get_snapshot within {timeout:g} s", 2)
except OSError as e:
    out("unknown", f"gateway socket error: {e}", 2)

if reply.get("v") != 1 or reply.get("ok") is not True:
    out("unknown", f"the gateway refused get_snapshot ({reply.get('code')}: {reply.get('reason')})", 2)
snap = reply.get("data")
if not isinstance(snap, dict):
    out("unknown", "the gateway answer has no snapshot", 2)
if mode == "ping":
    out("ok", "the gateway answered get_snapshot", 0)


def source(name):
    e = snap.get(name)
    if not isinstance(e, dict):
        return None, False
    d = e.get("data")
    return (d if isinstance(d, dict) else None), e.get("fresh") is True


def as_int(v):
    return v if isinstance(v, int) and not isinstance(v, bool) else None


vs, vs_fresh = source("vehicle_state")
ms, ms_fresh = source("mission")
arming = as_int(vs.get("arming_state")) if vs else None
mstate = as_int(ms.get("state")) if ms else None

# Any evidence of motion refuses, even stale: stale data cannot prove the opposite.
if arming == 2:
    out("busy", "the vehicle is ARMED" + ("" if vs_fresh else " (last known, stale)"), 1)
if mstate == 3:
    out("busy", "a mission is RUNNING" + ("" if ms_fresh else " (last known, stale)"), 1)
if vs is None or not vs_fresh:
    out("unknown", "no fresh vehicle_state from px4_link", 2)
if ms is None or not ms_fresh:
    out("unknown", "no fresh mission state from dyx3_mission", 2)
if arming != 1:
    out("unknown", f"the FCU arming state is unknown (arming_state={arming}; no FCU session?)", 2)
if mstate is None:
    out("unknown", "the mission state is not a number", 2)
note = " (mission PAUSED: a restart drops it)" if mstate == 4 else ""
out("idle", f"disarmed, mission state {mstate}{note}", 0)
PY
}

# _rover_ros_unit_active: dyx3-ros is running, starting or stopping (systemd only).
_rover_ros_unit_active() {
  systemd_available || return 1
  case "$(systemctl is-active dyx3-ros.service 2>/dev/null || true)" in
    active | activating | deactivating | reloading | refreshing) return 0 ;;
  esac
  return 1
}

# rover_idle_check: rc 0 idle, 1 busy, 2 unknown; prints the reason on one line.
rover_idle_check() {
  local ros=0 line rc=0
  _rover_ros_unit_active && ros=1
  if [ "${ros}" -eq 0 ] && [ ! -e "${DYX3_GATEWAY_SOCK}" ]; then
    echo "dyx3-ros is not running and there is no gateway socket"
    return 0
  fi
  if ! have python3; then
    echo "python3 is missing: the gateway cannot be asked"
    return 2
  fi
  line="$(_gateway_query idle)" || rc=$?
  case "${rc}" in
    0)
      echo "${line#* }"
      return 0
      ;;
    1)
      echo "${line#* }"
      return 1
      ;;
    3)
      if [ "${ros}" -eq 0 ]; then
        echo "dyx3-ros is not running (${line#* })"
        return 0
      fi
      echo "dyx3-ros is running but its gateway does not answer (${line#* })"
      return 2
      ;;
    *)
      echo "${line#* }"
      return 2
      ;;
  esac
}

# rover_may_restart <what>: 0 when the rover is known to be idle, or DYX3_FORCE_UNSAFE=1 (logged loudly); else 1
# with the reason in ROVER_BUSY_REASON.
rover_may_restart() {
  local what="${1:-this operation}" rc=0
  ROVER_BUSY_REASON="$(rover_idle_check)" || rc=$?
  if [ "${rc}" -eq 0 ]; then
    log "rover idle check before ${what}: ${ROVER_BUSY_REASON}"
    return 0
  fi
  if [ "${DYX3_FORCE_UNSAFE:-0}" = "1" ]; then
    warn "################################################################################"
    warn "DYX3_FORCE_UNSAFE=1: ${what} goes ahead although the rover is NOT known to be idle"
    warn "  reason: ${ROVER_BUSY_REASON}"
    warn "  a restart of dyx3-platform/dyx3-ros mid-mission drops offboard and PX4 disarms"
    warn "################################################################################"
    if [ -z "${DYX3_ROOT}" ] && have logger; then
      logger -p user.warning -t dyx3-installer \
        "DYX3_FORCE_UNSAFE=1: ${what} while the rover is not known idle: ${ROVER_BUSY_REASON}" || true
    fi
    return 0
  fi
  return 1
}

# require_rover_idle <what>: die unless rover_may_restart. Call it before anything that restarts dyx3-platform or
# dyx3-ros, switches `current`, or touches the FCU or Wi-Fi link.
require_rover_idle() {
  rover_may_restart "$@" && return 0
  die "refusing ${1:-this operation}: the rover is not known to be idle (${ROVER_BUSY_REASON}). Disarm and end or abort the mission, then retry. Bench only, with nobody near the rover: DYX3_FORCE_UNSAFE=1"
}
