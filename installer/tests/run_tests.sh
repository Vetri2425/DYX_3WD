#!/usr/bin/env bash
# shellcheck disable=SC2016,SC2329,SC2030,SC2031,SC2163,SC2317
# Installer tests. No root, no network, no systemd, no ROS: everything runs against a staged
# root (DYX3_ROOT) with fake `colcon`/`ss`/`ping`. Run: installer/tests/run_tests.sh
#
# What this does NOT prove: that apt, ROS, the pinned builds, NetworkManager, systemd or the
# real MicroXRCEAgent/mavlink-router work on the Jetson. Those need the rover.
set -uo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
T="$(mktemp -d)"
trap 'if [ -n "${KEEP_T:-}" ]; then echo "kept ${T}"; else rm -rf "${T}"; fi' EXIT
RESULTS="${T}/results"
: >"${RESULTS}"
ok() { printf 'ok   %s\n' "$1"; echo ok >>"${RESULTS}"; }
bad() { printf 'FAIL %s\n' "$1"; echo bad >>"${RESULTS}"; }
check() { if eval "$2"; then ok "$1"; else bad "$1"; fi; }

# ---------------------------------------------------------------- fake system gateway
# fake_gateway_start <socket> <reply-file>: a newline-JSON server like dyx3_system_gateway. Per connection it
# reads one request, sends one telemetry broadcast, then the contents of <reply-file> (re-read every time).
# "silent" never answers; "garbage" answers with a line that is not JSON.
fake_gateway_start() {
  mkdir -p "$(dirname "$1")"
  python3 - "$1" "$2" <<'PY' &
import os
import socket
import sys
import threading

path, state = sys.argv[1], sys.argv[2]
try:
    os.unlink(path)
except FileNotFoundError:
    pass
srv = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
srv.bind(path)
srv.listen(8)


def handle(c):
    with c:
        c.makefile("rb").readline()
        mode = open(state).read().strip()
        if mode == "silent":
            c.recv(1)
            return
        c.sendall(b'{"v":1,"type":"telemetry","snapshot":{}}\n')
        c.sendall((b"not json" if mode == "garbage" else mode.encode()) + b"\n")


while True:
    conn, _ = srv.accept()
    threading.Thread(target=handle, args=(conn,), daemon=True).start()
PY
  FAKE_GW_PID=$!
  local i
  for i in $(seq 50); do [ -S "$1" ] && break; sleep 0.1; done
}
# gw_state <reply-file> <arming_state> <vehicle fresh> <mission state> <mission fresh> [<mission waiting_on>]
gw_state() {
  printf '{"v":1,"id":1,"ok":true,"code":"ok","reason":"","data":{"vehicle_state":{"age_s":0.1,"fresh":%s,"data":{"arming_state":%s}},"mission":{"age_s":0.1,"fresh":%s,"data":{"state":%s,"waiting_on":%s}},"gateway":{}}}\n' \
    "$3" "$2" "$5" "$4" "${6:-0}" >"$1"
}
# agent_toggle: stands in for restart_enabled_services. The first call takes the fake XRCE agent down
# (FAKE_NO_AGENT="${T}/agent_down"), the next brings it back: a fault that starts with the switch and ends with the revert.
agent_toggle() { if [ -e "${T}/agent_down" ]; then rm -f "${T}/agent_down"; else : >"${T}/agent_down"; fi; }
idle_rc() {
  rover_idle_check >"${T}/idle_reason" 2>&1
  echo $?
}

# make_fakebin <dir>: fake colcon / ss / ping, and flock(1) over python's fcntl.flock (macOS has no util-linux).
make_fakebin() {
  mkdir -p "$1"
  cat >"$1/colcon" <<'F'
#!/usr/bin/env bash
if [ -e ros2_ws/src/BREAK ] || [ -e src/BREAK ]; then echo "fake colcon: broken build" >&2; exit 1; fi
mkdir -p install && : >install/setup.bash
F
  cat >"$1/ss" <<'F'
#!/usr/bin/env bash
[ -e "${FAKE_NO_AGENT:-/nonexistent}" ] || echo "UNCONN 0 0 0.0.0.0:8888 0.0.0.0:*"
F
  printf '#!/usr/bin/env bash\nexit 0\n' >"$1/ping"
  cat >"$1/flock" <<'F'
#!/usr/bin/env bash
# fake flock [-n] [-x] <fd>: lock the open file description the caller passed as <fd>
nb=0 fd=""
for a in "$@"; do case "${a}" in -n) nb=1 ;; -x | -e) ;; *) fd="${a}" ;; esac; done
exec python3 -c '
import fcntl, sys
try:
    fcntl.flock(int(sys.argv[1]), fcntl.LOCK_EX | (fcntl.LOCK_NB if sys.argv[2] == "1" else 0))
except OSError:
    sys.exit(1)' "${fd}" "${nb}"
F
  chmod +x "$1"/*
}

# make_src <dir>: a git repo with this installer and deployment, enabling only dyx3-platform (the staged root has no
# gateway, backend or RTK). Commit "A" is made.
make_src() {
  local src="$1"
  mkdir -p "${src}"
  cp -r "${REPO}/installer" "${REPO}/deployment" "${src}/"
  rm -rf "${src}/installer/tests"
  awk '/^\[enabled_services\]/ { print; print "dyx3-platform"; skip = 1; next }
       /^\[/ { skip = 0 }
       skip && /^dyx3-/ { next }
       { print }' "${src}/installer/manifests/production.manifest" >"${src}/manifest.tmp"
  mv "${src}/manifest.tmp" "${src}/installer/manifests/production.manifest"
  mkdir -p "${src}/ros2_ws/src"
  : >"${src}/ros2_ws/src/.keep"
  git -C "${src}" init -q -b main
  git -C "${src}" config user.email t@t
  git -C "${src}" config user.name t
  git -C "${src}" add -A
  git -C "${src}" commit -q -m A
}

# ---------------------------------------------------------------- supervisor
sup() {
  local d="${T}/sup"
  mkdir -p "${d}/bin" "${d}/state"
  # Agent that dies immediately (to prove restart) and counts its starts.
  cat >"${d}/bin/MicroXRCEAgent" <<'A'
#!/usr/bin/env bash
echo x >>"$COUNT"
sleep "${AGENT_LIFE:-0.2}"
exit 3
A
  cat >"${d}/bin/mavlink-routerd" <<'A'
#!/usr/bin/env bash
sleep 30
A
  chmod +x "${d}"/bin/*
  : >"${d}/router.conf"
  : >"${d}/count"
  env PATH="${d}/bin:${PATH}" COUNT="${d}/count" DYX3_PLATFORM_ENV=/nonexistent \
    DYX3_RESTART_DELAY_S=0.1 DYX3_MAVROUTER_CONF="${d}/router.conf" DYX3_PLATFORM_STATE_DIR="${d}/state" \
    "${REPO}/deployment/scripts/start-platform.sh" >"${d}/out" 2>&1 &
  local pid=$!
  sleep 2
  local starts
  starts="$(wc -l <"${d}/count")"
  check "supervisor restarts a crashing agent (starts=${starts})" '[ "${starts}" -ge 3 ]'
  check "supervisor logs each restart with its count and exit status" 'grep -q "xrce-agent: exited rc=3; restart #1 in" "${d}/out" && grep -q "xrce-agent: exited rc=3; restart #2 in" "${d}/out"'
  local sf="${d}/state/platform_restarts.xrce-agent"
  check "supervisor publishes the restart count (restarts, last exit status, time)" 'grep -qx "restarts=[1-9][0-9]*" "${sf}" && grep -qx "last_exit_status=3" "${sf}" && grep -q "^updated_utc=" "${sf}"'
  check "the router, which never crashed, publishes 0 restarts" 'grep -qx "restarts=0" "${d}/state/platform_restarts.mavlink-router"'
  check "dyx3-health reports a crash-looping child as WARN and a quiet one as PASS" '(DYX3_RUN="${d}/state"; INSTALLER_DIR="${REPO}/installer"; export INSTALLER_DIR; . "${REPO}/installer/lib/common.sh"; . "${REPO}/installer/lib/health_check.sh"; DYX3_RUN="${d}/state"; out="$(_platform_restart_counts)"; printf "%s\n" "${out}" | grep -q "^WARN  dyx3-platform xrce-agent: restarted [1-9][0-9]* time(s) .*last exit status 3" && printf "%s\n" "${out}" | grep -q "^PASS  dyx3-platform mavlink-router: no restarts")'
  check "supervisor runs the router" 'grep -q "mavlink-router: starting" "${d}/out"'
  kill -TERM "${pid}"
  local i
  for i in 1 2 3 4 5 6 7 8 9 10; do kill -0 "${pid}" 2>/dev/null || break; sleep 0.3; done
  check "supervisor exits on SIGTERM" '! kill -0 "${pid}" 2>/dev/null'
  sleep 0.5
  check "no orphaned router after SIGTERM" '! pgrep -f "${d}/bin/mavlink-routerd" >/dev/null && ! pgrep -f "sleep 30" >/dev/null || true'
  local n1 n2
  n1="$(wc -l <"${d}/count")"
  sleep 0.6
  n2="$(wc -l <"${d}/count")"
  check "agent is not restarted after SIGTERM" '[ "${n1}" = "${n2}" ]'
  check "no temp file is left in the state directory after the supervisors stop" '! ls "${d}/state" | grep -q "\.tmp\."'
  # Missing agent binary is fatal (systemd will surface it) — never silently "healthy".
  env PATH="/usr/bin:/bin" DYX3_PLATFORM_ENV=/nonexistent DYX3_AGENT_BIN=definitely-missing \
    "${REPO}/deployment/scripts/start-platform.sh" >/dev/null 2>&1
  rc=$?
  check "missing agent binary exits non-zero" '[ "${rc}" -ne 0 ]'
}

# ---------------------------------------------------------------- recorder launcher (REC-024, REC-022)
recorder_launcher() {
  local d="${T}/rec" ws="${T}/rec/ws" out rc
  mkdir -p "${d}/rel/installer/pins" "${d}/rel/ros2_ws/install" "${d}/px/abc/install" "${ws}/lib/dyx3_recorder" "${d}/other"
  echo FIRMWARE_SHA=abc >"${d}/rel/installer/pins/firmware.pin"
  : >"${d}/ros.bash"
  : >"${d}/px/abc/install/setup.bash"
  echo "export AMENT_PREFIX_PATH=${d}/other:${ws}" >"${d}/rel/ros2_ws/install/setup.bash"
  printf '#!/bin/sh\necho "ARGS:$*"\n' >"${ws}/lib/dyx3_recorder/recorder_node"
  chmod +x "${ws}/lib/dyx3_recorder/recorder_node"
  rl() {
    env -i PATH="${PATH}" ROS_DOMAIN_ID=42 DYX3_RELEASE_DIR="${d}/rel" DYX3_ROS_SETUP="${d}/ros.bash" DYX3_PX4_MSGS_DIR="${d}/px" \
      ROS_HOME="${d}/home" ROS_LOG_DIR="${d}/log" "$@" bash "${REPO}/deployment/scripts/start-recorder.sh" 2>&1
  }
  out="$(rl DYX3_RECORDER_PARAMS="${d}/absent.yaml")"
  check "recorder launcher: execs the binary found on AMENT_PREFIX_PATH, no parameter file needed" '[ "${out}" = "ARGS:" ]'
  echo "recorder: {ros__parameters: {vehicle_id: x}}" >"${d}/rec.yaml"
  out="$(rl DYX3_RECORDER_PARAMS="${d}/rec.yaml")"
  check "recorder launcher: an existing parameter file is passed with --ros-args --params-file" '[ "${out}" = "ARGS:--ros-args --params-file ${d}/rec.yaml" ]'
  chmod 000 "${d}/rec.yaml"
  if [ ! -r "${d}/rec.yaml" ]; then # skipped when the tests run as root
    out="$(rl DYX3_RECORDER_PARAMS="${d}/rec.yaml")"
    rc=$?
    check "recorder launcher: an unreadable parameter file is an error, not a silent default" '[ "${rc}" -ne 0 ] && printf "%s" "${out}" | grep -q "not readable"'
  fi
  chmod 600 "${d}/rec.yaml"
  chmod -x "${ws}/lib/dyx3_recorder/recorder_node"
  out="$(rl DYX3_RECORDER_PARAMS="${d}/absent.yaml")"
  check "recorder launcher: a missing binary fails loudly" 'printf "%s" "${out}" | grep -q "recorder_node not found"'
  local tpl="${REPO}/config/recorder/recorder.yaml.example" src="${REPO}/ros2_ws/src/dyx3_recorder/src/recorder_node.cpp"
  check "recorder template: keyed by the node name, sets vehicle_id and operator" 'grep -q "^recorder:" "${tpl}" && grep -q "^    vehicle_id:" "${tpl}" && grep -q "^    operator:" "${tpl}"'
  check "recorder template: the parameters are the ones the recorder declares" 'grep -q "Node(\"recorder\"" "${src}" && grep -q "declare_parameter<std::string>(\"vehicle_id\"" "${src}" && grep -q "declare_parameter<std::string>(\"operator\"" "${src}"'
}

# ---------------------------------------------------------------- libs
libs() {
  export INSTALLER_DIR="${REPO}/installer" DYX3_ROOT="${T}/root"
  # shellcheck disable=SC1091
  . "${INSTALLER_DIR}/lib/common.sh"
  for l in os_check dependencies ros_install permissions network_install systemd_install health_check usb_serial release; do
    # shellcheck disable=SC1090
    . "${INSTALLER_DIR}/lib/${l}.sh"
  done
  set +e

  local svc
  svc="$(manifest_section enabled_services)"
  # The six services verified on the 3WD rover 2026-10-08, with dyx3-ros split into dyx3-control + dyx3-services (2026-10-10):
  # the order is the restart order (control before services, recorder after control, backend after services).
  check "manifest: USB provisioning runs before the rover services, control before services, backend last" '[ "$(printf "%s" "${svc}" | tr "\n" " ")" = "dyx3-usb-serial-check dyx3-platform dyx3-control dyx3-services dyx3-rtk dyx3-spray-watchdog dyx3-recorder dyx3-backend" ]'
  check "manifest: the single dyx3-ros unit is gone from both lists" '! manifest_section services | grep -qx dyx3-ros && ! manifest_section enabled_services | grep -qx dyx3-ros'
  check "manifest: [services] is exactly the shipped unit files (the installer installs the intersection)" '[ "$(manifest_section services | LC_ALL=C sort | tr "\n" " ")" = "$(for u in "${REPO}"/deployment/systemd/*.service; do basename "${u}" .service; done | LC_ALL=C sort | tr "\n" " ")" ]'
  check "no dyx3-ros unit or start-ros.sh launcher is shipped any more" '[ ! -e "${REPO}/deployment/systemd/dyx3-ros.service" ] && [ ! -e "${REPO}/deployment/scripts/start-ros.sh" ]'
  check "manifest: legacy package absent from ros2_packages" '! manifest_section ros2_packages | grep -q legacy'
  check "manifest: spray watchdog is its own service" 'manifest_section services | grep -qx dyx3-spray-watchdog'
  local s2
  for s2 in $(manifest_section services); do
    if [ "${s2}" = dyx3-usb-serial-check ]; then
      check "service ${s2}: provisioning unit and setup script exist" '[ -f "${REPO}/deployment/systemd/${s2}.service" ] && [ -x "${REPO}/installer/usb_serial_setup.sh" ]'
    else
      check "service ${s2}: unit and launcher exist" '[ -f "${REPO}/deployment/systemd/${s2}.service" ] && [ -x "${REPO}/deployment/scripts/start-${s2#dyx3-}.sh" ]'
    fi
  done
  check "USB serial unit provisions during old and new release activation" 'grep -qx "dyx3-usb-serial-check" <(manifest_section enabled_services) && grep -q "ExecStart=/opt/dyx3/current/installer/usb_serial_setup.sh" "${REPO}/deployment/systemd/dyx3-usb-serial-check.service"'
  check "fresh installer provisions the kernel driver before release setup" 'grep -q "provision_usb_serial_support" "${REPO}/installer/install.sh"'
  check "upgrade enables the provisioning unit before restarting rover services" 'sed -n "/^\[enabled_services\]/,/^\[/p" "${REPO}/installer/manifests/production.manifest" | sed -n "2p" | grep -qx dyx3-usb-serial-check'
  check "the spray watchdog unit is not tied to either graph unit" '! grep -E "^(Requires|BindsTo|PartOf)=.*dyx3-(control|services|ros)" "${REPO}/deployment/systemd/dyx3-spray-watchdog.service"'
  # Only rpp and motion_guard (FIFO 80) and px4_link (FIFO 70) get CPU 4, via the launch prefix running as User=dyx3.
  local ru="${REPO}/deployment/systemd/dyx3-control.service" su="${REPO}/deployment/systemd/dyx3-services.service"
  check "dyx3-control sets no tree-wide RT policy or affinity" '! grep -Eq "^(CPUSchedulingPolicy|CPUSchedulingPriority|CPUAffinity)=" "${ru}"'
  check "dyx3-control lets the unprivileged chrt prefix set FIFO 80" '[ "$(sed -n "s/^LimitRTPRIO=//p" "${ru}")" -ge 80 ] && grep -q "chrt -f 80" "${REPO}/ros2_ws/src/dyx3_bringup/launch/control_graph.launch.py"'
  check "dyx3-control lets mlockall succeed and bounds a stop hang" 'grep -qx "LimitMEMLOCK=infinity" "${ru}" && grep -qx "TimeoutStopSec=15" "${ru}" && ! grep -q "^RestrictRealtime=yes" "${ru}"'
  check "dyx3-control requires the platform and runs the control launcher" 'grep -qx "After=dyx3-platform.service" "${ru}" && grep -qx "Requires=dyx3-platform.service" "${ru}" && grep -qx "ExecStart=/opt/dyx3/current/bin/dyx3-control" "${ru}" && grep -q "^exec ros2 launch dyx3_bringup control_graph.launch.py " "${REPO}/deployment/scripts/start-control.sh"'
  check "dyx3-services wants dyx3-control and never requires, binds to or is part of it" 'grep -qx "After=dyx3-control.service" "${su}" && grep -qx "Wants=dyx3-control.service" "${su}" && ! grep -Eq "^(Requires|BindsTo|PartOf|Requisite)=" "${su}"'
  check "dyx3-control does not depend on dyx3-services in any way" '! grep -Eq "^[A-Za-z]+=.*dyx3-services" "${ru}"'
  check "dyx3-services runs the services launcher under normal scheduling, stop hang bounded" 'grep -qx "ExecStart=/opt/dyx3/current/bin/dyx3-services" "${su}" && grep -q "^exec ros2 launch dyx3_bringup services_graph.launch.py " "${REPO}/deployment/scripts/start-services.sh" && grep -qx "TimeoutStopSec=15" "${su}" && ! grep -Eq "^(LimitRTPRIO|CPUSchedulingPolicy|CPUSchedulingPriority|CPUAffinity)=" "${su}"'
  local u
  for u in "${ru}" "${su}"; do
    check "${u##*/}: Restart=always, RestartSec=5, User=dyx3, ros.env, hardening, the stop-timeout DERIVED note" 'grep -qx "Restart=always" "${u}" && grep -qx "RestartSec=5" "${u}" && grep -qx "User=dyx3" "${u}" && grep -qx "EnvironmentFile=-/etc/dyx3/ros.env" "${u}" && grep -q "DERIVED — NOT FROM V1 SPEC: ros2 launch escalates" "${u}" && grep -qx "NoNewPrivileges=yes" "${u}" && grep -qx "ProtectSystem=strict" "${u}"'
  done
  check "the recorder starts after the control unit, the backend after the services unit" 'grep -qx "After=dyx3-control.service" "${REPO}/deployment/systemd/dyx3-recorder.service" && grep -qx "Wants=dyx3-control.service" "${REPO}/deployment/systemd/dyx3-recorder.service" && grep -qx "After=dyx3-services.service" "${REPO}/deployment/systemd/dyx3-backend.service" && grep -qx "Wants=dyx3-services.service" "${REPO}/deployment/systemd/dyx3-backend.service"'
  check "no shipped unit or launcher still names dyx3-ros" '! grep -rn "dyx3-ros" "${REPO}/deployment/systemd" "${REPO}/deployment/scripts"'
  check "the RTK unit creates its own 0700 state directory" 'grep -qx "StateDirectory=dyx3/rtk" "${REPO}/deployment/systemd/dyx3-rtk.service" && grep -qx "StateDirectoryMode=0700" "${REPO}/deployment/systemd/dyx3-rtk.service"'
  check "router template: PX4 endpoint is a UDP server on 0.0.0.0:14550 (survives restarts), QGC on TCP 5760" 't="${REPO}/deployment/network/mavlink-router.conf.tmpl"; grep -qx "Mode=Server" "${t}" && grep -qx "Address=0.0.0.0" "${t}" && grep -qx "Port=14550" "${t}" && grep -qx "TcpServerPort=5760" "${t}" && ! grep -qx "Mode=Normal" "${t}"'
  check "no unit or template hard-codes a secret (comments excluded)" '! grep -rEi "^[^#]*(password|token)=." "${REPO}/deployment/systemd" "${REPO}/deployment/network"'

  # runtime environment: never guess a ROS domain, never start without the pinned px4_msgs overlay
  local e="${T}/envrel"
  mkdir -p "${e}/rel/installer/pins" "${e}/rel/ros2_ws/install" "${e}/pm/$(sed -n 's/^FIRMWARE_SHA=//p' "${REPO}/installer/pins/firmware.pin")/install"
  cp "${REPO}/installer/pins/firmware.pin" "${e}/rel/installer/pins/"
  : >"${e}/ros.bash"
  : >"${e}/rel/ros2_ws/install/setup.bash"
  : >"${e}/pm/$(sed -n 's/^FIRMWARE_SHA=//p' "${REPO}/installer/pins/firmware.pin")/install/setup.bash"
  envcall() { # args: VAR=VAL to export, -VAR to unset
    (
      . "${REPO}/deployment/scripts/dyx3-env.sh"
      export DYX3_RELEASE_DIR="${e}/rel" DYX3_ROS_SETUP="${e}/ros.bash" DYX3_PX4_MSGS_DIR="${e}/pm"
      local a
      for a in "$@"; do case "${a}" in -*) unset "${a#-}" ;; *) export "${a}" ;; esac; done
      dyx3_env_load
    ) 2>&1
  }
  check "env: refuses to start without ROS_DOMAIN_ID" 'o="$(envcall -ROS_DOMAIN_ID)"; rc=$?; [ "${rc}" -ne 0 ] && case "${o}" in *"ROS_DOMAIN_ID is not set"*) true ;; *) false ;; esac'
  check "env: loads with a domain set" 'envcall ROS_DOMAIN_ID=7 >/dev/null'
  check "env: refuses when the px4_msgs overlay is missing" '! envcall ROS_DOMAIN_ID=7 DYX3_PX4_MSGS_DIR="${e}/nope" >/dev/null'
  check "launcher exits non-zero without a domain" '! env -u ROS_DOMAIN_ID DYX3_RELEASE_DIR="${e}/rel" "${REPO}/deployment/scripts/start-spray-watchdog.sh" >/dev/null 2>&1'
  check "backend launcher refuses without its venv" '! DYX3_RELEASE_DIR="${e}/rel" "${REPO}/deployment/scripts/start-backend.sh" >/dev/null 2>&1'
  check "backend launcher binds the hotspot address, never 0.0.0.0" 'grep -q "10.42.0.1" "${REPO}/deployment/scripts/start-backend.sh" && ! grep -v "^#" "${REPO}/deployment/scripts/start-backend.sh" | grep -q "0.0.0.0"'

  # INS-004: upgrade and rollback continue as a transient unit; never in a staged root.
  local sr="${T}/sysrun"
  mkdir -p "${sr}"
  printf '#!/usr/bin/env bash\nprintf "%%s\\n" "$@" >"%s/argv"\n' "${sr}" >"${sr}/systemd-run"
  chmod +x "${sr}/systemd-run"
  out="$( (PATH="${sr}:${PATH}" DYX3_ROOT="" DYX3_FORCE=1 DYX3_FORCE_UNSAFE=1 detach_or_continue upgrade "${REPO}/installer/upgrade.sh" main; echo returned) 2>&1)"
  check "detach: an upgrade re-runs itself under systemd-run with the ref and the force flags" '! printf "%s" "${out}" | grep -q returned && grep -q "^--unit=dyx3-upgrade-" "${sr}/argv" && grep -qx -- "--collect" "${sr}/argv" && grep -qx -- "--setenv=DYX3_DETACHED=1" "${sr}/argv" && grep -qx -- "--setenv=DYX3_FORCE=1" "${sr}/argv" && grep -qx -- "--setenv=DYX3_FORCE_UNSAFE=1" "${sr}/argv" && [ "$(tail -n1 "${sr}/argv")" = main ] && printf "%s" "${out}" | grep -q "journalctl -fu dyx3-upgrade-"'
  out="$( (PATH="${sr}:${PATH}" detach_or_continue upgrade "${REPO}/installer/upgrade.sh" main; echo returned) 2>&1)"
  check "detach: never in a staged root" 'printf "%s" "${out}" | grep -q returned'

  printf 'ID=ubuntu\nVERSION_ID="22.04"\n' >"${T}/os22"
  printf 'ID=ubuntu\nVERSION_ID="24.04"\n' >"${T}/os24"
  printf 'ID=debian\nVERSION_ID="12"\n' >"${T}/osdeb"
  check "os_check accepts ubuntu 22.04" '(DYX3_OS_RELEASE="${T}/os22" DYX3_ARCH=aarch64 os_check) 2>/dev/null'
  check "os_check rejects ubuntu 24.04" '! (DYX3_OS_RELEASE="${T}/os24" DYX3_ARCH=aarch64 os_check) 2>/dev/null'
  check "os_check rejects debian" '! (DYX3_OS_RELEASE="${T}/osdeb" DYX3_ARCH=aarch64 os_check) 2>/dev/null'

  check "pins: XRCE agent v2.4.3 pinned to a commit" '(load_pin microxrce_agent; [ "${XRCE_TAG}" = v2.4.3 ] && [ ${#XRCE_COMMIT} -eq 40 ])'
  check "pins: firmware SHA is the rover firmware 8279fa4b" '(load_pin firmware; [ "${FIRMWARE_SHA}" = 8279fa4be33d5fc26c3b895c7e4a0a8660fcfff1 ])'

  # dry-run of the whole install flow must complete and mention every required step.
  # PC-7c: the user step asks the HOST whether the service user exists, so a host with a dyx3 user skipped useradd and
  # failed this test. The fresh-install run names a user that cannot exist here; a second run takes the other branch.
  local out nouser="dyx3-absent-$$"
  while id "${nouser}" >/dev/null 2>&1; do nouser="${nouser}x"; done
  out="$(DYX3_USER="${nouser}" DYX3_ALLOW_ANY_OS=1 DYX3_DRY_RUN=1 "${REPO}/installer/install.sh" --production --dry-run --ref HEAD 2>&1)"
  rc=$?
  check "install --dry-run completes" '[ "${rc}" -eq 0 ]'
  for s in "useradd" "apt-get install" "MicroXRCEAgent" "mavlink-router" "nmcli connection add" "colcon build" "git archive"; do
    check "install dry-run mentions: ${s}" 'printf "%s" "${out}" | grep -q -- "${s}"'
  done
  check "install dry-run creates an absent service user as a system user with its own group" 'printf "%s" "${out}" | grep -q -- "useradd --system .* --user-group ${nouser}$"'
  local out_user
  out_user="$(DYX3_ROOT="${T}/root-user" DYX3_USER="$(id -un)" DYX3_ALLOW_ANY_OS=1 DYX3_DRY_RUN=1 "${REPO}/installer/install.sh" --production --dry-run --ref HEAD 2>&1)"
  check "install dry-run keeps an existing service user (no useradd)" 'printf "%s" "${out_user}" | grep -q "user $(id -un) exists" && ! printf "%s" "${out_user}" | grep -q useradd'
  check "install without --production is refused" '! "${REPO}/installer/install.sh" >/dev/null 2>&1'
  if [ "$(id -u)" -ne 0 ]; then
    out="$(env -u DYX3_ROOT "${REPO}/installer/verify.sh" 2>&1)"
    check "dyx3-health refuses to run without root" 'printf "%s" "${out}" | grep -q "must run as root"'
  else
    ok "dyx3-health root check skipped (tests run as root)"
  fi

  # INS-007: a fresh install stops before any build with the per-rover inputs it lacks and the files to edit.
  check "ros.env template ships loopback-only DDS as the default (matches PX4 UXRCE_DDS_PTCFG=1)" 'grep -qx "DYX3_ROS_LOCALHOST_ONLY=1" "${REPO}/deployment/network/ros.env.tmpl" && awk "\$3 == \"UXRCE_DDS_PTCFG\" { print \$4 }" "${REPO}/config/px4/3wd_6x_carry_from_proto.params" | grep -qx 1'
  check "fastdds_profiles.xml is well-formed and carries the two derived defaults and nothing owned by the code QoS" 'python3 -c "import xml.dom.minidom,sys; xml.dom.minidom.parse(sys.argv[1])" "${REPO}/deployment/network/fastdds_profiles.xml" && grep -q "<useBuiltinTransports>false</useBuiltinTransports>" "${REPO}/deployment/network/fastdds_profiles.xml" && grep -q "<nanosec>10000000</nanosec>" "${REPO}/deployment/network/fastdds_profiles.xml" && [ "$(grep -c "is_default_profile=\"true\"" "${REPO}/deployment/network/fastdds_profiles.xml")" -eq 2 ] && ! grep -Eq "<(kind|history|durability|depth)>" "${REPO}/deployment/network/fastdds_profiles.xml"'
  check "ros.env template ships the fleet ROS domain 42" 'grep -qx "ROS_DOMAIN_ID=42" "${REPO}/deployment/network/ros.env.tmpl"'
  check "README hotspot fleet plan matches the template (192.168.2.x; the site LAN is 192.168.3.x)" 'grep -q "192.168.2.100/24" "${REPO}/deployment/network/hotspot.env.tmpl" && grep -q "192.168.2.100/24. for the first 3WD" "${REPO}/installer/README.md" && ! grep -q "192.168.3.100" "${REPO}/installer/README.md"'
  local fresh="${T}/fresh" ffb="${T}/fresh-bin"
  make_fakebin "${ffb}"
  for s2 in useradd usermod apt-get; do printf '#!/usr/bin/env bash\necho "%s $*" >>"%s/forbidden"\nexit 1\n' "${s2}" "${T}" >"${ffb}/${s2}"; done
  chmod +x "${ffb}"/*
  out="$(PATH="${ffb}:${PATH}" DYX3_ROOT="${fresh}" DYX3_ALLOW_ANY_OS=1 "${REPO}/installer/install.sh" --production 2>&1)"
  rc=$?
  check "fresh install without a hotspot or backend address refuses early" '[ "${rc}" -ne 0 ] && printf "%s" "${out}" | grep -q "per-rover inputs are missing" && printf "%s" "${out}" | grep -q "${fresh}/etc/dyx3/hotspot.env" && printf "%s" "${out}" | grep -q "${fresh}/etc/dyx3/backend.env"'
  check "the refusal comes before any package, user or build work" '[ ! -e "${T}/forbidden" ] && [ -z "$(ls -A "${fresh}/opt/dyx3/releases" 2>/dev/null)" ]'
  check "the files to edit exist after the refusal; the domain is not reported missing" '[ -f "${fresh}/etc/dyx3/hotspot.env" ] && [ -f "${fresh}/etc/dyx3/ros.env" ] && ! printf "%s" "${out}" | grep -q "ROS_DOMAIN_ID ("'
  pf() { (DYX3_ETC="${fresh}/etc/dyx3" preflight_rover_inputs "${REPO}") >"${T}/pf" 2>&1; }
  printf 'DYX3_HOTSPOT_SSID=Rover01\nDYX3_HOTSPOT_PSK=DummyBenchPass123\n' >"${fresh}/etc/dyx3/hotspot.env"
  check "preflight: a configured hotspot satisfies the backend address" 'pf'
  printf 'DYX3_HOTSPOT_SSID=Rover01\nDYX3_HOTSPOT_PSK=DummyBenchPass123\nDYX3_HOTSPOT_ADDRESS=192.168.2.100/24\n' >"${fresh}/etc/dyx3/hotspot.env"
  check "preflight: a hotspot address the backend does not bind is named" '! pf && grep -q "DYX3_BACKEND_HOST=192.168.2.100" "${T}/pf"'
  echo "DYX3_BACKEND_HOST=192.168.2.100" >>"${fresh}/etc/dyx3/backend.env"
  sed -i.bak "s/^ROS_DOMAIN_ID=.*/#ROS_DOMAIN_ID=/" "${fresh}/etc/dyx3/ros.env" && rm -f "${fresh}/etc/dyx3/ros.env.bak"
  check "preflight: a missing ROS domain is named with its file" '! pf && grep -q "ROS_DOMAIN_ID (0-232) in ${fresh}/etc/dyx3/ros.env" "${T}/pf"'
  # FCU profile: production is static with no default route; the bench (FCU_KEEP_DHCP=1) keeps the
  # site router's DHCP route, its only WAN, or the release fetch fails (2026-10-08 on the rover).
  local net_prod net_bench
  net_prod="$(DYX3_DRY_RUN=1 install_fcu_network 2>&1)"
  net_bench="$(DYX3_DRY_RUN=1 FCU_KEEP_DHCP=1 install_fcu_network 2>&1)"
  check "fcu profile (production): manual, never-default yes" 'printf "%s" "${net_prod}" | grep -q "ipv4.method manual" && printf "%s" "${net_prod}" | grep -q "ipv4.never-default yes"'
  mkdir -p "${DYX3_ETC}"
  printf 'DYX3_LAN_ADDRESS=192.168.3.150/24\nDYX3_LAN_GATEWAY=192.168.3.1\n' >"${DYX3_ETC}/network.env"
  local net_lan
  net_lan="$(DYX3_DRY_RUN=1 install_fcu_network 2>&1)"
  check "site LAN address joins the FCU port, router default route at metric 200" 'printf "%s" "${net_lan}" | grep -q "ipv4.addresses 10.41.10.1/24,192.168.3.150/24" && printf "%s" "${net_lan}" | grep -q "ipv4.gateway 192.168.3.1" && printf "%s" "${net_lan}" | grep -q "ipv4.route-metric 200" && printf "%s" "${net_lan}" | grep -q "ipv4.never-default no"'
  printf 'DYX3_LAN_ADDRESS=10.41.10.9/24\n' >"${DYX3_ETC}/network.env"
  net_lan="$(DYX3_DRY_RUN=1 install_fcu_network 2>&1)"
  check "a site LAN address on the FCU subnet is refused" 'printf "%s" "${net_lan}" | grep -q "on the FCU subnet; ignored" && printf "%s" "${net_lan}" | grep -q "ipv4.addresses 10.41.10.1/24 "'
  printf 'DYX3_LAN_ADDRESS=10.41.0.5/16\n' >"${DYX3_ETC}/network.env"
  net_lan="$(DYX3_DRY_RUN=1 install_fcu_network 2>&1)"
  check "a site LAN /16 that contains the FCU link is refused" 'printf "%s" "${net_lan}" | grep -q "on the FCU subnet; ignored"'
  # INS-019: the bench setting persists in network.env; the environment still overrides it.
  printf 'FCU_KEEP_DHCP=1\n' >"${DYX3_ETC}/network.env"
  net_lan="$(DYX3_DRY_RUN=1 install_fcu_network 2>&1)"
  check "FCU_KEEP_DHCP=1 in network.env keeps DHCP on later runs" 'printf "%s" "${net_lan}" | grep -q "ipv4.method auto"'
  net_lan="$(DYX3_DRY_RUN=1 FCU_KEEP_DHCP=0 install_fcu_network 2>&1)"
  check "FCU_KEEP_DHCP=0 in the environment overrides network.env" 'printf "%s" "${net_lan}" | grep -q "ipv4.method manual"'
  rm -f "${DYX3_ETC}/network.env"
  check "fcu profile (bench): auto, keeps default route" 'printf "%s" "${net_bench}" | grep -q "ipv4.method auto" && printf "%s" "${net_bench}" | grep -q "ipv4.never-default no"'

  # staged real directory creation (no chown)
  (create_directories >/dev/null 2>&1)
  check "staged dirs created" '[ -d "${DYX3_VAR_LIB}/runs" ] && [ -d "${DYX3_PREFIX}/releases" ] && [ -d "${DYX3_ETC}" ]'
  check "RTK runtime directory has mode 0700" '[ "$(stat -c %a "${DYX3_VAR_LIB}/rtk" 2>/dev/null || stat -f %Lp "${DYX3_VAR_LIB}/rtk")" = 700 ]'
  # A staged root cannot chown to a dyx3 user that does not exist in CI. Capture the owner
  # arguments passed to install_dir, and check the on-disk mode separately above.
  rm -rf "${DYX3_VAR_LIB}/rtk"
  (
    install_dir() {
      printf '%s:%s %s\n' "$2" "$3" "$1" >"${T}/rtk_owner_request"
      command install -d -m "$1" "$4"
    }
    ensure_rtk_state_directory
  )
  check "RTK runtime directory requests dyx3:dyx3 ownership" '[ "$(cat "${T}/rtk_owner_request")" = "dyx3:dyx3 0700" ]'

  # CH341/BRLTTY provisioning against a fake sysfs and staged filesystem. This never loads
  # a kernel module or contacts the rover.
  local usbroot="${T}/usb-root" usbsys="${T}/usb-sysfs" usbtools="${T}/usb-tools" original_path="${PATH}"
  mkdir -p "${usbroot}/lib/udev/rules.d" "${usbsys}/1-2.1" "${usbtools}"
  printf '1a86\n' >"${usbsys}/1-2.1/idVendor"
  printf '7523\n' >"${usbsys}/1-2.1/idProduct"
  cat >"${usbroot}/lib/udev/rules.d/85-brltty.rules" <<'RULE'
# vendor rule sample
ENV{PRODUCT}=="1a86/7523/*", ENV{BRLTTY_BRAILLE_DRIVER}="bm", GOTO="brltty_usb_run"
RULE
  cat >"${usbtools}/udevadm" <<'TOOL'
#!/usr/bin/env bash
printf 'ID_PATH=platform-test-usb-0:2.1\n'
TOOL
  chmod +x "${usbtools}/udevadm"
  export PATH="${usbtools}:${PATH}" DYX3_ROOT="${usbroot}" DYX3_CH341_USB_SYSFS="${usbsys}" DYX3_CH341_KERNEL=5.15.185-tegra
  if provision_usb_serial_support >"${T}/usb-provision.out" 2>&1; then ok "CH341 source is staged for supported kernel"; else bad "CH341 source is staged for supported kernel"; fi
  check "BRLTTY exception matches only the detected physical ID_PATH" 'grep -Fq "ENV{ID_PATH}!=\"platform-test-usb-0:2.1\"" "${usbroot}/etc/udev/rules.d/85-brltty.rules" && grep -Fq "${DYX3_CH341_RULE_MARKER}" "${usbroot}/etc/udev/rules.d/85-brltty.rules"'
  check "DKMS source package and provenance are staged" '[ -s "${usbroot}/usr/src/ch341-dyx3-1.0.0/ch341.c" ] && [ -f "${usbroot}/usr/src/ch341-dyx3-1.0.0/dkms.conf" ] && grep -q "Source SHA-256" "${usbroot}/usr/src/ch341-dyx3-1.0.0/PROVENANCE"'
  check "per-rover USB identity is recorded from detected ID_PATH" 'grep -qx "DYX3_CH341_EXPECTED_ID_PATH=platform-test-usb-0:2.1" "${usbroot}/etc/dyx3/ch341-adapter.env"'
  _usb_serial_write_receiver_identity platform-test-usb-0:2.1 >"${T}/usb-receiver-pending.out" 2>&1
  check "receiver identity is not recorded for a port outside the 3WD profile" '[ ! -e "${usbroot}/etc/dyx3/usb-receiver.env" ]'
  local usb_rule_hash
  usb_rule_hash="$(sha256sum "${usbroot}/etc/udev/rules.d/85-brltty.rules" | awk '{print $1}')"
  provision_usb_serial_support >"${T}/usb-provision-repeat.out" 2>&1
  check "BRLTTY override is idempotent on repeated install" '[ "${usb_rule_hash}" = "$(sha256sum "${usbroot}/etc/udev/rules.d/85-brltty.rules" | awk "{print \$1}")" ]'
  if (DYX3_CH341_KERNEL=6.1.0-tegra _usb_serial_supported_kernel >/dev/null) >"${T}/usb-unsupported.out" 2>&1; then bad "unsupported Jetson kernel fails closed"; else ok "unsupported Jetson kernel fails closed"; fi
  mkdir -p "${usbroot}/etc/udev/rules.d"
  printf 'administrator rule\n' >"${usbroot}/etc/udev/rules.d/85-brltty.rules"
  if (_usb_serial_install_brltty_override platform-test-usb-0:2.1) >"${T}/usb-admin-rule.out" 2>&1; then bad "administrator udev override is preserved"; else ok "administrator udev override is preserved"; fi
  # Regression (rover 2026-10-09): on a first install /etc/dyx3/ch341-adapter.env does not exist yet;
  # the unguarded sed exited 2 and set -euo pipefail aborted provisioning before anything ran.
  freshroot="${T}/usb-fresh-root"
  mkdir -p "${freshroot}"
  if (set -euo pipefail; DYX3_ROOT="${freshroot}" _usb_serial_write_adapter_identity platform-fresh-usb-0:2.1) >"${T}/usb-fresh.out" 2>&1 &&
     grep -qx "DYX3_CH341_EXPECTED_ID_PATH=platform-fresh-usb-0:2.1" "${freshroot}/etc/dyx3/ch341-adapter.env"; then
    ok "first install records the adapter identity under set -euo pipefail"
  else
    bad "first install records the adapter identity under set -euo pipefail"
  fi
  # 3WD profile port: identity (by-path link + baud) is recorded only after a passive NMEA/GGA check.
  rxroot="${T}/usb-rx-root" rxdev="${T}/usb-rx-dev" rxtools="${T}/usb-rx-tools"
  mkdir -p "${rxroot}" "${rxdev}" "${rxtools}"
  : >"${rxdev}/ttyUSB0"
  cat >"${rxtools}/udevadm" <<'TOOL'
#!/usr/bin/env bash
printf 'ID_PATH=platform-3610000.usb-usb-0:2.1:1.0\n'
printf 'DEVLINKS=/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 /dev/serial/by-path/platform-3610000.usb-usb-0:2.1:1.0-port0\n'
TOOL
  chmod +x "${rxtools}/udevadm"
  if (set -euo pipefail; PATH="${rxtools}:${PATH}" DYX3_ROOT="${rxroot}" DYX3_CH341_DEV_ROOT="${rxdev}" DYX3_CH341_TEST_TTY=1
      _usb_serial_verify_receiver_stream() { [ "$1" = /dev/serial/by-path/platform-3610000.usb-usb-0:2.1:1.0-port0 ] && [ "$2" = 230400 ]; }
      _usb_serial_write_receiver_identity platform-3610000.usb-usb-0:2.1) >"${T}/usb-rx-ok.out" 2>&1 &&
     grep -qx "DYX3_USB_RECEIVER_DEVICE=/dev/serial/by-path/platform-3610000.usb-usb-0:2.1:1.0-port0" "${rxroot}/etc/dyx3/usb-receiver.env" &&
     grep -qx "DYX3_USB_RECEIVER_BAUD=230400" "${rxroot}/etc/dyx3/usb-receiver.env"; then
    ok "3WD profile receiver identity is recorded with by-path link and baud after NMEA verification"
  else
    bad "3WD profile receiver identity is recorded with by-path link and baud after NMEA verification"
  fi
  rm -f "${rxroot}/etc/dyx3/usb-receiver.env"
  if (set -euo pipefail; PATH="${rxtools}:${PATH}" DYX3_ROOT="${rxroot}" DYX3_CH341_DEV_ROOT="${rxdev}" DYX3_CH341_TEST_TTY=1
      _usb_serial_verify_receiver_stream() { return 1; }
      _usb_serial_write_receiver_identity platform-3610000.usb-usb-0:2.1) >"${T}/usb-rx-silent.out" 2>&1 &&
     [ ! -e "${rxroot}/etc/dyx3/usb-receiver.env" ]; then
    ok "no NMEA stream (or busy port) records no receiver identity"
  else
    bad "no NMEA stream (or busy port) records no receiver identity"
  fi
  check "RTK worker reads the recorded receiver identity and starts after provisioning" 'grep -qx "EnvironmentFile=-/etc/dyx3/usb-receiver.env" "${REPO}/deployment/systemd/dyx3-rtk.service" && grep -q "^After=.*dyx3-usb-serial-check.service" "${REPO}/deployment/systemd/dyx3-rtk.service"'
  # BRLTTY is masked (not removed) and recorded for the uninstaller; already-masked units are left alone.
  brlroot="${T}/usb-brltty-root"
  mkdir -p "${brlroot}"
  if (set -euo pipefail
      systemctl() {
        case "$1" in
          list-unit-files) printf '%s enabled\n' "$3" ;;
          is-enabled) [ "$2" = brltty.service ] && echo masked || echo enabled ;;
          mask) printf '%s\n' "$3" >>"${T}/brltty-masked-calls" ;;
        esac
      }
      DYX3_ROOT="${brlroot}" _usb_serial_mask_brltty) >"${T}/brltty-mask.out" 2>&1 &&
     [ "$(cat "${T}/brltty-masked-calls")" = "brltty-udev.service" ] &&
     [ "$(cat "${brlroot}/etc/dyx3/brltty-masked-by-dyx3")" = "brltty-udev.service" ]; then
    ok "BRLTTY units are masked once and recorded for the uninstaller"
  else
    bad "BRLTTY units are masked once and recorded for the uninstaller"
  fi
  # Regression (rover 2026-10-09): one unit failing to restart must not abort the upgrade before
  # health/rollback; every enabled unit is still restarted.
  if (set -euo pipefail
      systemd_available() { return 0; }
      systemctl() { printf '%s\n' "$2" >>"${T}/restart-calls"; [ "$2" != dyx3-usb-serial-check.service ]; }
      run() { "$@"; }
      restart_enabled_services "${REPO}") >"${T}/restart-continue.out" 2>&1 &&
     grep -qx "dyx3-usb-serial-check.service" "${T}/restart-calls" &&
     [ "$(grep -c . "${T}/restart-calls")" -eq "$(manifest_section enabled_services "${REPO}/installer/manifests/production.manifest" | grep -c .)" ]; then
    ok "a failed unit restart continues to the remaining units and returns to health"
  else
    bad "a failed unit restart continues to the remaining units and returns to health"
  fi
  if (set -euo pipefail; [ -z "$(_usb_serial_recorded_id_path "${T}/does-not-exist.env")" ]) >/dev/null 2>&1; then
    ok "missing adapter identity file reads as unprovisioned, not as an error"
  else
    bad "missing adapter identity file reads as unprovisioned, not as an error"
  fi
  check "administrator udev override contents remain untouched" 'grep -qx "administrator rule" "${usbroot}/etc/udev/rules.d/85-brltty.rules"'
  rm -f "${usbroot}/etc/udev/rules.d/85-brltty.rules"
  cat >"${usbtools}/lsusb" <<'TOOL'
#!/usr/bin/env bash
echo 'Bus 001 Device 004: ID 1a86:7523 USB Serial'
TOOL
  cat >"${usbtools}/modinfo" <<'TOOL'
#!/usr/bin/env bash
[ "${FAKE_CH341_MODINFO:-0}" = 1 ]
TOOL
  chmod +x "${usbtools}/lsusb" "${usbtools}/modinfo"
  export DYX3_CH341_USB_SYSFS="${usbsys}" DYX3_CH341_DEV_ROOT="${T}/usb-dev" DYX3_ROOT="${usbroot}" DYX3_CH341_KERNEL=5.15.185-tegra
  mkdir -p "${DYX3_CH341_DEV_ROOT}" "${usbsys}/1-2.1:1.0" "${T}/drivers/ch341"
  : >"${DYX3_CH341_DEV_ROOT}/ttyUSB0"
  _health_fail=0; FAKE_CH341_MODINFO=0 health_usb_serial >/dev/null 2>&1
  check "missing ch341 module is reported unhealthy" '[ "${_health_fail}" -ne 0 ]'
  _health_fail=0; FAKE_CH341_MODINFO=1 health_usb_serial >/dev/null 2>&1
  check "driver package does not pass health until ch341 binds" '[ "${_health_fail}" -ne 0 ]'
  ln -sfn "${T}/drivers/ch341" "${usbsys}/1-2.1:1.0/driver"
  _health_fail=0; FAKE_CH341_MODINFO=1 health_usb_serial >/dev/null 2>&1
  check "matching module, binding, and tty node pass health" '[ "${_health_fail}" -eq 0 ]'
  rm -rf "${usbsys}/1-2.1"
  if (provision_usb_serial_support) >"${T}/usb-missing.out" 2>&1; then bad "missing USB adapter fails with a blocker"; else ok "missing USB adapter fails with a blocker"; fi
  mkdir -p "${usbsys}/1-2.1"
  printf '1a86\n' >"${usbsys}/1-2.1/idVendor"
  printf '7523\n' >"${usbsys}/1-2.1/idProduct"
  mkdir -p "${usbsys}/1-3"
  printf '1a86\n' >"${usbsys}/1-3/idVendor"
  printf '7523\n' >"${usbsys}/1-3/idProduct"
  if (provision_usb_serial_support) >"${T}/usb-ambiguous.out" 2>&1; then bad "ambiguous USB adapter selection fails closed"; else ok "ambiguous USB adapter selection fails closed"; fi
  rm -rf "${usbsys}/1-3"
  export PATH="${original_path}" DYX3_ROOT="${T}/root"
  unset DYX3_CH341_USB_SYSFS DYX3_CH341_DEV_ROOT DYX3_CH341_KERNEL
}

# ---------------------------------------------------------------- release lifecycle
lifecycle() {
  local src="${T}/src" fakebin="${T}/fakebin"
  mkdir -p "${src}" "${fakebin}"
  cp -r "${REPO}/installer" "${REPO}/deployment" "${src}/"
  rm -rf "${src}/installer/tests"
  # The lifecycle proves switch / revert / rollback, not the service set: the staged releases enable the platform only
  # (gateway socket and backend cannot exist in a staged root). The real manifest is checked in libs().
  awk '/^\[enabled_services\]/ { print; print "dyx3-platform"; skip = 1; next }
       /^\[/ { skip = 0 }
       skip && /^dyx3-/ { next }
       { print }' "${src}/installer/manifests/production.manifest" >"${src}/manifest.tmp"
  mv "${src}/manifest.tmp" "${src}/installer/manifests/production.manifest"
  mkdir -p "${src}/ros2_ws/src"
  : >"${src}/ros2_ws/src/.keep"
  git -C "${src}" init -q -b main
  git -C "${src}" config user.email t@t
  git -C "${src}" config user.name t
  git -C "${src}" add -A
  git -C "${src}" commit -q -m A
  local A
  A="$(git -C "${src}" rev-parse HEAD)"
  echo b >"${src}/ros2_ws/src/b"
  git -C "${src}" add -A
  git -C "${src}" commit -q -m B
  local B
  B="$(git -C "${src}" rev-parse HEAD)"
  echo BREAK >"${src}/ros2_ws/src/BREAK"
  git -C "${src}" add -A
  git -C "${src}" commit -q -m C
  local C
  C="$(git -C "${src}" rev-parse HEAD)"

  # fake toolchain
  cat >"${fakebin}/colcon" <<'F'
#!/usr/bin/env bash
if [ -e ros2_ws/src/BREAK ] || [ -e src/BREAK ]; then echo "fake colcon: broken build" >&2; exit 1; fi
mkdir -p install && : >install/setup.bash
F
  cat >"${fakebin}/ss" <<'F'
#!/usr/bin/env bash
[ -e "${FAKE_NO_AGENT:-/nonexistent}" ] || echo "UNCONN 0 0 0.0.0.0:8888 0.0.0.0:*"
F
  cat >"${fakebin}/ping" <<'F'
#!/usr/bin/env bash
exit 0
F
  # sync -f <path>: record the path, whether its .complete exists yet, and what current points at.
  cat >"${fakebin}/sync" <<'F'
#!/usr/bin/env bash
[ "${1:-}" = -f ] || exit 0
printf '%s complete=%s current=%s\n' "$2" "$([ -e "$2/.complete" ] && echo 1 || echo 0)" \
  "$(basename "$(readlink -f "${DYX3_ROOT}/opt/dyx3/current" 2>/dev/null)" 2>/dev/null)" >>"${SYNC_LOG:-/dev/null}"
F
  chmod +x "${fakebin}"/*
  : >"${T}/ros_setup.bash"

  export PATH="${fakebin}:${PATH}" ROS_SETUP="${T}/ros_setup.bash" DYX3_REPO_URL="file://${src}"
  export DYX3_ROOT="${T}/lc" INSTALLER_DIR="${REPO}/installer" DYX3_SKIP_SYSTEMD=1 DYX3_SKIP_BACKEND=1 DYX3_HEALTH_SETTLE_S=0
  # shellcheck disable=SC1091
  . "${INSTALLER_DIR}/lib/common.sh"
  for l in os_check dependencies ros_install permissions network_install systemd_install health_check usb_serial release; do
    # shellcheck disable=SC1090
    . "${INSTALLER_DIR}/lib/${l}.sh"
  done
  set +e
  create_directories >/dev/null 2>&1
  printf 'persist RTK config through releases\n' >"${DYX3_VAR_LIB}/rtk/config.json"
  load_pin firmware
  local pm="${DYX3_PX4_MSGS_DIR}/${FIRMWARE_SHA}"
  mkdir -p "${pm}/install"
  touch "${pm}/install/setup.bash" "${pm}/.complete"
  echo abc123def456 >"${pm}/px4_msgs.sha256"
  check "RTK runtime state survives setup" 'grep -q "persist RTK config" "${DYX3_VAR_LIB}/rtk/config.json"'
  # Never really build px4_msgs in tests.
  # shellcheck disable=SC2317  # invoked indirectly by upgrade_to
  build_px4_msgs() { :; }

  # A failed first installation must not advertise a release or touch persistent state.
  printf 'keep mission and recorder data\n' >"${DYX3_VAR_LIB}/runs/persistent"
  : >"${T}/no_agent"
  (stop_enabled_services() { printf 'stopped\n' >"${T}/first_stop"; }; \
    FAKE_NO_AGENT="${T}/no_agent" upgrade_to "${A}") >"${T}/up_first_fail" 2>&1
  rc=$?
  check "build_release installs the launcher helper bin/dyx3-env.sh" 'ls "${DYX3_RELEASES}"/*/bin/dyx3-env.sh >/dev/null 2>&1'
  check "unhealthy first install fails" '[ "${rc}" -ne 0 ] && grep -q "first install" "${T}/up_first_fail"'
  check "failed first install stops new services" '[ -f "${T}/first_stop" ]'
  check "failed first install leaves no current symlink" '[ ! -e "${DYX3_CURRENT}" ] && [ ! -L "${DYX3_CURRENT}" ]'
  check "failed first release is marked unsuccessful and ineligible" '[ -f "${DYX3_RELEASES}/${A}/.failed" ] && [ ! -f "${DYX3_RELEASES}/${A}/.complete" ]'
  check "first-install failure preserves persistent data" 'grep -q "keep mission and recorder data" "${DYX3_VAR_LIB}/runs/persistent"'

  (upgrade_to "${A}") >"${T}/up_a" 2>&1
  rc=$?
  check "first release installs (rc=0)" '[ "${rc}" -eq 0 ]'
  check "current -> A" '[ "$(basename "$(readlink -f "${DYX3_CURRENT}")")" = "${A}" ]'
  check "release has .complete and launchers" '[ -f "${DYX3_RELEASES}/${A}/.complete" ] && [ -x "${DYX3_RELEASES}/${A}/bin/dyx3-platform" ]'
  check "config templates installed" '[ -f "${DYX3_ETC}/platform.env" ] && [ -f "${DYX3_ETC}/mavlink-router.conf" ]'
  check "operator shims installed" '[ -x "${DYX3_BIN}/dyx3-upgrade" ] && [ -x "${DYX3_BIN}/dyx3-health" ] && [ -x "${DYX3_BIN}/dyx3-install" ] && [ -x "${DYX3_BIN}/dyx3-rollback" ] && [ -x "${DYX3_BIN}/dyx3-version" ]'
  check "a shim execs the current release's script" '[ "$(sed -n 2p "${DYX3_BIN}/dyx3-upgrade")" = "exec \"${DYX3_CURRENT}/installer/upgrade.sh\" \"\$@\"" ]'
  # INS-023: shims and previous_release are replaced by rename, never rewritten in place.
  local shim_inode
  shim_inode="$(stat -c %i "${DYX3_BIN}/dyx3-upgrade")"
  install_operator_shims
  check "shims are replaced atomically (new inode, no temp file left)" '[ "$(stat -c %i "${DYX3_BIN}/dyx3-upgrade")" != "${shim_inode}" ] && [ -x "${DYX3_BIN}/dyx3-upgrade" ] && [ -z "$(find "${DYX3_BIN}" -name ".*")" ]'
  check "the Fast DDS profile is installed with the config templates" '[ -f "${DYX3_ETC}/fastdds_profiles.xml" ] && cmp -s "${DYX3_ETC}/fastdds_profiles.xml" "${REPO}/deployment/network/fastdds_profiles.xml"'
  check "config templates for ros/backend/ntrip installed" '[ -f "${DYX3_ETC}/ros.env" ] && [ -f "${DYX3_ETC}/backend.env" ] && [ -f "${DYX3_ETC}/ntrip.env" ]'
  check "hotspot template created with no credentials and mode 0640" '[ -f "${DYX3_ETC}/hotspot.env" ] && [ "$(stat -c %a "${DYX3_ETC}/hotspot.env")" = 640 ] && ! grep -Eq "^DYX3_HOTSPOT_(SSID|PSK)=." "${DYX3_ETC}/hotspot.env"'
  local hotspot_profile="${DYX3_ROOT}/etc/NetworkManager/system-connections/dyx3-hotspot.nmconnection"
  check "hotspot has no profile before configuration" '[ ! -e "${hotspot_profile}" ]'
  printf 'DYX3_HOTSPOT_SSID=TestRover\nDYX3_HOTSPOT_PSK=DummyBenchPass123\n' >"${DYX3_ETC}/hotspot.env"
  nmcli() { if [ "${1:-}" = "-t" ]; then printf '%s\n' "eth0:ethernet"; elif [ "${1:-}" = "--version" ]; then echo 'nmcli tool, version 1.50.0'; else printf '%s\n' "$*" >>"${T}/nmcli_argv"; fi; }
  install_hotspot_network >"${T}/hotspot_log" 2>&1
  check "configured hotspot skips cleanly without Wi-Fi hardware" '[ ! -e "${hotspot_profile}" ] && grep -q "no Wi-Fi device" "${T}/hotspot_log"'
  nmcli() { if [ "${1:-}" = "-t" ]; then printf '%s\n' "wlan0:wifi"; elif [ "${1:-}" = "--version" ]; then echo 'nmcli tool, version 1.50.0'; else printf '%s\n' "$*" >>"${T}/nmcli_argv"; fi; }
  lsmod() { printf 'Module Size Used by\nrtl8822ce 100 0\n'; }
  modinfo() { if [ "${1:-}" = "-p" ] && [ "${2:-}" = "rtl8822ce" ]; then echo 'rtw_power_mgnt:Power management'; else return 1; fi; }
  install_hotspot_network >>"${T}/hotspot_log" 2>&1
  check "hotspot profile created only when configured and Wi-Fi exists" '[ -f "${hotspot_profile}" ] && grep -qx "method=shared" "${hotspot_profile}" && grep -qx "address1=10.42.0.1/24" "${hotspot_profile}" && grep -qx "never-default=true" "${hotspot_profile}" && [ "$(stat -c %a "${hotspot_profile}")" = 600 ]'
  local hs_inode
  hs_inode="$(stat -c %i "${hotspot_profile}")"
  install_hotspot_network >"${T}/hotspot_again" 2>&1
  check "an unchanged hotspot profile is neither rewritten nor re-activated" 'grep -q "profile unchanged" "${T}/hotspot_again" && [ "$(stat -c %i "${hotspot_profile}")" = "${hs_inode}" ]'
  check "hotspot isolation hook blocks Wi-Fi to FCU both ways" 'grep -q -- "-i \"wlan0\" -o \"${FCU_IFACE}\" -j DROP" "${DYX3_ROOT}/etc/NetworkManager/dispatcher.d/pre-up.d/90-dyx3-hotspot-isolation" && grep -q -- "-i \"${FCU_IFACE}\" -o \"wlan0\" -j DROP" "${DYX3_ROOT}/etc/NetworkManager/dispatcher.d/pre-up.d/90-dyx3-hotspot-isolation"'
  check "client-internet cap (4WD 2026-09-28 fix) polices forwarded traffic and shortens the Wi-Fi queue" 'q="${DYX3_ROOT}/etc/NetworkManager/dispatcher.d/99-dyx3-hotspot-qos"; [ "$(stat -c %a "${q}")" = 700 ] && grep -q "txqueuelen 100" "${q}" && grep -q -- "-w -t mangle -A DYX3_HOTSPOT_QOS ! -i \"wlan0\" -o \"wlan0\" -m hashlimit --hashlimit-name dyx3_dl --hashlimit-above 2mb/s --hashlimit-burst 4mb -j DROP" "${q}" && grep -q -- "-i \"wlan0\" ! -o \"wlan0\" -m hashlimit --hashlimit-name dyx3_ul --hashlimit-above 1mb/s --hashlimit-burst 2mb -j DROP" "${q}" && bash -n "${q}"'
  check "hotspot profile pins 5 GHz channel 149 (the only AP-capable 5 GHz channel on the vendor driver) and WPA2 without power save" 'grep -qx "band=a" "${hotspot_profile}" && grep -qx "channel=149" "${hotspot_profile}" && grep -qx "channel-width=20" "${hotspot_profile}" && grep -qx "powersave=2" "${hotspot_profile}" && grep -qx "proto=rsn" "${hotspot_profile}"'
  check "Wi-Fi country is applied by persistent boot service" 'grep -qx "ExecStart=/usr/bin/env iw reg set IN" "${DYX3_ROOT}/etc/systemd/system/dyx3-wifi-regdom.service" && [ -L "${DYX3_ROOT}/etc/systemd/system/multi-user.target.wants/dyx3-wifi-regdom.service" ]'
  check "verified RTL8822CE module option is installed" 'grep -qx "options rtl8822ce rtw_power_mgnt=0" "${DYX3_ROOT}/etc/modprobe.d/dyx3-rtl8822ce-powersave.conf"'
  iw() { if [ "${1:-}" = reg ]; then echo 'global'; echo 'country IN: DFS-UNSET'; else echo 'Power save: off'; fi; }
  local wifi_health
  wifi_health="$(health_wifi)"
  check "health reports regulatory domain and power save" 'printf "%s" "${wifi_health}" | grep -q "iw reg get.*country IN" && printf "%s" "${wifi_health}" | grep -q "Power save: off"'
  lsmod() { printf 'Module Size Used by\nrtw88_8822ce 100 0\n'; }
  modinfo() { if [ "${1:-}" = "-p" ] && [ "${2:-}" = "rtw88_core" ]; then echo 'disable_lps_deep:Disable deep power save'; else return 1; fi; }
  install_hotspot_network >>"${T}/hotspot_log" 2>&1
  check "rtw88 core option is verified before installation" 'grep -qx "options rtw88_core disable_lps_deep=1" "${DYX3_ROOT}/etc/modprobe.d/dyx3-rtl8822ce-powersave.conf"'
  lsmod() { printf 'Module Size Used by\n'; }
  install_hotspot_network >>"${T}/hotspot_log" 2>&1
  check "missing RTL8822CE module skips power-save file cleanly" '[ ! -e "${DYX3_ROOT}/etc/modprobe.d/dyx3-rtl8822ce-powersave.conf" ] && grep -q "driver module not found" "${T}/hotspot_log"'
  printf 'DYX3_HOTSPOT_SSID=TestRover\nDYX3_HOTSPOT_PSK=DummyBenchPass123\nDYX3_WIFI_COUNTRY=IN\nDYX3_WIFI_BAND=bg\nDYX3_WIFI_CHANNEL=6\nDYX3_WIFI_WIDTH=40\n' >"${DYX3_ETC}/hotspot.env"
  install_hotspot_network >>"${T}/hotspot_log" 2>&1
  check "2.4 GHz 40 MHz configuration is generated" 'grep -qx "band=bg" "${hotspot_profile}" && grep -qx "channel=6" "${hotspot_profile}" && grep -qx "channel-width=40" "${hotspot_profile}"'
  cp "${hotspot_profile}" "${T}/hotspot_good"
  printf 'DYX3_HOTSPOT_SSID=TestRover\nDYX3_HOTSPOT_PSK=DummyBenchPass123\nDYX3_WIFI_BAND=a\nDYX3_WIFI_CHANNEL=52\n' >"${DYX3_ETC}/hotspot.env"
  install_hotspot_network >>"${T}/hotspot_log" 2>&1
  check "5 GHz DFS channel is refused and the working access point is kept" 'cmp -s "${T}/hotspot_good" "${hotspot_profile}" && grep -q "DFS or invalid channel refused; keeping the existing access point unchanged" "${T}/hotspot_log"'
  # INS-010: a bad hotspot.env never deletes the access point; only SSID and PSK both deliberately empty do.
  printf 'DYX3_HOTSPOT_SSID=TestRover\nDYX3_HOTSPOT_PSK=short\n' >"${DYX3_ETC}/hotspot.env"
  install_hotspot_network >>"${T}/hotspot_log" 2>&1
  check "an invalid passphrase keeps the working access point" 'cmp -s "${T}/hotspot_good" "${hotspot_profile}"'
  printf 'DYX3_HOTSPOT_SSID=TestRover\nDYX3_HOTSPOT_PSK=\n' >"${DYX3_ETC}/hotspot.env"
  install_hotspot_network >>"${T}/hotspot_log" 2>&1
  check "an SSID without a passphrase keeps the working access point" 'cmp -s "${T}/hotspot_good" "${hotspot_profile}" && grep -q "only one of DYX3_HOTSPOT_SSID" "${T}/hotspot_log"'
  printf 'DYX3_WIFI_COUNTRY=IN\n' >"${DYX3_ETC}/hotspot.env"
  install_hotspot_network >>"${T}/hotspot_log" 2>&1
  check "a hotspot.env without SSID/PSK lines keeps the working access point" 'cmp -s "${T}/hotspot_good" "${hotspot_profile}"'
  printf 'DYX3_HOTSPOT_SSID=\nDYX3_HOTSPOT_PSK=\n' >"${DYX3_ETC}/hotspot.env"
  install_hotspot_network >>"${T}/hotspot_log" 2>&1
  check "SSID and PSK both deliberately empty remove the access point" '[ ! -e "${hotspot_profile}" ] && grep -q "SSID and PSK both empty" "${T}/hotspot_log"'
  cp "${T}/hotspot_good" "${hotspot_profile}"
  nmcli() { if [ "${1:-}" = "-t" ]; then printf '%s\n' "wlan0:wifi"; elif [ "${1:-}" = "--version" ]; then echo 'nmcli tool, version 1.36.6'; else printf '%s\n' "$*" >>"${T}/nmcli_argv"; fi; }
  printf 'DYX3_HOTSPOT_SSID=TestRover\nDYX3_HOTSPOT_PSK=DummyBenchPass123\nDYX3_WIFI_WIDTH=40\n' >"${DYX3_ETC}/hotspot.env"
  install_hotspot_network >>"${T}/hotspot_log" 2>&1
  check "40 MHz is refused by old NetworkManager, keeping the access point" 'cmp -s "${T}/hotspot_good" "${hotspot_profile}" && grep -q "NetworkManager 1.50+ is required" "${T}/hotspot_log"'
  # Per-rover address and dongle-safe interface choice (2026-10-09 fleet plan).
  mkdir -p "${DYX3_ROOT}/sys/devices/platform/usbhost/usb1/1-1/net/wlx0" "${DYX3_ROOT}/sys/devices/pci0001/net/wlan0" "${DYX3_ROOT}/sys/class/net/wlx0" "${DYX3_ROOT}/sys/class/net/wlan0"
  ln -sfn "${DYX3_ROOT}/sys/devices/platform/usbhost/usb1/1-1" "${DYX3_ROOT}/sys/class/net/wlx0/device"
  ln -sfn "${DYX3_ROOT}/sys/devices/pci0001" "${DYX3_ROOT}/sys/class/net/wlan0/device"
  nmcli() { if [ "${1:-}" = "-t" ]; then printf '%s\n' "wlx0:wifi" "wlan0:wifi"; elif [ "${1:-}" = "--version" ]; then echo 'nmcli tool, version 1.36.6'; else printf '%s\n' "$*" >>"${T}/nmcli_argv"; fi; }
  printf 'DYX3_HOTSPOT_SSID=TestRover\nDYX3_HOTSPOT_PSK=DummyBenchPass123\nDYX3_HOTSPOT_ADDRESS=192.168.3.100/24\n' >"${DYX3_ETC}/hotspot.env"
  install_hotspot_network >>"${T}/hotspot_log" 2>&1
  check "hotspot uses the per-rover address and skips the USB Wi-Fi uplink" 'grep -qx "address1=192.168.3.100/24" "${hotspot_profile}" && grep -qx "interface-name=wlan0" "${hotspot_profile}"'
  printf 'DYX3_HOTSPOT_SSID=TestRover\nDYX3_HOTSPOT_PSK=DummyBenchPass123\nDYX3_HOTSPOT_IFACE=wlx0\n' >"${DYX3_ETC}/hotspot.env"
  install_hotspot_network >>"${T}/hotspot_log" 2>&1
  check "an explicitly named Wi-Fi device is used as given" 'grep -qx "interface-name=wlx0" "${hotspot_profile}"'
  printf 'DYX3_HOTSPOT_SSID=TestRover\nDYX3_HOTSPOT_PSK=DummyBenchPass123\nDYX3_HOTSPOT_IFACE=wlan9\n' >"${DYX3_ETC}/hotspot.env"
  install_hotspot_network >>"${T}/hotspot_log" 2>&1
  check "a named Wi-Fi device that does not exist creates no profile" 'grep -q "Wi-Fi device wlan9 not found" "${T}/hotspot_log"'
  rm -f "${hotspot_profile}"
  nmcli() { if [ "${1:-}" = "-t" ]; then printf '%s\n' "wlx0:wifi"; elif [ "${1:-}" = "--version" ]; then echo 'nmcli tool, version 1.36.6'; else printf '%s\n' "$*" >>"${T}/nmcli_argv"; fi; }
  printf 'DYX3_HOTSPOT_SSID=TestRover\nDYX3_HOTSPOT_PSK=DummyBenchPass123\n' >"${DYX3_ETC}/hotspot.env"
  install_hotspot_network >>"${T}/hotspot_log" 2>&1
  check "a USB-only Wi-Fi rover gets no access point" '[ ! -e "${hotspot_profile}" ] && grep -q "USB Wi-Fi is reserved for the internet uplink" "${T}/hotspot_log"'
  nmcli() { if [ "${1:-}" = "-t" ]; then printf '%s\n' "wlan0:wifi"; elif [ "${1:-}" = "--version" ]; then echo 'nmcli tool, version 1.36.6'; else printf '%s\n' "$*" >>"${T}/nmcli_argv"; fi; }
  printf 'DYX3_HOTSPOT_SSID=TestRover\nDYX3_HOTSPOT_PSK=DummyBenchPass123\nDYX3_HOTSPOT_ADDRESS=10.41.10.50/24\n' >"${DYX3_ETC}/hotspot.env"
  install_hotspot_network >>"${T}/hotspot_log" 2>&1
  check "a hotspot address on the FCU subnet is refused" '[ ! -e "${hotspot_profile}" ] && grep -q "invalid DYX3_HOTSPOT_ADDRESS" "${T}/hotspot_log"'
  printf 'DYX3_HOTSPOT_SSID=TestRover\nDYX3_HOTSPOT_PSK=DummyBenchPass123\nDYX3_HOTSPOT_ADDRESS=192.168.3.100\n' >"${DYX3_ETC}/hotspot.env"
  install_hotspot_network >>"${T}/hotspot_log" 2>&1
  check "a hotspot address without a prefix length is refused" '[ ! -e "${hotspot_profile}" ]'
  printf 'DYX3_LAN_ADDRESS=192.168.3.150/24\n' >"${DYX3_ETC}/network.env"
  printf 'DYX3_HOTSPOT_SSID=TestRover\nDYX3_HOTSPOT_PSK=DummyBenchPass123\nDYX3_HOTSPOT_ADDRESS=192.168.3.100/24\n' >"${DYX3_ETC}/hotspot.env"
  install_hotspot_network >>"${T}/hotspot_log" 2>&1
  check "a hotspot on the site LAN subnet is refused" '[ ! -e "${hotspot_profile}" ] && grep -q "on the site LAN subnet" "${T}/hotspot_log"'
  # INS-025: overlap, not string or network equality
  printf 'DYX3_LAN_ADDRESS=192.168.0.150/16\n' >"${DYX3_ETC}/network.env"
  printf 'DYX3_HOTSPOT_SSID=TestRover\nDYX3_HOTSPOT_PSK=DummyBenchPass123\nDYX3_HOTSPOT_ADDRESS=192.168.2.100/24\n' >"${DYX3_ETC}/hotspot.env"
  install_hotspot_network >"${T}/hotspot_overlap" 2>&1
  check "a hotspot inside a wider site LAN is refused" '[ ! -e "${hotspot_profile}" ] && grep -q "on the site LAN subnet" "${T}/hotspot_overlap"'
  rm -f "${DYX3_ETC}/network.env"
  printf 'DYX3_HOTSPOT_SSID=TestRover\nDYX3_HOTSPOT_PSK=DummyBenchPass123\nDYX3_HOTSPOT_ADDRESS=10.41.0.1/16\n' >"${DYX3_ETC}/hotspot.env"
  install_hotspot_network >"${T}/hotspot_overlap" 2>&1
  check "a hotspot /16 that contains the FCU link is refused" '[ ! -e "${hotspot_profile}" ] && grep -q "invalid DYX3_HOTSPOT_ADDRESS" "${T}/hotspot_overlap"'
  check "overlap: disjoint networks do not overlap" '! _ipv4_overlap 192.168.2.100/24 192.168.3.150/24 && ! _ipv4_overlap 10.42.0.1/24 10.41.10.1/24 && _ipv4_overlap 10.41.10.9/30 10.41.10.1/24'
  printf 'DYX3_HOTSPOT_SSID=TestRover\nDYX3_HOTSPOT_PSK=DummyBenchPass123\nDYX3_HOTSPOT_DOWNLOAD_LIMIT=1;reboot\n' >"${DYX3_ETC}/hotspot.env"
  install_hotspot_network >>"${T}/hotspot_log" 2>&1
  check "an invalid client-internet limit is refused (no shell injection into the hook)" '[ ! -e "${hotspot_profile}" ] && grep -q "invalid client-internet limit" "${T}/hotspot_log"'
  check "no hotspot credential appears in argv or installer logs" '! grep -q "DummyBenchPass123" "${T}/hotspot_log" "${T}/nmcli_argv" 2>/dev/null'
  install_config_templates "${DYX3_CURRENT}" >/dev/null 2>&1
  check "existing hotspot.env is never overwritten" 'grep -q "DummyBenchPass123" "${DYX3_ETC}/hotspot.env"'
  install_no_auto_updates >/dev/null 2>&1
  check "field rover never updates itself (apt periodic off)" 'c="${DYX3_ROOT}/etc/apt/apt.conf.d/99dyx3-no-auto-updates"; grep -qx "APT::Periodic::Unattended-Upgrade \"0\";" "${c}" && grep -qx "APT::Periodic::Update-Package-Lists \"0\";" "${c}"'
  check "release activation disables automatic updates" 'grep -q "^  install_no_auto_updates$" "${REPO}/installer/lib/release.sh"'
  check "versions.json written for the recorder" 'grep -q "\"stack_sha\": \"${A}\"" "${DYX3_ETC}/versions.json" && grep -q firmware_expected_sha "${DYX3_ETC}/versions.json"'
  check "systemd units copied" '[ -f "${DYX3_ROOT}/etc/systemd/system/dyx3-platform.service" ]'
  # INS-011: units the release does not ship are removed; the network code's regdom unit is not the release's.
  local sd="${DYX3_ROOT}/etc/systemd/system"
  mkdir -p "${sd}/multi-user.target.wants"
  printf '[Service]\nExecStart=/bin/true\n' >"${sd}/dyx3-retired.service"
  ln -sfn ../dyx3-retired.service "${sd}/multi-user.target.wants/dyx3-retired.service"
  install_units "${DYX3_CURRENT}" >"${T}/units_out" 2>&1
  check "a dyx3 unit the release does not ship is removed with its enablement" '[ ! -e "${sd}/dyx3-retired.service" ] && [ ! -L "${sd}/multi-user.target.wants/dyx3-retired.service" ] && grep -q "removing dyx3-retired.service" "${T}/units_out"'
  check "the Wi-Fi regdom unit and the shipped units are kept" '[ -f "${sd}/dyx3-wifi-regdom.service" ] && [ -L "${sd}/multi-user.target.wants/dyx3-wifi-regdom.service" ] && [ -f "${sd}/dyx3-platform.service" ] && [ -f "${sd}/dyx3-control.service" ] && [ -f "${sd}/dyx3-services.service" ]'
  (systemd_available() { return 0; }; systemctl() { printf '%s\n' "$*" >>"${T}/units_calls"; }
    printf '[Service]\n' >"${sd}/dyx3-retired.service"; install_units "${DYX3_CURRENT}") >/dev/null 2>&1
  check "under systemd the retired unit is stopped and disabled" 'grep -qx "stop dyx3-retired.service" "${T}/units_calls" && grep -qx "disable dyx3-retired.service" "${T}/units_calls" && ! grep -q "dyx3-wifi-regdom" "${T}/units_calls"'

  # This ledger belongs to the PX4 correlation epoch, not a software release.
  printf 'v2 12345\n' >"${DYX3_VAR_LIB}/state/px4_link_spray_ack_next"
  (create_directories >/dev/null 2>&1)
  check "reinstall preserves spray correlation ledger" '[ "$(cat "${DYX3_VAR_LIB}/state/px4_link_spray_ack_next")" = "v2 12345" ]'

  echo "EDITED=1" >>"${DYX3_ETC}/platform.env"
  (SYNC_LOG="${T}/sync_log" upgrade_to "${B}") >"${T}/up_b" 2>&1
  rc=$?
  check "the release is flushed to disk before .complete is written" 'grep -qx "${DYX3_RELEASES}/${B} complete=0 current=${A}" "${T}/sync_log"'
  check "the switch is flushed to disk" 'grep -qx "${DYX3_PREFIX} complete=0 current=${B}" "${T}/sync_log"'
  check "upgrade to B (rc=0)" '[ "${rc}" -eq 0 ]'
  check "current -> B, previous recorded as A" '[ "$(basename "$(readlink -f "${DYX3_CURRENT}")")" = "${B}" ] && [ "$(cat "${DYX3_VAR_LIB}/state/previous_release")" = "${A}" ]'
  check "previous_release is written by rename, no temp file left" '[ -z "$(find "${DYX3_VAR_LIB}/state" -maxdepth 1 -name ".previous_release.*")" ] && [ "$(stat -c %a "${DYX3_VAR_LIB}/state/previous_release")" = 644 ]'
  check "upgrade never overwrites edited /etc config" 'grep -q "EDITED=1" "${DYX3_ETC}/platform.env"'
  check "upgrade preserves spray correlation ledger" '[ "$(cat "${DYX3_VAR_LIB}/state/px4_link_spray_ack_next")" = "v2 12345" ]'
  check "upgrade preserves RTK runtime config" 'grep -q "persist RTK config" "${DYX3_VAR_LIB}/rtk/config.json"'

  (upgrade_to "${C}") >"${T}/up_c" 2>&1
  rc=$?
  check "broken build is refused (rc!=0)" '[ "${rc}" -ne 0 ]'
  check "broken build leaves current on B" '[ "$(basename "$(readlink -f "${DYX3_CURRENT}")")" = "${B}" ]'
  check "broken build leaves no .complete" '[ ! -f "${DYX3_RELEASES}/${C}/.complete" ]'

  # post-switch health failure must revert automatically
  echo b2 >"${src}/ros2_ws/src/b2"
  git -C "${src}" rm -q --cached ros2_ws/src/BREAK && rm -f "${src}/ros2_ws/src/BREAK"
  git -C "${src}" add -A && git -C "${src}" commit -q -m D
  local D
  D="$(git -C "${src}" rev-parse HEAD)"
  : >"${T}/no_agent"
  rm -f "${T}/agent_down"
  (install_hotspot_network() { touch "${T}/hs_on_fail"; }; restart_enabled_services() { agent_toggle; }
    FAKE_NO_AGENT="${T}/agent_down" upgrade_to "${D}") >"${T}/up_d" 2>&1
  rc=$?
  check "unhealthy upgrade fails (rc!=0)" '[ "${rc}" -ne 0 ]'
  check "unhealthy upgrade reverted to B" '[ "$(basename "$(readlink -f "${DYX3_CURRENT}")")" = "${B}" ]'
  check "the revert is health-checked and the result is in the final message" 'grep -q "health of ${B:0:10} after the revert: OK" "${T}/up_d"'
  check "a failed health gate keeps the verified build" '[ -f "${DYX3_RELEASES}/${D}/.verified" ] && [ -f "${DYX3_RELEASES}/${D}/ros2_ws/install/setup.bash" ]'
  check "unhealthy upgrade is ineligible and records its failure" '[ ! -f "${DYX3_RELEASES}/${D}/.complete" ] && [ -f "${DYX3_RELEASES}/${D}/.failed" ]'
  check "the access point is not touched by an upgrade that fails health" '[ ! -e "${T}/hs_on_fail" ]'
  check "a failed upgrade leaves no in-progress marker" '[ ! -e "${DYX3_VAR_LIB}/state/upgrade_in_progress" ]'
  check "unhealthy upgrade restores recorded version and persistent state" 'grep -q "\"stack_sha\": \"${B}\"" "${DYX3_ETC}/versions.json" && grep -q "keep mission and recorder data" "${DYX3_VAR_LIB}/runs/persistent"'

  # A static failure happens after a successful build but before activation.
  rm -f "${src}/deployment/scripts/start-platform.sh"
  git -C "${src}" add -A && git -C "${src}" commit -q -m E
  local E
  E="$(git -C "${src}" rev-parse HEAD)"
  (upgrade_to "${E}") >"${T}/up_static_fail" 2>&1
  rc=$?
  check "static verification rejects a built release" '[ "${rc}" -ne 0 ] && grep -q "failed verification" "${T}/up_static_fail"'
  check "static verification failure leaves no completion marker" '[ ! -f "${DYX3_RELEASES}/${E}/.complete" ] && [ "$(basename "$(readlink -f "${DYX3_CURRENT}")")" = "${B}" ]'

  # shellcheck disable=SC2094  # reads the log the run is writing, on purpose: health must already be logged OK
  (install_hotspot_network() { grep -q "health: OK" "${T}/up_d2" && touch "${T}/hs_after_health"; }; upgrade_to "${D}") >"${T}/up_d2" 2>&1
  rc=$?
  check "healthy retry of D succeeds" '[ "${rc}" -eq 0 ] && [ "$(basename "$(readlink -f "${DYX3_CURRENT}")")" = "${D}" ]'
  check "the retry reuses the verified build" 'grep -q "${D:0:10} already built" "${T}/up_d2"'
  check "the access point is configured only after the health gate passed" '[ -e "${T}/hs_after_health" ]'
  check "a completed upgrade leaves no in-progress marker" '[ ! -e "${DYX3_VAR_LIB}/state/upgrade_in_progress" ]'

  (DYX3_KEEP_RELEASES=1 prune_releases) >/dev/null 2>&1
  rc=$?
  check "prune keeps current and previous only" '[ -d "${DYX3_RELEASES}/${D}" ] && [ -d "${DYX3_RELEASES}/${B}" ] && [ ! -d "${DYX3_RELEASES}/${A}" ]'

  (upgrade_to "${D}") >"${T}/up_noop" 2>&1
  rc=$?
  check "re-running on the current SHA is a no-op" 'grep -q "nothing to do" "${T}/up_noop"'
  (upgrade_to nonexistent-ref) >"${T}/up_bad" 2>&1
  rc=$?
  check "unknown ref is refused" '[ "${rc}" -ne 0 ]'

  # ---- INS-004: a run killed between the switch and the health result
  (restart_enabled_services() { exit 9; }; upgrade_to "${B}") >"${T}/up_killed" 2>&1
  check "a killed upgrade leaves the in-progress marker" 'grep -qx "target=${B}" "${DYX3_VAR_LIB}/state/upgrade_in_progress" && grep -qx "previous=${D}" "${DYX3_VAR_LIB}/state/upgrade_in_progress"'
  (FAKE_NO_AGENT="${T}/no_agent" upgrade_to "${D}") >"${T}/up_after_kill" 2>&1
  rc=$?
  check "the next run reports it and reverts an unhealthy result" '[ "${rc}" -ne 0 ] && grep -q "interrupted upgrade was found" "${T}/up_after_kill" && grep -q "was reverted to ${D:0:10}" "${T}/up_after_kill" && [ "$(basename "$(readlink -f "${DYX3_CURRENT}")")" = "${D}" ] && [ ! -e "${DYX3_VAR_LIB}/state/upgrade_in_progress" ]'
  printf 'kind=upgrade\ntarget=%s\nprevious=%s\nstarted=x\n' "${D}" "${B}" >"${DYX3_VAR_LIB}/state/upgrade_in_progress"
  (upgrade_to "${D}") >"${T}/up_kill_ok" 2>&1
  rc=$?
  check "a healthy interrupted result is kept and the run continues" '[ "${rc}" -eq 0 ] && grep -q "is healthy: keeping it" "${T}/up_kill_ok" && grep -q "nothing to do" "${T}/up_kill_ok" && [ ! -e "${DYX3_VAR_LIB}/state/upgrade_in_progress" ]'
  printf '%s\n' "${B}" >"${DYX3_VAR_LIB}/state/previous_release"

  # ---- dyx3-version / dyx3-rollback
  (print_version) >"${T}/ver" 2>&1
  check "dyx3-version prints the stack SHA, the firmware pin and the message-set hash" 'grep -q "^stack_sha=${D}$" "${T}/ver" && grep -q "^firmware_expected_sha=8279fa4b" "${T}/ver" && grep -q "^px4_msgs_msg_set_sha256=abc123def456$" "${T}/ver"'
  check "dyx3-version admits the running firmware identity is unreadable" 'grep -q "^firmware_running=unavailable" "${T}/ver"'

  (rollback_release) >"${T}/rb1" 2>&1
  rc=$?
  check "rollback returns to B (rc=0)" '[ "${rc}" -eq 0 ] && [ "$(basename "$(readlink -f "${DYX3_CURRENT}")")" = "${B}" ]'
  check "rollback preserves RTK runtime config" 'grep -q "persist RTK config" "${DYX3_VAR_LIB}/rtk/config.json"'
  check "rollback records the release it left as previous" '[ "$(cat "${DYX3_VAR_LIB}/state/previous_release")" = "${D}" ]'
  check "versions.json describes the rolled-back release" 'grep -q "\"stack_sha\": \"${B}\"" "${DYX3_ETC}/versions.json"'
  (rollback_release) >"${T}/rb2" 2>&1
  check "a second rollback undoes the first" '[ "$(basename "$(readlink -f "${DYX3_CURRENT}")")" = "${D}" ]'
  rm -f "${T}/agent_down"
  (restart_enabled_services() { agent_toggle; }; FAKE_NO_AGENT="${T}/agent_down" rollback_release) >"${T}/rb3" 2>&1
  rc=$?
  check "an unhealthy rollback fails (rc!=0)" '[ "${rc}" -ne 0 ]'
  check "the restore is health-checked and reported" 'grep -q "health of ${D:0:10} after the restore: OK" "${T}/rb3"'
  check "an unhealthy rollback restores the release it started from" '[ "$(basename "$(readlink -f "${DYX3_CURRENT}")")" = "${D}" ] && [ "$(cat "${DYX3_VAR_LIB}/state/previous_release")" = "${B}" ]'
  printf '%s\n' "0000000000000000000000000000000000000000" >"${DYX3_VAR_LIB}/state/previous_release"
  (rollback_release) >"${T}/rb4" 2>&1
  rc=$?
  check "rollback to a pruned/unknown release is refused and changes nothing" '[ "${rc}" -ne 0 ] && [ "$(basename "$(readlink -f "${DYX3_CURRENT}")")" = "${D}" ]'
  rm -f "${DYX3_VAR_LIB}/state/previous_release"
  (rollback_release) >"${T}/rb5" 2>&1
  rc=$?
  check "rollback with nothing recorded is refused" '[ "${rc}" -ne 0 ] && grep -q "no previous release" "${T}/rb5"'

  # ---- health extras
  (health_extras "${DYX3_CURRENT}") >"${T}/hx" 2>&1
  check "health reports the data volume" 'grep -q "^PASS  disk" "${T}/hx"'

  # ---- INS-001: no switch or restart unless the rover is known idle (fake gateway on the staged socket)
  local gwf="${T}/gw_state"
  export DYX3_GATEWAY_QUERY_TIMEOUT_S=1
  check "idle: no graph unit running and no gateway socket is idle" '[ "$(idle_rc)" = 0 ]'
  gw_state "${gwf}" 1 true 0 true
  fake_gateway_start "${DYX3_GATEWAY_SOCK}" "${gwf}"
  check "idle: fresh DISARMED with the mission IDLE is idle" '[ "$(idle_rc)" = 0 ] && grep -q "disarmed, mission state 0" "${T}/idle_reason"'
  gw_state "${gwf}" 2 true 0 true
  check "idle: ARMED refuses" '[ "$(idle_rc)" = 1 ] && grep -q ARMED "${T}/idle_reason"'
  gw_state "${gwf}" 2 false 0 true
  check "idle: a stale ARMED still refuses" '[ "$(idle_rc)" = 1 ]'
  gw_state "${gwf}" 1 true 3 true
  check "idle: a RUNNING mission refuses" '[ "$(idle_rc)" = 1 ] && grep -q RUNNING "${T}/idle_reason"'
  gw_state "${gwf}" 1 false 0 true
  check "idle: a stale DISARMED is unknown, not idle" '[ "$(idle_rc)" = 2 ]'
  gw_state "${gwf}" 0 true 0 true
  check "idle: no FCU status (arming_state 0) is unknown, not idle" '[ "$(idle_rc)" = 2 ]'
  # mission contract v2: the mission arms by itself a moment after LOADING, while the vehicle still reads DISARMED
  for st in 1:LOADING 2:READY 8:PLACING 9:ARMING 10:ENGAGING; do
    gw_state "${gwf}" 1 true "${st%%:*}" true
    check "idle: a ${st##*:} mission refuses although the vehicle still reads DISARMED" '[ "$(idle_rc)" = 1 ] && grep -q "${st##*:}" "${T}/idle_reason"'
  done
  gw_state "${gwf}" 1 false 9 false
  check "idle: a stale ARMING mission still refuses" '[ "$(idle_rc)" = 1 ] && grep -q "ARMING.*stale" "${T}/idle_reason"'
  gw_state "${gwf}" 1 true 7 true 7
  check "idle: an ERROR mission still releasing OFFBOARD refuses" '[ "$(idle_rc)" = 1 ] && grep -q "releasing OFFBOARD" "${T}/idle_reason"'
  gw_state "${gwf}" 1 true 6 true 8
  check "idle: an ABORTED mission still disarming refuses" '[ "$(idle_rc)" = 1 ] && grep -q disarming "${T}/idle_reason"'
  gw_state "${gwf}" 1 true 6 true 0
  check "idle: an ABORTED mission with the release finished and the vehicle disarmed is idle" '[ "$(idle_rc)" = 0 ]'
  gw_state "${gwf}" 1 true 4 true
  check "idle: a PAUSED mission is idle but named" '[ "$(idle_rc)" = 0 ] && grep -q PAUSED "${T}/idle_reason"'
  echo silent >"${gwf}"
  check "idle: a gateway that never answers is unknown" '[ "$(idle_rc)" = 2 ]'
  echo garbage >"${gwf}"
  check "idle: a gateway that answers garbage is unknown" '[ "$(idle_rc)" = 2 ]'
  echo '{"v":1,"id":1,"ok":false,"code":"busy","reason":"x","data":{}}' >"${gwf}"
  check "idle: a refused get_snapshot is unknown" '[ "$(idle_rc)" = 2 ]'
  check "idle: a graph unit active under systemd with no answering gateway is unknown" \
    '[ "$( (systemd_available() { return 0; }; systemctl() { echo active; }; DYX3_GATEWAY_SOCK="${T}/none.sock"; idle_rc) )" = 2 ]'

  gw_state "${gwf}" 2 true 3 true
  printf '%s\n' "${B}" >"${DYX3_VAR_LIB}/state/previous_release"
  (upgrade_to "${B}") >"${T}/il_up" 2>&1
  rc=$?
  check "an upgrade is refused while ARMED and changes nothing" '[ "${rc}" -ne 0 ] && grep -q "refusing upgrade" "${T}/il_up" && [ "$(basename "$(readlink -f "${DYX3_CURRENT}")")" = "${D}" ]'
  (rollback_release) >"${T}/il_rb" 2>&1
  rc=$?
  check "a rollback is refused while a mission runs and changes nothing" '[ "${rc}" -ne 0 ] && grep -q "refusing rollback" "${T}/il_rb" && [ "$(basename "$(readlink -f "${DYX3_CURRENT}")")" = "${D}" ]'
  (DYX3_FORCE_UNSAFE=1 rollback_release) >"${T}/il_force" 2>&1
  rc=$?
  check "DYX3_FORCE_UNSAFE=1 overrides the interlock loudly" '[ "${rc}" -eq 0 ] && grep -q "DYX3_FORCE_UNSAFE=1: rollback" "${T}/il_force" && [ "$(basename "$(readlink -f "${DYX3_CURRENT}")")" = "${B}" ]'

  # A mission started during the build: the switch is refused.
  echo f >"${src}/ros2_ws/src/f"
  git -C "${src}" checkout -q "${D}" -- deployment/scripts/start-platform.sh
  git -C "${src}" add -A && git -C "${src}" commit -q -m F
  local F
  F="$(git -C "${src}" rev-parse HEAD)"
  eval "$(declare -f build_release | sed '1s/build_release/_real_build_release/')"
  gw_state "${gwf}" 1 true 0 true
  (build_release() { _real_build_release "$@" && gw_state "${gwf}" 1 true 3 true; }; upgrade_to "${F}") >"${T}/il_switch" 2>&1
  rc=$?
  check "the switch is refused when a mission started during the build" '[ "${rc}" -ne 0 ] && grep -q "refusing the switch" "${T}/il_switch" && [ "$(basename "$(readlink -f "${DYX3_CURRENT}")")" = "${B}" ]'
  # Armed right after the switch, then health fails: no revert under a moving rover.
  gw_state "${gwf}" 1 true 0 true
  rm -f "${T}/agent_down"
  (restart_enabled_services() { gw_state "${gwf}" 2 true 0 true; : >"${T}/agent_down"; }
    FAKE_NO_AGENT="${T}/agent_down" upgrade_to "${F}") >"${T}/il_revert" 2>&1
  rc=$?
  check "a failed upgrade is not reverted under an ARMED rover" '[ "${rc}" -ne 0 ] && grep -q "NOT reverting" "${T}/il_revert" && [ "$(basename "$(readlink -f "${DYX3_CURRENT}")")" = "${F}" ]'
  gw_state "${gwf}" 1 true 0 true
  rm -f "${T}/agent_down"
  (rollback_release) >"${T}/il_after" 2>&1
  check "once idle, dyx3-rollback returns to the healthy release" '[ "$(basename "$(readlink -f "${DYX3_CURRENT}")")" = "${B}" ]'
  kill "${FAKE_GW_PID}" 2>/dev/null
  wait "${FAKE_GW_PID}" 2>/dev/null
  check "idle: a stale socket file with the graph units stopped is idle" '[ -S "${DYX3_GATEWAY_SOCK}" ] && [ "$(idle_rc)" = 0 ]'
  rm -f "${DYX3_GATEWAY_SOCK}"

  # ---- X-013: the deep graph check lists /rpp
  printf '#!/usr/bin/env bash\n[ "$1 $2" = "node list" ] && cat "%s"\n' "${T}/nodes" >"${fakebin}/ros2"
  chmod +x "${fakebin}/ros2"
  printf '%s\n' /dyx3_mission /motion_guard /px4_link /spray /system_gateway >"${T}/nodes"
  make_ros_rel "${T}/x013_rel"
  (health_graph "${T}/x013_rel") >"${T}/graph_out" 2>&1
  check "deep health warns when /rpp is not in the graph" 'grep -q "^WARN  node /rpp not visible" "${T}/graph_out"'
  echo /rpp >>"${T}/nodes"
  (health_graph "${T}/x013_rel") >"${T}/graph_out" 2>&1
  check "deep health reports /rpp with the other control nodes" 'grep -q "^PASS  node /rpp up" "${T}/graph_out" && [ "$(grep -c "^PASS  node" "${T}/graph_out")" -eq 6 ]'

  # ---- INS-005: a service must stay up through the hold with no restart; the gateway must answer
  echo 0 >"${T}/nrestarts"
  : >"${T}/svc_state"
  fake_systemctl() {
    case "$*" in
      "show -p ActiveState --value dyx3-platform.service") if [ -s "${T}/svc_state" ]; then cat "${T}/svc_state"; else echo active; fi ;;
      "show -p NRestarts --value dyx3-platform.service")
        cat "${T}/nrestarts"
        [ -e "${T}/flapping" ] && echo $(($(cat "${T}/nrestarts") + 1)) >"${T}/nrestarts"
        ;;
    esac
  }
  svc_health() { (systemd_available() { return 0; }; systemctl() { fake_systemctl "$@"; }; DYX3_HEALTH_HOLD_S=2 health_platform "${DYX3_CURRENT}") >"${T}/svc_out" 2>&1; }
  svc_health
  check "service hold: a unit that stays active with no restart passes" 'grep -q "^PASS  dyx3-platform.service active for 2 s with no restart" "${T}/svc_out" && ! grep -q "^FAIL" "${T}/svc_out"'
  : >"${T}/flapping"
  svc_health
  check "service hold: a unit that restarts during the hold fails (crash loop)" 'grep -q "^FAIL  dyx3-platform.service restarted during the 2 s hold" "${T}/svc_out"'
  rm -f "${T}/flapping"
  echo failed >"${T}/svc_state"
  (systemd_available() { return 0; }; systemctl() { fake_systemctl "$@"; }; DYX3_HEALTH_SETTLE_S=1 health_platform "${DYX3_CURRENT}") >"${T}/svc_out" 2>&1
  check "service hold: a unit that never becomes active fails" 'grep -q "^FAIL  dyx3-platform.service is .failed." "${T}/svc_out"'
  gw_state "${gwf}" 1 true 0 true
  fake_gateway_start "${DYX3_GATEWAY_SOCK}" "${gwf}"
  check "gateway health: a gateway that answers get_snapshot passes" '_gateway_up'
  echo silent >"${gwf}"
  check "gateway health: a silent gateway fails" '! DYX3_GATEWAY_PING_TIMEOUT_S=1 _gateway_up'
  kill "${FAKE_GW_PID}" 2>/dev/null
  wait "${FAKE_GW_PID}" 2>/dev/null
  check "gateway health: a socket file with nothing behind it fails" '[ -S "${DYX3_GATEWAY_SOCK}" ] && ! _gateway_up'
  rm -f "${DYX3_GATEWAY_SOCK}"

  # ---- INS-012: an invalid artifact stops the upgrade in auto mode too (never a silent source build)
  (install_prebuilt() { return 2; }; upgrade_to "${D}") >"${T}/pb_invalid" 2>&1
  rc=$?
  check "an INVALID prebuilt artifact stops an auto-mode upgrade before any build or switch" '[ "${rc}" -ne 0 ] && grep -q "present but INVALID" "${T}/pb_invalid" && ! grep -q "building ${D:0:10} on this machine" "${T}/pb_invalid" && [ "$(basename "$(readlink -f "${DYX3_CURRENT}")")" = "${B}" ]'
  (install_prebuilt() { return 1; }; DYX3_ARTIFACTS=prebuilt upgrade_to "${D}") >"${T}/pb_none" 2>&1
  check "a missing artifact stops a prebuilt-mode upgrade" 'grep -q "no prebuilt artifacts for" "${T}/pb_none"'

  # ---- INS-006: a fault already present before the switch (agent down on B too) does not fail the upgrade
  (FAKE_NO_AGENT="${T}/no_agent" upgrade_to "${F}") >"${T}/bl_up" 2>&1
  rc=$?
  check "baseline: a pre-existing failure does not revert the upgrade" '[ "${rc}" -eq 0 ] && [ "$(basename "$(readlink -f "${DYX3_CURRENT}")")" = "${F}" ] && grep -q "already failing before the switch" "${T}/bl_up" && grep -q "health: OK apart from failures present before the switch" "${T}/bl_up"'
  check "baseline: it is saved and names the failing check" 'grep -q "^FAIL  nothing listening on udp/8888" "${DYX3_VAR_LIB}/state/health_baseline"'
  (FAKE_NO_AGENT="${T}/no_agent" rollback_release) >"${T}/bl_rb" 2>&1
  rc=$?
  check "baseline: nor the rollback" '[ "${rc}" -eq 0 ] && [ "$(basename "$(readlink -f "${DYX3_CURRENT}")")" = "${B}" ]'
}

# ---------------------------------------------------------------- INS-002/003: the target's own installer
handoff() {
  local src="${T}/ho_src" fb="${T}/ho_bin" out
  make_fakebin "${fb}"
  make_src "${src}"
  # Releases A and B differ only in what their install_hotspot_network does.
  local net="${src}/installer/lib/network_install.sh"
  printf '\ninstall_hotspot_network() { echo A >"${DYX3_ROOT}/hotspot_marker"; }\n' >>"${net}"
  git -C "${src}" commit -q -am "A: hotspot marker A"
  local A B C D
  A="$(git -C "${src}" rev-parse HEAD)"
  sed -i.bak 's/echo A >/echo B >/' "${net}" && rm -f "${net}.bak"
  git -C "${src}" commit -q -am "B: hotspot marker B"
  B="$(git -C "${src}" rev-parse HEAD)"

  local root="${T}/ho_root" fwpin
  fwpin="$(sed -n 's/^FIRMWARE_SHA=//p' "${REPO}/installer/pins/firmware.pin")"
  mkdir -p "${root}/opt/dyx3/px4_msgs/${fwpin}/install"
  touch "${root}/opt/dyx3/px4_msgs/${fwpin}/install/setup.bash" "${root}/opt/dyx3/px4_msgs/${fwpin}/.complete"
  echo abc >"${root}/opt/dyx3/px4_msgs/${fwpin}/px4_msgs.sha256"
  mkdir -p "${root}/var/lib/dyx3/state" "${root}/var/lib/dyx3/runs" "${root}/var/lib/dyx3/rtk" "${root}/etc/dyx3" "${root}/opt/dyx3/bin"
  chmod 0700 "${root}/var/lib/dyx3/rtk"
  : >"${T}/ho_ros.bash"
  up() { # up <installer-dir> <ref> [env...]: run that installer's upgrade.sh as the operator would, in the staged root
    local dir="$1" r="$2"
    shift 2
    env -u INSTALLER_DIR PATH="${fb}:${PATH}" DYX3_ROOT="${root}" DYX3_REPO_URL="file://${src}" ROS_SETUP="${T}/ho_ros.bash" \
      DYX3_SKIP_SYSTEMD=1 DYX3_SKIP_BACKEND=1 DYX3_HEALTH_SETTLE_S=0 DYX3_ARTIFACTS=source DYX3_ALLOW_ANY_OS=1 "$@" \
      bash "${dir}/upgrade.sh" "${r}"
  }
  cur() { basename "$(readlink -f "${root}/opt/dyx3/current")"; }

  up "${REPO}/installer" "${A}" >"${T}/ho_a" 2>&1
  check "handoff: the first release is installed by its own installer" 'grep -q "upgrade complete" "${T}/ho_a" && [ "$(cur)" = "${A}" ] && grep -q "handing over to the installer of ${A:0:10}" "${T}/ho_a" && [ "$(cat "${root}/hotspot_marker")" = A ]'
  up "${root}/opt/dyx3/current/installer" "${B}" >"${T}/ho_b" 2>&1
  check "handoff: A -> B runs B's install_hotspot_network, not A's" 'grep -q "upgrade complete" "${T}/ho_b" && [ "$(cur)" = "${B}" ] && [ "$(cat "${root}/hotspot_marker")" = B ]'

  # The interlock answers in the operator's terminal, before the detach and before any fetch.
  gw_state "${T}/ho_gw" 2 true 3 true
  fake_gateway_start "${root}/run/dyx3/gateway.sock" "${T}/ho_gw"
  up "${root}/opt/dyx3/current/installer" "${A}" >"${T}/ho_armed" 2>&1
  check "upgrade.sh refuses while ARMED before detaching or fetching" 'grep -q "refusing upgrade to ${A}" "${T}/ho_armed" && ! grep -q "handing over" "${T}/ho_armed" && [ "$(cur)" = "${B}" ]'
  out="$(env -u INSTALLER_DIR PATH="${fb}:${PATH}" DYX3_ROOT="${root}" bash "${root}/opt/dyx3/current/installer/rollback.sh" 2>&1)"
  check "rollback.sh refuses while ARMED before detaching" 'printf "%s" "${out}" | grep -q "refusing rollback" && [ "$(cur)" = "${B}" ]'
  kill "${FAKE_GW_PID}" 2>/dev/null
  wait "${FAKE_GW_PID}" 2>/dev/null
  rm -f "${root}/run/dyx3/gateway.sock"

  # C changes the firmware pin: px4_msgs must be built and expected for C's pin, from C's pin file.
  local fw="${T}/ho_fw" sk="${T}/ho_px4msgs" fwsha sksha
  mkdir -p "${fw}/msg/versioned" "${fw}/srv" "${sk}/msg"
  echo "uint64 timestamp" >"${fw}/msg/VehicleStatus.msg"
  echo "uint64 timestamp" >"${fw}/msg/versioned/VehicleLocalPosition.msg"
  echo "---" >"${fw}/srv/VehicleCommand.srv"
  : >"${sk}/CMakeLists.txt"
  : >"${sk}/package.xml"
  echo "stock" >"${sk}/msg/Stock.msg"
  local r
  for r in "${fw}" "${sk}"; do
    git -C "${r}" init -q -b main && git -C "${r}" add -A &&
      git -C "${r}" -c user.email=t@t -c user.name=t commit -q -m pin &&
      git -C "${r}" config uploadpack.allowFilter true && git -C "${r}" config uploadpack.allowAnySHA1InWant true
  done
  fwsha="$(git -C "${fw}" rev-parse HEAD)"
  sksha="$(git -C "${sk}" rev-parse HEAD)"
  printf 'FIRMWARE_REPO=file://%s\nFIRMWARE_BRANCH=main\nFIRMWARE_SHA=%s\nPX4_MSGS_REPO=file://%s\nPX4_MSGS_SKELETON_REF=%s\n' \
    "${fw}" "${fwsha}" "${sk}" "${sksha}" >"${src}/installer/pins/firmware.pin"
  git -C "${src}" commit -q -am "C: new firmware pin"
  C="$(git -C "${src}" rev-parse HEAD)"
  up "${root}/opt/dyx3/current/installer" "${C}" >"${T}/ho_c" 2>&1
  check "handoff: a pin change builds px4_msgs for the TARGET's pin" 'grep -q "upgrade complete" "${T}/ho_c" && [ "$(cur)" = "${C}" ] && [ -f "${root}/opt/dyx3/px4_msgs/${fwsha}/.complete" ] && [ "$(cat "${root}/opt/dyx3/px4_msgs/${fwsha}/firmware.sha")" = "${fwsha}" ] && [ -s "${root}/opt/dyx3/px4_msgs/${fwsha}/px4_msgs.sha256" ]'
  check "handoff: versions.json expects the target's firmware pin" 'grep -q "\"firmware_expected_sha\": \"${fwsha}\"" "${root}/etc/dyx3/versions.json"'
  out="$(env -u INSTALLER_DIR PATH="${fb}:${PATH}" DYX3_ROOT="${root}" ROS_SETUP="${T}/ho_ros.bash" DYX3_SKIP_SYSTEMD=1 \
    DYX3_HEALTH_SETTLE_S=0 bash "${root}/opt/dyx3/current/installer/rollback.sh" 2>&1)"
  check "handoff: a rollback across the pin change checks and records the older release's own pin" '[ "$(cur)" = "${B}" ] && grep -q "\"firmware_expected_sha\": \"${fwpin}\"" "${root}/etc/dyx3/versions.json"'

  # D declares an older installer API: refused unless DYX3_FORCE=1.
  sed -i.bak 's/^DYX3_INSTALLER_API=.*/DYX3_INSTALLER_API=0/' "${src}/installer/lib/common.sh" && rm -f "${src}/installer/lib/common.sh.bak"
  git -C "${src}" commit -q -am "D: older installer API"
  D="$(git -C "${src}" rev-parse HEAD)"
  up "${root}/opt/dyx3/current/installer" "${D}" >"${T}/ho_d" 2>&1
  check "handoff: a target with an older installer API is refused" '[ "$(cur)" = "${B}" ] && grep -q "is API 0, older than this one" "${T}/ho_d"'
  up "${root}/opt/dyx3/current/installer" "${D}" DYX3_FORCE=1 >"${T}/ho_d2" 2>&1
  check "handoff: DYX3_FORCE=1 installs it with its own (older) installer" '[ "$(cur)" = "${D}" ] && grep -q "OLDER installer" "${T}/ho_d2"'
  out="$(up "${root}/var/lib/dyx3/state/installer-stage/${D}/installer" "${D}" DYX3_REEXEC=1 DYX3_TARGET_SHA="${D}" DYX3_PARENT_API=9 2>&1)"
  check "handoff: the target side refuses a newer parent too" 'printf "%s" "${out}" | grep -q "older than the one that started it"'
  out="$(up "${root}/var/lib/dyx3/state/installer-stage/${D}/installer" "${D}" DYX3_REEXEC=1 DYX3_TARGET_SHA="${B}" 2>&1)"
  check "handoff: the target refuses a SHA other than the one handed over" 'printf "%s" "${out}" | grep -q "parent resolved"'
}

# ---------------------------------------------------------------- INS-009: the lock is not inherited by children
locking() {
  local fb="${T}/lk_bin" lk="${T}/lk/run/dyx3/install.lock" i
  make_fakebin "${fb}"
  export PATH="${fb}:${PATH}" INSTALLER_DIR="${REPO}/installer" DYX3_ROOT="${T}/lk"
  # shellcheck disable=SC1091
  . "${INSTALLER_DIR}/lib/common.sh"
  set +e
  holder() {
    echo "${BASHPID}" >"${T}/lk_holder"
    sleep 297 &
    echo $! >"${T}/lk_child"
    wait
  }
  (with_lock "${lk}" holder) >/dev/null 2>&1 &
  local outer=$!
  for i in $(seq 50); do [ -s "${T}/lk_child" ] && break; sleep 0.1; done
  check "lock: a second run is refused while the first is alive" '! (with_lock "${lk}" true) 2>/dev/null'
  kill -9 "$(cat "${T}/lk_holder")" "${outer}" 2>/dev/null
  wait "${outer}" 2>/dev/null
  check "lock: a killed run's long-lived child does not keep the lock" 'kill -0 "$(cat "${T}/lk_child")" 2>/dev/null && (with_lock "${lk}" true) 2>/dev/null'
  kill "$(cat "${T}/lk_child")" 2>/dev/null
}

# ---------------------------------------------------------------- prebuilt artifacts
prebuilt() {
  if ! tar --zstd -cf /dev/null --files-from /dev/null 2>/dev/null; then
    ok "prebuilt artifact tests skipped (tar has no zstd here)"
    return 0
  fi
  export INSTALLER_DIR="${REPO}/installer" DYX3_ROOT="${T}/pb" DYX3_ALLOW_ANY_OS=1 ROS_DISTRO_NAME=humble
  # shellcheck disable=SC1091
  . "${INSTALLER_DIR}/lib/common.sh"
  for l in os_check dependencies ros_install permissions network_install systemd_install health_check release; do
    # shellcheck disable=SC1090
    . "${INSTALLER_DIR}/lib/${l}.sh"
  done
  set +e
  load_pin firmware
  local sha=1111111111111111111111111111111111111111 art="${T}/art" stage="${T}/stage"
  # What CI would publish: a release tree and a px4_msgs tree under opt/dyx3, plus provenance + digests.
  mkdir -p "${stage}/opt/dyx3/releases/${sha}/ros2_ws/install" "${stage}/opt/dyx3/releases/${sha}/bin" \
    "${stage}/opt/dyx3/px4_msgs/${FIRMWARE_SHA}/install" "${art}"
  : >"${stage}/opt/dyx3/releases/${sha}/ros2_ws/install/setup.bash"
  printf '#!/bin/sh\n' >"${stage}/opt/dyx3/releases/${sha}/bin/dyx3-platform"
  chmod +x "${stage}/opt/dyx3/releases/${sha}/bin/dyx3-platform"
  : >"${stage}/opt/dyx3/releases/${sha}/bin/dyx3-env.sh"
  : >"${stage}/opt/dyx3/px4_msgs/${FIRMWARE_SHA}/install/setup.bash"
  : >"${stage}/opt/dyx3/px4_msgs/${FIRMWARE_SHA}/.complete"
  echo abc >"${stage}/opt/dyx3/px4_msgs/${FIRMWARE_SHA}/px4_msgs.sha256"
  # A real release carries the backend venv and its interpreter links: these must stay allowed.
  mkdir -p "${stage}/opt/dyx3/releases/${sha}/venv/bin" "${stage}/opt/dyx3/releases/${sha}/venv/lib"
  ln -s /usr/bin/python3 "${stage}/opt/dyx3/releases/${sha}/venv/bin/python3"
  ln -s python3 "${stage}/opt/dyx3/releases/${sha}/venv/bin/python"
  ln -s lib "${stage}/opt/dyx3/releases/${sha}/venv/lib64"
  tar -C "${stage}" --zstd -cf "${art}/release-${sha}.tar.zst" "opt/dyx3/releases/${sha}"
  tar -C "${stage}" --zstd -cf "${art}/px4_msgs-${FIRMWARE_SHA}.tar.zst" "opt/dyx3/px4_msgs/${FIRMWARE_SHA}"
  printf 'ARTIFACT_STACK_SHA=%s\nARTIFACT_FIRMWARE_SHA=%s\nARTIFACT_ROS_DISTRO=humble\nARTIFACT_CI_RUN=https://ci/run/1\n' \
    "${sha}" "${FIRMWARE_SHA}" >"${art}/artifacts.env"
  (cd "${art}" && sha256sum artifacts.env "release-${sha}.tar.zst" "px4_msgs-${FIRMWARE_SHA}.tar.zst" >SHA256SUMS)
  local rel="${DYX3_ROOT}/opt/dyx3/releases/${sha}" pm="${DYX3_ROOT}/opt/dyx3/px4_msgs/${FIRMWARE_SHA}"
  mkdir -p "${DYX3_VAR_LIB}/state" "${DYX3_RELEASES}"

  (DYX3_ARTIFACTS=source DYX3_ARTIFACT_DIR="${art}" install_prebuilt "${sha}") >/dev/null 2>&1
  rc=$?
  check "prebuilt: DYX3_ARTIFACTS=source never uses artifacts" '[ "${rc}" -ne 0 ] && [ ! -e "${rel}" ]'

  # tampered release archive: refused before anything is extracted
  cp -r "${art}" "${T}/art_bad"
  printf 'x' >>"${T}/art_bad/release-${sha}.tar.zst"
  (DYX3_ARTIFACT_DIR="${T}/art_bad" install_prebuilt "${sha}") >"${T}/pb_bad" 2>&1
  rc=$?
  check "prebuilt: a digest mismatch is refused as INVALID (2) and extracts nothing" '[ "${rc}" -eq 2 ] && grep -q "sha256 mismatch" "${T}/pb_bad" && [ ! -e "${rel}" ] && [ ! -e "${pm}" ]'

  # artifact for another firmware pin: refused
  cp -r "${art}" "${T}/art_fw"
  sed -i.bak "s/^ARTIFACT_FIRMWARE_SHA=.*/ARTIFACT_FIRMWARE_SHA=0000000000/" "${T}/art_fw/artifacts.env"
  (cd "${T}/art_fw" && sha256sum artifacts.env "release-${sha}.tar.zst" "px4_msgs-${FIRMWARE_SHA}.tar.zst" >SHA256SUMS)
  (DYX3_ARTIFACT_DIR="${T}/art_fw" install_prebuilt "${sha}") >"${T}/pb_fw" 2>&1
  rc=$?
  check "prebuilt: an artifact for another firmware pin is unusable (1), not invalid" '[ "${rc}" -eq 1 ] && grep -q "firmware pin" "${T}/pb_fw" && [ ! -e "${rel}" ]'

  # an archive member outside its prefix: refused
  mkdir -p "${T}/evil/etc" && : >"${T}/evil/etc/passwd-dyx3-test"
  cp -r "${art}" "${T}/art_evil"
  tar -C "${T}/evil" --zstd -cf "${T}/art_evil/release-${sha}.tar.zst" etc
  (cd "${T}/art_evil" && sha256sum artifacts.env "release-${sha}.tar.zst" "px4_msgs-${FIRMWARE_SHA}.tar.zst" >SHA256SUMS)
  (DYX3_ARTIFACT_DIR="${T}/art_evil" install_prebuilt "${sha}") >"${T}/pb_evil" 2>&1
  rc=$?
  check "prebuilt: an archive member outside its prefix is refused as INVALID (2)" '[ "${rc}" -eq 2 ] && grep -q "outside" "${T}/pb_evil" && [ ! -e "${DYX3_ROOT}/etc/passwd-dyx3-test" ]'

  # INS-020: a ".." member inside the prefix, and a symlink that escapes the release, are refused
  mkdir -p "${T}/art_dd"
  cp "${art}/artifacts.env" "${art}/px4_msgs-${FIRMWARE_SHA}.tar.zst" "${T}/art_dd/"
  python3 - "${T}/dd.tar" "opt/dyx3/releases/${sha}" <<'PY'
import io
import sys
import tarfile

with tarfile.open(sys.argv[1], "w") as tf:
    for name in (sys.argv[2] + "/ros2_ws/install/setup.bash", sys.argv[2] + "/../../../../../tmp/dyx3-dotdot-test"):
        info = tarfile.TarInfo(name)
        info.size = 0
        tf.addfile(info, io.BytesIO(b""))
PY
  # GNU tar has no bsdtar "@archive" member copy; compress the crafted tar as-is
  if command -v zstd >/dev/null 2>&1; then
    zstd -q -c "${T}/dd.tar" >"${T}/art_dd/release-${sha}.tar.zst"
  else
    tar --zstd -cf "${T}/art_dd/release-${sha}.tar.zst" "@${T}/dd.tar"
  fi
  (cd "${T}/art_dd" && sha256sum artifacts.env "release-${sha}.tar.zst" "px4_msgs-${FIRMWARE_SHA}.tar.zst" >SHA256SUMS)
  (DYX3_ARTIFACT_DIR="${T}/art_dd" install_prebuilt "${sha}") >"${T}/pb_dd" 2>&1
  rc=$?
  check "prebuilt: a member with a '..' component is refused as INVALID" '[ "${rc}" -eq 2 ] && grep -q "component" "${T}/pb_dd" && [ ! -e "${rel}" ] && [ ! -e "${DYX3_RELEASES}/.incoming-${sha}" ]'
  local esc="${T}/esc"
  mkdir -p "${esc}/opt/dyx3/releases/${sha}/ros2_ws/install" "${T}/art_esc"
  : >"${esc}/opt/dyx3/releases/${sha}/ros2_ws/install/setup.bash"
  ln -s ../../../../../../etc "${esc}/opt/dyx3/releases/${sha}/ros2_ws/escape"
  cp "${art}/artifacts.env" "${art}/px4_msgs-${FIRMWARE_SHA}.tar.zst" "${T}/art_esc/"
  tar -C "${esc}" --zstd -cf "${T}/art_esc/release-${sha}.tar.zst" "opt/dyx3/releases/${sha}"
  (cd "${T}/art_esc" && sha256sum artifacts.env "release-${sha}.tar.zst" "px4_msgs-${FIRMWARE_SHA}.tar.zst" >SHA256SUMS)
  (DYX3_ARTIFACT_DIR="${T}/art_esc" install_prebuilt "${sha}") >"${T}/pb_esc" 2>&1
  rc=$?
  check "prebuilt: a symlink escaping the release is refused as INVALID" '[ "${rc}" -eq 2 ] && grep -q "escapes the release" "${T}/pb_esc" && [ ! -e "${rel}" ] && [ ! -e "${DYX3_RELEASES}/.incoming-${sha}" ]'

  # missing artifacts: auto falls back (rc 1), nothing left behind
  mkdir -p "${T}/art_empty"
  (DYX3_ARTIFACT_DIR="${T}/art_empty" install_prebuilt "${sha}") >/dev/null 2>&1
  rc=$?
  check "prebuilt: missing artifacts return 1 (caller builds) and leave nothing" '[ "${rc}" -eq 1 ] && [ ! -e "${rel}" ] && [ -z "$(ls -A "${DYX3_VAR_LIB}/state")" ]'

  # INS-014: a px4_msgs archive that carries its own .complete but no install tree is refused, nothing left behind
  local bpm="${T}/badpm"
  rm -rf "${pm}" "${rel}"
  mkdir -p "${bpm}/opt/dyx3/px4_msgs/${FIRMWARE_SHA}" "${T}/art_pm"
  : >"${bpm}/opt/dyx3/px4_msgs/${FIRMWARE_SHA}/.complete"
  cp "${art}/artifacts.env" "${art}/release-${sha}.tar.zst" "${T}/art_pm/"
  tar -C "${bpm}" --zstd -cf "${T}/art_pm/px4_msgs-${FIRMWARE_SHA}.tar.zst" "opt/dyx3/px4_msgs/${FIRMWARE_SHA}"
  (cd "${T}/art_pm" && sha256sum artifacts.env "release-${sha}.tar.zst" "px4_msgs-${FIRMWARE_SHA}.tar.zst" >SHA256SUMS)
  (DYX3_ARTIFACT_DIR="${T}/art_pm" install_prebuilt "${sha}") >"${T}/pb_pm" 2>&1
  rc=$?
  check "prebuilt: an archived px4_msgs .complete is not trusted; an incomplete tree is refused" '[ "${rc}" -eq 2 ] && [ ! -e "${pm}" ] && [ ! -e "${pm}.incoming" ] && [ ! -e "${rel}" ]'
  mkdir -p "${pm}.incoming/install" && : >"${pm}.incoming/stale-from-an-interrupted-run"

  # the good set (offline directory): release + px4_msgs installed, marked prebuilt, provenance kept
  (DYX3_ARTIFACT_DIR="${art}" install_prebuilt "${sha}") >"${T}/pb_ok" 2>&1
  rc=$?
  check "prebuilt: a valid offline artifact set installs" '[ "${rc}" -eq 0 ] && [ -f "${rel}/ros2_ws/install/setup.bash" ] && [ -f "${pm}/.complete" ]'
  check "prebuilt: px4_msgs is renamed into place, without a leftover .incoming or stale files" '[ ! -e "${pm}.incoming" ] && [ ! -e "${pm}/stale-from-an-interrupted-run" ] && [ -f "${pm}/install/setup.bash" ]'
  check "prebuilt: the venv interpreter links are kept" '[ "$(readlink "${rel}/venv/bin/python3")" = /usr/bin/python3 ] && [ -L "${rel}/venv/lib64" ] && [ ! -e "${DYX3_RELEASES}/.incoming-${sha}" ]'
  check "prebuilt: release marked prebuilt, not complete, provenance kept" '[ -f "${rel}/.prebuilt" ] && [ ! -f "${rel}/.complete" ] && grep -q "ARTIFACT_CI_RUN=https://ci/run/1" "${rel}/artifacts.env"'
  check "prebuilt: build_release skips a prebuilt release" '(build_release "${sha}" 2>&1 | grep -q "nothing to build")'
  check "prebuilt: the extracted release passes static verification" '(health_release_only "${rel}" 0 >/dev/null 2>&1)'
  check "prebuilt: download state cleaned up" '[ ! -e "${DYX3_VAR_LIB}/state/artifacts-${sha}" ]'
}

# ---------------------------------------------------------------- HEALTH-DDS: deep health in the services' DDS environment
# make_ros_rel <dir>: a staged release the deep checks can load: the real manifest (dyx3-control + dyx3-services), the firmware pin,
# bin/dyx3-env.sh and an empty workspace setup.
make_ros_rel() {
  mkdir -p "$1/installer/manifests" "$1/installer/pins" "$1/bin" "$1/ros2_ws/install"
  cp "${REPO}/installer/manifests/production.manifest" "$1/installer/manifests/"
  cp "${REPO}/installer/pins/firmware.pin" "$1/installer/pins/"
  cp "${REPO}/deployment/scripts/dyx3-env.sh" "$1/bin/"
  : >"$1/ros2_ws/install/setup.bash"
}

health_ros_env() {
  local fb="${T}/hd_bin" rel="${T}/hd_rel" out
  make_fakebin "${fb}"
  # sudo -n -u <user> <cmd...>: records the call, then runs <cmd...> (no real user switch).
  cat >"${fb}/sudo" <<F
#!/usr/bin/env bash
printf '%s\n' "\$*" >>"${T}/hd_sudo"
[ "\$1" = -n ] && [ "\$2" = -u ] || exit 1
shift 3
exec "\$@"
F
  # ros2: a graph that exists only on ROS_DOMAIN_ID 42 with ROS_LOCALHOST_ONLY=1, as on the rover; any other DDS
  # environment sees nothing, as the root shell did. Every call's environment and arguments are recorded.
  cat >"${fb}/ros2" <<F
#!/usr/bin/env bash
printf 'domain=%s localhost=%s home=%s args=%s\n' "\${ROS_DOMAIN_ID:-}" "\${ROS_LOCALHOST_ONLY:-}" "\${ROS_HOME:-}" "\$*" >>"${T}/hd_ros2"
[ "\${ROS_DOMAIN_ID:-}" = 42 ] && [ "\${ROS_LOCALHOST_ONLY:-}" = 1 ] || exit 0
case "\$1 \$2" in
  "node list") if [ -f "${T}/hd_nodes_once" ]; then cat "${T}/hd_nodes_once"; rm -f "${T}/hd_nodes_once"; else cat "${T}/hd_nodes"; fi ;;
  "topic echo") [ -f "${T}/hd_status" ] && cat "${T}/hd_status" ;;
esac
exit 0
F
  chmod +x "${fb}"/*
  export PATH="${fb}:${PATH}" INSTALLER_DIR="${REPO}/installer" DYX3_ROOT="${T}/hd" ROS_SETUP="${T}/hd_ros_setup.bash"
  # shellcheck disable=SC1091
  . "${INSTALLER_DIR}/lib/common.sh"
  for l in os_check dependencies ros_install permissions network_install systemd_install health_check release; do
    # shellcheck disable=SC1090
    . "${INSTALLER_DIR}/lib/${l}.sh"
  done
  set +e
  load_pin firmware
  mkdir -p "${DYX3_PX4_MSGS_DIR}/${FIRMWARE_SHA}/install" "${DYX3_ETC}"
  : >"${DYX3_PX4_MSGS_DIR}/${FIRMWARE_SHA}/install/setup.bash"
  : >"${ROS_SETUP}"
  make_ros_rel "${rel}"
  printf '# fleet domain\nROS_DOMAIN_ID=42\n\nDYX3_ROS_LOCALHOST_ONLY="1"\n' >"${DYX3_ETC}/ros.env"
  printf '%s\n' /dyx3_mission /motion_guard /px4_link /rpp /spray /system_gateway >"${T}/hd_nodes"
  # The root shell's own DDS environment must not leak into the query.
  export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0
  # as_root <cmd...>: run <cmd...> with `id -u` reporting 0, as dyx3-health under sudo.
  as_root() { (id() { if [ "$*" = -u ]; then echo 0; else command id "$@"; fi; }; "$@"); }

  # ---- the DDS environment the launchers export (dyx3-env.sh): RMW pinned, profile file only when it exists
  printf '<profiles/>\n' >"${T}/hd_profiles.xml"
  dds_env() { (
    export DYX3_RELEASE_DIR="${rel}" DYX3_ROS_SETUP="${ROS_SETUP}" ROS_DOMAIN_ID=42 ROS_HOME="${T}/hd_rh" ROS_LOG_DIR="${T}/hd_rl"
    unset RMW_IMPLEMENTATION FASTRTPS_DEFAULT_PROFILES_FILE RMW_FASTRTPS_USE_QOS_FROM_XML ROS_LOCALHOST_ONLY
    # shellcheck disable=SC1091
    . "${rel}/bin/dyx3-env.sh" && dyx3_env_load && env
  ); }
  DYX3_FASTDDS_PROFILES="${T}/hd_profiles.xml" dds_env >"${T}/hd_denv" 2>&1
  check "dyx3-env: RMW pinned to rmw_fastrtps_cpp and the profile file exported when it exists" 'grep -qx "RMW_IMPLEMENTATION=rmw_fastrtps_cpp" "${T}/hd_denv" && grep -qx "FASTRTPS_DEFAULT_PROFILES_FILE=${T}/hd_profiles.xml" "${T}/hd_denv"'
  check "dyx3-env: the per-topic QoS is never taken from XML" '! grep -q "^RMW_FASTRTPS_USE_QOS_FROM_XML=" "${T}/hd_denv"'
  DYX3_FASTDDS_PROFILES="${T}/hd_no_such.xml" dds_env >"${T}/hd_denv" 2>&1
  check "dyx3-env: no profile file, no FASTRTPS_DEFAULT_PROFILES_FILE (the RMW stays pinned)" '! grep -q "^FASTRTPS_DEFAULT_PROFILES_FILE=" "${T}/hd_denv" && grep -qx "RMW_IMPLEMENTATION=rmw_fastrtps_cpp" "${T}/hd_denv"'
  DYX3_ROS_LOCALHOST_ONLY=1 dds_env >"${T}/hd_denv" 2>&1
  check "dyx3-env: DYX3_ROS_LOCALHOST_ONLY=1 exports ROS_LOCALHOST_ONLY=1" 'grep -qx "ROS_LOCALHOST_ONLY=1" "${T}/hd_denv"'

  # ---- the node list (secondary)
  as_root health_graph "${rel}" >"${T}/hd_graph" 2>&1
  check "deep health as root: ros2 runs as dyx3 through sudo -n" 'grep -q "^-n -u dyx3 env -i PATH=" "${T}/hd_sudo"'
  check "deep health: the query runs with ros.env (domain 42, localhost-only), not the root shell's DDS environment" 'grep -q "^domain=42 localhost=1 " "${T}/hd_ros2" && ! grep -q "^domain=0" "${T}/hd_ros2" && grep -q "ROS_DOMAIN_ID=42 DYX3_ROS_LOCALHOST_ONLY=1" "${T}/hd_sudo"'
  check "deep health: a private ROS_HOME, removed afterwards" 'h="$(sed -n "s/.* home=\([^ ]*\) .*/\1/p" "${T}/hd_ros2" | head -n1)"; case "${h}" in /tmp/dyx3-health-ros.*) [ ! -e "${h}" ] ;; *) false ;; esac'
  check "deep health: node list without the ros2 daemon" 'grep -q "args=node list --no-daemon --spin-time " "${T}/hd_ros2"'
  check "deep health: the six control nodes are seen in the services' environment" '[ "$(grep -c "^PASS  node" "${T}/hd_graph")" -eq 6 ] && ! grep -q "^WARN" "${T}/hd_graph"'
  : >"${T}/hd_ros2"
  echo /px4_link >"${T}/hd_nodes_once"
  as_root health_graph "${rel}" >"${T}/hd_graph" 2>&1
  check "deep health: a node list that under-reports after a start is asked again" '[ "$(grep -c "args=node list" "${T}/hd_ros2")" -eq 2 ] && [ "$(grep -c "^PASS  node" "${T}/hd_graph")" -eq 6 ]'
  : >"${T}/hd_sudo"
  health_graph "${rel}" >"${T}/hd_graph" 2>&1
  check "deep health not as root: no sudo, same environment" '[ ! -s "${T}/hd_sudo" ] && [ "$(grep -c "^PASS  node" "${T}/hd_graph")" -eq 6 ]'
  printf 'ROS_DOMAIN_ID=42\n' >"${DYX3_ETC}/ros.env"
  as_root health_graph "${rel}" >"${T}/hd_graph" 2>&1
  check "deep health: a graph missed in another DDS environment names that environment" 'grep -q "^WARN  node /px4_link not visible (ros2 as dyx3, ROS_DOMAIN_ID=42, ROS_LOCALHOST_ONLY=0, 3 tries)" "${T}/hd_graph"'
  printf 'ROS_DOMAIN_ID=42\nDYX3_ROS_LOCALHOST_ONLY=1\n' >"${DYX3_ETC}/ros.env"
  # P2 part 3: each unit's nodes are listed only when that unit is enabled; a pre-split dyx3-ros release lists all six.
  local rm_="${rel}/installer/manifests/production.manifest"
  cp "${rm_}" "${T}/hd_manifest"
  printf '[enabled_services]\ndyx3-control\n' >"${rm_}"
  as_root health_graph "${rel}" >"${T}/hd_graph" 2>&1
  check "deep health: dyx3-control alone lists its three nodes, not the services' nodes" '[ "$(grep "^PASS  node" "${T}/hd_graph" | sed "s/^PASS  node //; s/ up$//" | LC_ALL=C sort | tr "\n" " ")" = "/motion_guard /px4_link /rpp " ]'
  printf '[enabled_services]\ndyx3-services\n' >"${rm_}"
  as_root health_graph "${rel}" >"${T}/hd_graph" 2>&1
  check "deep health: dyx3-services alone lists mission, spray and the gateway" '[ "$(grep "^PASS  node" "${T}/hd_graph" | sed "s/^PASS  node //; s/ up$//" | LC_ALL=C sort | tr "\n" " ")" = "/dyx3_mission /spray /system_gateway " ]'
  printf '[enabled_services]\ndyx3-ros\n' >"${rm_}"
  as_root health_graph "${rel}" >"${T}/hd_graph" 2>&1
  check "deep health: a pre-split release (dyx3-ros) still lists all six nodes" '[ "$(grep -c "^PASS  node" "${T}/hd_graph")" -eq 6 ]'
  cp "${T}/hd_manifest" "${rm_}"

  # ---- the px4_link sample (authoritative)
  hd_status() { printf 'stamp:\n  sec: 1760000000\n  nanosec: 0\nsession_alive: %s\nhandshake_ok: %s\noffboard_heartbeat_active: false\nfailing_to_zero: true\nfault: %s\nstale_topics_mask: %s\nworst_topic_age_s: 0.02\n---\n' "$@" >"${T}/hd_status"; }
  : >"${T}/hd_sudo"
  : >"${T}/hd_ros2"
  hd_status true true 4 0
  as_root health_dds "${rel}" >"${T}/hd_dds" 2>&1
  check "px4_link: one status sample, as dyx3 in the services' environment" 'grep -q "^-n -u dyx3 env -i " "${T}/hd_sudo" && grep -q "^domain=42 localhost=1 .*args=topic echo --once --no-daemon .*/dyx3/px4_link/status dyx3_interfaces/msg/Px4LinkStatus" "${T}/hd_ros2"'
  check "px4_link: a live session passes and reports the fields" 'grep -q "^PASS  px4_link: FCU session alive (session_alive=true handshake_ok=true fault=4 COMMAND_STALE stale_topics_mask=0; via ros2)" "${T}/hd_dds"'
  check "px4_link: /fmu topics are no longer counted" '! grep -q "topic list" "${T}/hd_ros2" && ! grep -q "/fmu topics" "${T}/hd_dds"'
  hd_status false false 1 63
  as_root health_dds "${rel}" >"${T}/hd_dds" 2>&1
  check "px4_link: a dead session warns with the stale topics named" 'grep -q "^WARN  px4_link: FCU session down (session_alive=false handshake_ok=false fault=1 NO_SESSION stale_topics_mask=63 (timesync_status vehicle_local_position vehicle_status vehicle_attitude estimator_status_flags vehicle_gps_position); via ros2)" "${T}/hd_dds"'
  hd_status true false 2 0
  as_root health_dds "${rel}" >"${T}/hd_dds" 2>&1
  check "px4_link: a live session with a handshake mismatch warns" 'grep -q "^WARN  px4_link: FCU session alive but not healthy (.*fault=2 HANDSHAKE_MISMATCH" "${T}/hd_dds"'
  rm -f "${T}/hd_status"
  as_root health_dds "${rel}" >"${T}/hd_dds" 2>&1
  check "px4_link: no sample at all warns and names the environment" 'grep -q "^WARN  px4_link: no /dyx3/px4_link/status sample from the gateway or from ros2 as dyx3 (ROS_DOMAIN_ID=42, ROS_LOCALHOST_ONLY=1); dyx3-control down?" "${T}/hd_dds" && ! grep -q "^FAIL" "${T}/hd_dds"'

  # ---- PC-7a: a live session proves PX4 UXRCE_DDS_DOM_ID = ROS_DOMAIN_ID; a dead one names the likely causes
  hd_status true true 0 0
  as_root health_dds "${rel}" >"${T}/hd_dds" 2>&1
  check "PC-7a: a live session proves the PX4 DDS domain matches ROS_DOMAIN_ID 42" 'grep -q "^PASS  DDS domain: px4_link sees PX4 on ROS_DOMAIN_ID 42, so PX4 UXRCE_DDS_DOM_ID matches" "${T}/hd_dds"'
  hd_status false false 1 1
  mkdir -p "${rel}/config/px4"
  cp "${REPO}/config/px4/3wd_6x_carry_from_proto.params" "${rel}/config/px4/"
  rm -f "${T}/hd_agent_down"
  FAKE_NO_AGENT="${T}/hd_agent_down" as_root health_dds "${rel}" >"${T}/hd_dds" 2>&1
  check "PC-7a: session down while the agent listens names UXRCE_DDS_DOM_ID and UXRCE_DDS_PTCFG" 'grep -q "^FAIL  DDS domain: the XRCE agent listens on udp/8888 but px4_link sees no PX4 session on ROS_DOMAIN_ID 42; likely PX4 UXRCE_DDS_DOM_ID != 42 or UXRCE_DDS_PTCFG != 1" "${T}/hd_dds" && ! grep -q "^PASS  DDS domain" "${T}/hd_dds"'
  # The FAIL carries the baseline values, read from the repo's parameter file at check time, next to the domain in use.
  exp_dom="$(awk '$3 == "UXRCE_DDS_DOM_ID" { print $4 }' "${REPO}/config/px4/3wd_6x_carry_from_proto.params")"
  exp_pt="$(awk '$3 == "UXRCE_DDS_PTCFG" { print $4 }' "${REPO}/config/px4/3wd_6x_carry_from_proto.params")"
  check "PC-7a: the baseline parameter values are present in the repo file (42 and 1 at the time of writing)" '[ -n "${exp_dom}" ] && [ -n "${exp_pt}" ]'
  check "PC-7a: the FAIL names the baseline UXRCE_DDS_DOM_ID/PTCFG and the ROS environment in use" 'grep -q "^FAIL  DDS domain: .*Repo baseline expects UXRCE_DDS_DOM_ID=${exp_dom} and UXRCE_DDS_PTCFG=${exp_pt}; this rover runs ROS_DOMAIN_ID=42, ROS_LOCALHOST_ONLY=1$" "${T}/hd_dds"'
  check "PC-7a: a FAIL from the agent-listens case marks the health run failed" '_health_fail=0; FAKE_NO_AGENT="${T}/hd_agent_down_none" health_dds "${rel}" >/dev/null 2>&1; [ "${_health_fail}" -eq 1 ]'
  # The baseline is read from the release, not hardcoded: change it there and the message follows.
  mkdir -p "${rel}/config/px4"
  printf '1\t1\tUXRCE_DDS_DOM_ID\t77\t6\n1\t1\tUXRCE_DDS_PTCFG\t0\t6\n' >"${rel}/config/px4/3wd_6x_carry_from_proto.params"
  FAKE_NO_AGENT="${T}/hd_agent_down_none" as_root health_dds "${rel}" >"${T}/hd_dds_rel" 2>&1
  check "PC-7a: the expected values come from the release's baseline file at check time" 'grep -q "Repo baseline expects UXRCE_DDS_DOM_ID=77 and UXRCE_DDS_PTCFG=0;" "${T}/hd_dds_rel"'
  rm -rf "${rel}/config"
  FAKE_NO_AGENT="${T}/hd_agent_down_none" as_root health_dds "${rel}" >"${T}/hd_dds_rel" 2>&1
  check "PC-7a: a release without the baseline file says so (?) and still FAILs" 'grep -q "^FAIL  DDS domain: .*Repo baseline expects UXRCE_DDS_DOM_ID=? and UXRCE_DDS_PTCFG=?;" "${T}/hd_dds_rel"'
  : >"${T}/hd_agent_down"
  FAKE_NO_AGENT="${T}/hd_agent_down" as_root health_dds "${rel}" >"${T}/hd_dds" 2>&1
  check "PC-7a: session down with nothing on the agent port says the agent is down" 'grep -q "^WARN  DDS domain: no PX4 session and nothing listens on udp/8888: the XRCE agent is down" "${T}/hd_dds"'
  printf 'DYX3_XRCE_PORT=9999\n' >"${DYX3_ETC}/platform.env"
  (have() { [ "$1" != ss ] && command -v "$1" >/dev/null 2>&1; }; as_root health_dds "${rel}") >"${T}/hd_dds" 2>&1
  check "PC-7a: without ss all three causes are named" 'grep -q "^WARN  DDS domain: no PX4 session on ROS_DOMAIN_ID 42; likely the XRCE agent is down, PX4 UXRCE_DDS_DOM_ID != 42, or UXRCE_DDS_PTCFG != 1" "${T}/hd_dds" && ! grep -q "^FAIL" "${T}/hd_dds"'
  rm -f "${DYX3_ETC}/platform.env" "${T}/hd_agent_down"

  # ---- the same sample from the gateway's get_snapshot (INS-005 query), ros2 only when it has no fresh entry
  local gwf="${T}/hd_gw"
  export DYX3_GATEWAY_QUERY_TIMEOUT_S=1
  printf '{"v":1,"id":1,"ok":true,"code":"ok","reason":"","data":{"px4_link":{"age_s":0.1,"fresh":true,"data":{"session_alive":true,"handshake_ok":true,"fault":0,"stale_topics_mask":0}},"gateway":{}}}\n' >"${gwf}"
  fake_gateway_start "${DYX3_GATEWAY_SOCK}" "${gwf}"
  : >"${T}/hd_ros2"
  as_root health_dds "${rel}" >"${T}/hd_dds" 2>&1
  check "px4_link: a fresh gateway entry is the sample; no ros2 call" 'grep -q "^PASS  px4_link: FCU session alive (session_alive=true handshake_ok=true fault=0 NONE stale_topics_mask=0; via gateway)" "${T}/hd_dds" && [ ! -s "${T}/hd_ros2" ]'
  sed -i.bak 's/"fresh":true/"fresh":false/' "${gwf}" && rm -f "${gwf}.bak"
  hd_status true true 0 0
  as_root health_dds "${rel}" >"${T}/hd_dds" 2>&1
  check "px4_link: a stale gateway entry falls back to ros2" 'grep -q "^PASS  px4_link: FCU session alive (.*; via ros2)" "${T}/hd_dds"'
  kill "${FAKE_GW_PID}" 2>/dev/null
  wait "${FAKE_GW_PID}" 2>/dev/null
  rm -f "${DYX3_GATEWAY_SOCK}"
}

# ================================================================ release size (installer/ci/slim_release.sh)
# What CI removes from a built release before packaging it: test sources, the venv's packaging tools, symbol tables.
# The slimmed tree must still pass the static verification and install through the prebuilt path.
slim_release_block() {
  export INSTALLER_DIR="${REPO}/installer" DYX3_ROOT="${T}/slim" DYX3_ALLOW_ANY_OS=1 ROS_DISTRO_NAME=humble
  # shellcheck disable=SC1091
  . "${INSTALLER_DIR}/lib/common.sh"
  for l in os_check dependencies ros_install permissions network_install systemd_install health_check release; do
    # shellcheck disable=SC1090
    . "${INSTALLER_DIR}/lib/${l}.sh"
  done
  # shellcheck disable=SC1091
  . "${INSTALLER_DIR}/ci/slim_release.sh"
  set +e
  load_pin firmware
  local sha=2222222222222222222222222222222222222222 stage="${T}/slim_stage" art="${T}/slim_art" out rc
  local srel="${T}/slim_stage/opt/dyx3/releases/2222222222222222222222222222222222222222"

  # ---- test sources: removed; everything colcon/ament, launch, config, deployment, backend and docs use stays
  mkdir -p "${srel}/ros2_ws/install" "${srel}/bin" "${srel}/ros2_ws/src/pkg_a/test/fixtures" \
    "${srel}/ros2_ws/src/pkg_a/launch" "${srel}/ros2_ws/src/pkg_a/config" "${srel}/ros2_ws/src/pkg_a/src" \
    "${srel}/ros2_ws/src/pkg_b/tests" "${srel}/backend/tests" "${srel}/backend/src/dyx3_backend" \
    "${srel}/installer/tests" "${srel}/deployment/scripts" "${srel}/docs" "${srel}/ros2_ws/src/pkg_c"
  : >"${srel}/ros2_ws/install/setup.bash"
  printf '#!/bin/sh\n' >"${srel}/bin/dyx3-platform" && chmod +x "${srel}/bin/dyx3-platform"
  : >"${srel}/bin/dyx3-env.sh"
  for f in ros2_ws/src/pkg_a/package.xml ros2_ws/src/pkg_a/CMakeLists.txt ros2_ws/src/pkg_a/launch/a.launch.py \
    ros2_ws/src/pkg_a/config/a.yaml ros2_ws/src/pkg_a/src/a.cpp ros2_ws/src/pkg_b/package.xml \
    ros2_ws/src/pkg_c/package.xml backend/pyproject.toml backend/src/dyx3_backend/__init__.py \
    installer/tests/run_tests.sh deployment/scripts/start-control.sh docs/a.md; do
    echo keep >"${srel}/${f}"
  done
  echo vec >"${srel}/ros2_ws/src/pkg_a/test/fixtures/v.txt"
  echo t >"${srel}/ros2_ws/src/pkg_a/test/a_test.cpp"
  echo t >"${srel}/ros2_ws/src/pkg_b/tests/test_b.py"
  echo t >"${srel}/backend/tests/test_x.py"
  out="$(slim_release_tests "${srel}")"
  check "slim: test sources and fixtures are removed (ros2_ws packages and backend)" \
    '[ ! -e "${srel}/ros2_ws/src/pkg_a/test" ] && [ ! -e "${srel}/ros2_ws/src/pkg_b/tests" ] && [ ! -e "${srel}/backend/tests" ] && [ "$(printf "%s\n" "${out}" | grep -c .)" -eq 3 ]'
  check "slim: package.xml, CMakeLists, launch, config, sources, backend, installer, deployment and docs stay" \
    'for f in ros2_ws/src/pkg_a/package.xml ros2_ws/src/pkg_a/CMakeLists.txt ros2_ws/src/pkg_a/launch/a.launch.py ros2_ws/src/pkg_a/config/a.yaml ros2_ws/src/pkg_a/src/a.cpp ros2_ws/src/pkg_b/package.xml ros2_ws/src/pkg_c/package.xml backend/pyproject.toml backend/src/dyx3_backend/__init__.py installer/tests/run_tests.sh deployment/scripts/start-control.sh docs/a.md; do [ -f "${srel}/${f}" ] || exit 1; done'
  check "slim: a tree that is not a built release is refused" '! (slim_release_tests "${T}/slim_none" >/dev/null 2>&1)'

  # ---- venv: pip/setuptools/wheel go; whatever imported before still imports
  if python3 -m venv "${T}/slim_venv_probe" >/dev/null 2>&1 && [ -x "${T}/slim_venv_probe/bin/pip" ]; then
    rm -rf "${T}/slim_venv_probe"
    python3 -m venv "${srel}/venv" >/dev/null 2>&1
    local sp
    sp="$("${srel}/venv/bin/python" -c 'import sysconfig; print(sysconfig.get_paths()["purelib"])')"
    mkdir -p "${sp}/dyx3_backend/api"
    : >"${sp}/dyx3_backend/__init__.py"
    : >"${sp}/dyx3_backend/api/__init__.py"
    printf 'import json\n' >"${sp}/dyx3_backend/api/routes.py"
    printf 'import dyx3_backend.api.routes\nprint("noise on stdout")\n' >"${sp}/runtime_dep.py"
    (slim_release_venv "${srel}") >"${T}/slim_venv" 2>&1
    rc=$?
    check "slim: the venv loses pip (module, scripts) and keeps its interpreter" \
      '[ "${rc}" -eq 0 ] && ! "${srel}/venv/bin/python" -c "import pip" 2>/dev/null && [ -z "$(ls "${srel}/venv/bin" | grep -E "^(pip|easy_install|wheel)")" ] && [ -L "${srel}/venv/bin/python3" ] && grep -q "modules import as before" "${T}/slim_venv"'
    check "slim: the backend and its dependencies still import after the venv is slimmed" \
      '"${srel}/venv/bin/python" -I -c "import runtime_dep, dyx3_backend.api.routes" >/dev/null && [ -z "$(find "${srel}/venv" -name "distutils-precedence.pth" -o -name "pip-*.dist-info" -o -name "setuptools-*.dist-info")" ]'
    check "slim: slimming a venv twice is harmless" '(slim_release_venv "${srel}") >/dev/null 2>&1'
    # A runtime module that needs pip: removing pip would break it, so the slimming must refuse.
    python3 -m venv "${T}/slim_bad/venv" >/dev/null 2>&1
    sp="$("${T}/slim_bad/venv/bin/python" -c 'import sysconfig; print(sysconfig.get_paths()["purelib"])')"
    printf 'import pip\n' >"${sp}/needs_pip.py"
    (slim_release_venv "${T}/slim_bad") >"${T}/slim_bad.log" 2>&1
    rc=$?
    check "slim: removing a packaging tool that a runtime module imports is refused" '[ "${rc}" -ne 0 ] && grep -q "no longer importable.*needs_pip" "${T}/slim_bad.log"'
  else
    rm -rf "${T}/slim_venv_probe"
    ok "slim: venv tests skipped (python3 -m venv with pip unavailable here)"
  fi
  (slim_release_venv "${T}/slim_none") >"${T}/slim_novenv" 2>&1
  rc=$?
  check "slim: a release without a backend venv is left alone" '[ "${rc}" -eq 0 ] && grep -q "no backend venv" "${T}/slim_novenv"'

  # ---- strip: symbols move to .build-id/xx/yyyy.debug; the build-id stays; binaries still run
  # (No `readelf | grep -q` below: under pipefail an early grep exit can fail the pipeline with SIGPIPE.)
  local cc_ok=0
  if have cc && have readelf && have objcopy && have strip; then
    mkdir -p "${T}/slim_cc"
    printf 'int dyx3_slim_exported(int x) { return x + 1; }\nstatic int dyx3_slim_local(void) { return 41; }\nint dyx3_slim_lib(void) { return dyx3_slim_local(); }\n' >"${T}/slim_cc/lib.c"
    printf '#include <stdio.h>\nint dyx3_slim_lib(void);\nint main(void) { printf("answer %%d\\n", dyx3_slim_lib() + 1); return 0; }\n' >"${T}/slim_cc/main.c"
    if cc -shared -fPIC -Wl,--build-id -o "${T}/slim_cc/libdyx3_slim.so" "${T}/slim_cc/lib.c" 2>/dev/null &&
      cc -Wl,--build-id -o "${T}/slim_cc/node" "${T}/slim_cc/main.c" -L"${T}/slim_cc" -ldyx3_slim 2>/dev/null &&
      [ -n "$(_elf_kind "${T}/slim_cc/node")" ]; then
      cc_ok=1
    fi
  fi
  if [ "${cc_ok}" = 1 ]; then
    local inst="${srel}/ros2_ws/install/pkg_a" dbg="${T}/slim_dbg" id_node id_lib sum_py
    mkdir -p "${inst}/lib/pkg_a" "${inst}/lib/python3.10/site-packages"
    cp "${T}/slim_cc/node" "${inst}/lib/pkg_a/node"
    cp "${T}/slim_cc/libdyx3_slim.so" "${inst}/lib/libdyx3_slim.so"
    ln -s libdyx3_slim.so "${inst}/lib/libdyx3_slim.so.1"
    printf '#!/usr/bin/env python3\nprint(1)\n' >"${inst}/lib/pkg_a/script_node" && chmod +x "${inst}/lib/pkg_a/script_node"
    sum_py="$(sha256sum "${inst}/lib/pkg_a/script_node")"
    id_node="$(_build_id "${inst}/lib/pkg_a/node")"
    id_lib="$(_build_id "${inst}/lib/libdyx3_slim.so")"
    (strip_release_binaries "${srel}" "${dbg}") >"${T}/slim_strip" 2>&1
    rc=$?
    check "slim: ELF files in ros2_ws/install are stripped, their build-ids unchanged" \
      '[ "${rc}" -eq 0 ] && [ -n "${id_node}" ] && [ -z "$(readelf -S "${inst}/lib/pkg_a/node" | grep "\.symtab")" ] && [ -z "$(readelf -S "${inst}/lib/libdyx3_slim.so" | grep "\.symtab")" ] && [ "$(_build_id "${inst}/lib/pkg_a/node")" = "${id_node}" ] && [ "$(_build_id "${inst}/lib/libdyx3_slim.so")" = "${id_lib}" ]'
    check "slim: the removed symbols are kept under .build-id and indexed in BUILD_IDS" \
      'readelf -s "${dbg}/.build-id/${id_lib:0:2}/${id_lib:2}.debug" | grep dyx3_slim_local >/dev/null && [ -f "${dbg}/.build-id/${id_node:0:2}/${id_node:2}.debug" ] && grep -qx "${id_node} ros2_ws/install/pkg_a/lib/pkg_a/node" "${dbg}/BUILD_IDS" && [ "$(grep -c . "${dbg}/BUILD_IDS")" -eq 2 ]'
    check "slim: a stripped executable still runs and its library keeps its exported symbols" \
      '[ "$(LD_LIBRARY_PATH="${inst}/lib" "${inst}/lib/pkg_a/node")" = "answer 42" ] && readelf --dyn-syms "${inst}/lib/libdyx3_slim.so" | grep dyx3_slim_exported >/dev/null'
    check "slim: scripts and symlinks are not touched by the stripping" \
      '[ "$(sha256sum "${inst}/lib/pkg_a/script_node")" = "${sum_py}" ] && [ -L "${inst}/lib/libdyx3_slim.so.1" ]'
    cc -Wl,--build-id=none -o "${inst}/lib/pkg_a/noid" "${T}/slim_cc/main.c" -L"${T}/slim_cc" -ldyx3_slim 2>/dev/null
    (strip_release_binaries "${srel}" "${T}/slim_dbg2") >"${T}/slim_noid" 2>&1
    rc=$?
    check "slim: an ELF without a build-id is refused (its symbols could not be found again)" '[ "${rc}" -ne 0 ] && grep -q "no build-id" "${T}/slim_noid"'
    rm -f "${inst}/lib/pkg_a/noid"
  else
    ok "slim: strip tests skipped (no cc/readelf/objcopy/strip for ELF here)"
  fi

  # ---- the slimmed release passes static verification and installs through the prebuilt path
  if ! tar --zstd -cf /dev/null --files-from /dev/null 2>/dev/null; then
    ok "slim: prebuilt install of a slimmed release skipped (tar has no zstd here)"
    return 0
  fi
  local pmst="${stage}/opt/dyx3/px4_msgs/${FIRMWARE_SHA}"
  mkdir -p "${pmst}/install" "${art}"
  : >"${pmst}/install/setup.bash" && : >"${pmst}/.complete" && echo abc >"${pmst}/px4_msgs.sha256"
  # The test venv above links to this host's python; a CI venv links to /usr/bin/python3 (the only absolute link
  # install_prebuilt accepts).
  rm -rf "${srel}/venv" && mkdir -p "${srel}/venv/bin" && ln -s /usr/bin/python3 "${srel}/venv/bin/python3"
  tar -C "${stage}" --zstd -cf "${art}/release-${sha}.tar.zst" "opt/dyx3/releases/${sha}"
  tar -C "${stage}" --zstd -cf "${art}/px4_msgs-${FIRMWARE_SHA}.tar.zst" "opt/dyx3/px4_msgs/${FIRMWARE_SHA}"
  printf 'ARTIFACT_STACK_SHA=%s\nARTIFACT_FIRMWARE_SHA=%s\nARTIFACT_ROS_DISTRO=humble\nARTIFACT_CI_RUN=https://ci/run/2\nARTIFACT_DEBUG_SYMBOLS=rover-debug-%s.tar.zst\nARTIFACT_DEBUG_SYMBOLS_SHA256=%s\n' \
    "${sha}" "${FIRMWARE_SHA}" "${sha}" "$(printf x | sha256sum | cut -d' ' -f1)" >"${art}/artifacts.env"
  (cd "${art}" && sha256sum artifacts.env "release-${sha}.tar.zst" "px4_msgs-${FIRMWARE_SHA}.tar.zst" >SHA256SUMS)
  local rel="${DYX3_RELEASES}/${sha}"
  mkdir -p "${DYX3_VAR_LIB}/state" "${DYX3_RELEASES}"
  (DYX3_ARTIFACT_DIR="${art}" install_prebuilt "${sha}") >"${T}/slim_pb" 2>&1
  rc=$?
  check "slim: a slimmed release installs through the prebuilt path" '[ "${rc}" -eq 0 ] && [ -f "${rel}/.prebuilt" ] && [ -f "${rel}/ros2_ws/src/pkg_a/package.xml" ] && [ ! -e "${rel}/ros2_ws/src/pkg_a/test" ]'
  check "slim: the installed slimmed release passes the installer's static verification" '(health_release_only "${rel}" 0 >/dev/null 2>&1)'
  check "slim: build_release does not rebuild a slimmed prebuilt release" '(build_release "${sha}" 2>&1 | grep -q "nothing to build")'
}

# ---------------------------------------------------------------- start-control.sh: the bounded wait for the XRCE agent
# Fake dyx3-env.sh (dyx3_env_load succeeds), fake ros2 (records that the graph was started), fake ss (a UDP listener on the port
# in ${T}/sw_listen, one per line).
start_control_wait() {
  local d="${T}/sw" out rc
  mkdir -p "${d}/scripts" "${d}/bin"
  cp "${REPO}/deployment/scripts/start-control.sh" "${d}/scripts/"
  printf 'dyx3_env_load() { return 0; }\n' >"${d}/scripts/dyx3-env.sh"
  printf '#!/usr/bin/env bash\necho "ros2 $*" >>"%s/sw_ros2"\n' "${d}" >"${d}/bin/ros2"
  cat >"${d}/bin/ss" <<F
#!/usr/bin/env bash
echo "ss \$*" >>"${d}/sw_ss"
p="\${*: -1}"
p="\${p##*:}"
grep -qx "\${p}" "${d}/sw_listen" 2>/dev/null && echo "UNCONN 0 0 0.0.0.0:\${p} 0.0.0.0:*"
exit 0
F
  chmod +x "${d}/bin/ros2" "${d}/bin/ss"
  : >"${d}/sw_ros2"
  : >"${d}/sw_listen"
  printf '8888\n' >"${d}/sw_listen"
  out="$(PATH="${d}/bin:${PATH}" DYX3_PLATFORM_ENV="${d}/none.env" "${d}/scripts/start-control.sh" 2>&1)"
  rc=$?
  check "start-control: an agent already listening on the default udp/8888 starts the graph at once" '[ "${rc}" -eq 0 ] && printf "%s" "${out}" | grep -q "XRCE agent listening on udp/8888 after 0.0 s" && grep -q "^ros2 launch dyx3_bringup control_graph.launch.py" "${d}/sw_ros2"'
  # The port comes from platform.env (read, not executed).
  printf 'DYX3_XRCE_PORT=9999\n' >"${d}/platform.env"
  : >"${d}/sw_listen"
  : >"${d}/sw_ros2"
  ( sleep 1; printf '9999\n' >"${d}/sw_listen" ) &
  out="$(PATH="${d}/bin:${PATH}" DYX3_PLATFORM_ENV="${d}/platform.env" "${d}/scripts/start-control.sh" 2>&1)"
  rc=$?
  wait
  check "start-control: it waits for the agent on the port from platform.env, then starts the graph" '[ "${rc}" -eq 0 ] && printf "%s" "${out}" | grep -q "XRCE agent listening on udp/9999 after [1-9]" && grep -q "sport = :9999" "${d}/sw_ss" && grep -q "^ros2 launch" "${d}/sw_ros2"'
  # No agent at all: bounded wait, one WARN line, the graph still starts.
  : >"${d}/sw_listen"
  : >"${d}/sw_ros2"
  local t0 t1
  t0="$(date +%s)"
  out="$(PATH="${d}/bin:${PATH}" DYX3_PLATFORM_ENV="${d}/platform.env" DYX3_AGENT_WAIT_S=2 "${d}/scripts/start-control.sh" 2>&1)"
  rc=$?
  t1="$(date +%s)"
  check "start-control: no agent: the wait is bounded, it logs one WARN line and starts the graph anyway" '[ "${rc}" -eq 0 ] && [ $((t1 - t0)) -ge 2 ] && [ $((t1 - t0)) -le 5 ] && [ "$(printf "%s\n" "${out}" | grep -c "nothing listens on udp/9999 after 2 s; starting the control chain anyway")" -eq 1 ] && grep -q "^ros2 launch" "${d}/sw_ros2"'
  # No ss on PATH (a PATH of symlinks to just what the script needs): no wait, one line, the graph starts.
  mkdir -p "${d}/nossbin"
  for tool in bash env dirname sed tail sleep; do ln -sf "$(command -v "${tool}")" "${d}/nossbin/${tool}"; done
  ln -sf "${d}/bin/ros2" "${d}/nossbin/ros2"
  : >"${d}/sw_ros2"
  out="$(PATH="${d}/nossbin" DYX3_PLATFORM_ENV="${d}/platform.env" "${d}/scripts/start-control.sh" 2>&1)"
  rc=$?
  check "start-control: without ss it does not wait and still starts the graph" '[ "${rc}" -eq 0 ] && printf "%s" "${out}" | grep -q "ss not available" && grep -q "^ros2 launch" "${d}/sw_ros2"'

  # start-services.sh: the services launch at once; no XRCE-agent wait (no ss call): none of its nodes talks to the agent.
  cp "${REPO}/deployment/scripts/start-services.sh" "${d}/scripts/"
  : >"${d}/sw_ros2"
  : >"${d}/sw_ss"
  : >"${d}/sw_listen"
  out="$(env -u DYX3_CONFIG_DIR PATH="${d}/bin:${PATH}" DYX3_PLATFORM_ENV="${d}/platform.env" "${d}/scripts/start-services.sh" 2>&1)"
  rc=$?
  check "start-services: starts services_graph.launch.py at once and never waits for the XRCE agent" '[ "${rc}" -eq 0 ] && grep -qx "ros2 launch dyx3_bringup services_graph.launch.py config_dir:=/etc/dyx3" "${d}/sw_ros2" && [ ! -s "${d}/sw_ss" ]'
  printf 'dyx3_env_load() { return 1; }\n' >"${d}/scripts/dyx3-env.sh"
  : >"${d}/sw_ros2"
  PATH="${d}/bin:${PATH}" "${d}/scripts/start-services.sh" >/dev/null 2>&1
  rc=$?
  check "start-services: a failed environment load exits non-zero and launches nothing" '[ "${rc}" -ne 0 ] && [ ! -s "${d}/sw_ros2" ]'
}

# ---------------------------------------------------------------- P2 part 3: dyx3-ros split into dyx3-control + dyx3-services
# The upgrade path on a staged root with a recorded systemctl: an existing rover has dyx3-ros.service installed and enabled; the new
# release installs and enables dyx3-control and dyx3-services, restarts them in manifest order, and the stale unit is stopped,
# disabled and removed, idempotently. A rollback to a pre-split release does the reverse. Health treats dyx3-services as the
# gateway's unit. What this does NOT prove: real systemd ordering, Wants=/Requires= propagation, or the rover staying armed in
# OFFBOARD while dyx3-services restarts (docs/bench/fault_injection.md, on the rover).
unit_split() {
  export INSTALLER_DIR="${REPO}/installer" DYX3_ROOT="${T}/us"
  # shellcheck disable=SC1091
  . "${INSTALLER_DIR}/lib/common.sh"
  for l in systemd_install health_check; do
    # shellcheck disable=SC1090
    . "${INSTALLER_DIR}/lib/${l}.sh"
  done
  set +e
  local rel="${T}/us_rel" old="${T}/us_old" sd="${SYSTEMD_DIR}" calls="${T}/us_calls" out="${T}/us_out"
  mkdir -p "${rel}/installer" "${sd}/multi-user.target.wants"
  cp -r "${REPO}/deployment" "${rel}/"
  cp -r "${REPO}/installer/manifests" "${rel}/installer/"
  # The rover as a pre-split release left it: dyx3-ros installed and enabled next to the units that stay.
  printf '[Service]\nExecStart=/opt/dyx3/current/bin/dyx3-ros\n' >"${sd}/dyx3-ros.service"
  ln -sfn ../dyx3-ros.service "${sd}/multi-user.target.wants/dyx3-ros.service"
  printf '[Service]\n' >"${sd}/dyx3-platform.service"
  ln -sfn ../dyx3-platform.service "${sd}/multi-user.target.wants/dyx3-platform.service"
  us_systemd() { (systemd_available() { return 0; }; systemctl() { printf '%s\n' "$*" >>"${calls}"; }; "$@"); }
  : >"${calls}"
  us_systemd install_units "${rel}" >"${out}" 2>&1
  check "split upgrade: the stale dyx3-ros unit is stopped, disabled and reset" 'grep -qx "stop dyx3-ros.service" "${calls}" && grep -qx "disable dyx3-ros.service" "${calls}" && grep -qx "reset-failed dyx3-ros.service" "${calls}" && grep -q "removing dyx3-ros.service" "${out}"'
  check "split upgrade: the stale unit is stopped before the new units are enabled" '[ "$(grep -n -x "stop dyx3-ros.service" "${calls}" | cut -d: -f1)" -lt "$(grep -n -x "enable dyx3-control.service" "${calls}" | cut -d: -f1)" ]'
  check "split upgrade: the dyx3-ros unit file and its enablement link are gone" '[ ! -e "${sd}/dyx3-ros.service" ] && [ ! -L "${sd}/multi-user.target.wants/dyx3-ros.service" ]'
  check "split upgrade: dyx3-control and dyx3-services are installed as shipped" 'cmp -s "${sd}/dyx3-control.service" "${REPO}/deployment/systemd/dyx3-control.service" && cmp -s "${sd}/dyx3-services.service" "${REPO}/deployment/systemd/dyx3-services.service"'
  check "split upgrade: both new units are enabled, dyx3-ros is never enabled" 'grep -qx "enable dyx3-control.service" "${calls}" && grep -qx "enable dyx3-services.service" "${calls}" && ! grep -qx "enable dyx3-ros.service" "${calls}"'
  check "split upgrade: units that stay are kept and re-installed, not removed" '! grep -q "removing dyx3-platform" "${out}" && cmp -s "${sd}/dyx3-platform.service" "${REPO}/deployment/systemd/dyx3-platform.service" && [ -L "${sd}/multi-user.target.wants/dyx3-platform.service" ]'
  : >"${calls}"
  us_systemd restart_enabled_services "${rel}" >/dev/null 2>&1
  check "split upgrade: the enabled units restart in manifest order, control before services, backend last" '[ "$(sed -n "s/^restart //p" "${calls}" | tr "\n" " ")" = "dyx3-usb-serial-check.service dyx3-platform.service dyx3-control.service dyx3-services.service dyx3-rtk.service dyx3-spray-watchdog.service dyx3-recorder.service dyx3-backend.service " ]'
  : >"${calls}"
  us_systemd stop_enabled_services "${rel}" >/dev/null 2>&1
  check "a failed first install stops the units in reverse order: services before control" '[ "$(sed -n "s/^stop //p" "${calls}" | tr "\n" " ")" = "dyx3-backend.service dyx3-recorder.service dyx3-spray-watchdog.service dyx3-rtk.service dyx3-services.service dyx3-control.service dyx3-platform.service dyx3-usb-serial-check.service " ]'
  # Idempotent: a second run removes nothing and ends in the same state.
  : >"${calls}"
  us_systemd install_units "${rel}" >"${out}" 2>&1
  check "split upgrade: a second run removes nothing and touches no stale unit" '! grep -q "removing" "${out}" && ! grep -q "dyx3-ros" "${calls}" && [ -f "${sd}/dyx3-control.service" ] && [ -f "${sd}/dyx3-services.service" ]'
  # A run interrupted between deleting the unit file and its link: the dangling link alone is still swept.
  ln -sfn ../dyx3-ros.service "${sd}/multi-user.target.wants/dyx3-ros.service"
  : >"${calls}"
  us_systemd install_units "${rel}" >"${out}" 2>&1
  check "split upgrade: a dangling dyx3-ros enablement link left by an interrupted run is removed" '[ ! -L "${sd}/multi-user.target.wants/dyx3-ros.service" ] && grep -qx "disable dyx3-ros.service" "${calls}"'
  # Without systemd (staged root) the files are still converged.
  printf '[Service]\n' >"${sd}/dyx3-ros.service"
  install_units "${rel}" >"${out}" 2>&1
  check "split upgrade without systemd: the stale unit file is still removed" '[ ! -e "${sd}/dyx3-ros.service" ] && [ -f "${sd}/dyx3-control.service" ]'

  # A release that ships a unit file its manifest no longer lists: not installed, removed when present.
  cp -r "${rel}" "${T}/us_unlisted"
  printf '[Service]\n' >"${T}/us_unlisted/deployment/systemd/dyx3-ros.service"
  printf '[Service]\n' >"${sd}/dyx3-ros.service"
  install_units "${T}/us_unlisted" >"${out}" 2>&1
  check "a unit file the manifest no longer lists is neither installed nor kept" '[ ! -e "${sd}/dyx3-ros.service" ] && grep -q "removing dyx3-ros.service" "${out}"'
  # A manifest without [services] never turns into "remove every unit".
  cp -r "${rel}" "${T}/us_nosvc"
  awk '/^\[services\]/ { skip = 1; next } /^\[/ { skip = 0 } !skip' "${rel}/installer/manifests/production.manifest" >"${T}/us_nosvc/installer/manifests/production.manifest"
  install_units "${T}/us_nosvc" >"${out}" 2>&1
  check "a manifest without [services] installs every shipped unit and removes none of them" '! grep -q "removing" "${out}" && grep -q "no \[services\]" "${out}" && [ -f "${sd}/dyx3-control.service" ] && [ -f "${sd}/dyx3-services.service" ] && [ -f "${sd}/dyx3-backend.service" ]'

  # Rollback to a pre-split release (dyx3-ros shipped and enabled, no control/services units): the reverse.
  cp -r "${rel}" "${old}"
  rm -f "${old}/deployment/systemd/dyx3-control.service" "${old}/deployment/systemd/dyx3-services.service"
  printf '[Service]\nExecStart=/opt/dyx3/current/bin/dyx3-ros\n' >"${old}/deployment/systemd/dyx3-ros.service"
  sed -i.tmp -e 's/^dyx3-control$/dyx3-ros/' -e '/^dyx3-services$/d' "${old}/installer/manifests/production.manifest" && rm -f "${old}/installer/manifests/production.manifest.tmp"
  : >"${calls}"
  us_systemd install_units "${old}" >"${out}" 2>&1
  check "rollback across the split: dyx3-control and dyx3-services are stopped and removed, dyx3-ros comes back enabled" 'grep -qx "stop dyx3-control.service" "${calls}" && grep -qx "stop dyx3-services.service" "${calls}" && [ ! -e "${sd}/dyx3-control.service" ] && [ ! -e "${sd}/dyx3-services.service" ] && [ -f "${sd}/dyx3-ros.service" ] && grep -qx "enable dyx3-ros.service" "${calls}"'
  us_systemd install_units "${rel}" >/dev/null 2>&1

  # ---- health: dyx3-services is the gateway's unit (a pre-split dyx3-ros counts as both units)
  local m="${T}/us_health"
  us_manifest() { mkdir -p "${m}/installer/manifests"; printf '[enabled_services]\n%s\n' "$1" | tr ' ' '\n' >"${m}/installer/manifests/production.manifest"; }
  rm -f "${DYX3_GATEWAY_SOCK}"
  us_manifest "dyx3-control dyx3-services"
  (DYX3_HEALTH_SETTLE_S=1 health_extras "${m}") >"${out}" 2>&1
  check "health: with dyx3-services enabled the gateway must answer, and the FAIL names dyx3-services" 'grep -q "^FAIL  gateway does not answer on .* (dyx3-services / system_gateway down?)" "${out}"'
  us_manifest "dyx3-control"
  (DYX3_HEALTH_SETTLE_S=1 health_extras "${m}") >"${out}" 2>&1
  check "health: with dyx3-services not enabled the gateway is not checked (dyx3-control alone is not the gateway's unit)" '! grep -q "gateway" "${out}"'
  us_manifest "dyx3-ros"
  (DYX3_HEALTH_SETTLE_S=1 health_extras "${m}") >"${out}" 2>&1
  check "health: a pre-split release (dyx3-ros) keeps its gateway check, naming dyx3-ros" 'grep -q "^FAIL  gateway does not answer on .* (dyx3-ros / system_gateway down?)" "${out}"'
  us_manifest "dyx3-services"
  (health_dds "${m}") >"${out}" 2>&1
  check "health: the px4_link sample is gated on dyx3-control, not dyx3-services" '[ ! -s "${out}" ]'
}

sup
recorder_launcher
(start_control_wait)
(unit_split)
(libs)
(lifecycle)
(handoff)
(locking)
(prebuilt)
(health_ros_env)
(slim_release_block)
pass="$(grep -c '^ok' "${RESULTS}")"
fail="$(grep -c '^bad' "${RESULTS}")"
[ -n "${SHOW_LOGS:-}" ] && tail -n +1 "${T}"/up_* 2>/dev/null
printf '\n%s passed, %s failed\n' "${pass}" "${fail}"
[ "${fail}" -eq 0 ]
