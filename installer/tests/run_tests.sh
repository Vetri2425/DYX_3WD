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
trap 'rm -rf "${T}"' EXIT
RESULTS="${T}/results"
: >"${RESULTS}"
ok() { printf 'ok   %s\n' "$1"; echo ok >>"${RESULTS}"; }
bad() { printf 'FAIL %s\n' "$1"; echo bad >>"${RESULTS}"; }
check() { if eval "$2"; then ok "$1"; else bad "$1"; fi; }

# ---------------------------------------------------------------- supervisor
sup() {
  local d="${T}/sup"
  mkdir -p "${d}/bin"
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
    DYX3_RESTART_DELAY_S=0.1 DYX3_MAVROUTER_CONF="${d}/router.conf" \
    "${REPO}/deployment/scripts/start-platform.sh" >"${d}/out" 2>&1 &
  local pid=$!
  sleep 2
  local starts
  starts="$(wc -l <"${d}/count")"
  check "supervisor restarts a crashing agent (starts=${starts})" '[ "${starts}" -ge 3 ]'
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
  # Missing agent binary is fatal (systemd will surface it) — never silently "healthy".
  env PATH="/usr/bin:/bin" DYX3_PLATFORM_ENV=/nonexistent DYX3_AGENT_BIN=definitely-missing \
    "${REPO}/deployment/scripts/start-platform.sh" >/dev/null 2>&1
  rc=$?
  check "missing agent binary exits non-zero" '[ "${rc}" -ne 0 ]'
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
  # The six services verified on the 3WD rover 2026-10-08; a new one joins only after a rover run.
  check "manifest: USB provisioning runs before the six rover services" '[ "$(printf "%s" "${svc}" | tr "\n" " ")" = "dyx3-usb-serial-check dyx3-platform dyx3-ros dyx3-rtk dyx3-spray-watchdog dyx3-recorder dyx3-backend" ]'
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
  check "the spray watchdog unit is not tied to dyx3-ros" '! grep -E "^(Requires|BindsTo|PartOf)=.*dyx3-ros" "${REPO}/deployment/systemd/dyx3-spray-watchdog.service"'
  # Only rpp and motion_guard get FIFO 80 / CPU 4, via the launch prefix running as User=dyx3.
  local ru="${REPO}/deployment/systemd/dyx3-ros.service"
  check "dyx3-ros sets no tree-wide RT policy or affinity" '! grep -Eq "^(CPUSchedulingPolicy|CPUSchedulingPriority|CPUAffinity)=" "${ru}"'
  check "dyx3-ros lets the unprivileged chrt prefix set FIFO 80" '[ "$(sed -n "s/^LimitRTPRIO=//p" "${ru}")" -ge 80 ] && grep -q "chrt -f 80" "${REPO}/ros2_ws/src/dyx3_bringup/launch/control_graph.launch.py"'
  check "dyx3-ros lets mlockall succeed and bounds a stop hang" 'grep -qx "LimitMEMLOCK=infinity" "${ru}" && grep -qx "TimeoutStopSec=15" "${ru}" && ! grep -q "^RestrictRealtime=yes" "${ru}"'
  check "the RTK unit creates its own 0700 state directory" 'grep -qx "StateDirectory=dyx3/rtk" "${REPO}/deployment/systemd/dyx3-rtk.service" && grep -qx "StateDirectoryMode=0700" "${REPO}/deployment/systemd/dyx3-rtk.service"'
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

  printf 'ID=ubuntu\nVERSION_ID="22.04"\n' >"${T}/os22"
  printf 'ID=ubuntu\nVERSION_ID="24.04"\n' >"${T}/os24"
  printf 'ID=debian\nVERSION_ID="12"\n' >"${T}/osdeb"
  check "os_check accepts ubuntu 22.04" '(DYX3_OS_RELEASE="${T}/os22" DYX3_ARCH=aarch64 os_check) 2>/dev/null'
  check "os_check rejects ubuntu 24.04" '! (DYX3_OS_RELEASE="${T}/os24" DYX3_ARCH=aarch64 os_check) 2>/dev/null'
  check "os_check rejects debian" '! (DYX3_OS_RELEASE="${T}/osdeb" DYX3_ARCH=aarch64 os_check) 2>/dev/null'

  check "pins: XRCE agent v2.4.3 pinned to a commit" '(load_pin microxrce_agent; [ "${XRCE_TAG}" = v2.4.3 ] && [ ${#XRCE_COMMIT} -eq 40 ])'
  check "pins: firmware SHA is the flashed 27a7ac92" '(load_pin firmware; [ "${FIRMWARE_SHA}" = 27a7ac92845317b0276776242c504215809b2a0f ])'

  # dry-run of the whole install flow must complete and mention every required step
  local out
  out="$(DYX3_ALLOW_ANY_OS=1 DYX3_DRY_RUN=1 "${REPO}/installer/install.sh" --production --dry-run --ref HEAD 2>&1)"
  rc=$?
  check "install --dry-run completes" '[ "${rc}" -eq 0 ]'
  for s in "useradd" "apt-get install" "MicroXRCEAgent" "mavlink-router" "nmcli connection add" "colcon build" "git archive"; do
    check "install dry-run mentions: ${s}" 'printf "%s" "${out}" | grep -q -- "${s}"'
  done
  check "install without --production is refused" '! "${REPO}/installer/install.sh" >/dev/null 2>&1'
  # FCU profile: production is static with no default route; the bench (FCU_KEEP_DHCP=1) keeps the
  # site router's DHCP route, its only WAN, or the release fetch fails (2026-10-08 on the rover).
  local net_prod net_bench
  net_prod="$(DYX3_DRY_RUN=1 install_fcu_network 2>&1)"
  net_bench="$(DYX3_DRY_RUN=1 FCU_KEEP_DHCP=1 install_fcu_network 2>&1)"
  check "fcu profile (production): manual, never-default yes" 'printf "%s" "${net_prod}" | grep -q "ipv4.method manual" && printf "%s" "${net_prod}" | grep -q "ipv4.never-default yes"'
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
  check "hotspot isolation hook blocks Wi-Fi to FCU both ways" 'grep -q -- "-i \"wlan0\" -o \"${FCU_IFACE}\" -j DROP" "${DYX3_ROOT}/etc/NetworkManager/dispatcher.d/pre-up.d/90-dyx3-hotspot-isolation" && grep -q -- "-i \"${FCU_IFACE}\" -o \"wlan0\" -j DROP" "${DYX3_ROOT}/etc/NetworkManager/dispatcher.d/pre-up.d/90-dyx3-hotspot-isolation"'
  check "hotspot profile pins safe 5 GHz and WPA2 without power save" 'grep -qx "band=a" "${hotspot_profile}" && grep -qx "channel=36" "${hotspot_profile}" && grep -qx "channel-width=20" "${hotspot_profile}" && grep -qx "powersave=2" "${hotspot_profile}" && grep -qx "proto=rsn" "${hotspot_profile}"'
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
  printf 'DYX3_HOTSPOT_SSID=TestRover\nDYX3_HOTSPOT_PSK=DummyBenchPass123\nDYX3_WIFI_BAND=a\nDYX3_WIFI_CHANNEL=52\n' >"${DYX3_ETC}/hotspot.env"
  install_hotspot_network >>"${T}/hotspot_log" 2>&1
  check "5 GHz DFS channel is refused" '[ ! -e "${hotspot_profile}" ] && grep -q "DFS or invalid channel refused" "${T}/hotspot_log"'
  nmcli() { if [ "${1:-}" = "-t" ]; then printf '%s\n' "wlan0:wifi"; elif [ "${1:-}" = "--version" ]; then echo 'nmcli tool, version 1.36.6'; else printf '%s\n' "$*" >>"${T}/nmcli_argv"; fi; }
  printf 'DYX3_HOTSPOT_SSID=TestRover\nDYX3_HOTSPOT_PSK=DummyBenchPass123\nDYX3_WIFI_WIDTH=40\n' >"${DYX3_ETC}/hotspot.env"
  install_hotspot_network >>"${T}/hotspot_log" 2>&1
  check "40 MHz is refused by old NetworkManager" '[ ! -e "${hotspot_profile}" ] && grep -q "NetworkManager 1.50+ is required" "${T}/hotspot_log"'
  check "no hotspot credential appears in argv or installer logs" '! grep -q "DummyBenchPass123" "${T}/hotspot_log" "${T}/nmcli_argv" 2>/dev/null'
  install_config_templates "${DYX3_CURRENT}" >/dev/null 2>&1
  check "existing hotspot.env is never overwritten" 'grep -q "DummyBenchPass123" "${DYX3_ETC}/hotspot.env"'
  check "versions.json written for the recorder" 'grep -q "\"stack_sha\": \"${A}\"" "${DYX3_ETC}/versions.json" && grep -q firmware_expected_sha "${DYX3_ETC}/versions.json"'
  check "systemd units copied" '[ -f "${DYX3_ROOT}/etc/systemd/system/dyx3-platform.service" ]'

  # This ledger belongs to the PX4 correlation epoch, not a software release.
  printf 'v2 12345\n' >"${DYX3_VAR_LIB}/state/px4_link_spray_ack_next"
  (create_directories >/dev/null 2>&1)
  check "reinstall preserves spray correlation ledger" '[ "$(cat "${DYX3_VAR_LIB}/state/px4_link_spray_ack_next")" = "v2 12345" ]'

  echo "EDITED=1" >>"${DYX3_ETC}/platform.env"
  (upgrade_to "${B}") >"${T}/up_b" 2>&1
  rc=$?
  check "upgrade to B (rc=0)" '[ "${rc}" -eq 0 ]'
  check "current -> B, previous recorded as A" '[ "$(basename "$(readlink -f "${DYX3_CURRENT}")")" = "${B}" ] && [ "$(cat "${DYX3_VAR_LIB}/state/previous_release")" = "${A}" ]'
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
  (FAKE_NO_AGENT="${T}/no_agent" upgrade_to "${D}") >"${T}/up_d" 2>&1
  rc=$?
  check "unhealthy upgrade fails (rc!=0)" '[ "${rc}" -ne 0 ]'
  check "unhealthy upgrade reverted to B" '[ "$(basename "$(readlink -f "${DYX3_CURRENT}")")" = "${B}" ]'
  check "unhealthy upgrade is ineligible and records its failure" '[ ! -f "${DYX3_RELEASES}/${D}/.complete" ] && [ -f "${DYX3_RELEASES}/${D}/.failed" ]'
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

  (upgrade_to "${D}") >"${T}/up_d2" 2>&1
  rc=$?
  check "healthy retry of D succeeds" '[ "${rc}" -eq 0 ] && [ "$(basename "$(readlink -f "${DYX3_CURRENT}")")" = "${D}" ]'

  (DYX3_KEEP_RELEASES=1 prune_releases) >/dev/null 2>&1
  rc=$?
  check "prune keeps current and previous only" '[ -d "${DYX3_RELEASES}/${D}" ] && [ -d "${DYX3_RELEASES}/${B}" ] && [ ! -d "${DYX3_RELEASES}/${A}" ]'

  (upgrade_to "${D}") >"${T}/up_noop" 2>&1
  rc=$?
  check "re-running on the current SHA is a no-op" 'grep -q "nothing to do" "${T}/up_noop"'
  (upgrade_to nonexistent-ref) >"${T}/up_bad" 2>&1
  rc=$?
  check "unknown ref is refused" '[ "${rc}" -ne 0 ]'

  # ---- dyx3-version / dyx3-rollback
  (print_version) >"${T}/ver" 2>&1
  check "dyx3-version prints the stack SHA, the firmware pin and the message-set hash" 'grep -q "^stack_sha=${D}$" "${T}/ver" && grep -q "^firmware_expected_sha=27a7ac92" "${T}/ver" && grep -q "^px4_msgs_msg_set_sha256=abc123def456$" "${T}/ver"'
  check "dyx3-version admits the running firmware identity is unreadable" 'grep -q "^firmware_running=unavailable" "${T}/ver"'

  (rollback_release) >"${T}/rb1" 2>&1
  rc=$?
  check "rollback returns to B (rc=0)" '[ "${rc}" -eq 0 ] && [ "$(basename "$(readlink -f "${DYX3_CURRENT}")")" = "${B}" ]'
  check "rollback preserves RTK runtime config" 'grep -q "persist RTK config" "${DYX3_VAR_LIB}/rtk/config.json"'
  check "rollback records the release it left as previous" '[ "$(cat "${DYX3_VAR_LIB}/state/previous_release")" = "${D}" ]'
  check "versions.json describes the rolled-back release" 'grep -q "\"stack_sha\": \"${B}\"" "${DYX3_ETC}/versions.json"'
  (rollback_release) >"${T}/rb2" 2>&1
  check "a second rollback undoes the first" '[ "$(basename "$(readlink -f "${DYX3_CURRENT}")")" = "${D}" ]'
  (FAKE_NO_AGENT="${T}/no_agent" rollback_release) >"${T}/rb3" 2>&1
  rc=$?
  check "an unhealthy rollback fails (rc!=0)" '[ "${rc}" -ne 0 ]'
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
  check "prebuilt: a digest mismatch is refused and extracts nothing" '[ "${rc}" -ne 0 ] && grep -q "sha256 mismatch" "${T}/pb_bad" && [ ! -e "${rel}" ] && [ ! -e "${pm}" ]'

  # artifact for another firmware pin: refused
  cp -r "${art}" "${T}/art_fw"
  sed -i.bak "s/^ARTIFACT_FIRMWARE_SHA=.*/ARTIFACT_FIRMWARE_SHA=0000000000/" "${T}/art_fw/artifacts.env"
  (cd "${T}/art_fw" && sha256sum artifacts.env "release-${sha}.tar.zst" "px4_msgs-${FIRMWARE_SHA}.tar.zst" >SHA256SUMS)
  (DYX3_ARTIFACT_DIR="${T}/art_fw" install_prebuilt "${sha}") >"${T}/pb_fw" 2>&1
  rc=$?
  check "prebuilt: an artifact for another firmware pin is refused" '[ "${rc}" -ne 0 ] && grep -q "firmware pin" "${T}/pb_fw" && [ ! -e "${rel}" ]'

  # an archive member outside its prefix: refused
  mkdir -p "${T}/evil/etc" && : >"${T}/evil/etc/passwd-dyx3-test"
  cp -r "${art}" "${T}/art_evil"
  tar -C "${T}/evil" --zstd -cf "${T}/art_evil/release-${sha}.tar.zst" etc
  (cd "${T}/art_evil" && sha256sum artifacts.env "release-${sha}.tar.zst" "px4_msgs-${FIRMWARE_SHA}.tar.zst" >SHA256SUMS)
  (DYX3_ARTIFACT_DIR="${T}/art_evil" install_prebuilt "${sha}") >"${T}/pb_evil" 2>&1
  rc=$?
  check "prebuilt: an archive member outside its prefix is refused" '[ "${rc}" -ne 0 ] && grep -q "outside" "${T}/pb_evil" && [ ! -e "${DYX3_ROOT}/etc/passwd-dyx3-test" ]'

  # missing artifacts: auto falls back (rc 1), nothing left behind
  mkdir -p "${T}/art_empty"
  (DYX3_ARTIFACT_DIR="${T}/art_empty" install_prebuilt "${sha}") >/dev/null 2>&1
  rc=$?
  check "prebuilt: missing artifacts return 1 (caller builds) and leave nothing" '[ "${rc}" -ne 0 ] && [ ! -e "${rel}" ] && [ -z "$(ls -A "${DYX3_VAR_LIB}/state")" ]'

  # the good set (offline directory): release + px4_msgs installed, marked prebuilt, provenance kept
  (DYX3_ARTIFACT_DIR="${art}" install_prebuilt "${sha}") >"${T}/pb_ok" 2>&1
  rc=$?
  check "prebuilt: a valid offline artifact set installs" '[ "${rc}" -eq 0 ] && [ -f "${rel}/ros2_ws/install/setup.bash" ] && [ -f "${pm}/.complete" ]'
  check "prebuilt: release marked prebuilt, not complete, provenance kept" '[ -f "${rel}/.prebuilt" ] && [ ! -f "${rel}/.complete" ] && grep -q "ARTIFACT_CI_RUN=https://ci/run/1" "${rel}/artifacts.env"'
  check "prebuilt: build_release skips a prebuilt release" '(build_release "${sha}" 2>&1 | grep -q "nothing to build")'
  check "prebuilt: the extracted release passes static verification" '(health_release_only "${rel}" 0 >/dev/null 2>&1)'
  check "prebuilt: download state cleaned up" '[ ! -e "${DYX3_VAR_LIB}/state/artifacts-${sha}" ]'
}

sup
(libs)
(lifecycle)
(prebuilt)
pass="$(grep -c '^ok' "${RESULTS}")"
fail="$(grep -c '^bad' "${RESULTS}")"
[ -n "${SHOW_LOGS:-}" ] && tail -n +1 "${T}"/up_* 2>/dev/null
printf '\n%s passed, %s failed\n' "${pass}" "${fail}"
[ "${fail}" -eq 0 ]
