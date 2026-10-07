#!/usr/bin/env bash
# shellcheck disable=SC2016,SC2329
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
  for l in os_check dependencies ros_install permissions network_install systemd_install health_check release; do
    # shellcheck disable=SC1090
    . "${INSTALLER_DIR}/lib/${l}.sh"
  done
  set +e

  local svc
  svc="$(manifest_section enabled_services)"
  check "manifest: only dyx3-platform enabled" '[ "${svc}" = "dyx3-platform" ]'
  check "manifest: legacy package absent from ros2_packages" '! manifest_section ros2_packages | grep -q legacy'

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

  # staged real directory creation (no chown)
  (create_directories >/dev/null 2>&1)
  check "staged dirs created" '[ -d "${DYX3_VAR_LIB}/runs" ] && [ -d "${DYX3_PREFIX}/releases" ] && [ -d "${DYX3_ETC}" ]'
}

# ---------------------------------------------------------------- release lifecycle
lifecycle() {
  local src="${T}/src" fakebin="${T}/fakebin"
  mkdir -p "${src}" "${fakebin}"
  cp -r "${REPO}/installer" "${REPO}/deployment" "${src}/"
  rm -rf "${src}/installer/tests"
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
  export DYX3_ROOT="${T}/lc" INSTALLER_DIR="${REPO}/installer" DYX3_SKIP_SYSTEMD=1
  # shellcheck disable=SC1091
  . "${INSTALLER_DIR}/lib/common.sh"
  for l in os_check dependencies ros_install permissions network_install systemd_install health_check release; do
    # shellcheck disable=SC1090
    . "${INSTALLER_DIR}/lib/${l}.sh"
  done
  set +e
  create_directories >/dev/null 2>&1
  load_pin firmware
  local pm="${DYX3_PX4_MSGS_DIR}/${FIRMWARE_SHA}"
  mkdir -p "${pm}/install"
  touch "${pm}/install/setup.bash" "${pm}/.complete"
  echo abc123def456 >"${pm}/px4_msgs.sha256"
  # Never really build px4_msgs in tests.
  # shellcheck disable=SC2317  # invoked indirectly by upgrade_to
  build_px4_msgs() { :; }

  (upgrade_to "${A}") >"${T}/up_a" 2>&1
  rc=$?
  check "first release installs (rc=0)" '[ "${rc}" -eq 0 ]'
  check "current -> A" '[ "$(basename "$(readlink -f "${DYX3_CURRENT}")")" = "${A}" ]'
  check "release has .complete and launchers" '[ -f "${DYX3_RELEASES}/${A}/.complete" ] && [ -x "${DYX3_RELEASES}/${A}/bin/dyx3-platform" ]'
  check "config templates installed" '[ -f "${DYX3_ETC}/platform.env" ] && [ -f "${DYX3_ETC}/mavlink-router.conf" ]'
  check "operator shims installed" '[ -x "${DYX3_BIN}/dyx3-upgrade" ] && [ -x "${DYX3_BIN}/dyx3-health" ] && [ -x "${DYX3_BIN}/dyx3-install" ]'
  check "units copied, only platform marked enabled in manifest" '[ -f "${DYX3_ROOT}/etc/systemd/system/dyx3-platform.service" ]'

  echo "EDITED=1" >>"${DYX3_ETC}/platform.env"
  (upgrade_to "${B}") >"${T}/up_b" 2>&1
  rc=$?
  check "upgrade to B (rc=0)" '[ "${rc}" -eq 0 ]'
  check "current -> B, previous recorded as A" '[ "$(basename "$(readlink -f "${DYX3_CURRENT}")")" = "${B}" ] && [ "$(cat "${DYX3_VAR_LIB}/state/previous_release")" = "${A}" ]'
  check "upgrade never overwrites edited /etc config" 'grep -q "EDITED=1" "${DYX3_ETC}/platform.env"'

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
}

sup
(libs)
(lifecycle)
pass="$(grep -c '^ok' "${RESULTS}")"
fail="$(grep -c '^bad' "${RESULTS}")"
[ -n "${SHOW_LOGS:-}" ] && tail -n +1 "${T}"/up_* 2>/dev/null
printf '\n%s passed, %s failed\n' "${pass}" "${fail}"
[ "${fail}" -eq 0 ]
