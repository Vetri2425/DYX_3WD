#!/usr/bin/env bash
# Runs INSIDE the ros2-humble container. Invoked by tools/dev/ros2_humble.sh; not for direct use.
# Layout: /host-src (read-only Mac working tree) -> rsync -> /work/repo (Linux copy, persistent
# volume) with ros2_ws/{build,install,log} preserved between runs.
set -euo pipefail

SRC=/host-src
REPO=/work/repo
LOGS=/host-logs
STAMP="$(date -u +%Y%m%dT%H%M%SZ)"
read -r -a CARGS <<<"${COLCON_ARGS:-}"

source_state() {
  echo "source: ${DYX3_HOST_SHA:-unknown} (${DYX3_HOST_DIRTY:-unknown})"
}

sync_src() {
  mkdir -p "${REPO}"
  # Exclude: git metadata, Linux build products (protected from --delete), and the untracked
  # root-level patch files that are not part of the source under test.
  rsync -a --delete \
    --filter='P /ros2_ws/build/' --filter='P /ros2_ws/install/' --filter='P /ros2_ws/log/' \
    --exclude='/.git/' --exclude='/build/' --exclude='/ros2_ws/build/' --exclude='/ros2_ws/install/' \
    --exclude='/ros2_ws/log/' --exclude='/*.patch' --exclude='__pycache__/' --exclude='.pytest_cache/' \
    --exclude='.venv/' --exclude='.DS_Store' \
    "${SRC}/" "${REPO}/"
  echo "synced working tree -> ${REPO}"
  source_state
}

ensure_msgs() {
  # shellcheck disable=SC1091
  export DYX3_ALLOW_ANY_OS=1 DYX3_BUILD_JOBS="${DYX3_BUILD_JOBS:-4}"
  cd "${REPO}"
  # shellcheck disable=SC1091
  source installer/lib/common.sh
  source installer/lib/os_check.sh
  source installer/lib/dependencies.sh
  source installer/lib/ros_install.sh
  local inputs stamp="${DYX3_PX4_MSGS_DIR}/.inputs.sha256"
  inputs="$(cat installer/pins/firmware.pin installer/lib/ros_install.sh | sha256sum | cut -d' ' -f1)"
  if [ -f "${stamp}" ] && [ "$(cat "${stamp}")" != "${inputs}" ]; then
    log "firmware.pin or ros_install.sh changed since the overlay was built: rebuilding"
    rm -rf "${DYX3_PX4_MSGS_DIR:?}"/*
  fi
  build_px4_msgs
  printf '%s\n' "${inputs}" >"${stamp}"
  local root
  root="$(px4_msgs_dir)"
  echo "px4_msgs overlay: ${root} firmware=$(cat "${root}/firmware.sha") msgs_sha256=$(cat "${root}/px4_msgs.sha256")"
}

overlay_setup() {
  local root
  root="$(find /opt/dyx3/px4_msgs -mindepth 2 -maxdepth 2 -name install -type d | sort | head -1)"
  [ -n "${root}" ] && [ -f "${root}/setup.bash" ] || { echo "px4_msgs overlay missing: run 'setup'" >&2; exit 3; }
  echo "${root}/setup.bash"
}

with_ros() {
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  # shellcheck disable=SC1090
  source "$(overlay_setup)"
  set -u
}

do_build() {
  with_ros
  cd "${REPO}/ros2_ws"
  mkdir -p "${LOGS}"
  colcon build --symlink-install "${CARGS[@]}" 2>&1 | tee "${LOGS}/build-${STAMP}.log"
  local rc=${PIPESTATUS[0]}
  echo "BUILD_EXIT=${rc}"
  return "${rc}"
}

do_test() {
  with_ros
  cd "${REPO}/ros2_ws"
  mkdir -p "${LOGS}"
  set +e
  colcon test "${CARGS[@]}" --event-handlers console_cohesion+ 2>&1 | tee "${LOGS}/test-${STAMP}.log"
  local rc_test=${PIPESTATUS[0]}
  colcon test-result --verbose 2>&1 | tee "${LOGS}/test-result-${STAMP}.log"
  local rc_res=${PIPESTATUS[0]}
  set -e
  grep -E "^Summary:" "${LOGS}/test-result-${STAMP}.log" || true
  echo "TEST_EXIT colcon_test=${rc_test} test_result=${rc_res}"
  [ "${rc_test}" -eq 0 ] && [ "${rc_res}" -eq 0 ]
}

cmd="${1:-}"
shift || true
case "${cmd}" in
  sync) sync_src ;;
  msgs) sync_src; ensure_msgs ;;
  build) sync_src; do_build ;;
  test) do_test ;;
  build-test) sync_src; do_build; do_test ;;
  shell) sync_src; with_ros; cd "${REPO}/ros2_ws"; exec bash -i ;;
  exec) with_ros; cd "${REPO}/ros2_ws"; exec "$@" ;;
  *) echo "unknown container command: ${cmd}" >&2; exit 2 ;;
esac
