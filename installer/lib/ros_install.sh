#!/usr/bin/env bash
# DYX 3WD installer — ROS 2 Humble and the firmware-pinned px4_msgs. Source, do not execute.
# shellcheck shell=bash

ROS_DISTRO_NAME="${ROS_DISTRO_NAME:-humble}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/${ROS_DISTRO_NAME}/setup.bash}"

ROS_APT_PACKAGES=(
  "ros-${ROS_DISTRO_NAME}-ros-base"
  ros-dev-tools
  "ros-${ROS_DISTRO_NAME}-ament-cmake-gtest"
  python3-rosdep
)

install_ros() {
  if [ -f "${ROS_SETUP}" ]; then
    log "ROS 2 ${ROS_DISTRO_NAME} already present"
    return 0
  fi
  log "installing ROS 2 ${ROS_DISTRO_NAME} (apt)"
  run mkdir -p /usr/share/keyrings
  run curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
  if [ "${DYX3_DRY_RUN}" != "1" ]; then
    printf 'deb [arch=%s signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu %s main\n' \
      "$(dpkg --print-architecture)" "$(. /etc/os-release && printf '%s' "${UBUNTU_CODENAME}")" \
      >/etc/apt/sources.list.d/ros2.list
  fi
  run apt-get update
  # No `apt upgrade`: a full upgrade on the rover is a change nobody asked for.
  run env DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends "${ROS_APT_PACKAGES[@]}"
}

# px4_msgs_dir: where the px4_msgs overlay for the pinned firmware lives.
px4_msgs_dir() {
  load_pin firmware
  printf '%s/%s' "${DYX3_PX4_MSGS_DIR}" "${FIRMWARE_SHA}"
}

# build_px4_msgs: px4_msgs generated from the PINNED FIRMWARE's msg/ + srv/ (never stock).
# Built once per firmware SHA into /opt/dyx3/px4_msgs/<sha>/install, shared by every
# release, so an upgrade does not pay for it again. Low parallelism by design.
build_px4_msgs() {
  load_pin firmware
  local root
  root="$(px4_msgs_dir)"
  if [ -f "${root}/.complete" ]; then
    log "px4_msgs for firmware ${FIRMWARE_SHA:0:10} already built"
    return 0
  fi
  memory_check
  log "building px4_msgs from firmware ${FIRMWARE_SHA:0:10} (-j${DYX3_BUILD_JOBS}); this takes a while"
  run rm -rf "${root}"
  run mkdir -p "${root}/src"

  # 1. skeleton (CMakeLists.txt, package.xml) from the pinned px4_msgs commit
  _checkout_pinned "${PX4_MSGS_REPO}" "${PX4_MSGS_SKELETON_REF}" "${root}/src/px4_msgs"
  run rm -rf "${root}/src/px4_msgs/msg" "${root}/src/px4_msgs/srv"
  run mkdir -p "${root}/src/px4_msgs/msg" "${root}/src/px4_msgs/srv"

  # 2. the firmware's own msg/ + srv/ at the exact flashed SHA
  local fw="${root}/firmware_msgs"
  run rm -rf "${fw}"
  run git init -q "${fw}"
  run git -C "${fw}" remote add origin "${FIRMWARE_REPO}"
  run git -C "${fw}" sparse-checkout set --no-cone /msg /srv
  run git -C "${fw}" fetch -q --depth 1 --filter=blob:none origin "${FIRMWARE_SHA}"
  run git -C "${fw}" checkout -q FETCH_HEAD
  if [ "${DYX3_DRY_RUN}" != "1" ]; then
    [ "$(git -C "${fw}" rev-parse HEAD)" = "${FIRMWARE_SHA}" ] || die "firmware checkout is not ${FIRMWARE_SHA}"
    # The px4_msgs package is flat: msg/*.msg plus msg/versioned/*.msg (their MESSAGE_VERSION
    # yields the *_v1 topic names). msg/px4_msgs_old and translation_node are NOT part of it.
    cp "${fw}"/msg/*.msg "${root}/src/px4_msgs/msg/"
    cp "${fw}"/msg/versioned/*.msg "${root}/src/px4_msgs/msg/"
    cp "${fw}"/srv/*.srv "${root}/src/px4_msgs/srv/"
    # Identity of the message set, recorded for the px4_link startup handshake and the manifest.
    (cd "${root}/src/px4_msgs" && find msg srv -type f | LC_ALL=C sort | xargs sha256sum | sha256sum | cut -d' ' -f1) \
      >"${root}/px4_msgs.sha256"
    printf '%s\n' "${FIRMWARE_SHA}" >"${root}/firmware.sha"
  fi

  # The sparse firmware checkout contains its own package.xml files (px4_msgs_old,
  # translation_node); keep it out of the build and drop it once copied.
  run rm -rf "${fw}"

  # 3. build
  run bash -c "set +u; . '${ROS_SETUP}'; set -u; cd '${root}' && \
    MAKEFLAGS='-j${DYX3_BUILD_JOBS}' nice -n 10 colcon build \
      --base-paths src --parallel-workers 1 --cmake-args -DCMAKE_BUILD_TYPE=Release" ||
    die "px4_msgs colcon build failed"
  sync_fs "${root}"
  run touch "${root}/.complete"
}
