# DYX 3WD — runtime environment for the service launchers. Source, do not execute.
# shellcheck shell=bash
#
# Loads ROS 2, the px4_msgs overlay built from the PINNED firmware, and the release's workspace. It refuses to guess a
# ROS_DOMAIN_ID: the architecture says the domain is pinned (4.4) but names no number, and a wrong guess would silently
# join (or miss) another rover's graph. Set it in /etc/dyx3/ros.env.
#
# DDS scoping (architecture 4.4 says "scoped to eth0") is OPEN: the whole ROS graph is local to the Jetson (the XRCE agent talks
# to the FCU over XRCE, not DDS), so loopback-only is stricter and survives an unplugged FCU cable, while an eth0 interface
# whitelist would not. Opt in with DYX3_ROS_LOCALHOST_ONLY=1; the default changes nothing.

dyx3_env_load() {
  local rel="${DYX3_RELEASE_DIR:-/opt/dyx3/current}"
  local ros_setup="${DYX3_ROS_SETUP:-/opt/ros/humble/setup.bash}"
  local pm_root="${DYX3_PX4_MSGS_DIR:-/opt/dyx3/px4_msgs}"
  local fw_sha=""

  if [ -r "${rel}/installer/pins/firmware.pin" ]; then
    fw_sha="$(sed -n 's/^FIRMWARE_SHA=//p' "${rel}/installer/pins/firmware.pin" | head -n1)"
  fi
  [ -n "${fw_sha}" ] || {
    echo "dyx3: cannot read FIRMWARE_SHA from ${rel}/installer/pins/firmware.pin" >&2
    return 1
  }
  [ -n "${ROS_DOMAIN_ID:-}" ] || {
    echo "dyx3: ROS_DOMAIN_ID is not set; set it in /etc/dyx3/ros.env (refusing to guess)" >&2
    return 1
  }
  local f
  for f in "${ros_setup}" "${pm_root}/${fw_sha}/install/setup.bash" "${rel}/ros2_ws/install/setup.bash"; do
    [ -r "${f}" ] || {
      echo "dyx3: missing ${f}" >&2
      return 1
    }
    # ROS setup scripts are not nounset-clean.
    set +u
    # shellcheck disable=SC1090
    . "${f}"
    set -u
  done
  export ROS_DOMAIN_ID
  if [ "${DYX3_ROS_LOCALHOST_ONLY:-0}" = "1" ]; then export ROS_LOCALHOST_ONLY=1; fi
  # The service user's HOME (/var/lib/dyx3) is root-owned by design, so ROS cannot create ~/.ros there
  # (rcl aborts: "Failed to create log directory"). Point ROS at directories the service owns.
  export ROS_HOME="${ROS_HOME:-/var/lib/dyx3/state/ros}"
  export ROS_LOG_DIR="${ROS_LOG_DIR:-/var/log/dyx3/ros}"
  mkdir -p "${ROS_HOME}" "${ROS_LOG_DIR}" 2>/dev/null || true
  return 0
}
