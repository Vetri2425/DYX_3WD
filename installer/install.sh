#!/usr/bin/env bash
# dyx3-install --production [--ref <git-ref>] [--skip-deps]
#
# Idempotent: re-running converges the machine on the pinned state and never overwrites
# /etc/dyx3 files a human may have edited. Exits non-zero if any health check fails.
# Run from a git checkout of DYX_3WD (first install) or via /opt/dyx3/bin/dyx3-install.
set -euo pipefail

INSTALLER_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export INSTALLER_DIR
# shellcheck source=lib/common.sh
. "${INSTALLER_DIR}/lib/common.sh"
for lib in os_check dependencies ros_install permissions network_install systemd_install health_check usb_serial release; do
  # shellcheck disable=SC1090
  . "${INSTALLER_DIR}/lib/${lib}.sh"
done

usage() {
  cat <<USAGE
usage: install.sh --production [--ref <git-ref>] [--skip-deps] [--dry-run]
  --production   required; there is no development install here
  --ref <ref>    release to install (default: the commit of this checkout)
  --skip-deps    skip apt / XRCE agent / mavlink-router / ROS (already provisioned)
  --dry-run      print what would run
env: DYX3_BUILD_JOBS (default 1), FCU_IFACE (default enP8p1s0), FCU_KEEP_DHCP=1 (bench only)
USAGE
}

production=0
ref=""
skip_deps=0
while [ $# -gt 0 ]; do
  case "$1" in
    --production) production=1 ;;
    --ref)
      ref="${2:?--ref needs a value}"
      shift
      ;;
    --skip-deps) skip_deps=1 ;;
    --dry-run) DYX3_DRY_RUN=1 ;;
    -h | --help)
      usage
      exit 0
      ;;
    *)
      usage
      exit 2
      ;;
  esac
  shift
done
[ "${production}" -eq 1 ] || {
  usage
  exit 2
}

main() {
  require_root
  os_check
  create_user
  create_directories
  install_tmpfiles
  if [ "${skip_deps}" -eq 0 ]; then
    install_apt_packages
    install_xrce_agent
    install_mavlink_router
    install_ros
  fi
  # A receiver adapter is a required production input. Fail before switching releases if
  # this kernel, its headers, or its USB identity cannot be provisioned safely.
  provision_usb_serial_support
  install_fcu_network

  # Bootstrap the release from this checkout's origin so the release has real provenance.
  local checkout_root origin
  checkout_root="$(cd "${INSTALLER_DIR}/.." && pwd)"
  if [ -z "${DYX3_REPO_URL:-}" ] && origin="$(git -C "${checkout_root}" remote get-url origin 2>/dev/null)"; then
    export DYX3_REPO_URL="${origin}"
  fi
  if [ -z "${ref}" ]; then
    ref="$(git -C "${checkout_root}" rev-parse HEAD 2>/dev/null)" ||
      die "not in a git checkout; pass --ref"
  fi
  DYX3_FORCE=1 upgrade_to "${ref}"
  log "install complete"
}

with_lock "${DYX3_RUN}/install.lock" main
