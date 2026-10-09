#!/usr/bin/env bash
# dyx3-rollback
#   switch back to the previous known-good release (the one recorded at the last switch), verify, restart, health.
#   A rollback that turns out unhealthy restores the release it started from. Running it twice undoes it.
set -euo pipefail

INSTALLER_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export INSTALLER_DIR
# shellcheck source=lib/common.sh
. "${INSTALLER_DIR}/lib/common.sh"
for lib in os_check dependencies ros_install permissions network_install systemd_install health_check usb_serial release; do
  # shellcheck disable=SC1090
  . "${INSTALLER_DIR}/lib/${lib}.sh"
done

case "${1:-}" in
  -h | --help)
    echo "usage: dyx3-rollback" >&2
    exit 2
    ;;
esac

require_root
detach_or_continue rollback "${BASH_SOURCE[0]}"
with_lock "${DYX3_RUN}/install.lock" rollback_release
