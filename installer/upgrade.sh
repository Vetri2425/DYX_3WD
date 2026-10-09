#!/usr/bin/env bash
# dyx3-upgrade <git-ref>
#   fetch -> releases/<sha> -> build -> verify -> switch symlink -> restart -> health.
# A release that fails verification never becomes current. A release that fails the
# post-switch health check is reverted automatically to the previous one.
set -euo pipefail

INSTALLER_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export INSTALLER_DIR
# shellcheck source=lib/common.sh
. "${INSTALLER_DIR}/lib/common.sh"
for lib in os_check dependencies ros_install permissions network_install systemd_install health_check usb_serial release; do
  # shellcheck disable=SC1090
  . "${INSTALLER_DIR}/lib/${lib}.sh"
done

ref="${1:-}"
case "${ref}" in
  "" | -h | --help)
    echo "usage: dyx3-upgrade <git-ref>   (branch, tag, or commit SHA)" >&2
    exit 2
    ;;
esac
[ "${2:-}" = "--dry-run" ] && DYX3_DRY_RUN=1

require_root
with_lock "${DYX3_RUN}/install.lock" upgrade_to "${ref}"
