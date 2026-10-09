#!/usr/bin/env bash
# dyx3-upgrade <git-ref>
#   fetch -> hand over to the TARGET's own installer -> releases/<sha> -> build -> verify -> switch symlink
#   -> restart -> health.
# A release that fails verification never becomes current. A release that fails the
# post-switch health check is reverted automatically to the previous one.
# The running installer only resolves the ref and extracts the target's installer/ + deployment/; the target's
# upgrade.sh (DYX3_REEXEC=1) does the rest with its own libraries and pins (INS-002/INS-003).
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

if [ "${DYX3_REEXEC:-0}" = "1" ]; then
  # We are the target release's installer, started by the previous one.
  reexec_contract "${ref}"
  with_lock "${DYX3_RUN}/install.lock" upgrade_to "${ref}"
  exit 0
fi
# A dropped ssh session must not kill the upgrade half-way (INS-004): continue as a transient systemd unit.
detach_or_continue upgrade "${BASH_SOURCE[0]}" "${ref}"
if [ "${DYX3_DRY_RUN}" = "1" ]; then
  log "dry-run: would hand over to the installer of ${ref}; showing this installer's plan"
  with_lock "${DYX3_RUN}/install.lock" upgrade_to "${ref}"
  exit 0
fi
handoff_to_target "${ref}"
