#!/usr/bin/env bash
# dyx3-health [--deep]  — platform / release / px4_msgs health. Non-zero on any FAIL.
# (Phase 11 extends this to the control graph, RTK, spray, backend, recorder and disk.)
set -euo pipefail

INSTALLER_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export INSTALLER_DIR
# shellcheck source=lib/common.sh
. "${INSTALLER_DIR}/lib/common.sh"
for lib in os_check ros_install systemd_install health_check usb_serial; do
  # shellcheck disable=SC1090
  . "${INSTALLER_DIR}/lib/${lib}.sh"
done

# Root only (INS-022): several inputs are root-only (the 0600 hotspot keyfile, root:dyx3 0640 env files), and as
# another user their checks quietly read nothing.
require_root
health_run "$@"
