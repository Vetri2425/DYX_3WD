#!/usr/bin/env bash
# Idempotent installer hook for fresh setup and the first upgrade from older releases.
# Does not configure the UM982 or select/change the RTK transport.
set -euo pipefail
INSTALLER_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export INSTALLER_DIR
. "${INSTALLER_DIR}/lib/common.sh"
. "${INSTALLER_DIR}/lib/health_check.sh"
. "${INSTALLER_DIR}/lib/usb_serial.sh"
provision_usb_serial_support
_health_fail=0
health_usb_serial
[ "${_health_fail}" -eq 0 ]
