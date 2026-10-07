#!/usr/bin/env bash
# dyx3-version — stack SHA, previous release, firmware pin, px4_msgs message-set hash, profile (key=value lines).
# The firmware identity printed is the PINNED expectation; the running FCU's own identity is not readable yet (OPEN).
set -euo pipefail

INSTALLER_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export INSTALLER_DIR
# shellcheck source=lib/common.sh
. "${INSTALLER_DIR}/lib/common.sh"
for lib in ros_install release; do
  # shellcheck disable=SC1090
  . "${INSTALLER_DIR}/lib/${lib}.sh"
done

print_version
