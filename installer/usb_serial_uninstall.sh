#!/usr/bin/env bash
# Remove only the repository-managed CH341/BRLTTY provisioning artifacts.
# Run as root. Reboot after removal if ch341 is currently loaded or bound.
set -euo pipefail

if [ "$(id -u)" -ne 0 ]; then
  echo "must run as root (use sudo)" >&2
  exit 1
fi

version=1.0.0
rule=/etc/udev/rules.d/85-brltty.rules
source_dir="/usr/src/ch341-dyx3-${version}"
identity=/etc/dyx3/ch341-adapter.env
receiver_identity=/etc/dyx3/usb-receiver.env
unit=/etc/systemd/system/dyx3-usb-serial-check.service

if [ -e "${rule}" ]; then
  grep -Fq '# Managed by DYX 3WD installer: ch341-dyx3' "${rule}" || {
    echo "refusing to remove administrator udev file ${rule}" >&2
    exit 1
  }
  rm -f "${rule}"
fi
rm -rf /lib/modules/*/extra/ch341-dyx3 2>/dev/null || true
depmod -a 2>/dev/null || true
# Earlier DKMS-based provisioning (never deployed) is removed too if present.
if command -v dkms >/dev/null 2>&1; then
  dkms remove -m ch341-dyx3 -v "${version}" --all 2>/dev/null || true
fi
if command -v systemctl >/dev/null 2>&1; then systemctl disable --now dyx3-usb-serial-check.service 2>/dev/null || true; fi
rm -f "${unit}"
rm -rf "${source_dir}"
if [ -f "${identity}" ] && grep -Fq '# Managed by DYX 3WD installer: ch341-dyx3' "${identity}"; then rm -f "${identity}"; fi
if [ -f "${receiver_identity}" ] && grep -Fq '# Managed by DYX 3WD installer: confirmed UM982 USB identity' "${receiver_identity}"; then rm -f "${receiver_identity}"; fi
udevadm control --reload-rules
if command -v systemctl >/dev/null 2>&1; then systemctl daemon-reload; fi
depmod -a "$(uname -r)"
echo "DYX CH341 package and scoped BRLTTY override removed. Reboot to unload any in-use module and restore vendor BRLTTY behavior."
