#!/usr/bin/env bash
# DYX 3WD — persistent CH341 support for the validated Orin Nano kernel.
# Source, do not execute. The staged-root path is used by installer tests only.
# This function is intentionally never called by this repository's test harness on hardware.
# shellcheck shell=bash

DYX3_CH341_USB_SYSFS="${DYX3_CH341_USB_SYSFS:-/sys/bus/usb/devices}"
DYX3_CH341_DEV_ROOT="${DYX3_CH341_DEV_ROOT:-/dev}"
DYX3_CH341_SOURCE_VERSION="1.0.0"
DYX3_CH341_SUPPORTED_KERNEL="5.15.185-tegra"
DYX3_CH341_KERNEL_PACKAGE_VERSION="5.15.185-tegra-36.5.0-20260115194252"
DYX3_CH341_VID="1a86"
DYX3_CH341_PID="7523"
DYX3_CH341_RULE_MARKER="# Managed by DYX 3WD installer: ch341-dyx3"
DYX3_USB_BRLTTY_RULE_CHANGED=0

_usb_serial_sysroot() {
  if [ -n "${DYX3_ROOT}" ]; then printf '%s' "${DYX3_ROOT}"; else printf ''; fi
}

_usb_serial_supported_kernel() {
  local kernel="${DYX3_CH341_KERNEL:-$(uname -r)}"
  [ "${kernel}" = "${DYX3_CH341_SUPPORTED_KERNEL}" ] ||
    die "unsupported Jetson kernel '${kernel}': ch341-dyx3 1.0.0 is validated only for ${DYX3_CH341_SUPPORTED_KERNEL}; install matching NVIDIA L4T headers/source and validate before proceeding"
  printf '%s' "${kernel}"
}

# usb_serial_detect_adapter: print exactly one adapter's ID_PATH; refuse zero or ambiguity.
usb_serial_detect_adapter() {
  local root="${DYX3_CH341_USB_SYSFS}" d vid pid props path found="" count=0
  for d in "${root}"/*; do
    [ -r "${d}/idVendor" ] && [ -r "${d}/idProduct" ] || continue
    IFS= read -r vid <"${d}/idVendor" || true
    IFS= read -r pid <"${d}/idProduct" || true
    [ "${vid,,}" = "${DYX3_CH341_VID}" ] && [ "${pid,,}" = "${DYX3_CH341_PID}" ] || continue
    props="$(udevadm info --query=property --path="${d}" 2>/dev/null)" || die "cannot read udev identity for CH340 adapter at ${d}"
    path="$(printf '%s\n' "${props}" | sed -n 's/^ID_PATH=//p' | head -n1)"
    [[ "${path}" =~ ^[A-Za-z0-9_.:-]+$ ]] || die "CH340 adapter at ${d} has no safe, stable ID_PATH; refuse broad device matching"
    count=$((count + 1))
    found="${path}"
  done
  [ "${count}" -eq 1 ] || die "expected exactly one CH340 adapter (${DYX3_CH341_VID}:${DYX3_CH341_PID}); found ${count}; connect/identify the adapter and retry"
  printf '%s' "${found}"
}

_usb_serial_install_brltty_override() {
  local id_path="$1" root vendor dst tmp original_line
  root="$(_usb_serial_sysroot)"
  vendor="${root}/lib/udev/rules.d/85-brltty.rules"
  dst="${root}/etc/udev/rules.d/85-brltty.rules"
  [ -f "${vendor}" ] || {
    # No installed BRLTTY vendor rule means no conflict to override.
    return 0
  }
  if [ -e "${dst}" ] && ! grep -Fq "${DYX3_CH341_RULE_MARKER}" "${dst}"; then
    die "administrator udev override already exists at ${dst}; refusing to replace it"
  fi
  original_line="$(grep -F 'ENV{PRODUCT}=="1a86/7523/*"' "${vendor}" || true)"
  [ -n "${original_line}" ] || {
    # A package revision may have changed the rule; fail closed instead of assuming it is safe.
    die "BRLTTY rule format changed in ${vendor}; inspect before creating a device exclusion"
  }
  [ "$(printf '%s\n' "${original_line}" | wc -l | tr -d ' ')" = 1 ] ||
    die "expected one generic CH340 rule in ${vendor}; refusing ambiguous override"
  tmp="${dst}.tmp.$$"
  if [ "${DYX3_DRY_RUN}" = 1 ]; then
    log "would shadow BRLTTY rule at ${dst}, excluding only ID_PATH=${id_path}"
    return 0
  fi
  install -d -m 0755 "$(dirname "${dst}")"
  # Shadow, never edit the package-owned file. Add the ID_PATH predicate to the exact
  # vendor match; all other BRLTTY devices keep the vendor behavior.
  awk -v marker="${DYX3_CH341_RULE_MARKER}" -v idpath="${id_path}" '
    BEGIN { print marker }
    index($0, "ENV{PRODUCT}==\"1a86/7523/*\"") {
      sub(/, ENV\{BRLTTY_BRAILLE_DRIVER\}/,
          ", ENV{ID_PATH}!=\"" idpath "\", ENV{BRLTTY_BRAILLE_DRIVER}")
    }
    { print }
  ' "${vendor}" >"${tmp}"
  chmod 0644 "${tmp}"
  if [ -e "${dst}" ] && cmp -s "${tmp}" "${dst}"; then rm -f "${tmp}"; return 0; fi
  mv -f "${tmp}" "${dst}"
  DYX3_USB_BRLTTY_RULE_CHANGED=1
}

_usb_serial_install_dkms_source() {
  local root source dest module
  root="$(_usb_serial_sysroot)"
  source="${INSTALLER_DIR}/drivers/ch341-dyx3-${DYX3_CH341_SOURCE_VERSION}"
  dest="${root}/usr/src/ch341-dyx3-${DYX3_CH341_SOURCE_VERSION}"
  [ -s "${source}/ch341.c" ] && [ -f "${source}/dkms.conf" ] || die "versioned CH341 source package missing from installer"
  printf '%s  %s\n' "f66d070eab6235b8a5c7a06a283d2feecb8fa3d81bc1323c847c2d2cbf7bd410" "${source}/ch341.c" | sha256sum -c - >/dev/null ||
    die "vendored NVIDIA CH341 source hash mismatch"
  if [ -n "${DYX3_ROOT}" ]; then
    install -d -m 0755 "${dest}"
    local file
    for file in ch341.c Makefile dkms.conf PROVENANCE; do
      if ! cmp -s "${source}/${file}" "${dest}/${file}"; then install -m 0644 "${source}/${file}" "${dest}/${file}"; fi
    done
    return 0
  fi
  _usb_serial_ensure_host_build_tools
  install -d -m 0755 "${dest}"
  local file
  for file in ch341.c Makefile dkms.conf PROVENANCE; do
    if ! cmp -s "${source}/${file}" "${dest}/${file}"; then install -m 0644 "${source}/${file}" "${dest}/${file}"; fi
  done
  # Direct kbuild against the exact installed NVIDIA headers with the existing toolchain. No DKMS:
  # on the rover, installing dkms pulled gcc-12 and upgraded 11 system libraries (libstdc++6,
  # libgcc-s1, ...), which this installer refuses. The package is pinned to one kernel anyway, so
  # DKMS's auto-rebuild adds nothing; a kernel change fails closed at _usb_serial_supported_kernel.
  local kernel="${DYX3_CH341_SUPPORTED_KERNEL}" moddir ko stamp source_sha build
  moddir="/lib/modules/${kernel}/extra/ch341-dyx3"
  ko="${moddir}/ch341.ko"
  stamp="${moddir}/SOURCE_SHA256"
  source_sha="$(sha256sum "${source}/ch341.c" | awk '{print $1}')"
  if [ ! -f "${ko}" ] || [ "$(cat "${stamp}" 2>/dev/null || true)" != "${source_sha}" ]; then
    build="$(mktemp -d /tmp/ch341-dyx3-build.XXXXXX)"
    install -m 0644 "${source}/ch341.c" "${source}/Makefile" "${build}/"
    if ! make -C "/lib/modules/${kernel}/build" M="${build}" modules >"${build}/make.log" 2>&1; then
      die "could not build ch341 for ${kernel}; no module was installed or loaded (log: ${build}/make.log)"
    fi
    install -d -m 0755 "${moddir}"
    install -m 0644 "${build}/ch341.ko" "${ko}"
    printf '%s\n' "${source_sha}" >"${stamp}"
    depmod -a "${kernel}" || die "depmod failed for ${kernel}"
    rm -rf "${build}"
  fi
  modinfo -k "${kernel}" ch341 >/dev/null 2>&1 || die "installed ch341 module is not visible to modinfo"
  local vermagic
  vermagic="$(modinfo -k "${kernel}" -F vermagic ch341 2>/dev/null | awk '{print $1}')"
  [ "${vermagic}" = "${kernel}" ] || die "installed ch341 vermagic '${vermagic}' does not match ${kernel}"
}

# Install exact NVIDIA headers and the Ubuntu DKMS tool when absent. Simulate each apt
# transaction first and refuse any upgrade/removal or kernel package change.
_usb_serial_ensure_host_build_tools() {
  local kernel_pkg header_pkg candidate simulation upgraded
  have dpkg-query && have apt-get && have apt-cache || die "dpkg/apt tooling unavailable; cannot safely provision CH341"
  kernel_pkg="$(dpkg-query -W -f='${Version}' nvidia-l4t-kernel 2>/dev/null || true)"
  [ "${kernel_pkg}" = "${DYX3_CH341_KERNEL_PACKAGE_VERSION}" ] ||
    die "unsupported NVIDIA kernel package '${kernel_pkg:-missing}'; expected ${DYX3_CH341_KERNEL_PACKAGE_VERSION}; no kernel replacement will be attempted"

  header_pkg="$(dpkg-query -W -f='${Version}' nvidia-l4t-kernel-headers 2>/dev/null || true)"
  if [ "${header_pkg}" != "${DYX3_CH341_KERNEL_PACKAGE_VERSION}" ] ||
    [ ! -e "/lib/modules/${DYX3_CH341_SUPPORTED_KERNEL}/build/Makefile" ]; then
    candidate="nvidia-l4t-kernel-headers=${DYX3_CH341_KERNEL_PACKAGE_VERSION}"
    if ! apt-cache show "${candidate}" >/dev/null 2>&1; then
      apt-get update || die "apt metadata refresh failed while locating exact matching NVIDIA headers"
    fi
    apt-cache show "${candidate}" >/dev/null 2>&1 ||
      die "exact matching NVIDIA header package ${candidate} is unavailable; refusing a newer candidate"
    simulation="$(apt-get -s install --no-install-recommends "${candidate}" 2>&1)" ||
      die "APT simulation failed for ${candidate}"
    upgraded="$(printf '%s\n' "${simulation}" | awk '/^[0-9]+ upgraded,/ {print $1; exit}')"
    [ "${upgraded:-1}" = 0 ] || die "installing ${candidate} would upgrade packages; refusing kernel-adjacent changes"
    if printf '%s\n' "${simulation}" | grep -Eq '^(Remv|Inst) nvidia-l4t-kernel([[:space:]]|$)'; then
      die "installing ${candidate} would replace the running NVIDIA kernel package; refusing"
    fi
    apt-get install -y --no-install-recommends "${candidate}" || die "could not install exact matching headers ${candidate}"
  fi
  header_pkg="$(dpkg-query -W -f='${Version}' nvidia-l4t-kernel-headers 2>/dev/null || true)"
  [ "${header_pkg}" = "${DYX3_CH341_KERNEL_PACKAGE_VERSION}" ] &&
    [ -e "/lib/modules/${DYX3_CH341_SUPPORTED_KERNEL}/build/Makefile" ] ||
    die "matching headers are still unavailable for ${DYX3_CH341_SUPPORTED_KERNEL} after provisioning"

  have make && have gcc ||
    die "make/gcc missing (build-essential); cannot build the CH341 module"
}

# Read DYX3_CH341_EXPECTED_ID_PATH from an env file. A missing file means "not provisioned yet"
# and must not abort callers running under set -euo pipefail (first install, fresh rover).
_usb_serial_recorded_id_path() {
  local file="$1"
  [ -f "${file}" ] || return 0
  sed -n 's/^DYX3_CH341_EXPECTED_ID_PATH=//p' "${file}" | head -n1
}

_usb_serial_write_adapter_identity() {
  local id_path="$1" root dst tmp existing
  root="$(_usb_serial_sysroot)"
  dst="${root}/etc/dyx3/ch341-adapter.env"
  existing="$(_usb_serial_recorded_id_path "${dst}")"
  if [ -n "${existing}" ] && [ "${existing}" != "${id_path}" ]; then
    die "recorded CH340 ID_PATH differs from detected adapter (${existing} vs ${id_path}); verify physical connection and remove ${dst} only as part of an approved reprovision"
  fi
  [ "${existing}" = "${id_path}" ] && return 0
  [ "${DYX3_DRY_RUN}" = 1 ] && return 0
  install -d -m 0755 "$(dirname "${dst}")"
  tmp="${dst}.tmp.$$"
  {
    printf '%s\n' '# Managed by DYX 3WD installer: ch341-dyx3'
    printf 'DYX3_CH341_EXPECTED_ID_PATH=%s\n' "${id_path}"
  } >"${tmp}"
  chmod 0644 "${tmp}"
  mv -f "${tmp}" "${dst}"
}

_usb_serial_write_receiver_identity() {
  local detected="$1" expected="${DYX3_UM982_USB_ID_PATH:-}" tty props tty_path links link selected="" count=0 dst tmp
  [ -n "${expected}" ] || {
    log "UM982 receiver identity not recorded: physical USB COM wiring confirmation is pending"
    return 0
  }
  [ "${expected}" = "${detected}" ] ||
    die "DYX3_UM982_USB_ID_PATH does not match the detected adapter; refusing to bind a receiver identity"
  for tty in "${DYX3_CH341_DEV_ROOT}"/ttyUSB*; do
    [ -c "${tty}" ] || continue
    props="$(udevadm info --query=property --name="${tty}" 2>/dev/null)" || continue
    tty_path="$(printf '%s\n' "${props}" | sed -n 's/^ID_PATH=//p' | head -n1)"
    [[ "${tty_path}" == "${expected}"* ]] || continue
    links="$(printf '%s\n' "${props}" | sed -n 's/^DEVLINKS=//p' | head -n1)"
    for link in ${links}; do
      case "${link}" in /dev/serial/by-path/*) selected="${link}"; count=$((count + 1)) ;; esac
    done
  done
  [ "${count}" -eq 1 ] || die "physical UM982 wiring was confirmed for ${expected}, but exactly one matching /dev/serial/by-path identity was not found (matches=${count})"
  dst="${DYX3_ROOT}/etc/dyx3/usb-receiver.env"
  if [ -f "${dst}" ] && ! grep -Fq '# Managed by DYX 3WD installer: confirmed UM982 USB identity' "${dst}"; then
    die "receiver identity file ${dst} is not installer-managed; refusing to replace it"
  fi
  if [ -f "${dst}" ] && grep -qx "DYX3_USB_RECEIVER_DEVICE=${selected}" "${dst}"; then return 0; fi
  [ "${DYX3_DRY_RUN}" = 1 ] && return 0
  install -d -m 0755 "$(dirname "${dst}")"
  tmp="${dst}.tmp.$$"
  {
    printf '%s\n' '# Managed by DYX 3WD installer: confirmed UM982 USB identity'
    printf 'DYX3_USB_RECEIVER_ID_PATH=%s\n' "${expected}"
    printf 'DYX3_USB_RECEIVER_DEVICE=%s\n' "${selected}"
  } >"${tmp}"
  chmod 0644 "${tmp}"
  mv -f "${tmp}" "${dst}"
}

# The running brltty-udev daemon auto-detects USB serial bridges on its own and keeps the CH340
# claimed through usbfs; the scoped udev exclusion cannot release a device it already holds (rover
# 2026-10-09: ch341 loaded, device stayed on usbfs). The rover is headless with no braille display,
# so BRLTTY is masked. Only units that exist and are not already masked are touched, and they are
# recorded so usb_serial_uninstall.sh unmasks exactly those. No package is removed.
_usb_serial_mask_brltty() {
  local root marker unit
  root="$(_usb_serial_sysroot)"
  marker="${root}/etc/dyx3/brltty-masked-by-dyx3"
  for unit in brltty-udev.service brltty.service; do
    systemctl list-unit-files --no-legend "${unit}" 2>/dev/null | grep -q "^${unit}" || continue
    [ "$(systemctl is-enabled "${unit}" 2>/dev/null || true)" = masked ] && continue
    systemctl mask --now "${unit}" || die "could not mask ${unit}; CH340 stays claimed by BRLTTY"
    install -d -m 0755 "$(dirname "${marker}")"
    grep -qx "${unit}" "${marker}" 2>/dev/null || printf '%s\n' "${unit}" >>"${marker}"
  done
}

# Force the kernel to re-probe the CH340 so ch341 can claim it after BRLTTY released it.
_usb_serial_reenumerate_adapter() {
  local d vid pid
  for d in "${DYX3_CH341_USB_SYSFS}"/*; do
    [ -r "${d}/idVendor" ] && [ -r "${d}/idProduct" ] && [ -w "${d}/authorized" ] || continue
    IFS= read -r vid <"${d}/idVendor" || true; IFS= read -r pid <"${d}/idProduct" || true
    [ "${vid,,}" = "${DYX3_CH341_VID}" ] && [ "${pid,,}" = "${DYX3_CH341_PID}" ] || continue
    echo 0 >"${d}/authorized" || true
    sleep 1
    echo 1 >"${d}/authorized" || die "could not re-authorize CH340 at ${d}"
  done
  sleep 2
}

# provision_usb_serial_support: installer entry point. Hardware identity is derived per rover.
provision_usb_serial_support() {
  local kernel id_path
  if [ "${DYX3_DRY_RUN}" = 1 ]; then
    log "dry-run: would require one CH340 adapter, an exact supported kernel, matching headers, scoped BRLTTY exclusion, and a ch341-dyx3 module built against the exact headers (no DKMS)"
    return 0
  fi
  [ "${DYX3_ROOT}" != "" ] || kernel="$(_usb_serial_supported_kernel)"
  if [ -n "${DYX3_ROOT}" ]; then
    kernel="${DYX3_CH341_KERNEL:-${DYX3_CH341_SUPPORTED_KERNEL}}"
    [ "${kernel}" = "${DYX3_CH341_SUPPORTED_KERNEL}" ] || die "unsupported Jetson kernel '${kernel}': ch341-dyx3 1.0.0 supports only ${DYX3_CH341_SUPPORTED_KERNEL}"
  fi
  id_path="$(usb_serial_detect_adapter)" || return 1
  log "CH340 adapter detected at ${id_path}; checking scoped BRLTTY exclusion and ch341 provisioning"
  _usb_serial_write_adapter_identity "${id_path}"
  DYX3_USB_BRLTTY_RULE_CHANGED=0
  _usb_serial_install_brltty_override "${id_path}"
  _usb_serial_install_dkms_source
  if [ -z "${DYX3_ROOT}" ] && [ "${DYX3_DRY_RUN}" != 1 ]; then
    # The USB subsystem binds by modalias after the module is registered. Do not write
    # receiver configuration or send data; health below checks actual binding.
    local bound="${DYX3_CH341_USB_SYSFS}"
    local found=0 d vid pid driver
    for d in "${bound}"/*; do
      [ -r "${d}/idVendor" ] && [ -r "${d}/idProduct" ] || continue
      IFS= read -r vid <"${d}/idVendor" || true; IFS= read -r pid <"${d}/idProduct" || true
      [ "${vid,,}" = "${DYX3_CH341_VID}" ] && [ "${pid,,}" = "${DYX3_CH341_PID}" ] || continue
      driver="$(basename "$(readlink -f "${d}:1.0/driver" 2>/dev/null || true)")"
      [ "${driver}" = ch341 ] && found=1
    done
    if [ "${found}" -eq 0 ] || [ "${DYX3_USB_BRLTTY_RULE_CHANGED}" -eq 1 ]; then
      udevadm control --reload-rules
      _usb_serial_mask_brltty
      modprobe ch341 || die "modprobe ch341 failed; see dmesg"
      _usb_serial_reenumerate_adapter
      udevadm settle --timeout=10 || true
    fi
    found=0
    for d in "${bound}"/*; do
      [ -r "${d}/idVendor" ] && [ -r "${d}/idProduct" ] || continue
      IFS= read -r vid <"${d}/idVendor" || true; IFS= read -r pid <"${d}/idProduct" || true
      [ "${vid,,}" = "${DYX3_CH341_VID}" ] && [ "${pid,,}" = "${DYX3_CH341_PID}" ] || continue
      driver="$(basename "$(readlink -f "${d}:1.0/driver" 2>/dev/null || true)")"
      [ "${driver}" = ch341 ] && found=1
    done
    [ "${found}" -eq 1 ] || die "CH341 did not bind after provisioning; adapter remains unavailable. Roll back with installer/usb_serial_uninstall.sh"
    [ -c "${DYX3_CH341_DEV_ROOT}/ttyUSB0" ] || [ -n "$(compgen -G "${DYX3_CH341_DEV_ROOT}/ttyUSB*" || true)" ] || die "CH341 bound but no ttyUSB node appeared under ${DYX3_CH341_DEV_ROOT}"
    local tty group
    tty="$(compgen -G "${DYX3_CH341_DEV_ROOT}/ttyUSB*" | head -n1)"
    group="$(stat -c %G "${tty}")"
    [ "${group}" = dialout ] || die "${tty} belongs to '${group}', expected dialout; dyx3 must not be granted broad device access"
    _usb_serial_write_receiver_identity "${id_path}"
  fi
  log "CH341 support provisioned for ${kernel}; receiver COM wiring and baud remain unverified"
}

# health_usb_serial: absence is a warning only when no adapter has been detected yet;
# configured receiver identity is deliberately gated on separate physical confirmation.
health_usb_serial() {
  local kernel="${DYX3_CH341_KERNEL:-$(uname -r)}" count=0 d vid pid driver expected actual
  expected="$(_usb_serial_recorded_id_path "${DYX3_ROOT}/etc/dyx3/ch341-adapter.env")"
  if ! have lsusb || ! lsusb -d "${DYX3_CH341_VID}:${DYX3_CH341_PID}" 2>/dev/null | grep -q .; then
    if [ -n "${expected}" ]; then _fail "provisioned CH340 adapter ID_PATH=${expected} is absent"; else _warn "CH340 adapter not present; USB RTK device is not provisioned"; fi
    return 0
  fi
  if [ "${kernel}" != "${DYX3_CH341_SUPPORTED_KERNEL}" ]; then
    _fail "CH340 detected on unsupported kernel ${kernel}; validated CH341 support is ${DYX3_CH341_SUPPORTED_KERNEL}"
    return 0
  fi
  if ! modinfo ch341 >/dev/null 2>&1; then
    _fail "CH340 detected but ch341 module is unavailable"
    return 0
  fi
  for d in "${DYX3_CH341_USB_SYSFS}"/*; do
    [ -r "${d}/idVendor" ] && [ -r "${d}/idProduct" ] || continue
    IFS= read -r vid <"${d}/idVendor" || true; IFS= read -r pid <"${d}/idProduct" || true
    [ "${vid,,}" = "${DYX3_CH341_VID}" ] && [ "${pid,,}" = "${DYX3_CH341_PID}" ] || continue
    if [ -n "${expected}" ]; then
      actual="$(udevadm info --query=property --path="${d}" 2>/dev/null | sed -n 's/^ID_PATH=//p' | head -n1)"
      [ "${actual}" = "${expected}" ] || continue
    fi
    driver="$(basename "$(readlink -f "${d}:1.0/driver" 2>/dev/null || true)")"
    [ "${driver}" = ch341 ] && count=$((count + 1))
  done
  if [ "${count}" -eq 1 ] && compgen -G "${DYX3_CH341_DEV_ROOT}/ttyUSB*" >/dev/null; then
    _pass "CH340 adapter bound to ch341 with a ttyUSB node"
  else
    _fail "CH340 detected but CH341 binding/node is unavailable (bound interfaces=${count})"
  fi
}
