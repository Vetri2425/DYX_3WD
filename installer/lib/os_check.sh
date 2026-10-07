#!/usr/bin/env bash
# DYX 3WD installer — platform checks. Source, do not execute.
# shellcheck shell=bash

# os_check: Ubuntu 22.04 (ROS 2 Humble's supported release) on aarch64.
# DYX3_ALLOW_ANY_OS=1 skips it (CI containers, laptops) — never set on a rover.
os_check() {
  if [ "${DYX3_ALLOW_ANY_OS:-0}" = "1" ]; then
    warn "OS check skipped (DYX3_ALLOW_ANY_OS=1)"
    return 0
  fi
  local osrel="${DYX3_OS_RELEASE:-/etc/os-release}"
  [ -r "${osrel}" ] || die "cannot read ${osrel}"
  # shellcheck disable=SC1090
  local id version_id
  id="$(. "${osrel}" && printf '%s' "${ID:-}")"
  version_id="$(. "${osrel}" && printf '%s' "${VERSION_ID:-}")"
  [ "${id}" = "ubuntu" ] || die "unsupported OS '${id}' (need ubuntu 22.04)"
  [ "${version_id}" = "22.04" ] || die "unsupported Ubuntu ${version_id} (need 22.04 for ROS 2 Humble)"
  local arch
  arch="${DYX3_ARCH:-$(uname -m)}"
  case "${arch}" in
    aarch64 | x86_64) ;;
    *) die "unsupported architecture ${arch}" ;;
  esac
  [ "${arch}" = "aarch64" ] || warn "architecture ${arch}: production target is aarch64 (Jetson Orin Nano)"
  log "OS ok: ubuntu ${version_id} ${arch}"
}

# memory_check: warn when a build is likely to stall the board.
memory_check() {
  local avail_kb
  avail_kb="$(awk '/MemAvailable/ {print $2}' /proc/meminfo 2>/dev/null || echo 0)"
  if [ "${avail_kb:-0}" -lt 3000000 ]; then
    warn "only $((avail_kb / 1024)) MiB available; keep DYX3_BUILD_JOBS=1 and close other work"
  fi
}
