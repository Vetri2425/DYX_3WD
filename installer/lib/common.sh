#!/usr/bin/env bash
# DYX 3WD installer — shared helpers. Source, do not execute.
# shellcheck shell=bash
#
# Every path is rooted at $DYX3_ROOT (default empty = the real filesystem) so the
# installer can be exercised against a scratch directory in tests. Nothing here
# ever talks to the Jetson or to the FCU on its own.

set -euo pipefail

DYX3_ROOT="${DYX3_ROOT:-}"
DYX3_DRY_RUN="${DYX3_DRY_RUN:-0}"

INSTALLER_DIR="${INSTALLER_DIR:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
PINS_DIR="${INSTALLER_DIR}/pins"

DYX3_PREFIX="${DYX3_ROOT}/opt/dyx3"
DYX3_ETC="${DYX3_ROOT}/etc/dyx3"
DYX3_VAR_LIB="${DYX3_ROOT}/var/lib/dyx3"
DYX3_VAR_LOG="${DYX3_ROOT}/var/log/dyx3"
DYX3_RUN="${DYX3_ROOT}/run/dyx3"
DYX3_RELEASES="${DYX3_PREFIX}/releases"
DYX3_CURRENT="${DYX3_PREFIX}/current"
DYX3_BIN="${DYX3_PREFIX}/bin"
DYX3_PX4_MSGS_DIR="${DYX3_PREFIX}/px4_msgs"
DYX3_USER="${DYX3_USER:-dyx3}"
DYX3_GROUP="${DYX3_GROUP:-dyx3}"

# Low parallelism on purpose: the Orin Nano stalled at -j3 building px4_msgs
# (HANDOFF 2026-10-07). Raise only after watching memory on the rover.
DYX3_BUILD_JOBS="${DYX3_BUILD_JOBS:-1}"

log() { printf '[dyx3 %s] %s\n' "$(date -u +%H:%M:%SZ)" "$*" >&2; }
warn() { printf '[dyx3 WARN] %s\n' "$*" >&2; }
die() {
  printf '[dyx3 FATAL] %s\n' "$*" >&2
  exit 1
}

# run <cmd...>: execute, or print under DYX3_DRY_RUN=1.
run() {
  if [ "${DYX3_DRY_RUN}" = "1" ]; then
    printf '[dry-run] %s\n' "$*" >&2
    return 0
  fi
  "$@"
}

require_root() {
  if [ "${DYX3_ROOT}" = "" ] && [ "$(id -u)" -ne 0 ]; then
    die "must run as root (use sudo)"
  fi
}

have() { command -v "$1" >/dev/null 2>&1; }

# load_pin <name>: source installer/pins/<name>.pin
load_pin() {
  local f="${PINS_DIR}/$1.pin"
  [ -f "$f" ] || die "missing pin file: $f"
  # shellcheck disable=SC1090
  . "$f"
}

# manifest_section <section> [manifest]: print the non-comment lines of a section.
manifest_section() {
  local section="$1" manifest="${2:-${INSTALLER_DIR}/manifests/production.manifest}"
  sed 's/#.*//' "${manifest}" |
    awk -v want="[${section}]" '
      /^\[/ { in_s = ($0 == want); next }
      in_s && NF { gsub(/^[ \t]+|[ \t]+$/, ""); print }'
}

# Ownership is only applied on the real filesystem: a staged root (DYX3_ROOT, used by tests
# and CI) is not root-owned and the service user does not exist there.
_owner_args() {
  [ -n "${DYX3_ROOT}" ] && return 0
  printf '%s\n' -o "$1" -g "$2"
}

# install_file <mode> <owner:group> <src> <dst>
install_file() {
  local mode="$1" owner="$2" src="$3" dst="$4" own=()
  mapfile -t own < <(_owner_args "${owner%%:*}" "${owner##*:}")
  run install -D -m "${mode}" "${own[@]}" "${src}" "${dst}"
}

# install_dir <mode> <owner> <group> <dir...>
install_dir() {
  local mode="$1" owner="$2" group="$3" own=()
  shift 3
  mapfile -t own < <(_owner_args "${owner}" "${group}")
  run install -d -m "${mode}" "${own[@]}" "$@"
}

# atomic_symlink <target> <link>: replace a symlink without a window where it is missing.
atomic_symlink() {
  local target="$1" link="$2" tmp
  tmp="${link}.tmp.$$"
  run ln -sfn "${target}" "${tmp}"
  run mv -T "${tmp}" "${link}"
}

# with_lock <lockfile> <cmd...>: serialise installer/upgrade runs.
with_lock() {
  local lock="$1"
  shift
  if [ "${DYX3_DRY_RUN}" = "1" ]; then
    "$@"
    return
  fi
  mkdir -p "$(dirname "${lock}")"
  (
    flock -n 9 || die "another dyx3 install/upgrade is running (${lock})"
    "$@"
  ) 9>"${lock}"
}
