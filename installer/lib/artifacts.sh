#!/usr/bin/env bash
# DYX 3WD installer — prebuilt release artifacts. Source, do not execute.
# shellcheck shell=bash
#
# Proposal: docs/architecture/proposals/2026-10-08_prebuilt-release-artifacts.md (ACCEPTED).
#
# CI (job rover_artifacts, installer/ci/build_rover_artifacts.sh) builds a release with the SAME
# build_px4_msgs/build_release functions, at the same absolute paths, and publishes GitHub Release
# `rover-<stack-sha>` with:
#   release-<stack-sha>.tar.zst      opt/dyx3/releases/<stack-sha>/  (no ros2_ws/build, ros2_ws/log)
#   px4_msgs-<firmware-sha>.tar.zst  opt/dyx3/px4_msgs/<firmware-sha>/  (install + identity files)
#   artifacts.env                    provenance: stack/firmware SHA, OS, arch, ROS distro, CI run
#   SHA256SUMS                       sha256 of each file above
#
# DYX3_ARTIFACTS   auto (default): use the prebuilt release, else build on this machine
#                  prebuilt: fail when no valid artifact exists       source: always build here
# DYX3_ARTIFACT_DIR  read the files from this directory (USB stick, no WAN) instead of downloading.
#
# Integrity (human decision 2026-10-08): SHA-256 over GitHub TLS. Every file is checked against
# SHA256SUMS before anything is extracted; archives may only contain their own prefix.

DYX3_ARTIFACTS="${DYX3_ARTIFACTS:-auto}"
DYX3_ARTIFACT_DIR="${DYX3_ARTIFACT_DIR:-}"

artifact_tag() { printf 'rover-%s' "$1"; }

# _artifact_base_url: https://github.com/<owner>/<repo>/releases/download (GitHub https remotes only).
_artifact_base_url() {
  local url
  url="$(_repo_url)"
  url="${url%.git}"
  case "${url}" in
    https://github.com/*/*) printf '%s/releases/download' "${url}" ;;
    *) return 1 ;;
  esac
}

# _artifact_get <sha> <dest-dir> <file>: copy from DYX3_ARTIFACT_DIR or download one release asset.
_artifact_get() {
  local sha="$1" dir="$2" f="$3" base
  if [ -n "${DYX3_ARTIFACT_DIR}" ]; then
    [ -f "${DYX3_ARTIFACT_DIR}/${f}" ] || {
      warn "artifact ${f} not in ${DYX3_ARTIFACT_DIR}"
      return 1
    }
    cp "${DYX3_ARTIFACT_DIR}/${f}" "${dir}/${f}"
    return
  fi
  base="$(_artifact_base_url)" || {
    warn "no artifact URL for $(_repo_url)"
    return 1
  }
  curl -fsSL --retry 3 --connect-timeout 15 -o "${dir}/${f}" "${base}/$(artifact_tag "${sha}")/${f}" || {
    warn "no prebuilt ${f} for ${sha:0:10}"
    return 1
  }
}

# _artifact_verify <dir> <file>: the file's sha256 must equal its SHA256SUMS line (a file the list
# does not name is refused, not skipped).
_artifact_verify() {
  local dir="$1" f="$2" want got
  want="$(awk -v f="${f}" '$2 == f || $2 == "*" f { print $1; exit }' "${dir}/SHA256SUMS")"
  [ -n "${want}" ] || {
    warn "${f} is not listed in SHA256SUMS"
    return 1
  }
  got="$(sha256sum "${dir}/${f}" | cut -d' ' -f1)"
  [ "${got}" = "${want}" ] || {
    warn "sha256 mismatch for ${f}"
    return 1
  }
}

# _artifact_env <dir> <KEY>: read one value from artifacts.env without executing it.
_artifact_env() { sed -n "s/^$2=//p" "$1/artifacts.env" | head -n1; }

# _artifact_compatible <dir> <stack-sha>: built for this commit, firmware pin, OS, arch and ROS distro.
_artifact_compatible() {
  local dir="$1" sha="$2" v
  v="$(_artifact_env "${dir}" ARTIFACT_STACK_SHA)"
  [ "${v}" = "${sha}" ] || {
    warn "artifact is for stack ${v:0:10}, not ${sha:0:10}"
    return 1
  }
  v="$(_artifact_env "${dir}" ARTIFACT_FIRMWARE_SHA)"
  [ "${v}" = "${FIRMWARE_SHA}" ] || {
    warn "artifact firmware pin ${v:0:10} != ${FIRMWARE_SHA:0:10}"
    return 1
  }
  v="$(_artifact_env "${dir}" ARTIFACT_ROS_DISTRO)"
  [ "${v}" = "${ROS_DISTRO_NAME}" ] || {
    warn "artifact ROS distro '${v}' != ${ROS_DISTRO_NAME}"
    return 1
  }
  [ "${DYX3_ALLOW_ANY_OS:-0}" = "1" ] && return 0
  local os_id os_ver
  os_id="$(. "${DYX3_OS_RELEASE:-/etc/os-release}" && printf '%s' "${ID}")"
  os_ver="$(. "${DYX3_OS_RELEASE:-/etc/os-release}" && printf '%s' "${VERSION_ID}")"
  [ "$(_artifact_env "${dir}" ARTIFACT_OS_ID)" = "${os_id}" ] &&
    [ "$(_artifact_env "${dir}" ARTIFACT_OS_VERSION_ID)" = "${os_ver}" ] &&
    [ "$(_artifact_env "${dir}" ARTIFACT_ARCH)" = "${DYX3_ARCH:-$(uname -m)}" ] || {
    warn "artifact built for $(_artifact_env "${dir}" ARTIFACT_OS_ID) $(_artifact_env "${dir}" ARTIFACT_OS_VERSION_ID) $(_artifact_env "${dir}" ARTIFACT_ARCH), not this machine"
    return 1
  }
}

# _artifact_extract <archive> <required-prefix>: every member must live under the prefix.
_artifact_extract() {
  local archive="$1" prefix="$2" bad
  bad="$(tar --zstd -tf "${archive}" | grep -v -e "^${prefix}/" -e "^${prefix}\$" | head -n1)" || true
  [ -z "${bad}" ] || {
    warn "$(basename "${archive}") has a member outside ${prefix}/: ${bad}"
    return 1
  }
  tar --zstd -xf "${archive}" -C "${DYX3_ROOT:-/}" --no-same-owner
}

# install_prebuilt <stack-sha>: fetch, verify and extract the release (and px4_msgs when this machine
# lacks it). Returns 1 when no valid artifact exists; leaves nothing half-extracted behind.
install_prebuilt() {
  local sha="$1" rel="${DYX3_RELEASES}/$1"
  [ "${DYX3_ARTIFACTS}" = "source" ] && return 1
  [ -f "${rel}/.complete" ] && return 0
  load_pin firmware
  local pm need_msgs=0
  pm="$(px4_msgs_dir)"
  [ -f "${pm}/.complete" ] || need_msgs=1
  local rel_f="release-${sha}.tar.zst" msgs_f="px4_msgs-${FIRMWARE_SHA}.tar.zst"
  if [ "${DYX3_DRY_RUN}" = "1" ]; then
    printf '[dry-run] fetch + verify %s (%s%s)\n' "$(artifact_tag "${sha}")" "${rel_f}" \
      "$([ "${need_msgs}" -eq 1 ] && printf ', %s' "${msgs_f}")" >&2
    return 1
  fi

  local dir="${DYX3_VAR_LIB}/state/artifacts-${sha}"
  rm -rf "${dir}" && mkdir -p "${dir}" || return 1
  local files=(SHA256SUMS artifacts.env "${rel_f}") f ok=1
  [ "${need_msgs}" -eq 1 ] && files+=("${msgs_f}")
  for f in "${files[@]}"; do
    _artifact_get "${sha}" "${dir}" "${f}" || {
      ok=0
      break
    }
  done
  if [ "${ok}" -eq 1 ]; then
    for f in "${files[@]:1}"; do _artifact_verify "${dir}" "${f}" || ok=0; done
  fi
  [ "${ok}" -eq 1 ] && { _artifact_compatible "${dir}" "${sha}" || ok=0; }
  if [ "${ok}" -ne 1 ]; then
    rm -rf "${dir}"
    return 1
  fi

  log "installing prebuilt $(artifact_tag "${sha}") ($(_artifact_env "${dir}" ARTIFACT_CI_RUN))"
  if [ "${need_msgs}" -eq 1 ]; then
    rm -rf "${pm}"
    _artifact_extract "${dir}/${msgs_f}" "opt/dyx3/px4_msgs/${FIRMWARE_SHA}" && [ -f "${pm}/.complete" ] || {
      rm -rf "${pm}" "${dir}"
      warn "px4_msgs artifact did not install"
      return 1
    }
  fi
  rm -rf "${rel}"
  _artifact_extract "${dir}/${rel_f}" "opt/dyx3/releases/${sha}" && [ -d "${rel}" ] || {
    rm -rf "${rel}" "${dir}"
    warn "release artifact did not install"
    return 1
  }
  rm -f "${rel}/.complete" "${rel}/.failed"
  cp "${dir}/artifacts.env" "${rel}/artifacts.env"
  touch "${rel}/.prebuilt"
  rm -rf "${dir}"
}
