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

# _artifact_members_ok <archive> <required-prefix>: every member lives under the prefix and no member name has a
# ".." component (INS-020: "prefix/../../etc/x" passes a plain prefix test).
_artifact_members_ok() {
  local archive="$1" prefix="$2" list bad
  list="$(tar --zstd -tf "${archive}")" || {
    warn "$(basename "${archive}") cannot be listed"
    return 1
  }
  bad="$(printf '%s\n' "${list}" | grep -v -e "^${prefix}/" -e "^${prefix}\$" | head -n1)" || true
  [ -z "${bad}" ] || {
    warn "$(basename "${archive}") has a member outside ${prefix}/: ${bad}"
    return 1
  }
  bad="$(printf '%s\n' "${list}" | grep -E '(^|/)\.\.(/|$)' | head -n1)" || true
  [ -z "${bad}" ] || {
    warn "$(basename "${archive}") has a member with a '..' component: ${bad}"
    return 1
  }
}

# _artifact_links_ok <dir>: no symlink in the extracted tree points outside it (INS-020). Absolute targets are refused
# except the interpreter links python3 -m venv makes (venv/bin/python3 -> /usr/bin/python3[.N]).
_artifact_links_ok() {
  local dir="$1" l t rel
  while IFS= read -r -d '' l; do
    t="$(readlink "${l}")"
    case "${t}" in
      /*)
        [[ "${t}" =~ ^/usr/bin/python3(\.[0-9]+)?$ ]] && continue
        warn "artifact symlink ${l#"${dir}"/} -> ${t} points outside the release"
        return 1
        ;;
    esac
    rel="$(realpath -ms --relative-to="${dir}" "$(dirname "${l}")/${t}")"
    case "${rel}" in
      .. | ../*)
        warn "artifact symlink ${l#"${dir}"/} -> ${t} escapes the release"
        return 1
        ;;
    esac
  done < <(find "${dir}" -type l -print0)
}

# _artifact_extract_into <archive> <required-prefix> <dest>: the prefix's CONTENTS are extracted into <dest> (a fresh
# directory beside the final one), checked, and left for the caller to verify and rename into place.
_artifact_extract_into() {
  local archive="$1" prefix="$2" dest="$3" n
  _artifact_members_ok "${archive}" "${prefix}" || return 1
  n="$(printf '%s' "${prefix}" | awk -F/ '{print NF}')"
  rm -rf "${dest}"
  mkdir -p "${dest}"
  tar --zstd -xf "${archive}" -C "${dest}" --strip-components="${n}" --no-same-owner || return 1
  _artifact_links_ok "${dest}"
}

# _px4_msgs_tree_ok <dir>: what a usable px4_msgs overlay must contain, for the pinned firmware.
_px4_msgs_tree_ok() {
  [ -f "$1/install/setup.bash" ] && [ -s "$1/px4_msgs.sha256" ] || return 1
  [ ! -f "$1/firmware.sha" ] || [ "$(cat "$1/firmware.sha")" = "${FIRMWARE_SHA}" ]
}

# install_prebuilt <stack-sha>: fetch, verify and extract the release (and px4_msgs when this machine
# lacks it). Leaves nothing half-extracted behind. Returns
#   0  installed (or already built)
#   1  no usable artifact: not published, not in DYX3_ARTIFACT_DIR, built for another pin/OS, or source mode.
#      `auto` may build on this machine instead.
#   2  an artifact IS there but is invalid: digest mismatch, a file SHA256SUMS does not list, an unsafe or
#      broken archive. Never a reason to fall back: tampered must not look like missing (INS-012).
install_prebuilt() {
  local sha="$1" rel="${DYX3_RELEASES}/$1"
  [ "${DYX3_ARTIFACTS}" = "source" ] && return 1
  if [ -f "${rel}/.complete" ] || [ -f "${rel}/.verified" ]; then return 0; fi
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
  local files=(SHA256SUMS artifacts.env "${rel_f}") f
  [ "${need_msgs}" -eq 1 ] && files+=("${msgs_f}")
  for f in "${files[@]}"; do
    _artifact_get "${sha}" "${dir}" "${f}" || {
      rm -rf "${dir}"
      return 1
    }
  done
  for f in "${files[@]:1}"; do
    _artifact_verify "${dir}" "${f}" || {
      rm -rf "${dir}"
      warn "artifact ${f} for ${sha:0:10} is present but does not match SHA256SUMS"
      return 2
    }
  done
  _artifact_compatible "${dir}" "${sha}" || {
    rm -rf "${dir}"
    return 1
  }

  log "installing prebuilt $(artifact_tag "${sha}") ($(_artifact_env "${dir}" ARTIFACT_CI_RUN))"
  if [ "${need_msgs}" -eq 1 ]; then
    # INS-014: extract beside the final directory, verify, then one rename. The archive's own .complete is never
    # trusted: an interrupted extraction must not look complete, so the marker is written here after the checks.
    local inc="${pm}.incoming"
    if ! _artifact_extract_into "${dir}/${msgs_f}" "opt/dyx3/px4_msgs/${FIRMWARE_SHA}" "${inc}" ||
      ! rm -f "${inc}/.complete" || ! _px4_msgs_tree_ok "${inc}"; then
      rm -rf "${inc}" "${dir}"
      warn "px4_msgs artifact did not install (unsafe, broken or incomplete archive)"
      return 2
    fi
    sync_fs "${inc}"
    touch "${inc}/.complete"
    rm -rf "${pm}"
    mv -T "${inc}" "${pm}"
    sync_fs "${DYX3_PX4_MSGS_DIR}"
  fi
  local rinc="${DYX3_RELEASES}/.incoming-${sha}"
  _artifact_extract_into "${dir}/${rel_f}" "opt/dyx3/releases/${sha}" "${rinc}" || {
    rm -rf "${rinc}" "${dir}"
    warn "release artifact did not install (unsafe or broken archive)"
    return 2
  }
  rm -rf "${rel}"
  mv -T "${rinc}" "${rel}"
  rm -f "${rel}/.complete" "${rel}/.verified" "${rel}/.failed"
  cp "${dir}/artifacts.env" "${rel}/artifacts.env"
  touch "${rel}/.prebuilt"
  rm -rf "${dir}"
}
