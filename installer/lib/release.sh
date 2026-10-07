#!/usr/bin/env bash
# DYX 3WD installer — releases: fetch -> build -> verify -> switch -> restart -> health.
# Source, do not execute.
# shellcheck shell=bash
#
# Layout: /opt/dyx3/releases/<full-sha>/ is a `git archive` of that commit plus
#   ros2_ws/{build,install,log}  colcon products (manifest packages only)
#   bin/                         dyx3-<service> launchers copied from deployment/scripts
#   .complete                    written last; a release without it is never switched to
# /opt/dyx3/current is a symlink to one release; switching is a single atomic rename.

DYX3_REPO_URL_DEFAULT="https://github.com/Vetri2425/DYX_3WD.git"
DYX3_KEEP_RELEASES="${DYX3_KEEP_RELEASES:-5}"

_repo_url() {
  local url="${DYX3_REPO_URL:-}"
  if [ -z "${url}" ] && [ -r "${DYX3_ETC}/upgrade.env" ]; then
    # shellcheck disable=SC1091
    url="$(. "${DYX3_ETC}/upgrade.env" && printf '%s' "${DYX3_REPO_URL:-}")"
  fi
  printf '%s' "${url:-${DYX3_REPO_URL_DEFAULT}}"
}

_mirror() { printf '%s/state/repo.git' "${DYX3_VAR_LIB}"; }

# resolve_ref <git-ref>: fetch into the bare mirror and print the full commit SHA.
resolve_ref() {
  local ref="$1" mirror sha
  mirror="$(_mirror)"
  if [ "${DYX3_DRY_RUN}" = "1" ]; then
    printf '%s' "${ref}"
    return 0
  fi
  if [ ! -d "${mirror}" ]; then
    run git init -q --bare "${mirror}"
    run git -C "${mirror}" remote add origin "$(_repo_url)"
  fi
  run git -C "${mirror}" fetch -q --prune origin \
    '+refs/heads/*:refs/remotes/origin/*' '+refs/tags/*:refs/tags/*'
  local cand
  for cand in "${ref}" "origin/${ref}" "refs/tags/${ref}"; do
    if sha="$(git -C "${mirror}" rev-parse --verify --quiet "${cand}^{commit}")"; then
      printf '%s' "${sha}"
      return 0
    fi
  done
  # A bare commit SHA that is not reachable from a branch/tag.
  if printf '%s' "${ref}" | grep -Eq '^[0-9a-f]{40}$'; then
    run git -C "${mirror}" fetch -q origin "${ref}"
    git -C "${mirror}" rev-parse --verify --quiet "${ref}^{commit}" && return 0
  fi
  die "cannot resolve git ref '${ref}' in $(_repo_url)"
}

# build_release <sha>: extract + colcon build + launchers. Idempotent per SHA.
build_release() {
  local sha="$1" rel="${DYX3_RELEASES}/$1"
  if [ -f "${rel}/.complete" ]; then
    log "release ${sha:0:10} already built"
    return 0
  fi
  log "extracting ${sha:0:10}"
  # Built IN PLACE (no relocation): colcon embeds absolute paths. The release only becomes
  # eligible once `.complete` is written, after the build and verification both succeed.
  run rm -rf "${rel}"
  run mkdir -p "${rel}"
  if [ "${DYX3_DRY_RUN}" = "1" ]; then
    printf '[dry-run] git archive %s | tar -x -C %s\n' "${sha}" "${rel}" >&2
  else
    git -C "$(_mirror)" archive "${sha}" | tar -x -C "${rel}"
  fi

  local pm packages
  pm="$(px4_msgs_dir)"
  local manifest="${rel}/installer/manifests/production.manifest"
  [ "${DYX3_DRY_RUN}" = "1" ] && manifest="${INSTALLER_DIR}/manifests/production.manifest"
  packages="$(manifest_section ros2_packages "${manifest}" | tr '\n' ' ')"
  [ -n "${packages}" ] || [ "${DYX3_DRY_RUN}" = "1" ] || die "manifest lists no ros2_packages"
  log "colcon build (${packages}) with -j${DYX3_BUILD_JOBS}"
  # A failed build must never reach `.complete`: die here, leaving an incomplete (ignored) dir.
  run bash -c "set +u; . '${ROS_SETUP}'; . '${pm}/install/setup.bash'; set -u; cd '${rel}/ros2_ws' && \
    MAKEFLAGS='-j${DYX3_BUILD_JOBS}' nice -n 10 colcon build \
      --parallel-workers 1 --packages-select ${packages} --cmake-args -DCMAKE_BUILD_TYPE=Release" ||
    die "colcon build failed for ${sha:0:10}"

  # launchers: deployment/scripts/start-<svc>.sh -> bin/dyx3-<svc>
  run mkdir -p "${rel}/bin"
  local s name
  for s in "${rel}"/deployment/scripts/start-*.sh; do
    [ -e "${s}" ] || continue
    name="$(basename "${s}" .sh)"
    run install -m 0755 "${s}" "${rel}/bin/dyx3-${name#start-}"
  done
  run touch "${rel}/.complete"
}

# switch_release <sha>: atomically point current at it; remember the previous one.
switch_release() {
  local sha="$1" prev=""
  [ -L "${DYX3_CURRENT}" ] && prev="$(basename "$(readlink -f "${DYX3_CURRENT}")")"
  if [ -n "${prev}" ] && [ "${prev}" != "${sha}" ]; then
    run install -d "${DYX3_VAR_LIB}/state"
    if [ "${DYX3_DRY_RUN}" != "1" ]; then printf '%s\n' "${prev}" >"${DYX3_VAR_LIB}/state/previous_release"; fi
  fi
  atomic_symlink "${DYX3_RELEASES}/${sha}" "${DYX3_CURRENT}"
  log "current -> ${sha:0:10}${prev:+ (previous ${prev:0:10})}"
}

# install_operator_shims: /opt/dyx3/bin/dyx3-* exec into the CURRENT release's scripts, so a
# release is always operated by its own tooling and the shims never go stale.
install_operator_shims() {
  local pair name target
  for pair in "dyx3-install:install.sh" "dyx3-upgrade:upgrade.sh" "dyx3-health:verify.sh"; do
    name="${pair%%:*}"
    target="${pair##*:}"
    if [ "${DYX3_DRY_RUN}" = "1" ]; then
      printf '[dry-run] shim %s -> current/installer/%s\n' "${name}" "${target}" >&2
      continue
    fi
    cat >"${DYX3_BIN}/${name}" <<SHIM
#!/usr/bin/env bash
exec "${DYX3_CURRENT}/installer/${target}" "\$@"
SHIM
    chmod 0755 "${DYX3_BIN}/${name}"
  done
}

prune_releases() {
  local keep="${DYX3_KEEP_RELEASES}" cur prev
  cur="$(basename "$(readlink -f "${DYX3_CURRENT}" 2>/dev/null)" 2>/dev/null || true)"
  prev="$(cat "${DYX3_VAR_LIB}/state/previous_release" 2>/dev/null || true)"
  local n=0 d b
  # newest first
  while IFS= read -r d; do
    b="$(basename "${d}")"
    n=$((n + 1))
    [ "${n}" -le "${keep}" ] && continue
    if [ "${b}" = "${cur}" ] || [ "${b}" = "${prev}" ]; then continue; fi
    log "pruning old release ${b:0:10}"
    run rm -rf "${d}"
  done < <(find "${DYX3_RELEASES}" -mindepth 1 -maxdepth 1 -type d -name '[0-9a-f]*' -printf '%T@ %p\n' 2>/dev/null | sort -rn | cut -d' ' -f2-)
}

# upgrade_to <git-ref>: stop-free build, verify, switch, restart, health; auto-revert on failure.
upgrade_to() {
  local ref="$1" sha prev=""
  load_pin firmware
  sha="$(resolve_ref "${ref}")" || die "cannot resolve '${ref}'"
  [ -n "${sha}" ] || die "cannot resolve '${ref}'"
  log "upgrade: ${ref} = ${sha}"
  [ -L "${DYX3_CURRENT}" ] && prev="$(basename "$(readlink -f "${DYX3_CURRENT}")")"
  if [ "${prev}" = "${sha}" ] && [ "${DYX3_FORCE:-0}" != "1" ]; then
    log "already on ${sha:0:10}; nothing to do (DYX3_FORCE=1 to rebuild/restart)"
    return 0
  fi

  build_px4_msgs || die "px4_msgs build failed"
  build_release "${sha}" || die "release build failed"

  if [ "${DYX3_DRY_RUN}" = "1" ]; then
    log "dry-run: would verify, switch current -> ${sha:0:10}, install units, restart, health-check"
    return 0
  fi

  # Verify BEFORE switching: a bad build never becomes current.
  if ! (health_release_only "${DYX3_RELEASES}/${sha}"); then
    die "release ${sha:0:10} failed verification; current is unchanged"
  fi

  switch_release "${sha}"
  install_units "${DYX3_CURRENT}"
  install_config_templates "${DYX3_CURRENT}"
  install_operator_shims
  restart_enabled_services "${DYX3_CURRENT}"

  if health_run "${DYX3_CURRENT}"; then
    prune_releases
    log "upgrade complete: ${sha:0:10}"
    return 0
  fi
  if [ -n "${prev}" ] && [ -d "${DYX3_RELEASES}/${prev}" ]; then
    warn "post-switch health FAILED: reverting to ${prev:0:10}"
    atomic_symlink "${DYX3_RELEASES}/${prev}" "${DYX3_CURRENT}"
    install_units "${DYX3_CURRENT}"
    restart_enabled_services "${DYX3_CURRENT}"
    die "upgrade to ${sha:0:10} reverted to ${prev:0:10}"
  fi
  die "health failed on first install of ${sha:0:10}; no previous release to revert to"
}

# health_release_only <release-dir>: static verification (no services).
health_release_only() {
  _health_fail=0
  health_release "$1"
  return "${_health_fail}"
}
