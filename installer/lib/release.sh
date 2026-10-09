#!/usr/bin/env bash
# DYX 3WD installer — releases: fetch -> build -> verify -> switch -> restart -> health.
# Source, do not execute.
# shellcheck shell=bash
#
# Layout: /opt/dyx3/releases/<full-sha>/ is a `git archive` of that commit plus
#   ros2_ws/{build,install,log}  colcon products (manifest packages only)
#   bin/                         dyx3-<service> launchers copied from deployment/scripts
#   .complete                    written last; a release without it is never switched to
#   .verified                    built and statically verified; kept when the health gate fails, so a retry reuses it
# /opt/dyx3/current is a symlink to one release; switching is a single atomic rename.

DYX3_REPO_URL_DEFAULT="https://github.com/Vetri2425/DYX_3WD.git"

# Prebuilt artifacts (install_prebuilt): every caller of upgrade_to gets them through this file.
# shellcheck source=artifacts.sh
. "${INSTALLER_DIR}/lib/artifacts.sh"
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
  # A full SHA already in the mirror is immutable: no fetch (the re-executed target installer resolves its own SHA).
  if printf '%s' "${ref}" | grep -Eq '^[0-9a-f]{40}$' && git -C "${mirror}" rev-parse --verify --quiet "${ref}^{commit}" >/dev/null; then
    printf '%s' "${ref}"
    return 0
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

# installer_api_of <installer-dir>: the DYX3_INSTALLER_API that installer declares (0 = before the hand-over existed).
installer_api_of() {
  local v
  v="$(sed -n 's/^DYX3_INSTALLER_API=\([0-9][0-9]*\).*/\1/p' "$1/lib/common.sh" 2>/dev/null | head -n1)"
  printf '%s' "${v:-0}"
}

# stage_target_installer <ref>: fetch and resolve, then extract the TARGET commit's installer/ and deployment/ into a
# fresh staging directory. Prints "<sha> <dir>".
stage_target_installer() {
  local ref="$1" sha stage
  sha="$(resolve_ref "${ref}")" || die "cannot resolve '${ref}'"
  [ -n "${sha}" ] || die "cannot resolve '${ref}'"
  stage="${DYX3_VAR_LIB}/state/installer-stage"
  rm -rf "${stage}"
  mkdir -p "${stage}/${sha}"
  git -C "$(_mirror)" archive "${sha}" installer deployment | tar -x -C "${stage}/${sha}" ||
    die "cannot extract the installer of ${sha:0:10}"
  [ -f "${stage}/${sha}/installer/upgrade.sh" ] || die "release ${sha:0:10} has no installer/upgrade.sh"
  printf '%s %s\n' "${sha}" "${stage}/${sha}"
}

# handoff_to_target <ref> [upgrade.sh args...]: INS-002/INS-003. The release is installed by its OWN installer,
# libraries, units and pins, never by the running release's: a firmware-pin change then builds px4_msgs for the
# target's pin. Does not return (execs the target's upgrade.sh).
handoff_to_target() {
  local ref="$1" staged sha dir api
  shift
  # Gate here too: a target older than the interlock would not ask.
  require_rover_idle "upgrade to ${ref}"
  staged="$(with_lock "${DYX3_RUN}/install.lock" stage_target_installer "${ref}")" || exit 1
  sha="${staged%% *}"
  dir="${staged#* }"
  api="$(installer_api_of "${dir}/installer")"
  if [ "${api}" -lt "${DYX3_INSTALLER_API}" ]; then
    [ "${DYX3_FORCE:-0}" = "1" ] ||
      die "the installer of ${sha:0:10} is API ${api}, older than this one (API ${DYX3_INSTALLER_API}); it lacks protections added since. DYX3_FORCE=1 installs it anyway, with its own installer"
    warn "DYX3_FORCE=1: handing over to the OLDER installer (API ${api}) of ${sha:0:10}"
  fi
  log "handing over to the installer of ${sha:0:10} (API ${api}; this one is API ${DYX3_INSTALLER_API})"
  exec env DYX3_REEXEC=1 DYX3_TARGET_SHA="${sha}" DYX3_PARENT_API="${DYX3_INSTALLER_API}" \
    bash "${dir}/installer/upgrade.sh" "${sha}" "$@"
}

# reexec_contract <sha>: in a re-executed target installer, check what the parent handed over.
reexec_contract() {
  [ "$1" = "${DYX3_TARGET_SHA:-}" ] || die "re-executed for '${1}', but the parent resolved '${DYX3_TARGET_SHA:-}'"
  if [ "${DYX3_PARENT_API:-0}" -gt "${DYX3_INSTALLER_API}" ] && [ "${DYX3_FORCE:-0}" != "1" ]; then
    die "this installer (API ${DYX3_INSTALLER_API}) is older than the one that started it (API ${DYX3_PARENT_API}); DYX3_FORCE=1 to accept"
  fi
  log "installer of ${1:0:10} (API ${DYX3_INSTALLER_API}) takes over from API ${DYX3_PARENT_API:-0}"
}

# build_release <sha>: extract + colcon build + launchers. Idempotent per SHA.
build_release() {
  local sha="$1" rel="${DYX3_RELEASES}/$1"
  if [ -f "${rel}/.complete" ] || [ -f "${rel}/.verified" ]; then
    log "release ${sha:0:10} already built"
    return 0
  fi
  if [ -f "${rel}/.prebuilt" ]; then
    log "release ${sha:0:10} installed from CI artifacts; nothing to build"
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
      --parallel-workers ${DYX3_COLCON_WORKERS:-1} --packages-select ${packages} --cmake-args -DCMAKE_BUILD_TYPE=Release" ||
    die "colcon build failed for ${sha:0:10}"

  build_backend_venv "${rel}"

  # launchers: deployment/scripts/start-<svc>.sh -> bin/dyx3-<svc>
  run mkdir -p "${rel}/bin"
  local s name
  for s in "${rel}"/deployment/scripts/start-*.sh; do
    [ -e "${s}" ] || continue
    name="$(basename "${s}" .sh)"
    run install -m 0755 "${s}" "${rel}/bin/dyx3-${name#start-}"
  done
  # Every launcher sources this helper from its own directory (bin/); without it nothing but
  # dyx3-platform can start (found on the rover 2026-10-08).
  run install -m 0644 "${rel}/deployment/scripts/dyx3-env.sh" "${rel}/bin/dyx3-env.sh"
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
  for pair in "dyx3-install:install.sh" "dyx3-upgrade:upgrade.sh" "dyx3-health:verify.sh" "dyx3-rollback:rollback.sh" "dyx3-version:version.sh"; do
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

# ---- interruption (INS-004)
# DYX3 settings a detached run must see: systemd-run starts the unit with a clean environment.
DYX3_DETACH_ENV=(DYX3_FORCE DYX3_FORCE_UNSAFE DYX3_ARTIFACTS DYX3_ARTIFACT_DIR DYX3_REPO_URL DYX3_BUILD_JOBS
  DYX3_COLCON_WORKERS DYX3_SKIP_BACKEND DYX3_KEEP_RELEASES DYX3_HEALTH_SETTLE_S DYX3_HEALTH_HOLD_S
  FCU_IFACE FCU_KEEP_DHCP DYX3_UM982_USB_ID_PATH DYX3_UM982_USB_BAUD)

# detach_or_continue <kind> <script> [args...]: re-run <script> as a transient systemd unit, so a dropped ssh
# session (the access point coming back, a flaky link) cannot kill an upgrade or rollback half-way. Returns, to run
# attached, only when already detached, in a staged root (tests), in a dry run, with DYX3_NO_DETACH=1, or without
# systemd-run. Otherwise exits 0 once the unit has started.
detach_or_continue() {
  local kind="$1" script="$2" unit v
  shift 2
  if [ "${DYX3_DETACHED:-0}" = "1" ] || [ -n "${DYX3_ROOT}" ] || [ "${DYX3_DRY_RUN}" = "1" ]; then return 0; fi
  if [ "${DYX3_NO_DETACH:-0}" = "1" ] || ! have systemd-run; then
    warn "running attached (DYX3_NO_DETACH=1 or no systemd-run): a dropped session kills this ${kind}"
    return 0
  fi
  unit="dyx3-${kind}-$(date -u +%Y%m%dT%H%M%SZ)"
  local args=(--unit="${unit}" --collect --quiet --setenv=DYX3_DETACHED=1)
  for v in "${DYX3_DETACH_ENV[@]}"; do
    if [ -n "${!v+x}" ]; then args+=(--setenv="${v}=${!v}"); fi
  done
  systemd-run "${args[@]}" /bin/bash "$(readlink -f "${script}")" "$@" || die "could not start ${unit} with systemd-run"
  log "${kind} started in the background as ${unit}; it survives this session ending"
  log "follow it:  journalctl -fu ${unit}"
  log "it ends with '${kind} complete' or a FATAL line; dyx3-version shows what is current"
  exit 0
}

_switch_marker() { printf '%s/state/upgrade_in_progress' "${DYX3_VAR_LIB}"; }

# mark_switch_in_progress <kind> <target-sha> <previous-sha>: written before the switch, removed after the health
# result. Still present on the next run = that run was killed between the two.
mark_switch_in_progress() {
  [ "${DYX3_DRY_RUN}" = "1" ] && return 0
  local m
  m="$(_switch_marker)"
  install -d "$(dirname "${m}")"
  printf 'kind=%s\ntarget=%s\nprevious=%s\nstarted=%s\n' "$1" "$2" "$3" "$(date -u +%Y-%m-%dT%H:%M:%SZ)" >"${m}.tmp"
  mv -f "${m}.tmp" "${m}"
}

clear_switch_in_progress() { [ "${DYX3_DRY_RUN}" = "1" ] || rm -f "$(_switch_marker)"; }

# finish_interrupted_switch: report a switch whose run was killed. Keep the result when it is healthy now; otherwise
# go back to the release it replaced (and stop: the operator re-runs the command). DYX3_FORCE=1 keeps an unhealthy
# result and continues.
finish_interrupted_switch() {
  local m kind target previous started cur=""
  m="$(_switch_marker)"
  [ -f "${m}" ] || return 0
  kind="$(sed -n 's/^kind=//p' "${m}")"
  target="$(sed -n 's/^target=//p' "${m}")"
  previous="$(sed -n 's/^previous=//p' "${m}")"
  started="$(sed -n 's/^started=//p' "${m}")"
  [ -L "${DYX3_CURRENT}" ] && cur="$(basename "$(readlink -f "${DYX3_CURRENT}")")"
  warn "an interrupted ${kind:-switch} was found: ${previous:0:10} -> ${target:0:10}, started ${started}; current is ${cur:0:10}"
  if [ "${cur}" = "${previous}" ]; then
    log "the switch never happened; nothing to finish"
    clear_switch_in_progress
    return 0
  fi
  if [ -n "${cur}" ] && health_run "${DYX3_CURRENT}"; then
    log "current ${cur:0:10} is healthy: keeping it"
    clear_switch_in_progress
    return 0
  fi
  if [ "${DYX3_FORCE:-0}" = "1" ]; then
    warn "DYX3_FORCE=1: current ${cur:0:10} is unhealthy after the interrupted ${kind}; continuing without a revert"
    clear_switch_in_progress
    return 0
  fi
  [ -n "${previous}" ] && [ -f "${DYX3_RELEASES}/${previous}/.complete" ] ||
    die "current ${cur:0:10} is unhealthy after an interrupted ${kind} and ${previous:-no release} cannot be restored; DYX3_FORCE=1 to continue anyway"
  require_rover_idle "the revert of the interrupted ${kind}"
  warn "current ${cur:0:10} is unhealthy: reverting the interrupted ${kind} to ${previous:0:10}"
  atomic_symlink "${DYX3_RELEASES}/${previous}" "${DYX3_CURRENT}"
  if [ -n "${cur}" ]; then printf '%s\n' "${cur}" >"${DYX3_VAR_LIB}/state/previous_release"; fi
  install_units "${DYX3_CURRENT}"
  install_operator_shims
  write_versions_file "${DYX3_CURRENT}"
  restart_enabled_services "${DYX3_CURRENT}"
  local result=FAILED
  health_run "${DYX3_CURRENT}" && result=OK
  clear_switch_in_progress
  die "the interrupted ${kind} was reverted to ${previous:0:10} (health after the revert: ${result}); re-run the command"
}

# upgrade_to <git-ref>: stop-free build, verify, switch, restart, health; auto-revert on failure.
upgrade_to() {
  # An upgrade from the DDS-only release has no RTK runtime directory yet. The directory is
  # persistent vehicle state: release switches and rollback never copy or remove its contents.
  if [ -z "${DYX3_ROOT}" ]; then create_user; fi
  ensure_rtk_state_directory
  local ref="$1" sha prev=""
  # Fail fast, before a long fetch and build; asked again right before the switch (INS-001).
  require_rover_idle "upgrade to ${ref}"
  load_pin firmware
  sha="$(resolve_ref "${ref}")" || die "cannot resolve '${ref}'"
  [ -n "${sha}" ] || die "cannot resolve '${ref}'"
  log "upgrade: ${ref} = ${sha}"
  finish_interrupted_switch
  [ -L "${DYX3_CURRENT}" ] && prev="$(basename "$(readlink -f "${DYX3_CURRENT}")")"
  if [ "${prev}" = "${sha}" ] && [ "${DYX3_FORCE:-0}" != "1" ]; then
    log "already on ${sha:0:10}; nothing to do (DYX3_FORCE=1 to rebuild/restart)"
    return 0
  fi

  # Prebuilt first (proposal 2026-10-08): the same build, done once by green CI, digest-verified.
  if ! install_prebuilt "${sha}"; then
    [ "${DYX3_ARTIFACTS}" = "prebuilt" ] && die "no valid prebuilt artifacts for ${sha:0:10} (DYX3_ARTIFACTS=prebuilt)"
    log "building ${sha:0:10} on this machine (DYX3_ARTIFACTS=${DYX3_ARTIFACTS})"
  fi
  build_px4_msgs || die "px4_msgs build failed"
  build_release "${sha}" || die "release build failed"

  if [ "${DYX3_DRY_RUN}" = "1" ]; then
    log "dry-run: would verify, switch current -> ${sha:0:10}, install units, restart, health-check"
    return 0
  fi

  # Verify the built files before writing the marker that makes the release eligible.
  if ! (health_release_only "${DYX3_RELEASES}/${sha}" 0); then
    run rm -f "${DYX3_RELEASES}/${sha}/.complete" "${DYX3_RELEASES}/${sha}/.verified"
    die "release ${sha:0:10} failed verification; current is unchanged"
  fi
  run rm -f "${DYX3_RELEASES}/${sha}/.failed"
  # .verified outlives a failed health gate (INS-016): a retry of the same SHA reuses the build instead of an hour of
  # colcon. .complete (eligible to be switched to) does not.
  run touch "${DYX3_RELEASES}/${sha}/.verified" "${DYX3_RELEASES}/${sha}/.complete"

  # Baseline of the running release, so an existing fault is not blamed on the new one (INS-006).
  local base="${DYX3_VAR_LIB}/state/health_baseline" after="${DYX3_VAR_LIB}/state/health_after"
  rm -f "${base}"
  if [ -n "${prev}" ]; then
    log "health baseline: the running release ${prev:0:10}, before the switch"
    if health_capture "${DYX3_CURRENT}" "${base}"; then log "health baseline: OK"; else warn "health baseline: checks already failing before the switch (they will not cause a revert)"; fi
  fi

  # The build can take an hour: the rover may have started a mission meanwhile.
  require_rover_idle "the switch to ${sha:0:10}"
  mark_switch_in_progress upgrade "${sha}" "${prev}"
  switch_release "${sha}"
  install_units "${DYX3_CURRENT}"
  install_config_templates "${DYX3_CURRENT}"
  install_no_auto_updates
  install_operator_shims
  write_versions_file "${DYX3_CURRENT}"
  restart_enabled_services "${DYX3_CURRENT}"

  if health_judge "${DYX3_CURRENT}" "${base}" "${after}"; then
    clear_switch_in_progress
    # The access point last, and only once the release is healthy: re-activating it drops every Wi-Fi session,
    # including the ssh session an operator may be watching from (INS-004).
    install_hotspot_network
    prune_releases
    log "upgrade complete: ${sha:0:10} (health: ${HEALTH_RESULT})"
    return 0
  fi
  warn "release ${sha:0:10} failed post-switch health verification"
  run rm -f "${DYX3_RELEASES}/${sha}/.complete"
  run touch "${DYX3_RELEASES}/${sha}/.failed"
  if [ -n "${prev}" ] && [ -d "${DYX3_RELEASES}/${prev}" ]; then
    # Someone may have armed the new release already: a revert restarts the graph under them.
    rover_may_restart "the revert to ${prev:0:10}" ||
      die "release ${sha:0:10} failed health but the rover is not known idle (${ROVER_BUSY_REASON}); NOT reverting. When it is idle, the next dyx3-upgrade or dyx3-rollback reverts it"
    warn "post-switch health FAILED: reverting to ${prev:0:10}"
    stop_enabled_services "${DYX3_RELEASES}/${sha}"
    atomic_symlink "${DYX3_RELEASES}/${prev}" "${DYX3_CURRENT}"
    install_units "${DYX3_CURRENT}"
    install_operator_shims
    write_versions_file "${DYX3_CURRENT}"
    restart_enabled_services "${DYX3_CURRENT}"
    # Health again after the revert (INS-016): the operator must know where the rover was left.
    health_judge "${DYX3_CURRENT}" "${base}" "${DYX3_VAR_LIB}/state/health_after_revert" || true
    clear_switch_in_progress
    die "upgrade to ${sha:0:10} reverted to ${prev:0:10}; health of ${prev:0:10} after the revert: ${HEALTH_RESULT}"
  fi
  stop_enabled_services "${DYX3_RELEASES}/${sha}"
  disable_enabled_services "${DYX3_RELEASES}/${sha}"
  run rm -f "${DYX3_CURRENT}"
  run rm -f "${DYX3_ETC}/versions.json"
  clear_switch_in_progress
  die "health failed on first install of ${sha:0:10}; no previous release to revert to"
}

# health_release_only <release-dir>: static verification (no services).
health_release_only() {
  local pins
  pins="$(_pins_dir_of "$1")"
  local PINS_DIR="${pins}"
  _health_fail=0
  health_release "$1" "${2:-1}"
  return "${_health_fail}"
}

# build_backend_venv <release-dir>: the backend's Python environment, inside the release (so a rollback restores it
# with the code). NOT fatal: a failed pip install (no WAN) must not block the control graph's upgrade; it leaves the
# backend unavailable, which dyx3-health reports once dyx3-backend is enabled. Skipped under DYX3_SKIP_BACKEND=1.
build_backend_venv() {
  local rel="$1"
  [ "${DYX3_SKIP_BACKEND:-0}" = "1" ] && {
    log "backend venv skipped (DYX3_SKIP_BACKEND=1)"
    return 0
  }
  [ -f "${rel}/backend/pyproject.toml" ] || [ "${DYX3_DRY_RUN}" = "1" ] || {
    warn "no backend/ in release; backend venv not built"
    return 0
  }
  log "building backend venv"
  if ! run python3 -m venv "${rel}/venv" || ! run "${rel}/venv/bin/pip" install --quiet "${rel}/backend[path-engine]"; then
    warn "backend venv build FAILED (no network?); the backend will be unavailable until it is built"
    return 0
  fi
}

# write_versions_file <release-dir>: /etc/dyx3/versions.json — the provenance the recorder copies into every run.
# Machine-written at every switch/rollback, so it always describes what is CURRENT.
write_versions_file() {
  local rel="$1" stack pm msgs origin pins
  pins="$(_pins_dir_of "${rel}")"
  local PINS_DIR="${pins}"
  stack="$(basename "$(readlink -f "${rel}")")"
  origin="built on this machine"
  if [ -f "${rel}/artifacts.env" ]; then
    origin="prebuilt $(sed -n 's/^ARTIFACT_CI_RUN=//p' "${rel}/artifacts.env" | head -n1)"
  fi
  load_pin firmware
  pm="$(px4_msgs_dir)"
  msgs="$(cat "${pm}/px4_msgs.sha256" 2>/dev/null || echo unknown)"
  if [ "${DYX3_DRY_RUN}" = "1" ]; then
    printf '[dry-run] write %s/versions.json\n' "${DYX3_ETC}" >&2
    return 0
  fi
  install -d "${DYX3_ETC}"
  printf '{\n  "schema": 1,\n  "stack_sha": "%s",\n  "firmware_expected_sha": "%s",\n  "px4_msgs_msg_set_sha256": "%s",\n  "firmware_running": "unavailable: no FCU read path in this stack yet",\n  "build_origin": "%s",\n  "installed_utc": "%s"\n}\n' \
    "${stack}" "${FIRMWARE_SHA}" "${msgs}" "${origin}" "$(date -u +%Y-%m-%dT%H:%M:%SZ)" >"${DYX3_ETC}/versions.json.tmp"
  chmod 0644 "${DYX3_ETC}/versions.json.tmp"
  mv "${DYX3_ETC}/versions.json.tmp" "${DYX3_ETC}/versions.json"
}

# print_version: dyx3-version. The firmware identity is the PINNED expectation; the identity the running FCU reports needs a
# parameter/version read path that does not exist yet (architecture 3.8 wants the overlay hash: OPEN).
print_version() {
  local cur="" pm msgs
  [ -L "${DYX3_CURRENT}" ] && cur="$(basename "$(readlink -f "${DYX3_CURRENT}")")"
  load_pin firmware
  pm="$(px4_msgs_dir)"
  msgs="$(cat "${pm}/px4_msgs.sha256" 2>/dev/null || echo unknown)"
  printf 'stack_sha=%s\n' "${cur:-none}"
  printf 'previous_release=%s\n' "$(cat "${DYX3_VAR_LIB}/state/previous_release" 2>/dev/null || echo none)"
  printf 'firmware_expected_sha=%s\n' "${FIRMWARE_SHA}"
  printf 'firmware_branch=%s\n' "${FIRMWARE_BRANCH}"
  printf 'firmware_running=unavailable (no FCU read path yet)\n'
  printf 'px4_msgs_msg_set_sha256=%s\n' "${msgs}"
  printf 'px4_msgs_skeleton_ref=%s\n' "${PX4_MSGS_SKELETON_REF}"
  printf 'profile=%s\n' "$(cat "${DYX3_ETC}/profile" 2>/dev/null || echo unset)"
}

# rollback_release: switch back to the release recorded by the last switch, verify, restart, health; revert if unhealthy.
# After a rollback `previous_release` is the release we rolled away from, so a second rollback undoes the first.
rollback_release() {
  local cur="" prev=""
  finish_interrupted_switch
  [ -L "${DYX3_CURRENT}" ] && cur="$(basename "$(readlink -f "${DYX3_CURRENT}")")"
  prev="$(cat "${DYX3_VAR_LIB}/state/previous_release" 2>/dev/null || true)"
  [ -n "${prev}" ] || die "no previous release is recorded; nothing to roll back to"
  [ "${prev}" != "${cur}" ] || die "previous release equals current (${cur:0:10})"
  [ -d "${DYX3_RELEASES}/${prev}" ] && [ -f "${DYX3_RELEASES}/${prev}/.complete" ] ||
    die "previous release ${prev:0:10} is missing or incomplete (pruned?)"
  load_pin firmware
  health_release_only "${DYX3_RELEASES}/${prev}" || die "previous release ${prev:0:10} failed verification; current is unchanged"
  local base="${DYX3_VAR_LIB}/state/health_baseline" after="${DYX3_VAR_LIB}/state/health_after"
  rm -f "${base}"
  if [ -n "${cur}" ]; then
    log "health baseline: the running release ${cur:0:10}, before the rollback"
    if health_capture "${DYX3_CURRENT}" "${base}"; then log "health baseline: OK"; else warn "health baseline: checks already failing before the rollback (they will not cause a restore)"; fi
  fi
  require_rover_idle "rollback to ${prev:0:10}"

  log "rollback: ${cur:0:10} -> ${prev:0:10}"
  mark_switch_in_progress rollback "${prev}" "${cur}"
  atomic_symlink "${DYX3_RELEASES}/${prev}" "${DYX3_CURRENT}"
  printf '%s\n' "${cur}" >"${DYX3_VAR_LIB}/state/previous_release"
  install_units "${DYX3_CURRENT}"
  install_operator_shims
  write_versions_file "${DYX3_CURRENT}"
  restart_enabled_services "${DYX3_CURRENT}"
  if health_judge "${DYX3_CURRENT}" "${base}" "${after}"; then
    clear_switch_in_progress
    log "rollback complete: ${prev:0:10} (health: ${HEALTH_RESULT})"
    return 0
  fi
  rover_may_restart "restoring ${cur:0:10}" ||
    die "rollback to ${prev:0:10} failed health but the rover is not known idle (${ROVER_BUSY_REASON}); NOT restoring ${cur:0:10}. When it is idle, the next dyx3-upgrade or dyx3-rollback restores it"
  warn "post-rollback health FAILED: restoring ${cur:0:10}"
  atomic_symlink "${DYX3_RELEASES}/${cur}" "${DYX3_CURRENT}"
  printf '%s\n' "${prev}" >"${DYX3_VAR_LIB}/state/previous_release"
  install_units "${DYX3_CURRENT}"
  install_operator_shims
  write_versions_file "${DYX3_CURRENT}"
  restart_enabled_services "${DYX3_CURRENT}"
  health_judge "${DYX3_CURRENT}" "${base}" "${DYX3_VAR_LIB}/state/health_after_revert" || true
  clear_switch_in_progress
  die "rollback to ${prev:0:10} was unhealthy; restored ${cur:0:10}; health of ${cur:0:10} after the restore: ${HEALTH_RESULT}"
}
