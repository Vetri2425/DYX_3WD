#!/usr/bin/env bash
# DYX 3WD installer — systemd units. Source, do not execute.
# shellcheck shell=bash
#
# Only services listed in the manifest's [enabled_services] are enabled. The other units
# are stubs that exit 1; enabling them would put the rover in a restart loop (CLAUDE.md §3b).

SYSTEMD_DIR="${DYX3_ROOT}/etc/systemd/system"

# dyx3-* units under /etc/systemd/system that no release ships: the network code writes them, and a release switch
# must leave them alone. The detached upgrade/rollback units are transient (/run), listed for safety.
_non_release_unit() {
  case "$1" in
    dyx3-wifi-regdom.service | dyx3-upgrade-*.service | dyx3-rollback-*.service) return 0 ;;
  esac
  return 1
}

systemd_available() {
  [ "${DYX3_SKIP_SYSTEMD:-0}" != "1" ] && [ -z "${DYX3_ROOT}" ] && have systemctl
}

# _release_units <release-dir>: the unit file names this release installs, one per line: a file in deployment/systemd/ that the
# release's manifest also lists in [services]. A unit the manifest no longer lists (dyx3-ros.service after the 2026-10-10 split
# into dyx3-control + dyx3-services) is therefore not installed and, when an older release left it behind, removed as stale.
# DERIVED — NOT FROM V1 SPEC: an empty or unreadable [services] falls back to every shipped file, so a manifest that cannot be
# read never turns into "remove every unit".
_release_units() {
  local rel="$1" listed unit name
  listed="$(manifest_section services "${rel}/installer/manifests/production.manifest" 2>/dev/null || true)"
  [ -n "${listed}" ] || warn "no [services] in ${rel}/installer/manifests/production.manifest: installing every shipped unit"
  for unit in "${rel}"/deployment/systemd/*.service; do
    [ -e "${unit}" ] || continue
    name="$(basename "${unit}")"
    if [ -z "${listed}" ] || printf '%s\n' "${listed}" | grep -qx "${name%.service}"; then printf '%s\n' "${name}"; fi
  done
}

# _remove_stale_unit <unit-file-name>: stop, disable and delete a dyx3 unit the release does not install. Idempotent: every step
# tolerates a unit that is already stopped, disabled or gone, so a re-run (or a run after an interrupted one) converges.
_remove_stale_unit() {
  local name="$1"
  log "removing ${name}: this release does not install it"
  if systemd_available; then
    run systemctl stop "${name}" || warn "could not stop ${name}"
    run systemctl disable "${name}" 2>/dev/null || true
    # A unit that failed before it was removed would otherwise stay listed as failed until the next boot.
    run systemctl reset-failed "${name}" 2>/dev/null || true
  fi
  run rm -f "${SYSTEMD_DIR}/${name}"
  run find "${SYSTEMD_DIR}" -mindepth 2 -maxdepth 2 -path '*.wants/*' -name "${name}" -delete
  # A drop-in directory is rover-local configuration (never shipped by a release): left in place, inert without its unit.
  [ -d "${SYSTEMD_DIR}/${name}.d" ] && warn "${SYSTEMD_DIR}/${name}.d is left in place (inert without ${name}); remove it by hand if unused"
  return 0
}

# install_units <release-dir>: copy the release's units; enable the implemented ones; disable the rest; remove stale ones.
install_units() {
  local rel="$1" unit name units
  log "installing systemd units"
  run install -d -m 0755 "${SYSTEMD_DIR}"
  units="$(_release_units "${rel}")"
  # INS-011: a dyx3 unit this release does not install (rollback to an older release, a renamed or split service such as
  # dyx3-ros -> dyx3-control + dyx3-services) is stopped, disabled and removed, so it cannot keep running another release's
  # code or be restarted at boot. The enablement links are swept too, also when the unit file itself is already gone (a run
  # interrupted between the two), so a re-run converges.
  local -A stale=()
  for unit in "${SYSTEMD_DIR}"/dyx3-*.service "${SYSTEMD_DIR}"/*.wants/dyx3-*.service; do
    [ -e "${unit}" ] || [ -L "${unit}" ] || continue
    name="$(basename "${unit}")"
    printf '%s\n' "${units}" | grep -qxF "${name}" && continue
    _non_release_unit "${name}" && continue
    stale[${name}]=1
  done
  for name in "${!stale[@]}"; do _remove_stale_unit "${name}"; done
  while IFS= read -r name; do
    [ -n "${name}" ] || continue
    run install -m 0644 "${rel}/deployment/systemd/${name}" "${SYSTEMD_DIR}/${name}"
  done <<<"${units}"
  if ! systemd_available; then
    warn "systemd not available (or staged root): units copied, not enabled"
    return 0
  fi
  run systemctl daemon-reload
  local enabled
  enabled="$(manifest_section enabled_services "${rel}/installer/manifests/production.manifest")"
  while IFS= read -r name; do
    [ -n "${name}" ] || continue
    name="${name%.service}"
    if printf '%s\n' "${enabled}" | grep -qx "${name}"; then
      run systemctl enable "${name}.service"
    else
      # Idempotent: ignore "not enabled".
      run systemctl disable "${name}.service" 2>/dev/null || true
    fi
  done <<<"${units}"
}

# restart_enabled_services <release-dir>: in [enabled_services] order, which is the dependency order: dyx3-platform, then
# dyx3-control (Requires=dyx3-platform), then dyx3-services (Wants=dyx3-control), ... dyx3-backend last (Wants=dyx3-services).
restart_enabled_services() {
  local rel="$1" name
  systemd_available || {
    warn "systemd not available: not restarting services"
    return 0
  }
  while IFS= read -r name; do
    [ -n "${name}" ] || continue
    log "restarting ${name}"
    # Never abort here: a unit that fails to (re)start must still let every other unit restart and
    # the caller reach the health check, which fails on the inactive unit and reverts the release.
    # Aborting (set -e) left `current` switched with old processes running and no rollback (rover
    # 2026-10-09, dyx3-usb-serial-check).
    run systemctl restart "${name}.service" || warn "${name} failed to restart; health check will decide"
  done < <(manifest_section enabled_services "${rel}/installer/manifests/production.manifest")
}

# Stop services started by an unsuccessful first installation before removing current. Reverse [enabled_services] order:
# the consumers (backend, services) stop before what they depend on (dyx3-control, dyx3-platform).
stop_enabled_services() {
  local rel="$1" name
  systemd_available || return 0
  while IFS= read -r name; do
    [ -n "${name}" ] || continue
    log "stopping ${name}"
    run systemctl stop "${name}.service" || warn "could not stop ${name}.service after failed activation"
  done < <(manifest_section enabled_services "${rel}/installer/manifests/production.manifest" | tac)
}

disable_enabled_services() {
  local rel="$1" name
  systemd_available || return 0
  while IFS= read -r name; do
    [ -n "${name}" ] || continue
    run systemctl disable "${name}.service" || warn "could not disable ${name}.service after failed first install"
  done < <(manifest_section enabled_services "${rel}/installer/manifests/production.manifest")
}
