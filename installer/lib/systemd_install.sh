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

# install_units <release-dir>: copy every unit; enable the implemented ones; disable the rest.
install_units() {
  local rel="$1" unit name
  log "installing systemd units"
  run install -d -m 0755 "${SYSTEMD_DIR}"
  # INS-011: a dyx3 unit this release does not ship (rollback to an older release, a renamed service) is stopped,
  # disabled and removed, so it cannot keep running another release's code or be restarted at boot.
  for unit in "${SYSTEMD_DIR}"/dyx3-*.service; do
    [ -e "${unit}" ] || continue
    name="$(basename "${unit}")"
    [ -e "${rel}/deployment/systemd/${name}" ] && continue
    _non_release_unit "${name}" && continue
    log "removing ${name}: this release does not ship it"
    if systemd_available; then
      run systemctl stop "${name}" || warn "could not stop ${name}"
      run systemctl disable "${name}" 2>/dev/null || true
    fi
    run rm -f "${unit}"
    run find "${SYSTEMD_DIR}" -mindepth 2 -maxdepth 2 -path '*.wants/*' -name "${name}" -delete
  done
  for unit in "${rel}"/deployment/systemd/*.service; do
    run install -m 0644 "${unit}" "${SYSTEMD_DIR}/$(basename "${unit}")"
  done
  if ! systemd_available; then
    warn "systemd not available (or staged root): units copied, not enabled"
    return 0
  fi
  run systemctl daemon-reload
  local enabled
  enabled="$(manifest_section enabled_services "${rel}/installer/manifests/production.manifest")"
  for unit in "${rel}"/deployment/systemd/*.service; do
    name="$(basename "${unit}" .service)"
    if printf '%s\n' "${enabled}" | grep -qx "${name}"; then
      run systemctl enable "${name}.service"
    else
      # Idempotent: ignore "not enabled".
      run systemctl disable "${name}.service" 2>/dev/null || true
    fi
  done
}

# restart_enabled_services <release-dir>
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

# Stop services started by an unsuccessful first installation before removing current.
stop_enabled_services() {
  local rel="$1" name
  systemd_available || return 0
  while IFS= read -r name; do
    [ -n "${name}" ] || continue
    log "stopping ${name}"
    run systemctl stop "${name}.service" || warn "could not stop ${name}.service after failed activation"
  done < <(manifest_section enabled_services "${rel}/installer/manifests/production.manifest")
}

disable_enabled_services() {
  local rel="$1" name
  systemd_available || return 0
  while IFS= read -r name; do
    [ -n "${name}" ] || continue
    run systemctl disable "${name}.service" || warn "could not disable ${name}.service after failed first install"
  done < <(manifest_section enabled_services "${rel}/installer/manifests/production.manifest")
}
