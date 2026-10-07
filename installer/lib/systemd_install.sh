#!/usr/bin/env bash
# DYX 3WD installer — systemd units. Source, do not execute.
# shellcheck shell=bash
#
# Only services listed in the manifest's [enabled_services] are enabled. The other units
# are stubs that exit 1; enabling them would put the rover in a restart loop (CLAUDE.md §3b).

SYSTEMD_DIR="${DYX3_ROOT}/etc/systemd/system"

systemd_available() {
  [ "${DYX3_SKIP_SYSTEMD:-0}" != "1" ] && [ -z "${DYX3_ROOT}" ] && have systemctl
}

# install_units <release-dir>: copy every unit; enable the implemented ones; disable the rest.
install_units() {
  local rel="$1" unit name
  log "installing systemd units"
  run install -d -m 0755 "${SYSTEMD_DIR}"
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
    run systemctl restart "${name}.service"
  done < <(manifest_section enabled_services "${rel}/installer/manifests/production.manifest")
}
