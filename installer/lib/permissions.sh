#!/usr/bin/env bash
# DYX 3WD installer — service user, directories, config. Source, do not execute.
# shellcheck shell=bash

create_user() {
  if id "${DYX3_USER}" >/dev/null 2>&1; then
    log "user ${DYX3_USER} exists"
    return 0
  fi
  log "creating system user ${DYX3_USER}"
  run useradd --system --home-dir "${DYX3_VAR_LIB}" --shell /usr/sbin/nologin \
    --user-group "${DYX3_USER}"
  # dialout: GNSS/maintenance serial devices; no other group is needed for the platform.
  run usermod -aG dialout "${DYX3_USER}"
}

create_directories() {
  log "creating /opt/dyx3 /etc/dyx3 /var/lib/dyx3 /var/log/dyx3"
  install_dir 0755 root root "${DYX3_PREFIX}" "${DYX3_RELEASES}" "${DYX3_BIN}" \
    "${DYX3_PX4_MSGS_DIR}" "${DYX3_PREFIX}/third_party"
  # Secrets (ntrip.env, machine tokens) live here: root:dyx3 0640, never in Git (spec 12).
  install_dir 0750 root "${DYX3_GROUP}" "${DYX3_ETC}"
  local d
  for d in missions runs bags reports state; do
    install_dir 0750 "${DYX3_USER}" "${DYX3_GROUP}" "${DYX3_VAR_LIB}/${d}"
  done
  install_dir 0750 "${DYX3_USER}" "${DYX3_GROUP}" "${DYX3_VAR_LOG}"
}

# tmpfiles: /run is tmpfs, so /run/dyx3 must be recreated on every boot.
install_tmpfiles() {
  local dst="${DYX3_ROOT}/usr/lib/tmpfiles.d/dyx3.conf"
  log "installing tmpfiles.d entry for /run/dyx3"
  run install -d -m 0755 "$(dirname "${dst}")"
  if [ "${DYX3_DRY_RUN}" = "1" ]; then
    printf '[dry-run] write %s\n' "${dst}" >&2
    return 0
  fi
  printf 'd /run/dyx3 0775 %s %s -\n' "${DYX3_USER}" "${DYX3_GROUP}" >"${dst}"
  if have systemd-tmpfiles && [ -z "${DYX3_ROOT}" ]; then systemd-tmpfiles --create "${dst}"; fi
}

# install_config_templates <release-dir>: create /etc/dyx3 files ONLY if missing. A human may
# have edited them; an upgrade must never overwrite field configuration.
install_config_templates() {
  local rel="$1" f
  for f in platform.env mavlink-router.conf ros.env backend.env ntrip.env; do
    if [ -e "${DYX3_ETC}/${f}" ]; then
      log "keeping existing ${DYX3_ETC}/${f}"
    elif [ "${f}" = "ntrip.env" ]; then
      # Credentials: readable by root and the service group only.
      install_file 0640 "root:${DYX3_GROUP}" "${rel}/deployment/network/${f}.tmpl" "${DYX3_ETC}/${f}"
    else
      install_file 0644 "root:root" "${rel}/deployment/network/${f}.tmpl" "${DYX3_ETC}/${f}"
    fi
  done
}
