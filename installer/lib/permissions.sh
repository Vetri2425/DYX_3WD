#!/usr/bin/env bash
# DYX 3WD installer — service user, directories, config. Source, do not execute.
# shellcheck shell=bash

create_user() {
  if id "${DYX3_USER}" >/dev/null 2>&1; then
    log "user ${DYX3_USER} exists"
  else
    log "creating system user ${DYX3_USER}"
    run useradd --system --home-dir "${DYX3_VAR_LIB}" --shell /usr/sbin/nologin \
      --user-group "${DYX3_USER}"
  fi
  # Also run on upgrades: an existing dyx3 user may not yet be in dialout.
  run usermod -aG dialout "${DYX3_USER}"
}

ensure_rtk_state_directory() {
  local path="${DYX3_VAR_LIB}/rtk"
  [ ! -L "${path}" ] || die "RTK state directory must not be a symlink"
  if [ ! -d "${path}" ]; then
    install_dir 0700 "${DYX3_USER}" "${DYX3_GROUP}" "${path}"
  else
    local mode
    mode="$(stat -c %a "${path}" 2>/dev/null || stat -f %Lp "${path}")"
    [ "${mode}" = 700 ] || die "RTK state directory must be mode 0700"
    if [ -z "${DYX3_ROOT}" ]; then
      [ "$(stat -c %U:%G "${path}")" = "${DYX3_USER}:${DYX3_GROUP}" ] ||
        die "RTK state directory must be owned by ${DYX3_USER}:${DYX3_GROUP}"
    fi
  fi
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
  ensure_rtk_state_directory
  local token_state="${DYX3_VAR_LIB}/state/px4_link_spray_ack_next"
  if [ ! -e "${token_state}" ]; then
    if [ -L "${DYX3_CURRENT}" ]; then
      warn "spray ACK token state is missing on an installed rover; it will fail closed until PX4 and the companion are reset together"
    elif [ "${DYX3_DRY_RUN:-0}" = "1" ]; then
      log "would initialize spray ACK token state for a first install"
    else
      local token_tmp="${token_state}.tmp.$$"
      printf '2\n' >"${token_tmp}"
      if [ -z "${DYX3_ROOT}" ]; then chown "${DYX3_USER}:${DYX3_GROUP}" "${token_tmp}"; fi
      chmod 0600 "${token_tmp}"
      mv "${token_tmp}" "${token_state}"
    fi
  fi
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
  for f in platform.env mavlink-router.conf ros.env backend.env ntrip.env hotspot.env network.env; do
    if [ -e "${DYX3_ETC}/${f}" ]; then
      log "keeping existing ${DYX3_ETC}/${f}"
    elif [ "${f}" = "ntrip.env" ] || [ "${f}" = "hotspot.env" ]; then
      # Credentials: readable by root and the service group only.
      install_file 0640 "root:${DYX3_GROUP}" "${rel}/deployment/network/${f}.tmpl" "${DYX3_ETC}/${f}"
    else
      install_file 0644 "root:root" "${rel}/deployment/network/${f}.tmpl" "${DYX3_ETC}/${f}"
    fi
  done
  # The Fast DDS profile file is not a *.tmpl (it is used as shipped) but follows the same rule: created when missing, never overwritten.
  # dyx3-env.sh exports it as FASTRTPS_DEFAULT_PROFILES_FILE when it exists.
  if [ -e "${DYX3_ETC}/fastdds_profiles.xml" ]; then
    log "keeping existing ${DYX3_ETC}/fastdds_profiles.xml"
  else
    install_file 0644 "root:root" "${rel}/deployment/network/fastdds_profiles.xml" "${DYX3_ETC}/fastdds_profiles.xml"
  fi
}

# _env_value <file> <KEY>: the last uncommented KEY=value of an env file (empty when absent). Never executed.
_env_value() { sed -n "s/^$2=//p" "$1" 2>/dev/null | tail -n1; }

# _preflight_file <src> <name>: /etc/dyx3/<name>, or the template it will be created from.
_preflight_file() {
  if [ -e "${DYX3_ETC}/$2" ]; then printf '%s' "${DYX3_ETC}/$2"; else printf '%s' "$1/deployment/network/$2.tmpl"; fi
}

# preflight_rover_inputs <checkout-or-release-dir>: INS-007. The per-rover inputs an install cannot guess. Without them
# the release cannot pass its own health gate and reverts with no useful message. Stops with the list and the files to
# edit, before anything is built.
preflight_rover_inputs() {
  local src="$1" missing=() m domain host ssid psk addr
  domain="$(_env_value "$(_preflight_file "${src}" ros.env)" ROS_DOMAIN_ID)"
  if [[ ! "${domain}" =~ ^([0-9]|[1-9][0-9]|1[0-9][0-9]|2[0-2][0-9]|23[0-2])$ ]]; then
    missing+=("ROS_DOMAIN_ID (0-232) in ${DYX3_ETC}/ros.env; the fleet uses ROS_DOMAIN_ID=42")
  fi
  if _enabled dyx3-backend "${src}/installer/manifests/production.manifest"; then
    host="$(_env_value "$(_preflight_file "${src}" backend.env)" DYX3_BACKEND_HOST)"
    ssid="$(_env_value "$(_preflight_file "${src}" hotspot.env)" DYX3_HOTSPOT_SSID)"
    psk="$(_env_value "$(_preflight_file "${src}" hotspot.env)" DYX3_HOTSPOT_PSK)"
    addr="$(_env_value "$(_preflight_file "${src}" hotspot.env)" DYX3_HOTSPOT_ADDRESS)"
    if [ -z "${host}" ] && { [ -z "${ssid}" ] || [ -z "${psk}" ]; }; then
      missing+=("where the backend listens: by default it binds the hotspot address 10.42.0.1, and no hotspot is configured. Set DYX3_HOTSPOT_SSID and DYX3_HOTSPOT_PSK in ${DYX3_ETC}/hotspot.env, or DYX3_BACKEND_HOST in ${DYX3_ETC}/backend.env (the site-LAN address from network.env; 0.0.0.0 on the bench only)")
    elif [ -z "${host}" ] && [ -n "${addr}" ] && [ "${addr%/*}" != 10.42.0.1 ]; then
      missing+=("DYX3_BACKEND_HOST=${addr%/*} in ${DYX3_ETC}/backend.env: the hotspot is at ${addr}, but the backend binds 10.42.0.1 unless told otherwise")
    fi
  fi
  [ "${#missing[@]}" -eq 0 ] && return 0
  local tag="FATAL"
  [ "${DYX3_DRY_RUN}" = "1" ] && tag="WARN"
  {
    printf '[dyx3 %s] per-rover inputs are missing; nothing has been built yet:\n' "${tag}"
    for m in "${missing[@]}"; do printf '  - %s\n' "${m}"; done
    printf '  Edit the files named above (created from deployment/network/*.tmpl), then run the installer again.\n'
  } >&2
  [ "${DYX3_DRY_RUN}" = "1" ] && return 0
  exit 1
}
