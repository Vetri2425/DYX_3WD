#!/usr/bin/env bash
# DYX 3WD installer — FCU Ethernet link. Source, do not execute.
# shellcheck shell=bash
#
# Topology (spec 4.4): Jetson 10.41.10.1/24 <-> PX4 10.41.10.2/24, point-to-point, static.
# No gateway, no DNS, never the default route on the FCU link.

FCU_CON_NAME="${FCU_CON_NAME:-dyx3-fcu}"
FCU_IFACE="${FCU_IFACE:-enP8p1s0}"
FCU_JETSON_CIDR="${FCU_JETSON_CIDR:-10.41.10.1/24}"
# Bench only: keep DHCP alongside the static address while the board is on a site router.
# DERIVED — NOT FROM V1 SPEC (HANDOFF 2026-10-07). Production is static-only.
# On the bench the site router behind the FCU switch is the Jetson's only WAN, so its DHCP
# default route is kept too; otherwise the release fetch from GitHub fails (seen 2026-10-08).
FCU_KEEP_DHCP="${FCU_KEEP_DHCP:-0}"

install_fcu_network() {
  if ! have nmcli && [ "${DYX3_DRY_RUN}" != "1" ]; then
    warn "nmcli not found: skipping FCU network profile"
    return 0
  fi
  local method="manual" never_default="yes"
  if [ "${FCU_KEEP_DHCP}" = "1" ]; then
    method="auto"
    never_default="no"
  fi
  local common=(
    connection.interface-name "${FCU_IFACE}"
    connection.autoconnect yes
    ipv4.method "${method}"
    ipv4.addresses "${FCU_JETSON_CIDR}"
    ipv4.never-default "${never_default}"
    ipv6.method disabled
  )
  if [ "${DYX3_DRY_RUN}" != "1" ] && nmcli -t -f NAME connection show | grep -qx "${FCU_CON_NAME}"; then
    log "updating NetworkManager profile ${FCU_CON_NAME} (${FCU_IFACE} ${FCU_JETSON_CIDR})"
    run nmcli connection modify "${FCU_CON_NAME}" "${common[@]}"
  else
    log "creating NetworkManager profile ${FCU_CON_NAME} (${FCU_IFACE} ${FCU_JETSON_CIDR})"
    run nmcli connection add type ethernet con-name "${FCU_CON_NAME}" ifname "${FCU_IFACE}" "${common[@]}"
  fi
  # Bring up only if the interface exists; a missing cable must not fail the install.
  if ip link show "${FCU_IFACE}" >/dev/null 2>&1; then
    run nmcli connection up "${FCU_CON_NAME}" || warn "could not bring ${FCU_CON_NAME} up now (cable?)"
  else
    warn "interface ${FCU_IFACE} not present; profile will apply when it appears"
  fi
}

# Optional access point. Never pass the PSK to nmcli argv, run(), or log().
# NetworkManager's shared method provides DHCP but also forwards; the pre-up
# dispatcher blocks forwarding in both directions between Wi-Fi and the FCU.
install_hotspot_network() {
  local env_file="${DYX3_ETC}/hotspot.env" ssid="" psk="" key value iface profile dir tmp dispatcher old_umask
  profile="${DYX3_ROOT}/etc/NetworkManager/system-connections/dyx3-hotspot.nmconnection"
  if [ -r "${env_file}" ]; then
    while IFS='=' read -r key value || [ -n "${key:-}" ]; do
      case "${key}" in
        DYX3_HOTSPOT_SSID) ssid="${value}" ;;
        DYX3_HOTSPOT_PSK) psk="${value}" ;;
      esac
    done <"${env_file}"
  fi
  if [ -z "${ssid}" ] || [ -z "${psk}" ]; then
    log "hotspot not configured; leaving access point disabled"
    if [ -f "${profile}" ]; then
      run rm -f "${profile}"
      [ -n "${DYX3_ROOT}" ] || nmcli connection delete dyx3-hotspot >/dev/null 2>&1 || true
    fi
    return 0
  fi
  # DERIVED — NOT FROM V1 SPEC: accept an unquoted ASCII subset so keyfile
  # values stay literal and cannot inject another setting or log line.
  # Restrict keyfile values to a single literal line. This also keeps the config
  # parser unambiguous without ever putting a secret into an argument or log.
  if [[ ! "${ssid}" =~ ^[A-Za-z0-9._-]{1,32}$ ]] ||
     [[ ! "${psk}" =~ ^[A-Za-z0-9._@%+=:,/!?-]{8,63}$ ]]; then
    warn "hotspot.env has an invalid SSID or passphrase format; access point disabled"
    if [ -f "${profile}" ]; then
      run rm -f "${profile}"
      [ -n "${DYX3_ROOT}" ] || nmcli connection delete dyx3-hotspot >/dev/null 2>&1 || true
    fi
    return 0
  fi
  if ! have nmcli; then
    warn "nmcli not found; skipping hotspot"
    return 0
  fi
  # DERIVED — NOT FROM V1 SPEC: select the first reported Wi-Fi device; AP
  # capability still needs the bench check before this is used on a rover.
  iface="$(nmcli -t -f DEVICE,TYPE device status 2>/dev/null | awk -F: '$2 == "wifi" && $1 != "--" {print $1; exit}' || true)"
  if [ -z "${iface}" ]; then
    warn "no Wi-Fi device found; skipping hotspot profile"
    return 0
  fi
  if [[ ! "${iface}" =~ ^[A-Za-z0-9_.-]+$ ]] || [[ ! "${FCU_IFACE}" =~ ^[A-Za-z0-9_.-]+$ ]]; then
    warn "invalid network interface name; skipping hotspot"
    return 0
  fi
  dir="$(dirname "${profile}")"
  dispatcher="${DYX3_ROOT}/etc/NetworkManager/dispatcher.d/pre-up.d/90-dyx3-hotspot-isolation"
  if [ "${DYX3_DRY_RUN}" = "1" ]; then
    log "would configure hotspot profile on ${iface} (credentials redacted)"
    return 0
  fi
  install -d -m 0700 "${dir}" "$(dirname "${dispatcher}")"
  # The hook runs before NetworkManager reports the AP active, including after reboot.
  cat >"${dispatcher}" <<EOF
#!/usr/bin/env bash
set -euo pipefail
[ "\${CONNECTION_ID:-}" = "dyx3-hotspot" ] || exit 0
[ "\${1:-}" = "${iface}" ] || exit 0
iptables -D FORWARD -i "${iface}" -o "${FCU_IFACE}" -j DROP 2>/dev/null || true
iptables -I FORWARD -i "${iface}" -o "${FCU_IFACE}" -j DROP
iptables -D FORWARD -i "${FCU_IFACE}" -o "${iface}" -j DROP 2>/dev/null || true
iptables -I FORWARD -i "${FCU_IFACE}" -o "${iface}" -j DROP
EOF
  chmod 0700 "${dispatcher}"
  if [ -z "${DYX3_ROOT}" ]; then
    chown root:root "${dispatcher}"
    # Install the isolation rules before activation as well as on every pre-up.
    CONNECTION_ID=dyx3-hotspot "${dispatcher}" "${iface}" pre-up || {
      warn "FCU isolation rule failed; hotspot remains disabled"
      return 0
    }
  fi
  old_umask="$(umask)"
  umask 077
  tmp="$(mktemp "${dir}/.dyx3-hotspot.XXXXXX")"
  cat >"${tmp}" <<EOF
[connection]
id=dyx3-hotspot
uuid=9ca38e9f-cbb9-5d03-a237-6e17193bf4dc
type=wifi
interface-name=${iface}
autoconnect=true

[wifi]
mode=ap
ssid=${ssid}

[wifi-security]
key-mgmt=wpa-psk
psk=${psk}

[ipv4]
method=shared
address1=10.42.0.1/24
never-default=true

[ipv6]
method=disabled
EOF
  chmod 0600 "${tmp}"
  if [ -z "${DYX3_ROOT}" ]; then chown root:root "${tmp}"; fi
  mv -f "${tmp}" "${profile}"
  umask "${old_umask}"
  if [ -z "${DYX3_ROOT}" ]; then
    nmcli connection load "${profile}" >/dev/null 2>&1 || warn "NetworkManager could not load hotspot profile"
    nmcli connection up dyx3-hotspot >/dev/null 2>&1 || warn "hotspot could not start now (Wi-Fi AP support unverified)"
  fi
  log "hotspot profile configured on ${iface} (credentials redacted)"
}
