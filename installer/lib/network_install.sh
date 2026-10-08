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
_hotspot_driver_powersave() {
  local iface="$1" module="" target="" option="" params="" conf
  conf="${DYX3_ROOT}/etc/modprobe.d/dyx3-rtl8822ce-powersave.conf"
  module="$(basename "$(readlink "${DYX3_ROOT}/sys/class/net/${iface}/device/driver/module" 2>/dev/null)" 2>/dev/null)"
  if [ -z "${module}" ] && have lsmod; then
    module="$(lsmod | awk '$1 == "rtl8822ce" || $1 == "rtw88_8822ce" || $1 == "rtw_8822ce" {print $1; exit}')"
  fi
  case "${module}" in
    rtl8822ce) target=rtl8822ce; option=rtw_power_mgnt=0 ;;
    rtw88_8822ce | rtw_8822ce) target=rtw88_core; option=disable_lps_deep=1 ;;
    *) warn "RTL8822CE driver module not found; skipping driver power-save option"; rm -f "${conf}"; return 0 ;;
  esac
  if ! have modinfo || ! params="$(modinfo -p "${target}" 2>/dev/null)" ||
     ! printf '%s\n' "${params}" | grep -q "^${option%%=*}:"; then
    warn "${target} has no verified ${option%%=*} option; skipping driver power-save option"
    rm -f "${conf}"
    return 0
  fi
  install -d -m 0755 "$(dirname "${conf}")"
  printf 'options %s %s\n' "${target}" "${option}" >"${conf}"
  chmod 0644 "${conf}"
  log "installed verified ${target} driver power-save option"
}

_hotspot_install_regdom() {
  local country="$1" unit dir
  unit="${DYX3_ROOT}/etc/systemd/system/dyx3-wifi-regdom.service"
  dir="${DYX3_ROOT}/etc/systemd/system/multi-user.target.wants"
  install -d -m 0755 "$(dirname "${unit}")" "${dir}"
  cat >"${unit}" <<EOF
[Unit]
Description=Set DYX3 Wi-Fi regulatory country
Before=NetworkManager.service

[Service]
Type=oneshot
ExecStart=/usr/bin/env iw reg set ${country}
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
EOF
  chmod 0644 "${unit}"
  ln -sfn ../dyx3-wifi-regdom.service "${dir}/dyx3-wifi-regdom.service"
  if [ -z "${DYX3_ROOT}" ]; then
    systemctl daemon-reload
    systemctl restart dyx3-wifi-regdom.service || warn "could not apply Wi-Fi country now; check iw reg get"
  fi
}

_hotspot_drop_profile() {
  local profile="$1"
  if [ "${DYX3_DRY_RUN}" = "1" ]; then
    log "would disable hotspot profile"
    return 0
  fi
  rm -f "${profile}"
  if [ -z "${DYX3_ROOT}" ] && have nmcli; then
    nmcli connection delete dyx3-hotspot >/dev/null 2>&1 || true
  fi
}

install_hotspot_network() {
  local env_file="${DYX3_ETC}/hotspot.env" ssid="" psk="" key value iface profile dir tmp dispatcher old_umask
  local country="IN" band="a" channel="" width="20" nm_ver major minor width_setting=""
  profile="${DYX3_ROOT}/etc/NetworkManager/system-connections/dyx3-hotspot.nmconnection"
  if [ -r "${env_file}" ]; then
    while IFS='=' read -r key value || [ -n "${key:-}" ]; do
      case "${key}" in
        DYX3_HOTSPOT_SSID) ssid="${value}" ;;
        DYX3_HOTSPOT_PSK) psk="${value}" ;;
        DYX3_WIFI_COUNTRY) country="${value}" ;;
        DYX3_WIFI_BAND) band="${value}" ;;
        DYX3_WIFI_CHANNEL) channel="${value}" ;;
        DYX3_WIFI_WIDTH) width="${value}" ;;
      esac
    done <"${env_file}"
  fi
  if [ -z "${ssid}" ] || [ -z "${psk}" ]; then
    log "hotspot not configured; leaving access point disabled"
    if [ -f "${profile}" ]; then
      _hotspot_drop_profile "${profile}"
    fi
    return 0
  fi
  # DERIVED — NOT FROM V1 SPEC: accept an unquoted ASCII subset so keyfile
  # values stay literal and cannot inject another setting or log line.
  # Restrict keyfile values to a single literal line. This also keeps the config
  # parser unambiguous without ever putting a secret into an argument or log.
  if [[ ! "${ssid}" =~ ^[A-Za-z0-9._-]{1,32}$ ]] ||
     [[ ! "${psk}" =~ ^[A-Za-z0-9._@%+=:,/!?-]{8,63}$ ]] ||
     [[ ! "${country}" =~ ^[A-Z]{2}$ ]] ||
     [[ ! "${band}" =~ ^(a|bg)$ ]] ||
     [[ ! "${width}" =~ ^(20|40)$ ]]; then
    warn "hotspot.env has an invalid SSID or passphrase format; access point disabled"
    if [ -f "${profile}" ]; then
      _hotspot_drop_profile "${profile}"
    fi
    return 0
  fi
  if [ -z "${channel}" ]; then
    if [ "${band}" = "a" ]; then channel=36; else channel=6; fi
  fi
  if [ "${band}" = "a" ]; then
    case "${channel}" in 36 | 40 | 44 | 48 | 149 | 153 | 157 | 161 | 165) ;; *) warn "5 GHz DFS or invalid channel refused; hotspot disabled"; _hotspot_drop_profile "${profile}"; return 0 ;; esac
    if [ "${channel}" = 165 ] && [ "${width}" = 40 ]; then
      warn "channel 165 cannot use 40 MHz; hotspot disabled"
      _hotspot_drop_profile "${profile}"
      return 0
    fi
  elif ! [[ "${channel}" =~ ^([1-9]|10|11)$ ]]; then
    warn "invalid 2.4 GHz channel; hotspot disabled"
    _hotspot_drop_profile "${profile}"
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
  # NM gained explicit AP channel-width in 1.50. Older versions choose 20 MHz
  # with auto; refuse 40 MHz there rather than silently using a narrower channel.
  nm_ver="$(nmcli --version 2>/dev/null || true)"
  major="$(printf '%s\n' "${nm_ver}" | sed -nE 's/.*version ([0-9]+)\.([0-9]+).*/\1/p')"
  minor="$(printf '%s\n' "${nm_ver}" | sed -nE 's/.*version ([0-9]+)\.([0-9]+).*/\2/p')"
  if [ -n "${major}" ] && { [ "${major}" -gt 1 ] || { [ "${major}" -eq 1 ] && [ "${minor}" -ge 50 ]; }; }; then
    width_setting="channel-width=${width}"
  elif [ "${width}" = 40 ]; then
    warn "NetworkManager 1.50+ is required for 40 MHz AP width; hotspot disabled"
    _hotspot_drop_profile "${profile}"
    return 0
  fi
  dir="$(dirname "${profile}")"
  dispatcher="${DYX3_ROOT}/etc/NetworkManager/dispatcher.d/pre-up.d/90-dyx3-hotspot-isolation"
  if [ "${DYX3_DRY_RUN}" = "1" ]; then
    log "would configure hotspot profile on ${iface} (credentials redacted)"
    return 0
  fi
  _hotspot_install_regdom "${country}"
  _hotspot_driver_powersave "${iface}"
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
band=${band}
channel=${channel}
powersave=2
${width_setting}

[wifi-security]
key-mgmt=wpa-psk
proto=rsn
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
