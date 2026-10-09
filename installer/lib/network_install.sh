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
# The environment wins; otherwise FCU_KEEP_DHCP in /etc/dyx3/network.env (INS-019: a later reinstall or
# re-apply without the variable used to silently drop the bench WAN); otherwise 0.
FCU_KEEP_DHCP="${FCU_KEEP_DHCP:-}"

# Per-rover site LAN on the same Ethernet port (/etc/dyx3/network.env, created once, never overwritten).
# The Jetson baseboard has one Ethernet port behind an internal switch shared with the Pixhawk, so a site
# router cabled into the rover reaches the Jetson on this port. DYX3_LAN_ADDRESS adds a static address
# next to the FCU address; DYX3_LAN_GATEWAY makes that router the default route at metric 200, above a
# USB 4G dongle (NetworkManager default 100), so the dongle stays first when present.
# Proven on rover 01 2026-10-09: 192.168.3.150/24 via 192.168.3.1, with DDS and QGC unaffected.
_ipv4_ok() { [[ "$1" =~ ^((25[0-5]|2[0-4][0-9]|1?[0-9]?[0-9])\.){3}(25[0-5]|2[0-4][0-9]|1?[0-9]?[0-9])$ ]]; }
_ipv4_int() { local IFS=.; read -r a b c d <<<"$1"; echo $(((a << 24) | (b << 16) | (c << 8) | d)); }
# _ipv4_net CIDR -> "network/prefix" (prints nothing for an invalid CIDR)
_ipv4_net() {
  local ip="${1%/*}" pfx="${1#*/}" mask
  _ipv4_ok "${ip}" && [[ "${pfx}" =~ ^([0-9]|[12][0-9]|3[0-2])$ ]] || return 0
  mask=$(((0xFFFFFFFF << (32 - pfx)) & 0xFFFFFFFF))
  echo "$(($(_ipv4_int "${ip}") & mask))/${pfx}"
}
# Missing file = no site LAN. Must not fail: install.sh runs under set -e and pipefail.
_network_env() {
  [ -r "${DYX3_ETC}/network.env" ] || return 0
  sed -n "s/^$1=//p" "${DYX3_ETC}/network.env" | tail -n1
}

install_fcu_network() {
  if ! have nmcli && [ "${DYX3_DRY_RUN}" != "1" ]; then
    warn "nmcli not found: skipping FCU network profile"
    return 0
  fi
  local method="manual" never_default="yes" keep_dhcp="${FCU_KEEP_DHCP}"
  [ -n "${keep_dhcp}" ] || keep_dhcp="$(_network_env FCU_KEEP_DHCP)"
  if [ "${keep_dhcp}" = "1" ]; then
    warn "FCU_KEEP_DHCP=1: DHCP and its default route stay on the FCU port (bench only)"
    method="auto"
    never_default="no"
  fi
  local addresses="${FCU_JETSON_CIDR}" lan_cidr lan_gw route=(ipv4.gateway "" ipv4.dns "" ipv4.route-metric -1)
  lan_cidr="$(_network_env DYX3_LAN_ADDRESS)"
  lan_gw="$(_network_env DYX3_LAN_GATEWAY)"
  if [ -n "${lan_cidr}" ]; then
    if [ -z "$(_ipv4_net "${lan_cidr}")" ] || [ "$(_ipv4_net "${lan_cidr}")" = "$(_ipv4_net "${FCU_JETSON_CIDR}")" ]; then
      warn "network.env DYX3_LAN_ADDRESS '${lan_cidr}' is invalid or on the FCU subnet; ignored"
      lan_cidr=""
    else
      addresses="${FCU_JETSON_CIDR},${lan_cidr}"
    fi
  fi
  if [ -n "${lan_cidr}" ] && [ -n "${lan_gw}" ]; then
    if _ipv4_ok "${lan_gw}"; then
      never_default="no"
      route=(ipv4.gateway "${lan_gw}" ipv4.dns "${lan_gw}" ipv4.route-metric 200)
    else
      warn "network.env DYX3_LAN_GATEWAY '${lan_gw}' is invalid; no LAN default route"
    fi
  fi
  local common=(
    connection.interface-name "${FCU_IFACE}"
    connection.autoconnect yes
    ipv4.method "${method}"
    ipv4.addresses "${addresses}"
    ipv4.never-default "${never_default}"
    "${route[@]}"
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
  local country="$1" unit dir before=""
  unit="${DYX3_ROOT}/etc/systemd/system/dyx3-wifi-regdom.service"
  dir="${DYX3_ROOT}/etc/systemd/system/multi-user.target.wants"
  install -d -m 0755 "$(dirname "${unit}")" "${dir}"
  [ -f "${unit}" ] && before="$(cat "${unit}")"
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
  # Unchanged: leave the running Wi-Fi alone (INS-004).
  [ "${before}" = "$(cat "${unit}")" ] && return 0
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

# _hotspot_invalid <reason>: INS-010. A hotspot.env that fails validation never removes a working access point: it is
# often the operator's only link to the rover. Only SSID and PSK both deliberately empty remove it.
_hotspot_invalid() {
  if [ -f "${DYX3_ROOT}/etc/NetworkManager/system-connections/dyx3-hotspot.nmconnection" ]; then
    warn "hotspot.env: $1; keeping the existing access point unchanged. Fix ${DYX3_ETC}/hotspot.env (SSID and PSK both empty remove the access point)"
  else
    warn "hotspot.env: $1; access point not configured"
  fi
}

# _hotspot_pick_iface <wanted>: the Wi-Fi device for the access point. An explicit name must be a
# Wi-Fi device NetworkManager reports. Otherwise the first Wi-Fi device that is not on USB is used:
# a USB dongle is the rover's internet uplink (client mode), never the tablet access point.
_hotspot_pick_iface() {
  local wanted="$1" dev type path
  while IFS=: read -r dev type; do
    [ "${type}" = wifi ] && [ -n "${dev}" ] && [ "${dev}" != "--" ] || continue
    if [ -n "${wanted}" ]; then
      [ "${dev}" = "${wanted}" ] && { printf '%s\n' "${dev}"; return 0; }
      continue
    fi
    path="$(readlink -f "${DYX3_ROOT}/sys/class/net/${dev}/device" 2>/dev/null || true)"
    case "${path}" in */usb*) continue ;; esac
    printf '%s\n' "${dev}"
    return 0
  done < <(nmcli -t -f DEVICE,TYPE device status 2>/dev/null || true)
  return 0
}

install_hotspot_network() {
  local env_file="${DYX3_ETC}/hotspot.env" ssid="" psk="" key value iface profile dir tmp dispatcher old_umask
  local country="IN" band="a" channel="" width="20" nm_ver major minor width_setting="" address="" want_iface=""
  local dl_limit="2mb/s" dl_burst="4mb" ul_limit="1mb/s" ul_burst="2mb" txqlen="100" qos ssid_set=0 psk_set=0
  profile="${DYX3_ROOT}/etc/NetworkManager/system-connections/dyx3-hotspot.nmconnection"
  if [ -r "${env_file}" ]; then
    while IFS='=' read -r key value || [ -n "${key:-}" ]; do
      case "${key}" in
        DYX3_HOTSPOT_SSID)
          ssid="${value}"
          ssid_set=1
          ;;
        DYX3_HOTSPOT_PSK)
          psk="${value}"
          psk_set=1
          ;;
        DYX3_WIFI_COUNTRY) country="${value}" ;;
        DYX3_WIFI_BAND) band="${value}" ;;
        DYX3_WIFI_CHANNEL) channel="${value}" ;;
        DYX3_WIFI_WIDTH) width="${value}" ;;
        DYX3_HOTSPOT_ADDRESS) address="${value}" ;;
        DYX3_HOTSPOT_IFACE) want_iface="${value}" ;;
        DYX3_HOTSPOT_DOWNLOAD_LIMIT) dl_limit="${value:-${dl_limit}}" ;;
        DYX3_HOTSPOT_DOWNLOAD_BURST) dl_burst="${value:-${dl_burst}}" ;;
        DYX3_HOTSPOT_UPLOAD_LIMIT) ul_limit="${value:-${ul_limit}}" ;;
        DYX3_HOTSPOT_UPLOAD_BURST) ul_burst="${value:-${ul_burst}}" ;;
        DYX3_HOTSPOT_TXQUEUELEN) txqlen="${value:-${txqlen}}" ;;
      esac
    done <"${env_file}"
  fi
  if [ -z "${ssid}" ] && [ -z "${psk}" ]; then
    if [ "${ssid_set}" = 1 ] && [ "${psk_set}" = 1 ]; then
      log "hotspot disabled in hotspot.env (SSID and PSK both empty)"
      if [ -f "${profile}" ]; then
        _hotspot_drop_profile "${profile}"
      fi
    else
      log "hotspot.env has no SSID/PSK lines; access point left as it is"
    fi
    return 0
  fi
  if [ -z "${ssid}" ] || [ -z "${psk}" ]; then
    _hotspot_invalid "only one of DYX3_HOTSPOT_SSID and DYX3_HOTSPOT_PSK is set"
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
    _hotspot_invalid "invalid SSID, passphrase, country, band or width format"
    return 0
  fi
  # Per-rover access-point address (fleet plan 2026-10-09: 192.168.3.100/24 for the first 3WD, .101 for
  # the next). Blank keeps NetworkManager's shared default. It must never overlap the FCU link.
  address="${address:-10.42.0.1/24}"
  if [[ ! "${address}" =~ ^((25[0-5]|2[0-4][0-9]|1?[0-9]?[0-9])\.){3}(25[0-5]|2[0-4][0-9]|1?[0-9]?[0-9])/(1[6-9]|2[0-9]|30)$ ]] ||
     [[ "${address}" == 10.41.10.* ]] ||
     { [ -n "${want_iface}" ] && [[ ! "${want_iface}" =~ ^[A-Za-z0-9_.-]+$ ]]; }; then
    _hotspot_invalid "invalid DYX3_HOTSPOT_ADDRESS or DYX3_HOTSPOT_IFACE"
    return 0
  fi
  # 5 GHz default is 149, not 36. The Jetson's vendor rtl8822ce driver is self-managed for regulatory
  # and locked to its world channel plan (alpha2 00, chplan 0x7F). It ignores `iw reg set`, the
  # rtw_country_code module parameter and the proc country_code. Under that plan only 5745 MHz (149) is
  # not no-IR, so an AP on 36-48 or 153-165 fails ("Failed to start AP functionality", rover 2026-10-09).
  # 149 is legal in India (5725-5875 MHz).
  if [ -n "$(_network_env DYX3_LAN_ADDRESS)" ] && [ "$(_ipv4_net "${address}")" = "$(_ipv4_net "$(_network_env DYX3_LAN_ADDRESS)")" ]; then
    _hotspot_invalid "hotspot address ${address} is on the site LAN subnet (network.env)"
    return 0
  fi
  if [ -z "${channel}" ]; then
    if [ "${band}" = "a" ]; then channel=149; else channel=6; fi
  fi
  if [[ ! "${dl_limit}" =~ ^[0-9]{1,6}[km]?b/s$ ]] || [[ ! "${ul_limit}" =~ ^[0-9]{1,6}[km]?b/s$ ]] ||
     [[ ! "${dl_burst}" =~ ^[0-9]{1,6}[km]?b$ ]] || [[ ! "${ul_burst}" =~ ^[0-9]{1,6}[km]?b$ ]] ||
     [[ ! "${txqlen}" =~ ^[0-9]{2,4}$ ]]; then
    _hotspot_invalid "invalid client-internet limit or txqueuelen"
    return 0
  fi
  if [ "${band}" = "a" ]; then
    case "${channel}" in 36 | 40 | 44 | 48 | 149 | 153 | 157 | 161 | 165) ;; *) _hotspot_invalid "5 GHz DFS or invalid channel refused"; return 0 ;; esac
    if [ "${channel}" = 165 ] && [ "${width}" = 40 ]; then
      _hotspot_invalid "channel 165 cannot use 40 MHz"
      return 0
    fi
  elif ! [[ "${channel}" =~ ^([1-9]|10|11)$ ]]; then
    _hotspot_invalid "invalid 2.4 GHz channel"
    return 0
  fi
  if ! have nmcli; then
    warn "nmcli not found; skipping hotspot"
    return 0
  fi
  iface="$(_hotspot_pick_iface "${want_iface}")"
  if [ -z "${iface}" ]; then
    if [ -n "${want_iface}" ]; then
      warn "Wi-Fi device ${want_iface} not found; skipping hotspot profile"
    else
      warn "no Wi-Fi device found (USB Wi-Fi is reserved for the internet uplink); skipping hotspot profile"
    fi
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
    _hotspot_invalid "NetworkManager 1.50+ is required for 40 MHz AP width"
    return 0
  fi
  dir="$(dirname "${profile}")"
  dispatcher="${DYX3_ROOT}/etc/NetworkManager/dispatcher.d/pre-up.d/90-dyx3-hotspot-isolation"
  qos="${DYX3_ROOT}/etc/NetworkManager/dispatcher.d/99-dyx3-hotspot-qos"
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
iptables -w -D FORWARD -i "${iface}" -o "${FCU_IFACE}" -j DROP 2>/dev/null || true
iptables -w -I FORWARD -i "${iface}" -o "${FCU_IFACE}" -j DROP
iptables -w -D FORWARD -i "${FCU_IFACE}" -o "${iface}" -j DROP 2>/dev/null || true
iptables -w -I FORWARD -i "${FCU_IFACE}" -o "${iface}" -j DROP
EOF
  chmod 0700 "${dispatcher}"
  # Client-internet cap, ported from the 4WD prototype (rover_ws scripts/network/99-dyx-hotspot-qos).
  # Measured there on 2026-09-28: hotspot clients' own downloads through the rover's 4G filled the
  # 1000-packet Wi-Fi queue in front of the app websocket, and the app hung (worst ping 1459 ms).
  # Policing them and shortening the queue gave a 90 ms worst case and 0/100 pings over 100 ms.
  # 3WD defaults are generous (2 MiB/s down, 1 MiB/s up; owner decision 2026-10-09): with external
  # antennas the Wi-Fi link is much faster than 4G, so the cap only bounds big background downloads.
  # A Raspberry Pi hotspot needs no cap because its kernel runs fq_codel. The tegra kernel has no qdisc
  # beyond pfifo_fast; building sch_fq_codel for it is the planned real fix.
  # The tegra kernel has no HTB/fq_codel/cake/TBF, but it has xt_hashlimit. Dropping above a fixed
  # rate makes TCP back off. Only forwarded traffic is limited; the backend, ssh and NTRIP are not.
  # The mangle FORWARD hook runs before NetworkManager's shared-mode filter rules.
  cat >"${qos}" <<QOS
#!/usr/bin/env bash
set -uo pipefail
case "\${2:-}" in up | dhcp4-change | connectivity-change) ;; *) exit 0 ;; esac
[ -d "/sys/class/net/${iface}" ] || exit 0
ip link set dev "${iface}" txqueuelen ${txqlen} 2>/dev/null || true
iptables -w -t mangle -N DYX3_HOTSPOT_QOS 2>/dev/null || true
iptables -w -t mangle -F DYX3_HOTSPOT_QOS
iptables -w -t mangle -C FORWARD -j DYX3_HOTSPOT_QOS 2>/dev/null || iptables -w -t mangle -I FORWARD 1 -j DYX3_HOTSPOT_QOS
iptables -w -t mangle -A DYX3_HOTSPOT_QOS ! -i "${iface}" -o "${iface}" -m hashlimit --hashlimit-name dyx3_dl --hashlimit-above ${dl_limit} --hashlimit-burst ${dl_burst} -j DROP
iptables -w -t mangle -A DYX3_HOTSPOT_QOS -i "${iface}" ! -o "${iface}" -m hashlimit --hashlimit-name dyx3_ul --hashlimit-above ${ul_limit} --hashlimit-burst ${ul_burst} -j DROP
logger -t dyx3-hotspot-qos "applied on \${1:-?}/\${2:-?}: download<=${dl_limit} upload<=${ul_limit} txqueuelen=${txqlen}"
QOS
  chmod 0700 "${qos}"
  if [ -z "${DYX3_ROOT}" ]; then
    chown root:root "${dispatcher}"
    # Install the isolation rules before activation as well as on every pre-up.
    CONNECTION_ID=dyx3-hotspot "${dispatcher}" "${iface}" pre-up || {
      warn "FCU isolation rule failed; hotspot remains disabled"
      return 0
    }
    chown root:root "${qos}"
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
address1=${address}
never-default=true

[ipv6]
method=disabled
EOF
  chmod 0600 "${tmp}"
  umask "${old_umask}"
  # Byte-identical to the installed profile: do not reload or re-activate the access point (INS-004). Re-activating
  # drops every Wi-Fi client, including an operator's ssh session. Only bring it up if it is not active.
  if [ -f "${profile}" ] && cmp -s "${tmp}" "${profile}"; then
    rm -f "${tmp}"
    if [ -z "${DYX3_ROOT}" ] && ! nmcli -t -f NAME connection show --active 2>/dev/null | grep -qx dyx3-hotspot; then
      nmcli connection up dyx3-hotspot >/dev/null 2>&1 || warn "hotspot could not start now (Wi-Fi AP support unverified)"
    fi
    log "hotspot profile unchanged on ${iface}; access point left as it is (credentials redacted)"
    return 0
  fi
  if [ -z "${DYX3_ROOT}" ]; then chown root:root "${tmp}"; fi
  mv -f "${tmp}" "${profile}"
  if [ -z "${DYX3_ROOT}" ]; then
    # A disabled Wi-Fi radio (persisted as WirelessEnabled=false) leaves the device "unavailable".
    nmcli radio wifi on >/dev/null 2>&1 || warn "could not enable the Wi-Fi radio (rfkill?)"
    local other
    while IFS=: read -r other; do
      [ -n "${other}" ] && [ "${other}" != dyx3-hotspot ] || continue
      if [ "$(nmcli -g connection.interface-name connection show "${other}" 2>/dev/null)" = "${iface}" ]; then
        warn "Wi-Fi profile '${other}' is bound to the access-point device ${iface}; bind it to the USB uplink instead"
      fi
    done < <(nmcli -t -f NAME,TYPE connection show 2>/dev/null | awk -F: '$2 == "802-11-wireless" {print $1}')
    nmcli connection load "${profile}" >/dev/null 2>&1 || warn "NetworkManager could not load hotspot profile"
    nmcli connection up dyx3-hotspot >/dev/null 2>&1 || warn "hotspot could not start now (Wi-Fi AP support unverified)"
  fi
  log "hotspot profile configured on ${iface} at ${address} (credentials redacted)"
}

# Field rovers must not update themselves. Over the 4G uplink, Ubuntu's background updaters
# (apt-daily, unattended-upgrades, PackageKit, fwupd, update-notifier) compete with NTRIP and QGC
# for bandwidth and CPU, and an unattended upgrade can replace system packages under the running
# stack. On the rover (2026-10-09), the dongle pulled ~20 MB in 4 minutes after boot, with
# apt-daily-upgrade 14 minutes away. Manual `apt` (used by this installer) keeps working;
# updates are an operator action.
DYX3_AUTO_UPDATE_UNITS=(apt-daily.timer apt-daily-upgrade.timer update-notifier-download.timer
  update-notifier-motd.timer fwupd-refresh.timer packagekit.service)
install_no_auto_updates() {
  local conf="${DYX3_ROOT}/etc/apt/apt.conf.d/99dyx3-no-auto-updates" unit
  if [ "${DYX3_DRY_RUN}" = "1" ]; then
    log "would disable automatic OS updates"
    return 0
  fi
  install -d -m 0755 "$(dirname "${conf}")"
  cat >"${conf}" <<'APT'
// Managed by the DYX 3WD installer: no background package activity on a field rover.
APT::Periodic::Update-Package-Lists "0";
APT::Periodic::Download-Upgradeable-Packages "0";
APT::Periodic::Unattended-Upgrade "0";
APT::Periodic::AutocleanInterval "0";
APT
  chmod 0644 "${conf}"
  if [ -z "${DYX3_ROOT}" ] && have systemctl; then
    for unit in "${DYX3_AUTO_UPDATE_UNITS[@]}"; do
      systemctl stop "${unit}" >/dev/null 2>&1 || true
      systemctl mask "${unit}" >/dev/null 2>&1 || true
    done
  fi
  log "automatic OS updates disabled (manual apt still works)"
}
