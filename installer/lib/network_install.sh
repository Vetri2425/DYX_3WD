#!/usr/bin/env bash
# DYX 3WD installer — FCU Ethernet link. Source, do not execute.
# shellcheck shell=bash
#
# Topology (spec 4.4): Jetson 10.41.10.1/24 <-> PX4 10.41.10.2/24, point-to-point, static.
# No gateway, no DNS, never the default route. The hotspot / LTE profiles are Phase 11.

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
