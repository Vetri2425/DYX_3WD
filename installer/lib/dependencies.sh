#!/usr/bin/env bash
# DYX 3WD installer — OS packages and pinned third-party builds. Source, do not execute.
# shellcheck shell=bash

APT_PACKAGES=(
  build-essential cmake ninja-build meson pkg-config git curl ca-certificates tcpdump
  gnupg lsb-release python3-pip python3-venv network-manager iproute2 iputils-ping
  dnsmasq-base iptables iw
  libssl-dev libasio-dev libtinyxml2-dev nlohmann-json3-dev zstd
)

install_apt_packages() {
  log "apt: base packages"
  run apt-get update
  run env DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends "${APT_PACKAGES[@]}"
  # Here because install.sh installs the dependencies through this function; needs python3-venv from above.
  install_python_tools
}

# pymavlink: the FCU parameter and version read (tools/px4/param_dump.py), run by dyx3-recorder at every run start
# (fcu_param_dump_python) and by hand. In its own venv, like the backend's: never the system or ROS Python. Pinned
# (2.4.49 publishes cp310 aarch64 wheels; its dependencies lxml and fastcrc are resolved by pip). NOT fatal: without it
# every run records params_fcu.json as unavailable with the reason; nothing else depends on it.
PYMAVLINK_VERSION="2.4.49"
install_python_tools() {
  local venv="${DYX3_PREFIX}/third_party/pymavlink"
  local marker="${venv}/.dyx3-pymavlink-version"
  if _marker_ok "${marker}" "${PYMAVLINK_VERSION}" && [ -x "${venv}/bin/python3" ]; then
    log "pymavlink ${PYMAVLINK_VERSION} already installed"
    return 0
  fi
  log "installing pymavlink ${PYMAVLINK_VERSION} into ${venv}"
  if ! run rm -rf "${venv}" || ! run python3 -m venv "${venv}" ||
    ! run "${venv}/bin/pip" install --quiet --disable-pip-version-check "pymavlink==${PYMAVLINK_VERSION}"; then
    warn "pymavlink install FAILED (no network?); runs record params_fcu.json as unavailable until it is installed"
    return 0
  fi
  if [ "${DYX3_DRY_RUN}" != "1" ]; then printf '%s' "${PYMAVLINK_VERSION}" >"${marker}"; fi
}

# _marker_ok <marker-file> <expected-commit>
_marker_ok() { [ -f "$1" ] && [ "$(cat "$1")" = "$2" ]; }

# _checkout_pinned <repo> <commit> <dest>: clean clone at an exact commit, verified.
_checkout_pinned() {
  local repo="$1" commit="$2" dest="$3"
  run rm -rf "${dest}"
  run git init -q "${dest}"
  run git -C "${dest}" remote add origin "${repo}"
  run git -C "${dest}" fetch -q --depth 1 origin "${commit}"
  run git -C "${dest}" checkout -q FETCH_HEAD
  if [ "${DYX3_DRY_RUN}" != "1" ]; then
    [ "$(git -C "${dest}" rev-parse HEAD)" = "${commit}" ] ||
      die "checkout of ${repo} is not ${commit}"
  fi
}

# Micro XRCE-DDS Agent — installed to /usr/local (where the proven bring-up put it).
install_xrce_agent() {
  load_pin microxrce_agent
  local marker="${DYX3_PREFIX}/third_party/microxrce_agent.commit"
  if _marker_ok "${marker}" "${XRCE_COMMIT}" && have MicroXRCEAgent; then
    log "MicroXRCEAgent ${XRCE_TAG} already installed"
    return 0
  fi
  log "building MicroXRCEAgent ${XRCE_TAG} (${XRCE_COMMIT}) with -j${DYX3_BUILD_JOBS}"
  local src="${DYX3_PREFIX}/third_party/src/Micro-XRCE-DDS-Agent"
  run mkdir -p "$(dirname "${src}")"
  _checkout_pinned "${XRCE_REPO}" "${XRCE_COMMIT}" "${src}"
  run cmake -S "${src}" -B "${src}/build" -DCMAKE_BUILD_TYPE=Release
  run env MAKEFLAGS="-j${DYX3_BUILD_JOBS}" nice -n 10 cmake --build "${src}/build" --parallel "${DYX3_BUILD_JOBS}"
  run cmake --install "${src}/build"
  run ldconfig
  run mkdir -p "$(dirname "${marker}")"
  if [ "${DYX3_DRY_RUN}" != "1" ]; then printf '%s' "${XRCE_COMMIT}" >"${marker}"; fi
}

# mavlink-router — service plane only (QGC forward); built with meson.
install_mavlink_router() {
  load_pin mavlink_router
  local marker="${DYX3_PREFIX}/third_party/mavlink_router.commit"
  if _marker_ok "${marker}" "${MAVROUTER_COMMIT}" && have mavlink-routerd; then
    log "mavlink-router ${MAVROUTER_TAG} already installed"
    return 0
  fi
  log "building mavlink-router ${MAVROUTER_TAG} (${MAVROUTER_COMMIT})"
  local src="${DYX3_PREFIX}/third_party/src/mavlink-router"
  run mkdir -p "$(dirname "${src}")"
  _checkout_pinned "${MAVROUTER_REPO}" "${MAVROUTER_COMMIT}" "${src}"
  # mavlink-router vendors the MAVLink C library as a submodule.
  run git -C "${src}" submodule update --init --recursive --depth 1
  run meson setup "${src}/build" "${src}" --buildtype=release -Dsystemdsystemunitdir=/nonexistent
  run nice -n 10 ninja -C "${src}/build" -j "${DYX3_BUILD_JOBS}"
  run ninja -C "${src}/build" install
  run mkdir -p "$(dirname "${marker}")"
  if [ "${DYX3_DRY_RUN}" != "1" ]; then printf '%s' "${MAVROUTER_COMMIT}" >"${marker}"; fi
}
