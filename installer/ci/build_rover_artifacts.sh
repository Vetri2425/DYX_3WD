#!/usr/bin/env bash
# Build the rover release artifacts in CI: build_rover_artifacts.sh <out-dir>
#
# Runs as root in ros:humble-ros-base on an arm64 runner (Ubuntu 22.04 aarch64 + ROS Humble: the
# Jetson's userland; L4T/CUDA are not in the link path of these packages). Uses the installer's own
# build_px4_msgs / build_release, so the release is byte-for-byte the layout the rover would build,
# at the same absolute paths (/opt/dyx3/...): nothing needs relocating on the rover.
# Consumed by installer/lib/artifacts.sh (install_prebuilt). Proposal 2026-10-08_prebuilt-release-artifacts.md.
set -euo pipefail

OUT="${1:?usage: build_rover_artifacts.sh <out-dir>}"
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
export INSTALLER_DIR="${REPO_DIR}/installer" DYX3_ARTIFACTS=source
export DYX3_BUILD_JOBS="${DYX3_BUILD_JOBS:-$(nproc)}" DYX3_COLCON_WORKERS="${DYX3_COLCON_WORKERS:-$(nproc)}"

# shellcheck source=../lib/common.sh
. "${INSTALLER_DIR}/lib/common.sh"
for lib in os_check dependencies ros_install permissions network_install systemd_install health_check release; do
  # shellcheck disable=SC1090
  . "${INSTALLER_DIR}/lib/${lib}.sh"
done

# The release source is this checkout (CI has no mirror): git archive reads the checkout's object store.
# shellcheck disable=SC2317  # called by build_release
_mirror() { git -C "${REPO_DIR}" rev-parse --absolute-git-dir; }

sha="$(git -C "${REPO_DIR}" rev-parse HEAD)"
load_pin firmware
log "rover artifacts for ${sha} (firmware pin ${FIRMWARE_SHA:0:10}, -j${DYX3_BUILD_JOBS})"

mkdir -p "${DYX3_RELEASES}"
build_px4_msgs
build_release "${sha}"
health_release_only "${DYX3_RELEASES}/${sha}" 0 || die "release ${sha:0:10} failed static verification"

mkdir -p "${OUT}"
rel_f="release-${sha}.tar.zst"
msgs_f="px4_msgs-${FIRMWARE_SHA}.tar.zst"
# Build trees and logs are not needed at runtime (colcon install is self-contained without --symlink-install).
tar -C / -I 'zstd -T0 -15' -cf "${OUT}/${rel_f}" \
  --exclude="opt/dyx3/releases/${sha}/ros2_ws/build" --exclude="opt/dyx3/releases/${sha}/ros2_ws/log" \
  "opt/dyx3/releases/${sha}"
tar -C / -I 'zstd -T0 -15' -cf "${OUT}/${msgs_f}" \
  --exclude="opt/dyx3/px4_msgs/${FIRMWARE_SHA}/build" --exclude="opt/dyx3/px4_msgs/${FIRMWARE_SHA}/log" \
  --exclude="opt/dyx3/px4_msgs/${FIRMWARE_SHA}/src" \
  "opt/dyx3/px4_msgs/${FIRMWARE_SHA}"

# shellcheck disable=SC1091
. /etc/os-release
cat >"${OUT}/artifacts.env" <<ENV
ARTIFACT_STACK_SHA=${sha}
ARTIFACT_FIRMWARE_SHA=${FIRMWARE_SHA}
ARTIFACT_PX4_MSGS_SHA256=$(cat "$(px4_msgs_dir)/px4_msgs.sha256")
ARTIFACT_OS_ID=${ID}
ARTIFACT_OS_VERSION_ID=${VERSION_ID}
ARTIFACT_ARCH=$(uname -m)
ARTIFACT_ROS_DISTRO=${ROS_DISTRO_NAME}
ARTIFACT_RCLCPP_VERSION=$(dpkg-query -W -f='${Version}' "ros-${ROS_DISTRO_NAME}-rclcpp" 2>/dev/null || echo unknown)
ARTIFACT_BUILT_UTC=$(date -u +%Y-%m-%dT%H:%M:%SZ)
ARTIFACT_CI_RUN=${GITHUB_SERVER_URL:-local}/${GITHUB_REPOSITORY:-}/actions/runs/${GITHUB_RUN_ID:-}
ENV
(cd "${OUT}" && sha256sum artifacts.env "${rel_f}" "${msgs_f}" >SHA256SUMS)
ls -la "${OUT}"
cat "${OUT}/SHA256SUMS"
