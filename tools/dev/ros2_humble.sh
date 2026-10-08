#!/usr/bin/env bash
# Persistent local ROS 2 Humble build/test environment (Colima + ros:humble-ros-base).
# Usage: tools/dev/ros2_humble.sh {setup|status|sync|build|test|build-test|test-pkg <pkg>...|shell|exec <cmd>|stop}
# See docs/agents/LOCAL_ROS2_BUILD_ENV.md. Never pushes, never prunes, never touches other Docker state.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "${HERE}/../.." && pwd)"
PROFILE="${DYX3_COLIMA_PROFILE:-dyx3-ros2}"
IMAGE="dyx3-ros2-humble:local"
VOL_MSGS="dyx3-px4-msgs"
VOL_WS="dyx3-ws"
LOGDIR="${REPO}/build/local-ros2-logs"

# Dedicated Docker client config + socket: independent of the user's docker contexts and of any
# credential helper in ~/.docker/config.json.
export DOCKER_CONFIG="${DYX3_DOCKER_CONFIG:-${HOME}/.dyx3-docker}"
export DOCKER_HOST="unix://${HOME}/.colima/${PROFILE}/docker.sock"
mkdir -p "${DOCKER_CONFIG}"
[ -f "${DOCKER_CONFIG}/config.json" ] || echo '{}' >"${DOCKER_CONFIG}/config.json"

die() { echo "ros2_humble: $*" >&2; exit 1; }
command -v colima >/dev/null || die "colima not installed (brew install colima docker)"
command -v docker >/dev/null || die "docker CLI not installed (brew install docker)"

runtime_up() {
  if ! colima status -p "${PROFILE}" >/dev/null 2>&1; then
    echo "starting colima profile ${PROFILE}"
    colima start -p "${PROFILE}" --cpu "${DYX3_CPUS:-6}" --memory "${DYX3_MEM_GB:-12}" \
      --disk "${DYX3_DISK_GB:-80}" --arch aarch64 --vm-type vz
  fi
  docker info >/dev/null 2>&1 || die "docker daemon unreachable via ${DOCKER_HOST}"
}

image_up() {
  if ! docker image inspect "${IMAGE}" >/dev/null 2>&1; then
    echo "building ${IMAGE}"
    docker build -t "${IMAGE}" -f "${HERE}/Dockerfile.ros2-humble" "${HERE}"
  fi
}

host_state() {
  HOST_SHA="$(git -C "${REPO}" rev-parse HEAD)"
  # Dirty = tracked modifications or untracked files other than the root-level *.patch files.
  if [ -n "$(git -C "${REPO}" status --porcelain | grep -v -E '^\?\? [^/]+\.patch$' || true)" ]; then
    HOST_DIRTY="uncommitted changes present (included in build)"
  else
    HOST_DIRTY="clean"
  fi
  echo "host source: ${HOST_SHA} (${HOST_DIRTY})"
}

in_container() {
  local tty=()
  [ -t 0 ] && [ -t 1 ] && tty=(-it)
  mkdir -p "${LOGDIR}"
  docker run --rm "${tty[@]}" \
    -v "${REPO}:/host-src:ro" \
    -v "${LOGDIR}:/host-logs" \
    -v "${VOL_MSGS}:/opt/dyx3/px4_msgs" \
    -v "${VOL_WS}:/work" \
    -e DYX3_HOST_SHA="${HOST_SHA}" -e DYX3_HOST_DIRTY="${HOST_DIRTY}" \
    -e COLCON_ARGS="${COLCON_ARGS:-}" -e DYX3_BUILD_JOBS="${DYX3_BUILD_JOBS:-4}" \
    "${IMAGE}" bash /host-src/tools/dev/container_entry.sh "$@"
}

cmd="${1:-}"
shift || true
case "${cmd}" in
  setup)
    runtime_up; image_up; host_state
    docker image inspect "${IMAGE}" --format 'image {{.Id}} arch={{.Architecture}}'
    in_container msgs ;;
  status)
    colima status -p "${PROFILE}" || true
    docker images "${IMAGE}" 2>/dev/null || true
    docker volume ls --filter "name=dyx3-" 2>/dev/null || true ;;
  sync)       runtime_up; image_up; host_state; in_container sync ;;
  build)      runtime_up; image_up; host_state; in_container msgs; in_container build ;;
  test)       runtime_up; image_up; host_state; in_container test ;;
  build-test) runtime_up; image_up; host_state; in_container msgs; in_container build-test ;;
  test-pkg)
    [ "$#" -gt 0 ] || die "usage: test-pkg <package>..."
    runtime_up; image_up; host_state
    COLCON_ARGS="--packages-select $*" in_container build-test ;;
  shell)      runtime_up; image_up; host_state; in_container shell ;;
  exec)       runtime_up; image_up; host_state; in_container exec "$@" ;;
  stop)       colima stop -p "${PROFILE}" ;;
  *) die "usage: $0 {setup|status|sync|build|test|build-test|test-pkg <pkg>...|shell|exec <cmd>|stop}" ;;
esac
