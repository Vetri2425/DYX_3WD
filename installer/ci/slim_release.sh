#!/usr/bin/env bash
# DYX 3WD — slim a BUILT release tree before CI packages it. Source, do not execute.
# shellcheck shell=bash
#
# Used by build_rover_artifacts.sh only, never on the rover: a release built on the rover (build_release) is not
# touched. Every step removes or rewrites only what nothing on the rover reads at install or run time:
#   slim_release_tests   ros2_ws/src/<pkg>/test(s) and backend/tests: test sources and fixtures. The rover runs the
#                        colcon install tree and the backend installed in the venv, never these. package.xml,
#                        CMakeLists.txt, launch/, config/ and every other source file stay.
# The caller re-runs the installer's static verification on the slimmed tree before packaging it.

# slim_release_tests <release-dir>: drop test sources and fixtures; prints what was removed.
slim_release_tests() {
  local rel="$1" d
  [ -f "${rel}/ros2_ws/install/setup.bash" ] || die "slim_release_tests: ${rel} is not a built release"
  for d in "${rel}"/ros2_ws/src/*/test "${rel}"/ros2_ws/src/*/tests "${rel}/backend/tests"; do
    [ -d "${d}" ] && [ ! -L "${d}" ] || continue
    printf '%s\n' "${d#"${rel}"/}"
    rm -rf -- "${d}"
  done
}
