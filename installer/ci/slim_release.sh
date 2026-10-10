#!/usr/bin/env bash
# DYX 3WD — slim a BUILT release tree before CI packages it. Source, do not execute.
# shellcheck shell=bash
#
# Used by build_rover_artifacts.sh only, never on the rover: a release built on the rover (build_release) is not
# touched. Every step removes or rewrites only what nothing on the rover reads at install or run time:
#   slim_release_tests   ros2_ws/src/<pkg>/test(s) and backend/tests: test sources and fixtures. The rover runs the
#                        colcon install tree and the backend installed in the venv, never these. package.xml,
#                        CMakeLists.txt, launch/, config/ and every other source file stay.
#   slim_release_venv    pip, setuptools (with pkg_resources, _distutils_hack) and wheel, removed with pip's own
#                        uninstall. They only install packages; the rover never installs into a prebuilt venv
#                        (build_backend_venv runs only when the rover builds a release itself, into a fresh venv).
#   strip_release_binaries  the ELF files in ros2_ws/install lose their symbol tables (strip --strip-unneeded: the
#                        dynamic symbols that the loader, dlopen/dlsym and the rosidl typesupport use are kept). The
#                        removed symbols go to <debug-dir>/.build-id/xx/yyyy.debug for crash analysis; the build-id
#                        note is kept in the stripped file, so a core dump or backtrace finds its symbols.
# The caller re-runs the installer's static verification on the slimmed tree before packaging it.

# Removed from the shipped venv. Nothing in the backend or its dependencies imports them (checked 2026-10-10).
DYX3_VENV_BUILD_ONLY_PKGS=(setuptools wheel pip)
DYX3_VENV_BUILD_ONLY_MODULES=(pip setuptools pkg_resources _distutils_hack wheel)

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

# _venv_importable <python> <site-packages>: print every module the venv can import, one per line, sorted: each
# top-level module in site-packages and every submodule of dyx3_backend (the backend's own import graph, path engine
# included). Build-only modules are left out, so a before/after comparison measures only what the rover uses.
_venv_importable() {
  local py="$1" sp="$2"
  # Imports may print; only the "@@ " lines are the answer. -B: the check must not add .pyc files to the release.
  DYX3_SLIM_SKIP="${DYX3_VENV_BUILD_ONLY_MODULES[*]}" "${py}" -I -B - "${sp}" <<'PY' 2>/dev/null | sed -n 's/^@@ //p'
import importlib
import os
import pkgutil
import sys

skip = set(os.environ["DYX3_SLIM_SKIP"].split())
names = sorted({m.name for m in pkgutil.iter_modules([sys.argv[1]])} - skip)
ok = []
for name in names:
    try:
        mod = importlib.import_module(name)
    except BaseException:
        continue
    ok.append(name)
    if name == "dyx3_backend":
        for sub in pkgutil.walk_packages(mod.__path__, "dyx3_backend.", onerror=lambda _n: None):
            try:
                importlib.import_module(sub.name)
            except BaseException:
                continue
            ok.append(sub.name)
for name in sorted(ok):
    print("@@ " + name)
PY
}

# slim_release_venv <release-dir>: uninstall the packaging tools from <release>/venv. No venv (the backend build
# failed or was skipped): nothing to do. Dies when anything the venv could import before is no longer importable.
slim_release_venv() {
  local rel="$1" venv="$1/venv" py sp before after left
  py="${venv}/bin/python"
  [ -x "${py}" ] || {
    warn "no backend venv in ${rel}; nothing to slim"
    return 0
  }
  sp="$("${py}" -I -c 'import sysconfig; print(sysconfig.get_paths()["purelib"])')" || die "venv python does not run"
  before="$(_venv_importable "${py}" "${sp}")"
  local present
  present="$("${py}" -I -c 'import importlib.metadata as md, sys
for n in sys.argv[1:]:
    try:
        md.distribution(n)
        print(n)
    except md.PackageNotFoundError:
        pass' "${DYX3_VENV_BUILD_ONLY_PKGS[@]}" | tr '\n' ' ')" || die "venv python does not run"
  if [ -n "${present// /}" ] && "${py}" -I -m pip --version >/dev/null 2>&1; then
    # shellcheck disable=SC2086  # word-split on purpose: one argument per installed package
    PIP_DISABLE_PIP_VERSION_CHECK=1 PIP_NO_INPUT=1 PIP_ROOT_USER_ACTION=ignore \
      "${py}" -I -m pip uninstall --yes --quiet ${present} || die "pip uninstall of ${present}failed"
  fi
  left="$("${py}" -I -c 'import importlib.util, sys
print(" ".join(m for m in sys.argv[1:] if importlib.util.find_spec(m)))' "${DYX3_VENV_BUILD_ONLY_MODULES[@]}")"
  [ -z "${left}" ] || die "still importable after the uninstall: ${left}"
  left="$(find "${venv}/bin" -maxdepth 1 \( -name 'pip*' -o -name 'easy_install*' -o -name 'wheel' \) -print)"
  left+="$(find "${sp}" -maxdepth 1 -name 'distutils-precedence.pth' -print)"
  [ -z "${left}" ] || die "packaging tool files left in the venv: ${left}"
  after="$(_venv_importable "${py}" "${sp}")"
  left="$(LC_ALL=C comm -23 <(printf '%s\n' "${before}") <(printf '%s\n' "${after}"))"
  [ -z "${left}" ] || die "no longer importable after removing ${DYX3_VENV_BUILD_ONLY_PKGS[*]}: ${left}"
  log "venv: removed ${present:-nothing}; $(printf '%s\n' "${after}" | grep -c .) modules import as before"
}

# _elf_kind <file>: "exec" or "dyn" for an ELF executable / shared object, nothing otherwise (objects, archives,
# scripts and data are never stripped).
_elf_kind() {
  [ "$(od -An -tx1 -N4 -- "$1" 2>/dev/null | tr -d ' \n')" = 7f454c46 ] || return 0
  case "$(od -An -tu1 -j16 -N1 -- "$1" | tr -d ' \n')" in
    2) printf exec ;;
    3) printf dyn ;;
  esac
}

# _build_id <elf>: the GNU build-id as hex, or nothing. (Whole output read, no early-exit `head`: callers run under
# pipefail.)
_build_id() {
  local ids
  ids="$("${READELF:-readelf}" -n -- "$1" 2>/dev/null | sed -n 's/^[[:space:]]*Build ID: \([0-9a-f]*\)$/\1/p')" || true
  printf '%s' "${ids%%$'\n'*}"
}

# strip_release_binaries <release-dir> <debug-dir>: strip every ELF executable and shared object under
# ros2_ws/install, keeping its symbols in <debug-dir>/.build-id/<xx>/<rest>.debug. <debug-dir>/BUILD_IDS lists
# "<build-id> <path relative to the release>". Dies on an ELF without a build-id (its symbols could not be found
# again) or when stripping changed a build-id.
strip_release_binaries() {
  local rel="$1" dbg="$2" f id rid kind n=0 before=0 after=0
  local objcopy="${OBJCOPY:-objcopy}" strip="${STRIP:-strip}"
  [ -d "${rel}/ros2_ws/install" ] || die "strip_release_binaries: ${rel}/ros2_ws/install missing"
  mkdir -p "${dbg}/.build-id"
  : >"${dbg}/BUILD_IDS"
  while IFS= read -r -d '' f; do
    kind="$(_elf_kind "${f}")"
    [ -n "${kind}" ] || continue
    id="$(_build_id "${f}")"
    [ -n "${id}" ] || die "no build-id in ${f#"${rel}"/}; its debug symbols could not be matched"
    before=$((before + $(wc -c <"${f}")))
    mkdir -p "${dbg}/.build-id/${id:0:2}"
    "${objcopy}" --only-keep-debug -- "${f}" "${dbg}/.build-id/${id:0:2}/${id:2}.debug" ||
      die "objcopy --only-keep-debug failed for ${f#"${rel}"/}"
    "${strip}" --strip-unneeded -- "${f}" || die "strip failed for ${f#"${rel}"/}"
    rid="$(_build_id "${f}")"
    [ "${rid}" = "${id}" ] || die "strip changed the build-id of ${f#"${rel}"/}"
    after=$((after + $(wc -c <"${f}")))
    printf '%s %s\n' "${id}" "${f#"${rel}"/}" >>"${dbg}/BUILD_IDS"
    n=$((n + 1))
  done < <(find "${rel}/ros2_ws/install" -type f -print0 | LC_ALL=C sort -z)
  log "stripped ${n} ELF files in ros2_ws/install: ${before} -> ${after} bytes; symbols in ${dbg}"
}
