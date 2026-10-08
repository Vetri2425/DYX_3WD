# Proposal — prebuilt, verified release artifacts (no compiling on the rover)

**Status:** PROPOSED (2026-10-08, claude). Human decision required on §6.
**Spec:** architecture §12 (install/upgrade), CLAUDE.md "next rover = install only".
`DERIVED — NOT FROM V1 SPEC` throughout: V1 describes the release layout, not where the binaries come from.

## 1. Problem (measured on the rover, 2026-10-08)

The first real `dyx3-install` compiles everything on the Jetson:

| Item | Built on rover | Time on Orin Nano |
|---|---|---|
| MicroXRCEAgent v2.4.3 (+ vendored Fast DDS) | yes, `-j1` | ~40 min (killed at 30 %, already present at the pinned commit) |
| mavlink-router v4 | yes | ~5 min |
| `px4_msgs` for the firmware pin (~250 msgs × C/C++/Py/typesupport) | yes, `-j1`/`-j2` (`-j3` stalled the board) | the longest step |
| 11 stack packages | yes, `-j1` | tens of minutes |

Consequences: a fresh rover takes hours; every rover runs binaries compiled by *its own* toolchain run
(no byte-identical fleet); an upgrade in the field needs WAN *and* a long build; memory pressure risk.

Meanwhile CI job `colcon build + test` already compiles the same tree on `ubuntu-24.04-arm` inside
`ros:humble-ros-base` (Ubuntu 22.04 arm64, ROS Humble — the Jetson's ABI) and discards the result.

## 2. Proposal

Build once in CI, ship verified artifacts, install by extraction. Compiling on the rover becomes the
fallback, not the path.

### 2.1 Artifacts (content-addressed by what they are built from)

| Artifact | Key | Contents | Rebuilt when |
|---|---|---|---|
| `third_party-xrce-<XRCE_COMMIT>.tar.zst` | `microxrce_agent.pin` | `cmake --install` tree for `/usr/local` | pin changes |
| `third_party-mavrouter-<MAVROUTER_COMMIT>.tar.zst` | `mavlink_router.pin` | `mavlink-routerd` | pin changes |
| `px4_msgs-<FIRMWARE_SHA>-<SKELETON_REF>.tar.zst` | `firmware.pin` | `/opt/dyx3/px4_msgs/<fw-sha>/install` + `px4_msgs.sha256` | firmware msg set changes |
| `release-<STACK_SHA>.tar.zst` | stack commit | `/opt/dyx3/releases/<sha>/` incl. `ros2_ws/install`, `bin/`, `venv` wheels | every commit on a deploy branch |
| `SHA256SUMS` + `artifacts.json` | — | digest of every file; build provenance (runner image digest, apt package versions, CI run URL) | every publish |

Built at the **same absolute paths** the rover uses (`/opt/dyx3/...`) so colcon/CMake/ament prefix files
need no relocation.

### 2.2 Publication

GitHub Release per stack SHA (`rover-<sha>`), created only by a CI run whose build+test jobs are green.
Third-party and `px4_msgs` artifacts are re-attached (not rebuilt) when their key is unchanged.

### 2.3 Installer (`DYX3_ARTIFACTS=auto|prebuilt|source`, default `auto`)

1. Resolve the stack SHA exactly as today.
2. Download `artifacts.json`, `SHA256SUMS` and the needed artifacts — or read them from a local directory
   (`DYX3_ARTIFACT_DIR`, e.g. a USB stick for sites without WAN).
3. Verify every digest; refuse on mismatch (no partial extract).
4. Extract into `releases/<sha>.partial`, run the existing static verification, write `.complete`, switch.
5. `auto`: on missing artifact → today's source build (unchanged code path). `prebuilt`: fail instead.
6. Record artifact digests + CI run URL in `/etc/dyx3/versions.json` and the recorder run manifest.

Expected: fresh rover = apt + ROS + downloads + extract — **minutes**; upgrade = one release tarball.

## 3. Compatibility guard

The rover checks before extracting: `ID=ubuntu VERSION_ID=22.04`, `aarch64`, ROS `humble`, and that the
apt versions of the ROS core libraries the release links against match `artifacts.json` within the same
Humble sync (mismatch → `auto` falls back to a source build, `prebuilt` fails). L4T/JetPack is not in the
link path of these packages (no CUDA), which is why the CI container is a valid build host.

## 4. Tests / acceptance

- CI: an `artifacts` job extracts the release into a clean `ros:humble-ros-base` arm64 container and runs
  `dyx3-health` static checks + `ros2 pkg list` for the manifest packages.
- Installer staged-root tests: digest mismatch refused; missing artifact → fallback in `auto`, failure in
  `prebuilt`; offline `DYX3_ARTIFACT_DIR`; `versions.json` carries digests.
- Rover acceptance: wipe `/opt/dyx3`, `dyx3-install --production` completes in < 15 min on the office LAN
  with zero compiler invocations; graph identical to a source-built release (`ros2 node list`, `/fmu` topics).

## 5. Not in scope

Signing beyond TLS + digests (see §6.2); OTA scheduling; the hotspot/LTE network profiles.

## 6. Human decisions

1. Which branches publish rover artifacts (proposed: `master` and `claude/cloud-phases` while it is the
   deploy branch).
2. Integrity: digests over GitHub TLS (proposed for now), or add a signing key on each rover
   (minisign/cosign) — stronger, but a key to manage.
3. Retention: keep the last N release artifacts (GitHub storage is free for public repos; proposed N = 20).
