# Handoff prompt — DYX 3WD, bench day 2026-10-09 and field demo 2026-10-10

Paste this into a new Claude session. It is self-contained.

You are continuing work on the **DYX 3WD road pre-line marking rover**:
- Holybro Pixhawk Jetson Baseboard with a Pixhawk 6X and a Jetson Orin Nano;
- UM982 GNSS on TELEM1, RoboClaw on GPS2, spray on FMU PWM OUT 1;
- Jetson↔PX4 over Ethernet with uXRCE-DDS.

The owner is at the office with the rover from 08:00. Saturday 2026-10-10 is the field demo: **driving only, no
paint**, unless the valve-close test passes.

## Read first, in order
1. `~/Vetri/3WD_PROD/DYX_3WD/CLAUDE.md`: rules, §3b current status, §4 rules. Especially: **every hardware fix
   must persist in the repo**.
2. `~/Vetri/3WD_PROD/DYX_3WD/docs/agents/HANDOFF.md`: the entries dated 2026-10-08 and 2026-10-09.
3. `~/Vetri/3WD_PROD/BENCH_2026-10-09.md`: **today's runbook**. Follow it in order.
4. Your memory: `~/.claude/projects/-Users-dyx-a1-Vetri-3WD-PROD/memory/MEMORY.md`.

## Owner rules (binding)
- **One authoritative branch per repo:** DYX_3WD `master`; firmware `dyx-3wd-production`; app Three_Wheel_v2 `main`.
  - Agent work (Codex, Agy) is merged only after Claude has reviewed it and the checks pass.
  - **Never delete a branch unless the owner asks.**
- **Firmware `8279fa4be3` is the V1 final candidate.** No firmware work after today's bench validation, unless the
  bench finds a defect.
- **Every fix made on the rover lands in the repo the same day** (installer, unit, template, `config/px4/` params, or
  a documented step) and is recorded in HANDOFF. No temporary hand edits.
- Commits: no AI attribution lines; trailer `Agent: Claude`. Never force-push. Pushing straight to `master` is
  allowed. Deploy only when the owner asks. Never commit secrets (NTRIP, Wi-Fi PSK, tokens); document how to
  create them instead.
- PX4 never configures the UM982; the Jetson writes only CRC-valid RTCM3 to it.
- Don't share the DYX_3WD clone with a running agent. Edit in your own `git worktree`.

## State at handoff (2026-10-09 ~02:00 IST)

| What | Ref | Artifact | State |
|---|---|---|---|
| Rover stack | DYX_3WD `master` @ **`8af2595`** | GitHub release `rover-8af2595…` | CI was building at handoff; check `gh run list --branch master` and `gh release list` |
| Rover stack fallback | `ad58e72` / `cb8ea42` | `rover-ad58e72…`, `rover-cb8ea42…` | Published. `cb8ea42` = RTK only, no app gaps |
| **Firmware V1 candidate** | `dyx-3wd-production` @ **`8279fa4be3`** | `~/Vetri/3WD_PROD/PX4-Firmware/3WD/8279fa4be3-…/px4_fmu-v6x_rover.px4`, sha256 `21288e6d…` | CI green, **not flashed** |
| Firmware fallbacks | `4393fb07e1` (no WENC fix), `9ab2ad3162` (currently on the FCU) | archived next to it | — |
| NuttX fork | `Vetri2425/NuttX` `dyx-3wd-production` @ `e462af8eb3` | — | stall-fix backport |
| **App** | Three_Wheel_v2 **`main` @ `dbb2ba1`** (fast-forwarded from `agy/prod-transport` on 2026-10-09 08:30) | `~/Vetri/3WD_PROD/App-Releases/dbb2ba1-agy-prod-transport/app-release.apk`, **DYX release-signed**, sha256 `730c6111…` | Reviewed: tsc passes; tests 984/985, the 1 failure pre-exists on Runtime_Path (roadMarkingCsvPath polygon-ring test). Uninstall the old debug-signed build first |

What `master` @ `8af2595` contains (all reviewed by Claude):
- **Production RTK:** NTRIP or LoRa source → USB or PX4 DDS transport. Single transport, **no automatic failover**.
  Config and secrets in `/var/lib/dyx3/rtk/`; control socket and REST `/api/rtk/*`. An upgraded rover keeps
  NTRIP → DDS; a fresh install defaults to NTRIP → USB.
- `POST /api/missions/plan`: app-planned missions, rover contract `docs/contracts/app_planned_mission.md` v1.1.
  - R1 one spray state per run;
  - R2 strict mark/travel alternation;
  - R3 contiguous runs within 1 mm;
  - R4 shared-point encoding.
  It never re-plans.
- `POST /api/path/parse-dxf`: parse only.
- Optional Jetson hotspot at 10.42.0.1, configured from `/etc/dyx3/hotspot.env`: country IN, power save off,
  5 GHz channel 36 by default, DFS refused. Dormant until configured. **Not verified on hardware.**
- Socket.IO ping 5 s / 5 s (DERIVED); a body-size limit on `/missions/plan`.
- Tablet token: `sudo -u dyx3 /opt/dyx3/current/venv/bin/python -m dyx3_backend.auth.tokens create --file
  /var/lib/dyx3/state/auth.json --name tablet-1 --role operator`, then `sudo systemctl restart dyx3-backend`.
- Checks: CI green up to `ad58e72`. Codex locally: backend pytest 569 passed, installer 106/0, ROS 459 tests /
  0 failures.

Firmware `8279fa4be3` = `9ab2ad3162` plus:
- the NuttX STM32H7 TX-ring guard (the stall fix, upstream NuttX `be559984e549`);
- the XRCE fd closed once;
- GPS RTCM partial writes completed and counted (`gps status` counters);
- the wheel-encoder fix: fusion no longer refreshes the global velocity timers (yaw reset and dead reckoning
  detection work again).

`msg/` is unchanged. Not done: the GATE-2 replay and a byte-exact RTCM pty test; the bench decides.

## Today's sequence (details in BENCH_2026-10-09.md)
1. `ssh dyx-3wd`, then `sudo dyx3-upgrade 8af2595`. If its release isn't published, use `ad58e72`. Watch for the
   Ethernet stall: the upgrade restarts the XRCE agent.
2. Stall: reproduce on `9ab2ad3162` (≤ 1 h; captures at the stall and again 75 s later). Flash `8279fa4be3` with
   QGC over USB, custom file. Stress: 60 agent restarts plus FCU and Jetson power cycles.
3. RTK: the 4 combinations, 10 min each with 2 min gaps. Collect the UM982 USB and LoRa by-id names, bauds, and
   whether GGA is output on the USB COM. **Read-only: never configure the UM982.**
4. Wi-Fi:
   - `nmcli device`, `iw reg get`, `lsmod | grep -i rtl`;
   - fill in `/etc/dyx3/hotspot.env` (SSID and PSK are rover-local, not in Git);
   - re-run the installer's network step;
   - range test at 5/10/15/25 m (installer/README.md).
5. Calibration (gyro, level-only accel `PREFLIGHT_CALIBRATION` param5=4, level horizon); Acro wheels-up; one
   mission on the ground; the valve-close test.
6. Go / no-go for Saturday, then write HANDOFF.

## Open work for this session
**A. App: DONE 2026-10-09 08:30** (main @ dbb2ba1, signed APK archived). Remaining: the field test with the rover, and the pre-existing CSV test failure. The original checklist is kept below for reference.

**A (reference). App: finish, validate, merge, sign** (`~/Vetri/3WD_PROD/Three_Wheel_v2`, remote
`yasarbaiiiii-blip/Three_Wheel_v2`; Vetri2425 has push rights).
1. When Agy reports `agy/prod-transport` done (it had uncommitted `App.tsx`, `ModernHomeUI.tsx`,
   `ModernSettingsPage.tsx`), review it:
   - **One** Socket.IO connection, owned by the session.
   - The heartbeat is fixed-rate at 500 ms with ≤ 350 ms request timeout, and runs on every screen while an OPERATOR
     session is connected.
   - Disconnect → status shown immediately; 401 → stop retrying and prompt for the token.
   - AppState resume handled.
   - Staleness shown on every live value (1.0 s stale / 2.5 s disconnected).
   - The mission builder obeys R1–R3; 422 codes are mapped.
   - Spray hidden.
   - No prototype endpoints (port 5001, `X-Rover-Token`, `/api/path/upload|plan-trajectory|load-to-controller`) on
     a production connection.
   - Typecheck, lint and all tests green.
   - Prove it against a local backend from a worktree of DYX_3WD `master`.
2. **Release signing is missing.** Today release builds use `signingConfigs.debug`, and `android/` is git-ignored.
   Give Agy this task: "Add a committed Expo config plugin (e.g. `plugins/withDyxReleaseSigning.js`, registered in
   `app.json` plugins) that injects a `release` signingConfig into android/app/build.gradle at prebuild. It reads
   `DYX_RELEASE_STORE_FILE`, `DYX_RELEASE_STORE_PASSWORD`, `DYX_RELEASE_KEY_ALIAS` and `DYX_RELEASE_KEY_PASSWORD`
   from Gradle properties (they already exist in ~/.gradle/gradle.properties: the DYX key, also used by 4WD). A
   release build fails loudly if any is missing, and never falls back to the debug key. Never commit key material.
   Verify with `npx expo prebuild --clean`, `./gradlew assembleRelease`, `apksigner verify --print-certs` (the
   certificate must not be 'Android Debug')."
   Note: tablets with the debug-signed APK must uninstall it before installing the signed one.
3. When it's all green: merge into `main` (fast-forward or a merge commit; no force), push, build the signed release
   APK, and record its path and SHA-256 in HANDOFF. Don't delete the `agy/*` branches.

**B. Bring the five 2026-10-08 hand edits into the repo** (HANDOFF 2026-10-09 list):
1. `ros.env` domain 42 + localhost-only, as the template default plus a health check against the PX4 parameter;
2. `backend.env` back to 10.42.0.1 once the hotspot works;
3. mavlink-router server mode, after the bench re-test;
4. a documented or scripted step that applies `config/px4/3wd_6x_carry_from_proto.params`;
5. the NTRIP credentials creation step in installer/README.

**C. Housekeeping (owner OK first):**
- remove the firmware worktree `~/Vetri/3WD_PROD/PX4-3WD-ethfix`;
- update the main firmware checkout. It holds the uncommitted WENC edit, identical to `8279fa4be3`; verify with a
  diff, then `git checkout --` those files and fast-forward.
- Branch deletions only when the owner asks.

## Credentials (never put them in files or chat logs)
- The NTRIP "office" profile (Emlid caster) is already in `/etc/dyx3/ntrip.env` on the rover; the RTK worker
  imports it once.
- Wi-Fi SSID and PSK: the owner chooses them; they go only in `/etc/dyx3/hotspot.env` on the rover.
