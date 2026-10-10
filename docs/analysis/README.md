# docs/analysis

Analyses of recorded field data, and how to reproduce their numbers.

| Document | What it is |
|---|---|
| `stage0.md` | Stage 0 tooling (architecture §11, §4.6): the arc-floor and timebase tools in `tools/analysis/`. |
| `2026-10-10_last_two_missions_controller_robustness.md` | Missions 0007 and 0001 of 2026-10-10: tracking, rates, latency, corner/endpoint/drivetrain findings. |
| this file | How to compute the gate and field-ladder numbers from a recorder bag with `tools/analysis/mission_bag_metrics.py`. |

---

## `tools/analysis/mission_bag_metrics.py`

One command turns a recorder run (or a whole mission) into the numbers the gate table (architecture §10) and the
field ladder ask for. It has no ROS dependency: it reads the rosbag2 sqlite3 file directly and decodes every
`dyx3_interfaces/*` message from CDR using the `.msg` definitions. Python 3.10+, `numpy`; `zstandard` (or the
`zstd` command) only for `.db3.zstd` files. `rosbags` installs `zstandard`, so the Mac set-up from `stage0.md` is enough.

### Running it

```bash
# one recorder run
python3 tools/analysis/mission_bag_metrics.py 3WD_PROD/Bags/2026-10-10/2026-10-10_142612_mission_0001_run3

# a directory of runs: a whole day, or one mission's runs. Runs are grouped into missions by manifest.json
# mission_id and start order; each mission is computed over its runs merged in time.
python3 tools/analysis/mission_bag_metrics.py 3WD_PROD/Bags/2026-10-10

# JSON on stdout instead of Markdown; do not write metrics.json
python3 tools/analysis/mission_bag_metrics.py --json --no-write <run-or-dir>
```

Input layout (what `dyx3_recorder` writes): `<run>/rosbag2*/rosbag2_N.db3` or `.db3.zstd` (every split and every
`rosbag2_K` bag restart is read), `manifest.json`, `summary.json`, `versions.json`. If a `.db3` sits next to its
`.db3.zstd`, the plain file is used. Message-mode compression is handled when `zstandard` is installed.

Output:
- Markdown report on stdout (cross-track in cm, heading in degrees, latency in ms).
- `<run>/metrics.json` in every run directory: what that bag alone says.
- `<dir>/metrics.json` when the input is a directory of runs: `{"missions": [...]}`, one entry per mission.
  JSON values are SI (m, rad, s, Hz) unless the key says `_ms`, `_us` or `_cm`. Unknown values are `null`.

**Message definitions.** A bag must be decoded with the definitions of the stack that recorded it, since
interfaces only append fields. By default the tool reads `versions.json` `stack_sha` and loads the definitions at that
commit with `git show` (read-only). If this checkout does not have that commit, it falls back to the working tree.
`--stack-sha <sha>` or `--msgdir <dir>` force a set. A bag older than the definitions still decodes: trailing fields it
lacks are reported as missing (`null`). A bag newer than the definitions (bytes left over) is reported as a decode
warning at the top of the report, not silently accepted. Topics of other packages (`rcl_interfaces`, `px4_msgs`) are skipped.

### What each metric means

Times are seconds from the first message of the data. "Receipt" is the rosbag2 timestamp (the recorder's clock when
it received the message). That is not the time the controller received it.

**1. Rates** (per bag): for `/dyx3/vehicle_state` (receipt, and `px4_sample_stamp`, i.e. PX4's EKF2 sample cadence),
`/dyx3/rpp/motion_setpoint`, `/dyx3/motion_guard/command`, `/dyx3/px4_link/status`: n, mean Hz over the span, the
instantaneous rate 1/dt (min/p1/p50/p99/max) and the largest gap in ms. Zero or negative steps are counted
(`nonpositive_steps`) and left out of the instantaneous statistics.

**2. Chain latency and loop health** (per bag): `Px4LinkStatus.pose_to_write_age_s` p50/p95 and the maximum of
`pose_to_write_age_max_s`. These use only samples with `pose_to_write_age_valid`. The age runs from the PX4 sample, through
RPP and the guard, to the write to PX4. RPP `loop_jitter_max_us` (max) and `loop_overrun_count` (last value, a lifetime
counter, plus its change over the bag). px4_link `loop_overrun_count`, `command_gap_events` and `session_resets` (last and
change), plus the `fault` histogram by name.

**3. Tracking** (grouped by `RppStatus.run_index`, the controller's own run, which is not always the bag). For each RPP state
and for TRACKING+CREEP: |cross-track| p50/p95/max, signed mean, and |heading error| p50/p95/max.
- *STEADY*: TRACKING samples whose commanded speed is ≥ 0.8 × that run's maximum commanded speed, split into contiguous
  segments. For each segment: mean, std, peak-to-peak, zero crossings (sign changes of the signed cross-track) and the
  dominant period (largest non-DC peak of the mean-removed, resampled, Hann-windowed spectrum). Heading std, p2p and zero
  crossings are given too. If the period is about the segment length or longer, the segment has no oscillation: the error is
  a slow drift (as in the 2026-10-10 straights).
- *Corner entry*: the first 0.5 m of `path_travel_m` of every TRACKING interval. `after_pivot` marks the ones that follow a pivot.
- *Braking tail*: the last 1.0 m of `path_travel_m` before a TRACKING interval hands over to STOPPING (a corner). The
  final leg hands over to CREEP and is covered by the endpoint section.

**4. Pivot**: an episode starts at a PIVOTING interval and runs over the PIVOTING/STOPPING intervals after it, up to the
next other state. *cycles* = the number of PIVOTING intervals in the episode, so a clean pivot is 1 and each re-pivot
after a settle STOPPING adds 1. *duration* = from the first PIVOTING entry to the last PIVOTING exit. The heading error at
release is the last PIVOTING sample's. Episodes cut by the start or end of the data are flagged. The
`pivot_timed_out` flag is shown when the bag has it. *Full-rate yaw ratio*: over PIVOTING samples with |commanded
yaw rate| ≥ 0.4 rad/s, measured `VehicleState.yaw_rate_radps` (interpolated at the RppStatus receipt time) divided by
the commanded rate. It is given for the whole data and for each episode, because one stalled pivot pulls the pooled
median to 0 (mission 0001).

**5. Endpoint** (the last CREEP of the data, i.e. the last run): the CREEPING/STOPPING block that holds the last CREEP.
It reports CREEP and STOPPING durations, sign reversals of `commanded_speed_mps` (every + to − or − to + change, zeros
skipped) with the forward→reverse subset, max measured horizontal speed in the 6 s before the block ends, final
`dist_to_goal_m` (when the bag has the field), and *finished_by_timeout* = CREEP ≥ 7.9 s (the precise-stop timeout is 8 s).

**6. Mission**: `MissionState` transitions with time, reason code, detail and `waiting_on`. Also: `PointResult` per
point (`error_m`); the guard's reason histogram, clamp count and refusals (`/dyx3/motion_guard/status`); the
safety-gate ok/not-ok histogram with reasons while not ok (and the same for pre-arm); the RTK fix histogram, max
`horizontal_accuracy_m` and max correction age; and the estimator fault flags that were ever set and healthy flags
that were ever false, with max test ratios.

**7. Gate-table rows from the bag alone**: RMS, p95 and max of |`RppStatus.cross_track_right_m`| over TRACKING+CREEP for
the whole data. **This is the controller's own cross-track** (rover against the path RPP holds, from the EKF pose). It is
the closest bag-only stand-in for `tracking.overall`, but it is not that row.

### What it does NOT provide

| Gate / ladder row | Why not | Where it comes from |
|---|---|---|
| Full-mission RMS / p95 / max **against surveyed truth** (`tracking.overall`) | needs the surveyed points / an independent truth, not the EKF | survey comparison (GNSS vs surveyed points, HANDOFF 2026-10-10) |
| Shape RMS @0.35 m/s (arc / lshape / square / U-turn) **vs truth** | same; the bag gives only RPP's own cross-track per run | survey comparison |
| Pivot wobble radius (median) and pivot net walk | need the antenna or rotation-centre trajectory at full rate through the pivot | PX4 SD ULog (`bench_tools/ulog_pull.py`, `pyulog`) |
| Traversal coverage | needs the path artifact against the driven trace | path artifact + trace (not implemented here) |
| Spray boundary loss | needs the painted line / valve timing against the boundary geometry | spray evidence (not implemented here) |
| What PX4 receives at 100 Hz (`offboard_control_mode`, `/fmu/in/*`) | `/fmu/**` is not bagged; the SD logger decimates to 10 Hz | analysis 2026-10-10 §1c |
| Drivetrain health (RoboClaw errors, wheel speeds) | PX4-side only | PX4 SD ULog |

### Definitions that differ from the 2026-10-10 analysis

The 2026-10-10 analysis used scratch scripts. The tool fixes the definitions above, and three of them count differently:
- STEADY there was "commanded speed ≥ 0.5 m/s". Here it is ≥ 0.8 × the run's max commanded speed (0.48 m/s at 0.6 m/s).
- Endpoint reversals: the analysis quotes 14 for mission 0001 run 3. The tool counts 28 sign changes in the endpoint block
  (26 inside CREEP), of which 14 are forward→reverse. The analysis's number is the forward→reverse count.
- Pivot cycles: the number of PIVOTING intervals per episode (1 = no hunting).

On the two 2026-10-10 missions the tool reproduces the analysis's figures. On `7f9651d`: pose→write p50 3.2–3.5 ms,
p95 4.1–4.5 ms, max 14.5 ms, 1 RPP overrun. On `172b047`: p95 10.3–11.2 ms, max 25.3 ms, 32 overruns. Point errors
0.15 / 2.74 / 1.92 / 1.80 cm. Corner entries +0.9 / +1.5 / −2.4 / −2.8 cm. Endpoint CREEP 8.00 s, timeout finish (0001),
against CREEP 0.36 s + STOPPING 7.64 s (0007). The 26.4 s stalled first pivot of 0001. Full-rate pivots at 0.97–0.99 of
the command on 0007.
