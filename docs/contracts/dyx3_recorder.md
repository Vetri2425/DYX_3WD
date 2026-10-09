# dyx3_recorder — contract

**Status:** draft for review, written before the implementation. **Spec:** V1 §7.9, §5.4.3, §4 (service plane). **Authority:** none over motion. The
recorder only observes; it publishes `RecorderStatus` and nothing else, runs as its own service (`dyx3-recorder`) and keeps recording when the backend dies.
*A run without provenance is not evidence* (§7.9): the run directory is built so that a missing piece of provenance is **recorded as missing**, never silently absent.

## 1. Run directory

```text
<runs_dir>/<YYYY-MM-DD_HHMMSS>_mission_<id:04d>[_run<n>]/     (UTC, from the wall clock at start)
├── rosbag2/                  # produced by a supervised `ros2 bag record` child (sqlite3 WAL, a split every 300 s, closed splits zstd-compressed: *.db3.zstd)
├── ulog/stream.ulg          # cached ULog header + whole messages from /dyx3/ulog_chunk (section 3); ulog/gaps.json: header status + every missing chunk range
├── manifest.json             # run id, mission id/index, path artifact sha256, vehicle, operator, host, start time, FCU timesync at start
├── versions.json             # copy of the installer's versions file (stack SHA, px4_msgs SHA, firmware SHA, overlay hash) or {"status":"unavailable",...}
├── params_ros.json           # every ROS parameter of the configured nodes: "start" and "end"
├── params_fcu.json           # FCU parameters read live, or {"status":"unavailable","reason":...}
├── config_snapshot/          # copy of the config directory with secrets excluded
└── summary.json              # end time, final mission state, bytes, ulog gaps, bag health, provenance_complete, notes[]
```

Directory names are collision-free (a numeric suffix is appended if the name exists). Files are written atomically (temp + rename) except the bag/ulog streams.

## 2. Lifecycle (driven only by `MissionState`)

* **Pre-roll (REC-004).** A run **opens at `READY`** (the rover is still stopped), so the bag is already writing when motion is allowed; a `RUNNING`
  message with no open run (e.g. the recorder restarted mid-mission) also opens one (`start_state` RUNNING, note "no pre-roll"). The first `RUNNING`
  of the open run is recorded in `summary.json` as `running_utc` and `preroll_s` (bag time before motion). **Stop** when a run is open and the state is
  `COMPLETED`, `ABORTED`, `ERROR` or `IDLE`. A run that never reached `RUNNING` closes as **`NOT_STARTED`** (with a note naming the state that closed it).
  `PAUSED`, `LOADING` and `READY` of the same run do not stop it. A `READY` or `RUNNING` message with a **different `mission_id` or `run_index`**
  (DERIVED: one run directory per run of a mission) while recording closes the old run (`SUPERSEDED`, or `NOT_STARTED` if it never ran) and opens a new one.
* `record_idle` (default false) is not implemented: recording outside missions is an **open question**.
* Start order: directory -> manifest -> versions -> config snapshot -> `params_fcu.json` -> ulog -> bag -> `params_ros.json` start **collected
  on its own thread after the bag runs** (parameter RPCs never delay the bag). Stop joins that thread first, so its notes belong to the run. A step that fails is
  recorded in `summary.notes` and clears `provenance_complete`; only a bag that cannot be started sets `RecorderStatus.state = ERROR`.
  The recorder never blocks, delays or gates the mission: if it is dead or in ERROR the mission runs on, and the absence of evidence is itself visible (`RecorderStatus`, summary).
* Stop order: bag finalised (SIGINT to its process group, wait `bag_finalize_timeout_s`, then SIGTERM, then SIGKILL) -> ulog closed (`gaps.json`) -> `params_ros.json` end -> `summary.json`.

## 3. ULog reassembly

`UlogChunk.msg_sequence` is a wrapping `uint16`. A chunk equal to the previous sequence is a duplicate (dropped, counted); a jump of `n` missing
chunks (modulo 65536) is a **gap** (recorded with sequence numbers, the byte offset in the file at which it occurred, and `first_message_offset` of the chunk after it); a "negative" jump (> 32768) is out-of-order (dropped, counted).

**Header cache (REC-005).** The FCU sends LOGGING_START's stream once (at link-up), and only its beginning carries the 16-byte ULog file header and
the definitions section (`B`, `F`, `I`, `M`, `P`, `Q` messages, up to the first message of another type), followed by the subscriptions (`A`). A run
opens mid-stream, so the recorder is fed **every** chunk, run or not, and keeps in memory (bounded, 8 MiB): the header + definitions, every `A` not
removed by an `R`, and the latest data-section `P` change per parameter. A chunk with the ULog magic at `first_message_offset` 0 starts a stream
(the cache is reset). Every run's `ulog/stream.ulg` begins with that cache and then continues with **whole messages only** (the stream is cut at the
3-byte message headers; after a gap the parser resynchronises at the next chunk's `first_message_offset`), so it is a decodable ULog file. If the
stream restarts during a run (px4_link re-sent LOGGING_START), the run rolls to `stream_2.ulg` with the new header. If no stream start was seen
since the recorder started (or a gap hit the definitions), the file has no header: `summary.json` `ulog_header` and `gaps.json` `header` say
`incomplete: no header …` / `incomplete: header lost …` with a note. The streamed log is best-effort: MAVLink FTP / the SD card stays the recovery path (§5.4.3).

## 4. Provenance sources (and what is NOT available)

| File | Source | Gap |
|---|---|---|
| `versions.json` | `versions_file` (default `/etc/dyx3/versions.json`), written by the installer / `dyx3-version` (P11) | file format is defined by P11; missing -> "unavailable" |
| `params_ros.json` | `SyncParametersClient` per node in `param_nodes` (a helper node on its own executor); values stored as `{type, value-as-string}`; doubles and double arrays use `%.17g` (exact round trip, PC-8 precision), other types `rclcpp::to_string` | nodes that do not answer within `param_timeout_s` are listed `"reachable": false` with a `"note"` (no service, a list/get timeout or exception, or fewer values than names — `get_parameters` returns an empty vector on a timeout). The collector never throws out of the node (a discovered but stalled node used to crash the recorder) |
| `params_fcu.json` | **no FCU parameter read path exists in this stack yet** (DDS does not carry parameters; MAVLink is the service plane) | recorded as unavailable with this reason. **OPEN (human):** how to read FCU parameters live |
| timesync (`manifest.json` / `summary.json`) | `Px4LinkStatus.timesync_*` (interfaces 0.7.0), recorded at run start and end; a sample older than 1 s is recorded as `timesync_valid: false` (never as a number) with a note | F-tasks A1.4: the offset can be ~40 ms for minutes after boot; no gate consumes it (OPEN, see `dyx3_px4_link.md`) |
| `config_snapshot/` | copy of `config_dir` (default `/etc/dyx3/config`) excluding `*.env`, anything named `*secret*`, `*token*`, `*password*`, `*.key`, `*.pem`, `ntrip*` | secrets never reach a run directory (the run dir may be copied around) |

## 5. Parameters (all RESTART; none affects motion)

`runs_dir` `/var/lib/dyx3/runs` · `versions_file` · `config_dir` · `vehicle_id` `unknown` · `operator` `unknown` · `topics` (list of recorded topics; default = the `/dyx3/**` set) · `param_nodes` (default: every node of `dyx3_bringup/launch/control_graph.launch.py` GRAPH — `dyx3_mission`, `motion_guard`, `px4_link`, `rpp`, `spray`, `system_gateway` — plus the separate services `gnss_rtk`, `spray_watchdog` and `recorder` itself; `recorder_node_test` parses the launch file and fails if a graph node is missing) ·
`bag_storage` `sqlite3` · `bag_storage_preset` `resilient` (REC-021: WAL + `synchronous=NORMAL` instead of an in-memory journal + `synchronous=OFF`) ·
`bag_compression_mode` `file` · `bag_compression_format` `zstd` · `bag_compression_threads` 1 · `bag_max_duration_s` 300 (split; after a power cut only the
last split is uncompressed, and it is a valid WAL database). These options are appended to `bag_command` before the topics. Measured in the dev
container (22 topics at production rates, 60 s, every float field noisy): default sqlite3 153 MB/h, these defaults **60 MB/h** (zstd per message: 108 MB/h).
The payload alone compresses to ~30 MB/h; the rest is the sqlite row/index overhead. **OPEN (owner):** for < 50 MB/h install
`ros-humble-rosbag2-storage-mcap` and set `bag_storage: mcap`, `bag_storage_preset: zstd_small`, `bag_compression_mode: none` (chunk compression; not measured here) ·
`bag_finalize_timeout_s` 10 (DERIVED: the last split is compressed while finalising) · `param_timeout_s` 2 (DERIVED) · `status_hz` 2 (DERIVED) · `min_free_bytes` **0 = no check** (no source for a threshold; **OPEN**: a full disk is detected by the bag
process dying, not predicted) · `bag_stall_s` **0 = off** (rosbag2's sqlite file does not grow every second; no source).
**OPEN:** whether to also bag the raw `/fmu/out/**` topics (default: not recorded; the `/dyx3/vehicle_state` fan-out is).

## 6. Acceptance

Off-target: manifest/summary JSON (escaping, determinism, non-finite -> null), run naming and collision, config snapshot secret exclusion, ULog reassembly (wrap, duplicate, gap, out-of-order),
bag supervision against a fake child (start, finalise on SIGINT, escalation on a child that ignores SIGINT, death detected), the lifecycle table, a node test with a fake bag command and an injected clock.
**Not provable off-target:** `ros2 bag record` itself (rosbag2 is not installed in CI), disk-full behaviour, the systemd unit, real FCU ULog bytes.
