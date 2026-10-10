# Parameter profiles — how values reach a rover and how a tuned value is promoted

**Spec:** V1 section 9 (profiles `production.yaml`, `precision.yaml`, `development.yaml`, promoted by **explicit
save**; a live tweak never silently rewrites production config) and section 12 (`dyx3-param get|set|save` via the real
ROS parameter authority). **Status (2026-10-10):** the path exists; only one node (`rpp`) has a profile file, and its
two values equal the built-in defaults. Nothing here has run on the rover yet.

## 1. Where values come from

A node starts with its **built-in defaults** (the C++ `declare_parameter` defaults; for `rpp` and `spray` the tables
generated from `docs/tuning/parameter_registry.md`) and then applies its parameter file **only when that file exists**:

| Node | File read at start | Read by |
|---|---|---|
| `dyx3_mission`, `motion_guard`, `px4_link`, `rpp`, `spray`, `system_gateway` | `/etc/dyx3/<stem>.yaml`, stems `mission`, `motion_guard`, `px4`, `rpp`, `spray`, `gateway` | the graph launch (`dyx3_bringup/launch/control_graph.launch.py`, `plan()`: `<config_dir>/<stem>.yaml` is passed only if it is a file; `config_dir` defaults to `/etc/dyx3`) |
| `recorder` | `/etc/dyx3/recorder.yaml` (or `DYX3_RECORDER_PARAMS` in `ros.env`) | `deployment/scripts/start-recorder.sh` |
| `gnss_rtk`, `spray_watchdog` | **none** (started with `ros2 run`, no parameter file) | — **OPEN:** give their launchers a `--params-file` before they can have a profile |

No file means the built-in defaults, which today **are** the production configuration (review BR-001). A file with a
value the node rejects stops that node from starting (every node validates), so its unit restarts loudly instead of
running on a wrong value. `dyx3-param nodes` prints this table from the script itself.

Every run records which file was active: the recorder copies `/etc/dyx3` (secrets excluded) into `config_snapshot/`, the
values at the start and end of the run into `params_ros.json`, and every live change in between through
`/parameter_events` in the bag. FCU (PX4) parameters are **not** profiles: their baseline is `config/px4/*.params`, and
every run reads them live into `params_fcu.json` (recorder contract REC-025).

## 2. The profiles in the repository

`config/profiles/<profile>/<stem>.yaml` holds only the values that profile **decides**, each with its source in the
file's header. A profile without a decided value for a node has no file for it.

| Profile | Status |
|---|---|
| `production` | `rpp.yaml`: `mission_speed: 0.6`, `max_linear_vel: 0.85` (owner decision 2026-10-10; equal to the built-in defaults, written down so the explicit-save path starts from a reviewed file and a future change of a default cannot silently change a production rover) |
| `precision`, `development` | copies of production: **no divergent value is decided**. A value is changed here only with its evidence, in the same commit |

The corner-stop defaults (spec section 9) are not copied into any profile: they are the built-in defaults, carried
verbatim from the prototype, and stay there until GATE 4 re-validates them in the NED frame. `tools/tests/
test_parameter_profiles.py` fails if a profile file names an unknown node, if production stops matching the built-in
defaults it claims to equal, or if precision/development stop being copies of production.

**Installing a profile on a rover** (by hand today; the installer never overwrites `/etc/dyx3` and does not install
profiles: **OPEN (owner)** whether a fresh install should seed `/etc/dyx3` from `production`):

```bash
sudo install -m 0644 -o root -g dyx3 /opt/dyx3/current/config/profiles/production/rpp.yaml /etc/dyx3/rpp.yaml
# takes effect at the node's next start; restart its unit only while the rover is idle (a restart mid-mission in
# OFFBOARD aborts the run)
```

## 3. Tuning live and promoting a value (explicit save)

1. **Tune live:** `dyx3-param set rpp mission_speed 0.7`. The node decides: a `LIVE` parameter is accepted while
   driving, `IDLE_ONLY` is refused while a mission is loaded and `RESTART` always, with the node's reason printed
   verbatim (exit 1). Write doubles as doubles (`1.0`, not `1`): ros2 reads `1` as an integer and the node refuses the
   type. The change is journaled in the run's bag (`/parameter_events`) and lives until the node restarts.
2. **Check:** `dyx3-param get rpp mission_speed`, and `dyx3-param diff rpp` (running values against `/etc/dyx3/rpp.yaml`,
   compared value by value; exit 1 when they differ).
3. **Save on the rover:** `sudo dyx3-param save rpp`. Writes **every** running parameter of the node to
   `/etc/dyx3/rpp.yaml`: temp file in `/etc/dyx3`, `sync`, rename (atomic), `root:dyx3 0644`, no backup kept; the
   change against the previous file is printed as a diff first, and an unchanged file is not rewritten. The header
   records when, from which node and which release. Read at the node's next start.
4. **Promote into the repository:** copy the saved file off the rover (`scp dyx-3wd:/etc/dyx3/rpp.yaml .`), keep only
   the values that differ from the built-in defaults (`dyx3-param diff` against the repository profile shows them),
   put them into `config/profiles/<profile>/rpp.yaml` with their evidence (run directory, date, who decided) in the
   header comment, and commit. `config/` reaches rovers: a human reviews it (CLAUDE.md section 6). The next rover gets
   the value by installing the profile (section 2), never by a hand edit.

**Consequence to know (OPEN, owner):** a saved file freezes every value, the built-in defaults included. A later
release that changes a default does not change a rover with a saved file until the file is saved again or trimmed to
the decided values. `save` cannot trim by itself: a node does not report its built-in defaults, only its current values.
The repository profiles are trimmed by step 4.

## 4. Not verified

`dyx3-param` is tested against a fake `ros2` (`tools/tests/test_dyx3_param.py`: argument validation, the services' DDS
environment, refusal passthrough, atomic save, value diff). Not run against real Humble `ros2 param` verbs or a running
graph; the `--no-daemon --spin-time` options and the stdout format of `ros2 param dump` are Humble's as documented, not
observed on the rover.
