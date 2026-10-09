# Bench procedure — fault injection on the production control graph (BR-004)

**Status:** procedure only. Nothing here has been run; the expected outcomes below come from the unit files, `control_graph.launch.py`
and the contracts, not from measurements. Run it on the Jetson, record every result in the table at the end, and file each deviation
as a finding. Where the repository states no number (PX4 loss actions, time for the wheels to stop), the expected outcome is
"record the observed value": the owner decides acceptance.

Static part (already automated, no rover needed): `ros2_ws/src/dyx3_bringup/test/test_control_graph_launch.py` asserts that every
control-graph node has `on_exit=Shutdown` and that this document names every node's executable.

## 0. Safety and set-up (every test)

- **Wheels off the ground** (stands) or drive power isolated. Spray valve and pump physically disabled. RC transmitter on, kill switch
  tested. PX4 USB console (NSH) connected. Do not run any test with the rover on the ground.
- Restarting `dyx3-platform` restarts the XRCE agent, which is the known trigger of the PX4 Ethernet TX stall (see
  `docs/bench/2026-10-09_bench_runbook.md`). Keep the NSH console open; if PX4 goes silent after an agent restart, capture it as that
  runbook says before rebooting the FCU.
- One test at a time. After each test run the **reset** (section 7) and confirm the baseline before the next.

Helpers (paste once per shell; `r2` runs a ROS 2 command as the service user with the service environment):

```bash
r2() { sudo -u dyx3 bash -c 'set -a; . /etc/dyx3/ros.env; set +a; . /opt/dyx3/current/bin/dyx3-env.sh; dyx3_env_load && "$@"' _ "$@"; }
# When a unit last became active (compare before/after: a changed value means the unit was restarted)
since() { for u in "$@"; do printf '%-26s %s\n' "$u" "$(systemctl show -p ActiveEnterTimestamp --value "$u.service")"; done; }
UNITS="dyx3-platform dyx3-ros dyx3-backend dyx3-recorder dyx3-rtk dyx3-spray-watchdog"
```

### Baseline (before every test)

```bash
sudo dyx3-health --deep                       # all PASS (FCU ping may WARN if the FCU is off)
systemctl is-active $UNITS                    # six lines: active
since $UNITS | tee /tmp/before.txt
r2 ros2 node list | sort                      # expect /dyx3_mission /motion_guard /px4_link /recorder /rpp /spray /spray_watchdog /system_gateway (+ the RTK node, if running)
systemctl show -p NRestarts --value dyx3-ros dyx3-platform
cat /run/dyx3/platform_restarts.*             # restarts=0 for xrce-agent and mavlink-router
pgrep -af 'lib/dyx3_(rpp|motion_guard)/'      # each node: exactly one process
ps -o pid,cls,rtprio,psr,comm -p "$(pgrep -f lib/dyx3_rpp/rpp_node)" -p "$(pgrep -f lib/dyx3_motion_guard/motion_guard_node)"
                                              # expect cls FF, rtprio 80, psr 4 for both
```

Watch in a second terminal for the whole session: `journalctl -f -u dyx3-ros -u dyx3-platform -u dyx3-spray-watchdog -u dyx3-recorder`.

## 1. Kill each control-graph node (6 tests)

Design (`control_graph.launch.py`): every node has `on_exit=Shutdown`, so **any** node exiting makes `ros2 launch` shut the whole graph down;
`dyx3-ros` has `Restart=always`, `RestartSec=5`, `TimeoutStopSec=15`. Siblings (`dyx3-platform`, `dyx3-spray-watchdog`, `dyx3-recorder`,
`dyx3-rtk`, `dyx3-backend`) must not be restarted.

For each row of the table, one at a time (start from the baseline, motion NOT commanded, rover disarmed):

| # | Node | Process pattern (`pgrep -f`) |
|---|---|---|
| 1.1 | mission | `lib/dyx3_mission/mission_node` |
| 1.2 | motion_guard (RT, CPU 4) | `lib/dyx3_motion_guard/motion_guard_node` |
| 1.3 | px4_link | `lib/dyx3_px4_link/px4_link_node` |
| 1.4 | rpp (RT, CPU 4) | `lib/dyx3_rpp/rpp_node` |
| 1.5 | spray | `lib/dyx3_spray/spray_node` |
| 1.6 | system_gateway | `lib/dyx3_system_gateway/gateway_node` |

```bash
PAT='lib/dyx3_rpp/rpp_node'                  # one row at a time
pgrep -f "$PAT"                              # must print exactly ONE pid (the watchdog binary is spray_watchdog, a different path)
date +%T; sudo kill -KILL "$(pgrep -f "$PAT")"
# wait ~30 s, then:
journalctl -u dyx3-ros --since "-2min" --no-pager | grep -E "exited|hutdown|died|Started|Stopped"
systemctl show -p NRestarts --value dyx3-ros
since $UNITS | diff /tmp/before.txt -
r2 ros2 node list | sort
```

Expected for every row:

- The journal shows the killed node dying and the launch shutting down (shutdown reason `<node name> exited`, e.g. `rpp exited`), the other five nodes
  receiving SIGINT and stopping, then `dyx3-ros` starting again after about 5 s. Total outage (kill to six nodes back in `ros2 node list`):
  **record it**; the stop part must finish inside `TimeoutStopSec=15`, otherwise systemd kills the group and the journal says so.
- `NRestarts` of `dyx3-ros` increased by exactly 1.
- `since | diff`: **only** the `dyx3-ros` line changed. `dyx3-platform`, `dyx3-backend`, `dyx3-recorder`, `dyx3-rtk`, `dyx3-spray-watchdog` unchanged.
- The agent and the router were not restarted: `cat /run/dyx3/platform_restarts.*` still `restarts=0`.
- Rows 1.2 and 1.4: after the restart `ps -o cls,rtprio,psr` on the two processes shows `FF`, `80`, CPU `4` again (a lowered `LimitRTPRIO`
  would make `chrt` fail and the graph restart-loop; that is a failure, not a pass).
- During the outage: nothing publishes motion to the FCU, and `dyx3-spray-watchdog` is still running and its journal shows the lease absent and OFF being requested
  (`dyx3_spray.md` §2). OFF reaches the valve only through `px4_link`, so while the graph is down it cannot be delivered: that is the open hardware question in the contract, not a test result.
  The mission does **not** resume by itself afterwards (it comes back IDLE; `dyx3_mission.md`: never auto-resume).
- `dyx3-recorder` stays active; if a run was open it closes (`MISSION_STATE_LOST` after `mission_silence_s` = 3 s) and a later `READY` opens a new one.
- Row 1.6 (gateway): the backend stays up and its `/api/ping` still answers; its gateway-backed endpoints fail until the socket is back.

A deviation (a sibling restarted, more than one `dyx3-ros` restart, the graph left half alive, the stop exceeding 15 s) is a finding.

## 2. Agent restart keeps the graph

Design: `start-platform.sh` supervises the XRCE agent and restarts it after `DYX3_RESTART_DELAY_S` (2 s) without touching the unit; `dyx3-ros`
is not stopped, because only `dyx3-platform` itself restarting propagates to it.

```bash
date +%T; sudo kill -KILL "$(pgrep -x MicroXRCEAgent)"
sleep 20
journalctl -u dyx3-platform --since "-1min" --no-pager | grep xrce-agent
cat /run/dyx3/platform_restarts.xrce-agent
since $UNITS | diff /tmp/before.txt -
r2 ros2 node list | sort
r2 ros2 topic echo --once /dyx3/px4_link/status
r2 ros2 topic echo --once /dyx3/safety_gate
sudo dyx3-health --deep
```

Expected:

- Journal: `xrce-agent: exited rc=137; restart #1 in 2s`, then `xrce-agent: starting`. `platform_restarts.xrce-agent` shows `restarts=1`, `last_exit_status=137`.
  `dyx3-health` prints a WARN for `xrce-agent` with the count (not a FAIL). The router's count stays 0.
- `since | diff` is empty: **no unit restarted**, `dyx3-ros` included. All six graph nodes remain in `ros2 node list` the whole time.
- While the agent is down `px4_link` reports its session lost (`session_alive` false) and the guard outputs STOP with reason `PX4_LINK_UNHEALTHY` (8)
  (`dyx3_px4_link.md` §4, `dyx3_motion_guard.md` §3). After the agent is back and the session and handshake re-establish, the gate recovers
  on its own. **Record** the time from kill to `session_alive` true again. Motion does not restart by itself and an interrupted mission stays paused or aborted until an explicit operator action.
- Crash-loop visibility (BR-003): repeat the kill five times, 10 s apart. The count reaches 5, each restart is logged with its number, the unit stays `active`.
- If PX4 stops answering after the restart (the stall), stop and capture it; do not continue the sequence.

Repeat with the router (`pgrep -x mavlink-routerd`, file `platform_restarts.mavlink-router`): the graph and the agent must be unaffected.

## 3. Platform restart propagation

Design: `dyx3-ros` has `Requires=dyx3-platform.service`, so restarting the platform restarts the graph. `dyx3-spray-watchdog` (`Wants=` only),
the recorder, RTK and the backend are not restarted.

3.1 Operator restart:

```bash
date +%T; sudo systemctl restart dyx3-platform
sleep 40
since $UNITS | diff /tmp/before.txt -
systemctl is-active $UNITS
r2 ros2 node list | sort
sudo dyx3-health --deep
```

Expected: `dyx3-platform` **and** `dyx3-ros` have new timestamps (ros started after platform); the other four are unchanged. Total time to
six nodes and a live FCU session: **record**. `platform_restarts.*` are reset to `restarts=0` (new unit run). The spray watchdog stays up the whole time.

3.2 Platform crash (what `Restart=always` does to a `Requires=` dependent — this is the case the repository cannot answer from reading alone):

```bash
date +%T; sudo systemctl kill -s KILL dyx3-platform     # kills the supervisor and its children
sleep 45
systemctl is-active dyx3-platform dyx3-ros
since dyx3-platform dyx3-ros
journalctl -u dyx3-platform -u dyx3-ros --since "-2min" --no-pager
```

Expected: `dyx3-platform` is restarted by systemd after 5 s. **Pass:** `dyx3-ros` is `active` again within about 45 s with all six nodes.
**Fail (finding):** `dyx3-ros` stays `inactive` because it was stopped by the dependency and `Restart=` does not apply to dependency stops. In that case
the rover has a platform with no control graph and `dyx3-health` fails; record it before changing anything.

3.3 Stop propagation: `sudo systemctl stop dyx3-platform` stops `dyx3-ros` too (expected; `dyx3-spray-watchdog` stays active).
Start again with `sudo systemctl start dyx3-platform dyx3-ros`.

## 4. Stop while moving (wheels up)

Preconditions: wheels up, spray physically disabled, FCU armed in OFFBOARD with the graph driving the wheels slowly (a short wheels-up mission, or the
manual drive from the app at a low speed), RC in hand. In NSH keep `listener actuator_motors` ready, or watch the wheels. Start a stopwatch at each command.

| # | Action | Command |
|---|---|---|
| 4.1 | graceful stop of the graph | `sudo systemctl stop dyx3-ros` |
| 4.2 | graceful stop of the platform (also stops the graph) | `sudo systemctl stop dyx3-platform` |
| 4.3 | graph killed hard | `sudo systemctl kill -s KILL dyx3-ros` |
| 4.4 | agent killed while moving | `sudo kill -KILL "$(pgrep -x MicroXRCEAgent)"` |

Expected for every row:

- The wheels stop. **Record** the time from the command to zero wheel speed (stopwatch or the ULog) for each row. The repository states no number: PX4's offboard loss
  handling (`COM_OF_LOSS_T`, `COM_OBL_RC_ACT`) is firmware territory (`dyx3_px4_link.md` §4 closing paragraph). A wheel that keeps turning at the last speed is a failed test.
- Nothing resumes by itself. After `systemctl start`, the rover stays stopped until an operator arms, re-enters OFFBOARD and resumes the mission.
- 4.1: `time sudo systemctl stop dyx3-ros` completes in under 15 s; ros2 launch escalates SIGINT, SIGTERM, SIGKILL after 5 s + 5 s.
- `dyx3-spray-watchdog` stays active in all rows (OFF delivery needs `px4_link`, see section 1). `dyx3-recorder` stays active and closes the run:
  check the newest run directory under `/var/lib/dyx3/runs` has `summary.json` and `rosbag2*/metadata.yaml`.
- 4.4: as section 2, plus the wheel stop above.

## 5. A SIGSTOPped node

`on_exit=Shutdown` only fires when a node **exits**. A frozen node is alive, so launch, `dyx3-ros` and `systemctl is-active` all stay green: nothing supervises
liveness (BR-002 is the open finding for that). This test shows which defence the rover has instead: the staleness checks in the other nodes. Do it disarmed first;
then repeat armed with wheels up for rows 5.1 to 5.5.

```bash
PAT='lib/dyx3_rpp/rpp_node'                 # one row at a time, see the table
date +%T; sudo kill -STOP "$(pgrep -f "$PAT")"
sleep 10
systemctl is-active dyx3-ros                 # stays 'active'
r2 ros2 topic echo --once /dyx3/motion_guard/status
r2 ros2 topic echo --once /dyx3/safety_gate
r2 ros2 node list | sort                     # the frozen node may still be listed
sudo kill -CONT "$(pgrep -f "$PAT")"         # then run the reset (section 7)
```

| # | Frozen node | Pattern | Expected (from the contracts) |
|---|---|---|---|
| 5.1 | rpp | `lib/dyx3_rpp/rpp_node` | guard output STOP with reason `STALE` (2) after `command_max_age_s`; wheels stopped |
| 5.2 | motion_guard | `lib/dyx3_motion_guard/motion_guard_node` | `px4_link` stops receiving fresh commands and holds STOP/zero (`dyx3_px4_link.md` §4: never publishes a stale setpoint); wheels stopped; **record** the time and whether PX4 leaves OFFBOARD |
| 5.3 | px4_link | `lib/dyx3_px4_link/px4_link_node` | the offboard heartbeat stops; PX4's own loss action applies (`COM_OF_LOSS_T`, firmware): **record** time to stop and the PX4 mode afterwards. This is the case the repository cannot cover |
| 5.4 | mission | `lib/dyx3_mission/mission_node` | the guard's mission gate goes stale: `MISSION_GATE` (4), wheels stopped; the recorder closes the run with `MISSION_STATE_LOST` after 3 s |
| 5.5 | system_gateway | `lib/dyx3_system_gateway/gateway_node` | the operator-link verdict stops updating: `OPERATOR_LINK_LOST` (10), wheels stopped; the backend cannot reach the graph |
| 5.6 | spray | `lib/dyx3_spray/spray_node` | the lease goes stale, `dyx3-spray-watchdog` sends OFF (`dyx3_spray.md` §2); valve physically disabled, so check the watchdog journal and `/dyx3/spray/watchdog_status` |

Also: with one node frozen, `time sudo systemctl stop dyx3-ros` must still complete within 15 s (SIGKILL ends a stopped process).

Every row whose expected outcome does not happen, or where the wheels do not stop, is a **blocking finding**.

## 6. Results

| Test | Date | Result (PASS / FAIL / value) | Observed (times, journal lines) | Finding |
|---|---|---|---|---|
| 1.1 to 1.6 kill each node | | | | |
| 2 agent kill / router kill / crash-loop count | | | | |
| 3.1 platform restart | | | | |
| 3.2 platform crash | | | | |
| 3.3 platform stop | | | | |
| 4.1 to 4.4 stop while moving (wheel stop time each) | | | | |
| 5.1 to 5.6 SIGSTOP each node | | | | |

## 7. Reset after each test

```bash
sudo systemctl reset-failed 'dyx3-*'
sudo systemctl restart dyx3-platform             # also restarts dyx3-ros
sleep 30
systemctl is-active $UNITS
sudo dyx3-health --deep
since $UNITS | tee /tmp/before.txt
```
