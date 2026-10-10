"""The production control chain (dyx3-control.service): px4_link, motion_guard, rpp.

The production ROS graph runs as two systemd units (proposal 2026-10-10_control-services-unit-split.md, review P2 part 3):
  * dyx3-control.service  -> this file:                 dyx3_px4_link, dyx3_motion_guard, dyx3_rpp (the chain that commands PX4)
  * dyx3-services.service -> services_graph.launch.py:   dyx3_mission, dyx3_spray, dyx3_system_gateway (normal scheduling)
GRAPH below is the one table of the whole graph (both units); each launch file starts only the rows of its own unit, so no node can
be started twice or by neither. dyx3_recorder's REC-002 test parses GRAPH out of this file: keep the table here.

NOT in either unit, on purpose:
  * dyx3_gnss_rtk    : its own service (dyx3-rtk), a sibling of the backend, never a child.
  * dyx3_recorder    : its own service (dyx3-recorder), keeps recording when anything else dies.
  * spray_watchdog   : its own service (dyx3-spray-watchdog), must survive both units dying.
  * dyx3_rpp_legacy  : quarantined oracle, never part of a production graph (CLAUDE.md section 8).

DERIVED — NOT FROM V1 SPEC: when ANY node of a unit exits, that unit's launch shuts down and systemd restarts that unit only. A control
chain with a dead px4_link, guard or rpp is not worth keeping half alive. The services unit is separate so that a crash in non-safety
code (mission, spray, gateway) no longer takes px4_link down: before the split, any of the six exiting stopped the whole launch,
px4_link sent its 0.3 s STOP burst and exited, and PX4's offboard loss (0.5 s) disarmed the rover mid-line.

Behaviour when dyx3-services dies (not proven on the rover; docs/bench/fault_injection.md must be re-run):
  * this unit keeps running. dyx3_motion_guard sees /dyx3/mission/state go stale (mission_state_max_age_s, 0.5 s) and fails to zero
    (REASON_MISSION_GATE); px4_link keeps the offboard heartbeat with explicit STOP setpoints; PX4 stays in OFFBOARD, armed. Nothing
    disarms. The tablet E-stop path (backend -> gateway) is down meanwhile; the E-stop latch itself lives in motion_guard (this unit)
    and survives, and the RC kill is unaffected.
  * when dyx3-services comes back, the mission node starts IDLE (its progress was in memory), RPP unloads (STOP), and the operator
    restarts (or resumes) the mission from the tablet.
When this unit dies: px4_link's fail-to-zero STOP burst, then PX4 offboard loss -> disarm, exactly as before the split.

DERIVED — NOT FROM V1 SPEC: retain the existing FIFO 80 / CPU 4 allocation for RPP and motion_guard, the two control executors named by
architecture section 8, so they preempt everything else on that core. dyx3_px4_link, the only other hop in the control chain, shares CPU 4
at FIFO 70: below the two control executors, above everything else (70 is not a tuned value; it only has to order px4_link under the two
80s). The services unit (mission, spray, gateway) stays under normal scheduling. dyx3-recorder.service keeps the recorder off CPU 4
(CPUAffinity=0-3 5). The prefixes run as User=dyx3 without CAP_SYS_NICE, which the kernel allows only up to RLIMIT_RTPRIO
(dyx3-control.service LimitRTPRIO=99): if chrt fails the node exits and the control unit restarts, loud and never silently non-RT.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# The whole production ROS graph, both units: (package, executable, node name, parameter-file stem under <config_dir>).
GRAPH = (
    ("dyx3_mission", "mission_node", "dyx3_mission", "mission"),
    ("dyx3_motion_guard", "motion_guard_node", "motion_guard", "motion_guard"),
    ("dyx3_px4_link", "px4_link_node", "px4_link", "px4"),
    ("dyx3_rpp", "rpp_node", "rpp", "rpp"),
    ("dyx3_spray", "spray_node", "spray", "spray"),
    ("dyx3_system_gateway", "gateway_node", "system_gateway", "gateway"),
)

CONTROL_UNIT = "dyx3-control"
SERVICES_UNIT = "dyx3-services"
# unit -> the packages it starts. Every GRAPH package is in exactly one unit (test_control_graph_launch.py asserts it).
UNIT_PACKAGES = {
    CONTROL_UNIT: ("dyx3_px4_link", "dyx3_motion_guard", "dyx3_rpp"),
    SERVICES_UNIT: ("dyx3_mission", "dyx3_spray", "dyx3_system_gateway"),
}

RT_CONTROL_PREFIX = "taskset -c 4 chrt -f 80"
RT_LINK_PREFIX = "taskset -c 4 chrt -f 70"
# package -> real-time prefix; a package absent from this map runs under normal scheduling. Only control-unit packages appear here.
RT_PREFIXES = {
    "dyx3_motion_guard": RT_CONTROL_PREFIX,
    "dyx3_rpp": RT_CONTROL_PREFIX,
    "dyx3_px4_link": RT_LINK_PREFIX,
}


def unit_graph(unit: str) -> tuple:
    """The GRAPH rows one unit starts, in GRAPH order. An unknown unit is an error, never an empty graph."""
    packages = UNIT_PACKAGES[unit]
    return tuple(g for g in GRAPH if g[0] in packages)


def plan(config_dir: str, unit: str = CONTROL_UNIT) -> list[dict]:
    """What one unit will launch: package, executable, name, the parameter file that exists (or None) and the prefix."""
    out = []
    for package, executable, name, stem in unit_graph(unit):
        params_file = os.path.join(config_dir, f"{stem}.yaml")
        # A parameter file that exists is applied; a missing one means the node's built-in defaults. A malformed
        # value makes the node refuse to start (they all validate), which restarts the unit loudly.
        out.append(
            {
                "package": package,
                "executable": executable,
                "name": name,
                "params_file": params_file if os.path.isfile(params_file) else None,
                "prefix": RT_PREFIXES.get(package),
            }
        )
    return out


def build_nodes(config_dir: str, unit: str = CONTROL_UNIT) -> list:
    return [
        Node(
            package=p["package"],
            executable=p["executable"],
            name=p["name"],
            output="screen",
            parameters=[p["params_file"]] if p["params_file"] else [],
            prefix=p["prefix"],
            # Within this unit only: a dead node ends this launch and systemd restarts this unit, never the other one.
            on_exit=Shutdown(reason=f"{p['name']} exited"),
        )
        for p in plan(config_dir, unit)
    ]


def launch_description(unit: str) -> LaunchDescription:
    """The launch description of one unit (services_graph.launch.py calls this with SERVICES_UNIT)."""
    if unit not in UNIT_PACKAGES:  # an unknown unit fails at load time, never as an empty graph
        raise ValueError(f"unknown unit {unit!r}; expected one of {sorted(UNIT_PACKAGES)}")

    def _setup(context, *args, **kwargs):
        return build_nodes(LaunchConfiguration("config_dir").perform(context), unit)

    return LaunchDescription(
        [
            DeclareLaunchArgument("config_dir", default_value="/etc/dyx3", description="directory holding <stem>.yaml parameter files"),
            OpaqueFunction(function=_setup),
        ]
    )


def generate_launch_description() -> LaunchDescription:
    return launch_description(CONTROL_UNIT)
