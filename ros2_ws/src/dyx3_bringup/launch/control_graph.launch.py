"""The production control graph (dyx3-ros.service).

Runs: mission, motion_guard, px4_link, spray, system_gateway. NOT in this graph, on purpose:
  * dyx3_rpp         : the controller node does not exist yet (docs/contracts/dyx3_rpp.md section 2); nothing here can drive.
  * dyx3_gnss_rtk    : its own service (dyx3-rtk), a sibling of the backend, never a child.
  * dyx3_recorder    : its own service (dyx3-recorder), keeps recording when anything else dies.
  * spray_watchdog   : its own service (dyx3-spray-watchdog), must survive this graph dying.
  * dyx3_rpp_legacy  : quarantined oracle, never part of a production graph (CLAUDE.md section 8).

DERIVED — NOT FROM V1 SPEC: when ANY node of this graph exits, the whole launch shuts down and systemd restarts the unit. A graph with a
dead px4_link or guard is not a degraded graph worth keeping half alive, and a restart mid-mission aborts the run anyway.
Per-node real-time priority / CPU affinity (architecture 8) is NOT expressed here: OPEN.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# (package, executable, node name, parameter-file stem under <config_dir>)
GRAPH = (
    ("dyx3_mission", "mission_node", "dyx3_mission", "mission"),
    ("dyx3_motion_guard", "motion_guard_node", "motion_guard", "motion_guard"),
    ("dyx3_px4_link", "px4_link_node", "px4_link", "px4"),
    ("dyx3_spray", "spray_node", "spray", "spray"),
    ("dyx3_system_gateway", "gateway_node", "system_gateway", "gateway"),
)


def plan(config_dir: str) -> list[dict]:
    """What will be launched: package, executable, name and the parameter file that exists (or None)."""
    out = []
    for package, executable, name, stem in GRAPH:
        params_file = os.path.join(config_dir, f"{stem}.yaml")
        # A parameter file that exists is applied; a missing one means the node's built-in defaults. A malformed
        # value makes the node refuse to start (they all validate), which restarts the unit loudly.
        out.append(
            {
                "package": package,
                "executable": executable,
                "name": name,
                "params_file": params_file if os.path.isfile(params_file) else None,
            }
        )
    return out


def build_nodes(config_dir: str) -> list:
    return [
        Node(
            package=p["package"],
            executable=p["executable"],
            name=p["name"],
            output="screen",
            parameters=[p["params_file"]] if p["params_file"] else [],
            on_exit=Shutdown(reason=f"{p['name']} exited"),
        )
        for p in plan(config_dir)
    ]


def _setup(context, *args, **kwargs):
    return build_nodes(LaunchConfiguration("config_dir").perform(context))


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("config_dir", default_value="/etc/dyx3", description="directory holding <stem>.yaml parameter files"),
            OpaqueFunction(function=_setup),
        ]
    )
