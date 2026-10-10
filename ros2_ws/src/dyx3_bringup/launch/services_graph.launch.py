"""The production services (dyx3-services.service): mission, spray, system_gateway.

The non-safety half of the production ROS graph (proposal 2026-10-10_control-services-unit-split.md, review P2 part 3). The node table,
the unit partition and the node builder live in control_graph.launch.py (one table for both units, so no node is in both launches);
this file starts the rows of dyx3-services only, under normal scheduling (no real-time prefix).

DERIVED — NOT FROM V1 SPEC: when any node of this unit exits, this launch shuts down and systemd restarts dyx3-services only.
dyx3-services.service has After=/Wants=dyx3-control.service, never Requires=, so the control chain keeps running while this unit
restarts. What the rover does meanwhile (not proven on the rover; docs/bench/fault_injection.md must be re-run):
  * dyx3_motion_guard sees /dyx3/mission/state go stale (mission_state_max_age_s, 0.5 s) and fails to zero (REASON_MISSION_GATE);
    px4_link keeps the offboard heartbeat with explicit STOP setpoints; PX4 stays in OFFBOARD, armed. Nothing disarms.
  * spray: the spray controller is gone; dyx3-spray-watchdog (its own unit) keeps requesting OFF through px4_link, which is still up.
  * the tablet loses the gateway (backend -> gateway socket), so the tablet E-stop is unavailable until the unit is back; the E-stop
    latch lives in motion_guard (dyx3-control) and survives; the RC kill is unaffected.
  * when the unit comes back, the mission node is IDLE (progress was in memory), RPP unloads (STOP), and the operator restarts (or
    resumes) the mission from the tablet.
"""

import importlib.util
import os

_HERE = os.path.dirname(os.path.abspath(__file__))
_SPEC = importlib.util.spec_from_file_location("dyx3_bringup_control_graph_launch", os.path.join(_HERE, "control_graph.launch.py"))
graph = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(graph)

UNIT = graph.SERVICES_UNIT


def plan(config_dir: str) -> list[dict]:
    return graph.plan(config_dir, UNIT)


def build_nodes(config_dir: str) -> list:
    return graph.build_nodes(config_dir, UNIT)


def generate_launch_description():
    return graph.launch_description(UNIT)
