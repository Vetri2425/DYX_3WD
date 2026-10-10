import importlib.util
import os

from launch.actions import Shutdown
from launch.substitutions import TextSubstitution

HERE = os.path.dirname(os.path.abspath(__file__))
LAUNCH_DIR = os.path.join(HERE, "..", "launch")


def _load(stem):
    spec = importlib.util.spec_from_file_location(stem.replace(".", "_"), os.path.join(LAUNCH_DIR, f"{stem}.py"))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


services = _load("services_graph.launch")
control = _load("control_graph.launch")

SERVICES = ["dyx3_mission", "dyx3_spray", "dyx3_system_gateway"]


def test_services_unit_is_exactly_mission_spray_and_gateway():
    assert services.UNIT == "dyx3-services"
    assert [p["package"] for p in services.plan("/nonexistent")] == SERVICES
    assert [p["name"] for p in services.plan("/nonexistent")] == ["dyx3_mission", "spray", "system_gateway"]


def test_no_node_appears_in_both_launches_and_together_they_are_the_whole_graph():
    in_services = [(p["package"], p["executable"], p["name"]) for p in services.plan("/nonexistent")]
    in_control = [(p["package"], p["executable"], p["name"]) for p in control.plan("/nonexistent")]
    assert not set(in_services) & set(in_control)
    # by node name too: two nodes with one name would be one node to ROS
    assert not {n for _, _, n in in_services} & {n for _, _, n in in_control}
    assert sorted(in_services + in_control) == sorted((g[0], g[1], g[2]) for g in control.GRAPH)


def test_services_run_under_normal_scheduling():
    assert all(p["prefix"] is None for p in services.plan("/nonexistent"))
    # On the built Node too: with no prefix, launch keeps only its own `launch-prefix` configuration (empty by default), never a
    # literal text such as a taskset/chrt command.
    for node in services.build_nodes("/nonexistent"):
        prefix = node.process_description.prefix
        assert not [s for s in prefix if isinstance(s, TextSubstitution)], f"{node.node_executable}: literal prefix {prefix!r}"


def test_parameter_files_are_applied_only_when_they_exist(tmp_path):
    (tmp_path / "gateway.yaml").write_text("/**:\n  ros__parameters: {}\n")
    p = services.plan(str(tmp_path))
    assert [x["params_file"] is not None for x in p] == [False, False, True]


def test_launch_description_builds_and_every_node_is_created():
    assert len(services.build_nodes("/nonexistent")) == len(SERVICES)
    desc = services.generate_launch_description()
    assert len(desc.entities) == 2  # the config_dir argument and the node-building function


def test_every_services_unit_node_shuts_the_services_launch_down_when_it_exits():
    # Within dyx3-services only: a dead mission, spray or gateway ends this launch and systemd restarts this unit, not dyx3-control.
    nodes = services.build_nodes("/nonexistent")
    rows = control.unit_graph("dyx3-services")
    assert len(nodes) == len(rows) == 3
    for node, (_package, _executable, name, _stem) in zip(nodes, rows):
        found = [a for a in vars(node) if a.endswith("__on_exit")]
        assert len(found) == 1, f"launch no longer stores on_exit as expected: {sorted(vars(node))}"
        on_exit = getattr(node, found[0])
        assert isinstance(on_exit, Shutdown), f"{name}: on_exit is {on_exit!r}, not Shutdown"
        assert on_exit.event.reason == f"{name} exited"
