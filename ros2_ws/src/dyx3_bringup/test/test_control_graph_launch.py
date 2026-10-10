import importlib.util
import os

import pytest
from launch.actions import Shutdown

HERE = os.path.dirname(os.path.abspath(__file__))
SPEC = importlib.util.spec_from_file_location("control_graph_launch", os.path.join(HERE, "..", "launch", "control_graph.launch.py"))
mod = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(mod)


def test_graph_membership_is_exactly_the_implemented_control_nodes():
    packages = [g[0] for g in mod.GRAPH]
    assert packages == ["dyx3_mission", "dyx3_motion_guard", "dyx3_px4_link", "dyx3_rpp", "dyx3_spray", "dyx3_system_gateway"]
    # siblings and quarantine never appear in the control graph
    for forbidden in ("dyx3_gnss_rtk", "dyx3_recorder", "dyx3_rpp_legacy"):
        assert forbidden not in packages
    # no executable named after the independent watchdog either: it is its own service
    assert all("watchdog" not in g[1] for g in mod.GRAPH)


def test_parameter_files_are_applied_only_when_they_exist(tmp_path):
    (tmp_path / "px4.yaml").write_text("/**:\n  ros__parameters: {}\n")
    p = mod.plan(str(tmp_path))
    assert [x["name"] for x in p] == ["dyx3_mission", "motion_guard", "px4_link", "rpp", "spray", "system_gateway"]
    assert [x["params_file"] is not None for x in p] == [False, False, True, False, False, False]
    assert all(x["params_file"] is None for x in mod.plan(str(tmp_path / "missing")))


def test_only_control_executors_receive_real_time_scheduling():
    plans = mod.plan("/nonexistent")
    assert {p["package"] for p in plans if p["prefix"] is not None} == {
        "dyx3_motion_guard", "dyx3_rpp"
    }
    assert all(p["prefix"] == mod.RT_CONTROL_PREFIX for p in plans if p["prefix"] is not None)


def test_launch_description_builds_and_every_node_is_created():
    nodes = mod.build_nodes("/nonexistent")
    assert len(nodes) == len(mod.GRAPH)
    desc = mod.generate_launch_description()
    assert len(desc.entities) == 2  # the config_dir argument and the node-building function


def _on_exit_of(node):
    """The action a launch Node runs when its process exits.

    launch exposes no public getter (get_sub_entities() only returns lists), so read the name-mangled attribute. If a future launch
    release renames it, this fails loudly instead of silently passing.
    """
    found = [name for name in vars(node) if name.endswith("__on_exit")]
    assert len(found) == 1, f"launch no longer stores on_exit as expected: {sorted(vars(node))}"
    return getattr(node, found[0])


def test_every_control_graph_node_shuts_the_whole_launch_down_when_it_exits():
    # BR-004: the unit's fail-safe design is "any node exits -> ros2 launch exits -> systemd restarts dyx3-ros". A node added to the
    # graph without on_exit=Shutdown would die alone and leave a half-alive graph that systemd still reports as active.
    nodes = mod.build_nodes("/nonexistent")
    assert len(nodes) == len(mod.GRAPH) == 6
    for node, (_package, _executable, name, _stem) in zip(nodes, mod.GRAPH):
        on_exit = _on_exit_of(node)
        assert isinstance(on_exit, Shutdown), f"{name}: on_exit is {on_exit!r}, not Shutdown"
        assert on_exit.event.reason == f"{name} exited"


def test_no_node_is_respawned_by_launch():
    # A respawned node would hide a crash from systemd: the graph would stay up while the crashed part restarted on its own.
    with open(os.path.join(HERE, "..", "launch", "control_graph.launch.py")) as fh:
        assert "respawn" not in fh.read()


def test_bench_fault_injection_procedure_covers_every_graph_node():
    # BR-004: docs/bench/fault_injection.md must name each control-graph executable it kills; keeps the procedure and the graph in step.
    doc = os.path.abspath(os.path.join(HERE, "..", "..", "..", "..", "docs", "bench", "fault_injection.md"))
    if not os.path.isfile(doc):
        pytest.skip("repository docs are not next to this package (installed layout)")
    with open(doc) as fh:
        text = fh.read()
    for package, executable, name, _stem in mod.GRAPH:
        assert f"lib/{package}/{executable}" in text, f"{name}: lib/{package}/{executable} missing from the bench procedure"
