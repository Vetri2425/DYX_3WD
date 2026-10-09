import importlib.util
import os

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
