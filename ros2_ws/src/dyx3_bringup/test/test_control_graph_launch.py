import importlib.util
import os

import pytest
from launch.actions import Shutdown
from launch.substitutions import TextSubstitution

HERE = os.path.dirname(os.path.abspath(__file__))
LAUNCH_DIR = os.path.join(HERE, "..", "launch")
SPEC = importlib.util.spec_from_file_location("control_graph_launch", os.path.join(LAUNCH_DIR, "control_graph.launch.py"))
mod = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(mod)

CONTROL = ["dyx3_motion_guard", "dyx3_px4_link", "dyx3_rpp"]


def test_whole_graph_table_is_exactly_the_implemented_nodes():
    # GRAPH is the one table of both units (dyx3_recorder's REC-002 test parses it from this file).
    packages = [g[0] for g in mod.GRAPH]
    assert packages == ["dyx3_mission", "dyx3_motion_guard", "dyx3_px4_link", "dyx3_rpp", "dyx3_spray", "dyx3_system_gateway"]
    # siblings and quarantine never appear in either unit
    for forbidden in ("dyx3_gnss_rtk", "dyx3_recorder", "dyx3_rpp_legacy"):
        assert forbidden not in packages
    # no executable named after the independent watchdog either: it is its own service
    assert all("watchdog" not in g[1] for g in mod.GRAPH)


def test_every_graph_node_belongs_to_exactly_one_unit():
    assert set(mod.UNIT_PACKAGES) == {"dyx3-control", "dyx3-services"}
    owners = {}
    for unit, packages in mod.UNIT_PACKAGES.items():
        for package in packages:
            owners.setdefault(package, []).append(unit)
    assert {g[0] for g in mod.GRAPH} == set(owners)
    assert all(len(units) == 1 for units in owners.values()), owners


def test_control_unit_is_exactly_the_three_control_chain_nodes():
    assert [g[0] for g in mod.unit_graph("dyx3-control")] == CONTROL
    assert [p["name"] for p in mod.plan("/nonexistent")] == ["motion_guard", "px4_link", "rpp"]
    # the default unit of this file is the control unit
    assert mod.plan("/nonexistent") == mod.plan("/nonexistent", "dyx3-control")


def test_an_unknown_unit_is_an_error_not_an_empty_graph():
    with pytest.raises(KeyError):
        mod.plan("/nonexistent", "dyx3-ros")
    with pytest.raises(ValueError):
        mod.launch_description("dyx3-ros")


def test_parameter_files_are_applied_only_when_they_exist(tmp_path):
    (tmp_path / "px4.yaml").write_text("/**:\n  ros__parameters: {}\n")
    p = mod.plan(str(tmp_path))
    assert [x["name"] for x in p] == ["motion_guard", "px4_link", "rpp"]
    assert [x["params_file"] is not None for x in p] == [False, True, False]
    assert all(x["params_file"] is None for x in mod.plan(str(tmp_path / "missing")))


def test_real_time_prefixes_are_exactly_the_three_control_chain_executors():
    prefixes = {p["package"]: p["prefix"] for p in mod.plan("/nonexistent")}
    # RPP and motion_guard keep FIFO 80 on CPU 4; px4_link shares CPU 4 at FIFO 70, so both 80s preempt the writer.
    assert prefixes == {
        "dyx3_motion_guard": "taskset -c 4 chrt -f 80",
        "dyx3_px4_link": "taskset -c 4 chrt -f 70",
        "dyx3_rpp": "taskset -c 4 chrt -f 80",
    }
    # no package of the other unit carries a real-time prefix
    assert set(mod.RT_PREFIXES) == set(mod.UNIT_PACKAGES["dyx3-control"])
    # and the built Nodes carry exactly these prefixes, as literal text
    for node, p in zip(mod.build_nodes("/nonexistent"), mod.plan("/nonexistent")):
        prefix = node.process_description.prefix
        assert all(isinstance(s, TextSubstitution) for s in prefix), prefix
        assert "".join(s.text for s in prefix) == p["prefix"]


def test_px4_link_is_on_the_control_core_below_both_control_executors():
    prefixes = {p["package"]: p["prefix"] for p in mod.plan("/nonexistent")}

    def parse(prefix):
        words = prefix.split()
        return int(words[words.index("-c") + 1]), int(words[words.index("-f") + 1])

    link_cpu, link_prio = parse(prefixes["dyx3_px4_link"])
    for control in ("dyx3_rpp", "dyx3_motion_guard"):
        cpu, prio = parse(prefixes[control])
        assert link_cpu == cpu
        assert link_prio < prio


def test_launch_description_builds_and_every_node_is_created():
    nodes = mod.build_nodes("/nonexistent")
    assert len(nodes) == len(CONTROL)
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


def test_every_control_unit_node_shuts_the_control_launch_down_when_it_exits():
    # BR-004 within the unit: "any node exits -> ros2 launch exits -> systemd restarts dyx3-control". A node added without
    # on_exit=Shutdown would die alone and leave a half-alive control chain that systemd still reports as active.
    nodes = mod.build_nodes("/nonexistent")
    rows = mod.unit_graph("dyx3-control")
    assert len(nodes) == len(rows) == 3
    for node, (_package, _executable, name, _stem) in zip(nodes, rows):
        on_exit = _on_exit_of(node)
        assert isinstance(on_exit, Shutdown), f"{name}: on_exit is {on_exit!r}, not Shutdown"
        assert on_exit.event.reason == f"{name} exited"


def test_no_node_is_respawned_by_launch():
    # A respawned node would hide a crash from systemd: the unit would stay up while the crashed part restarted on its own.
    for launch_file in ("control_graph.launch.py", "services_graph.launch.py"):
        with open(os.path.join(LAUNCH_DIR, launch_file)) as fh:
            assert "respawn" not in fh.read(), launch_file


def test_bench_fault_injection_procedure_covers_every_graph_node():
    # BR-004: docs/bench/fault_injection.md must name each graph executable it kills (both units); keeps the procedure and the graph
    # in step.
    doc = os.path.abspath(os.path.join(HERE, "..", "..", "..", "..", "docs", "bench", "fault_injection.md"))
    if not os.path.isfile(doc):
        pytest.skip("repository docs are not next to this package (installed layout)")
    with open(doc) as fh:
        text = fh.read()
    for package, executable, name, _stem in mod.GRAPH:
        assert f"lib/{package}/{executable}" in text, f"{name}: lib/{package}/{executable} missing from the bench procedure"
