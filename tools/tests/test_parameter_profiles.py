"""config/profiles: every file is a ROS parameter file of a known node, and each profile is what its header says.

production/rpp.yaml claims its two values equal the built-in defaults (rpp_param_table.inc, generated from the
registry); precision and development claim to be copies of production. If either claim stops being true the profile's
documentation is wrong, so this fails.
"""

import re
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
PROFILES = ROOT / "config" / "profiles"
RPP_TABLE = ROOT / "ros2_ws" / "src" / "dyx3_rpp" / "include" / "dyx3_rpp" / "rpp_param_table.inc"
DYX3_PARAM = ROOT / "deployment" / "scripts" / "dyx3-param"


def load_yaml(path):
    try:
        import yaml

        return yaml.safe_load(path.read_text())
    except ImportError:
        pass
    try:
        from ruamel.yaml import YAML  # CI: installed with rosbags
    except ImportError:
        pytest.skip("no YAML parser")
    return YAML(typ="safe").load(path.read_text())


def node_stems():
    """node -> stem from dyx3-param's table (the one place that maps nodes to /etc/dyx3 files)."""
    rows = re.findall(r'^  "([a-z_0-9]+)\|([a-z_0-9]+)\|', DYX3_PARAM.read_text(), re.M)
    assert rows, "dyx3-param NODE_TABLE not found"
    return dict(rows)


def profile_files(profile):
    return sorted((PROFILES / profile).glob("*.yaml"))


def test_three_profiles_with_a_readme_each():
    assert sorted(p.name for p in PROFILES.iterdir() if p.is_dir()) == ["development", "precision", "production"]
    for p in ("development", "precision", "production"):
        assert (PROFILES / p / "README.md").is_file()


@pytest.mark.parametrize("profile", ["production", "precision", "development"])
def test_every_file_is_the_parameter_file_of_its_node(profile):
    stems = node_stems()
    for f in profile_files(profile):
        doc = load_yaml(f)
        nodes = [n for n, s in stems.items() if s == f.stem]
        assert nodes, f"{f}: no node reads /etc/dyx3/{f.name}"
        assert list(doc) == ["/" + nodes[0]], f"{f}: top-level key must be /{nodes[0]}"
        assert isinstance(doc["/" + nodes[0]]["ros__parameters"], dict)


def test_production_rpp_values_are_the_built_in_defaults():
    doc = load_yaml(PROFILES / "production" / "rpp.yaml")["/rpp"]["ros__parameters"]
    assert doc == {"mission_speed": 0.6, "max_linear_vel": 0.85}
    table = RPP_TABLE.read_text()
    for name, value in doc.items():
        m = re.search(r'\{"' + re.escape(name) + r'", Kind::\w+, ParamClass::(\w+), ([^,]+),', table)
        assert m, name
        assert float(m.group(2)) == value, f"{name}: default {m.group(2)} != profile {value}"
        assert m.group(1) == "Live", name  # changeable while driving, so `dyx3-param set` can tune it in the field


@pytest.mark.parametrize("profile", ["precision", "development"])
def test_undecided_profiles_are_copies_of_production(profile):
    prod = {f.name: load_yaml(f) for f in profile_files("production")}
    other = {f.name: load_yaml(f) for f in profile_files(profile)}
    assert other == prod
