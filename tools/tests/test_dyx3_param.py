"""deployment/scripts/dyx3-param against a fake `ros2` (no ROS, no running nodes).

The fake records its arguments and environment and answers like Humble's ros2 param verbs: `param set` exits 0 even
when the node refuses (the refusal is on stderr), `param dump` prints YAML on stdout.
"""

import os
import subprocess
import sys
from pathlib import Path

import pytest

SCRIPT = Path(__file__).resolve().parents[2] / "deployment" / "scripts" / "dyx3-param"

FAKE_ROS2 = r"""#!/usr/bin/env bash
d="$(cd "$(dirname "$0")" && pwd)"
printf 'env ROS_DOMAIN_ID=%s ROS_LOCALHOST_ONLY=%s ROS_HOME_SET=%s\nargs %s\n' "${ROS_DOMAIN_ID:-}" \
  "${ROS_LOCALHOST_ONLY:-}" "${ROS_HOME:+yes}" "$*" >>"${d}/calls"
mode="$(cat "${d}/mode" 2>/dev/null || true)"
case "$1 $2" in
  "param get") echo "Double value is: 0.6" ;;
  "param set")
    case "${mode}" in
      refuse) echo "Setting parameter failed: wheel_track_m is IDLE_ONLY: refused while a mission is loaded" >&2 ;;
      type)
        m="Wrong parameter type, parameter {mission_speed} is of type {double}, setting it to {integer} is not allowed."
        echo "Setting parameter failed: ${m}" >&2 ;;
      *) echo "Set parameter successful" ;;
    esac ;;
  "param dump") cat "${d}/dump.yaml" ;;
  *) echo "fake ros2: unexpected $*" >&2; exit 9 ;;
esac
"""

DUMP = """/rpp:
  ros__parameters:
    max_linear_vel: 0.85
    mission_speed: {speed}
    use_sim_time: false
"""


@pytest.fixture
def rig(tmp_path):
    fb = tmp_path / "bin"
    fb.mkdir()
    (fb / "ros2").write_text(FAKE_ROS2)
    (fb / "ros2").chmod(0o755)
    # the interpreter `diff` uses (needs PyYAML); a wrapper, since a symlinked venv interpreter loses its venv
    (fb / "python3").write_text(f'#!/bin/sh\nexec "{sys.executable}" "$@"\n')
    (fb / "python3").chmod(0o755)
    (fb / "dump.yaml").write_text(DUMP.format(speed=0.6))
    etc = tmp_path / "etc"
    etc.mkdir()
    (etc / "ros.env").write_text("# fleet domain\nROS_DOMAIN_ID=42\nDYX3_ROS_LOCALHOST_ONLY=1\n")
    envsh = tmp_path / "dyx3-env.sh"
    envsh.write_text('dyx3_env_load() {\n  export ROS_DOMAIN_ID\n'
                     '  if [ "${DYX3_ROS_LOCALHOST_ONLY:-0}" = 1 ]; then export ROS_LOCALHOST_ONLY=1; fi\n}\n')
    env = {
        "PATH": f"{fb}:{os.environ['PATH']}",
        "DYX3_ETC": str(etc),
        "DYX3_ENV_SH": str(envsh),
        "DYX3_RELEASE_DIR": str(tmp_path),
        "DYX3_USER": "dyx3-test-absent-user",
        "DYX3_PARAM_SPIN_S": "1",
        "TMPDIR": str(tmp_path),
    }

    def run(*args, mode=""):
        (fb / "mode").write_text(mode)
        return subprocess.run([str(SCRIPT), *args], env=env, capture_output=True, text=True, timeout=60)

    run.bin, run.etc = fb, etc
    return run


@pytest.mark.parametrize("args", [
    (), ("frobnicate",), ("get",), ("get", "rpp"), ("get", "rpp", "a", "b"), ("set", "rpp", "mission_speed"),
    ("save",), ("save", "rpp", "extra"), ("diff",), ("get", "no_such_node", "x"), ("get", "rpp;rm", "x"),
    ("get", "rpp", "bad name"), ("get", "rpp", "$(id)"), ("nodes", "x"),
])
def test_bad_arguments_are_usage_errors_and_reach_no_node(rig, args):
    r = rig(*args)
    assert r.returncode == 2, (args, r.stdout, r.stderr)
    assert not (rig.bin / "calls").exists()


def test_help_and_nodes(rig):
    assert rig("--help").returncode == 0
    r = rig("nodes")
    assert r.returncode == 0
    names = [line.split()[0] for line in r.stdout.splitlines()]
    assert names == ["dyx3_mission", "motion_guard", "px4_link", "rpp", "spray", "system_gateway", "gnss_rtk",
                     "recorder", "spray_watchdog"]
    assert f"{rig.etc}/px4.yaml" in r.stdout and f"{rig.etc}/gateway.yaml" in r.stdout


def test_get_runs_in_the_services_dds_environment(rig):
    r = rig("get", "/rpp", "mission_speed")
    assert r.returncode == 0, r.stderr
    assert "Double value is: 0.6" in r.stdout
    calls = (rig.bin / "calls").read_text()
    assert "env ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1 ROS_HOME_SET=yes" in calls  # from ros.env, never executed
    assert "args param get /rpp mission_speed --no-daemon --spin-time 1" in calls


def test_a_refusal_by_the_node_is_printed_verbatim_and_fails(rig):
    r = rig("set", "rpp", "wheel_track_m", "0.5", mode="refuse")
    assert r.returncode == 1
    assert "Setting parameter failed: wheel_track_m is IDLE_ONLY: refused while a mission is loaded" in r.stdout


def test_an_integer_for_a_double_gets_a_hint(rig):
    r = rig("set", "rpp", "mission_speed", "1", mode="type")
    assert r.returncode == 1
    assert "is of type {double}" in r.stdout and "'1.0'" in r.stderr


def test_an_accepted_set_says_it_is_not_persisted(rig):
    r = rig("set", "rpp", "mission_speed", "0.5")
    assert r.returncode == 0
    assert "Set parameter successful" in r.stdout and "save rpp" in r.stderr


def test_save_writes_atomically_prints_the_diff_and_keeps_no_backup(rig):
    r = rig("save", "rpp")
    assert r.returncode == 0, r.stderr
    target = rig.etc / "rpp.yaml"
    text = target.read_text()
    assert text.startswith("# Saved by dyx3-param save rpp at ")
    assert "mission_speed: 0.6" in text and text.rstrip().endswith("use_sim_time: false")
    assert "+    mission_speed: 0.6" in r.stdout  # the diff against the (absent) old file
    assert sorted(p.name for p in rig.etc.iterdir()) == ["ros.env", "rpp.yaml"]  # no temp, no backup
    assert oct(target.stat().st_mode & 0o777) == "0o644"
    r = rig("save", "rpp")
    assert "unchanged" in r.stdout
    (rig.bin / "dump.yaml").write_text(DUMP.format(speed=0.7))
    r = rig("save", "rpp")
    assert "-    mission_speed: 0.6" in r.stdout and "+    mission_speed: 0.7" in r.stdout
    assert "mission_speed: 0.7" in target.read_text()
    assert sorted(p.name for p in rig.etc.iterdir()) == ["ros.env", "rpp.yaml"]


def test_save_refuses_a_node_that_reads_no_parameter_file(rig):
    for node in ("gnss_rtk", "spray_watchdog"):
        r = rig("save", node)
        assert r.returncode == 3 and "refusing" in r.stderr
    assert not (rig.bin / "calls").exists()


def test_save_refuses_a_dump_that_is_not_the_nodes_document(rig):
    (rig.bin / "dump.yaml").write_text("Node not found\n")
    r = rig("save", "rpp")
    assert r.returncode == 1 and "unexpected ros2 param dump output" in r.stderr
    assert not (rig.etc / "rpp.yaml").exists()


def test_diff_compares_values_of_the_file_with_the_running_node(rig):
    pytest.importorskip("yaml")
    (rig.etc / "rpp.yaml").write_text("/rpp:\n  ros__parameters:\n    mission_speed: 0.6\n    max_linear_vel: 0.85\n")
    r = rig("diff", "rpp")
    assert r.returncode == 0, r.stdout + r.stderr
    assert "0 difference(s) in the 2 value(s)" in r.stdout and "1 running parameter(s) not in the file" in r.stdout
    (rig.bin / "dump.yaml").write_text(DUMP.format(speed=0.7))
    r = rig("diff", "rpp")
    assert r.returncode == 1
    assert "differs    mission_speed: running 0.7, file 0.6" in r.stdout


def test_diff_without_a_file_says_so(rig):
    r = rig("diff", "spray")
    assert r.returncode == 3 and "built-in defaults" in r.stderr
