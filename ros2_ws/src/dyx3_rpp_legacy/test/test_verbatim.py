"""The verbatim PX4_DXP copies must stay byte-identical to the recorded hashes."""
import hashlib
import os

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TEST = os.path.join(PKG, "test")


def _entries():
    rows = []
    for line in open(os.path.join(TEST, "VERBATIM.sha256"), encoding="utf-8"):
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        digest, path = line.split(None, 1)
        rows.append((digest, path.strip()))
    return rows


def _resolve(rel):
    base = os.path.join(PKG, "dyx3_rpp_legacy") if rel.startswith("_dxp/") else TEST
    return os.path.join(base, rel)


def test_recorded_hashes_exist_for_all_files():
    rows = _entries()
    assert len(rows) >= 6 + 20  # 6 modules + the prototype tests
    names = {p for _, p in rows}
    for must in ("_dxp/rpp_controller_node.py", "_dxp/mission_progress.py", "_dxp/precise_stop.py"):
        assert must in names


def test_verbatim_files_are_unmodified():
    for digest, rel in _entries():
        with open(_resolve(rel), "rb") as fh:
            assert hashlib.sha256(fh.read()).hexdigest() == digest, f"{rel} was modified"


def test_no_unrecorded_file_in_verbatim_dirs():
    recorded = {p for _, p in _entries()}
    for sub, prefix in (("dyx3_rpp_legacy/_dxp", "_dxp/"), ("test/dxp_verbatim", "dxp_verbatim/")):
        for f in os.listdir(os.path.join(PKG, sub)):
            if f.endswith(".py") and f != "__init__.py":
                assert prefix + f in recorded, f"{prefix}{f} is not in VERBATIM.sha256"


def test_legacy_node_overrides_only_the_output_stage():
    import pytest

    pytest.importorskip("rclpy", reason="needs ROS")
    pytest.importorskip("mavros_msgs", reason="needs mavros_msgs (messages only)")
    from dyx3_rpp_legacy import legacy_node

    base = legacy_node._dxp.RPPControllerNode
    own = {k for k in vars(legacy_node.LegacyRppNode) if not k.startswith("__")}
    overridden = {k for k in own if hasattr(base, k)}
    assert overridden == {"_publish_velocity", "_publish_yaw_rate"}, overridden
