"""Tests the EXTRACTOR on a synthetic rosbag2 (tool correctness only — it is not evidence
about the controller; real evidence comes from PX4_DXP bags, see HANDOFF)."""
import math
import sys
from pathlib import Path

import pytest

pytest.importorskip("rosbags")
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from rosbags.rosbag2 import Writer  # noqa: E402
from rosbags.typesys import Stores, get_typestore  # noqa: E402

import extract_legacy_decode_fixture as ex  # noqa: E402


def _bag(path: Path):
    ts = get_typestore(Stores.ROS2_HUMBLE)
    T = ts.types
    with Writer(path, version=9) as w:
        c_v = w.add_connection("/rpp/velocity_ned", "geometry_msgs/msg/Vector3Stamped", typestore=ts)
        c_y = w.add_connection("/rpp/yaw_rate_body", "std_msgs/msg/Float32", typestore=ts)
        c_p = w.add_connection("/mavros/local_position/pose", "geometry_msgs/msg/PoseStamped", typestore=ts)
        c_s = w.add_connection("/rpp/segment_debug", "std_msgs/msg/Float32MultiArray", typestore=ts)
        c_d = w.add_connection("/rpp/debug", "std_msgs/msg/Float32MultiArray", typestore=ts)

        def hdr(t):
            return T["std_msgs/msg/Header"](stamp=T["builtin_interfaces/msg/Time"](sec=0, nanosec=t), frame_id="x")

        def arr(vals):
            return T["std_msgs/msg/Float32MultiArray"](
                layout=T["std_msgs/msg/MultiArrayLayout"](dim=[], data_offset=0), data=np.array(vals, dtype=np.float32)
            )

        yaw_enu = math.radians(30.0)  # => NED heading 60 deg
        q = T["geometry_msgs/msg/Quaternion"](x=0.0, y=0.0, z=math.sin(yaw_enu / 2), w=math.cos(yaw_enu / 2))
        pose = T["geometry_msgs/msg/PoseStamped"](
            header=hdr(1), pose=T["geometry_msgs/msg/Pose"](position=T["geometry_msgs/msg/Point"](x=0.0, y=0.0, z=0.0), orientation=q)
        )
        w.write(c_p, 1, ts.serialize_cdr(pose, "geometry_msgs/msg/PoseStamped"))
        w.write(c_y, 2, ts.serialize_cdr(T["std_msgs/msg/Float32"](data=0.1), "std_msgs/msg/Float32"))
        w.write(c_s, 3, ts.serialize_cdr(arr([1, 3, 0, 0, 0, 0, 1.0, 0, 0, 0]), "std_msgs/msg/Float32MultiArray"))
        w.write(c_d, 4, ts.serialize_cdr(arr([0, 0, 0, 0.25, 0, 0, 0, 1]), "std_msgs/msg/Float32MultiArray"))
        vec = T["geometry_msgs/msg/Vector3Stamped"](header=hdr(5), vector=T["geometry_msgs/msg/Vector3"](x=0.2, y=0.1, z=0.0))
        w.write(c_v, 1_000_000_000, ts.serialize_cdr(vec, "geometry_msgs/msg/Vector3Stamped"))
        # a velocity tick BEFORE the other inputs must be dropped
    return


import numpy as np  # noqa: E402


def test_extract_roundtrip(tmp_path):
    bag = tmp_path / "bag"
    _bag(bag)
    fx = ex.extract(bag)
    assert len(fx["ticks"]) == 1
    t = fx["ticks"][0]
    assert t["v_n"] == pytest.approx(0.2) and t["v_e"] == pytest.approx(0.1)
    assert t["yaw_rate_body"] == pytest.approx(0.1)
    assert t["seg_state"] == 3
    assert t["speed_cmd"] == pytest.approx(0.25)
    # ENU yaw 30 deg == NED heading 60 deg (pi/2 - 30deg)
    assert math.degrees(t["yaw_ned"]) == pytest.approx(60.0, abs=1e-4)


def test_yaw_formula_known_values():
    class Q:  # pure ENU yaw
        def __init__(self, yaw):
            self.x = self.y = 0.0
            self.z = math.sin(yaw / 2)
            self.w = math.cos(yaw / 2)

    assert ex.yaw_ned_from_enu_pose(Q(0.0)) == pytest.approx(math.pi / 2)  # facing East
    assert ex.yaw_ned_from_enu_pose(Q(math.pi / 2)) == pytest.approx(0.0)  # facing North
