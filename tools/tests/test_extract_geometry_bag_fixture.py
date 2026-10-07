"""Tests the extractor on a synthetic rosbag2 (tool correctness only, not evidence)."""
import sys
from pathlib import Path

import numpy as np
import pytest

pytest.importorskip("rosbags")
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from rosbags.rosbag2 import Writer  # noqa: E402
from rosbags.typesys import Stores, get_typestore  # noqa: E402

import extract_geometry_bag_fixture as ex  # noqa: E402


def _bag(path: Path, n_ticks=150):
    ts = get_typestore(Stores.ROS2_HUMBLE)
    T = ts.types
    with Writer(path, version=9) as w:
        c_p = w.add_connection("/rpp/conditioned_path", "nav_msgs/msg/Path", typestore=ts)
        c_o = w.add_connection("/mavros/local_position/pose", "geometry_msgs/msg/PoseStamped", typestore=ts)
        c_d = w.add_connection("/rpp/debug", "std_msgs/msg/Float32MultiArray", typestore=ts)
        c_s = w.add_connection("/rpp/segment_debug", "std_msgs/msg/Float32MultiArray", typestore=ts)

        def hdr():
            return T["std_msgs/msg/Header"](stamp=T["builtin_interfaces/msg/Time"](sec=0, nanosec=0), frame_id="local_ned")

        def ps(n, e, z):
            return T["geometry_msgs/msg/PoseStamped"](
                header=hdr(),
                pose=T["geometry_msgs/msg/Pose"](
                    position=T["geometry_msgs/msg/Point"](x=n, y=e, z=z),
                    orientation=T["geometry_msgs/msg/Quaternion"](x=0.0, y=0.0, z=0.0, w=1.0),
                ),
            )

        def arr(v):
            return T["std_msgs/msg/Float32MultiArray"](
                layout=T["std_msgs/msg/MultiArrayLayout"](dim=[], data_offset=0), data=np.array(v, dtype=np.float32)
            )

        path = T["nav_msgs/msg/Path"](header=hdr(), poses=[ps(float(i), 0.0, 1.0) for i in range(11)])
        w.write(c_p, 1, ts.serialize_cdr(path, "nav_msgs/msg/Path"))
        t = 10
        for k in range(n_ticks):
            # ENU pose: x = East = 0.1 m right of a north-heading path, y = North
            w.write(c_o, t, ts.serialize_cdr(ps(0.1, 0.05 * k, 0.0), "geometry_msgs/msg/PoseStamped")); t += 1
            w.write(c_s, t, ts.serialize_cdr(arr([1, 1, 0, 0, 0, 0, 0, 0, 0, 0]), "std_msgs/msg/Float32MultiArray")); t += 1
            w.write(c_d, t, ts.serialize_cdr(arr([0.1, 0, 0, 0, 0, 0, 0, 1]), "std_msgs/msg/Float32MultiArray")); t += 1


def test_extract_roundtrip(tmp_path):
    bag = tmp_path / "bag"
    _bag(bag)
    path, ticks = ex.extract(bag)
    assert len(path) == 11 and len(ticks) == 150
    n, e, x = ticks[3]
    assert (n, e) == pytest.approx((0.15, 0.1))  # ENU -> NED swap
    assert x == pytest.approx(0.1)
    assert ex.single_run(path)
    out = tmp_path / "bag_xtrack_t.txt"
    ex.write(out, "t", path, ticks)
    text = out.read_text()
    assert text.startswith("BAGXTRACK 1\nPATH t 11\n") and text.count("TICK ") == 150


def test_multi_run_is_refused():
    flags = [1, 1, 0, 0, 1, 1, 0, 0]
    path = [(float(i), 0.0, f) for i, f in enumerate(flags)]
    assert not ex.single_run(path)
