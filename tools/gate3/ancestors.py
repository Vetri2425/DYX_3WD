"""Load the VERBATIM PX4_DXP RPP controller class so its geometry methods can be called directly.

GATE 3 (spec 7.2): every geometry function that exists on both sides must be proven numerically
equivalent to the Python implementation before the C++ version enters the loop. The Python side
of the comparison is the *carried, byte-identical* controller module (see
ros2_ws/src/dyx3_rpp_legacy/test/VERBATIM.sha256), never a re-implementation.

The module imports rclpy / ROS message packages at import time. When they are not installed
(a laptop, CI without ROS) minimal stand-ins are registered: only static/class methods and a few
instance methods that read plain attributes are used, so no ROS behaviour is needed.
"""
from __future__ import annotations

import importlib
import os
import sys
import types
from types import SimpleNamespace

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
DXP_DIR = os.path.join(REPO, "ros2_ws", "src", "dyx3_rpp_legacy", "dyx3_rpp_legacy", "_dxp")

_STUB_MODULES = {
    "rclpy": ["init", "spin", "try_shutdown"],
    "rclpy.executors": ["ExternalShutdownException"],
    "rclpy.node": ["Node"],
    "rclpy.time": ["Time"],
    "rclpy.qos": ["QoSProfile", "ReliabilityPolicy", "DurabilityPolicy", "HistoryPolicy"],
    "geometry_msgs": [],
    "geometry_msgs.msg": ["PoseStamped", "Vector3Stamped", "TwistStamped"],
    "mavros_msgs": [],
    "mavros_msgs.msg": ["GPSRAW"],
    "nav_msgs": [],
    "nav_msgs.msg": ["Path"],
    "std_msgs": [],
    "std_msgs.msg": ["Bool", "Float32MultiArray", "MultiArrayDimension", "Float32", "String"],
}


class _Auto(type):
    """Stand-in class whose unknown attributes resolve to their own name (enum-like members such
    as ``ReliabilityPolicy.BEST_EFFORT``)."""

    def __getattr__(cls, name):
        if name.startswith("__"):
            raise AttributeError(name)
        return name


def _install_stubs() -> None:
    for name, attrs in _STUB_MODULES.items():
        if name in sys.modules:
            continue
        mod = types.ModuleType(name)
        for a in attrs:
            setattr(mod, a, _Auto(a, (), {}))
        sys.modules[name] = mod
    # package attributes so `from rclpy.qos import ...` style and dotted access both work
    for name in _STUB_MODULES:
        if "." in name:
            parent, child = name.rsplit(".", 1)
            setattr(sys.modules[parent], child, sys.modules[name])


def load_controller_class():
    """Return the verbatim ``RPPControllerNode`` class (stubbing ROS only if it is absent)."""
    if DXP_DIR not in sys.path:
        sys.path.insert(0, DXP_DIR)
    try:
        import rclpy  # noqa: F401
        import mavros_msgs.msg  # noqa: F401
    except ImportError:
        _install_stubs()
    mod = importlib.import_module("rpp_controller_node")
    return mod.RPPControllerNode


def make_node(cls, path_ne):
    """An instance WITHOUT running ``__init__`` (no ROS node), carrying just the state the
    geometry methods read: ``_path`` (pose-like objects), ``_hint_valid``, ``_closest_seg_hint``."""
    node = object.__new__(cls)
    node._path = [
        SimpleNamespace(pose=SimpleNamespace(position=SimpleNamespace(x=n, y=e, z=0.0)))
        for n, e in path_ne
    ]
    node._hint_valid = False
    node._closest_seg_hint = 0
    return node
