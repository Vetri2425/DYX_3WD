"""pytest wiring for the quarantined legacy package.

* ``test/dxp_verbatim`` holds the PX4_DXP prototype's OWN tests, byte-for-byte. They need
  rclpy + mavros_msgs and import the prototype modules by bare name, so they are collected
  only when DYX3_RUN_DXP_TESTS=1 (the local Mac Claude / a ROS container).
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
sys.path.insert(0, PKG)
sys.path.insert(0, os.path.join(PKG, "dyx3_rpp_legacy", "_dxp"))

collect_ignore = []
if os.environ.get("DYX3_RUN_DXP_TESTS") != "1":
    collect_ignore.append("dxp_verbatim")
