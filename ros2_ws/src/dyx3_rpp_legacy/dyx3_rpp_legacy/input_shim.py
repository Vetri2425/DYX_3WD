"""State-input shim for the QUARANTINED legacy RPP.

The verbatim controller subscribes to MAVROS-typed topics (ENU pose, ENU velocity, GPSRAW).
Production has no MAVROS, so this node republishes the canonical state under those names:

  /dyx3/vehicle_state (VehicleState, NED)  ->  /mavros/local_position/pose            (ENU)
                                               /mavros/local_position/velocity_local  (ENU)
  /dyx3/rtk_status    (RtkStatus)          ->  /mavros/gpsstatus/gps1/raw             (GPSRAW)

DERIVED — NOT FROM V1 SPEC. Needs `mavros_msgs` (message definitions only; no MAVROS process).
NOT covered here: the `/path` feed (nav_msgs/Path, NED, position.z bitfield bit0=spray,
bit1=must-hit) and the point-handshake topics — those come from the path-artifact adapter
once dyx3_mission/Phase 4 define it. Untested on a rover.

Frame rules (docs/contracts/frames.md): ENU x = East = NED east, ENU y = North = NED north,
ENU z = Up = -down. ENU yaw (0 = East, CCW+) = pi/2 - NED heading. ENU yaw rate = -NED rate.
"""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import GPSRAW
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from dyx3_interfaces.msg import RtkStatus, VehicleState


def ned_heading_to_enu_quaternion(heading_ned: float) -> tuple[float, float, float, float]:
    """(x, y, z, w) of a pure-yaw ENU orientation for a NED heading."""
    yaw_enu = math.pi / 2.0 - heading_ned
    return 0.0, 0.0, math.sin(yaw_enu / 2.0), math.cos(yaw_enu / 2.0)


class LegacyInputShim(Node):
    def __init__(self) -> None:
        super().__init__("dyx3_rpp_legacy_input_shim")
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self._pose_pub = self.create_publisher(PoseStamped, "/mavros/local_position/pose", qos)
        self._vel_pub = self.create_publisher(TwistStamped, "/mavros/local_position/velocity_local", qos)
        self._gps_pub = self.create_publisher(GPSRAW, "/mavros/gpsstatus/gps1/raw", qos)
        self.create_subscription(VehicleState, "/dyx3/vehicle_state", self._on_state, qos)
        self.create_subscription(RtkStatus, "/dyx3/rtk_status", self._on_rtk, qos)

    def _on_state(self, s: VehicleState) -> None:
        # Fail safe: with no valid pose, publish nothing; the controller then goes STALE -> STOP.
        if not (s.position_valid and s.attitude_valid):
            return
        pose = PoseStamped()
        pose.header.stamp = s.stamp
        pose.header.frame_id = "map"
        pose.pose.position.x = float(s.east_m)
        pose.pose.position.y = float(s.north_m)
        pose.pose.position.z = -float(s.down_m)
        qx, qy, qz, qw = ned_heading_to_enu_quaternion(float(s.heading_rad))
        pose.pose.orientation.x, pose.pose.orientation.y = qx, qy
        pose.pose.orientation.z, pose.pose.orientation.w = qz, qw
        self._pose_pub.publish(pose)
        if s.velocity_valid:
            vel = TwistStamped()
            vel.header.stamp = s.stamp
            vel.header.frame_id = "map"
            vel.twist.linear.x = float(s.velocity_east_mps)
            vel.twist.linear.y = float(s.velocity_north_mps)
            vel.twist.linear.z = -float(s.velocity_down_mps)
            vel.twist.angular.z = -float(s.yaw_rate_radps)
            self._vel_pub.publish(vel)

    def _on_rtk(self, r: RtkStatus) -> None:
        gps = GPSRAW()
        gps.header.stamp = self.get_clock().now().to_msg()
        gps.fix_type = int(r.fix_type)  # same numbering as MAVLink GPS_FIX_TYPE (6 = RTK fixed)
        # Prototype reads h_acc in millimetres and treats 0 as "unknown".
        gps.h_acc = int(round(max(0.0, float(r.horizontal_accuracy_m)) * 1000.0))
        self._gps_pub.publish(gps)


def main() -> None:
    rclpy.init()
    node = LegacyInputShim()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
