"""The PX4_DXP RPP controller (verbatim, ``_dxp/rpp_controller_node.py``) with ONLY its
output stage rewired: ``_publish_velocity`` / ``_publish_yaw_rate`` publish a
``dyx3_interfaces/MotionSetpoint`` instead of ``/rpp/velocity_ned`` + ``/rpp/yaw_rate_body``.

QUARANTINED (CLAUDE.md §8): never shipped to a production rover, nothing depends on it,
deleted at Gate 7.

The verbatim module is imported unmodified; the rewiring is a subclass, so
``test/test_verbatim.py`` can prove byte-identity of the ancestor.
"""

from __future__ import annotations

import math
import os
import sys

# The prototype modules import each other by bare name (`import mission_progress`); keep the
# verbatim directory importable without touching a single line of them.
_DXP_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "_dxp")
if _DXP_DIR not in sys.path:
    sys.path.insert(0, _DXP_DIR)

import rclpy  # noqa: E402
from rclpy.executors import ExternalShutdownException  # noqa: E402
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402

from dyx3_interfaces.msg import MotionSetpoint  # noqa: E402
from dyx3_rpp_legacy.output_stage import OutputPolicy, OutputStage  # noqa: E402
import rpp_controller_node as _dxp  # noqa: E402  (verbatim PX4_DXP module)

DEFAULT_OUTPUT_TOPIC = "/rpp/motion_setpoint"


class LegacyRppNode(_dxp.RPPControllerNode):
    """Verbatim controller + MotionSetpoint output stage."""

    def __init__(self) -> None:
        super().__init__()
        # New parameters (they do not collide with any of the 119 inherited names).
        self.declare_parameter("output_topic", DEFAULT_OUTPUT_TOPIC)
        self.declare_parameter("output_mode", "heading")  # 'heading' | 'rate'
        self.declare_parameter("output_pivot_enter_deg", 30.0)
        self.declare_parameter("output_pivot_exit_deg", 5.0)
        self.declare_parameter("output_pivot_rate_gain", 1.5)
        self.declare_parameter("output_stop_speed_mps", 0.01)

        mode = str(self.get_parameter("output_mode").value)
        if mode == "rate" and float(self.get_parameter("yaw_rate_feedback_gain").value) <= 0.0:
            # TRACK_RATE with no heading feedback is open-loop in heading: refuse it.
            self.get_logger().error(
                "output_mode='rate' needs yaw_rate_feedback_gain > 0 (otherwise heading is "
                "open-loop). Falling back to 'heading'."
            )
            mode = "heading"
        self._stage = OutputStage(
            OutputPolicy(
                stop_speed_mps=float(self.get_parameter("output_stop_speed_mps").value),
                pivot_enter_rad=math.radians(float(self.get_parameter("output_pivot_enter_deg").value)),
                pivot_exit_rad=math.radians(float(self.get_parameter("output_pivot_exit_deg").value)),
                pivot_rate_gain=float(self.get_parameter("output_pivot_rate_gain").value),
                max_yaw_rate_radps=float(self.get_parameter("max_yaw_rate_body").value),
                mode=mode,
            )
        )
        sp_qos = QoSProfile(  # architecture §8: setpoints BEST_EFFORT + KEEP_LAST(1)
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self._sp_pub = self.create_publisher(
            MotionSetpoint, str(self.get_parameter("output_topic").value), sp_qos
        )
        self._seq = 0
        self._pending_vel: tuple[float, float] | None = None
        self.get_logger().warn(
            f"QUARANTINED legacy RPP: output_mode={mode}, topic="
            f"{self.get_parameter('output_topic').value}. Shadow oracle only; never ship."
        )

    # ------------------------------------------------------------------ output stage
    # The prototype always calls _publish_velocity() immediately followed by
    # _publish_yaw_rate() (see every call site in _dxp/rpp_controller_node.py), so the pair
    # is joined here and emitted once, on the second call.
    def _publish_velocity(self, v_n: float, v_e: float) -> None:  # noqa: D401
        self._pending_vel = (float(v_n), float(v_e))

    def _publish_yaw_rate(self, yaw_rate_body: float) -> None:
        if self._pending_vel is None:
            # Unpaired yaw-rate: never emit motion from half a command.
            self._emit_stop(valid=False)
            return
        v_n, v_e = self._pending_vel
        self._pending_vel = None
        yaw_ned = self._current_yaw_ned()
        sp = self._stage.step(v_n, v_e, float(yaw_rate_body), yaw_ned)
        self._emit(sp.mode, sp.speed_body_x, sp.yaw_setpoint, sp.yaw_rate_setpoint, sp.valid)

    def _current_yaw_ned(self) -> float:
        pose = getattr(self, "_pose", None)
        if pose is None:
            return float("nan")  # OutputStage turns this into an invalid STOP
        return self._enu_pose_to_ned(pose)[2]

    def _emit_stop(self, valid: bool = True) -> None:
        self._stage.reset()
        self._emit(0, 0.0, float("nan"), 0.0, valid)

    def _emit(self, mode: int, speed: float, yaw: float, rate: float, valid: bool) -> None:
        msg = MotionSetpoint()
        msg.stamp = self.get_clock().now().to_msg()
        msg.seq = self._seq
        self._seq += 1
        msg.mode = int(mode)
        msg.speed_body_x = float(speed)
        msg.yaw_setpoint = float(yaw)
        msg.yaw_rate_setpoint = float(rate)
        msg.valid = bool(valid)
        self._sp_pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = None
    try:
        node = LegacyRppNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node:
            try:  # last gasp: fail to zero
                node._emit_stop()
                node._publish_spray_active(False)
            except Exception:  # noqa: BLE001 - best effort on the way out
                pass
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
