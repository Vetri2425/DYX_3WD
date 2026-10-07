"""Legacy output stage: NED velocity vector + body yaw rate -> MotionSetpoint fields.

Pure Python, no ROS. This is the ONLY behavioural change relative to PX4_DXP's
``rpp_controller_node.py`` (architecture 7.5), and therefore the only thing in this package
that is new code.

Why a mapping is needed at all
------------------------------
The prototype published ``/rpp/velocity_ned`` (a NED velocity VECTOR) and PX4's velocity
OFFBOARD branch derived ``speed = |v|`` and ``yaw = atan2(vE, vN)`` from it, plus the old
fork's spot-turn FSM turned the rover in place when that bearing was far from the nose.
Under the production contract (docs/contracts/frames.md, F1.7 explicit-control contract)
rotation is explicit, so the vector is decoded back into the MotionSetpoint modes.

Every constant below that is not a prototype parameter is marked
``DERIVED — NOT FROM V1 SPEC`` and must be re-validated at GATE 1 (bench) and GATE 4.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

# MotionSetpoint ABI values (dyx3_interfaces/msg/MotionSetpoint.msg). Duplicated here ON
# PURPOSE so this module stays importable without ROS; test_output_stage.py pins them
# against the rosidl-generated constants when ROS is available.
MODE_STOP = 0
MODE_TRACK_HEADING = 1
MODE_TRACK_RATE = 2
MODE_PIVOT = 3
MODE_CREEP = 4

NAN = float("nan")


def wrap_pi(angle: float) -> float:
    """Wrap to [-pi, pi)."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


@dataclass(frozen=True)
class OutputPolicy:
    """Decoding policy. Defaults and their sources:

    stop_speed_mps      0.01   prototype: PX4 P4 patch / firmware ZERO_VEL_THRESHOLD "|v| < 1 cm/s"
                               (rpp_controller_node.py module docstring; test_corner_pivot.py).
    pivot_enter_deg     30.0   prototype docstring: "RD_TRANS_DRV_TRN (~30 deg)" — the old firmware's
                               drive->turn spot-turn threshold. DERIVED: reused as the PIVOT entry.
    pivot_exit_deg      5.0    prototype docstring: "RD_TRANS_TRN_DRV (~5 deg)". DERIVED: PIVOT exit.
    pivot_rate_gain     1.5    RO_YAW_P = 1.5 (V1 spec 3.1 / PX4_DXP CLAUDE.md). DERIVED: reused as
                               the pivot yaw-rate P gain, so the pivot approximates the pure-P
                               yaw loop the prototype ran under.
    max_yaw_rate_radps  0.45   prototype parameter ``max_yaw_rate_body`` default (demo-ready).
    mode                'heading' | 'rate'. 'heading' reproduces the prototype information flow
                               (a bearing, derived from the vector). 'rate' forwards the legacy
                               feed-forward yaw rate (TRACK_RATE) — only meaningful when
                               ``yaw_rate_feedback_gain`` > 0, else heading is open-loop.
    """

    stop_speed_mps: float = 0.01
    pivot_enter_rad: float = math.radians(30.0)
    pivot_exit_rad: float = math.radians(5.0)
    pivot_rate_gain: float = 1.5
    max_yaw_rate_radps: float = 0.45
    mode: str = "heading"


@dataclass(frozen=True)
class Setpoint:
    """Fields of dyx3_interfaces/msg/MotionSetpoint (minus stamp/seq, set by the node)."""

    mode: int
    speed_body_x: float
    yaw_setpoint: float
    yaw_rate_setpoint: float
    valid: bool


def _stop(valid: bool = True) -> Setpoint:
    # MotionSetpoint contract for STOP: speed 0, yaw NaN, yaw_rate 0.
    return Setpoint(MODE_STOP, 0.0, NAN, 0.0, valid)


class OutputStage:
    """Stateful decoder. State = the PIVOT hysteresis latch only."""

    def __init__(self, policy: OutputPolicy | None = None) -> None:
        self.policy = policy or OutputPolicy()
        if self.policy.mode not in ("heading", "rate"):
            raise ValueError(f"mode must be 'heading' or 'rate', got {self.policy.mode!r}")
        if not (self.policy.pivot_exit_rad < self.policy.pivot_enter_rad):
            raise ValueError("pivot_exit must be < pivot_enter (hysteresis)")
        self._pivoting = False

    @property
    def pivoting(self) -> bool:
        return self._pivoting

    def reset(self) -> None:
        self._pivoting = False

    def step(self, v_n: float, v_e: float, yaw_rate_body: float, yaw_ned: float) -> Setpoint:
        """Decode one legacy tick.

        v_n, v_e       NED velocity vector the prototype would have published (m/s)
        yaw_rate_body  prototype feed-forward yaw rate, NED/FRD clockwise-positive (rad/s)
        yaw_ned        current heading, NED, 0 = North, clockwise-positive (rad)
        """
        p = self.policy
        if not all(math.isfinite(x) for x in (v_n, v_e, yaw_rate_body, yaw_ned)):
            # Never decode garbage into motion: invalid == STOP (fail to zero).
            self._pivoting = False
            return _stop(valid=False)

        speed = math.hypot(v_n, v_e)
        if speed < p.stop_speed_mps:
            # The prototype relied on the firmware freezing heading below 1 cm/s.
            self._pivoting = False
            return _stop()

        bearing = math.atan2(v_e, v_n)  # NED: 0 = North, clockwise-positive
        err = wrap_pi(bearing - yaw_ned)  # >0: target is clockwise of the nose

        if self._pivoting:
            if abs(err) < p.pivot_exit_rad:
                self._pivoting = False
        elif abs(err) > p.pivot_enter_rad:
            self._pivoting = True

        if self._pivoting:
            rate = max(-p.max_yaw_rate_radps, min(p.max_yaw_rate_radps, p.pivot_rate_gain * err))
            return Setpoint(MODE_PIVOT, 0.0, NAN, rate, True)

        if p.mode == "rate":
            rate = max(-p.max_yaw_rate_radps, min(p.max_yaw_rate_radps, yaw_rate_body))
            return Setpoint(MODE_TRACK_RATE, speed, NAN, rate, True)
        return Setpoint(MODE_TRACK_HEADING, speed, wrap_pi(bearing), NAN, True)
