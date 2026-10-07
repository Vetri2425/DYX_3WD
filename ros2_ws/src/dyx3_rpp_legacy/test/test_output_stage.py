"""Output-stage decode tests (pure Python, no ROS).

Expectations here are DEFINITIONAL (frame conventions, the MotionSetpoint contract table in
msg/MotionSetpoint.msg) — none is copied from the implementation. Evidence-based checks
against recorded prototype behaviour live in test_fixture_decode.py.
"""
import math
import random

import pytest

from dyx3_rpp_legacy.output_stage import (
    MODE_CREEP,
    MODE_PIVOT,
    MODE_STOP,
    MODE_TRACK_HEADING,
    MODE_TRACK_RATE,
    OutputPolicy,
    OutputStage,
    wrap_pi,
)

D = math.radians


def check_contract(sp):
    """The rotation-command contract table from dyx3_interfaces/msg/MotionSetpoint.msg."""
    if sp.mode == MODE_STOP:
        assert sp.speed_body_x == 0.0 and math.isnan(sp.yaw_setpoint) and sp.yaw_rate_setpoint == 0.0
    elif sp.mode == MODE_TRACK_HEADING:
        assert math.isfinite(sp.yaw_setpoint) and math.isnan(sp.yaw_rate_setpoint)
        assert -math.pi <= sp.yaw_setpoint <= math.pi
    elif sp.mode == MODE_TRACK_RATE:
        assert math.isnan(sp.yaw_setpoint) and math.isfinite(sp.yaw_rate_setpoint)
    elif sp.mode == MODE_PIVOT:
        assert sp.speed_body_x == 0.0 and math.isnan(sp.yaw_setpoint) and math.isfinite(sp.yaw_rate_setpoint)
    elif sp.mode == MODE_CREEP:
        assert math.isnan(sp.yaw_setpoint) and math.isfinite(sp.yaw_rate_setpoint)
    else:
        pytest.fail(f"unknown mode {sp.mode}")


def test_zero_vector_is_a_valid_stop():
    sp = OutputStage().step(0.0, 0.0, 0.0, 1.0)
    assert sp.mode == MODE_STOP and sp.valid
    check_contract(sp)


def test_below_one_cm_per_s_is_stop():
    # Prototype relied on the firmware freezing heading below 1 cm/s.
    assert OutputStage().step(0.005, 0.005, 0.0, 0.0).mode == MODE_STOP


@pytest.mark.parametrize("bad", [float("nan"), float("inf"), -float("inf")])
@pytest.mark.parametrize("slot", range(4))
def test_non_finite_input_is_invalid_stop(bad, slot):
    args = [0.3, 0.1, 0.0, 0.2]
    args[slot] = bad
    sp = OutputStage().step(*args)
    assert sp.mode == MODE_STOP and sp.valid is False
    check_contract(sp)


def test_heading_mode_cardinal_directions():
    # NED heading: 0 = North, +90 deg = East (clockwise), +/-180 = South.
    st = OutputStage()
    n = st.step(0.3, 0.0, 0.0, 0.0)
    assert n.mode == MODE_TRACK_HEADING and n.yaw_setpoint == pytest.approx(0.0) and n.speed_body_x == pytest.approx(0.3)
    e = OutputStage().step(0.0, 0.3, 0.0, D(90))
    assert e.yaw_setpoint == pytest.approx(D(90))
    w = OutputStage().step(0.0, -0.3, 0.0, D(-90))
    assert w.yaw_setpoint == pytest.approx(D(-90))


def test_speed_magnitude_is_the_vector_norm():
    for vn, ve in [(0.3, 0.4), (0.2, 0.0), (0.0, 0.2)]:
        sp = OutputStage().step(vn, ve, 0.0, math.atan2(ve, vn))  # nose on the vector
        assert sp.speed_body_x == pytest.approx(math.hypot(vn, ve))


def test_brake_vector_decodes_as_reverse_holding_heading():
    # _corner_brake_velocity is exactly -+ along the nose. Moving forward at 0.1 m/s with the
    # nose at 60 deg, the brake vector points at bearing 240 deg: a reverse command that must
    # HOLD the nose heading, not spot-turn 180 deg (the prototype's BUG-T3 failure mode).
    yaw = D(60)
    b = yaw + math.pi
    sp = OutputStage().step(0.08 * math.cos(b), 0.08 * math.sin(b), 0.0, yaw)
    assert sp.mode == MODE_TRACK_HEADING
    assert sp.speed_body_x == pytest.approx(-0.08)
    assert sp.yaw_setpoint == pytest.approx(yaw)
    check_contract(sp)


def test_reverse_pivot_uses_the_nose_target_not_the_vector_bearing():
    # Vector 170 deg behind a north-facing nose: reversing, nose target is -10 deg => small
    # error, so TRACK_HEADING (reverse), not PIVOT.
    b = D(170)
    sp = OutputStage().step(0.1 * math.cos(b), 0.1 * math.sin(b), 0.0, 0.0)
    assert sp.mode == MODE_TRACK_HEADING and sp.speed_body_x < 0.0
    assert sp.yaw_setpoint == pytest.approx(D(170) - math.pi)


def test_ninety_degrees_off_the_nose_is_a_forward_pivot_not_a_reverse():
    sp = OutputStage().step(0.0, 0.1, 0.0, 0.0)  # bearing 90 deg, nose north
    assert sp.mode == MODE_PIVOT and sp.yaw_rate_setpoint > 0.0


def test_pivot_hysteresis_enter_40_exit_2():
    st = OutputStage()
    yaw = 0.0

    def vec(err_deg):
        b = D(err_deg)
        return 0.1 * math.cos(b), 0.1 * math.sin(b)  # bearing err_deg from a north-facing nose

    assert st.step(*vec(39.0), 0.0, yaw).mode == MODE_TRACK_HEADING  # below enter
    assert st.step(*vec(41.0), 0.0, yaw).mode == MODE_PIVOT  # enter
    assert st.step(*vec(20.0), 0.0, yaw).mode == MODE_PIVOT  # latched
    assert st.step(*vec(3.0), 0.0, yaw).mode == MODE_PIVOT  # still latched above exit
    assert st.step(*vec(1.5), 0.0, yaw).mode == MODE_TRACK_HEADING  # exit
    assert st.step(*vec(20.0), 0.0, yaw).mode == MODE_TRACK_HEADING  # does not re-enter below 40


@pytest.mark.parametrize("err_deg,sign", [(60.0, +1), (-60.0, -1)])
def test_pivot_rate_sign_is_clockwise_positive(err_deg, sign):
    b = D(err_deg)
    sp = OutputStage().step(0.1 * math.cos(b), 0.1 * math.sin(b), 0.0, 0.0)
    assert sp.mode == MODE_PIVOT and sp.speed_body_x == 0.0
    assert math.copysign(1.0, sp.yaw_rate_setpoint) == sign  # target clockwise of nose => +
    check_contract(sp)


def test_pivot_rate_is_clamped_to_max():
    st = OutputStage(OutputPolicy(max_yaw_rate_radps=0.45, pivot_rate_gain=10.0))
    sp = st.step(0.0, 0.1, 0.0, 0.0)  # 90 deg error * gain 10 = 15 rad/s requested
    assert sp.yaw_rate_setpoint == pytest.approx(0.45)


def test_pivot_error_wraps_through_south():
    # Nose 170 deg, target -170 deg => shortest turn is +20 deg (clockwise), not -340.
    st = OutputStage(OutputPolicy(pivot_enter_rad=D(10.0)))
    b = D(-170.0)
    sp = st.step(0.1 * math.cos(b), 0.1 * math.sin(b), 0.0, D(170.0))
    assert sp.mode == MODE_PIVOT and sp.yaw_rate_setpoint > 0.0
    assert wrap_pi(D(-170.0) - D(170.0)) == pytest.approx(D(20.0))


def test_rate_mode_forwards_clamped_feed_forward():
    st = OutputStage(OutputPolicy(mode="rate", max_yaw_rate_radps=0.45))
    sp = st.step(0.3, 0.0, 0.2, 0.0)
    assert sp.mode == MODE_TRACK_RATE and sp.yaw_rate_setpoint == pytest.approx(0.2) and math.isnan(sp.yaw_setpoint)
    assert st.step(0.3, 0.0, 9.0, 0.0).yaw_rate_setpoint == pytest.approx(0.45)
    assert st.step(0.3, 0.0, -9.0, 0.0).yaw_rate_setpoint == pytest.approx(-0.45)


def test_bad_policy_is_rejected():
    with pytest.raises(ValueError):
        OutputStage(OutputPolicy(mode="velocity"))
    with pytest.raises(ValueError):
        OutputStage(OutputPolicy(pivot_enter_rad=D(5), pivot_exit_rad=D(30)))


def test_every_output_satisfies_the_motion_setpoint_contract():
    rng = random.Random(20261007)  # fixed seed: deterministic
    for mode in ("heading", "rate"):
        st = OutputStage(OutputPolicy(mode=mode))
        for _ in range(5000):
            sp = st.step(
                rng.uniform(-1.0, 1.0),
                rng.uniform(-1.0, 1.0),
                rng.uniform(-1.0, 1.0),
                rng.uniform(-math.pi, math.pi),
            )
            check_contract(sp)


def test_abi_constants_match_generated_interface():
    msg = pytest.importorskip("dyx3_interfaces.msg", reason="needs a sourced ROS workspace")
    M = msg.MotionSetpoint
    assert (M.MODE_STOP, M.MODE_TRACK_HEADING, M.MODE_TRACK_RATE, M.MODE_PIVOT, M.MODE_CREEP) == (
        MODE_STOP,
        MODE_TRACK_HEADING,
        MODE_TRACK_RATE,
        MODE_PIVOT,
        MODE_CREEP,
    )
