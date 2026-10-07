"""Tool correctness on synthetic data (NOT evidence: real bags are a LOCAL ACTION)."""
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "analysis"))

import arc_floor as af
import timebase as tb


def _stream(offset_ns, drift_ppm, latency_ms, n=3000, hz=50.0, seed=1):
    rng = np.random.default_rng(seed)
    px4_us = 1_000_000 + np.arange(n) * 1e6 / hz
    px4_ns = px4_us * 1000.0
    true_bag = offset_ns + (1 + drift_ppm * 1e-6) * px4_ns
    lat = latency_ms * 1e6 * (1.0 + rng.exponential(1.0, n))  # always >= latency_ms
    return px4_us, true_bag + lat


def test_clock_map_recovers_offset_drift_and_latency_floor():
    px4_us, bag_ns = _stream(offset_ns=1.7e15, drift_ppm=25.0, latency_ms=2.0)
    m = tb.estimate_clock_map(px4_us, bag_ns, window_s=5.0)
    # recovered at a mid-run px4 time within a fraction of a ms (the exponential tail makes the minimum slightly pessimistic)
    mid = px4_us[len(px4_us) // 2]
    truth = 1.7e15 + (1 + 25e-6) * mid * 1000.0
    # the 2 ms latency floor is indistinguishable from the offset, so the map is biased by about that much and no more
    assert 0.0 <= m.px4_us_to_bag_ns(mid) - truth < 5e6
    assert m.drift_ppm == pytest.approx(25.0, abs=3.0)
    assert m.latency_median_ms >= 0.0 and m.latency_p99_ms >= m.latency_median_ms
    assert m.n == len(px4_us)


def test_clock_map_with_a_single_window_reports_zero_drift_and_rejects_tiny_input():
    px4_us, bag_ns = _stream(1e15, 0.0, 1.0, n=200)
    m = tb.estimate_clock_map(px4_us, bag_ns, window_s=1e6)
    assert m.drift_ppm == 0.0
    with pytest.raises(ValueError):
        tb.estimate_clock_map(px4_us[:5], bag_ns[:5])
    with pytest.raises(ValueError):
        tb.estimate_clock_map(px4_us, bag_ns[:-1])


def test_the_forty_millisecond_symptom_costs_1_4_cm():
    assert tb.offset_error_to_position_cm(40.0, 0.35) == pytest.approx(1.4)
    assert tb.offset_error_to_position_cm(-4.0, 0.35) == pytest.approx(0.14)


def test_cross_correlation_finds_a_known_lag_to_sub_sample_accuracy():
    t = np.arange(0, 60, 0.01)
    rng = np.random.default_rng(3)
    sig = np.convolve(rng.normal(size=t.size), np.ones(25) / 25, mode="same")  # band-limited
    lag_true = 0.0374
    # b's clock reads `lag_true` more than a's for the same instant: b(tau) = a(tau - lag_true), so a(t) = b(t + lag_true)
    t_b = t + lag_true
    lag, peak = tb.cross_correlation_lag(t, sig, t_b, sig, max_lag_s=0.5, dt_s=0.002)
    assert peak > 0.95
    assert lag == pytest.approx(lag_true, abs=0.002)
    # unrelated noise is flagged by a low peak
    noise = np.random.default_rng(9).normal(size=t.size)
    _, low = tb.cross_correlation_lag(t, sig, t, noise, max_lag_s=0.5, dt_s=0.002)
    assert low < 0.5
    with pytest.raises(ValueError):
        tb.cross_correlation_lag(t[:100], sig[:100], t[:100], sig[:100], max_lag_s=0.5)


def test_arc_floor_recovers_the_pure_p_slope_and_flags_a_different_one():
    rng = np.random.default_rng(5)
    w = rng.uniform(-0.4, 0.4, 2000)
    for slope_true, expect_ratio in ((1 / 1.5, 1.0), (0.3, 0.45)):
        e = slope_true * w + rng.normal(0, 0.002, w.size)
        fit = af.fit_floor(w, e, ro_yaw_p=1.5)
        assert fit.slope == pytest.approx(slope_true, rel=0.03)
        assert fit.ratio == pytest.approx(expect_ratio, rel=0.03)
        assert fit.r2 > 0.9
        assert fit.expected_slope == pytest.approx(1 / 1.5)


def test_arc_floor_input_validation_and_straight_sample_filter(tmp_path, capsys):
    with pytest.raises(ValueError):
        af.fit_floor([0.0] * 20, [0.1] * 20, 1.5)  # no turn rate at all
    with pytest.raises(ValueError):
        af.fit_floor([0.1] * 20, [0.1] * 20, 0.0)
    with pytest.raises(ValueError):
        af.fit_floor([0.1] * 5, [0.1] * 5, 1.5)
    w = np.concatenate([np.zeros(50), np.linspace(0.1, 0.4, 100)])
    e = np.concatenate([np.full(50, 0.05), np.linspace(0.1, 0.4, 100) / 1.5])  # straight samples carry an unrelated bias
    biased = af.fit_floor(w, e, 1.5)
    clean = af.fit_floor(w, e, 1.5, min_abs_omega=0.05)
    assert clean.ratio == pytest.approx(1.0, rel=1e-6)
    assert clean.n == 100 and biased.n == 150
    p = tmp_path / "arc.csv"
    p.write_text("omega_radps,heading_err_rad\n" + "\n".join(f"{a},{b}" for a, b in zip(w, e)))
    assert af.main([str(p), "--ro-yaw-p", "1.5", "--min-abs-omega", "0.05", "--xtrack-per-rad", "0.5"]) == 0
    out = capsys.readouterr().out
    assert '"ratio"' in out and "floor_rms_cm_if_xtrack_per_rad" in out
