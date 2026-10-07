"""Stage 0.2 — analysis-toolchain re-base for the DDS timebase (architecture 4.6, F-tasks A1.4).

MAVROS delivered ROS-time-stamped messages. The new stack's px4_msgs carry PX4 `hrt` microseconds; `dyx3_px4_link` receives them on the Jetson,
and the uXRCE-DDS client converts timestamps both ways inside the firmware using its own timesync (`TimesyncStatus.estimated_offset`). Every
bag<->ulog correlation and every gate in section 11 depends on knowing how a PX4 timestamp maps onto the bag's receive clock, and how wrong that
mapping is. This module is pure numpy (no ROS) so it runs on the Mac against real bags and in CI against synthetic data.

Three tools:
  * estimate_clock_map   : PX4 timestamps vs bag receive times -> offset, drift, latency floor, jitter (the lower-envelope method:
                           receive time = true time + latency, latency >= 0, so the minimum of (receive - px4) over a window is the offset).
  * offset_error_to_position_cm : what a timestamp error costs in position at a given speed (40 ms at 0.35 m/s = 1.4 cm).
  * cross_correlation_lag: residual lag between two recordings of the SAME physical signal (e.g. yaw rate in the bag and in the ulog).

Nothing here is evidence by itself: it returns numbers with the assumptions stated, to be run on real bags (LOCAL ACTION).
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class ClockMap:
    offset_ns: float          # bag_ns ~= offset_ns + (1 + drift) * px4_ns, at px4_ns = 0
    drift_ppm: float
    latency_median_ms: float  # median of (receive - mapped px4 time): transport + scheduling latency, >= 0 by construction
    latency_p99_ms: float
    jitter_ms: float          # robust spread (MAD * 1.4826) of the latency
    n: int

    def px4_us_to_bag_ns(self, px4_us):
        px4_ns = np.asarray(px4_us, dtype=float) * 1000.0
        return self.offset_ns + (1.0 + self.drift_ppm * 1e-6) * px4_ns


def estimate_clock_map(px4_us, bag_ns, window_s: float = 10.0) -> ClockMap:
    """Fit bag receive time against PX4 timestamps with the lower-envelope method.

    d_i = bag_ns_i - px4_ns_i = offset + drift*t + latency_i, latency_i >= 0. Take the minimum of d in each `window_s` window (the least-delayed
    sample), fit a line through those minima. Needs >= 2 windows to estimate drift; with one window drift is reported as 0.
    """
    px4_ns = np.asarray(px4_us, dtype=float) * 1000.0
    bag = np.asarray(bag_ns, dtype=float)
    if px4_ns.shape != bag.shape or px4_ns.size < 10:
        raise ValueError("need at least 10 paired samples of equal length")
    order = np.argsort(px4_ns)
    px4_ns, bag = px4_ns[order], bag[order]
    d = bag - px4_ns
    t0 = px4_ns[0]
    win = np.floor((px4_ns - t0) / (window_s * 1e9)).astype(int)
    xs, ys = [], []
    for w in np.unique(win):
        sel = win == w
        i = np.argmin(d[sel])
        xs.append(px4_ns[sel][i])
        ys.append(d[sel][i])
    xs_a, ys_a = np.array(xs), np.array(ys)
    if len(xs) >= 2:
        slope, c0 = np.polyfit(xs_a - t0, ys_a, 1)  # y = c0 + slope * (x - t0)
        drift = float(slope)
        offset_at_zero = float(c0) - drift * t0     # y = offset_at_zero + drift * x   (x = absolute px4_ns)
    else:
        drift = 0.0
        offset_at_zero = float(ys_a[0])
    lat = d - (offset_at_zero + drift * px4_ns)
    # The fit goes through the window MINIMA, so a few samples can sit slightly below it; latency is non-negative by definition.
    lat = np.maximum(lat, 0.0)
    med = float(np.median(lat))
    mad = float(np.median(np.abs(lat - med))) * 1.4826
    return ClockMap(offset_at_zero, drift * 1e6, med / 1e6, float(np.percentile(lat, 99)) / 1e6, mad / 1e6, int(lat.size))


def offset_error_to_position_cm(offset_error_ms: float, speed_mps: float) -> float:
    """Position error attributed to the wrong instant: |err| * speed. 40 ms at 0.35 m/s = 1.4 cm (F-tasks A1.4)."""
    return abs(offset_error_ms) * 1e-3 * speed_mps * 100.0


def cross_correlation_lag(t_a, y_a, t_b, y_b, max_lag_s: float = 1.0, dt_s: float = 0.002) -> tuple[float, float]:
    """Lag L (seconds) such that a(t) ~= b(t + L), and the normalised correlation at that lag.
    To align b onto a's clock SUBTRACT L from b's time stamps.

    Both signals are resampled on a common uniform grid; the peak is refined to sub-sample with a parabola. A peak correlation below ~0.5 means
    the two traces are not the same signal (or too noisy): do not trust the lag.
    """
    t_a, y_a, t_b, y_b = (np.asarray(v, dtype=float) for v in (t_a, y_a, t_b, y_b))
    lo = max(t_a[0], t_b[0]) + max_lag_s
    hi = min(t_a[-1], t_b[-1]) - max_lag_s
    if hi - lo < 4 * max_lag_s:
        raise ValueError("signals overlap too little for the requested max_lag_s")
    grid = np.arange(lo, hi, dt_s)
    a = np.interp(grid, t_a, y_a)
    a = (a - a.mean()) / (a.std() or 1.0)
    n = int(round(max_lag_s / dt_s))
    corr = np.empty(2 * n + 1)
    for k, lag in enumerate(range(-n, n + 1)):
        b = np.interp(grid + lag * dt_s, t_b, y_b)
        b = (b - b.mean()) / (b.std() or 1.0)
        corr[k] = float(np.mean(a * b))
    k = int(np.argmax(corr))
    frac = 0.0
    if 0 < k < len(corr) - 1:
        y0, y1, y2 = corr[k - 1], corr[k], corr[k + 1]
        den = y0 - 2 * y1 + y2
        if den != 0:
            frac = 0.5 * (y0 - y2) / den
    return (k - n + frac) * dt_s, float(corr[k])
