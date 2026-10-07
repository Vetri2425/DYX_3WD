"""Stage 0.1 — quantify the arc-tracking payoff from existing bags (architecture 3.1 and section 11).

Hypothesis (CLAUDE.md, arc case): velocity OFFBOARD discards `trajectory_setpoint.yawspeed`, so the yaw loop is a pure P on heading and a steady turn at
rate w leaves a steady heading error  e_psi = w / RO_YAW_P  (RO_YAW_P = 1.5). Falsifiable test from a recorded arc: regress the MEASURED heading error
on the measured turn rate; the slope through the origin should equal 1 / RO_YAW_P. If it does, a yaw-rate command (the new transport) removes the term; if
the slope is much smaller, the floor is not what limits the arc.

This module is the maths only. Extracting (turn rate, heading error) pairs from the prototype bags is a LOCAL ACTION (the topics/fields are not
reconstructed here); it takes a CSV with columns  omega_radps,heading_err_rad  and prints JSON. It does NOT convert the heading floor to centimetres:
that needs the controller's lookahead geometry, which is a modelling choice a human must make (--xtrack-per-rad is optional and has no default).
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from dataclasses import asdict, dataclass

import numpy as np


@dataclass(frozen=True)
class FloorFit:
    n: int
    slope: float              # fitted heading_err / omega through the origin (seconds)
    expected_slope: float     # 1 / RO_YAW_P
    ratio: float              # slope / expected_slope  (1.0 = the pure-P floor explains the lag completely)
    r2: float                 # coefficient of determination of the through-origin fit
    omega_abs_mean: float
    residual_rms_rad: float
    floor_rms_rad: float      # rms of expected_slope * omega: the predicted floor on these samples


def fit_floor(omega, heading_err, ro_yaw_p: float, min_abs_omega: float = 0.0) -> FloorFit:
    """Through-origin regression of heading error on turn rate. `min_abs_omega` drops (near-)straight samples; it has no default meaning, the caller decides."""
    if not (math.isfinite(ro_yaw_p) and ro_yaw_p > 0):
        raise ValueError("ro_yaw_p must be finite and > 0")
    w = np.asarray(omega, dtype=float)
    e = np.asarray(heading_err, dtype=float)
    if w.shape != e.shape:
        raise ValueError("omega and heading_err must have the same length")
    keep = np.isfinite(w) & np.isfinite(e) & (np.abs(w) >= min_abs_omega)
    w, e = w[keep], e[keep]
    if w.size < 10 or float(np.sum(w * w)) == 0.0:
        raise ValueError("need at least 10 usable samples with non-zero turn rate")
    slope = float(np.sum(w * e) / np.sum(w * w))
    res = e - slope * w
    ss_tot = float(np.sum(e * e))  # through-origin R^2 uses the uncentred total
    r2 = 1.0 - float(np.sum(res * res)) / ss_tot if ss_tot > 0 else 0.0
    exp = 1.0 / ro_yaw_p
    return FloorFit(
        n=int(w.size), slope=slope, expected_slope=exp, ratio=slope / exp, r2=r2,
        omega_abs_mean=float(np.mean(np.abs(w))), residual_rms_rad=float(np.sqrt(np.mean(res * res))),
        floor_rms_rad=float(np.sqrt(np.mean((exp * w) ** 2))),
    )


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("csv", help="columns: omega_radps,heading_err_rad (one row per sample, arcs only)")
    ap.add_argument("--ro-yaw-p", type=float, required=True, help="the FCU's live RO_YAW_P (do not assume the documented value)")
    ap.add_argument("--min-abs-omega", type=float, default=0.0)
    ap.add_argument("--xtrack-per-rad", type=float, default=None, help="optional: lateral metres per radian of heading error (a modelling choice; no default)")
    a = ap.parse_args(argv)
    with open(a.csv, newline="") as fh:
        rows = list(csv.DictReader(fh))
    fit = fit_floor([float(r["omega_radps"]) for r in rows], [float(r["heading_err_rad"]) for r in rows], a.ro_yaw_p, a.min_abs_omega)
    out = asdict(fit)
    if a.xtrack_per_rad is not None:
        out["floor_rms_cm_if_xtrack_per_rad"] = fit.floor_rms_rad * a.xtrack_per_rad * 100.0
    json.dump(out, sys.stdout, indent=2)
    print()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
