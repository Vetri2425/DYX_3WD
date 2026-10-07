#!/usr/bin/env python3
"""GATE 4 (module level) vector generator: run the VERBATIM PX4_DXP RPP ancestors, write C++ test vectors.

    tools/gate4/gen_rpp_vectors.py            # rewrites ros2_ws/src/dyx3_rpp/test/fixtures/gate4_rpp_vectors.txt
    tools/gate4/gen_rpp_vectors.py --check    # fails if the committed file is stale

Needs rclpy (rclpy.time.Time is used so the carried code runs unmodified): run it in the Humble container.
The Python side is always the carried, byte-identical controller module
(ros2_ws/src/dyx3_rpp_legacy/.../_dxp/rpp_controller_node.py, sha256-pinned), never a re-implementation.
Inputs: the archived DXF/waypoint missions in Git planned by the carried path engine and conditioned by the
verbatim ancestors, plus seeded random cases. Bag-derived inputs are a LOCAL ACTION (HANDOFF).

Format: one case per line, whitespace separated; numbers are repr(float). A path is a name `@<name>` defined by a `PATH <name> <n>` block (n `north east` lines + `END`).
Sequences are blocks `SEQ <kind> <params...> <n>` + n lines `T ... -> ...` + `END`.
"""
from __future__ import annotations

import argparse
import math
import os
import random
import sys
from types import SimpleNamespace

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, os.path.join(REPO, "tools", "gate3"))
sys.path.insert(0, os.path.join(REPO, "backend", "src"))

import ancestors  # noqa: E402
import gen_geometry_vectors as g3  # noqa: E402

OUT = os.path.join(REPO, "ros2_ws", "src", "dyx3_rpp", "test", "fixtures", "gate4_rpp_vectors.txt")
SEED = 20261008
PI = math.pi


def r(x):
    return repr(float(x))


def b(x):
    return "1" if x else "0"



class Log:
    def __init__(self):
        self.errors = 0

    def warn(self, *_a, **_k):
        pass

    def debug(self, *_a, **_k):
        pass

    def info(self, *_a, **_k):
        pass

    def error(self, *_a, **_k):
        self.errors += 1


class Clock:
    def __init__(self):
        self.ns = 0

    def now(self):
        from rclpy.time import Time

        return Time(nanoseconds=int(self.ns))


def mk(cls, params, **attrs):
    node = object.__new__(cls)
    log = Log()
    node.get_parameter = lambda name: SimpleNamespace(value=params[name])
    node.get_logger = lambda: log
    node._log = log
    for k, v in attrs.items():
        setattr(node, k, v)
    return node


def pose_list(pts):
    return [SimpleNamespace(pose=SimpleNamespace(position=SimpleNamespace(x=n, y=e, z=0.0))) for n, e in pts]


PATHS = {}


def cases(cls, rng, corpus):
    L = []
    # ---- speed profile statics -----------------------------------------------------------------
    for _ in range(1500):
        he = rng.uniform(-PI, PI) * rng.choice([1.0, 0.1, 0.01])
        cv = rng.uniform(-0.6, 0.6) * rng.choice([1.0, 0.2])
        fh, nh = rng.choice([(2.0, 5.0), (3.0, 3.0), (5.0, 2.0), (0.0, 10.0)])
        fc, nc = rng.choice([(0.05, 0.15), (0.1, 0.1), (0.2, 0.05), (0.0, 0.5)])
        L.append(f"ACC {r(he)} {r(cv)} {r(fh)} {r(nh)} {r(fc)} {r(nc)} -> {r(cls._alignment_accel_scale(he, cv, fh, nh, fc, nc))}")
    for _ in range(800):
        lat = rng.random() < 0.5
        k = rng.uniform(-0.5, 0.5)
        en, ex = rng.choice([(0.25, 0.15), (0.2, 0.2), (0.1, 0.3)])
        L.append(f"LATCH {b(lat)} {r(k)} {r(en)} {r(ex)} -> {b(cls._update_kappa_hard_latch(lat, k, en, ex))}")
    for _ in range(2500):
        raw = rng.uniform(0.0, 1.2)
        last = rng.uniform(0.0, 1.2)
        dt = rng.choice([0.02, 0.05, 0.1, 0.0, -0.01, rng.uniform(0, 0.12)])
        lat = rng.random() < 0.3
        dec = rng.choice([0.3, 0.0, 0.5])
        acc = rng.choice([0.2, 0.0, 1.0])
        sc = rng.uniform(-0.1, 1.1)
        ap = rng.random() < 0.3
        p4 = rng.choice([0.02, 0.05])
        sp, mode = cls._apply_smooth_speed_slew(
            raw, last, dt, hard_latched=lat, speed_cmd_decel=dec, max_accel=acc, accel_scale=sc,
            approach_active=ap, p4_floor=p4)
        L.append(f"SLEW {r(raw)} {r(last)} {r(dt)} {b(lat)} {r(dec)} {r(acc)} {r(sc)} {b(ap)} {r(p4)} -> {r(sp)} {mode}")

    # ---- guidance + terminal over the mission corpus -------------------------------------------
    for name, (pts, flags) in sorted(corpus.items()):
        if len(pts) < 2:
            continue
        ref = "@" + name
        PATHS[name] = pts
        node = mk(cls, {}, _path=pose_list(pts))
        n = len(pts)
        for _ in range(30):
            seg = rng.randrange(0, n - 1)
            a, bb = pts[seg], pts[seg + 1]
            t = rng.random()
            foot = (a[0] + t * (bb[0] - a[0]), a[1] + t * (bb[1] - a[1]))
            ld = rng.choice([0.05, 0.2, 0.52, 1.0, rng.uniform(0.01, 3.0), 50.0])
            ln, le, hit = node._get_lookahead_point(seg, foot[0], foot[1], ld)
            L.append(f"SLOOK {ref} {seg} {r(foot[0])} {r(foot[1])} {r(ld)} -> {r(ln)} {r(le)} {b(hit)}")
            for mj, ee, ec in [(5.0, True, False), (0.0, True, False), (5.0, False, False), (5.0, True, True),
                               (45.0, False, True), (180.0, True, True)]:
                sn, se = node._segment_lookahead_point(seg, foot[0], foot[1], ld, mj, ee, ec)
                L.append(f"SEGLOOK {ref} {seg} {r(foot[0])} {r(foot[1])} {r(ld)} {r(mj)} {b(ee)} {b(ec)} -> {r(sn)} {r(se)}")
        # pivot intercept
        for _ in range(40):
            seg = rng.randrange(0, n - 1)
            a, bb = pts[seg], pts[seg + 1]
            pos = (a[0] + rng.uniform(-0.3, 1.0) * (bb[0] - a[0]) + rng.uniform(-0.2, 0.2),
                   a[1] + rng.uniform(-0.3, 1.0) * (bb[1] - a[1]) + rng.uniform(-0.2, 0.2))
            leg = math.atan2(bb[1] - a[1], bb[0] - a[0])
            en = rng.random() < 0.8
            di = rng.choice([0.35, 0.0, 0.1, 1.0])
            pn = mk(cls, {"pivot_to_intercept_enabled": en, "pivot_intercept_dist_m": di})
            res = pn._pivot_intercept_heading(pos[0], pos[1], SimpleNamespace(x=a[0], y=a[1]), SimpleNamespace(x=bb[0], y=bb[1]), leg)
            L.append(f"PIVINT {r(pos[0])} {r(pos[1])} {r(a[0])} {r(a[1])} {r(bb[0])} {r(bb[1])} {r(leg)} {b(en)} {r(di)} -> {r(res)}")
        # run-level quantities
        for thr, ml in [(0.15, 1.0), (0.5, 0.1), (0.0, 0.0)]:
            cn = mk(cls, {"close_loop_threshold_m": thr, "close_loop_min_len_m": ml})
            L.append(f"CLOSED {ref} {r(thr)} {r(ml)} -> {b(cn._is_closed_run(pts))}")
        fl = flags[:n]
        tn = mk(cls, {}, _spray_flags=list(fl), _path=pose_list(pts))
        L.append(f"TAIL {ref} {len(fl)} {' '.join(b(f) for f in fl)} -> {r(tn._measure_tail_transit_m())}")
        # paint-then-run-out variants (the only case the tail rule is about)
        if n >= 6:
            cut = rng.randrange(3, n - 1)
            fl2 = [i < cut for i in range(n)]
            tn2 = mk(cls, {}, _spray_flags=list(fl2), _path=pose_list(pts))
            L.append(f"TAIL {ref} {len(fl2)} {' '.join(b(f) for f in fl2)} -> {r(tn2._measure_tail_transit_m())}")
            fl3 = [False] * n
            tn3 = mk(cls, {}, _spray_flags=fl3, _path=pose_list(pts))
            L.append(f"TAIL {ref} {n} {' '.join(b(f) for f in fl3)} -> {r(tn3._measure_tail_transit_m())}")
        cum = [0.0]
        for i in range(1, n):
            cum.append(cum[-1] + math.hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1]))
        length = cum[-1]
        pn_ = mk(cls, {}, _path_s=cum, _path=pose_list(pts))
        for _ in range(20):
            seg = rng.randrange(-1, n + 1)
            t = rng.uniform(-0.2, 1.2)
            L.append(f"PROGRESS {len(cum)} {' '.join(r(c) for c in cum)} {n} {seg} {r(t)} -> {r(pn_._path_progress_at(seg, t))}")
        for _ in range(20):
            travel = rng.uniform(0.0, length * 1.1)
            rn = mk(cls, {}, _runs=[{"length": length, "closed": False}], _run_idx=0, _path_s=cum,
                    _path=pose_list(pts), _path_travel_m=travel)
            res = rn._run_remaining_along()
            L.append(f"REMAIN 1 {r(length)} {len(cum)} {' '.join(r(c) for c in cum)} {n} {r(travel)} -> {0 if res is None else 1} {r(0.0 if res is None else res)}")
        for closed in (False, True):
            for mg, fr in [(0.5, 0.9), (0.0, 0.5), (3.0, 0.9)]:
                mn = mk(cls, {"min_goal_travel_m": mg, "closed_loop_min_travel_frac": fr},
                        _runs=[{"length": length, "closed": closed}], _run_idx=0)
                L.append(f"MINTRAVEL 1 {r(length)} {b(closed)} {r(mg)} {r(fr)} -> {r(mn._run_min_travel())}")
        # endpoint capture
        a, bb = pts[-2], pts[-1]
        ux, uy = bb[0] - a[0], bb[1] - a[1]
        seg = math.hypot(ux, uy)
        if seg > 1e-6:
            ux, uy = ux / seg, uy / seg
            for _ in range(40):
                along = rng.choice([-0.05, 0.0, 0.01, 0.03, 0.08, 0.3])
                perp = rng.choice([0.0, 0.02, 0.09, 0.11, 0.3, -0.05])
                pos = (bb[0] + along * ux + perp * (-uy), bb[1] + along * uy + perp * ux)
                travel = rng.choice([length, length - 0.01, length - 0.05, length - 0.5, length + 0.1])
                tail = rng.choice([0.0, 0.1, 0.9])
                en = rng.random() < 0.85
                gt = 0.02
                params = {"endpoint_capture_recover_enabled": en, "xy_goal_tolerance": gt,
                          "transit_runout_goal_tolerance_m": 0.10, "endpoint_capture_past_m": 0.02,
                          "endpoint_capture_max_miss_m": 0.10}
                cn = mk(cls, params, _runs=[{"length": length, "closed": False}], _run_idx=0, _path_s=cum,
                        _path=pose_list(pts), _path_travel_m=travel, _run_tail_transit_m=tail)
                res = cn._endpoint_capture_recovered(pos[0], pos[1], 0.0)
                verdict = 1 if res else (2 if cn._log.errors else 0)
                rem = cn._run_remaining_along()
                L.append(f"CAPTURE {ref} {r(pos[0])} {r(pos[1])} {0 if rem is None else 1} {r(0.0 if rem is None else rem)} "
                         f"{b(en)} {r(gt)} {r(tail)} {r(0.10)} {r(0.02)} {r(0.10)} -> {verdict}")
    for _ in range(300):
        gt = rng.choice([0.02, 0.01, 0.05])
        tail = rng.choice([0.0, 0.05, 0.1, 0.9, 5.0])
        ro = rng.choice([0.10, 0.0, 0.5])
        gn = mk(cls, {"transit_runout_goal_tolerance_m": ro}, _run_tail_transit_m=tail)
        L.append(f"GOALTOL {r(gt)} {r(tail)} {r(ro)} -> {r(gn._goal_tol_effective(gt))}")

    # ---- stop logic sequences (I1, I3) ----------------------------------------------------------
    for _ in range(120):
        thr = rng.choice([0.02, 0.05])
        ythr = rng.choice([0.05, 0.1])
        dwell = rng.choice([0.3, 0.0, 0.6])
        clock = Clock()
        node = mk(cls, {"segment_stop_speed_threshold": thr, "segment_stop_yaw_rate_threshold": ythr,
                        "segment_stop_dwell_s": dwell},
                  _corner_stop_entered=None, _corner_stop_settle_since=None, _latest_vel_time=None,
                  _latest_vel_ned=(0.0, 0.0), _latest_yaw_rate_ned=0.0)
        node.get_clock = lambda c=clock: c
        steps = rng.randrange(20, 140)
        rows = []
        t = rng.uniform(2, 5)
        stale_mode = rng.random() < 0.25
        for _k in range(steps):
            t += rng.choice([0.02, 0.02, 0.05, 0.1])
            clock.ns = int(round(t * 1e9))
            vel_age = rng.choice([0.5, 1.0]) if (stale_mode and rng.random() < 0.9) else rng.choice([0.0, 0.01, 0.1, 0.29, 0.31])
            speed = rng.choice([0.0, 0.0, 0.005, 0.015, 0.025, 0.1, 0.3])
            ang = rng.uniform(-PI, PI)
            node._latest_vel_ned = (speed * math.cos(ang), speed * math.sin(ang))
            node._latest_yaw_rate_ned = rng.choice([0.0, 0.01, 0.04, 0.06, 0.2, -0.03, -0.1])
            node._latest_vel_time = type(clock.now())(nanoseconds=int(round((t - vel_age) * 1e9)))
            sat = node._corner_stop_satisfied()
            rows.append(f"T {r(t)} {r(vel_age)} {r(node._latest_vel_ned[0])} {r(node._latest_vel_ned[1])} {r(node._latest_yaw_rate_ned)} -> {b(sat)}")
        L.append(f"SEQ STOP {r(thr)} {r(ythr)} {r(dwell)} {len(rows)}")
        L.extend(rows)
        L.append("END")
    for _ in range(80):
        tt, rate, margin, mx = rng.choice([(5.0, 0.4, 1.0, 9.0), (2.0, 0.5, 0.5, 4.0), (5.0, 0.0, 1.0, 9.0), (5.0, 0.4, 1.0, 0.0)])
        clock = Clock()
        node = mk(cls, {"segment_turn_timeout_s": tt, "segment_nominal_pivot_rate_rad_s": rate,
                        "segment_pivot_spinup_margin_s": margin, "segment_pivot_timeout_max_s": mx,
                        "segment_timeout_heading_tolerance_deg": 3.0},
                  _pivot_started=None, _pivot_timeout_warned=False, _pivot_turn_angle_rad=0.0)
        node.get_clock = lambda c=clock: c
        t = rng.uniform(0, 3)
        rows = []
        for _k in range(rng.randrange(5, 60)):
            t += rng.choice([0.05, 0.2, 0.5, 1.0])
            clock.ns = int(round(t * 1e9))
            ang = rng.choice([math.radians(30), math.radians(90), math.radians(179), float("nan"), 0.0])
            to = node._pivot_timed_out(ang)
            rows.append(f"T {r(t)} {r(ang)} -> {b(to)} {r(node._pivot_timeout_budget())}")
        L.append(f"SEQ PIVOT {r(tt)} {r(rate)} {r(margin)} {r(mx)} {len(rows)}")
        L.extend(rows)
        L.append("END")
    for _ in range(1500):
        clock = Clock()
        clock.ns = int(10e9)
        fresh = rng.random() < 0.8
        cap = rng.choice([0.08, 0.0, 0.2])
        thr = rng.choice([0.02, 0.05])
        yaw = rng.uniform(-PI, PI)
        speed = rng.choice([0.0, 0.01, 0.03, 0.15, 0.4])
        ang = yaw + rng.choice([0.0, PI, 0.3, -0.3, PI / 2, -PI / 2, rng.uniform(-PI, PI)])
        v = (speed * math.cos(ang), speed * math.sin(ang))
        node = mk(cls, {"segment_brake_velocity_cap_m_s": cap, "segment_stop_speed_threshold": thr},
                  _latest_vel_time=type(clock.now())(nanoseconds=int((10.0 - (0.0 if fresh else 1.0)) * 1e9)), _latest_vel_ned=v)
        node.get_clock = lambda c=clock: c
        bn, be = node._corner_brake_velocity(yaw)
        L.append(f"BRAKE {b(fresh)} {r(v[0])} {r(v[1])} {r(yaw)} {r(cap)} {r(thr)} -> {r(bn)} {r(be)}")
    return L


def render():
    cls = ancestors.load_controller_class()
    rng = random.Random(SEED)
    corpus = {k: v for k, v in g3.corpus(cls).items() if "resampled" not in k or len(v[0]) < 700}
    lines = ["GATE4 1",
             "# module-level equivalence vectors from the VERBATIM PX4_DXP ancestors (tools/gate4/gen_rpp_vectors.py)",
             f"# seed {SEED}"]
    body = cases(cls, rng, corpus)
    for name, pts in sorted(PATHS.items()):
        lines.append(f"PATH {name} {len(pts)}")
        lines += [f"{r(p[0])} {r(p[1])}" for p in pts]
        lines.append("END")
    lines += body
    return "\n".join(lines) + "\n"


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--check", action="store_true")
    a = ap.parse_args()
    text = render()
    if a.check:
        if not os.path.exists(OUT) or open(OUT).read() != text:
            sys.exit("gate4_rpp_vectors.txt is stale")
        print("ok")
    else:
        os.makedirs(os.path.dirname(OUT), exist_ok=True)
        with open(OUT, "w") as f:
            f.write(text)
        print(f"wrote {OUT}: {text.count(chr(10))} lines, {len(text) / 1e6:.2f} MB")
