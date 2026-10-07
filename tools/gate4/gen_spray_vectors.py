#!/usr/bin/env python3
"""GATE 4 (spray) vector generator: run the VERBATIM PX4_DXP spray modules, write C++ test vectors.

    tools/gate4/gen_spray_vectors.py            # rewrites ros2_ws/src/dyx3_spray/test/fixtures/gate4_spray_vectors.txt
    tools/gate4/gen_spray_vectors.py --check    # fails if the committed file is stale

The Python side is always the carried, byte-identical module in tools/gate4/spray_dxp/ (sha256-pinned in
VERBATIM.sha256) — the FSM, the lease, the flow modulator, rtk_quality and the controller's own module-level and
method code (the gate stack methods are called unbound on a minimal stand-in `self`) — never a re-implementation.
Needs the same environment as gen_rpp_vectors.py (the path engine, for the archived missions): run it in the Humble
container.  Bag-derived inputs are a LOCAL ACTION (HANDOFF).

Format: whitespace separated tokens, numbers are repr(float); `-` is None; spaces inside text become `_`.
"""
from __future__ import annotations

import argparse
import hashlib
import math
import os
import random
import sys
import types
from types import SimpleNamespace

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", ".."))
SPRAY_DXP = os.path.join(HERE, "spray_dxp")
DXP = os.path.join(REPO, "ros2_ws", "src", "dyx3_rpp_legacy", "dyx3_rpp_legacy", "_dxp")
sys.path.insert(0, os.path.join(REPO, "tools", "gate3"))
sys.path.insert(0, os.path.join(REPO, "backend", "src"))

import ancestors  # noqa: E402
import gen_geometry_vectors as g3  # noqa: E402

OUT = os.path.join(REPO, "ros2_ws", "src", "dyx3_spray", "test", "fixtures", "gate4_spray_vectors.txt")
SEED = 20261009
PI = math.pi


def r(x):
    return repr(float(x))


def b(x):
    return "1" if x else "0"


def opt(x):
    return "-" if x is None else r(x)


def txt(s):
    return (s or "-").replace(" ", "_")


def verify_pins():
    want = {}
    for line in open(os.path.join(SPRAY_DXP, "VERBATIM.sha256")):
        if line.startswith("#") or not line.strip():
            continue
        h, f = line.split()
        want[f] = h
    for f, h in want.items():
        got = hashlib.sha256(open(os.path.join(SPRAY_DXP, f), "rb").read()).hexdigest()
        if got != h:
            sys.exit(f"{f} is not byte-identical to the pinned prototype source")


def load_modules():
    verify_pins()
    for p in (SPRAY_DXP, DXP):
        if p not in sys.path:
            sys.path.insert(0, p)
    ancestors._STUB_MODULES.update({"rclpy.callback_groups": ["ReentrantCallbackGroup"], "mavros_msgs.srv": ["CommandLong"]})
    try:
        import rclpy  # noqa: F401
        import mavros_msgs.msg  # noqa: F401
        import mavros_msgs.srv  # noqa: F401
        import nav_msgs.msg  # noqa: F401
    except ImportError:
        ancestors._install_stubs()
    import spray_controller_node as scn
    import spray_fsm
    import spray_flow_model
    import spray_safety_lease as lease
    import rtk_quality
    return scn, spray_fsm, spray_flow_model, lease, rtk_quality


# ---------------------------------------------------------------------------------------------------------------------
# FSM
# ---------------------------------------------------------------------------------------------------------------------
def fsm_cases(rng, fsm_mod, L):
    for k in range(120):
        fsm = fsm_mod.SpraySafetyStateMachine()
        now = 1.0 + rng.random()
        n = rng.randint(60, 220)
        L.append(f"FSM {n}")
        desired = safety = enabled = True
        outstanding = None  # (seq, deliver_at)
        for _ in range(n):
            roll = rng.random()
            now += rng.choice([0.004, 0.02, 0.02, 0.05, 0.3, 0.7, 1.3, 2.9]) * (0.5 + rng.random())
            if roll < 0.60:
                if rng.random() < 0.15:
                    desired = not desired
                if rng.random() < 0.07:
                    safety = not safety
                if rng.random() < 0.03:
                    enabled = not enabled
                cmd = fsm.tick(desired=desired, safety_ok=safety, enabled=enabled, now=now)
                ev = f"T {r(now)} {b(desired)} {b(safety)} {b(enabled)}"
            elif roll < 0.92:
                seq = fsm.cmd_seq
                pick = rng.random()
                if pick < 0.15:
                    seq = max(0, seq - rng.randint(1, 3))
                elif pick < 0.2:
                    seq = seq + 1
                ok = rng.random() < 0.7
                cmd = fsm.on_ack(seq, ok, now)
                ev = f"A {r(now)} {seq} {b(ok)}"
            else:
                fsm.note_event_reset(now)
                cmd = None
                ev = f"R {r(now)}"
            out = "N" if cmd is None else f"C {b(cmd.on)} {cmd.seq} {b(cmd.force)}"
            L.append(f"{ev} => {out} {fsm.state.value} {b(fsm.spraying)} {b(fsm.commanded)} {fsm.cmd_seq}")
        L.append("END")


# ---------------------------------------------------------------------------------------------------------------------
# Lease
# ---------------------------------------------------------------------------------------------------------------------
BACKENDS = ["mavlink_actuator", "mavlink_servo_pwm", "bogus"]


def lease_cases(rng, lease, L):
    nan, inf = float("nan"), float("inf")
    vals = [-1.0, 0.0, 1.0, -1.0000001, 1.0000001, 0.5, nan, inf, -inf]
    for _ in range(600):
        backend = rng.choice(BACKENDS)
        seq = rng.choice([0, 1, 7, 2**31, -1])
        idx = rng.choice([0, 1, 3, 6, 7, -1])
        off = rng.choice(vals) if rng.random() < 0.6 else rng.uniform(-1.2, 1.2)
        servo = rng.choice([0, 1, 8, 16, 17, -1])
        pwm = rng.choice([0, 1100, 2200, 2201, -1])
        allow = rng.random() < 0.5
        try:
            lease.validate_lease(lease.SpraySafetyLease(allow, seq, backend, idx, off, servo, pwm))
            ok = True
        except lease.LeaseValidationError:
            ok = False
        bi = BACKENDS.index(backend)
        L.append(f"V {b(allow)} {seq} {bi} {idx} {r(off)} {servo} {pwm} => {b(ok)}")
    # monitor timelines
    for _ in range(80):
        timeout = rng.choice([0.35, 0.1, 1.0, 0.2])
        mon = lease.SprayLeaseMonitor(timeout_s=timeout)
        L.append(f"MON {r(timeout)}")
        now = 5.0 + rng.random()
        for _ in range(rng.randint(8, 30)):
            now += rng.uniform(0.0, 0.3) + 1e-7 * rng.random()
            roll = rng.random()
            if roll < 0.35:
                allow = rng.random() < 0.6
                backend = rng.choice(BACKENDS[:2])
                ls = lease.SpraySafetyLease(allow, rng.randint(0, 9), backend, rng.randint(1, 6), rng.choice([-1.0, 0.0, 0.25]),
                                            rng.randint(1, 16), rng.choice([0, 1000]))
                mon.observe(lease.lease_to_json(ls), now)
                L.append(f"O {r(now)} {b(allow)} {ls.command_seq} {BACKENDS.index(backend)} {ls.actuator_set_index} {r(ls.off_value)} "
                         f"{ls.servo_instance} {ls.off_pwm_us}")
            elif roll < 0.45:
                try:
                    mon.observe('{"schema":1}', now)
                except (lease.LeaseValidationError, ValueError) as exc:
                    mon.invalidate(f"invalid controller lease: {exc}")
                L.append(f"I {r(now)}")
            else:
                reason = mon.off_reason(now)
                if reason is not None and reason.startswith("invalid controller lease"):
                    reason = "INVALID"
                L.append(f"Q {r(now)} => {txt(reason)}")
        L.append("END")


# ---------------------------------------------------------------------------------------------------------------------
# Flow
# ---------------------------------------------------------------------------------------------------------------------
def flow_cases(rng, flow_mod, L):
    for _ in range(60):
        mn = rng.choice([0.2, 0.0, -0.5, 0.5])
        on = rng.choice([1.0, 0.8, -1.0, 0.3])
        rated = rng.choice([0.35, 0.0, 1.0, 0.1])
        slew = rng.choice([2.0, 0.0, 0.5, 10.0])
        fm = flow_mod.FlowModulator(mn, on, rated, slew)
        L.append(f"FLOW {r(mn)} {r(on)} {r(rated)} {r(slew)}")
        for _ in range(rng.randint(10, 60)):
            if rng.random() < 0.1:
                fm.reset()
                L.append("R")
                continue
            sp = rng.choice([0.0, 0.05, 0.2, 0.35, 0.5, 2.0, -0.1, rng.uniform(0, 0.6)])
            dt = rng.choice([0.0, 0.02, 0.02, 0.1, 1.0, rng.uniform(0, 0.2)])
            v = fm.update(sp, dt)
            L.append(f"U {r(sp)} {r(dt)} => {r(v)} {r(fm.value)}")
        L.append("END")


# ---------------------------------------------------------------------------------------------------------------------
# RTK quality + gate stack (the controller's own methods, unbound on a stand-in self)
# ---------------------------------------------------------------------------------------------------------------------
class FakeTime:
    def __init__(self, ns):
        self.nanoseconds = ns

    def __sub__(self, other):
        return FakeTime(self.nanoseconds - other.nanoseconds)


class FakeClock:
    def __init__(self):
        self.ns = 0

    def now(self):
        return FakeTime(self.ns)


def make_fake(scn, params):
    clock = FakeClock()
    me = SimpleNamespace()
    me._clock = clock
    me.get_clock = lambda: clock
    me.get_parameter = lambda n: SimpleNamespace(value=params[n])
    me._gps_fix_type = 0
    me._gps_h_acc_m = None
    me._gps_recv_time = None
    me._gps_recover_since = None
    me._segment_state = None
    me._segment_state_recv_time = None
    me._armed = True
    me._mode = "OFFBOARD"
    me._session_mode = "continuous"
    me._path_model = object()
    me._point_meter = None
    me._last_point_update = None
    me._tracking_seen_since_path_load = True
    for name in ("_gps_quality", "_gps_gate", "_pivot_is_active", "_auto_safety_status"):
        setattr(me, name, types.MethodType(getattr(scn.SprayControllerNode, name), me))
    return me, clock


def rtk_cases(rng, scn, rq, L):
    for _ in range(2500):
        fix = rng.choice([0, 1, 2, 3, 4, 5, 6, 6, 6, 7, 8, 9])
        hacc = rng.choice([None, 0.0, 0.01, 0.05, 0.1, 0.1000001, 0.2, float("nan"), -1.0, rng.uniform(0, 0.3)])
        age = rng.choice([None, 0.0, 0.2, 0.5, 0.5000001, 2.0, -0.1, float("nan"), rng.uniform(0, 1.0)])
        tmo = rng.choice([0.5, 0.0, 1.0, -1.0])
        minfix = rng.choice([5, 6, 0, 8])
        maxh = rng.choice([0.1, 0.0, 0.03])
        req = rng.random() < 0.7
        q = rq.evaluate_rtk_quality(fix_type=fix, h_acc_m=hacc, sample_age_s=age, timeout_s=tmo, min_fix_type=minfix,
                                    max_h_acc_m=maxh, require_accuracy=req)
        L.append(f"Q {fix} {opt(hacc)} {opt(age)} {r(tmo)} {minfix} {r(maxh)} {b(req)} => {b(q.fresh)} {b(q.acceptable)} "
                 f"{txt(q.fix_name)} {txt(q.reason)}")


def gate_sequences(rng, scn, L):
    for _ in range(160):
        params = {
            "require_offboard": rng.random() < 0.8,
            "spray_require_rtk_fix": rng.random() < 0.9,
            "spray_min_fix_type": rng.choice([5, 6]),
            "spray_max_hrms_m": rng.choice([0.1, 0.03, 0.0]),
            "spray_require_accuracy": rng.random() < 0.8,
            "gps_fix_timeout_s": rng.choice([0.5, 1.0, 0.2]),
            "gps_recover_hold_s": rng.choice([1.0, 0.0, 2.5]),
            "spray_off_during_pivot": rng.random() < 0.8,
            "segment_state_timeout_s": rng.choice([1.0, 0.3]),
        }
        fake, clock = make_fake(scn, params)
        L.append("GS " + " ".join([b(params["require_offboard"]), b(params["spray_require_rtk_fix"]),
                                   str(params["spray_min_fix_type"]), r(params["spray_max_hrms_m"]),
                                   b(params["spray_require_accuracy"]), r(params["gps_fix_timeout_s"]),
                                   r(params["gps_recover_hold_s"]), b(params["spray_off_during_pivot"]),
                                   r(params["segment_state_timeout_s"])]))
        ns = 10_000_000_000 + rng.randint(0, 999)
        fix, hacc = 6, 0.02
        tracking = True
        n = rng.randint(40, 160)
        L.append(str(n))
        recv_ns = ns
        for _ in range(n):
            ns += rng.randint(3_000_017, 700_000_031)
            clock.ns = ns
            # a GPS sample arrives most steps; sometimes a drop / bad fix / silence
            if rng.random() < 0.8:
                if rng.random() < 0.12:
                    fix = rng.choice([0, 3, 4, 5, 6, 6, 6, 7])
                if rng.random() < 0.12:
                    hacc = rng.choice([None, 0.02, 0.05, 0.2])
                fake._gps_fix_type = fix
                fake._gps_h_acc_m = hacc
                recv_ns = ns
                fake._gps_recv_time = FakeTime(ns)
            pivmsg = "-"
            if rng.random() < 0.25:
                pv = rng.random() < 0.5
                fake._segment_state = 3 if pv else rng.choice([0, 1, 2, 4, 5])
                fake._segment_state_recv_time = FakeTime(ns)
                pivmsg = "1" if fake._segment_state == 3 else "0"
            fake._tracking_seen_since_path_load = tracking
            ok, reason = fake._auto_safety_status(True, 0.3, velocity_fresh=True)
            age = None if fake._gps_recv_time is None else (ns - recv_ns) * 1e-9
            L.append(f"S {r(ns / 1e9)} {opt(fix if fake._gps_recv_time is not None else 0)} {opt(hacc)} "
                     f"{opt(None if fake._gps_recv_time is None else ns / 1e9 - recv_ns / 1e9)} {pivmsg} => {b(ok)} {txt(reason)}")
        L.append("END")
    # single evaluations of the early gates (fresh state each)
    for _ in range(400):
        params = {"require_offboard": rng.random() < 0.7, "spray_require_rtk_fix": True, "spray_min_fix_type": 6,
                  "spray_max_hrms_m": 0.1, "spray_require_accuracy": True, "gps_fix_timeout_s": 0.5,
                  "gps_recover_hold_s": 0.0, "spray_off_during_pivot": True, "segment_state_timeout_s": 1.0}
        fake, clock = make_fake(scn, params)
        clock.ns = 20_000_000_000
        armed, offb, path, pose, vel = (rng.random() < 0.8 for _ in range(5))
        fake._armed = armed
        fake._mode = "OFFBOARD" if offb else "POSCTL"
        fake._path_model = object() if path else None
        fake._gps_fix_type = 6
        fake._gps_h_acc_m = 0.02
        fake._gps_recv_time = FakeTime(clock.ns)
        ok, reason = fake._auto_safety_status(pose, 0.3, velocity_fresh=vel)
        L.append(f"E {b(params['require_offboard'])} {b(armed)} {b(offb)} {b(path)} {b(pose)} {b(vel)} => {b(ok)} {txt(reason)}")


# ---------------------------------------------------------------------------------------------------------------------
# Decision (path model, projection with direction gate, lead, terminal, hysteresis)
# ---------------------------------------------------------------------------------------------------------------------
def emit_path(L, name, pts, flags):
    L.append(f"PATH {name} {len(pts)}")
    for (n, e), f in zip(pts, flags):
        L.append(f"{r(n)} {r(e)} {b(f)}")
    L.append("END")


def out_and_back(rng, gap, nleg=11, leg=5.0, mark_from=4.5):
    pts, flags = [], []
    for i in range(nleg):
        pts.append((leg * i / (nleg - 1), 0.0))
        flags.append(False)
    for i in range(nleg):
        n = leg - leg * i / (nleg - 1)
        pts.append((n, gap))
        flags.append(n <= mark_from)
    return pts, flags


def drive(rng, pts, speed_fn, noise, dt, ticks_cap, reverse=False, jump_prob=0.0):
    """Nozzle positions + headings along the polyline at ~dt cadence with lateral noise (a rover model, not truth)."""
    seq = list(reversed(pts)) if reverse else pts
    out = []
    t = 0.0
    for i in range(len(seq) - 1):
        a, c = seq[i], seq[i + 1]
        seglen = math.hypot(c[0] - a[0], c[1] - a[1])
        if seglen < 1e-9:
            continue
        hd = math.atan2(c[1] - a[1], c[0] - a[0])
        d = 0.0
        while d < seglen:
            sp = speed_fn(rng)
            step = max(sp * dt, 0.0)
            d += step if step > 1e-9 else 0.0
            f = min(d, seglen) / seglen
            n = a[0] + (c[0] - a[0]) * f
            e = a[1] + (c[1] - a[1]) * f
            lat = rng.gauss(0.0, noise)
            n += -math.sin(hd) * lat
            e += math.cos(hd) * lat
            yaw = hd + rng.gauss(0.0, 0.02)
            if jump_prob and rng.random() < jump_prob:
                n += rng.uniform(-1.5, 1.5)
                e += rng.uniform(-1.5, 1.5)
            t += dt
            out.append((t, n, e, sp, yaw))
            if len(out) >= ticks_cap:
                return out
            if step <= 1e-9:
                d += 1e-3  # standing still: still advance so the loop ends
    return out


SPEEDS = [0.0, 0.03, 0.05, 0.2, 0.35, 0.35, 0.35, 0.5]


def decision_run(rng, scn, L, name, model_pts, model_flags, traj, params, pose_noise_offsets):
    """One DEC block: header (path + params) then per-tick lines with the node's state threading."""
    model = scn._build_path_model(model_pts, model_flags)
    gate_deg = params["gate_deg"]
    L.append("DEC " + name + " " + " ".join(r(params[k]) for k in (
        "open", "close", "on_m", "off_m", "max_xt", "trip", "min_off", "term_eps", "term_spd", "wb", "wf", "reacq",
        "gate_deg")) + f" {b(params['src'])} {len(traj)}")
    prev_s = None
    tripped = False
    trip_t = None
    for (t, n, e, sp, yaw, safety, reason, gone) in traj:
        nozzle_n = nozzle_e = None
        if not gone:
            nozzle_n, nozzle_e = n, e
        dec = scn._make_spray_decision(
            model=model, nozzle_n=nozzle_n, nozzle_e=nozzle_e, speed_mps=sp, safety_ok=safety, safety_reason=reason or "",
            solenoid_open_delay_s=params["open"], solenoid_close_delay_s=params["close"],
            on_overspray_margin_m=params["on_m"], off_overspray_margin_m=params["off_m"],
            max_xtrack_error_m=params["max_xt"], max_xtrack_source=params["src"] and "mission" or "param",
            xtrack_trip_error_m=params["trip"], xtrack_tripped=tripped,
            xtrack_tripped_elapsed_s=(t - trip_t) if trip_t is not None else float("inf"),
            xtrack_gate_min_off_s=params["min_off"], mode="continuous", yaw=yaw, now_s=t,
            terminal_off_epsilon_m=params["term_eps"], terminal_off_speed_mps=params["term_spd"],
            prev_projection_s=prev_s, projection_window_back_m=params["wb"], projection_window_fwd_m=params["wf"],
            projection_reacquire_dist_m=params["reacq"], projection_direction_gate_cos=scn._direction_gate_cos(gate_deg))
        if dec.projection is not None:
            prev_s = dec.projection.s
        if dec.xtrack_tripped and not tripped:
            trip_t = t
        tripped = dec.xtrack_tripped
        p = dec.projection
        proj = "-" if p is None else f"{r(p.s)} {r(p.xtrack_error_m)} {b(p.current_flag)} {p.segment_index} {r(p.t)}"
        nb = "-" if dec.next_boundary is None else f"{dec.next_boundary.kind[0]} {r(dec.next_boundary.s)}"
        L.append(f"T {r(t)} {opt(nozzle_n)} {opt(nozzle_e)} {r(sp)} {r(yaw)} {b(safety)} {txt(reason)} => {b(dec.desired)} "
                 f"{b(dec.geometry_desired)} {b(dec.safety_ok)} {txt(dec.safety_reason)} {b(dec.xtrack_tripped)} "
                 f"{txt(dec.event)} {r(dec.distance_to_boundary_m) if math.isfinite(dec.distance_to_boundary_m) else 'inf'} | "
                 f"{proj} | {nb}")
    L.append("END")


def decision_cases(rng, scn, L, corpus):
    def base_params(**kw):
        p = dict(open=0.18, close=0.05, on_m=0.02, off_m=0.0, max_xt=0.05, trip=0.08, min_off=0.2, term_eps=0.05,
                 term_spd=0.05, wb=0.5, wf=2.0, reacq=1.0, gate_deg=0.0, src=False)
        p.update(kw)
        return p

    named = {}
    # synthetic out-and-back (the open defect) with the gate off / on
    for gap in (0.02, 0.05):
        pts, flags = out_and_back(rng, gap)
        nm = f"outback_{int(gap * 100)}cm"
        named[nm] = (pts, flags)
        emit_path(L, nm, pts, flags)
    tm = {}
    # archived missions (planned by the carried path engine). The real missions are almost entirely MARK, so mixed
    # TRANSIT/MARK variants of the same geometry (flag runs of random length) give the boundary logic real work.
    picks = ["mission_straight_5m", "square_2x2", "square_2x2__segment", "soccer_field_penalty_area__segment"]
    for nm in picks:
        if nm in corpus:
            pts, flags = corpus[nm]
            named["m_" + nm] = (pts, flags)
            emit_path(L, "m_" + nm, pts, flags)
    for nm in ["square_2x2", "soccer_field_penalty_area__segment", "mission_straight_5m__smooth_resampled"]:
        if nm not in corpus:
            continue
        pts, _ = corpus[nm]
        pts = pts[:160]
        flags, cur, left = [], rng.random() < 0.5, 0
        for _ in pts:
            if left <= 0:
                cur = not cur
                left = rng.randint(2, 18)
            flags.append(cur)
            left -= 1
        named["x_" + nm] = (pts, flags)
        emit_path(L, "x_" + nm, pts, flags)
    # a path ending on MARK (synthetic terminal boundary) and a single point
    named["ends_on_mark"] = ([(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)], [False, True, True])
    emit_path(L, "ends_on_mark", *named["ends_on_mark"])
    named["single"] = ([(1.0, 1.0)], [True])
    emit_path(L, "single", *named["single"])

    def run(nm, params, traj_kind, seed_noise=0.004):
        pts, flags = named[nm]
        sub = random.Random(rng.random())
        if traj_kind == "forward":
            tr = drive(sub, pts, lambda g: sub.choice(SPEEDS), seed_noise, 0.1, 260)
        elif traj_kind == "reverse":
            tr = drive(sub, pts, lambda g: sub.choice(SPEEDS), seed_noise, 0.1, 260, reverse=True)
        elif traj_kind == "jumpy":
            tr = drive(sub, pts, lambda g: sub.choice(SPEEDS), seed_noise, 0.1, 260, jump_prob=0.03)
        else:  # noisy: wide lateral error to exercise the hysteresis
            tr = drive(sub, pts, lambda g: sub.choice(SPEEDS), 0.05, 0.1, 260)
        traj = []
        for (t, n, e, sp, yaw) in tr:
            safety = sub.random() > 0.04
            gone = sub.random() < 0.01
            traj.append((t, n, e, sp, yaw, safety, "gate_reason" if not safety else "", gone))
        decision_run(sub, scn, L, nm, pts, flags, traj, params, None)

    for nm in ("outback_2cm", "outback_5cm"):
        for gate in (0.0, 30.0, 60.0, 90.0, 120.0):
            run(nm, base_params(gate_deg=gate), "forward")
        run(nm, base_params(gate_deg=60.0), "reverse")
        run(nm, base_params(wb=0.0, wf=0.0), "forward")  # pre-fix global scan A/B arm
    for nm in named:
        if nm.startswith(("m_", "x_")):
            run(nm, base_params(), "forward")
            run(nm, base_params(gate_deg=rng.choice([45.0, 60.0, 90.0])), rng.choice(["forward", "reverse", "jumpy"]))
            run(nm, base_params(max_xt=rng.choice([0.03, 0.05]), trip=rng.choice([0.0, 0.08]), min_off=rng.choice([0.0, 0.5]),
                                src=rng.random() < 0.3), "noisy")
    run("ends_on_mark", base_params(), "forward")
    run("ends_on_mark", base_params(term_spd=0.5, term_eps=0.2), "forward")
    run("single", base_params(), "forward")
    # parameter sweeps on the out-and-back so the lead / margins / terminal paths are all hit
    for _ in range(14):
        run("outback_5cm", base_params(open=rng.choice([0.0, 0.18, 0.5]), close=rng.choice([0.0, 0.05, 0.3]),
                                       on_m=rng.choice([0.0, 0.02, 0.1]), off_m=rng.choice([0.0, 0.02, 0.5]),
                                       gate_deg=rng.choice([0.0, 60.0]), reacq=rng.choice([0.0, 1.0, 0.1]),
                                       term_eps=rng.choice([0.0, 0.05, 0.3])), "forward")


def gate_cos_cases(scn, L):
    for deg in [-5.0, 0.0, 1e-9, 1.0, 30.0, 45.0, 60.0, 89.9999, 90.0, 90.0001, 120.0, 179.9, 180.0, 200.0]:
        L.append(f"GC {r(deg)} => {r(scn._direction_gate_cos(deg))}")


def render():
    scn, fsm_mod, flow_mod, lease, rq = load_modules()
    cls = ancestors.load_controller_class()
    corpus = {k: v for k, v in g3.corpus(cls).items()}
    rng = random.Random(SEED)
    L = ["GATE4SPRAY 1", "# module-level equivalence vectors from the VERBATIM PX4_DXP spray modules (tools/gate4/gen_spray_vectors.py)",
         f"# seed {SEED}"]
    fsm_cases(rng, fsm_mod, L)
    lease_cases(rng, lease, L)
    flow_cases(rng, flow_mod, L)
    rtk_cases(rng, scn, rq, L)
    gate_sequences(rng, scn, L)
    gate_cos_cases(scn, L)
    decision_cases(rng, scn, L, corpus)
    return "\n".join(L) + "\n"


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--check", action="store_true")
    a = ap.parse_args()
    text = render()
    if a.check:
        if not os.path.exists(OUT) or open(OUT).read() != text:
            sys.exit("gate4_spray_vectors.txt is stale")
        print("ok")
    else:
        os.makedirs(os.path.dirname(OUT), exist_ok=True)
        with open(OUT, "w") as f:
            f.write(text)
        print(f"wrote {OUT}: {text.count(chr(10))} lines, {len(text) / 1e6:.2f} MB")
