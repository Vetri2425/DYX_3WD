#!/usr/bin/env python3
"""GATE 4 (control-tick orchestrator) vector generator: drive the VERBATIM PX4_DXP controller node tick by tick.

    tools/gate4/gen_orchestrator_vectors.py            # rewrites ros2_ws/src/dyx3_rpp/test/fixtures/gate4_orchestrator_vectors.txt
    tools/gate4/gen_orchestrator_vectors.py --check

Needs the Humble container (real rclpy, mavros_msgs): see gen_conditioner_vectors.py. The node is the REAL carried
`RPPControllerNode` (its `__init__`, parameters, `_path_cb`, `_control_loop`), with only three things replaced: the clock
(an injected nanosecond counter), the publishers (capture objects) and the message classes it builds (plain Python objects,
so float32 fields keep the exact double the controller computed).

A closed-loop kinematic rover model consumes the controller's own velocity vector and produces the pose, velocity and GPS
inputs, with noise and injected faults (pose / GPS / velocity blackouts, GPS fix drops, an EKF position jump). Every input and
every output is written down, so the C++ replays the SAME inputs and compares each tick.

An episode ends at the first tick in which the carried code enters a stop/pivot machine that the C++ orchestrator does not
port (docs/contracts/rpp_orchestrator.md section 2). Those entries are recorded as HANDOFF names; the carried methods are
wrapped (they still run) only to detect the entry.

Format (whitespace tokens, one record per line):
  SCEN <name> <align_done> <n_params> <n_runs>
  PARAM <name> <value>
  RUN <segment|smooth> <length> <closed> <n>   then n lines: north east flag must cum_s
  EV POSE <ns> <north> <east> <qw> <qx> <qy> <qz>
  EV VEL <ns> <lin_x> <lin_y> <ang_z>
  EV GPS <ns> <fix_type> <h_acc_mm>
  EV TICK <ns>
  EV INIT <last_speed_cmd>      (test hook: sets the commanded-speed memory after the first accepted tick)
  EXP <vel_pub> <vn> <ve> <yaw_rate> <dbg_valid> <16 debug values> <seg_valid> <8 segment values> <seg_publishes> <HANDOFF|NONE>
  ST <snapshot values>        (not every tick)
  END
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
sys.path.insert(0, HERE)

import ancestors  # noqa: E402
import gen_conditioner_vectors as gc  # noqa: E402
import gen_geometry_vectors as g3  # noqa: E402

OUT_DIR = os.path.join(REPO, "ros2_ws", "src", "dyx3_rpp", "test", "fixtures")
OUT_FMT = os.path.join(OUT_DIR, "gate4_orchestrator_vectors_{}.txt")
PART_BYTES = 3_500_000  # each file stays well under the 5 MB hygiene limit
SEED = 20261011
PI = math.pi
r = gc.r


def wrap(a):
    return (a + PI) % (2.0 * PI) - PI


# ---------------------------------------------------------------------------------------------- fake ROS plumbing
class Cap:
    def __init__(self):
        self.msgs = []

    def publish(self, m):
        self.msgs.append(m)


class FakeClock:
    def __init__(self):
        self.ns = 0

    def now(self):
        from rclpy.time import Time

        return Time(nanoseconds=self.ns)


class Quiet:
    def __getattr__(self, _n):
        return lambda *a, **k: None


_ORIG = {}


def patch_messages(mod):
    """Plain objects instead of ROS messages: float32 fields must keep the controller's double."""
    for nm in ("MultiArrayDimension", "Float32MultiArray", "Float32", "Bool", "Vector3Stamped"):
        _ORIG.setdefault(nm, getattr(mod, nm))

    class Dim:
        def __init__(self, label="", size=0, stride=0):
            self.label, self.size, self.stride = label, size, stride

    class Layout:
        def __init__(self):
            self.dim = []

    class FArr:
        def __init__(self):
            self.layout = Layout()
            self.data = []

    class F32:
        def __init__(self):
            self.data = 0.0

    class Bl:
        def __init__(self):
            self.data = False

    class V3:
        def __init__(self):
            self.header = SimpleNamespace(stamp=None, frame_id="")
            self.vector = SimpleNamespace(x=0.0, y=0.0, z=0.0)

    mod.MultiArrayDimension = Dim
    mod.Float32MultiArray = FArr
    mod.Float32 = F32
    mod.Bool = Bl
    mod.Vector3Stamped = V3


def build_node(cls):
    import rclpy

    if not rclpy.ok():
        rclpy.init()
    mod = sys.modules[cls.__module__]
    for nm, orig in _ORIG.items():
        setattr(mod, nm, orig)
    node = cls()
    clock = FakeClock()
    node.get_clock = lambda: clock
    node.get_logger = lambda: Quiet()
    patch_messages(mod)
    for nm in ("_vel_pub", "_dbg_pub", "_segment_dbg_pub", "_conditioned_path_pub", "_yaw_rate_pub", "_spray_active_pub",
               "_progress_pub", "_milestone_pub"):
        setattr(node, nm, Cap())
    node._ho = []
    node._ho_suppress = False
    wrap_handoffs(node)
    return node, clock


def wrap_handoffs(node):
    """Record entries into the one feature the C++ orchestrator does not run (the point hold). The carried code still runs."""

    def record(name):
        if not node._ho_suppress:
            node._ho.append(name)

    orig_pt = node._point_hold_tick

    def point(*a):
        if node.get_parameter("point_hold_enabled").value:
            record("POINT_HOLD")
        return orig_pt(*a)

    node._point_hold_tick = point


# ---------------------------------------------------------------------------------------------- rover model
class Rover:
    def __init__(self, n, e, yaw):
        self.n, self.e, self.yaw, self.v, self.w = n, e, yaw, 0.0, 0.0
        self.cmd_vn = self.cmd_ve = 0.0

    def advance(self, dt):
        # A crude stand-in for the firmware loop. Along the nose: drive. 10-97 deg off the nose: spot-turn (no travel).
        # Behind the nose (a reverse brake command): decelerate without turning, as the firmware's reverse detection does.
        s = math.hypot(self.cmd_vn, self.cmd_ve)
        target = 0.0
        self.w = 0.0
        if s >= 0.01:
            bearing = math.atan2(self.cmd_ve, self.cmd_vn)
            err = wrap(bearing - self.yaw)
            if abs(err) <= 1.7:
                self.w = max(-0.8, min(0.8, 3.0 * err))
                if abs(err) < 0.17:
                    target = s * math.cos(err)
        self.yaw = wrap(self.yaw + self.w * dt)
        self.v += (target - self.v) * (1.0 - math.exp(-dt / 0.12))
        self.n += self.v * math.cos(self.yaw) * dt
        self.e += self.v * math.sin(self.yaw) * dt


def quat_for_yaw_ned(yaw_ned):
    yaw_enu = PI / 2.0 - yaw_ned
    return math.cos(yaw_enu / 2.0), 0.0, 0.0, math.sin(yaw_enu / 2.0)


# ---------------------------------------------------------------------------------------------- episode
def fmt(vals):
    return " ".join(r(v) for v in vals)


def f13(x):
    """Expected outputs: 13 significant digits (the inputs stay exact repr); the C++ test tolerance is 1e-9."""
    x = float(x)
    return repr(x) if (x != x or x in (float("inf"), float("-inf"))) else "%.13g" % x


def state_line(node):
    lp = node._last_pos
    return ("ST " + " ".join(f13(x) for x in [node._last_speed_cmd, node._last_yaw_cmd, node._path_travel_m, node._tick_dt]) + " "
            + " ".join(str(int(x)) for x in (node._segment_idx, node._run_idx, node._closest_seg_hint, node._hint_valid,
                                              node._kappa_hard_latched, node._stop_latched, node._entry_spray_hold,
                                              node._path_done, node._run_align_pending))
            + " " + " ".join(f13(x) for x in [node._ekf_reset_offset[0], node._ekf_reset_offset[1]]) + " " + str(node._ekf_reset_count)
            + " " + str(int(lp is not None)) + " " + " ".join(f13(x) for x in [lp[0] if lp else 0.0, lp[1] if lp else 0.0])
            + " " + str(int(node._segment_state.value)) + " " + str(int(node._rtk_recover_since is not None))
            + " " + " ".join(str(int(x)) for x in (node._run_boundary_stop_pending, node._completion_stop_pending,
                                                    node._segment_endpoint_stop_active, node._corner_stop_complete))
            + " " + f13(node._run_align_turn_rad))


def coerce(defaults, name, value):
    d = defaults[name]
    if isinstance(d, bool):
        return bool(value)
    if isinstance(d, int):
        return int(value)
    if isinstance(d, float):
        return float(value)
    return value


def run_episode(cls, defaults, name, pts, flags, must, params, rng, *, period_ns, max_ticks, faults=None, align_done=True,
                start_offset=0.0, start_yaw_err=0.0, prepend_single_point_run=False, rover_speed_cap=None, start_frac=0.0, start_speed=0.0):
    params = {"xy_goal_tolerance": 0.04, "rtk_recover_hold_s": 0.2, **params}
    from geometry_msgs.msg import PoseStamped, TwistStamped
    from mavros_msgs.msg import GPSRAW
    from rclpy.parameter import Parameter

    node, clock = build_node(cls)
    for k, v in params.items():
        res = node.set_parameters([Parameter(k, value=coerce(defaults, k, v))])
        assert res[0].successful, (k, v)
    msg = SimpleNamespace(
        header=SimpleNamespace(frame_id="local_ned", stamp=__import__("builtin_interfaces.msg", fromlist=["Time"]).Time()),
        poses=[SimpleNamespace(pose=SimpleNamespace(position=SimpleNamespace(x=pt[0], y=pt[1], z=float((1 if f else 0) | (2 if m else 0)))))
               for pt, f, m in zip(pts, flags, must)])
    node._path_cb(msg)
    node._drain_pending_mission()
    if not node._runs:
        return None
    if prepend_single_point_run:
        p0 = node._runs[0]["poses"][0]
        single = {"poses": node._build_poses([(p0.pose.position.x, p0.pose.position.y)], [True], p0.header.stamp, "local_ned"),
                  "flags": [True], "cum_s": [0.0], "length": 0.0, "closed": False, "profile": "segment"}
        node._runs = [single] + node._runs
        node._apply_run(0)
    if align_done:
        node._run_align_pending = False

    first = node._runs[0]["poses"]
    cum = node._runs[0]["cum_s"]
    p0 = first[0].pose.position
    p1 = first[1].pose.position if len(first) > 1 else p0
    h0 = math.atan2(p1.y - p0.y, p1.x - p0.x) if len(first) > 1 else 0.0
    sx, sy, sh = p0.x, p0.y, h0
    if start_frac > 0.0 and len(first) > 1:
        target = start_frac * node._runs[0]["length"]
        i = max(0, min(len(first) - 2, next((k for k in range(len(cum) - 1) if cum[k + 1] >= target), len(first) - 2)))
        a, b = first[i].pose.position, first[i + 1].pose.position
        seg = max(1e-9, cum[i + 1] - cum[i])
        f = max(0.0, min(1.0, (target - cum[i]) / seg))
        sx, sy = a.x + (b.x - a.x) * f, a.y + (b.y - a.y) * f
        sh = math.atan2(b.y - a.y, b.x - a.x)
    # start beside the line: offset to the left of the path (negative cross-track), heading slightly off
    rov = Rover(sx - math.sin(sh) * start_offset, sy + math.cos(sh) * start_offset, wrap(sh + start_yaw_err))
    rov.v = start_speed
    init_pending = start_frac > 0.0 and start_speed > 0.0  # a warm start: the rover is already moving (EV INIT after RTK is accepted)
    rov.cmd_vn, rov.cmd_ve = start_speed * math.cos(sh), start_speed * math.sin(sh)

    numeric = {k: v for k, v in params.items() if not isinstance(v, str)}  # tracking_profile only shapes the runs
    L = [f"SCEN {name} {int(align_done)} {len(numeric)} {len(node._runs)}"]
    for k in sorted(numeric):
        v = numeric[k]
        L.append(f"PARAM {k} {r(v) if not isinstance(v, bool) else int(v)}")
    for run in node._runs:
        ps = run["poses"]
        L.append(f"RUN {run['profile']} {r(run['length'])} {int(bool(run['closed']))} {len(ps)}")
        for ps_i, cs in zip(ps, run["cum_s"]):
            z = int(round(ps_i.pose.position.z))
            L.append(f"{r(ps_i.pose.position.x)} {r(ps_i.pose.position.y)} {z & 1} {(z >> 1) & 1} {r(cs)}")

    sub_dt = 0.004
    t_ns = 1_000_000_000_000  # an arbitrary non-zero epoch
    next_tick = t_ns + period_ns
    next_pose = t_ns + 30_000_000
    next_vel = t_ns + 12_000_000
    next_gps = t_ns + 50_000_000
    jump_off = (0.0, 0.0)
    ticks = 0
    tick_no = 0
    last_ns = t_ns
    fault = faults or {}
    stuck = 0
    done_ticks = 0

    def in_window(key, t):
        w = fault.get(key)
        return w is not None and w[0] <= (t - 1_000_000_000_000) * 1e-9 < w[1]

    while ticks < max_ticks:
        t_next = min(next_tick, next_pose, next_vel, next_gps)
        # integrate the rover to t_next
        dt_total = (t_next - last_ns) * 1e-9
        steps = max(1, int(dt_total / sub_dt))
        for _ in range(steps):
            rov.advance(dt_total / steps)
        last_ns = t_next
        clock.ns = t_next
        rel = (t_next - 1_000_000_000_000) * 1e-9
        if "jump" in fault and fault["jump"][0] <= rel and jump_off == (0.0, 0.0):
            jump_off = fault["jump"][1]
        if t_next == next_pose:
            next_pose += int(rng.uniform(40e6, 130e6)) if rng.random() < 0.95 else int(rng.uniform(150e6, 300e6))
            if in_window("pose_blackout", t_next):
                continue
            n = rov.n + jump_off[0] + rng.gauss(0, 0.004)
            e = rov.e + jump_off[1] + rng.gauss(0, 0.004)
            qw, qx, qy, qz = quat_for_yaw_ned(rov.yaw + rng.gauss(0, 0.002))
            m = PoseStamped()
            m.pose.position.x, m.pose.position.y = e, n  # ENU: x = East, y = North
            m.pose.orientation.w, m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z = qw, qx, qy, qz
            node._pose_cb(m)
            L.append(f"EV POSE {t_next} {r(n)} {r(e)} {r(qw)} {r(qx)} {r(qy)} {r(qz)}")
        elif t_next == next_vel:
            next_vel += int(rng.uniform(25e6, 60e6))
            if in_window("vel_blackout", t_next):
                continue
            m = TwistStamped()
            lx = rov.v * math.sin(rov.yaw) + rng.gauss(0, 0.003)
            ly = rov.v * math.cos(rov.yaw) + rng.gauss(0, 0.003)
            az = -rov.w + rng.gauss(0, 0.002)
            if in_window("vel_scale", t_next):
                lx, ly = lx * fault["vel_scale"][2], ly * fault["vel_scale"][2]
            m.twist.linear.x, m.twist.linear.y, m.twist.angular.z = lx, ly, az
            node._vel_cb(m)
            L.append(f"EV VEL {t_next} {r(lx)} {r(ly)} {r(az)}")
        elif t_next == next_gps:
            next_gps += 200_000_000
            if in_window("gps_blackout", t_next):
                continue
            m = GPSRAW()
            m.fix_type = 5 if in_window("gps_float", t_next) else 6
            m.h_acc = 250 if in_window("gps_poor", t_next) else (0 if in_window("gps_unknown", t_next) else rng.randint(8, 30))
            node._gps_cb(m)
            L.append(f"EV GPS {t_next} {m.fix_type} {m.h_acc}")
        else:  # tick
            next_tick += period_ns + rng.randint(-2_000_000, 2_000_000)
            for nm in ("_vel_pub", "_dbg_pub", "_segment_dbg_pub", "_yaw_rate_pub"):
                getattr(node, nm).msgs.clear()
            node._ho.clear()
            node._control_loop()
            L.append(f"EV TICK {t_next}")
            vp = node._vel_pub.msgs
            yp = node._yaw_rate_pub.msgs
            dbg = node._dbg_pub.msgs
            sg = node._segment_dbg_pub.msgs
            vn = vp[-1].vector.x if vp else 0.0
            ve = vp[-1].vector.y if vp else 0.0
            yr = yp[-1].data if yp else 0.0
            d = dbg[-1].data if dbg else None
            dv = [d[i] for i in (0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 39, 47, 48, 49)] + [d[50]] if d else []
            # 16 values: cross, hdg, look, speed, kappa, dist, age_ms, state, l_d_raw, kappa_speed, yaw_rate, spray, speed_raw, v_lat, accel, mode
            s = sg[-1].data if sg else None
            sv = [s[i] for i in range(1, 9)] if s else []
            ho = node._ho[0] if node._ho else "NONE"
            L.append("EXP " + f"{int(bool(vp))} {f13(vn)} {f13(ve)} {f13(yr)} {int(bool(d))} "
                     + (" ".join(f13(x) for x in dv) if d else " ".join(["0"] * 16)) + f" {int(bool(s))} "
                     + (" ".join(f13(x) for x in sv) if s else " ".join(["0"] * 8)) + f" {len(sg)} {ho}")
            tick_no += 1
            if init_pending and d and int(d[7]) in (1, 2):
                node._last_speed_cmd = start_speed
                L.append(f"EV INIT {r(start_speed)}")
                init_pending = False
            if ho == "NONE" and (tick_no <= 40 or tick_no % 5 == 0):
                L.append(state_line(node))
            ticks += 1
            if vp:
                cap = rover_speed_cap
                rov.cmd_vn, rov.cmd_ve = vn, ve
                if cap is not None and math.hypot(vn, ve) > cap:
                    k = cap / math.hypot(vn, ve)
                    rov.cmd_vn, rov.cmd_ve = vn * k, ve * k
            stuck = stuck + 1 if (vp and vn == 0.0 and ve == 0.0 and not fault) else 0
            done_ticks = done_ticks + 1 if node._path_done else 0
            if ho != "NONE" or done_ticks > 5 or stuck > 150:
                break
    L.append("END")
    node.destroy_node()
    return L


# ---------------------------------------------------------------------------------------------- scenario set
def line_pts(length, step=0.05, heading=0.0, n0=0.0, e0=0.0):
    k = max(1, int(length / step))
    return [(n0 + math.cos(heading) * length * i / k, e0 + math.sin(heading) * length * i / k) for i in range(k + 1)]


def arc_pts(radius, sweep, n=60, start=0.0):
    return [(radius * math.sin(start + sweep * i / n), radius * (1.0 - math.cos(start + sweep * i / n))) for i in range(n + 1)]


def with_lead(pts, lead=1.0, step=0.05):
    k = int(lead / step)
    return [(-lead * (1.0 - i / k), 0.0) for i in range(k)] + list(pts)


def with_lead_dir(pts, lead=2.0, step=0.05):
    """A straight lead-in along the first leg's heading, so the rover can build speed before the shape starts."""
    p0 = pts[0]
    j = next((i for i in range(1, len(pts)) if math.hypot(pts[i][0] - p0[0], pts[i][1] - p0[1]) > 1e-3), None)
    if j is None:
        return list(pts)
    h = math.atan2(pts[j][1] - p0[1], pts[j][0] - p0[0])
    k = int(lead / step)
    return [(p0[0] - math.cos(h) * lead * (1.0 - i / k), p0[1] - math.sin(h) * lead * (1.0 - i / k)) for i in range(k)] + list(pts)


def polyline(vs, step=0.1):
    return gc.densify(vs, step)


def build():
    cls = ancestors.load_controller_class()
    defaults = gc.declared_defaults()
    rng = random.Random(SEED)
    L = ["GATE4ORCH 1", "# control-tick equivalence vectors from the VERBATIM PX4_DXP node (tools/gate4/gen_orchestrator_vectors.py)",
         f"# seed {SEED}"]
    eps = []

    def add(name, pts, flags=None, must=None, params=None, **kw):
        eps.append((name, pts, flags if flags is not None else [False] * len(pts), must if must is not None else [False] * len(pts),
                    params or {}, kw))

    SMOOTH = {"tracking_profile": "smooth"}
    SEGMENT = {"tracking_profile": "segment"}
    # ---- smooth profile
    P50, P20 = 50_000_000, 20_000_000
    L3 = line_pts(3.0)
    ARC15 = with_lead(arc_pts(1.5, 1.2, 50), 2.5)
    ARC30 = with_lead(arc_pts(3.0, 1.5, 50), 2.5)
    circ = [(1.5 * math.sin(2 * PI * i / 90), 1.5 * (1 - math.cos(2 * PI * i / 90))) for i in range(91)]
    CIRC = with_lead(circ, 2.5)
    for i, sp in enumerate((0.3, 0.6)):
        add(f"smooth_line_{i}", L3, params={**SMOOTH, "mission_speed": sp}, period_ns=P50, max_ticks=90, start_offset=0.015 * i,
            start_yaw_err=0.012 * i)
    for i, sp in enumerate((0.3, 0.45, 0.6)):
        add(f"smooth_line_tail_{i}", L3, params={**SMOOTH, "mission_speed": sp}, period_ns=P50, max_ticks=160, start_frac=0.55,
            start_speed=sp * 0.8, start_offset=0.012 * i)
    add("smooth_arc_0", ARC15, params={**SMOOTH, "mission_speed": 0.35}, period_ns=P50, max_ticks=110, start_frac=0.2, start_speed=0.3,
        start_offset=0.015, start_yaw_err=-0.02)
    add("smooth_arc_1", ARC30, params={**SMOOTH, "mission_speed": 0.35}, period_ns=P50, max_ticks=110, start_frac=0.25, start_speed=0.3)
    add("smooth_arc_tail_0", ARC15, params={**SMOOTH, "mission_speed": 0.35}, period_ns=P50, max_ticks=150, start_frac=0.8,
        start_speed=0.3, start_offset=0.01)
    add("smooth_arc_tail_1", ARC30, params={**SMOOTH, "mission_speed": 0.5}, period_ns=P50, max_ticks=150, start_frac=0.85, start_speed=0.4)
    add("smooth_circle", CIRC, params={**SMOOTH, "mission_speed": 0.45}, period_ns=P50, max_ticks=100, start_frac=0.3, start_speed=0.4)
    add("smooth_circle_tail", CIRC, params={**SMOOTH, "mission_speed": 0.45}, period_ns=P50, max_ticks=160, start_frac=0.9, start_speed=0.4)
    add("smooth_scurve", gc.densify([(0, 0), (3, 0), (4.5, 1.2), (6, 1.2), (7.2, 0.2), (9.5, 0.2)], 0.08),
        params={**SMOOTH, "mission_speed": 0.45, "corner_smooth_radius_m": 0.8}, period_ns=P50, max_ticks=140, start_frac=0.3,
        start_speed=0.4)
    add("smooth_params_a", ARC15, params={**SMOOTH, "mission_speed": 0.5, "smooth_lateral_gain": 0.5, "smooth_lateral_max_deg": 8.0,
                                           "use_feedforward_yaw_rate": False, "a_lat_max": 0.2, "preview_curvature_n": 4,
                                           "preview_curvature_distance_m": 1.2}, period_ns=P50, max_ticks=110, start_frac=0.3,
        start_speed=0.3)
    add("smooth_params_b", ARC30, params={**SMOOTH, "mission_speed": 0.3, "smooth_max_arc_cut_m": 0.0, "smooth_curvature_ld_coeff": 0.3,
                                           "xtrack_lookahead_gain": 1.0, "kappa_hard_enter": 0.5, "kappa_hard_exit": 0.3,
                                           "speed_cmd_decel_m_s2": 0.2}, period_ns=P50, max_ticks=110, start_frac=0.3, start_speed=0.3,
        start_offset=0.03)
    add("smooth_marks", gc.densify([(0, 0), (1, 0), (3, 0), (4, 0)], 0.05), params={**SMOOTH, "mission_speed": 0.45}, period_ns=P50,
        max_ticks=170, start_frac=0.5, start_speed=0.4)

    # ---- segment profile
    L4 = line_pts(4.0, 0.5)
    BENDS = [(0, 0), (2, 0), (4, 0.6), (6, 0.6), (8, 1.4), (10, 1.4)]
    RO = [(0, 0), (0.6, 0), (1.6, 0), (1.8, 0)]  # a segment run walks its vertices in order: start at the beginning
    SQ = [(0, 0), (2.5, 0), (2.5, 2.5), (0, 2.5), (0, 0)]
    add("seg_line", L4, params={**SEGMENT, "mission_speed": 0.35}, period_ns=P50, max_ticks=90, start_offset=0.015)
    add("seg_line_tail", L4, params={**SEGMENT, "mission_speed": 0.35}, period_ns=P50, max_ticks=190, start_frac=0.6, start_speed=0.3,
        start_offset=0.01)
    add("seg_line_fast", line_pts(5.0, 1.0), params={**SEGMENT, "mission_speed": 0.7}, period_ns=P20, max_ticks=150, start_yaw_err=0.03)
    add("seg_line_fast_tail", line_pts(5.0, 1.0), params={**SEGMENT, "mission_speed": 0.7}, period_ns=P20, max_ticks=330,
        start_frac=0.8, start_speed=0.6, start_yaw_err=0.02)
    add("seg_slight_bends", BENDS, params={**SEGMENT, "mission_speed": 0.5}, period_ns=P50, max_ticks=150, start_frac=0.15,
        start_speed=0.4)
    add("seg_square_corner", SQ, params={**SEGMENT, "mission_speed": 0.45}, period_ns=P50, max_ticks=230, start_frac=0.15,
        start_speed=0.4)
    add("seg_square_full", [(0, 0), (1.6, 0), (1.6, 1.6), (0, 1.6), (0, 0)], params={**SEGMENT, "mission_speed": 0.5}, period_ns=P50,
        max_ticks=520)
    add("seg_square_params", SQ, params={**SEGMENT, "mission_speed": 0.4, "segment_stop_dwell_s": 0.0, "segment_align_settle_s": 0.0,
                                         "segment_heading_tolerance_deg": 4.0, "segment_pivot_release_max_deg": 6.0,
                                         "pivot_to_intercept_enabled": False}, period_ns=P50, max_ticks=190, start_frac=0.2,
        start_speed=0.4)
    add("seg_square_nohold_vel", SQ, params={**SEGMENT, "mission_speed": 0.45}, period_ns=P50, max_ticks=330, start_frac=0.2,
        start_speed=0.4, faults={"vel_blackout": (2.0, 9.0)})
    add("seg_marks", [(0, 0), (1, 0), (2, 0), (3, 0), (4, 0), (5, 0)], flags=[False, True, True, True, False, False],
        params={**SEGMENT, "mission_speed": 0.45, "spray_entry_max_heading_deg": 3.0, "spray_entry_release_travel_m": 0.5},
        period_ns=P50, max_ticks=120, start_yaw_err=0.03)
    add("seg_marks_cut", [(0, 0), (1.5, 0), (3, 0), (4.5, 0)], flags=[True, True, True, True],
        params={**SEGMENT, "mission_speed": 0.45, "spray_heading_cut_deg": 5.0, "spray_entry_max_heading_deg": 2.0},
        period_ns=P50, max_ticks=100, start_yaw_err=0.03, start_offset=0.015)
    add("seg_two_runs", [(0, 0), (2, 0), (4, 0), (4, 2), (4, 4)], flags=[False, True, True, False, False],
        params={**SEGMENT, "mission_speed": 0.5}, period_ns=P50, max_ticks=90)
    add("seg_two_runs_tail", [(0, 0), (2, 0), (4, 0), (4, 2), (4, 4)], flags=[False, True, True, False, False],
        params={**SEGMENT, "mission_speed": 0.5}, period_ns=P50, max_ticks=330, start_frac=0.6, start_speed=0.4)
    add("seg_single_point_first", line_pts(3.0, 0.5), params={**SEGMENT, "mission_speed": 0.45}, period_ns=P50, max_ticks=90,
        prepend_single_point_run=True)
    add("seg_latch_tail", line_pts(3.0, 0.5), params={**SEGMENT, "mission_speed": 0.35, "stop_latch_enabled": True,
                                                       "segment_precise_endpoint_stop_enabled": False}, period_ns=P50, max_ticks=190,
        start_frac=0.6, start_speed=0.3)
    add("seg_no_runrem_tail", line_pts(4.0, 0.5), params={**SEGMENT, "mission_speed": 0.5, "endpoint_approach_run_remaining": False,
                                                           "segment_precise_endpoint_stop_enabled": False}, period_ns=P50, max_ticks=190,
        start_frac=0.7, start_speed=0.4)
    add("seg_runout", RO, flags=[False, True, True, False],
        params={**SEGMENT, "mission_speed": 0.45, "segment_precise_endpoint_stop_enabled": False}, period_ns=P50, max_ticks=190)
    add("seg_runout_precise", RO, flags=[False, True, True, False], params={**SEGMENT, "mission_speed": 0.45}, period_ns=P50,
        max_ticks=190)
    add("seg_precise_params", line_pts(3.0, 1.0), params={**SEGMENT, "mission_speed": 0.45, "segment_endpoint_arrival_tolerance_m": 0.005,
                                                           "segment_endpoint_cross_tolerance_m": 0.004,
                                                           "segment_endpoint_precise_max_s": 2.0}, period_ns=P50, max_ticks=260,
        start_frac=0.6, start_speed=0.4, start_offset=0.03)
    add("seg_precise_offline", line_pts(3.0, 1.0), params={**SEGMENT, "mission_speed": 0.4, "segment_endpoint_cross_tolerance_m": 0.005,
                                                            "segment_endpoint_max_correction_m": 0.05}, period_ns=P50, max_ticks=230,
        start_frac=0.6, start_speed=0.3, start_offset=0.12)
    add("seg_runout_min_speed", [(0, 0), (0.6, 0), (1.6, 0), (1.9, 0)], flags=[False, True, True, False],
        params={**SEGMENT, "mission_speed": 0.45, "segment_precise_endpoint_stop_enabled": False, "transit_runout_min_speed_m_s": 0.12,
                "transit_runout_goal_tolerance_m": 0.1, "segment_endpoint_approach_speed": 0.05}, period_ns=P50, max_ticks=190)
    add("seg_latch_corner", [(0, 0), (2, 0), (2, 2), (0, 2)], params={**SEGMENT, "mission_speed": 0.35, "stop_latch_enabled": True,
                                                                         "segment_min_corner_speed": 0.05, "segment_slowdown_dist": 0.8},
        period_ns=P50, max_ticks=140, start_frac=0.2, start_speed=0.3)
    # the entry pivot (a run that starts on a MARK, or after a hard boundary) is part of the mission now
    add("seg_boundary_55deg", [(0, 0), (2, 0), (3.15, 1.64), (3.15, 3.6)], flags=[True, True, False, False],
        params={**SEGMENT, "mission_speed": 0.45}, period_ns=P50, max_ticks=330, start_frac=0.2, start_speed=0.4)
    add("seg_overshoot", line_pts(3.0, 1.0), params={**SEGMENT, "mission_speed": 0.9, "segment_endpoint_precise_max_s": 3.0,
                                                      "max_linear_decel": 0.3}, period_ns=P50, max_ticks=260, start_frac=0.955,
        start_speed=0.9)
    add("seg_square_loose", SQ, params={**SEGMENT, "mission_speed": 0.4, "segment_stop_dwell_s": 0.0, "segment_align_settle_s": 0.0,
                                        "segment_align_speed_threshold": 0.3, "segment_stop_yaw_rate_threshold": 1.0,
                                        "segment_stop_speed_threshold": 0.3}, period_ns=P50, max_ticks=190, start_frac=0.2,
        start_speed=0.4)
    add("seg_square_slowpivot", SQ, params={**SEGMENT, "mission_speed": 0.4, "segment_min_corner_speed": 0.03,
                                            "segment_heading_tolerance_deg": 5.0, "segment_pivot_release_max_deg": 3.0,
                                            "segment_timeout_heading_tolerance_deg": 6.0, "segment_turn_timeout_s": 1.0,
                                            "segment_pivot_timeout_max_s": 2.0}, period_ns=P50, max_ticks=230, start_frac=0.2,
        start_speed=0.4)
    add("align_big", line_pts(3.0, 0.5), flags=[True] * 7, params={**SEGMENT, "mission_speed": 0.4}, period_ns=P50, max_ticks=200,
        align_done=False, start_yaw_err=0.9)
    add("align_slow_corner_speed", line_pts(3.0, 0.5), flags=[True] * 7,
        params={**SEGMENT, "mission_speed": 0.4, "segment_min_corner_speed": 0.03}, period_ns=P50, max_ticks=120, align_done=False,
        start_yaw_err=0.7)
    add("align_small", line_pts(3.0, 0.5), flags=[True] * 7, params={**SEGMENT, "mission_speed": 0.4}, period_ns=P50, max_ticks=110,
        align_done=False, start_yaw_err=0.02)
    add("align_stale_vel", line_pts(3.0, 0.5), flags=[True] * 7, params={**SEGMENT, "mission_speed": 0.4}, period_ns=P50, max_ticks=260,
        align_done=False, start_yaw_err=0.4, faults={"vel_blackout": (0.0, 20.0)})
    add("align_smooth", line_pts(3.0), flags=[True] * 61, params={**SMOOTH, "mission_speed": 0.4}, period_ns=P50, max_ticks=150,
        align_done=False, start_yaw_err=-0.5)
    add("align_prealign_param", line_pts(3.0, 0.5), params={**SEGMENT, "mission_speed": 0.4, "entry_prealign_enabled": True},
        period_ns=P50, max_ticks=130, align_done=False, start_yaw_err=-0.3)
    add("auto_mixed", gc.densify([(0, 0), (3, 0), (3, 2)], 0.1), params={"tracking_profile": "auto", "mission_speed": 0.5},
        period_ns=P50, max_ticks=330, start_frac=0.4, start_speed=0.4)
    # ---- gates and faults (short paths: the fault window is the point; they start moving)
    base = {**SMOOTH, "mission_speed": 0.45}
    F = dict(period_ns=P50, max_ticks=110, start_speed=0.4, start_frac=0.1)
    add("fault_pose_blackout", line_pts(4.0), params=base, faults={"pose_blackout": (1.5, 2.0)}, **F)
    add("fault_pose_blackout_short", line_pts(4.0), params=base, faults={"pose_blackout": (1.5, 1.62)}, **F)
    add("fault_pose_blackout_noextrap", line_pts(4.0), params={**base, "use_imu_extrapolation": False}, faults={"pose_blackout": (1.5, 1.68)}, **F)
    add("fault_gps_blackout", line_pts(4.0), params=base, faults={"gps_blackout": (1.0, 3.0)}, **F)
    add("fault_gps_float", line_pts(4.0), params=base, faults={"gps_float": (1.0, 2.0)}, **F)
    add("fault_gps_poor", line_pts(4.0), params={**base, "rtk_recover_hold_s": 1.0}, faults={"gps_poor": (1.0, 1.6)}, **F)
    add("fault_gps_unknown", line_pts(4.0), params={**base, "rtk_require_accuracy": False}, faults={"gps_unknown": (1.0, 3.0)}, **F)
    add("fault_gps_unknown_strict", line_pts(4.0), params=base, faults={"gps_unknown": (1.0, 3.0)}, **F)
    add("fault_rtk_off", line_pts(4.0), params={**base, "require_rtk_fix": False}, faults={"gps_blackout": (0.5, 9.0)}, **F)
    add("fault_vel_blackout", line_pts(4.0), params=base, faults={"vel_blackout": (1.0, 2.0)}, **F)
    add("fault_jump_skip", line_pts(5.0), params={**base, "ekf_reset_compensation": False}, faults={"jump": (1.5, (0.25, -0.2))}, **F)
    add("fault_jump_absorb", line_pts(5.0), params={**base, "ekf_reset_compensation": True, "ekf_reset_max_absorb_m": 1.0},
        faults={"jump": (1.5, (0.25, -0.2))}, **F)
    add("fault_jump_too_big", line_pts(5.0), params={**base, "ekf_reset_compensation": True, "ekf_reset_max_absorb_m": 0.2},
        faults={"jump": (1.5, (0.5, 0.4))}, **F)
    EX = {**base, "use_imu_extrapolation": True}
    add("extrap_bias0", line_pts(4.0), params=EX, faults={"pose_blackout": (1.5, 2.05)}, **F)
    add("extrap_bias", line_pts(4.0), params={**EX, "pose_latency_bias_s": 0.15}, faults={"pose_blackout": (1.5, 2.05)}, **F)
    add("extrap_seg", line_pts(4.0, 1.0), params={**SEGMENT, "mission_speed": 0.45, "use_imu_extrapolation": True,
                                                   "pose_latency_bias_s": 0.1}, faults={"pose_blackout": (1.5, 2.03)}, **F)
    add("extrap_vel_stale", line_pts(4.0), params={**EX, "pose_latency_bias_s": 0.05}, faults={"vel_blackout": (1.0, 1.5)}, **F)
    add("extrap_too_old", line_pts(4.0), params={**EX, "pose_latency_bias_s": 0.05}, faults={"pose_blackout": (1.5, 2.4)}, **F)
    add("fault_jump_vscale", line_pts(5.0), params={**SMOOTH, "mission_speed": 0.2, "ekf_reset_compensation": True,
                                                    "ekf_reset_max_absorb_m": 1.0},
        faults={"vel_scale": (0.6, 3.5, 3.0), "jump": (1.6, (0.06, 0.065))}, **F)
    add("fault_gps_never", line_pts(4.0), params=base, faults={"gps_blackout": (0.0, 1.0)}, **F)
    add("seg_latch_fine_tail", line_pts(3.0, 0.5), params={**SEGMENT, "mission_speed": 0.35, "stop_latch_enabled": True,
                                                            "stop_latch_capture_dist_m": 0.06, "xy_goal_tolerance": 0.01,
                                                            "segment_precise_endpoint_stop_enabled": False}, period_ns=P50,
        max_ticks=170, start_frac=0.7, start_speed=0.3)
    add("smooth_preview_dist", ARC15, params={**SMOOTH, "mission_speed": 0.35, "preview_curvature_n": 2, "preview_curvature_distance_m": 1.37},
        period_ns=P50, max_ticks=110, start_frac=0.3, start_speed=0.3)
    add("seg_runout_min_speed", [(0, 0), (0.6, 0), (1.6, 0), (1.9, 0)], flags=[False, True, True, False],
        params={**SEGMENT, "mission_speed": 0.45, "segment_precise_endpoint_stop_enabled": False, "transit_runout_min_speed_m_s": 0.12,
                "transit_runout_goal_tolerance_m": 0.1, "segment_endpoint_approach_speed": 0.05}, period_ns=P50, max_ticks=140)
    add("seg_latch_corner", [(0, 0), (2, 0), (2, 2), (0, 2)], params={**SEGMENT, "mission_speed": 0.35, "stop_latch_enabled": True,
                                                                         "segment_min_corner_speed": 0.05, "segment_slowdown_dist": 0.8},
        period_ns=P50, max_ticks=110, start_frac=0.2, start_speed=0.3)
    add("fault_seg_jump_absorb", line_pts(5.0, 1.0), params={**SEGMENT, "mission_speed": 0.45, "ekf_reset_compensation": True,
                                                              "ekf_reset_max_absorb_m": 1.0}, faults={"jump": (1.5, (0.25, 0.15))}, **F)
    # ---- archived missions (first part), both profiles
    corpus = g3.corpus(cls)
    names = [n for n in sorted(corpus) if "__" not in n][:4]
    for nm in names:
        pts, flags = corpus[nm]
        pts, flags = pts[:60], flags[:60]
        if len(pts) < 3:
            continue
        for prof in ("segment", "smooth"):
            add(f"corpus_{nm}_{prof}", pts, flags=flags, params={"tracking_profile": prof, "mission_speed": 0.5, "corner_smooth_radius_m": 0.5},
                period_ns=50_000_000, max_ticks=120, start_frac=0.0, start_speed=0.0)
    # ---- randomised parameters on random shapes
    LIVE = [
        ("mission_speed", lambda: rng.choice([0.2, 0.3, 0.45, 0.6, 0.8])),
        ("lookahead_time", lambda: rng.choice([0.8, 1.0, 1.5])),
        ("min_lookahead_dist", lambda: rng.choice([0.3, 0.52, 0.8])),
        ("max_lookahead_dist", lambda: rng.choice([1.0, 1.4])),
        ("xtrack_lookahead_gain", lambda: rng.choice([0.0, 0.5, 2.0])),
        ("a_lat_max", lambda: rng.choice([0.15, 0.3, 0.6])),
        ("max_linear_accel", lambda: rng.choice([0.1, 0.2, 0.5])),
        ("max_linear_decel", lambda: rng.choice([0.3, 0.5, 1.0])),
        ("min_approach_linear_velocity", lambda: rng.choice([0.05, 0.1, 0.15])),
        ("approach_velocity_scaling_dist", lambda: rng.choice([0.4, 0.9, 1.5])),
        ("segment_slowdown_dist", lambda: rng.choice([0.0, 0.5, 1.2])),
        ("segment_min_corner_speed", lambda: rng.choice([0.08, 0.12, 0.2])),
        ("segment_endpoint_approach_speed", lambda: rng.choice([0.05, 0.1])),
        ("segment_corner_acceptance_radius", lambda: rng.choice([0.05, 0.1, 0.2])),
        ("segment_corner_threshold_deg", lambda: rng.choice([30.0, 45.0, 60.0])),
        ("stop_latch_enabled", lambda: rng.random() < 0.5),
        ("use_feedforward_yaw_rate", lambda: rng.random() < 0.6),
        ("yaw_rate_feedback_gain", lambda: rng.choice([0.5, 1.0, 1.5])),
        ("max_yaw_rate_body", lambda: rng.choice([0.3, 0.45, 0.6])),
        ("segment_yaw_rate_gain", lambda: rng.choice([0.6, 1.0, 1.4])),
        ("use_imu_extrapolation", lambda: rng.random() < 0.7),
        ("pose_latency_bias_s", lambda: rng.choice([0.0, 0.15, 0.25])),
        ("curvature_baseline_m", lambda: rng.choice([0.05, 0.1, 0.3])),
        ("endpoint_approach_run_remaining", lambda: rng.random() < 0.7),
        ("accel_gate_heading_full_deg", lambda: rng.choice([1.0, 2.0, 4.0])),
        ("accel_gate_heading_none_deg", lambda: rng.choice([5.0, 8.0])),
        ("spray_entry_max_heading_deg", lambda: rng.choice([0.0, 2.0, 5.0])),
        ("spray_heading_cut_deg", lambda: rng.choice([0.0, 20.0, 60.0])),
        ("spray_entry_release_travel_m", lambda: rng.choice([0.1, 0.5, 2.0])),
        ("transit_runout_goal_tolerance_m", lambda: rng.choice([0.0, 0.05, 0.1])),
        ("transit_runout_min_speed_m_s", lambda: rng.choice([0.0, 0.1])),
        ("segment_precise_endpoint_stop_enabled", lambda: rng.random() < 0.5),
        ("endpoint_capture_recover_enabled", lambda: rng.random() < 0.7),
        ("min_goal_travel_m", lambda: rng.choice([0.2, 0.5, 1.0])),
        ("xy_goal_tolerance", lambda: rng.choice([0.02, 0.03, 0.05])),
    ]
    for i in range(16):
        shape = with_lead_dir(gc.random_shape(rng)[:90], 2.0)
        if len(shape) < 4:
            continue
        flags = gc.random_flags(rng, len(shape))
        must = gc.random_must(rng, shape)
        params = {"tracking_profile": rng.choice(["auto", "segment", "smooth"])}
        for k, f in rng.sample(LIVE, rng.randint(4, 12)):
            params[k] = f()
        faults = {}
        if rng.random() < 0.4:
            t0 = rng.uniform(1.5, 5.0)
            key = rng.choice(["pose_blackout", "gps_blackout", "gps_float", "gps_poor", "vel_blackout"])
            faults[key] = (t0, t0 + rng.choice([0.1, 0.3, 0.8, 1.5]))
        if rng.random() < 0.15:
            faults["jump"] = (rng.uniform(2.0, 5.0), (rng.uniform(-0.4, 0.4), rng.uniform(-0.4, 0.4)))
            params["ekf_reset_compensation"] = rng.random() < 0.5
        add(f"rand{i}", shape, flags=flags, must=must, params=params, period_ns=rng.choice([20_000_000, 50_000_000, 50_000_000]), start_frac=rng.choice([0.0, 0.0, 0.3]), start_speed=0.3,
            max_ticks=100, faults=faults, start_offset=rng.uniform(-0.02, 0.02), start_yaw_err=rng.uniform(-0.03, 0.03))

    parts, cur, cur_bytes, total = [], [], 0, 0
    for name, pts, flags, must, params, kw in eps:
        ep = run_episode(cls, defaults, name, pts, flags, must, params, rng, **kw)
        if ep is None:
            print(f"skip {name}: no runs", file=sys.stderr)
            continue
        size = sum(len(x) + 1 for x in ep)
        if cur and cur_bytes + size > PART_BYTES:
            parts.append(cur)
            cur, cur_bytes = [], 0
        cur.extend(ep)
        cur_bytes += size
        total += sum(1 for x in ep if x.startswith("EXP"))
    parts.append(cur)
    out = []
    for i, part in enumerate(parts):
        out.append("\n".join(L[:3] + [f"# part {i + 1} of {len(parts)}; {total} ticks in all"] + part) + "\n")
    return out


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--check", action="store_true")
    ap.add_argument("--only", default=None, help="debug: write to /tmp/orch_dbg.txt")
    a = ap.parse_args()
    texts = build()
    paths = [OUT_FMT.format(i + 1) for i in range(len(texts))]
    if a.check:
        stale = [p for p, t in zip(paths, texts) if not os.path.exists(p) or open(p).read() != t]
        extra = [f for f in os.listdir(OUT_DIR) if f.startswith("gate4_orchestrator_vectors_") and os.path.join(OUT_DIR, f) not in paths]
        if stale or extra:
            sys.exit(f"orchestrator vectors are stale: {stale + extra}")
        print("ok")
    else:
        os.makedirs(OUT_DIR, exist_ok=True)
        for f in os.listdir(OUT_DIR):
            if f.startswith("gate4_orchestrator_vectors"):
                os.remove(os.path.join(OUT_DIR, f))
        for p, t in zip(paths, texts):
            with open(p, "w") as f:
                f.write(t)
            print(f"wrote {p}: {t.count(chr(10))} lines, {len(t) / 1e6:.2f} MB")
