#!/usr/bin/env python3
"""GATE 4 (path conditioner) vector generator: run the VERBATIM PX4_DXP conditioning code, write C++ test vectors.

    tools/gate4/gen_conditioner_vectors.py            # rewrites ros2_ws/src/dyx3_rpp/test/fixtures/gate4_conditioner_vectors.txt
    tools/gate4/gen_conditioner_vectors.py --check

Same environment as gen_rpp_vectors.py (Humble container, the carried controller module, the archived missions). The unit cases call the
carried classmethods; the end-to-end cases call the carried `_path_cb` itself (bound to a minimal stand-in `self`) and read the runs it
installs, so the run-building glue is verbatim too.

Format (whitespace tokens): `PATH <name> <n>` + n lines `north east flag must` defines an input once; `CASE <kind> <params...> @<name>` then `=>` and the
verbatim result, `END`. (Each input is defined once: the first version repeated it per case and reached 20 MB.)
"""
from __future__ import annotations

import argparse
import ast
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

OUT = os.path.join(REPO, "ros2_ws", "src", "dyx3_rpp", "test", "fixtures", "gate4_conditioner_vectors.txt")
SEED = 20261010
PI = math.pi


def r(x):
    return repr(float(x))


def b(x):
    return "1" if x else "0"


def declared_defaults():
    path = os.path.join(ancestors.DXP_DIR, "rpp_controller_node.py")
    tree = ast.parse(open(path, encoding="utf-8").read())
    out = {}
    for n in ast.walk(tree):
        if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute) and n.func.attr == "declare_parameter" and len(n.args) >= 2:
            try:
                out[ast.literal_eval(n.args[0])] = ast.literal_eval(n.args[1])
            except Exception:  # noqa: BLE001
                pass
    return out


class Log:
    def __getattr__(self, _name):
        return lambda *a, **k: None


def mk(cls, params):
    node = object.__new__(cls)
    node.get_parameter = lambda name: SimpleNamespace(value=params[name])
    node.get_logger = lambda: Log()
    node._drain_pending_mission = lambda: None
    return node


# ------------------------------------------------------------------------------------------------ shapes
def densify(pts, step):
    out = [pts[0]]
    for a, c in zip(pts[:-1], pts[1:]):
        d = math.hypot(c[0] - a[0], c[1] - a[1])
        k = max(1, int(d / step))
        for i in range(1, k + 1):
            out.append((a[0] + (c[0] - a[0]) * i / k, a[1] + (c[1] - a[1]) * i / k))
    return out


def arc(cx, cy, rad, a0, a1, n):
    return [(cx + rad * math.cos(a0 + (a1 - a0) * i / n), cy + rad * math.sin(a0 + (a1 - a0) * i / n)) for i in range(n + 1)]


def random_shape(rng):
    kind = rng.choice(["zigzag", "square", "arc", "mixed", "connector", "outback", "noisy_line"])
    if kind == "zigzag":
        v = [(0.0, 0.0)]
        for _ in range(rng.randint(3, 8)):
            v.append((v[-1][0] + rng.uniform(-3, 3), v[-1][1] + rng.uniform(-3, 3)))
        pts = densify(v, rng.choice([0.05, 0.2, 0.5]))
    elif kind == "square":
        s = rng.uniform(1.0, 4.0)
        pts = densify([(0, 0), (s, 0), (s, s), (0, s), (0, 0)], rng.choice([0.05, 0.25]))
    elif kind == "arc":
        pts = arc(0.0, 0.0, rng.uniform(1.0, 5.0), 0.0, rng.uniform(1.0, 6.2), rng.randint(8, 80))
    elif kind == "mixed":
        pts = densify([(0, 0), (3, 0)], 0.1) + arc(3, 1.5, 1.5, -PI / 2, PI / 2, 20)[1:] + densify([(3, 3), (0, 3)], 0.1)[1:]
    elif kind == "connector":
        pts = densify([(0, 0), (3, 0), (3.06, 0.05), (3.0, 0.1), (3, 3)], 0.1)
    elif kind == "outback":
        pts = densify([(0, 0), (4, 0), (4, 0.03), (0, 0.03)], 0.1)
    else:
        pts = [(i * 0.05, rng.gauss(0, 0.004)) for i in range(rng.randint(20, 120))]
    if rng.random() < 0.3:  # exact duplicates, as planner output has
        pts = [p for q in pts for p in ((q, q) if rng.random() < 0.05 else (q,))]
    return pts


def random_flags(rng, n):
    mode = rng.choice(["off", "on", "runs", "alt"])
    if mode == "off":
        return [False] * n
    if mode == "on":
        return [True] * n
    cur, left, out = rng.random() < 0.5, 0, []
    for _ in range(n):
        if left <= 0:
            cur = not cur
            left = rng.randint(2, 30) if mode == "runs" else rng.randint(1, 3)
        out.append(cur)
        left -= 1
    return out


def random_must(rng, pts):
    p = rng.choice([0.0, 0.02, 0.1])
    return [rng.random() < p for _ in pts]


def emit_points(L, pts, flags, must):
    for (n, e), f, m in zip(pts, flags, must):
        L.append(f"{r(n)} {r(e)} {b(f)} {b(m)}")


def emit_out(L, pts, flags):
    L.append(f"=> {len(pts)}")
    for (n, e), f in zip(pts, flags):
        L.append(f"{r(n)} {r(e)} {b(f)}")


def emit_runs(L, runs):
    L.append(f"=> {len(runs)}")
    for pts, flags in runs:
        L.append(f"RUN {len(pts)}")
        for (n, e), f in zip(pts, flags):
            L.append(f"{r(n)} {r(e)} {b(f)}")


def key(p):
    return (int(round(p[0] * 1000.0)), int(round(p[1] * 1000.0)))


# ------------------------------------------------------------------------------------------------ cases
def build():
    cls = ancestors.load_controller_class()
    defaults = declared_defaults()
    rng = random.Random(SEED)
    L = ["GATE4COND 1", "# path-conditioner equivalence vectors from the VERBATIM PX4_DXP code (tools/gate4/gen_conditioner_vectors.py)", f"# seed {SEED}"]
    corpus = g3.corpus(cls)
    inputs = []
    for name, (pts, flags) in sorted(corpus.items()):
        if 2 <= len(pts) <= 130:
            inputs.append((name, pts, flags, [False] * len(pts)))
        elif len(pts) > 130:
            inputs.append((name + "_head", pts[:120], flags[:120], [False] * 120))
    # degenerate slivers (the ~3 cm reversed stub spray compensation folds back): lengths straddling the 5 cm drop threshold
    for k, ln in enumerate((0.03, 0.045, 0.049, 0.0501, 0.055)):
        sp = [(0.0, 0.0), (1.0, 0.0), (1.0, ln), (3.0, ln)]
        inputs.append((f"sliver{k}", sp, [False, False, True, False], [False] * 4))
    for i in range(70):
        pts = random_shape(rng)
        pts = pts[:110]
        inputs.append((f"rand{i}", pts, random_flags(rng, len(pts)), random_must(rng, pts)))

    node = mk(cls, defaults)
    for name, pts, flags, must in inputs:
        L.append(f"PATH {name} {len(pts)}")
        emit_points(L, pts, flags, must)
        L.append("END")
    for name, pts, flags, must in inputs:
        mh = frozenset(key(p) for p, m in zip(pts, must) if m)
        # simplify (angle only, and with a DP offset + must-hit)
        for tol, off in ((5.0, 0.0), (5.0, 0.01), (2.0, 0.03), (10.0, 0.0)):
            sp, sf = cls._simplify_path_for_profile(pts, flags, collinear_tol_deg=tol, max_offset_m=off, must_hit_keys=mh)
            L.append(f"CASE simplify {r(tol)} {r(off)} @{name}")
            emit_out(L, sp, sf)
            L.append("END")
        for th in (30.0, 45.0, 60.0):
            L.append(f"CASE classify {r(th)} @{name}")
            L.append(f"=> {cls._classify_auto_profile(pts, th)}")
            L.append("END")
        L.append(f"CASE split_flags @{name}")
        emit_runs(L, cls._split_runs_by_flag(pts, flags))
        L.append("END")
        for th, cabs, mc in ((45.0, 0.15, 20.0), (30.0, 0.30, 15.0), (45.0, 0.0, 20.0)):
            ap, af = cls._absorb_short_connectors(pts, flags, th, cabs, mc)
            L.append(f"CASE absorb {r(th)} {r(cabs)} {r(mc)} @{name}")
            emit_out(L, ap, af)
            L.append("END")
        for th in (30.0, 45.0, 90.0):
            L.append(f"CASE split_corners {r(th)} @{name}")
            emit_runs(L, cls._split_run_at_corners(pts, flags, th))
            L.append("END")
        for rad, k in ((0.5, 6), (0.2, 4), (1.5, 8)):
            sp, sf = cls._smooth_corners.__func__(node, pts, rad, k, flags) if hasattr(cls._smooth_corners, "__func__") else node._smooth_corners(pts, rad, k, flags)
            L.append(f"CASE smooth {r(rad)} {k} @{name}")
            emit_out(L, sp, sf)
            L.append("END")
        for thr, ml in ((0.5, 1.0), (0.1, 5.0)):
            L.append(f"CASE closed {r(thr)} {r(ml)} @{name}")
            p2 = dict(defaults)
            p2.update(close_loop_threshold_m=thr, close_loop_min_len_m=ml)
            L.append(f"=> {b(mk(cls, p2)._is_closed_run(pts))}")
            L.append("END")

    # merge_collinear over random run lists (split by flag, then merge)
    for name, pts, flags, must in inputs[:60]:
        runs = cls._split_runs_by_flag(pts, flags)
        for th, tm in ((45.0, 0.0), (45.0, 2.0), (30.0, 1.0)):
            merged = cls._merge_collinear_runs(runs, th, transit_merge_max_len_m=tm)
            L.append(f"CASE merge {r(th)} {r(tm)} @{name}")  # input runs = split_runs_by_flag of the path
            emit_runs(L, merged)
            L.append("END")

    # end to end: the carried _path_cb
    profiles = ["auto", "segment", "smooth", "sharp", "garbage"]
    for name, pts, flags, must in inputs:
        for prof in rng.sample(profiles, 2) if name.startswith("rand") else ["auto", "segment"]:
            p = dict(defaults)
            p.update(tracking_profile=prof, path_frame_id="local_ned")
            if rng.random() < 0.4:
                p.update(connector_absorb_m=rng.choice([0.0, 0.1, 0.3]), segment_simplify_max_offset_m=rng.choice([0.0, 0.01, 0.05]),
                         corner_smooth_radius_m=rng.choice([0.0, 0.5, 1.0]), path_resample_spacing_m=rng.choice([0.0, 0.08, 0.25]),
                         transit_merge_max_len_m=rng.choice([0.0, 2.0]))
            nd = mk(cls, p)
            msg = SimpleNamespace(
                header=SimpleNamespace(frame_id="local_ned", stamp=__import__("builtin_interfaces.msg", fromlist=["Time"]).Time()),
                poses=[SimpleNamespace(pose=SimpleNamespace(position=SimpleNamespace(x=pt[0], y=pt[1], z=float((1 if f else 0) | (2 if m else 0)))))
                       for pt, f, m in zip(pts, flags, must)])
            # _build_poses needs real PoseStamped; the container has geometry_msgs
            nd._path_cb(msg)
            runs = nd._pending_mission.runs
            L.append(f"CASE path_cb {prof} {r(p['segment_corner_threshold_deg'])} {r(p['connector_absorb_m'])} {r(p['connector_min_corner_deg'])} "
                     f"{r(p['transit_merge_max_len_m'])} {r(p['segment_simplify_max_offset_m'])} {r(p['corner_smooth_radius_m'])} "
                     f"{int(p['corner_smooth_arc_pts'])} {r(p['path_resample_spacing_m'])} {r(p['close_loop_threshold_m'])} "
                     f"{r(p['close_loop_min_len_m'])} @{name}")
            L.append(f"=> {len(runs)}")
            for run in runs:
                ps = run["poses"]
                L.append(f"RUN {run['profile']} {r(run['length'])} {b(run['closed'])} {len(ps)}")
                for ps_i, cs in zip(ps, run["cum_s"]):
                    z = int(round(ps_i.pose.position.z))
                    L.append(f"{r(ps_i.pose.position.x)} {r(ps_i.pose.position.y)} {b(z & 1)} {b(z & 2)} {r(cs)}")
            L.append("END")
    return "\n".join(L) + "\n"


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--check", action="store_true")
    a = ap.parse_args()
    text = build()
    if a.check:
        if not os.path.exists(OUT) or open(OUT).read() != text:
            sys.exit("gate4_conditioner_vectors.txt is stale")
        print("ok")
    else:
        os.makedirs(os.path.dirname(OUT), exist_ok=True)
        with open(OUT, "w") as f:
            f.write(text)
        print(f"wrote {OUT}: {text.count(chr(10))} lines, {len(text) / 1e6:.2f} MB")
