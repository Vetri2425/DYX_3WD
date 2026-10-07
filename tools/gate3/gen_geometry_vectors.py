#!/usr/bin/env python3
"""GATE 3 vector generator: run the VERBATIM Python geometry ancestors, write the C++ test vectors.

    python3 tools/gate3/gen_geometry_vectors.py            # rewrites the committed fixture
    python3 tools/gate3/gen_geometry_vectors.py --check    # fails if the committed file is stale

The expected values are produced by the carried PX4_DXP controller (`tools/gate3/ancestors.py`),
NOT written by hand, over inputs that are real: the archived DXF/waypoint missions in Git
(`backend/tests/data/missions`) planned by the carried path engine, then conditioned by the
verbatim `_split_runs_by_flag` / `_simplify_path_for_profile` / `_smooth_corners` / `_resample_path`.
The tracking sequences emulate a rover driving the path with lateral noise and occasional jumps.
Bag-derived positions are an additional corpus: tools/extract_geometry_bag_fixture.py (HANDOFF).

Fixture format (text; numbers are repr(float), parsed exactly by strtod):
    GATE3 1 / # comments
    PATH <name> <n>                 n lines: north east flag        (END)
    S <fn> <args...> -> <outputs>   scalar cases
    SEQ <kind> <path> <n> ...       n STEP lines                    (END)
"""
from __future__ import annotations

import argparse
import hashlib
import math
import os
import random
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(REPO, "backend", "src"))

import ancestors  # noqa: E402

OUT = os.path.join(REPO, "ros2_ws", "src", "dyx3_geometry", "test", "fixtures", "gate3_geometry_vectors.txt")
MISSIONS = os.path.join(REPO, "backend", "tests", "data", "missions")
SEED = 20261007
WINDOW = 500  # keep the committed fixture small: first N points of the big missions
PI = math.pi


def r(x: float) -> str:
    return repr(float(x))


def sha(path: str) -> str:
    with open(path, "rb") as fh:
        return hashlib.sha256(fh.read()).hexdigest()


def corpus(cls):
    """name -> (points[(n, e)], flags[bool]) from the archived missions + verbatim conditioning."""
    from dyx3_backend.path_engine.engine import PathEngine

    out = {}
    for fname in sorted(os.listdir(MISSIONS)):
        if not fname.endswith((".dxf", ".waypoints")):
            continue
        plan = PathEngine().plan_file(os.path.join(MISSIONS, fname))
        pts = [(float(a), float(b)) for a, b in plan.merged_waypoints][:WINDOW]
        flags = [bool(f) for f in plan.spray_flags][:WINDOW]
        base = fname.rsplit(".", 1)[0]
        out[base] = (pts, flags)
        # verbatim conditioning ancestors -> more realistic controller inputs
        simp_p, simp_f = cls._simplify_path_for_profile(pts, flags, max_offset_m=0.01)
        if len(simp_p) >= 2:
            out[base + "__segment"] = (simp_p, simp_f)
        sm = object.__new__(cls)
        sm.get_logger = lambda: type("L", (), {"warn": lambda *_a, **_k: None})()
        sp, sf = sm._smooth_corners(pts, 0.5, 6, flags)
        sp, sf = sp[:600], sf[:600]
        out[base + "__smooth"] = (sp, sf)
        rp, rf = cls._resample_path(sp, 0.08, sf)
        out[base + "__smooth_resampled"] = (rp, rf)
    return out


def scalar_cases(cls, rng):
    L = []
    specials = [0.0, -0.0, PI, -PI, 2 * PI, -2 * PI, 0.5 * PI, -0.5 * PI, 1.5 * PI, 3 * PI, 1e-12, -1e-12, 100.0, -100.0]
    angles = specials + [rng.uniform(-50, 50) for _ in range(500)]
    for a in angles:
        L.append(f"S angle_wrap {r(a)} -> {r(cls._angle_wrap(a))}")
    for _ in range(500):
        h0, h1 = rng.uniform(-PI, PI), rng.uniform(-PI, PI)
        L.append(f"S heading_delta {r(h0)} {r(h1)} -> {r(cls._heading_delta(h0, h1))}")
    for s in specials:
        for t in specials:
            L.append(f"S heading_delta {r(s)} {r(t)} -> {r(cls._heading_delta(s, t))}")
    pts = [(rng.uniform(-1e3, 1e3), rng.uniform(-1e3, 1e3)) for _ in range(400)] + [
        (rng.uniform(-1, 1), rng.uniform(-1, 1)) for _ in range(150)
    ] + [(0.0, 0.0), (1e-9, 0.0), (0.9e-9, 0.0)]
    for i in range(0, len(pts) - 1):
        a, b = pts[i], pts[i + 1]
        L.append(f"S distance {r(a[0])} {r(a[1])} {r(b[0])} {r(b[1])} -> {r(cls._dist(a[0], a[1], b[0], b[1]))}")
        L.append(f"S segment_heading {r(a[0])} {r(a[1])} {r(b[0])} {r(b[1])} -> {r(cls._segment_heading(a, b))}")
    for _ in range(800):
        p = (rng.uniform(-20, 20), rng.uniform(-20, 20))
        a = (rng.uniform(-20, 20), rng.uniform(-20, 20))
        b = (rng.uniform(-20, 20), rng.uniform(-20, 20))
        L.append(f"S perp {r(p[0])} {r(p[1])} {r(a[0])} {r(a[1])} {r(b[0])} {r(b[1])} -> {r(cls._perp_dist(p, a, b))}")
    for p, a, b in [  # degenerate / near-degenerate (threshold 1e-9 avoided exactly)
        ((1.0, 2.0), (0.0, 0.0), (0.0, 0.0)),
        ((1.0, 2.0), (0.0, 0.0), (0.9e-9, 0.0)),
        ((1.0, 2.0), (0.0, 0.0), (1.1e-9, 0.0)),
        ((1.0, 2.0), (3.0, 3.0), (3.0, 3.0)),
    ]:
        L.append(f"S perp {r(p[0])} {r(p[1])} {r(a[0])} {r(a[1])} {r(b[0])} {r(b[1])} -> {r(cls._perp_dist(p, a, b))}")
    for _ in range(800):
        q = [(rng.uniform(-10, 10), rng.uniform(-10, 10)) for _ in range(4)]
        res = cls._line_intersection(*q)
        flat = " ".join(r(v) for pt in q for v in pt)
        L.append(f"S line_intersection {flat} -> " + ("0" if res is None else f"1 {r(res[0])} {r(res[1])}"))
    for q in [  # parallel, anti-parallel, collinear, near-parallel
        [(0, 0), (1, 0), (0, 1), (1, 1)],
        [(0, 0), (1, 0), (5, 1), (3, 1)],
        [(0, 0), (1, 1), (5, 5), (9, 9)],
        [(0, 0), (1, 0), (0, 1), (1, 1e-6)],
        [(0.0, 0.0), (1.0, 0.0), (0.0, 0.0), (0.0, 1.0)],
    ]:
        q = [(float(a), float(b)) for a, b in q]
        res = cls._line_intersection(*q)
        flat = " ".join(r(v) for pt in q for v in pt)
        L.append(f"S line_intersection {flat} -> " + ("0" if res is None else f"1 {r(res[0])} {r(res[1])}"))
    return L


def emit_path(L, name, pts, flags):
    L.append(f"PATH {name} {len(pts)}")
    for (n, e), f in zip(pts, flags):
        L.append(f"{r(n)} {r(e)} {1 if f else 0}")
    L.append("END")


def sequences(cls, rng, name, pts, flags, L):
    """Tracking emulation: positions along the path with lateral noise, occasional jumps."""
    if len(pts) < 2:
        return
    node = ancestors.make_node(cls, pts)
    # arc-length walk
    cum = cls._pts_cumulative_lengths(pts)
    total = cum[-1]
    if total <= 0.0:
        return
    positions, resets = [], []
    s, step = 0.0, 0
    while s <= total + 0.2 and len(positions) < 150:
        # point at arc length s (clamped), with heading-normal noise
        i = max(0, min(len(pts) - 2, next((k for k in range(len(cum) - 1) if cum[k + 1] >= s), len(pts) - 2)))
        seg = cum[i + 1] - cum[i]
        t = 0.0 if seg < 1e-12 else max(0.0, min(1.0, (s - cum[i]) / seg))
        pn = pts[i][0] + t * (pts[i + 1][0] - pts[i][0])
        pe = pts[i][1] + t * (pts[i + 1][1] - pts[i][1])
        h = cls._segment_heading(pts[i], pts[i + 1])
        lat = rng.gauss(0.0, 0.02)
        pn += -math.sin(h) * lat + rng.gauss(0.0, 0.003)
        pe += math.cos(h) * lat + rng.gauss(0.0, 0.003)
        reset = 0
        if step % 150 == 149:  # emulate the jump guard: relocate and drop the hint
            pn += rng.uniform(-1, 1)
            pe += rng.uniform(-1, 1)
            reset = 1
        positions.append((pn, pe))
        resets.append(reset)
        s += rng.uniform(0.01, 0.06)
        step += 1
    # path_proj: hint-stateful windowed search
    node._hint_valid = False
    node._closest_seg_hint = 0
    rows = []
    segs = []
    for (pn, pe), reset in zip(positions, resets):
        if reset:
            node._hint_valid = False
            node._closest_seg_hint = 0
        seg_idx, t, fn, fe, signed = node._project_onto_path(pn, pe)
        rows.append(
            f"STEP {r(pn)} {r(pe)} {reset} -> {seg_idx} {r(t)} {r(fn)} {r(fe)} {r(signed)} "
            f"{1 if node._hint_valid else 0} {node._closest_seg_hint}"
        )
        segs.append((seg_idx, fn, fe))
    L.append(f"SEQ path_proj {name} {len(rows)}")
    L.extend(rows)
    L.append("END")
    # seg_proj on the tracked segment (segment-profile style)
    rows = []
    for (pn, pe), (seg_idx, _fn, _fe) in zip(positions, segs):
        # also probe the NEXT segment to exercise t clamping at handovers
        for si in (seg_idx, min(seg_idx + 1, len(pts) - 2)):
            t, fn, fe, signed, dend = node._project_onto_segment(pn, pe, si)
            rows.append(f"STEP {r(pn)} {r(pe)} {si} -> {r(t)} {r(fn)} {r(fe)} {r(signed)} {r(dend)}")
    L.append(f"SEQ seg_proj {name} {len(rows)}")
    L.extend(rows)
    L.append("END")
    # preview curvature from the tracked feet
    rows = []
    for k, ((pn, pe), (seg_idx, fn, fe)) in enumerate(zip(positions, segs)):
        if k % 5:
            continue
        for l_d, n_prev in ((0.2, 2), (0.4, 4), (0.8, 8), (0.52, 4)):
            kmax = node._max_preview_curvature(seg_idx, fn, fe, l_d, n_prev)
            rows.append(f"STEP {seg_idx} {r(fn)} {r(fe)} {r(l_d)} {n_prev} -> {r(kmax)}")
    L.append(f"SEQ max_preview {name} {len(rows)}")
    L.extend(rows)
    L.append("END")
    # curvature_at over sampled indices and baselines
    rows = []
    idxs = sorted(set(list(range(0, len(pts), max(1, len(pts) // 120))) + [0, 1, len(pts) - 1]))
    for i in idxs:
        for base in (0.0, 0.05, 0.1, 0.15, 0.5):
            rows.append(f"STEP {i} {r(base)} -> {r(node._path_curvature_at(i, base))}")
    L.append(f"SEQ curvature_at {name} {len(rows)}")
    L.extend(rows)
    L.append("END")


def path_level(cls, name, pts, flags, L):
    L.append(f"S path_length {name} -> {r(cls._pts_length(pts))}")
    cum = cls._pts_cumulative_lengths(pts)
    L.append(f"SEQ cumulative {name} {len(cum)}")
    L.extend(f"STEP -> {r(c)}" for c in cum)
    L.append("END")
    sub = pts[:150]
    subf = flags[:150]
    for spacing in (0.03, 0.05, 0.08, 0.5, 2.0):
        for use_flags in (1, 0):
            if use_flags:
                rp, rf = cls._resample_path(sub, spacing, subf)
            else:
                rp = cls._resample_path(sub, spacing)
                rf = [False] * len(rp)
            L.append(f"SEQ resample {name} {len(rp)} {r(spacing)} {use_flags} 150")
            for (n, e), f in zip(rp, rf):
                L.append(f"STEP {r(n)} {r(e)} {1 if f else 0}")
            L.append("END")


def build() -> list[str]:
    cls = ancestors.load_controller_class()
    rng = random.Random(SEED)
    corp = corpus(cls)
    L = [
        "GATE3 1",
        "# GENERATED by tools/gate3/gen_geometry_vectors.py - do not edit by hand.",
        f"# python {sys.version.split()[0]}  seed {SEED}  window {WINDOW}",
        f"# ancestor rpp_controller_node.py sha256 {sha(os.path.join(ancestors.DXP_DIR, 'rpp_controller_node.py'))}",
    ]
    for fname in sorted(os.listdir(MISSIONS)):
        if fname.endswith((".dxf", ".waypoints")):
            L.append(f"# mission {fname} sha256 {sha(os.path.join(MISSIONS, fname))}")
    for name in sorted(corp):
        pts, flags = corp[name]
        emit_path(L, name, pts, flags)
    L.extend(scalar_cases(cls, rng))
    for name in sorted(corp):
        pts, flags = corp[name]
        path_level(cls, name, pts, flags, L)
        sequences(cls, rng, name, pts, flags, L)
    return L


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--check", action="store_true")
    ns = ap.parse_args()
    text = "\n".join(build()) + "\n"
    if ns.check:
        with open(OUT, encoding="ascii") as fh:
            if fh.read() != text:
                print("fixture is stale: re-run tools/gate3/gen_geometry_vectors.py", file=sys.stderr)
                return 1
        print("fixture is up to date")
        return 0
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    with open(OUT, "w", encoding="ascii", newline="\n") as fh:
        fh.write(text)
    print(f"wrote {OUT} ({len(text)} bytes)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
