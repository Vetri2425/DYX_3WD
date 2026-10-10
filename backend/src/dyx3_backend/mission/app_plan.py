"""Validate app-owned runs and encode them without planning geometry.

Contract: docs/contracts/backend.md section 1b ("App-planned missions, v2"). The backend may only apply changes that
cannot alter the drawn path, and reports every one of them:

* lossless densify: a step longer than ``MAX_STEP_M`` is split into equal collinear sub-steps (meta ``densified_steps``);
* boundary snap: a run's first point within ``BOUNDARY_SNAP_M`` of the previous run's end is moved exactly onto it
  (meta ``max_boundary_snap_m``).

The frame is explicit: points relative to a geodetic ``anchor`` (``frame: "local_ned"``), or points already in the
rover's EKF local frame (``frame: "ekf_local_ned"``, no anchor). A ``local_ned`` payload without an anchor is refused,
so points relative to an app GPS origin can never be driven as if they were EKF-local.
"""

from __future__ import annotations

import math
from itertools import pairwise

from dyx3_backend.mission import path_artifact as pa
from dyx3_backend.mission.service import MissionError

MAX_POINTS = 50_000  # submitted points
# DERIVED — NOT FROM V1 SPEC: bound on the STORED points after densify (same budget as a DXF plan's default
# ``plan_max_points``). Without it a few submitted points spread over the envelope would densify into millions.
MAX_STORED_POINTS = 200_000
MAX_STEP_M = 5.0
ENVELOPE_M = 10_000.0
# The app's own join tolerance (plan 2026-10-10 section 2). A gap up to this is snapped and reported, over it refused.
BOUNDARY_SNAP_M = 0.010
# DERIVED — NOT FROM V1 SPEC: an anchor altitude (WGS84 ellipsoid height, m) outside this is a client bug.
ANCHOR_ALT_LIMIT_M = 10_000.0

FRAME_ANCHORED = "local_ned"  # app local NE (north, east metres) relative to ``anchor``
FRAME_EKF = "ekf_local_ned"  # already in the rover's EKF local frame (bench/debug); no anchor
FRAMES = (FRAME_ANCHORED, FRAME_EKF)


def _fail(code: str, reason: str) -> None:
    raise MissionError(422, code, reason)


def _coordinate(value: object, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        _fail("NON_FINITE_COORDINATE", f"{label} must be a finite number")
    try:
        number = float(value)
    except (OverflowError, ValueError):
        _fail("NON_FINITE_COORDINATE", f"{label} must be a finite number")
    if not math.isfinite(number):
        _fail("NON_FINITE_COORDINATE", f"{label} must be a finite number")
    if abs(number) > ENVELOPE_M:
        _fail("OUT_OF_BOUNDS", f"{label} exceeds the +/-{ENVELOPE_M:g} m envelope")
    return number


def _anchor_number(anchor: dict, key: str, limit: float) -> float:
    value = anchor[key]
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        _fail("INVALID_ANCHOR", f"anchor.{key} must be a finite number")
    try:
        number = float(value)
    except (OverflowError, ValueError):
        _fail("INVALID_ANCHOR", f"anchor.{key} must be a finite number")
    if not math.isfinite(number) or abs(number) > limit:
        _fail("INVALID_ANCHOR", f"anchor.{key} must be finite and within +/-{limit:g}")
    return number


def _anchor(body: dict, frame: str) -> dict | None:
    """The geodetic anchor as stored in meta: ``{"alt": float|None, "lat": float, "lon": float}`` or None."""
    anchor = body.get("anchor")
    if anchor is None:
        if frame == FRAME_ANCHORED:
            _fail("ANCHOR_REQUIRED", 'frame local_ned needs anchor {lat, lon, alt?}; send frame "ekf_local_ned" '
                                     "only for points already in the rover's EKF local frame")
        return None
    if frame == FRAME_EKF:
        _fail("INVALID_FRAME", "frame ekf_local_ned takes no anchor (the points are already EKF-local); "
                               "use frame local_ned with an anchor")
    if not isinstance(anchor, dict) or "lat" not in anchor or "lon" not in anchor:
        _fail("INVALID_ANCHOR", "anchor must be an object {lat, lon, alt?} in WGS84 degrees")
    extra = sorted(set(anchor) - {"lat", "lon", "alt"})
    if extra:
        _fail("INVALID_ANCHOR", f"anchor has unknown keys {extra}")
    alt = anchor.get("alt")
    return {
        "lat": _anchor_number(anchor, "lat", 90.0),
        "lon": _anchor_number(anchor, "lon", 180.0),
        "alt": None if alt is None else _anchor_number(anchor, "alt", ANCHOR_ALT_LIMIT_M),
    }


def _densify(run: list[tuple[float, float, int]], spray: int, budget: int) -> tuple[list[tuple[float, float, int]], int]:
    """Split every step over MAX_STEP_M into equal collinear sub-steps. Returns (points, steps split).

    Inserted points carry the run's spray bit and never the must-hit bit (they are not app vertices).
    """
    out = [run[0]]
    split = 0
    for (n0, e0, _), point in pairwise(run):
        length = math.hypot(point[0] - n0, point[1] - e0)
        if length > MAX_STEP_M:
            parts = math.ceil(length / MAX_STEP_M)
            if len(out) + parts > budget:
                _fail("POINTS_LIMIT_EXCEEDED", f"densifying long steps exceeds {MAX_STORED_POINTS} stored points")
            dn, de = point[0] - n0, point[1] - e0
            out.extend((n0 + dn * j / parts, e0 + de * j / parts, spray) for j in range(1, parts))
            split += 1
        out.append(point)
    return out, split


def compile_plan(body: object) -> bytes:
    """Compile submitted runs to DYX3PATH 1; reject any run RPP would reconstruct differently."""
    if not isinstance(body, dict) or not all(k in body for k in ("client", "client_version", "frame", "runs")):
        raise MissionError(400, "INVALID_PAYLOAD", "client, client_version, frame and runs are required")
    frame = body["frame"]
    if frame not in FRAMES:
        _fail("INVALID_FRAME", 'frame must be "local_ned" (with an anchor) or "ekf_local_ned" (no anchor)')
    if not isinstance(body["client"], str) or not body["client"] or not isinstance(body["client_version"], str) or not body["client_version"]:
        raise MissionError(400, "INVALID_PAYLOAD", "client and client_version must be nonempty strings")
    anchor = _anchor(body, frame)
    name = body.get("name") or "app_planned_mission"
    if not isinstance(name, str) or len(name) > 128:
        raise MissionError(400, "INVALID_PAYLOAD", "name must be a string of at most 128 characters")
    origin = body.get("origin_ne_m", [0.0, 0.0])
    if not isinstance(origin, list) or len(origin) != 2:
        raise MissionError(400, "INVALID_PAYLOAD", "origin_ne_m must have two coordinates")
    origin = [_coordinate(value, f"origin_ne_m[{i}]") for i, value in enumerate(origin)]
    runs = body["runs"]
    if not isinstance(runs, list):
        raise MissionError(400, "INVALID_PAYLOAD", "runs must be an array")
    if not runs:
        _fail("EMPTY_MISSION", "mission must have at least one run")

    flat: list[tuple[float, float, int]] = []
    mark_length = transit_length = 0.0
    total = 0
    densified_steps = 0
    max_snap = 0.0
    previous_type: str | None = None
    previous_end: tuple[float, float, int] | None = None
    for ri, run in enumerate(runs):
        if not isinstance(run, dict) or "type" not in run or "points" not in run:
            raise MissionError(400, "INVALID_PAYLOAD", f"run {ri} needs type and points")
        kind = run["type"]
        if kind not in ("mark", "travel"):
            _fail("INVALID_RUN_TYPE", f"run {ri} type must be mark or travel")
        if kind == previous_type:
            _fail("adjacent_runs_same_type", f"runs {ri - 1} and {ri} have the same type")
        points = run["points"]
        if not isinstance(points, list) or len(points) < 2:
            # DERIVED — NOT FROM V1 SPEC: the rover-side v1.1 contract needs two
            # points per run for RPP's reconstructed run to carry a segment.
            _fail("RUN_TOO_SHORT", f"run {ri} needs at least two points")
        total += len(points)
        if total > MAX_POINTS:
            _fail("POINTS_LIMIT_EXCEEDED", f"mission exceeds {MAX_POINTS} points")
        spray = pa.FLAG_SPRAY if kind == "mark" else 0
        clean: list[tuple[float, float, int]] = []
        for pi, point in enumerate(points):
            if not isinstance(point, list) or len(point) != 3:
                raise MissionError(400, "INVALID_PAYLOAD", f"run {ri} point {pi} must be [north,east,flags]")
            n = _coordinate(point[0], f"run {ri} point {pi} north")
            e = _coordinate(point[1], f"run {ri} point {pi} east")
            flags = point[2]
            if isinstance(flags, bool) or not isinstance(flags, int) or not 0 <= flags <= 3:
                _fail("INVALID_FLAGS", f"run {ri} point {pi} flags must be an integer in 0..3")
            if (flags & pa.FLAG_SPRAY) != spray:
                _fail("mixed_spray_in_run", f"run {ri} point {pi} spray bit differs from run type")
            clean.append((n, e, flags))
        if previous_end is not None:
            gap = math.hypot(clean[0][0] - previous_end[0], clean[0][1] - previous_end[1])
            if gap > BOUNDARY_SNAP_M:
                _fail("runs_not_contiguous", f"run {ri} starts {gap * 1000:.1f} mm from the previous run's endpoint "
                                             f"(snap limit {BOUNDARY_SNAP_M * 1000:g} mm)")
            max_snap = max(max_snap, gap)
            # Snap: the run starts exactly at the previous end (RPP reconstructs this point from it).
            clean[0] = (previous_end[0], previous_end[1], clean[0][2])
        for (n0, e0, _), (n1, e1, _) in pairwise(clean):
            if kind == "mark":
                mark_length += math.hypot(n1 - n0, e1 - e0)
            else:
                transit_length += math.hypot(n1 - n0, e1 - e0)
        dense, split = _densify(clean, spray, MAX_STORED_POINTS - len(flat) + (1 if flat else 0))
        densified_steps += split
        if previous_end is not None:
            # R4: the shared boundary point keeps the previous run's coordinates and spray bit; must-hit
            # provenance is the OR of both submitted copies.
            n, e, flags = flat[-1]
            flat[-1] = (n, e, flags | (dense[0][2] & pa.FLAG_MUST_HIT))
            flat.extend(dense[1:])
        else:
            flat.extend(dense)
        previous_type, previous_end = kind, clean[-1]
    if len(flat) > MAX_STORED_POINTS:
        _fail("POINTS_LIMIT_EXCEEDED", f"densifying long steps exceeds {MAX_STORED_POINTS} stored points")

    meta = {
        "origin_ne_m": origin,
        "total_mark_length_m": float(mark_length),
        "total_transit_length_m": float(transit_length),
        "num_waypoints": len(flat),
        "source": {"type": "app_planned", "client": body["client"], "client_version": body["client_version"], "name": name, "num_runs": len(runs)},
        # v2 (plan 2026-10-10): frame semantics and the normalisation report. The artifact's own `frame` header line
        # stays `local_ned` (the C++ reader accepts only that); these keys say what the coordinates are relative to.
        "frame": frame,
        "anchor": anchor,
        "densified_steps": densified_steps,
        "max_boundary_snap_m": float(max_snap),
    }
    return pa.encode(flat, engine_id="app_v1", meta=meta)
