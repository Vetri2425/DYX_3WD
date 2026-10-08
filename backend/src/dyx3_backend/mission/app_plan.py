"""Validate app-owned runs and encode them without planning geometry."""

from __future__ import annotations

import math

from dyx3_backend.mission import path_artifact as pa
from dyx3_backend.mission.service import MissionError

MAX_POINTS = 50_000
MAX_STEP_M = 5.0
ENVELOPE_M = 10_000.0
BOUNDARY_TOL_M = 0.001


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


def compile_plan(body: object) -> bytes:
    """Compile submitted runs to DYX3PATH 1; reject any run RPP would reconstruct differently."""
    if not isinstance(body, dict) or not all(k in body for k in ("client", "client_version", "frame", "runs")):
        raise MissionError(400, "INVALID_PAYLOAD", "client, client_version, frame and runs are required")
    if body["frame"] != "local_ned":
        _fail("INVALID_FRAME", "frame must be local_ned")
    if not isinstance(body["client"], str) or not body["client"] or not isinstance(body["client_version"], str) or not body["client_version"]:
        raise MissionError(400, "INVALID_PAYLOAD", "client and client_version must be nonempty strings")
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
        clean: list[tuple[float, float, int]] = []
        for pi, point in enumerate(points):
            if not isinstance(point, list) or len(point) != 3:
                raise MissionError(400, "INVALID_PAYLOAD", f"run {ri} point {pi} must be [north,east,flags]")
            n = _coordinate(point[0], f"run {ri} point {pi} north")
            e = _coordinate(point[1], f"run {ri} point {pi} east")
            flags = point[2]
            if isinstance(flags, bool) or not isinstance(flags, int) or not 0 <= flags <= 3:
                _fail("INVALID_FLAGS", f"run {ri} point {pi} flags must be an integer in 0..3")
            if bool(flags & pa.FLAG_SPRAY) != (kind == "mark"):
                _fail("mixed_spray_in_run", f"run {ri} point {pi} spray bit differs from run type")
            if clean:
                length = math.hypot(n - clean[-1][0], e - clean[-1][1])
                if length > MAX_STEP_M:
                    _fail("STEP_TOO_LARGE", f"run {ri} step {pi - 1}->{pi} is {length:.2f}m, over {MAX_STEP_M:.2f}m")
                if kind == "mark":
                    mark_length += length
                else:
                    transit_length += length
            clean.append((n, e, flags))
        if previous_end is not None:
            if abs(clean[0][0] - previous_end[0]) > BOUNDARY_TOL_M or abs(clean[0][1] - previous_end[1]) > BOUNDARY_TOL_M:
                _fail("runs_not_contiguous", f"run {ri} does not start at the previous run's endpoint")
            # RPP reconstructs this point from the prior run's endpoint. OR must-hit
            # provenance across both copies; the spray bit remains the previous run's.
            n, e, flags = flat[-1]
            flat[-1] = (n, e, flags | (clean[0][2] & pa.FLAG_MUST_HIT))
            flat.extend(clean[1:])
        else:
            flat.extend(clean)
        previous_type, previous_end = kind, clean[-1]

    meta = {
        "origin_ne_m": origin,
        "total_mark_length_m": float(mark_length),
        "total_transit_length_m": float(transit_length),
        "num_waypoints": len(flat),
        "source": {"type": "app_planned", "client": body["client"], "client_version": body["client_version"], "name": name, "num_runs": len(runs)},
    }
    return pa.encode(flat, engine_id="app_v1", meta=meta)
