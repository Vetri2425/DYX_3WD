"""Versioned, content-hashed path artifact (architecture 7.2).

The path engine runs once per mission upload and emits ONE immutable file. Its identity is the
SHA-256 of its exact bytes; that hash is what `StartMission.path_artifact_sha256` /
`ExecuteMission` goals carry, what `dyx3_mission` verifies before loading, and what the run
manifest records. A mission is therefore reproducible: same bytes, same hash, same path.

Format ``DYX3PATH 1`` — deliberately a tiny line-oriented ASCII text (no JSON dependency in the
C++ reader; ``strtod`` round-trips Python ``repr(float)`` exactly):

    DYX3PATH 1
    frame local_ned
    engine <engine_id>
    meta <canonical single-line JSON, sorted keys, no NaN>
    points <N>
    <north_m> <east_m> <flags>          # N lines; flags: bit0 = spray ON, bit1 = must-hit
    end <N>

``flags`` is the same bitfield the prototype put in ``nav_msgs/Path`` ``position.z`` (bit-test,
never ``> 0.5``: a spray-OFF must-hit point is ``2``). Coordinates are local NED metres
(north, east). The artifact holds the *planned* polyline only; conditioning into runs happens in
``dyx3_rpp::path_conditioner`` at mission install, so geometry decisions stay in one place.

Determinism rules (enforced by tests): no timestamps, no host paths, no wall-clock timings.
"""

from __future__ import annotations

import copy
import hashlib
import json
import math
import os
import tempfile
from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field

MAGIC = "DYX3PATH"
FORMAT_VERSION = 1
FRAME = "local_ned"
FLAG_SPRAY = 1
FLAG_MUST_HIT = 2
_FLAG_MASK = FLAG_SPRAY | FLAG_MUST_HIT
EXTENSION = ".dyx3path"

# Keys removed from engine metadata because they differ per run/host (they would make the same
# mission hash differently).
_VOLATILE_KEYS = frozenset({"filepath", "planning_time_s"})


class ArtifactError(ValueError):
    """The bytes are not a valid DYX3PATH artifact (or do not match the expected hash)."""


@dataclass(frozen=True)
class PathPoint:
    north_m: float
    east_m: float
    flags: int

    @property
    def spray(self) -> bool:
        return bool(self.flags & FLAG_SPRAY)

    @property
    def must_hit(self) -> bool:
        return bool(self.flags & FLAG_MUST_HIT)


@dataclass(frozen=True)
class PathArtifact:
    sha256: str
    version: int
    engine_id: str
    meta: dict = field(default_factory=dict)
    points: tuple[PathPoint, ...] = ()


def sha256_hex(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _strip_volatile(obj):
    if isinstance(obj, dict):
        return {k: _strip_volatile(v) for k, v in obj.items() if k not in _VOLATILE_KEYS}
    if isinstance(obj, (list, tuple)):
        return [_strip_volatile(v) for v in obj]
    return obj


def _canonical_json(meta: Mapping) -> str:
    # allow_nan=False: NaN/Infinity are not valid JSON and would break any non-Python reader.
    return json.dumps(
        meta, sort_keys=True, separators=(",", ":"), ensure_ascii=True, allow_nan=False
    )


def _num(x: float) -> str:
    if not math.isfinite(x):
        raise ArtifactError(f"non-finite coordinate {x!r}")
    return repr(float(x))


def engine_id_from_origin_file(origin_sha256_path: str) -> str:
    """Identity of the carried path engine = hash of its provenance file (first 16 hex)."""
    with open(origin_sha256_path, "rb") as fh:
        return sha256_hex(fh.read())[:16]


def encode(
    points: Sequence[tuple[float, float, int]],
    *,
    engine_id: str,
    meta: Mapping | None = None,
) -> bytes:
    """Serialise ``(north_m, east_m, flags)`` points to artifact bytes. Deterministic."""
    if not engine_id or any(c.isspace() for c in engine_id):
        raise ArtifactError("engine_id must be a non-empty token without whitespace")
    if len(points) < 1:
        raise ArtifactError("a path needs at least one point")
    clean_meta = _strip_volatile(copy.deepcopy(dict(meta or {})))
    lines = [
        f"{MAGIC} {FORMAT_VERSION}",
        f"frame {FRAME}",
        f"engine {engine_id}",
        f"meta {_canonical_json(clean_meta)}",
        f"points {len(points)}",
    ]
    for n, e, flags in points:
        if int(flags) != flags or not 0 <= int(flags) <= _FLAG_MASK:
            raise ArtifactError(f"flags must be 0..{_FLAG_MASK}, got {flags!r}")
        lines.append(f"{_num(n)} {_num(e)} {int(flags)}")
    lines.append(f"end {len(points)}")
    return ("\n".join(lines) + "\n").encode("ascii")


def decode(data: bytes, *, expected_sha256: str | None = None) -> PathArtifact:
    """Strictly parse and (optionally) hash-verify artifact bytes."""
    digest = sha256_hex(data)
    if expected_sha256 is not None and digest != expected_sha256.lower():
        raise ArtifactError(f"sha256 mismatch: expected {expected_sha256}, bytes hash to {digest}")
    try:
        text = data.decode("ascii")
    except UnicodeDecodeError as exc:
        raise ArtifactError("artifact is not ASCII") from exc
    if not text.endswith("\n") or "\r" in text:
        raise ArtifactError("artifact must use LF line endings and end with a newline")
    lines = text[:-1].split("\n")
    if len(lines) < 7:
        raise ArtifactError("artifact too short")

    head = lines[0].split(" ")
    if len(head) != 2 or head[0] != MAGIC:
        raise ArtifactError("bad magic")
    if head[1] != str(FORMAT_VERSION):
        raise ArtifactError(f"unsupported format version {head[1]!r} (this reader: {FORMAT_VERSION})")
    if lines[1] != f"frame {FRAME}":
        raise ArtifactError(f"unsupported frame line {lines[1]!r}")
    if not lines[2].startswith("engine ") or len(lines[2].split(" ")) != 2:
        raise ArtifactError("bad engine line")
    engine_id = lines[2].split(" ")[1]
    if not lines[3].startswith("meta "):
        raise ArtifactError("bad meta line")
    try:
        meta = json.loads(lines[3][5:])
    except json.JSONDecodeError as exc:
        raise ArtifactError("meta is not valid JSON") from exc
    if not isinstance(meta, dict):
        raise ArtifactError("meta must be a JSON object")
    if lines[3][5:] != _canonical_json(meta):
        raise ArtifactError("meta is not in canonical form")

    pl = lines[4].split(" ")
    if len(pl) != 2 or pl[0] != "points" or not pl[1].isdigit():
        raise ArtifactError("bad points line")
    n = int(pl[1])
    if n < 1 or len(lines) != 5 + n + 1:
        raise ArtifactError(f"declared {n} points but file has {len(lines) - 6} point lines")
    if lines[-1] != f"end {n}":
        raise ArtifactError("missing or wrong end marker (truncated?)")

    pts = []
    for i, raw in enumerate(lines[5 : 5 + n]):
        parts = raw.split(" ")
        if len(parts) != 3:
            raise ArtifactError(f"point {i}: expected 3 fields")
        try:
            north, east, flags = float(parts[0]), float(parts[1]), int(parts[2])
        except ValueError as exc:
            raise ArtifactError(f"point {i}: unparseable number") from exc
        if not (math.isfinite(north) and math.isfinite(east)):
            raise ArtifactError(f"point {i}: non-finite coordinate")
        if not 0 <= flags <= _FLAG_MASK:
            raise ArtifactError(f"point {i}: flags {flags} out of range")
        if (repr(north), repr(east)) != (parts[0], parts[1]):
            raise ArtifactError(f"point {i}: coordinates not in canonical repr form")
        pts.append(PathPoint(north, east, flags))
    return PathArtifact(digest, FORMAT_VERSION, engine_id, meta, tuple(pts))


def build_meta(plan, *, source_name: str | None = None, source_bytes: bytes | None = None) -> dict:
    """Machine-independent metadata for a ``PlannedPath``.

    Includes the engine's own planning/alignment metadata minus volatile keys, plus the source
    file's name and SHA-256 so the artifact traces back to the uploaded CAD file.
    """
    meta: dict = {
        "origin_ne_m": [float(plan.origin[0]), float(plan.origin[1])],
        "total_mark_length_m": float(plan.total_mark_length),
        "total_transit_length_m": float(plan.total_transit_length),
        "num_waypoints": int(plan.num_waypoints),
        "planning": _strip_volatile(copy.deepcopy(plan.planning_metadata)),
        "alignment": _strip_volatile(copy.deepcopy(plan.alignment_metadata)),
    }
    if source_name is not None:
        meta["source"] = {"name": os.path.basename(source_name)}
        if source_bytes is not None:
            meta["source"]["sha256"] = sha256_hex(source_bytes)
    return meta


def points_from_plan(plan) -> list[tuple[float, float, int]]:
    """``PlannedPath`` -> ``(north, east, flags)``. Missing must-hit provenance => all False."""
    n = len(plan.merged_waypoints)
    if len(plan.spray_flags) != n:
        raise ArtifactError("spray_flags length differs from merged_waypoints")
    must = list(plan.must_hit) if plan.must_hit else [False] * n
    if len(must) != n:
        raise ArtifactError("must_hit length differs from merged_waypoints")
    return [
        (float(p[0]), float(p[1]), (FLAG_SPRAY if s else 0) | (FLAG_MUST_HIT if m else 0))
        for p, s, m in zip(plan.merged_waypoints, plan.spray_flags, must, strict=True)
    ]


def encode_plan(plan, *, engine_id: str, source_name: str | None = None,
                source_bytes: bytes | None = None) -> bytes:
    return encode(
        points_from_plan(plan),
        engine_id=engine_id,
        meta=build_meta(plan, source_name=source_name, source_bytes=source_bytes),
    )


def store(directory: str, data: bytes) -> tuple[str, str]:
    """Write ``<sha256>.dyx3path`` atomically; return ``(sha256, path)``. Idempotent.

    Content-addressed: an existing file with this name is, by construction, identical.
    """
    decode(data)  # never store bytes this reader would refuse
    digest = sha256_hex(data)
    os.makedirs(directory, exist_ok=True)
    final = os.path.join(directory, digest + EXTENSION)
    if os.path.exists(final):
        return digest, final
    fd, tmp = tempfile.mkstemp(dir=directory, prefix=".tmp-", suffix=EXTENSION)
    try:
        with os.fdopen(fd, "wb") as fh:
            fh.write(data)
            fh.flush()
            os.fsync(fh.fileno())
        os.replace(tmp, final)
    except BaseException:
        if os.path.exists(tmp):
            os.unlink(tmp)
        raise
    return digest, final


def load(directory: str, sha256: str) -> PathArtifact:
    """Read ``<sha256>.dyx3path`` and verify it hashes to its own name."""
    if len(sha256) != 64 or any(c not in "0123456789abcdef" for c in sha256):
        raise ArtifactError("sha256 must be 64 lowercase hex characters")
    with open(os.path.join(directory, sha256 + EXTENSION), "rb") as fh:
        return decode(fh.read(), expected_sha256=sha256)
