"""Mission ingest and lookup. Contract: docs/contracts/backend.md section 1.

CAD/CRS geometry stays Python and in the backend (architecture 7.2). An upload becomes a content-addressed ``DYX3PATH 1`` artifact
(docs/contracts/path_artifact.md); the id is the sha256 of its bytes, so storing the same mission twice is a no-op.
"""

from __future__ import annotations

import math
import os
from dataclasses import dataclass

from dyx3_backend.config.settings import Settings
from dyx3_backend.mission import path_artifact as pa

ORIGIN_SHA = os.path.join(os.path.dirname(os.path.dirname(__file__)), "path_engine", "ORIGIN.sha256")


class MissionError(Exception):
    def __init__(self, status: int, code: str, reason: str) -> None:
        super().__init__(reason)
        self.status = status
        self.code = code
        self.reason = reason


@dataclass(frozen=True)
class PlanParams:
    origin_n: float = 0.0
    origin_e: float = 0.0
    rotation_deg: float = 0.0
    unit_scale: float | None = None
    close_loop: bool = False
    anchor: str = "drawing_origin"

    def validate(self) -> None:
        for name in ("origin_n", "origin_e", "rotation_deg"):
            if not math.isfinite(getattr(self, name)):
                raise MissionError(422, "invalid_parameter", f"{name} must be finite")
        if abs(self.origin_n) > 1e6 or abs(self.origin_e) > 1e6:
            raise MissionError(422, "invalid_parameter", "origin out of range")
        if abs(self.rotation_deg) > 360.0:
            raise MissionError(422, "invalid_parameter", "rotation_deg must be within +-360")
        if self.unit_scale is not None and not (math.isfinite(self.unit_scale) and 0.0 < self.unit_scale <= 1e6):
            raise MissionError(422, "invalid_parameter", "unit_scale must be a positive finite number")
        if self.anchor not in ("drawing_origin", "first_waypoint"):
            raise MissionError(422, "invalid_parameter", "anchor must be drawing_origin or first_waypoint")


def summarize(art: pa.PathArtifact) -> dict:
    pts = art.points
    marks = sum(1 for p in pts if p.spray)
    n = [p.north_m for p in pts]
    e = [p.east_m for p in pts]
    meta = art.meta or {}
    return {
        "sha256": art.sha256,
        "engine_id": art.engine_id,
        "num_points": len(pts),
        "num_spray_points": marks,
        "mark_length_m": meta.get("total_mark_length_m"),
        "transit_length_m": meta.get("total_transit_length_m"),
        "bbox_ne_m": [min(n), min(e), max(n), max(e)] if pts else None,
        "source": meta.get("source"),
    }


def normalisation(art: pa.PathArtifact) -> dict:
    """What app-plan admission changed (docs/contracts/backend.md section 1b); zero for an artifact that predates it."""
    meta = art.meta or {}
    return {
        "densified_steps": int(meta.get("densified_steps", 0)),
        "max_boundary_snap_m": float(meta.get("max_boundary_snap_m", 0.0)),
    }


class MissionService:
    def __init__(self, settings: Settings, engine_id: str | None = None) -> None:
        # Imported here because the planner imports this module (MissionError).
        from dyx3_backend.mission.planner import Planner

        self._s = settings
        self._engine_id = engine_id or pa.engine_id_from_origin_file(ORIGIN_SHA)
        # One planning job at a time, in its own process, within plan_timeout_s (BE-004).
        self.planner = Planner(settings.plan_timeout_s)

    def ingest(self, filename: str, data: bytes, params: PlanParams) -> dict:
        """Blocking: call it in a worker thread. Planning itself runs in a separate process (``planner``)."""
        from dyx3_backend.mission import planner

        ext = os.path.splitext(filename or "")[1].lower()
        if ext not in self._s.allowed_extensions:
            raise MissionError(415, "unsupported_type", f"extension {ext or '(none)'} is not one of {list(self._s.allowed_extensions)}")
        if len(data) == 0:
            raise MissionError(422, "empty_file", "the uploaded file is empty")
        if len(data) > self._s.upload_max_bytes:
            raise MissionError(413, "too_large", f"upload exceeds {self._s.upload_max_bytes} bytes")
        params.validate()
        blob = self.planner.run(planner.plan_dxf_job, filename, data, params, self._engine_id, self._s.plan_max_points)
        try:
            digest, _ = pa.store(self._s.missions_dir, blob)
            art = pa.load(self._s.missions_dir, digest)
        except (pa.ArtifactError, ValueError, OSError) as exc:
            raise MissionError(422, "artifact_failed", f"{type(exc).__name__}: {exc}") from exc
        return summarize(art)

    def ingest_app_plan(self, raw: bytes) -> tuple[dict, dict]:
        """Parse, validate and store an app plan (raw JSON body). Blocking: call it in a worker thread.

        Returns ``(summary, normalisation)``; both are read back from the stored artifact. Parsing and compiling run in
        the planning process; this path never imports the path engine.
        """
        from dyx3_backend.mission import planner

        blob = self.planner.run(planner.app_plan_job, bytes(raw))
        try:
            digest, _ = pa.store(self._s.missions_dir, blob)
            art = pa.load(self._s.missions_dir, digest)
        except (pa.ArtifactError, OSError) as exc:
            raise MissionError(422, "ARTIFACT_FAILED", f"{type(exc).__name__}: {exc}") from exc
        return summarize(art), normalisation(art)

    def parse_dxf(self, filename: str, data: bytes) -> dict:
        """Parse-only DXF import (no artifact). Blocking: call it in a worker thread; ezdxf runs in the planning process."""
        from dyx3_backend.mission import planner
        from dyx3_backend.mission.parse_import import check_dxf_upload

        check_dxf_upload(filename, data, self._s.upload_max_bytes)  # cheap refusals before a process is started
        return self.planner.run(planner.parse_dxf_job, filename, data, self._s.upload_max_bytes)

    def get(self, sha256: str) -> pa.PathArtifact:
        try:
            return pa.load(self._s.missions_dir, sha256)
        except pa.ArtifactError as exc:
            raise MissionError(400, "bad_id", str(exc)) from exc
        except FileNotFoundError as exc:
            raise MissionError(404, "not_found", "no such mission artifact") from exc

    def list(self) -> list[dict]:
        out = []
        try:
            names = [n for n in os.listdir(self._s.missions_dir) if n.endswith(pa.EXTENSION)]
        except OSError:
            return out
        names.sort(key=lambda n: os.path.getmtime(os.path.join(self._s.missions_dir, n)), reverse=True)
        for n in names:
            try:
                out.append(summarize(pa.load(self._s.missions_dir, n[: -len(pa.EXTENSION)])))
            except (pa.ArtifactError, OSError):
                continue  # a corrupt file is skipped, never served
        return out
