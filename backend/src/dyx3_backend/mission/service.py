"""Mission admission and lookup. Contract: docs/contracts/backend.md sections 1 and 1b.

The tablet app is the only trajectory author. A validated app plan (``app_plan.py``) becomes a content-addressed
``DYX3PATH 1`` artifact (docs/contracts/path_artifact.md); the id is the sha256 of its bytes, so storing the same
mission twice is a no-op. The backend plans no geometry.
"""

from __future__ import annotations

import os

from dyx3_backend.config.settings import Settings
from dyx3_backend.mission import path_artifact as pa


class MissionError(Exception):
    def __init__(self, status: int, code: str, reason: str) -> None:
        super().__init__(reason)
        self.status = status
        self.code = code
        self.reason = reason


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
    def __init__(self, settings: Settings) -> None:
        # Imported here because the planner imports this module (MissionError).
        from dyx3_backend.mission.planner import Planner

        self._s = settings
        # One admission job at a time, in its own process, within plan_timeout_s (BE-004).
        self.planner = Planner(settings.plan_timeout_s)

    def ingest_app_plan(self, raw: bytes) -> tuple[dict, dict]:
        """Parse, validate and store an app plan (raw JSON body). Blocking: call it in a worker thread.

        Returns ``(summary, normalisation)``; both are read back from the stored artifact. Parsing and compiling run in
        the planning process (BE-004), never on the event loop.
        """
        from dyx3_backend.mission import planner

        blob = self.planner.run(planner.app_plan_job, bytes(raw))
        try:
            digest, _ = pa.store(self._s.missions_dir, blob)
            art = pa.load(self._s.missions_dir, digest)
        except (pa.ArtifactError, OSError) as exc:
            raise MissionError(422, "ARTIFACT_FAILED", f"{type(exc).__name__}: {exc}") from exc
        return summarize(art), normalisation(art)

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
