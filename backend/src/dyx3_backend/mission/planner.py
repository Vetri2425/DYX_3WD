"""Mission planning in a separate process, with a wall-clock and an output budget (BE-004).

Contract: docs/contracts/backend.md section 1 ("Planning budget").

The path engine and the app-plan compiler are CPU-bound Python. In a worker thread they share the GIL with the event
loop that relays the operator-link heartbeat, so a pathological DXF uploaded while the rover is marking could slow
the relay into a false STOP and hold the planner indefinitely. Here every job runs in a fresh ``spawn``-ed process
(never ``fork``: the backend has threads), one job at a time; a job over its wall-clock budget is terminated (then
killed) and reported as an error. The caller blocks in ``Planner.run`` from a worker thread; that thread waits on a
pipe with the GIL released, so the event loop is never blocked.

Results cross the pipe as plain tuples (``MissionError`` does not round-trip through pickle).
"""

from __future__ import annotations

import json
import multiprocessing
import os
import tempfile
import threading

from dyx3_backend.mission import path_artifact as pa
from dyx3_backend.mission.service import MissionError

_CTX = multiprocessing.get_context("spawn")
_REAP_S = 2.0


# ------------------------------------------------------------------------------------------------ jobs (child side)
def plan_dxf_job(filename: str, data: bytes, params, engine_id: str, max_points: int) -> bytes:
    """Run the path engine on an upload and encode the artifact. Returns the ``DYX3PATH 1`` bytes."""
    ext = os.path.splitext(filename or "")[1].lower()
    from dyx3_backend.path_engine.engine import PathEngine  # heavy import (ezdxf): child only

    with tempfile.TemporaryDirectory(prefix="dyx3-upload-") as td:
        path = os.path.join(td, "upload" + ext)  # the client's name is never used as a path
        with open(path, "wb") as fh:
            fh.write(data)
        try:
            plan = PathEngine().plan_file(
                path,
                unit_scale=params.unit_scale,
                origin=(params.origin_n, params.origin_e),
                rotation_deg=params.rotation_deg,
                close_loop=params.close_loop,
                anchor=params.anchor,
            )
        except Exception as exc:  # the engine raises many types for bad CAD input
            raise MissionError(422, "plan_failed", f"{type(exc).__name__}: {exc}") from exc
    n = len(getattr(plan, "merged_waypoints", ()) or ())
    if n > max_points:
        raise MissionError(422, "points_limit_exceeded", f"the plan has {n} points, over the {max_points} limit")
    try:
        return pa.encode_plan(plan, engine_id=engine_id, source_name=filename, source_bytes=data)
    except (pa.ArtifactError, ValueError) as exc:
        raise MissionError(422, "artifact_failed", f"{type(exc).__name__}: {exc}") from exc


def app_plan_job(raw: bytes) -> bytes:
    """Parse and compile an app-planned mission body. Returns the ``DYX3PATH 1`` bytes."""
    from dyx3_backend.mission.app_plan import compile_plan

    try:
        body = json.loads(raw)
    except (json.JSONDecodeError, UnicodeDecodeError) as exc:
        raise MissionError(400, "INVALID_PAYLOAD", f"invalid JSON: {exc}") from exc
    except RecursionError as exc:
        raise MissionError(400, "INVALID_PAYLOAD", "invalid JSON: nested too deeply") from exc
    try:
        return compile_plan(body)
    except pa.ArtifactError as exc:
        raise MissionError(422, "ARTIFACT_FAILED", f"{type(exc).__name__}: {exc}") from exc


def parse_dxf_job(filename: str, data: bytes, max_bytes: int) -> dict:
    from dyx3_backend.mission.parse_import import parse_dxf_upload

    return parse_dxf_upload(filename, data, max_bytes)


def _child(conn, fn, args) -> None:
    try:
        try:
            result = ("ok", fn(*args))
        except MissionError as exc:
            result = ("err", exc.status, exc.code, exc.reason)
        except RecursionError:
            result = ("err", 422, "plan_failed", "input nested too deeply")
        except MemoryError:
            result = ("err", 422, "plan_failed", "planning ran out of memory")
        except Exception as exc:  # noqa: BLE001 - any failure is reported to the caller, never lost
            result = ("err", 422, "plan_failed", f"{type(exc).__name__}: {exc}")
        conn.send(result)
    finally:
        conn.close()


# ------------------------------------------------------------------------------------------------ runner (parent side)
class Planner:
    """One planning job at a time, each in its own process, each bounded by ``timeout_s`` of wall clock."""

    def __init__(self, timeout_s: float) -> None:
        self._timeout = float(timeout_s)
        self._lock = threading.Lock()

    @property
    def busy(self) -> bool:
        return self._lock.locked()

    def run(self, fn, *args):
        """Blocking: call from a worker thread. Raises ``MissionError`` (409 busy, 422 budget/plan, 500 crash)."""
        if not self._lock.acquire(blocking=False):
            raise MissionError(409, "busy", "another planning job is running; retry when it has finished")
        try:
            return self._run(fn, args)
        finally:
            self._lock.release()

    def _run(self, fn, args):
        reader, writer = _CTX.Pipe(duplex=False)
        proc = _CTX.Process(target=_child, args=(writer, fn, args), name="dyx3-planner", daemon=True)
        try:
            proc.start()
        except OSError as exc:
            reader.close()
            raise MissionError(503, "planner_unavailable", f"cannot start the planning process: {exc}") from exc
        finally:
            writer.close()  # the child holds the only write end: EOF if it dies without answering
        try:
            if not reader.poll(self._timeout):
                raise MissionError(
                    422, "plan_budget_exceeded", f"planning exceeded its {self._timeout:g} s budget and was stopped"
                )
            try:
                msg = reader.recv()
            except EOFError:
                proc.join(_REAP_S)
                raise MissionError(500, "planner_crashed", f"the planning process exited without a result "
                                                           f"(exit code {proc.exitcode})") from None
        finally:
            reader.close()
            _reap(proc)
        if msg[0] == "ok":
            return msg[1]
        _, status, code, reason = msg
        raise MissionError(status, code, reason)


def _reap(proc) -> None:
    """Never leave a planning process behind: terminate, then kill."""
    proc.join(0.05 if proc.is_alive() else _REAP_S)
    if proc.is_alive():
        proc.terminate()
        proc.join(_REAP_S)
    if proc.is_alive():
        proc.kill()
        proc.join(_REAP_S)
