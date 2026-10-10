"""REST surface. Contract: docs/contracts/backend.md section 1."""

from __future__ import annotations

import json
import re
from pathlib import Path
from typing import Annotated, Literal

import anyio
from fastapi import APIRouter, Depends, Header, HTTPException, Request
from fastapi.responses import JSONResponse, Response
from pydantic import BaseModel, ConfigDict, StrictBool, StrictStr

from dyx3_backend.api.admission import bearer_identity
from dyx3_backend.auth.tokens import Identity, Role
from dyx3_backend.gateway.client import GatewayError
from dyx3_backend.mission.service import MissionError, summarize
from dyx3_backend.rtk.client import RtkRejected, RtkUnavailable
from dyx3_backend.storage import runs as runs_store

router = APIRouter(prefix="/api")


class _Body(BaseModel):
    model_config = ConfigDict(extra="forbid")


class EstopBody(_Body):
    asserted: StrictBool


class AbortBody(_Body):
    reason: Literal["operator", "safety", "unspecified"] = "operator"


class ArmBody(_Body):
    arm: StrictBool


class OffboardBody(_Body):
    enable: StrictBool


class SprayBody(_Body):
    on: StrictBool


class StartBody(_Body):
    request_id: StrictStr | None = None


class RtkSourceBody(_Body):
    source: Literal["NTRIP", "LORA"]
    revision: int | None = None


class RtkTransportBody(_Body):
    transport: Literal["USB_DIRECT", "PX4_DDS"]
    revision: int | None = None


# ------------------------------------------------------------------------------------------------ auth
def _identity(request: Request, authorization: str | None) -> Identity:
    # AdmissionMiddleware already refused a missing or unknown token before the body was read; this repeats the
    # same check so a route is never reachable without it (and yields the Identity for the role check).
    ident, why = bearer_identity(request.app.state.tokens, authorization)
    if ident is None:
        raise HTTPException(401, why, headers={"WWW-Authenticate": "Bearer"})
    return ident


def require(role: Role):
    async def dep(request: Request, authorization: Annotated[str | None, Header()] = None) -> Identity:
        ident = _identity(request, authorization)
        if not ident.can(role):
            raise HTTPException(403, f"requires role {role.name.lower()}")
        return ident

    return dep


Viewer = Depends(require(Role.VIEWER))
Operator = Depends(require(Role.OPERATOR))


async def any_authenticated(request: Request, authorization: Annotated[str | None, Header()] = None) -> Identity:
    return _identity(request, authorization)


AnyRole = Depends(any_authenticated)


# ------------------------------------------------------------------------------------------------ gateway verdicts
_STATUS = {"rejected": 409, "invalid_command": 400, "bad_message": 400, "service_unavailable": 503, "timeout": 504, "busy": 503}


async def forward(request: Request, cmd: str, args: dict | None = None) -> tuple[int, dict]:
    """Forward one request; return (HTTP status, typed body) for exactly what happened, including 'not delivered'.

    Every body is ``{ok, code, reason, delivered, data}``; ``data`` (with the downstream ``reason_code``) is the gateway's,
    untouched. A delivered ``ok`` verdict is 200 here; a route may answer it differently (start: 202).
    """
    gw = request.app.state.gateway
    try:
        reply = await gw.request(cmd, args)
    except GatewayError as exc:
        status = 503 if exc.delivered is False else 504
        return status, {"ok": False, "code": type(exc).__name__, "reason": str(exc), "delivered": exc.delivered, "data": {}}
    ok = bool(reply.get("ok"))
    code = str(reply.get("code", "?"))
    body = {"ok": ok, "code": code, "reason": reply.get("reason", ""), "delivered": True, "data": reply.get("data", {})}
    return (200 if ok else _STATUS.get(code, 502)), body


async def send(request: Request, cmd: str, args: dict | None = None) -> JSONResponse:
    status, body = await forward(request, cmd, args)
    return JSONResponse(body, status_code=status)


# ------------------------------------------------------------------------------------------------ routes
@router.get("/ping")
async def ping(request: Request) -> dict[str, str]:
    # Unauthenticated liveness + identity, so the app can tell rovers apart by id (not IP) and key its
    # saved token per rover. No secret here.
    ident = request.app.state.identity
    return {"status": "ok", "rover_id": ident.rover_id, "rover_name": ident.rover_name}


@router.get("/health")
async def health(request: Request, _: Identity = Viewer) -> dict:
    gw, relay, settings = request.app.state.gateway, request.app.state.relay, request.app.state.settings
    age = gw.snapshot_age()
    fresh = age is not None and age <= settings.telemetry_stale_s
    return {
        "backend": "ok",
        "gateway_connected": gw.connected,
        "telemetry_age_s": age,
        "telemetry_fresh": fresh,
        "tablet_heartbeat_age_s": relay.tablet_age(),
        "tablet_alive": relay.tablet_alive(),
        "relay_running": relay.running,
        "operator_alive": relay.operator_alive(),
        "mission": _mission_health(gw.snapshot, fresh),
    }


# Names of the MissionState numbers (ros2_ws/src/dyx3_interfaces/msg/MissionState.msg, interfaces 0.15.0; the rover contract
# is docs/contracts/dyx3_mission.md). The tablet gets the numbers in the ``mission_state`` event; these names are for /health.
MISSION_STATES = {
    0: "IDLE", 1: "LOADING", 2: "READY", 3: "RUNNING", 4: "PAUSED", 5: "COMPLETED", 6: "ABORTED", 7: "ERROR",
    8: "PLACING", 9: "ARMING", 10: "ENGAGING",
}
MISSION_REASONS = {
    0: "NONE", 1: "OPERATOR", 2: "SAFETY", 3: "RTK", 4: "PATH_ERROR", 5: "INTERNAL_ERROR",
    6: "EKF_RESET", 7: "EKF_REFERENCE_INVALID", 8: "PLACEMENT_OUT_OF_BOUNDS", 9: "NO_PLACEMENT_FRAME", 10: "ARM_REFUSED",
    11: "ARM_TIMEOUT", 12: "OFFBOARD_REFUSED", 13: "OFFBOARD_TIMEOUT", 14: "RPP_ACK_TIMEOUT", 15: "ESTOP", 16: "RPP_ERROR",
    17: "RPP_STALE",
}
MISSION_WAITING_ON = {
    0: "NONE", 1: "ARTIFACT", 2: "PLACEMENT", 3: "ARM", 4: "OFFBOARD", 5: "RPP_ACK", 6: "OPERATOR", 7: "OFFBOARD_RELEASE",
    8: "DISARM",
}


def _name(table: dict[int, str], value: object) -> str | None:
    """The table's name for a snapshot number; ``UNKNOWN_<n>`` for a number a newer rover adds; null when not a number."""
    if isinstance(value, bool) or not isinstance(value, int):
        return None
    return table.get(value, f"UNKNOWN_{value}")


def _mission_health(snapshot: dict | None, telemetry_fresh: bool) -> dict:
    """Diagnostic view of the snapshot's ``mission`` source (the tablet gets mission progress as events, not here).

    ``lifecycle_state`` is the name of ``state``; ``waiting_on`` the name of the step being waited on; ``last_error`` is
    the current reason (``reason_code`` other than NONE) with its ``reason_detail`` and, for a guard gate, ``gate_reason_code``,
    or null when there is none. A field the snapshot does not carry is null; ``fresh`` is false unless both the snapshot and
    its mission source are.
    """
    m = snapshot.get("mission") if isinstance(snapshot, dict) else None
    m = m if isinstance(m, dict) else {}
    reason = m.get("reason_code")
    last_error = None
    if isinstance(reason, int) and not isinstance(reason, bool) and reason != 0:
        last_error = {
            "reason_code": reason,
            "reason": _name(MISSION_REASONS, reason),
            "detail": m.get("reason_detail"),
            "gate_reason_code": m.get("gate_reason_code"),
        }
    return {
        "lifecycle_state": _name(MISSION_STATES, m.get("state")),
        "last_error": last_error,
        "waiting_on": _name(MISSION_WAITING_ON, m.get("waiting_on")),
        "age_s": m.get("age_s"),
        "fresh": telemetry_fresh and m.get("fresh") is True,
    }


@router.get("/telemetry")
async def telemetry(request: Request, _: Identity = Viewer) -> dict:
    gw = request.app.state.gateway
    return {"connected": gw.connected, "age_s": gw.snapshot_age(), "snapshot": gw.snapshot}


@router.post("/heartbeat")
async def heartbeat(request: Request, _: Identity = Operator) -> dict:
    request.app.state.relay.note_tablet()
    return {"ok": True}


@router.post("/estop")
async def estop(request: Request, body: EstopBody, who: Identity = AnyRole) -> JSONResponse:
    if not body.asserted and not who.can(Role.OPERATOR):
        raise HTTPException(403, "clearing the emergency stop requires role operator")
    return await send(request, "estop", {"asserted": body.asserted, "source": "tablet"})


@router.post("/mission/abort")
async def abort(request: Request, body: AbortBody | None = None, _: Identity = Operator) -> JSONResponse:
    return await send(request, "abort_mission", {"reason": (body or AbortBody()).reason})


@router.post("/mission/pause")
async def pause(request: Request, _: Identity = Operator) -> JSONResponse:
    return await send(request, "pause_mission")


@router.post("/mission/resume")
async def resume(request: Request, _: Identity = Operator) -> JSONResponse:
    return await send(request, "resume_mission")


@router.post("/mission/skip_point")
async def skip_point(request: Request, _: Identity = Operator) -> JSONResponse:
    return await send(request, "skip_point")


@router.post("/vehicle/arm")
async def arm(request: Request, body: ArmBody, _: Identity = Operator) -> JSONResponse:
    return await send(request, "arm", {"arm": body.arm})


@router.post("/vehicle/offboard")
async def offboard(request: Request, body: OffboardBody, _: Identity = Operator) -> JSONResponse:
    return await send(request, "offboard", {"enable": body.enable})


@router.post("/spray/manual")
async def spray_manual(request: Request, body: SprayBody, _: Identity = Operator) -> JSONResponse:
    return await send(request, "spray_manual", {"on": body.on})


# ------------------------------------------------------------------------------------------------ missions
def _mission_error(exc: MissionError) -> JSONResponse:
    return JSONResponse({"ok": False, "code": exc.code, "reason": exc.reason}, status_code=exc.status)


@router.post("/missions/plan")
async def ingest_app_plan(request: Request, _: Identity = Operator) -> JSONResponse:
    limit = request.app.state.settings.upload_max_bytes
    content_length = request.headers.get("content-length")
    if content_length is not None and content_length.isdecimal() and int(content_length) > limit:
        return _mission_error(MissionError(413, "too_large", f"upload exceeds {limit} bytes"))
    data = bytearray()
    async for chunk in request.stream():
        if len(chunk) > limit - len(data):
            return _mission_error(MissionError(413, "too_large", f"upload exceeds {limit} bytes"))
        data.extend(chunk)
    try:
        # JSON parsing and compiling run in the planning process (BE-004), never on the event loop (BE-010).
        summary, normalisation = await anyio.to_thread.run_sync(request.app.state.missions.ingest_app_plan, bytes(data))
    except MissionError as exc:
        return _mission_error(exc)
    return JSONResponse({"ok": True, "mission": summary, "normalisation": normalisation}, status_code=201)


@router.get("/missions")
async def list_missions(request: Request, _: Identity = Viewer) -> dict:
    return {"missions": await anyio.to_thread.run_sync(request.app.state.missions.list)}


@router.get("/missions/{sha}")
async def get_mission(sha: str, request: Request, _: Identity = Viewer):
    svc = request.app.state.missions
    try:
        # read + hash + decode off the event loop (BE-005)
        return {"mission": await anyio.to_thread.run_sync(lambda: summarize(svc.get(sha)))}
    except MissionError as exc:
        return _mission_error(exc)


def _render_path(svc, sha: str) -> bytes:
    art = svc.get(sha)
    meta = art.meta or {}
    body = {
        "sha256": art.sha256,
        # What the points are relative to (meta, docs/contracts/backend.md section 1b): "local_ned" = the anchor's
        # local NE, "ekf_local_ned" = the rover's EKF local frame. null = the artifact records no frame; the rover refuses
        # to place it (NO_PLACEMENT_FRAME), so the preview must not invent one.
        "frame": meta.get("frame"),
        "anchor": meta.get("anchor"),
        "points": [[p.north_m, p.east_m, p.flags] for p in art.points],
    }
    # Same rendering as JSONResponse, done here so a 200 000-point body is not serialised on the event loop.
    return json.dumps(body, ensure_ascii=False, allow_nan=False, indent=None, separators=(",", ":")).encode("utf-8")


@router.get("/missions/{sha}/path")
async def get_mission_path(sha: str, request: Request, _: Identity = Viewer):
    try:
        content = await anyio.to_thread.run_sync(_render_path, request.app.state.missions, sha)  # BE-005
    except MissionError as exc:
        return _mission_error(exc)
    return Response(content=content, media_type="application/json")


# A client request id (idempotency key): printable token, so it is safe in the gateway's JSON, ROS strings and logs.
REQUEST_ID = re.compile(r"[A-Za-z0-9._:-]{1,64}")


def _request_id(body: StartBody | None, header: str | None) -> str | None:
    ids = {v for v in ((body.request_id if body else None), header) if v is not None}
    if len(ids) > 1:
        raise MissionError(422, "invalid_request_id", "request_id and the Idempotency-Key header differ")
    rid = ids.pop() if ids else None
    if rid is not None and not REQUEST_ID.fullmatch(rid):
        raise MissionError(422, "invalid_request_id", "request_id must be 1-64 characters of A-Z a-z 0-9 . _ : -")
    return rid


@router.post("/missions/{sha}/start")
async def start_mission(
    sha: str,
    request: Request,
    body: StartBody | None = None,
    idempotency_key: Annotated[str | None, Header()] = None,
    _: Identity = Operator,
) -> JSONResponse:
    """Ask the rover to start; 202 once the mission node has ACCEPTED the start (progress follows as events)."""
    if len(sha) != 64 or any(c not in "0123456789abcdef" for c in sha):
        return JSONResponse({"ok": False, "code": "bad_id", "reason": "sha256 must be 64 lowercase hex characters"}, status_code=400)
    try:
        rid = _request_id(body, idempotency_key)
        # never start a mission whose artifact this side cannot read; read + verify off the event loop (BE-005)
        await anyio.to_thread.run_sync(request.app.state.missions.get, sha)
    except MissionError as exc:
        return _mission_error(exc)
    args = {"path_artifact_sha256": sha}
    if rid is not None:
        args["request_id"] = rid
    status, reply = await forward(request, "start_mission", args)
    if not reply["ok"]:
        return JSONResponse(reply, status_code=status)
    data = reply["data"]
    # mission_id is the execution id; a duplicate is the existing execution (nothing new started).
    execution = {
        "mission_id": data.get("mission_id"),
        "request_id": rid,
        "duplicate": data.get("duplicate"),
        "gate_reason_code": data.get("gate_reason_code"),
    }
    return JSONResponse({"ok": True, "accepted": True, "execution": execution, "data": data}, status_code=202)


# ------------------------------------------------------------------------------------------------ runs
@router.get("/runs")
async def list_runs(request: Request, _: Identity = Viewer) -> dict:
    return {"runs": await anyio.to_thread.run_sync(runs_store.list_runs, request.app.state.settings.runs_dir)}


@router.get("/runs/{run_id}")
async def get_run(run_id: str, request: Request, _: Identity = Viewer):
    run = await anyio.to_thread.run_sync(runs_store.get_run, request.app.state.settings.runs_dir, run_id)
    if run is None:
        return JSONResponse({"ok": False, "code": "not_found", "reason": "no such run"}, status_code=404)
    return run


# ------------------------------------------------------------------------------------------------ RTK
async def _rtk(request: Request, command: str, **arguments) -> dict:
    try:
        return await request.app.state.rtk.request(command, **arguments)
    except RtkUnavailable as exc:
        raise HTTPException(503, "RTK worker unavailable") from exc
    except RtkRejected as exc:
        status = 409 if exc.code == "conflict" or "revision conflict" in exc.reason else 400
        raise HTTPException(status, {"code": exc.code, "reason": exc.reason}) from exc


async def _rtk_config(request: Request) -> dict:
    config = await _rtk(request, "GET_CONFIG")
    if not isinstance(config, dict):
        raise HTTPException(503, "invalid RTK worker reply")
    return config


@router.get("/rtk/status")
async def rtk_status(request: Request, _: Identity = Viewer) -> dict:
    return await _rtk(request, "GET_STATUS")


@router.get("/rtk/config")
async def rtk_config(request: Request, _: Identity = Viewer) -> dict:
    return await _rtk_config(request)


@router.put("/rtk/config")
async def set_rtk_config(request: Request, config: dict, _: Identity = Operator) -> dict:
    return await _rtk(request, "SET_CONFIG", config=config)


@router.post("/rtk/start")
async def start_rtk(request: Request, _: Identity = Operator) -> dict:
    return await _rtk(request, "START")


@router.post("/rtk/stop")
async def stop_rtk(request: Request, _: Identity = Operator) -> dict:
    return await _rtk(request, "STOP")


@router.get("/rtk/source")
async def get_rtk_source(request: Request, _: Identity = Viewer) -> dict:
    config = await _rtk_config(request)
    return {"source": config["source"], "revision": config["revision"]}


@router.put("/rtk/source")
async def set_rtk_source(request: Request, body: RtkSourceBody, _: Identity = Operator) -> dict:
    config = await _rtk_config(request)
    if body.revision is not None and body.revision != config["revision"]:
        raise HTTPException(409, "RTK configuration revision conflict")
    config["source"] = body.source
    return await _rtk(request, "SET_CONFIG", config=config)


@router.get("/rtk/transport")
async def get_rtk_transport(request: Request, _: Identity = Viewer) -> dict:
    config = await _rtk_config(request)
    return {"transport": config["transport"], "revision": config["revision"]}


@router.put("/rtk/transport")
async def set_rtk_transport(request: Request, body: RtkTransportBody, _: Identity = Operator) -> dict:
    config = await _rtk_config(request)
    if body.revision is not None and body.revision != config["revision"]:
        raise HTTPException(409, "RTK configuration revision conflict")
    config["transport"] = body.transport
    return await _rtk(request, "SET_CONFIG", config=config)


@router.get("/rtk/profiles")
async def list_rtk_profiles(request: Request, _: Identity = Viewer) -> dict:
    config = await _rtk_config(request)
    return {"profiles": config["ntrip"]["profiles"], "active_profile_id": config["ntrip"]["active_profile_id"],
            "revision": config["revision"]}


@router.post("/rtk/profiles")
async def create_rtk_profile(request: Request, profile: dict, _: Identity = Operator) -> dict:
    config = await _rtk_config(request)
    config["ntrip"]["profiles"].append(profile)
    return await _rtk(request, "SET_CONFIG", config=config)


@router.patch("/rtk/profiles/{profile_id}")
async def update_rtk_profile(profile_id: str, request: Request, changes: dict, _: Identity = Operator) -> dict:
    config = await _rtk_config(request)
    for profile in config["ntrip"]["profiles"]:
        if profile["id"] == profile_id:
            if "id" in changes and changes["id"] != profile_id:
                raise HTTPException(400, "profile id cannot change")
            profile.update(changes)
            return await _rtk(request, "SET_CONFIG", config=config)
    raise HTTPException(404, "RTK profile not found")


@router.delete("/rtk/profiles/{profile_id}")
async def delete_rtk_profile(profile_id: str, request: Request, _: Identity = Operator) -> dict:
    config = await _rtk_config(request)
    profiles = config["ntrip"]["profiles"]
    remaining = [profile for profile in profiles if profile["id"] != profile_id]
    if len(remaining) == len(profiles):
        raise HTTPException(404, "RTK profile not found")
    config["ntrip"]["profiles"] = remaining
    if config["ntrip"]["active_profile_id"] == profile_id:
        config["ntrip"]["active_profile_id"] = ""
    return await _rtk(request, "SET_CONFIG", config=config)


SERIAL_PORT_DIRS = (Path("/dev/serial/by-id"), Path("/dev/serial/by-path"))


def _list_serial_ports() -> list[dict]:
    # by-path covers adapters without a USB serial number (the rover's CH340). Never auto-selected.
    ports = []
    for directory in SERIAL_PORT_DIRS:
        if not directory.is_dir():
            continue
        for entry in sorted(directory.iterdir(), key=lambda item: item.name)[:128]:
            if entry.is_symlink() and entry.name not in (".", ".."):
                ports.append({"path": str(entry), "present": entry.exists()})
    return ports


@router.get("/rtk/serial-ports")
async def rtk_serial_ports(_: Identity = Viewer) -> dict:
    # the directory walk touches the filesystem: off the event loop, like the other routes
    return {"ports": await anyio.to_thread.run_sync(_list_serial_ports)}
