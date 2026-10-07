"""REST surface. Contract: docs/contracts/backend.md section 1."""

from __future__ import annotations

from typing import Annotated, Literal

import anyio
from fastapi import APIRouter, Depends, File, Form, Header, HTTPException, Request, UploadFile
from fastapi.responses import JSONResponse
from pydantic import BaseModel, ConfigDict, StrictBool

from dyx3_backend.auth.tokens import Identity, Role
from dyx3_backend.gateway.client import GatewayError
from dyx3_backend.mission.service import MissionError, PlanParams, summarize
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


# ------------------------------------------------------------------------------------------------ auth
def _identity(request: Request, authorization: str | None) -> Identity:
    if authorization is None or not authorization.lower().startswith("bearer "):
        raise HTTPException(401, "missing bearer token", headers={"WWW-Authenticate": "Bearer"})
    ident = request.app.state.tokens.verify(authorization[7:].strip())
    if ident is None:
        raise HTTPException(401, "invalid token", headers={"WWW-Authenticate": "Bearer"})
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


async def send(request: Request, cmd: str, args: dict | None = None) -> JSONResponse:
    """Forward one request; report exactly what happened, including 'not delivered'."""
    gw = request.app.state.gateway
    try:
        reply = await gw.request(cmd, args)
    except GatewayError as exc:
        status = 503 if exc.delivered is False else 504
        body = {"ok": False, "code": type(exc).__name__, "reason": str(exc), "delivered": exc.delivered, "data": {}}
        return JSONResponse(body, status_code=status)
    ok = bool(reply.get("ok"))
    code = str(reply.get("code", "?"))
    body = {"ok": ok, "code": code, "reason": reply.get("reason", ""), "delivered": True, "data": reply.get("data", {})}
    return JSONResponse(body, status_code=200 if ok else _STATUS.get(code, 502))


# ------------------------------------------------------------------------------------------------ routes
@router.get("/ping")
async def ping() -> dict[str, str]:
    return {"status": "ok"}


@router.get("/health")
async def health(request: Request, _: Identity = Viewer) -> dict:
    gw, relay, settings = request.app.state.gateway, request.app.state.relay, request.app.state.settings
    age = gw.snapshot_age()
    return {
        "backend": "ok",
        "gateway_connected": gw.connected,
        "telemetry_age_s": age,
        "telemetry_fresh": age is not None and age <= settings.telemetry_stale_s,
        "tablet_heartbeat_age_s": relay.tablet_age(),
        "tablet_alive": relay.tablet_alive(),
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


@router.post("/missions")
async def upload_mission(
    request: Request,
    file: Annotated[UploadFile, File()],
    origin_n: Annotated[float, Form()] = 0.0,
    origin_e: Annotated[float, Form()] = 0.0,
    rotation_deg: Annotated[float, Form()] = 0.0,
    unit_scale: Annotated[float | None, Form()] = None,
    close_loop: Annotated[bool, Form()] = False,
    anchor: Annotated[str, Form()] = "drawing_origin",
    _: Identity = Operator,
) -> JSONResponse:
    svc, settings = request.app.state.missions, request.app.state.settings
    data = await file.read(settings.upload_max_bytes + 1)  # never buffer more than the limit + 1
    params = PlanParams(origin_n, origin_e, rotation_deg, unit_scale, close_loop, anchor)
    try:
        summary = await anyio.to_thread.run_sync(svc.ingest, file.filename or "", data, params)
    except MissionError as exc:
        return _mission_error(exc)
    return JSONResponse({"ok": True, "mission": summary}, status_code=201)


@router.get("/missions")
async def list_missions(request: Request, _: Identity = Viewer) -> dict:
    return {"missions": await anyio.to_thread.run_sync(request.app.state.missions.list)}


@router.get("/missions/{sha}")
async def get_mission(sha: str, request: Request, _: Identity = Viewer):
    try:
        return {"mission": summarize(request.app.state.missions.get(sha))}
    except MissionError as exc:
        return _mission_error(exc)


@router.get("/missions/{sha}/path")
async def get_mission_path(sha: str, request: Request, _: Identity = Viewer):
    try:
        art = request.app.state.missions.get(sha)
    except MissionError as exc:
        return _mission_error(exc)
    return {
        "sha256": art.sha256,
        "frame": "local_ned",
        "points": [[p.north_m, p.east_m, p.flags] for p in art.points],
    }


@router.post("/missions/{sha}/start")
async def start_mission(sha: str, request: Request, _: Identity = Operator) -> JSONResponse:
    if len(sha) != 64 or any(c not in "0123456789abcdef" for c in sha):
        return JSONResponse({"ok": False, "code": "bad_id", "reason": "sha256 must be 64 lowercase hex characters"}, status_code=400)
    try:
        request.app.state.missions.get(sha)  # never start a mission whose artifact this side cannot read
    except MissionError as exc:
        return _mission_error(exc)
    return await send(request, "start_mission", {"path_artifact_sha256": sha})


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


