"""Parse-only import endpoints used by the operator app."""

from __future__ import annotations

from typing import Annotated

import anyio
from fastapi import APIRouter, Depends, File, Request, UploadFile

from dyx3_backend.api.routes import _mission_error, require
from dyx3_backend.auth.tokens import Role
from dyx3_backend.mission.service import MissionError

router = APIRouter(prefix="/api/path")


@router.post("/parse-dxf")
async def parse_dxf_file(
    request: Request, file: Annotated[UploadFile, File()], _: object = Depends(require(Role.OPERATOR))
):
    data = await file.read(request.app.state.settings.upload_max_bytes + 1)
    try:
        # ezdxf runs in the planning process (BE-004): one job at a time, within plan_timeout_s.
        return await anyio.to_thread.run_sync(request.app.state.missions.parse_dxf, file.filename or "", data)
    except MissionError as exc:
        return _mission_error(exc)
