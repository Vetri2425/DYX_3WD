"""Parse-only import endpoints used by the operator app."""

from __future__ import annotations

from typing import Annotated

import anyio
from fastapi import APIRouter, Depends, File, Request, UploadFile

from dyx3_backend.api.routes import _mission_error, require
from dyx3_backend.auth.tokens import Role
from dyx3_backend.mission.parse_import import parse_dxf_upload
from dyx3_backend.mission.service import MissionError

router = APIRouter(prefix="/api/path")


@router.post("/parse-dxf")
async def parse_dxf_file(
    request: Request, file: Annotated[UploadFile, File()], _: object = Depends(require(Role.OPERATOR))
):
    data = await file.read(request.app.state.settings.upload_max_bytes + 1)
    try:
        return await anyio.to_thread.run_sync(
            parse_dxf_upload, file.filename or "", data, request.app.state.settings.upload_max_bytes
        )
    except MissionError as exc:
        return _mission_error(exc)
