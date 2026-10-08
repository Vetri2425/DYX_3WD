"""DYX 3WD backend entrypoint.

Owns: REST, Socket.IO, auth, mission upload/report, telemetry delivery, settings,
storage, NTRIP profile management, and CAD/CRS path ingestion.

Does NOT own: rclpy, ROS executors, direct PX4 commands, motor safety logic,
steering, or RPP corrections. Reaches ROS only through dyx3_system_gateway over a
Unix domain socket.

Backend emergency stop is a REQUEST. Final motion authority lives in
dyx3_motion_guard and PX4. Contract: docs/contracts/backend.md.
"""

from __future__ import annotations

from contextlib import asynccontextmanager

import socketio
from fastapi import FastAPI

from dyx3_backend.api.parse_routes import router as parse_router
from dyx3_backend.api.routes import router
from dyx3_backend.auth.tokens import TokenStore
from dyx3_backend.config.settings import Settings
from dyx3_backend.gateway.client import GatewayClient
from dyx3_backend.mission.service import MissionService
from dyx3_backend.realtime.hub import RealtimeHub
from dyx3_backend.realtime.relay import OperatorLinkRelay
from dyx3_backend.rtk.client import RtkClient


class Combined:
    """Routes ``/socket.io`` to the Socket.IO app and everything else (including lifespan) to FastAPI."""

    def __init__(self, api: FastAPI, sio_app) -> None:
        self.api = api
        self.sio_app = sio_app

    async def __call__(self, scope, receive, send) -> None:
        if scope["type"] in ("http", "websocket") and scope.get("path", "").startswith("/socket.io"):
            await self.sio_app(scope, receive, send)
        else:
            await self.api(scope, receive, send)


def create_api(
    settings: Settings,
    *,
    tokens: TokenStore | None = None,
    gateway: GatewayClient | None = None,
    missions: MissionService | None = None,
    rtk: RtkClient | None = None,
    sio: socketio.AsyncServer | None = None,
) -> tuple[FastAPI, RealtimeHub, socketio.AsyncServer]:
    gw = gateway or GatewayClient(settings.gateway_socket, request_timeout_s=settings.request_timeout_s)
    relay = OperatorLinkRelay(gw, relay_s=settings.heartbeat_relay_s, tablet_timeout_s=settings.tablet_heartbeat_timeout_s)
    # Same-origin only until the tablet app's origin is decided (OPEN); native clients send no Origin header.
    server = sio or socketio.AsyncServer(
        async_mode="asgi",
        cors_allowed_origins=[],
        ping_interval=settings.sio_ping_interval_s,
        ping_timeout=settings.sio_ping_timeout_s,
    )
    hub = RealtimeHub(tokens or TokenStore.load(settings.auth_path), gw, relay, server.emit)

    @asynccontextmanager
    async def lifespan(_app: FastAPI):
        gw.on_telemetry(hub.broadcast_telemetry)
        gw.on_state(hub.broadcast_gateway_state)
        await gw.start()
        await relay.start()
        try:
            yield
        finally:
            await relay.stop()
            await gw.stop()

    api = FastAPI(title="DYX 3WD Backend", version="0.1.0", lifespan=lifespan)
    api.state.settings = settings
    api.state.tokens = tokens or TokenStore.load(settings.auth_path)
    api.state.gateway = gw
    api.state.relay = relay
    api.state.missions = missions or MissionService(settings)
    api.state.rtk = rtk or RtkClient(settings.rtk_socket, settings.request_timeout_s)
    api.include_router(router)
    api.include_router(parse_router)

    @server.event
    async def connect(sid, _environ, auth):
        if not hub.on_connect(sid, auth):
            raise socketio.exceptions.ConnectionRefusedError("unauthorized")

    @server.event
    async def disconnect(sid):
        hub.on_disconnect(sid)

    @server.on("heartbeat")
    async def heartbeat(sid, data=None):
        return hub.on_heartbeat(sid, data)

    @server.on("estop")
    async def estop(sid, data=None):
        return await hub.on_estop(sid, data)

    return api, hub, server


def create_app(settings: Settings | None = None, **kw) -> Combined:
    settings = settings or Settings.from_env()
    api, _hub, server = create_api(settings, **kw)
    return Combined(api, socketio.ASGIApp(server, socketio_path="socket.io"))


# `uvicorn dyx3_backend.main:app`
app = create_app()
