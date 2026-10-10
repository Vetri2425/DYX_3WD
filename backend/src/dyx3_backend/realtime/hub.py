"""Socket.IO event logic, independent of the transport (so it can be tested with a fake emitter).

Contract: docs/contracts/backend.md section 4. The hub holds no safety logic: ``estop`` is forwarded to the gateway as a request
and the verdict is returned in the ack.

Status reaches the tablet as ONE Socket.IO event, ``rover_event``, pushed the moment it changes: every gateway status event
(gateway contract section 1.3: ``mission_state``, ``operator_link``, ``fcu_link``, ``estop``) and the backend's own link to the
gateway (``gateway_link``). Each carries a hub ``seq`` (monotonic per backend process). A session gets the latest event of every
kind as soon as it connects (``replay`` true, original ``seq``), so the tablet never polls; per kind, the highest ``seq`` is the
current state.
"""

from __future__ import annotations

import asyncio
import logging
import time
from collections.abc import Awaitable, Callable
from typing import Any

from dyx3_backend.auth.tokens import Identity, Role, TokenStore
from dyx3_backend.gateway.client import GatewayClient, GatewayError
from dyx3_backend.realtime.relay import OperatorLinkRelay

log = logging.getLogger("dyx3.hub")
# emit(event, data, to=None): socketio.AsyncServer.emit; ``to`` None = every connected session.
Emit = Callable[..., Awaitable[None]]

STATUS_EVENT = "rover_event"
GATEWAY_LINK = "gateway_link"


def _wall_ms() -> int:
    return time.time_ns() // 1_000_000


class RealtimeHub:
    def __init__(self, tokens: TokenStore, gateway: GatewayClient, relay: OperatorLinkRelay, emit: Emit) -> None:
        self._tokens = tokens
        self._gw = gateway
        self._relay = relay
        self._emit = emit
        self._sessions: dict[str, Identity] = {}
        self._seq = 0
        self._latest: dict[str, dict] = {}  # kind -> the last rover_event payload of that kind
        self._tasks: set[asyncio.Task] = set()
        # Until the gateway client reports a connection, the gateway link is down.
        self._record(GATEWAY_LINK, {"connected": False}, wall_ms=_wall_ms())
        gateway.on_event(self.broadcast_event)

    # ---- connection lifecycle
    def on_connect(self, sid: str, auth: Any) -> bool:
        token = auth.get("token") if isinstance(auth, dict) else None
        ident = self._tokens.verify(token if isinstance(token, str) else None)
        if ident is None:
            return False  # refused: unknown or missing token
        self._sessions[sid] = ident
        # The current state, right after the connection is accepted (the handler returns first).
        task = asyncio.get_running_loop().create_task(self.send_current_state(sid))
        self._tasks.add(task)
        task.add_done_callback(self._tasks.discard)
        return True

    def on_disconnect(self, sid: str) -> None:
        ident = self._sessions.pop(sid, None)
        # The tablet is gone when the last OPERATOR session closes: do not wait for the heartbeat timeout.
        if ident is not None and not any(i.role >= Role.OPERATOR for i in self._sessions.values()):
            self._relay.clear()

    def sessions(self) -> int:
        return len(self._sessions)

    # ---- events
    def on_heartbeat(self, sid: str, _data: Any = None) -> dict:
        ident = self._sessions.get(sid)
        if ident is None or not ident.can(Role.OPERATOR):
            return {"ok": False, "code": "forbidden"}
        self._relay.note_tablet()
        return {"ok": True}

    async def on_estop(self, sid: str, data: Any) -> dict:
        ident = self._sessions.get(sid)
        if ident is None:
            return {"ok": False, "code": "unauthenticated", "delivered": False}
        if not isinstance(data, dict) or not isinstance(data.get("asserted"), bool):
            return {"ok": False, "code": "invalid_command", "delivered": False}
        asserted = data["asserted"]
        if not asserted and not ident.can(Role.OPERATOR):
            return {"ok": False, "code": "forbidden", "delivered": False}  # anyone may stop; only an operator may clear
        try:
            reply = await self._gw.request("estop", {"asserted": asserted, "source": "tablet"})
        except GatewayError as exc:
            return {"ok": False, "code": type(exc).__name__, "reason": str(exc), "delivered": exc.delivered}
        return {"ok": bool(reply.get("ok")), "code": reply.get("code"), "reason": reply.get("reason"), "data": reply.get("data"),
                "delivered": True}

    # ---- fan-out
    async def broadcast_telemetry(
        self, snapshot: dict | None, *, seq: int | None = None, t_mono_s: float | None = None, dropped: int = 0
    ) -> None:
        """``seq`` / ``t_mono_s`` are the gateway's (null when its frame has none); ``dropped`` = frames coalesced before this one."""
        await self._emit(
            "telemetry",
            {"snapshot": snapshot, "age_s": self._gw.snapshot_age(), "seq": seq, "t_mono_s": t_mono_s, "dropped": dropped},
        )

    async def broadcast_telemetry_frame(self, frame: dict) -> None:
        await self.broadcast_telemetry(
            frame.get("snapshot"), seq=frame.get("seq"), t_mono_s=frame.get("t_mono_s"), dropped=frame.get("dropped", 0)
        )

    def _record(self, kind: str, data: dict, *, gateway_event: dict | None = None, wall_ms: int | None = None) -> dict:
        self._seq += 1
        ev = gateway_event or {}
        payload = {
            "kind": kind,
            "seq": self._seq,
            "gateway_seq": ev.get("seq"),
            "t_mono_s": ev.get("t_mono_s"),
            "t_wall_ms": ev.get("t_wall_ms", wall_ms),
            "coalesced": ev.get("coalesced", 0),
            "replay": bool(ev.get("replay", False)),
            "data": data,
        }
        self._latest[kind] = payload
        return payload

    async def broadcast_event(self, event: dict) -> None:
        """A gateway status event (already ordered and de-duplicated by the gateway client): relay it at once."""
        payload = self._record(event["event"], event["data"], gateway_event=event)
        await self._emit(STATUS_EVENT, payload)

    async def broadcast_gateway_state(self, connected: bool) -> None:
        """The backend's own link to the gateway. While it is down every gateway-sourced kind is unknown."""
        await self._emit(STATUS_EVENT, self._record(GATEWAY_LINK, {"connected": connected}, wall_ms=_wall_ms()))

    async def send_current_state(self, sid: str) -> None:
        """The latest event of every kind, in seq order, marked replay, to one session."""
        for payload in sorted(self._latest.values(), key=lambda p: p["seq"]):
            if sid not in self._sessions:
                return  # gone meanwhile
            try:
                await self._emit(STATUS_EVENT, {**payload, "replay": True}, to=sid)
            except Exception:  # one session's transport must never break the hub
                log.exception("sending the current state to %s failed", sid)
                return
