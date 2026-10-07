"""Socket.IO event logic, independent of the transport (so it can be tested with a fake emitter).

Contract: docs/contracts/backend.md section 4. The hub holds no safety logic: ``estop`` is forwarded to the gateway as a request
and the verdict is returned in the ack.
"""

from __future__ import annotations

import logging
from collections.abc import Awaitable, Callable
from typing import Any

from dyx3_backend.auth.tokens import Identity, Role, TokenStore
from dyx3_backend.gateway.client import GatewayClient, GatewayError
from dyx3_backend.realtime.relay import OperatorLinkRelay

log = logging.getLogger("dyx3.hub")
Emit = Callable[[str, Any], Awaitable[None]]


class RealtimeHub:
    def __init__(self, tokens: TokenStore, gateway: GatewayClient, relay: OperatorLinkRelay, emit: Emit) -> None:
        self._tokens = tokens
        self._gw = gateway
        self._relay = relay
        self._emit = emit
        self._sessions: dict[str, Identity] = {}

    # ---- connection lifecycle
    def on_connect(self, sid: str, auth: Any) -> bool:
        token = auth.get("token") if isinstance(auth, dict) else None
        ident = self._tokens.verify(token if isinstance(token, str) else None)
        if ident is None:
            return False  # refused: unknown or missing token
        self._sessions[sid] = ident
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
    async def broadcast_telemetry(self, snapshot: dict | None) -> None:
        await self._emit("telemetry", {"snapshot": snapshot, "age_s": self._gw.snapshot_age()})

    async def broadcast_gateway_state(self, connected: bool) -> None:
        await self._emit("gateway", {"connected": connected})
