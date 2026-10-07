"""Shared fakes for the backend tests."""

from __future__ import annotations

from dyx3_backend.auth.tokens import Role, TokenStore, hash_token
from dyx3_backend.gateway.client import GatewayUnavailable

TOKENS = {"view-tok": Role.VIEWER, "oper-tok": Role.OPERATOR}


def token_store() -> TokenStore:
    return TokenStore([(f"user-{t}", role, hash_token(t)) for t, role in TOKENS.items()])


def H(token: str) -> dict:
    return {"Authorization": f"Bearer {token}"}


class FakeGateway:
    """Stands in for GatewayClient: records requests, returns scripted replies."""

    def __init__(self) -> None:
        self.connected = True
        self.calls: list[tuple[str, dict]] = []
        self.replies: dict[str, dict] = {}
        self.raises: Exception | None = None
        self.snapshot: dict | None = None
        self._age: float | None = None

    def snapshot_age(self):
        return self._age

    async def request(self, cmd: str, args: dict | None = None) -> dict:
        if not self.connected:
            raise GatewayUnavailable("gateway not connected; command NOT delivered")
        self.calls.append((cmd, args or {}))
        if self.raises is not None:
            raise self.raises
        return self.replies.get(cmd, {"v": 1, "ok": True, "code": "ok", "reason": "", "data": {"accepted": True, "reason_code": 0}})
