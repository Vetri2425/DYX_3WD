"""Request admission for ``/api``: bearer check and body-size cap BEFORE any body byte is read.

Contract: docs/contracts/backend.md section 1 ("Admission").

FastAPI reads and parses a request body before it resolves the route's auth dependency, so without this layer
an unauthenticated client could make the backend buffer an unbounded body (XR-BE-001, BE-002, BE-003). This
pure-ASGI middleware wraps the FastAPI app only (``/socket.io`` never reaches it) and, for every HTTP request
under ``/api`` except ``GET /api/ping``:

* answers 401 before reading the body when the bearer token is missing or unknown (same ``TokenStore``
  verification as the routes; the role checks stay in the routes);
* answers 413 when ``Content-Length`` exceeds the route's cap, and stops reading (413) once the streamed byte
  count exceeds it, so a chunked body is bounded too.

The per-route checks stay in place behind it.
"""

from __future__ import annotations

from collections.abc import Callable

from starlette.responses import JSONResponse

from dyx3_backend.auth.tokens import TokenStore

# Upload routes: their cap is ``upload_max_bytes``. The multipart routes get MULTIPART_ENVELOPE_BYTES on top,
# because ``upload_max_bytes`` bounds the FILE (the routes check it exactly) and the multipart envelope (boundaries,
# part headers, the small form fields) comes on top of it. ``/missions/plan`` is a raw JSON body: exact limit.
MULTIPART_UPLOAD_PATHS = frozenset({"/api/missions", "/api/path/parse-dxf"})
RAW_UPLOAD_PATHS = frozenset({"/api/missions/plan"})
# DERIVED — NOT FROM V1 SPEC: the envelope of a multipart DXF upload is a few hundred bytes.
MULTIPART_ENVELOPE_BYTES = 64 * 1024


class _BodyTooLarge(Exception):
    pass


def _header(scope, name: bytes) -> str | None:
    for key, value in scope.get("headers") or ():
        if key.lower() == name:
            return value.decode("latin-1")
    return None


def bearer_identity(tokens: TokenStore, authorization: str | None):
    """The same parsing as ``api.routes._identity``: ``Bearer <token>``, case-insensitive scheme."""
    if authorization is None or not authorization.lower().startswith("bearer "):
        return None, "missing bearer token"
    ident = tokens.verify(authorization[7:].strip())
    return ident, (None if ident is not None else "invalid token")


class AdmissionMiddleware:
    def __init__(
        self,
        app,
        *,
        tokens: Callable[[], TokenStore],
        upload_max_bytes: int,
        json_max_bytes: int,
    ) -> None:
        self.app = app
        self._tokens = tokens
        self._upload_max = int(upload_max_bytes)
        self._json_max = int(json_max_bytes)

    def cap_for(self, path: str) -> int:
        p = path.rstrip("/") or "/"
        if p in MULTIPART_UPLOAD_PATHS:
            return self._upload_max + MULTIPART_ENVELOPE_BYTES
        if p in RAW_UPLOAD_PATHS:
            return self._upload_max
        return self._json_max

    async def __call__(self, scope, receive, send) -> None:
        if scope["type"] != "http":
            await self.app(scope, receive, send)
            return
        path = scope.get("path", "")
        if not (path == "/api" or path.startswith("/api/")):
            await self.app(scope, receive, send)
            return
        if scope.get("method") == "GET" and path == "/api/ping":
            await self.app(scope, receive, send)
            return

        # 1. Authentication, before a single body byte is read.
        ident, why = bearer_identity(self._tokens(), _header(scope, b"authorization"))
        if ident is None:
            await JSONResponse({"detail": why}, status_code=401, headers={"WWW-Authenticate": "Bearer"})(scope, receive, send)
            return

        # 2. Declared size.
        cap = self.cap_for(path)
        declared = _header(scope, b"content-length")
        if declared is not None:
            declared = declared.strip()
            if not declared.isdecimal():
                await _reject(scope, receive, send, 400, "bad_request", "invalid Content-Length")
                return
            if int(declared) > cap:
                await _reject(scope, receive, send, 413, "too_large", f"request body exceeds {cap} bytes")
                return

        # 3. Streamed size (chunked bodies, or a client sending more than it declared).
        received = 0
        exceeded = False
        started = False

        async def limited_receive():
            nonlocal received, exceeded
            if exceeded:
                raise _BodyTooLarge()
            message = await receive()
            if message["type"] == "http.request":
                received += len(message.get("body", b""))
                if received > cap:
                    exceeded = True
                    raise _BodyTooLarge()
            return message

        async def guarded_send(message) -> None:
            nonlocal started
            if exceeded and not started:
                return  # whatever the app made of the truncated body is replaced by the 413 below
            if message["type"] == "http.response.start":
                started = True
            await send(message)

        try:
            await self.app(scope, limited_receive, guarded_send)
        except Exception:
            if not exceeded:
                raise
        if exceeded and not started:
            await _reject(scope, receive, send, 413, "too_large", f"request body exceeds {cap} bytes")


async def _reject(scope, receive, send, status: int, code: str, reason: str) -> None:
    await JSONResponse({"ok": False, "code": code, "reason": reason}, status_code=status)(scope, receive, send)
