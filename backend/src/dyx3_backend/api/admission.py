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

import re
from collections.abc import Callable

from starlette.responses import JSONResponse

from dyx3_backend.auth.tokens import TokenStore

# The one upload route: ``/missions/plan`` is a raw JSON body, capped at exactly ``upload_max_bytes``.
RAW_UPLOAD_PATHS = frozenset({"/api/missions/plan"})


# DERIVED — NOT FROM V1 SPEC (BE-010): the deepest real JSON body (an RTK config with its profiles) is 4 levels.
# Deeper nesting is refused before FastAPI parses it: depending on the Python version a deep body either fails to
# parse (RecursionError) or parses and then breaks the validation-error response (500).
MAX_JSON_DEPTH = 32

_JSON_TOKEN = re.compile(rb'[\[\]{}"\\]')


class _Refused(Exception):
    pass


class JsonDepthScanner:
    """Incremental nesting-depth check over a JSON byte stream (strings and escapes aware). Not a validator."""

    def __init__(self, limit: int = MAX_JSON_DEPTH) -> None:
        self.limit = limit
        self.depth = 0
        self._in_str = False
        self._skip_first = False  # a backslash ended the previous chunk: the next byte is escaped

    def feed(self, data: bytes) -> bool:
        """Returns False as soon as the nesting depth exceeds the limit."""
        start = 0
        if self._skip_first and data:
            self._skip_first = False
            start = 1
        for m in _JSON_TOKEN.finditer(data, start):
            pos = m.start()
            if pos < start:
                continue  # escaped by a backslash just before it
            c = data[pos]
            if self._in_str:
                if c == 0x5C:  # backslash: skip the escaped byte
                    if pos + 1 < len(data):
                        start = pos + 2
                    else:
                        self._skip_first = True
                elif c == 0x22:
                    self._in_str = False
            elif c == 0x22:
                self._in_str = True
            elif c in (0x5B, 0x7B):
                self.depth += 1
                if self.depth > self.limit:
                    return False
            elif c in (0x5D, 0x7D):
                self.depth = max(0, self.depth - 1)
        return True


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

    @staticmethod
    def is_upload(path: str) -> bool:
        p = path.rstrip("/") or "/"
        return p in RAW_UPLOAD_PATHS

    def cap_for(self, path: str) -> int:
        p = path.rstrip("/") or "/"
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

        # 3. Streamed size (chunked bodies, or a client sending more than it declared), and the JSON nesting depth on
        #    the small JSON routes (the plan route parses its body in the planning process).
        scanner = None if self.is_upload(path) else JsonDepthScanner()
        received = 0
        refusal: tuple[int, str, str] | None = None
        started = False

        async def limited_receive():
            nonlocal received, refusal
            if refusal is not None:
                raise _Refused()
            message = await receive()
            if message["type"] == "http.request":
                chunk = message.get("body", b"")
                received += len(chunk)
                if received > cap:
                    refusal = (413, "too_large", f"request body exceeds {cap} bytes")
                    raise _Refused()
                if scanner is not None and not scanner.feed(chunk):
                    refusal = (400, "bad_request", f"JSON nested deeper than {scanner.limit} levels")
                    raise _Refused()
            return message

        async def guarded_send(message) -> None:
            nonlocal started
            if refusal is not None and not started:
                return  # whatever the app made of the truncated body is replaced by the refusal below
            if message["type"] == "http.response.start":
                started = True
            await send(message)

        try:
            await self.app(scope, limited_receive, guarded_send)
        except Exception:
            if refusal is None:
                raise
        if refusal is not None and not started:
            await _reject(scope, receive, send, *refusal)


async def _reject(scope, receive, send, status: int, code: str, reason: str) -> None:
    await JSONResponse({"ok": False, "code": code, "reason": reason}, status_code=status)(scope, receive, send)
