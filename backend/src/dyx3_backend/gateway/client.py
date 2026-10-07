"""Asyncio client for the gateway's newline-delimited-JSON Unix socket.

Contract: docs/contracts/dyx3_system_gateway.md section 1 and docs/contracts/backend.md. A command is sent once: if the socket is
down it fails at once with ``GatewayUnavailable`` (``delivered`` is False), it is never queued for later. A request whose reply
does not arrive in time raises ``GatewayTimeout`` (delivery unknown).
"""

from __future__ import annotations

import asyncio
import contextlib
import json
import logging
import time
from collections.abc import Awaitable, Callable
from typing import Any

log = logging.getLogger("dyx3.gateway")

PROTOCOL_VERSION = 1
MAX_LINE = 4 * 1024 * 1024  # a snapshot is a few KiB; this only bounds a misbehaving peer


class GatewayError(RuntimeError):
    delivered: bool | None = None


class GatewayUnavailable(GatewayError):
    """Not connected: the command was NOT delivered."""

    delivered = False


class GatewayTimeout(GatewayError):
    """Sent but no reply in time: whether it was executed is unknown."""

    delivered = None


TelemetryCb = Callable[[dict], Awaitable[None] | None]
StateCb = Callable[[bool], Awaitable[None] | None]


class GatewayClient:
    def __init__(
        self,
        path: str,
        *,
        request_timeout_s: float = 3.0,
        reconnect_min_s: float = 0.2,
        reconnect_max_s: float = 3.0,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self._path = path
        self._timeout = request_timeout_s
        self._rmin = reconnect_min_s
        self._rmax = reconnect_max_s
        self._clock = clock
        self._writer: asyncio.StreamWriter | None = None
        self._task: asyncio.Task | None = None
        self._pending: dict[int, asyncio.Future] = {}
        self._next_id = 1
        self._stop = False
        self._on_telemetry: list[TelemetryCb] = []
        self._on_state: list[StateCb] = []
        self.snapshot: dict | None = None
        self.snapshot_stamp: float | None = None

    # ------------------------------------------------------------------ state
    @property
    def connected(self) -> bool:
        return self._writer is not None and not self._writer.is_closing()

    def snapshot_age(self) -> float | None:
        return None if self.snapshot_stamp is None else self._clock() - self.snapshot_stamp

    def on_telemetry(self, cb: TelemetryCb) -> None:
        self._on_telemetry.append(cb)

    def on_state(self, cb: StateCb) -> None:
        self._on_state.append(cb)

    async def _fire(self, cbs: list, arg: Any) -> None:
        for cb in cbs:
            try:
                r = cb(arg)
                if asyncio.iscoroutine(r):
                    await r
            except Exception:  # a subscriber must never break the link
                log.exception("gateway callback failed")

    # -------------------------------------------------------------- lifecycle
    async def start(self) -> None:
        self._stop = False
        self._task = asyncio.create_task(self._run(), name="gateway-client")

    async def stop(self) -> None:
        self._stop = True
        if self._writer is not None:
            self._writer.close()
        if self._task is not None:
            self._task.cancel()
            with contextlib.suppress(asyncio.CancelledError, Exception):
                await self._task
        self._fail_pending(GatewayUnavailable("gateway client stopped"))

    def _fail_pending(self, exc: Exception) -> None:
        for fut in self._pending.values():
            if not fut.done():
                fut.set_exception(exc)
        self._pending.clear()

    async def _run(self) -> None:
        delay = self._rmin
        while not self._stop:
            try:
                reader, writer = await asyncio.open_unix_connection(self._path, limit=MAX_LINE)
            except (OSError, asyncio.TimeoutError):
                await asyncio.sleep(delay)
                delay = min(self._rmax, delay * 2)
                continue
            delay = self._rmin
            self._writer = writer
            await self._fire(self._on_state, True)
            try:
                while True:
                    line = await reader.readline()
                    if not line:
                        break
                    self._handle(line)
            except (OSError, asyncio.LimitOverrunError, ValueError):
                log.warning("gateway link error", exc_info=True)
            finally:
                self._writer = None
                writer.close()
                self._fail_pending(GatewayUnavailable("gateway connection lost"))
                await self._fire(self._on_state, False)
            if not self._stop:
                await asyncio.sleep(delay)

    def _handle(self, line: bytes) -> None:
        try:
            msg = json.loads(line)
        except ValueError:
            log.warning("gateway sent invalid JSON")
            return
        if not isinstance(msg, dict) or msg.get("v") != PROTOCOL_VERSION:
            return
        if msg.get("type") == "telemetry":
            self.snapshot = msg.get("snapshot")
            self.snapshot_stamp = self._clock()
            asyncio.ensure_future(self._fire(self._on_telemetry, msg.get("snapshot")))
            return
        rid = msg.get("id")
        fut = self._pending.pop(rid, None) if isinstance(rid, int) else None
        if fut is not None and not fut.done():
            fut.set_result(msg)

    # --------------------------------------------------------------- requests
    async def request(self, cmd: str, args: dict | None = None) -> dict:
        if not self.connected:
            raise GatewayUnavailable("gateway not connected; command NOT delivered")
        rid = self._next_id
        self._next_id += 1
        fut: asyncio.Future = asyncio.get_running_loop().create_future()
        self._pending[rid] = fut
        line = json.dumps({"v": PROTOCOL_VERSION, "id": rid, "cmd": cmd, "args": args or {}}, separators=(",", ":"))
        try:
            assert self._writer is not None
            self._writer.write(line.encode("utf-8") + b"\n")
            await self._writer.drain()
        except (OSError, ConnectionError) as exc:
            self._pending.pop(rid, None)
            raise GatewayUnavailable(f"gateway write failed: {exc}") from exc
        try:
            return await asyncio.wait_for(fut, self._timeout)
        except asyncio.TimeoutError as exc:
            self._pending.pop(rid, None)
            raise GatewayTimeout(f"no reply to {cmd} within {self._timeout:.1f}s") from exc
