"""Asyncio client for the gateway's newline-delimited-JSON Unix socket.

Contract: docs/contracts/dyx3_system_gateway.md section 1 and docs/contracts/backend.md. A command is sent once: if the socket is
down it fails at once with ``GatewayUnavailable`` (``delivered`` is False), it is never queued for later. A request whose reply
does not arrive in time raises ``GatewayTimeout`` (delivery unknown).

Status events (gateway contract section 1.3) are handed to the ``on_event`` callbacks in arrival order, one at a time, from a
bounded queue: a slow subscriber can never block the socket reader (replies keep flowing) and never grows memory without bound.
On every (re)connect the gateway replays the latest event of each kind, so the subscribers get the current state at once.
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


# Events waiting for the subscribers. Past it the OLDEST is dropped (and counted): a newer event of the same kind supersedes it,
# so the latest state of every kind still gets through.
EVENT_QUEUE_MAX = 256

TelemetryCb = Callable[[dict], Awaitable[None] | None]
StateCb = Callable[[bool], Awaitable[None] | None]
EventCb = Callable[[dict], Awaitable[None] | None]


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
        self._on_event: list[EventCb] = []
        self._events: asyncio.Queue[dict] = asyncio.Queue(maxsize=EVENT_QUEUE_MAX)
        self._event_task: asyncio.Task | None = None
        self._event_seq: dict[str, int] = {}  # highest seq per kind on the current connection
        self.events_dropped = 0
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

    def on_event(self, cb: EventCb) -> None:
        """``cb(event)`` for every new gateway status event (the NDJSON object), in order."""
        self._on_event.append(cb)

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
        self._event_task = asyncio.create_task(self._pump_events(), name="gateway-events")
        self._task = asyncio.create_task(self._run(), name="gateway-client")

    async def stop(self) -> None:
        self._stop = True
        if self._writer is not None:
            self._writer.close()
        for task in (self._task, self._event_task):
            if task is not None:
                task.cancel()
                with contextlib.suppress(asyncio.CancelledError, Exception):
                    await task
        self._fail_pending(GatewayUnavailable("gateway client stopped"))

    async def _pump_events(self) -> None:
        while True:
            event = await self._events.get()
            await self._fire(self._on_event, event)

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
            # The backoff resets only once the gateway has sent a line (XR-GW-002): a peer that accepts and closes at
            # once (the gateway at max_clients) must not make this client reconnect at 1/reconnect_min_s.
            heard = False
            self._writer = writer
            self._event_seq = {}  # a new connection: the gateway's replay re-establishes every kind
            await self._fire(self._on_state, True)
            try:
                while True:
                    line = await reader.readline()
                    if not line:
                        break
                    if not heard:
                        heard = True
                        delay = self._rmin
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
                if not heard:
                    delay = min(self._rmax, delay * 2)

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
        if msg.get("type") == "event":
            self._handle_event(msg)
            return
        rid = msg.get("id")
        fut = self._pending.pop(rid, None) if isinstance(rid, int) else None
        if fut is not None and not fut.done():
            fut.set_result(msg)

    def _handle_event(self, msg: dict) -> None:
        kind, seq = msg.get("event"), msg.get("seq")
        if not isinstance(kind, str) or not isinstance(seq, int) or isinstance(seq, bool) or not isinstance(msg.get("data"), dict):
            log.warning("gateway sent a malformed event; ignored")
            return
        # A push can race the gateway's connect replay: per kind, the highest seq is the current one, anything else is a duplicate.
        if seq <= self._event_seq.get(kind, 0):
            return
        self._event_seq[kind] = seq
        if self._events.full():
            self._events.get_nowait()
            self.events_dropped += 1
            log.warning("gateway event subscribers are behind: oldest event dropped (%d in total)", self.events_dropped)
        self._events.put_nowait(msg)

    # --------------------------------------------------------------- requests
    async def request(self, cmd: str, args: dict | None = None, *, timeout_s: float | None = None) -> dict:
        """Send one command and await its typed reply. ``timeout_s`` overrides ``request_timeout_s`` for this call."""
        budget = self._timeout if timeout_s is None else timeout_s
        if not self.connected:
            raise GatewayUnavailable("gateway not connected; command NOT delivered")
        writer = self._writer
        assert writer is not None
        rid = self._next_id
        self._next_id += 1
        fut: asyncio.Future = asyncio.get_running_loop().create_future()
        self._pending[rid] = fut
        line = json.dumps({"v": PROTOCOL_VERSION, "id": rid, "cmd": cmd, "args": args or {}}, separators=(",", ":"))

        async def send_and_wait() -> dict:
            try:
                writer.write(line.encode("utf-8") + b"\n")
                await writer.drain()
            except (OSError, ConnectionError) as exc:
                raise GatewayUnavailable(f"gateway write failed: {exc}") from exc
            return await fut

        # One budget for the write (drain() blocks while the gateway does not read) and the reply (BE-006).
        try:
            return await asyncio.wait_for(send_and_wait(), budget)
        except asyncio.TimeoutError as exc:
            raise GatewayTimeout(f"no reply to {cmd} within {budget:.1f}s") from exc
        finally:
            self._pending.pop(rid, None)
