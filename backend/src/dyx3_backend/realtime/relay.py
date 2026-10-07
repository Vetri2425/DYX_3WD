"""Tablet heartbeat -> gateway heartbeat relay. Contract: docs/contracts/backend.md section 3.

The relay forwards a gateway ``heartbeat`` only while the TABLET heartbeat is fresh, so a tablet dropout propagates to the gateway's
operator-link timeout and ``dyx3_motion_guard`` stops the rover. The numbers are DERIVED (OPEN).
"""

from __future__ import annotations

import asyncio
import contextlib
import logging
import time
from collections.abc import Callable

from dyx3_backend.gateway.client import GatewayClient, GatewayError

log = logging.getLogger("dyx3.relay")


class OperatorLinkRelay:
    def __init__(
        self,
        gateway: GatewayClient,
        *,
        relay_s: float,
        tablet_timeout_s: float,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self._gw = gateway
        self._relay_s = relay_s
        self._timeout = tablet_timeout_s
        self._clock = clock
        self._last: float | None = None
        self._task: asyncio.Task | None = None

    def note_tablet(self) -> None:
        self._last = self._clock()

    def clear(self) -> None:
        """The tablet's last connection closed: stop relaying at once instead of waiting for the timeout."""
        self._last = None

    def tablet_age(self) -> float | None:
        return None if self._last is None else self._clock() - self._last

    def tablet_alive(self) -> bool:
        age = self.tablet_age()
        return age is not None and age <= self._timeout

    async def tick(self) -> bool:
        """Relay one heartbeat if the tablet is alive. Returns True when one was sent and acknowledged."""
        if not self.tablet_alive():
            return False
        try:
            reply = await self._gw.request("heartbeat")
            return bool(reply.get("ok"))
        except GatewayError:
            return False

    async def _run(self) -> None:
        while True:
            await self.tick()
            await asyncio.sleep(self._relay_s)

    async def start(self) -> None:
        self._task = asyncio.create_task(self._run(), name="operator-link-relay")

    async def stop(self) -> None:
        if self._task is not None:
            self._task.cancel()
            with contextlib.suppress(asyncio.CancelledError, Exception):
                await self._task
