"""Tablet heartbeat -> gateway heartbeat relay. Contract: docs/contracts/backend.md section 3.

The relay forwards a gateway ``heartbeat`` only while the TABLET heartbeat is fresh, so a tablet dropout propagates to the gateway's
operator-link timeout: ``dyx3_motion_guard`` then refuses a new start (pre-arm gate); a running mission continues (owner decision
2026-10-10). The numbers are DERIVED (OPEN).
"""

from __future__ import annotations

import asyncio
import contextlib
import logging
import time
from collections.abc import Callable

from dyx3_backend.gateway.client import GatewayClient, GatewayError

log = logging.getLogger("dyx3.relay")

# DERIVED — NOT FROM V1 SPEC (XR-BE-002): a relay tick later than relay_s + this margin means the event loop was
# blocked (or the gateway answered slowly); heartbeats processed right after it cannot be trusted as fresh.
STALL_MARGIN_S = 0.3


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
        self._last_ack: float | None = None  # last heartbeat the gateway acknowledged with ok
        self._task: asyncio.Task | None = None
        self._failures = 0  # consecutive ticks that raised an unexpected exception
        # Stall guard (XR-BE-002): a heartbeat is stamped when it is PROCESSED. After a loop stall the queued ones
        # are processed at once and would look fresh, extending the link by the stall time.
        self._prev_step: float | None = None  # clock at the start of the previous loop iteration
        self._trusted: float | None = None  # the tablet stamp as it was at the previous iteration
        self._quarantine_until: float | None = None
        self.stalls = 0

    def note_tablet(self) -> bool:
        """Stamp a tablet heartbeat. Returns False when it is ignored because the relay just saw a loop stall."""
        now = self._clock()
        if self._quarantine_until is not None:
            if now < self._quarantine_until:
                return False  # possibly a backlog heartbeat from before the stall: a live tablet re-proves itself
            self._quarantine_until = None
        self._last = now
        return True

    def clear(self) -> None:
        """The tablet's last connection closed: stop relaying at once instead of waiting for the timeout."""
        self._last = None
        self._trusted = None

    def tablet_age(self) -> float | None:
        return None if self._last is None else self._clock() - self._last

    def tablet_alive(self) -> bool:
        age = self.tablet_age()
        return age is not None and age <= self._timeout

    @property
    def running(self) -> bool:
        """The relay task exists and has not ended (it should only end at shutdown)."""
        return self._task is not None and not self._task.done()

    def operator_alive(self) -> bool:
        """The backend's view of the operator link: the relay runs, the tablet is fresh, and the gateway
        acknowledged a relayed heartbeat within the tablet timeout. The gateway's own timer stays authoritative."""
        if not (self.running and self.tablet_alive()) or self._last_ack is None:
            return False
        return self._clock() - self._last_ack <= self._timeout

    async def tick(self) -> bool:
        """Relay one heartbeat if the tablet is alive. Returns True when one was sent and acknowledged."""
        if not self.tablet_alive():
            return False
        try:
            reply = await self._gw.request("heartbeat")
        except GatewayError:
            return False
        ok = bool(reply.get("ok"))
        if ok:
            self._last_ack = self._clock()
        return ok

    def _check_stall(self) -> None:
        """Called at the start of each loop iteration. If this iteration is late by more than STALL_MARGIN_S, every
        heartbeat stamped since the previous iteration may be a backlog replayed after the stall: fall back to the
        stamp held at the previous iteration and ignore heartbeats stamped within the next relay_s."""
        now = self._clock()
        prev, self._prev_step = self._prev_step, now
        if prev is not None and now - prev > self._relay_s + STALL_MARGIN_S:
            self.stalls += 1
            log.warning("operator-link relay tick %.2f s late (event-loop stall or slow gateway): heartbeats since "
                        "the previous tick are not trusted, new ones ignored for %.2f s",
                        now - prev - self._relay_s, self._relay_s)
            self._last = self._trusted  # stamps only grow (clear() resets both), so this drops the suspect ones
            self._quarantine_until = now + self._relay_s
        self._trusted = self._last

    async def step(self) -> None:
        """One loop iteration. Never raises (except cancellation): an unexpected error is logged and the next tick
        runs as usual. A tick that fails relays nothing, so a persistent fault still ends in the gateway's STOP."""
        try:
            self._check_stall()
            await self.tick()
        except Exception:
            self._failures += 1
            if self._failures == 1 or self._failures % 20 == 0:
                log.exception("operator-link relay tick failed (%d in a row); the relay keeps running", self._failures)
            return
        if self._failures:
            log.warning("operator-link relay recovered after %d failed ticks", self._failures)
            self._failures = 0

    async def _run(self) -> None:
        while True:
            await self.step()
            await asyncio.sleep(self._relay_s)

    async def start(self) -> None:
        self._task = asyncio.create_task(self._run(), name="operator-link-relay")

    async def stop(self) -> None:
        if self._task is not None:
            self._task.cancel()
            with contextlib.suppress(asyncio.CancelledError, Exception):
                await self._task
