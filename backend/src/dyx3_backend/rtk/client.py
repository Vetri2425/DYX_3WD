"""Bounded client for the independent RTK worker's versioned local control socket."""

from __future__ import annotations

import asyncio
import json


class RtkUnavailable(Exception):
    """The worker did not answer; the backend must report 503, never cache success."""


class RtkRejected(Exception):
    def __init__(self, code: str, reason: str) -> None:
        self.code = code
        self.reason = reason
        super().__init__(reason)


class RtkClient:
    def __init__(self, socket_path: str = "/run/dyx3/rtk-control.sock", timeout_s: float = 3.0) -> None:
        self.socket_path = socket_path
        self.timeout_s = timeout_s

    async def request(self, command: str, **arguments) -> dict:
        message = json.dumps({"v": 1, "cmd": command, **arguments}, separators=(",", ":")) + "\n"
        if len(message.encode()) > 65536:
            raise RtkRejected("invalid_config", "RTK request is too large")
        conn: list[asyncio.StreamWriter] = []

        async def exchange() -> bytes:
            reader, writer = await asyncio.open_unix_connection(self.socket_path)
            conn.append(writer)
            writer.write(message.encode())
            await writer.drain()
            return await reader.readline()

        try:
            # asyncio.wait_for, not asyncio.timeout: the rover runs Python 3.10.
            response = await asyncio.wait_for(exchange(), self.timeout_s)
            if len(response) > 65536 or not response.endswith(b"\n"):
                raise RtkUnavailable("invalid RTK worker reply")
            reply = json.loads(response)
        except (OSError, TimeoutError, asyncio.TimeoutError, json.JSONDecodeError, ValueError) as exc:
            raise RtkUnavailable("RTK worker unavailable") from exc
        finally:
            for writer in conn:
                writer.close()
                try:
                    await writer.wait_closed()
                except OSError:
                    pass
        if not isinstance(reply, dict) or reply.get("v") != 1 or not isinstance(reply.get("ok"), bool):
            raise RtkUnavailable("invalid RTK worker reply")
        if not reply["ok"]:
            # The worker emits fixed, secret-free reasons; still cap the text before HTTP output.
            code = str(reply.get("code", "rejected"))[:64]
            reason = str(reply.get("reason", "RTK request rejected"))[:256]
            raise RtkRejected(code, reason)
        return reply.get("data", {})
