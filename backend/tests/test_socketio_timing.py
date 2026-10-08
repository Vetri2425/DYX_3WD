"""Socket.IO keepalive configuration and silent-client cleanup."""

import asyncio
import time

import pytest
from backend_helpers import FakeGateway, token_store
from engineio.async_socket import AsyncSocket

from dyx3_backend.config.settings import Settings
from dyx3_backend.main import create_api


def test_socketio_ping_settings_parse_and_reject_invalid_values():
    defaults = Settings.from_env({})
    assert (defaults.sio_ping_interval_s, defaults.sio_ping_timeout_s) == (5.0, 5.0)
    configured = Settings.from_env({"DYX3_SIO_PING_INTERVAL_S": "2.5", "DYX3_SIO_PING_TIMEOUT_S": "3.25"})
    assert (configured.sio_ping_interval_s, configured.sio_ping_timeout_s) == (2.5, 3.25)
    for key in ("DYX3_SIO_PING_INTERVAL_S", "DYX3_SIO_PING_TIMEOUT_S"):
        for value in ("0", "-1", "nan", "inf"):
            with pytest.raises(ValueError, match=key):
                Settings.from_env({key: value})


def test_socketio_server_uses_configured_intervals(tmp_path):
    settings = Settings(data_dir=str(tmp_path), sio_ping_interval_s=2.5, sio_ping_timeout_s=3.25)
    _, _, server = create_api(settings, tokens=token_store(), gateway=FakeGateway())
    assert server.eio.ping_interval == 2.5
    assert server.eio.ping_timeout == 3.25


@pytest.mark.anyio
async def test_silent_socket_is_closed_after_ping_deadline(tmp_path):
    interval, timeout = 0.04, 0.04
    settings = Settings(data_dir=str(tmp_path), sio_ping_interval_s=interval, sio_ping_timeout_s=timeout)
    _, _, server = create_api(settings, tokens=token_store(), gateway=FakeGateway())
    socket = AsyncSocket(server.eio, "silent-tablet")
    server.eio.sockets[socket.sid] = socket
    socket.schedule_ping()
    start = time.monotonic()
    try:
        # The client never sends PONG. The Engine.IO timeout check closes it once
        # ping_interval + ping_timeout has elapsed (allow scheduler jitter).
        await asyncio.sleep(interval + timeout + 0.015)
        assert await socket.check_ping_timeout() is False
        assert socket.closed
        assert time.monotonic() - start < interval + timeout + 0.1
    finally:
        server.eio.sockets.pop(socket.sid, None)
