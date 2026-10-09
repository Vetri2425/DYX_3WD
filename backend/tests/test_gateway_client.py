"""GatewayClient against a fake gateway speaking the NDJSON protocol over a real Unix socket."""

import asyncio
import json
import os
import tempfile

import pytest

from dyx3_backend.gateway.client import GatewayClient, GatewayTimeout, GatewayUnavailable

pytestmark = pytest.mark.anyio


class FakeServer:
    def __init__(self, path):
        self.path = path
        self.server = None
        self.writers = []
        self.received = []
        self.mode = "echo"  # echo | silent | drop

    async def start(self):
        self.server = await asyncio.start_unix_server(self._handle, path=self.path)

    async def _handle(self, reader, writer):
        self.writers.append(writer)
        try:
            while True:
                line = await reader.readline()
                if not line:
                    break
                msg = json.loads(line)
                self.received.append(msg)
                if self.mode == "silent":
                    continue
                if self.mode == "drop":
                    writer.close()
                    return
                reply = {"v": 1, "id": msg["id"], "ok": True, "code": "ok", "reason": "", "data": {"echo": msg["cmd"]}}
                writer.write((json.dumps(reply) + "\n").encode())
                await writer.drain()
        except (ConnectionError, asyncio.CancelledError):
            pass

    async def push(self, snapshot):
        for w in self.writers:
            w.write((json.dumps({"v": 1, "type": "telemetry", "snapshot": snapshot}) + "\n").encode())
            await w.drain()

    async def stop(self):
        for w in self.writers:
            w.close()
        self.server.close()
        await self.server.wait_closed()


@pytest.fixture
def sock_path():
    d = tempfile.mkdtemp(prefix="dyx3gw")
    yield os.path.join(d, "g.sock")


async def wait_until(pred, timeout=3.0):
    end = asyncio.get_running_loop().time() + timeout
    while asyncio.get_running_loop().time() < end:
        if pred():
            return True
        await asyncio.sleep(0.01)
    return False


async def test_request_reply_and_telemetry_cache(sock_path):
    srv = FakeServer(sock_path)
    await srv.start()
    gw = GatewayClient(sock_path, request_timeout_s=1.0)
    seen = []
    gw.on_telemetry(lambda s: seen.append(s))
    states = []
    gw.on_state(lambda c: states.append(c))
    await gw.start()
    assert await wait_until(lambda: gw.connected)
    r = await gw.request("pause_mission")
    assert r["ok"] and r["data"]["echo"] == "pause_mission"
    assert srv.received[0]["v"] == 1 and srv.received[0]["args"] == {}
    await srv.push({"rtk_status": None})
    assert await wait_until(lambda: gw.snapshot is not None)
    assert gw.snapshot_age() < 1.0
    assert await wait_until(lambda: seen == [{"rtk_status": None}])
    assert states == [True]
    await gw.stop()
    await srv.stop()


async def test_not_connected_fails_at_once_and_is_never_queued(sock_path):
    gw = GatewayClient(sock_path, request_timeout_s=0.5)
    with pytest.raises(GatewayUnavailable) as e:
        await gw.request("estop", {"asserted": True, "source": "tablet"})
    assert e.value.delivered is False
    # the gateway appears later: the old command must NOT be replayed
    srv = FakeServer(sock_path)
    await srv.start()
    await gw.start()
    assert await wait_until(lambda: gw.connected)
    await asyncio.sleep(0.1)
    assert srv.received == []
    await gw.stop()
    await srv.stop()


async def test_timeout_is_reported_as_delivery_unknown(sock_path):
    srv = FakeServer(sock_path)
    srv.mode = "silent"
    await srv.start()
    gw = GatewayClient(sock_path, request_timeout_s=0.2)
    await gw.start()
    assert await wait_until(lambda: gw.connected)
    with pytest.raises(GatewayTimeout) as e:
        await gw.request("pause_mission")
    assert e.value.delivered is None
    assert len(srv.received) == 1
    await gw.stop()
    await srv.stop()


async def test_pending_requests_fail_when_the_link_drops_and_the_client_reconnects(sock_path):
    srv = FakeServer(sock_path)
    srv.mode = "drop"
    await srv.start()
    gw = GatewayClient(sock_path, request_timeout_s=2.0, reconnect_min_s=0.05, reconnect_max_s=0.1)
    await gw.start()
    assert await wait_until(lambda: gw.connected)
    with pytest.raises(GatewayUnavailable):
        await gw.request("pause_mission")
    srv.mode = "echo"
    assert await wait_until(lambda: gw.connected)  # reconnected by itself
    assert (await gw.request("resume_mission"))["ok"]
    await gw.stop()
    await srv.stop()


async def test_garbage_and_foreign_versions_are_ignored(sock_path):
    srv = FakeServer(sock_path)
    await srv.start()
    gw = GatewayClient(sock_path, request_timeout_s=1.0)
    await gw.start()
    assert await wait_until(lambda: gw.connected)
    for w in srv.writers:
        w.write(b"not json\n" + json.dumps({"v": 9, "type": "telemetry", "snapshot": {"x": 1}}).encode() + b"\n")
        await w.drain()
    await asyncio.sleep(0.1)
    assert gw.snapshot is None
    assert (await gw.request("skip_point"))["ok"]
    await gw.stop()
    await srv.stop()


async def test_a_gateway_that_stops_reading_times_out_instead_of_blocking_in_drain(sock_path):
    accepted = []

    async def never_read(reader, writer):
        accepted.append(writer)
        await asyncio.sleep(3600)  # accept, then never read: the client's write buffer fills up

    server = await asyncio.start_unix_server(never_read, path=sock_path)
    gw = GatewayClient(sock_path, request_timeout_s=0.3)
    await gw.start()
    assert await wait_until(lambda: gw.connected)
    loop = asyncio.get_running_loop()
    t0 = loop.time()
    with pytest.raises(GatewayTimeout) as e:
        await gw.request("pause_mission", {"pad": "x" * (16 * 1024 * 1024)})  # far beyond socket + stream buffers
    assert loop.time() - t0 < 1.5  # bounded by the request timeout, drain() included
    assert e.value.delivered is None
    assert gw._pending == {}  # the pending id is removed
    await gw.stop()
    for w in accepted:
        w.close()
    server.close()


async def test_accept_then_close_backs_off_instead_of_flapping(sock_path):
    accepts = []

    async def accept_and_close(reader, writer):
        accepts.append(asyncio.get_running_loop().time())
        writer.close()  # like the gateway at max_clients

    server = await asyncio.start_unix_server(accept_and_close, path=sock_path)
    gw = GatewayClient(sock_path, request_timeout_s=0.5, reconnect_min_s=0.05, reconnect_max_s=0.4)
    await gw.start()
    await asyncio.sleep(1.5)
    await gw.stop()
    server.close()
    await server.wait_closed()
    # Without backoff growth this would be about 1.5 / 0.05 = 30 attempts; with it: 0.05, 0.1, 0.2, 0.4, 0.4, ...
    assert 3 <= len(accepts) <= 8, accepts
    assert accepts[-1] - accepts[-2] >= 0.3


async def test_backoff_resets_after_the_gateway_has_spoken(sock_path):
    srv = FakeServer(sock_path)
    await srv.start()
    gw = GatewayClient(sock_path, request_timeout_s=0.5, reconnect_min_s=0.05, reconnect_max_s=0.4)
    await gw.start()
    assert await wait_until(lambda: gw.connected)
    await srv.push({"x": 1})  # the gateway speaks: a healthy link, the backoff is back at its minimum
    assert await wait_until(lambda: gw.snapshot is not None)
    for w in srv.writers:
        w.close()
    srv.writers.clear()
    t0 = asyncio.get_running_loop().time()
    assert await wait_until(lambda: not gw.connected, 1.0)
    assert await wait_until(lambda: gw.connected, 1.0)
    assert asyncio.get_running_loop().time() - t0 < 0.3  # reconnected after reconnect_min_s, not a grown delay
    assert (await gw.request("pause_mission"))["ok"]
    await gw.stop()
    await srv.stop()
