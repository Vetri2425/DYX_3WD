"""GatewayClient against a fake gateway speaking the NDJSON protocol over a real Unix socket."""

import asyncio
import json
import os
import tempfile

import pytest

from dyx3_backend.gateway.client import (
    EVENT_QUEUE_MAX,
    GatewayClient,
    GatewayTimeout,
    GatewayUnavailable,
)

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

    async def push_event(self, kind, seq, data=None, **extra):
        msg = {"v": 1, "type": "event", "event": kind, "seq": seq, "t_mono_s": 10.0 + seq, "t_wall_ms": 1791624580000 + seq,
               "coalesced": 0, "replay": False, "data": data if data is not None else {"seq": seq}}
        msg.update(extra)
        for w in self.writers:
            if not w.is_closing():
                w.write((json.dumps(msg) + "\n").encode())
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


# ---- status events (gateway contract section 1.3)


async def test_events_are_dispatched_in_order_and_duplicates_per_kind_are_dropped(sock_path):
    srv = FakeServer(sock_path)
    await srv.start()
    gw = GatewayClient(sock_path, request_timeout_s=1.0)
    seen = []
    gw.on_event(lambda e: seen.append((e["event"], e["seq"])))
    await gw.start()
    assert await wait_until(lambda: gw.connected and srv.writers)
    await srv.push_event("mission_state", 1)
    await srv.push_event("estop", 2)
    await srv.push_event("mission_state", 3)
    await srv.push_event("mission_state", 3, replay=True)  # a replay that raced the push: duplicate
    await srv.push_event("estop", 2, replay=True)  # same
    await srv.push_event("operator_link", 4)
    for w in srv.writers:  # malformed events are ignored, the link stays up
        w.write(b'{"v":1,"type":"event","event":5,"seq":9,"data":{}}\n{"v":1,"type":"event","event":"x","seq":"9","data":{}}\n'
                b'{"v":1,"type":"event","event":"x","seq":true,"data":{}}\n{"v":1,"type":"event","event":"x","seq":9,"data":[]}\n')
        await w.drain()
    await srv.push_event("mission_state", 5)
    assert await wait_until(lambda: len(seen) >= 5)
    await asyncio.sleep(0.05)
    assert seen == [("mission_state", 1), ("estop", 2), ("mission_state", 3), ("operator_link", 4), ("mission_state", 5)]
    assert (await gw.request("pause_mission"))["ok"]
    await gw.stop()
    await srv.stop()


async def test_a_reconnect_resumes_the_event_flow_and_the_replay_is_delivered_again(sock_path):
    srv = FakeServer(sock_path)
    await srv.start()
    gw = GatewayClient(sock_path, request_timeout_s=1.0, reconnect_min_s=0.05, reconnect_max_s=0.1)
    seen = []
    gw.on_event(lambda e: seen.append((e["event"], e["seq"], e["replay"])))
    states = []
    gw.on_state(lambda c: states.append(c))
    await gw.start()
    assert await wait_until(lambda: gw.connected and srv.writers)
    await srv.push_event("mission_state", 7)
    assert await wait_until(lambda: len(seen) == 1)
    for w in srv.writers:  # the gateway restarts or the link drops
        w.close()
    srv.writers.clear()
    assert await wait_until(lambda: states[-1:] == [False])
    assert await wait_until(lambda: gw.connected and srv.writers)
    # the gateway replays the current state on connect: the same seq, delivered again (it is the state now)
    await srv.push_event("mission_state", 7, replay=True)
    await srv.push_event("mission_state", 8)
    assert await wait_until(lambda: len(seen) == 3)
    assert seen == [("mission_state", 7, False), ("mission_state", 7, True), ("mission_state", 8, False)]
    await gw.stop()
    await srv.stop()


async def test_a_slow_event_subscriber_never_blocks_replies_and_its_queue_is_bounded(sock_path):
    srv = FakeServer(sock_path)
    await srv.start()
    gw = GatewayClient(sock_path, request_timeout_s=1.0)
    release = asyncio.Event()
    seen = []

    async def slow(e):
        seen.append(e["seq"])
        await release.wait()  # a stuck Socket.IO emit

    gw.on_event(slow)
    await gw.start()
    assert await wait_until(lambda: gw.connected and srv.writers)
    n = EVENT_QUEUE_MAX + 50
    for seq in range(1, n + 1):
        await srv.push_event("mission_state", seq)
    assert (await gw.request("pause_mission"))["ok"]  # the reader is not held up by the subscriber
    assert await wait_until(lambda: gw.events_dropped > 0)
    assert gw._events.qsize() <= EVENT_QUEUE_MAX
    release.set()
    assert await wait_until(lambda: seen and seen[-1] == n)  # the newest always gets through
    # Only the oldest are dropped, so what arrives is in order and nothing is lost but the dropped ones. Which
    # event the subscriber takes first depends on when the dispatcher wakes (seq 1 on a fast machine, a later
    # one on a loaded CI runner), so it is not asserted.
    assert seen == sorted(set(seen)) and len(seen) == n - gw.events_dropped
    await gw.stop()
    await srv.stop()


async def test_event_line_to_subscriber_latency(sock_path):
    srv = FakeServer(sock_path)
    await srv.start()
    gw = GatewayClient(sock_path, request_timeout_s=1.0)
    loop = asyncio.get_running_loop()
    got = {}
    gw.on_event(lambda e: got.setdefault(e["seq"], loop.time()))
    await gw.start()
    assert await wait_until(lambda: gw.connected and srv.writers)
    lat = []
    for seq in range(1, 51):
        t0 = loop.time()
        await srv.push_event("mission_state", seq)
        assert await wait_until(lambda s=seq: s in got, 1.0)
        lat.append(got[seq] - t0)
    lat.sort()
    print(f"gateway event line -> subscriber: p50 {lat[25] * 1e3:.3f} ms, max {lat[-1] * 1e3:.3f} ms")
    assert lat[25] < 0.005
    await gw.stop()
    await srv.stop()


async def test_a_per_call_timeout_overrides_the_default(sock_path):
    srv = FakeServer(sock_path)
    srv.mode = "silent"
    await srv.start()
    gw = GatewayClient(sock_path, request_timeout_s=5.0)
    await gw.start()
    assert await wait_until(lambda: gw.connected)
    t0 = asyncio.get_running_loop().time()
    with pytest.raises(GatewayTimeout):
        await gw.request("offboard", {"enable": True}, timeout_s=0.1)
    assert asyncio.get_running_loop().time() - t0 < 1.0
    await gw.stop()
    await srv.stop()


def test_request_timeout_outlasts_the_gateways_offboard_wait():
    # The gateway's OFFBOARD command waits up to 5.0 s (docs/contracts/dyx3_system_gateway.md); the backend's reply
    # wait must exceed it, or a command the gateway later confirms is reported to the tablet as a timeout.
    from dyx3_backend.config.settings import MIN_REQUEST_TIMEOUT_S, Settings

    assert MIN_REQUEST_TIMEOUT_S == 6.0
    assert Settings().request_timeout_s == 6.0
    assert Settings.from_env({}).request_timeout_s == 6.0
    assert Settings.from_env({"DYX3_REQUEST_TIMEOUT_S": "8"}).request_timeout_s == 8.0
    assert Settings().with_(request_timeout_s=6.0).request_timeout_s == 6.0
    for bad in ("5.99", "3", "0", "-1", "nan", "inf"):
        with pytest.raises(ValueError):
            Settings.from_env({"DYX3_REQUEST_TIMEOUT_S": bad})
    with pytest.raises(ValueError):
        Settings(request_timeout_s=3.0)
    with pytest.raises(ValueError):
        Settings().with_(request_timeout_s=5.0)
