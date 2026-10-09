"""Heartbeat relay timing and the Socket.IO hub with a fake emitter."""

import pytest
from backend_helpers import FakeGateway, token_store

from dyx3_backend.gateway.client import GatewayTimeout
from dyx3_backend.realtime.hub import RealtimeHub
from dyx3_backend.realtime.relay import OperatorLinkRelay

pytestmark = pytest.mark.anyio


class Clock:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t


async def test_relay_forwards_only_while_the_tablet_heartbeat_is_fresh():
    gw, clk = FakeGateway(), Clock()
    relay = OperatorLinkRelay(gw, relay_s=0.5, tablet_timeout_s=1.5, clock=clk)
    assert await relay.tick() is False  # tablet never heard
    assert gw.calls == []
    relay.note_tablet()
    clk.t += 1.0
    assert await relay.tick() is True
    assert gw.calls == [("heartbeat", {})]
    clk.t += 0.6  # 1.6 s since the tablet: dead
    assert relay.tablet_alive() is False
    assert await relay.tick() is False
    assert len(gw.calls) == 1  # a tablet dropout stops the relay, so the gateway's timeout will fire
    relay.note_tablet()
    assert await relay.tick() is True


async def test_relay_survives_a_gateway_failure_and_clear_stops_it_at_once():
    gw, clk = FakeGateway(), Clock()
    relay = OperatorLinkRelay(gw, relay_s=0.5, tablet_timeout_s=1.5, clock=clk)
    relay.note_tablet()
    gw.raises = GatewayTimeout("x")
    assert await relay.tick() is False
    gw.raises = None
    assert await relay.tick() is True
    relay.clear()
    assert relay.tablet_alive() is False
    assert await relay.tick() is False


async def test_hub_auth_roles_heartbeat_and_disconnect():
    gw, clk = FakeGateway(), Clock()
    relay = OperatorLinkRelay(gw, relay_s=0.5, tablet_timeout_s=1.5, clock=clk)
    emitted = []

    async def emit(ev, data):
        emitted.append((ev, data))

    hub = RealtimeHub(token_store(), gw, relay, emit)
    assert hub.on_connect("s0", None) is False
    assert hub.on_connect("s0", {"token": "nope"}) is False
    assert hub.on_connect("s0", {"token": 5}) is False
    assert hub.on_connect("v", {"token": "view-tok"}) is True
    assert hub.on_connect("o", {"token": "oper-tok"}) is True
    assert hub.on_heartbeat("v") == {"ok": False, "code": "forbidden"}  # a viewer cannot keep the operator link alive
    assert hub.on_heartbeat("ghost") == {"ok": False, "code": "forbidden"}
    assert relay.tablet_alive() is False
    assert hub.on_heartbeat("o") == {"ok": True}
    assert relay.tablet_alive() is True
    hub.on_disconnect("v")
    assert relay.tablet_alive() is True  # a viewer leaving does not drop the tablet
    hub.on_disconnect("o")
    assert relay.tablet_alive() is False  # the last operator session closing clears it at once
    await hub.broadcast_telemetry({"a": 1})
    await hub.broadcast_gateway_state(False)
    assert emitted[0][0] == "telemetry" and emitted[0][1]["snapshot"] == {"a": 1}
    assert emitted[1] == ("gateway", {"connected": False})


async def test_hub_estop_rules_and_honest_verdicts():
    gw, clk = FakeGateway(), Clock()
    relay = OperatorLinkRelay(gw, relay_s=0.5, tablet_timeout_s=1.5, clock=clk)
    hub = RealtimeHub(token_store(), gw, relay, lambda *_: None)
    hub.on_connect("v", {"token": "view-tok"})
    hub.on_connect("o", {"token": "oper-tok"})
    assert (await hub.on_estop("ghost", {"asserted": True}))["code"] == "unauthenticated"
    assert (await hub.on_estop("v", {"asserted": "yes"}))["code"] == "invalid_command"
    assert (await hub.on_estop("v", None))["code"] == "invalid_command"
    r = await hub.on_estop("v", {"asserted": True})
    assert r["ok"] and r["delivered"] and gw.calls[-1] == ("estop", {"asserted": True, "source": "tablet"})
    n = len(gw.calls)
    assert (await hub.on_estop("v", {"asserted": False}))["code"] == "forbidden" and len(gw.calls) == n
    assert (await hub.on_estop("o", {"asserted": False}))["ok"]
    gw.connected = False
    r = await hub.on_estop("v", {"asserted": True})
    assert r["ok"] is False and r["delivered"] is False
    gw.connected = True
    gw.raises = GatewayTimeout("slow")
    r = await hub.on_estop("v", {"asserted": True})
    assert r["ok"] is False and r["delivered"] is None


def test_socketio_endpoint_is_mounted_and_rejects_tokenless_clients(tmp_path):
    from fastapi.testclient import TestClient

    from dyx3_backend.config.settings import Settings
    from dyx3_backend.main import create_app

    app = create_app(Settings(data_dir=str(tmp_path)), tokens=token_store(), gateway=FakeGateway())
    c = TestClient(app)
    assert c.get("/api/ping").json()["status"] == "ok"
    r = c.get("/socket.io/?EIO=4&transport=polling")
    assert r.status_code == 200 and r.text.startswith("0{")  # engine.io handshake answered by the Socket.IO app


class FlakyGateway(FakeGateway):
    """Raises a non-gateway error once (a bug, not a link failure), then behaves."""

    def __init__(self, fail_times=1):
        super().__init__()
        self.fail_times = fail_times

    async def request(self, cmd, args=None):
        if self.fail_times > 0:
            self.fail_times -= 1
            self.calls.append((cmd, args or {}))
            raise RuntimeError("unexpected bug in the request path")
        return await super().request(cmd, args)


async def test_relay_task_survives_an_unexpected_exception_and_logs_it(caplog):
    import asyncio

    gw = FlakyGateway(fail_times=1)
    relay = OperatorLinkRelay(gw, relay_s=0.01, tablet_timeout_s=1.5)
    relay.note_tablet()
    assert relay.running is False and relay.operator_alive() is False
    await relay.start()
    try:
        for _ in range(200):
            await asyncio.sleep(0.01)
            relay.note_tablet()
            if len(gw.calls) >= 3:
                break
        assert len(gw.calls) >= 3  # it kept ticking after the RuntimeError
        assert relay.running is True
        assert relay.operator_alive() is True
        assert any("relay tick failed" in r.getMessage() and r.exc_info for r in caplog.records)
    finally:
        await relay.stop()
    assert relay.running is False and relay.operator_alive() is False


async def test_operator_alive_needs_a_recent_acknowledged_heartbeat():
    gw, clk = FakeGateway(), Clock()
    relay = OperatorLinkRelay(gw, relay_s=0.5, tablet_timeout_s=1.5, clock=clk)
    relay._task = _Running()  # pretend the loop is running; ticks are driven by hand
    relay.note_tablet()
    assert relay.operator_alive() is False  # nothing relayed yet
    assert await relay.tick() is True
    assert relay.operator_alive() is True
    gw.replies["heartbeat"] = {"v": 1, "ok": False, "code": "rejected"}
    clk.t += 1.0
    relay.note_tablet()
    assert await relay.tick() is False
    clk.t += 0.6  # the last ack is 1.6 s old
    relay.note_tablet()
    assert relay.operator_alive() is False


class _Running:
    def done(self):
        return False


def test_health_reports_relay_running_and_operator_alive(tmp_path):
    from fastapi.testclient import TestClient

    from dyx3_backend.config.settings import Settings
    from dyx3_backend.main import create_api

    class LifespanGateway(FakeGateway):
        def on_telemetry(self, _cb):
            pass

        def on_state(self, _cb):
            pass

        async def start(self):
            pass

        async def stop(self):
            pass

    api, _, _ = create_api(Settings(data_dir=str(tmp_path)), tokens=token_store(), gateway=LifespanGateway())
    with TestClient(api) as c:  # runs the lifespan: the relay task starts
        h = c.get("/api/health", headers={"Authorization": "Bearer view-tok"}).json()
        assert h["relay_running"] is True and h["operator_alive"] is False
    h = TestClient(api).get("/api/health", headers={"Authorization": "Bearer view-tok"}).json()
    assert h["relay_running"] is False


async def test_heartbeats_processed_right_after_a_loop_stall_do_not_extend_the_link():
    gw, clk = FakeGateway(), Clock()
    relay = OperatorLinkRelay(gw, relay_s=0.5, tablet_timeout_s=1.5, clock=clk)
    relay.note_tablet()  # t=100.0, the last heartbeat before the stall
    await relay.step()  # t=100.0 relayed
    clk.t += 0.5
    await relay.step()  # t=100.5 relayed (on time: no stall)
    assert len(gw.calls) == 2 and relay.stalls == 0
    # The event loop blocks for 2.0 s. The tablet is gone, but heartbeats it sent before vanishing sat in the socket
    # buffer and are processed now, stamped "now", before the relay task runs.
    clk.t += 2.0
    relay.note_tablet()
    assert relay.tablet_alive() is True  # without the guard this backlog would look fresh
    await relay.step()  # 2.0 s since the previous tick > 0.5 + 0.3: stall
    assert relay.stalls == 1
    assert len(gw.calls) == 2  # nothing relayed: the trusted stamp (100.0) is 2.5 s old
    assert relay.tablet_alive() is False
    clk.t += 0.2
    assert relay.note_tablet() is False  # still inside the quarantine (relay_s): ignored
    await relay.step()
    assert len(gw.calls) == 2
    clk.t += 0.4  # quarantine over: a live tablet re-proves itself with its next heartbeat
    assert relay.note_tablet() is True
    await relay.step()
    assert len(gw.calls) == 3 and relay.tablet_alive() is True


async def test_a_short_stall_keeps_a_truthful_heartbeat_and_on_time_ticks_change_nothing():
    gw, clk = FakeGateway(), Clock()
    relay = OperatorLinkRelay(gw, relay_s=0.5, tablet_timeout_s=1.5, clock=clk)
    for _ in range(10):  # 20 Hz-ish jitter but on time: never a stall
        relay.note_tablet()
        await relay.step()
        clk.t += 0.5 + 0.29
    assert relay.stalls == 0 and len(gw.calls) == 10
    relay.note_tablet()  # stamped before the stall: truthful
    await relay.step()
    clk.t += 0.9  # late by 0.4 s: a stall
    await relay.step()
    assert relay.stalls == 1
    assert len(gw.calls) == 12  # the pre-stall stamp is 0.9 s old: still alive, still relayed
