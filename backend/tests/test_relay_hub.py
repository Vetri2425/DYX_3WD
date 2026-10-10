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

    async def emit(ev, data, to=None):
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
    emitted.clear()  # (the connect replays ran meanwhile; covered below)
    await hub.broadcast_telemetry({"a": 1})
    await hub.broadcast_gateway_state(False)
    assert emitted[0][0] == "telemetry" and emitted[0][1]["snapshot"] == {"a": 1}
    assert emitted[1][0] == "rover_event"
    assert emitted[1][1]["kind"] == "gateway_link" and emitted[1][1]["data"] == {"connected": False}


async def test_hub_telemetry_carries_seq_t_mono_s_and_dropped():
    gw, clk = FakeGateway(), Clock()
    relay = OperatorLinkRelay(gw, relay_s=0.5, tablet_timeout_s=1.5, clock=clk)
    emitted = []

    async def emit(ev, data, to=None):
        emitted.append((ev, data))

    hub = RealtimeHub(token_store(), gw, relay, emit)
    emitted.clear()
    await hub.broadcast_telemetry_frame({"snapshot": {"a": 1}, "seq": 7, "t_mono_s": 123.25, "dropped": 2})
    await hub.broadcast_telemetry({"a": 2})  # no gateway stamp: null, nothing dropped
    assert emitted[0] == ("telemetry", {"snapshot": {"a": 1}, "age_s": None, "seq": 7, "t_mono_s": 123.25, "dropped": 2})
    assert emitted[1] == ("telemetry", {"snapshot": {"a": 2}, "age_s": None, "seq": None, "t_mono_s": None, "dropped": 0})


async def test_hub_estop_rules_and_honest_verdicts():
    gw, clk = FakeGateway(), Clock()
    relay = OperatorLinkRelay(gw, relay_s=0.5, tablet_timeout_s=1.5, clock=clk)
    async def no_emit(*_a, **_k):
        return None

    hub = RealtimeHub(token_store(), gw, relay, no_emit)
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
        def on_telemetry_frame(self, _cb):
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


# ---- status events: one Socket.IO event, rover_event


class Emitter:
    def __init__(self):
        self.out = []  # (event, data, to)

    async def __call__(self, ev, data, to=None):
        self.out.append((ev, data, to))

    def status(self, to=None):
        return [d for ev, d, t in self.out if ev == "rover_event" and t == to]


def gw_event(kind, seq, data, **extra):
    e = {"v": 1, "type": "event", "event": kind, "seq": seq, "t_mono_s": 5.0, "t_wall_ms": 1791624580123,
         "coalesced": 0, "replay": False, "data": data}
    e.update(extra)
    return e


async def test_gateway_events_are_relayed_at_once_with_the_hub_sequence():
    import asyncio

    gw, clk = FakeGateway(), Clock()
    relay = OperatorLinkRelay(gw, relay_s=0.5, tablet_timeout_s=1.5, clock=clk)
    em = Emitter()
    hub = RealtimeHub(token_store(), gw, relay, em)
    assert gw.event_cbs == [hub.broadcast_event]  # the hub subscribes itself to the gateway's events
    await hub.broadcast_gateway_state(True)
    await hub.broadcast_event(gw_event("mission_state", 11, {"state": 1, "reason_code": 0}))
    await hub.broadcast_event(gw_event("estop", 12, {"asserted": True, "source": "ble"}, coalesced=2))
    s = em.status()
    assert [p["kind"] for p in s] == ["gateway_link", "mission_state", "estop"]
    assert [p["seq"] for p in s] == sorted(p["seq"] for p in s) and len({p["seq"] for p in s}) == 3
    assert s[0]["data"] == {"connected": True} and s[0]["gateway_seq"] is None and isinstance(s[0]["t_wall_ms"], int)
    m = s[1]
    assert m == {"kind": "mission_state", "seq": m["seq"], "gateway_seq": 11, "t_mono_s": 5.0, "t_wall_ms": 1791624580123,
                 "coalesced": 0, "replay": False, "data": {"state": 1, "reason_code": 0}}
    assert s[2]["coalesced"] == 2 and s[2]["data"]["asserted"] is True
    # a session that connects now gets the latest of every kind, to itself only, in seq order, marked replay
    em.out.clear()
    assert hub.on_connect("o", {"token": "oper-tok"}) is True
    await asyncio.sleep(0)
    await asyncio.sleep(0)
    mine = em.status(to="o")
    assert [p["kind"] for p in mine] == ["gateway_link", "mission_state", "estop"]
    assert all(p["replay"] for p in mine)
    assert [p["seq"] for p in mine] == [p["seq"] for p in s]  # the original sequence numbers
    assert em.status() == []  # nothing broadcast
    # a newer mission state replaces the old one in the replay
    await hub.broadcast_event(gw_event("mission_state", 13, {"state": 3, "reason_code": 0}))
    em.out.clear()
    hub.on_connect("v", {"token": "view-tok"})
    await asyncio.sleep(0)
    await asyncio.sleep(0)
    kinds = {p["kind"]: p for p in em.status(to="v")}
    assert kinds["mission_state"]["data"]["state"] == 3 and kinds["mission_state"]["gateway_seq"] == 13


async def test_the_v2_mission_fields_reach_the_tablet_unchanged_and_a_step_change_is_an_event():
    import asyncio

    gw, clk = FakeGateway(), Clock()
    em = Emitter()
    hub = RealtimeHub(token_store(), gw, OperatorLinkRelay(gw, relay_s=0.5, tablet_timeout_s=1.5, clock=clk), em)
    data = {"state": 7, "mission_id": 7, "run_index": 0, "point_index": 0, "reason_code": 13,
            "path_artifact_sha256": "e" * 64, "source_artifact_sha256": "a" * 64, "request_id": "tab-1:9f2c",
            "reason_detail": "OFFBOARD not confirmed; release: disarm \u2014 timeout", "gate_reason_code": 0, "waiting_on": 7,
            "state_entered": 1791624580.5, "fresh": True, "stamp_s": 1791624581.25}
    await hub.broadcast_event(gw_event("mission_state", 21, data))
    # the same state, reason and request, the next step of the release: another event, never merged away by the hub
    await hub.broadcast_event(gw_event("mission_state", 22, {**data, "waiting_on": 8}))
    sent = em.status()
    assert [p["data"] for p in sent] == [data, {**data, "waiting_on": 8}]  # untouched: nothing added, renamed or dropped
    assert [p["gateway_seq"] for p in sent] == [21, 22] and sent[0]["seq"] < sent[1]["seq"]
    # the replay for a session that connects later carries the latest, whole
    hub.on_connect("o", {"token": "oper-tok"})
    await asyncio.sleep(0)
    await asyncio.sleep(0)
    mine = {p["kind"]: p for p in em.status(to="o")}
    assert mine["mission_state"]["data"] == {**data, "waiting_on": 8} and mine["mission_state"]["replay"] is True


async def test_before_the_gateway_connects_a_session_learns_the_link_is_down():
    import asyncio

    gw, clk = FakeGateway(), Clock()
    em = Emitter()
    hub = RealtimeHub(token_store(), gw, OperatorLinkRelay(gw, relay_s=0.5, tablet_timeout_s=1.5, clock=clk), em)
    hub.on_connect("v", {"token": "view-tok"})
    await asyncio.sleep(0)
    mine = em.status(to="v")
    assert len(mine) == 1 and mine[0]["kind"] == "gateway_link" and mine[0]["data"] == {"connected": False}


async def test_end_to_end_gateway_socket_to_socketio_emit():
    """A real GatewayClient on a Unix socket: an event line becomes a rover_event emit, a reconnect resumes the flow."""
    import asyncio
    import json
    import os
    import tempfile

    from dyx3_backend.gateway.client import GatewayClient

    path = os.path.join(tempfile.mkdtemp(prefix="dyx3hub"), "g.sock")  # short: AF_UNIX paths are limited
    writers = []

    async def handle(_reader, writer):
        writers.append(writer)
        await asyncio.sleep(3600)

    server = await asyncio.start_unix_server(handle, path=path)
    gw = GatewayClient(path, request_timeout_s=1.0, reconnect_min_s=0.05, reconnect_max_s=0.1)
    em = Emitter()
    hub = RealtimeHub(token_store(), gw, OperatorLinkRelay(gw, relay_s=0.5, tablet_timeout_s=1.5), em)
    gw.on_state(hub.broadcast_gateway_state)
    await gw.start()
    loop = asyncio.get_running_loop()

    async def until(pred, timeout=2.0):
        end = loop.time() + timeout
        while loop.time() < end and not pred():
            await asyncio.sleep(0.001)
        return pred()

    async def push(seq, state):
        line = {"v": 1, "type": "event", "event": "mission_state", "seq": seq, "t_mono_s": 1.0, "t_wall_ms": 1, "coalesced": 0,
                "replay": False, "data": {"state": state}}
        writers[-1].write((json.dumps(line) + "\n").encode())
        await writers[-1].drain()

    def mission_states():
        return [p["data"]["state"] for p in em.status() if p["kind"] == "mission_state"]

    try:
        assert await until(lambda: writers and gw.connected)
        lat = []
        for i in range(1, 21):
            t0 = loop.time()
            await push(i, i)
            assert await until(lambda n=i: len(mission_states()) == n)
            lat.append(loop.time() - t0)
        lat.sort()
        print(f"gateway event line -> Socket.IO emit: p50 {lat[10] * 1e3:.3f} ms, max {lat[-1] * 1e3:.3f} ms")
        assert lat[10] < 0.005
        writers[-1].close()  # the link drops and comes back
        assert await until(lambda: [p["data"] for p in em.status() if p["kind"] == "gateway_link"][-1] == {"connected": False})
        n = len(writers)
        assert await until(lambda: len(writers) > n and gw.connected)
        await push(21, 99)
        assert await until(lambda: mission_states()[-1:] == [99])
        links = [p["data"]["connected"] for p in em.status() if p["kind"] == "gateway_link"]
        assert links == [True, False, True]
    finally:
        await gw.stop()
        for w in writers:
            w.close()
        server.close()
