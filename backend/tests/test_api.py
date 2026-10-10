"""REST surface with a fake gateway: auth matrix, error mapping, the E-stop role rule, 'delivered: false'."""

import json

import pytest
from backend_helpers import FakeGateway, H, token_store
from fastapi.testclient import TestClient

from dyx3_backend.config.settings import Settings
from dyx3_backend.gateway.client import GatewayTimeout
from dyx3_backend.main import create_api

SHA = "a" * 64


@pytest.fixture
def rig(tmp_path):
    gw = FakeGateway()
    s = Settings(data_dir=str(tmp_path))
    api, _hub, _sio = create_api(s, tokens=token_store(), gateway=gw)
    return TestClient(api), gw, api


def test_ping_is_open_everything_else_needs_a_token(rig):
    c, _, _ = rig
    assert c.get("/api/ping").status_code == 200
    for method, path in [("get", "/api/health"), ("get", "/api/telemetry"), ("get", "/api/missions"), ("get", "/api/runs"),
                         ("post", "/api/estop"), ("post", "/api/mission/pause"), ("post", "/api/heartbeat"),
                         ("post", "/api/vehicle/arm"), ("post", "/api/spray/manual"), ("post", f"/api/missions/{SHA}/start")]:
        r = getattr(c, method)(path)
        assert r.status_code == 401, path
        r = getattr(c, method)(path, headers={"Authorization": "Bearer wrong"})
        assert r.status_code == 401, path
        assert r.headers.get("www-authenticate") == "Bearer"


def test_no_token_file_denies_everything(tmp_path):
    from dyx3_backend.auth.tokens import TokenStore

    api, _, _ = create_api(Settings(data_dir=str(tmp_path)), tokens=TokenStore(), gateway=FakeGateway())
    c = TestClient(api)
    assert c.get("/api/health", headers=H("view-tok")).status_code == 401


def test_viewer_can_read_but_not_command(rig):
    c, gw, _ = rig
    assert c.get("/api/health", headers=H("view-tok")).status_code == 200
    assert c.get("/api/telemetry", headers=H("view-tok")).status_code == 200
    for method, path, body in [("post", "/api/mission/pause", None), ("post", "/api/vehicle/arm", {"arm": True}),
                               ("post", "/api/vehicle/offboard", {"enable": True}), ("post", "/api/spray/manual", {"on": True}),
                               ("post", "/api/heartbeat", None), ("post", "/api/mission/abort", None),
                               ("post", f"/api/missions/{SHA}/start", None)]:
        r = getattr(c, method)(path, headers=H("view-tok"), **({"json": body} if body is not None else {}))
        assert r.status_code == 403, path
    assert gw.calls == []


def test_commands_are_forwarded_with_the_right_arguments(rig):
    c, gw, _ = rig
    h = H("oper-tok")
    assert c.post("/api/mission/pause", headers=h).status_code == 200
    assert c.post("/api/mission/resume", headers=h).status_code == 200
    assert c.post("/api/mission/skip_point", headers=h).status_code == 200
    assert c.post("/api/mission/abort", headers=h, json={"reason": "safety"}).status_code == 200
    assert c.post("/api/mission/abort", headers=h).status_code == 200
    assert c.post("/api/vehicle/arm", headers=h, json={"arm": False}).status_code == 200
    assert c.post("/api/vehicle/offboard", headers=h, json={"enable": True}).status_code == 200
    assert c.post("/api/spray/manual", headers=h, json={"on": True}).status_code == 200
    assert gw.calls == [
        ("pause_mission", {}), ("resume_mission", {}), ("skip_point", {}), ("abort_mission", {"reason": "safety"}),
        ("abort_mission", {"reason": "operator"}), ("arm", {"arm": False}), ("offboard", {"enable": True}),
        ("spray_manual", {"on": True}),
    ]


def test_bad_bodies_never_reach_the_gateway(rig):
    c, gw, _ = rig
    h = H("oper-tok")
    assert c.post("/api/vehicle/arm", headers=h, json={"arm": "yes"}).status_code == 422
    assert c.post("/api/vehicle/arm", headers=h, json={"arm": True, "force": True}).status_code == 422
    assert c.post("/api/mission/abort", headers=h, json={"reason": "because"}).status_code == 422
    assert c.post("/api/estop", headers=h, json={}).status_code == 422
    assert c.post("/api/missions/NOT-A-SHA/start", headers=h).status_code == 400
    assert gw.calls == []


def test_estop_anyone_may_assert_only_an_operator_may_clear(rig):
    c, gw, _ = rig
    r = c.post("/api/estop", headers=H("view-tok"), json={"asserted": True})
    assert r.status_code == 200
    assert gw.calls[-1] == ("estop", {"asserted": True, "source": "tablet"})
    n = len(gw.calls)
    assert c.post("/api/estop", headers=H("view-tok"), json={"asserted": False}).status_code == 403
    assert len(gw.calls) == n
    assert c.post("/api/estop", headers=H("oper-tok"), json={"asserted": False}).status_code == 200
    assert gw.calls[-1] == ("estop", {"asserted": False, "source": "tablet"})


def test_gateway_verdicts_map_to_http_without_reinterpretation(rig):
    c, gw, _ = rig
    h = H("oper-tok")
    gw.replies["arm"] = {"v": 1, "ok": False, "code": "rejected", "reason": "refused", "data": {"accepted": False, "reason_code": 1}}
    r = c.post("/api/vehicle/arm", headers=h, json={"arm": True})
    assert r.status_code == 409
    assert r.json()["data"]["reason_code"] == 1 and r.json()["delivered"] is True
    for code, status in [("service_unavailable", 503), ("timeout", 504), ("invalid_command", 400)]:
        gw.replies["pause_mission"] = {"v": 1, "ok": False, "code": code, "reason": "x", "data": {}}
        assert c.post("/api/mission/pause", headers=h).status_code == status


def test_an_undelivered_estop_is_reported_as_not_delivered(rig):
    c, gw, _ = rig
    gw.connected = False
    r = c.post("/api/estop", headers=H("view-tok"), json={"asserted": True})
    assert r.status_code == 503
    assert r.json()["delivered"] is False and r.json()["ok"] is False
    gw.connected = True
    gw.raises = GatewayTimeout("slow")
    r = c.post("/api/estop", headers=H("view-tok"), json={"asserted": True})
    assert r.status_code == 504
    assert r.json()["delivered"] is None  # unknown, not "accepted"


def test_heartbeat_marks_the_tablet_alive_and_health_reports_it(rig):
    c, _gw, api = rig
    assert c.get("/api/health", headers=H("view-tok")).json()["tablet_alive"] is False
    assert c.post("/api/heartbeat", headers=H("oper-tok")).json() == {"ok": True}
    h = c.get("/api/health", headers=H("view-tok")).json()
    assert h["tablet_alive"] is True and h["gateway_connected"] is True
    assert api.state.relay.tablet_alive()


def test_telemetry_endpoint_exposes_the_cached_snapshot_and_its_age(rig):
    c, gw, _ = rig
    assert c.get("/api/telemetry", headers=H("view-tok")).json()["snapshot"] is None
    gw.snapshot = {"rtk_status": {"fresh": True}}
    gw._age = 0.4
    r = c.get("/api/telemetry", headers=H("view-tok")).json()
    assert r["age_s"] == 0.4 and r["snapshot"]["rtk_status"]["fresh"] is True


def test_runs_listing_is_read_only_and_rejects_path_tricks(rig, tmp_path):
    c, _, _ = rig
    run = tmp_path / "runs" / "2026-09-05_141530_mission_0042"
    run.mkdir(parents=True)
    (run / "manifest.json").write_text(json.dumps({"mission_id": 42}))
    r = c.get("/api/runs", headers=H("view-tok")).json()
    assert r["runs"][0]["manifest"]["mission_id"] == 42 and r["runs"][0]["summary"] is None
    assert c.get("/api/runs/2026-09-05_141530_mission_0042", headers=H("view-tok")).status_code == 200
    assert c.get("/api/runs/..", headers=H("view-tok")).status_code in (404, 400)
    assert c.get("/api/runs/does-not-exist", headers=H("view-tok")).status_code == 404


def test_health_has_a_mission_block_from_the_snapshot(rig):
    c, gw, _ = rig
    empty = {"lifecycle_state": None, "last_error": None, "waiting_on": None, "age_s": None, "fresh": False}
    assert c.get("/api/health", headers=H("view-tok")).json()["mission"] == empty  # no snapshot yet
    # a healthy running mission: state named, no error, nothing waited on
    gw.snapshot, gw._age = {"mission": {"state": 3, "mission_id": 4, "reason_code": 0, "gate_reason_code": 0, "waiting_on": 0,
                                        "reason_detail": "", "age_s": 0.1, "fresh": True}}, 0.2
    assert c.get("/api/health", headers=H("view-tok")).json()["mission"] == {
        "lifecycle_state": "RUNNING", "last_error": None, "waiting_on": "NONE", "age_s": 0.1, "fresh": True}
    # a snapshot without the lifecycle fields: tolerated as null
    gw.snapshot = {"mission": {"age_s": 0.1, "fresh": True}}
    assert c.get("/api/health", headers=H("view-tok")).json()["mission"] == {**empty, "age_s": 0.1, "fresh": True}
    # the error: reason code and name, the rover's detail, the guard gate, and the step still being waited on
    gw.snapshot = {"mission": {"state": 7, "reason_code": 13, "reason_detail": "OFFBOARD not confirmed; release: disarm timed out",
                               "gate_reason_code": 0, "waiting_on": 8, "age_s": 0.1, "fresh": True}}
    assert c.get("/api/health", headers=H("view-tok")).json()["mission"] == {
        "lifecycle_state": "ERROR",
        "last_error": {"reason_code": 13, "reason": "OFFBOARD_TIMEOUT", "detail": "OFFBOARD not confirmed; release: disarm timed out",
                       "gate_reason_code": 0},
        "waiting_on": "DISARM", "age_s": 0.1, "fresh": True}
    gw._age = 60.0  # the whole snapshot is stale: never fresh
    assert c.get("/api/health", headers=H("view-tok")).json()["mission"]["fresh"] is False
    gw.snapshot, gw._age = {"mission": None}, 0.2  # source never received
    assert c.get("/api/health", headers=H("view-tok")).json()["mission"] == empty


def test_health_names_every_v2_state_reason_and_step(rig):
    from dyx3_backend.api.routes import MISSION_REASONS, MISSION_STATES, MISSION_WAITING_ON

    c, gw, _ = rig
    # the numbers of interfaces 0.17.0: states 0..10, reasons 0..18, steps 0..8 (MissionState.msg)
    assert MISSION_STATES == {0: "IDLE", 1: "LOADING", 2: "READY", 3: "RUNNING", 4: "PAUSED", 5: "COMPLETED", 6: "ABORTED",
                              7: "ERROR", 8: "PLACING", 9: "ARMING", 10: "ENGAGING"}
    assert list(MISSION_REASONS) == list(range(19)) and MISSION_REASONS[6] == "EKF_RESET" and MISSION_REASONS[17] == "RPP_STALE"
    assert MISSION_REASONS[18] == "RPP_PIVOT_TIMEOUT"
    assert list(MISSION_WAITING_ON) == list(range(9)) and MISSION_WAITING_ON[5] == "RPP_ACK"
    for state, name in MISSION_STATES.items():
        gw.snapshot, gw._age = {"mission": {"state": state, "reason_code": 0, "waiting_on": 0, "age_s": 0.1, "fresh": True}}, 0.2
        assert c.get("/api/health", headers=H("view-tok")).json()["mission"]["lifecycle_state"] == name
    for reason, name in MISSION_REASONS.items():
        gw.snapshot = {"mission": {"state": 6, "reason_code": reason, "age_s": 0.1, "fresh": True}}
        err = c.get("/api/health", headers=H("view-tok")).json()["mission"]["last_error"]
        assert (err is None) if reason == 0 else (err["reason"] == name and err["reason_code"] == reason)
    for step, name in MISSION_WAITING_ON.items():
        gw.snapshot = {"mission": {"state": 1, "reason_code": 0, "waiting_on": step, "age_s": 0.1, "fresh": True}}
        assert c.get("/api/health", headers=H("view-tok")).json()["mission"]["waiting_on"] == name
    # a number a newer rover adds is reported, not hidden or invented
    gw.snapshot = {"mission": {"state": 40, "reason_code": 99, "waiting_on": 77, "age_s": 0.1, "fresh": True}}
    block = c.get("/api/health", headers=H("view-tok")).json()["mission"]
    assert (block["lifecycle_state"], block["last_error"]["reason"], block["waiting_on"]) == ("UNKNOWN_40", "UNKNOWN_99", "UNKNOWN_77")
