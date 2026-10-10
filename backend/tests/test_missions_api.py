"""App plan -> content-addressed artifact -> list / read / start, through the REST surface."""

import os

import pytest
from backend_helpers import FakeGateway, H, token_store
from fastapi.testclient import TestClient

from dyx3_backend.config.settings import Settings
from dyx3_backend.main import create_api


@pytest.fixture
def rig(tmp_path):
    gw = FakeGateway()
    s = Settings(data_dir=str(tmp_path), upload_max_bytes=1_000_000)
    api, _, _ = create_api(s, tokens=token_store(), gateway=gw)
    return TestClient(api), gw, s


def body(**kw):
    out = {"client": "t", "client_version": "1", "frame": "ekf_local_ned", "name": "square",
           "runs": [{"type": "mark", "points": [[0, 0, 3], [1, 0, 1], [2, 0, 3]]},
                    {"type": "travel", "points": [[2, 0, 2], [2, 1, 2]]}]}
    out.update(kw)
    return out


def plan(c, **kw):
    return c.post("/api/missions/plan", headers=H("oper-tok"), json=body(**kw))


def test_plan_stores_lists_reads_and_is_idempotent(rig):
    c, _, s = rig
    r = plan(c)
    assert r.status_code == 201, r.text
    m = r.json()["mission"]
    assert len(m["sha256"]) == 64 and m["num_points"] == 4 and m["num_spray_points"] == 3
    assert m["source"]["name"] == "square"
    assert os.path.exists(os.path.join(s.missions_dir, m["sha256"] + ".dyx3path"))
    assert plan(c).json()["mission"]["sha256"] == m["sha256"]  # same plan, same artifact
    assert len(os.listdir(s.missions_dir)) == 1
    listing = c.get("/api/missions", headers=H("view-tok")).json()["missions"]
    assert m["sha256"] in [x["sha256"] for x in listing]
    one = c.get(f"/api/missions/{m['sha256']}", headers=H("view-tok")).json()["mission"]
    assert one["sha256"] == m["sha256"]
    path = c.get(f"/api/missions/{m['sha256']}/path", headers=H("view-tok")).json()
    assert path["frame"] == "ekf_local_ned" and len(path["points"]) == m["num_points"]
    assert all(len(p) == 3 for p in path["points"])


def test_a_different_plan_is_a_different_mission(rig):
    c, _, _ = rig
    a = plan(c).json()["mission"]["sha256"]
    b = plan(c, runs=[{"type": "mark", "points": [[0, 0, 3], [1.5, 0, 3]]}]).json()["mission"]["sha256"]
    assert a != b


def test_planning_requires_the_operator_role(rig):
    c, _, s = rig
    assert c.post("/api/missions/plan", headers=H("view-tok"), json=body()).status_code == 403
    assert c.post("/api/missions/plan", json=body()).status_code == 401
    assert not os.path.isdir(s.missions_dir)


@pytest.mark.parametrize("method,path", [("post", "/api/missions"), ("put", "/api/missions"),
                                         ("post", "/api/path/parse-dxf")])
def test_the_dxf_planning_routes_are_gone(rig, method, path):
    # The tablet app is the only trajectory author: the backend has no file-upload planner and no DXF parser.
    c, _, s = rig
    r = getattr(c, method)(path, headers=H("oper-tok"), files={"file": ("square_2x2.dxf", b"0\nEOF\n")})
    assert r.status_code in (404, 405), r.text
    assert not os.path.isdir(s.missions_dir)


def test_start_goes_to_the_gateway_only_for_a_readable_artifact(rig):
    c, gw, _ = rig
    sha = plan(c).json()["mission"]["sha256"]
    r = c.post(f"/api/missions/{sha}/start", headers=H("oper-tok"))
    assert r.status_code == 202
    assert gw.calls[-1] == ("start_mission", {"path_artifact_sha256": sha})
    n = len(gw.calls)
    assert c.post(f"/api/missions/{'0' * 64}/start", headers=H("oper-tok")).status_code == 404
    assert len(gw.calls) == n  # an artifact this side cannot read is never started
    assert c.get(f"/api/missions/{'0' * 64}", headers=H("view-tok")).status_code == 404
    assert c.get("/api/missions/xyz", headers=H("view-tok")).status_code == 400


def test_a_corrupt_artifact_is_skipped_by_the_listing(rig):
    c, _, s = rig
    sha = plan(c).json()["mission"]["sha256"]
    with open(os.path.join(s.missions_dir, "f" * 64 + ".dyx3path"), "wb") as fh:
        fh.write(b"junk")
    shas = [m["sha256"] for m in c.get("/api/missions", headers=H("view-tok")).json()["missions"]]
    assert shas == [sha]


# ------------------------------------------------------------------------------------------------ start v2 (202 accept)
def plan_mission(c) -> str:
    body = {"client": "t", "client_version": "1", "frame": "ekf_local_ned",
            "runs": [{"type": "mark", "points": [[0, 0, 3], [1, 0, 1], [2, 0, 3]]}]}
    r = c.post("/api/missions/plan", headers=H("oper-tok"), json=body)
    assert r.status_code == 201, r.text
    return r.json()["mission"]["sha256"]


def accepted(mission_id: int, duplicate: bool = False, resumed_run_index: int = 0) -> dict:
    return {"v": 1, "ok": True, "code": "ok", "reason": "",
            "data": {"accepted": True, "reason_code": 0, "mission_id": mission_id, "duplicate": duplicate, "gate_reason_code": 0,
                     "resumed_run_index": resumed_run_index}}


def test_start_answers_202_accepted_with_the_execution(rig):
    c, gw, _ = rig
    sha = plan_mission(c)
    gw.replies["start_mission"] = accepted(7)
    r = c.post(f"/api/missions/{sha}/start", headers=H("oper-tok"))
    assert r.status_code == 202, r.text
    assert r.json() == {"ok": True, "accepted": True,
                        "execution": {"mission_id": 7, "request_id": None, "duplicate": False, "gate_reason_code": 0,
                                      "resumed_run_index": 0},
                        "data": {"accepted": True, "reason_code": 0, "mission_id": 7, "duplicate": False, "gate_reason_code": 0,
                                 "resumed_run_index": 0}}
    assert gw.calls[-1] == ("start_mission", {"path_artifact_sha256": sha})  # no id and no resume: none is invented


@pytest.mark.parametrize("resume", [True, False])
def test_start_passes_resume_through_and_reports_the_resumed_run(rig, resume):
    c, gw, _ = rig
    sha = plan_mission(c)
    gw.replies["start_mission"] = accepted(8, resumed_run_index=3 if resume else 0)
    r = c.post(f"/api/missions/{sha}/start", headers=H("oper-tok"), json={"request_id": "tab-2:1", "resume": resume})
    assert r.status_code == 202, r.text
    assert gw.calls[-1] == ("start_mission", {"path_artifact_sha256": sha, "request_id": "tab-2:1", "resume": resume})
    assert r.json()["execution"] == {"mission_id": 8, "request_id": "tab-2:1", "duplicate": False, "gate_reason_code": 0,
                                     "resumed_run_index": 3 if resume else 0}
    assert r.json()["data"]["resumed_run_index"] == (3 if resume else 0)  # the gateway's data, untouched


def test_start_resume_alone_and_an_older_gateway_reply(rig):
    c, gw, _ = rig
    sha = plan_mission(c)
    gw.replies["start_mission"] = accepted(9, resumed_run_index=1)
    r = c.post(f"/api/missions/{sha}/start", headers=H("oper-tok"), json={"resume": True})
    assert r.status_code == 202, r.text
    assert gw.calls[-1] == ("start_mission", {"path_artifact_sha256": sha, "resume": True})
    assert r.json()["execution"]["resumed_run_index"] == 1
    # a reply without the field (a gateway older than interfaces 0.17.0) reads null, never an invented 0
    reply = accepted(9)
    del reply["data"]["resumed_run_index"]
    gw.replies["start_mission"] = reply
    r = c.post(f"/api/missions/{sha}/start", headers=H("oper-tok"), json={"resume": True})
    assert r.status_code == 202, r.text
    assert r.json()["execution"]["resumed_run_index"] is None


@pytest.mark.parametrize("value", [1, 0, "true", "yes", None, [], {}])
def test_a_resume_that_is_not_a_boolean_is_422_and_never_reaches_the_gateway(rig, value):
    c, gw, _ = rig
    sha = plan_mission(c)
    gw.calls.clear()
    r = c.post(f"/api/missions/{sha}/start", headers=H("oper-tok"), json={"resume": value})
    if value is None:
        # an explicit null is the same as absent: a fresh start, nothing forwarded for resume
        assert r.status_code == 202, r.text
        assert gw.calls[-1] == ("start_mission", {"path_artifact_sha256": sha})
        return
    assert r.status_code == 422, r.text
    assert gw.calls == []


def test_start_passes_the_request_id_through_also_on_a_duplicate(rig):
    c, gw, _ = rig
    sha = plan_mission(c)
    gw.replies["start_mission"] = accepted(7, duplicate=True)
    first = c.post(f"/api/missions/{sha}/start", headers=H("oper-tok"), json={"request_id": "tab-1:9f2c"})
    again = c.post(f"/api/missions/{sha}/start", headers={**H("oper-tok"), "Idempotency-Key": "tab-1:9f2c"})
    both = c.post(f"/api/missions/{sha}/start", headers={**H("oper-tok"), "Idempotency-Key": "tab-1:9f2c"},
                  json={"request_id": "tab-1:9f2c"})
    for r in (first, again, both):
        assert r.status_code == 202, r.text
        assert r.json()["execution"] == {"mission_id": 7, "request_id": "tab-1:9f2c", "duplicate": True, "gate_reason_code": 0,
                                         "resumed_run_index": 0}
    assert gw.calls[-3:] == [("start_mission", {"path_artifact_sha256": sha, "request_id": "tab-1:9f2c"})] * 3


@pytest.mark.parametrize("kwargs", [
    {"json": {"request_id": ""}},
    {"json": {"request_id": "x" * 65}},
    {"json": {"request_id": "has space"}},
    {"json": {"request_id": 12}},
    {"json": {"request_id": "a", "force": True}},
    {"headers": {"Idempotency-Key": "a/b"}},
    {"headers": {"Idempotency-Key": "a"}, "json": {"request_id": "b"}},
])
def test_a_bad_request_id_never_reaches_the_gateway(rig, kwargs):
    c, gw, _ = rig
    sha = plan_mission(c)
    headers = {**H("oper-tok"), **kwargs.pop("headers", {})}
    r = c.post(f"/api/missions/{sha}/start", headers=headers, **kwargs)
    assert r.status_code == 422, r.text
    assert gw.calls == []


@pytest.mark.parametrize("reply,status", [
    ({"ok": False, "code": "rejected", "reason": "refused by the target (see data.reason_code)",
      "data": {"accepted": False, "reason_code": 3, "mission_id": 0}}, 409),
    ({"ok": False, "code": "rejected", "reason": "busy", "data": {"accepted": False, "reason_code": 2, "detail": "x"}}, 409),
    ({"ok": False, "code": "service_unavailable", "reason": "mission start service is not available", "data": {}}, 503),
    ({"ok": False, "code": "timeout", "reason": "no answer", "data": {}}, 504),
    ({"ok": False, "code": "invalid_command", "reason": "bad args", "data": {}}, 400),
    # the pre-arm gate names the guard gate that failed; the mission node's refused request id (REASON_INVALID_REQUEST 4)
    ({"ok": False, "code": "rejected", "reason": "refused by the target (see data.reason_code)",
      "data": {"accepted": False, "reason_code": 3, "mission_id": 0, "duplicate": False, "gate_reason_code": 6}}, 409),
    ({"ok": False, "code": "invalid_command", "reason": "refused by the target (see data.reason_code)",
      "data": {"accepted": False, "reason_code": 4, "mission_id": 0, "duplicate": False, "gate_reason_code": 0,
               "request_id": "r1"}}, 400),
])
def test_start_errors_pass_through_typed_and_untouched(rig, reply, status):
    c, gw, _ = rig
    sha = plan_mission(c)
    gw.replies["start_mission"] = {"v": 1, **reply}
    r = c.post(f"/api/missions/{sha}/start", headers=H("oper-tok"), json={"request_id": "r1"})
    assert r.status_code == status, r.text
    assert r.json() == {"ok": False, "code": reply["code"], "reason": reply["reason"], "delivered": True, "data": reply["data"]}


def test_start_not_delivered_is_503_and_unknown_is_504(rig):
    from dyx3_backend.gateway.client import GatewayTimeout

    c, gw, _ = rig
    sha = plan_mission(c)
    gw.connected = False
    r = c.post(f"/api/missions/{sha}/start", headers=H("oper-tok"))
    assert r.status_code == 503 and r.json()["delivered"] is False and r.json()["ok"] is False
    gw.connected = True
    gw.raises = GatewayTimeout("no reply to start_mission within 2.0s")
    r = c.post(f"/api/missions/{sha}/start", headers=H("oper-tok"))
    assert r.status_code == 504 and r.json()["delivered"] is None  # unknown: re-read the state
