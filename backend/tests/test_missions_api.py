"""Upload -> path engine -> content-addressed artifact -> start, through the REST surface."""

import os
from pathlib import Path

import pytest
from backend_helpers import FakeGateway, H, token_store
from fastapi.testclient import TestClient

from dyx3_backend.config.settings import Settings
from dyx3_backend.main import create_api

DATA = os.path.join(os.path.dirname(__file__), "data", "missions")


@pytest.fixture
def rig(tmp_path):
    gw = FakeGateway()
    s = Settings(data_dir=str(tmp_path), upload_max_bytes=1_000_000)
    api, _, _ = create_api(s, tokens=token_store(), gateway=gw)
    return TestClient(api), gw, s


def up(c, name, data, **form):
    return c.post("/api/missions", headers=H("oper-tok"), files={"file": (name, data)}, data=form)


def test_upload_plans_stores_and_is_idempotent(rig):
    c, _, s = rig
    data = Path(os.path.join(DATA, "square_2x2.dxf")).read_bytes()
    r = up(c, "square_2x2.dxf", data)
    assert r.status_code == 201, r.text
    m = r.json()["mission"]
    assert len(m["sha256"]) == 64 and m["num_points"] > 4 and m["num_spray_points"] > 0
    assert m["source"]["name"] == "square_2x2.dxf"
    assert os.path.exists(os.path.join(s.missions_dir, m["sha256"] + ".dyx3path"))
    again = up(c, "renamed.dxf", data).json()["mission"]
    assert again["sha256"] != "" and again["num_points"] == m["num_points"]
    listing = c.get("/api/missions", headers=H("view-tok")).json()["missions"]
    assert m["sha256"] in [x["sha256"] for x in listing]
    one = c.get(f"/api/missions/{m['sha256']}", headers=H("view-tok")).json()["mission"]
    assert one["sha256"] == m["sha256"]
    path = c.get(f"/api/missions/{m['sha256']}/path", headers=H("view-tok")).json()
    assert path["frame"] == "local_ned" and len(path["points"]) == m["num_points"]
    assert all(len(p) == 3 for p in path["points"])


def test_waypoints_file_and_parameters(rig):
    c, _, _ = rig
    data = Path(os.path.join(DATA, "mission_straight_5m.waypoints")).read_bytes()
    a = up(c, "m.waypoints", data).json()["mission"]
    b = up(c, "m.waypoints", data, origin_n="1.5", origin_e="2.5").json()["mission"]
    assert a["sha256"] != b["sha256"]  # a different origin is a different mission
    assert b["bbox_ne_m"][0] == pytest.approx(a["bbox_ne_m"][0] + 1.5, abs=1e-6)


@pytest.mark.parametrize(
    "name,data,form,status",
    [
        ("evil.sh", b"x", {}, 415),
        ("noext", b"x", {}, 415),
        ("x.dxf", b"", {}, 422),
        ("x.dxf", b"A" * 1_000_001, {}, 413),
        ("x.dxf", b"this is not a dxf file", {}, 422),
        ("x.dxf", b"0\nEOF\n", {"rotation_deg": "400"}, 422),
        ("x.dxf", b"0\nEOF\n", {"origin_n": "nan"}, 422),
        ("x.dxf", b"0\nEOF\n", {"anchor": "somewhere"}, 422),
        ("x.dxf", b"0\nEOF\n", {"unit_scale": "-1"}, 422),
    ],
)
def test_bad_uploads_are_refused_with_a_reason(rig, name, data, form, status):
    c, _, s = rig
    r = up(c, name, data, **form)
    assert r.status_code == status, r.text
    assert r.json()["ok"] is False and r.json()["code"]
    assert not os.path.isdir(s.missions_dir) or os.listdir(s.missions_dir) == []


def test_the_client_filename_is_never_a_path(rig, tmp_path):
    c, _, _s = rig
    data = Path(os.path.join(DATA, "square_2x2.dxf")).read_bytes()
    r = up(c, "../../../../tmp/evil.dxf", data)
    assert r.status_code == 201
    assert r.json()["mission"]["source"]["name"] == "evil.dxf"
    assert not os.path.exists("/tmp/evil.dxf")


def test_upload_requires_the_operator_role(rig):
    c, _, _ = rig
    r = c.post("/api/missions", headers=H("view-tok"), files={"file": ("a.dxf", b"x")})
    assert r.status_code == 403
    assert c.post("/api/missions", files={"file": ("a.dxf", b"x")}).status_code == 401


def test_start_goes_to_the_gateway_only_for_a_readable_artifact(rig):
    c, gw, _ = rig
    data = Path(os.path.join(DATA, "square_2x2.dxf")).read_bytes()
    sha = up(c, "s.dxf", data).json()["mission"]["sha256"]
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
    data = Path(os.path.join(DATA, "square_2x2.dxf")).read_bytes()
    sha = up(c, "s.dxf", data).json()["mission"]["sha256"]
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


def accepted(mission_id: int) -> dict:
    return {"v": 1, "ok": True, "code": "ok", "reason": "", "data": {"accepted": True, "reason_code": 0, "mission_id": mission_id}}


def test_start_answers_202_accepted_with_the_execution(rig):
    c, gw, _ = rig
    sha = plan_mission(c)
    gw.replies["start_mission"] = accepted(7)
    r = c.post(f"/api/missions/{sha}/start", headers=H("oper-tok"))
    assert r.status_code == 202, r.text
    assert r.json() == {"ok": True, "accepted": True, "execution": {"mission_id": 7, "request_id": None},
                        "data": {"accepted": True, "reason_code": 0, "mission_id": 7}}
    assert gw.calls[-1] == ("start_mission", {"path_artifact_sha256": sha})  # no id: none is invented


def test_start_passes_the_request_id_through_also_on_a_duplicate(rig):
    c, gw, _ = rig
    sha = plan_mission(c)
    gw.replies["start_mission"] = accepted(7)
    first = c.post(f"/api/missions/{sha}/start", headers=H("oper-tok"), json={"request_id": "tab-1:9f2c"})
    again = c.post(f"/api/missions/{sha}/start", headers={**H("oper-tok"), "Idempotency-Key": "tab-1:9f2c"})
    both = c.post(f"/api/missions/{sha}/start", headers={**H("oper-tok"), "Idempotency-Key": "tab-1:9f2c"},
                  json={"request_id": "tab-1:9f2c"})
    for r in (first, again, both):
        assert r.status_code == 202, r.text
        assert r.json()["execution"] == {"mission_id": 7, "request_id": "tab-1:9f2c"}
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
