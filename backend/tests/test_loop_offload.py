"""BE-005 / BE-010: artifact reads and large JSON work stay off the event loop; deep nesting is 400, not 500."""

import asyncio
import json
from pathlib import Path

import pytest
from backend_helpers import FakeGateway, H, token_store
from fastapi.testclient import TestClient

from dyx3_backend.config.settings import Settings
from dyx3_backend.main import create_api

DATA = Path(__file__).parent / "data" / "missions"


def on_event_loop() -> bool:
    try:
        asyncio.get_running_loop()
    except RuntimeError:
        return False
    return True


@pytest.fixture
def rig(tmp_path):
    gw = FakeGateway()
    api, _, _ = create_api(Settings(data_dir=str(tmp_path)), tokens=token_store(), gateway=gw)
    c = TestClient(api)
    r = c.post("/api/missions", headers=H("oper-tok"), files={"file": ("s.dxf", (DATA / "square_2x2.dxf").read_bytes())})
    assert r.status_code == 201, r.text
    return c, gw, api, r.json()["mission"]


def test_artifact_reads_run_in_a_worker_thread(rig, monkeypatch):
    c, gw, api, mission = rig
    svc = api.state.missions
    original, where = svc.get, []

    def spy(sha):
        where.append(on_event_loop())
        return original(sha)

    monkeypatch.setattr(svc, "get", spy)
    sha = mission["sha256"]
    assert c.get(f"/api/missions/{sha}", headers=H("view-tok")).json()["mission"] == mission
    path = c.get(f"/api/missions/{sha}/path", headers=H("view-tok"))
    assert c.post(f"/api/missions/{sha}/start", headers=H("oper-tok")).status_code == 202
    assert gw.calls[-1] == ("start_mission", {"path_artifact_sha256": sha})
    assert where == [False, False, False]  # never on the event loop
    # The pre-rendered /path body is the same JSON as before.
    assert path.status_code == 200 and path.headers["content-type"] == "application/json"
    body = path.json()
    assert body["sha256"] == sha and body["frame"] == "local_ned" and len(body["points"]) == mission["num_points"]
    assert path.content == json.dumps(body, ensure_ascii=False, allow_nan=False, separators=(",", ":")).encode()


def test_missing_and_bad_ids_keep_their_status(rig):
    c, gw, _, _ = rig
    n = len(gw.calls)
    assert c.get(f"/api/missions/{'0' * 64}/path", headers=H("view-tok")).status_code == 404
    assert c.get("/api/missions/xyz/path", headers=H("view-tok")).status_code == 400
    assert c.get(f"/api/missions/{'0' * 64}", headers=H("view-tok")).status_code == 404
    assert c.post(f"/api/missions/{'0' * 64}/start", headers=H("oper-tok")).status_code == 404
    assert len(gw.calls) == n


def test_excessive_json_nesting_is_400_not_500(rig):
    c, gw, _, _ = rig
    deep = b"[" * 30_000 + b"]" * 30_000  # fits under the 64 KiB JSON cap
    for method, path in (("put", "/api/rtk/config"), ("post", "/api/estop"), ("post", "/api/rtk/profiles"),
                         ("post", "/api/vehicle/arm")):
        for body in (deep, b'{"a":' + b"[" * 40 + b"]" * 40 + b"}"):
            r = getattr(c, method)(path, headers={**H("oper-tok"), "Content-Type": "application/json"}, content=body)
            assert r.status_code == 400, (path, r.status_code, r.text)
            assert r.json()["code"] == "bad_request"
    # Moderate nesting still reaches FastAPI and gets its normal validation answer.
    ok_depth = b'{"asserted": true, "x": ' + b"[" * 20 + b"]" * 20 + b"}"
    assert c.post("/api/estop", headers=H("oper-tok"), content=ok_depth).status_code == 422
    # The app-plan body is parsed in the planning process: deep nesting there is a 400 as well.
    for depth in (5_000, 500_000):
        r = c.post("/api/missions/plan", headers=H("oper-tok"), content=b"[" * depth + b"]" * depth)
        assert r.status_code == 400 and r.json()["code"] == "INVALID_PAYLOAD"
    r = c.post("/api/missions/plan", headers=H("oper-tok"),
               content=b'{"client":"a","client_version":"1","frame":"local_ned","runs":' + b"[" * 5_000 + b"]" * 5_000 + b"}")
    assert r.status_code in (400, 422) and r.json()["ok"] is False
    assert gw.calls == []


@pytest.mark.parametrize("chunks,depth_ok", [
    ([b'{"a": "[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[["}'], True),  # brackets inside a string do not count
    ([b'{"a": "\\\\", "b": ' + b"[" * 33], False),  # an escaped backslash ends the string: these count
    ([b'{"a": "x\\', b'"[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[["}'], True),  # escape split across chunks
    ([b"[" * 32, b"]" * 32], True),
    ([b'["\\"' + b"[" * 40 + b'"]'], True),  # an escaped quote keeps the string open
    ([b"[" * 20, b"{" * 13], False),
    ([b'"\\\\"' + b"[" * 33], False),  # the string closed after the escaped backslash: these count
])
def test_json_depth_scanner(chunks, depth_ok):
    from dyx3_backend.api.admission import JsonDepthScanner

    scanner = JsonDepthScanner(limit=32)
    assert all(scanner.feed(c) for c in chunks) is depth_ok
