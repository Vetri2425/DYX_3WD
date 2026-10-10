"""XR-BE-001: bearer check and body cap happen before FastAPI reads any body byte."""

import json

import pytest
from backend_helpers import FakeGateway, H, token_store
from fastapi.testclient import TestClient

from dyx3_backend.config.settings import Settings
from dyx3_backend.main import create_api, create_app

MiB = 1024 * 1024


@pytest.fixture
def rig(tmp_path):
    gw = FakeGateway()
    s = Settings(data_dir=str(tmp_path), upload_max_bytes=1 * MiB)
    api, _hub, _sio = create_api(s, tokens=token_store(), gateway=gw)
    return TestClient(api), gw, api


async def call_raw(app, method, path, headers, chunks):
    """Drive the ASGI app directly; count how many body bytes it pulled from the client."""
    pulled = {"bytes": 0, "calls": 0}
    pending = list(chunks)

    async def receive():
        pulled["calls"] += 1
        if not pending:
            return {"type": "http.request", "body": b"", "more_body": False}
        chunk = pending.pop(0)
        pulled["bytes"] += len(chunk)
        return {"type": "http.request", "body": chunk, "more_body": bool(pending)}

    sent = []

    async def send(message):
        sent.append(message)

    scope = {
        "type": "http", "asgi": {"version": "3.0"}, "http_version": "1.1", "method": method, "scheme": "http",
        "path": path, "raw_path": path.encode(), "query_string": b"", "root_path": "",
        "headers": [(k.lower().encode(), v.encode()) for k, v in headers.items()],
        "client": ("127.0.0.1", 5000), "server": ("127.0.0.1", 8000),
    }
    await app(scope, receive, send)
    start = next(m for m in sent if m["type"] == "http.response.start")
    body = b"".join(m.get("body", b"") for m in sent if m["type"] == "http.response.body")
    return start["status"], {k.decode(): v.decode() for k, v in start["headers"]}, body, pulled


def test_tokenless_bad_body_is_401_not_422(rig):
    c, gw, _ = rig
    r = c.post("/api/estop", content=b"{", headers={"Content-Type": "application/json"})
    assert r.status_code == 401
    assert r.headers.get("www-authenticate") == "Bearer"
    assert r.json() == {"detail": "missing bearer token"}
    r = c.post("/api/estop", content=b"{", headers={"Content-Type": "application/json", **H("wrong")})
    assert r.status_code == 401 and r.json() == {"detail": "invalid token"}
    assert gw.calls == []


@pytest.mark.anyio
async def test_a_50mb_tokenless_body_is_refused_without_reading_it(rig):
    _, gw, api = rig
    chunks = [b"x" * MiB] * 50
    for path in ("/api/estop", "/api/missions/plan", "/api/rtk/config"):
        for headers in ({"content-length": str(50 * MiB)}, {}):  # declared, and chunked (no length)
            status, hdrs, _body, pulled = await call_raw(api, "POST", path, headers, chunks)
            assert status == 401, path
            assert hdrs.get("www-authenticate") == "Bearer"
            assert pulled["calls"] == 0 and pulled["bytes"] == 0, path  # not one body byte read
    assert gw.calls == []


@pytest.mark.anyio
async def test_oversized_authorized_bodies_are_413(rig):
    _, gw, api = rig
    tok = {"authorization": "Bearer oper-tok"}
    # Declared too large: refused before reading.
    status, _, body, pulled = await call_raw(api, "POST", "/api/estop", {**tok, "content-length": str(50 * MiB)},
                                             [b"x" * MiB] * 50)
    assert status == 413 and json.loads(body)["code"] == "too_large"
    assert pulled["bytes"] == 0
    # Chunked, no length: reading stops just past the 64 KiB JSON cap.
    status, _, body, pulled = await call_raw(api, "POST", "/api/estop", tok, [b" " * 16 * 1024] * 3200)
    assert status == 413 and json.loads(body)["code"] == "too_large"
    assert pulled["bytes"] <= 64 * 1024 + 16 * 1024
    # Upload routes use upload_max_bytes (1 MiB here), not the JSON cap.
    status, _, body, pulled = await call_raw(api, "POST", "/api/missions/plan", tok, [b" " * 256 * 1024] * 200)
    assert status == 413 and json.loads(body)["code"] == "too_large"
    assert pulled["bytes"] <= MiB + 256 * 1024
    assert gw.calls == []


def test_oversized_authorized_json_is_413_via_http(rig):
    c, gw, _ = rig
    big = json.dumps({"asserted": True, "pad": "x" * (70 * 1024)})
    r = c.post("/api/estop", headers=H("oper-tok"), content=big)
    assert r.status_code == 413 and r.json()["code"] == "too_large"
    r = c.put("/api/rtk/config", headers=H("oper-tok"), json={"pad": "x" * (70 * 1024)})
    assert r.status_code == 413
    # The viewer is authenticated, so the cap (not the role) answers first for a huge body; the route still
    # refuses the viewer for a normal one.
    assert c.post("/api/vehicle/arm", headers=H("view-tok"), content=big).status_code == 413
    assert c.post("/api/vehicle/arm", headers=H("view-tok"), json={"arm": True}).status_code == 403
    assert gw.calls == []


def test_authorized_normal_requests_are_unchanged(rig):
    c, gw, _ = rig
    assert c.post("/api/estop", headers=H("view-tok"), json={"asserted": True}).status_code == 200
    assert c.post("/api/estop", headers=H("oper-tok"), content=b"{").status_code == 422  # authorized: FastAPI as before
    assert c.post("/api/mission/pause", headers=H("oper-tok")).status_code == 200
    assert c.get("/api/health", headers=H("view-tok")).status_code == 200
    assert c.get("/api/ping").status_code == 200  # still open
    assert gw.calls == [("estop", {"asserted": True, "source": "tablet"}), ("pause_mission", {})]


def test_an_invalid_content_length_is_refused(rig):
    _, _, api = rig
    import anyio

    status, _, _, pulled = anyio.run(call_raw, api, "POST", "/api/estop",
                                     {"authorization": "Bearer oper-tok", "content-length": "12abc"}, [b"{}"])
    assert status == 400 and pulled["bytes"] == 0


def test_socketio_is_not_behind_the_bearer_check(tmp_path):
    app = create_app(Settings(data_dir=str(tmp_path)), tokens=token_store(), gateway=FakeGateway())
    c = TestClient(app)
    r = c.get("/socket.io/?EIO=4&transport=polling")  # no Authorization header: Socket.IO authenticates on connect
    assert r.status_code == 200 and r.text.startswith("0{")
    assert c.post("/api/estop", content=b"{").status_code == 401
    assert c.get("/api/ping").status_code == 200


def test_cap_settings_parse(tmp_path):
    assert Settings.from_env({}).json_body_max_bytes == 64 * 1024
    assert Settings.from_env({"DYX3_JSON_BODY_MAX_BYTES": "4096"}).json_body_max_bytes == 4096
    with pytest.raises(ValueError):
        Settings.from_env({"DYX3_JSON_BODY_MAX_BYTES": "0"})
