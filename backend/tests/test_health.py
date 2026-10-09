from fastapi.testclient import TestClient

from dyx3_backend.main import app


def test_ping() -> None:
    client = TestClient(app)
    response = client.get("/api/ping")
    assert response.status_code == 200
    body = response.json()
    assert body["status"] == "ok"
    # identity for the app's per-rover token and discovery; never a secret
    assert body["rover_id"].startswith("dyx3-") or body["rover_id"]
    assert set(body) == {"status", "rover_id", "rover_name"}
