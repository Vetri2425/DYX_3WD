from fastapi.testclient import TestClient

from dyx3_backend.main import app


def test_ping() -> None:
    client = TestClient(app)
    response = client.get("/api/ping")
    assert response.status_code == 200
    assert response.json() == {"status": "ok"}
