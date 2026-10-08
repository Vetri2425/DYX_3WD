"""Parse-only import has the prototype response shape and no mission side effects."""

from pathlib import Path

from backend_helpers import FakeGateway, H, token_store
from fastapi.testclient import TestClient

from dyx3_backend.config.settings import Settings
from dyx3_backend.main import create_api
from dyx3_backend.path_engine.parsers.csv_parser import read_ned_csv

DATA = Path(__file__).parent / "data/missions"


def test_parse_dxf_returns_source_entities_without_mission(tmp_path):
    settings = Settings(data_dir=str(tmp_path))
    api, _, _ = create_api(settings, tokens=token_store(), gateway=FakeGateway())
    client = TestClient(api)
    data = (DATA / "square_2x2.dxf").read_bytes()
    response = client.post("/api/path/parse-dxf", headers=H("oper-tok"),
                           files={"file": ("square_2x2.dxf", data)})
    assert response.status_code == 200, response.text
    body = response.json()
    assert body["filename"] == "square_2x2.dxf"
    assert body["num_entities"] == 4
    assert body["unit_scale"] == 1.0 and body["layer_names"] == ["0"]
    assert [(e["entity_type"], e["entity_id"], e["is_mark"], e["length_m"]) for e in body["entities"]] == [
        ("LINE", "8A", True, 2.0), ("LINE", "8B", True, 2.0),
        ("LINE", "8C", True, 2.0), ("LINE", "8D", True, 2.0)]
    assert not Path(settings.missions_dir).exists()


def test_csv_parser_reads_points_without_a_mission(tmp_path):
    path = tmp_path / "points.csv"
    path.write_text("0,0\n1,2\n3,4\n")
    assert read_ned_csv(str(path)) == [(0.0, 0.0), (1.0, 2.0), (3.0, 4.0)]
    assert not (tmp_path / "missions").exists()


def test_bad_and_oversized_dxf_refused(tmp_path):
    settings = Settings(data_dir=str(tmp_path), upload_max_bytes=100)
    api, _, _ = create_api(settings, tokens=token_store(), gateway=FakeGateway())
    client = TestClient(api)
    for filename, content, status in (("bad.dxf", b"garbage", 422), ("big.dxf", b"x" * 101, 413),
                                      ("wrong.csv", b"0,1", 415)):
        response = client.post("/api/path/parse-dxf", headers=H("oper-tok"),
                               files={"file": (filename, content)})
        assert response.status_code == status, response.text
        assert response.json()["code"]
    assert client.post("/api/path/parse-dxf", headers=H("view-tok"),
                       files={"file": ("bad.dxf", b"x")}).status_code == 403
    assert client.post("/api/path/parse-dxf", files={"file": ("bad.dxf", b"x")}).status_code == 401
    assert not Path(settings.missions_dir).exists()
