"""App-owned mission ingest and its RPP run-boundary contract."""

import copy
import json
from pathlib import Path

import pytest
from backend_helpers import FakeGateway, H, token_store
from fastapi.testclient import TestClient
from generate_app_plan_fixture import FIXTURE, PAYLOAD

from dyx3_backend.config.settings import Settings
from dyx3_backend.main import create_api, create_app
from dyx3_backend.mission import path_artifact as pa
from dyx3_backend.mission.app_plan import compile_plan


def plan(*kinds):
    runs = []
    n = 0.0
    for kind in kinds:
        spray = 1 if kind == "mark" else 0
        runs.append({"type": kind, "points": [[n, 0.0, spray | 2], [n + 1, 0.0, spray], [n + 2, 0.0, spray | 2]]})
        n += 2
    return {"client": "Three_Wheel_v2", "client_version": "1.0.0", "frame": "local_ned", "runs": runs}


def mirror_split(points):
    # Mirror dyx3_rpp/src/path_conditioner.cpp:179 split_runs_by_flag.
    # RPP passes only (z & 1) as flags, so must-hit cannot split a run.
    flags = [p.flags & 1 for p in points]
    starts = [0] + [i for i in range(1, len(points)) if flags[i] != flags[i - 1]]
    groups = [list(points[start:end]) for start, end in zip(starts, starts[1:] + [len(points)])]
    return [[(p, group[0].flags & 1) for p in (([groups[i - 1][-1]] if i else []) + group)]
            for i, group in enumerate(groups)]


@pytest.fixture
def rig(tmp_path):
    settings = Settings(data_dir=str(tmp_path))
    api, _, _ = create_api(settings, tokens=token_store(), gateway=FakeGateway())
    return TestClient(api), settings


@pytest.mark.parametrize("kinds", [("travel",), ("mark",), ("travel", "mark"), ("mark", "travel"), ("travel", "mark", "travel", "mark", "travel")])
def test_round_trip_run_boundaries(rig, kinds):
    client, settings = rig
    body = plan(*kinds)
    response = client.post("/api/missions/plan", headers=H("oper-tok"), json=body)
    assert response.status_code == 201, response.text
    sha = response.json()["mission"]["sha256"]
    artifact = pa.load(settings.missions_dir, sha)
    assert artifact.sha256 == sha
    reconstructed = mirror_split(artifact.points)
    assert len(reconstructed) == len(body["runs"])
    for run, expected in zip(reconstructed, body["runs"]):
        assert [(p.north_m, p.east_m, spray) for p, spray in run] == [
            (p[0], p[1], p[2] & 1) for p in expected["points"]]
    # Every shared point keeps the prior spray state and both must-hit bits.
    for index in range(1, len(body["runs"])):
        prior = reconstructed[index - 1][-1][0]
        current = reconstructed[index][0][0]
        assert (prior.north_m, prior.east_m) == (current.north_m, current.east_m)
        assert prior.must_hit and current.must_hit
    assert client.get(f"/api/missions/{sha}/path", headers=H("view-tok")).status_code == 200
    assert client.post(f"/api/missions/{sha}/start", headers=H("oper-tok")).status_code == 200
    assert client.post("/api/missions/plan", headers=H("oper-tok"), json=body).json()["mission"]["sha256"] == sha


def test_app_builder_fixture_and_content(rig):
    client, settings = rig
    body = json.loads((Path(__file__).parent / "data/app_builder_payload.json").read_text())
    response = client.post("/api/missions/plan", headers=H("oper-tok"), json=body)
    assert response.status_code == 201, response.text
    art = pa.load(settings.missions_dir, response.json()["mission"]["sha256"])
    assert [(p.north_m, p.east_m, p.flags) for p in art.points] == [(0, 0, 2), (2, 0, 2), (6, 0, 1), (10, 0, 3)]
    assert art.meta["source"]["num_runs"] == 2


def test_cpp_fixture_is_generated_by_backend_encoder():
    assert FIXTURE.read_bytes() == compile_plan(PAYLOAD)
    art = pa.decode(FIXTURE.read_bytes())
    runs = mirror_split(art.points)
    assert [[(p.north_m, p.east_m, spray) for p, spray in run] for run in runs] == [
        [(p[0], p[1], p[2] & 1) for p in run["points"]] for run in PAYLOAD["runs"]]
    assert art.points[2].must_hit  # the interior corner survives intact


def test_near_boundary_uses_prior_exact_coordinate_and_or_must_hit():
    body = plan("travel", "mark")
    body["runs"][0]["points"][-1][2] = 0
    body["runs"][1]["points"][0][0] += 0.0005
    art = pa.decode(compile_plan(body))
    assert (art.points[2].north_m, art.points[2].east_m, art.points[2].flags) == (2.0, 0.0, 2)


@pytest.mark.parametrize("mutate,code", [
    (lambda b: b["runs"][0]["points"][0].__setitem__(2, 3), "mixed_spray_in_run"),
    (lambda b: b["runs"].append(copy.deepcopy(b["runs"][1])), "adjacent_runs_same_type"),
    (lambda b: b["runs"][1]["points"][0].__setitem__(0, 1.5), "runs_not_contiguous"),
    (lambda b: b.__setitem__("frame", "enu"), "INVALID_FRAME"),
    (lambda b: b["runs"][0].__setitem__("type", "paint"), "INVALID_RUN_TYPE"),
    (lambda b: b["runs"][0]["points"][0].__setitem__(2, 4), "INVALID_FLAGS"),
    (lambda b: b["runs"][0]["points"][0].__setitem__(0, float("nan")), "NON_FINITE_COORDINATE"),
    (lambda b: b["runs"][0]["points"][0].__setitem__(0, 10001), "OUT_OF_BOUNDS"),
    (lambda b: b["runs"][0]["points"][1].__setitem__(0, 6), "STEP_TOO_LARGE"),
    (lambda b: b["runs"][0].__setitem__("points", [[0, 0, 2]]), "RUN_TOO_SHORT"),
    (lambda b: b.__setitem__("runs", []), "EMPTY_MISSION"),
])
def test_rejections(rig, mutate, code):
    client, settings = rig
    body = plan("travel", "mark")
    mutate(body)
    response = client.post("/api/missions/plan", headers=H("oper-tok"), content=json.dumps(body))
    assert response.status_code == 422, response.text
    assert response.json()["code"] == code
    assert not Path(settings.missions_dir).exists()


def test_point_limit_and_auth_and_bad_json(rig):
    client, _ = rig
    body = plan("mark")
    body["runs"][0]["points"] = [[0, 0, 1]] * 50_001
    assert client.post("/api/missions/plan", headers=H("oper-tok"), json=body).json()["code"] == "POINTS_LIMIT_EXCEEDED"
    assert client.post("/api/missions/plan", headers=H("view-tok"), json=plan("mark")).status_code == 403
    assert client.post("/api/missions/plan", json=plan("mark")).status_code == 401
    assert client.post("/api/missions/plan", headers=H("oper-tok"), content=b"{").json()["code"] == "INVALID_PAYLOAD"
    missing = plan("mark")
    del missing["client"]
    response = client.post("/api/missions/plan", headers=H("oper-tok"), json=missing)
    assert response.status_code == 400 and response.json()["code"] == "INVALID_PAYLOAD"


def test_route_never_imports_path_engine(rig, monkeypatch):
    client, _ = rig
    original = __import__

    def guarded(name, *args, **kwargs):
        if "path_engine" in name:
            raise AssertionError(f"unexpected path engine import: {name}")
        return original(name, *args, **kwargs)

    monkeypatch.setattr("builtins.__import__", guarded)
    assert client.post("/api/missions/plan", headers=H("oper-tok"), json=plan("mark")).status_code == 201


def test_full_app_registers_route_once(tmp_path):
    app = create_app(Settings(data_dir=str(tmp_path)), tokens=token_store(), gateway=FakeGateway()).api
    routes = [*app.routes]
    for included in app.routes:
        if hasattr(included, "original_router"):
            routes.extend(included.original_router.routes)
    assert sum(getattr(r, "path", None) == "/api/missions/plan" and "POST" in r.methods for r in routes) == 1
