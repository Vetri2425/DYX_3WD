"""App-owned mission ingest and its RPP run-boundary contract."""

import copy
import json
import math
from itertools import pairwise
from pathlib import Path

import pytest
from backend_helpers import FakeGateway, H, token_store
from fastapi.testclient import TestClient
from generate_app_plan_fixture import FIXTURE, PAYLOAD

from dyx3_backend.config.settings import Settings
from dyx3_backend.main import create_api, create_app
from dyx3_backend.mission import path_artifact as pa
from dyx3_backend.mission.app_plan import compile_plan
from dyx3_backend.mission.service import MissionError

ANCHOR = {"lat": 48.137154, "lon": 11.576124, "alt": 519.5}


def plan(*kinds):
    runs = []
    n = 0.0
    for kind in kinds:
        spray = 1 if kind == "mark" else 0
        runs.append({"type": kind, "points": [[n, 0.0, spray | 2], [n + 1, 0.0, spray], [n + 2, 0.0, spray | 2]]})
        n += 2
    return {"client": "Three_Wheel_v2", "client_version": "1.0.0", "frame": "local_ned", "anchor": ANCHOR, "runs": runs}


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


def test_cpp_fixture_matches_backend_encoder_geometry():
    # The checked-in RPP fixture predates the v2 meta keys (frame, anchor, normalisation report); RPP reads only the
    # points. Its geometry and flags must still be exactly what the encoder writes for the same payload.
    fixture = pa.decode(FIXTURE.read_bytes())
    current = pa.decode(compile_plan(PAYLOAD))
    assert fixture.points == current.points and fixture.engine_id == current.engine_id
    v2_keys = {"frame", "anchor", "densified_steps", "max_boundary_snap_m"}
    assert {k: v for k, v in current.meta.items() if k not in v2_keys} == fixture.meta
    art = fixture
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


def test_plan_body_limit_uses_content_length_before_json_parsing(tmp_path):
    settings = Settings(data_dir=str(tmp_path), upload_max_bytes=80)
    api, _, _ = create_api(settings, tokens=token_store(), gateway=FakeGateway())
    client = TestClient(api)
    response = client.post("/api/missions/plan", headers=H("oper-tok"), content=b"{" + b" " * 80)
    assert response.status_code == 413 and response.json()["code"] == "too_large"
    assert not Path(settings.missions_dir).exists()


def test_plan_body_limit_bounds_stream_without_content_length(tmp_path):
    settings = Settings(data_dir=str(tmp_path), upload_max_bytes=80)
    api, _, _ = create_api(settings, tokens=token_store(), gateway=FakeGateway())
    client = TestClient(api)
    request = client.build_request("POST", "/api/missions/plan", headers=H("oper-tok"),
                                   content=iter([b"{" + b" " * 40, b" " * 40]))
    assert "content-length" not in request.headers
    response = client.send(request)
    assert response.status_code == 413 and response.json()["code"] == "too_large"
    assert not Path(settings.missions_dir).exists()


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


# ------------------------------------------------------------------------------------------------ v2 normalisation
def on_segment(p, a, b, tol=1e-9):
    cross = (b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0])
    dot = (p[0] - a[0]) * (b[0] - a[0]) + (p[1] - a[1]) * (b[1] - a[1])
    return abs(cross) <= tol and -tol <= dot <= (b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + tol


def test_long_travel_step_is_densified_on_the_same_segment(rig):
    client, settings = rig
    body = plan("mark", "travel", "mark")
    # travel run: one 12 m diagonal jump (the app's 2-point travel leg), then the next mark run starts there
    a, b = (2.0, 0.0), (2.0 + 12.0 * 0.6, 12.0 * 0.8)
    body["runs"][1]["points"] = [[a[0], a[1], 2], [b[0], b[1], 2]]
    body["runs"][2]["points"] = [[b[0], b[1], 3], [b[0] + 1, b[1], 1], [b[0] + 2, b[1], 3]]
    response = client.post("/api/missions/plan", headers=H("oper-tok"), json=body)
    assert response.status_code == 201, response.text
    assert response.json()["normalisation"] == {"densified_steps": 1, "max_boundary_snap_m": 0.0}
    art = pa.load(settings.missions_dir, response.json()["mission"]["sha256"])
    pts = [(p.north_m, p.east_m, p.flags) for p in art.points]
    start = pts.index((a[0], a[1], 3))  # shared boundary: previous run's spray bit, must-hit OR-ed
    end = pts.index((b[0], b[1], 2))
    inserted = pts[start + 1:end]
    assert len(inserted) == 2  # ceil(12 / 5) = 3 equal sub-steps
    assert all(on_segment(p, a, b) and p[2] == 0 for p in inserted)  # travel spray bit, never must-hit
    chain = [pts[start]] + inserted + [pts[end]]
    steps = [math.hypot(q[0] - p[0], q[1] - p[1]) for p, q in pairwise(chain)]
    assert all(s <= 5.0 for s in steps) and steps == pytest.approx([4.0] * 3, abs=1e-9)
    assert sum(steps) == pytest.approx(12.0, abs=1e-9)
    assert art.meta["densified_steps"] == 1
    assert art.meta["total_transit_length_m"] == pytest.approx(12.0, abs=1e-9)  # the drawn length, unchanged
    # RPP's run split is unchanged: the inserted points belong to the travel run.
    assert [len(run) for run in mirror_split(art.points)] == [3, 4, 3]


def test_densified_mark_points_keep_spray_but_not_must_hit():
    body = plan("mark")
    body["runs"][0]["points"] = [[0, 0, 3], [0, 10.5, 3]]
    art = pa.decode(compile_plan(body))
    assert [p.flags for p in art.points] == [3, 1, 1, 3]
    assert [p.east_m for p in art.points] == [0.0, 3.5, 7.0, 10.5]


def test_densify_has_a_stored_point_budget(monkeypatch):
    from dyx3_backend.mission import app_plan

    monkeypatch.setattr(app_plan, "MAX_STORED_POINTS", 10)
    body = plan("travel")
    body["runs"][0]["points"] = [[0, 0, 0], [0, 100, 0]]
    with pytest.raises(MissionError) as err:
        compile_plan(body)
    assert err.value.code == "POINTS_LIMIT_EXCEEDED"


def test_boundary_gap_within_10mm_is_snapped_and_reported(rig):
    client, settings = rig
    body = plan("travel", "mark")
    body["runs"][1]["points"][0][0] += 0.005
    body["runs"][1]["points"][0][1] += 0.005  # 7.07 mm diagonal gap
    response = client.post("/api/missions/plan", headers=H("oper-tok"), json=body)
    assert response.status_code == 201, response.text
    snap = response.json()["normalisation"]["max_boundary_snap_m"]
    assert snap == pytest.approx(math.hypot(0.005, 0.005), abs=1e-12)
    art = pa.load(settings.missions_dir, response.json()["mission"]["sha256"])
    assert art.meta["max_boundary_snap_m"] == snap and art.meta["densified_steps"] == 0
    assert (art.points[2].north_m, art.points[2].east_m) == (2.0, 0.0)  # exactly the previous run's end


def test_boundary_gap_over_10mm_is_still_refused(rig):
    client, settings = rig
    body = plan("travel", "mark")
    body["runs"][1]["points"][0][1] += 0.012
    response = client.post("/api/missions/plan", headers=H("oper-tok"), json=body)
    assert response.status_code == 422 and response.json()["code"] == "runs_not_contiguous"
    assert not Path(settings.missions_dir).exists()


# ------------------------------------------------------------------------------------------------ v2 frame and anchor
def test_local_ned_without_anchor_is_refused(rig):
    client, settings = rig
    for anchor in (None, "absent"):
        body = plan("mark")
        if anchor == "absent":
            del body["anchor"]
        else:
            body["anchor"] = anchor
        response = client.post("/api/missions/plan", headers=H("oper-tok"), json=body)
        assert response.status_code == 422 and response.json()["code"] == "ANCHOR_REQUIRED"
    assert not Path(settings.missions_dir).exists()


def test_ekf_local_ned_without_anchor_is_accepted(rig):
    client, settings = rig
    body = plan("travel", "mark")
    body["frame"] = "ekf_local_ned"
    del body["anchor"]
    response = client.post("/api/missions/plan", headers=H("oper-tok"), json=body)
    assert response.status_code == 201, response.text
    art = pa.load(settings.missions_dir, response.json()["mission"]["sha256"])
    assert art.meta["frame"] == "ekf_local_ned" and art.meta["anchor"] is None
    path = client.get(f"/api/missions/{art.sha256}/path", headers=H("view-tok")).json()
    assert path["frame"] == "ekf_local_ned" and path["anchor"] is None


def test_meta_and_preview_hold_the_anchor(rig):
    client, settings = rig
    body = plan("travel", "mark")
    body["anchor"] = {"lat": 48, "lon": -11.5}  # integers are stored as floats; alt is optional
    response = client.post("/api/missions/plan", headers=H("oper-tok"), json=body)
    assert response.status_code == 201, response.text
    sha = response.json()["mission"]["sha256"]
    art = pa.load(settings.missions_dir, sha)
    assert art.meta["anchor"] == {"alt": None, "lat": 48.0, "lon": -11.5} and art.meta["frame"] == "local_ned"
    assert '"anchor":{"alt":null,"lat":48.0,"lon":-11.5}' in (Path(settings.missions_dir) / f"{sha}.dyx3path").read_text()
    path = client.get(f"/api/missions/{sha}/path", headers=H("view-tok")).json()
    assert path["frame"] == "local_ned" and path["anchor"] == art.meta["anchor"]
    assert path["points"] == [[p.north_m, p.east_m, p.flags] for p in art.points]
    body["anchor"] = ANCHOR
    other = client.post("/api/missions/plan", headers=H("oper-tok"), json=body).json()["mission"]["sha256"]
    assert other != sha  # a different anchor is a different mission
    assert pa.load(settings.missions_dir, other).meta["anchor"] == {"alt": 519.5, "lat": 48.137154, "lon": 11.576124}


@pytest.mark.parametrize("anchor,code", [
    ({"lat": 90.5, "lon": 0}, "INVALID_ANCHOR"),
    ({"lat": 0, "lon": -180.01}, "INVALID_ANCHOR"),
    ({"lat": float("nan"), "lon": 0}, "INVALID_ANCHOR"),
    ({"lat": 0, "lon": 0, "alt": float("inf")}, "INVALID_ANCHOR"),
    ({"lat": True, "lon": 0}, "INVALID_ANCHOR"),
    ({"lat": "48.1", "lon": 0}, "INVALID_ANCHOR"),
    ({"lat": 48.1}, "INVALID_ANCHOR"),
    ({"lat": 48.1, "lon": 11.5, "datum": "WGS84"}, "INVALID_ANCHOR"),
    ([48.1, 11.5], "INVALID_ANCHOR"),
])
def test_invalid_anchor_is_refused(rig, anchor, code):
    client, settings = rig
    body = plan("mark")
    body["anchor"] = anchor
    response = client.post("/api/missions/plan", headers=H("oper-tok"), content=json.dumps(body))
    assert response.status_code == 422 and response.json()["code"] == code, response.text
    assert not Path(settings.missions_dir).exists()


def test_ekf_frame_with_an_anchor_is_refused():
    body = plan("mark")
    body["frame"] = "ekf_local_ned"
    with pytest.raises(MissionError) as err:
        compile_plan(body)
    assert err.value.code == "INVALID_FRAME"


def test_header_frame_line_is_unchanged_and_meta_canonical():
    body = plan("mark", "travel")
    body["runs"][1]["points"] = [[2, 0, 2], [2, 0.003, 0], [2, 14.003, 0]]
    data = compile_plan(body)
    lines = data.decode("ascii").split("\n")
    assert lines[1] == "frame local_ned"  # the C++ reader accepts only this header
    meta = json.loads(lines[3][5:])
    assert lines[3][5:] == json.dumps(meta, sort_keys=True, separators=(",", ":"), ensure_ascii=True, allow_nan=False)
    assert pa.decode(data).meta["densified_steps"] == 1
