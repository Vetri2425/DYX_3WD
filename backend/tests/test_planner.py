"""BE-004: planning runs in its own process, one job at a time, within a wall-clock and a point budget."""

import asyncio
import multiprocessing
import os
import threading
import time
from pathlib import Path

import anyio
import pytest
from backend_helpers import FakeGateway, H, exit_hard, sleep_then, spin_forever, token_store
from fastapi.testclient import TestClient

from dyx3_backend.config.settings import Settings
from dyx3_backend.main import create_api
from dyx3_backend.mission.planner import Planner
from dyx3_backend.mission.service import MissionError

DATA = Path(__file__).parent / "data" / "missions"


def planners_alive() -> list:
    return [p for p in multiprocessing.active_children() if p.name == "dyx3-planner"]


@pytest.fixture
def rig(tmp_path):
    s = Settings(data_dir=str(tmp_path))
    api, _, _ = create_api(s, tokens=token_store(), gateway=FakeGateway())
    return TestClient(api), api, s


@pytest.mark.anyio
async def test_an_over_budget_job_is_terminated_and_never_blocks_the_event_loop():
    planner = Planner(timeout_s=1.5)
    gaps: list[float] = []
    done = asyncio.Event()

    async def ticker():
        last = time.monotonic()
        while not done.is_set():
            await asyncio.sleep(0.02)
            now = time.monotonic()
            gaps.append(now - last)
            last = now

    task = asyncio.create_task(ticker())
    t0 = time.monotonic()
    with pytest.raises(MissionError) as err:
        await anyio.to_thread.run_sync(planner.run, spin_forever)
    elapsed = time.monotonic() - t0
    done.set()
    await task
    assert err.value.status == 422 and err.value.code == "plan_budget_exceeded"
    assert 1.4 <= elapsed < 6.0
    assert len(gaps) > 30 and max(gaps) < 0.25  # the loop kept ticking (heartbeats would keep flowing)
    assert planners_alive() == []  # the runaway process was terminated
    assert planner.busy is False


def test_one_job_at_a_time_second_is_409_busy():
    planner = Planner(timeout_s=20.0)
    out = {}
    worker = threading.Thread(target=lambda: out.setdefault("r", planner.run(sleep_then, 1.0, "first")))
    worker.start()
    deadline = time.monotonic() + 5.0
    while not planner.busy and time.monotonic() < deadline:
        time.sleep(0.01)
    with pytest.raises(MissionError) as err:
        planner.run(sleep_then, 0.0, "second")
    assert err.value.status == 409 and err.value.code == "busy"
    worker.join(30.0)
    assert out["r"] == "first"
    assert planner.run(sleep_then, 0.0, "third") == "third"  # free again


def test_busy_planner_is_409_on_every_planning_route(rig):
    c, api, s = rig
    lock = api.state.missions.planner._lock
    assert lock.acquire(blocking=False)
    try:
        dxf = (DATA / "square_2x2.dxf").read_bytes()
        r = c.post("/api/missions", headers=H("oper-tok"), files={"file": ("s.dxf", dxf)})
        assert r.status_code == 409 and r.json()["code"] == "busy"
        r = c.post("/api/path/parse-dxf", headers=H("oper-tok"), files={"file": ("s.dxf", dxf)})
        assert r.status_code == 409 and r.json()["code"] == "busy"
        r = c.post("/api/missions/plan", headers=H("oper-tok"), json={"client": "x"})
        assert r.status_code == 409 and r.json()["code"] == "busy"
    finally:
        lock.release()
    assert not os.path.isdir(s.missions_dir)
    r = c.post("/api/missions", headers=H("oper-tok"), files={"file": ("s.dxf", dxf)})
    assert r.status_code == 201, r.text


def test_the_point_budget_refuses_an_oversized_plan(tmp_path):
    s = Settings(data_dir=str(tmp_path), plan_max_points=3)
    api, _, _ = create_api(s, tokens=token_store(), gateway=FakeGateway())
    c = TestClient(api)
    r = c.post("/api/missions", headers=H("oper-tok"), files={"file": ("s.dxf", (DATA / "square_2x2.dxf").read_bytes())})
    assert r.status_code == 422 and r.json()["code"] == "points_limit_exceeded"
    assert not os.path.isdir(s.missions_dir)


def test_a_crashed_planning_process_is_an_error_not_a_hang():
    planner = Planner(timeout_s=20.0)
    with pytest.raises(MissionError) as err:
        planner.run(exit_hard, 3)
    assert err.value.status == 500 and err.value.code == "planner_crashed"
    assert planners_alive() == []


def test_a_deeply_nested_app_plan_is_400_not_500(rig):
    c, _, s = rig
    depth = 200_000
    r = c.post("/api/missions/plan", headers=H("oper-tok"), content=b"[" * depth + b"]" * depth)
    assert r.status_code == 400 and r.json()["code"] == "INVALID_PAYLOAD"
    assert not os.path.isdir(s.missions_dir)


def test_planning_budget_settings():
    d = Settings.from_env({})
    assert (d.plan_timeout_s, d.plan_max_points) == (60.0, 200_000)
    e = Settings.from_env({"DYX3_PLAN_TIMEOUT_S": "5", "DYX3_PLAN_MAX_POINTS": "1000"})
    assert (e.plan_timeout_s, e.plan_max_points) == (5.0, 1000)
    for key in ("DYX3_PLAN_TIMEOUT_S", "DYX3_PLAN_MAX_POINTS"):
        with pytest.raises(ValueError):
            Settings.from_env({key: "0"})
