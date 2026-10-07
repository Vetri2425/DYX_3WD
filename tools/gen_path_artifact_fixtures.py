#!/usr/bin/env python3
"""Generate the C++ path-artifact fixtures from the archived missions with the Python writer.

    python3 tools/gen_path_artifact_fixtures.py

Writes ros2_ws/src/dyx3_mission/test/fixtures/<sha256>.dyx3path for two small archived missions
(`square_2x2.dxf`, `mission_straight_5m.waypoints`) plus `manifest.txt`:
    <name> <sha256> <n_points> <n_spray> <n_must_hit> <first_n> <first_e> <last_n> <last_e>
The C++ reader (`dyx3_mission/path_artifact`) and SHA-256 must reproduce every column — the
expectations come from the Python implementation run on real mission files, not from hand.
"""
import os
import sys

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.join(REPO, "backend", "src"))

from dyx3_backend.mission import path_artifact as pa  # noqa: E402
from dyx3_backend.path_engine.engine import PathEngine  # noqa: E402

MISSIONS = os.path.join(REPO, "backend", "tests", "data", "missions")
OUT = os.path.join(REPO, "ros2_ws", "src", "dyx3_mission", "test", "fixtures")
ORIGIN = os.path.join(REPO, "backend", "src", "dyx3_backend", "path_engine", "ORIGIN.sha256")


def main() -> None:
    os.makedirs(OUT, exist_ok=True)
    for old in os.listdir(OUT):
        if old.endswith(".dyx3path") or old == "manifest.txt":
            os.remove(os.path.join(OUT, old))
    engine_id = pa.engine_id_from_origin_file(ORIGIN)
    rows = []
    for name in ("square_2x2.dxf", "mission_straight_5m.waypoints"):
        path = os.path.join(MISSIONS, name)
        with open(path, "rb") as fh:
            src = fh.read()
        plan = PathEngine().plan_file(path)
        data = pa.encode_plan(plan, engine_id=engine_id, source_name=path, source_bytes=src)
        digest, _ = pa.store(OUT, data)
        art = pa.decode(data)
        pts = art.points
        rows.append(
            f"{name} {digest} {len(pts)} {sum(p.spray for p in pts)} {sum(p.must_hit for p in pts)} "
            f"{pts[0].north_m!r} {pts[0].east_m!r} {pts[-1].north_m!r} {pts[-1].east_m!r}"
        )
    with open(os.path.join(OUT, "manifest.txt"), "w", encoding="ascii") as fh:
        fh.write("\n".join(rows) + "\n")
    print("\n".join(rows))


if __name__ == "__main__":
    main()
