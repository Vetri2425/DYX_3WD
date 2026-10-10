#!/usr/bin/env python3
"""Verify the committed C++ path-artifact fixtures against the Python reader.

    python3 tools/gen_path_artifact_fixtures.py

The fixtures in ros2_ws/src/dyx3_mission/test/fixtures/ (`<sha256>.dyx3path` plus `manifest.txt`) were written once, by
the Python writer, from two archived missions (`square_2x2.dxf`, `mission_straight_5m.waypoints`). The backend no
longer contains the path engine that planned them (owner decision 2026-10-10: the tablet app is the only trajectory
author), so they cannot be regenerated from here and are now frozen reference data. This script does not write them. It
re-checks each manifest row with the backend's strict reader, `dyx3_backend.mission.path_artifact`:
    <name> <sha256> <n_points> <n_spray> <n_must_hit> <first_n> <first_e> <last_n> <last_e>
that the file hashes to its name, decodes strictly, and reproduces every column. The C++ reader
(`dyx3_mission/path_artifact`) must reproduce the same columns (ros2_ws/src/dyx3_mission/test/path_artifact_test.cpp).
"""
import os
import sys

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.join(REPO, "backend", "src"))

from dyx3_backend.mission import path_artifact as pa  # noqa: E402

FIXTURES = os.path.join(REPO, "ros2_ws", "src", "dyx3_mission", "test", "fixtures")


def main() -> int:
    with open(os.path.join(FIXTURES, "manifest.txt"), encoding="ascii") as fh:
        rows = [line.split() for line in fh.read().splitlines() if line.strip()]
    bad = 0
    for name, digest, n, spray, must, fn, fe, ln, le in rows:
        art = pa.load(FIXTURES, digest)  # hash == file name, strict decode
        pts = art.points
        got = [str(len(pts)), str(sum(p.spray for p in pts)), str(sum(p.must_hit for p in pts)),
               repr(pts[0].north_m), repr(pts[0].east_m), repr(pts[-1].north_m), repr(pts[-1].east_m)]
        if got != [n, spray, must, fn, fe, ln, le]:
            bad += 1
            print(f"MISMATCH {name} {digest}: manifest {[n, spray, must, fn, fe, ln, le]} reader {got}", file=sys.stderr)
        else:
            print(f"ok {name} {digest[:12]} {n} points")
    return 1 if bad else 0


if __name__ == "__main__":
    raise SystemExit(main())
