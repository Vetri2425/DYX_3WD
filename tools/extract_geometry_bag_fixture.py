#!/usr/bin/env python3
"""Extract a geometry evidence fixture (recorded cross-track) from a PX4_DXP rosbag2.

    python3 tools/extract_geometry_bag_fixture.py <bag_dir> -o ros2_ws/src/dyx3_geometry/test/fixtures/bag_xtrack_<name>.txt

Reads `/rpp/conditioned_path` (nav_msgs/Path, the conditioned mission the controller tracked),
`/mavros/local_position/pose` (ENU pose -> NED), `/rpp/debug` ([0] = recorded signed cross-track,
+ = right; [7] state) and `/rpp/segment_debug` ([0] profile: 1 segment, 2 smooth; [1] segment state).
Emits one TICK per `/rpp/debug` message taken while the controller was TRACKING a SEGMENT-profile run
(segment state 1), using the latest pose at/before it. Missions with more than one run are refused:
the conditioned path concatenates runs, so the per-run segment index cannot be recovered
(see test/geometry_bag_replay_test.cpp).

Pure Python (`rosbags`), no ROS needed. Bags are NOT in Git (CLAUDE.md): run this on the Mac.
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

TOPICS = {
    "/rpp/conditioned_path",
    "/mavros/local_position/pose",
    "/rpp/debug",
    "/rpp/segment_debug",
}


def enu_pose_to_ned(position) -> tuple[float, float]:
    """ENU (x = East, y = North) -> NED (north, east)."""
    return float(position.y), float(position.x)


def extract(bag: Path) -> tuple[list[tuple[float, float, int]], list[tuple[float, float, float]]]:
    from rosbags.highlevel import AnyReader

    path: list[tuple[float, float, int]] = []
    ticks: list[tuple[float, float, float]] = []
    pose = None
    seg = None
    with AnyReader([bag]) as reader:
        conns = [c for c in reader.connections if c.topic in TOPICS]
        for conn, _ts, raw in reader.messages(connections=conns):
            msg = reader.deserialize(raw, conn.msgtype)
            if conn.topic == "/rpp/conditioned_path":
                if not path:  # latched: keep the first full mission
                    path = [
                        (float(p.pose.position.x), float(p.pose.position.y), int(round(p.pose.position.z)))
                        for p in msg.poses
                    ]
            elif conn.topic == "/mavros/local_position/pose":
                pose = enu_pose_to_ned(msg.pose.position)
            elif conn.topic == "/rpp/segment_debug":
                seg = list(msg.data)
            elif conn.topic == "/rpp/debug" and pose is not None and seg is not None:
                xtrack = float(msg.data[0])
                profile, state = int(round(seg[0])), int(round(seg[1]))
                if profile == 1 and state == 1 and math.isfinite(xtrack):
                    ticks.append((pose[0], pose[1], xtrack))
    return path, ticks


def single_run(path: list[tuple[float, float, int]]) -> bool:
    """A path is treated as ONE run when no point repeats an earlier vertex out of order and the
    spray flag changes at most at the usual PRE/MARK/AFT boundaries (<= 2 changes). Conservative."""
    changes = sum(1 for a, b in zip(path, path[1:]) if (a[2] & 1) != (b[2] & 1))
    return changes <= 2


def write(out: Path, name: str, path, ticks) -> None:
    lines = ["BAGXTRACK 1", f"PATH {name} {len(path)}"]
    lines += [f"{n!r} {e!r} {z}" for n, e, z in path]
    lines.append("END")
    lines += [f"TICK {n!r} {e!r} {x!r}" for n, e, x in ticks]
    out.write_text("\n".join(lines) + "\n", encoding="ascii")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag", type=Path)
    ap.add_argument("-o", "--out", type=Path, required=True)
    ns = ap.parse_args(argv)
    path, ticks = extract(ns.bag)
    if len(path) < 2:
        print("no /rpp/conditioned_path found", file=sys.stderr)
        return 1
    if not single_run(path):
        print("refusing: multi-run mission (segment index not recoverable from the concatenated path)", file=sys.stderr)
        return 2
    if len(ticks) < 100:
        print(f"only {len(ticks)} segment-tracking ticks; need >= 100", file=sys.stderr)
        return 3
    write(ns.out, ns.bag.name, path, ticks)
    print(f"wrote {len(ticks)} ticks, {len(path)} path points -> {ns.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
