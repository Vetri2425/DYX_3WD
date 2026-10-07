#!/usr/bin/env python3
"""Extract a legacy-RPP decode fixture from a PX4_DXP rosbag2 (pure Python, `rosbags`, no ROS).

Evidence for dyx3_rpp_legacy.output_stage: it records, per `/rpp/velocity_ned` tick, what the
prototype commanded and what state it said it was in, so the NED-vector -> MotionSetpoint
decode can be checked against RECORDED behaviour instead of hand-written expectations.

    python3 tools/extract_legacy_decode_fixture.py <bag_dir> -o fixture.json

Fixture (JSON): {"source": ..., "ticks": [ {t, v_n, v_e, yaw_rate_body, yaw_ned,
                 seg_state, seg_target_heading_ned, speed_cmd} ... ]}

Topics read (all from the prototype): /rpp/velocity_ned (Vector3Stamped, NED),
/rpp/yaw_rate_body (Float32, NED CW+), /mavros/local_position/pose (PoseStamped, ENU),
/rpp/segment_debug (Float32MultiArray: [1]=segment state, [6]=target heading NED, [8]=yaw-rate),
/rpp/debug (Float32MultiArray: [3]=speed_cmd, [7]=state code).
Each tick carries the latest sample of the other topics at or before its timestamp; ticks before
every input has been seen are dropped.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

TOPICS = {
    "/rpp/velocity_ned",
    "/rpp/yaw_rate_body",
    "/mavros/local_position/pose",
    "/rpp/segment_debug",
    "/rpp/debug",
}


def yaw_ned_from_enu_pose(q) -> float:
    """Same formula as PX4_DXP rpp_controller_node._enu_pose_to_ned."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw_enu = math.atan2(siny_cosp, cosy_cosp)
    yaw_ned = math.pi / 2.0 - yaw_enu
    return (yaw_ned + math.pi) % (2.0 * math.pi) - math.pi


def extract(bag: Path) -> dict:
    from rosbags.highlevel import AnyReader

    latest: dict[str, object] = {}
    ticks: list[dict] = []
    with AnyReader([bag]) as reader:
        conns = [c for c in reader.connections if c.topic in TOPICS]
        for conn, ts, raw in reader.messages(connections=conns):
            msg = reader.deserialize(raw, conn.msgtype)
            if conn.topic == "/rpp/velocity_ned":
                need = ("yaw_rate", "pose", "seg", "dbg")
                if all(k in latest for k in need):
                    seg = latest["seg"]
                    dbg = latest["dbg"]
                    ticks.append(
                        {
                            "t": ts * 1e-9,
                            "v_n": float(msg.vector.x),
                            "v_e": float(msg.vector.y),
                            "yaw_rate_body": float(latest["yaw_rate"]),
                            "yaw_ned": float(latest["pose"]),
                            "seg_state": int(round(seg[1])),
                            "seg_target_heading_ned": float(seg[6]),
                            "speed_cmd": float(dbg[3]),
                        }
                    )
            elif conn.topic == "/rpp/yaw_rate_body":
                latest["yaw_rate"] = msg.data
            elif conn.topic == "/mavros/local_position/pose":
                latest["pose"] = yaw_ned_from_enu_pose(msg.pose.orientation)
            elif conn.topic == "/rpp/segment_debug":
                latest["seg"] = list(msg.data)
            elif conn.topic == "/rpp/debug":
                latest["dbg"] = list(msg.data)
    return {"source": bag.name, "ticks": ticks}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag", type=Path)
    ap.add_argument("-o", "--out", type=Path, required=True)
    ns = ap.parse_args(argv)
    fx = extract(ns.bag)
    if not fx["ticks"]:
        print("no complete ticks found (is this a PX4_DXP RPP bag?)", file=sys.stderr)
        return 1
    ns.out.write_text(json.dumps(fx))
    print(f"wrote {len(fx['ticks'])} ticks from {ns.bag.name} -> {ns.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
