#!/usr/bin/env python3
"""Smoke test of a RUNNING dyx3_system_gateway through the backend's own client (stdlib only).

    tools/gateway_smoke.py [socket_path]

Expected with no mission / guard / px4_link nodes running: heartbeat ok, snapshot ok, every service command reports
service_unavailable (never accepted), an unknown command is invalid_command, and a telemetry push arrives.
Verified once against the C++ node in the Humble container; not part of CI (needs a live ROS graph).
"""
import asyncio
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "backend", "src"))
from dyx3_backend.gateway.client import GatewayClient


async def main(path: str) -> int:
    gw = GatewayClient(path, request_timeout_s=6.0)
    await gw.start()
    for _ in range(100):
        if gw.connected:
            break
        await asyncio.sleep(0.05)
    if not gw.connected:
        print("not connected")
        return 1
    print("heartbeat", (await gw.request("heartbeat"))["ok"])
    snap = await gw.request("get_snapshot")
    print("snapshot", snap["ok"], snap["data"]["gateway"])
    for cmd, args in [
        ("estop", {"asserted": True, "source": "tablet"}),
        ("arm", {"arm": True}),
        ("start_mission", {"path_artifact_sha256": "a" * 64}),
        ("abort_mission", {"reason": "operator"}),
        ("reboot", {}),
    ]:
        r = await gw.request(cmd, args)
        print(cmd, r["ok"], r["code"])
    await asyncio.sleep(0.6)
    print("telemetry pushed", gw.snapshot is not None)
    await gw.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(asyncio.run(main(sys.argv[1] if len(sys.argv) > 1 else "/run/dyx3/gateway.sock")))
