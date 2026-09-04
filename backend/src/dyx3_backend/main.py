"""DYX 3WD backend entrypoint.

Owns: REST, Socket.IO, auth, mission upload/report, telemetry delivery, settings,
storage, NTRIP profile management, and CAD/CRS path ingestion.

Does NOT own: rclpy, ROS executors, direct PX4 commands, motor safety logic,
steering, or RPP corrections. Reaches ROS only through dyx3_system_gateway over a
Unix domain socket.

Backend emergency stop is a REQUEST. Final motion authority lives in
dyx3_motion_guard and PX4.
"""

from fastapi import FastAPI

app = FastAPI(title="DYX 3WD Backend", version="0.0.0")


@app.get("/api/ping")
async def ping() -> dict[str, str]:
    return {"status": "ok"}
