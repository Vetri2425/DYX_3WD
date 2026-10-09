"""Backend settings, read once from the environment (``DYX3_*``). Contract: docs/contracts/backend.md.

Nothing here is a tuning value for motion. The DERIVED numbers (upload limit, heartbeat relay) are recorded in the
contract as OPEN questions.
"""

from __future__ import annotations

import math
import os
from dataclasses import dataclass, field, replace


def _f(env: dict, key: str, default: float) -> float:
    v = float(env.get(key, default))
    if not math.isfinite(v) or v <= 0.0:
        raise ValueError(f"{key} must be finite and > 0")
    return v


@dataclass(frozen=True)
class Settings:
    data_dir: str = "/var/lib/dyx3"
    auth_file: str = ""
    gateway_socket: str = "/run/dyx3/gateway.sock"
    rtk_socket: str = "/run/dyx3/rtk-control.sock"
    # DERIVED — NOT FROM V1 SPEC: large DXF drawings exist; the cap only bounds memory. OPEN.
    upload_max_bytes: int = 20 * 1024 * 1024
    # gateway reply wait; must exceed the gateway's own service_timeout_s (2.0) so its verdict arrives first
    request_timeout_s: float = 3.0
    # DERIVED — NOT FROM V1 SPEC (see contract section 3). OPEN.
    heartbeat_relay_s: float = 0.5
    tablet_heartbeat_timeout_s: float = 1.5
    telemetry_stale_s: float = 2.0
    # DERIVED — re-validate in field Wi-Fi. Session cleanup, not motion safety.
    sio_ping_interval_s: float = 5.0
    # DERIVED — re-validate in field Wi-Fi. Tablet heartbeat owns the safety timeout.
    sio_ping_timeout_s: float = 5.0
    allowed_extensions: tuple[str, ...] = field(default=(".dxf", ".csv", ".waypoints"))
    # Rover identity and LAN discovery beacon (contract section 1a). Empty id/name = derived from the
    # machine id and hostname. The beacon is off by default here (tests); from_env turns it on.
    rover_id: str = ""
    rover_name: str = ""
    api_port: int = 8000
    beacon_enabled: bool = False
    beacon_port: int = 5003
    # DERIVED — NOT FROM V1 SPEC: 1 s keeps first discovery under ~1 s; one ~120-byte datagram per network.
    beacon_interval_s: float = 1.0
    beacon_exclude: tuple[str, ...] = ("10.41.10.0/24",)

    @property
    def missions_dir(self) -> str:
        return os.path.join(self.data_dir, "missions")

    @property
    def runs_dir(self) -> str:
        return os.path.join(self.data_dir, "runs")

    @property
    def state_dir(self) -> str:
        return os.path.join(self.data_dir, "state")

    @property
    def auth_path(self) -> str:
        return self.auth_file or os.path.join(self.state_dir, "auth.json")

    def with_(self, **kw) -> Settings:
        return replace(self, **kw)

    @staticmethod
    def from_env(env: dict | None = None) -> Settings:
        e = dict(os.environ if env is None else env)
        s = Settings(
            data_dir=e.get("DYX3_DATA_DIR", "/var/lib/dyx3"),
            auth_file=e.get("DYX3_AUTH_FILE", ""),
            gateway_socket=e.get("DYX3_GATEWAY_SOCKET", "/run/dyx3/gateway.sock"),
            rtk_socket=e.get("DYX3_RTK_CONTROL_SOCKET", "/run/dyx3/rtk-control.sock"),
            upload_max_bytes=int(_f(e, "DYX3_UPLOAD_MAX_BYTES", 20 * 1024 * 1024)),
            request_timeout_s=_f(e, "DYX3_REQUEST_TIMEOUT_S", 3.0),
            heartbeat_relay_s=_f(e, "DYX3_HEARTBEAT_RELAY_S", 0.5),
            tablet_heartbeat_timeout_s=_f(e, "DYX3_TABLET_HEARTBEAT_TIMEOUT_S", 1.5),
            telemetry_stale_s=_f(e, "DYX3_TELEMETRY_STALE_S", 2.0),
            sio_ping_interval_s=_f(e, "DYX3_SIO_PING_INTERVAL_S", 5.0),
            sio_ping_timeout_s=_f(e, "DYX3_SIO_PING_TIMEOUT_S", 5.0),
            rover_id=e.get("DYX3_ROVER_ID", ""),
            rover_name=e.get("DYX3_ROVER_NAME", ""),
            api_port=int(_f(e, "DYX3_BACKEND_PORT", 8000)),
            beacon_enabled=e.get("DYX3_BEACON_ENABLE", "1").strip() not in ("0", "false", "no", ""),
            beacon_port=int(_f(e, "DYX3_BEACON_PORT", 5003)),
            beacon_interval_s=_f(e, "DYX3_BEACON_INTERVAL_S", 1.0),
            beacon_exclude=tuple(x.strip() for x in e.get("DYX3_BEACON_EXCLUDE", "10.41.10.0/24").split(",") if x.strip()),
        )
        if s.heartbeat_relay_s >= s.tablet_heartbeat_timeout_s:
            raise ValueError("heartbeat_relay_s must be shorter than tablet_heartbeat_timeout_s")
        return s
