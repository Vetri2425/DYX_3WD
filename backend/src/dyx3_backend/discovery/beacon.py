"""Rover identity and UDP discovery beacon.

The tablet app finds the rover on whatever network it is on (site router, Jetson hotspot) without
a fixed IP. Every interval the backend sends one JSON datagram to the subnet broadcast address of
each IPv4 network it is on, with the address it has on that network:

    {"type": "dyx3_beacon", "v": 1, "rover_id": "dyx3-…", "rover_name": "…", "ip": "192.168.3.150", "port": 8000}

The FCU link (10.41.10.0/24 by default) is never beaconed: nothing there needs it, and PX4's Ethernet
should only carry DDS and MAVLink. The beacon is transport only. It carries no secret and controls
nothing; a send failure never affects the API. Pattern ported from the 4WD prototype
(rover_ws rover_backend/beacon.py), changed to beacon per interface because the 3WD is multi-homed.
"""

from __future__ import annotations

import asyncio
import hashlib
import ipaddress
import json
import logging
import socket
import subprocess
from dataclasses import dataclass

LOGGER = logging.getLogger(__name__)

BEACON_TYPE = "dyx3_beacon"
BEACON_VERSION = 1


@dataclass(frozen=True)
class RoverIdentity:
    rover_id: str
    rover_name: str


def rover_identity(configured_id: str = "", configured_name: str = "", machine_id_path: str = "/etc/machine-id") -> RoverIdentity:
    """Stable per-rover identity. An explicit id/name wins; otherwise derived from the machine id and hostname,
    so it survives IP changes, network switches, reboots and stack upgrades."""
    rover_id = configured_id.strip()
    if not rover_id:
        try:
            with open(machine_id_path, encoding="ascii") as fh:
                mid = fh.read().strip()
        except OSError:
            mid = ""
        rover_id = "dyx3-" + hashlib.sha256((mid or socket.gethostname()).encode()).hexdigest()[:10]
    rover_name = configured_name.strip() or socket.gethostname()
    return RoverIdentity(rover_id=rover_id, rover_name=rover_name)


@dataclass(frozen=True)
class BeaconTarget:
    ifname: str
    ip: str
    broadcast: str


def beacon_targets(ip_json: list[dict], exclude: tuple[str, ...]) -> list[BeaconTarget]:
    """One target per usable IPv4 address in `ip -j -4 addr` output: not loopback, not excluded."""
    nets = [ipaddress.IPv4Network(c, strict=False) for c in exclude if c.strip()]
    out: list[BeaconTarget] = []
    for link in ip_json:
        if "LOOPBACK" in link.get("flags", []) or "UP" not in link.get("flags", []):
            continue
        for a in link.get("addr_info", []):
            if a.get("family") != "inet" or a.get("scope") != "global":
                continue
            try:
                iface = ipaddress.IPv4Interface(f"{a['local']}/{a['prefixlen']}")
            except (KeyError, ValueError):
                continue
            if any(iface.ip in n for n in nets) or iface.network.prefixlen >= 31:
                continue
            out.append(BeaconTarget(link.get("ifname", "?"), str(iface.ip), str(iface.network.broadcast_address)))
    return out


def beacon_payload(identity: RoverIdentity, ip: str, port: int) -> bytes:
    return json.dumps(
        {
            "type": BEACON_TYPE,
            "v": BEACON_VERSION,
            "rover_id": identity.rover_id,
            "rover_name": identity.rover_name,
            "ip": ip,
            "port": port,
        },
        separators=(",", ":"),
    ).encode()


def _read_ip_json() -> list[dict]:
    out = subprocess.run(["ip", "-j", "-4", "addr", "show"], capture_output=True, text=True, timeout=2, check=False)
    return json.loads(out.stdout or "[]")


class DiscoveryBeacon:
    def __init__(self, identity: RoverIdentity, api_port: int, beacon_port: int, interval_s: float, exclude: tuple[str, ...]):
        self._identity = identity
        self._api_port = api_port
        self._beacon_port = beacon_port
        self._interval_s = interval_s
        self._exclude = exclude
        self._task: asyncio.Task | None = None
        self._last_error = ""

    async def start(self) -> None:
        self._task = asyncio.create_task(self._run(), name="dyx3-discovery-beacon")

    async def stop(self) -> None:
        if self._task:
            self._task.cancel()
            try:
                await self._task
            except asyncio.CancelledError:
                pass

    def send_once(self) -> int:
        """Send one beacon per target; returns how many were sent. Never raises."""
        sent = 0
        try:
            targets = beacon_targets(_read_ip_json(), self._exclude)
        except (OSError, ValueError, subprocess.SubprocessError) as exc:
            self._log_once(f"cannot list interfaces: {exc}")
            return 0
        for t in targets:
            try:
                with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
                    s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
                    s.bind((t.ip, 0))
                    s.sendto(beacon_payload(self._identity, t.ip, self._api_port), (t.broadcast, self._beacon_port))
                sent += 1
            except OSError as exc:
                self._log_once(f"beacon on {t.ifname} {t.ip} failed: {exc}")
        return sent

    def _log_once(self, msg: str) -> None:
        if msg != self._last_error:
            LOGGER.warning("discovery beacon: %s", msg)
            self._last_error = msg

    async def _run(self) -> None:
        loop = asyncio.get_running_loop()
        while True:
            await loop.run_in_executor(None, self.send_once)
            await asyncio.sleep(self._interval_s)
