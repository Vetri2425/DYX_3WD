"""Discovery beacon: identity, per-network targets (FCU excluded), payload, and a real UDP send."""

import json
import socket

from dyx3_backend.discovery.beacon import (
    BEACON_TYPE,
    RoverIdentity,
    beacon_payload,
    beacon_targets,
    rover_identity,
)

IP_JSON = [
    {"ifname": "lo", "flags": ["LOOPBACK", "UP"], "addr_info": [{"family": "inet", "local": "127.0.0.1", "prefixlen": 8, "scope": "host"}]},
    {
        "ifname": "enP8p1s0",
        "flags": ["BROADCAST", "UP", "LOWER_UP"],
        "addr_info": [
            {"family": "inet", "local": "10.41.10.1", "prefixlen": 24, "scope": "global"},
            {"family": "inet", "local": "192.168.3.150", "prefixlen": 24, "scope": "global"},
        ],
    },
    {"ifname": "wlP1p1s0", "flags": ["BROADCAST", "UP", "LOWER_UP"], "addr_info": [{"family": "inet", "local": "192.168.2.100", "prefixlen": 24, "scope": "global"}]},
    {"ifname": "usb2", "flags": ["BROADCAST"], "addr_info": [{"family": "inet", "local": "192.168.8.199", "prefixlen": 24, "scope": "global"}]},
]


def test_one_target_per_network_and_never_the_fcu_link():
    targets = beacon_targets(IP_JSON, ("10.41.10.0/24",))
    assert [(t.ifname, t.ip, t.broadcast) for t in targets] == [
        ("enP8p1s0", "192.168.3.150", "192.168.3.255"),
        ("wlP1p1s0", "192.168.2.100", "192.168.2.255"),
    ]


def test_identity_is_stable_and_configurable(tmp_path):
    mid = tmp_path / "machine-id"
    mid.write_text("0123456789abcdef0123456789abcdef\n")
    a = rover_identity(machine_id_path=str(mid))
    b = rover_identity(machine_id_path=str(mid))
    assert a == b and a.rover_id.startswith("dyx3-") and len(a.rover_id) == 15
    assert rover_identity("dyx3-rover-01", "Rover 01", str(mid)) == RoverIdentity("dyx3-rover-01", "Rover 01")


def test_payload_carries_address_for_that_network():
    msg = json.loads(beacon_payload(RoverIdentity("dyx3-x", "Rover"), "192.168.2.100", 8000))
    assert msg == {"type": BEACON_TYPE, "v": 1, "rover_id": "dyx3-x", "rover_name": "Rover", "ip": "192.168.2.100", "port": 8000}


def test_payload_is_receivable_over_udp():
    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.bind(("127.0.0.1", 0))
    rx.settimeout(2)
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    tx.sendto(beacon_payload(RoverIdentity("dyx3-x", "Rover"), "127.0.0.1", 8000), rx.getsockname())
    data, _ = rx.recvfrom(2048)
    tx.close()
    rx.close()
    assert json.loads(data)["rover_id"] == "dyx3-x"
