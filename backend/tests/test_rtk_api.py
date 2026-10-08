"""The assembled app exposes one RTK API, with operator writes and write-only secrets."""

from __future__ import annotations

import copy

from backend_helpers import FakeGateway, H, token_store
from fastapi.testclient import TestClient

from dyx3_backend.config.settings import Settings
from dyx3_backend.main import create_api
from dyx3_backend.rtk.client import RtkRejected, RtkUnavailable


class FakeRtk:
    def __init__(self) -> None:
        self.calls: list[tuple[str, dict]] = []
        self.available = True
        self.secret = "never-return-this-password"
        self.config = {
            "schema": 1, "revision": 3, "updated_at": "", "desired_state": "RUNNING",
            "source": "NTRIP", "transport": "PX4_DDS",
            "ntrip": {"active_profile_id": "office", "profiles": [{
                "id": "office", "name": "Office", "host": "caster.example", "port": 2101,
                "mountpoint": "MOUNT", "username": "user", "security": "PLAINTEXT",
                "password_set": True,
            }]},
            "lora": {"serial_device": "", "baud": 115200},
            "usb": {"receiver_device": "", "baud": 115200},
        }

    async def request(self, command: str, **arguments) -> dict:
        if not self.available:
            raise RtkUnavailable("offline")
        self.calls.append((command, copy.deepcopy(arguments)))
        if command == "GET_STATUS":
            return {"worker_state": "INJECTING", "config_revision": self.config["revision"]}
        if command == "GET_CONFIG":
            return copy.deepcopy(self.config)
        if command == "START" or command == "STOP":
            self.config["desired_state"] = "RUNNING" if command == "START" else "STOPPED"
            return {"worker_state": self.config["desired_state"]}
        if command == "SET_CONFIG":
            candidate = arguments["config"]
            if candidate["revision"] != self.config["revision"]:
                raise RtkRejected("conflict", "configuration revision conflict")
            self.config = copy.deepcopy(candidate)
            self.config["revision"] += 1
            for profile in self.config["ntrip"]["profiles"]:
                profile.pop("password", None)
                profile["password_set"] = True
            return copy.deepcopy(self.config)
        raise AssertionError(command)


def rig(tmp_path):
    rtk = FakeRtk()
    api, _, _ = create_api(Settings(data_dir=str(tmp_path)), tokens=token_store(),
                           gateway=FakeGateway(), rtk=rtk)
    return TestClient(api), rtk, api


def test_only_one_status_route_and_worker_unavailable_is_503(tmp_path):
    client, rtk, api = rig(tmp_path)
    mounted = [route for entry in api.routes
               for route in getattr(getattr(entry, "original_router", None), "routes", [entry])]
    routes = [route for route in mounted if getattr(route, "path", None) == "/api/rtk/status"]
    assert len(routes) == 1
    assert client.get("/api/rtk/status", headers=H("view-tok")).json()["worker_state"] == "INJECTING"
    rtk.available = False
    assert client.get("/api/rtk/status", headers=H("view-tok")).status_code == 503
    assert client.get("/api/rtk/config", headers=H("view-tok")).status_code == 503


def test_rtk_auth_and_source_transport_controls(tmp_path):
    client, rtk, _ = rig(tmp_path)
    assert client.get("/api/rtk/status").status_code == 401
    assert client.put("/api/rtk/source", json={"source": "LORA"},
                      headers=H("view-tok")).status_code == 403
    assert client.put("/api/rtk/source", json={"source": "LORA", "revision": 2},
                      headers=H("oper-tok")).status_code == 409
    assert client.put("/api/rtk/source", json={"source": "LORA"},
                      headers=H("oper-tok")).status_code == 200
    assert rtk.config["source"] == "LORA"
    assert client.put("/api/rtk/transport", json={"transport": "USB_DIRECT"},
                      headers=H("oper-tok")).status_code == 200
    assert rtk.config["transport"] == "USB_DIRECT"
    assert client.post("/api/rtk/stop", headers=H("oper-tok")).status_code == 200
    assert rtk.config["desired_state"] == "STOPPED"
    assert client.post("/api/rtk/start", headers=H("oper-tok")).status_code == 200
    assert rtk.config["desired_state"] == "RUNNING"


def test_profile_password_is_write_only_across_config_and_profile_routes(tmp_path):
    client, rtk, _ = rig(tmp_path)
    h = H("oper-tok")
    assert rtk.secret not in client.get("/api/rtk/config", headers=h).text
    result = client.patch("/api/rtk/profiles/office", headers=h,
                          json={"password": "new-secret", "host": "new-caster.example"})
    assert result.status_code == 200
    assert rtk.calls[-1][1]["config"]["ntrip"]["profiles"][0]["password"] == "new-secret"
    assert "new-secret" not in result.text
    assert "new-secret" not in client.get("/api/rtk/profiles", headers=h).text
    assert client.get("/api/rtk/serial-ports", headers=h).status_code == 200
