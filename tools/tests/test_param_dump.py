"""tools/px4/param_dump.py against a fake MAVLink connection (no vehicle, no pymavlink needed).

The fake vehicle answers like PX4 (byte-wise INT32 in the float field, _HASH_CHECK at the end of the list stream,
COMMAND_ACK then AUTOPILOT_VERSION) and records everything the tool sends. The clock is fake: no test waits for real
time except the two subprocess tests, which check that the CLI never outlives its timeout.
"""

import json
import socket
import struct
import subprocess
import sys
import time
from pathlib import Path

import pytest

TOOL = Path(__file__).resolve().parents[1] / "px4" / "param_dump.py"
sys.path.insert(0, str(TOOL.parent))

import param_dump as pd  # noqa: E402


def f32(x):
    return struct.unpack("<f", struct.pack("<f", x))[0]


class Msg:
    def __init__(self, mtype, src=(1, 1), raw4=None, **fields):
        self._type = mtype
        self._src = src
        self._raw4 = raw4
        self.__dict__.update(fields)

    def get_type(self):
        return self._type

    def get_srcSystem(self):
        return self._src[0]

    def get_srcComponent(self):
        return self._src[1]

    def get_msgbuf(self):
        if self._raw4 is None:
            raise AttributeError("no wire buffer")
        # MAVLink 2 frame: 0xFD + 9 more header bytes, then the payload (param_value first).
        return bytearray(b"\xfd" + bytes(9) + self._raw4 + bytes(20))


def param_msg(name, typ, value, index, count, src=(1, 1), raw4=None):
    if raw4 is None:
        raw4 = struct.pack("<i", value) if typ == 6 else struct.pack("<f", value)
    return Msg("PARAM_VALUE", src=src, raw4=raw4, param_id=name, param_value=struct.unpack("<f", raw4)[0],
               param_type=typ, param_index=index, param_count=count)


HASH_RAW = struct.pack("<I", 0xDEADBEEF)
CUSTOM = [0xC2, 0x5F, 0x3D, 0xE3, 0x4B, 0xFA, 0x79, 0x82]  # PX4 memcpy of 0x8279fa4be33d5fc2 (little endian)


def version_msg():
    return Msg("AUTOPILOT_VERSION", flight_sw_version=(1 << 24) | (17 << 16) | (0 << 8) | 0,
               middleware_sw_version=0, os_sw_version=0, board_version=0x0035, flight_custom_version=CUSTOM,
               middleware_custom_version=[0] * 8, os_custom_version=[0] * 8, vendor_id=0x3185, product_id=0x0035,
               uid=0x1122334455667788, capabilities=0xE4EF, uid2=[0] * 18)


class Mav:
    def __init__(self, vehicle):
        self.v = vehicle

    def __getattr__(self, name):
        def send(*args):
            self.v.sent.append((name, args))
            handler = getattr(self.v, "on_" + name, None)
            if handler:
                handler(*args)

        return send


class FakeVehicle:
    """PX4 behind mavlink-router as the tool sees it."""

    def __init__(self, params, drop=(), answer_reads=True, ack_512=0, send_version=True, heartbeat=True,
                 stop_after=None):
        self.t = 0.0
        self.params = params  # [(name, type, value)]
        self.drop = set(drop)
        self.answer_reads = answer_reads
        self.ack_512 = ack_512
        self.send_version = send_version
        self.stop_after = stop_after  # stream only this many, then fall silent (no re-read answers either)
        self.sent = []
        self.closed = False
        self.queue = []
        if heartbeat:
            # QGC's heartbeat (a GCS) comes first and must be ignored, then a foreign component's PARAM_VALUE noise.
            self.queue.append(Msg("HEARTBEAT", src=(255, 190), autopilot=8, type=6))
            self.queue.append(Msg("HEARTBEAT", src=(1, 1), autopilot=12, type=10))
        self.mav = Mav(self)

    def clock(self):
        return self.t

    def p(self, i):
        name, typ, value = self.params[i]
        return param_msg(name, typ, value, i, len(self.params))

    def on_param_request_list_send(self, ts, tc):
        assert (ts, tc) == (1, 1)
        self.queue.append(param_msg("FOREIGN", 6, 1, 0, 1, src=(1, 100)))
        n = len(self.params) if self.stop_after is None else self.stop_after
        for i in range(n):
            if i not in self.drop:
                self.queue.append(self.p(i))
        if self.stop_after is None:
            self.queue.append(Msg("PARAM_VALUE", raw4=HASH_RAW, param_id=pd.HASH_PARAM,
                                  param_value=struct.unpack("<f", HASH_RAW)[0], param_type=5,
                                  param_index=pd.PARAM_INDEX_NONE, param_count=len(self.params)))

    def on_param_request_read_send(self, ts, tc, param_id, index):
        assert param_id == b""
        if self.answer_reads and self.stop_after is None:
            self.queue.append(self.p(index))

    def on_command_long_send(self, ts, tc, cmd, conf, p1, *rest):
        if cmd == pd.MAV_CMD_REQUEST_MESSAGE:
            assert p1 == 148.0
            self.queue.append(Msg("COMMAND_ACK", command=cmd, result=self.ack_512))
            if self.ack_512 == 0 and self.send_version:
                self.queue.append(version_msg())
        elif cmd == pd.MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES:
            self.queue.append(Msg("COMMAND_ACK", command=cmd, result=0))
            if self.send_version:
                self.queue.append(version_msg())

    def recv_match(self, type=None, blocking=True, timeout=None):  # noqa: A002 - pymavlink's signature
        # Like mavutil: messages that do not match are read and dropped.
        while self.queue:
            m = self.queue.pop(0)
            self.t += 0.002
            if type is None or m.get_type() in type:
                return m
        self.t += timeout or 0.0
        return None

    def close(self):
        self.closed = True


PARAMS = [
    ("EKF2_WENC_CTRL", 6, 1),
    ("EKF2_WENC_NOISE", 9, 0.1),
    ("MAV_2_CONFIG", 6, 1000),
    ("RO_SPEED_LIM", 9, 0.85),
    ("RO_YAW_RATE_LIM", 9, 22.0),
]


def run(tmp_path, vehicle, timeout=20.0, **kw):
    out, vout = tmp_path / "params_fcu.json", tmp_path / "versions_fcu.json"
    logs = []
    code = pd.dump("tcp:fake:5760", str(out), str(vout), timeout, connect=lambda *a: vehicle, clock=vehicle.clock,
                   log=logs.append, **kw)
    return code, json.loads(out.read_text()), json.loads(vout.read_text()), logs


def test_a_lost_index_is_reread_and_every_value_has_its_type(tmp_path):
    v = FakeVehicle(PARAMS, drop={2})
    code, p, ver, _ = run(tmp_path, v)
    assert code == pd.EXIT_COMPLETE
    assert p["status"] == "complete" and p["complete"] is True
    assert p["count_expected"] == 5 and p["count_received"] == 5
    assert p["reread_requested"] == 1 and p["reread_recovered"] == 1
    assert list(p["params"]) == sorted(n for n, _, _ in PARAMS)  # sorted by name
    assert p["params"]["MAV_2_CONFIG"] == 1000 and isinstance(p["params"]["MAV_2_CONFIG"], int)
    assert p["params"]["EKF2_WENC_CTRL"] == 1
    # REAL32 is the exact float32 value, not a re-rounded decimal
    assert p["params"]["EKF2_WENC_NOISE"] == f32(0.1) == 0.10000000149011612
    assert p["params"]["RO_SPEED_LIM"] == f32(0.85)
    assert p["types"]["MAV_2_CONFIG"] == "INT32" and p["types"]["RO_SPEED_LIM"] == "REAL32"
    assert "FOREIGN" not in p["params"]  # another component's PARAM_VALUE is not the autopilot's
    assert p["hash_check"] == 0xDEADBEEF
    assert p["target"] == {"system": 1, "component": 1, "autopilot": 12}
    assert ver["status"] == "ok"
    assert v.closed


def test_only_read_requests_are_sent_never_a_set_or_a_heartbeat(tmp_path):
    v = FakeVehicle(PARAMS, drop={0, 4})
    run(tmp_path, v)
    names = {n for n, _ in v.sent}
    assert names <= {"param_request_list_send", "param_request_read_send", "command_long_send"}
    reads = [a[3] for n, a in v.sent if n == "param_request_read_send"]
    assert reads == [0, 4]  # each missing index once
    cmds = [a[2] for n, a in v.sent if n == "command_long_send"]
    assert cmds == [pd.MAV_CMD_REQUEST_MESSAGE]


def test_autopilot_version_bytes_become_the_commit_order_git_hash(tmp_path):
    _, _, ver, _ = run(tmp_path, FakeVehicle(PARAMS))
    assert ver["flight_custom_version"] == "c25f3de34bfa7982"  # as sent
    assert ver["firmware_git_hash"] == "8279fa4be33d5fc2"  # as `ver git` / QGC print it
    assert ver["firmware_git_hash"].startswith("8279fa4be3")  # the firmware pin prefix compares directly
    assert ver["flight_sw_version_str"] == "1.17.0 dev"
    assert ver["uid"] == "1122334455667788"
    assert ver["vendor_id"] == 0x3185 and ver["product_id"] == 0x0035 and ver["board_version"] == 0x35
    assert ver["requested_with"] == "MAV_CMD_REQUEST_MESSAGE"
    assert "uid2" not in ver  # all zero


def test_a_refused_request_message_falls_back_to_the_legacy_command(tmp_path):
    v = FakeVehicle(PARAMS, ack_512=3)  # MAV_RESULT_UNSUPPORTED
    code, _, ver, _ = run(tmp_path, v)
    assert code == pd.EXIT_COMPLETE
    assert ver["requested_with"] == "MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES"
    assert [a[2] for n, a in v.sent if n == "command_long_send"] == [512, 520]


def test_no_autopilot_version_is_recorded_as_unavailable_and_incomplete(tmp_path):
    v = FakeVehicle(PARAMS, send_version=False)
    code, p, ver, _ = run(tmp_path, v)
    assert code == pd.EXIT_INCOMPLETE
    assert p["complete"] is True  # the parameters themselves are complete
    assert ver["status"] == "unavailable" and "unanswered" in ver["reason"]
    assert "firmware_git_hash" not in ver


def test_timeout_keeps_what_was_read_and_names_what_is_missing(tmp_path):
    v = FakeVehicle(PARAMS, stop_after=2)  # the stream stops after 2 of 5 and nothing answers a re-read
    code, p, _, _ = run(tmp_path, v, timeout=6.0)
    assert code == pd.EXIT_INCOMPLETE
    assert p["status"] == "incomplete" and p["complete"] is False
    assert p["count_expected"] == 5 and p["count_received"] == 2
    assert p["missing_indices"] == [2, 3, 4]
    assert "3 of 5 parameters missing" in p["reason"]
    assert set(p["params"]) == {"EKF2_WENC_CTRL", "EKF2_WENC_NOISE"}
    assert v.t <= 6.0  # never longer than the timeout


def test_signalling_nan_int_bits_survive_and_non_finite_floats_are_null(tmp_path):
    snan_int = -8388607  # 0xff800001: a signalling NaN when read as a float
    params = [("BIG_INT", 6, snan_int), ("BAD_FLOAT", 9, float("inf"))]
    code, p, _, _ = run(tmp_path, FakeVehicle(params))
    assert p["params"]["BIG_INT"] == snan_int  # taken from the wire bytes, not a re-packed float
    assert p["params"]["BAD_FLOAT"] is None
    assert p["value_problems"] == {"BAD_FLOAT": "non-finite"}
    assert code == pd.EXIT_COMPLETE  # every index was received; the value problem is recorded, not hidden


def test_without_a_wire_buffer_the_float_field_is_decoded_bytewise():
    m = Msg("PARAM_VALUE", param_value=struct.unpack("<f", struct.pack("<i", 1000))[0])
    assert pd.decode_param_value(pd.value_bytes(m), 6) == (1000, None)
    assert pd.decode_param_value(struct.pack("<f", 1.5), 9) == (1.5, None)
    assert pd.decode_param_value(b"\0\0\0\0", 8)[0] is None  # INT64 does not fit


def test_unreachable_link_writes_both_files_with_the_reason(tmp_path):
    out, vout = tmp_path / "p.json", tmp_path / "v.json"

    def refuse(*_a):
        raise pd.LinkError("cannot open tcp:127.0.0.1:5760: [Errno 111] Connection refused")

    code = pd.dump("tcp:127.0.0.1:5760", str(out), str(vout), 5.0, connect=refuse, log=lambda s: None)
    assert code == pd.EXIT_UNREACHABLE
    p, ver = json.loads(out.read_text()), json.loads(vout.read_text())
    assert p["status"] == "unavailable" and p["complete"] is False and p["params"] == {}
    assert "Connection refused" in p["reason"] and "unreachable" in p["reason"]
    assert ver["status"] == "unavailable" and "Connection refused" in ver["reason"]


def test_no_autopilot_heartbeat_is_unreachable(tmp_path):
    v = FakeVehicle(PARAMS, heartbeat=False)
    out, vout = tmp_path / "p.json", tmp_path / "v.json"
    code = pd.dump("tcp:x:5760", str(out), str(vout), 3.0, connect=lambda *a: v, clock=v.clock, log=lambda s: None)
    assert code == pd.EXIT_UNREACHABLE
    assert "no autopilot HEARTBEAT" in json.loads(out.read_text())["reason"]
    assert v.t <= 3.0


def test_an_interrupt_mid_read_still_writes_what_was_read(tmp_path):
    v = FakeVehicle(PARAMS, drop={1})
    real = v.on_param_request_read_send

    def interrupt(*a):
        real(*a)
        raise pd.Interrupted("SIGINT")

    v.on_param_request_read_send = interrupt
    code, p, ver, _ = run(tmp_path, v)
    assert code == pd.EXIT_INCOMPLETE
    assert p["count_received"] == 4 and p["missing_indices"] == [1]
    assert "interrupted (SIGINT)" in p["reason"]
    assert ver["status"] == "ok"  # the version came first


def test_missing_pymavlink_is_reported(tmp_path, monkeypatch):
    monkeypatch.setitem(sys.modules, "pymavlink", None)  # import fails as if not installed
    out, vout = tmp_path / "p.json", tmp_path / "v.json"
    code = pd.dump("tcp:127.0.0.1:9", str(out), str(vout), 2.0, log=lambda s: None)
    assert code == pd.EXIT_NO_PYMAVLINK
    assert "pymavlink is not installed" in json.loads(out.read_text())["reason"]


def test_url_comes_from_the_environment_when_not_given(monkeypatch):
    monkeypatch.setenv(pd.URL_ENV, "udpin:0.0.0.0:14551")
    assert pd.parse_args([]).url == "udpin:0.0.0.0:14551"
    monkeypatch.delenv(pd.URL_ENV)
    a = pd.parse_args([])
    assert a.url == "tcp:127.0.0.1:5760" and a.timeout == 20.0
    assert a.out == "params_fcu.json" and a.version_out == "versions_fcu.json"
    with pytest.raises(SystemExit):
        pd.parse_args(["--timeout", "0"])


def _cli(tmp_path, url, timeout):
    out, vout = tmp_path / "p.json", tmp_path / "v.json"
    t0 = time.monotonic()
    r = subprocess.run([sys.executable, "-I", str(TOOL), "--url", url, "--timeout", str(timeout), "--out", str(out),
                        "--version-out", str(vout)], capture_output=True, text=True, timeout=timeout + 15)
    return r, time.monotonic() - t0, json.loads(out.read_text()), json.loads(vout.read_text())


def test_cli_on_a_silent_link_stays_within_its_timeout(tmp_path):
    # A TCP server that accepts and never speaks: with pymavlink, no heartbeat (exit 4); without it, exit 3.
    srv = socket.socket()
    srv.bind(("127.0.0.1", 0))
    srv.listen(1)
    try:
        r, elapsed, p, ver = _cli(tmp_path, f"tcp:127.0.0.1:{srv.getsockname()[1]}", 3.0)
    finally:
        srv.close()
    assert r.returncode in (pd.EXIT_NO_PYMAVLINK, pd.EXIT_UNREACHABLE), r.stderr
    assert elapsed < 3.0 + 2.0  # interpreter start-up margin only
    assert p["status"] == "unavailable" and p["reason"]
    assert ver["status"] == "unavailable"


def test_cli_on_a_closed_port_exits_non_zero_with_a_reason(tmp_path):
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()  # nothing listens here now
    r, elapsed, p, _ = _cli(tmp_path, f"tcp:127.0.0.1:{port}", 4.0)
    assert r.returncode in (pd.EXIT_NO_PYMAVLINK, pd.EXIT_UNREACHABLE, pd.EXIT_INCOMPLETE), r.stderr
    assert r.returncode != 0
    assert elapsed < 4.0 + 2.0
    assert p["complete"] is False and p["reason"]
