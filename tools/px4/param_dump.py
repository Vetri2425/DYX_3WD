#!/usr/bin/env python3
"""param_dump: read every PX4 parameter and the autopilot version over MAVLink. Read-only.

The FCU parameter read path of the stack (architecture 5.4.2: "Parameters keep MAVLink"; 7.9: params_fcu.json
"every FCU parameter, read live"). dyx3-recorder runs it at every run start; it is also a bench tool:

    python3 -I tools/px4/param_dump.py                     # tcp:127.0.0.1:5760 (mavlink-router on the Jetson)
    python3 -I tools/px4/param_dump.py --url tcp:<jetson>:5760 --out p.json --version-out v.json

What it sends, and nothing else:
  * COMMAND_LONG MAV_CMD_REQUEST_MESSAGE(AUTOPILOT_VERSION), and the legacy MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES
    when the first one is refused or unanswered;
  * PARAM_REQUEST_LIST, then PARAM_REQUEST_READ (by index) once for every index the list stream lost.
It never sends PARAM_SET, a mode or arm command, or a HEARTBEAT: a GCS heartbeat that stops when this tool exits
would look to PX4 like a lost GCS link (NAV_DLL_ACT). It changes no vehicle state.

Output (both written atomically: temp file, fsync, rename, fsync of the directory):
  --out          {schema, source, url, collected_utc, status, complete, count_expected, count_received, params{NAME: value},
                  types{NAME: MAV_PARAM_TYPE}, ...}, params sorted by name. INT32 values are ints (PX4 encodes them
                  byte-wise in the float field); REAL32 values are the exact float32 value as a JSON number (Python repr:
                  the shortest text that round-trips, the same information as %.17g). A non-finite float is null and
                  listed under non_finite.
  --version-out  {schema, ..., status, flight_sw_version, flight_sw_version_str, flight_custom_version, firmware_git_hash,
                  board_version, vendor_id, product_id, uid, ...}. firmware_git_hash is the 16 hex characters PX4 puts in
                  flight_custom_version (the first 8 bytes of the commit, sent least significant byte first), printed in
                  commit order: the first 16 characters of the hash `ver git` prints, so a prefix of the firmware pin
                  (e.g. 8279fa4be3...) compares directly.
Every failure that can be caught still writes both files, with "status": "unavailable" and the reason.

Exit codes: 0 every parameter and the version were read; 1 files written but incomplete (parameters missing, or no
AUTOPILOT_VERSION); 2 usage; 3 pymavlink not installed; 4 link unreachable or no autopilot heartbeat; 5 an output
file could not be written.

pymavlink is imported only when a link is opened, so the parsing and writing are testable without it
(tools/tests/test_param_dump.py drives them with a fake connection).
"""

from __future__ import annotations

import argparse
import datetime
import json
import math
import os
import signal
import socket
import struct
import sys
import time
from typing import Any, Callable, Dict, List, Optional, Tuple

SCHEMA = 1
DEFAULT_URL = "tcp:127.0.0.1:5760"  # mavlink-router TcpServerPort (deployment/network/mavlink-router.conf.tmpl)
URL_ENV = "DYX3_MAVLINK_URL"
# DERIVED — NOT FROM V1 SPEC: PX4 streams ~900-1000 parameters, each a PARAM_VALUE paced by the MAVLink instance's
# rate; on the rover's Ethernet link that is a few seconds. 20 s leaves room for the per-index re-reads.
DEFAULT_TIMEOUT_S = 20.0
# DERIVED — NOT FROM V1 SPEC: our MAVLink identity. Not QGC's 255/190 (mavlink-router routes by system/component, a
# shared identity would split the replies between QGC and us); 191 = MAV_COMP_ID_ONBOARD_COMPUTER.
DEFAULT_SOURCE_SYSTEM = 245
DEFAULT_SOURCE_COMPONENT = 191
# DERIVED — NOT FROM V1 SPEC, loop pacing only (no vehicle effect): wait for the first PARAM_VALUE before re-sending the
# list once; how long the stream may pause before the missing indices are re-read; how long to wait per re-read;
# the share of the timeout AUTOPILOT_VERSION may use; the time kept back to write the files.
FIRST_PARAM_WAIT_S = 3.0
STREAM_IDLE_S = 2.0
READ_WAIT_S = 0.5
VERSION_SHARE = 0.25
VERSION_MAX_S = 4.0
WRITE_MARGIN_S = 0.5

EXIT_COMPLETE = 0
EXIT_INCOMPLETE = 1
EXIT_USAGE = 2
EXIT_NO_PYMAVLINK = 3
EXIT_UNREACHABLE = 4
EXIT_WRITE_FAILED = 5

# MAVLink common.xml values, here so that parsing needs no pymavlink.
MAV_TYPE_GCS = 6
MAV_AUTOPILOT_INVALID = 8
MAV_AUTOPILOT_PX4 = 12
MAV_CMD_REQUEST_MESSAGE = 512
MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES = 520
MSG_ID_AUTOPILOT_VERSION = 148
MAV_RESULT_ACCEPTED = 0
MAV_RESULT_IN_PROGRESS = 5
PARAM_INDEX_NONE = 65535  # param_index is uint16; PX4 sends -1 for _HASH_CHECK
HASH_PARAM = "_HASH_CHECK"
# MAV_PARAM_TYPE -> (name, struct format of the value in the 4 bytes of param_value; None = does not fit)
PARAM_TYPES: Dict[int, Tuple[str, Optional[str]]] = {
    1: ("UINT8", "<B"),
    2: ("INT8", "<b"),
    3: ("UINT16", "<H"),
    4: ("INT16", "<h"),
    5: ("UINT32", "<I"),
    6: ("INT32", "<i"),
    7: ("UINT64", None),
    8: ("INT64", None),
    9: ("REAL32", "<f"),
    10: ("REAL64", None),
}
FIRMWARE_VERSION_TYPES = {0: "dev", 64: "alpha", 128: "beta", 192: "rc", 255: "official"}

Clock = Callable[[], float]


class HardDeadline(Exception):
    """The overall timeout expired inside a call that did not return in time (SIGALRM backstop)."""


class Interrupted(Exception):
    """SIGINT / SIGTERM (the recorder stops a dump that outlives its run)."""


class LinkError(Exception):
    """The MAVLink link could not be opened, or no autopilot answered on it."""


class NoPymavlink(Exception):
    pass


def utc_now() -> str:
    return datetime.datetime.now(datetime.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


def _text(v: Any) -> str:
    """A MAVLink char[] field as str (pymavlink gives str on Python 3; bytes are tolerated)."""
    if isinstance(v, (bytes, bytearray)):
        v = bytes(v).split(b"\0", 1)[0].decode("ascii", errors="replace")
    return str(v).split("\0", 1)[0]


def _u8_list(v: Any) -> List[int]:
    """A MAVLink uint8_t[] field as a list of ints (pymavlink gives a list; bytes/str are tolerated)."""
    if isinstance(v, str):
        return [ord(c) & 0xFF for c in v]
    return [int(b) & 0xFF for b in v]


def value_bytes(msg: Any) -> bytes:
    """The 4 raw bytes of PARAM_VALUE.param_value.

    Taken from the wire buffer when the message has one: param_value is the first payload field, so it starts right
    after the header (MAVLink 2: 10 bytes, MAVLink 1: 6). Re-packing the unpacked float would quieten a signalling-NaN
    bit pattern and change an INT32 value in that range. Falls back to packing the float.
    """
    get = getattr(msg, "get_msgbuf", None)
    if get is not None:
        try:
            buf = bytes(get())
            hdr = {0xFD: 10, 0xFE: 6}.get(buf[0]) if buf else None
            if hdr is not None and len(buf) >= hdr + 4:
                return buf[hdr : hdr + 4]
        except Exception:  # noqa: BLE001 - any odd buffer: use the float
            pass
    return struct.pack("<f", float(msg.param_value))


def decode_param_value(raw4: bytes, param_type: int) -> Tuple[Any, Optional[str]]:
    """(value, problem). PX4 encodes every type byte-wise into the 4 bytes of the float field."""
    name, fmt = PARAM_TYPES.get(param_type, (None, None))
    if name is None:
        return None, f"unknown MAV_PARAM_TYPE {param_type}"
    if fmt is None:
        return None, f"{name} does not fit the 4-byte PARAM_VALUE field"
    size = struct.calcsize(fmt)
    (v,) = struct.unpack(fmt, raw4[:size])
    if fmt == "<f":
        v = float(v)
        if not math.isfinite(v):
            return None, "non-finite"
    return v, None


def firmware_git_hash(custom_version: Any) -> str:
    """flight_custom_version (8 bytes, PX4 memcpy of a uint64 holding the first 16 hex digits of the commit, little
    endian) -> those 16 hex digits in commit order (what QGC shows and `ver git` begins with)."""
    b = _u8_list(custom_version)[:8]
    return "".join(f"{x:02x}" for x in reversed(b))


def flight_sw_version_str(v: int) -> str:
    major, minor, patch, typ = (v >> 24) & 0xFF, (v >> 16) & 0xFF, (v >> 8) & 0xFF, v & 0xFF
    return f"{major}.{minor}.{patch} {FIRMWARE_VERSION_TYPES.get(typ, f'type {typ}')}"


def _recv(conn: Any, types: List[str], deadline: float, clock: Clock, step: float = 0.25) -> Any:
    remaining = deadline - clock()
    if remaining <= 0:
        return None
    return conn.recv_match(type=types, blocking=True, timeout=min(step, remaining))


def wait_autopilot(conn: Any, deadline: float, clock: Clock, want_system: Optional[int]) -> Tuple[int, int, int]:
    """(system, component, autopilot) of the first HEARTBEAT from an autopilot (never from a GCS such as QGC)."""
    while clock() < deadline:
        m = _recv(conn, ["HEARTBEAT"], deadline, clock)
        if m is None:
            continue
        if m.autopilot == MAV_AUTOPILOT_INVALID or m.type == MAV_TYPE_GCS:
            continue
        if want_system is not None and m.get_srcSystem() != want_system:
            continue
        return m.get_srcSystem(), m.get_srcComponent(), m.autopilot
    raise LinkError("no autopilot HEARTBEAT")


def _from_target(m: Any, ts: int, tc: int) -> bool:
    return m.get_srcSystem() == ts and m.get_srcComponent() == tc


def request_version(conn: Any, ts: int, tc: int, deadline: float, clock: Clock) -> Tuple[Optional[Any], str]:
    """(AUTOPILOT_VERSION message or None, how it was obtained / why not)."""
    tried = []
    attempts = (
        ("MAV_CMD_REQUEST_MESSAGE", MAV_CMD_REQUEST_MESSAGE, float(MSG_ID_AUTOPILOT_VERSION)),
        ("MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES", MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES, 1.0),
    )
    for i, (label, cmd, p1) in enumerate(attempts):
        now = clock()
        if now >= deadline:
            break
        # The first request gets half of what is left, so the legacy one still has time.
        sub = deadline if i == len(attempts) - 1 else now + (deadline - now) / 2.0
        conn.mav.command_long_send(ts, tc, cmd, 0, p1, 0, 0, 0, 0, 0, 0)
        refused = None
        while clock() < sub:
            m = _recv(conn, ["AUTOPILOT_VERSION", "COMMAND_ACK"], sub, clock)
            if m is None or not _from_target(m, ts, tc):
                continue
            if m.get_type() == "AUTOPILOT_VERSION":
                return m, label
            if m.command == cmd and m.result not in (MAV_RESULT_ACCEPTED, MAV_RESULT_IN_PROGRESS):
                refused = m.result
                break
        tried.append(f"{label} " + (f"refused (MAV_RESULT {refused})" if refused is not None else "unanswered"))
    return None, "; ".join(tried) if tried else "no time left for AUTOPILOT_VERSION"


class ParamCollection:
    def __init__(self) -> None:
        self.by_index: Dict[int, Tuple[str, int, bytes]] = {}  # index -> (name, type, raw4)
        self.unindexed: Dict[str, Tuple[int, bytes]] = {}  # name -> (type, raw4) for index 65535 replies
        self.count: Optional[int] = None
        self.hash_check: Optional[int] = None
        self.list_requests = 0
        self.reread_requested = 0
        self.reread_recovered = 0

    def missing(self) -> List[int]:
        if self.count is None:
            return []
        return [i for i in range(self.count) if i not in self.by_index]

    def complete(self) -> bool:
        return self.count is not None and not self.missing()

    def add(self, m: Any) -> Optional[int]:
        """Store one PARAM_VALUE; returns its index when it was new."""
        name = _text(m.param_id)
        raw = value_bytes(m)
        if name == HASH_PARAM:
            (self.hash_check,) = struct.unpack("<I", raw)
            return None
        self.count = int(m.param_count)
        idx = int(m.param_index)
        if idx == PARAM_INDEX_NONE or idx >= self.count:
            self.unindexed[name] = (int(m.param_type), raw)
            return None
        new = idx not in self.by_index
        self.by_index[idx] = (name, int(m.param_type), raw)
        return idx if new else None


def collect_params(conn: Any, ts: int, tc: int, deadline: float, clock: Clock, pc: ParamCollection) -> None:
    """PARAM_REQUEST_LIST, take the stream until it is complete or pauses, then PARAM_REQUEST_READ every missing
    index once. Fills `pc` in place, so a deadline or an interrupt keeps what was already received."""

    def take(until: float) -> Optional[int]:
        m = _recv(conn, ["PARAM_VALUE"], until, clock)
        if m is None or not _from_target(m, ts, tc):
            return None
        return pc.add(m)

    for _attempt in range(2):  # the list is re-sent once if nothing at all came back
        conn.mav.param_request_list_send(ts, tc)
        pc.list_requests += 1
        first_until = min(deadline, clock() + FIRST_PARAM_WAIT_S)
        while clock() < first_until and pc.count is None:
            take(first_until)
        if pc.count is not None:
            break
    if pc.count is None:
        return
    last_new = clock()
    while clock() < deadline and not pc.complete():
        if take(deadline) is not None:
            last_new = clock()
        elif clock() - last_new >= STREAM_IDLE_S:
            break
    if pc.complete():
        # PX4 ends the list stream with _HASH_CHECK (a hash over every value); it usually follows the last index.
        until = min(deadline, clock() + READ_WAIT_S)
        while clock() < until and pc.hash_check is None:
            take(until)
        return
    for idx in pc.missing():  # one re-read per missing index
        if clock() >= deadline:
            break
        if idx in pc.by_index:  # arrived meanwhile
            continue
        conn.mav.param_request_read_send(ts, tc, b"", idx)
        pc.reread_requested += 1
        until = min(deadline, clock() + READ_WAIT_S)
        while clock() < until and idx not in pc.by_index:
            take(until)
        if idx in pc.by_index:
            pc.reread_recovered += 1


def params_document(url: str, collected_utc: str, pc: Optional[ParamCollection], target: Optional[Tuple[int, int, int]],
                    reason: Optional[str], duration_s: float) -> Dict[str, Any]:
    doc: Dict[str, Any] = {"schema": SCHEMA, "source": "mavlink", "url": url, "collected_utc": collected_utc}
    if pc is None or pc.count is None:
        doc.update({
            "status": "unavailable",
            "reason": reason or "no PARAM_VALUE received",
            "complete": False,
            "count_expected": None,
            "count_received": 0,
            "params": {},
        })
        if target is not None:
            doc["target"] = {"system": target[0], "component": target[1], "autopilot": target[2]}
        doc["duration_s"] = round(duration_s, 3)
        return doc
    values: Dict[str, Any] = {}
    types: Dict[str, str] = {}
    problems: Dict[str, str] = {}
    entries = [(name, typ, raw) for (name, typ, raw) in pc.by_index.values()]
    entries += [(name, typ, raw) for name, (typ, raw) in pc.unindexed.items()]
    for name, typ, raw in entries:
        v, problem = decode_param_value(raw, typ)
        values[name] = v
        types[name] = PARAM_TYPES.get(typ, (f"type {typ}", None))[0]
        if problem:
            problems[name] = problem
    missing = pc.missing()
    complete = not missing
    doc.update({
        "status": "complete" if complete else "incomplete",
        "complete": complete,
        "count_expected": pc.count,
        "count_received": len(pc.by_index),
    })
    if reason:
        doc["reason"] = reason
    if missing:
        doc["missing_indices"] = missing
    doc.update({
        "target": {"system": target[0], "component": target[1], "autopilot": target[2]} if target else None,
        "param_encoding": "bytewise (PX4)",
        "hash_check": pc.hash_check,
        "list_requests": pc.list_requests,
        "reread_requested": pc.reread_requested,
        "reread_recovered": pc.reread_recovered,
        "duration_s": round(duration_s, 3),
        "params": {k: values[k] for k in sorted(values)},
        "types": {k: types[k] for k in sorted(types)},
    })
    if problems:
        doc["value_problems"] = {k: problems[k] for k in sorted(problems)}
    return doc


def version_document(url: str, collected_utc: str, msg: Any, how: str, target: Optional[Tuple[int, int, int]]) -> Dict[str, Any]:
    doc: Dict[str, Any] = {"schema": SCHEMA, "source": "mavlink", "url": url, "collected_utc": collected_utc}
    if target is not None:
        doc["target"] = {"system": target[0], "component": target[1], "autopilot": target[2]}
    if msg is None:
        doc.update({"status": "unavailable", "reason": how})
        return doc
    custom = _u8_list(msg.flight_custom_version)[:8]
    doc.update({
        "status": "ok",
        "requested_with": how,
        "flight_sw_version": int(msg.flight_sw_version),
        "flight_sw_version_str": flight_sw_version_str(int(msg.flight_sw_version)),
        "flight_custom_version": "".join(f"{x:02x}" for x in custom),
        "firmware_git_hash": firmware_git_hash(custom),
        "middleware_sw_version": int(msg.middleware_sw_version),
        "os_sw_version": int(msg.os_sw_version),
        "os_custom_version": "".join(f"{x:02x}" for x in _u8_list(msg.os_custom_version)[:8]),
        "board_version": int(msg.board_version),
        "vendor_id": int(msg.vendor_id),
        "product_id": int(msg.product_id),
        # uint64: as a hex string, JSON readers lose integers above 2^53
        "uid": f"{int(msg.uid):016x}",
        "capabilities": int(msg.capabilities),
    })
    uid2 = getattr(msg, "uid2", None)
    if uid2 is not None and any(_u8_list(uid2)):
        doc["uid2"] = "".join(f"{x:02x}" for x in _u8_list(uid2))
    return doc


def unavailable(url: str, collected_utc: str, reason: str) -> Dict[str, Any]:
    return {
        "schema": SCHEMA, "source": "mavlink", "url": url, "collected_utc": collected_utc,
        "status": "unavailable", "reason": reason, "complete": False,
        "count_expected": None, "count_received": 0, "params": {},
    }


def write_json_atomic(path: str, doc: Dict[str, Any]) -> None:
    text = json.dumps(doc, indent=2, allow_nan=False) + "\n"
    d = os.path.dirname(os.path.abspath(path))
    tmp = os.path.join(d, f".{os.path.basename(path)}.tmp.{os.getpid()}")
    try:
        with open(tmp, "w", encoding="ascii") as f:
            f.write(text)
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp, path)
    except BaseException:
        try:
            os.unlink(tmp)
        except OSError:
            pass
        raise
    fd = os.open(d, os.O_RDONLY)
    try:
        os.fsync(fd)
    finally:
        os.close(fd)


def open_mavlink(url: str, source_system: int, source_component: int, connect_timeout_s: float) -> Any:
    """The default connector: pymavlink, MAVLink 2, common dialect. Imported here, never at module load."""
    os.environ["MAVLINK20"] = "1"
    try:
        from pymavlink import mavutil  # noqa: PLC0415 - lazy on purpose
    except ImportError as e:
        raise NoPymavlink(f"pymavlink is not installed for {sys.executable} ({e}); the installer puts it in "
                          "/opt/dyx3/third_party/pymavlink (installer/lib/dependencies.sh)") from e
    old = socket.getdefaulttimeout()
    socket.setdefaulttimeout(max(0.1, connect_timeout_s))  # a TCP connect to an absent host must not hang
    try:
        return mavutil.mavlink_connection(url, source_system=source_system, source_component=source_component,
                                          dialect="common", autoreconnect=False)
    except Exception as e:  # noqa: BLE001 - every connect failure is "unreachable"
        raise LinkError(f"cannot open {url}: {e}") from e
    finally:
        socket.setdefaulttimeout(old)


Connector = Callable[[str, int, int, float], Any]


def dump(url: str, out: str, version_out: str, timeout_s: float, connect: Connector = open_mavlink,
         clock: Clock = time.monotonic, target_system: Optional[int] = None, target_component: Optional[int] = None,
         source_system: int = DEFAULT_SOURCE_SYSTEM, source_component: int = DEFAULT_SOURCE_COMPONENT,
         log: Callable[[str], None] = lambda s: print(s, file=sys.stderr),
         hold_signals: Callable[[], None] = lambda: None) -> int:
    """Run one dump and write both files. Returns the exit code. Never raises for link or vehicle trouble.

    `hold_signals` is called once the read is over (main: cancel the SIGALRM backstop and defer SIGINT/SIGTERM), so a
    signal cannot abort the writing of what was read.
    """
    start = clock()
    deadline = start + max(0.0, timeout_s - WRITE_MARGIN_S)
    stamp = utc_now()
    conn = None
    target: Optional[Tuple[int, int, int]] = None
    pc: Optional[ParamCollection] = None
    vmsg, vhow = None, "not requested"
    reason: Optional[str] = None
    code = EXIT_COMPLETE
    try:
        conn = connect(url, source_system, source_component, max(0.1, deadline - clock()))
        try:
            ap = wait_autopilot(conn, deadline, clock, target_system)
        except LinkError:
            raise LinkError(f"no autopilot HEARTBEAT on {url} within {timeout_s:.1f} s (is the Ethernet MAVLink "
                            "instance enabled on the FCU, MAV_2_CONFIG, and mavlink-router running?)") from None
        target = (ap[0], target_component if target_component is not None else ap[1], ap[2])
        if ap[2] != MAV_AUTOPILOT_PX4:
            log(f"param_dump: autopilot type {ap[2]} is not PX4 ({MAV_AUTOPILOT_PX4}); values are decoded byte-wise")
        vdeadline = min(deadline, clock() + min(VERSION_MAX_S, VERSION_SHARE * timeout_s))
        vmsg, vhow = request_version(conn, target[0], target[1], vdeadline, clock)
        pc = ParamCollection()
        collect_params(conn, target[0], target[1], deadline, clock, pc)
        if pc.count is None:
            reason = f"no PARAM_VALUE from {target[0]}/{target[1]} after {pc.list_requests} PARAM_REQUEST_LIST"
        elif not pc.complete():
            reason = (f"{len(pc.missing())} of {pc.count} parameters missing after the list and one re-read each "
                      f"(timeout {timeout_s:.1f} s)")
    except NoPymavlink as e:
        reason, code = str(e), EXIT_NO_PYMAVLINK
    except LinkError as e:
        reason, code = f"MAVLink link unreachable: {e}", EXIT_UNREACHABLE
    except HardDeadline:
        reason = f"timeout: {timeout_s:.1f} s elapsed before the read finished"
    except Interrupted as e:
        reason = f"interrupted ({e}) before the read finished"
    except Exception as e:  # noqa: BLE001 - a run must never lose the reason
        reason = f"unexpected error: {type(e).__name__}: {e}"
    finally:
        hold_signals()
        if conn is not None:
            try:
                conn.close()
            except Exception:  # noqa: BLE001
                pass
    duration = clock() - start
    if code in (EXIT_NO_PYMAVLINK, EXIT_UNREACHABLE):
        pdoc = unavailable(url, stamp, reason or "unavailable")
        vdoc = version_document(url, stamp, None, reason or "unavailable", target)
    else:
        pdoc = params_document(url, stamp, pc, target, reason, duration)
        if vmsg is None and vhow == "not requested" and reason:
            vhow = reason
        vdoc = version_document(url, stamp, vmsg, vhow, target)
        if not pdoc["complete"] or vmsg is None:
            code = EXIT_INCOMPLETE
    failed = []
    for path, doc in ((out, pdoc), (version_out, vdoc)):
        try:
            write_json_atomic(path, doc)
        except OSError as e:
            failed.append(f"{path}: {e}")
    summary = (f"param_dump: {pdoc.get('status')} {pdoc.get('count_received')}/{pdoc.get('count_expected')} parameters"
               f", firmware {vdoc.get('firmware_git_hash', 'unavailable')}, {duration:.1f} s")
    log(summary + (f" ({reason})" if reason else ""))
    if failed:
        log("param_dump: could not write " + "; ".join(failed))
        return EXIT_WRITE_FAILED
    return code


def _hold_signals() -> None:
    """Only the files are left to write: no SIGALRM, and SIGINT/SIGTERM are ignored (the recorder escalates to
    SIGKILL after its grace period anyway)."""
    signal.setitimer(signal.ITIMER_REAL, 0)
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    signal.signal(signal.SIGTERM, signal.SIG_IGN)


def _install_signal_guards(timeout_s: float) -> None:
    def on_alarm(_s: int, _f: Any) -> None:
        signal.setitimer(signal.ITIMER_REAL, 0)
        raise HardDeadline()

    def on_stop(s: int, _f: Any) -> None:
        signal.setitimer(signal.ITIMER_REAL, 0)
        raise Interrupted(signal.Signals(s).name)

    signal.signal(signal.SIGALRM, on_alarm)
    signal.signal(signal.SIGINT, on_stop)
    signal.signal(signal.SIGTERM, on_stop)
    # Backstop for a call that blocks inside pymavlink: the loops themselves stop WRITE_MARGIN_S before the timeout.
    signal.setitimer(signal.ITIMER_REAL, max(0.1, timeout_s))


def parse_args(argv: Optional[List[str]]) -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Read every PX4 parameter and AUTOPILOT_VERSION over MAVLink (read-only).")
    p.add_argument("--url", default=os.environ.get(URL_ENV) or DEFAULT_URL,
                   help=f"pymavlink connection string (default ${URL_ENV}, else {DEFAULT_URL})")
    p.add_argument("--timeout", type=float, default=DEFAULT_TIMEOUT_S, help="overall limit in seconds (default 20)")
    p.add_argument("--out", default="params_fcu.json")
    p.add_argument("--version-out", default="versions_fcu.json")
    p.add_argument("--target-system", type=int, default=None, help="default: the first autopilot heartbeat")
    p.add_argument("--target-component", type=int, default=None)
    p.add_argument("--source-system", type=int, default=DEFAULT_SOURCE_SYSTEM)
    p.add_argument("--source-component", type=int, default=DEFAULT_SOURCE_COMPONENT)
    a = p.parse_args(argv)
    if not math.isfinite(a.timeout) or a.timeout <= 0:
        p.error("--timeout must be a positive number of seconds")
    return a


def main(argv: Optional[List[str]] = None) -> int:
    a = parse_args(argv)
    _install_signal_guards(a.timeout)
    try:
        return dump(a.url, a.out, a.version_out, a.timeout, target_system=a.target_system,
                    target_component=a.target_component, source_system=a.source_system,
                    source_component=a.source_component, hold_signals=_hold_signals)
    finally:
        signal.setitimer(signal.ITIMER_REAL, 0)


if __name__ == "__main__":
    sys.exit(main())
