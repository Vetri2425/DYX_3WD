#!/usr/bin/env python3
"""Mission bag metrics: the numbers the acceptance gates (architecture section 10) and the field ladder need, from recorder bags.

Input: one recorder run directory (``<run>/rosbag2*/*.db3[.zstd]``, ``manifest.json``, ``summary.json``, ``versions.json``) or a
directory holding several run directories (a mission, or a whole ``3WD_PROD/Bags/<date>/`` day: runs are grouped into missions by
``manifest.json`` ``mission_id`` and start order).

No ROS dependency. The rosbag2 sqlite3 ``topics``/``messages`` tables are read directly and every ``dyx3_interfaces/*`` message is
decoded from CDR (little-endian, 4-byte encapsulation header, alignment relative to the payload start) by a codec driven by the
``.msg`` definitions. The definitions are, by default, those of the stack version that recorded the bag (``versions.json``
``stack_sha`` through ``git show``), falling back to the working tree; ``--msgdir`` or ``--stack-sha`` force a set. Topics of any
other package (``rcl_interfaces``, ``px4_msgs``) are skipped.

Output: a Markdown report on stdout (or JSON with ``--json``) and ``metrics.json`` next to the data: in every run directory (what that
bag alone says) and, for a directory of runs, in that directory (each mission computed over its runs merged in time, so a pivot that
starts at the end of one bag and finishes in the next is one episode).

Units in JSON are SI (m, rad, s, Hz) unless the key says otherwise (``_ms``, ``_us``, ``_cm``); the Markdown shows cm, deg and ms.
Everything here is the controller's own view (RppStatus cross-track is measured against the path the controller holds); it is
NOT surveyed truth. Gate rows that need surveyed points or the PX4 ULog are listed in docs/analysis/README.md as not provided.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import re
import shutil
import sqlite3
import struct
import subprocess
import sys
import tempfile
from collections.abc import Callable, Iterable
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import numpy as np

SCHEMA = 1
TOOL = "tools/analysis/mission_bag_metrics.py"
REPO_ROOT = Path(__file__).resolve().parents[2]
MSG_SUBDIR = "ros2_ws/src/dyx3_interfaces/msg"
PACKAGE = "dyx3_interfaces"

# Recorder topic names (dyx3_bringup / dyx3_recorder topic list).
T_VEHICLE = "/dyx3/vehicle_state"
T_SETPOINT = "/dyx3/rpp/motion_setpoint"
T_GUARD_CMD = "/dyx3/motion_guard/command"
T_GUARD_STATUS = "/dyx3/motion_guard/status"
T_LINK = "/dyx3/px4_link/status"
T_RPP = "/dyx3/rpp/status"
T_MISSION = "/dyx3/mission/state"
T_POINT = "/dyx3/mission/point_result"
T_SAFETY = "/dyx3/safety_gate"
T_RTK = "/dyx3/rtk_status"
T_ESTIMATOR = "/dyx3/estimator_health"

# RppStatus.state values (RppStatus.msg constants; resolved by name at run time, these are the fallbacks).
RPP_TRACKING, RPP_STOPPING, RPP_PIVOTING, RPP_CREEPING = 1, 2, 3, 4

# Analysis windows. These are the definitions requested for the field ladder (docs/analysis/README.md), not controller tuning.
STEADY_SPEED_FRACTION = 0.8   # STEADY = TRACKING and commanded speed >= 0.8 x the run's max commanded speed
CORNER_ENTRY_M = 0.5          # first 0.5 m of path_travel_m of a TRACKING interval
BRAKING_TAIL_M = 1.0          # last 1.0 m of path_travel_m before the next STOPPING
FULL_RATE_YAW_RADPS = 0.4     # |commanded yaw rate| at which a pivot is "full rate" (segment_nominal_pivot_rate_rad_s)
ENDPOINT_TAIL_S = 6.0         # window for the max measured speed at the endpoint
# The precise-stop (CREEP) timeout is 8 s (docs/analysis/2026-10-10_last_two_missions_controller_robustness.md section 4);
# 7.9 s leaves one 20 ms tick of margin for the state-interval quantisation of a 50 Hz status stream.
CREEP_TIMEOUT_FLAG_S = 7.9
SIGN_EPS = 1e-6               # |commanded speed| below this is "zero" for the sign-reversal count (a brake command is exactly 0)


# --------------------------------------------------------------------------------------------------------------------------------
# Message definitions
# --------------------------------------------------------------------------------------------------------------------------------

PRIMITIVES: dict[str, tuple[str, int]] = {
    "bool": ("?", 1), "byte": ("B", 1), "char": ("B", 1),
    "uint8": ("B", 1), "int8": ("b", 1),
    "uint16": ("H", 2), "int16": ("h", 2),
    "uint32": ("I", 4), "int32": ("i", 4),
    "uint64": ("Q", 8), "int64": ("q", 8),
    "float32": ("f", 4), "float64": ("d", 8),
}
STRING_TYPES = ("string",)
BUILTIN_DEFS = {
    "builtin_interfaces/Time": "int32 sec\nuint32 nanosec\n",
    "builtin_interfaces/Duration": "int32 sec\nuint32 nanosec\n",
}

_FIELD_RE = re.compile(r"^(?P<type>[A-Za-z_][A-Za-z0-9_/]*(?:<=\d+)?)(?P<arr>\[(?:<=)?\d*\])?\s+(?P<name>[A-Za-z_][A-Za-z0-9_]*)"
                       r"(?:\s+(?P<default>.+))?$")
_CONST_RE = re.compile(r"^(?P<type>[A-Za-z_][A-Za-z0-9_]*)\s+(?P<name>[A-Za-z_][A-Za-z0-9_]*)\s*=\s*(?P<value>.+)$")


@dataclass(frozen=True)
class FieldDef:
    name: str
    type: str                 # primitive, "string", or a fully qualified "pkg/Name"
    array: int | None = None  # None scalar, N fixed array, -1 sequence (bounded or unbounded)


@dataclass
class MsgDef:
    name: str                 # fully qualified "pkg/Name"
    fields: list[FieldDef]
    constants: dict[str, Any]


def _qualify(type_name: str, package: str) -> str:
    """'Time' (same package), 'pkg/Name' or 'pkg/msg/Name' -> 'pkg/Name'; primitives and strings unchanged."""
    base = type_name.split("<=")[0]
    if base in PRIMITIVES or base in STRING_TYPES:
        return base
    parts = base.split("/")
    if len(parts) == 1:
        return f"{package}/{parts[0]}"
    if len(parts) == 3 and parts[1] == "msg":
        return f"{parts[0]}/{parts[2]}"
    if len(parts) == 2:
        return base
    raise ValueError(f"unsupported type name {type_name!r}")


def _const_value(type_name: str, text: str) -> Any:
    text = text.strip()
    if type_name in STRING_TYPES:
        return text.strip("'\"")
    if type_name == "bool":
        return text.lower() in ("true", "1")
    if type_name.startswith("float"):
        return float(text)
    return int(text, 0)


def parse_msg(text: str, name: str) -> MsgDef:
    """Parse a ROS 2 .msg body. Comments, blank lines and default values are ignored; constants are collected."""
    package = name.split("/")[0]
    fields: list[FieldDef] = []
    constants: dict[str, Any] = {}
    for raw in text.splitlines():
        line = raw.split("#", 1)[0].strip()
        if not line:
            continue
        c = _CONST_RE.match(line)
        if c and "[" not in c.group("type"):
            constants[c.group("name")] = _const_value(c.group("type"), c.group("value"))
            continue
        m = _FIELD_RE.match(line)
        if not m:
            raise ValueError(f"{name}: cannot parse line {raw!r}")
        arr = m.group("arr")
        if arr is None:
            count = None
        else:
            inner = arr[1:-1]
            count = -1 if (inner == "" or inner.startswith("<=")) else int(inner)
        fields.append(FieldDef(m.group("name"), _qualify(m.group("type"), package), count))
    return MsgDef(name, fields, constants)


class MsgDefs:
    """A set of message definitions, loaded lazily through `loader(qualified_name) -> .msg text`."""

    def __init__(self, loader: Callable[[str], str], names: Iterable[str], source: str):
        self._loader = loader
        self._cache: dict[str, MsgDef] = {}
        self.names = sorted(names)  # dyx3_interfaces message names available in this set
        self.source = source

    @classmethod
    def from_dir(cls, msgdir: str | os.PathLike) -> MsgDefs:
        d = Path(msgdir)
        if not d.is_dir():
            raise FileNotFoundError(f"message definition directory not found: {d}")

        def load(qname: str) -> str:
            return (d / (qname.split("/")[1] + ".msg")).read_text(encoding="utf-8")

        return cls(load, [p.stem for p in d.glob("*.msg")], f"dir:{d}")

    @classmethod
    def from_git(cls, repo: str | os.PathLike, sha: str) -> MsgDefs:
        """Definitions as they were at `sha` (read-only `git show` / `git ls-tree`)."""
        def git(*argv: str) -> str:
            return subprocess.run(["git", "-C", str(repo), *argv], check=True, capture_output=True, text=True).stdout

        listing = git("ls-tree", "--name-only", f"{sha}:{MSG_SUBDIR}")
        names = [n[:-4] for n in listing.split() if n.endswith(".msg")]
        if not names:
            raise FileNotFoundError(f"no .msg files at {sha}:{MSG_SUBDIR}")

        def load(qname: str) -> str:
            return git("show", f"{sha}:{MSG_SUBDIR}/{qname.split('/')[1]}.msg")

        return cls(load, names, f"git:{sha}")

    def get(self, type_name: str) -> MsgDef:
        qname = _qualify(type_name, PACKAGE)
        if qname not in self._cache:
            if qname in BUILTIN_DEFS:
                text = BUILTIN_DEFS[qname]
            elif qname.startswith(PACKAGE + "/"):
                text = self._loader(qname)
            else:
                raise KeyError(f"no definition for {qname} (only {PACKAGE} and builtin_interfaces are known)")
            self._cache[qname] = parse_msg(text, qname)
        return self._cache[qname]

    def enum_name(self, type_name: str, prefix: str, value: Any) -> str:
        """Name of the constant `prefix*` equal to `value` (prefix stripped), or the value as text."""
        if value is None:
            return "n/a"
        try:
            consts = self.get(type_name).constants
        except (KeyError, OSError, subprocess.CalledProcessError):
            consts = {}
        hits = [k[len(prefix):] for k, v in consts.items() if k.startswith(prefix) and v == value]
        return hits[0] if len(hits) == 1 else str(value)


# --------------------------------------------------------------------------------------------------------------------------------
# CDR codec (XCDR1 / classic CDR, little-endian, as rmw_fastrtps and rmw_cyclonedds write it on ROS 2 Humble)
# --------------------------------------------------------------------------------------------------------------------------------

CDR_LE_HEADER = b"\x00\x01\x00\x00"


class TruncatedBuffer(Exception):
    """A read went past the end of the serialized message."""


class CdrReader:
    def __init__(self, buf: bytes):
        if len(buf) < 4:
            raise ValueError("CDR buffer shorter than its encapsulation header")
        if buf[0:2] != b"\x00\x01":
            raise ValueError(f"unsupported CDR encapsulation {buf[0:2].hex()} (only CDR_LE 0001)")
        self.buf = buf
        self.pos = 4

    def _align(self, n: int) -> None:
        r = (self.pos - 4) % n
        if r:
            self.pos += n - r

    def _take(self, n: int) -> int:
        start = self.pos
        if start + n > len(self.buf):
            raise TruncatedBuffer
        self.pos += n
        return start

    def prim(self, t: str) -> Any:
        fmt, size = PRIMITIVES[t]
        self._align(size)
        return struct.unpack_from("<" + fmt, self.buf, self._take(size))[0]

    def prim_array(self, t: str, count: int) -> list:
        fmt, size = PRIMITIVES[t]
        if count == 0:
            return []
        self._align(size)
        return list(struct.unpack_from(f"<{count}{fmt}", self.buf, self._take(size * count)))

    def string(self) -> str:
        n = self.prim("uint32")
        if n == 0:
            return ""
        start = self._take(n)
        raw = self.buf[start:start + n]
        if raw[-1:] != b"\x00":
            raise ValueError("CDR string is not NUL-terminated")
        return raw[:-1].decode("utf-8", "replace")

    @property
    def remaining(self) -> int:
        return len(self.buf) - self.pos


class CdrWriter:
    def __init__(self) -> None:
        self.buf = bytearray(CDR_LE_HEADER)

    def _align(self, n: int) -> None:
        r = (len(self.buf) - 4) % n
        if r:
            self.buf.extend(b"\x00" * (n - r))

    def prim(self, t: str, v: Any) -> None:
        fmt, size = PRIMITIVES[t]
        self._align(size)
        self.buf.extend(struct.pack("<" + fmt, v))

    def prim_array(self, t: str, values: list) -> None:
        if not values:
            return
        fmt, size = PRIMITIVES[t]
        self._align(size)
        self.buf.extend(struct.pack(f"<{len(values)}{fmt}", *values))

    def string(self, s: str) -> None:
        raw = s.encode("utf-8") + b"\x00"
        self.prim("uint32", len(raw))
        self.buf.extend(raw)


def _read_value(r: CdrReader, defs: MsgDefs, t: str, count: int | None) -> Any:
    if count is None:
        if t in PRIMITIVES:
            return r.prim(t)
        if t in STRING_TYPES:
            return r.string()
        return _read_struct(r, defs, t)
    n = r.prim("uint32") if count < 0 else count
    if t in PRIMITIVES:
        return r.prim_array(t, n)
    return [_read_value(r, defs, t, None) for _ in range(n)]


def _read_struct(r: CdrReader, defs: MsgDefs, t: str) -> dict:
    return {f.name: _read_value(r, defs, f.type, f.array) for f in defs.get(t).fields}


@dataclass
class DecodeResult:
    value: dict
    missing: list[str]   # trailing top-level fields absent from the buffer (the bag predates them)
    extra_bytes: int     # bytes left after the last field (>= 4: the bag has fields these definitions do not know)


def decode(defs: MsgDefs, type_name: str, buf: bytes, tolerant: bool = True) -> DecodeResult:
    """Decode one serialized message. With `tolerant`, a buffer that ends at a top-level field boundary yields the fields present
    and lists the rest as missing (None), which is how an older bag reads with newer append-only definitions."""
    r = CdrReader(bytes(buf))
    out: dict = {}
    fields = defs.get(type_name).fields
    for i, f in enumerate(fields):
        mark = r.pos
        try:
            out[f.name] = _read_value(r, defs, f.type, f.array)
        except TruncatedBuffer:
            if not tolerant:
                raise
            r.pos = mark
            missing = [g.name for g in fields[i:]]
            for name in missing:
                out[name] = None
            return DecodeResult(out, missing, 0)
    return DecodeResult(out, [], r.remaining)


def _write_value(w: CdrWriter, defs: MsgDefs, t: str, count: int | None, v: Any) -> None:
    if count is None:
        if t in PRIMITIVES:
            w.prim(t, v)
        elif t in STRING_TYPES:
            w.string(v)
        else:
            _write_struct(w, defs, t, v)
        return
    values = list(v)
    if count < 0:
        w.prim("uint32", len(values))
    elif len(values) != count:
        raise ValueError(f"fixed array of {count} {t} given {len(values)} values")
    if t in PRIMITIVES:
        w.prim_array(t, values)
    else:
        for item in values:
            _write_value(w, defs, t, None, item)


def _write_struct(w: CdrWriter, defs: MsgDefs, t: str, v: dict) -> None:
    for f in defs.get(t).fields:
        _write_value(w, defs, f.type, f.array, v[f.name])


def encode(defs: MsgDefs, type_name: str, value: dict) -> bytes:
    """Serialize `value` (a dict per the definition, nested messages as dicts) to CDR_LE with its encapsulation header."""
    w = CdrWriter()
    _write_struct(w, defs, type_name, value)
    return bytes(w.buf)


def default_value(defs: MsgDefs, type_name: str) -> dict:
    """A default-constructed message: zeros, False, empty strings and sequences, zero-filled fixed arrays, nested defaults."""
    def one(t: str) -> Any:
        if t == "bool":
            return False
        if t in PRIMITIVES:
            return 0.0 if t.startswith("float") else 0
        if t in STRING_TYPES:
            return ""
        return default_value(defs, t)

    out: dict = {}
    for f in defs.get(type_name).fields:
        if f.array is None:
            out[f.name] = one(f.type)
        elif f.array < 0:
            out[f.name] = []
        else:
            out[f.name] = [one(f.type) for _ in range(f.array)]
    return out


# --------------------------------------------------------------------------------------------------------------------------------
# rosbag2 reading
# --------------------------------------------------------------------------------------------------------------------------------

ZSTD_MAGIC = b"\x28\xb5\x2f\xfd"


def _zstd_decompress_file(src: Path, dst: Path) -> None:
    try:
        import zstandard  # optional: CI does not install it; tests use plain .db3
    except ImportError:
        zstandard = None
    if zstandard is not None:
        with open(src, "rb") as fi, open(dst, "wb") as fo:
            zstandard.ZstdDecompressor().copy_stream(fi, fo)
        return
    exe = shutil.which("zstd")
    if exe:
        with open(dst, "wb") as fo:
            subprocess.run([exe, "-d", "-c", str(src)], check=True, stdout=fo)
        return
    raise RuntimeError(f"{src} is zstd-compressed and neither the python 'zstandard' module nor the 'zstd' tool is available: "
                       f"pip install zstandard, or decompress it first (zstd -d {src.name})")


def _zstd_decompress_bytes(blob: bytes) -> bytes:
    try:
        import zstandard
    except ImportError as e:
        raise RuntimeError("message-mode zstd compression needs the python 'zstandard' module") from e
    return zstandard.ZstdDecompressor().decompress(blob, max_output_size=64 << 20)


def bag_files(run_dir: Path) -> list[Path]:
    """The sqlite splits of a run: every rosbag2* directory (bag restarts), plain .db3 preferred over .db3.zstd."""
    out: list[Path] = []
    for bag_dir in sorted(p for p in run_dir.iterdir() if p.is_dir() and p.name.startswith("rosbag2")):
        plain = {p.name: p for p in bag_dir.glob("*.db3")}
        for z in bag_dir.glob("*.db3.zstd"):
            plain.setdefault(z.name[:-5], z)

        def split_index(p: Path) -> tuple:
            m = re.search(r"_(\d+)\.db3", p.name)
            return (int(m.group(1)) if m else 0, p.name)

        out.extend(sorted(plain.values(), key=split_index))
    return out


@dataclass
class TopicStats:
    type: str
    n: int = 0
    failed: int = 0
    truncated: int = 0   # decoded with trailing fields missing
    extra: int = 0       # >= 4 bytes left over
    missing_fields: list[str] = field(default_factory=list)
    errors: list[str] = field(default_factory=list)


@dataclass
class Series:
    """All messages of one topic, in receipt order. t_ns = rosbag2 receipt timestamp (recorder clock)."""
    topic: str
    type: str
    t_ns: np.ndarray
    msgs: list[dict]
    _cols: dict = field(default_factory=dict, repr=False)

    def __len__(self) -> int:
        return len(self.msgs)

    def has(self, name: str) -> bool:
        return any(m.get(name) is not None for m in self.msgs)

    def col(self, name: str) -> np.ndarray:
        """Numeric column, NaN where the field is missing (older definitions)."""
        if name not in self._cols:
            vals = [m.get(name) for m in self.msgs]
            self._cols[name] = np.array([np.nan if v is None else float(v) for v in vals], dtype=float)
        return self._cols[name]

    def stamp_ns(self, name: str) -> np.ndarray:
        """builtin_interfaces/Time column as int64 ns (0 where missing)."""
        out = np.zeros(len(self.msgs), dtype=np.int64)
        for i, m in enumerate(self.msgs):
            v = m.get(name)
            if v is not None:
                out[i] = int(v["sec"]) * 1_000_000_000 + int(v["nanosec"])
        return out

    def values(self, name: str) -> list:
        return [m.get(name) for m in self.msgs]

    @property
    def t(self) -> np.ndarray:
        return self.t_ns.astype(float) / 1e9


def _empty_series(topic: str) -> Series:
    return Series(topic, "", np.zeros(0, dtype=np.int64), [])


@dataclass
class RunData:
    run_dir: Path
    manifest: dict
    summary: dict
    versions: dict
    defs: MsgDefs
    defs_note: str
    topics: dict[str, Series]
    stats: dict[str, TopicStats]
    skipped_topics: dict[str, str]
    raw: dict[str, list[tuple[int, dict]]] = field(default_factory=dict, repr=False)

    @property
    def run_id(self) -> str:
        return str(self.manifest.get("run_id") or self.run_dir.name)

    def series(self, topic: str) -> Series:
        return self.topics.get(topic) or _empty_series(topic)

    @property
    def t_first_ns(self) -> int | None:
        firsts = [int(s.t_ns[0]) for s in self.topics.values() if len(s)]
        return min(firsts) if firsts else None

    @property
    def t_last_ns(self) -> int | None:
        lasts = [int(s.t_ns[-1]) for s in self.topics.values() if len(s)]
        return max(lasts) if lasts else None


def _read_json(p: Path) -> dict:
    try:
        return json.loads(p.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return {}


def resolve_defs(run_dir: Path, msgdir: str | None, stack_sha: str | None, repo: Path) -> tuple[MsgDefs, str]:
    """--msgdir > --stack-sha > versions.json stack_sha via git (when this checkout has it) > the working tree."""
    if msgdir:
        return MsgDefs.from_dir(msgdir), "--msgdir"
    sha = stack_sha
    note = "--stack-sha"
    if not sha:
        sha = _read_json(run_dir / "versions.json").get("stack_sha")
        note = "versions.json stack_sha"
    if sha:
        try:
            return MsgDefs.from_git(repo, sha), note
        except (OSError, subprocess.CalledProcessError, FileNotFoundError) as e:
            if stack_sha:
                raise RuntimeError(f"--stack-sha {stack_sha}: definitions not readable from {repo}: {e}") from e
            fallback = MsgDefs.from_dir(repo / MSG_SUBDIR)
            return fallback, f"working tree (stack {sha[:10]} not in this checkout; trailing fields may be missing)"
    return MsgDefs.from_dir(repo / MSG_SUBDIR), "working tree (no versions.json stack_sha)"


def load_run(run_dir: str | os.PathLike, defs: MsgDefs | None = None, defs_note: str = "given",
             msgdir: str | None = None, stack_sha: str | None = None, repo: Path = REPO_ROOT) -> RunData:
    run_dir = Path(run_dir)
    if defs is None:
        defs, defs_note = resolve_defs(run_dir, msgdir, stack_sha, repo)
    files = bag_files(run_dir)
    if not files:
        raise FileNotFoundError(f"{run_dir}: no rosbag2*/ *.db3 or *.db3.zstd")
    raw: dict[str, list[tuple[int, dict]]] = {}
    stats: dict[str, TopicStats] = {}
    skipped: dict[str, str] = {}
    with tempfile.TemporaryDirectory(prefix="dyx3_bag_") as tmp:
        for f in files:
            db = f
            if f.name.endswith(".zstd"):
                db = Path(tmp) / f.name[:-5]
                _zstd_decompress_file(f, db)
            con = sqlite3.connect(f"file:{db}?mode=ro", uri=True)
            try:
                topics = {int(i): (n, t) for i, n, t in con.execute("SELECT id, name, type FROM topics")}
                for tid, ts, data in con.execute("SELECT topic_id, timestamp, data FROM messages ORDER BY timestamp, id"):
                    name, typ = topics[int(tid)]
                    if not typ.startswith(PACKAGE + "/"):
                        skipped[name] = typ
                        continue
                    st = stats.setdefault(name, TopicStats(typ))
                    st.n += 1
                    blob = bytes(data)
                    if blob[:4] == ZSTD_MAGIC:
                        blob = _zstd_decompress_bytes(blob)
                    try:
                        res = decode(defs, typ, blob)
                    except (ValueError, KeyError, OSError, struct.error, TruncatedBuffer,
                            subprocess.CalledProcessError) as e:
                        st.failed += 1
                        if len(st.errors) < 3:
                            st.errors.append(f"{type(e).__name__}: {e}")
                        continue
                    if res.missing:
                        st.truncated += 1
                        if not st.missing_fields:
                            st.missing_fields = res.missing
                    if res.extra_bytes >= 4:
                        st.extra += 1
                    raw.setdefault(name, []).append((int(ts), res.value))
            finally:
                con.close()
    topics_out = {name: Series(name, stats[name].type, np.array([t for t, _ in v], dtype=np.int64), [m for _, m in v])
                  for name, v in raw.items()}
    return RunData(run_dir, _read_json(run_dir / "manifest.json"), _read_json(run_dir / "summary.json"),
                   _read_json(run_dir / "versions.json"), defs, defs_note, topics_out, stats, skipped, raw)


def merge_runs(runs: list[RunData]) -> dict[str, Series]:
    """Topics of several runs merged in receipt time; an identical (topic, timestamp) seen in two overlapping bags is kept once."""
    merged: dict[str, list[tuple[int, dict]]] = {}
    types: dict[str, str] = {}
    for r in runs:
        for name, items in r.raw.items():
            merged.setdefault(name, []).extend(items)
            types[name] = r.stats[name].type
    out = {}
    for name, items in merged.items():
        items.sort(key=lambda x: x[0])
        dedup: list[tuple[int, dict]] = []
        for ts, m in items:
            if dedup and dedup[-1][0] == ts and dedup[-1][1] == m:
                continue
            dedup.append((ts, m))
        out[name] = Series(name, types[name], np.array([t for t, _ in dedup], dtype=np.int64), [m for _, m in dedup])
    return out


# --------------------------------------------------------------------------------------------------------------------------------
# Small statistics helpers
# --------------------------------------------------------------------------------------------------------------------------------

def _finite(a: np.ndarray) -> np.ndarray:
    a = np.asarray(a, dtype=float)
    return a[np.isfinite(a)]


def _q(a: np.ndarray, q: float) -> float | None:
    a = _finite(a)
    return float(np.percentile(a, q)) if a.size else None


def _fmax(a: np.ndarray) -> float | None:
    a = _finite(a)
    return float(a.max()) if a.size else None


def _fmean(a: np.ndarray) -> float | None:
    a = _finite(a)
    return float(a.mean()) if a.size else None


def _last(a: np.ndarray) -> float | None:
    a = _finite(a)
    return float(a[-1]) if a.size else None


def zero_crossings(x: np.ndarray) -> int:
    """Sign changes of x, exact zeros skipped."""
    s = np.sign(_finite(x))
    s = s[s != 0]
    return int(np.count_nonzero(s[1:] != s[:-1])) if s.size > 1 else 0


def dominant_period(t: np.ndarray, x: np.ndarray, pad: int = 8) -> float | None:
    """Period of the largest non-DC spectral peak of x(t): mean removed, resampled to the median step, Hann window, zero-padded
    `pad` times. A result close to (or above) the window length means there is no oscillation inside the window."""
    t = np.asarray(t, dtype=float)
    x = np.asarray(x, dtype=float)
    ok = np.isfinite(t) & np.isfinite(x)
    t, x = t[ok], x[ok]
    if t.size < 8:
        return None
    dt = float(np.median(np.diff(t)))
    dur = float(t[-1] - t[0])
    if dt <= 0 or dur <= 0:
        return None
    n = int(dur / dt) + 1
    tu = t[0] + np.arange(n) * dt
    xu = np.interp(tu, t, x)
    xu = xu - xu.mean()
    if not np.any(xu):
        return None
    nfft = 1 << math.ceil(math.log2(n * pad))
    spec = np.abs(np.fft.rfft(xu * np.hanning(n), nfft))
    freqs = np.fft.rfftfreq(nfft, dt)
    spec[0] = 0.0
    k = int(np.argmax(spec))
    return float(1.0 / freqs[k]) if freqs[k] > 0 else None


def rate_stats(t_ns: np.ndarray) -> dict:
    """n, mean Hz over the span, instantaneous Hz (1/dt) min/p1/p50/p99/max, max gap. Non-positive steps are counted and
    excluded from the instantaneous statistics."""
    t_ns = np.asarray(t_ns, dtype=np.int64)
    out: dict[str, Any] = {"n": int(t_ns.size)}
    if t_ns.size < 2:
        return out
    dt = np.diff(t_ns).astype(float) / 1e9
    pos = dt[dt > 0]
    span = (t_ns[-1] - t_ns[0]) / 1e9
    out["mean_hz"] = (t_ns.size - 1) / span if span > 0 else None
    out["nonpositive_steps"] = int(np.count_nonzero(dt <= 0))
    if pos.size:
        hz = 1.0 / pos
        out.update(min_hz=float(hz.min()), p1_hz=float(np.percentile(hz, 1)), p50_hz=float(np.percentile(hz, 50)),
                   p99_hz=float(np.percentile(hz, 99)), max_hz=float(hz.max()), max_gap_ms=float(pos.max() * 1e3))
    return out


def _abs_stats(x: np.ndarray, key: str) -> dict:
    a = np.abs(_finite(x))
    return {f"{key}_abs_p50": _q(a, 50), f"{key}_abs_p95": _q(a, 95), f"{key}_abs_max": _fmax(a),
            f"{key}_mean": _fmean(_finite(x))}


def _hist(values: Iterable[Any], namer: Callable[[Any], str] | None = None) -> dict[str, int]:
    out: dict[str, int] = {}
    for v in values:
        if v is None or (isinstance(v, float) and math.isnan(v)):
            continue
        k = namer(v) if namer else str(v)
        out[k] = out.get(k, 0) + 1
    return out


# --------------------------------------------------------------------------------------------------------------------------------
# RPP state intervals
# --------------------------------------------------------------------------------------------------------------------------------

@dataclass
class Interval:
    state: int
    i0: int          # first sample index (inclusive)
    i1: int          # last sample index (inclusive)
    t0: float        # bag time of the first sample, s
    t1: float        # bag time of the first sample of the next interval (or of the last sample at the end of the data)

    @property
    def duration(self) -> float:
        return self.t1 - self.t0


def state_intervals(t: np.ndarray, state: np.ndarray) -> list[Interval]:
    out: list[Interval] = []
    n = len(state)
    i = 0
    while i < n:
        j = i
        while j + 1 < n and state[j + 1] == state[i]:
            j += 1
        t1 = t[j + 1] if j + 1 < n else t[j]
        out.append(Interval(int(state[i]), i, j, float(t[i]), float(t1)))
        i = j + 1
    return out


def _rpp_codes(defs: MsgDefs) -> dict[str, int]:
    try:
        c = defs.get("dyx3_interfaces/RppStatus").constants
    except (KeyError, OSError, subprocess.CalledProcessError):
        c = {}
    return {"TRACKING": c.get("STATE_TRACKING", RPP_TRACKING), "STOPPING": c.get("STATE_STOPPING", RPP_STOPPING),
            "PIVOTING": c.get("STATE_PIVOTING", RPP_PIVOTING), "CREEPING": c.get("STATE_CREEPING", RPP_CREEPING)}


# --------------------------------------------------------------------------------------------------------------------------------
# Metrics
# --------------------------------------------------------------------------------------------------------------------------------

def metrics_rates(run: RunData) -> dict:
    vs = run.series(T_VEHICLE)
    px4 = vs.stamp_ns("px4_sample_stamp") if len(vs) else np.zeros(0, dtype=np.int64)
    return {
        "vehicle_state_receipt": rate_stats(vs.t_ns),
        "vehicle_state_px4_sample_stamp": rate_stats(px4[px4 > 0]),
        "rpp_motion_setpoint": rate_stats(run.series(T_SETPOINT).t_ns),
        "motion_guard_command": rate_stats(run.series(T_GUARD_CMD).t_ns),
        "px4_link_status": rate_stats(run.series(T_LINK).t_ns),
    }


def metrics_latency(run: RunData) -> dict:
    out: dict[str, Any] = {}
    ls = run.series(T_LINK)
    if len(ls) and ls.has("pose_to_write_age_valid"):
        valid = ls.col("pose_to_write_age_valid") == 1.0
        age = ls.col("pose_to_write_age_s")[valid]
        out["pose_to_write"] = {"n_valid": int(valid.sum()), "n": len(ls), "p50_ms": _ms(_q(age, 50)), "p95_ms": _ms(_q(age, 95)),
                                "max_of_max_ms": _ms(_fmax(ls.col("pose_to_write_age_max_s")[valid]))}
    else:
        out["pose_to_write"] = {"unavailable": "Px4LinkStatus.pose_to_write_age_* not in this bag"}
    rs = run.series(T_RPP)
    if len(rs):
        ov = rs.col("loop_overrun_count")
        out["rpp"] = {"loop_jitter_p50_us": _q(rs.col("loop_jitter_us"), 50), "loop_jitter_p99_us": _q(rs.col("loop_jitter_us"), 99),
                      "loop_jitter_max_us": _fmax(rs.col("loop_jitter_max_us")), "loop_overrun_count_last": _last(ov),
                      "loop_overrun_delta": _delta(ov)}
    if len(ls):
        link: dict[str, Any] = {}
        for k in ("loop_overrun_count", "command_gap_events", "session_resets"):
            c = ls.col(k)
            link[f"{k}_last"] = _last(c)
            link[f"{k}_delta"] = _delta(c)
        link["fault_histogram"] = _hist(ls.values("fault"),
                                        lambda v: run.defs.enum_name("dyx3_interfaces/Px4LinkStatus", "FAULT_", v))
        link["failing_to_zero_samples"] = int(np.nansum(ls.col("failing_to_zero")))
        out["px4_link"] = link
    return out


def _ms(v: float | None) -> float | None:
    return None if v is None else v * 1e3


def _delta(c: np.ndarray) -> float | None:
    c = _finite(c)
    return float(c[-1] - c[0]) if c.size else None


def _segments(mask: np.ndarray) -> list[tuple[int, int]]:
    """Contiguous runs of True as (first, last) inclusive index pairs."""
    idx = np.flatnonzero(mask)
    if idx.size == 0:
        return []
    breaks = np.flatnonzero(np.diff(idx) > 1)
    starts = np.concatenate(([idx[0]], idx[breaks + 1]))
    ends = np.concatenate((idx[breaks], [idx[-1]]))
    return [(int(a), int(b)) for a, b in zip(starts, ends)]


def _window_stats(t: np.ndarray, xt: np.ndarray, he: np.ndarray) -> dict:
    n = int(np.isfinite(xt).sum())
    d: dict[str, Any] = {"n": n, "t_start": float(t[0]) if len(t) else None, "duration_s": float(t[-1] - t[0]) if len(t) else None,
                         "xt_mean_m": _fmean(xt), "xt_abs_mean_m": _fmean(np.abs(xt)), "xt_abs_max_m": _fmax(np.abs(xt)),
                         "xt_abs_p95_m": _q(np.abs(xt), 95), "he_mean_rad": _fmean(he)}
    return d


def metrics_tracking(rs: Series, defs: MsgDefs, t_ref: float) -> dict:
    """Per RPP run_index: per state stats, STEADY subset, corner entries, braking tails."""
    if not len(rs):
        return {"runs": []}
    codes = _rpp_codes(defs)
    t = rs.t - t_ref
    state = rs.col("state")
    run_idx = rs.col("run_index")
    xt = rs.col("cross_track_right_m")
    he = rs.col("heading_error_rad")
    v = rs.col("commanded_speed_mps")
    travel = rs.col("path_travel_m")
    has_travel = rs.has("path_travel_m")
    intervals = state_intervals(t, np.nan_to_num(state, nan=-1).astype(int))
    runs_out = []
    for ri in [int(x) for x in sorted(set(_finite(run_idx).tolist()))]:
        sel = run_idx == ri
        per_state = {}
        for s in sorted(set(_finite(state[sel]).astype(int).tolist())):
            m = sel & (state == s)
            per_state[defs.enum_name("dyx3_interfaces/RppStatus", "STATE_", s)] = {
                "n": int(m.sum()), **_abs_stats(xt[m], "xt"), **_abs_stats(he[m], "he")}
        tc = sel & ((state == codes["TRACKING"]) | (state == codes["CREEPING"]))
        trk = sel & (state == codes["TRACKING"])
        vmax = _fmax(v[trk])
        steady: dict[str, Any] = {"vmax_cmd_mps": vmax, "segments": []}
        if vmax is not None and vmax > 0:
            smask = trk & (v >= STEADY_SPEED_FRACTION * vmax)
            for a, b in _segments(smask):
                sl = slice(a, b + 1)
                tt, x, h = t[sl], xt[sl], he[sl]
                zc = zero_crossings(x)
                dur = float(tt[-1] - tt[0])
                steady["segments"].append({
                    "t_start": float(tt[0]), "duration_s": dur, "n": int(b - a + 1),
                    "xt_mean_m": _fmean(x), "xt_std_m": float(np.nanstd(x)), "xt_p2p_m": float(np.nanmax(x) - np.nanmin(x)),
                    "xt_zero_crossings": zc, "xt_dominant_period_s": dominant_period(tt, x),
                    "xt_zero_crossing_period_s": (2.0 * dur / zc) if zc else None,
                    "he_mean_rad": _fmean(h), "he_std_rad": float(np.nanstd(h)), "he_p2p_rad": float(np.nanmax(h) - np.nanmin(h)),
                    "he_zero_crossings": zero_crossings(h)})
            pooled = xt[smask]
            steady["pooled"] = {"n": int(smask.sum()), "xt_std_m": float(np.nanstd(pooled)) if pooled.size else None,
                                "xt_p2p_m": float(np.nanmax(pooled) - np.nanmin(pooled)) if pooled.size else None,
                                "xt_zero_crossings": int(sum(s["xt_zero_crossings"] for s in steady["segments"]))}
        corner_entries, braking_tails = [], []
        if has_travel:
            for k, iv in enumerate(intervals):
                if iv.state != codes["TRACKING"] or not sel[iv.i0]:
                    continue
                sl = slice(iv.i0, iv.i1 + 1)
                tr = travel[sl]
                if not np.isfinite(tr).any():
                    continue
                after_pivot = False
                for prev in reversed(intervals[:k]):
                    if prev.state == codes["TRACKING"]:
                        break
                    if prev.state == codes["PIVOTING"]:
                        after_pivot = True
                        break
                m0 = (tr - tr[np.isfinite(tr)][0]) <= CORNER_ENTRY_M
                corner_entries.append({"after_pivot": after_pivot, "travel_start_m": float(tr[np.isfinite(tr)][0]),
                                       **_window_stats(t[sl][m0], xt[sl][m0], he[sl][m0])})
                nxt = intervals[k + 1].state if k + 1 < len(intervals) else None
                if nxt == codes["STOPPING"]:
                    end = tr[np.isfinite(tr)][-1]
                    m1 = tr >= end - BRAKING_TAIL_M
                    braking_tails.append({"travel_end_m": float(end), "cmd_speed_mean_mps": _fmean(v[sl][m1]),
                                          **_window_stats(t[sl][m1], xt[sl][m1], he[sl][m1])})
        runs_out.append({"run_index": ri, "n": int(sel.sum()), "per_state": per_state,
                         "tracking_creeping": {"n": int(tc.sum()), **_abs_stats(xt[tc], "xt"), **_abs_stats(he[tc], "he")},
                         "steady": steady, "corner_entry": corner_entries, "braking_tail": braking_tails,
                         "path_travel_available": has_travel})
    return {"runs": runs_out}


def metrics_pivot(rs: Series, vs: Series, defs: MsgDefs, t_ref: float) -> dict:
    if not len(rs):
        return {"episodes": []}
    codes = _rpp_codes(defs)
    t = rs.t - t_ref
    state = np.nan_to_num(rs.col("state"), nan=-1).astype(int)
    he = rs.col("heading_error_rad")
    cmd = rs.col("commanded_yaw_rate_radps")
    run_idx = rs.col("run_index")
    timed_out = rs.col("pivot_timed_out") if rs.has("pivot_timed_out") else None
    ivs = state_intervals(t, state)
    episodes = []
    k = 0
    while k < len(ivs):
        if ivs[k].state != codes["PIVOTING"]:
            k += 1
            continue
        group = []
        j = k
        while j < len(ivs) and ivs[j].state in (codes["PIVOTING"], codes["STOPPING"]):
            group.append(ivs[j])
            j += 1
        pivots = [g for g in group if g.state == codes["PIVOTING"]]
        first, last = pivots[0], pivots[-1]
        sl = slice(first.i0, last.i1 + 1)
        ep = {"run_index": int(run_idx[first.i0]) if np.isfinite(run_idx[first.i0]) else None,
              "t_start": first.t0, "t_release": last.t1, "cycles": len(pivots), "duration_s": last.t1 - first.t0,
              "he_entry_rad": float(he[first.i0]), "he_release_rad": float(he[last.i1]),
              "cmd_yaw_rate_abs_max_radps": _fmax(np.abs(cmd[sl])),
              "starts_at_data_start": first.i0 == 0, "ends_at_data_end": j >= len(ivs),
              "next_state": defs.enum_name("dyx3_interfaces/RppStatus", "STATE_", ivs[j].state) if j < len(ivs) else None}
        if timed_out is not None:
            ep["pivot_timed_out_ever"] = bool(np.nansum(timed_out[sl]) > 0)
        full = np.zeros(len(state), dtype=bool)
        full[sl] = (state[sl] == codes["PIVOTING"]) & (np.abs(cmd[sl]) >= FULL_RATE_YAW_RADPS)
        ep["full_rate_n"] = int(full.sum())
        ep["full_rate_yaw_ratio_p50"] = _yaw_ratio(rs, vs, full).get("ratio_p50")
        episodes.append(ep)
        k = j
    out: dict[str, Any] = {"episodes": episodes}
    pmask = (state == codes["PIVOTING"]) & (np.abs(cmd) >= FULL_RATE_YAW_RADPS)
    out["full_rate_yaw"] = {"threshold_radps": FULL_RATE_YAW_RADPS, **_yaw_ratio(rs, vs, pmask)}
    return out


def _yaw_ratio(rs: Series, vs: Series, mask: np.ndarray) -> dict:
    """Measured (VehicleState.yaw_rate_radps, interpolated at the RppStatus receipt time) over commanded yaw rate."""
    out: dict[str, Any] = {"n": int(mask.sum())}
    if not mask.any() or not len(vs) or not vs.has("yaw_rate_radps"):
        return out
    meas = np.interp(rs.t[mask], vs.t, vs.col("yaw_rate_radps"))
    ratio = meas / rs.col("commanded_yaw_rate_radps")[mask]
    out.update(ratio_p50=_q(ratio, 50), ratio_mean=_fmean(ratio), ratio_p10=_q(ratio, 10), ratio_p90=_q(ratio, 90))
    return out


def metrics_endpoint(rs: Series, vs: Series, defs: MsgDefs, t_ref: float) -> dict:
    if not len(rs):
        return {"present": False, "note": "no RppStatus"}
    codes = _rpp_codes(defs)
    t = rs.t - t_ref
    state = np.nan_to_num(rs.col("state"), nan=-1).astype(int)
    ivs = state_intervals(t, state)
    creeps = [k for k, iv in enumerate(ivs) if iv.state == codes["CREEPING"]]
    if not creeps:
        return {"present": False, "note": "no CREEPING interval in the data"}
    end_states = (codes["CREEPING"], codes["STOPPING"])
    k0 = creeps[-1]
    while k0 > 0 and ivs[k0 - 1].state in end_states:
        k0 -= 1
    while ivs[k0].state != codes["CREEPING"]:
        k0 += 1
    k1 = creeps[-1]
    while k1 + 1 < len(ivs) and ivs[k1 + 1].state in end_states:
        k1 += 1
    block = ivs[k0:k1 + 1]
    i0, i1 = block[0].i0, block[-1].i1
    t_end = block[-1].t1
    sl = slice(i0, i1 + 1)
    v = rs.col("commanded_speed_mps")[sl]
    sgn = np.sign(v[np.abs(v) > SIGN_EPS])
    reversals = int(np.count_nonzero(sgn[1:] != sgn[:-1])) if sgn.size > 1 else 0
    fwd_to_rev = int(np.count_nonzero((sgn[:-1] > 0) & (sgn[1:] < 0))) if sgn.size > 1 else 0
    creep_s = float(sum(iv.duration for iv in block if iv.state == codes["CREEPING"]))
    stop_s = float(sum(iv.duration for iv in block if iv.state == codes["STOPPING"]))
    out: dict[str, Any] = {
        "present": True, "run_index": _int_or_none(rs.col("run_index")[i0]), "t_start": block[0].t0, "t_end": t_end,
        "creep_s": creep_s, "stopping_s": stop_s, "creep_intervals": sum(1 for iv in block if iv.state == codes["CREEPING"]),
        "speed_sign_reversals": reversals, "forward_to_reverse": fwd_to_rev, "cmd_speed_abs_max_mps": _fmax(np.abs(v)),
        "finished_by_timeout": creep_s >= CREEP_TIMEOUT_FLAG_S,
        "next_state": defs.enum_name("dyx3_interfaces/RppStatus", "STATE_", ivs[k1 + 1].state) if k1 + 1 < len(ivs) else None,
        "xt_creep_abs_p50_m": _q(np.abs(rs.col("cross_track_right_m")[sl][state[sl] == codes["CREEPING"]]), 50)}
    if rs.has("dist_to_goal_m"):
        dg = _finite(rs.col("dist_to_goal_m")[sl])
        out["final_dist_to_goal_m"] = float(dg[-1]) if dg.size else None
        out["dist_to_goal_at_creep_entry_m"] = float(dg[0]) if dg.size else None
    else:
        out["final_dist_to_goal_m"] = None
        out["dist_to_goal_note"] = "RppStatus.dist_to_goal_m not in this bag"
    if len(vs):
        tv = vs.t - t_ref
        w = (tv >= t_end - ENDPOINT_TAIL_S) & (tv <= t_end)
        spd = np.hypot(vs.col("velocity_north_mps")[w], vs.col("velocity_east_mps")[w])
        out["measured_speed_max_last6s_mps"] = _fmax(spd)
    return out


def _int_or_none(x: float) -> int | None:
    return int(x) if np.isfinite(x) else None


def metrics_mission(topics: dict[str, Series], defs: MsgDefs, t_ref: float) -> dict:
    out: dict[str, Any] = {}
    ms = topics.get(T_MISSION) or _empty_series(T_MISSION)
    trans = []
    prev = None
    for tt, m in zip(ms.t - t_ref, ms.msgs):
        st = m.get("state")
        if st != prev:
            trans.append({"t": float(tt), "state": defs.enum_name("dyx3_interfaces/MissionState", "STATE_", st),
                          "from": None if prev is None else defs.enum_name("dyx3_interfaces/MissionState", "STATE_", prev),
                          "reason": defs.enum_name("dyx3_interfaces/MissionState", "REASON_", m.get("reason_code")),
                          "reason_detail": m.get("reason_detail") or "",
                          "waiting_on": defs.enum_name("dyx3_interfaces/MissionState", "WAIT_", m.get("waiting_on")),
                          "run_index": m.get("run_index"), "point_index": m.get("point_index")})
            prev = st
    out["transitions"] = trans
    ps = topics.get(T_POINT) or _empty_series(T_POINT)
    out["points"] = [{"t": float(tt), "point_index": m.get("point_index"), "error_m": m.get("error_m"),
                      "result": defs.enum_name("dyx3_interfaces/PointResult", "RESULT_", m.get("result_code"))}
                     for tt, m in zip(ps.t - t_ref, ps.msgs)]

    def reason_name(v: Any) -> str:
        return defs.enum_name("dyx3_interfaces/MotionSetpointStatus", "REASON_", v)

    gs = topics.get(T_GUARD_STATUS) or _empty_series(T_GUARD_STATUS)
    out["guard"] = {"n": len(gs), "reason_histogram": _hist(gs.values("reason_code"), reason_name),
                    "clamped": int(np.nansum(gs.col("clamped"))) if len(gs) else 0,
                    "refused": int(np.count_nonzero(gs.col("accepted") == 0)) if len(gs) else 0}
    sg = topics.get(T_SAFETY) or _empty_series(T_SAFETY)
    if len(sg):
        ok = sg.col("ok")
        reasons = [r for r, o in zip(sg.values("reason_code"), ok) if o == 0]
        out["safety_gate"] = {"n": len(sg), "ok_histogram": {"true": int(np.count_nonzero(ok == 1)), "false": int(np.count_nonzero(ok == 0))},
                              "reason_histogram_when_not_ok": _hist(reasons, reason_name)}
        if sg.has("pre_arm_ok"):
            pa = sg.col("pre_arm_ok")
            out["safety_gate"]["pre_arm_ok_histogram"] = {"true": int(np.count_nonzero(pa == 1)), "false": int(np.count_nonzero(pa == 0))}
            out["safety_gate"]["pre_arm_reason_histogram_when_not_ok"] = _hist(
                [r for r, o in zip(sg.values("pre_arm_reason_code"), pa) if o == 0], reason_name)
    else:
        out["safety_gate"] = {"n": 0}
    rk = topics.get(T_RTK) or _empty_series(T_RTK)
    out["rtk"] = {"n": len(rk), "fix_histogram": _hist(rk.values("fix_type"), lambda v: defs.enum_name("dyx3_interfaces/RtkStatus", "FIX_", v)),
                  "hrms_max_m": _fmax(rk.col("horizontal_accuracy_m")) if len(rk) else None,
                  "correction_age_max_s": _fmax(rk.col("correction_age_s")) if len(rk) else None,
                  "corrections_stale_samples": int(np.count_nonzero(rk.col("corrections_fresh") == 0)) if len(rk) else 0}
    es = topics.get(T_ESTIMATOR) or _empty_series(T_ESTIMATOR)
    est: dict[str, Any] = {"n": len(es)}
    if len(es):
        bools = [f.name for f in defs.get("dyx3_interfaces/EstimatorHealth").fields if f.type == "bool" and f.array is None]
        est["true_counts"] = {b: int(np.nansum(es.col(b))) for b in bools if es.has(b)}
        healthy_true = {"flags_valid", "test_ratios_valid", "gnss_yaw_fusion_intended"}
        est["faults_ever_set"] = [b for b, c in est["true_counts"].items() if b not in healthy_true and c > 0]
        est["healthy_flags_ever_false"] = [b for b in healthy_true if es.has(b) and np.count_nonzero(es.col(b) == 0) > 0]
        for k in ("yaw_test_ratio", "position_test_ratio", "velocity_test_ratio"):
            if es.has(k):
                est[f"{k}_max"] = _fmax(es.col(k))
        if es.has("innovation_fault_status_changes"):
            est["innovation_fault_status_changes_delta"] = _delta(es.col("innovation_fault_status_changes"))
    out["estimator"] = est
    return out


def metrics_gate(rs: Series, defs: MsgDefs) -> dict:
    if not len(rs):
        return {"n": 0}
    codes = _rpp_codes(defs)
    state = rs.col("state")
    m = (state == codes["TRACKING"]) | (state == codes["CREEPING"])
    x = _finite(rs.col("cross_track_right_m")[m])
    return {"label": "RPP's own cross-track (RppStatus.cross_track_right_m) over TRACKING+CREEP; NOT surveyed truth",
            "n": int(x.size), "rms_m": float(np.sqrt(np.mean(x * x))) if x.size else None,
            "p95_abs_m": _q(np.abs(x), 95), "max_abs_m": _fmax(np.abs(x))}


def _decode_report(run: RunData) -> dict:
    topics = {}
    warnings = []
    for name, st in sorted(run.stats.items()):
        topics[name] = {"type": st.type, "n": st.n, "failed": st.failed, "truncated": st.truncated, "extra_bytes": st.extra}
        if st.failed:
            warnings.append(f"{name}: {st.failed}/{st.n} messages failed to decode ({'; '.join(st.errors)})")
        if st.truncated:
            warnings.append(f"{name}: {st.truncated}/{st.n} messages lack trailing fields {st.missing_fields} (bag predates them)")
        if st.extra:
            warnings.append(f"{name}: {st.extra}/{st.n} messages have >= 4 unread bytes (bag has fields these definitions lack)")
    return {"definitions": run.defs.source, "definitions_from": run.defs_note, "topics": topics,
            "skipped_topics": dict(sorted(run.skipped_topics.items())), "warnings": warnings}


def _utc(ns: int | None) -> str | None:
    if ns is None:
        return None
    return datetime.fromtimestamp(ns / 1e9, tz=timezone.utc).strftime("%Y-%m-%dT%H:%M:%S.%fZ")


def analyze(runs: list[RunData]) -> dict:
    """Metrics for one or more runs of one mission. Per-bag sections (rates, latency, decode) are per run; controller sections
    are computed over the runs merged in receipt time (times are seconds from the first message of the first run)."""
    runs = sorted(runs, key=lambda r: (r.t_first_ns or 0))
    topics = merge_runs(runs) if len(runs) > 1 else runs[0].topics
    defs = runs[-1].defs
    firsts = [r.t_first_ns for r in runs if r.t_first_ns is not None]
    lasts = [r.t_last_ns for r in runs if r.t_last_ns is not None]
    t_ref_ns = min(firsts) if firsts else 0
    t_ref = t_ref_ns / 1e9
    rs = topics.get(T_RPP) or _empty_series(T_RPP)
    vs = topics.get(T_VEHICLE) or _empty_series(T_VEHICLE)
    per_run = []
    for r in runs:
        per_run.append({
            "run_id": r.run_id, "run_dir": str(r.run_dir), "mission_id": r.manifest.get("mission_id"),
            "run_index": r.manifest.get("run_index"), "stack_sha": r.versions.get("stack_sha"),
            "final_state": r.summary.get("final_state"), "summary_duration_s": r.summary.get("duration_s"),
            "t_start": ((r.t_first_ns or t_ref_ns) - t_ref_ns) / 1e9,
            "data_span_s": ((r.t_last_ns or 0) - (r.t_first_ns or 0)) / 1e9 if r.t_first_ns else None,
            "decode": _decode_report(r), "rates": metrics_rates(r), "latency": metrics_latency(r)})
    mission_ids = sorted({str(r.manifest.get("mission_id")) for r in runs})
    return {
        "schema": SCHEMA, "tool": TOOL, "generated_utc": datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
        "mission_id": runs[0].manifest.get("mission_id"), "mission_ids": mission_ids,
        "t_ref_utc": _utc(t_ref_ns), "span_s": ((max(lasts) - t_ref_ns) / 1e9) if lasts else None,
        "runs": per_run,
        "tracking": metrics_tracking(rs, defs, t_ref),
        "pivot": metrics_pivot(rs, vs, defs, t_ref),
        "endpoint": metrics_endpoint(rs, vs, defs, t_ref),
        "mission": metrics_mission(topics, defs, t_ref),
        "gate": metrics_gate(rs, defs),
    }


# --------------------------------------------------------------------------------------------------------------------------------
# Markdown
# --------------------------------------------------------------------------------------------------------------------------------

def _fmt(v: Any, scale: float = 1.0, nd: int = 2) -> str:
    if v is None:
        return "n/a"
    if isinstance(v, bool):
        return "yes" if v else "no"
    if isinstance(v, (int, np.integer)) and scale == 1.0:
        return str(int(v))
    if isinstance(v, (float, int, np.floating)):
        x = float(v) * scale
        return "n/a" if not math.isfinite(x) else f"{x:.{nd}f}"
    return str(v)


def _table(headers: list[str], rows: list[list[Any]]) -> list[str]:
    out = ["| " + " | ".join(headers) + " |", "|" + "|".join("---" for _ in headers) + "|"]
    out += ["| " + " | ".join(str(c) for c in r) + " |" for r in rows]
    return out


CM, DEG, MS = 100.0, 180.0 / math.pi, 1.0


def render_markdown(doc: dict, title: str) -> str:
    L: list[str] = [f"# Mission bag metrics: {title}", ""]
    L.append(f"Mission id(s) {', '.join(doc['mission_ids'])}; data start {doc['t_ref_utc']}; span {_fmt(doc['span_s'], nd=1)} s. "
             "Times below are seconds from the data start. Cross-track is RPP's own (`RppStatus`), not surveyed truth.")
    L.append("")
    L += _table(["run", "run_index", "stack", "definitions", "final", "span s", "decode warnings"],
                [[r["run_id"], _fmt(r["run_index"]), (r["stack_sha"] or "n/a")[:10], r["decode"]["definitions_from"],
                  r["final_state"] or "n/a", _fmt(r["data_span_s"], nd=1), len(r["decode"]["warnings"])] for r in doc["runs"]])
    for r in doc["runs"]:
        for w in r["decode"]["warnings"]:
            L.append(f"- WARNING {r['run_id']}: {w}")
    L += ["", "## 1. Rates (bag receipt time unless stated)", ""]
    rows = []
    for r in doc["runs"]:
        for k, s in r["rates"].items():
            rows.append([r["run_id"], k, s.get("n"), _fmt(s.get("mean_hz"), nd=1), _fmt(s.get("min_hz"), nd=1), _fmt(s.get("p1_hz"), nd=1),
                         _fmt(s.get("p50_hz"), nd=1), _fmt(s.get("p99_hz"), nd=1), _fmt(s.get("max_hz"), nd=1),
                         _fmt(s.get("max_gap_ms"), nd=1)])
    L += _table(["run", "stream", "n", "mean Hz", "min", "p1", "p50", "p99", "max", "max gap ms"], rows)
    L += ["", "## 2. Chain latency and loop health", ""]
    rows = []
    for r in doc["runs"]:
        lat = r["latency"]
        p = lat.get("pose_to_write", {})
        rp = lat.get("rpp", {})
        lk = lat.get("px4_link", {})
        rows.append([r["run_id"], _fmt(p.get("p50_ms")), _fmt(p.get("p95_ms")), _fmt(p.get("max_of_max_ms")),
                     _fmt(rp.get("loop_jitter_max_us"), nd=0), _fmt(rp.get("loop_overrun_count_last"), nd=0),
                     _fmt(lk.get("loop_overrun_count_last"), nd=0), _fmt(lk.get("command_gap_events_last"), nd=0),
                     _fmt(lk.get("session_resets_last"), nd=0),
                     ", ".join(f"{k} {v}" for k, v in lk.get("fault_histogram", {}).items()) or "n/a"])
    L += _table(["run", "pose->write p50 ms", "p95 ms", "max ms", "RPP jitter max us", "RPP overruns", "link overruns",
                 "cmd gap events", "session resets", "link faults"], rows)
    L += ["", "## 3. Tracking (per RPP run_index and state)", ""]
    rows = []
    for tr in doc["tracking"]["runs"]:
        for sname, s in list(tr["per_state"].items()) + [("TRACKING+CREEP", tr["tracking_creeping"])]:
            rows.append([tr["run_index"], sname, s["n"], _fmt(s["xt_abs_p50"], CM), _fmt(s["xt_abs_p95"], CM), _fmt(s["xt_abs_max"], CM),
                         _fmt(s["xt_mean"], CM), _fmt(s["he_abs_p50"], DEG), _fmt(s["he_abs_p95"], DEG), _fmt(s["he_abs_max"], DEG)])
    L += _table(["run", "state", "n", "|xt| p50 cm", "p95 cm", "max cm", "xt mean cm", "|he| p50 deg", "p95 deg", "max deg"], rows)
    L += ["", f"STEADY = TRACKING with commanded speed >= {STEADY_SPEED_FRACTION} x the run's max commanded speed.", ""]
    rows = []
    for tr in doc["tracking"]["runs"]:
        for s in tr["steady"]["segments"]:
            rows.append([tr["run_index"], _fmt(s["t_start"]), _fmt(s["duration_s"]), _fmt(s["xt_mean_m"], CM), _fmt(s["xt_std_m"], CM),
                         _fmt(s["xt_p2p_m"], CM), s["xt_zero_crossings"], _fmt(s["xt_dominant_period_s"]),
                         _fmt(s["he_std_rad"], DEG), _fmt(s["he_p2p_rad"], DEG), s["he_zero_crossings"]])
    L += _table(["run", "t s", "dur s", "xt mean cm", "xt std cm", "xt p2p cm", "xt zero x", "xt period s",
                 "he std deg", "he p2p deg", "he zero x"], rows)
    L += ["", (f"Corner entry = first {CORNER_ENTRY_M} m of `path_travel_m` of each TRACKING interval; "
               f"braking tail = last {BRAKING_TAIL_M} m before the next STOPPING."), ""]
    rows = []
    for tr in doc["tracking"]["runs"]:
        for s in tr["corner_entry"]:
            rows.append([tr["run_index"], "entry" + (" (after pivot)" if s["after_pivot"] else ""), _fmt(s["t_start"]), s["n"],
                         _fmt(s["xt_mean_m"], CM), _fmt(s["xt_abs_p95_m"], CM), _fmt(s["xt_abs_max_m"], CM), _fmt(s["he_mean_rad"], DEG)])
        for s in tr["braking_tail"]:
            rows.append([tr["run_index"], "braking tail", _fmt(s["t_start"]), s["n"], _fmt(s["xt_mean_m"], CM),
                         _fmt(s["xt_abs_p95_m"], CM), _fmt(s["xt_abs_max_m"], CM), _fmt(s["he_mean_rad"], DEG)])
    L += _table(["run", "window", "t s", "n", "xt mean cm", "|xt| p95 cm", "|xt| max cm", "he mean deg"], rows)
    L += ["", "## 4. Pivot", ""]
    rows = []
    for e in doc["pivot"]["episodes"]:
        flags = []
        if e["starts_at_data_start"]:
            flags.append("starts at data start")
        if e["ends_at_data_end"]:
            flags.append("ends at data end")
        if e.get("pivot_timed_out_ever"):
            flags.append("pivot_timed_out")
        rows.append([_fmt(e["run_index"]), _fmt(e["t_start"]), e["cycles"], _fmt(e["duration_s"]), _fmt(e["he_entry_rad"], DEG),
                     _fmt(e["he_release_rad"], DEG), _fmt(e["cmd_yaw_rate_abs_max_radps"], nd=3),
                     _fmt(e.get("full_rate_yaw_ratio_p50"), nd=3), e["next_state"] or "n/a", "; ".join(flags)])
    L += _table(["run", "t s", "cycles", "dur s", "he entry deg", "he release deg", "|cmd yaw| max rad/s",
                 "full-rate meas/cmd p50", "next", "flags"], rows)
    fr = doc["pivot"]["full_rate_yaw"]
    L += ["", (f"Full-rate pivot (|cmd| >= {fr['threshold_radps']} rad/s, n {fr['n']}): measured/commanded yaw rate "
               f"p50 {_fmt(fr.get('ratio_p50'), nd=3)}, mean {_fmt(fr.get('ratio_mean'), nd=3)}, p10-p90 "
               f"{_fmt(fr.get('ratio_p10'), nd=3)}-{_fmt(fr.get('ratio_p90'), nd=3)}."), ""]
    L += ["## 5. Endpoint (last CREEP of the data)", ""]
    ep = doc["endpoint"]
    if ep.get("present"):
        L += _table(["run", "CREEP s", "STOPPING s", "reversals (fwd->rev)", "max meas speed last 6 s m/s", "final dist_to_goal cm",
                     "|xt| CREEP p50 cm", "timeout finish", "next"],
                    [[_fmt(ep["run_index"]), _fmt(ep["creep_s"]), _fmt(ep["stopping_s"]),
                      f"{ep['speed_sign_reversals']} ({ep['forward_to_reverse']})",
                      _fmt(ep.get("measured_speed_max_last6s_mps"), nd=3), _fmt(ep.get("final_dist_to_goal_m"), CM),
                      _fmt(ep["xt_creep_abs_p50_m"], CM), _fmt(ep["finished_by_timeout"]), ep["next_state"] or "n/a"]])
    else:
        L.append(ep.get("note", "n/a"))
    ms = doc["mission"]
    L += ["", "## 6. Mission", "", "State transitions:", ""]
    L += _table(["t s", "from", "to", "reason", "detail", "waiting_on", "run", "point"],
                [[_fmt(x["t"]), x["from"] or "-", x["state"], x["reason"], x["reason_detail"] or "", x["waiting_on"],
                  _fmt(x["run_index"]), _fmt(x["point_index"])] for x in ms["transitions"]])
    L += ["", "Point results:", ""]
    L += _table(["t s", "point", "result", "error cm"],
                [[_fmt(p["t"]), _fmt(p["point_index"]), p["result"], _fmt(p["error_m"], CM)] for p in ms["points"]])
    g, sg, rk, es = ms["guard"], ms["safety_gate"], ms["rtk"], ms["estimator"]
    L += ["",
          f"- Guard (`{T_GUARD_STATUS}`, n {g['n']}): reasons {_hd(g['reason_histogram'])}; clamped {g['clamped']}; refused {g['refused']}.",
          (f"- Safety gate (n {sg.get('n', 0)}): ok {_hd(sg.get('ok_histogram', {}))}; reasons when not ok "
           f"{_hd(sg.get('reason_histogram_when_not_ok', {}))}; pre-arm ok {_hd(sg.get('pre_arm_ok_histogram', {}))}."),
          (f"- RTK (n {rk['n']}): fix {_hd(rk['fix_histogram'])}; hrms max {_fmt(rk['hrms_max_m'], CM)} cm; correction age max "
           f"{_fmt(rk['correction_age_max_s'])} s; stale-correction samples {rk['corrections_stale_samples']}."),
          (f"- Estimator (n {es['n']}): faults ever set {es.get('faults_ever_set', []) or 'none'}; healthy flags ever false "
           f"{es.get('healthy_flags_ever_false', []) or 'none'}; yaw test ratio max {_fmt(es.get('yaw_test_ratio_max'))}."), ""]
    gt = doc["gate"]
    L += ["## 7. Gate-table rows computable from the bag", ""]
    L += _table(["row", "n", "RMS cm", "p95 cm", "max cm"],
                [["Full-run cross-track, TRACKING+CREEP (RPP's own, NOT surveyed truth)", gt.get("n", 0), _fmt(gt.get("rms_m"), CM),
                  _fmt(gt.get("p95_abs_m"), CM), _fmt(gt.get("max_abs_m"), CM)]])
    L += ["", ("Not provided here: surveyed-truth tracking (`tracking.overall`), shape RMS vs survey, pivot wobble radius and "
               "net walk (PX4 ULog), traversal coverage, spray boundary loss. See docs/analysis/README.md."), ""]
    return "\n".join(L)


def _hd(h: dict) -> str:
    return ", ".join(f"{k} {v}" for k, v in h.items()) if h else "none"


# --------------------------------------------------------------------------------------------------------------------------------
# Inputs and CLI
# --------------------------------------------------------------------------------------------------------------------------------

def is_run_dir(p: Path) -> bool:
    return p.is_dir() and any(c.is_dir() and c.name.startswith("rosbag2") for c in p.iterdir())


def group_missions(runs: list[RunData]) -> list[list[RunData]]:
    """Consecutive runs (by start) with the same manifest mission_id and a non-decreasing run_index form one mission."""
    runs = sorted(runs, key=lambda r: (str(r.manifest.get("start_utc") or ""), r.t_first_ns or 0, r.run_dir.name))
    groups: list[list[RunData]] = []
    for r in runs:
        mid, ri = r.manifest.get("mission_id"), r.manifest.get("run_index")
        if groups:
            last = groups[-1][-1]
            lri = last.manifest.get("run_index")
            if last.manifest.get("mission_id") == mid and (ri is None or lri is None or ri >= lri):
                groups[-1].append(r)
                continue
        groups.append([r])
    return groups


def _json_default(o: Any) -> Any:
    if isinstance(o, np.integer):
        return int(o)
    if isinstance(o, np.floating):
        return float(o)
    if isinstance(o, np.bool_):
        return bool(o)
    if isinstance(o, Path):
        return str(o)
    raise TypeError(type(o))


def _clean(o: Any) -> Any:
    """NaN/inf -> None so the output is strict JSON."""
    if isinstance(o, dict):
        return {k: _clean(v) for k, v in o.items()}
    if isinstance(o, list):
        return [_clean(v) for v in o]
    if isinstance(o, (float, np.floating)) and not math.isfinite(float(o)):
        return None
    return o


def dumps(doc: dict) -> str:
    return json.dumps(_clean(doc), indent=2, default=_json_default, allow_nan=False) + "\n"


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0], formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("path", help="a recorder run directory, or a directory of run directories (mission / Bags/<date>)")
    src = ap.add_mutually_exclusive_group()
    src.add_argument("--msgdir", help="dyx3_interfaces msg directory to decode with (overrides versions.json)")
    src.add_argument("--stack-sha", help="decode with the definitions at this commit (git show, read-only)")
    ap.add_argument("--repo", default=str(REPO_ROOT), help="repository for git show and the working-tree definitions")
    ap.add_argument("--json", action="store_true", help="print JSON instead of Markdown")
    ap.add_argument("--no-write", action="store_true", help="do not write metrics.json files")
    args = ap.parse_args(argv)

    root = Path(args.path)
    if not root.is_dir():
        ap.error(f"not a directory: {root}")
    repo = Path(args.repo)
    run_dirs = [root] if is_run_dir(root) else sorted(p for p in root.iterdir() if is_run_dir(p))
    if not run_dirs:
        ap.error(f"{root}: no run directories (expected rosbag2*/ inside the run directory)")
    runs = [load_run(d, msgdir=args.msgdir, stack_sha=args.stack_sha, repo=repo) for d in run_dirs]

    single = is_run_dir(root)
    out_docs = []
    md = []
    for r in runs:
        doc = analyze([r])
        if not args.no_write:
            (r.run_dir / "metrics.json").write_text(dumps(doc), encoding="utf-8")
        if single:
            out_docs.append(doc)
            md.append(render_markdown(doc, r.run_id))
    if not single:
        for group in group_missions(runs):
            doc = analyze(group)
            out_docs.append(doc)
            md.append(render_markdown(doc, f"{root.name} mission {doc['mission_id']} ({len(group)} runs)"))
        if not args.no_write:
            (root / "metrics.json").write_text(dumps({"schema": SCHEMA, "tool": TOOL, "missions": out_docs}), encoding="utf-8")
    if args.json:
        sys.stdout.write(dumps(out_docs[0] if single else {"schema": SCHEMA, "tool": TOOL, "missions": out_docs}))
    else:
        sys.stdout.write("\n\n".join(md))
    return 0


if __name__ == "__main__":
    sys.exit(main())
