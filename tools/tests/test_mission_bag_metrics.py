"""mission_bag_metrics: CDR codec round trips and the metrics on a synthetic recorder run.

The synthetic run is CDR-encoded with the tool's own definition-driven encoder from the repo's dyx3_interfaces definitions and
written as a plain rosbag2 sqlite .db3 (CI does not install zstandard). The byte layout of the codec is anchored to the CDR rules by
hand-computed buffers and, when `rosbags` is importable (CI installs it), to an independent CDR implementation.
NOT evidence about the rover: real bags are analysed on the Mac / Jetson (docs/analysis/README.md).
"""
from __future__ import annotations

import dataclasses
import json
import math
import random
import sqlite3
import struct
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "analysis"))

import mission_bag_metrics as mbm

REPO = Path(__file__).resolve().parents[2]
MSGDIR = REPO / "ros2_ws" / "src" / "dyx3_interfaces" / "msg"
ALL_MSGS = sorted(p.stem for p in MSGDIR.glob("*.msg"))


@pytest.fixture(scope="module")
def defs():
    return mbm.MsgDefs.from_dir(MSGDIR)


# ------------------------------------------------------------------------------------------------------------------------------
# Codec
# ------------------------------------------------------------------------------------------------------------------------------

PRIM_TEST_MSG = """\
# every primitive, strings, fixed arrays, sequences and nested messages
uint8 CONST_A=7
int8 CONST_B=-3
bool b
uint8 u8
int8 i8
uint16 u16
int16 i16
uint32 u32
int32 i32
uint64 u64
int64 i64
float32 f32
float64 f64
string s
string<=8 bounded_s
builtin_interfaces/Time stamp
float32[3] f32_fixed
uint8[] u8_seq
int16[<=4] i16_bounded
string[2] s_fixed
Inner inner
Inner[] inner_seq
float64 tail  # default ignored 1.5
"""
INNER_MSG = "bool flag\nuint64 big\nstring name\nbuiltin_interfaces/Time t\n"


@pytest.fixture()
def prim_defs(tmp_path):
    d = tmp_path / "msg"
    d.mkdir()
    (d / "AllPrims.msg").write_text(PRIM_TEST_MSG)
    (d / "Inner.msg").write_text(INNER_MSG)
    return mbm.MsgDefs.from_dir(d)


def _f32(x: float) -> float:
    return struct.unpack("<f", struct.pack("<f", x))[0]


def test_every_primitive_string_array_and_nested_time_round_trips(prim_defs):
    md = prim_defs.get("dyx3_interfaces/AllPrims")
    assert md.constants == {"CONST_A": 7, "CONST_B": -3}
    extremes = [
        {"b": True, "u8": 255, "i8": -128, "u16": 65535, "i16": -32768, "u32": 2**32 - 1, "i32": -(2**31),
         "u64": 2**64 - 1, "i64": -(2**63), "f32": _f32(-3.4e38), "f64": -1.7976931348623157e308},
        {"b": False, "u8": 0, "i8": 127, "u16": 0, "i16": 32767, "u32": 0, "i32": 2**31 - 1, "u64": 0, "i64": 2**63 - 1,
         "f32": _f32(1.1754944e-38), "f64": 5e-324},
    ]
    for ext in extremes:
        v = {**ext, "s": "héllo rover", "bounded_s": "", "stamp": {"sec": -5, "nanosec": 999_999_999},
             "f32_fixed": [_f32(0.1), _f32(-2.5), _f32(1e-7)], "u8_seq": [0, 1, 254, 255], "i16_bounded": [-1, 2],
             "s_fixed": ["", "x"],
             "inner": {"flag": True, "big": 2**63 + 7, "name": "a", "t": {"sec": 1, "nanosec": 2}},
             "inner_seq": [{"flag": False, "big": 1, "name": "", "t": {"sec": 0, "nanosec": 0}},
                           {"flag": True, "big": 2, "name": "zz", "t": {"sec": 3, "nanosec": 4}}],
             "tail": 1.0 / 3.0}
        raw = mbm.encode(prim_defs, "dyx3_interfaces/AllPrims", v)
        assert raw[:4] == b"\x00\x01\x00\x00"
        res = mbm.decode(prim_defs, "dyx3_interfaces/AllPrims", raw, tolerant=False)
        assert res.value == v
        assert res.missing == [] and res.extra_bytes == 0
    nan = mbm.decode(prim_defs, "dyx3_interfaces/AllPrims",
                     mbm.encode(prim_defs, "dyx3_interfaces/AllPrims",
                                {**mbm.default_value(prim_defs, "dyx3_interfaces/AllPrims"), "f64": float("nan")})).value
    assert math.isnan(nan["f64"])


def test_cdr_layout_is_aligned_relative_to_the_payload_start(tmp_path):
    d = tmp_path / "msg"
    d.mkdir()
    (d / "Layout.msg").write_text("uint8 a\nfloat64 b\nstring s\nuint16 c\nbuiltin_interfaces/Time t\nuint64 q\n")
    defs = mbm.MsgDefs.from_dir(d)
    v = {"a": 1, "b": 2.0, "s": "ab", "c": 3, "t": {"sec": 4, "nanosec": 5}, "q": 6}
    raw = mbm.encode(defs, "dyx3_interfaces/Layout", v)
    # payload offsets: a@0, pad 1..7, b@8, len@16 (=3), "ab\0"@20..22, pad 23, c@24, pad 26..27, sec@28, nsec@32, pad 36..39, q@40
    expected = (b"\x00\x01\x00\x00" + b"\x01" + b"\x00" * 7 + struct.pack("<d", 2.0) + struct.pack("<I", 3) + b"ab\x00"
                + b"\x00" + struct.pack("<H", 3) + b"\x00\x00" + struct.pack("<iI", 4, 5) + b"\x00" * 4 + struct.pack("<Q", 6))
    assert raw == expected
    assert mbm.decode(defs, "dyx3_interfaces/Layout", raw + b"\x00\x00").value == v  # trailing padding is accepted


def test_tolerant_decode_of_a_bag_older_than_the_definitions(tmp_path):
    d_old, d_new = tmp_path / "old", tmp_path / "new"
    d_old.mkdir()
    d_new.mkdir()
    (d_old / "S.msg").write_text("builtin_interfaces/Time stamp\nfloat32 x\n")
    (d_new / "S.msg").write_text("builtin_interfaces/Time stamp\nfloat32 x\n# appended\nfloat32 y\nbool z\n")
    raw = mbm.encode(mbm.MsgDefs.from_dir(d_old), "dyx3_interfaces/S", {"stamp": {"sec": 1, "nanosec": 2}, "x": 0.5})
    res = mbm.decode(mbm.MsgDefs.from_dir(d_new), "dyx3_interfaces/S", raw)
    assert res.value == {"stamp": {"sec": 1, "nanosec": 2}, "x": 0.5, "y": None, "z": None}
    assert res.missing == ["y", "z"]
    with pytest.raises(mbm.TruncatedBuffer):
        mbm.decode(mbm.MsgDefs.from_dir(d_new), "dyx3_interfaces/S", raw, tolerant=False)
    newer = mbm.decode(mbm.MsgDefs.from_dir(d_old), "dyx3_interfaces/S",
                       mbm.encode(mbm.MsgDefs.from_dir(d_new), "dyx3_interfaces/S",
                                  {"stamp": {"sec": 1, "nanosec": 2}, "x": 0.5, "y": 1.0, "z": True}))
    assert newer.extra_bytes >= 4  # a bag newer than the definitions is reported, not silently accepted


def _random_value(defs, type_name, rng):
    def one(t):
        if t == "bool":
            return rng.random() < 0.5
        if t in ("float32",):
            return _f32(rng.uniform(-1e3, 1e3))
        if t == "float64":
            return rng.uniform(-1e6, 1e6)
        if t in mbm.PRIMITIVES:
            fmt, size = mbm.PRIMITIVES[t]
            signed = fmt.islower()
            bits = 8 * size
            lo, hi = (-(2 ** (bits - 1)), 2 ** (bits - 1) - 1) if signed else (0, 2**bits - 1)
            return rng.randint(lo, hi)
        if t == "string":
            return "".join(rng.choice("abcXYZ_09 é") for _ in range(rng.randint(0, 9)))
        return _random_value(defs, t, rng)

    out = {}
    for f in defs.get(type_name).fields:
        if f.array is None:
            out[f.name] = one(f.type)
        elif f.array < 0:
            out[f.name] = [one(f.type) for _ in range(rng.randint(0, 5))]
        else:
            out[f.name] = [one(f.type) for _ in range(f.array)]
    return out


@pytest.mark.parametrize("name", ALL_MSGS)
def test_every_dyx3_interfaces_definition_round_trips(defs, name):
    t = f"dyx3_interfaces/msg/{name}"
    for v in (mbm.default_value(defs, t), _random_value(defs, t, random.Random(name))):
        raw = mbm.encode(defs, t, v)
        res = mbm.decode(defs, t, raw, tolerant=False)
        assert res.value == v
        assert res.missing == [] and res.extra_bytes == 0


def _rosbags_to_plain(o):
    if dataclasses.is_dataclass(o):
        # rosbags carries constants (ROS 2: UPPER_CASE) and __msgtype__ as dataclass fields; message fields are lower case
        return {f.name: _rosbags_to_plain(getattr(o, f.name)) for f in dataclasses.fields(o)
                if f.name != "__msgtype__" and not f.name.isupper()}
    if isinstance(o, np.ndarray):
        return [x.item() for x in o]
    if isinstance(o, (list, tuple)):
        return [_rosbags_to_plain(x) for x in o]
    if isinstance(o, np.generic):
        return o.item()
    return o


@pytest.mark.parametrize("name", ALL_MSGS)
def test_codec_agrees_with_the_rosbags_cdr_implementation(defs, name):
    typesys = pytest.importorskip("rosbags.typesys")
    store = typesys.get_typestore(typesys.Stores.ROS2_HUMBLE)
    types = {}
    for p in MSGDIR.glob("*.msg"):
        types.update(typesys.get_types_from_msg(p.read_text(), f"dyx3_interfaces/msg/{p.stem}"))
    store.register(types)
    t = f"dyx3_interfaces/msg/{name}"
    v = _random_value(defs, t, random.Random("rosbags" + name))
    raw = mbm.encode(defs, t, v)
    theirs = store.deserialize_cdr(raw, t)
    assert _rosbags_to_plain(theirs) == v
    back = bytes(store.serialize_cdr(theirs, t))
    assert back.rstrip(b"\x00") == raw.rstrip(b"\x00")  # identical up to trailing alignment padding


# ------------------------------------------------------------------------------------------------------------------------------
# Synthetic run
# ------------------------------------------------------------------------------------------------------------------------------

DT_NS = 20_000_000          # 50 Hz
BASE_NS = 1_791_000_000_000_000_000
PHASE = 0.3                 # sinusoid phase, rad: keeps samples off exact zeros
ENTRY_XT = 0.02             # leg 1 entry offset (first 0.5 m), m
LEG2_XT = -0.01             # leg 2 constant offset after the pivot, m
CREEP_PATTERN = [0.05, 0.0, -0.05, 0.05, -0.05, 0.0, 0.05, -0.05]  # non-zero signs + - + - + -: 5 reversals (3 forward->reverse)


def _phases(creep_s):
    """(name, t0, t1) on the RppStatus clock, seconds."""
    p = [("entry", 0.0, 1.3), ("steady", 1.3, 11.3), ("brake", 11.3, 13.3), ("stop1", 13.3, 13.8),
         ("pivot1", 13.8, 15.8), ("stop2", 15.8, 16.0), ("pivot2", 16.0, 16.4), ("stop3", 16.4, 16.6),
         ("pivot3", 16.6, 17.0), ("stop4", 17.0, 17.4), ("leg2", 17.4, 20.4)]
    c1 = 20.4 + creep_s
    p += [("creep", 20.4, c1), ("stop5", c1, c1 + 0.6), ("complete", c1 + 0.6, c1 + 1.1)]
    return p


def _rpp_sample(t, phases):
    """state, run_index, cross-track, heading error, commanded speed, commanded yaw rate, speed (true), yaw rate (true)."""
    name, t0, t1 = next(ph for ph in phases if ph[1] <= t < ph[2])
    st = {"entry": 1, "steady": 1, "brake": 1, "leg2": 1, "creep": 4, "complete": 5}.get(name, 2)
    if name.startswith("pivot"):
        st = 3
    run = 0 if t < 13.8 else 1
    xt, he, v, wz = 0.0, 0.001, 0.0, 0.0
    if name == "entry":
        xt, v = ENTRY_XT, 0.4
    elif name == "steady":
        xt, v = 0.01 * math.sin(2 * math.pi * 0.5 * (t - 1.3) + PHASE), 0.6
    elif name == "brake":
        xt, v = -0.015, 0.2
    elif name == "pivot1":
        he, wz = -1.5 + 0.45 * (t - t0), 0.45
    elif name in ("pivot2", "pivot3", "stop2", "stop3", "stop4"):
        he = 0.035
        wz = 0.06 if name.startswith("pivot") else 0.0
    elif name == "leg2":
        xt, v = LEG2_XT, 0.6
    elif name == "creep":
        k = int((t - t0) / ((t1 - t0) / len(CREEP_PATTERN)))
        v = CREEP_PATTERN[min(k, len(CREEP_PATTERN) - 1)]
        xt = 0.002
    return st, run, xt, he, v, wz


def build_run(root: Path, defs, creep_s=7.0) -> Path:
    run = root / "2026-10-10_120000_mission_0042"
    (run / "rosbag2").mkdir(parents=True)
    (run / "manifest.json").write_text(json.dumps({"schema": 1, "run_id": run.name, "mission_id": 42, "run_index": 0,
                                                   "start_utc": "2026-10-10T12:00:00Z"}))
    (run / "summary.json").write_text(json.dumps({"schema": 1, "final_state": "COMPLETED", "duration_s": 30.0}))
    phases = _phases(creep_s)
    t_end = phases[-1][2]
    n = round(t_end / 0.02)
    con = sqlite3.connect(run / "rosbag2" / "rosbag2_0.db3")
    con.execute("CREATE TABLE topics(id INTEGER PRIMARY KEY, name TEXT NOT NULL, type TEXT NOT NULL, "
                "serialization_format TEXT NOT NULL, offered_qos_profiles TEXT NOT NULL)")
    con.execute("CREATE TABLE messages(id INTEGER PRIMARY KEY, topic_id INTEGER NOT NULL, timestamp INTEGER NOT NULL, "
                "data BLOB NOT NULL)")
    topic_ids: dict[str, int] = {}
    rows = []

    def put(topic, typ, ts_ns, value=None, raw=None):
        if topic not in topic_ids:
            topic_ids[topic] = len(topic_ids) + 1
            con.execute("INSERT INTO topics VALUES (?,?,?,?,?)", (topic_ids[topic], topic, typ, "cdr", ""))
        rows.append((topic_ids[topic], int(ts_ns), raw if raw is not None else mbm.encode(defs, typ, value)))

    def msg(typ, **kw):
        v = mbm.default_value(defs, typ)
        for k, x in kw.items():
            assert k in v, k
            v[k] = x
        return v

    def stamp(ns):
        return {"sec": int(ns // 1_000_000_000), "nanosec": int(ns % 1_000_000_000)}

    travel = 0.0
    run_travel_reset_done = False
    for k in range(n):
        t = k * 0.02
        g = BASE_NS + k * DT_NS
        st, ri, xt, he, v, wz = _rpp_sample(t, phases)
        if st == 1 and t >= 17.4 and not run_travel_reset_done:
            travel, run_travel_reset_done = 0.0, True
        travel += abs(v) * 0.02 if st in (1, 4) else 0.0
        meas_v = 0.04 if (st == 4 and v != 0.0) else (v if st == 1 else 0.0)
        meas_wz = 0.9 * wz if wz >= 0.4 else 0.0
        # VehicleState: 50 Hz on the bag clock with ONE 30 ms gap (at t = 5.0 s); px4 sample stamps exactly 50 Hz
        vs_ts = g + (10_000_000 if t >= 5.0 - 1e-9 else 0)
        put("/dyx3/vehicle_state", "dyx3_interfaces/msg/VehicleState", vs_ts,
            msg("dyx3_interfaces/msg/VehicleState", stamp=stamp(vs_ts), px4_sample_stamp=stamp(g - 3_000_000),
                position_valid=True, velocity_valid=True, attitude_valid=True, velocity_north_mps=meas_v,
                yaw_rate_radps=meas_wz, q_frd_to_ned=[1.0, 0.0, 0.0, 0.0]))
        put("/dyx3/rpp/status", "dyx3_interfaces/msg/RppStatus", g + 1_000_000,
            msg("dyx3_interfaces/msg/RppStatus", stamp=stamp(g), state=st, mission_id=42, run_index=ri,
                cross_track_right_m=_f32(xt), heading_error_rad=_f32(he), commanded_speed_mps=_f32(v),
                commanded_yaw_rate_radps=_f32(wz), loop_jitter_us=100.0 + (k % 7), loop_jitter_max_us=900.0,
                loop_overrun_count=1 if t > 10 else 0, path_travel_m=_f32(travel),
                dist_to_goal_m=_f32(max(0.004, 0.6 - (t - 20.4) * 0.3)) if t >= 20.4 else 2.0,
                conditioned_execution_sha256="ab" * 32))
        put("/dyx3/rpp/motion_setpoint", "dyx3_interfaces/msg/MotionSetpoint", g + 2_000_000,
            msg("dyx3_interfaces/msg/MotionSetpoint", stamp=stamp(g), seq=k, mode=1, speed_body_x=_f32(v), valid=True))
        put("/dyx3/motion_guard/command", "dyx3_interfaces/msg/MotionSetpoint", g + 3_000_000,
            msg("dyx3_interfaces/msg/MotionSetpoint", stamp=stamp(g), seq=k, mode=1, speed_body_x=_f32(v), valid=True))
        clamped = k in (100, 200, 300)
        put("/dyx3/motion_guard/status", "dyx3_interfaces/msg/MotionSetpointStatus", g + 3_500_000,
            msg("dyx3_interfaces/msg/MotionSetpointStatus", stamp=stamp(g), input_seq=k, accepted=True, clamped=clamped,
                reason_code=7 if clamped else 0))
        if k % 5 == 0:
            j = k // 5
            put("/dyx3/px4_link/status", "dyx3_interfaces/msg/Px4LinkStatus", g + 4_000_000,
                msg("dyx3_interfaces/msg/Px4LinkStatus", stamp=stamp(g), session_alive=True, handshake_ok=True,
                    fault=5 if j in (10, 11) else 0, loop_overrun_count=2 if j >= 11 else (1 if j >= 10 else 0),
                    command_gap_events=0, session_resets=0, pose_to_write_age_valid=j > 0,
                    pose_to_write_age_s=_f32(0.003 + 0.001 * (j % 5)), pose_to_write_age_max_s=_f32(0.0145)))
            put("/dyx3/safety_gate", "dyx3_interfaces/msg/SafetyGateStatus", g + 4_500_000,
                msg("dyx3_interfaces/msg/SafetyGateStatus", stamp=stamp(g), ok=j != 3, reason_code=6 if j == 3 else 0,
                    pre_arm_ok=True))
        if k % 10 == 0:
            j = k // 10
            put("/dyx3/rtk_status", "dyx3_interfaces/msg/RtkStatus", g + 5_000_000,
                msg("dyx3_interfaces/msg/RtkStatus", stamp=stamp(g), fix_type=5 if j == 2 else 6, corrections_fresh=True,
                    correction_age_s=_f32(0.4 if j == 7 else 0.2), horizontal_accuracy_m=_f32(0.02 if j == 4 else 0.012)))
            put("/dyx3/estimator_health", "dyx3_interfaces/msg/EstimatorHealth", g + 5_500_000,
                msg("dyx3_interfaces/msg/EstimatorHealth", stamp=stamp(g), flags_valid=True, test_ratios_valid=True,
                    gnss_yaw_fusion_intended=True, reject_yaw=j == 9))
            put("/dyx3/mission/state", "dyx3_interfaces/msg/MissionState", g + 6_000_000,
                msg("dyx3_interfaces/msg/MissionState", stamp=stamp(g), state=5 if t >= phases[-1][1] else 3, mission_id=42,
                    reason_detail="rpp reports complete" if t >= phases[-1][1] else "rpp acknowledged the execution",
                    waiting_on=7 if t >= phases[-1][1] else 0))
    for t, idx, err in ((13.5, 0, 0.004), (t_end - 0.6, 1, 0.017)):
        g = BASE_NS + int(t * 1e9)
        put("/dyx3/mission/point_result", "dyx3_interfaces/msg/PointResult", g,
            msg("dyx3_interfaces/msg/PointResult", stamp=stamp(g), mission_id=42, point_index=idx, result_code=1,
                error_m=_f32(err)))
    put("/rosout", "rcl_interfaces/msg/Log", BASE_NS, raw=b"\x00\x01\x00\x00not-a-dyx3-message")
    con.executemany("INSERT INTO messages(topic_id, timestamp, data) VALUES (?,?,?)", rows)
    con.commit()
    con.close()
    return run


@pytest.fixture(scope="module")
def synthetic(tmp_path_factory, defs):
    root = tmp_path_factory.mktemp("bags")
    run = build_run(root, defs)
    data = mbm.load_run(run, defs=defs)
    return run, data, mbm.analyze([data])


def test_reader_decodes_dyx3_topics_and_skips_others(synthetic):
    _, data, doc = synthetic
    assert "/rosout" not in data.topics and data.skipped_topics == {"/rosout": "rcl_interfaces/msg/Log"}
    dec = doc["runs"][0]["decode"]
    assert dec["warnings"] == []
    assert all(v["failed"] == 0 and v["truncated"] == 0 for v in dec["topics"].values())


def test_rates(synthetic):
    _, data, doc = synthetic
    r = doc["runs"][0]["rates"]
    n = len(data.series(mbm.T_RPP))
    vs = r["vehicle_state_receipt"]
    assert vs["n"] == n
    assert vs["max_gap_ms"] == pytest.approx(30.0, abs=1e-6)
    assert vs["min_hz"] == pytest.approx(1000.0 / 30.0, rel=1e-9)
    assert vs["p50_hz"] == pytest.approx(50.0, rel=1e-9)
    assert vs["mean_hz"] == pytest.approx((n - 1) / ((n - 1) * 0.02 + 0.010), rel=1e-9)
    px4 = r["vehicle_state_px4_sample_stamp"]
    assert px4["max_gap_ms"] == pytest.approx(20.0, abs=1e-6) and px4["min_hz"] == pytest.approx(50.0, rel=1e-9)
    assert r["rpp_motion_setpoint"]["mean_hz"] == pytest.approx(50.0, rel=1e-9)
    assert r["motion_guard_command"]["p99_hz"] == pytest.approx(50.0, rel=1e-9)
    assert r["px4_link_status"]["p50_hz"] == pytest.approx(10.0, rel=1e-9)
    assert r["px4_link_status"]["max_gap_ms"] == pytest.approx(100.0, abs=1e-6)


def test_latency_and_loop_health(synthetic):
    _, _, doc = synthetic
    lat = doc["runs"][0]["latency"]
    p = lat["pose_to_write"]
    ages = np.array([np.float32(0.003 + 0.001 * (j % 5)) for j in range(1, p["n"])], dtype=float)
    assert p["n_valid"] == p["n"] - 1  # the first sample is flagged invalid
    assert p["p50_ms"] == pytest.approx(np.percentile(ages, 50) * 1e3, rel=1e-6)
    assert p["p95_ms"] == pytest.approx(np.percentile(ages, 95) * 1e3, rel=1e-6)
    assert p["max_of_max_ms"] == pytest.approx(14.5, abs=1e-3)
    assert lat["rpp"]["loop_jitter_max_us"] == 900.0 and lat["rpp"]["loop_overrun_count_last"] == 1.0
    link = lat["px4_link"]
    assert link["loop_overrun_count_last"] == 2.0 and link["loop_overrun_count_delta"] == 2.0
    assert link["command_gap_events_last"] == 0.0 and link["session_resets_last"] == 0.0
    assert link["fault_histogram"] == {"NONE": p["n"] - 2, "LOOP_OVERRUN": 2}


def test_steady_tracking_oscillation(synthetic):
    _, _, doc = synthetic
    run0 = next(r for r in doc["tracking"]["runs"] if r["run_index"] == 0)
    seg = run0["steady"]["segments"]
    assert len(seg) == 1
    s = seg[0]
    assert run0["steady"]["vmax_cmd_mps"] == pytest.approx(0.6, rel=1e-6)
    assert s["duration_s"] == pytest.approx(10.0, abs=0.03)
    # sin(pi*(t-1.3) + PHASE) is zero at t - 1.3 = k - PHASE/pi: k = 1..10 lie inside the 0..9.98 s window
    assert s["xt_zero_crossings"] == 10
    assert s["xt_dominant_period_s"] == pytest.approx(2.0, abs=0.05)
    assert s["xt_std_m"] == pytest.approx(0.01 / math.sqrt(2), rel=0.02)
    assert s["xt_p2p_m"] == pytest.approx(0.02, rel=0.01)
    assert abs(s["xt_mean_m"]) < 5e-4
    ts = run0["per_state"]["TRACKING"]
    assert ts["xt_abs_max"] == pytest.approx(ENTRY_XT, rel=1e-6)


def test_corner_entry_and_braking_tail(synthetic):
    _, _, doc = synthetic
    runs = {r["run_index"]: r for r in doc["tracking"]["runs"]}
    e0 = runs[0]["corner_entry"]
    assert len(e0) == 1 and e0[0]["after_pivot"] is False
    assert e0[0]["xt_mean_m"] == pytest.approx(ENTRY_XT, rel=1e-6)
    e1 = runs[1]["corner_entry"]
    assert len(e1) == 1 and e1[0]["after_pivot"] is True
    assert e1[0]["xt_mean_m"] == pytest.approx(LEG2_XT, rel=1e-6)
    # first 0.5 m at 0.6 m/s = 0.83 s = 42 samples of 20 ms
    assert e1[0]["n"] == pytest.approx(42, abs=1)
    bt = runs[0]["braking_tail"]
    assert len(bt) == 1  # leg 2 ends in CREEP, not STOPPING: no braking tail there
    # last 1.0 m: 0.4 m of braking at 0.2 m/s (2 s, -1.5 cm) + 0.6 m of the steady leg (1 s)
    assert bt[0]["duration_s"] == pytest.approx(3.0, abs=0.05)
    assert runs[1]["braking_tail"] == []


def test_pivot_episode_cycles_and_yaw_ratio(synthetic):
    _, _, doc = synthetic
    eps = doc["pivot"]["episodes"]
    assert len(eps) == 1
    e = eps[0]
    assert e["cycles"] == 3
    assert e["t_start"] == pytest.approx(13.8 + 0.001, abs=0.021)
    assert e["duration_s"] == pytest.approx(17.0 - 13.8, abs=0.021)
    assert e["he_release_rad"] == pytest.approx(0.035, rel=1e-5)
    assert e["next_state"] == "TRACKING" and not e["starts_at_data_start"] and not e["ends_at_data_end"]
    fr = doc["pivot"]["full_rate_yaw"]
    assert fr["n"] == 100  # pivot1: 2 s of 50 Hz at 0.45 rad/s; the 0.06 rad/s re-pivots are below the full-rate threshold
    assert fr["ratio_p50"] == pytest.approx(0.9, abs=0.01)
    assert e["full_rate_yaw_ratio_p50"] == pytest.approx(0.9, abs=0.01)


def test_endpoint(synthetic):
    _, _, doc = synthetic
    ep = doc["endpoint"]
    assert ep["present"] and ep["run_index"] == 1
    assert ep["speed_sign_reversals"] == 5
    assert ep["forward_to_reverse"] == 3
    assert ep["creep_s"] == pytest.approx(7.0, abs=0.021)
    assert ep["stopping_s"] == pytest.approx(0.6, abs=0.021)
    assert ep["finished_by_timeout"] is False
    assert ep["next_state"] == "COMPLETE"
    assert ep["measured_speed_max_last6s_mps"] == pytest.approx(0.04, rel=1e-6)
    assert ep["final_dist_to_goal_m"] == pytest.approx(0.004, rel=1e-5)


def test_endpoint_timeout_flag(tmp_path, defs):
    run = build_run(tmp_path, defs, creep_s=8.0)
    doc = mbm.analyze([mbm.load_run(run, defs=defs)])
    assert doc["endpoint"]["creep_s"] == pytest.approx(8.0, abs=0.021)
    assert doc["endpoint"]["finished_by_timeout"] is True


def test_mission_section(synthetic):
    _, _, doc = synthetic
    m = doc["mission"]
    assert [(x["from"], x["state"]) for x in m["transitions"]] == [(None, "RUNNING"), ("RUNNING", "COMPLETED")]
    assert m["transitions"][1]["waiting_on"] == "OFFBOARD_RELEASE"
    assert [(p["point_index"], p["result"]) for p in m["points"]] == [(0, "COMPLETED"), (1, "COMPLETED")]
    assert [p["error_m"] for p in m["points"]] == pytest.approx([0.004, 0.017], rel=1e-6)
    assert m["guard"]["clamped"] == 3 and m["guard"]["refused"] == 0
    assert m["guard"]["reason_histogram"]["LIMIT_CLAMPED"] == 3
    assert m["safety_gate"]["ok_histogram"]["false"] == 1
    assert m["safety_gate"]["reason_histogram_when_not_ok"] == {"RTK_GATE": 1}
    assert m["rtk"]["fix_histogram"]["RTK_FLOAT"] == 1
    assert m["rtk"]["hrms_max_m"] == pytest.approx(0.02, rel=1e-6)
    assert m["rtk"]["correction_age_max_s"] == pytest.approx(0.4, rel=1e-6)
    assert m["estimator"]["faults_ever_set"] == ["reject_yaw"]


def test_gate_rows(synthetic):
    _, data, doc = synthetic
    rs = data.series(mbm.T_RPP)
    st = rs.col("state")
    x = rs.col("cross_track_right_m")[(st == 1) | (st == 4)]
    g = doc["gate"]
    assert "NOT surveyed truth" in g["label"]
    assert g["n"] == x.size
    assert g["rms_m"] == pytest.approx(float(np.sqrt(np.mean(x * x))), rel=1e-9)
    assert g["max_abs_m"] == pytest.approx(ENTRY_XT, rel=1e-6)


def test_cli_writes_metrics_json_and_markdown(tmp_path, defs, capsys):
    run = build_run(tmp_path, defs)
    assert mbm.main([str(run), "--msgdir", str(MSGDIR)]) == 0
    out = capsys.readouterr().out
    assert "## 7. Gate-table rows" in out and "NOT surveyed truth" in out
    doc = json.loads((run / "metrics.json").read_text())
    assert doc["pivot"]["episodes"][0]["cycles"] == 3
    # a directory of runs: one mission, a directory-level metrics.json, --json on stdout
    assert mbm.main([str(tmp_path), "--msgdir", str(MSGDIR), "--json"]) == 0
    top = json.loads(capsys.readouterr().out)
    assert len(top["missions"]) == 1 and top["missions"][0]["mission_id"] == 42
    assert (tmp_path / "metrics.json").is_file()


def test_zstd_compressed_split_when_zstandard_is_available(tmp_path, defs):
    zstandard = pytest.importorskip("zstandard")
    run = build_run(tmp_path, defs)
    db = run / "rosbag2" / "rosbag2_0.db3"
    (run / "rosbag2" / "rosbag2_0.db3.zstd").write_bytes(zstandard.ZstdCompressor().compress(db.read_bytes()))
    db.unlink()
    doc = mbm.analyze([mbm.load_run(run, defs=defs)])
    assert doc["pivot"]["episodes"][0]["cycles"] == 3


def test_definitions_from_git_match_the_working_tree_for_head(defs):
    try:
        g = mbm.MsgDefs.from_git(REPO, "HEAD")
    except (OSError, FileNotFoundError, mbm.subprocess.CalledProcessError) as e:  # not a git checkout
        pytest.skip(f"git unavailable: {e}")
    assert "RppStatus" in g.names
    assert [f.name for f in g.get("dyx3_interfaces/msg/PointResult").fields] == \
        [f.name for f in defs.get("dyx3_interfaces/msg/PointResult").fields]
