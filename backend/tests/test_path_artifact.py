"""Path artifact: determinism, strict decoding, content addressing, end-to-end on the corpus."""

import math
import os

import pytest

pytest.importorskip("ezdxf")
pytest.importorskip("geographiclib")

from dyx3_backend.mission import path_artifact as pa
from dyx3_backend.path_engine.engine import PathEngine

DATA = os.path.join(os.path.dirname(__file__), "data", "missions")
ENGINE_ID = "abcd1234abcd1234"
PTS = [(0.0, 0.0, 1), (0.05, 0.0, 3), (0.1, 0.0, 2), (0.1, 0.05, 0)]


def _enc(points=PTS, **kw):
    return pa.encode(points, engine_id=ENGINE_ID, meta=kw.get("meta", {"k": 1}))


def test_roundtrip_is_exact():
    art = pa.decode(_enc())
    assert [(p.north_m, p.east_m, p.flags) for p in art.points] == PTS
    assert art.version == 1 and art.engine_id == ENGINE_ID and art.meta == {"k": 1}
    assert art.points[1].spray and art.points[1].must_hit
    assert not art.points[2].spray and art.points[2].must_hit  # spray-OFF must-hit encodes as 2


def test_encoding_is_byte_deterministic_and_hash_is_of_the_bytes():
    a, b = _enc(), _enc()
    assert a == b
    assert pa.decode(a).sha256 == pa.sha256_hex(a)


def test_meta_key_order_does_not_change_the_hash():
    a = pa.encode(PTS, engine_id=ENGINE_ID, meta={"b": 2, "a": {"y": 1, "x": 2}})
    b = pa.encode(PTS, engine_id=ENGINE_ID, meta={"a": {"x": 2, "y": 1}, "b": 2})
    assert a == b


def test_volatile_metadata_is_stripped():
    a = pa.encode(PTS, engine_id=ENGINE_ID, meta={"planning_time_s": 0.1, "src": {"filepath": "/a/b"}})
    b = pa.encode(PTS, engine_id=ENGINE_ID, meta={"planning_time_s": 9.9, "src": {"filepath": "/c/d"}})
    assert a == b


def test_one_ulp_changes_the_hash():
    n = math.nextafter(0.1, 1.0)
    other = [*PTS[:-1], (PTS[-1][0], n, PTS[-1][2])]
    assert pa.sha256_hex(_enc(other)) != pa.sha256_hex(_enc())


def test_float_text_round_trips_every_bit():
    vals = [0.1, 1 / 3, 1e-300, 123456.789012345678, -0.0, 5e-324]
    pts = [(v, -v, 0) for v in vals]
    art = pa.decode(pa.encode(pts, engine_id=ENGINE_ID))
    for (n, e, _), p in zip(pts, art.points, strict=True):
        assert math.copysign(1, p.north_m) == math.copysign(1, n)
        assert p.north_m == n and p.east_m == e


def test_expected_hash_is_enforced():
    data = _enc()
    pa.decode(data, expected_sha256=pa.sha256_hex(data))
    with pytest.raises(pa.ArtifactError, match="sha256 mismatch"):
        pa.decode(data, expected_sha256="0" * 64)


@pytest.mark.parametrize(
    "mutate,why",
    [
        (lambda t: t.replace("DYX3PATH 1", "DYX3PATH 2", 1), "version"),
        (lambda t: t.replace("DYX3PATH", "DYX3PATX", 1), "magic"),
        (lambda t: t.replace("frame local_ned", "frame enu", 1), "frame"),
        (lambda t: t.replace("points 4", "points 5", 1), "count"),
        (lambda t: t.replace("end 4\n", "", 1), "truncated"),
        (lambda t: t.replace("\n", "\r\n"), "crlf"),
        (lambda t: t.replace("0.05 0.0 3", "nan 0.0 3", 1), "nan"),
        (lambda t: t.replace("0.05 0.0 3", "0.05 0.0 4", 1), "flags"),
        (lambda t: t.replace("0.05 0.0 3", "0.050 0.0 3", 1), "non-canonical float"),
        (lambda t: t.replace('meta {"k":1}', 'meta {"k": 1}', 1), "non-canonical meta"),
        (lambda t: t + "extra\n", "trailing line"),
        (lambda t: t.rstrip("\n"), "no final newline"),
    ],
)
def test_decode_rejects_corrupt_artifacts(mutate, why):
    bad = mutate(_enc().decode("ascii")).encode("ascii")
    with pytest.raises(pa.ArtifactError):
        pa.decode(bad)


def test_encode_refuses_bad_input():
    with pytest.raises(pa.ArtifactError):
        pa.encode([], engine_id=ENGINE_ID)
    with pytest.raises(pa.ArtifactError):
        pa.encode([(float("nan"), 0.0, 0)], engine_id=ENGINE_ID)
    with pytest.raises(pa.ArtifactError):
        pa.encode([(0.0, 0.0, 4)], engine_id=ENGINE_ID)
    with pytest.raises(pa.ArtifactError):
        pa.encode(PTS, engine_id="has space")
    with pytest.raises(ValueError):
        pa.encode(PTS, engine_id=ENGINE_ID, meta={"x": float("nan")})


def test_store_is_content_addressed_atomic_and_idempotent(tmp_path):
    data = _enc()
    digest, path = pa.store(str(tmp_path), data)
    assert os.path.basename(path) == digest + ".dyx3path"
    assert (os.stat(path).st_mode & 0o777) == 0o644
    assert pa.store(str(tmp_path), data) == (digest, path)
    assert sorted(os.listdir(tmp_path)) == [digest + ".dyx3path"]  # no temp files left
    assert pa.load(str(tmp_path), digest).sha256 == digest


def test_load_detects_a_file_that_does_not_hash_to_its_name(tmp_path):
    data = _enc()
    digest, path = pa.store(str(tmp_path), data)
    tampered = data.replace(b"0.05 0.0 3", b"0.05 0.0 1")
    with open(path, "wb") as fh:
        fh.write(tampered)
    with pytest.raises(pa.ArtifactError, match="sha256 mismatch"):
        pa.load(str(tmp_path), digest)
    with pytest.raises(pa.ArtifactError):
        pa.load(str(tmp_path), "XYZ")


def test_store_refuses_bytes_the_reader_would_refuse(tmp_path):
    with pytest.raises(pa.ArtifactError):
        pa.store(str(tmp_path), b"not an artifact\n")
    assert os.listdir(tmp_path) == []


# --- end to end on the archived corpus (the DXF/waypoint inputs that ARE in Git) -------------
# CHARACTERISATION of the carried engine as of 2026-10-07 (ezdxf 1.4.4, geographiclib 2.1): these
# values pin today's output so an accidental behaviour change is caught. They are NOT ground truth.
CORPUS = {
    "square_2x2.dxf": {"n": 161, "spray": 161, "must": 5, "mark": 8.0, "transit": 0.0},
    "soccer_pitch_fifa_edited.dxf": {"n": 19578, "spray": 17241, "must": 1309, "mark": 860.317323, "transit": 347.049473},
    "soccer_field_penalty_area.dxf": {"n": 8699, "spray": 8474, "must": 4204, "mark": 423.324878, "transit": 33.256087},
    "mission_straight_5m.waypoints": {"n": 105, "spray": 103, "must": 11, "mark": 4.994876, "transit": 0.1},
}


@pytest.mark.parametrize("name", sorted(CORPUS))
def test_corpus_mission_to_artifact_roundtrip(name, tmp_path):
    path = os.path.join(DATA, name)
    with open(path, "rb") as fh:
        src = fh.read()
    plan = PathEngine().plan_file(path)
    exp = CORPUS[name]
    assert len(plan.merged_waypoints) == exp["n"]
    assert sum(plan.spray_flags) == exp["spray"]
    assert sum(plan.must_hit) == exp["must"]
    assert plan.total_mark_length == pytest.approx(exp["mark"], abs=1e-5)
    assert plan.total_transit_length == pytest.approx(exp["transit"], abs=1e-5)

    data = pa.encode_plan(plan, engine_id=ENGINE_ID, source_name=path, source_bytes=src)
    art = pa.decode(data)
    assert len(art.points) == len(plan.merged_waypoints)
    for p, w, s, m in zip(art.points, plan.merged_waypoints, plan.spray_flags, plan.must_hit, strict=True):
        assert (p.north_m, p.east_m) == (w[0], w[1])  # bit-exact
        assert p.spray is bool(s) and p.must_hit is bool(m)
    assert art.meta["source"] == {"name": name, "sha256": pa.sha256_hex(src)}
    assert "filepath" not in str(art.meta) and "planning_time_s" not in str(art.meta)

    # planning twice (different wall-clock timings) must give the SAME artifact hash
    again = pa.encode_plan(
        PathEngine().plan_file(path), engine_id=ENGINE_ID, source_name=path, source_bytes=src
    )
    assert again == data
    digest, _ = pa.store(str(tmp_path), data)
    assert pa.load(str(tmp_path), digest).points == art.points
