import json
import os
from pathlib import Path

from dyx3_backend.auth.tokens import Role, TokenStore, add_token, hash_token


def test_verify_accepts_only_known_tokens_and_reports_role():
    s = TokenStore([("a", Role.VIEWER, hash_token("t1")), ("b", Role.OPERATOR, hash_token("t2"))])
    assert s.verify("t1").role is Role.VIEWER
    assert s.verify("t2").can(Role.OPERATOR)
    assert not s.verify("t1").can(Role.OPERATOR)
    for bad in (None, "", "t3", "T1", "t1 "):
        assert s.verify(bad) is None


def test_missing_corrupt_or_empty_file_denies_everything(tmp_path):
    assert len(TokenStore.load(str(tmp_path / "nope.json"))) == 0
    p = tmp_path / "auth.json"
    p.write_text("not json")
    assert TokenStore.load(str(p)).verify("anything") is None
    p.write_text(json.dumps({"version": 2, "tokens": [{"name": "x", "role": "operator", "sha256": hash_token("t")}]}))
    assert TokenStore.load(str(p)).verify("t") is None  # unknown schema version
    p.write_text(json.dumps({"version": 1, "tokens": []}))
    assert TokenStore.load(str(p)).verify("t") is None


def test_malformed_entries_never_grant_access(tmp_path):
    p = tmp_path / "auth.json"
    p.write_text(json.dumps({"version": 1, "tokens": [
        {"name": "short", "role": "operator", "sha256": "abc"},
        {"name": "role", "role": "root", "sha256": hash_token("t")},
        {"name": "ok", "role": "viewer", "sha256": hash_token("good")},
    ]}))
    s = TokenStore.load(str(p))
    assert s.verify("t") is None
    assert s.verify("good").name == "ok"


def test_add_token_stores_only_the_hash_with_private_permissions(tmp_path):
    p = str(tmp_path / "state" / "auth.json")
    tok = add_token(p, "tablet-1", Role.OPERATOR)
    raw = Path(p).read_text()
    assert tok not in raw and hash_token(tok) in raw
    assert (os.stat(p).st_mode & 0o777) == 0o600
    assert TokenStore.load(p).verify(tok).role is Role.OPERATOR
    try:
        add_token(p, "tablet-1", Role.VIEWER)
    except ValueError:
        pass
    else:  # pragma: no cover
        raise AssertionError("duplicate name accepted")
