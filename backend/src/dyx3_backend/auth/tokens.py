"""Static bearer tokens with a role, stored hashed. Contract: docs/contracts/backend.md section 2.

DERIVED — NOT FROM V1 SPEC (OPEN): a user/password model, pairing, rotation and expiry are not designed.
Fail closed: no file, an unreadable file or an empty list means every protected route is denied.
"""

from __future__ import annotations

import argparse
import hashlib
import hmac
import json
import os
import secrets
import tempfile
from dataclasses import dataclass
from enum import IntEnum


class Role(IntEnum):
    VIEWER = 1
    OPERATOR = 2


@dataclass(frozen=True)
class Identity:
    name: str
    role: Role

    def can(self, needed: Role) -> bool:
        return self.role >= needed


def hash_token(token: str) -> str:
    return hashlib.sha256(token.encode("utf-8")).hexdigest()


class TokenStore:
    def __init__(self, entries: list[tuple[str, Role, str]] | None = None) -> None:
        self._entries = list(entries or [])

    def __len__(self) -> int:
        return len(self._entries)

    @staticmethod
    def load(path: str) -> TokenStore:
        try:
            with open(path, encoding="utf-8") as fh:
                doc = json.load(fh)
            if doc.get("version") != 1:
                return TokenStore()
            entries = []
            for t in doc.get("tokens", []):
                try:
                    h = str(t["sha256"]).lower()
                    if len(h) != 64 or any(c not in "0123456789abcdef" for c in h):
                        continue
                    entries.append((str(t["name"]), Role[str(t["role"]).upper()], h))
                except (KeyError, TypeError):
                    continue  # a malformed entry never grants anything, and never discards its valid neighbours
            return TokenStore(entries)
        except (OSError, ValueError, KeyError, TypeError):
            return TokenStore()

    def verify(self, token: str | None) -> Identity | None:
        if not token:
            return None
        digest = hash_token(token)
        found: Identity | None = None
        for name, role, h in self._entries:  # no early exit: constant work per entry
            if hmac.compare_digest(digest, h) and found is None:
                found = Identity(name, role)
        return found


def new_token(name: str, role: Role) -> tuple[str, dict]:
    token = secrets.token_urlsafe(32)
    return token, {"name": name, "role": role.name.lower(), "sha256": hash_token(token)}


def add_token(path: str, name: str, role: Role) -> str:
    """Append a token entry atomically (0600) and return the new token (shown once, never stored)."""
    try:
        with open(path, encoding="utf-8") as fh:
            doc = json.load(fh)
    except (OSError, ValueError):
        doc = {"version": 1, "tokens": []}
    if any(t.get("name") == name for t in doc.get("tokens", [])):
        raise ValueError(f"a token named {name!r} already exists")
    token, entry = new_token(name, role)
    doc.setdefault("tokens", []).append(entry)
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    fd, tmp = tempfile.mkstemp(dir=os.path.dirname(path) or ".", prefix=".auth-")
    with os.fdopen(fd, "w", encoding="utf-8") as fh:
        json.dump(doc, fh, indent=2)
    os.chmod(tmp, 0o600)
    os.replace(tmp, path)
    return token


def main(argv: list[str] | None = None) -> int:
    from dyx3_backend.config.settings import Settings

    ap = argparse.ArgumentParser(description="Manage DYX3 backend bearer tokens")
    sub = ap.add_subparsers(dest="cmd", required=True)
    c = sub.add_parser("create")
    c.add_argument("--name", required=True)
    c.add_argument("--role", choices=["viewer", "operator"], required=True)
    c.add_argument("--file", default=Settings.from_env().auth_path)
    a = ap.parse_args(argv)
    print(add_token(a.file, a.name, Role[a.role.upper()]))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
