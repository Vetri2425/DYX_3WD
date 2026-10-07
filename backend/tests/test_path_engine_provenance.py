"""The carried path engine must equal the PX4_DXP original, modulo the import prefix."""

import hashlib
import os
import re

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC = os.path.join(ROOT, "src", "dyx3_backend", "path_engine")
TESTS = os.path.join(ROOT, "tests", "path_engine")
_FWD = re.compile(r"^(\s*)(from|import) dyx3_backend\.path_engine\b", re.MULTILINE)


def _rows():
    with open(os.path.join(SRC, "ORIGIN.sha256"), encoding="utf-8") as fh:
        for line in fh:
            line = line.strip()
            if line and not line.startswith("#"):
                digest, path = line.split(None, 1)
                yield digest, path.strip()


def test_every_file_is_original_except_for_the_import_prefix():
    rows = list(_rows())
    assert len(rows) >= 45
    for digest, rel in rows:
        base, name = rel.split("/", 1)
        path = os.path.join(SRC if base == "src" else TESTS, name)
        with open(path, encoding="utf-8", newline="") as fh:
            text = fh.read()
        original = _FWD.sub(r"\1\2 path_engine", text)
        assert hashlib.sha256(original.encode()).hexdigest() == digest, f"{rel} was modified"


def test_no_unrecorded_engine_file():
    recorded = {rel for _, rel in _rows()}
    for root, _, files in os.walk(SRC):
        for f in files:
            if f.endswith((".py", ".md")) and "__pycache__" not in root:
                rel = "src/" + os.path.relpath(os.path.join(root, f), SRC).replace(os.sep, "/")
                assert rel in recorded, f"{rel} is not in ORIGIN.sha256"
