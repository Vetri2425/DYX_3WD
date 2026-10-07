"""Read-only view of the recorder's run directories (docs/contracts/dyx3_recorder.md section 1)."""

from __future__ import annotations

import json
import os
import re

_ID = re.compile(r"^[0-9A-Za-z_.-]{1,128}$")


def valid_run_id(run_id: str) -> bool:
    return bool(_ID.match(run_id)) and run_id not in (".", "..")


def _read_json(path: str) -> dict | None:
    try:
        with open(path, encoding="utf-8") as fh:
            doc = json.load(fh)
        return doc if isinstance(doc, dict) else None
    except (OSError, ValueError):
        return None


def get_run(runs_dir: str, run_id: str) -> dict | None:
    if not valid_run_id(run_id):
        return None
    base = os.path.join(runs_dir, run_id)
    if not os.path.isdir(base):
        return None
    return {
        "run_id": run_id,
        "manifest": _read_json(os.path.join(base, "manifest.json")),
        "summary": _read_json(os.path.join(base, "summary.json")),  # None while the run is still open
        "has_ulog": os.path.exists(os.path.join(base, "ulog", "stream.ulg")),
        "has_bag": os.path.isdir(os.path.join(base, "rosbag2")),
    }


def list_runs(runs_dir: str, limit: int = 200) -> list[dict]:
    try:
        names = [n for n in os.listdir(runs_dir) if valid_run_id(n) and os.path.isdir(os.path.join(runs_dir, n))]
    except OSError:
        return []
    names.sort(reverse=True)  # names start with a UTC timestamp
    out = []
    for n in names[:limit]:
        run = get_run(runs_dir, n)
        if run is not None:
            out.append(run)
    return out
