"""pytest wiring for the carried path-engine tests.

Four of the prototype's tests (TestSegmentHelpers, TestPathManagerEntityOrder and two builtin-path
cases) exercise `server/path_manager.py` — the old mission-orchestrator that Phase 2 replaces. It is
NOT carried, so those tests are skipped here with a reason (they are not passing, they are not run).
"""
import importlib.util

import pytest

_HAS_PATH_MANAGER = importlib.util.find_spec("path_manager") is not None


def pytest_collection_modifyitems(config, items):
    if _HAS_PATH_MANAGER:
        return
    skip = pytest.mark.skip(reason="needs server/path_manager.py (old orchestrator; replaced in Phase 2)")
    for item in items:
        if "path_mgr" in getattr(item, "fixturenames", ()) or "TestPathManagerEntityOrder" in item.nodeid:
            item.add_marker(skip)
