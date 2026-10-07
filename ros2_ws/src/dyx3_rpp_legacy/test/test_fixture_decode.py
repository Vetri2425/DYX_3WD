"""Evidence test: decode RECORDED prototype ticks and compare with the prototype's own state.

The fixture is produced from a PX4_DXP rosbag by
    python3 tools/extract_legacy_decode_fixture.py <bag_dir> -o ros2_ws/src/dyx3_rpp_legacy/test/fixtures/legacy_decode_<name>.json
(bags are NOT in Git; see HANDOFF "LOCAL ACTIONS NEEDED"). Until a fixture exists these tests
are SKIPPED, not passed.
"""
import glob
import json
import math
import os

import pytest

from dyx3_rpp_legacy.output_stage import MODE_PIVOT, MODE_STOP, OutputStage, wrap_pi

FIXDIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "fixtures")
FILES = sorted(glob.glob(os.path.join(FIXDIR, "legacy_decode_*.json")))

CORNER_ALIGN = 3  # PX4_DXP SegmentStateCode.CORNER_ALIGN: the prototype's own pivot state
# DERIVED — NOT FROM V1 SPEC: acceptance threshold for "the heuristic captures the prototype's
# pivots". Proposed, to be adjusted against real data by the human.
PIVOT_CAPTURE_MIN = 0.90

pytestmark = pytest.mark.skipif(
    not FILES, reason="no legacy_decode_*.json fixture yet; extract one from a PX4_DXP bag (HANDOFF)"
)


@pytest.mark.parametrize("path", FILES or ["-"])
def test_decode_against_recorded_prototype(path):
    ticks = json.load(open(path))["ticks"]
    assert len(ticks) > 100, "fixture too small to be evidence"
    st = OutputStage()
    n_align = captured = 0
    for t in ticks:
        sp = st.step(t["v_n"], t["v_e"], t["yaw_rate_body"], t["yaw_ned"])
        assert all(math.isfinite(x) for x in (sp.speed_body_x, sp.yaw_rate_setpoint)) or sp.mode != MODE_PIVOT
        if sp.mode not in (MODE_STOP, MODE_PIVOT):
            assert sp.speed_body_x == pytest.approx(math.hypot(t["v_n"], t["v_e"]))
        if t["seg_state"] == CORNER_ALIGN and math.hypot(t["v_n"], t["v_e"]) >= 0.01:
            n_align += 1
            captured += sp.mode == MODE_PIVOT
    if n_align:
        assert captured / n_align >= PIVOT_CAPTURE_MIN, (
            f"only {captured}/{n_align} CORNER_ALIGN ticks decoded as PIVOT"
        )
