# Contract — `dyx3_geometry` (pure C++17, no ROS)

Spec §7.3. Ancestors: `PX4_DXP` `build/demo-ready` @ `fc6436b`, `src/rpp_controller_node.py` (carried byte-identical in
`ros2_ws/src/dyx3_rpp_legacy`). Frames/signs: `docs/contracts/frames.md` — NED, heading 0 = North clockwise-positive, **cross-track + = RIGHT**.

## API (`include/dyx3_geometry/*.hpp`, namespace `dyx3_geometry`)

| Function | Ancestor | Notes / carried quirks |
|---|---|---|
| `distance(Point, Point)` | `_dist` | `hypot` |
| `angle_wrap(x)` | `_angle_wrap` | `[-π, π)`; reproduces CPython's **floored** float modulo exactly (`py_float_mod`) |
| `heading_delta(h0, h1)` | `_heading_delta` | `|wrap(h1 − h0)|` ∈ [0, π] |
| `segment_heading(a, b)` | `_segment_heading` | `atan2(e, n)`; degenerate → 0 |
| `perpendicular_distance(p, a, b)` | `_perp_dist` | infinite line, unsigned; `|ab| < 1e-9` → distance to `a` |
| `project_onto_segment(pos, a, b)` / `(pos, path, idx)` | `_project_onto_segment` | `signed_cross = cross_z/|ab|` (exact perpendicular even when `t` clamps — field fix 2026-07-30). **Degenerate segment quirk kept:** `dist_to_end_along = |pos − a|`. Path overload clamps idx; n=1 / n=0 defined |
| `project_onto_path(pos, path, hint&)` | `_project_onto_path` | windowed `[hint−2, hint+4)` (full scan if invalid / window < 3 / stale); `signed_cross = copysign(d, cross_z)`; **open defect C9** (no monotonic window) carried and documented |
| `line_intersection(a, b, c, d)` | `_line_intersection` | `nullopt` when `|d1×d2| < 1e-9` |
| `path_length(path)` | `_pts_length` | left-to-right accumulation — see "Interpreter note" |
| `cumulative_lengths(path[, out, cap])` | `_pts_cumulative_lengths` | span form **allocates nothing** |
| `resample(pts, spacing, flags*)` | `_resample_path` | endpoints exact, flag AND rule, `max(2, ceil(total/spacing)+1)` samples. **Allocates** (mission install only) |
| `menger_curvature(a, b, c)` | inner of `_path_curvature_at` | 0 when any side < 1e-6 |
| `curvature_at(path, idx, baseline)` | `_path_curvature_at` | half-baseline on **arc length**; adjacent-vertex form (baseline 0) is noise-dominated on 4–10 cm spacing |
| `max_preview_curvature(path, seg, foot, l_d, n)` | `_max_preview_curvature` + `_walk_path_samples` | streaming walker, **no allocation**; `seg_idx` clamped (ancestor raises `IndexError`); n<2 path → 0 |

Hot-path rule (spec §7.3/§8): everything except `resample` and the `vector` overload of `cumulative_lengths` is allocation-free and `noexcept`-in-practice.
Build flags: `-ffp-contract=off`, no `-ffast-math` (GATE 3 needs the same IEEE operation order as the ancestors).

## Evidence — GATE 3 (spec §7.2) status

`test/gate3_equivalence_test.cpp` replays `test/fixtures/gate3_geometry_vectors.txt`, produced by `tools/gate3/gen_geometry_vectors.py`
from the **verbatim** ancestors (never re-implemented) over the archived missions in Git (`backend/tests/data/missions`: 3 DXF + 1 waypoints file) planned by the carried path
engine and conditioned by the verbatim `_simplify_path_for_profile` / `_smooth_corners` / `_resample_path`, plus seeded random scalar cases and 150-step tracking
emulations with lateral noise and jump/hint-reset events.

**Result (2026-10-07, fixture generated under Python 3.10.12 = the rover's interpreter): 119 147 values compared, 0 failed; max |diff| 1.1e-13 (`distance`,
1 ULP), ≤ 2 ULP everywhere else, integer/flag/hint fields exact, `resample`/`cumulative`/`seg_proj`/`path_length` bit-identical.** The acceptance bar is
`|a−b| ≤ 1e-12·max(1,|a|,|b|)`; the printed per-function table records the real margin.

**GATE 3 is NOT closed by this**: the corpus here is the four missions available in Git. The field missions (`tes_cross_line*`, surveyed CSVs, the arc/circle/square/L bags'
conditioned paths) live on the Mac — **LOCAL ACTION**: copy them into `backend/tests/data/missions/` (or point the generator at them), re-run the generator under Python 3.10,
commit the fixture. Bag-recorded poses (not synthetic noise) are the second leg: `tools/extract_geometry_bag_fixture.py` → `test/fixtures/bag_xtrack_*.txt` →
`geometry_bag_replay_test` (currently **SKIPPED** — exit 77 — and not counted as a pass).

### Interpreter note (a real finding)
CPython ≥ 3.12 changed `sum()` of floats to compensated (Neumaier) summation. `_pts_length` uses `sum(...)`, so its value differs from a left-to-right accumulation by up to
~24 ULP under 3.13 but is **bit-identical under the rover's Python 3.10**. The fixture must therefore be generated under 3.10; regenerating under a newer Python silently
changes the oracle. (`--check` is deliberately not wired into CI for this reason.)

## Tests
* `geometry_unit_test` — 855 definitional checks (frame conventions, sign of cross-track, the 2026-07-30 handover regression, hint/window behaviour, resample flags, circle
  curvature = 1/R, the noise-vs-baseline property reproducing the documented 3× adjacent-vertex error); clean under ASan + UBSan.
* `gate3_equivalence_test` — above. `geometry_bag_replay_test` — evidence replay, SKIPPED until a bag fixture exists.
