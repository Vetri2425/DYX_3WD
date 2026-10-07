# Contract — `dyx3_rpp::path_conditioner`

**Status:** extracted 2026-10-07 from `PX4_DXP` `build/demo-ready` @ `fc6436b`
(`src/rpp_controller_node.py`). Spec §7.4 row `path_conditioner`. Written BEFORE the C++.
Evidence level: **source-verified; field-unverified here** (no bags in this environment — see
"Evidence needed").

## 1. Purpose

Turn the raw mission `Path` (NED polyline + per-point flags) into an ordered list of **runs**, each
with a tracking **profile** (`segment` = straight/hard-corner polyline, `smooth` = continuous
curvature), conditioned geometry, per-point spray flags and cumulative arc length. Runs are tracked
one at a time; the run boundary is where the rover stops/aligns (see `stop_pivot_fsm`).

Runs **once per mission install**, never in the control loop (this is why §9 classes all its
parameters `IDLE_ONLY`).

## 2. Source map

| C++ function (proposed) | Python ancestor (`rpp_controller_node.py`) | Lines |
|---|---|---|
| `condition_path` | `_path_cb` | 1273–1425 |
| `pt_key` | `_pt_key` | 1743 |
| `simplify_for_profile` | `_simplify_path_for_profile`, `_dp_mark_keep` | 1752–1879 |
| `simplify_with_indices` | `_simplify_with_indices` | 2070 |
| `classify_profile` | `_classify_auto_profile`, `_normalize_tracking_profile` | 1713, 1882 |
| `split_runs_by_flag` | `_split_runs_by_flag` | 1949 |
| `runs_collinear`, `is_short_transit_run`, `merge_collinear_runs` | same names | 1976–2067 |
| `absorb_short_connectors` | `_absorb_short_connectors` (+ `_line_intersection`) | 2125–2231 |
| `split_run_at_corners` | `_split_run_at_corners` | 2234 |
| `resample` | `_resample_path` (→ `dyx3_geometry::resample`) | 3777 |
| `smooth_corners` | `_smooth_corners` | 3841 |
| `build_poses` | `_build_poses` | 2285 |

## 3. Interface

**Input:** `points[i] = (north_m, east_m, z)`; `z` is a **bitfield**: bit0 = spray ON, bit1 =
*must-hit* (source CAD/survey vertex, not densification fill). `frame_id` must equal the
`path_frame_id` parameter (`"local_ned"`) or the whole path is **rejected** (not conditioned).
An empty path is ignored (warn).

**Output:** `runs[]`, each `{poses, flags, profile ∈ {segment, smooth}, length_m, cum_s[], closed}`
plus `must_hit_keys`. Poses re-emit the same `z` bitfield.

## 4. Behaviour (normative, numbered)

1. **Bit-test, never `> 0.5`.** A spray-OFF must-hit point encodes as `2.0`.
2. **Provenance by coordinate.** `must_hit_keys` = set of points quantised to a **1 mm grid**
   (`round(x*1000)`); conditioning reorders/regroups points but never moves them, so the key is the
   identity. Do not replace with indices.
3. **`tracking_profile`** is normalised: `"sharp"`→`segment`; `auto|segment|smooth` kept; anything
   else → `auto`. A forced profile keeps the whole path as ONE run.
4. **`auto` pipeline per path**, in this order:
   1. `split_runs_by_flag` — split at every spray-flag change; each run after the first is
      prepended with the previous run's last point (takes the run's own flag) so runs share the
      boundary vertex and geometry is gap-free.
   2. per run: `absorb_short_connectors` (disabled when `connector_absorb_m ≤ 0`), then
      `split_run_at_corners`.
   3. `merge_collinear_runs` over the result (fuses PRE/MARK/AFT flag runs that continue in the
      same direction, **preserving per-point flags**).
5. **`simplify_for_profile`** drops duplicate (<1 µm) and same-heading vertices, retaining a point
   iff ANY of: endpoint; spray-flag boundary; **must-hit**; heading change from the *last retained*
   point > `collinear_tol_deg` (5°, measured from the last RETAINED point so gradual curvature
   accumulates); or (when `segment_simplify_max_offset_m > 0`) Douglas–Peucker offset against the
   **collapsing span**. DP is iterative (explicit stack; recursion limit 1000 < road alignments).
   Rule "must-hit is never dropped" is the authoritative fix for deleted survey vertices.
6. **`classify_profile`**: simplify at 5°; ≤2 points → `segment`; any heading delta ≥
   `segment_corner_threshold_deg` → `segment`; else `smooth` only if there are **≥3 turning
   vertices (>2° each) summing >20°** (sustained turning = a discretised arc); otherwise `segment`.
7. **`absorb_short_connectors`**: a segment between two simplified corners is absorbed iff
   `len < connector_absorb_m` AND it is interior AND **both** bends exceed `connector_min_corner_deg`.
   Replacement vertex = intersection of the adjacent leg lines, unless parallel or farther than
   `max(0.5, 5·len)` from the connector midpoint → midpoint. The merge vertex inherits the
   connector's flag. The dual-corner gate protects a deliberate short MARK stroke.
8. **`split_run_at_corners`**: split at simplified-vertex corners ≥ threshold, locating them in the
   raw run by **exact tuple equality**, in order (so simplification must preserve the original
   tuples — do not recompute coordinates).
9. **`merge_collinear_runs`**: merge two runs iff (same profile OR either is a *short pure-transit
   run* ≤ `transit_merge_max_len_m` with all flags OFF) AND they are collinear (gap ≤ 5 cm and
   heading delta < threshold at the joint). Classification runs AFTER the merge. (2026-08-01 fix:
   without the short-transit exception every extension↔mark boundary produced a full stop+align
   and a 54 s terminal crawl.)
10. **Conditioning by profile.** `segment`: `simplify_for_profile` with
    `segment_simplify_max_offset_m` and `must_hit_keys`. `smooth`: `smooth_corners` if
    `corner_smooth_radius_m > 0` and ≥3 points, then `resample` if `path_resample_spacing_m > 0`
    and ≥2 points.
11. **`smooth_corners`**: inscribed circular arc at every interior vertex, tangent length
    `d = R / tan(θ/2)`; skipped (sharp corner kept, warning counted) if `d > 0.45·min(|AP|,|PB|)`;
    nearly collinear (θ<1e-3 or π−θ<1e-3) kept as is. Arc point flag = AND of the three flags.
    `arc_pts` points per arc. Bounds curvature at 1/R.
12. **`resample`**: uniform spacing along cumulative length, `n = max(2, ceil(total/spacing)+1)`;
    endpoints forced **exactly**; interior flag = AND of the two bracketing flags; if
    `total < spacing` returns just the endpoints.
13. **Sliver drop**: with >1 run, runs shorter than **5 cm** are dropped (they would command a
    pointless stop + double 180° pivot); the next run starts within goal tolerance.
14. **Atomic install**: a fully conditioned mission is staged and installed as ONE reference
    assignment on the control thread; the loop must never observe a half-installed mission
    (history: an e-stop 1-point path swapped under a tick whose `_closest_seg_hint` sat in a ~20k-point
    path caused an `IndexError` mid-e-stop).

## 5. Known defects / bug history this module carries

| Item | Where |
|---|---|
| Surveyed vertices silently deleted (64→2 points) — fixed by must-hit provenance + DP-against-span (`64c12ff`; field-proven 64→4) | rule 5 |
| `_merge_chain` dropped other segments' `vertex_indices` (A9) — **backend path engine**, not this module | `path_engine/optimizers/shape_grouping.py` |
| Extension↔mark boundary full stop (2026-07-31/08-01) | rule 9 |
| Triangle-apex-2: short connector survives as its own pivot target, leg entered un-pivoted | rule 7 |
| Circle run flipped to `segment` by one hard corner in a chained entity | rule 8 |
| `C5` segment↔smooth ~5 cm seam | unverified; needs a mission whose profile flips mid-drive |

## 6. Evidence needed (fixtures must come from recorded data)

* Prototype tests that pin parts of this: `test_sprint2_geometry.py`, `test_spray_flag_conditioning.py`,
  `test_curvature_lookahead_cap.py`, `test_entry_prealign.py` (carried byte-for-byte under
  `ros2_ws/src/dyx3_rpp_legacy/test/dxp_verbatim/`).
* **LOCAL ACTION:** archived missions → `/path` messages (the staged missions under
  `PX4_DXP/bags` + the DXF set in Git) → run the verbatim Python `_path_cb` conditioning and dump
  `conditioned_path` (`tools/` extractor, see HANDOFF) as the C++ numeric-equivalence corpus (GATE 3).

## 7. Port notes

* Pure functions on `std::vector<Point>` + `std::vector<uint8_t>` flags; no ROS; no allocation outside
  mission install (this module is NOT on the RT path — allocation is allowed here, but the output
  must be pre-sized buffers handed to the control loop).
* Shares `heading`, `perp_dist`, `line_intersection`, `resample`, `cumulative_lengths` with
  `dyx3_geometry`; GATE 3 requires numeric equivalence for those.
* All 11 parameters of this module are `IDLE_ONLY` (parameter_registry.md).
