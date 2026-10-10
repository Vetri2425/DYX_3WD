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
| `resample` | `_resample_path` (→ `dyx3_geometry::resample_fixed`, see "Must-hit divergence") | 3777 |
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
   (`round(x*1000)`); conditioning reorders/regroups points but never moves a must-hit one (the
   prototype did, in smooth runs and absorbed connectors; fixed, see "Must-hit divergence" below),
   so the key is the identity: **every input must-hit vertex appears in the output at exactly its
   input coordinates (bit-identical) with `must_hit = 1`**. The output flag is the 1 mm re-match of
   the conditioned points against the key set. Indices are not carried through the pipeline (seven
   functions would need a parallel vector); the guarantee is held by tests instead
   (`conditioner_musthit_test.cpp`, including a seeded property test over 180 random paths in all
   three profiles). Known imprecision, harmless: a non-must-hit conditioned point within 0.5 mm of a
   must-hit vertex (e.g. a path that revisits the vertex) is flagged too.
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
   `len < connector_absorb_m` AND it is interior AND **both** bends exceed `connector_min_corner_deg`
   AND **no raw vertex it would erase is must-hit** (b, c, and any collinear vertex the simplifier
   dropped between them; deliberate divergence, below).
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
    and ≥2 points. A must-hit vertex is never replaced by `smooth_corners` and is a fixed knot of
   `resample` (rules 11, 12).
11. **`smooth_corners`**: inscribed circular arc at every interior vertex, tangent length
    `d = R / tan(θ/2)`; skipped (sharp corner kept, warning counted) if `d > 0.45·min(|AP|,|PB|)`;
    nearly collinear (θ<1e-3 or π−θ<1e-3) kept as is. **A must-hit interior vertex is never replaced:
    it is kept sharp, exactly like the `d > 0.45·min` case, and is not counted as a skip.** Arc
    point flag = AND of the three flags.
    `arc_pts` points per arc. Bounds curvature at 1/R.
12. **`resample`**: uniform spacing along cumulative length, `n = max(2, ceil(total/spacing)+1)`;
    endpoints forced **exactly**; interior flag = AND of the two bracketing flags; if
    `total < spacing` returns just the endpoints. **Must-hit vertices are fixed knots**: the
    conditioner calls `dyx3_geometry::resample_fixed` with their indices and each span between
    consecutive knots (endpoints included) is resampled on its own with
    `max(2, ceil(L_span/spacing)+1)` samples, i.e. spacing ≤ `path_resample_spacing_m` everywhere
    and uniform inside a span. The knot is emitted verbatim with its own flag.
13. **Sliver drop**: with >1 run, runs shorter than **5 cm** are dropped (they would command a
    pointless stop + double 180° pivot); the next run starts within goal tolerance.
14. **Atomic install**: a fully conditioned mission is staged and installed as ONE reference
    assignment on the control thread; the loop must never observe a half-installed mission
    (history: an e-stop 1-point path swapped under a tick whose `_closest_seg_hint` sat in a ~20k-point
    path caused an `IndexError` mid-e-stop).

### Must-hit divergence from the prototype (deliberate, 2026-10-10)

The prototype (`PX4_DXP` `build/demo-ready` @ `fc6436b`) keeps a must-hit vertex in segment runs
(rule 5) but loses it in two other places. Both are fixed here on purpose; the C++ no longer
matches the prototype for paths that have a must-hit vertex in those places.

| Where | Prototype | Here |
|---|---|---|
| `smooth_corners` (smooth runs) | replaces every interior vertex by an arc, ignoring must-hit | a must-hit interior vertex is kept sharp |
| `resample` (smooth runs) | fixes only the two path endpoints; interior vertices fall between samples | must-hit vertices are fixed knots; spans between knots are resampled separately |
| `absorb_short_connectors` | replaces b and c by the apex/midpoint without checking must-hit; contradicts the "never moves them" claim of rule 2 | the connector is NOT absorbed when b, c or any raw vertex between them is must-hit |

Consequences a reader should expect: a must-hit vertex in a smooth run is a sharp kink of at most
the original bend (the curvature bound 1/R holds only between must-hit vertices), and a path with a
must-hit on a short connector keeps that connector as its own run, with the stop/align it implies.
Keeping a vertex the operator marked must-hit is the intended trade.

Test consequences (`conditioner_equivalence_test.cpp`): the recorded gate-4 vectors were NOT
regenerated. A recorded end-to-end `_path_cb` case is excluded from the comparison iff the
predicate documented in `diverges_by_must_hit` holds (a must-hit vertex in a connector the prototype
absorbs, or a must-hit interior vertex of a smooth run while smoothing or resampling is on); the
test counts the excluded cases and checks the new contract on them instead. Current corpus: 28 of 182
end-to-end cases (26 genuinely differ from the prototype, 2 coincide by luck: the vertex sat on the
resample grid). All 1,909 other recorded vectors, and the 154 other end-to-end cases, are still
bit-for-bit against the prototype. New behaviour is pinned by `conditioner_musthit_test.cpp`.

Not covered, stated so nobody assumes otherwise: rule 13 (sliver drop) still removes a run shorter
than 5 cm when more than one run exists. A must-hit vertex that is only inside such a sliver, and
not shared with the neighbouring run, is dropped with it.

## 5. Known defects / bug history this module carries

| Item | Where |
|---|---|
| Surveyed vertices silently deleted (64→2 points) — fixed by must-hit provenance + DP-against-span (`64c12ff`; field-proven 64→4) | rule 5 |
| `_merge_chain` dropped other segments' `vertex_indices` (A9) — **backend path engine**, not this module | `path_engine/optimizers/shape_grouping.py` |
| Extension↔mark boundary full stop (2026-07-31/08-01) | rule 9 |
| Smooth-run and absorbed-connector must-hit loss (prototype) | rules 2, 7, 11, 12: fixed 2026-10-10, see "Must-hit divergence" |
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
