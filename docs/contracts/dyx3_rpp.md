# dyx3_rpp — package contract and build status

**Status:** PARTIAL by design (2026-10-07, cloud session). Read with the per-module contracts
`rpp_overview.md`, `rpp_guidance.md`, `rpp_speed_profile.md`, `rpp_stop_pivot_fsm.md`, `rpp_terminal.md`,
`rpp_motion_output.md`, `rpp_path_conditioner.md`, `rpp_spray_gate.md`. **Spec:** V1 section 7.4.
**Authority:** `dyx3_rpp` owns the path-following decision and produces exactly one `MotionSetpoint` stream.
It never publishes a safety verdict; `dyx3_motion_guard` is the last authority before PX4.

## 1. What is built and how it is proven

| Module | Content | Proof |
|---|---|---|
| `rpp_params` | all 119 parameters: descriptor table + index enum **generated** from `docs/tuning/parameter_registry.md` (`tools/gen_rpp_params.py --check`), structural validation, LIVE / IDLE_ONLY / RESTART enforcement, atomic batches, change journal | 6 unit tests incl. registry-vs-table comparison |
| `guidance` | lookahead distance and low-pass, smooth arc-cut cap, smooth and segment lookahead point (collinear walk, path-end extension, corner extension), steering geometry, pivot intercept | equivalence vs the verbatim Python (`gate4`) |
| `speed_profile` | alignment accel scale, hard-kappa latch, smooth slew, derived approach distance, lateral-acceleration law, closed/open approach scaling, P4 floor | equivalence vs the verbatim Python (statics) + definitional tests (inline smooth blocks, which the prototype does not expose as functions) |
| `stop_pivot_fsm` | **explicit** state machine: `StopConfirm` (I3), brake (I1), `PivotWatchdog`, `CornerFsm` (TRACKING -> BRAKE -> PIVOT -> RELEASE_SETTLE -> ADVANCE, collinear shortcut, carried stop), `StopHold` (completion hold D3); every transition logged with a reason | stop confirmation (10 093 steps) and pivot watchdog (2 405 steps) replayed against the verbatim Python; FSM walk tests |
| `terminal` | closed-run test, min travel, path progress, tail transit, remaining-along, effective goal tolerance, endpoint-capture recovery (wide miss refused) | equivalence vs the verbatim Python |
| `motion_output` | **rewritten**, not ported: STOP / TRACK_HEADING / TRACK_RATE / PIVOT / CREEP builders, signed reverse with the nose held, fail-to-zero sanitiser | contract tests |

Equivalence vectors (`test/fixtures/gate4_rpp_vectors.txt`, 3.9 MB) are produced by running the carried,
sha256-pinned prototype controller (`tools/gate4/gen_rpp_vectors.py`, needs rclpy: run in the Humble
container) over the archived DXF/waypoint missions in Git and seeded random cases. About 24 000
comparisons, worst difference 0 (bit-exact on the x86 container; the test tolerance is 1e-9 for other
hosts' libm). A deliberately introduced 1 % error in the slew law was caught (mutation check, not committed).
This is **module-level** proof on synthetic and archived-mission inputs; the field bags remain a LOCAL ACTION.

## 2. What is NOT built (do not read the package as a replacement for `dyx3_rpp_legacy` yet)

* **Orchestrator** (`RppCore::tick`): the 50 Hz control loop (`_control_loop_impl`, `_control_segment_profile`),
  its ordering (pose/RTK/jump gates, goal test, holds), per-run reset (`_apply_run`), `run_sequencer`.
* **Path conditioner**: `_simplify_path_for_profile`, `_split_runs_by_flag`, `_merge_collinear_runs`,
  `_absorb_short_connectors`, `_split_run_at_corners`, `_resample_path`, `_smooth_corners` (the geometry
  library has resample and curvature; the conditioner itself is not ported).
* **`rpp_node`** (ROS wiring, artifact loading by id, `RppStatus` with loop-jitter measurement) and the
  **spray gate** interface to `dyx3_spray`.
* Features not ported: point hold, point handshake, precise point stop, endpoint precise-stop tick, entry
  pre-align, stop latch, EKF-reset compensation (replaceable by `VehicleState.xy_reset_counter`), progress
  publication.

Until the orchestrator exists the precision path on a rover is `dyx3_rpp_legacy` (quarantined, GATE 7 deletes it).
The C++ modules above are the building blocks the shadow-run oracle will validate tick by tick.

## 3. DERIVED — NOT FROM V1 SPEC

* Time in the stop logic is `int64` nanoseconds on the node clock, mirroring the prototype's
  `(now - t0).nanoseconds * 1e-9` exactly (a seconds-as-double rewrite differed at dwell boundaries in testing).
* Pivot rate law `omega = clamp(1.5 * err, +/- max_yaw_rate_body)` (1.5 = `RO_YAW_P`). The old firmware owned it;
  open question for the human (see `rpp_motion_output.md` section 4). Re-validate at GATE 1 / GATE 4.
* Parameter validation bounds are structural (finite, non-negative, positive for divisors, a fraction in [0,1],
  closed sets for the three enumerations) plus five ordering relations; none is a tuning value.
  The prototype silently normalised an unknown `tracking_profile` to `auto`; here it is refused.
* A refused parameter change leaves **no** journal entry; the node must log the refusal (rosout) and report it.
