# PX4_DXP A/B arm audit

Audit date: 2026-10-08.  Prototype inspected read-only at
`~/Vetri/3WD_Proto/PX4_DXP`, branch `build/demo-ready` (`fc6436b`).  This is an
evidence report, not approval to remove prototype code.

## Method and limits

An A/B is a feature/profile/disable switch with two materially different
behaviours, identified from `declare_parameter`, the source comments, and the
call sites.  Ordinary numeric tuning constants are not called A/Bs merely
because they can be changed.  Operational controls (`dry_run`, `auto_origin`,
`allow_legacy_lifecycle`, `require_offboard`, `spray_enabled`) are also not
A/B experiments; they are listed under scope below.

For each bundle, the evidence order was:

1. `manifest.json.environment.git_sha`, then `git show <sha>:src/...` for the
   declaration default at the code that ran.
2. The matching launch file (all forwarded arguments default to `__unset__`)
   and `rpp_start.sh` (starts `python3 <script> &` with no ROS parameters).
3. Bag evidence.  `rosbags` decoded `/rpp/debug` and `/rpp/segment_debug`;
   `metadata.yaml` supplied message counts.

No final bag records `/parameter_events` or `/rosout`.  A runtime `ros2 param
set` therefore remains an unavoidable residual risk for parameters not in the
append-only `/rpp/debug` snapshot.  “KEEP” means the arm is the deterministic
default/recorded value at the running SHA and is compatible with observed
behaviour; it does **not** claim to disprove an unrecorded runtime change.

`A` consistently denotes the frozen/disabled or older arm, and `B` the
enabled/newer arm.  `KEEP-A`/`KEEP-B` means freeze that arm in the production
port and remove the switch plus its other arm after approval.  `UNKNOWN` is
not a removal recommendation.

### Boolean discovery scope

The core candidates came from all boolean declarations in the two controller
nodes plus numeric/profile switches which actually disable a feature.  Other
`src/*.py` booleans were reviewed: `path_publisher_node.py:auto_origin`,
`mission_runner_node.py:dry_run`, and
`mission_runner_node.py:allow_legacy_lifecycle` are operation/lifecycle
controls, not frozen-vs-experiment choices.  In spray,
`require_offboard` and `spray_enabled` are safety/operator gates, not A/Bs.
Post-field safety gates are deliberately separated below.

## Final field missions

“Final missions” means every unique bundle with
`outcome.status == COMPLETE` and a timestamp in its directory name on or after
2026-08-14.  Copy-time mtimes were ignored.  There are 44 unique bundles; six
duplicate paths below `untitled folder` were de-duplicated by bundle id.  A
recorder-complete bundle can still have `traversal.status=PARTIAL`; those seven
are retained in the inventory but are not used as positive completion proof.

Mission coverage: 20 curve runs, 3 long-straight runs, 15 short/straight or
27/52-m line runs, and 6 stopped-early line/curve attempts.  There is **no
square** and **no point/dot** mission in this final-session set.  All are
`GPS_SURVEYED` paths.  This matters: point and square-specific arms were not
field-exercised here; a road pre-line product should not discard a
long-straight-specific choice merely because the default was different.

| Code SHA | Date/session | COMPLETE bundle ids (chronological; `*` = traversal partial/truncated/pending) | Mission type / outcome |
|---|---|---|---|
| `57bf8580e68b` | 2026-08-14 morning | `95fcede6`, `0e605ac1*`, `d9ae18b8`, `a3b3b9dc*`, `690b14d0`, `9f7af4ec*` | straight x4; curve x2 |
| `e76d93d646f4` | 2026-08-14 evening | `667b3fd9`, `498e8aff`, `c60df2d2`, `f84a319d`, `fa03b354`, `e0abc732*`, `63c292f2`, `ec48a874*`, `c1960d96` | short/27-m/long-straight/curve |
| `9b4716c7e6e8` | 2026-08-18 morning | `09a28027`, `0e432307`, `ca640cec`, `550a69fd`, `d0fc1830*`, `ae1d0ae1*`, `41896dbd`, `f7327048*`, `6d65e1ff`, `2ac041fa`, `72afaa14`, `4e039433` | short/27-m/curve; three stopped early |
| `70ef65ee386c` | 2026-08-18 evening | `bbfb3cf7`, `fe5aca73`, `a6c89b28`, `ab06863e`, `c02c8b95`, `4798fc3a*`, `f16a582b`, `f82c076c`, `a805a018`, `3554bf61`, `a5d9d892*`, `de87e18b*`, `a390018d*`, `3713940a`, `da8e3a59`, `2aa961ac`, `e814e554` | short/27-m/52-m/long-straight/curve; four stopped early |

The newest bundle is
`bags/18_08_2026/evening_session/curve_run/A_to_B/Bags/stg_e814e554_1787050001_20260818_161645`.
It ran `70ef65ee386c`, is recorder- and traversal-complete, and its report
shows marking RMS 0.55 cm (PASS) and endpoint coast-past 55.4 cm (stop FAIL).

### Cross-bag confirmation

All 44 decoded bundles had `/rpp/segment_debug[0] == 1` (segment profile), not
smooth.  Their segment-state counts were DRIVE=235,783, PRE-CORNER=2,860,
ALIGN=18,455, DONE=6,039, and CORNER_STOP=20,279.  Thus segment code really
ran; smooth-only code did not.  In the newest bundle, the verified RPP snapshot
was `require_rtk_fix=1`, `preview_curvature_n=4`,
`xtrack_lookahead_gain=.05`, `path_resample_spacing_m=.08`,
`corner_smooth_radius_m=.5`, `use_imu_extrapolation=0`,
`use_feedforward_yaw_rate=1`, and `yaw_rate_feedback_gain=0`.
`/rpp/progress`, `/rpp/milestone`, `/spray/point_done`, and `/point/advance`
each have zero messages; `/rpp/debug` and `/rpp/segment_debug` have 10,072.

## RPP A/B decisions

Line numbers refer to the current prototype `src/rpp_controller_node.py`
(`fc6436b`).  History cites the introducing/default-changing commit identified
with `git log -S'<parameter>' -- src/`; field-result text is from the adjacent
source comment or named prototype documentation.

| ID / parameter(s), file:line | Arm A | Arm B | Final active arm and evidence | History | Decision; code/registry effect |
|---|---|---|---|---|---|
| R1 `endpoint_approach_run_remaining`, :386; `_run_remaining_along` / goal approach :4648 | per-segment ramp | run-remaining ramp | A default False at all four SHAs; no launch override. | `1aecf66` (2026-08-01); B overshot a single straight, then failed 2x2 squares (20/35/37% vs three A squares at 100%). | **HUMAN.** Retain the per-mission long-straight option; road pre-line work commonly has long straights. |
| R2 `endpoint_capture_recover_enabled`, :432; :6012-6034 | no bounded overshoot acceptance | bounded crossing recovery | B=True at all SHAs; final bundles contain endpoint arrivals. | `f5b2b1d`, promoted False→True on 2026-08-08 after 5/5 replay catches and 3/3 clean controls. | **KEEP-B.** Freeze recovery; remove flag and false branch. |
| R3 `segment_precise_endpoint_stop_enabled`, :442; endpoint servo :1076,2845 | physical-stop only | bounded final endpoint servo | B=True at all final SHAs; final paths have endpoints, but this bag format has no direct flag snapshot. | `0930a22` (2026-08-13), introduced before the final sessions. | **KEEP-B** with runtime-override caveat; remove flag/legacy endpoint-only branch after approval. |
| R4 `ekf_reset_compensation`, :476; :5059-5070 | skip one cycle, no offset | absorb bounded reset offset | A=False at all SHAs; no recorded reset proves B was never enabled. | `8f17e5b` (2026-07-23), explicitly “leave FALSE until named A/B”. Checklist says induced-reset demo never ran. | **KEEP-A.** Remove opt-in offset code and its params; retain ordinary jump guard. |
| R5 `preview_curvature_n`, :498; speed preview :4915,5329 | `1`, baseline RPP | `4`, predictive preview | B=4 is in every verified debug snapshot, but all final trackers are segment; no smooth execution. | `d33c869` (2026-05-23). | **UNKNOWN.** Do not infer smooth preview use from its configured value. |
| R6 `xtrack_lookahead_gain`, :504; :4555,4916 | `0`, velocity-only lookahead | `.05`, cross-track term | B=.05 in debug; smooth path unexecuted. | `d33c869`. | **UNKNOWN.** |
| R7 `smooth_lateral_gain`, :519 / `smooth_curvature_ld_coeff`, :528; :5265-5275,5465 | gain 0 and old `.35` floor | gain 1.5 and `.20` floor | Configured B, not executed: every final bundle resolved segment. | `3634200` (2026-07-25); curve A/B reported 5.13 cm to 1.00/1.41 cm, but not in these final sessions. | **UNKNOWN.** Keep evidence and code pending a smooth-profile field run. |
| R8 `smooth_max_arc_cut_m`, :549 / `smooth_min_arc_ld_m`, :563 / `curvature_baseline_m`, :569; :5239,5269-5275 | cap 0; prior .25 floor; adjacent-vertex curvature | .005 m cap; .40 m floor; .15 m baseline | Configured B, never executed under final segment selection. | `ff3a9bb` (2026-07-30); cap validated on three curve runs, but square regression explicitly absent. | **UNKNOWN.** |
| R9 `path_resample_spacing_m`, :582; conditioning :1306 | `0`, no resampling | `.08`, resample | B=.08 in every debug snapshot; conditioned paths were produced in every bag. | `d33c869`. | **KEEP-B.** Freeze resampling and remove disable parameter/branch. |
| R10 `corner_smooth_radius_m`, :583; :1307,3964 | `0`, sharp vertices | `.5 m` inscribed arcs | Configured B but final segment profile means the smooth-corner treatment was not proved. | `d33c869`. | **UNKNOWN.** |
| R11 `tracking_profile`, :591; resolution :1309,1713,5137,5202 | fixed segment | `auto` (can select smooth) | Auto was configured, but recorded resolved profile is **segment in all 44** bundles. | `cab281c` (2026-06-11). | **KEEP-A (resolved behaviour).** Port segment only; remove auto/smooth selector after the R5-R10 decision. |
| R12 `segment_lookahead_cross_collinear_deg`, :605; :4565 | 0, clip at each vertex | 5°, cross collinear vertices | B default at all SHAs; segment code executed in every final bag. | `dbe04b9` (2026-07-30), exact old geometry at 0. | **KEEP-B.** Freeze crossing rule. |
| R13 `segment_endpoint_lookahead_extend`, :629; :4566 | endpoint aim collapses | extend final tangent | B=True at all SHAs; segment endpoint transitions observed. | `32b667c` (2026-08-04); False was measured as a 19.5x terminal steering-gain spike, reverted to True. | **KEEP-B.** Freeze extension. |
| R14 `segment_corner_lookahead_extend`, :648; :4567 | pin real-corner aim at vertex | extend incoming tangent | A=False at all SHAs; segment corner states recorded. | `f5b2b1d` (2026-08-08), explicitly “keep False until replay-validated”. | **KEEP-A.** Remove dormant treatment. |
| R15 `segment_simplify_max_offset_m`, :656; simplification :1362 | 0, angle-only simplify | .01 m offset preservation | B default, with conditioned paths observed; exact threshold hits are not individually logged. | `16480d9` (2026-07-18). | **KEEP-B.** Freeze the preservation rule; residual runtime risk noted. |
| R16 `entry_prealign_enabled`, :686; :2400 | emergent entry arc | pre-align run 0 | B=True at all SHAs; ALIGN states are recorded. | `019e22d` (2026-07-13), default False→True 2026-07-29 after OFF 2.01/10.58 cm vs ON .96/1.65/2.15/2.17 cm. | **KEEP-B.** Freeze pre-align. |
| R17 `stop_latch_enabled`, :721; :4247 | frozen continuous speed law | actuator-dead-band latch | A=False at all SHAs. | `8ee84cd` (2026-08-04), field candidate left OFF. | **KEEP-A.** Remove latch treatment; do not confuse this with ordinary endpoint braking. |
| R18 `point_hold_enabled`, :788; :2592-2702 | no point hold overlay | dwell at must-hit points | A=False defaults; point/dot missions absent. | `25850c9` (2026-07-23), field-unverified by its plan. | **HUMAN.** Requires a point/dot field decision. |
| R19 `progress_publish_enabled`, :796; :4758-4786 | no progress/milestones | publish progress | A=False at all SHAs and 0 messages in every final bag. | `40c11d0` / `87d71b7`; G1 bench remains open. | **KEEP-A.** Remove publications and registry entries unless point workflow is approved. |
| R20 `point_precise_stop_enabled`, :799; :2612,2772 | frozen brake-when-near | 2 cm point stop | A=False; no point/dot missions. | `f41fc37`; G3 field A/B deferred. | **HUMAN.** |
| R21 `point_handshake_enabled`, :813; :2531,2650,2702 | fixed `point_hold_s` | spray/operator handshake | A=False; all four handshake topics are zero-message; no point/dot mission. | `055eb6c`; G4/G5 bench/field closure remains open. | **HUMAN.** |
| R22 `segment_brake_velocity_cap_m_s`, :823; :5695 | 0, no active brake | .08 m/s opposing brake | B=.08 at all SHAs; final segment stop states recorded. | `9d0d2e9` (2026-06-18). | **KEEP-B.** Freeze active braking. |
| R23 `pivot_to_intercept_enabled`, :885; :3684 | pivot to leg direction | pivot to intercept point | B=True at all SHAs; 18,455 ALIGN samples and 20,279 CORNER_STOP samples confirm pivot path execution. | `81eca04` (2026-08-03); 2026-08-07 False data showed 1.7–6.6 cm bad entries, then returned True. | **KEEP-B.** Freeze intercept pivot. |
| R24 `connector_absorb_m`, :894; :2151-2199 | 0, preserve short connector | .20 m absorption | B default, but no bag event identifies a qualifying connector. | `a10eccf` (2026-06-18). | **UNKNOWN.** No topology-specific proof. |
| R25 `use_imu_extrapolation`, :912; :4958-4967 | raw pose/frozen behavior | velocity extrapolation | A=False in every debug snapshot and at every SHA. | `d33c869`; True restored briefly then reverted in the 2026-08-07 June-15 comparison. | **KEEP-A.** Remove extrapolation arm. |
| R26 `use_feedforward_yaw_rate`, :933; :4369,5437 | heading PID only | body-rate feed-forward | B=True in debug; `/rpp/debug[10]` is produced by the active branch. | `d33c869`; documented validated baseline. | **KEEP-B.** Freeze feed-forward yaw. |
| R27 `yaw_rate_feedback_gain`, :936; :5440 | 0, no feedback correction | nonzero feedback correction | A=0 in every debug snapshot. | `d33c869`; later plans contain unvalidated `.3/.6` trials. | **KEEP-A.** Remove feedback treatment. |
| R28 `speed_cmd_decel_m_s2`, :996; :5414 | 0, unbounded legacy decel | .30 m/s² slew | B default only on `70ef65e`; newest bag has finite Patch-2 diagnostics and speed modes 0/1/2/3. | `70ef65e` (2026-08-18), introduced on final mission day. | **KEEP-B.** Freeze bounded decel; no separate default history exists. |
| R29 `preview_curvature_distance_m`, :1001; :5330-5334 | 0, `preview_curvature_n` only | positive metre horizon | A=0 in `70ef65e` final bags. | `70ef65e`; comment calls a 12 m setting a field A/B, but no recorded override. | **KEEP-A.** Remove unrun metre-horizon arm. |

## Spray A/B decisions

Line numbers refer to the current prototype `src/spray_controller_node.py`.
Spray's relevant final snapshot values agree with the running defaults
(`use_distance_aware_spray=True`, `flow_modulation_enabled=False`,
`consume_rpp_progress=False`, `spray_require_rtk_fix=True`,
`spray_max_hrms_m=.10`), unlike the RPP manifest block discussed below.

| ID / parameter(s), file:line | Arm A | Arm B | Final active arm and evidence | History | Decision; code/registry effect |
|---|---|---|---|---|---|
| S1 `flow_modulation_enabled`, :833; :2462-2473 | full flow | speed-proportional flow | A=False in final spray snapshots. | `d2b1b9e` (2026-07-23), bench calibration required before B. | **KEEP-A.** Remove modulation code/params. |
| S2 `use_distance_aware_spray`, :852; :1409,1717,2353 | legacy path timing | distance-aware lead FSM | B=True snapshots; newest bundle has desired/commanded/state edges 3/3/3 with zero misfire samples. | `8b664fb` (2026-06-17). | **KEEP-B.** Freeze distance-aware FSM. |
| S3 `spray_off_during_pivot`, :901; :2227 | may paint during pivot | suppress pivot spray | B=True defaults; final bags include ALIGN/CORNER_STOP states. | Replaced the removed speed threshold after 2026-07-17’s 132 spurious valve fires. | **KEEP-B.** Freeze state-based pivot gate. |
| S4 `consume_rpp_progress`, :909; :1822-2086 | local `/path` projection | RPP boundary source + fallback | A=False snapshots and `/rpp/progress=0` messages. | `87d71b7`; G2 field A/B remains open. | **KEEP-A.** Remove consumption branch with R19 unless point/progress is approved. |
| S5 `projection_window_back_m`/`projection_window_fwd_m`, :953-954; :1899-1900 | both 0, global projection | .5 m/2.0 m continuity windows | B defaults in final code; newest run’s spray edges are clean. | `7965be3` (2026-08-04), fixed doubled-back leg swapping. | **KEEP-B.** Freeze windowed projection. |
| S6 `projection_direction_gate_deg`, :972; :1910 | 0, gate off | positive heading gate | A=0 at all final SHAs. | `b385712` (2026-08-05); replay control arm was wrong, so treatment prohibited. | **KEEP-A.** Remove unvalidated direction gate. |
| S7 `xtrack_trip_error_m` with `max_xtrack_error_m`, :930,937; :1872-1874 | trip <= clear, single threshold | .08/.05 hysteresis | B default in final code; final reports show continuous valve edges, but no direct trip event proves exercise. | P0-2, 2026-07-30: .05 single threshold made a 31 cm hole. | **KEEP-B.** Freeze hysteresis; retain the two fixed thresholds as internal constants. |
| S8 `spray_require_rtk_fix`, :980; :2189,2271,2723 | no RTK gate | RTK_FIXED gate | B=True snapshots; GPSRAW recorded (1,064 newest messages). | `fdf83b0` (2026-07-23), Phase B bench-verified. | **KEEP-B.** Freeze the safety gate; treat as safety, not a customer tuning parameter. |
| S9 `spray_max_hrms_m`, :1002; :2160 | 0, no accuracy half | .10 m accuracy gate | B=.10 in final snapshots. | `eb6a040` (2026-07-27), A14 accuracy gate. | **KEEP-B.** Freeze it; later fail-closed semantics are separately human-gated below. |
| S10 `allow_legacy_spray_active_fallback`, :1011; :1410,1719 | no legacy active fallback | use legacy fallback | B=True default, but final bags do not identify fallback selection. | `8b664fb`. | **UNKNOWN.** Retain pending a bag/replay that shows the fallback path. |

## Never field-run post-final safety gates

Commit `42d8d4b` (2026-08-20) is after the last mission.  Its
`rtk_fix_timeout_s`, `rtk_require_accuracy`, `rtk_max_hrms_m`, and
`rtk_recover_hold_s` in RPP (`:487-490`), plus `spray_watchdog_required`,
`spray_watchdog_timeout_s`, and `spray_require_accuracy` in spray
(`:850-851`, `:998`), are **NEVER FIELD-RUN**.  The changed spray
`gps_fix_timeout_s=.5` is also post-final.  These are fail-closed safety gates:
they are **HUMAN**, not DROP, regardless of their default value.

## Recorder defect and Phase 10 requirement

Do not use `manifest.as_run_config.rpp_params.values` for arm evidence.  In
the newest bundle it pairs names with wrong values: e.g.
`require_rtk_fix=0.05`, `preview_curvature_n=.08`, and
`use_imu_extrapolation=1.0`.  The decoded contemporaneous `/rpp/debug` data
instead proves the correct values `1`, `4`, and `0`.  This is a positional
name/value pairing failure in the recorder snapshot, not a plausible runtime
configuration.

The same spot check found the relevant final `spray_params` values coherent
with code (the values listed before the spray table).  FCU capture is also
present with `method=ros2_param` and no missing entries in the newest bundle,
but it was not used to decide controller A/B arms.  Coherence is not a
substitute for verification.

**Phase 10 requirement for `dyx3_recorder`:** capture each parameter as an
atomic `{name, type, value}` record (or independently query each named key),
then validate the stored name/value mapping against the live response before
finalisation.  Store a schema/version and explicit missing/error fields; add a
test that would fail on this exact off-by-position corruption.

## Summary and human decisions

For the 39 documented A/B rows: **KEEP 28, HUMAN 4, UNKNOWN 7**.  There are no
standalone “DROP both arms” rows: every resolved KEEP row drops its opposing
arm when frozen (28 opposing-arm removals); UNKNOWN and HUMAN rows remain.
The migration projection is therefore **RPP 119 → 100** parameters and
**spray 54 → 43** parameters (S7 freezes two thresholds, and S5 freezes two
window values), excluding human/unknown rows and all post-final safety
gates.  These are planning counts, not edits to the registry.

Human decisions required:

1. Should `endpoint_approach_run_remaining=True` exist as a long-straight
   per-mission mode for road pre-line work, or be redesigned as fixed
   long-straight logic?
2. Do point/dot marking missions need point hold, precise stopping, and the
   RPP/spray/operator handshake?  No final mission exercised any of them.
3. Should the smooth tracker and its R5-R10 A/B treatments be preserved for a
   deliberately smooth-profile field qualification, or retired in favour of
   the actually-run segment tracker?
4. Does a real mission topology exercise connector absorption (R24), and does
   any deployment need the legacy spray-active fallback (S10)?
5. Are the `42d8d4b` fail-closed RTK/watchdog gates accepted after dedicated
   safety review and field validation?
