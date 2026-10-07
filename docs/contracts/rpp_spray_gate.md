# Contract — `dyx3_rpp::spray_gate`

Source: `rpp_controller_node.py` — `_segment_spray_active` 5900, `_publish_spray_active` 5909, `_gate_spray` 6079, `_apply_run` entry hold 2380.
Spray position is part of the precision product: this gate decides, from the rover's *heading*, whether the planner's MARK flag may reach the valve.

## 1. Planner flag → spray request
`_segment_spray_active(seg)`: `False` if the path is done or < 2 points or the flag vector length ≠ path length; otherwise
`flags[seg] ∧ flags[seg+1]` (a segment is MARK only when **both** endpoints are). (Smooth profile uses the same per-segment lookup.)

## 2. Heading gates (`_gate_spray(spray_active, heading_err)`) — the single choke point
Every `spray_active` value passes through `_publish_debug`, so gating here covers both profiles and every branch.
* `not spray_active ⇒ False`.
* **Cut (P7)**: `|heading_err| ≥ spray_heading_cut_deg` (30°; `≤ 0` disables) ⇒ closed — pivoting/spinning is not tracking (bag 183235 painted through its terminal pivot at 80°).
* **Entry (P3)**: `_entry_spray_hold` is set at every run start. While held, the valve stays closed until `|heading_err| ≤ spray_entry_max_heading_deg` (5°), **or**
  `path_travel ≥ spray_entry_release_travel_m` (0.6 m backstop — never withhold paint indefinitely), **or** `spray_entry_max_heading_deg ≤ 0`. Evidence (2026-07-31): the valve opened at
  4–13° residual in 8/10 runs, painting up to 8.03 cm off-line. Once released the hold clears for the run.
* NaN heading (the zero-publish paths) passes both comparisons safely — those branches already pass `spray_active = False`.
* Corner/pivot/stop/hold branches publish `spray_active = False` explicitly; run alignment forces spray off throughout (`_run_alignment_hold`).

## 3. Boundary
The RPP output `/spray/active` is a **request**; the actuator, the safety lease and the independent watchdog live in `dyx3_spray` (spec §7.8). The spray node additionally gates on
RTK, pivot state (`6523a84`, replay-verified 157→53, field-unverified `C6`) and cross-track (`max_xtrack_error_m`).
In production the three thresholds above are `IDLE_ONLY` (they move where the valve may open).

## 4. Evidence / tests
`test_spray_flag_conditioning.py` (carried). `test_spray_pivot_gate.py`, `test_spray_rpp_boundary.py` need the spray modules — Phase 9 evidence.
