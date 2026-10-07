# Stage 0 tooling (architecture §11, §4.6)

Code only; **no result here is evidence** — every tool needs real bags, which are not in Git (LOCAL ACTION, Mac).

| Item | Tool | State |
|---|---|---|
| 0.1 Quantify the arc payoff | `tools/analysis/arc_floor.py` | The maths: regress measured heading error on measured turn rate through the origin; the slope should be `1/RO_YAW_P` (0.667 s at 1.5) if the pure-P floor is what limits arcs. Reports slope, ratio to the expected slope, R², floor RMS. **Not built:** extracting `(omega, heading_err)` from the prototype bags (the topics/fields are not reconstructed here) and the conversion of a heading floor into centimetres (needs the controller's lookahead geometry: a modelling choice for a human; `--xtrack-per-rad` is optional and has no default). Use the FCU's **live** `RO_YAW_P`, not the documented one. |
| 0.2 Re-base the toolchain for the DDS timebase | `tools/analysis/timebase.py` | `estimate_clock_map` (PX4 `hrt` µs vs bag receive time → offset, drift, latency floor, jitter by the lower-envelope method), `offset_error_to_position_cm` (40 ms at 0.35 m/s = 1.4 cm), `cross_correlation_lag` (residual bag↔ulog lag from one shared signal, with a peak-correlation sanity value). Tested on synthetic data only. **Not done:** rewiring `tools/analyze_bag_ulog.py`, the replay harness and the other prototype analysis tools (they live in `PX4_DXP`, which is read-only evidence and not in this repository). Also feeds the A1.4 question: record `Px4LinkStatus.timesync_*` per run (the recorder does, interfaces 0.7.0). |
| 0.3 Close the open baseline | — | Not code: the `RO_YAW_RATE_TH` 1.0→0.5 A/B, the `RO_YAW_RATE_P` 0.17 backfire, three unexplained failures, the spray boundary gap (the replay harness for the last is the pending LOCAL ACTION in HANDOFF). |
| 0.4 F2 folded into F1 | — | Process decision, nothing to build. |

Run: `python -m pytest tools/tests/test_analysis_tools.py`. The CI job `tools (rosbags extractors)` already installs numpy and runs `pytest tools/tests`.
