# Customer readiness — next session

Reviewed **2026-10-11**, DYX_3WD `master` **0157792**. Focused source review of recovery,
RPP, persistence, parameter saving, deployment and CI; not a whole-stack certification.

**I broadly agree with Astra: about 7.5/10 is reasonable.** The architecture is strong enough;
the remaining work is closing specific recovery gaps and proving the shipped system on the
rover. “9.5–9.8” should mean repeatable customer acceptance, not a more precise desk score.
Supervised dry testing is conditional on the bench checks below; customer delivery remains open.

Two updates to Astra's snapshot:
- [CI 38080153223](https://github.com/Vetri2425/DYX_3WD/actions/runs/38080153223) is now **green, all 11 jobs**, including publication. Artifacts took 3m25s alongside colcon's 5m29s; first job start → publication finish was **5m41s**, versus the handoff's earlier 8m08s. One run demonstrates overlap, not a stable performance estimate.
- The current split puts **px4_link, guard and RPP in control; mission, spray and gateway in services**. The earlier handoff table is inaccurate. A services restart therefore loses the mission owner while control continues STOP/heartbeat. STOP is not disarm.

## Fix order — keep it small

### 1. Resolve recovery before widening field tests

These are source-backed concerns; reproduce each with one focused regression before fixing.
Keep the existing ownership model and request an explicit operator action before motion.

| Priority | Current evidence / failure scenario | Smallest useful closure |
|---|---|---|
| High — restart ownership | `services_graph.launch.py:10–18`; `mission_node.cpp:752–772`: services return IDLE after a crash while PX4 can remain armed/OFFBOARD, without the previous mission's release state. | Decide who reconciles an already-armed vehicle after restart; implement an explicit recovery/stop policy. Test services-only restart, control restart and reboot separately: STOP, spray OFF, truthful operator status, no automatic motion. Do not blindly disarm a vehicle under another legitimate owner. |
| High — resumed entry | `rpp_core.cpp:112–164`: `start_at_run()` only enlarges an alignment budget if alignment was already requested by the old boundary angle. A moved rover resuming a <45° boundary can skip entry alignment. | Evaluate resumed entry against the actual pose using the existing entry policy. Test a sub-threshold boundary with a large initial heading error; verify alignment before tracking. |
| High — resume identity | `mission_progress.hpp:26–40`; `rpp_node.cpp:312–319`: progress stores source SHA + run index, but reload reconditions with current parameters; an in-range index can now identify a different run. | Bind progress to conditioning configuration/version and stable run identity; reject incompatible resume visibly. Test changed conditioning after restart. Source SHA alone is insufficient. |

### 2. Close four smaller concerns with focused checks

- **Medium, stall thresholds:** `motion_guard_node.cpp:229–238` copies RPP defaults that are LIVE elsewhere. Treat guard thresholds as independently justified safety limits, validate supported tuning combinations and bench-test real stalls/slow pivots. Do not add automatic cross-node parameter mirroring.
- **Medium, equivalence:** `orchestrator_equivalence_test.cpp:78,360` pins only the aggregate **373** deviations. Pin expected differences per scenario/window (and fields where needed), so one regression cannot cancel another. Keep published-command tests.
- **Medium, persistence:** `mission_node.cpp:967–981,777–781` advances memory before durable writes; failed batches are logged and discarded. Define the visible “resume record not saved” behavior, retain/retry the newest failed record with bounded retries, and test disk-full/restart. Atomic writes already exist; no database needed.
- **Low, parameter save:** `deployment/scripts/dyx3-param:204–228` saves every parameter; directory-sync failure is ignored. Verify save → restart → effective values on the Jetson, surface durability failure and correct stale loader descriptions. Existing mock tests do not prove restart behavior. This is a verification/hardening item, not a demonstrated bad rover configuration.

### 3. Prove one frozen release on the rover

Use existing [fault-injection procedures](../bench/fault_injection.md) and
`tools/analysis/mission_bag_metrics.py`; record stack/firmware SHA, parameters and bags.

- **Bench first:** E-stop/RC kill, services/control/XRCE loss, Ethernet pull, Jetson loss, OFFBOARD loss + deliberate resume, RTK/heading faults. Measure actual wheel stopping time/distance and valve closure with water; a STOP message or ACK is not physical proof. Confirm FCU parameter capture does not disrupt control timing.
- **Field ladder:** repeat square with the new endpoint law → circle/arc → multi-shape and resumed entries → supported speed steps → water → paint. One change at a time. Report full-mission RMS/p95/max, coverage, endpoint along/cross error, reversals, timeout finishes and paint boundary error against the existing gates.
- **Accuracy honesty:** the first-order model reaches 17.5 mm along-error; lateral completion currently allows 20 mm. Neither establishes a universal 10 mm product claim. Measure first; change tolerances only by an explicit decision backed by data.
- Repeat across cold starts, battery/load conditions and another installed rover before claiming fleet readiness. Agree a repeatability sample size before testing; a single successful run cannot justify 9.8.

### 4. Close the delivery gate

Protect master with required checks/review; sign releases and verify the signature in the
installer, including offline installs. GitHub reports master **unprotected**; current SHA-256
checks establish integrity, not independent publisher authenticity. Prove fresh install,
failed upgrade and rollback on a Jetson. Resolve the customer-network/credential and physical
stop decisions already in the [open-item register](production/open_items.md).

Update stale status pointers and the conflicting `Agent:` attribution instructions; remove
legacy only after Gate 7. Keep that register authoritative rather than creating another backlog.
Do not spend this phase rewriting the architecture or accelerating CI for a score.

**Next session starts with items 1–3 in the recovery table, one at a time.** A credible 9.5+
requires those closures, repeatable field/paint acceptance and a reproducible delivery process.
This session changed documentation only: inspected source and GitHub status; no local runtime
tests, deployment, hardware checks or tuning were performed.
