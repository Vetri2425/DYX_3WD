# Proposal — update CLAUDE.md §3b status and next steps

**For human review.** `CLAUDE.md` is human-owned and was not edited by this change.
Companion to `2026-10-08_claude-parameter-count-clarification.md` (parameter counts, lines 95 and 129).

## 1. §3b header and table (currently "Milestone 1 complete. Skeleton only")

Replace "**Milestone 1 complete. Skeleton only: no control logic, no firmware patches.**" with a short
statement that the control graph and the Cloud Review fixes (phases A–F) are implemented and tested
off-target on branch `claude/cloud-phases`, CI run `37766642047` green (ROS 2 Humble: 12 packages, 440 tests,
0 failures, 2 skipped), not merged to `master`, and that nothing has run on a rover.
Replace "Green CI does not prove: any behaviour. There is none yet." with: green CI proves off-target
behaviour only; it does not prove hardware, timing, RTK, valve closure, or accuracy.
Point to `docs/agents/CLOUD_REVIEW_STATUS.md`.

## 2. "What exists"

Replace "12 package skeletons … empty namespaces, no targets" and "All non-functional stubs" with the
implemented packages (see README), and keep "backend path engine is Python" as is.

## 3. "Immediate next steps"

Items 1–2 (Milestone 2 interface freeze for `MotionSetpoint`, parameter classification) are done in code
(`dyx3_interfaces` frozen; parameter tables generated). Suggested replacement: C3 valve close path (bench),
GATE 1 PX4 bench, Jetson timing, real-bag replay, E4 production RTK plan, then field accuracy validation.

## 4. §9 Build and test

Already carries the local ROS 2 pointer. Optionally note that the installer suite needs GNU
coreutils/findutils first on `PATH` on macOS.

## 5. Why this is a proposal

`CLAUDE.md` governs agents; a human should decide the wording of its status section.
