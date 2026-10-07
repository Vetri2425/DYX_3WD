# Proposal — correct the parameter count to 173

**Status:** accepted human decision, 2026-10-07. The frozen V1 architecture is not edited.

## Decision

The production parameter registry is **173** entries: **119 RPP + 54 spray**.

The source-of-truth method is the count of executable `declare_parameter(` calls in the
read-only prototype:

```text
PX4_DXP/src/rpp_controller_node.py    119
PX4_DXP/src/spray_controller_node.py   54
                                      ---
                                      173
```

Comment-only occurrences are excluded. The historical count of 120/174 came from a broad
`grep -c declare_parameter` that also matched the comment at
`rpp_controller_node.py:679`. The source has had 119 unique RPP parameter names since
prototype commit `42d8d4b` (2026-08-20); no parameter is missing.

## Proposed V1 corrections

The next approved V1 revision should replace **120 RPP + 54 spray = 174** with
**119 RPP + 54 spray = 173** in §1.1, §7.4, and §9. `docs/agents/Phase_plan.md` is not frozen
and has been corrected directly.
