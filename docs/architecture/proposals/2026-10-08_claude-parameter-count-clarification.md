# Proposal — clarify CLAUDE.md parameter counts

**For human review.** `CLAUDE.md` is human-owned and was not edited in Phase F.

At `CLAUDE.md` line 95, replace the historical broad-grep count "174 parameters
(120 RPP + 54 spray)" with "173 legacy parameters (119 RPP + 54 spray)".
At line 129, replace "all 120 of them" with "all 119 legacy RPP parameters".
The frozen V1 architecture retains its historical wording until an approved revision.

The original prototype has 173 executable declarations: 119 RPP and 54 spray.
The generated **current production** parameter tables have 117 RPP and 49 spray entries
(`python3 tools/gen_param_tables.py --check`, 2026-10-08). They contain 164 carried legacy
parameters plus two production-only additions: RPP `segment_command_mode` and spray
`rpp_timeout_s`. Three legacy heading-verdict parameters moved from RPP to spray; nine
legacy spray parameters are excluded from the current production tables with recorded
reasons in the generator. MotionGuard's 15 declared parameters are separate production
parameters and are not included in either RPP/spray table count.
