# Parameter profile: precision

Named by spec section 9 (`production`, `precision`, `development`). **No precision value is decided yet**: no field
evidence or owner decision gives this profile values that differ from production, so its files are copies of
`config/profiles/production/` (each says so in its header). Do not invent a divergent value; change one here only with
its evidence, in the same commit. How a profile is installed, changed and promoted: `docs/tuning/parameter_profiles.md`.

| File | Node | Installed as | Values |
|---|---|---|---|
| `rpp.yaml` | `rpp` | `/etc/dyx3/rpp.yaml` | same as production |

This is `config/`: changes reach the rover and are reviewed by a human (CLAUDE.md section 6).
