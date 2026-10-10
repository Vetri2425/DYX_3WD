# Parameter profile: production

The parameter values a production rover runs with, beyond the nodes' built-in defaults. Spec section 9: profiles are
promoted by **explicit save**; a live tweak never silently rewrites production config. How a profile is installed,
changed and promoted: `docs/tuning/parameter_profiles.md`.

| File | Node | Installed as | Values | Source |
|---|---|---|---|---|
| `rpp.yaml` | `rpp` | `/etc/dyx3/rpp.yaml` | `mission_speed: 0.6`, `max_linear_vel: 0.85` | owner decision 2026-10-10; `docs/tuning/parameter_registry.md` "Production overrides" |

Only nodes with a decided override have a file. Both values equal today's built-in defaults: the file documents the
explicit-save path and pins them against a future change of a default. Every other parameter is the built-in default,
carried from the prototype and **not re-validated in the NED frame** (GATE 4); none is copied here before it is.

This is `config/`: changes reach the rover and are reviewed by a human (CLAUDE.md section 6).
