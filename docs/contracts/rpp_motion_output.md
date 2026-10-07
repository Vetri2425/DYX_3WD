# Contract — `dyx3_rpp::motion_output`  (REWRITTEN, not ported)

Source (the *legacy* stage being replaced): `_publish_velocity` 5832, `_publish_yaw_rate` 5842, `_publish_zero` 5847,
`_clamp_velocity_to_forward_cone` 5631, `_corner_pivot_velocity` 5614, `_corner_brake_velocity` 5684; and `twist_to_setpoint_node.py` (the ENU/mask bridge it fed).

## 1. What the legacy output is
Per tick: a NED velocity **vector** `(v_n, v_e)` (published on `/rpp/velocity_ned`) immediately followed by a body yaw rate `ω` (`/rpp/yaw_rate_body`, **NED clockwise-positive**).
`twist_to_setpoint_node` then streamed MAVROS `PositionTarget` at 50 Hz (velocity ENU, explicit yaw = `atan2(v_n, v_e)` ENU, optional yaw-rate feed-forward — type_mask 455 vs 2503) and
PX4's velocity OFFBOARD branch derived `speed = |v|`, `yaw = atan2(vE, vN)` and **discarded `ω`** — the structural tracking floor `err ≈ ω/RO_YAW_P` (spec §3.1).
Consequences that shaped the controller: pivots were commanded as a small vector at the exit heading (clamped ≤ 75° off the nose) because zero velocity + yaw rate froze heading below 1 cm/s
(square bag 20260611_170539); brakes were ±body-axis vectors; reverse was selected by the firmware when the vector's forward component was negative.

## 2. Production mapping (spec §5.2) — what `dyx3_rpp` must publish

| Legacy situation | `MotionSetpoint.mode` | speed_body_x | yaw_setpoint | yaw_rate_setpoint |
|---|---|---|---|---|
| any `_publish_zero` (IDLE/STALE/RTK_WAIT/JUMP_SKIP/DONE) | `STOP` | 0 | NaN | 0 |
| straight / segment tracking | `TRACK_HEADING` (bearing to lookahead) **or** `TRACK_RATE` (`segment_yaw_rate_gain·θe`) | `+|v|` | ψ / NaN | NaN / ω |
| smooth/arc tracking | `TRACK_RATE` (`κ·v`, + optional FB) — **recovers the floor** | `+|v|` | NaN | ω |
| corner / run-alignment pivot | `PIVOT` | 0 | NaN | explicit rate toward the exit heading |
| terminal approach (`segment_endpoint_approach_speed` 0.03 m/s) | `CREEP` | `+v` (small) | NaN | ω |
| brake / precise-stop reverse | `TRACK_HEADING`/`CREEP` with **negative** speed (reverse), nose heading held | `−|v|` | nose | NaN |

Frames/signs: NED, yaw 0 = North, clockwise-positive, rad and rad/s (`docs/contracts/frames.md`). **No ENU conversion anywhere.** `valid = false` ⇒ STOP.
The legacy yaw rate is already closed on the pose (pure-pursuit κ·v or P on θe), so `TRACK_RATE` is meaningful without extra feedback.

## 3. Reverse
The prototype emits anti-parallel vectors **on purpose** (I1 brake; `precise_stop` with a negative residual; endpoint precise-stop along the segment with `sign < 0`). Under the explicit
contract this is a signed `speed_body_x` with the *nose* heading target (opposite bearing) — never a 180° spot-turn (BUG-T3). **Reverse permission is the motion guard's decision** (`MotionSetpoint.msg`).
GATE 1 closes firmware rows 15/16 only after the bench shows straight reverse and no North-snap at STOP.

## 4. Pivot-rate law (unspecified in the prototype — DERIVED)
The old firmware owned the spot-turn rate. The explicit PIVOT needs one. Until measured, `dyx3_rpp_legacy` uses `ω = clamp(1.5·err, ±max_yaw_rate_body)` with `1.5 = RO_YAW_P` (spec §3.1)
and `0.45 = max_yaw_rate_body` default. Enter/exit hysteresis (40° / 2°) are the **as-flown** `RD_TRANS_DRV_TRN = 0.70 rad` / `RD_TRANS_TRN_DRV = 0.0349 rad`
(`params/31_07_2026_6_08pm.params`, DXP `CLAUDE.md`); the prototype source itself disagrees with itself (docstring ≈30°/5°, inline comment 10°/5°). **Open question for the human**:
what pivot rate and hysteresis should the C++ and the bench adopt (GATE 1 / GATE 4)?

## 5. Safety rules owned here
Fail to zero on every error path; never publish a non-finite value; sequence monotonic per publisher session; `stamp` from the node clock (replay-safe). Everything past this point
(clamps, freshness, gates) is `dyx3_motion_guard`, which never invents a correction.
