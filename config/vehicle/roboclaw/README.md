# RoboClaw configuration (rover 01)

`Roboclaw_01_DYX_3WD.cfg`: BasicMicro Motion Studio export of the production RoboClaw (address 128),
saved 2026-10-09 16:53. Restore it with Motion Studio → *Device* → *Load settings*, then **Write Settings**.

| Setting | Value | Why |
|---|---|---|
| Serial timeout | **0.5 s** | The RoboClaw stops the motors if PX4 stops sending (FCU crash, cut cable). 0 = never stops: unsafe |
| Main battery min / max | **21.4 V / 28.4 V** | 8S LiFePO4: over-discharge cut and overvoltage limit. Check that a fully charged pack stays below 28.4 V, otherwise the RoboClaw trips and stops the drive |
| Velocity QPPS (M1, M2) | **168000, 168000** | Autotuned maximum encoder speed |
| Velocity P / I (M1) | 0.71695 / 0.0804 | Autotuned |
| Velocity P / I (M2) | 0.83852 / 0.10062 | Autotuned |
| Current limit | ±60 A | |

**Wiring and output mapping — verified 2026-10-10 on the bench (wheels up, logs below).**
- **M1 = RIGHT motor, M2 = LEFT motor** (Motion Studio, owner).
- PX4: **`RBCLW_FUNC1 101`** (right command → M1), **`RBCLW_FUNC2 102`** (left command → M2), **`RBCLW_REV 0`**.
- Stick: **`RC1_REV +1`** (stick right ⇒ `manual_control_setpoint.roll` +1), `RC3_REV −1` (stick forward ⇒
  throttle +1).
- Proof (FCU logs `2026-10-10/03_50_14.ulg` and `03_54_46.ulg`, copies in `bench_tools/logs_2026-10-10/`):
  stick right → left wheel forward, right wheel back (right turn); stick forward → both `wheel_speed` positive;
  reverse → both negative. Command sign = encoder sign on both motors.
- **Do not use `RBCLW_REV` to "fix" direction.** A test with `RBCLW_REV 3` made stick forward drive backwards:
  the RoboClaw already follows the commanded sign and its encoders read the physical direction. EKF2 fuses
  `(wheel_speed[0] + wheel_speed[1]) / 2` (`EKF2_WENC_CTRL 1`), so the encoder sign must stay physical.
- `actuator_test set -f <fn> -v <v>` gave a direction that disagreed with the stick path on this setup; prove
  direction with a logged stick drive (or Offboard), not with `actuator_test` alone.
- Rule: fix a left/right swap in the RBCLW_FUNC output mapping, never with `RC1_REV` (that only changes manual
  driving).
- Wheels down (log `04_14_36.ulg`): stick right → gyro yaw +0.70 rad/s (right turn), stick left → −0.64 rad/s,
  forward straight. T1 closed 2026-10-10.

**PX4 link:** `RBCLW_QPPS_MAX` = 90 % of the tuned QPPS = **151200**
(`config/px4/3wd_6x_carry_from_proto.params`). Full stick then stays inside the RoboClaw's tuned speed, so the
velocity loop never saturates and both sides keep tracking. Re-derive it after every autotune.
