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

M1 = right wheel, M2 = left wheel (`RBCLW_FUNC1 101`, `RBCLW_FUNC2 102`; verified from logs 2026-10-09).

**PX4 link:** `RBCLW_QPPS_MAX` = 90 % of the tuned QPPS = **151200**
(`config/px4/3wd_6x_carry_from_proto.params`). Full stick then stays inside the RoboClaw's tuned speed, so the
velocity loop never saturates and both sides keep tracking. Re-derive it after every autotune.
