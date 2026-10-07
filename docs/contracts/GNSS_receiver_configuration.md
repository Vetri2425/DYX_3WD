# GNSS receiver configuration — ownership contract

**Status:** hard production requirement, human decision 2026-10-07.
**Applies to:** UM982 dual-antenna receiver ↔ PX4 (`Vetri2425/PX4-Autopilot-3WD-Prod`) ↔ companion.

## The rule

**PX4 must not auto-configure the GNSS receiver.**

```
UM982 production configuration stored in receiver persistent memory (SAVECONFIG)
→ receiver boots into that configuration by itself
→ PX4 only consumes GNSS / heading data
→ PX4 sends NO receiver configuration commands, ever
→ RTCM corrections are still injected into the receiver (data, not configuration)
```

The receiver owns its configuration. Configuration changes are a deliberate, recorded
maintenance action on the receiver — never a side effect of PX4 boot, reconnect or heading loss.

## What PX4 v1.17 does today (verified in source, `src/drivers/gps/devices` @ `0b96958`)

| Path | Sends to receiver | Allowed? |
|---|---|---|
| `GPS::injectData()` (`gps.cpp:625`) | RTCM correction bytes | ✅ data, not configuration |
| `GPSDriverNMEA::configure()` (`nmea.cpp:1175`) | nothing — probes PX4's own UART baud only (9600–230400 if the set baud fails) | ✅ |
| `GPSDriverNMEA::request_unicore_messages()` (`nmea.cpp:1053`, single call site `:996`) | 6 log commands, unsaved: `GPGGA COM1 0.2`, `UNIAGRICA COM1 0.2`, `UNIHEADINGA COM1 0.2`, `GPGST COM1 1.0`, `GPGSA COM1 1.0`, `GPRMC COM1 1.0` — sent on every `UNIAGRICA` while heading has been missing > 1 s | ❌ **violates this contract** |

**Current production firmware does not meet the requirement.** Required firmware change (not yet
made; lives in the GPS-driver submodule `PX4-GPSDrivers`, so it needs a submodule fork):
remove or permanently disable the `request_unicore_messages()` call. No parameter that re-enables
it in production.

## Receiver-side production configuration (stored on the UM982)

The messages PX4's NMEA/Unicore parser consumes must be enabled **on the receiver** and saved:

| Message | Rate | PX4 uses it for |
|---|---|---|
| `GPGGA` | 5 Hz (0.2 s) | position, fix type, satellites |
| `UNIAGRICA` | 5 Hz | velocity (and Unicore detection) |
| `UNIHEADINGA` | 5 Hz | dual-antenna heading + σ |
| `GPGST` | 1 Hz | eph / epv |
| `GPGSA` | 1 Hz | vdop |
| `GPRMC` | 1 Hz | time / velocity fallback |

Plus: the PX4-facing port's baud rate fixed and matching PX4 (`GPS_1_BAUD` / `SER_*_BAUD`), RTCM
input accepted on that same port, then `SAVECONFIG`.

> DERIVED — NOT FROM V1 SPEC: the message set and rates above are exactly what PX4 v1.17 requests
> today, i.e. what its parser was written against. The PX4 driver hardcodes `COM1` in those
> commands; under this contract the port is whichever UM982 port is wired to the FCU — record it
> with the saved configuration.

PX4 side: `GPS_1_PROTOCOL = 6` (NMEA), fixed baud. `GPS_YAW_OFFSET` is the active heading-offset
knob with this driver; `EKF2_GPS_YAW_OFF` stays dead code while the driver publishes a finite
`heading_offset` (verified v1.17 `gps.cpp:315/949`, `EKF2.cpp:2537-2544`).

## Consequences for open GNSS fixes

Every fix is evaluated under this rule. In particular **no reconnect, timeout or heading-loss
problem may be solved by letting PX4 (re)configure the receiver.**

| Item | Defect | Direction under this contract |
|---|---|---|
| C3 | config-command spam during heading loss | **remove** `request_unicore_messages()` (above) — not throttle it |
| C2 | NMEA driver returns −1 after 500 ms without pos+vel → driver restart → baud re-probe; RTCM injection pauses | keep the link and keep injecting RTCM; recover by resynchronising parsing, never by reconfiguring the receiver |
| F7 | `s_variance_m_s` filled with a variance, read as σ | parsing fix only; unaffected by this rule |

## Acceptance

- Bench: with the receiver's saved configuration erased of the required logs, PX4 must **not**
  restore them (proves PX4 sends nothing); with the saved configuration present, PX4 runs normally
  across power cycles and heading loss.
- Serial capture on the PX4→receiver line contains only RTCM frames after boot.
