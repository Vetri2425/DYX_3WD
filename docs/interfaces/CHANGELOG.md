# Interface changelog

## 0.1.0 — 2026-10-07

- Initial frozen `dyx3_interfaces` surface: canonical motion command/status, vehicle and
  subsystem status messages, mission services, and `ExecuteMission` action.
- Motion commands are NED/FRD with signed body-X speed. See
  `docs/contracts/frames.md` for the frozen frame, sign, unit, and clock contract.
