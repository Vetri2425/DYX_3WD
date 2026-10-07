# Frames, signs, units, and clocks

**Status:** frozen for the production companion stack. This is the single source for
frame and time semantics; interfaces and package contracts reference this document rather
than restating a divergent convention.

## Earth and body frames

| Name | Axes / convention | Applies to |
|---|---|---|
| Local NED | +X North, +Y East, +Z Down | paths, position, earth-frame velocity, headings |
| Body FRD | +X forward, +Y right, +Z down | `speed_body_x`, body angular rates, PX4 attitude quaternion |
| Heading/yaw | 0 rad North; positive clockwise viewed from above; wrapped `[-pi, pi]` | `yaw_setpoint`, `heading_rad`, heading errors |
| Yaw rate | positive clockwise viewed from above, rad/s | `yaw_rate_setpoint`, `commanded_yaw_rate_radps` |
| Cross-track | positive when the vehicle is to the **right** of the directed path | RPP diagnostics and spray gates |

There is no ENU↔NED conversion in the production command path. `dyx3_px4_link` is the only
package that may access `/fmu/**`; it maps these canonical NED/FRD values directly to PX4.

## Motion units and safe semantics

- Distances are metres (`m`), speeds are metres per second (`m/s`), accelerations are
  metres per second squared (`m/s²`), angles are radians (`rad`), and angular rates are
  radians per second (`rad/s`) unless a field name explicitly ends in `deg`.
- `speed_body_x` is signed: positive forward; negative reverse. The message carries intent;
  reverse permission is a motion-guard decision.
- `speed_body_x` is zero in `STOP` and `PIVOT`.
- An invalid `MotionSetpoint` is semantically `STOP`, regardless of its other field values.
  Failure handling must command speed zero, yaw-rate zero, mode `STOP`, and a reason code.

## Clocks and counters

- Every `builtin_interfaces/Time stamp` in `dyx3_interfaces` is the publishing node's ROS
  clock. It therefore honours `use_sim_time` during replay.
- PX4 sample time is represented separately as `VehicleState.px4_sample_stamp`, converted by
  `dyx3_px4_link` to the same ROS-time domain before publication.
- `MotionSetpoint.seq` starts at zero after a publisher restart. A decrease denotes a new
  publisher session; the Phase 6 motion guard defines how many fresh commands are required
  before it accepts motion.
- `VehicleState.xy_reset_counter` is the authoritative PX4 local-position reset indicator;
  consumers must use it rather than infer resets from an apparent position jump.
