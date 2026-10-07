> **Carried from the v1.16.2 fork** (`Vetri2425/PX4-Autopilot` @ `d90864a374`, F-tasks B1 #12)
> as the design contract for wheel-encoder fusion. The body below is verbatim and describes the
> **v1.16.2 / CubeOrangePlus** implementation. The production v1.17 implementation is
> `Vetri2425/PX4-Autopilot-3WD-Prod` @ `b5189bd734` (F1.3). Differences in v1.17:
>
> - **IMU lever-arm correction added** (`255453d967`): the measurement is corrected for
>   `ω × (−EKF2_IMU_POS_*)` before fusion. Not in this spec.
> - `Ekf::fuseBodyFrameVelocity()` moved to `EKF/aid_sources/body_velocity_fusion.cpp`
>   (byte-identical body) so WENC no longer depends on `CONFIG_EKF2_EXTERNAL_VISION`.
> - Parameter-struct fields use v1.17 names (`ekf2_wenc_ctrl`, `ekf2_wenc_rad`, `ekf2_wenc_delay`,
>   `ekf2_wenc_noise`, `ekf2_wenc_lat_n`, `ekf2_wenc_gate`); `PublishAidSourceStatus(timestamp, ...)`.
> - Board: `boards/px4/fmu-v6x/rover.px4board` (Pixhawk 6X), not the CubeOrangePlus defconfig.
> - **Spec/code discrepancy (both versions):** §4/§5.3 give the body-Z variance as `1e3f`; the
>   implementation uses `sq(1000.f)` = 1e6. Both give a near-zero Kalman gain on Z; the code is
>   authoritative.
> - Field values differ from the defaults below: the 2026-08 logs ran `EKF2_WENC_GATE=3`.
>   The Pixhawk 6X mount invalidates `EKF2_IMU_POS_X=0.10`; re-measure before enabling WENC.

---

# Wheel-Encoder Velocity Fusion for EKF2 — Design Spec

**Date:** 2026-06-03
**Author:** Vetri
**Status:** Draft — pending review
**Target firmware:** PX4 **v1.16.2** fork, branch `main`, HEAD `09bcec63` (2026-06-01)
**Vehicle:** DYX 3WD marking rover — 2 driven wheels + front passive caster, wheelbase 470 mm, CubeOrangePlus.

---

## 1. Version & architecture baseline (verified, not assumed)

Before designing the patch, the actual tree was inspected. Everything below is
confirmed against source in this exact checkout — file:line anchors are real.

### 1.1 Repo version
- Base: PX4 **v1.16.2** (confirmed via `.github/workflows/build_all_targets.yml`
  version-tag logic and the rover retargeting commits, e.g.
  `24d78a81 ci(rover): drop incompatible DifferentialPosControl from v1.16.2 overlay`).
- This is a **fork** carrying custom rover patches (RoboClaw creep fixes,
  `RD_TANK_MODE` retarget, navigator/land-detector v1.16.2 compatibility).
- No upstream tags are merged into `main`; version is pinned by commit, not tag.

### 1.2 EKF2 aiding architecture (the pattern this patch follows)

The EKF2 estimator uses a uniform per-aid-source pattern. The cleanest analog
for a new body-frame velocity aid is a **hybrid** of two existing sources:

| Concern | Reference source | Anchor |
|---|---|---|
| Driver→uORB sample plumbing, ring buffer, enable gating | **auxvel** | `aid_sources/auxvel/auxvel_fusion.cpp:36` |
| The actual **body-frame** velocity math (rotation-aware Jacobian) | **external_vision** | `aid_sources/external_vision/ev_vel_control.cpp:168` |

**Why hybrid:** `auxvel` gives the right *scaffolding* (buffer, `setAuxVelData`
at `estimator_interface.cpp:405`, sample struct at `common.h:260`, control call
site at `control.cpp:151`), but its fusion (`fuseHorizontalVelocity`) works in
**NED**, not body frame. Wheel speed is inherently a **body-X** measurement, so
the math must come from `Ekf::fuseBodyFrameVelocity()`.

### 1.3 `fuseBodyFrameVelocity()` — exact contract (verified)

`ev_vel_control.cpp:168`:
```cpp
void Ekf::fuseBodyFrameVelocity(estimator_aid_source3d_s &aid_src,
                                const uint64_t &timestamp,
                                const Vector3f &measurement,       // body-frame (x,y,z) m/s
                                const Vector3f &measurement_var,   // per-axis variance
                                const float &innovation_gate);
```
- Innovation = `_R_to_earth.transpose() * _state.vel - measurement` (line 173) —
  i.e. it rotates the NED state velocity into body frame, so the measurement is
  compared correctly regardless of heading. This is exactly why
  `fuseDirectStateMeasurement` (the original sketch) is **wrong**: body-X
  velocity is a linear combination of vel_N/E/D weighted by attitude, and its
  Jacobian touches the quaternion states. The SymForce-generated
  `ComputeBodyVelInnovVarH` (line 175) handles that.
- Fuses all three axes sequentially (lines 186–198). **Axes with huge variance
  get a near-zero Kalman gain**, so they don't meaningfully update the state.
- **Side effect to document:** sets both `_time_last_hor_vel_fuse` *and*
  `_time_last_ver_vel_fuse` (lines 203–204) whenever any axis fuses. We fuse Z
  with a large-but-finite variance, so Z produces ~no state change, but the
  vertical-fuse *timestamp* is refreshed. On this rover, height is independently
  aided by baro/GPS, so this is acceptable — but it is noted as a known
  interaction, not a silent assumption.

### 1.4 Driver contract (verified, unchanged by this patch)

`Roboclaw.cpp:267-273` already publishes the topic with no geometry knowledge:
```cpp
wheel_encoders.wheel_speed[0] = speed_right / RBCLW_COUNTS_REV * 2π;  // rad/s, RIGHT
wheel_encoders.wheel_speed[1] = speed_left  / RBCLW_COUNTS_REV * 2π;  // rad/s, LEFT
_wheel_encoders_pub.publish(wheel_encoders);
```
`msg/WheelEncoders.msg`: `float32[2] wheel_speed # [rad/s]` with the documented
convention **`[0]=right, [1]=left`**, both positive moving forward. The driver
stays untouched — it remains geometry-agnostic per its own header doc
(`Roboclaw.cpp:576`).

---

## 2. Goal & scope

Fuse RoboClaw wheel-encoder velocity into EKF2 as a **secondary** body-frame
velocity aid to improve 3WD rover accuracy:

1. **GPS-denied dead reckoning** — bound horizontal drift during GPS dropout.
2. **Low-speed accuracy** — encoders beat GPS velocity at crawl speed.
3. **Non-holonomic lateral constraint** — fuse "body-Y velocity ≈ 0" (no
   side-slip), the single biggest heading-drift win for a ground rover.

**Out of scope (YAGNI):** flying-vehicle use, primary/standalone aiding, slip
detection beyond innovation gating, third-wheel odometry (caster is passive).

Entire feature is compile-guarded behind `CONFIG_EKF2_WHEEL_ENCODER` (default
`n`) so no other airframe is affected.

---

## 3. Locked design decisions

| # | Decision | Choice | Rationale |
|---|---|---|---|
| 1 | rad/s → m/s conversion location | **EKF2 module** (`EKF2_WENC_RAD`) | Driver stays geometry-agnostic; message contract (rad/s) preserved. |
| 2 | Lateral side-slip constraint | **Enabled**, tunable `EKF2_WENC_LAT_N` | Largest non-holonomic accuracy payoff. |
| 3 | 3WD geometry | **2 driven + passive caster** | `(ωR+ωL)/2` differential-drive forward-speed math holds. |
| 4 | Aiding role | **Secondary only** | GPS/flow stays primary; gate behind `isHorizontalAidingActive()`. |
| 5 | Fusion math | **Reuse `fuseBodyFrameVelocity`** | Validated SymForce Jacobians; correct rotation handling. |
| 6 | Aid source type | **`estimator_aid_source3d_s`** | Required by `fuseBodyFrameVelocity`. |

---

## 4. Data flow

```
RoboClaw.cpp ──publish──> wheel_encoders {wheel_speed[2] rad/s, R=0 L=1}
        │  (UNCHANGED)
        ▼
EKF2.cpp::UpdateWheelEncoderSample()            [new, mirrors UpdateAuxVelSample @2307]
        │  v_fwd = 0.5*(wheel_speed[0]+wheel_speed[1]) * EKF2_WENC_RAD
        │  timestamp -= EKF2_WENC_DELAY
        ▼
_ekf.setWheelEncoderData(sample)                [new, mirrors setAuxVelData @405]
        ▼
_wheel_encoder_buffer  (TimestampedRingBuffer<wheelEncoderSample>, lazy-alloc)
        ▼
control.cpp ──> controlWheelEncoderFusion(imu_delayed)   [new call @ ~line 155, after AUXVEL block]
        │  pop_first_older_than(imu.time_us)
        │  if isHorizontalAidingActive():
        │    fuseBodyFrameVelocity(
        │        _aid_src_wheel_encoder, sample.time_us,
        │        measurement = Vector3f(v_fwd, 0.f, 0.f),
        │        var = Vector3f(wenc_noise², lat_n², 1e3f),
        │        gate = max(EKF2_WENC_GATE, 1.f))
        ▼
estimator_aid_src_wheel_encoder (uORB)          [new, mirrors estimator_aid_src_aux_vel]
        └─> logging + QGC innovation/test-ratio for tuning
```

---

## 5. Components (isolated units, each ≤ one purpose)

### 5.1 Sample struct + params — `EKF/common.h`
Add after `auxVelSample` (line 264), guarded by `CONFIG_EKF2_WHEEL_ENCODER`:
```cpp
struct wheelEncoderSample {
    uint64_t time_us{};       ///< measurement timestamp (uSec)
    float    vel_body_fwd{};  ///< body-X forward velocity (m/s)
    float    vel_fwd_var{};   ///< variance of forward velocity ((m/s)^2)
};
```
Parameters in `parameters_t` (mirroring `auxvel_gate` etc.):
`wenc_ctrl`, `wenc_rad`, `wenc_noise`, `wenc_lat_noise`, `wenc_gate`, `wenc_delay_ms`.

> **Param-name limit:** PX4 param names are ≤ 16 chars (verified: longest
> existing is `EKF2_NOAID_NOISE` = 16). All public names below respect this —
> note `EKF2_WHEEL_RADIUS` (17) would be **invalid**, hence `EKF2_WENC_RAD`.

### 5.2 Interface + ring buffer — `EKF/estimator_interface.{h,cpp}`
- `.h`: declare `void setWheelEncoderData(const wheelEncoderSample &);` and
  `TimestampedRingBuffer<wheelEncoderSample> *_wheel_encoder_buffer{nullptr};`
- `.cpp`: copy `setAuxVelData` (lines 405–438) verbatim — lazy buffer alloc,
  `_min_obs_interval_us` rate-limit, `delete` in destructor (line 74).

### 5.3 Fusion control — `EKF/aid_sources/wheel_encoder/wheel_encoder_fusion.cpp` (new)
```cpp
void Ekf::controlWheelEncoderFusion(const imuSample &imu_sample) {
    if (_wheel_encoder_buffer) {
        wheelEncoderSample sample;
        if (_wheel_encoder_buffer->pop_first_older_than(imu_sample.time_us, &sample)) {
            if (isHorizontalAidingActive()) {
                fuseBodyFrameVelocity(_aid_src_wheel_encoder, sample.time_us,
                    Vector3f(sample.vel_body_fwd, 0.f, 0.f),
                    Vector3f(sample.vel_fwd_var,
                             sq(_params.wenc_lat_noise),
                             1e3f),                       // Z: large-but-finite → ~zero gain
                    math::max(_params.wenc_gate, 1.f));
            }
        }
    }
}
void Ekf::stopWheelEncoderFusion() { ECL_INFO("stopping wheel encoder fusion"); }
```
- `ekf.h`: add `estimator_aid_source3d_s _aid_src_wheel_encoder{};`, a
  `aid_src_wheel_encoder()` getter, and declare both methods (mirror lines
  420 / 617 / 982–983).

### 5.4 Control wiring — `EKF/control.cpp`
After the `CONFIG_EKF2_AUXVEL` block (lines 151–154):
```cpp
#if defined(CONFIG_EKF2_WHEEL_ENCODER)
    controlWheelEncoderFusion(imu_delayed);
#endif
```

### 5.5 uORB bridge — `EKF2.{hpp,cpp}`
- `.hpp`: `uORB::Subscription _wheel_encoders_sub{ORB_ID(wheel_encoders)};`,
  aid-source publisher `_estimator_aid_src_wheel_encoder_pub`, `hrt_abstime`
  last-publish, and `void UpdateWheelEncoderSample(ekf2_timestamps_s &);`
- `.cpp`: implement `UpdateWheelEncoderSample` (mirror `UpdateAuxVelSample`
  @2307): read topic, compute `v_fwd = 0.5f*(ws[0]+ws[1])*wheel_radius`,
  apply delay, set `vel_fwd_var = sq(wenc_noise)`, call `setWheelEncoderData`.
  Call from `Run()` next to line 797. Add `PublishAidSourceStatus(...)` for the
  new aid source near line 1165.

### 5.6 Build wiring — `EKF/Kconfig` + `EKF/CMakeLists.txt`
- `Kconfig` after `EKF2_AUXVEL` (line 47):
  ```
  menuconfig EKF2_WHEEL_ENCODER
  depends on MODULES_EKF2
          bool "wheel encoder velocity fusion support"
          default n
          ---help---
              EKF2 wheel-encoder body-frame velocity fusion (ground rovers).
  ```
- `CMakeLists.txt` after the AUXVEL block (lines 62–64):
  ```cmake
  if(CONFIG_EKF2_WHEEL_ENCODER)
      list(APPEND EKF_SRCS aid_sources/wheel_encoder/wheel_encoder_fusion.cpp)
  endif()
  ```

### 5.7 Parameter metadata — `params_wheel_encoder.yaml` (new)
Create a dedicated file mirroring `params_aux_velocity.yaml` (the EKF2 convention
is one `params_*.yaml` per aid source, not `module.yaml`). Define:

| Param (≤16 ch) | Unit | Default | Meaning |
|---|---|---|---|
| `EKF2_WENC_CTRL` | bitmask/bool | 0 | Enable wheel-encoder fusion |
| `EKF2_WENC_RAD`  | m   | 0.0 (disabled until calibrated) | Effective rolling radius |
| `EKF2_WENC_NOISE`| m/s | 0.1 | Forward-velocity measurement noise |
| `EKF2_WENC_LAT_N`| m/s | 0.1 | Side-slip (body-Y) constraint noise |
| `EKF2_WENC_GATE` | σ   | 5.0 | Innovation gate |
| `EKF2_WENC_DELAY`| ms  | 5   | Sensor delay relative to IMU |

### 5.8 Board config
Add `CONFIG_EKF2_WHEEL_ENCODER=y` only to the CubeOrangePlus rover board defconfig
used by this vehicle — not to generic/flying defaults.

---

## 6. Conversion math

- Forward speed: `v_fwd = 0.5 * (ω_right + ω_left) * r`  where `r = EKF2_WENC_RAD`.
- Lateral constraint: measured body-Y velocity = `0` with variance `EKF2_WENC_LAT_N²`.
- Sign: both `wheel_speed[0]` (R) and `[1]` (L) positive forward → `v_fwd > 0` forward. ✔ matches message contract.

---

## 7. Error handling / failure modes

| Failure | Mechanism | Mitigation |
|---|---|---|
| Wheel slip (mud/skid/turn) | Biased, correlated innovation | `EKF2_WENC_GATE` rejects outliers; tune `EKF2_WENC_NOISE`; visible in aid-src test ratio |
| Radius miscalibration | Constant velocity scale bias | Observable as steady innovation offset; documented calibration step; `EKF2_WENC_RAD=0` disables |
| GPS loss | No init capability | Secondary-only gating; standard EKF dead-reckon timeout still applies if all aiding lost |
| Vertical-fuse timestamp refresh (§1.3) | Z axis marks `_time_last_ver_vel_fuse` | Acceptable — baro/GPS height independent; documented, monitored |
| Bad/old samples | Stale or too-fast data | `_min_obs_interval_us` rate-limit + buffer ordering (inherited from auxvel) |

---

## 8. Testing strategy

1. **Build:** `boardconfig` with `CONFIG_EKF2_WHEEL_ENCODER=y`; `make px4_sitl`
   default (guard `n`) must still compile unchanged.
2. **SITL:** rover sim with `EKF2_WENC_CTRL=1`, plausible `EKF2_WENC_RAD`;
   straight-line run → `estimator_aid_src_wheel_encoder.innovation` ≈ 0, test
   ratio < 1; cornering → bounded innovation, no rejection storm.
3. **GPS-dropout SITL:** disable GPS mid-run; confirm horizontal position drift
   is bounded vs. baseline (encoder off) over the same interval.
4. **Field replay:** compare against validated baseline (log 59); inspect aid-src
   innovation/test-ratio to tune `EKF2_WENC_NOISE` / `EKF2_WENC_GATE`.

---

## 9. Documentation references

- **PX4 EKF2 tuning / aiding** — https://docs.px4.io/main/en/advanced_config/tuning_the_ecl_ekf.html
- **PX4 EKF2 module/params** — https://docs.px4.io/main/en/advanced_config/parameter_reference.html#ekf2 (style guide for new `EKF2_WENC_*` metadata)
- **PX4 RoboClaw driver** — https://docs.px4.io/main/en/modules/modules_driver.html (and in-tree `Roboclaw.cpp` header doc, line 576)
- **PX4 uORB / message conventions** — https://docs.px4.io/main/en/middleware/uorb.html
- **RoboClaw user manual (BasicMicro)** — encoder counts/rev → rad/s relationship already applied by the driver via `RBCLW_COUNTS_REV`.
- **In-tree pattern references:** `aid_sources/auxvel/auxvel_fusion.cpp`,
  `aid_sources/external_vision/ev_vel_control.cpp:168`, `common.h:260`,
  `estimator_interface.cpp:405`, `EKF2.cpp:2307`.

---

## 10. Implementation order (dependency-sorted)

1. `common.h` — sample struct + param fields (+ `ekf.h` param defaults).
2. `Kconfig` + `CMakeLists.txt` — guard symbol + source registration.
3. `estimator_interface.{h,cpp}` — buffer + `setWheelEncoderData`.
4. `ekf.h` + `aid_sources/wheel_encoder/wheel_encoder_fusion.cpp` — fusion.
5. `control.cpp` — call site.
6. `EKF2.{hpp,cpp}` — uORB sub, conversion, aid-src publish.
7. `params_wheel_encoder.yaml` — parameter metadata.
8. Board defconfig — enable on rover only.
9. `make format`; SITL build (guard on/off); SITL + replay tests.
