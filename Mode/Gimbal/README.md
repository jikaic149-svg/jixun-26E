# Gimbal Mode

This directory contains the reusable control mathematics for a two-axis
gimbal. It contains no BSP, Device, FreeRTOS, Task, ISR, CAN, UART, or global
state. Every stateful API is `Context + Input -> Output`; the caller supplies
both `dt_seconds` and `now_ms` explicitly.

The only actuator-like values returned here are Pitch/Yaw current intentions.
They are not motor-enable permission. Product APP must keep the motor lock
closed by default and apply feedback-offline handling, emergency stop,
physical soft-limit protection, final current limiting, and Device/BSP output
submission.

## Formal modules

- `AngleLoop` is the encoder angle-to-current controller used by the other
  modes. It provides target/error clamps, deadband, integral separation,
  integral clamp, speed damping, optional feedforward, and wrapped Yaw error.
- `StabCalib` waits at most 100 ms for both motor encoders, then latches and
  holds their current angles. Its input type intentionally contains no IMU
  value. It does not sample or claim to calibrate gyroscope bias.
- `GimbalStab` latches current IMU attitude on entry. Pitch uses attitude PI
  plus filtered IMU-rate damping. Yaw converts IMU attitude error into an
  encoder target with polarity `-1`, then uses `AngleLoop`.
- `VisionStab` applies validated pixel error at a nominal 10 ms control period,
  then delegates current generation to `GimbalStab`. Input expires after
  250 ms. After another 500 ms without a valid target, Yaw scans around the
  last target over `+/-45 deg` at `8 deg/s`.
- `GravityCalib` visits nine Pitch encoder targets from low to high and the
  same nine from high to low. Each point settles for 1200 ms and samples for
  900 ms. It exposes 18 averaged angle/current pairs; fitting remains an
  external engineering step.
- `Scheduler` reinitializes a mode when it is entered and dispatches one of
  Idle, Relax, EncoderLock, GimbalStab, VisionStab, or GravityCalib. It does not
  silently auto-transition after encoder lock; APP decides the next mode after
  observing `ENCODERS_LOCKED`.

AngleTest and open-loop TorqueTest are deliberately absent from the formal
source tree because they are not verified production modes.

## Coordinate and timing contracts

All angles are degrees and rates are degrees per second. Wrapped angle errors
use the shortest path in `[-180, 180)`. Positive motor speed must mean the
measured encoder angle is increasing.

Vision `dx_pixels` and `dy_pixels` are already physical control errors: a
positive value requests an increase in the corresponding target angle. The
camera/Device/Product adapter must flip raw image axes, including image Y-down,
before calling Mode. Both audited Mode polarities therefore default to `+1`.

The vision predictor is fixed off:

```c
#define MODE_GIMBAL_VISION_PREDICTOR_ENABLED 0
```

There is no predictor option in the configuration, no target velocity state,
and no extrapolation. The input EMA is explicitly
`filtered = 0.72 * new + 0.28 * old` for a new sequence number.

Vision I/D coefficients are discrete gains referenced to the 10 ms nominal
period. Actual elapsed time is expressed as a ratio to that period, preserving
the audited 10 ms behavior without assuming that every function call is 10 ms.
The single-step clamps scale by the same ratio.

Unsigned `now_ms` subtraction supports normal `uint32_t` rollover. A call gap
must remain below half the counter range so an old timestamp is not confused
with a future epoch.

## Audited defaults

| Controller | Key defaults |
|---|---|
| Pitch AngleLoop | `Kp=.038`, `Ki=.006`, speed `Kd=.0045`, deadband `.3 deg`, error `14 deg`, I separation `1.5 deg`, I limit `.025`, output `.75`, target `19..112 deg` |
| Yaw AngleLoop | `Kp=.032`, `Ki=.006`, speed `Kd=.005`, deadband `.35 deg`, error `8 deg`, I separation `2 deg`, I limit `.015`, output `.32` |
| Pitch GimbalStab | `Kp=.030`, `Ki=0`, rate `Kd=.0055`, deadband `.3 deg`, error `16 deg`, output `.75`; rate filter `.3*old + .7*input` |
| Normal vision | Yaw `.00018/.000002/0`, I `.004`, step `.010 deg`; Pitch `.00018/.000003/0`, I `.006`, step `.012 deg` |
| Circle vision | Yaw `.00024/.0000012/.000030`, I `.009`, step `.022 deg`; Pitch `.00027/.0000018/.000052`, I `.014`, step `.028 deg` |
| Gravity feedforward | `.0300124*sin(angle) + .0200932*cos(angle) + .0038665`, clamped to `+/-0.6` |

AngleLoop feedforward activates outside `.8 deg`; both audited positive and
negative feedforward defaults are currently zero. These algorithm-local
limits do not replace Product APP's final safety clamp or physical limits.
Gravity feedforward is calculated by
`Mode_Gimbal_GravityFeedforward_Calculate()` but is not automatically added by
Scheduler, so Product APP retains one clearly auditable output-composition
point.

## Failure behavior and ownership

Contexts and their input/output buffers are caller-owned; no allocation occurs.
Update functions are not re-entrant for the same context. Missing feedback is
an ordinary safe-zero state. Invalid pointers, non-finite active inputs, or
non-positive `dt_seconds` return `false`, write a zero current intention, and
do not commit the candidate context update.

## Build and tests

This subdirectory has its own explicit `CMakeLists.txt`, but the template does
not add it by default. A Product must deliberately add and link
`mode_gimbal`.

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\tests\run_gcc_tests.ps1 -OutputDirectory D:\Project
powershell -NoProfile -ExecutionPolicy Bypass -File .\tests\run_arm_compile.ps1
```

`-OutputDirectory` lets host tests run from a location permitted by Windows
Application Control. `-CompileOnly` is available when policy permits compiling
but not executing a new test binary.
