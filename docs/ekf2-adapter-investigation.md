# PX4 EKF2 Adapter Investigation

## Goal

Expose the PX4 EKF core as a Hakoniwa-facing adapter that can be linked
directly with Hakoniwa, without going through PX4 SITL uORB orchestration.

The intended final structure is:

```text
Hakoniwa sensors
  -> HIL_SENSOR / HIL_GPS compatible data
  -> Hakoniwa-side PX4 EKF adapter
  -> PX4 EKF core (`src/modules/ekf2/EKF/*`)
  -> estimated attitude / velocity / position outputs
```

## What To Reuse From PX4

### PX4-side orchestrator

The PX4 module entrypoint is:

- `thirdparty/PX4-Autopilot/src/modules/ekf2/EKF2.cpp`

This file is PX4-specific glue. It:

- subscribes to uORB topics
- converts sensor topics into EKF sample structs
- calls `_ekf.set*Data(...)`
- calls `_ekf.update()`
- publishes estimated states back to uORB

This file is **not** the main extraction target.

### Reusable EKF core

The reusable estimator core is under:

- `thirdparty/PX4-Autopilot/src/modules/ekf2/EKF/*`

Important files:

- `common.h`
- `estimator_interface.h`
- `ekf.h`

These provide:

- input sample structs such as `imuSample`, `gnssSample`, `magSample`,
  `baroSample`
- sensor input functions such as `setIMUData(...)`, `setGpsData(...)`,
  `setMagData(...)`, `setBaroData(...)`
- state getters such as `getQuaternion()`, `getVelocity()`, `getPosition()`,
  `getLatLonAlt()`

## Upstream Source Policy

The PX4 upstream source tree under:

- `thirdparty/PX4-Autopilot`

should be treated as read-only.

Adapter integration policy:

- do not patch PX4 source files directly
- add local compatibility headers under this repository when PX4 expects
  generated/platform-specific headers
- add thin wrapper code in this repository for Hakoniwa-facing orchestration
- resolve standalone build gaps through local CMake glue, include paths,
  compile definitions, or source-specific compile options

This keeps the EKF extraction auditable and makes it easier to rebase to newer
PX4 revisions later.

## Hakoniwa Input Contract

The adapter should treat the following as its primary sensor inputs:

- `HIL_SENSOR`
- `HIL_GPS`

These are already generated on the Hakoniwa side today by:

- `src/service/aircraft/impl/aricraft_mavlink_message_buider.hpp`
  - `build_hil_sensor(...)`
  - `build_hil_gps(...)`

Therefore, the adapter input contract should follow the existing Hakoniwa HIL
semantics, instead of introducing a separate sensor message family.

## Mapping Policy

### `HIL_SENSOR` to PX4 EKF samples

`HIL_SENSOR` should be split into three PX4 EKF inputs:

- `imuSample`
- `magSample`
- `baroSample`

Mapping:

- `time_usec` -> `imuSample.time_us`, `magSample.time_us`,
  `baroSample.time_us`
- `xgyro`, `ygyro`, `zgyro` -> `imuSample.delta_ang = gyro * dt`
- `xacc`, `yacc`, `zacc` -> `imuSample.delta_vel = accel * dt`
- `dt` -> `imuSample.delta_ang_dt`, `imuSample.delta_vel_dt`
- `xmag`, `ymag`, `zmag` -> `magSample.mag`
- `pressure_alt` -> `baroSample.hgt`

Important note:

- PX4 EKF does **not** accept raw gyro/accel rates directly.
- `imuSample` requires integrated quantities:
  - `delta_ang`
  - `delta_vel`

Therefore the adapter must keep track of the HIL sensor update interval and
form:

- `delta_ang = gyro_rad_s * dt_sec`
- `delta_vel = accel_mps2 * dt_sec`

### `HIL_GPS` to PX4 EKF samples

`HIL_GPS` should be mapped into `gnssSample`.

Mapping:

- `time_usec` -> `gnssSample.time_us`
- `lat` (degE7) -> `gnssSample.lat` (deg)
- `lon` (degE7) -> `gnssSample.lon` (deg)
- `alt` (mm) -> `gnssSample.alt` (m)
- `vn`, `ve`, `vd` (cm/s) -> `gnssSample.vel` (m/s, NED)
- `eph` -> `gnssSample.hacc`
- `epv` -> `gnssSample.vacc`
- `fix_type` -> `gnssSample.fix_type`
- `satellites_visible` -> `gnssSample.nsats`

Current non-goals for the first version:

- dual-GNSS yaw
- GPS antenna body offset
- GPS spoof/jam flags

For the first version:

- `gnssSample.yaw = NaN`
- `gnssSample.yaw_acc = 0`
- `gnssSample.yaw_offset = 0`
- `gnssSample.spoofed = false`
- `gnssSample.jammed = false`
- `gnssSample.pos_body = {0, 0, 0}`

## `sacc` Handling

`gnssSample.sacc` is required by PX4 EKF as the 1-sigma GPS speed accuracy.

This value is **not present** in the current `HIL_GPS` payload.

Design decision:

- keep `HIL_SENSOR` and `HIL_GPS` as the adapter's primary runtime inputs
- add `sacc` as GPS sensor configuration on the Hakoniwa side
- let the adapter read that configured GPS speed accuracy and inject it into
  `gnssSample.sacc`

This keeps runtime sensor transport unchanged while still providing EKF with
the required quality metadata.

Planned Hakoniwa-side change:

- extend the GPS sensor config section with `sacc`

Example future config:

```json
"gps": {
  "sampleCount": 1,
  "noise": 0.0,
  "sacc": 0.5
}
```

## Public Adapter Direction

The public adapter should be Hakoniwa-facing, not PX4-uORB-facing.

Suggested responsibilities:

- accept HIL-compatible sensor input
- convert into PX4 EKF sample structs
- call PX4 EKF core update
- expose estimated state through a small adapter API

The public interface should remain small and stable even if the internal PX4
EKF integration changes later.

## Current Scope

This note intentionally stops at:

- identifying the reusable PX4 EKF core
- fixing the Hakoniwa input contract
- defining the first-pass sample mapping policy
- deciding how `sacc` should enter the system

Implementation of the actual PX4 EKF wrapper is the next step.
