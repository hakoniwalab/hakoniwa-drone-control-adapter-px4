# PX4 EKF Adapter Evaluation Plan

## Goal

Define the first evaluation path for the PX4 EKF adapter without deeply
integrating new runtime machinery yet.

The immediate goal is to make it possible to compare:

- simulation truth
- EKF input sensor data
- EKF estimated state

on the same time axis, with the same timestamp key:

- `time_usec`

This document intentionally defines the evaluation contract first. A concrete
runtime logger can be added later as a small follow-up.

## Evaluation Output Policy

The first evaluation output should be CSV-based and should align the following
three series by `time_usec`:

- `truth`
- `sensor`
- `estimate`

The intended shape is either:

- one unified CSV with prefixed columns, or
- three CSV files that all use the same `time_usec` and can be joined later

For the first implementation, either is acceptable as long as the comparison is
stable and reproducible.

### Recommended column groups

#### Truth

Truth should be logged from the simulator state that represents the vehicle's
actual state.

Minimum truth fields:

- `time_usec`
- `truth_pos_n_m`
- `truth_pos_e_m`
- `truth_pos_d_m`
- `truth_vel_n_mps`
- `truth_vel_e_mps`
- `truth_vel_d_mps`
- `truth_q_w`
- `truth_q_x`
- `truth_q_y`
- `truth_q_z`

#### Sensor

Sensor should reflect the exact values pushed into the EKF adapter interface,
not the raw internal simulator state before HIL-style conversion.

Minimum sensor fields:

- `time_usec`
- `sensor_xacc_mps2`
- `sensor_yacc_mps2`
- `sensor_zacc_mps2`
- `sensor_xgyro_rad_s`
- `sensor_ygyro_rad_s`
- `sensor_zgyro_rad_s`
- `sensor_xmag_gauss`
- `sensor_ymag_gauss`
- `sensor_zmag_gauss`
- `sensor_pressure_alt_m`
- `sensor_gps_lat_deg`
- `sensor_gps_lon_deg`
- `sensor_gps_alt_m`
- `sensor_gps_vn_mps`
- `sensor_gps_ve_mps`
- `sensor_gps_vd_mps`
- `sensor_gps_eph_m`
- `sensor_gps_epv_m`
- `sensor_gps_fix_type`
- `sensor_gps_satellites_visible`
- `sensor_gps_sacc_mps`

#### Estimate

Estimate should reflect the adapter output after `update()`.

Minimum estimate fields:

- `time_usec`
- `estimate_pos_n_m`
- `estimate_pos_e_m`
- `estimate_pos_d_m`
- `estimate_vel_n_mps`
- `estimate_vel_e_mps`
- `estimate_vel_d_mps`
- `estimate_q_w`
- `estimate_q_x`
- `estimate_q_y`
- `estimate_q_z`
- `estimate_attitude_valid`
- `estimate_local_position_valid`
- `estimate_global_position_valid`

## Frame Convention

All comparisons should be expressed in PX4-compatible frames.

### World frame

Use:

- `NED` for world position and velocity

This means:

- `+X = North`
- `+Y = East`
- `+Z = Down`

### Body frame

Use:

- `FRD` for body-frame sensor and attitude-related interpretation

This means:

- `+X = Forward`
- `+Y = Right`
- `+Z = Down`

This is important because the first validation step is expected to catch
sign/axis mistakes before any fine estimator tuning is attempted.

## First Evaluation Scenario

The first evaluation scenario should be deliberately simple:

- stationary hover, or
- ground static

Recommended starting point:

- `ground static` first
- then `stationary hover`

Rationale:

- removes translational dynamics as a confounder
- makes frame/sign mismatches obvious
- allows early validation of sensor wiring and estimator initialization

The first pass does **not** need maneuvering flight.

## Comparison Metrics

The first evaluation should define the following comparison metrics.

### Position error

Compare estimate vs truth in NED:

```text
position_error_ned_m =
  estimate_position_ned_m - truth_position_ned_m
```

Derived scalar:

```text
position_error_norm_m = ||position_error_ned_m||
```

### Velocity error

Compare estimate vs truth in NED:

```text
velocity_error_ned_mps =
  estimate_velocity_ned_mps - truth_velocity_ned_mps
```

Derived scalar:

```text
velocity_error_norm_mps = ||velocity_error_ned_mps||
```

### Attitude error

Compare estimate vs truth using quaternion difference:

```text
q_error = q_truth^{-1} * q_estimate
```

Derived scalar:

- quaternion angle error in radians or degrees

For the first phase, it is sufficient to compute and inspect:

- `attitude_error_angle_deg`

The precise implementation detail can be added later, but the comparison should
be based on quaternion difference rather than Euler-angle subtraction.

## Initial Pass Criteria

The initial pass/fail criteria should remain intentionally weak.

Do **not** start with tight numerical thresholds.

The first checks are only:

1. EKF valid flags eventually rise
   - at minimum, attitude valid
   - local/global position valid when the sensor configuration allows it
2. the estimate does not diverge
   - no obviously unbounded position/velocity growth under static conditions
   - quaternion remains finite and normalized enough for inspection
3. the sign and axis conventions are correct
   - N/E/D do not appear swapped
   - FRD-related signs do not appear inverted

This first phase is intended to catch:

- frame mismatch
- sign inversion
- bad timestamp wiring
- invalid initialization
- obviously broken sensor-to-sample mapping

It is **not** intended yet to prove estimator quality.

## Suggested Logger Shape

The first runtime implementation can remain small.

Recommended minimal shape:

- a small standalone logger or logger skeleton
- append one row per adapter update
- each row contains:
  - truth snapshot
  - last sensor snapshot used for that update
  - estimate snapshot after that update

Pseudo shape:

```text
on sensor push:
  cache latest sensor sample

on adapter update:
  read truth snapshot
  read estimate snapshot
  write CSV row with:
    time_usec
    truth_*
    sensor_*
    estimate_*
```

This avoids deep runtime integration while keeping the first evaluation path
clear.

## Out Of Scope For This Step

This document does not add:

- runtime integration into the main Hakoniwa loop
- automatic plotting
- tight numeric thresholds
- moving-flight validation
- sensor fault or delay injection evaluation

Those should be added only after the first static evaluation path is working.
