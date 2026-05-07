# Altitude Control Investigation

## Purpose

This note captures the initial investigation result for the vertical control
layer based on PX4 `PositionControl`.

## PX4 Core

Target class:

- `src/modules/mc_pos_control/PositionControl/PositionControl.hpp`
- `src/modules/mc_pos_control/PositionControl/PositionControl.cpp`

Relevant public API:

- `setPositionGains(...)`
- `setVelocityGains(...)`
- `setVelocityLimits(...)`
- `setThrustLimits(...)`
- `setHorizontalThrustMargin(...)`
- `setTiltLimit(...)`
- `setHoverThrust(...)`
- `setState(...)`
- `setInputSetpoint(...)`
- `update(...)`
- `getLocalPositionSetpoint(...)`
- `getAttitudeSetpoint(...)`

Important core behavior:

- the core is 3-axis, not z-only
- z control is part of a combined position/velocity/thrust controller
- output thrust is normalized, not physical thrust force
- output attitude is derived from thrust plus yaw

For a vertical-only backend, the practical extraction strategy is:

- drive only z state and z setpoint
- set x/y position, velocity, acceleration setpoints to `NAN`
- read z thrust from the resulting local-position setpoint

## PX4 Parameters

Direct z-related parameters:

- `MPC_Z_P`
- `MPC_Z_VEL_P_ACC`
- `MPC_Z_VEL_I_ACC`
- `MPC_Z_VEL_D_ACC`
- `MPC_Z_VEL_MAX_UP`
- `MPC_Z_VEL_MAX_DN`
- `MPC_THR_HOVER`
- `MPC_THR_MIN`
- `MPC_THR_MAX`

Related but not part of the minimal vertical-only backend:

- `MPC_THR_XY_MARG`
- `MPC_TILTMAX_AIR`
- `MPC_ACC_DECOUPLE`

## Hakoniwa-Side Equivalent

Closest existing component:

- `src/controller/impl/drone_alt_controller.hpp`

Current role:

- altitude target -> target vertical speed
- target vertical speed + current ground-frame z velocity -> thrust

Current characteristics:

- position loop and speed loop are split
- output is physical thrust, not normalized thrust
- gravity and mass compensation are explicit in the native implementation
- body velocity is transformed into ground-frame velocity before z-speed PID

## Consistency Notes

The semantic mismatch is larger than in rate and attitude control.

Hakoniwa native:

- outputs physical thrust
- owns gravity and mass compensation explicitly

PX4 `PositionControl`:

- outputs normalized thrust vector
- uses hover thrust as the main normalization anchor

Therefore the altitude backend contract should be PX4-first:

- input:
  - current z position
  - current z velocity
  - current z acceleration
  - target altitude
- output:
  - normalized vertical thrust

Native Hakoniwa compatibility should adapt to this contract later instead of
forcing PX4 to emit native thrust units.

## Interface Direction

The first public interface direction is:

- `IAltitudeControlBackend`
- input:
  - z position
  - z velocity
  - z acceleration
  - target altitude
  - `dt`
- output:
  - normalized vertical thrust command

## Next Implementation Steps

- add `Px4AltitudeControlBackend`
- add minimal uORB topic shims needed by `PositionControl`
- add `altitude_control` section to `px4-controller-config.json`
- extend Python converter for vertical parameters
- extend C++ config loader for vertical parameters
- add backend smoke and config-only smoke
