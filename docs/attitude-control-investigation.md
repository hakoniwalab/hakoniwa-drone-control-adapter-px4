# Attitude Control Investigation

## Purpose

This note captures the initial investigation result for the next control layer:
PX4 `AttitudeControl`.

It follows the standard workflow defined in
`docs/control-porting-workflow.md`.

## PX4 Core

Target class:

- `src/modules/mc_att_control/AttitudeControl/AttitudeControl.hpp`
- `src/modules/mc_att_control/AttitudeControl/AttitudeControl.cpp`

Relevant public API:

- `setProportionalGain(...)`
- `setRateLimit(...)`
- `setAttitudeSetpoint(...)`
- `adaptAttitudeSetpoint(...)`
- `update(...)`

Core behavior:

- input attitude is a unit quaternion
- desired attitude is a unit quaternion
- yaw feed-forward is given as a world-frame yaw rate
- output is a body-frame angular-rate setpoint

Important implication:

- PX4 attitude control is not an Euler PID layer
- it is a quaternion-based attitude-to-rate conversion layer

## PX4 Parameters

Direct core-related parameters:

- `MC_ROLL_P`
- `MC_PITCH_P`
- `MC_YAW_P`
- `MC_YAW_WEIGHT`
- `MC_ROLLRATE_MAX`
- `MC_PITCHRATE_MAX`
- `MC_YAWRATE_MAX`

Based on the existing mapping study, the current Hakoniwa-side relation is:

- `PID_ROLL_Kp` -> `MC_ROLL_P`
- `PID_PITCH_Kp` -> `MC_PITCH_P`
- `PID_YAW_Kp` -> `MC_YAW_P`
- `PID_ROLL_RPM_MAX` -> `MC_ROLLRATE_MAX`
- `PID_PITCH_RPM_MAX` -> `MC_PITCHRATE_MAX`
- `PID_YAW_RPM_MAX` -> `MC_YAWRATE_MAX`

PX4 extra:

- `MC_YAW_WEIGHT`

Current non-directly-mapped Hakoniwa parameters:

- `PID_ROLL_Ki`
- `PID_ROLL_Kd`
- `PID_PITCH_Ki`
- `PID_PITCH_Kd`
- `PID_YAW_Ki`
- `PID_YAW_Kd`

Reason:

- PX4 `AttitudeControl` core is proportional only at this layer

## Hakoniwa-Side Equivalent

Closest existing components:

- `src/controller/impl/drone_angle_controller.hpp`
- `src/controller/impl/drone_heading_controller.hpp`

### `DroneAngleController::run_angle(...)`

Current role:

- roll angle target -> roll rate target
- pitch angle target -> pitch rate target
- yaw rate target -> passthrough

Current characteristics:

- Euler-based
- own cycle management
- per-axis PID
- rate limiting after roll/pitch control

### `DroneHeadingController`

Current role:

- current yaw + target yaw angle -> yaw rate target

Current characteristics:

- angle normalization in degrees
- separate cycle management
- PID-based yaw-rate generation

## Consistency Notes

The semantic mismatch is not in the output.

- Hakoniwa angle/heading path output: target angular rate
- PX4 attitude core output: target angular rate

The semantic mismatch is in the input representation and decomposition.

- Hakoniwa splits yaw handling into a separate heading controller
- Hakoniwa angle control is Euler-based
- PX4 attitude control expects quaternion attitude setpoints and owns yaw
  prioritization through `MC_YAW_WEIGHT`

Therefore the adapter contract should be PX4-first:

- current attitude: quaternion
- target attitude: quaternion
- target yaw rate feed-forward: scalar
- output: body-frame angular-rate target

Hakoniwa-native compatibility should adapt into this contract instead of
forcing PX4 to mirror `run_angle(...)` and `DroneHeadingController`
separately.

## Interface Direction

The first public interface direction is:

- `IAttitudeControlBackend`
- input:
  - current attitude quaternion
  - target attitude quaternion
  - target yaw rate feed-forward
- output:
  - `AngularRateTarget`

This matches PX4 core usage directly and keeps the rate-control backend
composition clean:

- attitude backend -> rate target
- rate backend -> body torque

## Euler-To-Quaternion Bridge Responsibility

Hakoniwa-side control and state handling is still centered around Euler angles.

PX4 `AttitudeControl` is quaternion-based.

Therefore a bridge step is required before calling the PX4 attitude backend.

The intended responsibility split is:

- bridge/orchestration side:
  - current Euler attitude -> current quaternion
  - target roll/pitch/yaw or heading result -> target quaternion
  - target yaw-rate feed-forward scalar construction
- PX4 attitude backend:
  - consume quaternion inputs only
  - do not own Euler parsing or conversion policy

Reason:

- this keeps PX4 backend natural to the upstream core API
- this isolates frame-order and angle-convention checks to the bridge side
- this avoids mixing Hakoniwa-specific angle semantics into the backend

## Next Implementation Steps

- add `IAttitudeControlBackend` to `hakoniwa-drone-control-adapter`
- add `Px4AttitudeControlBackend`
- add `attitude_control` section to `px4-controller-config.json`
- extend Python converter for attitude parameters
- extend C++ config loader for attitude parameters
- add backend smoke and config-only smoke
