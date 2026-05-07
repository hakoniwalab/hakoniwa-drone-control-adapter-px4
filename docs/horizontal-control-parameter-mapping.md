# PX4 Horizontal Control Parameter Mapping

## Purpose

This document defines the public parameter contract used by
`Px4HorizontalPositionControlBackend`.

The goal is to make the following explicit:

- which PX4 parameters are directly consumed by the backend
- which Hakoniwa-side parameters can map directly
- which values are only approximate mappings
- which values must be treated as PX4-specific extras

## Scope

This document covers the Hakoniwa-shaped horizontal backend currently wrapped
around PX4 `PositionControl`.

Out of scope:

- vertical control
- attitude control
- rate control
- control allocation
- scheduler policy

## Backend Config Surface

`Px4HorizontalPositionControlBackend` currently accepts:

- horizontal position proportional gain
- horizontal velocity PID gains
- horizontal velocity limit
- tilt limit
- hover thrust
- thrust min/max
- horizontal thrust margin
- acceleration decouple flag

The current config struct is:

- `Px4HorizontalPositionControlBackendConfig`

## PX4 Parameters Used By The Horizontal Backend

The backend currently consumes these PX4 parameters:

- `MPC_XY_P`
- `MPC_XY_VEL_P_ACC`
- `MPC_XY_VEL_I_ACC`
- `MPC_XY_VEL_D_ACC`
- `MPC_XY_VEL_MAX`
- `MPC_TILTMAX_AIR`
- `MPC_THR_XY_MARG`
- `MPC_ACC_DECOUPLE`
- `MPC_THR_HOVER`
- `MPC_THR_MIN`
- `MPC_THR_MAX`

Important note:

- the backend returns roll/pitch tilt targets as a Hakoniwa-facing derived
  contract
- PX4 `PositionControl` itself naturally outputs thrust and attitude setpoints

## Hakoniwa To PX4 Mapping

### Direct mapping

The following Hakoniwa native parameters are mapped directly when X/Y values
are symmetric.

| Hakoniwa | PX4 |
|---|---|
| `PID_POS_X_Kp`, `PID_POS_Y_Kp` | `MPC_XY_P` |
| `PID_POS_VX_Kp`, `PID_POS_VY_Kp` | `MPC_XY_VEL_P_ACC` |
| `PID_POS_VX_Ki`, `PID_POS_VY_Ki` | `MPC_XY_VEL_I_ACC` |
| `PID_POS_VX_Kd`, `PID_POS_VY_Kd` | `MPC_XY_VEL_D_ACC` |
| `PID_POS_MAX_SPD` | `MPC_XY_VEL_MAX` |

Current converter rule:

- X/Y pairs must match within tolerance
- otherwise config composition fails explicitly

Reason:

- PX4 horizontal parameters are shared across X and Y
- silently averaging or selecting one axis would hide semantic mismatch

### Approximate mapping

The following mapping is practical but not semantic 1:1.

| Hakoniwa | PX4 | Rule |
|---|---|---|
| `PID_POS_MAX_ROLL`, `PID_POS_MAX_PITCH` | `MPC_TILTMAX_AIR` | use `min(roll, pitch)` |

Reason:

- Hakoniwa keeps roll and pitch angle limits separately
- PX4 uses one tilt limit for both axes

## PX4 Extra Parameters

The following values are handled as PX4-specific extras.

- `MPC_THR_XY_MARG`
- `MPC_ACC_DECOUPLE`
- `MPC_THR_HOVER`
- `MPC_THR_MIN`
- `MPC_THR_MAX`

This means:

- they may exist even when Hakoniwa-native parameters do not
- they belong in PX4-side config, not in Hakoniwa-native naming

## Runtime Interpretation

The backend supports two runtime input modes:

- `HorizontalControlMode::Position`
- `HorizontalControlMode::Velocity`

These modes do not change backend configuration.

They only change how the runtime input is converted into PX4 setpoint shape:

- position mode:
  - x/y position setpoints are provided
  - x/y velocity setpoints are left unset
- velocity mode:
  - x/y velocity setpoints are provided
  - x/y position setpoints are left unset

This matches PX4 `PositionControl` usage more closely than exposing separate
public PX4-style controller classes.

## Converter Rule

The converter that writes `px4-controller-config.json` should:

- map symmetric Hakoniwa XY gains into shared PX4 XY gains
- reject non-symmetric XY mappings by default
- convert tilt limits from degrees into radians
- merge PX4 extras explicitly

The C++ runtime is not responsible for this conversion logic.
