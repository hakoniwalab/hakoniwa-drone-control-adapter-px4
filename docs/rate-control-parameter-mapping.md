# PX4 RateControl Parameter Mapping

## Purpose

This document defines the public parameter contract used by
`Px4RateControlBackend`.

The goal is to make the following explicit:

- which PX4 parameters are directly consumed by the backend
- which Hakoniwa-side parameters can map directly
- which values must be treated as PX4-specific extra parameters
- which values must not be auto-mapped

This document is intentionally implementation-facing.
Higher-level design rationale is maintained in `hakoniwa-drone-pro`.

The runtime-facing source of truth is expected to be
`px4-controller-config.json`.
This document defines how rate-control-related values should appear in that
file.

## Scope

This document covers only the PX4 `RateControl` core wrapped by
`Px4RateControlBackend`.

Out of scope:

- attitude control
- position control
- control allocation
- scheduler frequency policy
- radio/mode/takeoff/landing orchestration

## Backend Config Surface

`Px4RateControlBackend` currently accepts:

- PID gains for roll, pitch, yaw
- integrator limits for roll, pitch, yaw
- feed-forward gains for roll, pitch, yaw

The current config struct is:

- `Px4RateControlBackendConfig`
- `Px4RateControlGains`
- `Px4RateControlLimits`
- `Px4RateControlFeedForward`

These are applied to PX4 core through:

- `RateControl::setPidGains(...)`
- `RateControl::setIntegratorLimit(...)`
- `RateControl::setFeedForwardGain(...)`

## PX4 Parameters Used By RateControl

The following PX4 parameters are treated as core-direct for this backend.

### PID gains

- `MC_ROLLRATE_P`
- `MC_ROLLRATE_I`
- `MC_ROLLRATE_D`
- `MC_PITCHRATE_P`
- `MC_PITCHRATE_I`
- `MC_PITCHRATE_D`
- `MC_YAWRATE_P`
- `MC_YAWRATE_I`
- `MC_YAWRATE_D`

### Feed-forward gains

- `MC_ROLLRATE_FF`
- `MC_PITCHRATE_FF`
- `MC_YAWRATE_FF`

### Integrator limits

- `MC_RR_INT_LIM`
- `MC_PR_INT_LIM`
- `MC_YR_INT_LIM`

## Parameters Not Yet Applied In The Current Backend Config

The following PX4 parameters are related to rate control, but are not yet
represented directly in the current backend config struct.

- `MC_ROLLRATE_K`
- `MC_PITCHRATE_K`
- `MC_YAWRATE_K`

Reason:

- in PX4, these are used in module-side gain composition
- the current backend wrapper expects already-expanded gains
- therefore these should be handled in the mapping layer, not inside the
  `RateControl` core wrapper

## Hakoniwa To PX4 Mapping

### Direct mapping

The following Hakoniwa native parameters can be mapped directly.

| Hakoniwa | PX4 |
|---|---|
| `PID_ROLL_RATE_Kp` | `MC_ROLLRATE_P` |
| `PID_ROLL_RATE_Ki` | `MC_ROLLRATE_I` |
| `PID_ROLL_RATE_Kd` | `MC_ROLLRATE_D` |
| `PID_PITCH_RATE_Kp` | `MC_PITCHRATE_P` |
| `PID_PITCH_RATE_Ki` | `MC_PITCHRATE_I` |
| `PID_PITCH_RATE_Kd` | `MC_PITCHRATE_D` |
| `PID_YAW_RATE_Kp` | `MC_YAWRATE_P` |
| `PID_YAW_RATE_Ki` | `MC_YAWRATE_I` |
| `PID_YAW_RATE_Kd` | `MC_YAWRATE_D` |

### Do not auto-map

The following Hakoniwa parameters must not be treated as direct rate-control
equivalents.

| Hakoniwa | PX4 candidate | Status |
|---|---|---|
| `PID_ROLL_TORQUE_MAX` | `MC_RR_INT_LIM` | do not auto-map |
| `PID_PITCH_TORQUE_MAX` | `MC_PR_INT_LIM` | do not auto-map |
| `PID_YAW_TORQUE_MAX` | `MC_YR_INT_LIM` | do not auto-map |

Reason:

- Hakoniwa meaning: output torque limit
- PX4 meaning: integrator limit
- these are not equivalent quantities

If a migration tool wants to fill integrator limits automatically, it must do so
as an explicit heuristic, not as a semantic 1:1 mapping.

## PX4 Extra Parameters

The following values should be handled as PX4-specific extras for the rate
backend.

- `MC_ROLLRATE_FF`
- `MC_PITCHRATE_FF`
- `MC_YAWRATE_FF`
- `MC_ROLLRATE_K`
- `MC_PITCHRATE_K`
- `MC_YAWRATE_K`
- `MC_RR_INT_LIM`
- `MC_PR_INT_LIM`
- `MC_YR_INT_LIM`

This means:

- they may exist even when Hakoniwa-native parameters do not
- they should be stored in PX4-side config, not forced into Hakoniwa-native naming

## Current Mapping Policy

The current practical policy for `Px4RateControlBackend` is:

1. direct PID gains may be mapped from Hakoniwa-native tuning files
2. feed-forward gains are PX4 extras
3. integrator limits are PX4 extras
4. gain-composition parameters `MC_*RATE_K` are PX4 extras

This is the safest public contract for the current stage.

## Converter Rule

The converter that writes `px4-controller-config.json` should:

- accept direct rate PID mappings from Hakoniwa-native parameters
- merge PX4 extra parameters explicitly
- reject implicit torque-limit to integrator-limit conversion by default
- apply `MC_*RATE_K` only if an explicit expansion option is enabled

The C++ runtime is not responsible for this conversion logic.
