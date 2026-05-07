# PX4 Controller Config

## Purpose

This document defines the role of `px4-controller-config.json`.

This file is the single runtime-facing configuration artifact for the PX4
adapter backend.

Its role is to:

- serve as the source of truth for PX4 backend execution
- capture the exact PX4-side controller configuration used at initialization
- allow backend execution without requiring Hakoniwa-native parameter files
- provide a stable evidence artifact for replay, debugging, and regression runs

## Design Rule

The PX4 backend must depend only on `px4-controller-config.json`.

It must not depend directly on:

- Hakoniwa-native controller `txt` files
- intermediate mapped parameter sets
- PX4 extra-only configuration files

Those are upstream inputs to the config-generation process, not runtime inputs
to the backend.

## Input Modes

Two input modes are expected.

### 1. Compose mode

Inputs:

- Hakoniwa-native controller `txt`
- PX4 extra JSON

Process:

- map compatible Hakoniwa parameters into PX4 parameters
- merge them with PX4-specific extras
- write `px4-controller-config.json`

Then:

- the backend starts from `px4-controller-config.json`

### 2. Config-only mode

Input:

- `px4-controller-config.json`

Then:

- the backend starts directly from this file

This mode is intended for:

- reproducible execution
- generic runtime tests
- regression checks
- distribution of known PX4 controller settings

## Why This File Exists

If runtime execution depends on multiple inputs simultaneously, it becomes hard
to answer:

- what exact PX4 parameter set was used
- whether a failure came from mapping or from backend execution
- how to replay the same run without the original Hakoniwa parameter context

Using a single runtime-facing config solves this.

## Recommended File Shape

At minimum, the config file should be allowed to contain:

- schema version
- PX4 source version metadata
- runtime configuration
- controller parameter sections

Example high-level shape:

```json
{
  "schema_version": 1,
  "px4_version": {
    "git_commit": "a1726d316a941af9524f6279eb293a713d8fdcac",
    "git_describe": "v1.17.0-alpha1-1702-ga1726d316a"
  },
  "runtime": {
    "altitude_hz": 250.0,
    "attitude_hz": 250.0,
    "rate_hz": 250.0
  },
  "altitude_control": {
    "parameters": {
      "MPC_Z_P": 10.0,
      "MPC_Z_VEL_P_ACC": 15.0,
      "MPC_Z_VEL_I_ACC": 0.0,
      "MPC_Z_VEL_D_ACC": 10.0,
      "MPC_Z_VEL_MAX_UP": 10.0,
      "MPC_Z_VEL_MAX_DN": 10.0,
      "MPC_THR_HOVER": 0.5,
      "MPC_THR_MIN": 0.1,
      "MPC_THR_MAX": 0.9
    }
  },
  "attitude_control": {
    "parameters": {
      "MC_ROLL_P": 2.5,
      "MC_PITCH_P": 2.5,
      "MC_YAW_P": 0.1,
      "MC_YAW_WEIGHT": 0.4,
      "MC_ROLLRATE_MAX": 188.49,
      "MC_PITCHRATE_MAX": 188.49,
      "MC_YAWRATE_MAX": 18.84
    }
  },
  "rate_control": {
    "parameters": {
      "MC_ROLLRATE_P": 0.1,
      "MC_ROLLRATE_I": 0.01,
      "MC_ROLLRATE_D": 0.001,
      "MC_ROLLRATE_FF": 0.0,
      "MC_RR_INT_LIM": 0.3
    }
  }
}
```

The exact schema can evolve, but the runtime dependency rule should not.

## Relationship To Mapping

The mapping pipeline may produce intermediate results such as:

- `mapped`
- `extra`

These are useful for tooling, inspection, and debugging.

However:

- they are not runtime inputs
- they are not the backend source of truth

The backend source of truth is only:

- `px4-controller-config.json`

## Current Stage

At the current implementation stage:

- `Px4AltitudeControlBackend` already has a typed config surface
- `Px4RateControlBackend` already has a typed config surface
- `Px4AttitudeControlBackend` already has a typed config surface
- the loader reads altitude, attitude, and rate control sections
- the Python converter emits altitude, attitude, and rate sections

## Practical Direction

The intended long-term flow is:

1. Hakoniwa `txt`
2. PX4 extra JSON
3. converter tool
4. `px4-controller-config.json`
5. PX4 backend startup

This keeps runtime logic clean and makes backend execution testable without
Hakoniwa.
