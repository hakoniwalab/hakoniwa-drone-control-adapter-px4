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
    "horizontal_hz": 250.0,
    "rate_hz": 250.0
  },
  "common": {
    "parameters": {
      "MPC_THR_HOVER": 0.5,
      "MPC_THR_MAX": 0.9,
      "MPC_THR_MIN": 0.1
    }
  },
  "position_control": {
    "parameters": {
      "MPC_ACC_DECOUPLE": 1.0,
      "MPC_THR_XY_MARG": 0.3,
      "MPC_TILTMAX_AIR": 0.2617993877991494,
      "MPC_XY_P": 6.0,
      "MPC_XY_VEL_P_ACC": 10.0,
      "MPC_XY_VEL_I_ACC": 0.0,
      "MPC_XY_VEL_D_ACC": 0.1,
      "MPC_XY_VEL_MAX": 20.0,
      "MPC_Z_P": 10.0,
      "MPC_Z_VEL_P_ACC": 15.0,
      "MPC_Z_VEL_I_ACC": 0.0,
      "MPC_Z_VEL_D_ACC": 10.0,
      "MPC_Z_VEL_MAX_UP": 10.0,
      "MPC_Z_VEL_MAX_DN": 10.0
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
  "control_allocation": {
    "parameters": {
      "CA_RPY_NORMALIZE": 1.0,
      "CA_METRIC_ALLOCATION": 0.0,
      "CA_UPDATE_NORMALIZATION_SCALE": 1.0
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

## Thrust Normalization Contract

The allocation backend does not know the plant.  Its actuator values are
**normalized rotor thrust**:

```text
u_i = T_i / T_hover_per_rotor        (u_i = 1.0: one rotor at hover thrust)
```

This applies to `ControlAllocationOutput::actuator_commands`, and to the
`limit`, `trim` and `linearization_point` of each actuator in
`ControlAllocationInput`.  `u_i` is **not** a PWM duty.  Converting it to the
plant command is the caller's responsibility; Hakoniwa Drone PRO does it in
`AdapterAircraftMixer` with its rotor model
(`pro-docs/control-link/interface-spec.md` "ControlAllocation / Mixer").

### Conversion chain

```text
PX4 altitude / position control
  thrust_body_z (PX4 units, hover = -MPC_THR_HOVER)
      |  body_z = thrust_body_z / MPC_THR_HOVER          (px4_altitude_control_backend.cpp)
      v
Control Adapter contract
  command.thrust.body_z (hover = -1.0)
      |  PX4 allocator, one internal actuator unit = one rotor at hover thrust
      v
  actuator_commands: normalized rotor thrust u_i (hover = 1.0 per rotor)
      |  caller: plant conversion (Drone PRO: u -> thrust -> omega -> PWM duty)
      v
plant actuator command
```

### Parameters

| Parameter | Section | Meaning |
|---|---|---|
| `MPC_THR_HOVER` | `common` | Hover collective thrust as a fraction of maximum thrust (PX4 native meaning) |
| `MPC_THR_MIN` / `MPC_THR_MAX` | `common` | Collective thrust limits, same unit as `MPC_THR_HOVER` |
| `CA_HOVER_DUTY` | `control_allocation` | **Obsolete.** Ignored by the loader. It scaled the output into a duty-like value, which assumed thrust linear in duty |

- `MPC_THR_HOVER` cancels out of `body_z`, so it sets the PX4 internal thrust
  unit. With the PX4 native meaning, `MPC_THR_MAX / MPC_THR_HOVER` is the
  largest collective thrust in hover units. Generate it from vehicle physics
  as `T_hover_per_rotor / T_max_per_rotor`; the Drone PRO converter does this
  (`pro-docs/control-link/px4/px4-param-convert-spec.md` section 7).
- Before 2026-10-07 the backend output `u_i * CA_HOVER_DUTY` and the caller
  used it as PWM duty. Because rotor thrust is close to quadratic in duty, a
  request of twice the hover thrust produced more than three times the hover thrust.

## Torque Normalization Contract

`ThrustTorqueCommand::torque_x/y/z` is the output of PX4 `RateControl`: a
normalized control demand in PX4 actuator units (1.0 = one rotor at maximum
thrust), not a physical torque.

The allocator runs in PX4 actuator units, as on a PX4 vehicle, so PX4
constants keep their meaning. `MPC_THR_HOVER` (= `T_hover / T_max`) converts
between PX4 units and the hover units of the public interface:

```text
allocator torque  = torque_x/y/z                        (unchanged)
allocator thrust  = body_z * MPC_THR_HOVER              (hover: -MPC_THR_HOVER)
allocator limits  = limit / trim / linearization_point * MPC_THR_HOVER
output u_i        = allocator actuator_i / MPC_THR_HOVER (hover units)
```

In hover units the torque demand therefore acts as `torque / MPC_THR_HOVER`,
the torque counterpart of `body_z = px4_thrust_z / MPC_THR_HOVER` in the
altitude backend. The loader copies `MPC_THR_HOVER` into
`Px4ControlAllocationBackendConfig::hover_thrust`; the backend rejects a value
outside `(0, 1]`.

With `CA_RPY_NORMALIZE = 1` the PX4 allocator renormalizes each torque column
of the mix matrix, so a demand of 1.0 moves each rotor by about 0.707
(roll/pitch) and 1.0 (yaw) in PX4 units on a 4-rotor X frame, as on a real
PX4 vehicle. `status.unallocated_torque_*` is returned in the input unit and
`status.unallocated_thrust_body_z` in `body_z` units.

### Saturation

The backend uses PX4's multicopter allocator,
`ControlAllocationSequentialDesaturation`. When the actuators saturate it keeps
thrust and roll/pitch and gives up yaw first (with `MC_AIRMODE = 0`), instead
of clipping each actuator independently. `MC_AIRMODE` (optional, default 0,
section `control_allocation`) selects PX4's air mode: 0 disabled, 1 roll/pitch,
2 roll/pitch/yaw. PX4 parameters are process-global, so all backends in one
process use the same value.

Because desaturation keeps the outputs inside the limits by reducing part of
the demand, `status.clipped` is true when any actuator was clipped **or** any
part of the demand stayed unallocated.

The library is built without the PX4 parameter system. `px4_stubs/` provides a
minimal `px4_platform_common/module_params.h` for this class only; PX4 sources
stay unmodified.

- Before 2026-10-07 the backend used `ControlAllocationPseudoInverse` and only
  clipped each actuator. A large yaw demand then pushed rotors to their limits
  without preserving thrust: after a rotor-fault recovery the vehicle climbed
  at the minimum thrust command.
- Before 2026-10-07 the backend divided the demand by per-axis geometry scales
  (sum of arm lengths, and sum of `|moment_ratio|` for yaw). The column
  normalization cancels those scales on the effectiveness side, so only the
  division of the demand remained and each axis gain became `1 / scale`. The
  yaw scale is an order of magnitude smaller than the roll/pitch scale, so yaw
  was far stronger than on PX4. PID gains tuned before that date do
  not carry over.

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
- `Px4ControlAllocationBackend` already has a typed config surface
- `Px4HorizontalPositionControlBackend` already has a typed config surface
- the loader reads altitude, attitude, control-allocation, horizontal, and rate control sections
- the Python converter emits altitude, attitude, control-allocation, horizontal, and rate sections

## Runtime Mode And Config Responsibility

The runtime config file contains backend parameters only.

It does not contain runtime input mode such as:

- altitude hold vs climb/descent velocity command
- horizontal position hold vs horizontal velocity command

Those are runtime input decisions expressed through the backend API on each
`run()` call.

## Practical Direction

The intended long-term flow is:

1. Hakoniwa `txt`
2. PX4 extra JSON
3. converter tool
4. `px4-controller-config.json`
5. PX4 backend startup

This keeps runtime logic clean and makes backend execution testable without
Hakoniwa.
