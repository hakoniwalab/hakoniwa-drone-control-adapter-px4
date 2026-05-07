# Control Porting Workflow

## Purpose

This document defines the standard workflow for bringing a PX4 control layer
into `hakoniwa-drone-control-adapter-px4`.

The same pattern is expected to be reused for:

- rate control
- attitude control
- position control
- control allocation

The goal is to keep implementation work repeatable and verifiable.

## Scope

This workflow covers the full path from investigation to executable validation.

It does not define:

- Hakoniwa-side orchestration policy
- final runtime integration inside `hakoniwa-drone-pro`
- tuning methodology itself

## Workflow Overview

Each control layer should be implemented with the same sequence:

1. investigate PX4 core
2. investigate Hakoniwa-side equivalent
3. design adapter interface
4. define parameter/config contract
5. implement backend wrapper
6. implement converter/tooling
7. implement config loader
8. add smoke tests
9. perform consistency check against Hakoniwa-side semantics
10. document the result

## 1. Investigate PX4 Core

Start by identifying the exact PX4 core class to reuse.

Examples:

- `RateControl`
- `AttitudeControl`
- `PositionControl`
- `ControlAllocation*`

Collect the following:

- public API
- required inputs
- outputs
- internal assumptions
- required parameters
- optional parameters
- module-side logic that is not part of the core

The main question is:

- what does the pure PX4 core actually need if extracted from PX4 runtime?

## 2. Investigate Hakoniwa-Side Equivalent

Identify the existing Hakoniwa control logic that is closest in responsibility.

Examples:

- `DroneAngleController::run_rate(...)`
- `DroneAngleController::run_angle(...)`
- `DroneHeadingController`
- `DroneAltController`
- `DronePosController`
- `AircraftMixer`

Collect the following:

- current inputs
- current outputs
- current update-cycle behavior
- current parameter names
- current coordinate/frame assumptions

The main question is:

- what semantic role already exists on the Hakoniwa side?

## 3. Design Adapter Interface

Design the interface from the PX4 side first.

Rule:

- the interface should be natural for PX4 core usage
- Hakoniwa-native behavior should adapt to that interface, not the reverse

Examples:

- `IRateControlBackend`
- `IAttitudeControlBackend`
- `IPositionControlBackend`
- `IControlAllocationBackend`

The interface should be:

- narrow
- one-step oriented
- explicit about inputs and outputs
- independent from Hakoniwa `txt` files

## 4. Define Parameter And Config Contract

Separate the parameter/config problem into 3 layers.

### 4.1 Hakoniwa-native parameters

- current Hakoniwa `txt`
- input format used by existing tuning flows

### 4.2 PX4 extra parameters

- PX4-specific values not expressible by direct Hakoniwa mapping

### 4.3 `px4-controller-config.json`

- runtime-facing source of truth
- only input consumed by the C++ backend side

Rule:

- runtime must depend only on `px4-controller-config.json`
- conversion from `txt` must happen outside runtime

## 5. Implement Backend Wrapper

Add the narrow C++ backend wrapper around PX4 core.

Examples:

- `Px4RateControlBackend`
- `Px4AttitudeControlBackend`

The wrapper should:

- accept typed C++ input
- call PX4 core directly
- return typed C++ output
- avoid Hakoniwa-specific parsing logic

## 6. Implement Converter Tooling

Converter tooling belongs outside the runtime.

Current direction:

- converter implemented in Python
- input:
  - Hakoniwa `txt`
  - PX4 extra JSON
- output:
  - `px4-controller-config.json`

Rule:

- conversion logic should be easy to inspect and evolve
- runtime should not own mapping responsibility

## 7. Implement Config Loader

The C++ runtime side must provide a loader for `px4-controller-config.json`.

The loader should:

- read only the runtime-facing config file
- build typed config structs
- avoid dependence on intermediate conversion artifacts

At early stages, a focused loader for one control section is acceptable.

## 8. Add Smoke Tests

Each layer should get smoke tests as soon as it exists.

Recommended minimum set:

### 8.1 Backend smoke

- directly construct backend config
- call backend for one step
- verify output and reset behavior

### 8.2 Converter smoke

- use Hakoniwa `txt` + PX4 extra JSON
- generate `px4-controller-config.json`
- verify expected keys and values

### 8.3 Loader smoke

- read sample `px4-controller-config.json`
- verify typed config values

### 8.4 Config-only end-to-end smoke

- load config
- initialize backend
- run one step

This proves that config-only execution works.

## 9. Perform Consistency Check

Implementation correctness is not only about compiling and running.

Each control layer must also be checked for consistency against Hakoniwa-side
control semantics.

Typical checks:

- coordinate/frame sign conventions
- unit conventions
- target meaning
- cycle/update interpretation
- output meaning
- parameter meaning mismatches

Examples of known mismatch categories:

- Hakoniwa output torque limit vs PX4 integrator limit
- Hakoniwa target yaw update logic vs PX4 yaw-weight / yaw-rate semantics
- native cycle-hold behavior vs PX4 one-step update behavior

These must be documented explicitly.

## 10. Document The Result

Each completed control layer should leave behind public documentation for:

- parameter contract
- runtime config usage
- conversion/tooling usage
- tests available
- known semantic mismatches

This repository should remain understandable without reading the full
`hakoniwa-drone-pro` design history.

## Deliverables Per Control Layer

For each control layer, the target deliverables are:

- adapter interface
- PX4 backend wrapper
- parameter mapping document
- Python converter support
- `px4-controller-config.json` section support
- C++ config loader support
- smoke tests
- consistency notes

## Current Status

This workflow has already been exercised for `rate control`.

Implemented so far:

- `IRateControlBackend`
- `Px4RateControlBackend`
- rate-control extra sample JSON
- Python converter for rate control
- C++ loader for rate-control config
- backend smoke
- converter smoke
- loader smoke
- config-only end-to-end smoke

This rate-control path should be used as the reference pattern for the next
control layers.
