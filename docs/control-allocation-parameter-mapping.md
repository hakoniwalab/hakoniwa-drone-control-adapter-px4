# PX4 Control Allocation Parameter Mapping

## Purpose

This note records how existing Hakoniwa rotor and mixer parameters can be
converted into the first PX4 control-allocation input surface.

The main question is:

- can current Hakoniwa multirotor rotor parameters be mapped into the allocator
  geometry/effectiveness contract without introducing hidden unit mismatches

## Current Hakoniwa-Side Inputs

Relevant existing Hakoniwa-side data:

- rotor position
  - `components.thruster.rotorPositions[].position`
  - unit: `m`
- rotor rotation direction
  - `components.thruster.rotorPositions[].rotationDirection`
  - expected values: `+1` for CCW, `-1` for CW
- thrust coefficient
  - `Ct`
  - unit: `N s^2 / rad^2`
- torque coefficient
  - `Cq`
  - unit: `N m s^2 / rad^2`
- electrical/motor constants
  - `K`, `R`, `V_bat`

## First PX4 Allocator-Side Inputs

The current allocator backend expects:

- rotor position
- rotor axis
- thrust coefficient
- moment ratio
- actuator min/max
- actuator trim
- actuator linearization point

## Direct Mapping

The following values map directly.

| Hakoniwa | PX4 allocator input | Note |
|---|---|---|
| rotor position `[x, y, z]` | `geometry.position` | same unit: `m` |
| `Ct` | `geometry.thrust_coefficient` | same unit: `N s^2 / rad^2` |
| default multirotor thrust axis | `geometry.axis = {0, 0, -1}` | PX4 rotor convention for upward lift |

## Derived Mapping

The main derived parameter is `moment_ratio`.

PX4 rotor effectiveness uses:

- `thrust = Ct * axis`
- `moment = Ct * position x axis - Ct * moment_ratio * axis`

Hakoniwa yaw anti-torque uses:

- `yaw_torque = rotationDirection * Cq * omega^2`

Under the shared downward-axis assumption `axis = {0, 0, -1}`, the yaw signs
match when:

- `moment_ratio = rotationDirection * (Cq / Ct)`

This is the first conversion rule adopted by the current helper.

## Important Unit/Sign Note

This mapping is valid under these assumptions:

- `Ct` and `Cq` use the current Hakoniwa units
- rotor axis is downward in body FRD coordinates
- `rotationDirection = +1` means CCW in Hakoniwa and yields positive yaw
  effectiveness in the current PX4 allocator sign convention

This should be treated as a documented first mapping, not as a mathematically
universal rule for every possible airframe.

## What Does Not Belong To The First Allocator Mapping

The following Hakoniwa parameters do not directly belong to the first PX4
allocator geometry conversion:

- `K`
- `R`
- `V_bat`

Reason:

- they are relevant for duty-to-rotor-speed or motor-electrical behavior
- the first allocator contract works at actuator-command/effectiveness level,
  not at electrical rotor dynamics level

## Practical Direction

For the first integration step:

- use a dedicated converter/builder at initialization time
- do not push Hakoniwa-specific naming directly into the allocator backend API
- keep the converter as an explicit, testable mapping component

## Current Helper

The current workspace now includes a helper that performs this initial
conversion:

- `compute_px4_moment_ratio_from_hakoniwa(...)`
- `compose_control_allocation_input_from_hakoniwa(...)`

These helpers are intended for initialization-time composition, not for
per-step runtime mutation.
