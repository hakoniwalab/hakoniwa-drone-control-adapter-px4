# Position Control Layer Investigation

## Purpose

This note captures the investigation result for how PX4 `PositionControl`
should be reflected into public adapter interfaces.

The immediate question is not just how to add horizontal control, but whether
vertical and horizontal position/velocity layers can be exposed as independent
backends without contradicting the natural PX4 core boundary.

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

Important public-shape facts:

- PX4 exposes `PositionControl` as one combined 3-axis controller
- the public execution boundary is one `update(dt)` call
- intermediate loops are not exposed as public API
- output is not roll/pitch directly
- output is local-position setpoint, acceleration/thrust setpoint, and
  attitude setpoint derived from thrust plus yaw

## Internal Structure

Internally, `PositionControl` is a cascade:

1. `_positionControl()`
2. `_velocityControl(dt)`
3. `_accelerationControl()`

Semantically this means:

- position error contributes to velocity setpoint
- velocity error contributes to acceleration/thrust
- thrust plus yaw is converted into attitude

However, these stages are private implementation detail, not public PX4
adapter boundaries.

## Important PX4 Semantics

### Setpoint completeness

`PositionControl::_inputValid()` requires each axis to have at least one of:

- position setpoint
- velocity setpoint
- acceleration setpoint

The controller is designed around selective control through `NAN` setpoints.

This is important because it means:

- position mode and velocity mode are represented through setpoint shape
- PX4 does not expose separate public classes for horizontal position loop,
  horizontal velocity loop, vertical position loop, or vertical velocity loop

### Horizontal/vertical coupling

Horizontal and vertical control are not independent at output level.

Examples:

- horizontal thrust saturation depends on available total thrust
- vertical thrust is prioritized while keeping horizontal margin
- tilt limiting changes how horizontal acceleration becomes attitude
- `MPC_THR_XY_MARG`, `MPC_TILTMAX_AIR`, and `MPC_ACC_DECOUPLE` affect
  horizontal and vertical behavior together

This means:

- splitting horizontal and vertical execution into completely separate PX4
  wrappers is not naturally aligned with the core implementation

### Public output shape

The public outputs that can be read after `update(dt)` are:

- `vehicle_local_position_setpoint_s`
  - executed position setpoint
  - executed velocity setpoint
  - acceleration setpoint
  - thrust vector
- `vehicle_attitude_setpoint_s`
  - quaternion attitude setpoint
  - yaw move rate

Notably:

- there is no direct public roll/pitch output
- roll/pitch is implicit in the attitude/thrust result
- there is no public "target horizontal velocity only" or
  "target vertical speed only" output API

## Hakoniwa-Side Equivalent

Closest existing native components:

- `src/controller/impl/drone_alt_controller.hpp`
- `src/controller/impl/drone_pos_controller.hpp`

Current native structure is more explicitly layered:

- vertical position -> vertical speed -> thrust
- horizontal position -> horizontal velocity -> roll/pitch

This native split is useful for orchestration and RC-style operation, but it
does not map 1:1 to PX4 public core boundaries.

## Key Consistency Question

The main design question is:

- should public adapter interfaces mirror Hakoniwa native layers
- or should they mirror the natural PX4 public execution boundary

For `RateControl` and `AttitudeControl`, these are almost the same.

For `PositionControl`, they are not.

## Findings

### Finding 1

`IAltitudeControlBackend` and a future horizontal backend can be implemented as
thin wrappers only if they accept that the wrapper internally drives the full
`PositionControl` object with partial setpoints.

Examples:

- z-only backend:
  - x/y setpoints become `NAN`
  - only z-related output is read
- xy-only backend:
  - z setpoint still needs a valid control path
  - attitude/thrust interpretation still depends on shared state and limits

This is workable, but it is not a true public split in PX4 itself.

### Finding 2

A backend whose output is "target roll/pitch" is not a direct PX4 public-core
output.

PX4 naturally outputs:

- thrust vector
- attitude quaternion

If roll/pitch are returned, that conversion policy belongs to the wrapper.

Therefore:

- a roll/pitch backend output is a Hakoniwa-oriented derived contract,
  not a direct PX4-core contract

### Finding 3

A backend whose output is "target vertical speed" or "target horizontal
velocity" is also not a direct PX4 public-core output.

Those values exist as internal loop products, but are not exposed as public
results of the PX4 class.

Therefore:

- exposing them as backend outputs would either require
  - duplicating internal loop logic outside PX4, or
  - modifying / forking PX4 source, or
  - treating private implementation detail as public contract

All of these weaken the current "thin wrapper around upstream PX4 core" policy.

### Finding 4

Position-mode and velocity-mode support in PX4 should be viewed first as
different setpoint shapes on one controller, not as separate PX4 public
controller classes.

Practically:

- position mode:
  - provide position setpoints
  - optionally provide velocity feed-forward
- velocity mode:
  - provide velocity setpoints
  - leave position setpoints `NAN`

This is a strong argument for defining one combined position-control-oriented
public contract before splitting into finer layers.

## Interface Direction Implication

At the current stage, the most PX4-consistent public direction is:

- one backend around PX4 `PositionControl`
- input:
  - current xyz position
  - current xyz velocity
  - current xyz acceleration
  - yaw
  - position setpoints and/or velocity setpoints
  - optional acceleration feed-forward
- output:
  - thrust vector and/or attitude setpoint
  - possibly executed local-position setpoint for inspection

Less PX4-consistent directions are:

- separate public vertical-position backend returning target vertical speed
- separate public horizontal-position backend returning target roll/pitch
- separate public horizontal-velocity backend returning roll/pitch

These may still be useful as Hakoniwa-facing abstractions later, but they are
not the natural first wrapper around PX4 core.

## Decision Taken In This Repository

The current repository chose Hakoniwa-shaped public sublayers for the first
usable adapter surface:

- `IAltitudeControlBackend`
- `IHorizontalPositionControlBackend`

with these additional rules:

- both backends are mode-aware
- both backends still drive one PX4 `PositionControl` object internally
- horizontal output is documented as a derived Hakoniwa-facing tilt contract,
  not a direct PX4 public-core output

This is intentionally a compromise:

- less PX4-natural than one combined position backend
- more immediately usable for existing Hakoniwa control and autotune flow

## Recommended Near-Term Decision

Before finalizing new public interfaces, do not assume that:

- horizontal should return roll/pitch
- altitude should be split into altitude and vertical-speed backends

Instead, first decide whether the public adapter layer prioritizes:

1. PX4-first wrapper naturalness
2. Hakoniwa-native layer symmetry

If priority is PX4-first, the next interface should likely be a combined
position-control backend rather than separate horizontal/vertical sublayers.

If priority is Hakoniwa-layer symmetry, the documentation must explicitly say
that these interfaces are derived adapter contracts, not direct PX4 public-core
contracts.

## Practical Recommendation For This Repository

Recommended immediate path:

1. keep current rate / attitude wrappers as they are
2. treat `PositionControl` as a special case
3. document explicitly that horizontal/vertical wrappers are derived adapter
   contracts
4. keep config, converter, loader, and smoke tests aligned with that decision

This keeps the repository aligned with its current policy:

- thin wrapper around upstream PX4 core
- minimal semantic invention inside the C++ runtime
- explicit documentation when the adapter contract is more Hakoniwa-shaped
  than PX4-shaped
