# Control Allocation Investigation

## Purpose

This note captures the initial investigation result for the control allocation
layer.

The main question is:

- can Hakoniwa native mixer behavior and PX4 allocator behavior be wrapped by
  the same public adapter boundary

## Hakoniwa-Side Equivalent

Closest existing component:

- `src/controller/impl/mixer/aircraft_mixer.hpp`

Current public-facing shape:

- `IAircraftMixer::run(mi_aircraft_control_out_t&) -> PwmDuty`

Current native characteristics:

- rotor geometry is configured at initialization time
- inverse matrix is computed once and reused
- runtime input is effectively:
  - thrust
  - body torque x/y/z
  - mass is passed through but not central to the allocation boundary itself
- negative `omega^2` results are clamped to zero
- output is PWM duty only
- no explicit unallocated-control or saturation-status contract is returned

## PX4 Core

The PX4 side naturally splits into these layers:

1. allocation algorithm core
2. effectiveness matrix generation
3. module/runtime wiring

Most relevant allocator core classes:

- `ControlAllocation`
- `ControlAllocationPseudoInverse`
- `ControlAllocationSequentialDesaturation`

Most relevant geometry/effectiveness support:

- `ActuatorEffectiveness`
- `ActuatorEffectivenessRotors::computeEffectivenessMatrix(...)`

Important public allocator-core behavior:

- set effectiveness matrix
- set actuator min/max
- set actuator trim / linearization point
- set control setpoint
- allocate actuator setpoint
- query allocated control
- clip or normalize actuator setpoint

This is already close to an adapter-friendly core boundary.

## Immediate Similarity

The most important similarity between Hakoniwa mixer and PX4 allocator is:

- input/output boundary is conceptually the same

Practical shared boundary:

- input:
  - collective thrust
  - body torque x/y/z
  - actuator geometry or effectiveness information
  - actuator constraints
- output:
  - actuator command array

This is a much cleaner public boundary than the `PositionControl` case.

## Main Differences

### Hakoniwa native mixer

- fixed inverse-matrix based mapping
- negative actuator solution is clamped
- little explicit status feedback at the public boundary

### PX4 allocator

- effectiveness-matrix-based allocation
- actuator min/max and trim are first-class inputs
- clipping and normalization are part of the allocator behavior
- allocated control can be queried after allocation
- unallocated control can be derived
- saturation/clipping handling is part of the intended control-loop behavior

## Interface Implication

Because the input/output boundary is so similar, the public adapter interface
does not need to expose:

- Hakoniwa `IAircraftMixer`
- PX4 `ControlAllocation` directly

Instead, the adapter boundary should likely be one layer below
`IAircraftMixer`, at the pure allocation backend level.

Natural responsibility:

- thrust + body torque + geometry/limits -> actuator command

## Why `IAircraftMixer` Should Probably Stay Outside

`IAircraftMixer` is still a Hakoniwa controller-side façade concern because it
is tied to:

- `mi_aircraft_control_out_t`
- `PwmDuty`
- ownership from `IAircraftController`

These are higher-level integration details for `hakoniwa-drone-pro`.

The adapter repository should instead define the narrower backend boundary that
`IAircraftMixer` could call internally.

## Candidate Public Backend Shape

Likely public input categories:

- thrust command
- body torque command
- actuator geometry / effectiveness
- actuator min/max
- actuator trim / linearization point

Likely public output categories:

- actuator command array

Likely optional status:

- allocated control
- unallocated control
- saturation / clipping information

## First PX4 Implementation Candidate

The most realistic first implementation target is:

- `ControlAllocationPseudoInverse`

Reason:

- lightest dependency surface
- already works with effectiveness matrix + limits + trim + setpoint
- good enough for initial adapter contract validation

`ControlAllocationSequentialDesaturation` is a natural next step after that.

## Geometry Boundary

One major design question is whether the public adapter interface should accept:

1. raw rotor geometry
2. already-built effectiveness matrix

Current practical guidance:

- public adapter contract should probably prefer geometry-oriented input
- PX4 backend can translate that into effectiveness matrix
- native Hakoniwa backend can keep using its own matrix path

This keeps the public contract closer to vehicle description than backend math.

## Consistency Notes

The main semantic mismatch to document is:

- Hakoniwa native mixer is closer to a fixed mapping implementation
- PX4 allocator is closer to a constrained allocation implementation with
  richer status semantics

But this mismatch is internal.

At the public boundary level, the layer is still a good adapter candidate
because both sides fundamentally solve:

- control wrench to actuator command allocation

## Allocator Feedback Boundary

There is a second, separate design question after basic allocation:

- how allocator feedback should be returned to upstream rate control

PX4 currently does two distinct things:

1. the allocator computes and publishes continuous unallocated control
2. the multicopter rate controller converts the sign of unallocated torque
   into per-axis positive/negative saturation flags for PID anti-windup

Important nuance:

- this sign-based conversion is PX4-specific control semantics
- current PX4 multicopter rate control does not feed the continuous
  unallocated-torque value directly into anti-windup compensation
- the source currently carries an explicit TODO for that future improvement

Implication for this adapter work:

- basic allocator implementation should stop at the allocation backend
- allocator-feedback interpretation should not be silently embedded into
  Hakoniwa-side orchestration as ad hoc PX4 logic
- if feedback handling becomes part of the public adapter surface, it should
  be documented and designed as a separate interface boundary

Current practical decision:

- keep allocation status in the first allocator backend output
- defer feedback-interpretation interface design until after allocator
  implementation is complete and reviewed on the design surface

## Next Implementation Steps

1. define `IControlAllocationBackend`
2. define public common types for:
   - wrench-style input
   - actuator geometry / limits
   - actuator command output
   - optional allocation status
3. add a PX4-first backend around `ControlAllocationPseudoInverse`
4. decide whether the first public contract includes allocation status or keeps
   it as a follow-up extension
5. design allocator-feedback interpretation as a separate boundary if PX4-style
   anti-windup feedback needs to be preserved outside the allocator itself
