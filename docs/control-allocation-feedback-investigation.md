# Control Allocation Feedback Investigation

## Purpose

This note separates allocator-core behavior from allocator-feedback behavior.

The allocator backend itself answers:

- given a control wrench and actuator geometry, what actuator command is produced

Allocator feedback answers a different question:

- how should allocation status be interpreted before the next rate-control step

## PX4 Multicopter Path

Current PX4 multicopter behavior is:

1. the allocator computes unallocated torque and thrust
2. `control_allocator_status` is published
3. multicopter rate control reads that status
4. the sign of `unallocated_torque` is converted into positive/negative
   saturation flags
5. those saturation flags are passed into rate PID anti-windup

Important nuance:

- the current multicopter rate controller does not feed the continuous
  unallocated-torque value directly into anti-windup compensation
- PX4 source carries a TODO for that future improvement

## Adapter Boundary

For this adapter design, the clean boundary is:

- allocator backend returns allocation status
- a separate feedback-policy component interprets that status
- rate control consumes only `RateControlSaturation`

This keeps:

- allocator math inside the allocator backend
- rate-control anti-windup input inside the rate backend contract
- PX4-specific interpretation logic out of Hakoniwa-side orchestration

## Why This Is Not A Backend Interface

The feedback policy is different from the controller backends.

It is not:

- a flight-control core algorithm
- a direct PX4 library wrapper

It is instead:

- an integration-policy component between allocator and rate control

That means users may want different policies even when allocator and rate
backend choices remain the same.

## First Policy

The first policy provided by this workspace is:

- PX4-compatible sign-based allocation feedback

Its rule is simple:

- `unallocated_torque > 0` -> positive saturation
- `unallocated_torque < 0` -> negative saturation
- near zero -> no saturation

## Current Decision

Current practical direction:

- keep allocator-core implementation complete as its own layer
- keep allocator-feedback policy documented and separately testable
- avoid embedding PX4 feedback interpretation directly into Hakoniwa-side
  orchestration logic
