# Task

## Purpose

This file tracks implementation progress for `hakoniwa-drone-control-adapter-px4`.

It is intentionally lightweight and task-oriented.
Architecture rationale and interface policy belong in `docs/`.

## Current Status

- PX4 source is managed as a submodule
- runtime-facing source of truth is `px4-controller-config.json`
- rate-control path is established end-to-end
- attitude-control path is established through backend, converter, loader, and config-only smoke
- altitude-control path is established through backend, converter, loader, and config-only smoke
- horizontal-control path is established through backend, converter, loader, and config-only smoke
- control-allocation path is established through backend, converter, loader, and config-only smoke

## Done

- [x] Add PX4-Autopilot as submodule under `thirdparty/PX4-Autopilot`
- [x] Remove unmanaged workspace copy of PX4 source
- [x] Add `Px4RateControlBackend`
- [x] Add smoke test for direct `Px4RateControlBackend` execution
- [x] Define `px4-controller-config.json` as runtime-facing source of truth
- [x] Add public documentation for rate-control parameter handling
- [x] Add Python converter:
  - [x] Hakoniwa controller `txt` + PX4 extra JSON
  - [x] output `px4-controller-config.json`
- [x] Add sample PX4 controller extra JSON
- [x] Add C++ loader for `px4-controller-config.json`
- [x] Add config-only end-to-end smoke test
- [x] Add reusable control porting workflow document
- [x] Remove C++ parameter-mapping responsibility from runtime side

## In Progress

- [ ] Prepare the next control layer using the same workflow

## Next

### Attitude Control

- [x] Investigate PX4 `AttitudeControl` core API
- [x] Investigate Hakoniwa-side equivalents:
  - [x] `DroneAngleController::run_angle(...)`
  - [x] `DroneHeadingController`
- [x] Write consistency notes between Hakoniwa and PX4 attitude semantics
- [x] Define `IAttitudeControlBackend`
- [x] Add `Px4AttitudeControlBackend`
- [x] Add attitude-control section to `px4-controller-config.json`
- [x] Add Python converter support for attitude control
- [x] Add C++ loader support for attitude control
- [x] Add backend smoke test
- [x] Add config-only smoke test

### Runtime Direction

- [ ] Decide minimum generic runtime scope for adapter-side testing
- [ ] Decide whether rate + attitude can share one small executable harness

### Altitude Control

- [x] Investigate PX4 `PositionControl` core z-axis usage
- [x] Investigate Hakoniwa-side `DroneAltController`
- [x] Write consistency notes between Hakoniwa and PX4 vertical-thrust semantics
- [x] Define `IAltitudeControlBackend`
- [x] Add `Px4AltitudeControlBackend`
- [x] Add altitude-control section to `px4-controller-config.json`
- [x] Add Python converter support for altitude control
- [x] Add C++ loader support for altitude control
- [x] Add backend smoke test
- [x] Add config-only smoke test

### Position Control Boundary

- [x] Investigate PX4 `PositionControl` public boundary vs internal cascaded loops
- [x] Record consistency notes for horizontal/vertical split risk
- [x] Decide whether the next public adapter should be:
  - [ ] one combined PX4-first position-control backend
  - [x] or separate Hakoniwa-shaped horizontal/vertical layer backends
- [x] Revisit provisional horizontal interface after the boundary decision

### Horizontal Control

- [x] Define `IHorizontalPositionControlBackend`
- [x] Add `Px4HorizontalPositionControlBackend`
- [x] Add horizontal-control section to `px4-controller-config.json`
- [x] Add Python converter support for horizontal control
- [x] Add C++ loader support for horizontal control
- [x] Add backend smoke test
- [x] Add config-only smoke test

### Control Allocation

- [x] Investigate Hakoniwa `AircraftMixer` vs PX4 allocator boundary
- [x] Record control-allocation investigation note
- [x] Define `IControlAllocationBackend`
- [x] Decide first public geometry/effectiveness contract
- [x] Decide whether allocation status is part of the first public interface
- [x] Add first PX4 allocator backend around `ControlAllocationPseudoInverse`
- [x] Add config section, loader support, and smoke test
- [x] Decide whether allocator-feedback interpretation needs its own public interface
- [x] Document PX4 `unallocated_torque -> rate saturation` feedback path separately from allocator core
- [x] Add PX4-compatible sign-based allocation feedback policy example

### Usage Samples

- [x] Add adapter-side shared usage test skeleton
- [x] Add PX4 concrete usage test runner
- [x] Add adapter-side shared frequency usage test skeleton
- [x] Add PX4 concrete frequency usage test runner
- [x] Document adapter-side skeleton / backend-side runner split
- [ ] Expand usage examples further after interface stabilization
- [x] Cover the Hakoniwa PID autotune flow order:
  - [x] hover bootstrap with rate + attitude control
  - [x] attitude-angle stabilization example
  - [x] altitude control example
  - [x] horizontal velocity control example
- [ ] Keep usage samples as final validation of interface usability, not as the first implementation step

### Config

- [ ] Refine `px4-controller-config.json` structure after more than one control layer exists
- [ ] Decide when to freeze a schema version
- [ ] Decide whether usage/frequency example fixtures should be referenced from config documentation

## Notes

- C++ runtime/backend side should depend only on `px4-controller-config.json`
- Hakoniwa-native `txt` handling belongs to the Python converter side
- The workflow in `docs/control-porting-workflow.md` is the standard path for all future control layers
