# hakoniwa-drone-control-adapter-px4
PX4-specific implementation of the Hakoniwa drone control adapter for integrating PX4-Autopilot with Hakoniwa.

## Why This Exists

This repository is the concrete PX4 backend implementation for the public
control interfaces defined in:

- `hakoniwa-drone-control-adapter`

Related simulator-side context:

- `hakoniwa-drone-core`
  - https://github.com/toppers/hakoniwa-drone-core

The purpose of this repository is not to reproduce the full PX4 runtime.

Instead, it reuses PX4 control core components behind a stable adapter
boundary so they can participate in Hakoniwa-oriented autotuning and
regression workflows without requiring a full external TCP/UDP integration path
for every tuning run.

In other words:

- `hakoniwa-drone-control-adapter` defines the public control contract
- this repository makes that contract executable with PX4 control logic

## Why It Matters

Without this style of integration, PX4-based control experiments tend to rely
on external runtime coupling, which is useful for connection testing but heavy
for large parameter sweeps.

This repository makes a different tradeoff:

- reuse PX4 controller cores directly
- keep Hakoniwa-side fast iteration workflows
- enable high-volume PID autotuning, replay, and regression runs with PX4-based
  control behavior

That is the main value of this repository.

## PX4 Source Management

This repository manages PX4 as a git submodule.

This repository also manages the public adapter contract as a git submodule.

- Submodule path: `thirdparty/hakoniwa-drone-control-adapter`
- Upstream: `https://github.com/hakoniwalab/hakoniwa-drone-control-adapter.git`

- Submodule path: `thirdparty/PX4-Autopilot`
- Upstream: `https://github.com/PX4/PX4-Autopilot.git`

Current pinned submodule revision:

- `a1726d316a941af9524f6279eb293a713d8fdcac`
- `v1.17.0-alpha1-1702-ga1726d316a`

Policy:

- Prefer using the public adapter repository through the managed submodule
  rather than a sibling workspace checkout.
- Prefer tracking upstream PX4 without local source patches.
- If Hakoniwa-specific changes become necessary, record them explicitly in this repository rather than relying on an unmanaged workspace copy.

## Build

Initialize submodules first:

```bash
git submodule update --init --recursive
```

Then use the local build helper:

```bash
bash build.bash build
bash build.bash test
bash build.bash install
```

Default install output:

- `install/lib/libhakoniwa_drone_control_adapter_px4.a`
- `install/include/...`
- `install/lib/cmake/hakoniwa_drone_control_adapter_px4/...`

To change the install destination:

```bash
INSTALL_PREFIX=/path/to/install bash build.bash install
```

## Relationship To Other Repositories

- `hakoniwa-drone-core`
  - public simulator core
  - provides the wider Hakoniwa drone simulation context
- `hakoniwa-drone-control-adapter`
  - public backend-facing control interface
  - this repository implements that interface for PX4

## Current Status

Implemented control layers:

- `IRateControlBackend`
- `IAttitudeControlBackend`
- `IAltitudeControlBackend`
- `IHorizontalPositionControlBackend`
- `IControlAllocationBackend`

Current repository contents now include:

- PX4 submodule wiring
- minimal CMake build for PX4 `RateControl`
- minimal CMake build for PX4 `AttitudeControl`
- minimal CMake build for PX4 `PositionControl`
- `Px4RateControlBackend` wrapper skeleton
- `Px4AttitudeControlBackend` wrapper
- `Px4AltitudeControlBackend` wrapper
- `Px4HorizontalPositionControlBackend` wrapper
- `Px4ControlAllocationBackend` wrapper
- `Px4ControllerConfigLoader`
- converter tool for `txt + extra.json -> px4-controller-config.json`
- config-only end-to-end smoke path
- smoke test executable for `Px4RateControlBackend`
- smoke test executable for `Px4AttitudeControlBackend`
- smoke test executable for `Px4AltitudeControlBackend`
- smoke test executable for `Px4HorizontalPositionControlBackend`
- smoke test executable for `Px4ControlAllocationBackend`
- executable usage examples built from adapter-side shared test sources
- executable frequency/scheduler usage examples built from adapter-side shared test sources
- local compatibility shims required to build `RateControl` and `PositionControl`

The wrapper is intentionally narrow:

- input: `RateControlInput`
- output: `BodyTorqueCommand`
- configuration: PX4 PID gains, integrator limits, feed-forward gains
- runtime dependency: `px4-controller-config.json` only

The attitude wrapper is also intentionally narrow:

- input: quaternion attitude, quaternion target attitude, yaw-rate feed-forward
- output: body-frame angular-rate target
- Euler-to-quaternion conversion belongs outside the backend

The altitude wrapper is intentionally narrow as well:

- input: z position, z velocity, z acceleration, mode-aware target, `dt`
- output: normalized vertical thrust
- physical thrust conversion belongs outside the backend

The horizontal wrapper is intentionally narrow too:

- input: x/y position, x/y velocity, x/y acceleration, current yaw, mode-aware target, `dt`
- output: roll/pitch tilt target derived from PX4 `PositionControl`
- RC-style velocity commands and position-hold commands are both expressed through the same backend input mode

The control-allocation wrapper is intentionally narrow as well:

- input: thrust, body torque, actuator geometry, actuator limits, trim, and linearization point
- output: actuator command array plus minimal allocation status
- PX4 `ControlAllocationPseudoInverse` is the first implementation target

Higher-level orchestration and scheduler policy are not included yet.

## Documents

- [Control Porting Workflow](docs/control-porting-workflow.md)
- [PX4 Controller Config](docs/px4-controller-config.md)
- [RateControl Parameter Mapping](docs/rate-control-parameter-mapping.md)
- [Attitude Control Investigation](docs/attitude-control-investigation.md)
- [Altitude Control Investigation](docs/altitude-control-investigation.md)
- [Control Allocation Investigation](docs/control-allocation-investigation.md)
- [Control Allocation Parameter Mapping](docs/control-allocation-parameter-mapping.md)
- [Control Allocation Feedback Investigation](docs/control-allocation-feedback-investigation.md)
- [Horizontal Control Parameter Mapping](docs/horizontal-control-parameter-mapping.md)
- [Position Control Layer Investigation](docs/position-control-layer-investigation.md)
- [Usage Test Structure](docs/usage-test-structure.md)

## Converter Tool

Current tool:

- `tools/compose_px4_rate_control_config.py`

Current scope:

- altitude control
- attitude control
- control allocation
- horizontal control
- rate control
- input:
  - Hakoniwa controller `txt`
  - PX4 extra JSON
- output:
  - `px4-controller-config.json`

Converter responsibility:

- compose runtime-facing PX4 controller config
- keep Hakoniwa-native parameter handling outside the C++ runtime
- allow config-only execution paths later

Loader responsibility:

- read `px4-controller-config.json`
- build typed C++ config for backend initialization
- avoid depending on Hakoniwa-native `txt` or PX4 extra files at runtime

Sample extra config:

- `config/px4-controller-extra.sample.json`

## Build And Test

```sh
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure
```
