# hakoniwa-drone-control-adapter-px4
PX4-specific implementation of the Hakoniwa drone control adapter for integrating PX4-Autopilot with Hakoniwa.

## PX4 Source Management

This repository manages PX4 as a git submodule.

- Submodule path: `thirdparty/PX4-Autopilot`
- Upstream: `https://github.com/PX4/PX4-Autopilot.git`

Current pinned submodule revision:

- `a1726d316a941af9524f6279eb293a713d8fdcac`
- `v1.17.0-alpha1-1702-ga1726d316a`

Policy:

- Prefer tracking upstream PX4 without local source patches.
- If Hakoniwa-specific changes become necessary, record them explicitly in this repository rather than relying on an unmanaged workspace copy.

## Current Status

Implemented control layers:

- `IRateControlBackend`
- `IAttitudeControlBackend`
- `IAltitudeControlBackend`

Current repository contents now include:

- PX4 submodule wiring
- minimal CMake build for PX4 `RateControl`
- minimal CMake build for PX4 `AttitudeControl`
- minimal CMake build for PX4 `PositionControl`
- `Px4RateControlBackend` wrapper skeleton
- `Px4AttitudeControlBackend` wrapper
- `Px4AltitudeControlBackend` wrapper
- `Px4ControllerConfigLoader`
- converter tool for `txt + extra.json -> px4-controller-config.json`
- config-only end-to-end smoke path
- smoke test executable for `Px4RateControlBackend`
- smoke test executable for `Px4AttitudeControlBackend`
- smoke test executable for `Px4AltitudeControlBackend`
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

- input: z position, z velocity, z acceleration, target altitude, `dt`
- output: normalized vertical thrust
- physical thrust conversion belongs outside the backend

Higher-level orchestration, scheduler policy, and mixer/control allocation integration are not included yet.

## Documents

- [Control Porting Workflow](docs/control-porting-workflow.md)
- [PX4 Controller Config](docs/px4-controller-config.md)
- [RateControl Parameter Mapping](docs/rate-control-parameter-mapping.md)
- [Attitude Control Investigation](docs/attitude-control-investigation.md)
- [Altitude Control Investigation](docs/altitude-control-investigation.md)

## Converter Tool

Current tool:

- `tools/compose_px4_rate_control_config.py`

Current scope:

- altitude control
- attitude control
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
