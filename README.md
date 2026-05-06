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

The initial implementation target is `IRateControlBackend`.

Current repository contents now include:

- PX4 submodule wiring
- minimal CMake build for PX4 `RateControl`
- `Px4RateControlBackend` wrapper skeleton
- converter tool for `txt + extra.json -> px4-controller-config.json`
- smoke test executable for `Px4RateControlBackend`
- local compatibility shims required to build `src/lib/rate_control`

The wrapper is intentionally narrow:

- input: `RateControlInput`
- output: `BodyTorqueCommand`
- configuration: PX4 PID gains, integrator limits, feed-forward gains
- runtime dependency: `px4-controller-config.json` only

Higher-level orchestration, scheduler policy, and mixer/control allocation integration are not included yet.

## Documents

- [PX4 Controller Config](docs/px4-controller-config.md)
- [RateControl Parameter Mapping](docs/rate-control-parameter-mapping.md)

## Converter Tool

Current tool:

- `tools/compose_px4_rate_control_config.py`

Current scope:

- rate control only
- input:
  - Hakoniwa controller `txt`
  - PX4 extra JSON
- output:
  - `px4-controller-config.json`

Converter responsibility:

- compose runtime-facing PX4 controller config
- keep Hakoniwa-native parameter handling outside the C++ runtime
- allow config-only execution paths later

## Build And Test

```sh
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure
```
