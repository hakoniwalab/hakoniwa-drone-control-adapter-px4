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
- Pinned revision: managed via this repository's submodule state

- Submodule path: `thirdparty/PX4-Autopilot`
- Upstream: `https://github.com/PX4/PX4-Autopilot.git`
- Pinned revision: `a1726d316a` (`v1.17.0-alpha1-1702-ga1726d316a`)

Current pinned submodule revision:

- `a1726d316a941af9524f6279eb293a713d8fdcac`
- `v1.17.0-alpha1-1702-ga1726d316a`

Policy:

- Prefer using the public adapter repository through the managed submodule
  rather than a sibling workspace checkout.
- Prefer tracking upstream PX4 without local source patches.
- If Hakoniwa-specific changes become necessary, record them explicitly in this repository rather than relying on an unmanaged workspace copy.

## License And Upstream Sources

This repository keeps third-party source provenance and license boundaries
explicit.

Repository-level license:

- This repository's own code is licensed under MIT.
- See [LICENSE](LICENSE).

Upstream-managed sources:

- `thirdparty/hakoniwa-drone-control-adapter`
  - upstream public adapter contract
  - used as a git submodule
  - upstream: `https://github.com/hakoniwalab/hakoniwa-drone-control-adapter.git`
  - upstream license: MIT
  - see `thirdparty/hakoniwa-drone-control-adapter/LICENSE`

- `thirdparty/PX4-Autopilot`
  - upstream PX4 control implementation source
  - used as a git submodule
  - upstream: `https://github.com/PX4/PX4-Autopilot.git`
  - pinned revision: `a1726d316a` (`v1.17.0-alpha1-1702-ga1726d316a`)
  - upstream license: BSD 3-Clause
  - see `thirdparty/PX4-Autopilot/LICENSE`

What this repository adds on top:

- adapter wrapper implementations in `src/`
- PX4 config loading and composition glue
- build integration and local compatibility shims
- tests, examples, and investigation documents

PX4 source files currently compiled by this repository:

- `thirdparty/PX4-Autopilot/src/lib/rate_control/rate_control.cpp`
- `thirdparty/PX4-Autopilot/src/modules/mc_att_control/AttitudeControl/AttitudeControl.cpp`
- `thirdparty/PX4-Autopilot/src/modules/mc_pos_control/PositionControl/ControlMath.cpp`
- `thirdparty/PX4-Autopilot/src/modules/mc_pos_control/PositionControl/PositionControl.cpp`
- `thirdparty/PX4-Autopilot/src/lib/control_allocation/control_allocation/ControlAllocation.cpp`
- `thirdparty/PX4-Autopilot/src/lib/control_allocation/control_allocation/ControlAllocationPseudoInverse.cpp`

PX4 headers and support code currently used by this repository include:

- `rate_control.hpp`
- `AttitudeControl.hpp`
- `PositionControl.hpp`
- `ControlAllocationPseudoInverse.hpp`
- PX4 matrix / mathlib / control allocation support headers required by the
  above modules

PX4-facing headers directly included from adapter-side implementation code:

- `src/px4_rate_control_backend.cpp`
  - `rate_control.hpp`
- `src/px4_attitude_control_backend.cpp`
  - `AttitudeControl.hpp`
- `src/px4_altitude_control_backend.cpp`
  - `PositionControl.hpp`
- `src/px4_horizontal_position_control_backend.cpp`
  - `PositionControl.hpp`
- `src/px4_position_control_3d_backend.cpp`
  - `PositionControl.hpp`
- `src/px4_control_allocation_backend.cpp`
  - `ControlAllocationPseudoInverse.hpp`

Local compatibility headers shipped by this repository for PX4 integration:

- `include/px4_platform_common/defines.h`
- `include/uORB/topics/control_allocator_status.h`
- `include/uORB/topics/rate_ctrl_status.h`
- `include/uORB/topics/trajectory_setpoint.h`
- `include/uORB/topics/vehicle_attitude_setpoint.h`
- `include/uORB/topics/vehicle_local_position_setpoint.h`

License grouping:

- the repository's own code in `src/`, `include/`, `tools/`, `test/`, and
  `docs/` is MIT
- the public adapter contract submodule is MIT
- the currently imported PX4 implementation files listed above all come from
  upstream PX4 and therefore all share the upstream BSD 3-Clause license

This means the PX4-derived portion used by this repository is internally
license-consistent as upstream PX4 source, while the adapter-side integration
code remains MIT.

License check status:

- The imported PX4 `.cpp` files listed above were verified as upstream PX4
  source covered by the BSD 3-Clause license, checked against the pinned
  revision `a1726d316a` (`v1.17.0-alpha1-1702-ga1726d316a`).
- the directly used PX4 headers listed above are treated as part of that same
  upstream PX4 BSD 3-Clause source set
- PX4 matrix, mathlib, and control allocation support headers transitively
  included by the above were also verified as BSD 3-Clause upstream PX4
  source.
- No GPL-licensed PX4 source files are compiled or linked by this repository.
  This was verified against the files listed above.
- the currently compiled PX4 implementation files are limited to the files
  listed above
- when the CMake build adds or removes PX4 source files, this list must be
  updated in the same change
- TODO: add a CI check that cross-validates the PX4 source file list in
  `CMakeLists.txt` against this README to enforce this policy automatically.
- the local compatibility headers under `include/px4_platform_common/` and
  `include/uORB/topics/` are repository-side shim headers, so they are not
  claimed here as upstream BSD files; they remain part of this repository's
  own MIT-licensed integration layer unless explicitly replaced by upstream
  copies later
- These headers are local compatibility shims written for build compatibility.
  They are not copied from upstream PX4 source files.

License intent:

- keep upstream PX4 source traceable as upstream PX4 source
- keep adapter-side original code traceable as this repository's MIT-licensed
  code
- avoid relying on opaque sibling-workspace copies or unrecorded local edits
- keep any future Hakoniwa-specific PX4-side changes explicit and reviewable in
  this repository

Practical interpretation:

- if you redistribute this repository, preserve this repository's MIT license
  notice
- if you redistribute PX4-derived source or binaries, also preserve the BSD
  3-Clause notice and conditions from upstream PX4
- binary redistribution should include both this repository's MIT license
  notice and the upstream PX4 BSD 3-Clause license notice
- the submodules remain governed by their own upstream licenses; this README is
  a provenance summary, not a replacement for those license texts

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
