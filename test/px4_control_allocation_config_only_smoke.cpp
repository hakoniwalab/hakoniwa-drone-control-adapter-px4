#include <cstdlib>
#include <iostream>
#include <cmath>

#include "hakoniwa/drone/control_adapter/px4_control_allocation_backend.hpp"
#include "hakoniwa/drone/control_adapter/px4_controller_config_loader.hpp"

using namespace hakoniwa::drone::control_adapter;

namespace {

void require(bool condition, const char* message)
{
    if (!condition) {
        std::cerr << message << std::endl;
        std::exit(EXIT_FAILURE);
    }
}

ControlAllocationInput make_quadx_input()
{
    ControlAllocationInput input{};
    input.actuator_count = 4;
    input.command.thrust.body_z = -1.0;

    input.actuators[0].geometry.position = {0.18, 0.18, 0.0};
    input.actuators[1].geometry.position = {-0.18, -0.18, 0.0};
    input.actuators[2].geometry.position = {0.18, -0.18, 0.0};
    input.actuators[3].geometry.position = {-0.18, 0.18, 0.0};

    input.actuators[0].geometry.axis = {0.0, 0.0, -1.0};
    input.actuators[1].geometry.axis = {0.0, 0.0, -1.0};
    input.actuators[2].geometry.axis = {0.0, 0.0, -1.0};
    input.actuators[3].geometry.axis = {0.0, 0.0, -1.0};

    input.actuators[0].geometry.thrust_coefficient = 1.12e-4;
    input.actuators[1].geometry.thrust_coefficient = 1.12e-4;
    input.actuators[2].geometry.thrust_coefficient = 1.12e-4;
    input.actuators[3].geometry.thrust_coefficient = 1.12e-4;

    const double moment_ratio = 2.64e-6 / 1.12e-4;
    input.actuators[0].geometry.moment_ratio = moment_ratio;
    input.actuators[1].geometry.moment_ratio = moment_ratio;
    input.actuators[2].geometry.moment_ratio = -moment_ratio;
    input.actuators[3].geometry.moment_ratio = -moment_ratio;

    for (std::size_t i = 0; i < input.actuator_count; ++i) {
        input.actuators[i].limit = {0.0, 1.0};
    }

    return input;
}

}  // namespace

int main()
{
    Px4ControllerConfigLoader loader;
    const Px4ControllerConfig config =
        loader.load_from_file("../config/px4-controller-config.sample.json");

    Px4ControlAllocationBackend backend(config.control_allocation);
    const ControlAllocationOutput output = backend.run(make_quadx_input());

    require(output.actuator_commands.count == 4, "unexpected config-only actuator count");
    require(output.actuator_commands.values[0] > 0.0, "expected positive motor output");
    require(output.actuator_commands.values[1] > 0.0, "expected positive motor output");
    require(output.actuator_commands.values[2] > 0.0, "expected positive motor output");
    require(output.actuator_commands.values[3] > 0.0, "expected positive motor output");
    require(output.actuator_commands.values[0] < 0.2, "hover duty should stay below idle-throttle-like values");
    require(std::fabs(output.actuator_commands.values[0] - 0.120311) < 1e-3, "expected hover-calibrated duty");
    require(std::fabs(output.status.unallocated_thrust_body_z) < 1e-3, "expected collective thrust to allocate");

    return EXIT_SUCCESS;
}
