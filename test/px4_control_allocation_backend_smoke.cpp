#include <cmath>
#include <cstdlib>
#include <iostream>

#include "hakoniwa/drone/control_adapter/px4_control_allocation_backend.hpp"

using namespace hakoniwa::drone::control_adapter;

namespace {

void require(bool condition, const char* message)
{
    if (!condition) {
        std::cerr << message << std::endl;
        std::exit(EXIT_FAILURE);
    }
}

bool nearly_equal(double a, double b, double eps = 1e-3)
{
    return std::fabs(a - b) <= eps;
}

ControlAllocationInput make_quadx_input()
{
    ControlAllocationInput input{};
    input.actuator_count = 4;
    input.command.thrust.body_z = -0.5;

    input.actuators[0].geometry.position = {1.0, 1.0, 0.0};
    input.actuators[1].geometry.position = {-1.0, 1.0, 0.0};
    input.actuators[2].geometry.position = {-1.0, -1.0, 0.0};
    input.actuators[3].geometry.position = {1.0, -1.0, 0.0};

    input.actuators[0].geometry.axis = {0.0, 0.0, -1.0};
    input.actuators[1].geometry.axis = {0.0, 0.0, -1.0};
    input.actuators[2].geometry.axis = {0.0, 0.0, -1.0};
    input.actuators[3].geometry.axis = {0.0, 0.0, -1.0};

    input.actuators[0].geometry.thrust_coefficient = 1.0;
    input.actuators[1].geometry.thrust_coefficient = 1.0;
    input.actuators[2].geometry.thrust_coefficient = 1.0;
    input.actuators[3].geometry.thrust_coefficient = 1.0;

    input.actuators[0].geometry.moment_ratio = 1.0;
    input.actuators[1].geometry.moment_ratio = -1.0;
    input.actuators[2].geometry.moment_ratio = 1.0;
    input.actuators[3].geometry.moment_ratio = -1.0;

    for (std::size_t i = 0; i < input.actuator_count; ++i) {
        input.actuators[i].limit = {0.0, 1.0};
    }

    return input;
}

}  // namespace

int main()
{
    Px4ControlAllocationBackend backend(Px4ControlAllocationBackendConfig{});

    const ControlAllocationOutput collective = backend.run(make_quadx_input());
    require(collective.actuator_commands.count == 4, "unexpected actuator count");
    require(nearly_equal(collective.actuator_commands.values[0], 0.5), "unexpected motor0 collective output");
    require(nearly_equal(collective.actuator_commands.values[1], 0.5), "unexpected motor1 collective output");
    require(nearly_equal(collective.actuator_commands.values[2], 0.5), "unexpected motor2 collective output");
    require(nearly_equal(collective.actuator_commands.values[3], 0.5), "unexpected motor3 collective output");
    require(!collective.status.clipped, "collective thrust should not clip");

    ControlAllocationInput roll_input = make_quadx_input();
    roll_input.command.torque_x = 1.0;
    const ControlAllocationOutput roll = backend.run(roll_input);
    require(roll.actuator_commands.values[0] < roll.actuator_commands.values[2], "positive roll should increase rear motors");
    require(roll.actuator_commands.values[1] < roll.actuator_commands.values[3], "positive roll should increase rear motors pair");

    ControlAllocationInput clipped_input = make_quadx_input();
    clipped_input.command.torque_z = 2.0;
    const ControlAllocationOutput clipped = backend.run(clipped_input);
    require(clipped.status.clipped, "large yaw command should clip");

    return EXIT_SUCCESS;
}
