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
    Px4ControlAllocationBackendConfig config{};
    config.vehicle_mass_kg = 0.61079;
    config.gravity_mps2 = 9.81;
    config.hover_duty = 0.120311;
    Px4ControlAllocationBackend backend(config);

    const ControlAllocationOutput collective = backend.run(make_quadx_input());
    require(collective.actuator_commands.count == 4, "unexpected actuator count");
    require(nearly_equal(collective.actuator_commands.values[0], 0.120311), "unexpected motor0 hover output");
    require(nearly_equal(collective.actuator_commands.values[1], 0.120311), "unexpected motor1 hover output");
    require(nearly_equal(collective.actuator_commands.values[2], 0.120311), "unexpected motor2 hover output");
    require(nearly_equal(collective.actuator_commands.values[3], 0.120311), "unexpected motor3 hover output");
    require(!collective.status.clipped, "collective thrust should not clip");
    require(nearly_equal(collective.status.unallocated_thrust_body_z, 0.0), "hover thrust should be fully allocated");

    ControlAllocationInput zero_input = make_quadx_input();
    zero_input.command.thrust.body_z = 0.0;
    const ControlAllocationOutput zero = backend.run(zero_input);
    require(nearly_equal(zero.actuator_commands.values[0], 0.0), "zero collective should stop motor0");
    require(nearly_equal(zero.actuator_commands.values[1], 0.0), "zero collective should stop motor1");
    require(nearly_equal(zero.actuator_commands.values[2], 0.0), "zero collective should stop motor2");
    require(nearly_equal(zero.actuator_commands.values[3], 0.0), "zero collective should stop motor3");

    ControlAllocationInput climb_input = make_quadx_input();
    climb_input.command.thrust.body_z = -1.6;
    const ControlAllocationOutput climb = backend.run(climb_input);
    require(nearly_equal(climb.actuator_commands.values[0], 1.6 * config.hover_duty), "unexpected motor0 climb output");
    require(nearly_equal(climb.actuator_commands.values[1], 1.6 * config.hover_duty), "unexpected motor1 climb output");
    require(nearly_equal(climb.actuator_commands.values[2], 1.6 * config.hover_duty), "unexpected motor2 climb output");
    require(nearly_equal(climb.actuator_commands.values[3], 1.6 * config.hover_duty), "unexpected motor3 climb output");

    ControlAllocationInput roll_input = make_quadx_input();
    roll_input.command.torque_x = 0.2;
    const ControlAllocationOutput roll = backend.run(roll_input);
    require(!nearly_equal(roll.actuator_commands.values[0], roll.actuator_commands.values[2]),
        "positive roll should create differential output");
    require(!nearly_equal(roll.actuator_commands.values[1], roll.actuator_commands.values[3]),
        "positive roll should create paired differential output");

    ControlAllocationInput clipped_input = make_quadx_input();
    clipped_input.command.torque_z = 2.0;
    const ControlAllocationOutput clipped = backend.run(clipped_input);
    require(clipped.status.clipped, "large yaw command should clip");

    return EXIT_SUCCESS;
}
