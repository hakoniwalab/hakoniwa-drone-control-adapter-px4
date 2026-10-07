#include <array>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <stdexcept>

#include "hakoniwa/drone/control_adapter/px4_control_allocation_backend.hpp"
#include "hakoniwa/drone/control_adapter/px4_rate_control_backend.hpp"

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
    constexpr double arm = 0.2;
    ControlAllocationInput input{};
    input.actuator_count = 4;
    input.command.thrust.body_z = -1.0;

    input.actuators[0].geometry.position = {arm, arm, 0.0};
    input.actuators[1].geometry.position = {-arm, -arm, 0.0};
    input.actuators[2].geometry.position = {arm, -arm, 0.0};
    input.actuators[3].geometry.position = {-arm, arm, 0.0};

    input.actuators[0].geometry.axis = {0.0, 0.0, -1.0};
    input.actuators[1].geometry.axis = {0.0, 0.0, -1.0};
    input.actuators[2].geometry.axis = {0.0, 0.0, -1.0};
    input.actuators[3].geometry.axis = {0.0, 0.0, -1.0};

    input.actuators[0].geometry.thrust_coefficient = 1.12e-4;
    input.actuators[1].geometry.thrust_coefficient = 1.12e-4;
    input.actuators[2].geometry.thrust_coefficient = 1.12e-4;
    input.actuators[3].geometry.thrust_coefficient = 1.12e-4;

    const double moment_ratio = 0.02;
    input.actuators[0].geometry.moment_ratio = moment_ratio;
    input.actuators[1].geometry.moment_ratio = moment_ratio;
    input.actuators[2].geometry.moment_ratio = -moment_ratio;
    input.actuators[3].geometry.moment_ratio = -moment_ratio;

    for (std::size_t i = 0; i < input.actuator_count; ++i) {
        input.actuators[i].limit = {0.0, 4.0};
    }

    return input;
}

void require_values(
    const ControlAllocationOutput& output,
    const std::array<double, 4>& expected,
    const char* message)
{
    for (std::size_t i = 0; i < expected.size(); ++i) {
        if (!nearly_equal(output.actuator_commands.values[i], expected[i])) {
            std::cerr << message << " rotor=" << i
                      << " expected=" << expected[i]
                      << " actual=" << output.actuator_commands.values[i]
                      << std::endl;
            std::exit(EXIT_FAILURE);
        }
    }
}

ControlAllocationOutput run_torque(
    double hover_thrust,
    double roll,
    double pitch,
    double yaw)
{
    Px4ControlAllocationBackendConfig config{};
    config.hover_thrust = hover_thrust;
    Px4ControlAllocationBackend backend(config);
    auto input = make_quadx_input();
    input.command.torque_x = roll;
    input.command.torque_y = pitch;
    input.command.torque_z = yaw;
    return backend.run(input);
}

double sum_outputs(const ControlAllocationOutput& output)
{
    double sum = 0.0;
    for (std::size_t i = 0; i < output.actuator_commands.count; ++i) {
        sum += output.actuator_commands.values[i];
    }
    return sum;
}

double allocated_yaw(const ControlAllocationOutput& output, double hover_thrust)
{
    return hover_thrust * 0.25 * (
        output.actuator_commands.values[0]
        + output.actuator_commands.values[1]
        - output.actuator_commands.values[2]
        - output.actuator_commands.values[3]);
}

double allocated_roll(const ControlAllocationOutput& output, double hover_thrust)
{
    const double normalized_roll_effect = 1.0 / (2.0 * std::sqrt(2.0));
    return hover_thrust * normalized_roll_effect * (
        -output.actuator_commands.values[0]
        + output.actuator_commands.values[1]
        + output.actuator_commands.values[2]
        - output.actuator_commands.values[3]);
}

void require_invalid_hover_thrust(double value)
{
    Px4ControlAllocationBackendConfig config{};
    config.hover_thrust = value;
    bool threw = false;
    try {
        Px4ControlAllocationBackend backend(config);
    } catch (const std::invalid_argument&) {
        threw = true;
    }
    require(threw, "invalid hover_thrust must be rejected by constructor");
}

}  // namespace

int main()
{
    Px4ControlAllocationBackendConfig config{};
    config.hover_thrust = 0.25;
    Px4ControlAllocationBackend backend(config);

    const ControlAllocationOutput collective = backend.run(make_quadx_input());
    require(collective.actuator_commands.count == 4, "unexpected actuator count");
    require(nearly_equal(collective.actuator_commands.values[0], 1.0), "unexpected motor0 hover output");
    require(nearly_equal(collective.actuator_commands.values[1], 1.0), "unexpected motor1 hover output");
    require(nearly_equal(collective.actuator_commands.values[2], 1.0), "unexpected motor2 hover output");
    require(nearly_equal(collective.actuator_commands.values[3], 1.0), "unexpected motor3 hover output");
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
    require(nearly_equal(climb.actuator_commands.values[0], 1.6), "unexpected motor0 climb output");
    require(nearly_equal(climb.actuator_commands.values[1], 1.6), "unexpected motor1 climb output");
    require(nearly_equal(climb.actuator_commands.values[2], 1.6), "unexpected motor2 climb output");
    require(nearly_equal(climb.actuator_commands.values[3], 1.6), "unexpected motor3 climb output");

    ControlAllocationInput limited_input = make_quadx_input();
    limited_input.command.thrust.body_z = -1.6;
    for (std::size_t i = 0; i < limited_input.actuator_count; ++i) {
        limited_input.actuators[i].limit = {0.0, 1.2};
    }
    const ControlAllocationOutput limited = backend.run(limited_input);
    require(limited.status.clipped, "normalized thrust limit should clip");
    require(nearly_equal(limited.actuator_commands.values[0], 1.2), "unexpected normalized thrust limit");

    ControlAllocationInput roll_input = make_quadx_input();
    roll_input.command.torque_x = 0.2;
    const ControlAllocationOutput roll = backend.run(roll_input);
    require(!nearly_equal(roll.actuator_commands.values[0], roll.actuator_commands.values[2]),
        "positive roll should create differential output");
    require(!nearly_equal(roll.actuator_commands.values[1], roll.actuator_commands.values[3]),
        "positive roll should create paired differential output");

    const auto roll_positive = run_torque(0.25, 0.1, 0.0, 0.0);
    const auto roll_negative = run_torque(0.25, -0.1, 0.0, 0.0);
    const auto pitch_positive = run_torque(0.25, 0.0, 0.1, 0.0);
    const auto pitch_negative = run_torque(0.25, 0.0, -0.1, 0.0);
    const auto yaw_positive = run_torque(0.25, 0.0, 0.0, 0.1);
    const auto yaw_negative = run_torque(0.25, 0.0, 0.0, -0.1);
    require_values(roll_positive, {0.7172, 1.2828, 1.2828, 0.7172},
        "unexpected positive roll allocation");
    require_values(pitch_positive, {1.2828, 0.7172, 1.2828, 0.7172},
        "unexpected positive pitch allocation");
    require_values(yaw_positive, {1.4, 1.4, 0.6, 0.6},
        "unexpected positive yaw allocation");
    for (std::size_t i = 0; i < 4; ++i) {
        require(nearly_equal(
            roll_negative.actuator_commands.values[i],
            2.0 - roll_positive.actuator_commands.values[i]),
            "roll allocation must be sign symmetric");
        require(nearly_equal(
            pitch_negative.actuator_commands.values[i],
            2.0 - pitch_positive.actuator_commands.values[i]),
            "pitch allocation must be sign symmetric");
        require(nearly_equal(
            yaw_negative.actuator_commands.values[i],
            2.0 - yaw_positive.actuator_commands.values[i]),
            "yaw allocation must be sign symmetric");
    }
    const double roll_effect =
        std::fabs(roll_positive.actuator_commands.values[0] - 1.0);
    const double pitch_effect =
        std::fabs(pitch_positive.actuator_commands.values[0] - 1.0);
    const double yaw_effect =
        std::fabs(yaw_positive.actuator_commands.values[0] - 1.0);
    require(nearly_equal(roll_effect / yaw_effect, std::sqrt(0.5)),
        "roll:yaw effectiveness must be sqrt(0.5):1");
    require(nearly_equal(pitch_effect / yaw_effect, std::sqrt(0.5)),
        "pitch:yaw effectiveness must be sqrt(0.5):1");

    const auto roll_hover_half = run_torque(0.5, 0.1, 0.0, 0.0);
    require(nearly_equal(
        std::fabs(roll_hover_half.actuator_commands.values[0] - 1.0),
        roll_effect * 0.5),
        "torque effect must be inversely proportional to hover_thrust");

    ControlAllocationInput clipped_input = make_quadx_input();
    clipped_input.command.torque_z = 1.0;
    const ControlAllocationOutput clipped = backend.run(clipped_input);
    require(clipped.status.clipped, "large yaw command should clip");
    require(nearly_equal(sum_outputs(clipped), 4.0),
        "yaw desaturation must preserve requested collective thrust");
    require(nearly_equal(
        clipped.status.unallocated_torque_z,
        clipped_input.command.torque_z - allocated_yaw(clipped, config.hover_thrust)),
        "unallocated yaw torque must equal demand minus allocation in input units");
    require(clipped.status.unallocated_torque_z > 0.0,
        "large yaw command must report unallocated yaw");

    ControlAllocationInput over_collective = make_quadx_input();
    over_collective.command.thrust.body_z = -5.0;
    const ControlAllocationOutput collective_limited = backend.run(over_collective);
    require(collective_limited.status.clipped,
        "collective above the actuator maximum must clip");
    for (std::size_t i = 0; i < collective_limited.actuator_commands.count; ++i) {
        require(nearly_equal(collective_limited.actuator_commands.values[i], 4.0),
            "collective output must stay at the public normalized-thrust limit");
    }
    require(nearly_equal(collective_limited.status.unallocated_thrust_body_z, -1.0),
        "unallocated collective must be returned in body-z input units");

    ControlAllocationInput roll_and_yaw = make_quadx_input();
    roll_and_yaw.command.torque_x = 0.1;
    roll_and_yaw.command.torque_z = 1.0;
    const ControlAllocationOutput prioritized = backend.run(roll_and_yaw);
    require(prioritized.status.clipped, "roll plus large yaw must saturate");
    require(nearly_equal(
        allocated_roll(prioritized, config.hover_thrust),
        roll_and_yaw.command.torque_x),
        "roll must remain fully allocated before yaw");
    require(nearly_equal(prioritized.status.unallocated_torque_x, 0.0),
        "priority roll must have no unallocated demand");
    require(prioritized.status.unallocated_torque_z > 0.0,
        "yaw must be the axis sacrificed by sequential desaturation");

    for (int valid_airmode = 0; valid_airmode <= 2; ++valid_airmode) {
        Px4ControlAllocationBackendConfig valid{};
        valid.hover_thrust = 0.25;
        valid.airmode = valid_airmode;
        Px4ControlAllocationBackend valid_backend(valid);
        (void)valid_backend;
    }
    for (int invalid_airmode : {-1, 3}) {
        bool threw = false;
        try {
            Px4ControlAllocationBackendConfig invalid{};
            invalid.hover_thrust = 0.25;
            invalid.airmode = invalid_airmode;
            Px4ControlAllocationBackend invalid_backend(invalid);
            (void)invalid_backend;
        } catch (const std::invalid_argument&) {
            threw = true;
        }
        require(threw, "airmode outside 0..2 must be rejected");
    }

    require_invalid_hover_thrust(0.0);
    require_invalid_hover_thrust(-0.1);
    require_invalid_hover_thrust(1.01);
    require_invalid_hover_thrust(std::numeric_limits<double>::quiet_NaN());
    bool set_config_threw = false;
    try {
        Px4ControlAllocationBackendConfig invalid{};
        invalid.hover_thrust = 0.0;
        backend.set_config(invalid);
    } catch (const std::invalid_argument&) {
        set_config_threw = true;
    }
    require(set_config_threw, "set_config must reject invalid hover_thrust");
    bool set_airmode_threw = false;
    try {
        Px4ControlAllocationBackendConfig invalid{};
        invalid.hover_thrust = 0.25;
        invalid.airmode = 3;
        backend.set_config(invalid);
    } catch (const std::invalid_argument&) {
        set_airmode_threw = true;
    }
    require(set_airmode_threw, "set_config must reject airmode outside 0..2");

    ControlAllocationInput no_actuators{};
    no_actuators.command.thrust.body_z = -1.0;
    const auto unallocated = backend.run(no_actuators);
    require(unallocated.status.clipped,
        "a wholly unallocated command must set status.clipped");
    require(nearly_equal(unallocated.status.unallocated_thrust_body_z, -1.0),
        "a wholly unallocated command must preserve body-z demand units");

    Px4RateControlBackendConfig rate_config{};
    rate_config.gains.roll.p = 0.1;
    Px4RateControlBackend rate_backend(rate_config);
    RateControlInput rate_input{};
    rate_input.target.p = 0.1;
    rate_input.dt_sec = 0.01;
    const BodyTorqueCommand rate_torque = rate_backend.run(rate_input);
    ControlAllocationInput pipeline_input = make_quadx_input();
    pipeline_input.command.torque_x = rate_torque.x;
    const auto pipeline_output = backend.run(pipeline_input);
    require(nearly_equal(
        allocated_roll(pipeline_output, config.hover_thrust),
        rate_torque.x),
        "PX4 RateControl and ControlAllocation torque units must match");

    return EXIT_SUCCESS;
}
