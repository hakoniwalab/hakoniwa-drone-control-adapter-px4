#include "hakoniwa/drone/control_adapter/px4_altitude_control_backend.hpp"

#include <cstdlib>
#include <cmath>
#include <iostream>

using namespace hakoniwa::drone::control_adapter;

namespace {

void require(bool condition, const char* message)
{
    if (!condition) {
        std::cerr << message << std::endl;
        std::exit(EXIT_FAILURE);
    }
}

void require_near(double actual, double expected, double tolerance, const char* message)
{
    if (std::fabs(actual - expected) > tolerance) {
        std::cerr << message << " actual=" << actual << " expected=" << expected << std::endl;
        std::exit(EXIT_FAILURE);
    }
}

AttitudeQuaternion quaternion_from_euler(double roll_rad, double pitch_rad, double yaw_rad)
{
    const double cr = std::cos(roll_rad * 0.5);
    const double sr = std::sin(roll_rad * 0.5);
    const double cp = std::cos(pitch_rad * 0.5);
    const double sp = std::sin(pitch_rad * 0.5);
    const double cy = std::cos(yaw_rad * 0.5);
    const double sy = std::sin(yaw_rad * 0.5);
    return AttitudeQuaternion{
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy
    };
}

}  // namespace

int main()
{
    Px4AltitudeControlBackendConfig config{};
    config.position_gain_z = 1.0;
    config.velocity_p_z = 2.5;
    config.velocity_i_z = 0.2;
    config.velocity_d_z = 0.0;
    config.velocity_max_up_mps = 2.0;
    config.velocity_max_down_mps = 1.0;
    config.hover_thrust = 0.5;
    config.thrust_min = 0.1;
    config.thrust_max = 0.9;

    Px4AltitudeControlBackend backend(config);

    const NormalizedVerticalThrustCommand climb = backend.run(
        AltitudeControlInput{
            AltitudeControlMode::Position,
            {},
            {0.0},
            {0.0},
            {},
            {0.0},
            0.7,
            {}
        },
        0.01);

    require(climb.body_z < -0.2, "expected upward thrust command");
    require(climb.body_z > -1.8, "expected thrust within configured limits");

    const NormalizedVerticalThrustCommand descend = backend.run(
        AltitudeControlInput{
            AltitudeControlMode::Position,
            {},
            {0.0},
            {0.0},
            {},
            {0.0},
            -0.7,
            {}
        },
        0.01);

    const NormalizedVerticalThrustCommand climb_velocity = backend.run(
        AltitudeControlInput{
            AltitudeControlMode::Velocity,
            {},
            {0.0},
            {0.0},
            {},
            {0.0},
            0.0,
            {0.5}
        },
        0.01);

    require(descend.body_z > climb.body_z, "expected descent command to reduce upward thrust");
    require(climb_velocity.body_z < descend.body_z, "expected positive vz to command more upward thrust");

    backend.reset();
    const NormalizedVerticalThrustCommand hold = backend.run(
        AltitudeControlInput{
            AltitudeControlMode::Position,
            {},
            {0.0},
            {0.0},
            {},
            {0.0},
            0.0,
            {}
        },
        0.01);

    require_near(hold.body_z, -1.0, 0.05, "expected hover thrust normalized to Hakoniwa convention");

    AltitudeControlInput level_input{};
    level_input.mode = AltitudeControlMode::Velocity;
    level_input.attitude = quaternion_from_euler(0.0, 0.0, 0.0);
    level_input.target_velocity.vz = 0.3;
    AltitudeControlInput tilted_input = level_input;
    tilted_input.attitude = quaternion_from_euler(0.3, -0.4, 0.2);
    Px4AltitudeControlBackend level_backend(config);
    Px4AltitudeControlBackend tilted_backend(config);
    const auto level_output = level_backend.run(level_input, 0.01);
    const auto tilted_output = tilted_backend.run(tilted_input, 0.01);
    require_near(
        tilted_output.body_z,
        level_output.body_z,
        1e-6,
        "altitude backend must not add attitude tilt compensation");

    AltitudeControlInput stationary_input{};
    stationary_input.mode = AltitudeControlMode::Velocity;
    stationary_input.target_velocity.vz = 0.0;
    AltitudeControlInput rising_input = stationary_input;
    rising_input.velocity.vz = 0.5;
    Px4AltitudeControlBackend stationary_backend(config);
    Px4AltitudeControlBackend rising_backend(config);
    const auto stationary_output = stationary_backend.run(stationary_input, 0.01);
    const auto rising_output = rising_backend.run(rising_input, 0.01);
    require(
        rising_output.body_z > stationary_output.body_z,
        "up-positive current velocity must reduce upward thrust for a zero-velocity target");

    Px4AltitudeControlBackendConfig integral_config = config;
    integral_config.velocity_p_z = 0.0;
    integral_config.velocity_i_z = 1.0;
    integral_config.velocity_d_z = 0.0;
    Px4AltitudeControlBackend short_dt_backend(integral_config);
    Px4AltitudeControlBackend long_dt_backend(integral_config);
    AltitudeControlInput integral_input{};
    integral_input.mode = AltitudeControlMode::Velocity;
    integral_input.target_velocity.vz = 0.1;
    (void)short_dt_backend.run(integral_input, 0.01);
    (void)long_dt_backend.run(integral_input, 0.04);
    const auto short_dt_output = short_dt_backend.run(integral_input, 0.01);
    const auto long_dt_output = long_dt_backend.run(integral_input, 0.04);
    require(
        std::fabs(long_dt_output.body_z + 1.0) > std::fabs(short_dt_output.body_z + 1.0) * 3.5,
        "altitude integrator must use call dt_sec");

    return EXIT_SUCCESS;
}
