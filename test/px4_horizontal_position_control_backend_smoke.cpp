#include "hakoniwa/drone/control_adapter/px4_horizontal_position_control_backend.hpp"

#include <cmath>
#include <cstdlib>
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

bool near_zero(double value, double eps = 1e-3)
{
    return std::fabs(value) <= eps;
}

}  // namespace

int main()
{
    Px4HorizontalPositionControlBackendConfig config{};
    config.position_gain_xy = 1.0;
    config.velocity_p_xy = 2.5;
    config.velocity_i_xy = 0.2;
    config.velocity_d_xy = 0.0;
    config.velocity_max_xy_mps = 2.0;
    config.tilt_limit_rad = 0.4;
    config.hover_thrust = 0.5;
    config.thrust_min = 0.1;
    config.thrust_max = 0.9;
    config.horizontal_thrust_margin = 0.3;
    config.decouple_horizontal_and_vertical_acceleration = true;

    Px4HorizontalPositionControlBackend backend(config);

    const HorizontalTiltTarget forward_velocity = backend.run(
        HorizontalPositionControlInput{
            HorizontalControlMode::Velocity,
            {0.0, 0.0},
            {0.0, 0.0},
            {0.0, 0.0},
            0.0,
            {},
            {1.0, 0.0}
        },
        0.01);

    require(forward_velocity.pitch_rad < 0.0, "expected forward velocity to tilt nose-down");
    require(near_zero(forward_velocity.roll_rad), "expected forward velocity to keep roll near zero");

    const HorizontalTiltTarget right_velocity = backend.run(
        HorizontalPositionControlInput{
            HorizontalControlMode::Velocity,
            {0.0, 0.0},
            {0.0, 0.0},
            {0.0, 0.0},
            0.0,
            {},
            {0.0, 1.0}
        },
        0.01);

    require(right_velocity.roll_rad > 0.0, "expected +y velocity to command positive roll");

    backend.reset();
    const HorizontalTiltTarget position_hold = backend.run(
        HorizontalPositionControlInput{
            HorizontalControlMode::Position,
            {0.0, 0.0},
            {0.0, 0.0},
            {0.0, 0.0},
            0.0,
            {0.0, 0.0},
            {}
        },
        0.01);

    require(near_zero(position_hold.roll_rad), "expected zero-hold roll near zero");
    require(near_zero(position_hold.pitch_rad), "expected zero-hold pitch near zero");

    return EXIT_SUCCESS;
}
