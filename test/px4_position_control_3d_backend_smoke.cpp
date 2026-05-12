#include "hakoniwa/drone/control_adapter/px4_position_control_3d_backend.hpp"

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

bool finite_quaternion(const AttitudeQuaternion& q)
{
    return std::isfinite(q.w) && std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z);
}

}  // namespace

int main()
{
    Px4PositionControl3DBackendConfig config{};
    config.position_gain_xy = 1.0;
    config.position_gain_z = 1.0;
    config.velocity_p_xy = 2.5;
    config.velocity_i_xy = 0.2;
    config.velocity_d_xy = 0.0;
    config.velocity_p_z = 2.5;
    config.velocity_i_z = 0.2;
    config.velocity_d_z = 0.0;
    config.velocity_max_xy_mps = 2.0;
    config.velocity_max_up_mps = 1.0;
    config.velocity_max_down_mps = 1.0;
    config.tilt_limit_rad = 0.4;
    config.hover_thrust = 0.5;
    config.thrust_min = 0.1;
    config.thrust_max = 0.9;
    config.horizontal_thrust_margin = 0.3;
    config.decouple_horizontal_and_vertical_acceleration = false;

    Px4PositionControl3DBackend backend(config);

    const PositionControl3DState hover_state{
        {0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0},
        {},
        0.0
    };

    const PositionControl3DOutput hold = backend.run_position(
        PositionControl3DPositionInput{
            hover_state,
            {0.0, 0.0, 0.0},
            {},
            {},
            0.0,
            0.0
        },
        0.01);

    require(finite_quaternion(hold.target_attitude), "expected finite hover target attitude");
    require(std::isfinite(hold.thrust.body_z), "expected finite hover thrust");
    require(hold.thrust.body_z < 0.0, "expected hover thrust to be upward/negative body-z command");
    require(hold.debug_thrust_sp.has_value(), "expected debug thrust vector to be populated");

    backend.reset();
    const PositionControl3DOutput velocity = backend.run_velocity(
        PositionControl3DVelocityInput{
            hover_state,
            {1.0, 0.0, 0.0},
            {},
            0.0,
            0.0
        },
        0.01);

    require(finite_quaternion(velocity.target_attitude), "expected finite velocity target attitude");
    require(std::isfinite(velocity.thrust.body_z), "expected finite velocity thrust");
    require(velocity.thrust.body_z < 0.0, "expected velocity command to keep upward/negative body-z thrust");
    require(velocity.debug_thrust_sp.has_value(), "expected velocity debug thrust vector to be populated");

    return EXIT_SUCCESS;
}
