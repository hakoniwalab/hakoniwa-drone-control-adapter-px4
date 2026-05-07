#include "hakoniwa/drone/control_adapter/px4_altitude_control_backend.hpp"

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
            {0.0},
            {0.0},
            {0.0},
            -0.7
        },
        0.01);

    require(climb.body_z < -0.1, "expected upward thrust command");
    require(climb.body_z > -0.9, "expected thrust within configured limits");

    const NormalizedVerticalThrustCommand descend = backend.run(
        AltitudeControlInput{
            {0.0},
            {0.0},
            {0.0},
            0.7
        },
        0.01);

    require(descend.body_z > climb.body_z, "expected descent command to reduce upward thrust");

    backend.reset();
    const NormalizedVerticalThrustCommand hold = backend.run(
        AltitudeControlInput{
            {0.0},
            {0.0},
            {0.0},
            0.0
        },
        0.01);

    require(hold.body_z < -0.1, "expected hover thrust after reset");
    require(hold.body_z > -0.9, "expected hover thrust within limits");

    return EXIT_SUCCESS;
}
