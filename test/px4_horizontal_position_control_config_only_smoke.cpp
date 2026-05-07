#include <cmath>
#include <cstdlib>
#include <iostream>

#include "hakoniwa/drone/control_adapter/px4_controller_config_loader.hpp"
#include "hakoniwa/drone/control_adapter/px4_horizontal_position_control_backend.hpp"

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
    Px4ControllerConfigLoader loader;
    const Px4ControllerConfig config =
        loader.load_from_file("../config/px4-controller-config.sample.json");

    Px4HorizontalPositionControlBackend backend(config.horizontal_control);

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
        1.0 / config.runtime.horizontal_hz);

    require(forward_velocity.pitch_rad < 0.0, "unexpected config-only forward pitch");
    require(std::fabs(forward_velocity.roll_rad) < 1e-3, "unexpected config-only forward roll");

    return EXIT_SUCCESS;
}
