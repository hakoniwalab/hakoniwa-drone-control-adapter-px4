#include <cstdlib>
#include <iostream>

#include "hakoniwa/drone/control_adapter/px4_altitude_control_backend.hpp"
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

}  // namespace

int main()
{
    Px4ControllerConfigLoader loader;
    const Px4ControllerConfig config =
        loader.load_from_file("../config/px4-controller-config.sample.json");

    Px4AltitudeControlBackend backend(config.altitude_control);

    const NormalizedVerticalThrustCommand climb = backend.run(
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
        1.0 / config.runtime.altitude_hz);

    require(climb.body_z < -0.1, "unexpected config-only climb thrust");
    require(climb.body_z > -0.9, "unexpected config-only thrust limit");

    return EXIT_SUCCESS;
}
