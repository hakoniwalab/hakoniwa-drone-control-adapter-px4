#include <cmath>
#include <cstdlib>
#include <iostream>

#include "hakoniwa/drone/control_adapter/px4_controller_config_loader.hpp"
#include "hakoniwa/drone/control_adapter/px4_rate_control_backend.hpp"

using namespace hakoniwa::drone::control_adapter;

namespace {

bool nearly_equal(double a, double b, double eps = 1e-6)
{
    return std::fabs(a - b) <= eps;
}

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

    Px4RateControlBackend backend(config.rate_control);

    RateControlInput input{};
    input.rate = {0.10, -0.05, 0.02};
    input.angular_accel = {0.01, -0.02, 0.00};
    input.target = {0.25, 0.00, -0.03};
    input.dt_sec = 1.0 / config.runtime.rate_hz;
    input.landed = false;

    const BodyTorqueCommand command = backend.run(input);

    std::cout << "torque: ["
              << command.x << ", "
              << command.y << ", "
              << command.z << "]\n";

    require(nearly_equal(command.x, 0.2248), "unexpected config-only roll torque");
    require(nearly_equal(command.y, 0.0754), "unexpected config-only pitch torque");
    require(nearly_equal(command.z, -0.0226), "unexpected config-only yaw torque");

    return EXIT_SUCCESS;
}
