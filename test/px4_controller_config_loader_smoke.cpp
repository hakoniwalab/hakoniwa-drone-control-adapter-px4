#include <cmath>
#include <cstdlib>
#include <iostream>

#include "hakoniwa/drone/control_adapter/px4_controller_config_loader.hpp"

using namespace hakoniwa::drone::control_adapter;

namespace {

bool nearly_equal(double a, double b, double eps = 1e-9)
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

    require(nearly_equal(config.runtime.rate_hz, 1000.0), "unexpected rate_hz");
    require(nearly_equal(config.rate_control.gains.roll.p, 1.5), "unexpected roll p");
    require(nearly_equal(config.rate_control.gains.pitch.d, 0.02), "unexpected pitch d");
    require(nearly_equal(config.rate_control.gains.yaw.p, 0.452), "unexpected yaw p");
    require(nearly_equal(config.rate_control.feed_forward.roll, 0.0), "unexpected roll ff");
    require(nearly_equal(config.rate_control.integrator_limits.yaw_integrator, 0.2), "unexpected yaw int lim");

    std::cout << "loader smoke test passed" << std::endl;
    return EXIT_SUCCESS;
}
