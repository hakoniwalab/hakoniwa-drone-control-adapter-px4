#include <cmath>
#include <cstdlib>
#include <iostream>

#include "hakoniwa/drone/control_adapter/px4_attitude_control_backend.hpp"
#include "hakoniwa/drone/control_adapter/px4_controller_config_loader.hpp"

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

AttitudeQuaternion pure_yaw(double yaw_rad)
{
    return AttitudeQuaternion{
        std::cos(yaw_rad / 2.0),
        0.0,
        0.0,
        std::sin(yaw_rad / 2.0)
    };
}

}  // namespace

int main()
{
    Px4ControllerConfigLoader loader;
    const Px4ControllerConfig config =
        loader.load_from_file("../config/px4-controller-config.sample.json");

    Px4AttitudeControlBackend backend(config.attitude_control);

    const double yaw_sp = 0.1;
    const AngularRateTarget rate_target = backend.run(AttitudeControlInput{
        AttitudeQuaternion{},
        pure_yaw(yaw_sp),
        0.0
    });

    require(nearly_equal(rate_target.p, 0.0), "unexpected config-only roll rate");
    require(nearly_equal(rate_target.q, 0.0), "unexpected config-only pitch rate");
    require(nearly_equal(rate_target.r, yaw_sp * config.attitude_control.proportional_gains.yaw, 1e-4),
        "unexpected config-only yaw rate");

    return EXIT_SUCCESS;
}
