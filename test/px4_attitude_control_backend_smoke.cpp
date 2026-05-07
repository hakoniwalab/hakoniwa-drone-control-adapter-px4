#include "hakoniwa/drone/control_adapter/px4_attitude_control_backend.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>

namespace {

bool near(double lhs, double rhs, double tolerance)
{
    return std::fabs(lhs - rhs) <= tolerance;
}

hakoniwa::drone::control_adapter::AttitudeQuaternion pure_yaw(double yaw_rad)
{
    return hakoniwa::drone::control_adapter::AttitudeQuaternion{
        std::cos(yaw_rad / 2.0),
        0.0,
        0.0,
        std::sin(yaw_rad / 2.0)
    };
}

}

int main()
{
    using namespace hakoniwa::drone::control_adapter;

    Px4AttitudeControlBackendConfig config{};
    config.proportional_gains.roll = 6.5;
    config.proportional_gains.pitch = 6.5;
    config.proportional_gains.yaw = 2.8;
    config.yaw_weight = 0.4;
    config.rate_limits.roll_rad_sec = 1000.0;
    config.rate_limits.pitch_rad_sec = 1000.0;
    config.rate_limits.yaw_rad_sec = 1000.0;

    Px4AttitudeControlBackend backend(config);

    const double yaw_sp = 0.1;
    const AngularRateTarget rate_target = backend.run(AttitudeControlInput{
        AttitudeQuaternion{},
        pure_yaw(yaw_sp),
        0.0
    });

    if (!near(rate_target.p, 0.0, 1e-6)) {
        std::cerr << "unexpected roll rate target: " << rate_target.p << std::endl;
        return EXIT_FAILURE;
    }

    if (!near(rate_target.q, 0.0, 1e-6)) {
        std::cerr << "unexpected pitch rate target: " << rate_target.q << std::endl;
        return EXIT_FAILURE;
    }

    if (!near(rate_target.r, yaw_sp * config.proportional_gains.yaw, 1e-4)) {
        std::cerr << "unexpected yaw rate target: " << rate_target.r << std::endl;
        return EXIT_FAILURE;
    }

    backend.reset();
    const AngularRateTarget zero_target = backend.run(AttitudeControlInput{
        AttitudeQuaternion{},
        AttitudeQuaternion{},
        0.0
    });

    if (!near(zero_target.p, 0.0, 1e-6) ||
        !near(zero_target.q, 0.0, 1e-6) ||
        !near(zero_target.r, 0.0, 1e-6)) {
        std::cerr << "unexpected zero reset output" << std::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
