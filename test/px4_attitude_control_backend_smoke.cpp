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

hakoniwa::drone::control_adapter::AttitudeQuaternion from_euler(
    double roll_rad,
    double pitch_rad,
    double yaw_rad)
{
    const double cr = std::cos(roll_rad * 0.5);
    const double sr = std::sin(roll_rad * 0.5);
    const double cp = std::cos(pitch_rad * 0.5);
    const double sp = std::sin(pitch_rad * 0.5);
    const double cy = std::cos(yaw_rad * 0.5);
    const double sy = std::sin(yaw_rad * 0.5);
    return hakoniwa::drone::control_adapter::AttitudeQuaternion{
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy
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

    constexpr double roll = 0.3;
    constexpr double pitch = -0.4;
    constexpr double yaw = 0.2;
    constexpr double yaw_rate = 0.7;
    const AttitudeQuaternion tilted = from_euler(roll, pitch, yaw);
    const AngularRateTarget tilted_yaw_feedforward = backend.run(AttitudeControlInput{
        tilted,
        tilted,
        yaw_rate
    });

    if (!near(tilted_yaw_feedforward.p, -std::sin(pitch) * yaw_rate, 1e-5) ||
        !near(tilted_yaw_feedforward.q, std::sin(roll) * std::cos(pitch) * yaw_rate, 1e-5) ||
        !near(tilted_yaw_feedforward.r, std::cos(roll) * std::cos(pitch) * yaw_rate, 1e-5)) {
        std::cerr << "world yaw-rate feed-forward was not converted to body p/q/r" << std::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
