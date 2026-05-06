#include <cmath>
#include <cstdlib>
#include <iostream>

#include "hakoniwa/drone/control_adapter/px4_rate_control_backend.hpp"

using namespace hakoniwa::drone::control_adapter;

namespace {

bool nearly_equal(double a, double b, double eps = 1e-6)
{
    return std::fabs(a - b) <= eps;
}

}  // namespace

int main()
{
    Px4RateControlBackendConfig config{};
    config.gains.roll = {0.15, 0.10, 0.01};
    config.gains.pitch = {0.15, 0.10, 0.01};
    config.gains.yaw = {0.20, 0.05, 0.00};
    config.integrator_limits = {0.30, 0.30, 0.20};
    config.feed_forward = {0.00, 0.00, 0.00};

    Px4RateControlBackend backend(config);

    RateControlInput input{};
    input.rate = {0.10, -0.05, 0.02};
    input.angular_accel = {0.01, -0.02, 0.00};
    input.target = {0.25, 0.00, -0.03};
    input.dt_sec = 0.002;
    input.landed = false;

    const BodyTorqueCommand command = backend.run(input);
    const Px4RateControlBackendStatus status = backend.get_status();

    std::cout << "torque: ["
              << command.x << ", "
              << command.y << ", "
              << command.z << "]\n";
    std::cout << "integral: ["
              << status.roll_integral << ", "
              << status.pitch_integral << ", "
              << status.yaw_integral << "]\n";

    if (!nearly_equal(command.x, 0.0224) ||
        !nearly_equal(command.y, 0.0077) ||
        !nearly_equal(command.z, -0.01)) {
        std::cerr << "unexpected torque output" << std::endl;
        return EXIT_FAILURE;
    }

    if (!(status.roll_integral > 0.0) ||
        !(status.pitch_integral > 0.0) ||
        !(status.yaw_integral < 0.0)) {
        std::cerr << "unexpected integrator sign" << std::endl;
        return EXIT_FAILURE;
    }

    backend.reset();
    const Px4RateControlBackendStatus reset_status = backend.get_status();

    if (!nearly_equal(reset_status.roll_integral, 0.0) ||
        !nearly_equal(reset_status.pitch_integral, 0.0) ||
        !nearly_equal(reset_status.yaw_integral, 0.0)) {
        std::cerr << "reset did not clear integrator state" << std::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
