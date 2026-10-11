#include "hakoniwa/drone/control_adapter/px4_position_control_3d_backend.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <optional>

using namespace hakoniwa::drone::control_adapter;

namespace {

void require(bool condition, const char* message)
{
    if (!condition) {
        std::cerr << message << std::endl;
        std::exit(EXIT_FAILURE);
    }
}

bool nearly_equal(double a, double b, double eps = 1e-5)
{
    return std::fabs(a - b) <= eps;
}

double yaw_from_quaternion(const AttitudeQuaternion& q)
{
    return std::atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

Px4PositionControl3DBackendConfig make_config(bool lock)
{
    Px4PositionControl3DBackendConfig result{};
    result.position_gain_xy = 1.0;
    result.position_gain_z = 1.0;
    result.velocity_p_xy = 2.5;
    result.velocity_p_z = 2.5;
    result.velocity_max_xy_mps = 2.0;
    result.velocity_max_up_mps = 1.0;
    result.velocity_max_down_mps = 1.0;
    result.tilt_limit_rad = 0.4;
    result.hover_thrust = 0.5;
    result.thrust_min = 0.1;
    result.thrust_max = 0.9;
    result.horizontal_thrust_margin = 0.3;
    result.manual_yaw_lock = lock;
    return result;
}

PositionControl3DOutput run(
    Px4PositionControl3DBackend& backend,
    double current_yaw,
    std::optional<double> target_yaw = {},
    std::optional<double> target_rate = {})
{
    PositionControl3DVelocityInput input{};
    input.state.yaw_rad = current_yaw;
    input.target_yaw_rad = target_yaw;
    input.target_yaw_rate_rad_sec = target_rate;
    return backend.run_velocity(input, 0.01);
}

} // namespace

int main()
{
    Px4PositionControl3DBackend backend(make_config(true));
#ifdef HAKO_EKF_IMU_ACCELERATION
    require(nearly_equal(yaw_from_quaternion(run(backend, 0.3).target_attitude), 0.3),
        "lock must start at current yaw");
    require(nearly_equal(yaw_from_quaternion(run(backend, 0.9).target_attitude), 0.3),
        "zero-rate velocity control must hold initial yaw");
    require(nearly_equal(yaw_from_quaternion(run(backend, 0.9, 0.6).target_attitude), 0.6),
        "explicit yaw must override the lock");
    require(nearly_equal(yaw_from_quaternion(run(backend, 1.1).target_attitude), 1.1),
        "lock must restart after explicit yaw");
    require(nearly_equal(yaw_from_quaternion(run(backend, 1.3, {}, 0.2).target_attitude), 1.3),
        "nonzero yaw rate must release the lock");
    require(nearly_equal(yaw_from_quaternion(run(backend, 1.5).target_attitude), 1.5),
        "lock must restart after yaw-rate control");
    backend.reset();
    require(nearly_equal(yaw_from_quaternion(run(backend, 1.7).target_attitude), 1.7),
        "reset must clear the lock");
#else
    (void)run(backend, 0.3);
    require(nearly_equal(yaw_from_quaternion(run(backend, 0.9).target_attitude), 0.9),
        "OFF build must ignore manual_yaw_lock and follow current yaw");
#endif

    Px4PositionControl3DBackend legacy(make_config(false));
    (void)run(legacy, 0.2);
    require(nearly_equal(yaw_from_quaternion(run(legacy, 0.8).target_attitude), 0.8),
        "manual_yaw_lock false must follow current yaw");
    return EXIT_SUCCESS;
}
