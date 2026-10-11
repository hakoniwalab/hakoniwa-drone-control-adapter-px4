#include "hakoniwa/drone/control_adapter/px4_position_control_3d_backend.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <vector>

using namespace hakoniwa::drone::control_adapter;

namespace {

void require(bool condition, const char* message)
{
    if (!condition) {
        std::cerr << message << std::endl;
        std::exit(EXIT_FAILURE);
    }
}

bool nearly_equal(double a, double b, double eps = 1e-7)
{
    return std::fabs(a - b) <= eps;
}

double thrust_x(const PositionControl3DOutput& output)
{
    require(output.debug_thrust_sp.has_value(), "missing trajectory debug thrust");
    return output.debug_thrust_sp->x;
}

Px4PositionControl3DBackendConfig config(double interval_sec)
{
    Px4PositionControl3DBackendConfig value{};
    value.position_gain_xy = 1.0;
    value.position_gain_z = 1.0;
    value.velocity_p_xy = 2.5;
    value.velocity_p_z = 2.5;
    value.velocity_max_xy_mps = 5.0;
    value.velocity_max_up_mps = 1.0;
    value.velocity_max_down_mps = 1.0;
    value.tilt_limit_rad = 0.4;
    value.hover_thrust = 0.5;
    value.thrust_min = 0.1;
    value.thrust_max = 0.9;
    value.horizontal_thrust_margin = 0.3;
    value.trajectory.enabled = true;
    value.trajectory.update_interval_sec = interval_sec;
    return value;
}

PositionControl3DPositionInput target(double x = 30.0, double yaw = M_PI_2)
{
    PositionControl3DPositionInput value{};
    value.target_position = {x, 0.0, 0.0};
    value.target_yaw_rad = yaw;
    return value;
}

}  // namespace

int main()
{
#ifdef HAKO_EKF_IMU_ACCELERATION
    const auto original = target();
    Px4PositionControl3DBackend interval_backend(config(0.020));
    std::vector<PositionControl3DOutput> outputs;
    for (int i = 0; i < 9; ++i) {
        outputs.push_back(interval_backend.run_position(original, 0.008));
    }
    const auto changed = [&outputs](int lhs, int rhs) {
        return !nearly_equal(thrust_x(outputs[lhs]), thrust_x(outputs[rhs]), 1e-8);
    };
    require(!changed(0, 1) && !changed(1, 2), "20 ms interval must hold the first output for 24 ms");
    require(changed(2, 3), "trajectory must update at 24 ms");
    require(!changed(3, 4) && changed(4, 5), "next trajectory interval must be 16 ms");
    require(!changed(5, 6) && !changed(6, 7) && changed(7, 8),
        "trajectory intervals must continue with the next 24 ms interval");
    require(nearly_equal(*outputs[0].target_yaw_rate_rad_sec,
                         *outputs[1].target_yaw_rate_rad_sec, 1e-9),
        "held trajectory must keep its yaw-rate output");

    // Replay only the real update instants. The result must match if dt is elapsed since the
    // previous actual trajectory run rather than the nominal 20 ms interval.
    Px4PositionControl3DBackend elapsed_reference(config(0.0));
    auto elapsed = elapsed_reference.run_position(original, 0.008);
    for (double dt : {0.024, 0.016, 0.024}) {
        elapsed = elapsed_reference.run_position(original, dt);
    }
    require(nearly_equal(thrust_x(outputs[8]), thrust_x(elapsed), 1e-6),
        "trajectory dt must sum to elapsed time without over-counting");
    require(nearly_equal(*outputs[8].target_yaw_rate_rad_sec,
                         *elapsed.target_yaw_rate_rad_sec, 1e-6),
        "yaw trajectory dt must sum to elapsed time without over-counting");

    // The first call always updates, while a subsequent navigator target waits for the interval.
    Px4PositionControl3DBackend delayed_target(config(0.020));
    const auto first = delayed_target.run_position(original, 0.008);
    const auto opposite = target(-30.0, -M_PI_2);
    const auto held_once = delayed_target.run_position(opposite, 0.008);
    const auto held_twice = delayed_target.run_position(opposite, 0.008);
    const auto applied = delayed_target.run_position(opposite, 0.008);
    require(nearly_equal(thrust_x(first), thrust_x(held_once), 1e-8)
            && nearly_equal(thrust_x(first), thrust_x(held_twice), 1e-8),
        "new position target must wait for the next trajectory update");
    require(!nearly_equal(thrust_x(first), thrust_x(applied), 1e-8),
        "new position target must apply on the next trajectory update");
    require(nearly_equal(*first.target_yaw_rate_rad_sec, *held_once.target_yaw_rate_rad_sec, 1e-9),
        "new yaw target must wait with the position target");

    // Without an explicit yaw target, the yaw-rate input remains a pass-through while the
    // position trajectory is held.
    Px4PositionControl3DBackend no_yaw(config(0.020));
    auto no_yaw_input = original;
    no_yaw_input.target_yaw_rad.reset();
    no_yaw_input.target_yaw_rate_rad_sec = 0.25;
    const auto no_yaw_first = no_yaw.run_position(no_yaw_input, 0.008);
    no_yaw_input.target_yaw_rate_rad_sec = -0.15;
    const auto no_yaw_held = no_yaw.run_position(no_yaw_input, 0.008);
    require(nearly_equal(thrust_x(no_yaw_first), thrust_x(no_yaw_held), 1e-8),
        "position trajectory must be held without a yaw target");
    require(nearly_equal(*no_yaw_held.target_yaw_rate_rad_sec, -0.15),
        "yaw-rate input must pass through without a yaw target");

    Px4PositionControl3DBackend every_call(config(0.0));
    const auto every_first = every_call.run_position(original, 0.008);
    const auto every_second = every_call.run_position(original, 0.008);
    require(!nearly_equal(thrust_x(every_first), thrust_x(every_second), 1e-8),
        "zero interval must preserve the every-call behavior");
#else
    // The option is inactive in an OFF build: a changed target takes effect immediately.
    Px4PositionControl3DBackend backend(config(0.020));
    const auto positive = backend.run_position(target(30.0, 0.0), 0.008);
    const auto negative = backend.run_position(target(-30.0, 0.0), 0.008);
    require(thrust_x(positive) * thrust_x(negative) < 0.0,
        "OFF build must ignore the trajectory interval and apply a changed target immediately");
#endif
    return EXIT_SUCCESS;
}
