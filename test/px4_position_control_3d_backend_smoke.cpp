#include "hakoniwa/drone/control_adapter/px4_position_control_3d_backend.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>

using namespace hakoniwa::drone::control_adapter;

namespace {

void require(bool condition, const char* message)
{
    if (!condition) {
        std::cerr << message << std::endl;
        std::exit(EXIT_FAILURE);
    }
}

bool finite_quaternion(const AttitudeQuaternion& q)
{
    return std::isfinite(q.w) && std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z);
}

bool nearly_equal(double a, double b, double eps = 1e-5)
{
    return std::fabs(a - b) <= eps;
}

}  // namespace

int main()
{
    Px4PositionControl3DBackendConfig config{};
    config.position_gain_xy = 1.0;
    config.position_gain_z = 1.0;
    config.velocity_p_xy = 2.5;
    config.velocity_i_xy = 0.2;
    config.velocity_d_xy = 0.0;
    config.velocity_p_z = 2.5;
    config.velocity_i_z = 0.2;
    config.velocity_d_z = 0.0;
    config.velocity_max_xy_mps = 2.0;
    config.velocity_max_up_mps = 1.0;
    config.velocity_max_down_mps = 1.0;
    config.tilt_limit_rad = 0.4;
    config.hover_thrust = 0.5;
    config.thrust_min = 0.1;
    config.thrust_max = 0.9;
    config.horizontal_thrust_margin = 0.3;
    config.decouple_horizontal_and_vertical_acceleration = false;

    Px4PositionControl3DBackend backend(config);

    const PositionControl3DState hover_state{
        {0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0},
        {},
        0.0
    };

    const PositionControl3DOutput hold = backend.run_position(
        PositionControl3DPositionInput{
            hover_state,
            {0.0, 0.0, 0.0},
            {},
            {},
            0.0,
            0.0
        },
        0.01);

    require(finite_quaternion(hold.target_attitude), "expected finite hover target attitude");
    require(std::isfinite(hold.thrust.body_z), "expected finite hover thrust");
    require(hold.thrust.body_z < 0.0, "expected hover thrust to be upward/negative body-z command");
    require(nearly_equal(hold.thrust.body_z, -1.0, 0.05),
        "expected 3D hover thrust normalized to Hakoniwa convention");
    require(hold.debug_thrust_sp.has_value(), "expected debug thrust vector to be populated");

    PositionControl3DPositionInput tilted_input{
        hover_state,
        {0.0, 0.0, 0.0},
        {},
        Vector3D{2.0, 0.0, 0.0},
        0.0,
        0.0
    };
    backend.reset();
    const PositionControl3DOutput tilted = backend.run_position(tilted_input, 0.01);
    require(
        std::fabs(tilted.thrust.body_z) > std::fabs(hold.thrust.body_z),
        "3D position body_z must be derived from tilted thrust_body magnitude");

    backend.reset();
    const PositionControl3DOutput velocity = backend.run_velocity(
        PositionControl3DVelocityInput{
            hover_state,
            {1.0, 0.0, 0.0},
            {},
            0.0,
            0.0
        },
        0.01);

    require(finite_quaternion(velocity.target_attitude), "expected finite velocity target attitude");
    require(std::isfinite(velocity.thrust.body_z), "expected finite velocity thrust");
    require(velocity.thrust.body_z < 0.0, "expected velocity command to keep upward/negative body-z thrust");
    require(velocity.debug_thrust_sp.has_value(), "expected velocity debug thrust vector to be populated");

    Px4PositionControl3DBackendConfig integral_config = config;
    integral_config.velocity_p_xy = 0.0;
    integral_config.velocity_i_xy = 1.0;
    integral_config.velocity_d_xy = 0.0;
    Px4PositionControl3DBackend short_dt_backend(integral_config);
    Px4PositionControl3DBackend long_dt_backend(integral_config);
    PositionControl3DVelocityInput integral_input{
        hover_state,
        {0.1, 0.0, 0.0},
        {},
        0.0,
        0.0
    };
    (void)short_dt_backend.run_velocity(integral_input, 0.01);
    (void)long_dt_backend.run_velocity(integral_input, 0.04);
    const auto short_dt_output = short_dt_backend.run_velocity(integral_input, 0.01);
    const auto long_dt_output = long_dt_backend.run_velocity(integral_input, 0.04);
    require(short_dt_output.debug_thrust_sp.has_value(), "missing short-dt debug thrust");
    require(long_dt_output.debug_thrust_sp.has_value(), "missing long-dt debug thrust");
    require(
        std::fabs(long_dt_output.debug_thrust_sp->x) >
            std::fabs(short_dt_output.debug_thrust_sp->x) * 3.5,
        "3D position integrator must use call dt_sec");

#ifdef HAKO_EKF_IMU_ACCELERATION
    Px4PositionControl3DBackendConfig trajectory_config = config;
    trajectory_config.velocity_max_xy_mps = 5.0;
    trajectory_config.trajectory.enabled = true;
    Px4PositionControl3DBackendConfig direct_config = trajectory_config;
    direct_config.trajectory.enabled = false;

    PositionControl3DPositionInput distant_target{};
    distant_target.state = hover_state;
    distant_target.target_position = {30.0, 0.0, 0.0};
    distant_target.target_yaw_rad = M_PI_2;

    Px4PositionControl3DBackend direct_backend(direct_config);
    Px4PositionControl3DBackend smoothed_backend(trajectory_config);
    const auto direct = direct_backend.run_position(distant_target, 0.01);
    const auto smoothed = smoothed_backend.run_position(distant_target, 0.01);
    require(direct.debug_thrust_sp.has_value() && smoothed.debug_thrust_sp.has_value(),
        "trajectory comparison requires thrust setpoints");
    require(std::fabs(smoothed.debug_thrust_sp->x) < std::fabs(direct.debug_thrust_sp->x),
        "enabled trajectory must soften the first position step");
    require(smoothed.target_yaw_rate_rad_sec.has_value(),
        "heading smoothing must publish a yaw-rate target");
    require(*smoothed.target_yaw_rate_rad_sec > 0.0
            && *smoothed.target_yaw_rate_rad_sec <= 20.0 * M_PI / 180.0 * 0.01 + 1e-5,
        "heading smoothing must obey the yaw acceleration limit");

    PositionControl3DPositionInput feedforward = distant_target;
    feedforward.feedforward_velocity = Vector3D{0.2, 0.0, 0.0};
    Px4PositionControl3DBackend feedforward_direct(direct_config);
    Px4PositionControl3DBackend feedforward_smoothed(trajectory_config);
    const auto ff_direct = feedforward_direct.run_position(feedforward, 0.01);
    const auto ff_smoothed = feedforward_smoothed.run_position(feedforward, 0.01);
    require(nearly_equal(ff_direct.debug_thrust_sp->x, ff_smoothed.debug_thrust_sp->x),
        "feedforward position target must bypass trajectory generation");

    PositionControl3DPositionInput partial = distant_target;
    partial.target_position.z = NAN;
    Px4PositionControl3DBackend partial_direct(direct_config);
    Px4PositionControl3DBackend partial_smoothed(trajectory_config);
    const auto nan_direct = partial_direct.run_position(partial, 0.01);
    const auto nan_smoothed = partial_smoothed.run_position(partial, 0.01);
    require(nearly_equal(nan_direct.debug_thrust_sp->x, nan_smoothed.debug_thrust_sp->x),
        "partial position target must bypass trajectory generation");

    smoothed_backend.reset();
    const auto after_reset = smoothed_backend.run_position(distant_target, 0.01);
    require(nearly_equal(after_reset.debug_thrust_sp->x, smoothed.debug_thrust_sp->x),
        "reset must restart trajectory from the vehicle state");
    PositionControl3DVelocityInput zero_velocity{};
    zero_velocity.state = hover_state;
    (void)smoothed_backend.run_velocity(zero_velocity, 0.01);
    const auto after_velocity = smoothed_backend.run_position(distant_target, 0.01);
    require(nearly_equal(after_velocity.debug_thrust_sp->x, smoothed.debug_thrust_sp->x),
        "run_velocity must restart the next position trajectory");
#endif

    return EXIT_SUCCESS;
}
