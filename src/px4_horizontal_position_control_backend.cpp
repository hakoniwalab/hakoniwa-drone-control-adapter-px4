#include "hakoniwa/drone/control_adapter/px4_horizontal_position_control_backend.hpp"

#include "PositionControl.hpp"

namespace hakoniwa::drone::control_adapter {

namespace {

float f64(double value)
{
    return static_cast<float>(value);
}

HorizontalTiltTarget to_tilt_target(const vehicle_attitude_setpoint_s& attitude_setpoint)
{
    matrix::Quatf q{
        attitude_setpoint.q_d[0],
        attitude_setpoint.q_d[1],
        attitude_setpoint.q_d[2],
        attitude_setpoint.q_d[3]
    };
    q.normalize();
    const matrix::Eulerf euler{q};

    return HorizontalTiltTarget{
        euler(0),
        euler(1)
    };
}

}  // namespace

Px4HorizontalPositionControlBackend::Px4HorizontalPositionControlBackend(
    const Px4HorizontalPositionControlBackendConfig& config)
    : config_(config)
    , controller_(new PositionControl())
{
    apply_config();
    reset();
}

Px4HorizontalPositionControlBackend::~Px4HorizontalPositionControlBackend()
{
    delete controller_;
}

void Px4HorizontalPositionControlBackend::reset()
{
    controller_->resetIntegral();
}

HorizontalTiltTarget Px4HorizontalPositionControlBackend::run(
    const HorizontalPositionControlInput& input,
    double dt_sec)
{
    PositionControlStates state{};
    state.position = matrix::Vector3f{f64(input.position.x), f64(input.position.y), 0.0f};
    state.velocity = matrix::Vector3f{f64(input.velocity.vx), f64(input.velocity.vy), 0.0f};
    state.acceleration = matrix::Vector3f{f64(input.acceleration.ax), f64(input.acceleration.ay), 0.0f};
    state.yaw = f64(input.yaw_rad);
    controller_->setState(state);

    trajectory_setpoint_s setpoint = PositionControl::empty_trajectory_setpoint;
    if (input.mode == HorizontalControlMode::Position) {
        setpoint.position[0] = f64(input.target_position.x);
        setpoint.position[1] = f64(input.target_position.y);
    }
    else {
        setpoint.velocity[0] = f64(input.target_velocity.vx);
        setpoint.velocity[1] = f64(input.target_velocity.vy);
    }
    setpoint.acceleration[2] = 0.0f;
    setpoint.yaw = f64(input.yaw_rad);
    setpoint.yawspeed = 0.0f;
    controller_->setInputSetpoint(setpoint);

    const bool ok = controller_->update(f64(dt_sec > 0.0 ? dt_sec : 0.0));

    if (!ok) {
        return HorizontalTiltTarget{};
    }

    vehicle_attitude_setpoint_s attitude_setpoint{};
    controller_->getAttitudeSetpoint(attitude_setpoint);
    return to_tilt_target(attitude_setpoint);
}

void Px4HorizontalPositionControlBackend::set_config(
    const Px4HorizontalPositionControlBackendConfig& config)
{
    config_ = config;
    apply_config();
}

void Px4HorizontalPositionControlBackend::apply_config()
{
    controller_->setPositionGains(matrix::Vector3f{f64(config_.position_gain_xy), f64(config_.position_gain_xy), 0.0f});
    controller_->setVelocityGains(
        matrix::Vector3f{f64(config_.velocity_p_xy), f64(config_.velocity_p_xy), 0.0f},
        matrix::Vector3f{f64(config_.velocity_i_xy), f64(config_.velocity_i_xy), 0.0f},
        matrix::Vector3f{f64(config_.velocity_d_xy), f64(config_.velocity_d_xy), 0.0f});
    controller_->setVelocityLimits(f64(config_.velocity_max_xy_mps), 1.0f, 1.0f);
    controller_->setThrustLimits(f64(config_.thrust_min), f64(config_.thrust_max));
    controller_->setHorizontalThrustMargin(f64(config_.horizontal_thrust_margin));
    controller_->setTiltLimit(f64(config_.tilt_limit_rad));
    controller_->setHoverThrust(f64(config_.hover_thrust));
    controller_->decoupleHorizontalAndVecticalAcceleration(config_.decouple_horizontal_and_vertical_acceleration);
}

}  // namespace hakoniwa::drone::control_adapter
