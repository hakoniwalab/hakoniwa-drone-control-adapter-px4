#include "hakoniwa/drone/control_adapter/px4_altitude_control_backend.hpp"

#include "PositionControl.hpp"

namespace hakoniwa::drone::control_adapter {

namespace {

float f64(double value)
{
    return static_cast<float>(value);
}

}  // namespace

Px4AltitudeControlBackend::Px4AltitudeControlBackend(const Px4AltitudeControlBackendConfig& config)
    : config_(config)
    , controller_(new PositionControl())
{
    apply_config();
    reset();
}

Px4AltitudeControlBackend::~Px4AltitudeControlBackend()
{
    delete controller_;
}

void Px4AltitudeControlBackend::reset()
{
    controller_->resetIntegral();
}

NormalizedVerticalThrustCommand Px4AltitudeControlBackend::run(const AltitudeControlInput& input, double dt_sec)
{
    PositionControlStates state{};
    state.position = matrix::Vector3f{0.0f, 0.0f, f64(input.position.z)};
    state.velocity = matrix::Vector3f{0.0f, 0.0f, f64(input.velocity.vz)};
    state.acceleration = matrix::Vector3f{0.0f, 0.0f, f64(input.acceleration.az)};
    state.yaw = 0.0f;
    controller_->setState(state);

    trajectory_setpoint_s setpoint = PositionControl::empty_trajectory_setpoint;
    setpoint.position[0] = 0.0f;
    setpoint.position[1] = 0.0f;
    setpoint.position[2] = f64(input.target_altitude);
    setpoint.velocity[0] = 0.0f;
    setpoint.velocity[1] = 0.0f;
    setpoint.acceleration[0] = 0.0f;
    setpoint.acceleration[1] = 0.0f;
    setpoint.yaw = 0.0f;
    setpoint.yawspeed = 0.0f;
    controller_->setInputSetpoint(setpoint);

    const bool ok = controller_->update(f64(dt_sec > 0.0 ? dt_sec : 0.0));

    if (!ok) {
        return NormalizedVerticalThrustCommand{};
    }

    vehicle_local_position_setpoint_s local_position_setpoint{};
    controller_->getLocalPositionSetpoint(local_position_setpoint);

    return NormalizedVerticalThrustCommand{
        local_position_setpoint.thrust[2]
    };
}

void Px4AltitudeControlBackend::set_config(const Px4AltitudeControlBackendConfig& config)
{
    config_ = config;
    apply_config();
}

void Px4AltitudeControlBackend::apply_config()
{
    controller_->setPositionGains(matrix::Vector3f{0.0f, 0.0f, f64(config_.position_gain_z)});
    controller_->setVelocityGains(
        matrix::Vector3f{0.0f, 0.0f, f64(config_.velocity_p_z)},
        matrix::Vector3f{0.0f, 0.0f, f64(config_.velocity_i_z)},
        matrix::Vector3f{0.0f, 0.0f, f64(config_.velocity_d_z)});
    controller_->setVelocityLimits(1.0f, f64(config_.velocity_max_up_mps), f64(config_.velocity_max_down_mps));
    controller_->setThrustLimits(f64(config_.thrust_min), f64(config_.thrust_max));
    controller_->setHorizontalThrustMargin(0.0f);
    controller_->setTiltLimit(0.0f);
    controller_->setHoverThrust(f64(config_.hover_thrust));
    controller_->decoupleHorizontalAndVecticalAcceleration(true);
}

}  // namespace hakoniwa::drone::control_adapter
