#include "hakoniwa/drone/control_adapter/px4_position_control_3d_backend.hpp"

#include "PositionControl.hpp"

#include <cmath>

namespace hakoniwa::drone::control_adapter {

namespace {

constexpr float kMinHoverThrust = 1e-6f;

float f64(double value)
{
    return static_cast<float>(value);
}

matrix::Vector3f to_px4_vector(const Vector3D& value)
{
    return matrix::Vector3f{f64(value.x), f64(value.y), f64(value.z)};
}

AttitudeQuaternion to_adapter_quaternion(const vehicle_attitude_setpoint_s& attitude_setpoint)
{
    return AttitudeQuaternion{
        attitude_setpoint.q_d[0],
        attitude_setpoint.q_d[1],
        attitude_setpoint.q_d[2],
        attitude_setpoint.q_d[3]
    };
}

NormalizedVerticalThrustCommand px4_body_thrust_to_adapter(float px4_thrust_body_z, float hover_thrust)
{
    if (!std::isfinite(px4_thrust_body_z) || !std::isfinite(hover_thrust) || std::fabs(hover_thrust) <= kMinHoverThrust) {
        return NormalizedVerticalThrustCommand{};
    }

    // Existing adapter thrust contract normalizes hover to approximately -1.0.
    // PX4 attitude thrust is normalized around hover=-MPC_THR_HOVER.
    return NormalizedVerticalThrustCommand{
        px4_thrust_body_z / hover_thrust
    };
}

void apply_common_state(PositionControl& controller, const PositionControl3DState& input_state)
{
    PositionControlStates state{};
    state.position = to_px4_vector(input_state.position);
    state.velocity = to_px4_vector(input_state.velocity);
    state.acceleration = to_px4_vector(input_state.acceleration);
    state.yaw = f64(input_state.yaw_rad);
    controller.setState(state);
}

void apply_common_yaw_setpoint(
    trajectory_setpoint_s& setpoint,
    const PositionControl3DState& state,
    const std::optional<double>& target_yaw_rad,
    const std::optional<double>& target_yaw_rate_rad_sec)
{
    setpoint.yaw = f64(target_yaw_rad.value_or(state.yaw_rad));
    setpoint.yawspeed = f64(target_yaw_rate_rad_sec.value_or(0.0));
}

PositionControl3DOutput make_output(
    PositionControl& controller,
    double hover_thrust,
    const std::optional<double>& target_yaw_rate_rad_sec)
{
    vehicle_attitude_setpoint_s attitude_setpoint{};
    controller.getAttitudeSetpoint(attitude_setpoint);

    vehicle_local_position_setpoint_s local_position_setpoint{};
    controller.getLocalPositionSetpoint(local_position_setpoint);

    PositionControl3DOutput output{};
    output.target_attitude = to_adapter_quaternion(attitude_setpoint);
    output.thrust = px4_body_thrust_to_adapter(attitude_setpoint.thrust_body[2], f64(hover_thrust));
    output.target_yaw_rate_rad_sec = target_yaw_rate_rad_sec;
    output.debug_thrust_sp = Vector3D{
        local_position_setpoint.thrust[0],
        local_position_setpoint.thrust[1],
        local_position_setpoint.thrust[2]
    };
    return output;
}

}  // namespace

Px4PositionControl3DBackend::Px4PositionControl3DBackend(const Px4PositionControl3DBackendConfig& config)
    : config_(config)
    , controller_(new PositionControl())
{
    apply_config();
    reset();
}

Px4PositionControl3DBackend::~Px4PositionControl3DBackend()
{
    delete controller_;
}

void Px4PositionControl3DBackend::reset()
{
    controller_->resetIntegral();
}

PositionControl3DOutput Px4PositionControl3DBackend::run_position(
    const PositionControl3DPositionInput& input,
    double dt_sec)
{
    apply_common_state(*controller_, input.state);

    trajectory_setpoint_s setpoint = PositionControl::empty_trajectory_setpoint;
    setpoint.position[0] = f64(input.target_position.x);
    setpoint.position[1] = f64(input.target_position.y);
    setpoint.position[2] = f64(input.target_position.z);

    if (input.feedforward_velocity) {
        setpoint.velocity[0] = f64(input.feedforward_velocity->x);
        setpoint.velocity[1] = f64(input.feedforward_velocity->y);
        setpoint.velocity[2] = f64(input.feedforward_velocity->z);
    }

    if (input.feedforward_acceleration) {
        setpoint.acceleration[0] = f64(input.feedforward_acceleration->x);
        setpoint.acceleration[1] = f64(input.feedforward_acceleration->y);
        setpoint.acceleration[2] = f64(input.feedforward_acceleration->z);
    }

    apply_common_yaw_setpoint(setpoint, input.state, input.target_yaw_rad, input.target_yaw_rate_rad_sec);
    controller_->setInputSetpoint(setpoint);

    const bool ok = controller_->update(f64(dt_sec > 0.0 ? dt_sec : 0.0));
    if (!ok) {
        return PositionControl3DOutput{};
    }

    return make_output(*controller_, config_.hover_thrust, input.target_yaw_rate_rad_sec);
}

PositionControl3DOutput Px4PositionControl3DBackend::run_velocity(
    const PositionControl3DVelocityInput& input,
    double dt_sec)
{
    apply_common_state(*controller_, input.state);

    trajectory_setpoint_s setpoint = PositionControl::empty_trajectory_setpoint;
    setpoint.velocity[0] = f64(input.target_velocity.x);
    setpoint.velocity[1] = f64(input.target_velocity.y);
    setpoint.velocity[2] = f64(input.target_velocity.z);

    if (input.feedforward_acceleration) {
        setpoint.acceleration[0] = f64(input.feedforward_acceleration->x);
        setpoint.acceleration[1] = f64(input.feedforward_acceleration->y);
        setpoint.acceleration[2] = f64(input.feedforward_acceleration->z);
    }

    apply_common_yaw_setpoint(setpoint, input.state, input.target_yaw_rad, input.target_yaw_rate_rad_sec);
    controller_->setInputSetpoint(setpoint);

    const bool ok = controller_->update(f64(dt_sec > 0.0 ? dt_sec : 0.0));
    if (!ok) {
        return PositionControl3DOutput{};
    }

    return make_output(*controller_, config_.hover_thrust, input.target_yaw_rate_rad_sec);
}

void Px4PositionControl3DBackend::set_config(const Px4PositionControl3DBackendConfig& config)
{
    config_ = config;
    apply_config();
}

void Px4PositionControl3DBackend::apply_config()
{
    controller_->setPositionGains(
        matrix::Vector3f{
            f64(config_.position_gain_xy),
            f64(config_.position_gain_xy),
            f64(config_.position_gain_z)
        });

    controller_->setVelocityGains(
        matrix::Vector3f{
            f64(config_.velocity_p_xy),
            f64(config_.velocity_p_xy),
            f64(config_.velocity_p_z)
        },
        matrix::Vector3f{
            f64(config_.velocity_i_xy),
            f64(config_.velocity_i_xy),
            f64(config_.velocity_i_z)
        },
        matrix::Vector3f{
            f64(config_.velocity_d_xy),
            f64(config_.velocity_d_xy),
            f64(config_.velocity_d_z)
        });

    controller_->setVelocityLimits(
        f64(config_.velocity_max_xy_mps),
        f64(config_.velocity_max_up_mps),
        f64(config_.velocity_max_down_mps));

    controller_->setThrustLimits(
        f64(config_.thrust_min),
        f64(config_.thrust_max));

    controller_->setHorizontalThrustMargin(f64(config_.horizontal_thrust_margin));
    controller_->setTiltLimit(f64(config_.tilt_limit_rad));
    controller_->setHoverThrust(f64(config_.hover_thrust));
    controller_->decoupleHorizontalAndVecticalAcceleration(config_.decouple_horizontal_and_vertical_acceleration);
}

}  // namespace hakoniwa::drone::control_adapter
