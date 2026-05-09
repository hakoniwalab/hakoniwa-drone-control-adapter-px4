#include "hakoniwa/drone/control_adapter/px4_altitude_control_backend.hpp"

#include "PositionControl.hpp"
#include <iostream>
#include <cmath>

namespace hakoniwa::drone::control_adapter {

namespace {

constexpr float kMinHoverThrust = 1e-6f;

float f64(double value)
{
    return static_cast<float>(value);
}

// Hakoniwa/native altitude convention:
//   altitude z, vertical velocity vz, vertical acceleration az are up-positive.
//
// PX4 PositionControl convention:
//   local position is NED, so z, vz, az are down-positive.
//
// Therefore, every vertical quantity crossing into PX4 must be sign-flipped.
float hako_up_to_px4_down(double value)
{
    return -f64(value);
}

float px4_thrust_to_hako_body_z(float px4_thrust_z, float hover_thrust)
{
    if (!std::isfinite(px4_thrust_z) || !std::isfinite(hover_thrust) || std::fabs(hover_thrust) <= kMinHoverThrust) {
        return 0.0f;
    }

    // PX4 thrust[2] is normalized around hover=-MPC_THR_HOVER.
    // Hakoniwa adapter contract uses hover=-1.0.
    return px4_thrust_z / hover_thrust;
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
    const float dt = f64(dt_sec > 0.0 ? dt_sec : 0.0);

    const float px4_z  = hako_up_to_px4_down(input.position.z);
    const float px4_vz = hako_up_to_px4_down(input.velocity.vz);
    const float px4_az = hako_up_to_px4_down(input.acceleration.az);

#if 0
    std::cout << "Running Px4AltitudeControlBackend with input: "
              << "mode=" << (input.mode == AltitudeControlMode::Position ? "Position" : "Velocity") << ", "
              << "target_altitude=" << input.target_altitude << ", "
              << "target_velocity=" << input.target_velocity.vz
              << std::endl;
#endif
    PositionControlStates state{};
    state.position = matrix::Vector3f{0.0f, 0.0f, px4_z};
    state.velocity = matrix::Vector3f{0.0f, 0.0f, px4_vz};
    state.acceleration = matrix::Vector3f{0.0f, 0.0f, px4_az};
    state.yaw = 0.0f;
    controller_->setState(state);

    trajectory_setpoint_s setpoint = PositionControl::empty_trajectory_setpoint;

    // Keep x/y acceleration finite so PX4 input validation passes for the
    // unused horizontal axes. Z is generated from position or velocity control.
    setpoint.acceleration[0] = 0.0f;
    setpoint.acceleration[1] = 0.0f;

    if (input.mode == AltitudeControlMode::Position) {
        setpoint.position[2] = hako_up_to_px4_down(input.target_altitude);
    }
    else {
        setpoint.velocity[2] = hako_up_to_px4_down(input.target_velocity.vz);
    }

    setpoint.yaw = 0.0f;
    setpoint.yawspeed = 0.0f;
    controller_->setInputSetpoint(setpoint);

    const bool ok = controller_->update(dt);

    vehicle_local_position_setpoint_s local_position_setpoint{};
    controller_->getLocalPositionSetpoint(local_position_setpoint);
#if 0
    std::cout
        << "[Px4Altitude] dt=" << dt_sec
        << " ok=" << ok
        << " hako_pos_z=" << input.position.z
        << " hako_vel_z=" << input.velocity.vz
        << " hako_acc_z=" << input.acceleration.az
        << " hako_target_altitude=" << input.target_altitude
        << " hako_target_velocity=" << input.target_velocity.vz
        << " px4_z=" << px4_z
        << " px4_vz=" << px4_vz
        << " px4_az=" << px4_az
        << " px4_z_sp=" << local_position_setpoint.z
        << " px4_vz_sp=" << local_position_setpoint.vz
        << " px4_acc_sp_z=" << local_position_setpoint.acceleration[2]
        << " px4_thrust_z=" << local_position_setpoint.thrust[2]
        << std::endl;
#endif
    if (!ok || !std::isfinite(local_position_setpoint.thrust[2])) {
        std::cerr
            << "Warning: PositionControl update failed or produced invalid thrust. "
            << "Returning zero vertical thrust command."
            << std::endl;

        return NormalizedVerticalThrustCommand{};
    }

    const float hako_body_z = px4_thrust_to_hako_body_z(
        local_position_setpoint.thrust[2],
        f64(config_.hover_thrust));

#if 0
    std::cout
        << " hako_body_z=" << hako_body_z
        << std::endl;
#endif

    return NormalizedVerticalThrustCommand{
        hako_body_z
    };
}

void Px4AltitudeControlBackend::set_config(const Px4AltitudeControlBackendConfig& config)
{
    config_ = config;
    apply_config();
}

void Px4AltitudeControlBackend::apply_config()
{
    controller_->setPositionGains(
        matrix::Vector3f{
            0.0f,
            0.0f,
            f64(config_.position_gain_z)
        });

    controller_->setVelocityGains(
        matrix::Vector3f{
            0.0f,
            0.0f,
            f64(config_.velocity_p_z)
        },
        matrix::Vector3f{
            0.0f,
            0.0f,
            f64(config_.velocity_i_z)
        },
        matrix::Vector3f{
            0.0f,
            0.0f,
            f64(config_.velocity_d_z)
        });

    controller_->setVelocityLimits(
        1.0f,
        f64(config_.velocity_max_up_mps),
        f64(config_.velocity_max_down_mps));

    controller_->setThrustLimits(
        f64(config_.thrust_min),
        f64(config_.thrust_max));

    controller_->setHorizontalThrustMargin(0.0f);
    controller_->setTiltLimit(0.0f);
    controller_->setHoverThrust(f64(config_.hover_thrust));

    controller_->decoupleHorizontalAndVecticalAcceleration(true);

    std::cout
        << "[Px4AltitudeConfig]"
        << " position_gain_z=" << config_.position_gain_z
        << " velocity_p_z=" << config_.velocity_p_z
        << " velocity_i_z=" << config_.velocity_i_z
        << " velocity_d_z=" << config_.velocity_d_z
        << " velocity_max_up_mps=" << config_.velocity_max_up_mps
        << " velocity_max_down_mps=" << config_.velocity_max_down_mps
        << " hover_thrust=" << config_.hover_thrust
        << " thrust_min=" << config_.thrust_min
        << " thrust_max=" << config_.thrust_max
        << std::endl;
}

}  // namespace hakoniwa::drone::control_adapter
