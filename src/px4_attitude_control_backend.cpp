#include "hakoniwa/drone/control_adapter/px4_attitude_control_backend.hpp"

#include "AttitudeControl.hpp"

namespace hakoniwa::drone::control_adapter {

namespace {

matrix::Vector3f to_vector3f(double x, double y, double z)
{
    return matrix::Vector3f{
        static_cast<float>(x),
        static_cast<float>(y),
        static_cast<float>(z)
    };
}

matrix::Quatf to_quatf(const AttitudeQuaternion& q)
{
    matrix::Quatf quat{
        static_cast<float>(q.w),
        static_cast<float>(q.x),
        static_cast<float>(q.y),
        static_cast<float>(q.z)
    };
    quat.normalize();
    return quat;
}

}  // namespace

Px4AttitudeControlBackend::Px4AttitudeControlBackend(const Px4AttitudeControlBackendConfig& config)
    : config_(config)
    , controller_(new AttitudeControl())
{
    apply_config();
    reset();
}

Px4AttitudeControlBackend::~Px4AttitudeControlBackend()
{
    delete controller_;
}

void Px4AttitudeControlBackend::reset()
{
    controller_->setAttitudeSetpoint(matrix::Quatf{}, 0.0f);
}

AngularRateTarget Px4AttitudeControlBackend::run(const AttitudeControlInput& input)
{
    controller_->setAttitudeSetpoint(
        to_quatf(input.target_attitude),
        static_cast<float>(input.target_yaw_rate_rad_sec));

    const matrix::Vector3f rate_target = controller_->update(to_quatf(input.attitude));

    return AngularRateTarget{
        rate_target(0),
        rate_target(1),
        rate_target(2)
    };
}

void Px4AttitudeControlBackend::set_config(const Px4AttitudeControlBackendConfig& config)
{
    config_ = config;
    apply_config();
}

void Px4AttitudeControlBackend::apply_config()
{
    controller_->setProportionalGain(
        to_vector3f(
            config_.proportional_gains.roll,
            config_.proportional_gains.pitch,
            config_.proportional_gains.yaw),
        static_cast<float>(config_.yaw_weight));

    controller_->setRateLimit(
        to_vector3f(
            config_.rate_limits.roll_rad_sec,
            config_.rate_limits.pitch_rad_sec,
            config_.rate_limits.yaw_rad_sec));
}

}  // namespace hakoniwa::drone::control_adapter
