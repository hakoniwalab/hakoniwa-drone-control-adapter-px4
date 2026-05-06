#include "hakoniwa/drone/control_adapter/px4_rate_control_backend.hpp"

#include "rate_control.hpp"

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

}  // namespace

Px4RateControlBackend::Px4RateControlBackend(const Px4RateControlBackendConfig& config)
    : config_(config)
    , controller_(new RateControl())
{
    apply_config();
}

Px4RateControlBackend::~Px4RateControlBackend()
{
    delete controller_;
}

void Px4RateControlBackend::reset()
{
    controller_->resetIntegral();
}

BodyTorqueCommand Px4RateControlBackend::run(const RateControlInput& input)
{
    controller_->setSaturationStatus(
        matrix::Vector3<bool>{
            input.saturation.roll.positive,
            input.saturation.pitch.positive,
            input.saturation.yaw.positive
        },
        matrix::Vector3<bool>{
            input.saturation.roll.negative,
            input.saturation.pitch.negative,
            input.saturation.yaw.negative
        });

    const matrix::Vector3f torque = controller_->update(
        to_vector3f(input.rate.p, input.rate.q, input.rate.r),
        to_vector3f(input.target.p, input.target.q, input.target.r),
        to_vector3f(input.angular_accel.p_dot, input.angular_accel.q_dot, input.angular_accel.r_dot),
        static_cast<float>(is_valid_dt(input.dt_sec) ? input.dt_sec : 0.0),
        input.landed);

    return BodyTorqueCommand{
        torque(0),
        torque(1),
        torque(2)
    };
}

void Px4RateControlBackend::set_config(const Px4RateControlBackendConfig& config)
{
    config_ = config;
    apply_config();
}

Px4RateControlBackendStatus Px4RateControlBackend::get_status() const
{
    rate_ctrl_status_s status{};
    controller_->getRateControlStatus(status);

    return Px4RateControlBackendStatus{
        status.rollspeed_integ,
        status.pitchspeed_integ,
        status.yawspeed_integ
    };
}

void Px4RateControlBackend::apply_config()
{
    controller_->setPidGains(
        to_vector3f(config_.gains.roll.p, config_.gains.pitch.p, config_.gains.yaw.p),
        to_vector3f(config_.gains.roll.i, config_.gains.pitch.i, config_.gains.yaw.i),
        to_vector3f(config_.gains.roll.d, config_.gains.pitch.d, config_.gains.yaw.d));

    controller_->setIntegratorLimit(
        to_vector3f(
            config_.integrator_limits.roll_integrator,
            config_.integrator_limits.pitch_integrator,
            config_.integrator_limits.yaw_integrator));

    controller_->setFeedForwardGain(
        to_vector3f(
            config_.feed_forward.roll,
            config_.feed_forward.pitch,
            config_.feed_forward.yaw));
}

bool Px4RateControlBackend::is_valid_dt(double dt_sec)
{
    return dt_sec > 0.0;
}

}  // namespace hakoniwa::drone::control_adapter
