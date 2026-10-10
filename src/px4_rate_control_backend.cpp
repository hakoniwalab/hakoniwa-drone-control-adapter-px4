#include "hakoniwa/drone/control_adapter/px4_rate_control_backend.hpp"

#include "rate_control.hpp"

#include <cmath>

#include <lib/mathlib/math/filter/AlphaFilter.hpp>
#include <lib/mathlib/math/filter/LowPassFilter2p.hpp>

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

// The filters of PX4's VehicleAngularVelocity, in the order PX4 runs them: the angular rate
// through LowPassFilter2p (IMU_GYRO_CUTOFF), the angular acceleration through AlphaFilter
// (IMU_DGYRO_CUTOFF). Configured for the stage's sample rate on the first call and whenever it
// changes, and started at the first sample (PX4 resets them to the current value).
struct Px4RateControlBackend::ImuFilters {
    math::LowPassFilter2p<float> rate[3]{};
    AlphaFilter<float> acceleration[3]{};
    float sample_rate_hz{0.0f};
    bool started{false};
};

Px4RateControlBackend::Px4RateControlBackend(const Px4RateControlBackendConfig& config)
    : config_(config)
    , controller_(new RateControl())
    , imu_filters_(new ImuFilters())
{
    apply_config();
}

Px4RateControlBackend::~Px4RateControlBackend()
{
    delete controller_;
    delete imu_filters_;
}

void Px4RateControlBackend::reset()
{
    controller_->resetIntegral();
    imu_filters_->started = false;
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

    matrix::Vector3f rate = to_vector3f(input.rate.p, input.rate.q, input.rate.r);
    matrix::Vector3f angular_accel =
        to_vector3f(input.angular_accel.p_dot, input.angular_accel.q_dot, input.angular_accel.r_dot);
#ifdef HAKO_EKF_IMU_ACCELERATION
    if (is_valid_dt(input.dt_sec)) {
        ImuFilters& filters = *imu_filters_;
        const float sample_rate_hz = static_cast<float>(1.0 / input.dt_sec);
        if (!filters.started || std::fabs(sample_rate_hz - filters.sample_rate_hz) > 1.0f) {
            for (int axis = 0; axis < 3; ++axis) {
                filters.rate[axis].set_cutoff_frequency(sample_rate_hz,
                    static_cast<float>(config_.imu_filters.gyro_cutoff_hz));
                if (config_.imu_filters.dgyro_cutoff_hz <= 0.0
                    || !filters.acceleration[axis].setCutoffFreq(sample_rate_hz,
                        static_cast<float>(config_.imu_filters.dgyro_cutoff_hz))) {
                    filters.acceleration[axis].setAlpha(1.f);  // disabled, as PX4
                }
                if (!filters.started) {
                    // PX4 starts the filters at the current value; a later rate change keeps their state.
                    filters.rate[axis].reset(rate(axis));
                    filters.acceleration[axis].reset(angular_accel(axis));
                }
            }
            filters.sample_rate_hz = sample_rate_hz;
            filters.started = true;
        }
        for (int axis = 0; axis < 3; ++axis) {
            rate(axis) = filters.rate[axis].apply(rate(axis));
            angular_accel(axis) = filters.acceleration[axis].update(angular_accel(axis));
        }
    }
#endif

    const matrix::Vector3f torque = controller_->update(
        rate,
        to_vector3f(input.target.p, input.target.q, input.target.r),
        angular_accel,
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
