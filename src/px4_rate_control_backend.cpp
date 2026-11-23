#include "hakoniwa/drone/control_adapter/px4_rate_control_backend.hpp"
#include "hakoniwa/drone/control_adapter/px4_sensor_data_type.hpp"

#include "rate_control.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

#include <lib/mathlib/math/filter/AlphaFilter.hpp>
#include <lib/mathlib/math/filter/LowPassFilter2p.hpp>
#include <lib/mathlib/math/filter/NotchFilter.hpp>

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
    AlphaFilter<float> yaw_torque{};  // MC_YAW_TQ_CUTOFF (MulticopterRateControl::_output_lpf_yaw)
    matrix::Vector3f previous_target{};  // previous_rates_setpoint: the last cycle's rates setpoint
    bool has_previous_target{false};
    float sample_rate_hz{0.0f};
    bool started{false};
#ifdef HAKO_ENABLE_GYRO_SAMPLE_INPUT
    struct SamplePipeline {
        math::NotchFilter<float> notch0[3]{};
        math::NotchFilter<float> notch1[3]{};
        math::LowPassFilter2p<float> rate[3]{};
        AlphaFilter<float> acceleration[3]{};
        std::vector<math::NotchFilter<float>> dynamic_notch{};
        std::vector<bool> rotor_available{};
        matrix::Vector3f previous_filtered_rate{};
        std::uint64_t previous_time_usec{0};
        std::size_t rotor_count{0};
        int harmonics{0};
        float sample_rate_hz{0.0f};
        bool started{false};
    } samples{};
#endif
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
    imu_filters_->yaw_torque.reset(0.f);
    imu_filters_->has_previous_target = false;
#ifdef HAKO_ENABLE_GYRO_SAMPLE_INPUT
    imu_filters_->samples = {};
#endif
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

    matrix::Vector3f rate = (config_.imu_data_type == sensor_data_type::kImuFifoInt16)
        ? to_vector3f(px4_imu_fifo_gyro(input.rate.p), px4_imu_fifo_gyro(input.rate.q), px4_imu_fifo_gyro(input.rate.r))
        : to_vector3f(input.rate.p, input.rate.q, input.rate.r);
#ifdef HAKO_ENABLE_SENSOR_MODEL_V2
    // [SEN-COM-340] Rotate v2 angular velocity back to body axes before rate control.
    const auto body_rate = rotate_sensor_to_body(config_.sensor_model_v2, {rate(0), rate(1), rate(2)});
    rate = to_vector3f(body_rate[0], body_rate[1], body_rate[2]);
#endif
    matrix::Vector3f angular_accel =
        to_vector3f(input.angular_accel.p_dot, input.angular_accel.q_dot, input.angular_accel.r_dot);
#ifdef HAKO_ENABLE_SENSOR_MODEL_V2
    const auto body_accel = rotate_sensor_to_body(config_.sensor_model_v2,
        {angular_accel(0), angular_accel(1), angular_accel(2)});
    angular_accel = to_vector3f(body_accel[0], body_accel[1], body_accel[2]);
#endif
#ifdef HAKO_ENABLE_GYRO_SAMPLE_INPUT
    const bool has_gyro_samples = input.gyro_samples != nullptr && input.gyro_sample_count > 0;
    if (has_gyro_samples) {
        auto& filters = imu_filters_->samples;
        const int harmonics = std::max(0, config_.imu_filters.dynamic_notch_harmonics);
        const bool dynamic_notch_shape_changed =
            filters.rotor_count != input.rotor_count || filters.harmonics != harmonics;
        if (dynamic_notch_shape_changed) {
            filters.dynamic_notch.assign(
                input.rotor_count * static_cast<std::size_t>(harmonics) * 3U,
                math::NotchFilter<float>{});
            filters.rotor_available.assign(input.rotor_count, false);
            filters.rotor_count = input.rotor_count;
            filters.harmonics = harmonics;
        }

        double first_sample_dt_sec = 0.0;
        const auto& first_sample = input.gyro_samples[0];
        if (filters.previous_time_usec > 0 && first_sample.time_usec > filters.previous_time_usec) {
            first_sample_dt_sec = static_cast<double>(first_sample.time_usec - filters.previous_time_usec) * 1e-6;
        } else if (input.gyro_sample_count > 1
            && input.gyro_samples[1].time_usec > first_sample.time_usec) {
            first_sample_dt_sec = static_cast<double>(input.gyro_samples[1].time_usec - first_sample.time_usec) * 1e-6;
        } else if (is_valid_dt(input.dt_sec)) {
            first_sample_dt_sec = input.dt_sec / static_cast<double>(input.gyro_sample_count);
        }
        const float run_sample_rate_hz = first_sample_dt_sec > 0.0
            ? static_cast<float>(1.0 / first_sample_dt_sec)
            : filters.sample_rate_hz;
        const bool sample_rate_changed = run_sample_rate_hz > 0.0f
            && (!filters.started || filters.sample_rate_hz <= 0.0f
                || std::fabs(run_sample_rate_hz - filters.sample_rate_hz) / run_sample_rate_hz > 0.01f);

        if (sample_rate_changed) {
            filters.dynamic_notch.assign(
                input.rotor_count * static_cast<std::size_t>(harmonics) * 3U,
                math::NotchFilter<float>{});
            filters.rotor_available.assign(input.rotor_count, false);
            filters.sample_rate_hz = run_sample_rate_hz;
        }

        // PX4 updates ESC-RPM notch coefficients once per work-queue run, before filtering
        // the samples in that run.  It initializes at most one new notch per axis per run.
        const bool dynamic_notch_enabled = (config_.imu_filters.dynamic_notch_enable & 1) != 0;
        if (dynamic_notch_enabled && filters.sample_rate_hz > 0.0f) {
            bool axis_init[3]{false, false, false};
            const float bandwidth_hz = static_cast<float>(config_.imu_filters.dynamic_notch_bandwidth_hz);
            const float freq_min = std::max(
                static_cast<float>(config_.imu_filters.dynamic_notch_min_frequency_hz),
                bandwidth_hz);

            for (std::size_t rotor = 0; rotor < input.rotor_count; ++rotor) {
                const bool rotor_connected = input.rotor_rad_per_sec != nullptr
                    && input.rotor_rad_per_sec[rotor] != 0.0;
                if (!rotor_connected) {
                    filters.rotor_available[rotor] = false;
                    continue;
                }

                const float rotor_hz = static_cast<float>(
                    std::fabs(input.rotor_rad_per_sec[rotor]) / (2.0 * M_PI));
                const bool force_update = sample_rate_changed
                    || dynamic_notch_shape_changed
                    || !filters.rotor_available[rotor];

                for (int harmonic = 0; harmonic < harmonics; ++harmonic) {
                    const float frequency_hz = std::max(
                        rotor_hz * static_cast<float>(harmonic + 1),
                        freq_min + static_cast<float>(harmonic) * 0.5f * bandwidth_hz);

                    for (int axis = 0; axis < 3; ++axis) {
                        const std::size_t index =
                            (rotor * static_cast<std::size_t>(harmonics) + static_cast<std::size_t>(harmonic)) * 3U
                            + static_cast<std::size_t>(axis);
                        auto& notch = filters.dynamic_notch[index];
                        const float notch_freq_delta = std::fabs(notch.getNotchFreq() - frequency_hz);
                        const bool notch_freq_changed = notch_freq_delta > 0.1f;
                        const bool allow_update = !axis_init[axis]
                            || (notch.initialized() && notch_freq_delta < notch.getBandwidth());

                        if ((force_update || notch_freq_changed) && allow_update
                            && notch.setParameters(filters.sample_rate_hz, frequency_hz, bandwidth_hz)
                            && !notch.initialized()) {
                            axis_init[axis] = true;
                        }
                    }
                }

                filters.rotor_available[rotor] = true;
            }
        } else {
            std::fill(filters.rotor_available.begin(), filters.rotor_available.end(), false);
        }

        for (std::size_t sample_index = 0; sample_index < input.gyro_sample_count; ++sample_index) {
            const auto& sample = input.gyro_samples[sample_index];
            matrix::Vector3f sample_rate = (config_.imu_data_type == sensor_data_type::kImuFifoInt16)
                ? to_vector3f(px4_imu_fifo_gyro(sample.p), px4_imu_fifo_gyro(sample.q), px4_imu_fifo_gyro(sample.r))
                : to_vector3f(sample.p, sample.q, sample.r);
#ifdef HAKO_ENABLE_SENSOR_MODEL_V2
            const auto body_sample = rotate_sensor_to_body(
                config_.sensor_model_v2, {sample_rate(0), sample_rate(1), sample_rate(2)});
            sample_rate = to_vector3f(body_sample[0], body_sample[1], body_sample[2]);
#endif

            double sample_dt_sec = 0.0;
            if (filters.previous_time_usec > 0 && sample.time_usec > filters.previous_time_usec) {
                sample_dt_sec = static_cast<double>(sample.time_usec - filters.previous_time_usec) * 1e-6;
            } else if (sample_index + 1 < input.gyro_sample_count
                && input.gyro_samples[sample_index + 1].time_usec > sample.time_usec) {
                sample_dt_sec = static_cast<double>(input.gyro_samples[sample_index + 1].time_usec - sample.time_usec) * 1e-6;
            } else if (is_valid_dt(input.dt_sec)) {
                sample_dt_sec = input.dt_sec / static_cast<double>(input.gyro_sample_count);
            }
            if (sample_index == 0 && sample_rate_changed) {
                for (int axis = 0; axis < 3; ++axis) {
                    filters.notch0[axis].setParameters(
                        filters.sample_rate_hz,
                        static_cast<float>(config_.imu_filters.gyro_notch0_frequency_hz),
                        static_cast<float>(config_.imu_filters.gyro_notch0_bandwidth_hz));
                    filters.notch0[axis].reset(sample_rate(axis));
                    filters.notch1[axis].setParameters(
                        filters.sample_rate_hz,
                        static_cast<float>(config_.imu_filters.gyro_notch1_frequency_hz),
                        static_cast<float>(config_.imu_filters.gyro_notch1_bandwidth_hz));
                    filters.notch1[axis].reset(sample_rate(axis));
                    filters.rate[axis].set_cutoff_frequency(
                        filters.sample_rate_hz, static_cast<float>(config_.imu_filters.gyro_cutoff_hz));
                    filters.rate[axis].reset(sample_rate(axis));
                    if (config_.imu_filters.dgyro_cutoff_hz <= 0.0
                        || !filters.acceleration[axis].setCutoffFreq(
                            filters.sample_rate_hz, static_cast<float>(config_.imu_filters.dgyro_cutoff_hz))) {
                        filters.acceleration[axis].setAlpha(1.f);
                    }
                    filters.acceleration[axis].reset(0.f);
                }
                filters.previous_filtered_rate = sample_rate;
                filters.started = true;
            }

            matrix::Vector3f filtered_rate = sample_rate;
            if (dynamic_notch_enabled) {
                for (std::size_t rotor = 0; rotor < input.rotor_count; ++rotor) {
                    if (filters.rotor_available[rotor]) {
                        for (int harmonic = 0; harmonic < harmonics; ++harmonic) {
                            for (int axis = 0; axis < 3; ++axis) {
                                const std::size_t index =
                                    (rotor * static_cast<std::size_t>(harmonics) + static_cast<std::size_t>(harmonic)) * 3U
                                    + static_cast<std::size_t>(axis);
                                auto& notch = filters.dynamic_notch[index];
                                if (notch.getNotchFreq() > 0.0f) {
                                    filtered_rate(axis) = notch.apply(filtered_rate(axis));
                                }
                            }
                        }
                    }
                }
            }

            matrix::Vector3f filtered_accel{};
            for (int axis = 0; axis < 3; ++axis) {
                if (filters.notch0[axis].getNotchFreq() > 0.0f) {
                    filtered_rate(axis) = filters.notch0[axis].apply(filtered_rate(axis));
                }
                if (filters.notch1[axis].getNotchFreq() > 0.0f) {
                    filtered_rate(axis) = filters.notch1[axis].apply(filtered_rate(axis));
                }
                filtered_rate(axis) = filters.rate[axis].apply(filtered_rate(axis));
                const float derivative = filters.started && sample_dt_sec > 0.0
                    ? (filtered_rate(axis) - filters.previous_filtered_rate(axis)) / static_cast<float>(sample_dt_sec)
                    : 0.0f;
                filtered_accel(axis) = filters.acceleration[axis].update(derivative);
            }
            filters.previous_filtered_rate = filtered_rate;
            if (sample.time_usec > 0) {
                filters.previous_time_usec = sample.time_usec;
            }
            rate = filtered_rate;
            angular_accel = filtered_accel;
        }
    } else
#endif
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

    matrix::Vector3f target = to_vector3f(input.target.p, input.target.q, input.target.r);
    if (config_.previous_rates_setpoint) {
        // PX4: the rate controller runs before the attitude controller publishes this cycle's setpoint.
        const matrix::Vector3f current = target;
        if (imu_filters_->has_previous_target) {
            target = imu_filters_->previous_target;
        }
        imu_filters_->previous_target = current;
        imu_filters_->has_previous_target = true;
    }
    matrix::Vector3f torque = controller_->update(
        rate,
        target,
        angular_accel,
        static_cast<float>(is_valid_dt(input.dt_sec) ? input.dt_sec : 0.0),
        input.landed);
    if (config_.yaw_torque_cutoff_hz > 0.0 && is_valid_dt(input.dt_sec)) {
        // MulticopterRateControl: low-pass filter the yaw torque against rotor acceleration.
        imu_filters_->yaw_torque.setCutoffFreq(static_cast<float>(config_.yaw_torque_cutoff_hz));
        torque(2) = imu_filters_->yaw_torque.update(torque(2), static_cast<float>(input.dt_sec));
    }

    return BodyTorqueCommand{
        torque(0),
        torque(1),
        torque(2)
    };
}

void Px4RateControlBackend::set_config(const Px4RateControlBackendConfig& config)
{
    config_ = config;
#ifdef HAKO_ENABLE_GYRO_SAMPLE_INPUT
    // Filter coefficients are derived lazily from both this configuration and
    // the incoming sample rate.  Force a rebuild after a runtime reconfigure.
    imu_filters_->samples = {};
#endif
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
