#include "hakoniwa/drone/control_adapter/px4_rate_control_backend.hpp"

#include <lib/mathlib/math/filter/AlphaFilter.hpp>
#include <lib/mathlib/math/filter/LowPassFilter2p.hpp>
#include <lib/mathlib/math/filter/NotchFilter.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <iostream>

using namespace hakoniwa::drone::control_adapter;

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDt = 1.0 / 8000.0;

void require(bool condition, const char* message)
{
    if (!condition) {
        std::cerr << message << std::endl;
        std::exit(EXIT_FAILURE);
    }
}

double sine_response(bool dynamic_notch_enabled)
{
    Px4RateControlBackendConfig config{};
    config.gains.roll.p = 1.0;
    config.imu_filters.gyro_cutoff_hz = 3000.0;
    config.imu_filters.dgyro_cutoff_hz = 0.0;
    config.imu_filters.dynamic_notch_enable = dynamic_notch_enabled ? 1 : 0;
    config.imu_filters.dynamic_notch_harmonics = 3;
    config.imu_filters.dynamic_notch_bandwidth_hz = 15.0;
    config.imu_filters.dynamic_notch_min_frequency_hz = 25.0;
    Px4RateControlBackend backend(config);

    double rotor = 93.0 * 2.0 * kPi;
    double sum_square = 0.0;
    constexpr int sample_count = 16000;
    for (int i = 0; i < sample_count; ++i) {
        GyroSample sample{
            static_cast<std::uint64_t>(i * 125),
            std::sin(2.0 * kPi * 93.0 * i * kDt),
            0.0,
            0.0};
        RateControlInput input{};
        input.rate = {sample.p, 0.0, 0.0};
        input.dt_sec = kDt;
        input.gyro_samples = &sample;
        input.gyro_sample_count = 1;
        input.rotor_rad_per_sec = &rotor;
        input.rotor_count = 1;
        const double output = backend.run(input).x;
        if (i >= sample_count / 2) {
            sum_square += output * output;
        }
    }
    return std::sqrt(sum_square / (sample_count / 2));
}

void test_dynamic_notch_attenuates_rotor_frequency()
{
    const double enabled = sine_response(true);
    const double disabled = sine_response(false);
    require(disabled > 0.65, "disabled dynamic notch must leave the 93 Hz sine substantially intact");
    require(enabled < disabled * 0.25, "enabled ESC dynamic notch must attenuate the 93 Hz sine");
}

struct Px4EscNotchReference {
    std::array<math::NotchFilter<float>, 2> dynamic_notch{};
    math::NotchFilter<float> static_notch0{};
    math::NotchFilter<float> static_notch1{};
    math::LowPassFilter2p<float> lowpass{};
    AlphaFilter<float> dgyro{};
    float previous_filtered{0.0f};
    bool rotor_available{false};
    bool started{false};

    std::array<float, 2> run(float raw, float rotor_hz)
    {
        bool axis_init = false;
        if (rotor_hz == 0.0f) {
            rotor_available = false;
        } else {
            const bool force_update = !rotor_available;
            constexpr float bandwidth_hz = 15.0f;
            constexpr float freq_min = 25.0f;
            for (std::size_t harmonic = 0; harmonic < dynamic_notch.size(); ++harmonic) {
                const float frequency_hz = std::max(
                    rotor_hz * static_cast<float>(harmonic + 1),
                    freq_min + static_cast<float>(harmonic) * 0.5f * bandwidth_hz);
                auto& notch = dynamic_notch[harmonic];
                const float notch_freq_delta = std::fabs(notch.getNotchFreq() - frequency_hz);
                const bool notch_freq_changed = notch_freq_delta > 0.1f;
                const bool allow_update = !axis_init
                    || (notch.initialized() && notch_freq_delta < notch.getBandwidth());
                if ((force_update || notch_freq_changed) && allow_update
                    && notch.setParameters(8000.0f, frequency_hz, bandwidth_hz)
                    && !notch.initialized()) {
                    axis_init = true;
                }
            }
            rotor_available = true;
        }

        if (!started) {
            static_notch0.setParameters(8000.0f, 71.0f, 18.0f);
            static_notch0.reset(raw);
            static_notch1.setParameters(8000.0f, 113.0f, 22.0f);
            static_notch1.reset(raw);
            lowpass.set_cutoff_frequency(8000.0f, 300.0f);
            lowpass.reset(raw);
            dgyro.setCutoffFreq(8000.0f, 100.0f);
            dgyro.reset(0.0f);
            previous_filtered = raw;
            started = true;
        }

        float filtered = raw;
        if (rotor_available) {
            for (auto& notch : dynamic_notch) {
                if (notch.getNotchFreq() > 0.0f) {
                    filtered = notch.apply(filtered);
                }
            }
        }
        if (static_notch0.getNotchFreq() > 0.0f) {
            filtered = static_notch0.apply(filtered);
        }
        if (static_notch1.getNotchFreq() > 0.0f) {
            filtered = static_notch1.apply(filtered);
        }
        filtered = lowpass.apply(filtered);
        const float derivative = (filtered - previous_filtered) / static_cast<float>(kDt);
        const float filtered_derivative = dgyro.update(derivative);
        previous_filtered = filtered;
        return {filtered, filtered_derivative};
    }
};

void test_pipeline_matches_px4_dynamic_notch_update_reference()
{
    Px4RateControlBackendConfig config{};
    config.gains.roll.p = 0.2;
    config.gains.roll.d = 0.01;
    config.imu_filters.gyro_notch0_frequency_hz = 71.0;
    config.imu_filters.gyro_notch0_bandwidth_hz = 18.0;
    config.imu_filters.gyro_notch1_frequency_hz = 113.0;
    config.imu_filters.gyro_notch1_bandwidth_hz = 22.0;
    config.imu_filters.dynamic_notch_enable = 1;
    config.imu_filters.dynamic_notch_harmonics = 2;
    config.imu_filters.dynamic_notch_bandwidth_hz = 15.0;
    config.imu_filters.dynamic_notch_min_frequency_hz = 25.0;
    config.imu_filters.gyro_cutoff_hz = 300.0;
    config.imu_filters.dgyro_cutoff_hz = 100.0;
    Px4RateControlBackend backend(config);
    Px4EscNotchReference reference{};

    for (int i = 0; i < 600; ++i) {
        const float raw = static_cast<float>(
            0.30 * std::sin(2.0 * kPi * 25.0 * i * kDt)
            + 0.25 * std::sin(2.0 * kPi * 32.5 * i * kDt)
            + 0.35 * std::sin(2.0 * kPi * 71.0 * i * kDt)
            + 0.20 * std::sin(2.0 * kPi * 93.0 * i * kDt)
            + 0.15 * std::sin(2.0 * kPi * 113.0 * i * kDt));

        float rotor_hz = 5.0f;  // below MIN: park H1/H2 at 25/32.5 Hz
        if (i >= 80 && i < 100) {
            rotor_hz = 0.0f;  // unavailable: do not apply this rotor's notches
        } else if (i >= 100 && i < 360) {
            rotor_hz = 60.0f + static_cast<float>(i - 100) * 0.25f;  // moving centers
        } else if (i >= 360 && i < 390) {
            rotor_hz = 0.0f;
        } else if (i >= 390) {
            rotor_hz = 93.0f;  // 0 -> nonzero: force update, with PX4 allow_update gating
        }

        const auto expected_filter = reference.run(raw, rotor_hz);

        GyroSample sample{static_cast<std::uint64_t>(i * 125), raw, 0.0, 0.0};
        double rotor = static_cast<double>(rotor_hz) * 2.0 * kPi;
        RateControlInput input{};
        input.rate.p = raw;
        input.dt_sec = kDt;
        input.gyro_samples = &sample;
        input.gyro_sample_count = 1;
        input.rotor_rad_per_sec = &rotor;
        input.rotor_count = 1;
        const double expected = -0.2 * expected_filter[0] - 0.01 * expected_filter[1];
        const double actual = backend.run(input).x;
        require(std::fabs(actual - expected) < 2e-5,
            "gyro sample pipeline must match PX4 ESC notch order, parking, and update gating");
    }
}

void test_zero_samples_preserves_legacy_path()
{
    Px4RateControlBackendConfig config{};
    config.gains.roll = {0.15, 0.10, 0.01};
    config.imu_filters.gyro_cutoff_hz = 40.0;
    config.imu_filters.dgyro_cutoff_hz = 20.0;
    Px4RateControlBackend baseline(config);
    Px4RateControlBackend zero_samples(config);
    GyroSample unused{};
    for (int i = 0; i < 40; ++i) {
        RateControlInput a{};
        a.rate.p = 0.2 * std::sin(i * 0.3);
        a.angular_accel.p_dot = 0.4 * std::cos(i * 0.3);
        a.target.p = 0.1;
        a.dt_sec = 0.002;
        RateControlInput b = a;
        b.gyro_samples = &unused;
        b.gyro_sample_count = 0;
        const auto expected = baseline.run(a);
        const auto actual = zero_samples.run(b);
        require(expected.x == actual.x && expected.y == actual.y && expected.z == actual.z,
            "zero gyro samples must use the byte-identical legacy path");
    }
}

}  // namespace

int main()
{
    test_dynamic_notch_attenuates_rotor_frequency();
    test_pipeline_matches_px4_dynamic_notch_update_reference();
    test_zero_samples_preserves_legacy_path();
    return EXIT_SUCCESS;
}
