#include "hakoniwa/drone/control_adapter/px4_rate_control_backend.hpp"
#include "hakoniwa/drone/control_adapter/px4_sensor_data_type.hpp"
#include "px4_velocity_state_filter.hpp"

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

double rate_response(double gyro_cutoff_hz, double dgyro_cutoff_hz, bool acceleration_channel)
{
    Px4RateControlBackendConfig config{};
    config.gains.roll.p = acceleration_channel ? 0.0 : 1.0;
    config.gains.roll.d = acceleration_channel ? 1.0 : 0.0;
    config.imu_filters.gyro_cutoff_hz = gyro_cutoff_hz;
    config.imu_filters.dgyro_cutoff_hz = dgyro_cutoff_hz;
    Px4RateControlBackend backend(config);

    double sum_square = 0.0;
    constexpr int samples = 400;
    for (int i = 0; i < samples; ++i) {
        RateControlInput input{};
        const double signal = (i & 1) ? 1.0 : -1.0; // 250 Hz at a 500 Hz sample rate
        input.rate.p = acceleration_channel ? 0.0 : signal;
        input.angular_accel.p_dot = acceleration_channel ? signal : 0.0;
        input.dt_sec = 0.002;
        const double output = backend.run(input).x;
        if (i >= samples / 2) {
            sum_square += output * output;
        }
    }
    return std::sqrt(sum_square / (samples / 2));
}

void test_rate_filters()
{
    const double rate_default = rate_response(40.0, 20.0, false);
    const double accel_default = rate_response(40.0, 20.0, true);
    const double rate_disabled = rate_response(0.0, 20.0, false);
    const double accel_disabled = rate_response(40.0, 0.0, true);

#ifdef EXPECT_IMU_FILTERS
    require(rate_default < 0.2, "gyro LowPassFilter2p must attenuate high-frequency rate input");
    require(accel_default < 0.2, "dgyro AlphaFilter must attenuate high-frequency acceleration input");
    require(rate_disabled > 0.99, "zero gyro cutoff must pass rate through");
    require(accel_disabled > 0.99, "zero dgyro cutoff must pass acceleration through");
#else
    require(rate_default > 0.99, "OFF build must pass rate through");
    require(accel_default > 0.99, "OFF build must pass angular acceleration through");
    require(rate_disabled > 0.99 && accel_disabled > 0.99,
        "OFF build must remain pass-through for zero cutoffs");
#endif
}

void test_fifo_conversion_precedes_rate_filter()
{
    Px4RateControlBackendConfig quantized_config{};
    quantized_config.gains.roll.p = 1.0;
    quantized_config.gains.pitch.p = 1.0;
    quantized_config.gains.yaw.p = 1.0;
    quantized_config.imu_filters.gyro_cutoff_hz = 40.0;
    quantized_config.imu_data_type = sensor_data_type::kImuFifoInt16;
    Px4RateControlBackend quantized(quantized_config);

    Px4RateControlBackendConfig exact_config = quantized_config;
    exact_config.imu_data_type = sensor_data_type::kExact;
    Px4RateControlBackend exact_prequantized(exact_config);
    for (int i = 0; i < 40; ++i) {
        RateControlInput raw{};
        raw.rate.p = 0.123456 + i * 0.00017;
        raw.rate.q = -0.234567 - i * 0.00011;
        raw.rate.r = 0.345678 + i * 0.00013;
        raw.dt_sec = 0.002;
        RateControlInput prequantized = raw;
        prequantized.rate.p = px4_imu_fifo_gyro(raw.rate.p);
        prequantized.rate.q = px4_imu_fifo_gyro(raw.rate.q);
        prequantized.rate.r = px4_imu_fifo_gyro(raw.rate.r);
        const auto actual = quantized.run(raw);
        const auto expected = exact_prequantized.run(prequantized);
        require(std::fabs(actual.x - expected.x) < 1e-7
                && std::fabs(actual.y - expected.y) < 1e-7
                && std::fabs(actual.z - expected.z) < 1e-7,
            "IMU FIFO conversion must occur before the stateful rate filters");
    }
}

void test_yaw_torque_filter()
{
    Px4RateControlBackendConfig config{};
    config.gains.roll.p = 1.0;
    config.gains.pitch.p = 1.0;
    config.gains.yaw.p = 1.0;
    config.yaw_torque_cutoff_hz = 2.0;
    Px4RateControlBackend backend(config);

    RateControlInput input{};
    input.target = {1.0, 1.0, 1.0};
    input.dt_sec = 0.004;
    const auto first = backend.run(input);

#ifdef EXPECT_IMU_FILTERS
    constexpr double time_constant = 1.0 / (2.0 * 3.14159265358979323846 * 2.0);
    constexpr double alpha = 0.004 / (time_constant + 0.004);
    require(std::fabs(first.x - 1.0) < 1e-6 && std::fabs(first.y - 1.0) < 1e-6,
        "yaw torque filter must not alter roll or pitch");
    require(std::fabs(first.z - alpha) < 1e-6,
        "2 Hz yaw torque filter must follow PX4 AlphaFilter's first-step equation");
    const auto second = backend.run(input);
    const double expected_second = alpha + alpha * (1.0 - alpha);
    require(std::fabs(second.z - expected_second) < 1e-6,
        "yaw torque filter must retain its AlphaFilter state");
    backend.reset();
    require(std::fabs(backend.run(input).z - alpha) < 1e-6,
        "reset must clear yaw torque filter state to zero");
#else
    require(std::fabs(first.x - 1.0) < 1e-6 && std::fabs(first.y - 1.0) < 1e-6
            && std::fabs(first.z - 1.0) < 1e-6,
        "OFF build must leave every torque axis unchanged");
#endif

    Px4RateControlBackendConfig disabled_config = config;
    disabled_config.yaw_torque_cutoff_hz = 0.0;
    Px4RateControlBackend disabled(disabled_config);
    const auto disabled_output = disabled.run(input);
    require(std::fabs(disabled_output.x - 1.0) < 1e-6
            && std::fabs(disabled_output.y - 1.0) < 1e-6
            && std::fabs(disabled_output.z - 1.0) < 1e-6,
        "zero or missing MC_YAW_TQ_CUTOFF must preserve the legacy output");
}

void test_velocity_filter()
{
    Px4VelocityStateFilter filter;
    Px4VelocityFilterConfig config{};
    config.velocity_cutoff_hz = 0.0;
    config.velocity_derivative_cutoff_hz = 5.0;
    filter.configure(config);

    constexpr float dt = 0.01f;
    constexpr float ramp_acceleration = 2.0f;
    matrix::Vector3f velocity{};
    matrix::Vector3f acceleration{};
    for (int i = 0; i < 300; ++i) {
        velocity = matrix::Vector3f{ramp_acceleration * i * dt, 0.0f, 0.0f};
        acceleration = matrix::Vector3f{1234.0f, -4321.0f, 77.0f};
        filter.apply(velocity, acceleration, dt);
        require(std::fabs(velocity(0) - ramp_acceleration * i * dt) < 1e-6f,
            "MPC_VEL_LP=0 must pass velocity through");
    }
    require(std::fabs(acceleration(0) - ramp_acceleration) < 1e-3f,
        "filtered velocity derivative must converge to the ramp acceleration");
    require(std::fabs(acceleration(1)) < 1e-6f && std::fabs(acceleration(2)) < 1e-6f,
        "position acceleration must be derived from velocity, not copied from input acceleration");

    filter.reset();
    velocity = matrix::Vector3f{50.0f, -20.0f, 10.0f};
    acceleration = matrix::Vector3f{9.0f, 8.0f, 7.0f};
    filter.apply(velocity, acceleration, dt);
    require(acceleration.norm() < 1e-6f, "first velocity sample after reset must start with zero derivative");
}

} // namespace

int main()
{
    test_rate_filters();
    test_fifo_conversion_precedes_rate_filter();
    test_yaw_torque_filter();
    test_velocity_filter();
    return EXIT_SUCCESS;
}
