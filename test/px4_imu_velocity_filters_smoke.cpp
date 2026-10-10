#include "hakoniwa/drone/control_adapter/px4_rate_control_backend.hpp"
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
    test_velocity_filter();
    return EXIT_SUCCESS;
}
