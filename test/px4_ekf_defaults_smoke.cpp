#include "EKF/common.h"
#include "EKF/ekf.h"

#include <cmath>
#include <cstdlib>
#include <iostream>

// Read-only test access to the wrapped EKF parameter handle. Production visibility is unchanged.
#define private public
#include "hakoniwa/drone/control_adapter/px4_ekf_adapter.hpp"
#undef private

using namespace hakoniwa::drone::control_adapter;

namespace {

void require(bool condition, const char* message)
{
    if (!condition) {
        std::cerr << message << std::endl;
        std::exit(EXIT_FAILURE);
    }
}

bool nearly_equal(double left, double right, double tolerance = 1e-7)
{
    return std::fabs(left - right) <= tolerance;
}

void require_library_defaults(const parameters& actual, const parameters& expected)
{
    require(nearly_equal(actual.ekf2_delay_max, expected.ekf2_delay_max), "OFF: EKF2_DELAY_MAX changed");
    require(nearly_equal(actual.ekf2_acc_b_noise, expected.ekf2_acc_b_noise), "OFF: EKF2_ACC_B_NOISE changed");
    require(nearly_equal(actual.ekf2_gnd_eff_dz, expected.ekf2_gnd_eff_dz), "OFF: EKF2_GND_EFF_DZ changed");
    require(nearly_equal(actual.ekf2_req_hdrift, expected.ekf2_req_hdrift), "OFF: EKF2_REQ_HDRIFT changed");
    require(nearly_equal(actual.ekf2_req_vdrift, expected.ekf2_req_vdrift), "OFF: EKF2_REQ_VDRIFT changed");
    require(actual.ekf2_mag_check == expected.ekf2_mag_check, "OFF: EKF2_MAG_CHECK changed");
    require(nearly_equal(actual.ekf2_gyr_b_lim, expected.ekf2_gyr_b_lim), "OFF: EKF2_GYR_B_LIM changed");
}

void require_expected_defaults(const parameters& actual)
{
#ifdef EXPECT_PX4_EKF_DEFAULTS
    require(nearly_equal(actual.ekf2_delay_max, 200.0), "ON: EKF2_DELAY_MAX is not PX4's default");
    require(nearly_equal(actual.ekf2_acc_b_noise, 0.003), "ON: EKF2_ACC_B_NOISE is not PX4's default");
    require(nearly_equal(actual.ekf2_gnd_eff_dz, 4.0), "ON: EKF2_GND_EFF_DZ is not PX4's default");
    require(nearly_equal(actual.ekf2_req_hdrift, 0.1), "ON: EKF2_REQ_HDRIFT is not PX4's default");
    require(nearly_equal(actual.ekf2_req_vdrift, 0.2), "ON: EKF2_REQ_VDRIFT is not PX4's default");
    require(actual.ekf2_mag_check == 1, "ON: EKF2_MAG_CHECK is not PX4's default");
    require(nearly_equal(actual.ekf2_gyr_b_lim, 0.15), "ON: EKF2_GYR_B_LIM is not PX4's default");
#else
    const parameters library_defaults{};
    require_library_defaults(actual, library_defaults);
#endif
}

const parameters& params(const Px4EkfAdapter& adapter)
{
    return *adapter.ekf_->getParamHandle();
}

void initialize(Px4EkfAdapter& adapter, std::uint64_t time_usec)
{
    EkfImuInput input{};
    input.time_usec = time_usec;
    input.zacc_mps2 = -9.81;
    adapter.push_imu(input, 0.004);
}

} // namespace

int main()
{
    EkfAdapterConfig config{};
    config.px4_params.ekf2_req_eph = 42.0;
    Px4EkfAdapter adapter(config);

    require(adapter.sensor_data_types_.imu == sensor_data_type::kExact
            && adapter.sensor_data_types_.gps == sensor_data_type::kExact,
        "EKF sensor data types must default to exact");
    adapter.set_sensor_data_types(
        {sensor_data_type::kImuFifoInt16, sensor_data_type::kMavlinkHilGps});
    require(adapter.sensor_data_types_.imu == sensor_data_type::kImuFifoInt16
            && adapter.sensor_data_types_.gps == sensor_data_type::kMavlinkHilGps,
        "EKF did not retain configured sensor data types");
    bool unsupported_rejected = false;
    try { adapter.set_sensor_data_types({"bad", sensor_data_type::kExact}); }
    catch (const std::runtime_error&) { unsupported_rejected = true; }
    require(unsupported_rejected, "EKF accepted an unsupported sensor data type");

    require_expected_defaults(params(adapter));
    initialize(adapter, 1'000'000);
    require(nearly_equal(params(adapter).ekf2_req_eph, 42.0),
        "configured EKF value must override the defaults during initialization");
    require_expected_defaults(params(adapter));

    adapter.reset();
    require_expected_defaults(params(adapter));
    initialize(adapter, 2'000'000);
    require(nearly_equal(params(adapter).ekf2_req_eph, 42.0),
        "configured EKF value must be restored after reset and initialization");
    require_expected_defaults(params(adapter));
    return EXIT_SUCCESS;
}
