#include "hakoniwa/drone/control_adapter/px4_sensor_data_type.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <stdexcept>

using namespace hakoniwa::drone::control_adapter;

namespace {
void require(bool condition, const char* message)
{
    if (!condition) { std::cerr << message << '\n'; std::exit(EXIT_FAILURE); }
}
bool nearly_equal(double a, double b, double tolerance = 1e-8)
{
    return std::fabs(a - b) <= tolerance;
}
void test_fifo_conversion()
{
    constexpr float scale = 0.1f;
    require(nearly_equal(px4_fifo_int16(0.39, scale), 0.3, 1e-6), "positive FIFO conversion must truncate toward zero");
    require(nearly_equal(px4_fifo_int16(-0.39, scale), -0.3, 1e-6), "negative FIFO conversion must truncate toward zero");
    require(nearly_equal(px4_fifo_int16(5000.0, scale), 3276.7, 1e-3), "positive FIFO conversion must clamp");
    require(nearly_equal(px4_fifo_int16(-5000.0, scale), -3276.8, 1e-3), "negative FIFO conversion must clamp");
    EkfImuInput input{};
    input.xacc_mps2 = 0.0123; input.yacc_mps2 = -0.0123; input.zacc_mps2 = 9.81;
    input.xgyro_rad_s = 0.0019; input.ygyro_rad_s = -0.0019; input.zgyro_rad_s = 0.5;
    require(px4_imu_input(input, sensor_data_type::kExact).xacc_mps2 == input.xacc_mps2, "exact IMU must pass through");
    const auto out = px4_imu_input(input, sensor_data_type::kImuFifoInt16);
    require(nearly_equal(out.xacc_mps2, px4_imu_fifo_accel(input.xacc_mps2)), "accel x not converted");
    require(nearly_equal(out.yacc_mps2, px4_imu_fifo_accel(input.yacc_mps2)), "accel y not converted");
    require(nearly_equal(out.zacc_mps2, px4_imu_fifo_accel(input.zacc_mps2)), "accel z not converted");
    require(nearly_equal(out.xgyro_rad_s, px4_imu_fifo_gyro(input.xgyro_rad_s)), "gyro x not converted");
    require(nearly_equal(out.ygyro_rad_s, px4_imu_fifo_gyro(input.ygyro_rad_s)), "gyro y not converted");
    require(nearly_equal(out.zgyro_rad_s, px4_imu_fifo_gyro(input.zgyro_rad_s)), "gyro z not converted");
}
void test_gps_conversion()
{
    EkfHilGpsInput in{};
    in.lat_deg = 35.123456789; in.lon_deg = 139.987654321; in.alt_m = 123.45678;
    in.vn_mps = 1.239; in.ve_mps = -2.349; in.vd_mps = 0.009; in.vel_mps = 2.678;
    in.eph_m = 1.239; in.epv_m = 2.349; in.sacc_mps = 9.0;
    require(px4_gps_input(in, sensor_data_type::kExact).lat_deg == in.lat_deg, "exact GPS must pass through");
    const auto out = px4_gps_input(in, sensor_data_type::kMavlinkHilGps);
    require(out.lat_deg == static_cast<double>(static_cast<std::int32_t>(in.lat_deg * 1e7)) / 1e7, "latitude degE7 changed");
    require(out.lon_deg == static_cast<double>(static_cast<std::int32_t>(in.lon_deg * 1e7)) / 1e7, "longitude degE7 changed");
    require(out.alt_m == static_cast<double>(static_cast<std::int32_t>(in.alt_m * 1000)) / 1e3, "altitude mm changed");
    require(out.vn_mps == static_cast<double>(static_cast<float>(static_cast<std::int16_t>(in.vn_mps * 100)) / 100.0f), "north velocity changed");
    require(out.ve_mps == static_cast<double>(static_cast<float>(static_cast<std::int16_t>(in.ve_mps * 100)) / 100.0f), "east velocity changed");
    require(out.vd_mps == static_cast<double>(static_cast<float>(static_cast<std::int16_t>(in.vd_mps * 100)) / 100.0f), "down velocity changed");
    require(nearly_equal(out.vel_mps, 2.67), "ground speed must use uint16 cm/s");
    require(nearly_equal(out.eph_m, 1.23, 1e-6), "eph must use uint16 cm");
    require(nearly_equal(out.epv_m, 2.34, 1e-6), "epv must use uint16 cm");
    require(out.sacc_mps == 0.25, "sacc must match PX4 SITL");
}
void test_validation()
{
    validate_px4_sensor_data_types({sensor_data_type::kExact, sensor_data_type::kExact});
    validate_px4_sensor_data_types({sensor_data_type::kImuFifoInt16, sensor_data_type::kMavlinkHilGps});
    bool imu_rejected = false, gps_rejected = false;
    try { validate_px4_sensor_data_types({"bad", sensor_data_type::kExact}); } catch (const std::runtime_error&) { imu_rejected = true; }
    try { validate_px4_sensor_data_types({sensor_data_type::kExact, "bad"}); } catch (const std::runtime_error&) { gps_rejected = true; }
    require(imu_rejected, "unsupported IMU type accepted");
    require(gps_rejected, "unsupported GPS type accepted");
}
}
int main() { test_fifo_conversion(); test_gps_conversion(); test_validation(); return EXIT_SUCCESS; }
