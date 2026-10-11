#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>

#include "hakoniwa/drone/control_adapter/ekf_adapter.hpp"
#include "hakoniwa/drone/control_adapter/sensor_data_type.hpp"

namespace hakoniwa::drone::control_adapter {

// The PX4 adapter's sensor data types (sensor_data_type.hpp names what they are; this is what they mean
// for PX4 SITL). Selected in the PX4 controller config, "sensor_data_types": {"imu": ..., "gps": ...};
// absent: exact, as before.
struct Px4SensorDataTypes {
    std::string imu{sensor_data_type::kExact};
    std::string gps{sensor_data_type::kExact};
};

inline void validate_px4_sensor_data_types(const Px4SensorDataTypes& types)
{
    if (types.imu != sensor_data_type::kExact && types.imu != sensor_data_type::kImuFifoInt16) {
        throw std::runtime_error("sensor_data_types.imu: unsupported by the PX4 adapter: " + types.imu);
    }
    if (types.gps != sensor_data_type::kExact && types.gps != sensor_data_type::kMavlinkHilGps) {
        throw std::runtime_error("sensor_data_types.gps: unsupported by the PX4 adapter: " + types.gps);
    }
}

// imu_fifo_int16: PX4 SITL (SimulatorMavlink) stores each HIL_SENSOR gyro and accelerometer value in its
// simulated FIFO as value / scale in an int16 (float-to-integer conversion: truncation toward zero), and
// PX4Gyroscope/PX4Accelerometer report it times the scale.
inline constexpr float kPx4GyroFifoScale = 2000.f / 32768.f * 3.14159265358979323846f / 180.f;  // rad/s
inline constexpr float kPx4AccelFifoScale = 9.80665f / 2048.f;                                  // m/s^2

inline double px4_fifo_int16(double value, float scale)
{
    const float raw = std::clamp(static_cast<float>(value) / scale, -32768.f, 32767.f);
    return static_cast<double>(static_cast<float>(static_cast<std::int16_t>(raw)) * scale);
}

inline double px4_imu_fifo_gyro(double rad_s) { return px4_fifo_int16(rad_s, kPx4GyroFifoScale); }
inline double px4_imu_fifo_accel(double mps2) { return px4_fifo_int16(mps2, kPx4AccelFifoScale); }

inline EkfImuInput px4_imu_input(const EkfImuInput& input, const std::string& type)
{
    if (type != sensor_data_type::kImuFifoInt16) {
        return input;
    }
    EkfImuInput out = input;
    out.xacc_mps2 = px4_imu_fifo_accel(input.xacc_mps2);
    out.yacc_mps2 = px4_imu_fifo_accel(input.yacc_mps2);
    out.zacc_mps2 = px4_imu_fifo_accel(input.zacc_mps2);
    out.xgyro_rad_s = px4_imu_fifo_gyro(input.xgyro_rad_s);
    out.ygyro_rad_s = px4_imu_fifo_gyro(input.ygyro_rad_s);
    out.zgyro_rad_s = px4_imu_fifo_gyro(input.zgyro_rad_s);
    return out;
}

// mavlink_hil_gps: the fields as the Hakoniwa SITL service encodes HIL_GPS (static_cast: truncation
// toward zero) and PX4 SITL decodes it (SimulatorMavlink::handle_message_hil_gps). HIL_GPS has no speed
// accuracy: PX4 SITL sets s_variance_m_s 0.25. eph/epv follow the MAVLink unit (cm).
inline EkfHilGpsInput px4_gps_input(const EkfHilGpsInput& input, const std::string& type)
{
    if (type != sensor_data_type::kMavlinkHilGps) {
        return input;
    }
    EkfHilGpsInput out = input;
    out.lat_deg = static_cast<std::int32_t>(input.lat_deg * 1e7) / 1e7;
    out.lon_deg = static_cast<std::int32_t>(input.lon_deg * 1e7) / 1e7;
    out.alt_m = static_cast<std::int32_t>(input.alt_m * 1000) / 1e3;
    const auto cm_s = [](double mps) {
        return static_cast<double>(static_cast<float>(static_cast<std::int16_t>(mps * 100)) / 100.0f);
    };
    out.vn_mps = cm_s(input.vn_mps);
    out.ve_mps = cm_s(input.ve_mps);
    out.vd_mps = cm_s(input.vd_mps);
    out.vel_mps = input.vel_mps >= 0 ? static_cast<std::uint16_t>(input.vel_mps * 100) / 100.0 : input.vel_mps;
    const auto cm = [](double m) {
        return m >= 0 ? static_cast<double>(static_cast<float>(static_cast<std::uint16_t>(m * 100)) * 1e-2f) : m;
    };
    out.eph_m = cm(input.eph_m);
    out.epv_m = cm(input.epv_m);
    out.sacc_mps = 0.25;
    return out;
}

}  // namespace hakoniwa::drone::control_adapter
