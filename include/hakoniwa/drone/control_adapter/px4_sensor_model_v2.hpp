#pragma once

#include <array>

namespace hakoniwa::drone::control_adapter {

struct Px4SensorModelV2Config {
    int board_rotation{0};
    double board_x_offset_deg{0.0};
    double board_y_offset_deg{0.0};
    double board_z_offset_deg{0.0};
    std::array<double, 3> imu_position_body_m{};
    std::array<double, 3> gps_position_body_m{};
    std::array<double, 3> mag_offset_gauss{};
    std::array<double, 3> mag_scale{{1.0, 1.0, 1.0}};
    std::array<double, 3> mag_offdiagonal{};
    // [SEN-BAR-325] PX4 sensors' VehicleAirData: QNH for the height, and how often the averaged pressure reaches EKF2.
    double baro_qnh_hpa{1013.25};
    double baro_rate_hz{20.0};
};

#ifdef HAKO_ENABLE_SENSOR_MODEL_V2
std::array<double, 3> rotate_sensor_to_body(
    const Px4SensorModelV2Config& config,
    const std::array<double, 3>& sensor_value);
std::array<double, 3> correct_mag_sensor_to_body(
    const Px4SensorModelV2Config& config,
    const std::array<double, 3>& sensor_value);
#endif

}  // namespace hakoniwa::drone::control_adapter
