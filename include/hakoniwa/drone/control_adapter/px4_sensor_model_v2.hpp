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
};

#ifdef HAKO_ENABLE_SENSOR_MODEL_V2
std::array<double, 3> rotate_sensor_to_body(
    const Px4SensorModelV2Config& config,
    const std::array<double, 3>& sensor_value);
#endif

}  // namespace hakoniwa::drone::control_adapter
