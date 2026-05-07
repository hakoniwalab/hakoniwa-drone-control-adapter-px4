#pragma once

#include <cstdint>

struct vehicle_attitude_setpoint_s {
    std::uint32_t MESSAGE_VERSION{1};
    std::uint64_t timestamp{0};
    float yaw_sp_move_rate{0.0f};
    float q_d[4]{1.0f, 0.0f, 0.0f, 0.0f};
    float thrust_body[3]{0.0f, 0.0f, 0.0f};
};
