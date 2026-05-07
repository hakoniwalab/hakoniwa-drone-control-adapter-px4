#pragma once

#include <cstdint>

struct vehicle_local_position_setpoint_s {
    std::uint64_t timestamp{0};
    float x{0.0f};
    float y{0.0f};
    float z{0.0f};
    float vx{0.0f};
    float vy{0.0f};
    float vz{0.0f};
    float acceleration[3]{0.0f, 0.0f, 0.0f};
    float thrust[3]{0.0f, 0.0f, 0.0f};
    float yaw{0.0f};
    float yawspeed{0.0f};
};
