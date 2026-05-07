#pragma once

#include <cstdint>

struct trajectory_setpoint_s {
    std::uint64_t timestamp{0};
    float position[3]{};
    float velocity[3]{};
    float acceleration[3]{};
    float jerk[3]{};
    float yaw{0.0f};
    float yawspeed{0.0f};
};
