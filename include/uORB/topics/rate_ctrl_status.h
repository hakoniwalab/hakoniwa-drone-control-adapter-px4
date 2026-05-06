#pragma once

#include <cstdint>

struct rate_ctrl_status_s {
    std::uint64_t timestamp{0};
    float rollspeed_integ{0.0f};
    float pitchspeed_integ{0.0f};
    float yawspeed_integ{0.0f};
};
