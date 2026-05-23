#pragma once

#include <cstdint>

struct estimator_aid_source2d_s {
    std::uint64_t timestamp{0};
    std::uint64_t timestamp_sample{0};
    std::uint8_t estimator_instance{0};
    std::uint32_t device_id{0};
    std::uint64_t time_last_fuse{0};
    double observation[2]{0.0, 0.0};
    float observation_variance[2]{0.0f, 0.0f};
    float innovation[2]{0.0f, 0.0f};
    float innovation_filtered[2]{0.0f, 0.0f};
    float innovation_variance[2]{0.0f, 0.0f};
    float test_ratio[2]{0.0f, 0.0f};
    float test_ratio_filtered[2]{0.0f, 0.0f};
    bool innovation_rejected{false};
    bool fused{false};
};
