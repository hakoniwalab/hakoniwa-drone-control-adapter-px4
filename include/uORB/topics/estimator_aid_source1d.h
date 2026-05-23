#pragma once

#include <cstdint>

struct estimator_aid_source1d_s {
    std::uint64_t timestamp{0};
    std::uint64_t timestamp_sample{0};
    std::uint8_t estimator_instance{0};
    std::uint32_t device_id{0};
    std::uint64_t time_last_fuse{0};
    float observation{0.0f};
    float observation_variance{0.0f};
    float innovation{0.0f};
    float innovation_filtered{0.0f};
    float innovation_variance{0.0f};
    float test_ratio{0.0f};
    float test_ratio_filtered{0.0f};
    bool innovation_rejected{false};
    bool fused{false};
};
