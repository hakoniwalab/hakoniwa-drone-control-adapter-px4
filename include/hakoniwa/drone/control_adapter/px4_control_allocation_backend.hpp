#pragma once

#include "hakoniwa/drone/control_adapter/control_allocation_backend.hpp"

class ControlAllocationPseudoInverse;

namespace hakoniwa::drone::control_adapter {

struct Px4ControlAllocationBackendConfig {
    bool normalize_rpy{true};
    bool metric_allocation{false};
    bool update_normalization_scale{true};
    double hover_duty{0.120311};
};

class Px4ControlAllocationBackend final : public IControlAllocationBackend {
public:
    explicit Px4ControlAllocationBackend(const Px4ControlAllocationBackendConfig& config);
    ~Px4ControlAllocationBackend() override;

    void reset() override;
    ControlAllocationOutput run(const ControlAllocationInput& input) override;

    void set_config(const Px4ControlAllocationBackendConfig& config);

private:
    void apply_config();

    Px4ControlAllocationBackendConfig config_{};
    ::ControlAllocationPseudoInverse* controller_{nullptr};
};

}  // namespace hakoniwa::drone::control_adapter
