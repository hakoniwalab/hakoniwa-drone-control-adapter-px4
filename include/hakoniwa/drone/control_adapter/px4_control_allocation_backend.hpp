#pragma once

#include "hakoniwa/drone/control_adapter/control_allocation_backend.hpp"

class ControlAllocationSequentialDesaturation;

namespace hakoniwa::drone::control_adapter {

// Actuator values in ControlAllocationInput (limit, trim, linearization_point)
// and ControlAllocationOutput::actuator_commands are normalized rotor thrust:
// 1.0 means one rotor producing its share of hover thrust.  Converting that
// to a plant command (e.g. PWM duty) is the caller's responsibility; see
// docs/px4-controller-config.md "Thrust Normalization Contract".
//
// command.torque_x/y/z is the PX4 RateControl output: a normalized control
// demand in PX4 actuator units (1.0 = one rotor at maximum thrust).  The
// allocator (PX4 sequential desaturation) runs in those units; hover_thrust
// (MPC_THR_HOVER = T_hover / T_max) converts thrust, limits and outputs
// between them and the hover units above.  docs/px4-controller-config.md
// "Torque Normalization Contract".
struct Px4ControlAllocationBackendConfig {
    bool normalize_rpy{true};
    bool metric_allocation{false};
    bool update_normalization_scale{true};
    double hover_thrust{0.0};  // MPC_THR_HOVER, required: (0, 1]
    // MC_AIRMODE for PX4's sequential desaturation: 0 disabled (PX4 default),
    // 1 roll/pitch, 2 roll/pitch/yaw.  PX4 parameters are process-global, so
    // backends in one process must use the same value.
    int airmode{0};
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
    ::ControlAllocationSequentialDesaturation* controller_{nullptr};
};

}  // namespace hakoniwa::drone::control_adapter
