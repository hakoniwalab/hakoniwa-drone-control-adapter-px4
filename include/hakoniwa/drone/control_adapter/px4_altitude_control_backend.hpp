#pragma once

#include "hakoniwa/drone/control_adapter/altitude_control_backend.hpp"

class PositionControl;

namespace hakoniwa::drone::control_adapter {

struct Px4AltitudeControlBackendConfig {
    double position_gain_z{0.0};
    double velocity_p_z{0.0};
    double velocity_i_z{0.0};
    double velocity_d_z{0.0};
    double velocity_max_up_mps{0.0};
    double velocity_max_down_mps{0.0};
    double hover_thrust{0.5};
    double thrust_min{0.0};
    double thrust_max{1.0};
};

class Px4AltitudeControlBackend final : public IAltitudeControlBackend {
public:
    explicit Px4AltitudeControlBackend(const Px4AltitudeControlBackendConfig& config);
    ~Px4AltitudeControlBackend() override;

    void reset() override;
    NormalizedVerticalThrustCommand run(const AltitudeControlInput& input, double dt_sec) override;

    void set_config(const Px4AltitudeControlBackendConfig& config);

private:
    void apply_config();

    Px4AltitudeControlBackendConfig config_{};
    ::PositionControl* controller_{nullptr};
};

}  // namespace hakoniwa::drone::control_adapter
