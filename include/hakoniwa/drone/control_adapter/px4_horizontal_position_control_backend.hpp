#pragma once

#include "hakoniwa/drone/control_adapter/horizontal_position_control_backend.hpp"

class PositionControl;

namespace hakoniwa::drone::control_adapter {

struct Px4HorizontalPositionControlBackendConfig {
    double position_gain_xy{0.0};
    double velocity_p_xy{0.0};
    double velocity_i_xy{0.0};
    double velocity_d_xy{0.0};
    double velocity_max_xy_mps{0.0};
    double tilt_limit_rad{0.0};
    double hover_thrust{0.5};
    double thrust_min{0.0};
    double thrust_max{1.0};
    double horizontal_thrust_margin{0.0};
    bool decouple_horizontal_and_vertical_acceleration{true};
};

class Px4HorizontalPositionControlBackend final : public IHorizontalPositionControlBackend {
public:
    explicit Px4HorizontalPositionControlBackend(const Px4HorizontalPositionControlBackendConfig& config);
    ~Px4HorizontalPositionControlBackend() override;

    void reset() override;
    HorizontalTiltTarget run(const HorizontalPositionControlInput& input, double dt_sec) override;

    void set_config(const Px4HorizontalPositionControlBackendConfig& config);

private:
    void apply_config();

    Px4HorizontalPositionControlBackendConfig config_{};
    ::PositionControl* controller_{nullptr};
};

}  // namespace hakoniwa::drone::control_adapter
