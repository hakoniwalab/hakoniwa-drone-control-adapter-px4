#pragma once

#include "hakoniwa/drone/control_adapter/position_control_3d_backend.hpp"

class PositionControl;

namespace hakoniwa::drone::control_adapter {

struct Px4PositionControl3DBackendConfig {
    double position_gain_xy{0.0};
    double position_gain_z{0.0};

    double velocity_p_xy{0.0};
    double velocity_i_xy{0.0};
    double velocity_d_xy{0.0};

    double velocity_p_z{0.0};
    double velocity_i_z{0.0};
    double velocity_d_z{0.0};

    double velocity_max_xy_mps{0.0};
    double velocity_max_up_mps{0.0};
    double velocity_max_down_mps{0.0};

    double tilt_limit_rad{0.0};
    double hover_thrust{0.5};
    double thrust_min{0.0};
    double thrust_max{1.0};
    double horizontal_thrust_margin{0.0};
    bool decouple_horizontal_and_vertical_acceleration{false};
};

class Px4PositionControl3DBackend final : public IPositionControl3DBackend {
public:
    explicit Px4PositionControl3DBackend(const Px4PositionControl3DBackendConfig& config);
    ~Px4PositionControl3DBackend() override;

    void reset() override;

    PositionControl3DOutput run_position(
        const PositionControl3DPositionInput& input,
        double dt_sec) override;

    PositionControl3DOutput run_velocity(
        const PositionControl3DVelocityInput& input,
        double dt_sec) override;

    void set_config(const Px4PositionControl3DBackendConfig& config);

private:
    void apply_config();

    Px4PositionControl3DBackendConfig config_{};
    ::PositionControl* controller_{nullptr};
};

}  // namespace hakoniwa::drone::control_adapter
