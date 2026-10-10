#pragma once

#include "hakoniwa/drone/control_adapter/px4_velocity_filter_config.hpp"
#include "hakoniwa/drone/control_adapter/position_control_3d_backend.hpp"

class PositionControl;
struct trajectory_setpoint_s;

namespace hakoniwa::drone::control_adapter {

class Px4VelocityStateFilter;

// PX4 Auto trajectory (FlightTaskAuto::_updateTrajConstraints, _smoothYaw); PX4's defaults.
// Used only in a HAKO_EKF_IMU_ACCELERATION build, and only when enabled in the PX4 controller config
// (position_control.trajectory_generation 1, set by the PX4 SITL timing profile).
struct Px4PositionTrajectoryConfig {
    bool enabled{false};

    double cruise_speed_mps{5.0};          // MPC_XY_CRUISE
    double acceleration_xy_mps2{3.0};      // MPC_ACC_HOR
    double jerk_mps3{4.0};                 // MPC_JERK_AUTO
    double trajectory_gain_xy{0.5};        // MPC_XY_TRAJ_P
    double max_horizontal_error_m{2.0};    // MPC_XY_ERR_MAX
    double vertical_acceptance_m{0.8};     // NAV_MC_ALT_RAD
    double target_acceptance_m{10.0};      // NAV_ACC_RAD
    double acceleration_up_mps2{4.0};      // MPC_ACC_UP_MAX
    double acceleration_down_mps2{3.0};    // MPC_ACC_DOWN_MAX
    double velocity_up_mps{3.0};           // MPC_Z_V_AUTO_UP
    double velocity_down_mps{1.5};         // MPC_Z_V_AUTO_DN
    double yaw_rate_max_deg_s{60.0};       // MPC_YAWRAUTO_MAX
    double yaw_acceleration_max_deg_s2{20.0};  // MPC_YAWRAUTO_ACC
};

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
    Px4VelocityFilterConfig velocity_filter{};
    Px4PositionTrajectoryConfig trajectory{};
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
#ifdef HAKO_EKF_IMU_ACCELERATION
    std::optional<double> apply_trajectory(
        const PositionControl3DPositionInput& input,
        float dt_sec,
        ::trajectory_setpoint_s& setpoint);
#endif

    Px4PositionControl3DBackendConfig config_{};
    ::PositionControl* controller_{nullptr};
    Px4VelocityStateFilter* velocity_filter_{nullptr};
    struct TrajectoryState;
    TrajectoryState* trajectory_{nullptr};  // HAKO_EKF_IMU_ACCELERATION build only
};

}  // namespace hakoniwa::drone::control_adapter
