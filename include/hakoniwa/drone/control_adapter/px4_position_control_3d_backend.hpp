#pragma once

#include <cstdint>
#include <optional>
#include <vector>

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
    // PX4's FlightModeManager takes the vehicle position at most every 20 ms (set_interval_us(20_ms)), so
    // the trajectory setpoint is new only every second or third position-control cycle and held in
    // between; the position error then grows by speed x cycle until the next update. 0: every cycle.
    // position_control.trajectory_update_interval_ms. The updates follow the position updates (every
    // position-control cycle) from the stage's first cycle on, and a new target or a restarted trajectory
    // does not reset them (FlightModeManager keeps running across tasks and commands).
    double update_interval_sec{0.0};
    // An option for imitating PX4 SITL on POSIX (not a model of PX4 on NuttX): the navigator runs as its own
    // task, so a new reposition reaches FlightModeManager 4, 8 or 12 ms after PX4 received it (x500 SITL,
    // 30 commands: 11 / 7 / 12, independent of the position update phase, 2026-10-11). A new position
    // target takes effect after one of these delays (ms), picked per target by a generator seeded with
    // navigator_delay_seed (independent of the sensor noise seed); one value is a fixed delay. Empty: none.
    // position_control.navigator_delay_ms (a number or a list), navigator_delay_seed.
    std::vector<double> navigator_delay_ms{};
    std::uint32_t navigator_delay_seed{0};
    // PX4's automatic takeoff (position_control.takeoff_profile 1; an option for the PX4 SITL comparison,
    // off: the takeoff target is tracked as any position target, as before). A trajectory that starts at
    // rest under a target above it is a takeoff, until the target changes. As PX4: COM_SPOOLUP_TIME with
    // no thrust; then mc_pos_control's TakeoffHandling ramps the upward velocity limit from -g/MPC_Z_VEL_P_ACC
    // to the climb limit over MPC_TKO_RAMP_T (no thrust floor, MPC_TILTMAX_LND) until flight; FlightTaskAuto
    // climbs at most MPC_TKO_SPEED with min(MPC_ACC_UP_MAX, MPC_TKO_SPEED/MPC_TKO_RAMP_T) below MPC_LAND_ALT1,
    // holding the altitude setpoint at the vehicle's, until the vehicle is within NAV_MC_ALT_RAD of the takeoff
    // height (navigator completes the takeoff and loiters there); until the ramp-up FlightModeManager re-activates the
    // task every update (FlightTaskAuto::reActivate: the trajectory restarts at the vehicle's position with
    // 0.7 m/s down). HAKO_EKF_IMU_ACCELERATION build only.
    bool takeoff_profile{false};
    double takeoff_speed_mps{1.5};           // MPC_TKO_SPEED
    double takeoff_ramp_sec{3.0};            // MPC_TKO_RAMP_T
    double spoolup_sec{1.0};                 // COM_SPOOLUP_TIME
    double land_alt1_m{10.0};                // MPC_LAND_ALT1
    double tilt_max_land_rad{0.20943951};    // MPC_TILTMAX_LND (12 deg)
    // HAKO_ENABLE_FLIGHT_PHASE: the caller's phase starts the takeoff above (no inference from the target once
    // the caller has sent a phase), and a Landing phase descends as FlightTaskAuto::_prepareLandSetpoints
    // (position_control.landing_profile 1; off: the landing target is tracked as any position target). The
    // speed is interpolated between MPC_LAND_SPEED at MPC_LAND_ALT2 and MPC_Z_V_AUTO_DN at MPC_LAND_ALT1 by the
    // height above the takeoff point, shaped within MPC_ACC_DOWN_MAX and MPC_JERK_AUTO; the horizontal target
    // is held. HAKO_EKF_IMU_ACCELERATION build only.
    bool landing_profile{false};
    double land_speed_mps{0.7};              // MPC_LAND_SPEED
    double land_alt2_m{5.0};                 // MPC_LAND_ALT2
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
    // PX4's manual heading lock (StickYaw::updateYawLock): a velocity target without a yaw holds the
    // heading at which the yaw stick came back to 0, and follows the vehicle while the stick turns it.
    // Off: the heading setpoint is the vehicle's own (no hold). HAKO_EKF_IMU_ACCELERATION build only,
    // enabled by position_control.manual_yaw_lock 1 (set by the PX4 SITL timing profile).
    bool manual_yaw_lock{false};
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
    void apply_takeoff(::trajectory_setpoint_s& setpoint, double dt_sec);
#ifdef HAKO_ENABLE_FLIGHT_PHASE
    void apply_landing(PositionControl3DPositionInput& input, double dt_sec);
#endif
#endif

    Px4PositionControl3DBackendConfig config_{};
    ::PositionControl* controller_{nullptr};
    Px4VelocityStateFilter* velocity_filter_{nullptr};
    struct TrajectoryState;
    TrajectoryState* trajectory_{nullptr};  // HAKO_EKF_IMU_ACCELERATION build only
    void advance_trajectory_clock(double dt_sec);
    bool yaw_locked_{false};                 // manual_yaw_lock: a heading is held
    double locked_yaw_rad_{0.0};
};

}  // namespace hakoniwa::drone::control_adapter
