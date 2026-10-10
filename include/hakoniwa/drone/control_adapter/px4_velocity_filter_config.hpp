#pragma once

namespace hakoniwa::drone::control_adapter {

// PX4 MulticopterPositionControl's velocity filters: MPC_VEL_LP on the velocity, MPC_VELD_LP on
// its time derivative, which PositionControl uses as the acceleration (the velocity D term). The
// position stages apply them only when built with HAKO_EKF_IMU_ACCELERATION; without it they use
// Drone PRO's velocity and acceleration unchanged, as before. 0 disables a filter, as in PX4.
struct Px4VelocityFilterConfig {
    double velocity_cutoff_hz{0.0};             // MPC_VEL_LP (PX4 default 0: off)
    double velocity_derivative_cutoff_hz{5.0};  // MPC_VELD_LP (PX4 default 5 Hz)
};

}  // namespace hakoniwa::drone::control_adapter
