#pragma once

#include "hakoniwa/drone/control_adapter/rate_control_backend.hpp"

class RateControl;

namespace hakoniwa::drone::control_adapter {

struct Px4RateControlAxisGains {
    double p{0.0};
    double i{0.0};
    double d{0.0};
};

struct Px4RateControlGains {
    Px4RateControlAxisGains roll{};
    Px4RateControlAxisGains pitch{};
    Px4RateControlAxisGains yaw{};
};

struct Px4RateControlLimits {
    double roll_integrator{0.0};
    double pitch_integrator{0.0};
    double yaw_integrator{0.0};
};

struct Px4RateControlFeedForward {
    double roll{0.0};
    double pitch{0.0};
    double yaw{0.0};
};

// PX4's IMU filters (sensors/vehicle_angular_velocity): IMU_GYRO_CUTOFF, the 2nd-order low-pass
// on the angular rate, and IMU_DGYRO_CUTOFF, the 1st-order low-pass on the angular acceleration
// (the D-term input). The adapter applies them only when built with HAKO_EKF_IMU_ACCELERATION,
// where Drone PRO hands it the IMU's rate and the gyro's raw derivative; without it the inputs pass
// through unchanged. 0 disables a filter, as in PX4.
struct Px4ImuFilterConfig {
    double gyro_cutoff_hz{40.0};
    double dgyro_cutoff_hz{20.0};
};

struct Px4RateControlBackendConfig {
    Px4RateControlGains gains{};
    Px4RateControlLimits integrator_limits{};
    Px4RateControlFeedForward feed_forward{};
    Px4ImuFilterConfig imu_filters{};
};

struct Px4RateControlBackendStatus {
    double roll_integral{0.0};
    double pitch_integral{0.0};
    double yaw_integral{0.0};
};

class Px4RateControlBackend final : public IRateControlBackend {
public:
    explicit Px4RateControlBackend(const Px4RateControlBackendConfig& config);
    ~Px4RateControlBackend() override;

    void reset() override;
    BodyTorqueCommand run(const RateControlInput& input) override;

    void set_config(const Px4RateControlBackendConfig& config);
    Px4RateControlBackendStatus get_status() const;

private:
    void apply_config();
    static bool is_valid_dt(double dt_sec);

    struct ImuFilters;

    Px4RateControlBackendConfig config_{};
    ::RateControl* controller_{nullptr};
    ImuFilters* imu_filters_{nullptr};
};

}  // namespace hakoniwa::drone::control_adapter
