#pragma once

#include "hakoniwa/drone/control_adapter/attitude_control_backend.hpp"

class AttitudeControl;

namespace hakoniwa::drone::control_adapter {

struct Px4AttitudeControlAxisGains {
    double roll{0.0};
    double pitch{0.0};
    double yaw{0.0};
};

struct Px4AttitudeControlRateLimits {
    double roll_rad_sec{0.0};
    double pitch_rad_sec{0.0};
    double yaw_rad_sec{0.0};
};

struct Px4AttitudeControlBackendConfig {
    Px4AttitudeControlAxisGains proportional_gains{};
    double yaw_weight{0.0};
    Px4AttitudeControlRateLimits rate_limits{};
};

class Px4AttitudeControlBackend final : public IAttitudeControlBackend {
public:
    explicit Px4AttitudeControlBackend(const Px4AttitudeControlBackendConfig& config);
    ~Px4AttitudeControlBackend() override;

    void reset() override;
    AngularRateTarget run(const AttitudeControlInput& input) override;

    void set_config(const Px4AttitudeControlBackendConfig& config);

private:
    void apply_config();

    Px4AttitudeControlBackendConfig config_{};
    ::AttitudeControl* controller_{nullptr};
};

}  // namespace hakoniwa::drone::control_adapter
