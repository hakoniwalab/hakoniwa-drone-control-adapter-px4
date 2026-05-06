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

struct Px4RateControlBackendConfig {
    Px4RateControlGains gains{};
    Px4RateControlLimits integrator_limits{};
    Px4RateControlFeedForward feed_forward{};
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

    Px4RateControlBackendConfig config_{};
    ::RateControl* controller_{nullptr};
};

}  // namespace hakoniwa::drone::control_adapter
