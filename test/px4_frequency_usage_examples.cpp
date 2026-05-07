#include <exception>
#include <iostream>

#include "hakoniwa/drone/control_adapter/frequency_usage_examples.hpp"
#include "hakoniwa/drone/control_adapter/px4_altitude_control_backend.hpp"
#include "hakoniwa/drone/control_adapter/px4_attitude_control_backend.hpp"
#include "hakoniwa/drone/control_adapter/px4_controller_config_loader.hpp"
#include "hakoniwa/drone/control_adapter/px4_horizontal_position_control_backend.hpp"
#include "hakoniwa/drone/control_adapter/px4_rate_control_backend.hpp"
#include "hakoniwa/drone/control_adapter/usage_examples.hpp"

namespace {

class Px4UsageExampleContext final : public hakoniwa::drone::control_adapter::test::IUsageExampleContext {
public:
    explicit Px4UsageExampleContext(const hakoniwa::drone::control_adapter::Px4ControllerConfig& config)
        : rate_backend_(config.rate_control)
        , attitude_backend_(config.attitude_control)
        , altitude_backend_(config.altitude_control)
        , horizontal_backend_(config.horizontal_control)
        , rate_dt_sec_(1.0 / config.runtime.rate_hz)
        , attitude_dt_sec_(1.0 / config.runtime.attitude_hz)
        , altitude_dt_sec_(1.0 / config.runtime.altitude_hz)
        , horizontal_dt_sec_(1.0 / config.runtime.horizontal_hz)
    {
    }

    hakoniwa::drone::control_adapter::IRateControlBackend& rate_backend() override { return rate_backend_; }
    hakoniwa::drone::control_adapter::IAttitudeControlBackend& attitude_backend() override { return attitude_backend_; }
    hakoniwa::drone::control_adapter::IAltitudeControlBackend& altitude_backend() override { return altitude_backend_; }
    hakoniwa::drone::control_adapter::IHorizontalPositionControlBackend& horizontal_backend() override { return horizontal_backend_; }

    double rate_dt_sec() const override { return rate_dt_sec_; }
    double attitude_dt_sec() const override { return attitude_dt_sec_; }
    double altitude_dt_sec() const override { return altitude_dt_sec_; }
    double horizontal_dt_sec() const override { return horizontal_dt_sec_; }

    void reset_all() override
    {
        rate_backend_.reset();
        attitude_backend_.reset();
        altitude_backend_.reset();
        horizontal_backend_.reset();
    }

private:
    hakoniwa::drone::control_adapter::Px4RateControlBackend rate_backend_;
    hakoniwa::drone::control_adapter::Px4AttitudeControlBackend attitude_backend_;
    hakoniwa::drone::control_adapter::Px4AltitudeControlBackend altitude_backend_;
    hakoniwa::drone::control_adapter::Px4HorizontalPositionControlBackend horizontal_backend_;
    double rate_dt_sec_;
    double attitude_dt_sec_;
    double altitude_dt_sec_;
    double horizontal_dt_sec_;
};

}  // namespace

int main()
{
    using namespace hakoniwa::drone::control_adapter;

    try {
        Px4ControllerConfigLoader loader;
        const Px4ControllerConfig config =
            loader.load_from_file("../config/px4-controller-config.sample.json");

        Px4UsageExampleContext context(config);
        return test::run_frequency_usage_examples(context);
    } catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return 1;
    }
}
