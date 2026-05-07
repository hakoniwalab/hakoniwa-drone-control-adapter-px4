#pragma once

#include <string>

#include "hakoniwa/drone/control_adapter/px4_altitude_control_backend.hpp"
#include "hakoniwa/drone/control_adapter/px4_attitude_control_backend.hpp"
#include "hakoniwa/drone/control_adapter/px4_rate_control_backend.hpp"

namespace hakoniwa::drone::control_adapter {

struct Px4ControllerRuntimeConfig {
    double altitude_hz{0.0};
    double attitude_hz{0.0};
    double rate_hz{0.0};
};

struct Px4ControllerConfig {
    Px4ControllerRuntimeConfig runtime{};
    Px4AltitudeControlBackendConfig altitude_control{};
    Px4AttitudeControlBackendConfig attitude_control{};
    Px4RateControlBackendConfig rate_control{};
};

class Px4ControllerConfigLoader {
public:
    Px4ControllerConfig load_from_file(const std::string& path) const;
    Px4ControllerConfig load_from_text(const std::string& text) const;
};

}  // namespace hakoniwa::drone::control_adapter
