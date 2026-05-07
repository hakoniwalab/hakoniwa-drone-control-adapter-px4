#include "hakoniwa/drone/control_adapter/px4_controller_config_loader.hpp"

#include <fstream>
#include <regex>
#include <sstream>
#include <stdexcept>

namespace hakoniwa::drone::control_adapter {

namespace {

std::string load_file_text(const std::string& path)
{
    std::ifstream file(path);

    if (!file.is_open()) {
        throw std::runtime_error("failed to open config file: " + path);
    }

    return std::string(std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>());
}

double extract_number(const std::string& text, const char* key)
{
    const std::regex pattern("\"" + std::string(key) + "\"\\s*:\\s*([-+]?(?:\\d+\\.?\\d*|\\d*\\.\\d+)(?:[eE][-+]?\\d+)?)");
    std::smatch match;

    if (!std::regex_search(text, match, pattern)) {
        throw std::runtime_error(std::string("missing required config key: ") + key);
    }

    return std::stod(match[1].str());
}

}  // namespace

Px4ControllerConfig Px4ControllerConfigLoader::load_from_file(const std::string& path) const
{
    return load_from_text(load_file_text(path));
}

Px4ControllerConfig Px4ControllerConfigLoader::load_from_text(const std::string& text) const
{
    Px4ControllerConfig config{};

    config.runtime.attitude_hz = extract_number(text, "attitude_hz");
    config.runtime.rate_hz = extract_number(text, "rate_hz");

    config.attitude_control.proportional_gains.roll = extract_number(text, "MC_ROLL_P");
    config.attitude_control.proportional_gains.pitch = extract_number(text, "MC_PITCH_P");
    config.attitude_control.proportional_gains.yaw = extract_number(text, "MC_YAW_P");
    config.attitude_control.yaw_weight = extract_number(text, "MC_YAW_WEIGHT");
    config.attitude_control.rate_limits.roll_rad_sec = extract_number(text, "MC_ROLLRATE_MAX");
    config.attitude_control.rate_limits.pitch_rad_sec = extract_number(text, "MC_PITCHRATE_MAX");
    config.attitude_control.rate_limits.yaw_rad_sec = extract_number(text, "MC_YAWRATE_MAX");

    config.rate_control.gains.roll.p = extract_number(text, "MC_ROLLRATE_P");
    config.rate_control.gains.roll.i = extract_number(text, "MC_ROLLRATE_I");
    config.rate_control.gains.roll.d = extract_number(text, "MC_ROLLRATE_D");

    config.rate_control.gains.pitch.p = extract_number(text, "MC_PITCHRATE_P");
    config.rate_control.gains.pitch.i = extract_number(text, "MC_PITCHRATE_I");
    config.rate_control.gains.pitch.d = extract_number(text, "MC_PITCHRATE_D");

    config.rate_control.gains.yaw.p = extract_number(text, "MC_YAWRATE_P");
    config.rate_control.gains.yaw.i = extract_number(text, "MC_YAWRATE_I");
    config.rate_control.gains.yaw.d = extract_number(text, "MC_YAWRATE_D");

    config.rate_control.feed_forward.roll = extract_number(text, "MC_ROLLRATE_FF");
    config.rate_control.feed_forward.pitch = extract_number(text, "MC_PITCHRATE_FF");
    config.rate_control.feed_forward.yaw = extract_number(text, "MC_YAWRATE_FF");

    config.rate_control.integrator_limits.roll_integrator = extract_number(text, "MC_RR_INT_LIM");
    config.rate_control.integrator_limits.pitch_integrator = extract_number(text, "MC_PR_INT_LIM");
    config.rate_control.integrator_limits.yaw_integrator = extract_number(text, "MC_YR_INT_LIM");

    return config;
}

}  // namespace hakoniwa::drone::control_adapter
