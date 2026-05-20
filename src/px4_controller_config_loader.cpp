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

double extract_scoped_number(
    const std::string& text,
    const char* outer_key,
    const char* inner_key,
    const char* key)
{
    const std::string number_pattern = "([-+]?(?:\\d+\\.?\\d*|\\d*\\.\\d+)(?:[eE][-+]?\\d+)?)";
    const std::regex pattern(
        "\"" + std::string(outer_key) + "\"\\s*:\\s*\\{[\\s\\S]*?"
        "\"" + std::string(inner_key) + "\"\\s*:\\s*\\{[\\s\\S]*?"
        "\"" + std::string(key) + "\"\\s*:\\s*" + number_pattern);
    std::smatch match;

    if (!std::regex_search(text, match, pattern)) {
        throw std::runtime_error(
            std::string("missing required scoped config key: ") + outer_key + "." + inner_key + "." + key);
    }

    return std::stod(match[1].str());
}

bool has_key(const std::string& text, const char* key)
{
    const std::regex pattern("\"" + std::string(key) + "\"\\s*:");
    return std::regex_search(text, pattern);
}

bool has_scoped_key(const std::string& text, const char* outer_key, const char* inner_key, const char* key)
{
    const std::regex pattern(
        "\"" + std::string(outer_key) + "\"\\s*:\\s*\\{[\\s\\S]*?"
        "\"" + std::string(inner_key) + "\"\\s*:\\s*\\{[\\s\\S]*?"
        "\"" + std::string(key) + "\"\\s*:");
    return std::regex_search(text, pattern);
}

double extract_common_or_section_or_legacy_parameter_number(
    const std::string& text,
    const char* section_key,
    const char* key)
{
    if (has_scoped_key(text, "common", "parameters", key)) {
        return extract_scoped_number(text, "common", "parameters", key);
    }
    if (section_key != nullptr && has_scoped_key(text, section_key, "parameters", key)) {
        return extract_scoped_number(text, section_key, "parameters", key);
    }
    if (section_key != nullptr &&
        std::string(section_key) != "position_control" &&
        has_scoped_key(text, "position_control", "parameters", key)) {
        return extract_scoped_number(text, "position_control", "parameters", key);
    }
    return extract_number(text, key);
}

double extract_position_or_legacy_parameter_number(const std::string& text, const char* key)
{
    if (has_scoped_key(text, "position_control", "parameters", key)) {
        return extract_scoped_number(text, "position_control", "parameters", key);
    }
    return extract_number(text, key);
}

}  // namespace

Px4ControllerConfig Px4ControllerConfigLoader::load_from_file(const std::string& path) const
{
    return load_from_text(load_file_text(path));
}

Px4ControllerConfig Px4ControllerConfigLoader::load_from_text(const std::string& text) const
{
    Px4ControllerConfig config{};

    config.runtime.altitude_hz = extract_number(text, "altitude_hz");
    config.runtime.attitude_hz = extract_number(text, "attitude_hz");
    config.runtime.horizontal_hz = extract_number(text, "horizontal_hz");
    config.runtime.position_hz = has_key(text, "position_hz")
        ? extract_number(text, "position_hz")
        : 0.0;
    config.runtime.rate_hz = extract_number(text, "rate_hz");

    config.altitude_control.position_gain_z = extract_position_or_legacy_parameter_number(text, "MPC_Z_P");
    config.altitude_control.velocity_p_z = extract_position_or_legacy_parameter_number(text, "MPC_Z_VEL_P_ACC");
    config.altitude_control.velocity_i_z = extract_position_or_legacy_parameter_number(text, "MPC_Z_VEL_I_ACC");
    config.altitude_control.velocity_d_z = extract_position_or_legacy_parameter_number(text, "MPC_Z_VEL_D_ACC");
    config.altitude_control.velocity_max_up_mps = extract_position_or_legacy_parameter_number(text, "MPC_Z_VEL_MAX_UP");
    config.altitude_control.velocity_max_down_mps = extract_position_or_legacy_parameter_number(text, "MPC_Z_VEL_MAX_DN");
    config.altitude_control.hover_thrust =
        extract_common_or_section_or_legacy_parameter_number(text, "position_control", "MPC_THR_HOVER");
    config.altitude_control.thrust_min =
        extract_common_or_section_or_legacy_parameter_number(text, "position_control", "MPC_THR_MIN");
    config.altitude_control.thrust_max =
        extract_common_or_section_or_legacy_parameter_number(text, "position_control", "MPC_THR_MAX");

    config.attitude_control.proportional_gains.roll = extract_number(text, "MC_ROLL_P");
    config.attitude_control.proportional_gains.pitch = extract_number(text, "MC_PITCH_P");
    config.attitude_control.proportional_gains.yaw = extract_number(text, "MC_YAW_P");
    config.attitude_control.yaw_weight = extract_number(text, "MC_YAW_WEIGHT");
    config.attitude_control.rate_limits.roll_rad_sec = extract_number(text, "MC_ROLLRATE_MAX");
    config.attitude_control.rate_limits.pitch_rad_sec = extract_number(text, "MC_PITCHRATE_MAX");
    config.attitude_control.rate_limits.yaw_rad_sec = extract_number(text, "MC_YAWRATE_MAX");

    config.control_allocation.normalize_rpy = extract_number(text, "CA_RPY_NORMALIZE") != 0.0;
    config.control_allocation.metric_allocation = extract_number(text, "CA_METRIC_ALLOCATION") != 0.0;
    config.control_allocation.update_normalization_scale =
        extract_number(text, "CA_UPDATE_NORMALIZATION_SCALE") != 0.0;
    config.control_allocation.hover_duty = extract_number(text, "CA_HOVER_DUTY");

    config.horizontal_control.position_gain_xy = extract_position_or_legacy_parameter_number(text, "MPC_XY_P");
    config.horizontal_control.velocity_p_xy = extract_position_or_legacy_parameter_number(text, "MPC_XY_VEL_P_ACC");
    config.horizontal_control.velocity_i_xy = extract_position_or_legacy_parameter_number(text, "MPC_XY_VEL_I_ACC");
    config.horizontal_control.velocity_d_xy = extract_position_or_legacy_parameter_number(text, "MPC_XY_VEL_D_ACC");
    config.horizontal_control.velocity_max_xy_mps = extract_position_or_legacy_parameter_number(text, "MPC_XY_VEL_MAX");
    config.horizontal_control.tilt_limit_rad = extract_position_or_legacy_parameter_number(text, "MPC_TILTMAX_AIR");
    config.horizontal_control.horizontal_thrust_margin = extract_position_or_legacy_parameter_number(text, "MPC_THR_XY_MARG");
    config.horizontal_control.hover_thrust =
        extract_common_or_section_or_legacy_parameter_number(text, "position_control", "MPC_THR_HOVER");
    config.horizontal_control.thrust_min =
        extract_common_or_section_or_legacy_parameter_number(text, "position_control", "MPC_THR_MIN");
    config.horizontal_control.thrust_max =
        extract_common_or_section_or_legacy_parameter_number(text, "position_control", "MPC_THR_MAX");
    config.horizontal_control.decouple_horizontal_and_vertical_acceleration =
        extract_position_or_legacy_parameter_number(text, "MPC_ACC_DECOUPLE") != 0.0;

    if (has_key(text, "position_control")) {
        config.position_control.position_gain_xy = extract_position_or_legacy_parameter_number(text, "MPC_XY_P");
        config.position_control.position_gain_z = extract_position_or_legacy_parameter_number(text, "MPC_Z_P");

        config.position_control.velocity_p_xy = extract_position_or_legacy_parameter_number(text, "MPC_XY_VEL_P_ACC");
        config.position_control.velocity_i_xy = extract_position_or_legacy_parameter_number(text, "MPC_XY_VEL_I_ACC");
        config.position_control.velocity_d_xy = extract_position_or_legacy_parameter_number(text, "MPC_XY_VEL_D_ACC");

        config.position_control.velocity_p_z = extract_position_or_legacy_parameter_number(text, "MPC_Z_VEL_P_ACC");
        config.position_control.velocity_i_z = extract_position_or_legacy_parameter_number(text, "MPC_Z_VEL_I_ACC");
        config.position_control.velocity_d_z = extract_position_or_legacy_parameter_number(text, "MPC_Z_VEL_D_ACC");

        config.position_control.velocity_max_xy_mps = extract_position_or_legacy_parameter_number(text, "MPC_XY_VEL_MAX");
        config.position_control.velocity_max_up_mps = extract_position_or_legacy_parameter_number(text, "MPC_Z_VEL_MAX_UP");
        config.position_control.velocity_max_down_mps = extract_position_or_legacy_parameter_number(text, "MPC_Z_VEL_MAX_DN");

        config.position_control.tilt_limit_rad = extract_position_or_legacy_parameter_number(text, "MPC_TILTMAX_AIR");
        config.position_control.hover_thrust =
            extract_common_or_section_or_legacy_parameter_number(text, "position_control", "MPC_THR_HOVER");
        config.position_control.thrust_min =
            extract_common_or_section_or_legacy_parameter_number(text, "position_control", "MPC_THR_MIN");
        config.position_control.thrust_max =
            extract_common_or_section_or_legacy_parameter_number(text, "position_control", "MPC_THR_MAX");
        config.position_control.horizontal_thrust_margin =
            extract_position_or_legacy_parameter_number(text, "MPC_THR_XY_MARG");
        config.position_control.decouple_horizontal_and_vertical_acceleration =
            extract_position_or_legacy_parameter_number(text, "MPC_ACC_DECOUPLE") != 0.0;
    }

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
