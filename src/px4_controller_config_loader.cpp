#include "hakoniwa/drone/control_adapter/px4_controller_config_loader.hpp"

#include <cmath>
#include <fstream>
#include <regex>
#include <sstream>
#include <stdexcept>

#ifdef HAKO_ENABLE_SENSOR_MODEL_V2
    // [SEN-COM-340] Load PX4 board rotation and IMU/GPS offsets for v2 compensation.
#include <conversion/rotation.h>
#endif

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

// "sensor_data_types": {"imu": "...", "gps": "..."}; an absent field keeps its default (exact).
Px4SensorDataTypes extract_sensor_data_types(const std::string& text)
{
    Px4SensorDataTypes types{};
    std::smatch section;
    if (!std::regex_search(text, section, std::regex("\"sensor_data_types\"\\s*:\\s*\\{([^}]*)\\}"))) {
        return types;
    }
    const std::string body = section[1].str();
    const auto field = [&body](const char* key, std::string& value) {
        std::smatch match;
        if (std::regex_search(body, match, std::regex("\"" + std::string(key) + "\"\\s*:\\s*\"([^\"]*)\""))) {
            value = match[1].str();
        }
    };
    field("imu", types.imu);
    field("gps", types.gps);
    validate_px4_sensor_data_types(types);
    return types;
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
    // CA_HOVER_DUTY is obsolete: allocation outputs normalized rotor thrust.
    // MPC_VEL_LP / MPC_VELD_LP are optional; PX4's defaults 0 (off) / 5 Hz. Applied only in a
    // standard PX4 behavior build (px4_velocity_filter_config.hpp).
    {
        Px4VelocityFilterConfig velocity_filter{};
        if (has_key(text, "MPC_VEL_LP")) {
            velocity_filter.velocity_cutoff_hz = extract_number(text, "MPC_VEL_LP");
        }
        if (has_key(text, "MPC_VELD_LP")) {
            velocity_filter.velocity_derivative_cutoff_hz = extract_number(text, "MPC_VELD_LP");
        }
        config.altitude_control.velocity_filter = velocity_filter;
        config.horizontal_control.velocity_filter = velocity_filter;
        config.position_control.velocity_filter = velocity_filter;
    }
    // IMU_GYRO_CUTOFF / IMU_DGYRO_CUTOFF are optional; PX4's defaults 40 Hz / 20 Hz. Applied only in a
    // standard PX4 behavior build (px4_rate_control_backend.hpp).
    if (has_key(text, "IMU_GYRO_CUTOFF")) {
        config.rate_control.imu_filters.gyro_cutoff_hz = extract_number(text, "IMU_GYRO_CUTOFF");
    }
    config.sensor_data_types = extract_sensor_data_types(text);
    config.rate_control.imu_data_type = config.sensor_data_types.imu;
#ifdef HAKO_ENABLE_SENSOR_MODEL_V2
    const auto sensor_parameter = [&text](const char* key, double default_value) {
        return has_key(text, key) ? extract_number(text, key) : default_value;
    };
    auto& sensor_model = config.sensor_model_v2;
    const double board_rotation = sensor_parameter("SENS_BOARD_ROT", 0.0);
    if (board_rotation < 0.0
        || board_rotation >= ROTATION_MAX
        || std::trunc(board_rotation) != board_rotation) {
        throw std::runtime_error("SENS_BOARD_ROT must be an integer in [0, ROTATION_MAX)");
    }
    sensor_model.board_rotation = static_cast<int>(board_rotation);
    sensor_model.board_x_offset_deg = sensor_parameter("SENS_BOARD_X_OFF", 0.0);
    sensor_model.board_y_offset_deg = sensor_parameter("SENS_BOARD_Y_OFF", 0.0);
    sensor_model.board_z_offset_deg = sensor_parameter("SENS_BOARD_Z_OFF", 0.0);
    sensor_model.imu_position_body_m = {
        sensor_parameter("EKF2_IMU_POS_X", 0.0),
        sensor_parameter("EKF2_IMU_POS_Y", 0.0),
        sensor_parameter("EKF2_IMU_POS_Z", 0.0)};
    sensor_model.gps_position_body_m = {
        sensor_parameter("SENS_GPS0_OFFX", 0.0),
        sensor_parameter("SENS_GPS0_OFFY", 0.0),
        sensor_parameter("SENS_GPS0_OFFZ", 0.0)};
    config.rate_control.sensor_model_v2 = sensor_model;
#endif
    // The rate stage on the previous cycle's rates setpoint, optional (off); standard PX4 behavior build only.
    config.rate_control.previous_rates_setpoint = has_key(text, "previous_rates_setpoint")
        && extract_number(text, "previous_rates_setpoint") != 0.0;
    // MC_YAW_TQ_CUTOFF is optional; 0 (off) when absent. standard PX4 behavior build only.
    if (has_key(text, "MC_YAW_TQ_CUTOFF")) {
        config.rate_control.yaw_torque_cutoff_hz = extract_number(text, "MC_YAW_TQ_CUTOFF");
    }
    if (has_key(text, "IMU_DGYRO_CUTOFF")) {
        config.rate_control.imu_filters.dgyro_cutoff_hz = extract_number(text, "IMU_DGYRO_CUTOFF");
    }
    // MC_AIRMODE is optional; PX4's default 0 disables air mode.
    if (has_key(text, "MC_AIRMODE")) {
        config.control_allocation.airmode = static_cast<int>(extract_number(text, "MC_AIRMODE"));
    }
    config.control_allocation.hover_thrust =
        extract_common_or_section_or_legacy_parameter_number(text, "position_control", "MPC_THR_HOVER");

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

        // PX4 Auto trajectory, optional with PX4's defaults; used only in a standard PX4 behavior
        // build (px4_position_control_3d_backend.hpp).
        auto& trajectory = config.position_control.trajectory;
        trajectory.enabled = has_key(text, "trajectory_generation")
            && extract_number(text, "trajectory_generation") != 0.0;
        const auto optional = [&text](const char* key, double& value) {
            if (has_key(text, key)) {
                value = extract_number(text, key);
            }
        };
        // PX4's manual heading lock, optional (off); standard PX4 behavior build only.
        config.position_control.manual_yaw_lock = has_key(text, "manual_yaw_lock")
            && extract_number(text, "manual_yaw_lock") != 0.0;
        optional("MPC_XY_CRUISE", trajectory.cruise_speed_mps);
        // PX4's automatic takeoff, optional (off); standard PX4 behavior build only.
        trajectory.takeoff_profile = has_key(text, "takeoff_profile") && extract_number(text, "takeoff_profile") != 0.0;
        optional("MPC_TKO_SPEED", trajectory.takeoff_speed_mps);
        optional("MPC_TKO_RAMP_T", trajectory.takeoff_ramp_sec);
        optional("COM_SPOOLUP_TIME", trajectory.spoolup_sec);
        optional("MPC_LAND_ALT1", trajectory.land_alt1_m);
        // PX4's landing descent, optional (off); flight-phase support and standard PX4 behavior builds only.
        trajectory.landing_profile = has_key(text, "landing_profile") && extract_number(text, "landing_profile") != 0.0;
        optional("MPC_LAND_SPEED", trajectory.land_speed_mps);
        optional("MPC_LAND_ALT2", trajectory.land_alt2_m);
        if (has_key(text, "MPC_TILTMAX_LND")) {
            trajectory.tilt_max_land_rad = extract_number(text, "MPC_TILTMAX_LND") * M_PI / 180.0;
        }
        // The navigator's delay (an option imitating PX4 SITL on POSIX): a number or a list of ms.
        {
            std::smatch list;
            if (std::regex_search(text, list, std::regex("\"navigator_delay_ms\"\\s*:\\s*\\[([^\\]]*)\\]"))) {
                const std::string body = list[1].str();
                const std::regex number("[-+]?(?:\\d+\\.?\\d*|\\d*\\.\\d+)(?:[eE][-+]?\\d+)?");
                for (auto it = std::sregex_iterator(body.begin(), body.end(), number); it != std::sregex_iterator(); ++it) {
                    trajectory.navigator_delay_ms.push_back(std::stod(it->str()));
                }
            }
            else if (has_key(text, "navigator_delay_ms")) {
                trajectory.navigator_delay_ms.push_back(extract_number(text, "navigator_delay_ms"));
            }
            for (double ms : trajectory.navigator_delay_ms) {
                if (!(ms >= 0.0)) {
                    throw std::runtime_error("position_control.navigator_delay_ms must be >= 0");
                }
            }
            if (has_key(text, "navigator_delay_seed")) {
                trajectory.navigator_delay_seed = static_cast<std::uint32_t>(extract_number(text, "navigator_delay_seed"));
            }
        }
        if (has_key(text, "trajectory_update_interval_ms")) {
            trajectory.update_interval_sec = extract_number(text, "trajectory_update_interval_ms") / 1000.0;
        }
        optional("MPC_ACC_HOR", trajectory.acceleration_xy_mps2);
        optional("MPC_JERK_AUTO", trajectory.jerk_mps3);
        optional("MPC_XY_TRAJ_P", trajectory.trajectory_gain_xy);
        optional("MPC_XY_ERR_MAX", trajectory.max_horizontal_error_m);
        optional("NAV_MC_ALT_RAD", trajectory.vertical_acceptance_m);
        optional("NAV_ACC_RAD", trajectory.target_acceptance_m);
        optional("MPC_ACC_UP_MAX", trajectory.acceleration_up_mps2);
        optional("MPC_ACC_DOWN_MAX", trajectory.acceleration_down_mps2);
        optional("MPC_Z_V_AUTO_UP", trajectory.velocity_up_mps);
        optional("MPC_Z_V_AUTO_DN", trajectory.velocity_down_mps);
        optional("MPC_YAWRAUTO_MAX", trajectory.yaw_rate_max_deg_s);
        optional("MPC_YAWRAUTO_ACC", trajectory.yaw_acceleration_max_deg_s2);
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
