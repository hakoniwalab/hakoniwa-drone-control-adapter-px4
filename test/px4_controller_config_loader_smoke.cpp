#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

#include "hakoniwa/drone/control_adapter/px4_controller_config_loader.hpp"

using namespace hakoniwa::drone::control_adapter;

namespace {

bool nearly_equal(double a, double b, double eps = 1e-9)
{
    return std::fabs(a - b) <= eps;
}

void require(bool condition, const char* message)
{
    if (!condition) {
        std::cerr << message << std::endl;
        std::exit(EXIT_FAILURE);
    }
}

std::string sample_config_text()
{
    std::ifstream input("../config/px4-controller-config.sample.json");
    std::ostringstream buffer;
    buffer << input.rdbuf();
    return buffer.str();
}

void replace_once(std::string& text, const std::string& from, const std::string& to)
{
    const auto position = text.find(from);
    require(position != std::string::npos, "test fixture marker was not found");
    text.replace(position, from.size(), to);
}

void require_hover_source(
    const Px4ControllerConfigLoader& loader,
    const std::string& text,
    double expected,
    const char* message)
{
    const auto config = loader.load_from_text(text);
    require(nearly_equal(config.altitude_control.hover_thrust, expected), message);
    require(nearly_equal(config.control_allocation.hover_thrust, expected), message);
}

}  // namespace

int main()
{
    Px4ControllerConfigLoader loader;
    const Px4ControllerConfig config =
        loader.load_from_file("../config/px4-controller-config.sample.json");

    require(nearly_equal(config.runtime.altitude_hz, 1000.0), "unexpected altitude_hz");
    require(nearly_equal(config.runtime.attitude_hz, 1000.0), "unexpected attitude_hz");
    require(nearly_equal(config.runtime.horizontal_hz, 1000.0), "unexpected horizontal_hz");
    require(nearly_equal(config.runtime.position_hz, 1000.0), "unexpected position_hz");
    require(nearly_equal(config.runtime.rate_hz, 1000.0), "unexpected rate_hz");

    require(nearly_equal(config.altitude_control.position_gain_z, 10.0), "unexpected altitude pos p");
    require(nearly_equal(config.altitude_control.velocity_p_z, 15.0), "unexpected altitude vel p");
    require(nearly_equal(config.altitude_control.velocity_d_z, 10.0), "unexpected altitude vel d");
    require(nearly_equal(config.altitude_control.velocity_max_up_mps, 10.0), "unexpected altitude vel up");
    require(nearly_equal(config.altitude_control.hover_thrust, 0.5), "unexpected hover thrust");
    require(nearly_equal(config.altitude_control.thrust_max, 0.9), "unexpected thrust max");

    require(nearly_equal(config.position_control.position_gain_xy, 6.0), "unexpected position-control xy p");
    require(nearly_equal(config.position_control.position_gain_z, 10.0), "unexpected position-control z p");
    require(nearly_equal(config.position_control.velocity_p_xy, 10.0), "unexpected position-control xy vel p");
    require(nearly_equal(config.position_control.velocity_i_xy, 0.0), "unexpected position-control xy vel i");
    require(nearly_equal(config.position_control.velocity_d_xy, 0.1), "unexpected position-control xy vel d");
    require(nearly_equal(config.position_control.velocity_p_z, 15.0), "unexpected position-control z vel p");
    require(nearly_equal(config.position_control.velocity_i_z, 0.0), "unexpected position-control z vel i");
    require(nearly_equal(config.position_control.velocity_d_z, 10.0), "unexpected position-control z vel d");
    require(nearly_equal(config.position_control.velocity_max_xy_mps, 20.0), "unexpected position-control xy vel max");
    require(nearly_equal(config.position_control.velocity_max_up_mps, 10.0), "unexpected position-control z vel up");
    require(nearly_equal(config.position_control.velocity_max_down_mps, 10.0), "unexpected position-control z vel down");
    require(nearly_equal(config.position_control.tilt_limit_rad, 0.2617993877991494), "unexpected position-control tilt limit");
    require(nearly_equal(config.position_control.horizontal_thrust_margin, 0.3), "unexpected position-control horizontal thrust margin");
    require(nearly_equal(config.position_control.hover_thrust, 0.5), "unexpected position-control hover thrust");
    require(nearly_equal(config.position_control.thrust_min, 0.1), "unexpected position-control thrust min");
    require(nearly_equal(config.position_control.thrust_max, 0.9), "unexpected position-control thrust max");
    require(config.position_control.decouple_horizontal_and_vertical_acceleration, "unexpected position-control decouple flag");

    require(nearly_equal(config.attitude_control.proportional_gains.roll, 2.5), "unexpected attitude roll p");
    require(nearly_equal(config.attitude_control.proportional_gains.pitch, 2.5), "unexpected attitude pitch p");
    require(nearly_equal(config.attitude_control.proportional_gains.yaw, 0.1), "unexpected attitude yaw p");
    require(nearly_equal(config.attitude_control.yaw_weight, 0.4), "unexpected yaw weight");
    require(nearly_equal(config.attitude_control.rate_limits.roll_rad_sec, 314.1592653589793), "unexpected roll rate limit");
    require(nearly_equal(config.attitude_control.rate_limits.yaw_rad_sec, 31.41592653589793), "unexpected yaw rate limit");

    require(config.control_allocation.normalize_rpy, "unexpected control allocation normalize_rpy");
    require(!config.control_allocation.metric_allocation, "unexpected control allocation metric_allocation");
    require(config.control_allocation.update_normalization_scale, "unexpected control allocation normalization scale update");
    require(config.control_allocation.airmode == 0,
        "missing MC_AIRMODE must keep PX4 default zero");
    require(nearly_equal(config.control_allocation.hover_thrust, 0.5),
        "control allocation must receive common MPC_THR_HOVER");

    require(nearly_equal(config.horizontal_control.position_gain_xy, 6.0), "unexpected horizontal pos p");
    require(nearly_equal(config.horizontal_control.velocity_p_xy, 10.0), "unexpected horizontal vel p");
    require(nearly_equal(config.horizontal_control.velocity_d_xy, 0.1), "unexpected horizontal vel d");
    require(nearly_equal(config.horizontal_control.velocity_max_xy_mps, 20.0), "unexpected horizontal vel max");
    require(nearly_equal(config.horizontal_control.tilt_limit_rad, 0.2617993877991494), "unexpected tilt limit");
    require(nearly_equal(config.horizontal_control.horizontal_thrust_margin, 0.3), "unexpected horizontal thrust margin");

    require(nearly_equal(config.rate_control.gains.roll.p, 1.5), "unexpected roll p");
    require(nearly_equal(config.rate_control.gains.pitch.d, 0.02), "unexpected pitch d");
    require(nearly_equal(config.rate_control.gains.yaw.p, 0.452), "unexpected yaw p");
    require(nearly_equal(config.rate_control.feed_forward.roll, 0.0), "unexpected roll ff");
    require(nearly_equal(config.rate_control.integrator_limits.yaw_integrator, 0.2), "unexpected yaw int lim");
    require(nearly_equal(config.rate_control.imu_filters.gyro_cutoff_hz, 40.0),
        "missing IMU_GYRO_CUTOFF must keep PX4 default 40 Hz");
    require(nearly_equal(config.rate_control.imu_filters.dgyro_cutoff_hz, 20.0),
        "missing IMU_DGYRO_CUTOFF must keep PX4 default 20 Hz");
    require(nearly_equal(config.rate_control.yaw_torque_cutoff_hz, 0.0),
        "missing MC_YAW_TQ_CUTOFF must keep legacy filtering disabled");
    require(nearly_equal(config.altitude_control.velocity_filter.velocity_cutoff_hz, 0.0),
        "missing MPC_VEL_LP must keep PX4 default zero");
    require(nearly_equal(config.altitude_control.velocity_filter.velocity_derivative_cutoff_hz, 5.0),
        "missing MPC_VELD_LP must keep PX4 default 5 Hz");
    require(!config.position_control.trajectory.enabled,
        "missing trajectory_generation must keep trajectory disabled");
    require(nearly_equal(config.position_control.trajectory.cruise_speed_mps, 5.0),
        "missing MPC_XY_CRUISE must keep PX4 default");
    require(nearly_equal(config.position_control.trajectory.jerk_mps3, 4.0),
        "missing MPC_JERK_AUTO must keep PX4 default");
    require(nearly_equal(config.position_control.trajectory.update_interval_sec, 0.0),
        "missing trajectory_update_interval_ms must update every call");
    require(nearly_equal(config.position_control.trajectory.yaw_rate_max_deg_s, 60.0),
        "missing MPC_YAWRAUTO_MAX must keep PX4 default");
    require(!config.position_control.trajectory.takeoff_profile,
        "missing takeoff_profile must preserve legacy behavior");
    require(nearly_equal(config.position_control.trajectory.takeoff_speed_mps, 1.5),
        "missing MPC_TKO_SPEED must keep PX4 default");
    require(nearly_equal(config.position_control.trajectory.takeoff_ramp_sec, 3.0),
        "missing MPC_TKO_RAMP_T must keep PX4 default");
    require(nearly_equal(config.position_control.trajectory.spoolup_sec, 1.0),
        "missing COM_SPOOLUP_TIME must keep PX4 default");
    require(nearly_equal(config.position_control.trajectory.land_alt1_m, 10.0),
        "missing MPC_LAND_ALT1 must keep PX4 default");
    require(nearly_equal(config.position_control.trajectory.tilt_max_land_rad, 12.0 * M_PI / 180.0),
        "missing MPC_TILTMAX_LND must keep PX4 default and convert it to radians");
    require(config.sensor_data_types.imu == sensor_data_type::kExact,
        "missing sensor_data_types.imu must default to exact");
    require(config.sensor_data_types.gps == sensor_data_type::kExact,
        "missing sensor_data_types.gps must default to exact");
    require(config.rate_control.imu_data_type == sensor_data_type::kExact,
        "rate controller must receive the default IMU data type");
    require(!config.rate_control.previous_rates_setpoint,
        "missing previous_rates_setpoint must default to false");

    auto previous_setpoint = sample_config_text();
    replace_once(previous_setpoint, "  \"rate_control\": {\n",
        "  \"rate_control\": {\n    \"previous_rates_setpoint\": 2,\n");
    require(loader.load_from_text(previous_setpoint).rate_control.previous_rates_setpoint,
        "nonzero previous_rates_setpoint must enable the delay");

    auto imu_only = sample_config_text();
    replace_once(imu_only, "{\n", "{\n  \"sensor_data_types\": {\"imu\": \"imu_fifo_int16\"},\n");
    const auto imu_only_config = loader.load_from_text(imu_only);
    require(imu_only_config.sensor_data_types.imu == sensor_data_type::kImuFifoInt16,
        "partial sensor_data_types must load IMU");
    require(imu_only_config.sensor_data_types.gps == sensor_data_type::kExact,
        "partial sensor_data_types must default GPS to exact");
    require(imu_only_config.rate_control.imu_data_type == sensor_data_type::kImuFifoInt16,
        "rate controller did not receive configured IMU data type");

    auto both_types = sample_config_text();
    replace_once(both_types, "{\n", "{\n  \"sensor_data_types\": {\"imu\": \"imu_fifo_int16\", \"gps\": \"mavlink_hil_gps\"},\n");
    const auto both_types_config = loader.load_from_text(both_types);
    require(both_types_config.sensor_data_types.gps == sensor_data_type::kMavlinkHilGps,
        "sensor_data_types.gps was not loaded");

    bool bad_imu_rejected = false;
    bool bad_gps_rejected = false;
    try {
        auto text = sample_config_text();
        replace_once(text, "{\n", "{\n  \"sensor_data_types\": {\"imu\": \"bad\"},\n");
        (void)loader.load_from_text(text);
    } catch (const std::runtime_error&) { bad_imu_rejected = true; }
    try {
        auto text = sample_config_text();
        replace_once(text, "{\n", "{\n  \"sensor_data_types\": {\"gps\": \"bad\"},\n");
        (void)loader.load_from_text(text);
    } catch (const std::runtime_error&) { bad_gps_rejected = true; }
    require(bad_imu_rejected, "unsupported configured IMU data type was accepted");
    require(bad_gps_rejected, "unsupported configured GPS data type was accepted");

    const std::string position_marker =
        "  \"position_control\": {\n    \"parameters\": {\n";
    const std::string position_with_hover =
        position_marker + "      \"MPC_THR_HOVER\": 0.4,\n";
    const std::string root_marker = "{\n";
    const std::string root_with_hover = "{\n  \"MPC_THR_HOVER\": 0.3,\n";
    const std::string common_hover = "      \"MPC_THR_HOVER\": 0.5,\n";

    auto all_sources = sample_config_text();
    replace_once(all_sources, position_marker, position_with_hover);
    replace_once(all_sources, root_marker, root_with_hover);
    require_hover_source(loader, all_sources, 0.5,
        "common MPC_THR_HOVER must have highest priority");

    auto position_and_legacy = all_sources;
    replace_once(position_and_legacy, common_hover, "");
    require_hover_source(loader, position_and_legacy, 0.4,
        "position_control MPC_THR_HOVER must precede legacy");

    auto legacy_only = sample_config_text();
    replace_once(legacy_only, common_hover, "");
    replace_once(legacy_only, root_marker, root_with_hover);
    require_hover_source(loader, legacy_only, 0.3,
        "legacy MPC_THR_HOVER must be the final fallback");

    auto explicit_airmode = sample_config_text();
    replace_once(explicit_airmode, root_marker, "{\n  \"MC_AIRMODE\": 2,\n");
    require(loader.load_from_text(explicit_airmode).control_allocation.airmode == 2,
        "explicit MC_AIRMODE must be loaded");

    auto explicit_filters = sample_config_text();
    replace_once(explicit_filters, root_marker,
        "{\n  \"IMU_GYRO_CUTOFF\": 31.0,\n  \"IMU_DGYRO_CUTOFF\": 17.0,\n"
        "  \"MC_YAW_TQ_CUTOFF\": 2.0,\n"
        "  \"MPC_VEL_LP\": 4.0,\n  \"MPC_VELD_LP\": 8.0,\n");
    const auto filter_config = loader.load_from_text(explicit_filters);
    require(nearly_equal(filter_config.rate_control.imu_filters.gyro_cutoff_hz, 31.0),
        "explicit IMU_GYRO_CUTOFF must be loaded");
    require(nearly_equal(filter_config.rate_control.imu_filters.dgyro_cutoff_hz, 17.0),
        "explicit IMU_DGYRO_CUTOFF must be loaded");
    require(nearly_equal(filter_config.rate_control.yaw_torque_cutoff_hz, 2.0),
        "explicit MC_YAW_TQ_CUTOFF must be loaded");
    require(nearly_equal(filter_config.altitude_control.velocity_filter.velocity_cutoff_hz, 4.0),
        "explicit MPC_VEL_LP must be loaded");
    require(nearly_equal(filter_config.horizontal_control.velocity_filter.velocity_derivative_cutoff_hz, 8.0),
        "explicit MPC_VELD_LP must be shared by position stages");
    require(nearly_equal(filter_config.position_control.velocity_filter.velocity_derivative_cutoff_hz, 8.0),
        "explicit MPC_VELD_LP must be shared by the 3D position stage");

    auto explicit_trajectory = sample_config_text();
    replace_once(explicit_trajectory, position_marker,
        position_marker
        + "      \"trajectory_generation\": 1,\n"
          "      \"trajectory_update_interval_ms\": 20,\n"
          "      \"navigator_delay_ms\": [4, 8, 12],\n"
          "      \"navigator_delay_seed\": 42,\n"
          "      \"MPC_XY_CRUISE\": 4.1,\n"
          "      \"MPC_ACC_HOR\": 2.2,\n"
          "      \"MPC_JERK_AUTO\": 3.3,\n"
          "      \"MPC_XY_TRAJ_P\": 0.7,\n"
          "      \"MPC_XY_ERR_MAX\": 1.2,\n"
          "      \"NAV_MC_ALT_RAD\": 0.6,\n"
          "      \"NAV_ACC_RAD\": 8.0,\n"
          "      \"MPC_ACC_UP_MAX\": 3.1,\n"
          "      \"MPC_ACC_DOWN_MAX\": 2.1,\n"
          "      \"MPC_Z_V_AUTO_UP\": 2.5,\n"
          "      \"MPC_Z_V_AUTO_DN\": 1.1,\n"
          "      \"MPC_YAWRAUTO_MAX\": 55.0,\n"
          "      \"MPC_YAWRAUTO_ACC\": 15.0,\n"
          "      \"takeoff_profile\": 1,\n"
          "      \"MPC_TKO_SPEED\": 1.2,\n"
          "      \"MPC_TKO_RAMP_T\": 2.5,\n"
          "      \"COM_SPOOLUP_TIME\": 0.8,\n"
          "      \"MPC_LAND_ALT1\": 7.0,\n"
          "      \"MPC_TILTMAX_LND\": 15.0,\n");
    const auto trajectory = loader.load_from_text(explicit_trajectory).position_control.trajectory;
    require(trajectory.enabled, "trajectory_generation must enable trajectory");
    require(nearly_equal(trajectory.update_interval_sec, 0.020),
        "trajectory_update_interval_ms must be converted to seconds");
    require(trajectory.navigator_delay_ms == std::vector<double>({4.0, 8.0, 12.0}),
        "navigator_delay_ms list must be loaded");
    require(trajectory.navigator_delay_seed == 42,
        "navigator_delay_seed must be loaded");
    require(nearly_equal(trajectory.cruise_speed_mps, 4.1), "MPC_XY_CRUISE not loaded");
    require(nearly_equal(trajectory.acceleration_xy_mps2, 2.2), "MPC_ACC_HOR not loaded");
    require(nearly_equal(trajectory.jerk_mps3, 3.3), "MPC_JERK_AUTO not loaded");
    require(nearly_equal(trajectory.trajectory_gain_xy, 0.7), "MPC_XY_TRAJ_P not loaded");
    require(nearly_equal(trajectory.max_horizontal_error_m, 1.2), "MPC_XY_ERR_MAX not loaded");
    require(nearly_equal(trajectory.vertical_acceptance_m, 0.6), "NAV_MC_ALT_RAD not loaded");
    require(nearly_equal(trajectory.target_acceptance_m, 8.0), "NAV_ACC_RAD not loaded");
    require(nearly_equal(trajectory.acceleration_up_mps2, 3.1), "MPC_ACC_UP_MAX not loaded");
    require(nearly_equal(trajectory.acceleration_down_mps2, 2.1), "MPC_ACC_DOWN_MAX not loaded");
    require(nearly_equal(trajectory.velocity_up_mps, 2.5), "MPC_Z_V_AUTO_UP not loaded");
    require(nearly_equal(trajectory.velocity_down_mps, 1.1), "MPC_Z_V_AUTO_DN not loaded");
    require(nearly_equal(trajectory.yaw_rate_max_deg_s, 55.0), "MPC_YAWRAUTO_MAX not loaded");
    require(nearly_equal(trajectory.yaw_acceleration_max_deg_s2, 15.0), "MPC_YAWRAUTO_ACC not loaded");
    require(trajectory.takeoff_profile, "takeoff_profile not loaded");
    require(nearly_equal(trajectory.takeoff_speed_mps, 1.2), "MPC_TKO_SPEED not loaded");
    require(nearly_equal(trajectory.takeoff_ramp_sec, 2.5), "MPC_TKO_RAMP_T not loaded");
    require(nearly_equal(trajectory.spoolup_sec, 0.8), "COM_SPOOLUP_TIME not loaded");
    require(nearly_equal(trajectory.land_alt1_m, 7.0), "MPC_LAND_ALT1 not loaded");
    require(nearly_equal(trajectory.tilt_max_land_rad, 15.0 * M_PI / 180.0),
        "MPC_TILTMAX_LND must be converted to radians");

    std::cout << "loader smoke test passed" << std::endl;
    return EXIT_SUCCESS;
}
