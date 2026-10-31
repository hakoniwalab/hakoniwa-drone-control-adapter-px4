// [SEN-COM-340] [SEN-COM-295] The mount parameters change what PX4 EKF2 estimates, not only its settings.
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <vector>

#include "hakoniwa/drone/control_adapter/px4_ekf_adapter.hpp"

using namespace hakoniwa::drone::control_adapter;

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kHomeLatDeg = 35.681236;
constexpr double kHomeLonDeg = 139.767125;
constexpr double kEarthRadiusM = 6378137.0;

struct Run {
    Px4SensorModelV2Config mount{};
    // ROTATION_ROLL_180 (8): the sensor reads body (x, y, z) as (x, -y, -z).
    bool sensor_upside_down{false};
    // Raw field = body field + this bias; CAL_MAG0_*OFF removes it.
    std::array<double, 3> mag_bias_gauss{};
    // A 40 s, 1 rad/s yaw spin after alignment, with the IMU this far ahead of the (still) center of mass.
    // EKF2 settles in about 12 s, so the speed is judged over the last 20 s.
    double spin_lever_arm_m{0.0};
};

struct Outcome {
    EkfEstimatedState state{};
    double late_spin_speed_mps{0.0};
};

Outcome fly(const Run& run)
{
    Px4EkfAdapter ekf;
    ekf.set_sensor_model_v2_config(run.mount);
    EkfAdapterConfig config{};
    config.mag_declination_deg = 0.0;
    ekf.set_config(config);
    ekf.set_armed_status(false);
    ekf.set_in_air_status(false);
    ekf.set_vehicle_at_rest(true);
    const double sign = run.sensor_upside_down ? -1.0 : 1.0;
    constexpr double dt = 0.004;
    constexpr std::uint64_t dt_usec = 4000;
    constexpr int align_steps = 15000;
    const int spin_steps = run.spin_lever_arm_m > 0.0 ? 10000 : 0;
    Outcome outcome;
    double heading = 0.0;
    for (int step = 1; step <= align_steps + spin_steps; ++step) {
        const std::uint64_t now = static_cast<std::uint64_t>(step) * dt_usec;
        const bool spinning = step > align_steps;
        if (step == align_steps + 1 && spinning) {
            ekf.set_vehicle_at_rest(false);
        }
        const double yaw_rate = spinning ? 1.0 : 0.0;
        // Circling the center of mass at 1 rad/s, the IMU feels w^2 r toward it (body -x).
        const double centripetal = spinning ? -run.spin_lever_arm_m : 0.0;
        ekf.push_imu(EkfImuInput{now, centripetal, 0.0, sign * -9.80665, 0.0, 0.0, sign * yaw_rate}, dt);
        heading += yaw_rate * dt;
        if (step % 5 == 0) {
            const double north = 0.22 * std::cos(heading);
            const double east = -0.22 * std::sin(heading);
            ekf.push_mag(EkfMagInput{now, north + run.mag_bias_gauss[0], sign * east + run.mag_bias_gauss[1],
                sign * 0.43 + run.mag_bias_gauss[2]});
        }
        if (step % 5 == 0) {
            ekf.push_baro(EkfBaroInput{now, 0.0});
        }
        if (step % 50 == 0) {
            EkfHilGpsInput gps{};
            gps.time_usec = now;
            gps.fix_type = 3;
            gps.lat_deg = kHomeLatDeg;
            gps.lon_deg = kHomeLonDeg;
            gps.alt_m = 40.0;
            gps.satellites_visible = 16;
            gps.eph_m = 0.5;
            gps.epv_m = 0.8;
            gps.sacc_mps = 0.2;
            ekf.push_gps(gps);
        }
        ekf.update();
        if (step > align_steps + 5000) {
            const auto state = ekf.get_estimated_state();
            const double speed = std::hypot(state.velocity_ned_mps[0], state.velocity_ned_mps[1]);
            outcome.late_spin_speed_mps = std::fmax(outcome.late_spin_speed_mps, speed);
        }
    }
    outcome.state = ekf.get_estimated_state();
    return outcome;
}

// [SEN-BAR-325] Barometer runs at rest: pressure_of(t) is the pressure the host sends at time t (10 ms period).
template <class PressureOf>
std::vector<double> baro_heights(double baro_rate_hz, PressureOf pressure_of)
{
    Px4EkfAdapter ekf;
    Px4SensorModelV2Config mount{};
    mount.baro_rate_hz = baro_rate_hz;
    ekf.set_sensor_model_v2_config(mount);
    EkfAdapterConfig config{};
    config.mag_declination_deg = 0.0;
    ekf.set_config(config);
    ekf.set_armed_status(false);
    ekf.set_in_air_status(false);
    ekf.set_vehicle_at_rest(true);
    std::vector<double> heights;
    for (int step = 1; step <= 20000; ++step) {
        const std::uint64_t now = static_cast<std::uint64_t>(step) * 4000;
        ekf.push_imu(EkfImuInput{now, 0.0, 0.0, -9.80665, 0.0, 0.0, 0.0}, 0.004);
        if (step % 5 == 0) {
            ekf.push_mag(EkfMagInput{now, 0.22, 0.0, 0.43});
        }
        if (step % 50 == 0) {
            EkfHilGpsInput gps{};
            gps.time_usec = now;
            gps.fix_type = 3;
            gps.lat_deg = kHomeLatDeg;
            gps.lon_deg = kHomeLonDeg;
            gps.alt_m = 40.0;
            gps.satellites_visible = 16;
            gps.eph_m = 0.5;
            gps.epv_m = 0.8;
            gps.sacc_mps = 0.2;
            ekf.push_gps(gps);
        }
        if ((now % 10000) == 0) {
            EkfBaroInput baro{};
            baro.time_usec = now;
            baro.pressure_pa = pressure_of(now / 1e6, step);
            ekf.push_baro(baro);
        }
        ekf.update();
        heights.push_back(-ekf.get_estimated_state().position_local_ned_m[2]);
    }
    return heights;
}

// International standard atmosphere below 11 km, independent of PX4's atmosphere library.
double isa_pressure_pa(double height_m)
{
    return 101325.0 * std::pow(1.0 - 2.25577e-5 * height_m, 5.25588);
}

double tilt_from_level_deg(const EkfEstimatedState& state)
{
    const auto& q = state.attitude_quaternion_wxyz;
    const double body_z_down = 1.0 - 2.0 * (q[1] * q[1] + q[2] * q[2]);
    return std::acos(std::fmax(-1.0, std::fmin(1.0, body_z_down))) * 180.0 / kPi;
}

double heading_deg(const EkfEstimatedState& state)
{
    const auto& q = state.attitude_quaternion_wxyz;
    return std::atan2(2.0 * (q[0] * q[3] + q[1] * q[2]), 1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3])) * 180.0 / kPi;
}

double north_of_home_m(const EkfEstimatedState& state)
{
    return (state.lat_deg - kHomeLatDeg) * kPi / 180.0 * kEarthRadiusM;
}

void require(bool condition, const char* message)
{
    if (!condition) {
        std::cerr << "px4_sensor_model_v2_ekf_smoke: " << message << std::endl;
        std::exit(EXIT_FAILURE);
    }
}

}  // namespace

int main()
{
    const auto reference = fly(Run{}).state;

    Run upside_down;
    upside_down.mount.board_rotation = 8;
    upside_down.sensor_upside_down = true;
    const auto flipped = fly(upside_down).state;

    Run antenna_ahead;
    antenna_ahead.mount.gps_position_body_m = {0.5, 0.0, 0.0};
    const auto antenna = fly(antenna_ahead).state;

    Run spin_known;
    spin_known.mount.imu_position_body_m = {0.5, 0.0, 0.0};
    spin_known.spin_lever_arm_m = 0.5;
    Run spin_unknown;
    spin_unknown.spin_lever_arm_m = 0.5;
    const double known_speed = fly(spin_known).late_spin_speed_mps;
    const double unknown_speed = fly(spin_unknown).late_spin_speed_mps;

    // Magnetometer::Correct(): S * (raw - offset); a raw field biased by +offset gives the reference heading.
    Run calibrated;
    calibrated.mount.mag_offset_gauss = {0.03, -0.015, 0.008};
    calibrated.mag_bias_gauss = {0.03, -0.015, 0.008};
    Run uncalibrated;
    uncalibrated.mag_bias_gauss = {0.03, -0.015, 0.008};
    const auto with_calibration = fly(calibrated).state;
    const auto without_calibration = fly(uncalibrated).state;

    std::cout << "tilt reference=" << tilt_from_level_deg(reference) << " flipped=" << tilt_from_level_deg(flipped)
              << " | north reference=" << north_of_home_m(reference) << " antenna_ahead=" << north_of_home_m(antenna)
              << " | spin speed known=" << known_speed << " unknown=" << unknown_speed
              << " | heading reference=" << heading_deg(reference) << " calibrated=" << heading_deg(with_calibration)
              << " uncalibrated=" << heading_deg(without_calibration) << std::endl;

    require(reference.attitude_valid && flipped.attitude_valid, "attitude is not valid");
    require(tilt_from_level_deg(flipped) < 1.0, "SENS_BOARD_ROT did not bring the upside-down IMU back to level");
    // The estimate is the center of mass; an antenna 0.5 m ahead of it puts it 0.5 m behind the fix.
    require(std::fabs(north_of_home_m(antenna) - north_of_home_m(reference) + 0.5) < 0.1,
        "SENS_GPS0_OFF did not reach EKF2");
    require(known_speed < 0.1 && unknown_speed > 0.2, "EKF2_IMU_POS did not keep the center of mass still");
    require(std::fabs(heading_deg(with_calibration) - heading_deg(reference)) < 0.5, "CAL_MAG0 did not reach EKF2");
    require(std::fabs(heading_deg(without_calibration) - heading_deg(reference)) > 1.5,
        "the compass bias case is too weak");
    // [SEN-BAR-325] A 2 m pressure step (60 s at rest, then the pressure of 2 m) raises the height by about 2 m.
    const auto step_heights = baro_heights(20.0, [](double t, int) { return isa_pressure_pa(t < 50.0 ? 0.0 : 2.0); });
    const double before = step_heights[12000];  // 48 s
    const double after = step_heights.back();  // 80 s
    std::cout << "baro step: height " << before << " -> " << after << std::endl;
    require(std::fabs((after - before) - 2.0) < 0.2, "the pressure step did not become a 2 m height step");
    std::cout << "px4_sensor_model_v2_ekf_smoke: PASS" << std::endl;
    return EXIT_SUCCESS;
}
