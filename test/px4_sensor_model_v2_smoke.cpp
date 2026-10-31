#include <array>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>

#include "EKF/ekf.h"

#define private public
#include "hakoniwa/drone/control_adapter/px4_ekf_adapter.hpp"
#undef private
#include "hakoniwa/drone/control_adapter/px4_controller_config_loader.hpp"
#include "hakoniwa/drone/control_adapter/px4_rate_control_backend.hpp"

using namespace hakoniwa::drone::control_adapter;

namespace {

using Vector = std::array<double, 3>;
using Matrix = std::array<std::array<double, 3>, 3>;

void require(bool condition, const char* message)
{
    if (!condition) {
        std::cerr << message << std::endl;
        std::exit(EXIT_FAILURE);
    }
}

Matrix multiply(const Matrix& left, const Matrix& right)
{
    Matrix result{};
    for (int row = 0; row < 3; ++row) {
        for (int column = 0; column < 3; ++column) {
            for (int index = 0; index < 3; ++index) {
                result[row][column] += left[row][index] * right[index][column];
            }
        }
    }
    return result;
}

Matrix rotation(double roll_deg, double pitch_deg, double yaw_deg)
{
    const double pi = 3.14159265358979323846;
    const double roll = roll_deg * pi / 180.0;
    const double pitch = pitch_deg * pi / 180.0;
    const double yaw = yaw_deg * pi / 180.0;
    const Matrix rx{{{1, 0, 0}, {0, std::cos(roll), -std::sin(roll)},
        {0, std::sin(roll), std::cos(roll)}}};
    const Matrix ry{{{std::cos(pitch), 0, std::sin(pitch)}, {0, 1, 0},
        {-std::sin(pitch), 0, std::cos(pitch)}}};
    const Matrix rz{{{std::cos(yaw), -std::sin(yaw), 0},
        {std::sin(yaw), std::cos(yaw), 0}, {0, 0, 1}}};
    return multiply(multiply(rz, ry), rx);
}

Vector transpose_apply(const Matrix& matrix, const Vector& value)
{
    Vector result{};
    for (int row = 0; row < 3; ++row) {
        for (int column = 0; column < 3; ++column) {
            result[row] += matrix[column][row] * value[column];
        }
    }
    return result;
}

void require_vector(const Vector& actual, const Vector& expected, const char* message)
{
    for (int axis = 0; axis < 3; ++axis) {
        require(std::fabs(actual[axis] - expected[axis]) <= 1e-5, message);
    }
}

}  // namespace

int main()
{
    const std::array<std::array<double, 3>, 41> angles{{
        {0, 0, 0}, {0, 0, 45}, {0, 0, 90}, {0, 0, 135}, {0, 0, 180},
        {0, 0, 225}, {0, 0, 270}, {0, 0, 315}, {180, 0, 0}, {180, 0, 45},
        {180, 0, 90}, {180, 0, 135}, {0, 180, 0}, {180, 0, 225}, {180, 0, 270},
        {180, 0, 315}, {90, 0, 0}, {90, 0, 45}, {90, 0, 90}, {90, 0, 135},
        {270, 0, 0}, {270, 0, 45}, {270, 0, 90}, {270, 0, 135}, {0, 90, 0},
        {0, 270, 0}, {0, 180, 90}, {0, 180, 270}, {90, 90, 0}, {180, 90, 0},
        {270, 90, 0}, {90, 180, 0}, {270, 180, 0}, {90, 270, 0}, {180, 270, 0},
        {270, 270, 0}, {90, 180, 90}, {90, 0, 270}, {90, 68, 293}, {0, 315, 0},
        {90, 315, 0}}};
    const Vector body{1.25, -2.5, 3.75};
    for (int value = 0; value < 41; ++value) {
        if (value == 38) {
            continue;
        }
        Px4SensorModelV2Config config{};
        config.board_rotation = value;
        config.board_x_offset_deg = 3.0;
        config.board_y_offset_deg = -4.0;
        config.board_z_offset_deg = 5.0;
        const Matrix board = rotation(angles[value][0], angles[value][1], angles[value][2]);
        const Matrix adjustment = rotation(3.0, -4.0, 5.0);
        const Matrix expected_rotation = multiply(adjustment, board);
        const Vector sensor = transpose_apply(expected_rotation, body);
        require_vector(rotate_sensor_to_body(config, sensor), body, "rotation did not recover body value");
    }

    Px4SensorModelV2Config defaults{};
    require_vector(rotate_sensor_to_body(defaults, body), body, "missing parameters changed sensor value");

    std::ifstream file("../config/px4-controller-config.sample.json");
    std::ostringstream text;
    text << file.rdbuf();
    std::string config_text = text.str();
    config_text.insert(config_text.find('{') + 1,
        R"("SENS_BOARD_ROT":2,"SENS_BOARD_X_OFF":1,"SENS_BOARD_Y_OFF":2,"SENS_BOARD_Z_OFF":3,)"
        R"("EKF2_IMU_POS_X":0.1,"EKF2_IMU_POS_Y":0.2,"EKF2_IMU_POS_Z":0.3,)"
        R"("SENS_GPS0_OFFX":0.4,"SENS_GPS0_OFFY":0.5,"SENS_GPS0_OFFZ":0.6,)"
        R"("CAL_MAG0_XOFF":0.1,"CAL_MAG0_YOFF":-0.2,"CAL_MAG0_ZOFF":0.3,)"
        R"("CAL_MAG0_XSCALE":2,"CAL_MAG0_YSCALE":3,"CAL_MAG0_ZSCALE":4,)"
        R"("CAL_MAG0_XODIAG":0.5,"CAL_MAG0_YODIAG":-0.25,"CAL_MAG0_ZODIAG":0.75,)"
    );
    const Px4ControllerConfig loaded = Px4ControllerConfigLoader{}.load_from_text(config_text);
    require(loaded.sensor_model_v2.board_rotation == 2, "SENS_BOARD_ROT was not loaded");
    require(loaded.rate_control.sensor_model_v2.board_rotation == 2, "rate config did not receive rotation");

    const auto require_invalid_board_rotation = [&text](const char* value) {
        std::string invalid_text = text.str();
        invalid_text.insert(invalid_text.find('{') + 1,
            std::string("\"SENS_BOARD_ROT\":") + value + ",");
        bool rejected = false;
        try {
            static_cast<void>(Px4ControllerConfigLoader{}.load_from_text(invalid_text));
        } catch (const std::runtime_error&) {
            rejected = true;
        }
        require(rejected, "invalid SENS_BOARD_ROT was not rejected");
    };
    require_invalid_board_rotation("-1");
    require_invalid_board_rotation("41");
    require_invalid_board_rotation("1.5");

    // Handoff: the exact set Drone PRO's generate_px4_sensor_mount_config.py writes into common.parameters for a
    // sample mount (IMU 10/0/30 deg at 0.1/-0.05/0.02 m, GPS at -0.1/0/-0.2 m, COM 0.01/0/0.03 m, hard and soft iron).
    std::string generated_text = text.str();
    const std::string common_parameters = "\"common\": {\n    \"parameters\": {";
    require(generated_text.find(common_parameters) != std::string::npos, "sample has no common.parameters");
    generated_text.insert(generated_text.find(common_parameters) + common_parameters.size(),
        R"("SENS_BOARD_ROT": 1, "SENS_BOARD_X_OFF": 7.107076110446535, "SENS_BOARD_Y_OFF": 7.053022130283184,)"
        R"("SENS_BOARD_Z_OFF": -14.561451413257695, "EKF2_IMU_POS_X": 0.09000000000000001, "EKF2_IMU_POS_Y": -0.05,)"
        R"("EKF2_IMU_POS_Z": -0.009999999999999998, "SENS_GPS0_OFFX": -0.11, "SENS_GPS0_OFFY": 0.0,)"
        R"("SENS_GPS0_OFFZ": -0.23, "CAL_MAG0_XOFF": 0.03, "CAL_MAG0_YOFF": -0.015, "CAL_MAG0_ZOFF": 0.008,)"
        R"("CAL_MAG0_XSCALE": 0.8, "CAL_MAG0_YSCALE": 1.25, "CAL_MAG0_ZSCALE": 2.0, "CAL_MAG0_XODIAG": 0.0,)"
        R"("CAL_MAG0_YODIAG": 0.0, "CAL_MAG0_ZODIAG": 0.0,)");
    const auto generated = Px4ControllerConfigLoader{}.load_from_text(generated_text).sensor_model_v2;
    require(generated.board_rotation == 1, "generated SENS_BOARD_ROT was not loaded");
    require_vector({generated.board_x_offset_deg, generated.board_y_offset_deg, generated.board_z_offset_deg},
        {7.107076110446535, 7.053022130283184, -14.561451413257695}, "generated SENS_BOARD_*_OFF was not loaded");
    require_vector(generated.imu_position_body_m, {0.09, -0.05, -0.01}, "generated EKF2_IMU_POS was not loaded");
    require_vector(generated.gps_position_body_m, {-0.11, 0.0, -0.23}, "generated SENS_GPS0_OFF was not loaded");
    require_vector(generated.mag_offset_gauss, {0.03, -0.015, 0.008}, "generated CAL_MAG0_*OFF was not loaded");
    require_vector(generated.mag_scale, {0.8, 1.25, 2.0}, "generated CAL_MAG0_*SCALE was not loaded");
    require_vector(generated.mag_offdiagonal, {0.0, 0.0, 0.0}, "generated CAL_MAG0_*ODIAG was not loaded");
    // The generated rotation brings a sensor-axis value back to the body axes (R = Rz(30) Rx(10)).
    require_vector(rotate_sensor_to_body(generated, transpose_apply(rotation(10.0, 0.0, 30.0), body)), body,
        "generated rotation did not recover the mount");

    // [SEN-COM-340] The rate stage turns the sensor-axis gyro to the body axes: ROTATION_YAW_90 reads body p as -q.
    std::string yaw90_text = text.str();
    yaw90_text.insert(yaw90_text.find('{') + 1, R"("SENS_BOARD_ROT":2,)");
    Px4RateControlBackend plain_rate(Px4ControllerConfigLoader{}.load_from_text(text.str()).rate_control);
    Px4RateControlBackend rotated_rate(Px4ControllerConfigLoader{}.load_from_text(yaw90_text).rate_control);
    RateControlInput body_input{};
    body_input.dt_sec = 0.001;
    RateControlInput sensor_input = body_input;
    double difference = 0.0;
    double magnitude = 0.0;
    for (int step = 0; step < 50; ++step) {
        const double rate = 0.5 * std::sin(0.1 * step);
        body_input.rate.p = rate;
        sensor_input.rate.q = -rate;
        const BodyTorqueCommand expected = plain_rate.run(body_input);
        const BodyTorqueCommand actual = rotated_rate.run(sensor_input);
        difference += std::fabs(expected.x - actual.x) + std::fabs(expected.y - actual.y)
            + std::fabs(expected.z - actual.z);
        magnitude += std::fabs(expected.x);
    }
    require(magnitude > 0.0, "rate control produced no roll torque");
    require(difference <= 1e-6 * magnitude, "rate control did not use SENS_BOARD_ROT");

    Px4EkfAdapter adapter;
    adapter.set_sensor_model_v2_config(loaded.sensor_model_v2);
    const auto& params = *adapter.ekf_->getParamHandle();
    require_vector({params.imu_pos_body(0), params.imu_pos_body(1), params.imu_pos_body(2)},
        {0.1, 0.2, 0.3}, "EKF imu_pos_body was not configured");
    require_vector(adapter.sensor_model_v2_.gps_position_body_m,
        {0.4, 0.5, 0.6}, "EKF gps_pos_body was not configured");

    // [SEN-MAG-340] Missing calibration is the identity correction.
    require_vector(correct_mag_sensor_to_body(defaults, body), body,
        "missing magnetometer calibration changed the value");
    Px4SensorModelV2Config hard_iron{};
    hard_iron.mag_offset_gauss = {0.1, -0.2, 0.3};
    // [SEN-MAG-350] Hard-iron offsets are subtracted before scaling.
    require_vector(correct_mag_sensor_to_body(hard_iron, {1.1, 1.8, 3.3}),
        {1.0, 2.0, 3.0}, "hard-iron correction did not recover the value");
    Px4SensorModelV2Config soft_iron{};
    soft_iron.mag_scale = {2.0, 3.0, 4.0};
    soft_iron.mag_offdiagonal = {0.5, -0.25, 0.75};
    // [SEN-MAG-340] [SEN-MAG-360] The symmetric scale matrix uses X/Y/Z ODIAG as xy/xz/yz.
    require_vector(correct_mag_sensor_to_body(soft_iron, {1.0, 2.0, 3.0}),
        {2.25, 8.75, 13.25}, "soft-iron matrix was not applied correctly");
    return EXIT_SUCCESS;
}
