#include "hakoniwa/drone/control_adapter/px4_sensor_model_v2.hpp"

#ifdef HAKO_ENABLE_SENSOR_MODEL_V2

#include <px4_platform_common/defines.h>
#include <conversion/rotation.h>
#include <mathlib/mathlib.h>
#include <matrix/matrix/math.hpp>

namespace hakoniwa::drone::control_adapter {

std::array<double, 3> rotate_sensor_to_body(
    const Px4SensorModelV2Config& config,
    const std::array<double, 3>& sensor_value)
{
    const Rotation rotation = static_cast<Rotation>(config.board_rotation);
    const matrix::Dcmf level_adjustment{matrix::Eulerf{
        math::radians(static_cast<float>(config.board_x_offset_deg)),
        math::radians(static_cast<float>(config.board_y_offset_deg)),
        math::radians(static_cast<float>(config.board_z_offset_deg))}};
    const matrix::Dcmf sensor_to_body = level_adjustment * get_rot_matrix(rotation);
    const matrix::Vector3f result = sensor_to_body * matrix::Vector3f{
        static_cast<float>(sensor_value[0]),
        static_cast<float>(sensor_value[1]),
        static_cast<float>(sensor_value[2])};
    return {result(0), result(1), result(2)};
}

std::array<double, 3> correct_mag_sensor_to_body(
    const Px4SensorModelV2Config& config,
    const std::array<double, 3>& sensor_value)
{
    // [SEN-MAG-340] [SEN-MAG-350] [SEN-MAG-360]
    // Match PX4 Magnetometer::Correct(): rotation * (scale * (data - offset)).
    const matrix::Vector3f centered{
        static_cast<float>(sensor_value[0] - config.mag_offset_gauss[0]),
        static_cast<float>(sensor_value[1] - config.mag_offset_gauss[1]),
        static_cast<float>(sensor_value[2] - config.mag_offset_gauss[2])};
    matrix::Matrix3f scale{};
    scale(0, 0) = config.mag_scale[0];
    scale(1, 1) = config.mag_scale[1];
    scale(2, 2) = config.mag_scale[2];
    scale(0, 1) = scale(1, 0) = config.mag_offdiagonal[0];
    scale(0, 2) = scale(2, 0) = config.mag_offdiagonal[1];
    scale(1, 2) = scale(2, 1) = config.mag_offdiagonal[2];
    const matrix::Vector3f corrected = scale * centered;
    return rotate_sensor_to_body(config, {corrected(0), corrected(1), corrected(2)});
}

}  // namespace hakoniwa::drone::control_adapter

#endif
