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

}  // namespace hakoniwa::drone::control_adapter

#endif
