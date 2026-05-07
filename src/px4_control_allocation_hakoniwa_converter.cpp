#include "hakoniwa/drone/control_adapter/px4_control_allocation_hakoniwa_converter.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace hakoniwa::drone::control_adapter {

double compute_px4_moment_ratio_from_hakoniwa(
    double thrust_coefficient,
    double torque_coefficient,
    double rotation_direction)
{
    if (std::fabs(thrust_coefficient) <= 1e-12) {
        throw std::invalid_argument("hakoniwa converter requires non-zero thrust_coefficient");
    }

    return rotation_direction * (torque_coefficient / thrust_coefficient);
}

ControlAllocationInput compose_control_allocation_input_from_hakoniwa(
    const HakoniwaControlAllocationConverterInput& input)
{
    ControlAllocationInput converted{};
    converted.actuator_count = std::min<std::size_t>(input.rotor_count, kMaxActuatorCount);

    for (std::size_t i = 0; i < converted.actuator_count; ++i) {
        const HakoniwaRotorDirectionGeometry& rotor = input.rotors[i];
        RotorActuatorConfig& actuator = converted.actuators[i];

        actuator.geometry.position = {
            rotor.position_x,
            rotor.position_y,
            rotor.position_z
        };
        actuator.geometry.axis = {
            input.rotor_model.axis_x,
            input.rotor_model.axis_y,
            input.rotor_model.axis_z
        };
        actuator.geometry.thrust_coefficient = input.rotor_model.thrust_coefficient;
        actuator.geometry.moment_ratio = compute_px4_moment_ratio_from_hakoniwa(
            input.rotor_model.thrust_coefficient,
            input.rotor_model.torque_coefficient,
            rotor.rotation_direction);

        actuator.limit = {
            input.rotor_model.actuator_min,
            input.rotor_model.actuator_max
        };
        actuator.trim = input.rotor_model.actuator_trim;
        actuator.linearization_point = input.rotor_model.actuator_linearization_point;
    }

    return converted;
}

}  // namespace hakoniwa::drone::control_adapter
