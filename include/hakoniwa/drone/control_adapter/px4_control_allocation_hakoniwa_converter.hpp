#pragma once

#include <array>
#include <cstddef>

#include "hakoniwa/drone/control_adapter/control_allocation_backend.hpp"

namespace hakoniwa::drone::control_adapter {

struct HakoniwaRotorDirectionGeometry {
    double position_x{0.0};
    double position_y{0.0};
    double position_z{0.0};
    double rotation_direction{1.0};
};

struct HakoniwaRotorModelParameters {
    double thrust_coefficient{0.0};
    double torque_coefficient{0.0};
    double axis_x{0.0};
    double axis_y{0.0};
    double axis_z{-1.0};
    double actuator_min{0.0};
    double actuator_max{1.0};
    double actuator_trim{0.0};
    double actuator_linearization_point{0.0};
};

struct HakoniwaControlAllocationConverterInput {
    std::array<HakoniwaRotorDirectionGeometry, kMaxActuatorCount> rotors{};
    std::size_t rotor_count{0};
    HakoniwaRotorModelParameters rotor_model{};
};

double compute_px4_moment_ratio_from_hakoniwa(
    double thrust_coefficient,
    double torque_coefficient,
    double rotation_direction);

ControlAllocationInput compose_control_allocation_input_from_hakoniwa(
    const HakoniwaControlAllocationConverterInput& input);

}  // namespace hakoniwa::drone::control_adapter
