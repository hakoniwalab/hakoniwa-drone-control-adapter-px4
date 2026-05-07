#include "hakoniwa/drone/control_adapter/px4_control_allocation_backend.hpp"

#include <algorithm>
#include <cmath>

#include "ControlAllocationPseudoInverse.hpp"

namespace hakoniwa::drone::control_adapter {

namespace {

using Px4Allocator = ControlAllocationPseudoInverse;
using Px4ActuatorVector = ControlAllocation::ActuatorVector;
using Px4ControlVector = matrix::Vector<float, ControlAllocation::NUM_AXES>;
using Px4EffectivenessMatrix = matrix::Matrix<float, ControlAllocation::NUM_AXES, ControlAllocation::NUM_ACTUATORS>;

constexpr float kClipEpsilon = 1e-6f;

float f32(double value)
{
    return static_cast<float>(value);
}

Px4ControlVector to_control_vector(const ThrustTorqueCommand& command)
{
    Px4ControlVector control{};
    control(ControlAllocation::ControlAxis::ROLL) = f32(command.torque_x);
    control(ControlAllocation::ControlAxis::PITCH) = f32(command.torque_y);
    control(ControlAllocation::ControlAxis::YAW) = f32(command.torque_z);
    control(ControlAllocation::ControlAxis::THRUST_Z) = f32(command.thrust.body_z);
    return control;
}

Px4EffectivenessMatrix to_effectiveness_matrix(const ControlAllocationInput& input, std::size_t actuator_count)
{
    Px4EffectivenessMatrix effectiveness{};

    for (std::size_t i = 0; i < actuator_count; ++i) {
        const RotorActuatorGeometry& geometry = input.actuators[i].geometry;
        matrix::Vector3f axis{f32(geometry.axis.x), f32(geometry.axis.y), f32(geometry.axis.z)};
        const float axis_norm = axis.norm();

        if (axis_norm <= FLT_EPSILON) {
            continue;
        }

        axis /= axis_norm;

        const float thrust_coefficient = f32(geometry.thrust_coefficient);
        if (std::fabs(thrust_coefficient) <= FLT_EPSILON) {
            continue;
        }

        const matrix::Vector3f position{
            f32(geometry.position.x),
            f32(geometry.position.y),
            f32(geometry.position.z)
        };

        const float moment_ratio = f32(geometry.moment_ratio);
        const matrix::Vector3f thrust = thrust_coefficient * axis;
        const matrix::Vector3f moment = thrust_coefficient * position.cross(axis) - thrust_coefficient * moment_ratio * axis;

        effectiveness(ControlAllocation::ControlAxis::ROLL, i) = moment(0);
        effectiveness(ControlAllocation::ControlAxis::PITCH, i) = moment(1);
        effectiveness(ControlAllocation::ControlAxis::YAW, i) = moment(2);
        effectiveness(ControlAllocation::ControlAxis::THRUST_X, i) = thrust(0);
        effectiveness(ControlAllocation::ControlAxis::THRUST_Y, i) = thrust(1);
        effectiveness(ControlAllocation::ControlAxis::THRUST_Z, i) = thrust(2);
    }

    return effectiveness;
}

Px4ActuatorVector to_trim_vector(const ControlAllocationInput& input, std::size_t actuator_count)
{
    Px4ActuatorVector trim{};

    for (std::size_t i = 0; i < actuator_count; ++i) {
        trim(i) = f32(input.actuators[i].trim);
    }

    return trim;
}

Px4ActuatorVector to_linearization_point_vector(const ControlAllocationInput& input, std::size_t actuator_count)
{
    Px4ActuatorVector linearization{};

    for (std::size_t i = 0; i < actuator_count; ++i) {
        linearization(i) = f32(input.actuators[i].linearization_point);
    }

    return linearization;
}

Px4ActuatorVector to_min_vector(const ControlAllocationInput& input, std::size_t actuator_count)
{
    Px4ActuatorVector actuator_min{};

    for (std::size_t i = 0; i < actuator_count; ++i) {
        actuator_min(i) = f32(input.actuators[i].limit.min);
    }

    return actuator_min;
}

Px4ActuatorVector to_max_vector(const ControlAllocationInput& input, std::size_t actuator_count)
{
    Px4ActuatorVector actuator_max{};

    for (std::size_t i = 0; i < actuator_count; ++i) {
        actuator_max(i) = f32(input.actuators[i].limit.max);
    }

    return actuator_max;
}

bool did_clip(const Px4ActuatorVector& before, const Px4ActuatorVector& after, std::size_t actuator_count)
{
    for (std::size_t i = 0; i < actuator_count; ++i) {
        if (std::fabs(before(i) - after(i)) > kClipEpsilon) {
            return true;
        }
    }

    return false;
}

ControlAllocationOutput make_unallocated_output(const ControlAllocationInput& input, std::size_t actuator_count)
{
    ControlAllocationOutput output{};
    output.actuator_commands.count = actuator_count;
    output.status.unallocated_torque_x = input.command.torque_x;
    output.status.unallocated_torque_y = input.command.torque_y;
    output.status.unallocated_torque_z = input.command.torque_z;
    output.status.unallocated_thrust_body_z = input.command.thrust.body_z;
    return output;
}

}  // namespace

Px4ControlAllocationBackend::Px4ControlAllocationBackend(const Px4ControlAllocationBackendConfig& config)
    : config_(config)
    , controller_(new Px4Allocator())
{
    apply_config();
    reset();
}

Px4ControlAllocationBackend::~Px4ControlAllocationBackend()
{
    delete controller_;
}

void Px4ControlAllocationBackend::reset()
{
    controller_->setActuatorSetpoint(Px4ActuatorVector{});
}

ControlAllocationOutput Px4ControlAllocationBackend::run(const ControlAllocationInput& input)
{
    const std::size_t actuator_count = std::min<std::size_t>(input.actuator_count, kMaxActuatorCount);

    if (actuator_count == 0U) {
        return make_unallocated_output(input, actuator_count);
    }

    controller_->setEffectivenessMatrix(
        to_effectiveness_matrix(input, actuator_count),
        to_trim_vector(input, actuator_count),
        to_linearization_point_vector(input, actuator_count),
        static_cast<int>(actuator_count),
        config_.update_normalization_scale);
    controller_->setActuatorMin(to_min_vector(input, actuator_count));
    controller_->setActuatorMax(to_max_vector(input, actuator_count));
    controller_->setControlSetpoint(to_control_vector(input.command));
    controller_->allocate();

    const Px4ActuatorVector unclipped = controller_->getActuatorSetpoint();
    controller_->clipActuatorSetpoint();
    const Px4ActuatorVector clipped = controller_->getActuatorSetpoint();
    const Px4ControlVector allocated = controller_->getAllocatedControl();
    const Px4ControlVector control_sp = controller_->getControlSetpoint();

    ControlAllocationOutput output{};
    output.actuator_commands.count = actuator_count;
    for (std::size_t i = 0; i < actuator_count; ++i) {
        output.actuator_commands.values[i] = clipped(i);
    }

    output.status.clipped = did_clip(unclipped, clipped, actuator_count);
    output.status.unallocated_torque_x = static_cast<double>(
        control_sp(ControlAllocation::ControlAxis::ROLL) - allocated(ControlAllocation::ControlAxis::ROLL));
    output.status.unallocated_torque_y = static_cast<double>(
        control_sp(ControlAllocation::ControlAxis::PITCH) - allocated(ControlAllocation::ControlAxis::PITCH));
    output.status.unallocated_torque_z = static_cast<double>(
        control_sp(ControlAllocation::ControlAxis::YAW) - allocated(ControlAllocation::ControlAxis::YAW));
    output.status.unallocated_thrust_body_z = static_cast<double>(
        control_sp(ControlAllocation::ControlAxis::THRUST_Z) - allocated(ControlAllocation::ControlAxis::THRUST_Z));
    return output;
}

void Px4ControlAllocationBackend::set_config(const Px4ControlAllocationBackendConfig& config)
{
    config_ = config;
    apply_config();
}

void Px4ControlAllocationBackend::apply_config()
{
    controller_->setNormalizeRPY(config_.normalize_rpy);
    controller_->setMetricAllocation(config_.metric_allocation);
}

}  // namespace hakoniwa::drone::control_adapter
