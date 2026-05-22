#include "hakoniwa/drone/control_adapter/px4_control_allocation_backend.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "ControlAllocationPseudoInverse.hpp"

namespace hakoniwa::drone::control_adapter {

namespace {

using Px4Allocator = ControlAllocationPseudoInverse;
using Px4ActuatorVector = ControlAllocation::ActuatorVector;
using Px4ControlVector = matrix::Vector<float, ControlAllocation::NUM_AXES>;
using Px4EffectivenessMatrix = matrix::Matrix<float, ControlAllocation::NUM_AXES, ControlAllocation::NUM_ACTUATORS>;

constexpr float kClipEpsilon = 1e-6f;
constexpr double kMinScale = 1e-9;

float f32(double value)
{
    return static_cast<float>(value);
}

struct NormalizedAllocationModel {
    Px4EffectivenessMatrix effectiveness{};
    double roll_torque_scale_nm{1.0};
    double pitch_torque_scale_nm{1.0};
    double yaw_torque_scale_nm{1.0};
};

double axis_scale_from_physical_effects(const std::array<double, kMaxActuatorCount>& values, std::size_t actuator_count)
{
    double positive_sum = 0.0;
    double negative_sum = 0.0;

    for (std::size_t i = 0; i < actuator_count; ++i) {
        if (values[i] > 0.0) {
            positive_sum += values[i];
        } else {
            negative_sum += -values[i];
        }
    }

    const double scale = std::max(positive_sum, negative_sum);
    return (std::isfinite(scale) && scale > kMinScale) ? scale : 1.0;
}

NormalizedAllocationModel build_normalized_allocation_model(
    const ControlAllocationInput& input,
    std::size_t actuator_count)
{
    NormalizedAllocationModel model{};
    const float per_rotor_collective_effect = -1.0f / static_cast<float>(actuator_count);

    std::array<double, kMaxActuatorCount> roll_physical{};
    std::array<double, kMaxActuatorCount> pitch_physical{};
    std::array<double, kMaxActuatorCount> yaw_physical{};

    for (std::size_t i = 0; i < actuator_count; ++i) {
        const RotorActuatorGeometry& geometry = input.actuators[i].geometry;
        matrix::Vector3f axis{f32(geometry.axis.x), f32(geometry.axis.y), f32(geometry.axis.z)};
        const float axis_norm = axis.norm();

        if (axis_norm <= FLT_EPSILON) {
            continue;
        }

        axis /= axis_norm;

        const matrix::Vector3f position{
            f32(geometry.position.x),
            f32(geometry.position.y),
            f32(geometry.position.z)
        };

        const float moment_ratio = f32(geometry.moment_ratio);

        // Public actuator output is duty/normalized command in [0,1].
        // One full actuator command is interpreted as one rotor producing
        // hover-equivalent thrust. Physical Ct/Cq magnitudes are not used
        // directly in the allocator matrix; geometry and moment_ratio define
        // relative authority.
        const matrix::Vector3f physical_moment =
            position.cross(axis) - moment_ratio * axis;

        roll_physical[i] = static_cast<double>(physical_moment(0));
        pitch_physical[i] = static_cast<double>(physical_moment(1));
        yaw_physical[i] = static_cast<double>(physical_moment(2));

        // One internal actuator unit means "this rotor produces hover thrust".
        // Therefore each rotor contributes 1 / actuator_count of total vehicle hover.
        model.effectiveness(ControlAllocation::ControlAxis::THRUST_Z, i) =
            per_rotor_collective_effect * ((axis(2) < 0.0f) ? 1.0f : -1.0f);
    }

    model.roll_torque_scale_nm = axis_scale_from_physical_effects(roll_physical, actuator_count);
    model.pitch_torque_scale_nm = axis_scale_from_physical_effects(pitch_physical, actuator_count);
    model.yaw_torque_scale_nm = axis_scale_from_physical_effects(yaw_physical, actuator_count);

    for (std::size_t i = 0; i < actuator_count; ++i) {
        model.effectiveness(ControlAllocation::ControlAxis::ROLL, i) =
            f32(roll_physical[i] / model.roll_torque_scale_nm);
        model.effectiveness(ControlAllocation::ControlAxis::PITCH, i) =
            f32(pitch_physical[i] / model.pitch_torque_scale_nm);
        model.effectiveness(ControlAllocation::ControlAxis::YAW, i) =
            f32(yaw_physical[i] / model.yaw_torque_scale_nm);
    }

    return model;
}

Px4ControlVector to_control_vector(
    const ThrustTorqueCommand& command,
    const NormalizedAllocationModel& model)
{
    Px4ControlVector control{};
    control(ControlAllocation::ControlAxis::ROLL) = f32(command.torque_x / model.roll_torque_scale_nm);
    control(ControlAllocation::ControlAxis::PITCH) = f32(command.torque_y / model.pitch_torque_scale_nm);
    control(ControlAllocation::ControlAxis::YAW) = f32(command.torque_z / model.yaw_torque_scale_nm);
    control(ControlAllocation::ControlAxis::THRUST_Z) = f32(command.thrust.body_z);
    return control;
}

Px4ActuatorVector to_trim_vector(
    const ControlAllocationInput& input,
    std::size_t actuator_count,
    const Px4ControlAllocationBackendConfig& config)
{
    Px4ActuatorVector trim{};

    for (std::size_t i = 0; i < actuator_count; ++i) {
        trim(i) = f32(input.actuators[i].trim / config.hover_duty);
    }

    return trim;
}

Px4ActuatorVector to_linearization_point_vector(
    const ControlAllocationInput& input,
    std::size_t actuator_count,
    const Px4ControlAllocationBackendConfig& config)
{
    Px4ActuatorVector linearization{};

    for (std::size_t i = 0; i < actuator_count; ++i) {
        linearization(i) = f32(input.actuators[i].linearization_point / config.hover_duty);
    }

    return linearization;
}

Px4ActuatorVector to_min_vector(
    const ControlAllocationInput& input,
    std::size_t actuator_count,
    const Px4ControlAllocationBackendConfig& config)
{
    Px4ActuatorVector actuator_min{};

    for (std::size_t i = 0; i < actuator_count; ++i) {
        actuator_min(i) = f32(input.actuators[i].limit.min / config.hover_duty);
    }

    return actuator_min;
}

Px4ActuatorVector to_max_vector(
    const ControlAllocationInput& input,
    std::size_t actuator_count,
    const Px4ControlAllocationBackendConfig& config)
{
    Px4ActuatorVector actuator_max{};

    for (std::size_t i = 0; i < actuator_count; ++i) {
        actuator_max(i) = f32(input.actuators[i].limit.max / config.hover_duty);
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

ControlAllocationOutput make_unallocated_output(
    const ControlAllocationInput& input,
    std::size_t actuator_count)
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
    if (!(std::isfinite(config_.hover_duty) && config_.hover_duty > kMinScale)) {
        throw std::runtime_error("Px4ControlAllocationBackend requires positive hover_duty");
    }

    const NormalizedAllocationModel model =
        build_normalized_allocation_model(input, actuator_count);

    controller_->setEffectivenessMatrix(
        model.effectiveness,
        to_trim_vector(input, actuator_count, config_),
        to_linearization_point_vector(input, actuator_count, config_),
        static_cast<int>(actuator_count),
        config_.update_normalization_scale);
    controller_->setActuatorMin(to_min_vector(input, actuator_count, config_));
    controller_->setActuatorMax(to_max_vector(input, actuator_count, config_));
    controller_->setControlSetpoint(to_control_vector(input.command, model));
    controller_->allocate();

    const Px4ActuatorVector unclipped = controller_->getActuatorSetpoint();
    controller_->clipActuatorSetpoint();
    const Px4ActuatorVector clipped = controller_->getActuatorSetpoint();
    const Px4ControlVector allocated = controller_->getAllocatedControl();
    const Px4ControlVector control_sp = controller_->getControlSetpoint();

    ControlAllocationOutput output{};
    output.actuator_commands.count = actuator_count;
    for (std::size_t i = 0; i < actuator_count; ++i) {
        output.actuator_commands.values[i] = clipped(i) * config_.hover_duty;
    }

    output.status.clipped = did_clip(unclipped, clipped, actuator_count);
    output.status.unallocated_torque_x = static_cast<double>(
        control_sp(ControlAllocation::ControlAxis::ROLL) - allocated(ControlAllocation::ControlAxis::ROLL))
        * model.roll_torque_scale_nm;
    output.status.unallocated_torque_y = static_cast<double>(
        control_sp(ControlAllocation::ControlAxis::PITCH) - allocated(ControlAllocation::ControlAxis::PITCH))
        * model.pitch_torque_scale_nm;
    output.status.unallocated_torque_z = static_cast<double>(
        control_sp(ControlAllocation::ControlAxis::YAW) - allocated(ControlAllocation::ControlAxis::YAW))
        * model.yaw_torque_scale_nm;
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
