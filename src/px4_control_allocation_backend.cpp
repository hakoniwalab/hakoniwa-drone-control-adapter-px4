#include "hakoniwa/drone/control_adapter/px4_control_allocation_backend.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "ControlAllocationSequentialDesaturation.hpp"

namespace hakoniwa::drone::control_adapter {

namespace px4_stub {

// Storage for the PX4 parameters that px4_stubs/px4_platform_common/module_params.h
// exposes to PX4 library code.  PX4 parameters are process-global, and so is this.
int32_t& param_int_storage(int id)
{
    static int32_t values[px4::params::PARAM_COUNT] = {};
    if (id < 0 || id >= px4::params::PARAM_COUNT) {
        throw std::out_of_range("unknown PX4 stub parameter id");
    }
    return values[id];
}

}  // namespace px4_stub

namespace {

// PX4's multicopter allocator: thrust and roll/pitch keep priority, and yaw
// is reduced first when the actuators saturate (MC_AIRMODE selects the mode).
using Px4Allocator = ControlAllocationSequentialDesaturation;
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

        // Public actuator output is normalized rotor thrust (not duty).
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

    // Per-axis scaling keeps the effectiveness rows well conditioned.  With
    // CA_RPY_NORMALIZE the allocator renormalizes the mix columns, so these
    // scales do not change the allocation and are not applied to the demand.
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

// The allocator runs in PX4 actuator units (1.0 = one rotor at maximum
// thrust), so PX4 constants such as the desaturation yaw margin keep their
// meaning.  The public interface uses hover units (1.0 = one rotor at hover
// thrust); MPC_THR_HOVER = T_hover / T_max converts between them.
//
// PX4 RateControl torque is already a normalized demand in PX4 units and goes
// in unchanged.  With CA_RPY_NORMALIZE the allocator renormalizes each torque
// column, so the geometry scales on the effectiveness rows cancel out and must
// not be applied to the demand.  Thrust body_z (hover = -1) becomes
// body_z * MPC_THR_HOVER (hover = -MPC_THR_HOVER), as in PX4.
Px4ControlVector to_control_vector(
    const ThrustTorqueCommand& command,
    double hover_thrust)
{
    Px4ControlVector control{};
    control(ControlAllocation::ControlAxis::ROLL) = f32(command.torque_x);
    control(ControlAllocation::ControlAxis::PITCH) = f32(command.torque_y);
    control(ControlAllocation::ControlAxis::YAW) = f32(command.torque_z);
    control(ControlAllocation::ControlAxis::THRUST_Z) = f32(command.thrust.body_z * hover_thrust);
    return control;
}

Px4ActuatorVector to_trim_vector(
    const ControlAllocationInput& input,
    std::size_t actuator_count,
    double hover_units_to_px4)
{
    Px4ActuatorVector trim{};

    for (std::size_t i = 0; i < actuator_count; ++i) {
        trim(i) = f32(input.actuators[i].trim * hover_units_to_px4);
    }

    return trim;
}

Px4ActuatorVector to_linearization_point_vector(
    const ControlAllocationInput& input,
    std::size_t actuator_count,
    double hover_units_to_px4)
{
    Px4ActuatorVector linearization{};

    for (std::size_t i = 0; i < actuator_count; ++i) {
        linearization(i) = f32(input.actuators[i].linearization_point * hover_units_to_px4);
    }

    return linearization;
}

Px4ActuatorVector to_min_vector(
    const ControlAllocationInput& input,
    std::size_t actuator_count,
    double hover_units_to_px4)
{
    Px4ActuatorVector actuator_min{};

    for (std::size_t i = 0; i < actuator_count; ++i) {
        actuator_min(i) = f32(input.actuators[i].limit.min * hover_units_to_px4);
    }

    return actuator_min;
}

Px4ActuatorVector to_max_vector(
    const ControlAllocationInput& input,
    std::size_t actuator_count,
    double hover_units_to_px4)
{
    Px4ActuatorVector actuator_max{};

    for (std::size_t i = 0; i < actuator_count; ++i) {
        actuator_max(i) = f32(input.actuators[i].limit.max * hover_units_to_px4);
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

bool has_unallocated_control(const Px4ControlVector& setpoint, const Px4ControlVector& allocated)
{
    for (int axis = 0; axis < ControlAllocation::NUM_AXES; ++axis) {
        if (std::fabs(setpoint(axis) - allocated(axis)) > kClipEpsilon) {
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

    const NormalizedAllocationModel model =
        build_normalized_allocation_model(input, actuator_count);
    const double h = config_.hover_thrust;  // hover units -> PX4 units

    controller_->setEffectivenessMatrix(
        model.effectiveness,
        to_trim_vector(input, actuator_count, h),
        to_linearization_point_vector(input, actuator_count, h),
        static_cast<int>(actuator_count),
        config_.update_normalization_scale);
    controller_->setActuatorMin(to_min_vector(input, actuator_count, h));
    controller_->setActuatorMax(to_max_vector(input, actuator_count, h));
    px4_stub::set_param_int(px4::params::MC_AIRMODE, config_.airmode);
    controller_->setControlSetpoint(to_control_vector(input.command, config_.hover_thrust));
    controller_->allocate();

    const Px4ActuatorVector unclipped = controller_->getActuatorSetpoint();
    controller_->clipActuatorSetpoint();
    const Px4ActuatorVector clipped = controller_->getActuatorSetpoint();
    const Px4ControlVector allocated = controller_->getAllocatedControl();
    const Px4ControlVector control_sp = controller_->getControlSetpoint();

    ControlAllocationOutput output{};
    output.actuator_commands.count = actuator_count;
    for (std::size_t i = 0; i < actuator_count; ++i) {
        // Normalized rotor thrust: 1.0 is one rotor at hover thrust.  The
        // caller converts it to the plant actuator command (for example PWM
        // duty through the motor model); it is not a duty value.
        output.actuator_commands.values[i] = static_cast<double>(clipped(i)) / h;
    }

    // Sequential desaturation keeps the outputs inside the limits by giving up
    // part of the demand, so a saturated allocation shows up as an unallocated
    // control rather than as clipping.  Report either as clipped.
    output.status.clipped = did_clip(unclipped, clipped, actuator_count)
        || has_unallocated_control(control_sp, allocated);
    output.status.unallocated_torque_x = static_cast<double>(
        control_sp(ControlAllocation::ControlAxis::ROLL) - allocated(ControlAllocation::ControlAxis::ROLL));
    output.status.unallocated_torque_y = static_cast<double>(
        control_sp(ControlAllocation::ControlAxis::PITCH) - allocated(ControlAllocation::ControlAxis::PITCH));
    output.status.unallocated_torque_z = static_cast<double>(
        control_sp(ControlAllocation::ControlAxis::YAW) - allocated(ControlAllocation::ControlAxis::YAW));
    output.status.unallocated_thrust_body_z = static_cast<double>(
        control_sp(ControlAllocation::ControlAxis::THRUST_Z) - allocated(ControlAllocation::ControlAxis::THRUST_Z))
        / h;

    return output;
}

void Px4ControlAllocationBackend::set_config(const Px4ControlAllocationBackendConfig& config)
{
    config_ = config;
    apply_config();
}

void Px4ControlAllocationBackend::apply_config()
{
    if (!std::isfinite(config_.hover_thrust) || config_.hover_thrust <= 0.0 || config_.hover_thrust > 1.0) {
        throw std::invalid_argument(
            "Px4ControlAllocationBackend requires hover_thrust (MPC_THR_HOVER) in (0, 1]");
    }
    if (config_.airmode < 0 || config_.airmode > 2) {
        throw std::invalid_argument("Px4ControlAllocationBackend requires airmode (MC_AIRMODE) 0, 1 or 2");
    }
    controller_->setNormalizeRPY(config_.normalize_rpy);
    controller_->setMetricAllocation(config_.metric_allocation);
}

}  // namespace hakoniwa::drone::control_adapter
