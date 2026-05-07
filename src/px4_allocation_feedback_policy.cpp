#include "hakoniwa/drone/control_adapter/px4_allocation_feedback_policy.hpp"

namespace hakoniwa::drone::control_adapter {

namespace {

constexpr double kEpsilon = 1e-6;

AxisSaturationFlags to_axis_flags(double unallocated)
{
    AxisSaturationFlags flags{};

    if (unallocated > kEpsilon) {
        flags.positive = true;
    }
    else if (unallocated < -kEpsilon) {
        flags.negative = true;
    }

    return flags;
}

}  // namespace

void Px4AllocationFeedbackPolicy::reset()
{
}

RateControlSaturation Px4AllocationFeedbackPolicy::run(const AllocationFeedbackPolicyInput& input)
{
    return RateControlSaturation{
        to_axis_flags(input.allocation_status.unallocated_torque_x),
        to_axis_flags(input.allocation_status.unallocated_torque_y),
        to_axis_flags(input.allocation_status.unallocated_torque_z)
    };
}

}  // namespace hakoniwa::drone::control_adapter
